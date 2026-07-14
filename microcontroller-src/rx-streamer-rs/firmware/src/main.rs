//! kv4p RX streamer — RX-only WiFi audio streaming firmware for the kv4p-ht
//! board (ESP32-WROOM-32 + SA818). Rust rewrite of the Arduino C++ firmware.
//!
//! This binary wires the board up in `setup`-equivalent order (board detect ->
//! config -> radio -> audio -> wifi -> streamer/decoder) and then runs a
//! supervisor loop on core 0: squelch debounce + LED, factory-reset button,
//! OTA app-validate, and periodic status logging. The audio frame pump runs on
//! its own core-1 thread and fans each frame out to the telegram decoder
//! (unmuted, pre-squelch) and the /stream.wav broadcast ring (squelch-muted).
//! The web server, backend uplink and OTA updater are added by later modules
//! at the marked spawn points; everything they need is grouped in
//! [`SharedState`].

mod audio;
mod board;
mod broadcast_ring;
mod config;
mod decoder;
mod frames;
mod ota;
mod radio;
mod rt;
mod stream;
mod uplink;
mod web;
mod wifi;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::gpio::{AnyIOPin, PinDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::uart::{config as uart_config, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys;

use crate::audio::{AdcCapture, AudioPipeline};
use crate::broadcast_ring::BroadcastRing;
use crate::config::{AppState, FRAME_SAMPLES_16K};
use crate::decoder::{AudioFrame, DecoderShared};
use crate::frames::Frames;
use crate::ota::OtaStatus;
use crate::radio::Radio;
use crate::stream::StreamStats;
use crate::uplink::UplinkStatus;
use crate::wifi::WifiManager;

const OTA_VALIDATE_FALLBACK: Duration = Duration::from_secs(90);

/// Handles the phase-3 modules (web UI, uplink, OTA updater) need, grouped so
/// each can take one clone. Everything is `Arc`d and thread-safe:
///   * web UI:  `app` (config/channels + generation bumps), `frames` (history
///     snapshot + stats), `stream_stats`, `decoder` (requested proto + feed
///     drops), `ota_in_progress` (set during upload so the streamer drops
///     clients), `squelch_open`.
///   * uplink:  `frames` (queue `pop_timeout`/`drain` + sent/accepted/dropped
///     stats), `app` (uplink URL/token + `gen.uplink`), `uplink` (status for
///     the web UI), `wifi_connected` (send gating).
///   * OTA:     `ota_in_progress`, `app` (update URL + `gen.ota`), `ota`
///     (status + wake), `wifi_connected`.
#[derive(Clone)]
#[allow(dead_code)] // consumed by the phase-3 modules
pub struct SharedState {
    pub app: Arc<AppState>,
    pub frames: Arc<Frames>,
    pub ring: Arc<BroadcastRing>,
    pub stream_stats: Arc<StreamStats>,
    pub decoder: Arc<DecoderShared>,
    pub ota_in_progress: Arc<AtomicBool>,
    pub squelch_open: Arc<AtomicBool>,
    /// Backend uplink status (mode/connected/last-error) for `/api/status`.
    pub uplink: Arc<UplinkStatus>,
    /// OTA updater status (+ the wake condvar the web POST kicks).
    pub ota: Arc<OtaStatus>,
    /// WiFi association state, published by the supervisor loop and read by the
    /// uplink/OTA tasks to gate network I/O (C `WiFi.status() == WL_CONNECTED`).
    pub wifi_connected: Arc<AtomicBool>,
}

fn main() -> anyhow::Result<()> {
    // Required by esp-idf-sys to keep runtime patches from being stripped.
    sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!(
        "===== kv4p RX streamer v{} (build {}) =====",
        config::FIRMWARE_VERSION,
        config::FIRMWARE_BUILD
    );

    let peripherals = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    // --- Board detection (NVS hwconfig override, else strap pins GPIO39/36) ---
    let pin39 = PinDriver::input(peripherals.pins.gpio39)?;
    let pin36 = PinDriver::input(peripherals.pins.gpio36)?;
    let hw = board::detect(&nvs, pin39, pin36);
    log::info!(
        "[board] rf={:?} sq_pin={} atten={} volume={} hl_pin={}",
        hw.rf_module_type,
        hw.pins.pin_sq,
        hw.adc_attenuation,
        hw.volume,
        hw.pins.pin_hl
    );

    // --- Config + channel table ---
    let (cfg, channels) = config::load(&nvs, &hw);

    // --- Static pin setup (mirrors the .ino setup(): radio powered, RX only) ---
    rt::gpio::output(hw.pins.pin_pd);
    rt::gpio::set(hw.pins.pin_pd, true);
    rt::gpio::input(hw.pins.pin_sq, false);
    rt::gpio::output(hw.pins.pin_ptt);
    rt::gpio::set(hw.pins.pin_ptt, true); // HIGH = RX (PTT never asserted)
    if hw.pins.pin_hl != -1 {
        rt::gpio::output(hw.pins.pin_hl);
        rt::gpio::set(hw.pins.pin_hl, true); // low power (TX unused)
    }
    rt::gpio::output(hw.pins.pin_led);
    // Physical PTT buttons double as the factory-reset trigger.
    rt::gpio::input(hw.pins.pin_ptt_phys1, true);
    rt::gpio::input(hw.pins.pin_ptt_phys2, true);

    // Watchdog: reboot if the supervisor loop stalls. Worker tasks that may
    // block on sockets are deliberately NOT subscribed (see .ino rationale).
    rt::wdt_subscribe_current();

    let state = Arc::new(AppState::new(hw, cfg, channels, nvs.clone()));

    // --- Radio (SA818 over UART1, 9600 8N1) ---
    // Pin numbers come from the board description so a factory NVS `hwconfig`
    // override is honored like the C `Serial2.begin(..., pinRfModuleRxd,
    // pinRfModuleTxd)`. SAFETY: these two pins are owned by the RF UART for
    // the firmware's lifetime; nothing else drives them (board.rs defaults or
    // the factory blob, same exclusivity contract as the C).
    let ucfg = uart_config::Config::new().baudrate(Hertz(9600));
    let (rf_tx, rf_rx) = unsafe {
        (
            AnyIOPin::new(hw.pins.pin_rf_module_txd as i32),
            AnyIOPin::new(hw.pins.pin_rf_module_rxd as i32),
        )
    };
    let uart = UartDriver::new(
        peripherals.uart1,
        rf_tx,
        rf_rx,
        None::<AnyIOPin>,
        None::<AnyIOPin>,
        &ucfg,
    )?;
    let mut radio = Radio::new(uart, hw.rf_module_type);
    heap_log("boot");
    {
        let cfg = state.config.read().unwrap();
        let ch = state.channels.read().unwrap();
        radio.init(&cfg, &ch);
    }
    heap_log("after radio init");

    // --- Audio: DAC bias, ADC continuous capture, DSP pipeline ---
    audio::dac_bias(hw.adc_bias);
    let adc = AdcCapture::new(board::I2S_ADC_CHANNEL, hw.adc_attenuation);
    let pipe = AudioPipeline::new();
    heap_log("after audio init");

    // squelch_open: supervisor writes it, the audio pump reads it to mute.
    let squelch_open = Arc::new(AtomicBool::new(false));

    // --- Streaming/decoding shared state ---
    // The 32 kB ring itself is allocated lazily on the first stream client.
    let ring = Arc::new(BroadcastRing::new());
    let frames = Arc::new(Frames::new());
    let stream_stats = Arc::new(StreamStats::default());
    let ota_in_progress = Arc::new(AtomicBool::new(false));
    let dec_shared = Arc::new(DecoderShared::default());
    // Bounded frame channel to the decoder; the audio pump try_sends (never
    // blocks) and the decoder drains (replaces the C decRing SPSC ring).
    let (dec_tx, dec_rx) = std::sync::mpsc::sync_channel::<AudioFrame>(decoder::DEC_CHANNEL_DEPTH);

    // Audio frame pump on core 1: decoder feed is unmuted (pre-squelch), the
    // squelch mute applies to the stream copy only (audio.h ordering).
    {
        let state_a = state.clone();
        let squelch_a = squelch_open.clone();
        let ring_a = ring.clone();
        let dec_shared_a = dec_shared.clone();
        let mut adc = adc;
        let mut pipe = pipe;
        // 12 kB stack: audio_task's frame buffers (~2 kB) + the FIR scratch
        // (~1.6 kB) + the decoder feed's by-value frame copy (~2.9 kB peak).
        rt::spawn(b"audio\0", 12288, 10, Core::Core1, move || {
            audio::audio_task(
                &state_a,
                &mut adc,
                &mut pipe,
                &squelch_a,
                |b48k, b16k| decoder::feed(&dec_shared_a, &dec_tx, b48k, b16k),
                |b16k| {
                    // 16 kHz PCM16LE into the /stream.wav broadcast ring.
                    let mut bytes = [0u8; FRAME_SAMPLES_16K * 2];
                    for (i, s) in b16k.iter().enumerate() {
                        bytes[i * 2..i * 2 + 2].copy_from_slice(&s.to_le_bytes());
                    }
                    ring_a.write(&bytes);
                },
            );
        })?;
    }

    // --- WiFi / mDNS / SNTP ---
    let mut wifi = {
        let cfg = state.config.read().unwrap();
        WifiManager::new(peripherals.modem, sysloop, nvs.clone(), &cfg)?
    };
    // The C firmware disables modem sleep (WiFi.setSleep(false)) to avoid
    // latency spikes in the audio stream; same call, now the streamer exists.
    unsafe {
        sys::esp_wifi_set_ps(sys::wifi_ps_type_t_WIFI_PS_NONE);
    }
    heap_log("after wifi setup");

    // --- /stream.wav streamer (streamer.h) + telegram decoder (decoder.h) ---
    {
        let port = state.config.read().unwrap().stream_port;
        stream::start(
            port,
            ring.clone(),
            stream_stats.clone(),
            ota_in_progress.clone(),
        )?;
    }
    decoder::start(state.clone(), frames.clone(), dec_shared.clone(), dec_rx)?;
    heap_log("after streamer+decoder start");

    // WiFi association flag (supervisor publishes it; uplink/OTA gate on it).
    let wifi_connected = Arc::new(AtomicBool::new(false));

    // Everything the phase-3 modules consume, in one bundle.
    let shared = SharedState {
        app: state.clone(),
        frames,
        ring,
        stream_stats,
        decoder: dec_shared,
        ota_in_progress,
        squelch_open: squelch_open.clone(),
        uplink: Arc::new(UplinkStatus::default()),
        ota: Arc::new(OtaStatus::default()),
        wifi_connected: wifi_connected.clone(),
    };

    // Backend uplink (uplink.h) + OTA auto-updater (otaUpdate.h): both core-0
    // worker threads. The web server (webui.h) owns the ESP-IDF httpd task; its
    // handle is held for the process lifetime so the server keeps serving.
    uplink::start(shared.clone())?;
    ota::start(shared.clone())?;
    let _web = web::start(shared.clone())?;
    heap_log("after phase-3 start");

    log::info!(
        "[setup] done. stream on :{}/stream.wav",
        state.config.read().unwrap().stream_port
    );

    supervisor_loop(&state, &mut radio, &mut wifi, &squelch_open, &wifi_connected);
}

fn heap_log(stage: &str) {
    unsafe {
        log::info!("[heap] {}: free={}", stage, sys::esp_get_free_heap_size());
    }
}

/// Core-0 supervisor: squelch debounce/LED, factory-reset button, radio
/// re-tune on config change, WiFi/mDNS/SNTP upkeep, OTA app-validate and status
/// logging. Ports the .ino `loop()` and its per-function static state. Never
/// returns.
fn supervisor_loop(
    state: &AppState,
    radio: &mut Radio<'_>,
    wifi: &mut WifiManager,
    squelch_open: &AtomicBool,
    wifi_connected: &AtomicBool,
) -> ! {
    let hw = state.hw;

    // squelchLoop() state.
    let mut sq_raw = false;
    let mut sq_last_change = Instant::now();

    // resetButtonLoop() state.
    let mut held_since: Option<Instant> = None;

    // otaValidateLoop() state.
    let mut ota_marked = false;
    let boot = Instant::now();

    // statusLoop() state.
    let mut status_last = Instant::now();

    // Radio re-tune tracking (generation counter bumped on config change).
    let mut applied_radio_gen = 0u32;

    loop {
        // --- squelch debounce (30 ms) + LED ---
        let now_open = !rt::gpio::get(hw.pins.pin_sq); // LOW = squelch open
        if now_open != sq_raw {
            sq_raw = now_open;
            sq_last_change = Instant::now();
        } else if sq_last_change.elapsed() > Duration::from_millis(30)
            && squelch_open.load(Ordering::Relaxed) != sq_raw
        {
            squelch_open.store(sq_raw, Ordering::Relaxed);
            rt::gpio::set(hw.pins.pin_led, sq_raw);
        }

        // --- factory-reset button (hold either physical PTT for 10 s) ---
        let pressed =
            !rt::gpio::get(hw.pins.pin_ptt_phys1) || !rt::gpio::get(hw.pins.pin_ptt_phys2);
        if !pressed {
            held_since = None;
        } else {
            match held_since {
                None => held_since = Some(Instant::now()),
                Some(since) => {
                    let held = since.elapsed();
                    if held >= Duration::from_secs(10) {
                        log::warn!("[reset] factory reset via PTT hold");
                        rt::gpio::set(hw.pins.pin_led, true);
                        std::thread::sleep(Duration::from_millis(300));
                        config::factory_reset(&state.nvs); // diverges (restart)
                    } else if held >= Duration::from_secs(5) {
                        let on = (held.as_millis() / 100) % 2 != 0;
                        rt::gpio::set(hw.pins.pin_led, on);
                    }
                }
            }
        }

        // --- radio re-tune when config/channels changed ---
        let g = state.gen.radio.load(Ordering::SeqCst);
        if g != applied_radio_gen {
            applied_radio_gen = g;
            if let (Ok(cfg), Ok(ch)) = (state.config.read(), state.channels.read()) {
                radio.apply_tuning(&cfg, &ch);
            }
        }

        // --- WiFi / mDNS / SNTP upkeep ---
        if let Ok(cfg) = state.config.read() {
            wifi.poll(&cfg);
        }
        // Publish association state for the uplink/OTA network-I/O gating.
        let connected = wifi.is_connected();
        wifi_connected.store(connected, Ordering::Relaxed);

        // --- OTA app-validate (cancel rollback), once WiFi is up or after 90 s ---
        if !ota_marked && (connected || boot.elapsed() > OTA_VALIDATE_FALLBACK) {
            ota_marked = true;
            match esp_idf_svc::ota::EspOta::new().and_then(|mut o| o.mark_running_slot_valid()) {
                Ok(()) => log::info!("[ota] app marked valid, rollback cancelled"),
                Err(e) => log::warn!("[ota] mark valid failed: {e:?}"),
            }
        }

        // --- status log every 5 s ---
        if status_last.elapsed() >= Duration::from_secs(5) {
            status_last = Instant::now();
            unsafe {
                log::info!(
                    "[status] heap={} wifi={} radio={} sq={}",
                    sys::esp_get_free_heap_size(),
                    wifi.is_connected() as u8,
                    radio.found() as u8,
                    squelch_open.load(Ordering::Relaxed) as u8
                );
            }
        }

        rt::wdt_reset();
        std::thread::sleep(Duration::from_millis(5));
    }
}
