//! SA818 (DRA818-compatible) RF module driver over UART1.
//!
//! Port of the C `radio.h`, with the AT protocol hand-rolled to reproduce the
//! exact bytes the `arduino-dra818` library emitted (so the module sees an
//! identical command stream):
//!
//!   * handshake:  `AT+DMOCONNECT\r\n`, success when the reply char before CRLF
//!                 is `'0'` (`+DMOCONNECT:0`). 3 inner sends x 3 outer tries.
//!   * group:      `AT+DMOSETGROUP=<bw>,<tx 8.4f>,<rx 8.4f>,0000,<sq>,0000\r\n`
//!   * volume:     `AT+DMOSETVOLUME=<v>\r\n`
//!   * filters:    `AT+SETFILTER=<!pre>,<!high>,<!low>\r\n`
//!
//! Filters are bypassed unconditionally: the DRA818 library *inverts* each
//! boolean, so `filters(false,false,false)` transmits `AT+SETFILTER=1,1,1`
//! (all filters OFF). A flat audio path is mandatory because data channels
//! carry FFSK/NEMO modem signals; if the bypass command fails the radio is
//! marked absent rather than left shaping the audio.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use esp_idf_svc::hal::uart::UartDriver;
use esp_idf_svc::sys;

use crate::board::RfModuleType;
use crate::config::{self, ChannelTable, Config};

/// Mirror of the C `radioModuleFound` global. The `Radio` itself lives on the
/// supervisor loop (not `Arc`-shared), so this static lets the web status
/// handler report `moduleFound` without owning the driver.
static RADIO_MODULE_FOUND: AtomicBool = AtomicBool::new(false);

/// Whether the SA818 answered its handshake and accepted the filter bypass.
pub fn module_found() -> bool {
    RADIO_MODULE_FOUND.load(Ordering::Relaxed)
}

// DRA818 constants (DRA818.h).
const DRA818_12K5: u8 = 0x0;
const DRA818_25K: u8 = 0x1;
const VHF_MIN: f32 = 134.0;
const VHF_MAX: f32 = 174.0;
const UHF_MIN: f32 = 400.0;
const SA8X8_UHF_MAX: f32 = 480.0;

const SQUELCH_MAX: u8 = 8;
const VOLUME_MAX: u8 = 8;
const VOLUME_MIN: u8 = 1;
const RESPONSE_TIMEOUT_MS: u64 = 2000; // DRA818 TIMEOUT
const HANDSHAKE_REPEAT: u8 = 3;

fn wdt_reset() {
    // Best-effort; no-op if the calling task isn't subscribed to the WDT.
    unsafe {
        sys::esp_task_wdt_reset();
    }
}

pub struct Radio<'d> {
    uart: UartDriver<'d>,
    module: RfModuleType,
    found: bool,
    // Change-detection caches (0xFF = "never applied"), mirroring radio.h.
    applied_freq_hz: u32,
    applied_bandwidth: u8,
    applied_squelch: u8,
    applied_volume: u8,
}

impl<'d> Radio<'d> {
    pub fn new(uart: UartDriver<'d>, module: RfModuleType) -> Radio<'d> {
        Radio {
            uart,
            module,
            found: false,
            applied_freq_hz: 0,
            applied_bandwidth: 0xFF,
            applied_squelch: 0xFF,
            applied_volume: 0xFF,
        }
    }

    pub fn found(&self) -> bool {
        self.found
    }

    fn send(&self, bytes: &[u8]) {
        let _ = self.uart.write(bytes);
    }

    fn drain(&self) {
        let mut b = [0u8; 64];
        // Discard whatever is buffered; timeout 0 returns immediately when empty.
        for _ in 0..64 {
            match self.uart.read(&mut b, 0) {
                Ok(n) if n > 0 => continue,
                _ => break,
            }
        }
    }

    /// Read until LF or the 2 s timeout; success when the char two positions
    /// before the terminating LF is `'0'` (identical to DRA818::read_response).
    fn read_response(&self) -> bool {
        let deadline = Instant::now() + Duration::from_millis(RESPONSE_TIMEOUT_MS);
        let mut win = [0u8; 3];
        while Instant::now() < deadline {
            let mut b = [0u8; 1];
            match self.uart.read(&mut b, 10) {
                Ok(1) => {
                    win[0] = win[1];
                    win[1] = win[2];
                    win[2] = b[0];
                    if b[0] == 0x0a {
                        break;
                    }
                }
                _ => {}
            }
        }
        win[0] == b'0'
    }

    fn handshake(&self) -> bool {
        for _ in 0..HANDSHAKE_REPEAT {
            self.send(b"AT+DMOCONNECT\r\n");
            if self.read_response() {
                return true;
            }
        }
        false
    }

    fn group(
        &self,
        bw: u8,
        freq_tx: f32,
        freq_rx: f32,
        ctcss_tx: u16,
        squelch: u8,
        ctcss_rx: u16,
    ) -> bool {
        // Clamp exactly as DRA818::group (CHECK macros).
        let bw = bw.clamp(DRA818_12K5, DRA818_25K);
        let (fmin, fmax) = if self.module == RfModuleType::Sa818Uhf {
            (UHF_MIN, SA8X8_UHF_MAX)
        } else {
            (VHF_MIN, VHF_MAX)
        };
        let ftx = freq_tx.clamp(fmin, fmax);
        let frx = freq_rx.clamp(fmin, fmax);
        let sq = squelch.min(SQUELCH_MAX);

        // dtostrf(freq, 8, 4) == "{:8.4}" (space-padded, 4 decimals).
        let cmd = format!(
            "AT+DMOSETGROUP={},{:8.4},{:8.4},{:04},{},{:04}\r\n",
            bw,
            ftx,
            frx,
            ctcss_tx,
            (b'0' + sq) as char,
            ctcss_rx,
        );
        self.send(cmd.as_bytes());
        self.read_response()
    }

    fn volume(&self, volume: u8) -> bool {
        let v = volume.clamp(VOLUME_MIN, VOLUME_MAX);
        let cmd = format!("AT+DMOSETVOLUME={}\r\n", v);
        self.send(cmd.as_bytes());
        self.read_response()
    }

    /// `pre`/`high`/`low` are the DRA818 semantics: the library inverts them, so
    /// `false` yields the digit `1` (filter bypassed).
    fn filters(&self, pre: bool, high: bool, low: bool) -> bool {
        let d = |on: bool| -> char {
            if on {
                '0'
            } else {
                '1'
            }
        };
        let cmd = format!("AT+SETFILTER={},{},{}\r\n", d(pre), d(high), d(low));
        self.send(cmd.as_bytes());
        self.read_response()
    }

    /// Bring the module up: handshake, mandatory filter bypass, initial tune.
    /// Mirrors C `initRadio()`.
    pub fn init(&mut self, cfg: &Config, channels: &ChannelTable) {
        self.found = false;
        // 3 outer tries; handshake() itself retries 3x with 2 s waits, giving the
        // module up to ~18 s to power up.
        for _ in 0..3 {
            wdt_reset();
            if self.handshake() {
                self.found = true;
                break;
            }
        }
        if !self.found {
            RADIO_MODULE_FOUND.store(false, Ordering::Relaxed);
            log::error!("[radio] SA818 module not responding!");
            return;
        }

        self.drain();
        let mut filters_ok = false;
        for _ in 0..5 {
            wdt_reset();
            if self.filters(false, false, false) {
                filters_ok = true;
                break;
            }
        }
        if !filters_ok {
            // Never proceed with a shaped audio path — data channels need it flat.
            log::error!("[radio] filter bypass failed, radio disabled");
            self.found = false;
            RADIO_MODULE_FOUND.store(false, Ordering::Relaxed);
            return;
        }

        RADIO_MODULE_FOUND.store(true, Ordering::Relaxed);
        self.apply_tuning(cfg, channels);
    }

    /// Re-tune to the active channel/VFO if anything changed. Mirrors C
    /// `applyRadioTuning()`.
    pub fn apply_tuning(&mut self, cfg: &Config, channels: &ChannelTable) -> bool {
        if !self.found {
            return false;
        }
        let (freq_hz, bandwidth) = config::active_tuning(cfg, channels);
        let freq_mhz = freq_hz as f32 / 1e6;
        let bw = if bandwidth == 1 {
            DRA818_25K
        } else {
            DRA818_12K5
        };

        if freq_hz != self.applied_freq_hz
            || bw != self.applied_bandwidth
            || cfg.squelch != self.applied_squelch
        {
            self.drain();
            let mut ok = false;
            for _ in 0..3 {
                wdt_reset();
                ok = self.group(bw, freq_mhz, freq_mhz, 0, cfg.squelch, 0);
                if ok {
                    break;
                }
            }
            if !ok {
                log::warn!("[radio] group({:.4} MHz) failed", freq_mhz);
                return false;
            }
            self.applied_freq_hz = freq_hz;
            self.applied_bandwidth = bw;
            self.applied_squelch = cfg.squelch;
            log::info!(
                "[radio] tuned {:.4} MHz bw={} sq={}",
                freq_mhz,
                if bw == DRA818_25K { "25k" } else { "12.5k" },
                cfg.squelch
            );
        }

        if cfg.volume != self.applied_volume {
            self.drain();
            if self.volume(cfg.volume) {
                self.applied_volume = cfg.volume;
                log::info!("[radio] volume={}", cfg.volume);
            }
        }
        true
    }
}
