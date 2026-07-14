//! Audio capture and DSP, port of the C `audio.h`.
//!
//! Chain (mirrors the upstream RX path, minus ADPCM/AFSK):
//! ADC continuous DMA @ 48 kHz on GPIO34/ADC1 -> DC removal -> x16 gain ->
//! 65-tap Hamming windowed-sinc FIR low-pass (fc 6.8 kHz) + decimate /3 ->
//! PCM16 @ 16 kHz.
//!
//! The DSP is written as pure functions/structs (no ESP-IDF deps) so it is
//! unit-testable on the host; only [`AdcCapture`] and [`dac_bias`] touch the
//! peripheral FFI.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use esp_idf_svc::sys;

use crate::config::{AppState, CAPTURE_SAMPLE_RATE, FRAME_SAMPLES_16K, FRAME_SAMPLES_48K};

pub const FIR_TAPS: usize = 65;

/// Peak absolute sample of the most recent pre-squelch 16 kHz frame (0..32767),
/// published by the frame pump for the `[status]` log.
///
/// This is the gauge that separates "the RF/ADC path is dead" from "there is
/// audio but the decoder isn't configured" — the two failure modes look
/// identical in the frame counters alone. Pre-squelch on purpose: with the
/// squelch closed the stream copy is muted, but this still shows the noise
/// floor, so a plausible reading here proves the ADC is alive even on a silent
/// channel.
#[derive(Default)]
pub struct AudioLevel {
    peak: AtomicU32,
}

/// ADC delivery counters, for diagnosing a starved capture path. `accepted`
/// should track `conversions` almost exactly and `conversions` should advance
/// at CAPTURE_SAMPLE_RATE; any gap localizes the loss (channel-tag mismatch vs.
/// the DMA simply not producing).
#[derive(Default)]
pub struct AdcStats {
    /// 16-bit conversion words parsed out of the DMA buffer.
    pub conversions: AtomicU32,
    /// Conversions whose channel tag matched, i.e. actually kept.
    pub accepted: AtomicU32,
    /// Conversions discarded because the channel tag did not match.
    pub ch_mismatch: AtomicU32,
    /// `adc_continuous_read` calls that returned nothing.
    pub timeouts: AtomicU32,
    /// First raw DMA word seen, for decoding the on-wire format once.
    pub first_word: AtomicU32,
}

/// DMA pool overflows, counted in the driver's ISR callback. Non-zero means the
/// ADC produced faster than the audio thread consumed, so the driver dropped
/// samples — the stream then has *gaps*, which is far more destructive to FFSK
/// than a mere rate error: the bit clock slips at every gap. Static rather than
/// carried through `user_data` because the callback runs in ISR context.
pub static ADC_POOL_OVF: AtomicU32 = AtomicU32::new(0);

// Must live in IRAM: sdkconfig sets CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE=y, and
// the driver rejects a callback in flash (ESP_ERR_INVALID_ARG) — which silently
// kills ADC bring-up entirely.
#[link_section = ".iram1.adc_pool_ovf"]
unsafe extern "C" fn on_pool_ovf(
    _h: sys::adc_continuous_handle_t,
    _e: *const sys::adc_continuous_evt_data_t,
    _user: *mut core::ffi::c_void,
) -> bool {
    ADC_POOL_OVF.fetch_add(1, Ordering::Relaxed);
    false // no task woken
}

impl AudioLevel {
    pub fn update(&self, buf16k: &[i16]) {
        let peak = buf16k
            .iter()
            .map(|s| s.unsigned_abs() as u32)
            .max()
            .unwrap_or(0);
        self.peak.store(peak, Ordering::Relaxed);
    }

    pub fn peak(&self) -> u32 {
        self.peak.load(Ordering::Relaxed)
    }
}

// --- One-pole DC blocker (same time constant as upstream's DCOffsetRemover) ---

pub struct DcBlocker {
    alpha: f32,
    prev: f32,
}

impl DcBlocker {
    pub fn new() -> DcBlocker {
        // dcAlpha = 1 - exp(-1 / (fs * (0.25 / ln2)))
        let alpha = 1.0 - (-1.0 / (CAPTURE_SAMPLE_RATE as f32 * (0.25 / f32::ln(2.0)))).exp();
        DcBlocker { alpha, prev: 0.0 }
    }

    #[inline]
    pub fn process(&mut self, x: i16) -> i16 {
        self.prev = self.alpha * x as f32 + (1.0 - self.alpha) * self.prev;
        // Matches C `(int16_t)(x - (int16_t)dcPrev)`; ADC data stays in 0..4095
        // so neither the float->int truncation nor the subtraction overflows i16.
        ((x as i32) - (self.prev as i16 as i32)) as i16
    }
}

impl Default for DcBlocker {
    fn default() -> Self {
        DcBlocker::new()
    }
}

/// x16 gain with i16 clamp (12-bit ADC data -> full 16-bit scale).
#[inline]
pub fn gain16(x: i16) -> i16 {
    (x as i32 * 16).clamp(-32768, 32767) as i16
}

/// Windowed-sinc low-pass FIR coefficients, fc 6.8 kHz @ 48 kHz, Hamming
/// window, normalized to unity DC gain. Identical to C `designFir()`.
pub fn design_fir() -> [f32; FIR_TAPS] {
    let fc = 6800.0f32 / CAPTURE_SAMPLE_RATE as f32;
    let m = (FIR_TAPS - 1) as f32; // 64
    let mut coeffs = [0.0f32; FIR_TAPS];
    let mut sum = 0.0f32;
    for k in 0..FIR_TAPS {
        let n = k as f32 - m / 2.0;
        let sinc = if n == 0.0 {
            2.0 * fc
        } else {
            (2.0 * std::f32::consts::PI * fc * n).sin() / (std::f32::consts::PI * n)
        };
        let hamming = 0.54 - 0.46 * (2.0 * std::f32::consts::PI * k as f32 / m).cos();
        coeffs[k] = sinc * hamming;
        sum += coeffs[k];
    }
    for c in coeffs.iter_mut() {
        *c /= sum;
    }
    coeffs
}

/// Decimating FIR: 48 kHz in -> 16 kHz out. Keeps the FIR history across
/// frames so no samples are lost at frame edges. Port of C `decimateFrame()`.
pub struct Decimator {
    coeffs: [f32; FIR_TAPS],
    history: [i16; FIR_TAPS - 1],
}

impl Decimator {
    pub fn new() -> Decimator {
        Decimator {
            coeffs: design_fir(),
            history: [0; FIR_TAPS - 1],
        }
    }

    /// `input` is [`FRAME_SAMPLES_48K`] samples, `output` is
    /// [`FRAME_SAMPLES_16K`] samples.
    pub fn process(&mut self, input: &[i16], output: &mut [i16]) {
        debug_assert_eq!(input.len(), FRAME_SAMPLES_48K);
        debug_assert_eq!(output.len(), FRAME_SAMPLES_16K);
        const H: usize = FIR_TAPS - 1;
        let mut ext = [0i16; (FIR_TAPS - 1) + FRAME_SAMPLES_48K];
        ext[..H].copy_from_slice(&self.history);
        ext[H..].copy_from_slice(input);
        for j in 0..FRAME_SAMPLES_16K {
            let base = H + j * crate::config::DECIMATION_RATIO;
            let mut acc = 0.0f32;
            for k in 0..FIR_TAPS {
                acc += self.coeffs[k] * ext[base - k] as f32;
            }
            acc = acc.clamp(-32768.0, 32767.0);
            output[j] = acc.round() as i16;
        }
        self.history
            .copy_from_slice(&ext[FRAME_SAMPLES_48K..FRAME_SAMPLES_48K + H]);
    }
}

impl Default for Decimator {
    fn default() -> Self {
        Decimator::new()
    }
}

/// Full per-frame DSP pipeline: raw ADC frame -> processed 48 kHz + decimated
/// 16 kHz. Neither output is muted here (squelch mute is applied by the caller
/// to the stream copy only).
pub struct AudioPipeline {
    dc: DcBlocker,
    decim: Decimator,
}

impl AudioPipeline {
    pub fn new() -> AudioPipeline {
        AudioPipeline {
            dc: DcBlocker::new(),
            decim: Decimator::new(),
        }
    }

    /// Process one frame in place: `buf48k` arrives as raw ADC samples and is
    /// overwritten with DC-blocked, x16-gained samples; `buf16k` receives the
    /// decimated output.
    pub fn process_frame(&mut self, buf48k: &mut [i16], buf16k: &mut [i16]) {
        for s in buf48k.iter_mut() {
            *s = gain16(self.dc.process(*s));
        }
        self.decim.process(buf48k, buf16k);
    }
}

impl Default for AudioPipeline {
    fn default() -> Self {
        AudioPipeline::new()
    }
}

// --- DAC bias (biases the ADC analog front-end; board requirement) ---

/// Drive DAC on GPIO26 (DAC_CHAN_1 in IDF5, "channel 2" in the legacy Arduino
/// API) to the fixed bias voltage. Mirrors C `injectADCBias()`.
pub fn dac_bias(adc_bias_volts: f32) {
    let value = ((255.0f32 / 3.3f32) * adc_bias_volts) as u8;
    unsafe {
        let mut cfg: sys::dac_oneshot_config_t = core::mem::zeroed();
        cfg.chan_id = sys::dac_channel_t_DAC_CHAN_1;
        let mut handle: sys::dac_oneshot_handle_t = core::ptr::null_mut();
        if sys::dac_oneshot_new_channel(&cfg, &mut handle) == sys::ESP_OK {
            sys::dac_oneshot_output_voltage(handle, value);
        }
    }
}

// --- ADC continuous DMA capture ---

// ESP32 DMA result is `adc_digi_output_data_t` TYPE1: 2 bytes, bits[11:0]=data,
// bits[15:12]=channel. VERIFY(phase2): SOC_ADC_DIGI_RESULT_BYTES == 2 on ESP32.
const RESULT_BYTES: usize = 2;
/// Per-read size. Keep at 1 kB: `AdcCapture` holds `dma`/`pending` inline and
/// lives on the audio thread's stack, which overflows if these grow.
const DMA_READ_BYTES: usize = 1024;
/// DMA pool depth (driver-allocated, not on our stack). 4 kB is ~42 ms at
/// 48 kHz, which measures as ample: with the audio thread on core 1 the pool
/// never overflows (`poolovf` stays 0). Enlarging it was tried and changed
/// nothing except costing ~28 kB of heap, which the uplink's TLS handshake
/// needs more.
const DMA_POOL_BYTES: usize = DMA_READ_BYTES * 4;

/// Rate we ASK the driver for, to actually get [`CAPTURE_SAMPLE_RATE`].
///
/// On this ESP32 + IDF 5.3, `adc_continuous` delivers a stable 0.8192x of the
/// requested `sample_freq_hz` (measured: request 48000 -> 39321.6 Hz, over many
/// 5 s windows, with zero pool overflows and zero dropped conversions — so it
/// is the hardware clock, not lost samples). Undoing that factor here keeps the
/// true capture rate at 48 kHz, which the whole DSP chain and the 16 kHz stream
/// header depend on. Getting this wrong does not sound broken — it silently
/// skews the FFSK bit clock and every telegram fails CRC.
const ADC_REQUEST_HZ: u32 = (CAPTURE_SAMPLE_RATE as f32 / 0.8192) as u32; // 58593

/// ADC1 continuous (I2S-DMA-backed) capture on a single channel. Written
/// against the raw `adc_continuous_*` FFI rather than the HAL wrapper, which
/// does not cover classic ESP32 uniformly.
pub struct AdcCapture {
    handle: sys::adc_continuous_handle_t,
    channel: u8,
    stats: std::sync::Arc<AdcStats>,
    dma: [u8; DMA_READ_BYTES],
    /// Samples decoded from the last DMA burst that did not fit the caller's
    /// buffer; drained first on the next read (a DMA burst never yields more
    /// than DMA_READ_BYTES / RESULT_BYTES samples). The C AudioTools path kept
    /// its own residual buffer the same way — nothing is ever discarded.
    pending: [i16; DMA_READ_BYTES / RESULT_BYTES],
    pending_len: usize,
    pending_off: usize,
}

// SAFETY: the driver handle is created on the main thread and then moved into
// (and only ever used from) the audio thread; `adc_continuous_*` has no
// thread-affinity requirement, it just isn't concurrently reentrant.
unsafe impl Send for AdcCapture {}

impl AdcCapture {
    /// Start continuous capture on `channel` (ADC1) at 48 kHz, 12-bit, with the
    /// board's attenuation. Mirrors the ADC portion of C `audioInit()`.
    pub fn new(channel: u8, atten: sys::adc_atten_t, stats: std::sync::Arc<AdcStats>) -> AdcCapture {
        let handle = unsafe {
            let mut hcfg: sys::adc_continuous_handle_cfg_t = core::mem::zeroed();
            hcfg.max_store_buf_size = DMA_POOL_BYTES as u32;
            hcfg.conv_frame_size = DMA_READ_BYTES as u32;
            let mut handle: sys::adc_continuous_handle_t = core::ptr::null_mut();
            let mut err = sys::adc_continuous_new_handle(&hcfg, &mut handle);

            if err == sys::ESP_OK {
                let mut pattern: sys::adc_digi_pattern_config_t = core::mem::zeroed();
                pattern.atten = atten as u8;
                pattern.channel = channel;
                pattern.unit = sys::adc_unit_t_ADC_UNIT_1 as u8;
                pattern.bit_width = sys::adc_bitwidth_t_ADC_BITWIDTH_12 as u8;

                let mut ccfg: sys::adc_continuous_config_t = core::mem::zeroed();
                ccfg.pattern_num = 1;
                ccfg.adc_pattern = &mut pattern;
                ccfg.sample_freq_hz = ADC_REQUEST_HZ;
                ccfg.conv_mode = sys::adc_digi_convert_mode_t_ADC_CONV_SINGLE_UNIT_1;
                ccfg.format = sys::adc_digi_output_format_t_ADC_DIGI_OUTPUT_FORMAT_TYPE1;
                err = sys::adc_continuous_config(handle, &ccfg);
            }
            if err == sys::ESP_OK {
                let mut cbs: sys::adc_continuous_evt_cbs_t = core::mem::zeroed();
                cbs.on_pool_ovf = Some(on_pool_ovf);
                err = sys::adc_continuous_register_event_callbacks(
                    handle,
                    &cbs,
                    core::ptr::null_mut(),
                );
            }
            if err == sys::ESP_OK {
                err = sys::adc_continuous_start(handle);
            }
            if err != sys::ESP_OK {
                // Match the C posture (a failed AudioTools begin() was logged
                // and the firmware carried on serving the web UI): capture
                // stays silent but diagnosable instead of aborting boot.
                log::error!("[audio] ADC continuous bring-up failed: esp_err {err}");
                if !handle.is_null() {
                    sys::adc_continuous_deinit(handle);
                }
                handle = core::ptr::null_mut();
            }
            handle
        };
        AdcCapture {
            handle,
            channel,
            stats,
            dma: [0u8; DMA_READ_BYTES],
            pending: [0i16; DMA_READ_BYTES / RESULT_BYTES],
            pending_len: 0,
            pending_off: 0,
        }
    }

    /// Extract this channel's 12-bit samples into `out`, draining any residue
    /// from the previous DMA burst first, then reading a fresh burst if there
    /// is still room. Samples that don't fit `out` are kept in `pending` for
    /// the next call — nothing is ever discarded. Returns the number of
    /// samples written (0 on timeout or failed bring-up). Waits up to
    /// `timeout_ms` for the DMA buffer.
    pub fn read_samples(&mut self, out: &mut [i16], timeout_ms: u32) -> usize {
        if self.handle.is_null() {
            // Failed bring-up: behave like a timed-out read instead of spinning.
            std::thread::sleep(core::time::Duration::from_millis(timeout_ms as u64));
            return 0;
        }
        let mut n = 0usize;

        // Drain the residue from the previous burst first.
        while self.pending_off < self.pending_len && n < out.len() {
            out[n] = self.pending[self.pending_off];
            self.pending_off += 1;
            n += 1;
        }
        if n == out.len() {
            return n;
        }
        self.pending_len = 0;
        self.pending_off = 0;

        let mut got: u32 = 0;
        let err = unsafe {
            sys::adc_continuous_read(
                self.handle,
                self.dma.as_mut_ptr(),
                self.dma.len() as u32,
                &mut got,
                timeout_ms,
            )
        };
        if err != sys::ESP_OK || got == 0 {
            self.stats.timeouts.fetch_add(1, Ordering::Relaxed);
            return n;
        }
        let mut i = 0usize;
        while i + RESULT_BYTES <= got as usize {
            let raw = u16::from_le_bytes([self.dma[i], self.dma[i + 1]]);
            i += RESULT_BYTES;
            self.stats.conversions.fetch_add(1, Ordering::Relaxed);
            self.stats
                .first_word
                .compare_exchange(0, raw as u32, Ordering::Relaxed, Ordering::Relaxed)
                .ok();
            let data = raw & 0x0FFF;
            let ch = ((raw >> 12) & 0x0F) as u8;
            if ch != self.channel {
                self.stats.ch_mismatch.fetch_add(1, Ordering::Relaxed);
                continue;
            }
            self.stats.accepted.fetch_add(1, Ordering::Relaxed);
            if n < out.len() {
                out[n] = data as i16;
                n += 1;
            } else {
                self.pending[self.pending_len] = data as i16;
                self.pending_len += 1;
            }
        }
        n
    }
}

/// Frame pump: continuously pull 15 ms frames, run the DSP, and hand each frame
/// to the decoder (unmuted 48 kHz + 16 kHz) and to the stream sink (16 kHz,
/// squelch-muted). Never returns; run on its own thread. Port of C
/// `audioLoop()` (ordering preserved: decoder feed is unmuted, mute applies to
/// the stream copy only).
pub fn audio_task(
    state: &AppState,
    adc: &mut AdcCapture,
    pipe: &mut AudioPipeline,
    squelch_open: &AtomicBool,
    mut decoder_feed: impl FnMut(&[i16], &[i16]),
    mut stream_write: impl FnMut(&[i16]),
) -> ! {
    // The C audioLoop ran on the WDT-subscribed loopTask; subscribe this
    // thread so a wedged ADC driver still trips the 25 s watchdog and reboots
    // the (remotely mounted) device. Without this, the wdt_reset calls below
    // are silent no-ops (ESP_ERR_NOT_FOUND).
    crate::rt::wdt_subscribe_current();

    let mut buf48k = [0i16; FRAME_SAMPLES_48K];
    let mut buf16k = [0i16; FRAME_SAMPLES_16K];
    let mut filled = 0usize;
    loop {
        let got = adc.read_samples(&mut buf48k[filled..], 100);
        filled += got;
        unsafe {
            sys::esp_task_wdt_reset();
        }
        if filled < FRAME_SAMPLES_48K {
            continue;
        }
        filled = 0;

        pipe.process_frame(&mut buf48k, &mut buf16k);

        // Decoders see unmuted audio (they gate on their own envelopes).
        decoder_feed(&buf48k, &buf16k);

        let mute = state
            .config
            .read()
            .map(|c| c.mute_when_closed)
            .unwrap_or(true);
        if mute && !squelch_open.load(Ordering::Relaxed) {
            buf16k.fill(0);
        }
        stream_write(&buf16k);
        unsafe {
            sys::esp_task_wdt_reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gain_clamps() {
        assert_eq!(gain16(3000), 32767);
        assert_eq!(gain16(-3000), -32768);
        assert_eq!(gain16(100), 1600);
    }

    #[test]
    fn fir_unity_dc_gain() {
        let c = design_fir();
        let sum: f32 = c.iter().sum();
        assert!((sum - 1.0).abs() < 1e-4, "sum={sum}");
    }

    #[test]
    fn fir_symmetric() {
        let c = design_fir();
        for k in 0..FIR_TAPS / 2 {
            assert!((c[k] - c[FIR_TAPS - 1 - k]).abs() < 1e-6);
        }
    }

    #[test]
    fn decimator_dc_passes() {
        // A constant input should decimate to (near) the same constant.
        let mut d = Decimator::new();
        let input = [1000i16; FRAME_SAMPLES_48K];
        let mut out = [0i16; FRAME_SAMPLES_16K];
        // Prime the history so the tail of the frame is in steady state.
        d.process(&input, &mut out);
        d.process(&input, &mut out);
        assert!((out[FRAME_SAMPLES_16K - 1] - 1000).abs() <= 2);
    }

    #[test]
    fn dc_blocker_removes_offset() {
        let mut dc = DcBlocker::new();
        // Feed a constant offset; after convergence output trends toward 0.
        let mut last = 0i16;
        for _ in 0..20000 {
            last = dc.process(2048);
        }
        assert!(last.abs() < 50, "residual={last}");
    }
}
