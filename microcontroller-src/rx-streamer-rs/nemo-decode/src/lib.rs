//! NEMO / VicosLio ("G2") telegram decoder — a host-portable, dependency-free
//! Rust port of the **validated** on-air chain in `lio-decoder`
//! (`src/nemo_hysteresis_demod.py` front-end + `src/decode_air.py` framing/CRC).
//!
//! Physical layer (patent EP 0566773 B1): the FM baseband carries AMI-coded
//! cos^2 half-wave pulses of a ~2400 Hz tone (mark = pulse, space = gap), with
//! the polarity of every mark alternating. Real captures run the tone near
//! 2213 Hz, i.e. ~4427 Bd — **not** the nominal 2400/4800.
//!
//! Front-end (streaming, 48 kHz in) — the vehicle DSP's own receive path,
//! reverse-engineered from its firmware (`rpsL2aL1.c`):
//!
//! ```text
//!   x - moving_avg(x, 4 ms)        DC / baseline block
//!   -> Butterworth LP 3400 Hz      (BIPOLAR — no rectification: polarity is the code)
//!   -> decimate by 2               => signal @ 24 kHz
//!   -> energy gate                 => burst
//! ```
//!
//! Per burst: signed Schmitt comparator (the firmware's `receiveTriggerLevel` +/-
//! `receiveHysteresis`) -> pulses; baud estimated from the dominant tone;
//! per-pulse-resync symbol decode (patent claim 2: the symbol clock is reset on
//! every pulse) -> bits; HDLC deframe (EOF = run of six 1s, destuff, byte-offset
//! sweep) -> CRC.
//!
//! Two CRC conventions close on real captures (verified against the Python
//! references): LSB-first bytes + X.25 FCS (LE trailer) — what the backend
//! re-verifies — and MSB-first bytes + CRC-16/CCITT 0x1021 with the
//! `decode_air.py` `(init, xorout)` variants. Both are accepted; a caller (or
//! the server's `crc_ok`) decides what drives the map.
//!
//! # History: why the front-end was replaced
//!
//! This crate previously ported `bast_chain.py`, which rectified the signal
//! (destroying AMI polarity), recovered the clock with Mueller & Müller (whose
//! timing error accumulates across a telegram's long zero-runs), assumed exactly
//! 4800 Bd, and gated out any burst under 80 ms — longer than a whole telegram.
//! It decoded a synthesized fixture and, as its own test admitted, **never a real
//! capture**. On air it produced bursts and zero frames. The chain above is the
//! one that closes CRCs on real data.
//!
//! # Deviations from the Python reference
//!
//! Both are forced by the node's budget (~25-47 kB free heap, 240 MHz), and
//! neither changes the algorithm:
//!
//! * The low-pass is **causal** (biquad cascade) where the reference uses a
//!   zero-phase `filtfilt`. `filtfilt` needs the whole burst as f32; a causal
//!   filter costs a constant group delay, and per-pulse resync measures only
//!   gaps *between* pulses, so a constant delay cancels.
//! * The burst's 99th-percentile amplitude and the tone search use a 256-bin
//!   histogram and a Goertzel sweep instead of an exact percentile and an FFT.
//!
//! # Scope
//!
//! This crate covers the DSP front-end, burst capture, demodulation, HDLC
//! deframing, and dual-CRC acceptance. Caller-side responsibilities are
//! deliberately left out:
//!
//! * **Within-burst dedup.** [`NemoDecoder::feed`] returns every frame the
//!   offset/polarity sweep accepts, so the same telegram can appear more than
//!   once (e.g. a valid frame under X.25 and a shifted alias under CCITT). The
//!   firmware's `decNemoFrameSink` dedups by byte content per feed call; a
//!   caller wanting the same behavior should dedup the returned `Vec`.
//! * **Frame labelling / enqueue / upload.** The firmware builds a
//!   `g2.<byte3> <len>B <how>` label and enqueues; that is left to the caller.

mod demod;
mod dsp;
mod hdlc;

pub mod crc;
pub use crc::CrcConvention;

use dsp::Frontend;

// --- constants ---

pub(crate) const NEMO_FS: f32 = 48000.0; // input rate
pub(crate) const NEMO_FS2: f32 = 24000.0; // after decimation — the rate everything below runs at
pub(crate) const NEMO_DC_LEN: usize = 200; // moving-average DC block @48k (~4.2 ms; reference: 4 ms)
pub(crate) const NEMO_LP_CORNER: f32 = 3400.0; // upperCornerFrequencyRxData

/// Plausible baud range. Real captures sit near 4427 Bd (a ~2213 Hz tone); the
/// nominal rate is 4800. Anything outside this is a mis-estimate, not a burst.
pub(crate) const NEMO_BAUD_MIN: f32 = 3600.0;
pub(crate) const NEMO_BAUD_MAX: f32 = 5400.0;

pub(crate) const NEMO_SIG_MAX: usize = 9000; // 375 ms of 24 kHz bipolar signal (int16) = 18 kB
pub(crate) const NEMO_PRE: usize = 256; // ~10.7 ms preroll kept before gate-open
pub(crate) const NEMO_MAX_BITS: usize = 2048;

/// Shortest burst worth decoding: ~10 ms @ 24 kHz.
///
/// The old front-end demanded 1920 (80 ms) "like the reference pipeline" — but a
/// whole telegram is only ~12-35 ms on air, so that threshold threw away every
/// real single-telegram burst before decoding it. That single constant is why
/// this decoder reported bursts and never a frame.
pub(crate) const NEMO_MIN_BURST: usize = 240;
pub(crate) const NEMO_HOLD: i32 = 480; // 20 ms sustained-below before the gate closes
pub(crate) const NEMO_FRAME_MAX: usize = 32;
pub(crate) const NEMO_MIN_SEG: usize = 16; // bits; decode_air extract_frames min_seg

/// One CRC-valid telegram extracted from a burst. Carries what the C sink
/// carried: the decoded bytes (including the 2-byte trailer) and the CRC
/// convention that closed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NemoFrame {
    /// Decoded frame bytes, trailer included, in the convention's byte order
    /// (LSB-first for X.25, MSB-first for CCITT).
    pub bytes: Vec<u8>,
    /// Which CRC convention closed.
    pub crc: CrcConvention,
}

/// Streaming NEMO decoder for a fixed 48 kHz i16 input.
///
/// Feed raw audio frames with [`feed`](NemoDecoder::feed); each call returns the
/// frames finalized during that call. The ~18 kB envelope buffer and small bit
/// scratch buffers are heap-allocated inside the decoder, so the struct itself
/// is cheap to move.
pub struct NemoDecoder {
    front: Frontend,
    bits: Box<[u8]>,
    scratch: Box<[u8]>,
    pulses: Vec<demod::Pulse>,
    bursts: u64,
}

impl NemoDecoder {
    /// Create a decoder for 48 kHz input (the rate the C is fixed to).
    pub fn new() -> Self {
        NemoDecoder {
            front: Frontend::new(),
            bits: vec![0u8; NEMO_MAX_BITS].into_boxed_slice(),
            scratch: vec![0u8; NEMO_MAX_BITS].into_boxed_slice(),
            pulses: Vec::with_capacity(512),
            bursts: 0,
        }
    }

    /// Lifetime count of bursts the energy gate finalized, whether or not any
    /// CRC-valid frame came out (the C `stBursts` semantics — a climbing count
    /// with zero frames is how an operator sees the demodulator is alive on a
    /// noisy or mis-tuned channel). Callers wanting per-`feed` counts take the
    /// delta across the call.
    pub fn bursts(&self) -> u64 {
        self.bursts
    }

    /// Feed a block of raw 48 kHz i16 samples. Returns every CRC-valid frame
    /// finalized during this call (may be empty; may contain duplicates across
    /// the offset/polarity sweep — see the crate-level scope note).
    ///
    /// The firmware feeds 720-sample frames, but any block length works; state
    /// carries across calls.
    pub fn feed(&mut self, samples: &[i16]) -> Vec<NemoFrame> {
        let mut out = Vec::new();
        for &x in samples {
            if let Some(len) = self.front.push(x) {
                self.bursts += 1;
                self.process_burst(len, &mut out);
            }
        }
        out
    }

    fn process_burst(&mut self, len: usize, out: &mut Vec<NemoFrame>) {
        let sig = &self.front.sig[..len];
        // Schmitt comparator -> pulses; tone -> baud; per-pulse resync -> bits.
        demod::detect_pulses(sig, 0.4, &mut self.pulses);
        if self.pulses.len() < 4 {
            return;
        }
        let baud = demod::estimate_baud(sig);
        let nbits = demod::bits_from_pulses(&self.pulses, baud, &mut self.bits);
        if nbits < NEMO_MIN_SEG {
            return;
        }
        hdlc::extract_frames(&self.bits[..nbits], &mut self.scratch, out);
    }
}

impl Default for NemoDecoder {
    fn default() -> Self {
        Self::new()
    }
}
