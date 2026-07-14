//! NEMO / VicosLio ("G2") telegram decoder — a host-portable, dependency-free
//! Rust port of the on-device C decoder (`kv4p_rx_streamer/decoder_nemo.h`),
//! itself a best-effort port of the validated on-air chain in `lio-decoder`
//! (`src/bast_chain.py` `demod()` front-end, `clock_recovery_mm`, and the HDLC
//! framing + CRC acceptance from `frames()` / `decode_air.py extract_frames()`).
//!
//! Physical layer (patent EP 0566773): the FM baseband carries AMI-coded cos^2
//! half-wave pulses of a ~2400 Hz tone at 4800 Bd (mark = pulse, space = gap).
//! Front-end (streaming, 48 kHz in):
//!
//! ```text
//!   x - moving_avg(x,200) -> |x| -> freq-xlate @2400 Hz + 43-tap FIR,
//!   decimate by 2 -> |z| -> moving_avg(5)  => envelope @ 24 kHz
//! ```
//!
//! Bursts are gated by an adaptive energy detector into a buffer; on burst end:
//! approximate-median bias removal -> Mueller & Müller clock recovery (sps=5) ->
//! HDLC deframe (EOF = run of six 1s, destuff, byte-offset sweep) -> CRC.
//!
//! Two CRC conventions close on real captures (verified against the Python
//! references): LSB-first bytes + X.25 FCS (LE trailer) — what the backend
//! re-verifies — and MSB-first bytes + CRC-16/CCITT 0x1021 with the
//! `decode_air.py` `(init, xorout)` variants. Both are accepted; a caller (or
//! the server's `crc_ok`) decides what drives the map.
//!
//! # Fidelity
//!
//! This is a 1:1 numeric port of the C: identical integer/f32 types, constants,
//! thresholds, and processing order, so behavior is bit-comparable with the
//! firmware. The one intentional deviation (shared with the C) is the burst
//! median, computed via a 256-bin histogram rather than an exact sort.
//!
//! # Scope
//!
//! This crate covers exactly what `decoder_nemo.h` covers: the DSP front-end,
//! burst capture, clock recovery, HDLC deframing, and dual-CRC acceptance.
//! Caller-side responsibilities are deliberately left out:
//!
//! * **Within-burst dedup.** [`NemoDecoder::feed`] returns every frame the
//!   offset/polarity sweep accepts, so the same telegram can appear more than
//!   once (e.g. a valid frame under X.25 and a shifted alias under CCITT). The
//!   firmware's `decNemoFrameSink` dedups by byte content per feed call; a
//!   caller wanting the same behavior should dedup the returned `Vec`.
//! * **Frame labelling / enqueue / upload.** The firmware builds a
//!   `g2.<byte3> <len>B <how>` label and enqueues; that is left to the caller.

mod clock;
mod dsp;
mod hdlc;

pub mod crc;
pub use crc::CrcConvention;

use dsp::Frontend;

// --- constants (mirrors the NEMO_* macros in decoder_nemo.h) ---

pub(crate) const NEMO_FS: f32 = 48000.0;
pub(crate) const NEMO_DC_LEN: usize = 200; // moving-average DC removal window @48k
pub(crate) const NEMO_XL_TAPS: usize = 43; // firwin(43, 4000/(fs/2)) — cutoff is load-bearing
pub(crate) const NEMO_XL_CUTOFF: f32 = 4000.0;
pub(crate) const NEMO_NCO_LEN: usize = 20; // 2400/48000 = 1/20: the NCO cycles in 20 steps
pub(crate) const NEMO_SPS: f32 = 5.0; // 24000 / 4800

pub(crate) const NEMO_ENV_MAX: usize = 9000; // 375 ms of 24 kHz envelope (int16) = 18 kB
pub(crate) const NEMO_PRE_ENV: usize = 256; // ~10.7 ms preroll kept before gate-open
pub(crate) const NEMO_MAX_BITS: usize = 2048;
pub(crate) const NEMO_MIN_BURST: usize = 1920; // 80 ms @ 24 kHz, like the reference pipeline
pub(crate) const NEMO_HOLD_ENV: i32 = 480; // 20 ms sustained-below before the gate closes
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
    bursts: u64,
}

impl NemoDecoder {
    /// Create a decoder for 48 kHz input (the rate the C is fixed to).
    pub fn new() -> Self {
        NemoDecoder {
            front: Frontend::new(),
            bits: vec![0u8; NEMO_MAX_BITS].into_boxed_slice(),
            scratch: vec![0u8; NEMO_MAX_BITS].into_boxed_slice(),
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
        let env = &self.front.env[..len];
        let median = clock::median(env);
        let nbits = clock::clock_recover(env, median, &mut self.bits);
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
