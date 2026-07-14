//! Streaming front-end and energy-gated burst capture (48 kHz i16 in).
//!
//! Port of the receive chain in `lio-decoder/src/nemo_hysteresis_demod.py`
//! (`detect_pulses`'s preamble): DC/baseline block, then a low-pass to the data
//! corner — and **no rectification**. The signal that reaches the burst buffer is
//! still bipolar, because polarity is the line code (AMI): the previous
//! front-end's `abs()` threw away exactly the information the demodulator needs.
//!
//! Chain, per 48 kHz sample:
//!
//! ```text
//!   x - moving_avg(x, 4 ms)      DC / baseline block
//!   -> Butterworth LP, 3400 Hz   receiveDigitalFilter / upperCornerFrequencyRxData
//!   -> decimate by 2             => bipolar signal @ 24 kHz, stored as i16
//! ```
//!
//! The low-pass is **causal** (a biquad cascade), where the reference uses a
//! zero-phase `filtfilt`. That is deliberate: `filtfilt` needs the whole burst
//! as f32 (~36 kB for a 375 ms burst), and the node has ~25-47 kB of heap free
//! with the decoder running. A causal filter costs a constant group delay, and
//! the demodulator measures only *gaps between* pulses (per-pulse resync), so a
//! constant delay cancels exactly.
//!
//! It also serves as the anti-alias filter for the 48->24 kHz decimation: at
//! 3.4 kHz corner nothing survives near the 12 kHz Nyquist.

use crate::{NEMO_DC_LEN, NEMO_FS, NEMO_HOLD, NEMO_LP_CORNER, NEMO_MIN_BURST, NEMO_PRE, NEMO_SIG_MAX};

/// Direct-form-I biquad.
#[derive(Default, Clone, Copy)]
struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Biquad {
    /// RBJ low-pass section at `f0` with quality `q`.
    fn lowpass(fs: f32, f0: f32, q: f32) -> Biquad {
        let w0 = 2.0 * core::f32::consts::PI * f0 / fs;
        let (sin_w0, cos_w0) = (w0.sin(), w0.cos());
        let alpha = sin_w0 / (2.0 * q);
        let a0 = 1.0 + alpha;
        Biquad {
            b0: ((1.0 - cos_w0) / 2.0) / a0,
            b1: (1.0 - cos_w0) / a0,
            b2: ((1.0 - cos_w0) / 2.0) / a0,
            a1: (-2.0 * cos_w0) / a0,
            a2: (1.0 - alpha) / a0,
            ..Default::default()
        }
    }

    fn run(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

pub(crate) struct Frontend {
    // DC / baseline removal (sliding sum) on the raw 48 kHz samples.
    dc_sum: i32,
    dc_hist: [i16; NEMO_DC_LEN],
    dc_pos: usize,
    // 4th-order Butterworth low-pass = two biquads (Q = 0.5412, 1.3066).
    lp1: Biquad,
    lp2: Biquad,
    decim_phase: u8,

    // Burst gate on |y|, with an adaptive noise floor (same shape as the FFSK
    // squelch): a burst opens at 4x the floor and closes after HOLD below it.
    act: f32,
    act_alpha: f32,
    floor: f32,
    floor_rise: f32,
    floor_fall: f32,
    bursting: bool,
    below: i32,

    // Preroll, so the gate's own attack does not clip the first pulses.
    pre_ring: [i16; NEMO_PRE],
    pre_pos: usize,
    pre_fill: usize,

    /// Bipolar burst signal @ 24 kHz. Public to the crate: the demodulator reads
    /// it in place, so a burst is never copied.
    pub(crate) sig: Box<[i16]>,
    pub(crate) sig_len: usize,
}

impl Frontend {
    pub(crate) fn new() -> Self {
        let fs2 = NEMO_FS / 2.0; // 24 kHz, the rate the gate runs at
        Frontend {
            dc_sum: 0,
            dc_hist: [0; NEMO_DC_LEN],
            dc_pos: 0,
            // Butterworth order 4 = cascade of two sections at these Qs.
            lp1: Biquad::lowpass(NEMO_FS, NEMO_LP_CORNER, 0.541_196),
            lp2: Biquad::lowpass(NEMO_FS, NEMO_LP_CORNER, 1.306_563),
            decim_phase: 0,
            act: 0.0,
            act_alpha: 1.0 - (-1.0f32 / (fs2 * 0.002)).exp(),
            floor: 0.05,
            floor_rise: 1.0 - (-1.0f32 / (fs2 * 5.0)).exp(),
            floor_fall: 1.0 - (-1.0f32 / (fs2 * 0.05)).exp(),
            bursting: false,
            below: 0,
            pre_ring: [0; NEMO_PRE],
            pre_pos: 0,
            pre_fill: 0,
            sig: vec![0i16; NEMO_SIG_MAX].into_boxed_slice(),
            sig_len: 0,
        }
    }

    /// Push one raw 48 kHz sample. Returns `Some(len)` when a burst has just
    /// finalized and is ready to demodulate in `self.sig[..len]`.
    pub(crate) fn push(&mut self, x: i16) -> Option<usize> {
        // DC / baseline block (reference: x - moving_avg(x, 4 ms)).
        self.dc_sum += x as i32 - self.dc_hist[self.dc_pos] as i32;
        self.dc_hist[self.dc_pos] = x;
        self.dc_pos = (self.dc_pos + 1) % NEMO_DC_LEN;
        let dc = x as f32 - self.dc_sum as f32 / NEMO_DC_LEN as f32;

        // Low-pass to the data corner. NOT rectified: AMI polarity is the code.
        let y = self.lp2.run(self.lp1.run(dc));

        self.decim_phase ^= 1;
        if self.decim_phase == 1 {
            return None; // 48 kHz -> 24 kHz
        }

        let q: i16 = y.clamp(-32768.0, 32767.0) as i16;

        // Gate on the rectified *envelope* — rectifying is fine here, because
        // this only decides where a burst starts and ends; the stored signal
        // stays bipolar.
        let norm = y.abs() / 32768.0;
        self.act += (norm - self.act) * self.act_alpha;
        let alpha = if self.act < self.floor {
            self.floor_fall
        } else {
            self.floor_rise
        };
        self.floor += (self.act - self.floor) * alpha;
        if self.floor < 1e-5 {
            self.floor = 1e-5;
        }

        if !self.bursting {
            self.pre_ring[self.pre_pos] = q;
            self.pre_pos = (self.pre_pos + 1) % NEMO_PRE;
            if self.pre_fill < NEMO_PRE {
                self.pre_fill += 1;
            }
            if self.act > self.floor * 4.0 {
                // Open: replay the preroll so the burst's first pulses survive.
                self.bursting = true;
                self.below = 0;
                self.sig_len = 0;
                let start = (self.pre_pos + NEMO_PRE - self.pre_fill) % NEMO_PRE;
                for k in 0..self.pre_fill {
                    self.sig[self.sig_len] = self.pre_ring[(start + k) % NEMO_PRE];
                    self.sig_len += 1;
                }
            }
            return None;
        }

        if self.sig_len < NEMO_SIG_MAX {
            self.sig[self.sig_len] = q;
            self.sig_len += 1;
        }

        if self.act < self.floor * 2.0 {
            self.below += 1;
        } else {
            self.below = 0;
        }

        let full = self.sig_len >= NEMO_SIG_MAX;
        if self.below >= NEMO_HOLD || full {
            self.bursting = false;
            self.pre_fill = 0;
            let len = self.sig_len;
            self.sig_len = 0;
            // A real telegram is >= ~12 ms on air. The old front-end demanded
            // 80 ms, which discarded every single-telegram burst before it was
            // ever decoded — the reason this decoder never saw a real frame.
            if len >= NEMO_MIN_BURST {
                return Some(len);
            }
        }
        None
    }
}
