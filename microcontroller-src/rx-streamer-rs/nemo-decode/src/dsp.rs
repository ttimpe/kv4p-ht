//! Streaming DSP front-end and energy-gated burst capture (48 kHz i16 in).
//!
//! Physical layer (patent EP 0566773): the FM baseband carries AMI-coded cos^2
//! half-wave pulses of a ~2400 Hz tone at 4800 Bd (mark = pulse, space = gap).
//! Per-sample chain, faithful to `decoder_nemo.h` `nemoFeed`, which ports
//! `bast_chain.py demod()`:
//!
//!   x - moving_avg(x, 200) -> |x| -> freq-xlate @2400 Hz + 43-tap FIR
//!   -> decimate by 2 -> |z| -> moving_avg(5)  => envelope @ 24 kHz
//!
//! Bursts are gated by an adaptive energy detector (same shape as the FFSK
//! squelch) into a heap-allocated envelope buffer, with preroll / hold / min-burst
//! exactly as the C.

use crate::{
    NEMO_DC_LEN, NEMO_ENV_MAX, NEMO_FS, NEMO_HOLD_ENV, NEMO_MIN_BURST, NEMO_NCO_LEN, NEMO_PRE_ENV,
    NEMO_XL_CUTOFF, NEMO_XL_TAPS,
};

pub(crate) struct Frontend {
    // DC removal (sliding sum) on the raw 48 kHz samples.
    dc_sum: i32,
    dc_hist: [i16; NEMO_DC_LEN],
    dc_pos: usize,
    // NCO mix + FIR history (complex), consumed at the decimated rate.
    nco_cos: [f32; NEMO_NCO_LEN],
    nco_sin: [f32; NEMO_NCO_LEN],
    nco_phase: usize,
    taps: [f32; NEMO_XL_TAPS],
    xr: [f32; NEMO_XL_TAPS],
    xi: [f32; NEMO_XL_TAPS],
    x_pos: usize,
    decim_phase: i32,
    // moving_avg(5) on the magnitude.
    ma5: [f32; 5],
    ma5_sum: f32,
    ma5_pos: usize,
    // burst gate (normalized envelope, adaptive floor like the FFSK squelch).
    act_env: f32,
    act_alpha: f32,
    act_floor: f32,
    floor_rise: f32,
    floor_fall: f32,
    bursting: bool,
    below_count: i32,
    // envelope capture.
    pre_ring: [i16; NEMO_PRE_ENV],
    pre_pos: usize,
    pre_fill: usize,
    /// ~18 kB envelope buffer, heap-allocated (mirrors the C's lazy alloc).
    pub(crate) env: Box<[i16]>,
    env_len: usize,
}

impl Frontend {
    pub(crate) fn new() -> Self {
        let mut nco_cos = [0.0f32; NEMO_NCO_LEN];
        let mut nco_sin = [0.0f32; NEMO_NCO_LEN];
        for k in 0..NEMO_NCO_LEN {
            let a = -2.0 * std::f32::consts::PI * k as f32 / NEMO_NCO_LEN as f32;
            nco_cos[k] = a.cos();
            nco_sin[k] = a.sin();
        }
        // windowed-sinc (Hamming) low-pass like scipy.signal.firwin, unity DC gain.
        let fc = NEMO_XL_CUTOFF / NEMO_FS;
        let mut taps = [0.0f32; NEMO_XL_TAPS];
        let mut sum = 0.0f32;
        for k in 0..NEMO_XL_TAPS {
            let m = k as f32 - (NEMO_XL_TAPS as f32 - 1.0) / 2.0;
            let sinc = if m == 0.0 {
                2.0 * fc
            } else {
                (2.0 * std::f32::consts::PI * fc * m).sin() / (std::f32::consts::PI * m)
            };
            let w = 0.54 - 0.46 * (2.0 * std::f32::consts::PI * k as f32 / (NEMO_XL_TAPS as f32 - 1.0)).cos();
            taps[k] = sinc * w;
            sum += taps[k];
        }
        for k in 0..NEMO_XL_TAPS {
            taps[k] /= sum;
        }
        let fs2 = NEMO_FS / 2.0; // 24 kHz envelope rate
        Frontend {
            dc_sum: 0,
            dc_hist: [0; NEMO_DC_LEN],
            dc_pos: 0,
            nco_cos,
            nco_sin,
            nco_phase: 0,
            taps,
            xr: [0.0; NEMO_XL_TAPS],
            xi: [0.0; NEMO_XL_TAPS],
            x_pos: 0,
            decim_phase: 0,
            ma5: [0.0; 5],
            ma5_sum: 0.0,
            ma5_pos: 0,
            act_env: 0.0,
            act_alpha: 1.0 - (-1.0 / (fs2 * 0.005)).exp(),
            act_floor: 0.05,
            floor_rise: 1.0 - (-1.0 / (fs2 * 5.0)).exp(),
            floor_fall: 1.0 - (-1.0 / (fs2 * 0.05)).exp(),
            bursting: false,
            below_count: 0,
            pre_ring: [0; NEMO_PRE_ENV],
            pre_pos: 0,
            pre_fill: 0,
            env: vec![0i16; NEMO_ENV_MAX].into_boxed_slice(),
            env_len: 0,
        }
    }

    /// Push one raw 48 kHz sample. Returns `Some(len)` when a burst has just
    /// finalized and is ready for decoding in `self.env[..len]`.
    pub(crate) fn push(&mut self, x: i16) -> Option<usize> {
        self.dc_sum += x as i32 - self.dc_hist[self.dc_pos] as i32;
        self.dc_hist[self.dc_pos] = x;
        self.dc_pos = (self.dc_pos + 1) % NEMO_DC_LEN;
        let dc = x as f32 - self.dc_sum as f32 / NEMO_DC_LEN as f32;
        let y = dc.abs();

        let c = self.nco_cos[self.nco_phase];
        let s = self.nco_sin[self.nco_phase];
        self.nco_phase = (self.nco_phase + 1) % NEMO_NCO_LEN;
        self.xr[self.x_pos] = y * c;
        self.xi[self.x_pos] = y * s;
        self.x_pos = (self.x_pos + 1) % NEMO_XL_TAPS;

        self.decim_phase += 1;
        if self.decim_phase < 2 {
            return None; // 48 kHz -> 24 kHz
        }
        self.decim_phase = 0;

        let mut zr = 0.0f32;
        let mut zi = 0.0f32;
        let base = self.x_pos + NEMO_XL_TAPS - 1;
        for k in 0..NEMO_XL_TAPS {
            let mut idx = base - k;
            if idx >= NEMO_XL_TAPS {
                idx -= NEMO_XL_TAPS;
            }
            zr += self.taps[k] * self.xr[idx];
            zi += self.taps[k] * self.xi[idx];
        }
        let mag = (zr * zr + zi * zi).sqrt();

        self.ma5_sum += mag - self.ma5[self.ma5_pos];
        self.ma5[self.ma5_pos] = mag;
        self.ma5_pos = (self.ma5_pos + 1) % 5;
        let env = self.ma5_sum / 5.0;
        let env_q: i16 = if env > 32767.0 { 32767 } else { env as i16 };

        // burst gate on the normalized envelope
        let norm = env / 32768.0;
        self.act_env += (norm - self.act_env) * self.act_alpha;
        let alpha = if self.act_env < self.act_floor {
            self.floor_fall
        } else {
            self.floor_rise
        };
        self.act_floor += (self.act_env - self.act_floor) * alpha;
        if self.act_floor < 1e-5 {
            self.act_floor = 1e-5;
        }

        if !self.bursting {
            self.pre_ring[self.pre_pos] = env_q;
            self.pre_pos = (self.pre_pos + 1) % NEMO_PRE_ENV;
            if self.pre_fill < NEMO_PRE_ENV {
                self.pre_fill += 1;
            }
            if self.act_env > self.act_floor * 4.0 {
                self.bursting = true;
                self.below_count = 0;
                self.env_len = 0;
                let mut k = self.pre_fill as isize;
                while k > 0 {
                    let mut idx = self.pre_pos as isize - k;
                    if idx < 0 {
                        idx += NEMO_PRE_ENV as isize;
                    }
                    self.env[self.env_len] = self.pre_ring[idx as usize];
                    self.env_len += 1;
                    k -= 1;
                }
            }
            return None;
        }

        self.env[self.env_len] = env_q;
        self.env_len += 1;
        let mut finalize = false;
        if self.act_env < self.act_floor * 2.0 {
            self.below_count += 1;
            if self.below_count >= NEMO_HOLD_ENV {
                finalize = true;
            }
        } else {
            self.below_count = 0;
        }
        if self.env_len >= NEMO_ENV_MAX {
            finalize = true; // buffer full: decode what we have
        }

        if finalize {
            let keep_bursting = self.env_len >= NEMO_ENV_MAX && self.below_count < NEMO_HOLD_ENV;
            let ready = if self.env_len >= NEMO_MIN_BURST {
                Some(self.env_len)
            } else {
                None
            };
            self.env_len = 0;
            self.bursting = keep_bursting;
            self.below_count = 0;
            self.pre_fill = 0;
            self.pre_pos = 0;
            return ready;
        }
        None
    }
}
