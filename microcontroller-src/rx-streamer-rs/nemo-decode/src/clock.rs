//! Burst back-end clock recovery: approximate-median bias removal followed by a
//! Mueller & Müller clock-recovery loop with Catmull-Rom cubic interpolation.
//!
//! Faithful port of `gr::digital::clock_recovery_mm_ff` as used by
//! `lio-decoder/src/bast_chain.py` (`clock_recovery_mm`). The gains and omega
//! constants are copied exactly; values are normalized to the int16 scale so
//! the +/-1 error clamp behaves like the float reference.

use crate::NEMO_SPS;

/// Approximate median of the burst envelope via a 256-bin histogram
/// (documented deviation from the reference's exact median).
pub(crate) fn median(env: &[i16]) -> f32 {
    let n = env.len();
    let mut hist = [0u16; 256];
    for &e in env {
        let v = if e < 0 { 0 } else { e as i32 };
        hist[(v >> 7) as usize] += 1;
    }
    let half = n / 2;
    let mut acc = 0usize;
    for b in 0..256 {
        acc += hist[b] as usize;
        if acc >= half {
            return ((b << 7) + 64) as f32;
        }
    }
    0.0
}

/// Mueller & Müller clock recovery on the median-centred envelope. Writes hard
/// bits into `bits` (capacity is its length) and returns the count.
///
/// omega = sps = 5, gain_omega = 7.65625e-3, mu = 0.5, gain_mu = 0.175,
/// omega_rel = 5e-3.
pub(crate) fn clock_recover(env: &[i16], median: f32, bits: &mut [u8]) -> usize {
    let n = env.len() as i64;
    let max_bits = bits.len();
    let scale = 1.0f32 / 32768.0;
    let omega_mid = NEMO_SPS;
    let mut omega = NEMO_SPS;
    let omega_lim = 5e-3f32 * omega_mid;
    let mut ii: i64 = 2;
    let mut mu = 0.5f32;
    let mut last = 0.0f32;
    let mut nbits = 0usize;
    while ii < n - 2 && nbits < max_bits {
        if ii < 1 {
            break;
        }
        let i = ii as usize;
        let t = mu;
        let p0 = (env[i - 1] as f32 - median) * scale;
        let p1 = (env[i] as f32 - median) * scale;
        let p2 = (env[i + 1] as f32 - median) * scale;
        let p3 = (env[i + 2] as f32 - median) * scale;
        let y = p1
            + 0.5 * t
                * (p2 - p0
                    + t * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3
                        + t * (3.0 * (p1 - p2) + p3 - p0)));
        let d_y = if y > 0.0 { 1.0 } else { -1.0 };
        let d_lo = if last > 0.0 { 1.0 } else { -1.0 };
        let mut err = d_lo * y - d_y * last;
        if err > 1.0 {
            err = 1.0;
        }
        if err < -1.0 {
            err = -1.0;
        }
        bits[nbits] = if y > 0.0 { 1 } else { 0 };
        nbits += 1;
        last = y;
        omega += 7.65625e-3 * err;
        let mut dev = omega - omega_mid;
        if dev > omega_lim {
            dev = omega_lim;
        }
        if dev < -omega_lim {
            dev = -omega_lim;
        }
        omega = omega_mid + dev;
        mu += omega + 0.175 * err;
        ii += mu as i64; // truncation toward zero, matching C's (int)mu
        mu -= (mu as i64) as f32;
    }
    nbits
}
