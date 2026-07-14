//! Burst demodulation: the manufacturer's own receive algorithm.
//!
//! Port of `lio-decoder/src/nemo_hysteresis_demod.py`, the pipeline that was
//! reverse-engineered from the vehicle DSP firmware (TMS320C54x, module
//! `rpsL2aL1.c`) and is the first one in that project to close CRCs on real
//! capture data. It replaces the earlier `bast_chain` front-end, which this
//! crate previously ported and which — as its own fixture test admitted — never
//! decoded a real capture, only a synthesized burst.
//!
//! Three things the old front-end got wrong, all fixed here:
//!
//! 1. **It rectified the signal.** `abs()` destroys the pulse polarity, and
//!    polarity *is* the line code: NEMO is AMI, so consecutive marks alternate
//!    sign. The DSP firmware low-passes and keeps the signal **bipolar**; the
//!    two polarities are its `RxEvent1`/`RxEvent2`.
//! 2. **It recovered the clock with Mueller & Müller.** The patent
//!    (EP 0566773 B1, claim 2) says the symbol clock is **reset on every pulse**.
//!    A continuous M&M loop accumulates timing error across the long runs of
//!    zeros a telegram is full of; per-pulse resync cannot.
//! 3. **It assumed 4800 Bd exactly.** Real captures run a ~2213 Hz tone, i.e.
//!    ~4427 Bd. The baud is therefore estimated per burst from the dominant
//!    line-code tone (`baud = 2 * f_tone`), as the reference does.
//!
//! Pulse detection is a signed Schmitt trigger — the firmware's
//! `receiveTriggerLevel` +/- `receiveHysteresis` comparator. The idle band
//! between +hi and -hi, plus the zero crossing between two adjacent
//! opposite-polarity half-waves, is what keeps consecutive AMI marks apart.

use crate::{NEMO_BAUD_MAX, NEMO_BAUD_MIN, NEMO_FS2, NEMO_MAX_BITS};

/// One detected half-wave pulse: sample index of its peak, and its polarity.
pub(crate) struct Pulse {
    pub at: usize,
    /// AMI polarity (+1/-1). Not consumed yet — a future check can flag two
    /// consecutive same-polarity marks as a bipolar violation (patent claim 2),
    /// but the CRC already rejects the frames those errors produce.
    #[allow(dead_code)]
    pub pol: i8,
}

/// 99th percentile of |x|, via a 256-bin histogram.
///
/// The reference takes an exact `np.percentile(|x|, 99)`; a histogram is the
/// same answer to within a bin and costs one pass and 512 bytes instead of a
/// sort of the whole burst.
fn p99_abs(x: &[i16]) -> f32 {
    if x.is_empty() {
        return 0.0;
    }
    let max = x.iter().map(|&v| (v as i32).unsigned_abs()).max().unwrap_or(0);
    if max == 0 {
        return 0.0;
    }
    let mut hist = [0u32; 256];
    let scale = 255.0 / max as f32;
    for &v in x {
        let a = (v as i32).unsigned_abs() as f32;
        hist[(a * scale) as usize & 0xFF] += 1;
    }
    let target = (x.len() as f32 * 0.99) as u32;
    let mut acc = 0u32;
    for (bin, &c) in hist.iter().enumerate() {
        acc += c;
        if acc >= target {
            return bin as f32 / scale;
        }
    }
    max as f32
}

/// Signed dual-threshold Schmitt comparator — the firmware's receive front-end.
///
/// Fires on `+hi` (positive half-wave) and `-hi` (negative), releasing at
/// `+/-lo`; the peak sample inside each excursion is the pulse's timestamp.
/// `trigger` defaults to 40% of the burst's 99th-percentile amplitude, the
/// reference's stand-in for the firmware's unextracted `receiveTriggerLevel`.
pub(crate) fn detect_pulses(x: &[i16], hyst_frac: f32, out: &mut Vec<Pulse>) {
    out.clear();
    let trigger = 0.4 * p99_abs(x);
    if trigger <= 0.0 {
        return;
    }
    let hi = trigger * (1.0 + hyst_frac);
    let lo = trigger * (1.0 - hyst_frac);

    let mut state: i8 = 0; // 0 idle, +1 inside a positive pulse, -1 negative
    let mut seg0 = 0usize;
    let mut best = 0i32;
    let mut best_at = 0usize;

    for (i, &xi) in x.iter().enumerate() {
        let v = xi as f32;
        match state {
            0 => {
                if v > hi {
                    state = 1;
                    seg0 = i;
                    best = xi as i32;
                    best_at = i;
                } else if v < -hi {
                    state = -1;
                    seg0 = i;
                    best = xi as i32;
                    best_at = i;
                }
            }
            1 => {
                if (xi as i32) > best {
                    best = xi as i32;
                    best_at = i;
                }
                if v < lo {
                    out.push(Pulse { at: best_at, pol: 1 });
                    state = 0;
                }
            }
            _ => {
                if (xi as i32) < best {
                    best = xi as i32;
                    best_at = i;
                }
                if v > -lo {
                    out.push(Pulse { at: best_at, pol: -1 });
                    state = 0;
                }
            }
        }
    }
    // A pulse still open at the buffer's end still happened.
    if state != 0 && seg0 < x.len() {
        out.push(Pulse { at: best_at, pol: state });
    }
}

/// Baud rate of this burst, from the dominant line-code tone.
///
/// `baud = 2 * f_tone` (each mark is one half-wave of the tone). The tone is
/// found by sweeping Goertzel bins across 1500..3200 Hz — the reference does the
/// same thing with an FFT peak, but a sweep of ~85 bins over one burst is far
/// cheaper than an FFT and needs no scratch buffer, which matters on the node.
///
/// Real captures sit near 2213 Hz (≈4427 Bd), not the nominal 2400/4800 — this
/// estimate is not a refinement, it is why real frames decode at all.
pub(crate) fn estimate_baud(x: &[i16]) -> f32 {
    const F_LO: f32 = 1500.0;
    const F_HI: f32 = 3200.0;
    const STEP: f32 = 20.0;

    if x.len() < 64 {
        return (NEMO_BAUD_MIN + NEMO_BAUD_MAX) * 0.5;
    }
    // Mean removal keeps a DC residue from dominating the low bins.
    let mean = x.iter().map(|&v| v as f32).sum::<f32>() / x.len() as f32;

    let mut best_f = 2400.0;
    let mut best_p = -1.0f32;
    let mut f = F_LO;
    while f <= F_HI {
        // Goertzel: power at frequency f without a full transform.
        let w = 2.0 * core::f32::consts::PI * f / NEMO_FS2;
        let coeff = 2.0 * w.cos();
        let (mut s1, mut s2) = (0.0f32, 0.0f32);
        for &v in x {
            let s0 = (v as f32 - mean) + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }
        let power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
        if power > best_p {
            best_p = power;
            best_f = f;
        }
        f += STEP;
    }
    (2.0 * best_f).clamp(NEMO_BAUD_MIN, NEMO_BAUD_MAX)
}

/// Per-pulse-resync symbol decode (patent claim 2).
///
/// Every pulse is a `1` **and re-anchors the symbol clock**; the gap back to the
/// previous pulse, rounded to whole symbol periods, supplies the intervening
/// `0`s. Timing error therefore never accumulates across a long zero-run, which
/// is precisely where the old Mueller & Müller loop lost the frame.
///
/// Returns the number of bits written into `bits` (as 0/1 bytes).
pub(crate) fn bits_from_pulses(pulses: &[Pulse], baud: f32, bits: &mut [u8]) -> usize {
    let t_sym = NEMO_FS2 / baud;
    if t_sym <= 0.0 {
        return 0;
    }
    let mut n = 0usize;
    let mut prev: Option<usize> = None;

    for p in pulses {
        match prev {
            None => {
                if n < bits.len() {
                    bits[n] = 1;
                    n += 1;
                }
            }
            Some(pt) => {
                let gap = ((p.at - pt) as f32 / t_sym).round() as i32;
                let zeros = (gap - 1).max(0) as usize;
                // A silence longer than the buffer is a dead channel, not a
                // telegram: stop rather than fill megabytes of zeros.
                if zeros > NEMO_MAX_BITS {
                    break;
                }
                for _ in 0..zeros {
                    if n >= bits.len() {
                        return n;
                    }
                    bits[n] = 0;
                    n += 1;
                }
                if n >= bits.len() {
                    return n;
                }
                bits[n] = 1;
                n += 1;
            }
        }
        prev = Some(p.at);
    }
    n
}
