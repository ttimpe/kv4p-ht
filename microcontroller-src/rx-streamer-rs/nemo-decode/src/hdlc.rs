//! HDLC deframing: split the recovered bit stream on runs of >= 6 ones
//! (EOF/preamble tone), destuff each segment, sweep the eight byte offsets under
//! both bit polarities, and accept segments that close either CRC convention.
//!
//! Faithful port of `decoder_nemo.h` `nemoFramesForPolarity` / `nemoTrySegment`,
//! which fuse `bast_chain.py frames()` (X.25, LSB-first) and
//! `decode_air.py extract_frames()` (CCITT, MSB-first).

use crate::crc::{ccitt_ok, frame_nontrivial, x25_ok, CrcConvention};
use crate::{NemoFrame, NEMO_FRAME_MAX, NEMO_MIN_SEG};

/// One destuffed segment: sweep byte offsets 0..7 under both conventions.
fn try_segment(ds: &[u8], out: &mut Vec<NemoFrame>) {
    let ds_len = ds.len();
    for off in 0..8 {
        if ds_len < off {
            continue;
        }
        let nby = (ds_len - off) / 8;
        if nby < 4 || nby > NEMO_FRAME_MAX {
            continue;
        }
        let mut lsb = [0u8; NEMO_FRAME_MAX];
        let mut msb = [0u8; NEMO_FRAME_MAX];
        for i in 0..nby {
            let mut vl = 0u8;
            let mut vm = 0u8;
            for j in 0..8 {
                let b = ds[off + i * 8 + j] & 1;
                vl |= b << j;
                vm |= b << (7 - j);
            }
            lsb[i] = vl;
            msb[i] = vm;
        }
        if frame_nontrivial(&lsb[..nby]) && x25_ok(&lsb[..nby]) {
            out.push(NemoFrame { bytes: lsb[..nby].to_vec(), crc: CrcConvention::X25 });
        }
        if frame_nontrivial(&msb[..nby]) && ccitt_ok(&msb[..nby]) {
            out.push(NemoFrame { bytes: msb[..nby].to_vec(), crc: CrcConvention::Ccitt });
        }
    }
}

/// Deframe one polarity: split on runs of >= 6 ones (EOF/preamble tone),
/// destuff each segment, try the CRC conventions. `pol` is 0 (true) or 1
/// (inverted); the stream is XORed with `pol` before framing.
fn frames_for_polarity(bits: &[u8], pol: u8, scratch: &mut [u8], out: &mut Vec<NemoFrame>) {
    let nbits = bits.len();
    let mut i = 0usize;
    while i < nbits {
        let seg_start = i;
        let mut seg_end = nbits;
        let mut ones = 0;
        let mut j = i;
        while j < nbits {
            if (bits[j] ^ pol) & 1 != 0 {
                ones += 1;
                if ones == 6 {
                    seg_end = j - 5;
                    break;
                }
            } else {
                ones = 0;
            }
            j += 1;
        }
        if seg_end - seg_start >= NEMO_MIN_SEG {
            // destuff on the polarity-adjusted view
            let mut m = 0usize;
            let mut ones2 = 0;
            for k in seg_start..seg_end {
                let ch = (bits[k] ^ pol) & 1;
                if ones2 == 5 {
                    ones2 = 0;
                    if ch == 0 {
                        continue;
                    }
                }
                scratch[m] = ch;
                m += 1;
                ones2 = if ch != 0 { ones2 + 1 } else { 0 };
            }
            try_segment(&scratch[..m], out);
        }
        if j >= nbits {
            break;
        }
        j += 1;
        while j < nbits && (bits[j] ^ pol) & 1 != 0 {
            j += 1;
        }
        i = j;
    }
}

/// Deframe both polarities of a recovered burst. `pol=1` (inverted) is the
/// validated convention and is tried first, then true polarity, matching the C.
pub(crate) fn extract_frames(bits: &[u8], scratch: &mut [u8], out: &mut Vec<NemoFrame>) {
    frames_for_polarity(bits, 1, scratch, out);
    frames_for_polarity(bits, 0, scratch, out);
}
