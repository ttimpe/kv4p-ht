//! CRC conventions the G2 on-air link layer closes under.
//!
//! Two conventions close on real captures (verified against the lio-decoder
//! Python references, `src/bast_chain.py` and `src/decode_air.py`):
//!
//!   * **X.25 FCS** — reflected poly 0x8408, init 0xFFFF, xorout 0xFFFF, with a
//!     little-endian trailer over LSB-first bytes. This is what the backend
//!     re-verifies (see `bielefeld-live` `crates/g2/src/crc.rs`).
//!   * **CRC-16/CCITT** — poly 0x1021, MSB-first / non-reflected, big-endian
//!     trailer over MSB-first bytes, across the `(init, xorout)` variants that
//!     `decode_air.py` accepts.
//!
//! Both are accepted; a caller (or the backend) decides which drives the map.

/// Which CRC convention closed for a decoded frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CrcConvention {
    /// X.25 FCS (reflected 0x8408, LSB-first bytes, LE trailer).
    X25,
    /// CRC-16/CCITT (0x1021, MSB-first bytes, BE trailer).
    Ccitt,
}

impl CrcConvention {
    /// The short tag the C sink used (`"x25"` / `"ccitt"`).
    pub fn as_str(self) -> &'static str {
        match self {
            CrcConvention::X25 => "x25",
            CrcConvention::Ccitt => "ccitt",
        }
    }
}

/// X.25 FCS: reflected poly 0x8408, init 0xFFFF, xorout 0xFFFF.
pub fn crc_x25(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0x8408 } else { crc >> 1 };
        }
    }
    crc ^ 0xFFFF
}

/// CRC-16/CCITT poly 0x1021, MSB-first, non-reflected.
pub fn crc_ccitt(data: &[u8], init: u16, xorout: u16) -> u16 {
    let mut crc: u16 = init;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc ^ xorout
}

/// True when the frame's trailing two bytes (little-endian) are the X.25 FCS
/// of everything before them.
pub fn x25_ok(frame: &[u8]) -> bool {
    let n = frame.len();
    n >= 4 && crc_x25(&frame[..n - 2]) == u16::from_le_bytes([frame[n - 2], frame[n - 1]])
}

/// `(init, xorout)` variants seen to close on real frames. `(0, 0)` is
/// excluded — it false-positives on near-zero data (see `decode_air.py`).
const CCITT_VARIANTS: [(u16, u16); 3] = [(0x0000, 0xFFFF), (0xFFFF, 0xFFFF), (0xFFFF, 0x0000)];

/// True when the frame's trailing two bytes (big-endian) are a valid
/// CRC-16/CCITT over the rest under any accepted `(init, xorout)` variant.
pub fn ccitt_ok(frame: &[u8]) -> bool {
    let n = frame.len();
    if n < 4 {
        return false;
    }
    let recv = ((frame[n - 2] as u16) << 8) | frame[n - 1] as u16;
    CCITT_VARIANTS
        .iter()
        .any(|&(init, xorout)| crc_ccitt(&frame[..n - 2], init, xorout) == recv)
}

/// `> 2` distinct byte values, so constant/degenerate segments don't pass
/// (`bast_chain.py frames()` uses the same guard).
pub fn frame_nontrivial(frame: &[u8]) -> bool {
    if frame.is_empty() {
        return false;
    }
    let first = frame[0];
    let mut second = 0u8;
    let mut have_second = false;
    for &b in &frame[1..] {
        if b == first {
            continue;
        }
        if !have_second {
            second = b;
            have_second = true;
            continue;
        }
        if b != second {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    // --- Golden vectors: the two known-good frames from nemoSelfTest(). ---

    // GPS dwell frame (decode_air.py): closes CCITT (0x0000, 0xFFFF) only.
    const GPS: &str = "00000ed5098080000000000000f8e9";
    // Vamos 5020 beacon (bielefeld-live golden frame): closes X.25 only.
    const BEACON: &str = "140000a34e04056d071a01583a007cc1ff0faba7";

    #[test]
    fn gps_frame_closes_ccitt_only() {
        let f = hex(GPS);
        assert!(ccitt_ok(&f), "GPS frame must close CCITT");
        assert!(!x25_ok(&f), "GPS frame must NOT close X.25");
    }

    #[test]
    fn beacon_frame_closes_x25_only() {
        let f = hex(BEACON);
        assert!(x25_ok(&f), "beacon must close X.25");
        assert!(!ccitt_ok(&f), "beacon must NOT close CCITT");
    }

    #[test]
    fn gps_closes_the_documented_ccitt_variant() {
        let f = hex(GPS);
        let (body, trailer) = f.split_at(f.len() - 2);
        let recv = ((trailer[0] as u16) << 8) | trailer[1] as u16;
        assert_eq!(crc_ccitt(body, 0x0000, 0xFFFF), recv);
    }

    #[test]
    fn single_byte_corruption_rejected() {
        let mut f = hex(BEACON);
        for i in 0..f.len() {
            f[i] ^= 0x01;
            assert!(!x25_ok(&f), "single-byte corruption at {i} must fail X.25");
            f[i] ^= 0x01;
        }
    }

    #[test]
    fn short_frame_rejected() {
        assert!(!x25_ok(&[0x01, 0x02, 0x03]));
        assert!(!ccitt_ok(&[0x01, 0x02, 0x03]));
    }

    #[test]
    fn nontrivial_guard() {
        assert!(!frame_nontrivial(&[0xAA; 8])); // one distinct value
        assert!(!frame_nontrivial(&[0xAA, 0xBB, 0xAA, 0xBB])); // two distinct values
        assert!(frame_nontrivial(&[0xAA, 0xBB, 0xCC])); // three -> ok
    }
}
