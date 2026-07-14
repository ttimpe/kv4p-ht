//! End-to-end DSP test: drive the full front-end -> burst gate -> demod -> HDLC
//! -> CRC chain from a 48 kHz WAV of **real on-air signal** and assert it
//! extracts known G2 telegrams.
//!
//! Fixture `onair_g2_48k.wav` is the first 6 s of `lio-decoder`'s
//! `test_audio/real_signal.wav` (mono, 48 kHz) — genuine IBISplus/"G2" traffic
//! captured off air. Every frame asserted below is byte-for-byte identical to
//! the output of the validated Python reference (`nemo_hysteresis_demod.py` +
//! `decode_air.py`) on the same audio, so this test is a differential check
//! against that pipeline, not against the decoder's own past behavior.
//!
//! This replaces the previous synthesized-burst fixture. That fixture existed
//! because the old rectifying + Mueller & Müller front-end could not decode real
//! captures at all (its own docstring said so); it only ever closed a CRC on a
//! clean synthetic tone. The current front-end decodes real air traffic, so the
//! test now uses real air traffic.

use nemo_decode::{CrcConvention, NemoDecoder};

fn hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

fn load_fixture() -> Vec<i16> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/onair_g2_48k.wav");
    let mut r = hound::WavReader::open(path).expect("open fixture WAV");
    let spec = r.spec();
    assert_eq!(spec.sample_rate, 48000, "fixture must be 48 kHz");
    assert_eq!(spec.channels, 1, "fixture must be mono");
    r.samples::<i16>().map(|s| s.unwrap()).collect()
}

/// Decode the whole fixture (fed in 720-sample frames like the firmware) and
/// return the unique CCITT-convention frames — the convention the G2 location
/// telegrams close under, and the one the Python reference emits.
fn decode_ccitt() -> Vec<Vec<u8>> {
    let samples = load_fixture();
    let mut dec = NemoDecoder::new();
    let mut frames = Vec::new();
    for chunk in samples.chunks(720) {
        frames.extend(dec.feed(chunk));
    }
    let mut ccitt: Vec<Vec<u8>> = frames
        .into_iter()
        .filter(|f| f.crc == CrcConvention::Ccitt)
        .map(|f| f.bytes)
        .collect();
    ccitt.sort();
    ccitt.dedup();
    ccitt
}

#[test]
fn decodes_known_g2_telegrams_from_real_capture() {
    let got = decode_ccitt();

    // Four telegrams the Python reference also extracts byte-for-byte from this
    // audio: three `FFFF…1A40` location frames and one status frame. If the
    // demodulator regresses, these stop closing their CRC.
    let expected = [
        "ffff00000e741a40800000000000003f7e",
        "ffff00000e261a40800000000000000778",
        "ffff00000ee21a40800000000000003c63",
        "50050009daab50c0e07e4e907e4e13e0009f61",
    ];
    for e in expected {
        let want = hex(e);
        assert!(
            got.iter().any(|f| *f == want),
            "expected G2 telegram {e} to close CCITT; decoded {} unique CCITT frames: {:?}",
            got.len(),
            got.iter().map(|f| f.iter().map(|b| format!("{b:02x}")).collect::<String>()).collect::<Vec<_>>(),
        );
    }
}

#[test]
fn real_capture_yields_a_healthy_frame_count() {
    // A blunt regression guard on overall sensitivity: the 6 s clip carries many
    // telegrams, so a front-end that quietly half-works still trips this.
    let n = decode_ccitt().len();
    assert!(n >= 8, "expected >= 8 unique CCITT frames from the 6 s capture, got {n}");
}
