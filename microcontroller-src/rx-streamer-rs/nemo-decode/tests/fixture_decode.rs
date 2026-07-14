//! End-to-end DSP test: drive the full front-end -> burst gate -> clock recovery
//! -> HDLC -> CRC chain from a 48 kHz WAV and assert it extracts the expected
//! CRC-valid frame(s).
//!
//! Fixture `beacon_burst_48k.wav` is a synthesized on-air burst carrying the
//! Vamos-5020 beacon telegram (the `bielefeld-live` golden frame). It uses the
//! patent physical layer — AMI cos^2 half-wave pulses of a 2400 Hz tone at
//! 4800 Bd, with an alternating preamble so the Mueller & Müller loop locks, the
//! frame delimited by HDLC flag runs and repeated to exceed the 80 ms
//! min-burst. Real-capture excerpts do NOT decode through this best-effort
//! front-end (the lio-decoder project's validated pipeline uses a different grid
//! demod, and single telegrams fall under the 80 ms min-burst); a synthesized
//! burst is the reliable, decodable end-to-end vector.
//!
//! The expected output is the exact, byte-for-byte output of the reference C
//! decoder (`decoder_nemo.h`) compiled and run on the identical samples, so this
//! doubles as a differential check of the port.

use nemo_decode::{CrcConvention, NemoDecoder};

fn hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

fn load_fixture() -> Vec<i16> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/beacon_burst_48k.wav");
    let mut r = hound::WavReader::open(path).expect("open fixture WAV");
    let spec = r.spec();
    assert_eq!(spec.sample_rate, 48000, "fixture must be 48 kHz");
    assert_eq!(spec.channels, 1, "fixture must be mono");
    r.samples::<i16>().map(|s| s.unwrap()).collect()
}

#[test]
fn decodes_beacon_from_synthesized_burst() {
    let samples = load_fixture();
    let mut dec = NemoDecoder::new();

    // Feed in 720-sample frames exactly like the firmware.
    let mut frames = Vec::new();
    for chunk in samples.chunks(720) {
        frames.extend(dec.feed(chunk));
    }

    // At least one CRC-valid frame, and the exact Vamos-5020 beacon closes X.25.
    let beacon = hex("140000a34e04056d071a01583a007cc1ff0faba7");
    assert!(
        frames
            .iter()
            .any(|f| f.crc == CrcConvention::X25 && f.bytes == beacon),
        "expected the beacon to close X.25; got {:?}",
        frames
    );
}

#[test]
fn matches_reference_c_output_exactly() {
    let samples = load_fixture();
    let mut dec = NemoDecoder::new();
    let frames = dec.feed(&samples);

    // Golden = verbatim output of the C harness around decoder_nemo.h on the
    // same samples: an X.25 beacon and a CCITT-aliased shift of the same burst.
    let expected: Vec<(CrcConvention, Vec<u8>)> = vec![
        (CrcConvention::X25, hex("140000a34e04056d071a01583a007cc1ff0faba7")),
        (CrcConvention::Ccitt, hex("280000c57220a0b6e058801a5c003e83fff0d5e5")),
    ];
    let got: Vec<(CrcConvention, Vec<u8>)> =
        frames.iter().map(|f| (f.crc, f.bytes.clone())).collect();
    assert_eq!(got, expected, "Rust output must match the C decoder byte-for-byte");
}
