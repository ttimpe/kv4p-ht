//! Read raw i16 (LE) 48 kHz mono samples from stdin, print one line per decoded
//! frame ("<how> <hexbytes>"). Mirrors the C differential harness for testing.
use nemo_decode::NemoDecoder;
use std::io::Read;

fn main() {
    let mut buf = Vec::new();
    std::io::stdin().read_to_end(&mut buf).unwrap();
    let samples: Vec<i16> = buf
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect();
    let mut dec = NemoDecoder::new();
    for chunk in samples.chunks(720) {
        for f in dec.feed(chunk) {
            print!("{} ", f.crc.as_str());
            for b in &f.bytes {
                print!("{:02x}", b);
            }
            println!();
        }
    }
}
