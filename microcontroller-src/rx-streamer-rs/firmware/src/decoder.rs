//! Telegram decoder task, port of the C `decoder.h`.
//!
//! The audio pump (core 1, watchdog-registered) copies each unmuted audio
//! frame into a bounded channel; this task (core 0) drains it and runs the
//! demodulator selected by the active channel's `data_proto`. Only one decoder
//! ever runs — one radio, one channel. FFSK consumes the 16 kHz stream, NEMO
//! the 48 kHz one, so the frame variant also identifies the rate (the C used
//! the ring-frame size for the same purpose).
//!
//! The demodulators themselves live in the `ffsk-decode` and `nemo-decode`
//! crates (the same code the C headers were ported from); this module owns the
//! plumbing: protocol hot-swap, the burst -> telegram byte sweep that keeps the
//! raw (checkable) bytes for the uplink, R09 field extraction into
//! [`FrameRecord`], and the NEMO within-burst dedup.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender};
use std::sync::Arc;
use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::sys;

use ffsk_decode::crc::{Repairer, MAX_TELEGRAM_BYTES, MIN_TELEGRAM_BYTES};
use ffsk_decode::dsp::{BitEvent, Demodulator};
use ffsk_decode::framer::Framer;
use ffsk_decode::r09::{self, Telegram};
use nemo_decode::NemoDecoder;

use crate::config::{
    self, AppState, FRAME_SAMPLES_16K, FRAME_SAMPLES_48K, PROTO_FFSK_VDV, PROTO_NEMO_LIO,
    PROTO_NONE, STREAM_SAMPLE_RATE,
};
use crate::frames::{FrameRecord, Frames};
use crate::rt;

/// Channel depth: ~12 frames of 15 ms each, ~180 ms of backlog (the C ring
/// held ~170 ms of 48 kHz audio).
pub const DEC_CHANNEL_DEPTH: usize = 12;

/// Deduped telegrams within one burst window (C `NemoSinkState::seen[6]`).
const NEMO_SEEN_MAX: usize = 6;
/// Telegram candidates kept per FFSK burst (C `FFSK_MAX_RESULTS`).
const FFSK_MAX_RESULTS: usize = 4;

/// One unmuted audio frame, sent by value from the audio pump. The variant
/// identifies the sample rate (16 kHz for FFSK, 48 kHz for NEMO).
pub enum AudioFrame {
    Pcm16k([i16; FRAME_SAMPLES_16K]),
    Pcm48k([i16; FRAME_SAMPLES_48K]),
}

/// Producer-visible decoder state: which protocol wants audio right now
/// (written by the decoder task on config change, read by the audio pump) and
/// the feed-drop counter (C `decRequestedProto` / `decFeedDrops`).
#[derive(Default)]
pub struct DecoderShared {
    pub requested_proto: AtomicU8,
    pub feed_drops: AtomicU32,
    /// A TLS user (wss uplink handshake, https batch, OTA fetch) asks the
    /// decoder to drop its demodulator state — the NEMO front-end alone holds
    /// ~13 kB heap that a WROOM cannot spare while mbedTLS sets up a session.
    /// The decoder acks via [`suspended`](Self::suspended) once the memory is
    /// actually free and rebuilds its state when the flag clears (same
    /// contract as the OTA↔uplink suspend pair in `uplink.rs`).
    pub suspend: AtomicBool,
    pub suspended: AtomicBool,
}

/// Ask the decoder to release its demod state and wait (bounded) until it has.
/// Call from the task that is about to open a TLS connection; pair with
/// [`resume`]. Waiting is best-effort: after 2 s the caller proceeds anyway —
/// a wedged decoder must not also take the uplink down.
pub fn pause_for_tls(shared: &DecoderShared) {
    shared.suspend.store(true, Ordering::SeqCst);
    let t0 = std::time::Instant::now();
    while !shared.suspended.load(Ordering::SeqCst)
        && t0.elapsed() < Duration::from_secs(2)
    {
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// Let the decoder rebuild its demod state (after the TLS handshake settled).
pub fn resume(shared: &DecoderShared) {
    shared.suspend.store(false, Ordering::SeqCst);
}

/// Audio-pump side (core 1): copy the frame the active protocol wants into
/// the channel. `try_send` only — the audio thread must never block; a full
/// channel bumps the drop counter, like the C `decoderFeed`.
pub fn feed(
    shared: &DecoderShared,
    tx: &SyncSender<AudioFrame>,
    b48k: &[i16],
    b16k: &[i16],
) {
    let frame = match shared.requested_proto.load(Ordering::Relaxed) {
        PROTO_FFSK_VDV => {
            let mut buf = [0i16; FRAME_SAMPLES_16K];
            buf.copy_from_slice(b16k);
            AudioFrame::Pcm16k(buf)
        }
        PROTO_NEMO_LIO => {
            let mut buf = [0i16; FRAME_SAMPLES_48K];
            buf.copy_from_slice(b48k);
            AudioFrame::Pcm48k(buf)
        }
        _ => return,
    };
    if tx.try_send(frame).is_err() {
        shared.feed_drops.fetch_add(1, Ordering::Relaxed);
    }
}

/// Start the decoder thread (core 0, priority 2 like the C `decoderTask`).
pub fn start(
    state: Arc<AppState>,
    frames: Arc<Frames>,
    shared: Arc<DecoderShared>,
    rx: Receiver<AudioFrame>,
) -> std::io::Result<()> {
    // 16 kB stack, measured need: each loop moves a ~1.5 kB AudioFrame by
    // value, plus demod scratch and log formatting — 8 kB overflowed on real
    // hardware the moment FFSK frames arrived (crash-loop, caught on bench).
    rt::spawn(b"decoder\0", 16384, 2, Core::Core0, move || {
        decoder_task(&state, &frames, &shared, &rx);
    })
    .map(|_| ())
}

/// FFSK demodulator + framer pair, created together on protocol entry
/// (C `ffskDemodInit` + `ffskFramerInit` in `decApplyProto`).
struct FfskState {
    demod: Demodulator,
    framer: Framer,
    events: Vec<BitEvent>,
}

impl FfskState {
    fn new() -> FfskState {
        FfskState {
            demod: Demodulator::new(STREAM_SAMPLE_RATE as f32),
            framer: Framer::new(),
            events: Vec::new(),
        }
    }
}

fn decoder_task(
    state: &AppState,
    frames: &Frames,
    shared: &DecoderShared,
    rx: &Receiver<AudioFrame>,
) {
    // Search-mode repairer (the crate's only mode now): repairs run on CRC
    // failures only, no repair table held in RAM.
    let repairer = Repairer::new(true);

    let mut applied_gen = 0u32;
    let mut active = PROTO_NONE;
    let mut ffsk: Option<FfskState> = None;
    let mut nemo: Option<NemoDecoder> = None;
    let mut paused = false;

    loop {
        // TLS pause: drop the demod state so a handshake elsewhere gets the
        // heap; ack via `suspended`, rebuild when the flag clears. The audio
        // pump stops copying frames because requested_proto reads NONE.
        if shared.suspend.load(Ordering::SeqCst) {
            if !paused {
                paused = true;
                ffsk = None;
                nemo = None;
                active = PROTO_NONE;
                shared.requested_proto.store(PROTO_NONE, Ordering::Relaxed);
                while rx.try_recv().is_ok() {}
                let heap = unsafe { sys::esp_get_free_heap_size() };
                log::info!("[decoder] paused for TLS, demod state freed, heap={heap}");
                shared.suspended.store(true, Ordering::SeqCst);
            }
            std::thread::sleep(Duration::from_millis(20));
            continue;
        } else if paused {
            paused = false;
            shared.suspended.store(false, Ordering::SeqCst);
            // Force the re-apply below to rebuild the decoder for the active
            // channel (the generation counter itself has not changed).
            applied_gen = applied_gen.wrapping_sub(1);
            log::info!("[decoder] resumed");
        }

        // Config hot-reload: re-read the active channel/VFO protocol and swap
        // decoder state (C decoderReconfigure + decApplyProto).
        let g = state.gen.decoder.load(Ordering::SeqCst);
        if g != applied_gen {
            applied_gen = g;
            let proto = match (state.config.read(), state.channels.read()) {
                (Ok(cfg), Ok(ch)) => config::active_data_proto(&cfg, &ch),
                _ => active,
            };
            shared.requested_proto.store(proto, Ordering::Relaxed);
            if proto != active {
                // Discard queued audio from the previous mode (C decRd = decWr).
                while rx.try_recv().is_ok() {}
                ffsk = (proto == PROTO_FFSK_VDV).then(FfskState::new);
                // NemoDecoder owns ~18 kB; drop it whenever NEMO is inactive
                // and construct fresh on re-entry (C alloc/free of nemoEnvBuf).
                nemo = (proto == PROTO_NEMO_LIO).then(NemoDecoder::new);
                active = proto;
                let heap = unsafe { sys::esp_get_free_heap_size() };
                log::info!("[decoder] proto={proto} heap={heap}");
            }
        }

        match rx.recv_timeout(Duration::from_millis(100)) {
            Ok(AudioFrame::Pcm16k(buf)) if active == PROTO_FFSK_VDV => {
                if let Some(f) = ffsk.as_mut() {
                    process_ffsk(f, &buf, &repairer, frames);
                }
            }
            Ok(AudioFrame::Pcm48k(buf)) if active == PROTO_NEMO_LIO => {
                if let Some(n) = nemo.as_mut() {
                    process_nemo(n, &buf, frames);
                }
            }
            Ok(_) => {} // stale frame from before a protocol switch; drop it
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => {
                // The audio pump never exits; treat as idle just in case.
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }
}

// --- FFSK sink (C decFfskBitSink) ---

fn process_ffsk(f: &mut FfskState, buf: &[i16], repairer: &Repairer, frames: &Frames) {
    f.events.clear();
    f.demod.process(buf, &mut f.events);
    // Disjoint field borrows: events read-only, framer mutable.
    let FfskState { framer, events, .. } = f;
    for ev in events.iter() {
        let burst = match ev {
            BitEvent::Bit(b) => framer.push_bit(*b),
            // Gate closed: the burst is over, flush the partial collection.
            BitEvent::GateClosed => framer.flush(),
        };
        if let Some(burst) = burst {
            frames.stats.bursts.fetch_add(1, Ordering::Relaxed);
            // Consume the peak level once per burst, not once per record: a
            // burst can yield several hits, and they all describe the same
            // transmission, so they must all carry the same signal level.
            let rssi = frames.rssi.take_peak();
            decode_ffsk_burst(&burst.bits, repairer, frames, rssi);
        }
    }
}

/// One accepted telegram candidate within a burst: the repaired, checkable
/// bytes (incl. de-inverted CRC — exactly what the C stored in
/// `FfskResult.bytes` and the uplink hex-dumps) plus the parsed telegram.
struct FfskHit {
    raw: Vec<u8>,
    repaired: u32,
    telegram: Telegram,
}

/// Byte-level burst sweep, port of the C `ffskDecodeBurst` (structurally the
/// same as `r09::decode_burst`, but re-implemented here because the crate API
/// does not return the raw telegram bytes the uplink needs): every plausible
/// bit offset and length is tried, `parse_frame` applies CRC/repair and the
/// structural acceptance rules, and overlapping hits are deduped on their raw
/// bytes keeping the least-repaired instance.
fn decode_ffsk_burst(bits: &[u8], repairer: &Repairer, frames: &Frames, rssi: Option<u8>) {
    let mut found: Vec<FfskHit> = Vec::new();

    for offset in 0..3usize {
        if bits.len() <= offset {
            break;
        }
        let bits = &bits[offset..];
        // On air each byte is 9 bits: 8 data bits LSB-first plus one fill bit.
        let byte_array: Vec<u8> = bits
            .chunks_exact(9)
            .map(|c| c[..8].iter().enumerate().fold(0, |b, (i, &v)| b | (v << i)))
            .collect();

        let max_len = byte_array.len().min(MAX_TELEGRAM_BYTES);
        for len in MIN_TELEGRAM_BYTES..=max_len {
            let mut cand = byte_array[..len].to_vec();
            // The CRC bytes are transmitted inverted.
            cand[len - 2] ^= 0xff;
            cand[len - 1] ^= 0xff;

            // parse_frame repairs an internal copy and applies the acceptance
            // rules with the true repair count; on success, repair `cand`
            // itself (deterministic — same syndrome, same flips) so the
            // record carries the corrected bytes like the C did.
            let Some(decoded) = r09::parse_frame(&cand, repairer) else {
                continue;
            };
            if decoded.repaired_bits > 0 {
                let _ = repairer.check_and_repair(&mut cand);
            }

            // Dedup overlapping length/offset hits, keeping the least-repaired.
            if let Some(existing) = found.iter_mut().find(|h| h.raw == cand) {
                existing.repaired = existing.repaired.min(decoded.repaired_bits);
                continue;
            }
            if found.len() < FFSK_MAX_RESULTS {
                found.push(FfskHit {
                    raw: cand,
                    repaired: decoded.repaired_bits,
                    telegram: decoded.telegram,
                });
            }
        }
    }

    for hit in found {
        let mut r = ffsk_record(&hit);
        r.rssi = rssi;
        frames.enqueue(r);
    }
}

/// Map a decoded FFSK telegram into a [`FrameRecord`], mirroring how the C
/// `decFfskBitSink` copied `FfskResult` fields:
///   * `raw` = telegram incl. de-inverted CRC (checkable form),
///   * R09.14/16 -> label "R09.<type>", line/run/meldepunkt (+destination on 16),
///   * R09.0.7 -> the crate's native vendor-telegram parse (line/route/run/
///     meldepunkt/zuglaenge — the field extraction that used to live here),
///   * anything else -> label only ("R09.x.y" / "C09.x.y" / "Rnn" / "Cnn").
fn ffsk_record(hit: &FfskHit) -> FrameRecord {
    let mut r = FrameRecord::new(PROTO_FFSK_VDV);
    r.set_raw(&hit.raw);
    r.repaired_bits = hit.repaired as u8;

    match &hit.telegram {
        Telegram::R09(t) => {
            r.label = format!("R09.{}", t.r09_type);
            r.line = Some(t.line as u16);
            r.run = Some(t.run_number as u8);
            r.meldepunkt = Some(t.reporting_point);
            r.destination = t.destination.map(|d| d as u16);
        }
        Telegram::R09_0_7(t) => {
            r.label = "R09.0.7".to_string();
            r.line = Some(t.line as u16);
            r.run = Some(t.run as u8);
            r.route = Some(t.route as u16);
            r.meldepunkt = Some(t.reporting_point);
            r.zuglaenge = Some(t.train_length);
        }
        Telegram::Raw { label, data: _ } => {
            r.label = label
                .split_whitespace()
                .next()
                .unwrap_or_default()
                .to_string();
        }
    }
    r
}

// --- NEMO sink (C decNemoFrameSink) ---

/// Feed one 48 kHz frame; the crate returns every CRC-valid telegram
/// finalized during the call (all frames of a burst finalize in one call, so
/// the call is the dedup window — the C reset `nemoSink.nSeen` per feed).
fn process_nemo(dec: &mut NemoDecoder, buf: &[i16], frames: &Frames) {
    let before = dec.bursts();
    let out = dec.feed(buf);
    // Count every burst the energy gate finalized, decodable or not — the C
    // stBursts semantics an operator relies on to see the demodulator firing
    // on a noisy channel.
    let n_bursts = dec.bursts() - before;
    if n_bursts > 0 {
        frames.stats.bursts.fetch_add(n_bursts as u32, Ordering::Relaxed);
    }
    if out.is_empty() {
        return;
    }

    // One level for every record out of this call — see process_ffsk.
    let rssi = frames.rssi.take_peak();

    // Within-burst dedup by byte content, up to NEMO_SEEN_MAX distinct frames
    // (the offset/polarity sweep can accept the same telegram twice).
    let mut seen: Vec<&[u8]> = Vec::with_capacity(NEMO_SEEN_MAX);
    for f in &out {
        if seen.iter().any(|s| *s == f.bytes.as_slice()) {
            continue;
        }
        if seen.len() < NEMO_SEEN_MAX {
            seen.push(&f.bytes);
        }
        let mut r = FrameRecord::new(PROTO_NEMO_LIO);
        // Raw bytes = decoded frame incl. the 2-byte CRC trailer.
        r.set_raw(&f.bytes);
        r.label = format!(
            "g2.{:02X} {}B {}",
            f.bytes.get(3).copied().unwrap_or(0),
            f.bytes.len(),
            f.crc.as_str()
        );
        r.rssi = rssi;
        frames.enqueue(r);
    }
}
