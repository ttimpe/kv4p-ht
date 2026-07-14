//! Live HTTP audio streaming (pull model), port of the C `streamer.h`.
//!
//! Clients `GET /stream.wav` on `cfg.stream_port` and receive an endless
//! RIFF/WAV (PCM16LE mono 16 kHz, RIFF/data sizes 0xFFFFFFFF). Playable
//! directly in VLC/ffplay; the decoder server consumes the same URL.
//!
//! Structure differs from the C (which polled all clients from one task):
//! an accept thread on core 0 parses the request line and hands each stream
//! client to its own pump thread. Semantics are preserved — max 3 clients,
//! laggard resync near live, drop on zero-progress writes, all clients
//! dropped while an OTA update runs. None of these threads are
//! watchdog-registered because socket writes may block (see .ino rationale).

use std::io::{Read, Write};
use std::net::{Shutdown, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use esp_idf_svc::hal::cpu::Core;

use crate::broadcast_ring::{BroadcastRing, RESYNC_THRESHOLD};
use crate::config::STREAM_SAMPLE_RATE;
use crate::rt;

pub const MAX_STREAM_CLIENTS: u32 = 3;
pub const STREAM_CHUNK: usize = 1400;

/// C bounds each client write to 1 s (`WiFiClient::setTimeout(1)`) and drops
/// the client when a write makes no progress; the same timeout is used here.
const WRITE_TIMEOUT: Duration = Duration::from_secs(1);
/// Header-drain budget after the request line (C: 500 ms).
const REQUEST_TIMEOUT: Duration = Duration::from_millis(500);

/// Stream counters (C `streamClientCount` / `streamOverruns` /
/// `streamBytesOut`), read by the web UI status endpoint.
#[derive(Default)]
pub struct StreamStats {
    pub clients: AtomicU32,
    pub overruns: AtomicU32,
    pub bytes_out: AtomicU32,
}

/// The exact 44-byte endless-WAV header the C `writeWavStreamHeader` emitted:
/// RIFF size 0xFFFFFFFF, PCM (fmt 1), mono, 16 kHz, 16-bit, data size
/// 0xFFFFFFFF. All multi-byte fields little-endian.
pub fn wav_stream_header() -> [u8; 44] {
    let mut h = [0u8; 44];
    h[0..4].copy_from_slice(b"RIFF");
    h[4..8].copy_from_slice(&0xFFFF_FFFFu32.to_le_bytes());
    h[8..16].copy_from_slice(b"WAVEfmt ");
    h[16..20].copy_from_slice(&16u32.to_le_bytes()); // fmt chunk length
    h[20..22].copy_from_slice(&1u16.to_le_bytes()); // PCM
    h[22..24].copy_from_slice(&1u16.to_le_bytes()); // mono
    h[24..28].copy_from_slice(&STREAM_SAMPLE_RATE.to_le_bytes());
    h[28..32].copy_from_slice(&(STREAM_SAMPLE_RATE * 2).to_le_bytes()); // byte rate
    h[32..34].copy_from_slice(&2u16.to_le_bytes()); // block align
    h[34..36].copy_from_slice(&16u16.to_le_bytes()); // bits per sample
    h[36..40].copy_from_slice(b"data");
    h[40..44].copy_from_slice(&0xFFFF_FFFFu32.to_le_bytes());
    h
}

/// Start the stream server on its own core-0 thread. `port` is read once at
/// start (like the C `streamerStart`; a port change needs a reboot).
pub fn start(
    port: u16,
    ring: Arc<BroadcastRing>,
    stats: Arc<StreamStats>,
    ota_in_progress: Arc<AtomicBool>,
) -> std::io::Result<()> {
    rt::spawn(b"streamer\0", 6144, 1, Core::Core0, move || {
        accept_loop(port, &ring, &stats, &ota_in_progress);
    })?;
    Ok(())
}

fn accept_loop(
    port: u16,
    ring: &Arc<BroadcastRing>,
    stats: &Arc<StreamStats>,
    ota: &Arc<AtomicBool>,
) {
    let listener = match TcpListener::bind(("0.0.0.0", port)) {
        Ok(l) => l,
        Err(e) => {
            log::error!("[stream] bind :{port} failed: {e}");
            return;
        }
    };
    log::info!("[stream] listening on :{port}/stream.wav");

    loop {
        let Ok((sock, peer)) = listener.accept() else {
            std::thread::sleep(Duration::from_millis(200));
            continue;
        };
        // The C streamer neither accepts nor serves during an OTA update
        // (clients are dropped, connections wait in the backlog); closing
        // immediately is the polite equivalent.
        if ota.load(Ordering::Relaxed) {
            let _ = sock.shutdown(Shutdown::Both);
            continue;
        }
        handle_connection(sock, peer, ring, stats, ota);
    }
}

/// Minimal HTTP request parse: first line only, drain the rest of the headers
/// (bounded by `REQUEST_TIMEOUT`), exactly like the C `acceptStreamClient`.
fn read_request_line(sock: &mut TcpStream) -> Option<String> {
    let _ = sock.set_read_timeout(Some(REQUEST_TIMEOUT));
    let start = Instant::now();
    let mut buf = [0u8; 512];
    let mut used = 0usize;
    // Read until the blank line ending the headers, the buffer fills, or the
    // budget runs out. Responding before every header arrived is fine.
    while used < buf.len() && start.elapsed() < REQUEST_TIMEOUT {
        match sock.read(&mut buf[used..]) {
            Ok(0) => break,
            Ok(n) => {
                used += n;
                if buf[..used].windows(4).any(|w| w == b"\r\n\r\n") {
                    break;
                }
            }
            Err(_) => break,
        }
    }
    let text = String::from_utf8_lossy(&buf[..used]);
    text.lines().next().map(|l| l.to_string())
}

fn handle_connection(
    mut sock: TcpStream,
    peer: std::net::SocketAddr,
    ring: &Arc<BroadcastRing>,
    stats: &Arc<StreamStats>,
    ota: &Arc<AtomicBool>,
) {
    let Some(req_line) = read_request_line(&mut sock) else {
        return;
    };
    if !req_line.starts_with("GET /stream") {
        let _ = sock.write_all(
            b"HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\nOnly /stream.wav lives here.\n",
        );
        return;
    }
    if !ring.ensure_allocated() {
        let _ = sock.write_all(
            b"HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\nOut of memory.\n",
        );
        return;
    }
    // Reserve a client slot (drops back on refusal or spawn failure).
    if stats.clients.fetch_add(1, Ordering::AcqRel) >= MAX_STREAM_CLIENTS {
        stats.clients.fetch_sub(1, Ordering::AcqRel);
        let _ = sock.write_all(
            b"HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\nToo many stream clients.\n",
        );
        return;
    }

    log::info!("[stream] client {peer} connected");
    let ring = ring.clone();
    let stats2 = stats.clone();
    let ota = ota.clone();
    let spawned = rt::spawn(b"stream_cli\0", 6144, 1, Core::Core0, move || {
        client_pump(sock, &ring, &stats2, &ota);
        stats2.clients.fetch_sub(1, Ordering::AcqRel);
    });
    if spawned.is_err() {
        stats.clients.fetch_sub(1, Ordering::AcqRel);
        log::warn!("[stream] client thread spawn failed");
    }
}

/// Per-client pump: HTTP headers + endless-WAV header, then bytes from the
/// broadcast ring at this client's cursor. Port of the per-client body of the
/// C `streamerTask`, including the laggard skip-to-near-live.
fn client_pump(
    mut sock: TcpStream,
    ring: &BroadcastRing,
    stats: &StreamStats,
    ota: &AtomicBool,
) {
    let _ = sock.set_nodelay(true);
    let _ = sock.set_write_timeout(Some(WRITE_TIMEOUT));

    let mut preamble = Vec::with_capacity(160 + 44);
    preamble.extend_from_slice(
        b"HTTP/1.1 200 OK\r\n\
          Content-Type: audio/wav\r\n\
          Cache-Control: no-store\r\n\
          Access-Control-Allow-Origin: *\r\n\
          Connection: close\r\n\r\n",
    );
    preamble.extend_from_slice(&wav_stream_header());
    if sock.write_all(&preamble).is_err() {
        return;
    }

    let mut chunk = [0u8; STREAM_CHUNK];
    let mut pos = ring.write_pos(); // start live
    loop {
        if ota.load(Ordering::Relaxed) {
            break; // OTA in progress: drop the client (C dropStreamClient)
        }
        let mut avail = ring.write_pos().wrapping_sub(pos);
        if avail > RESYNC_THRESHOLD {
            // Laggard (TCP backpressure): skip ahead to near-live instead of
            // stalling; the client hears a glitch, the counter records it.
            pos = ring.resync_pos();
            avail = ring.write_pos().wrapping_sub(pos);
            stats.overruns.fetch_add(1, Ordering::Relaxed);
        }
        if avail == 0 {
            std::thread::sleep(Duration::from_millis(4)); // C polling cadence
            continue;
        }
        let n = (avail as usize).min(STREAM_CHUNK);
        if !ring.read_at(pos, &mut chunk[..n]) {
            stats.overruns.fetch_add(1, Ordering::Relaxed);
            pos = ring.resync_pos();
            continue;
        }
        // Partial writes advance the cursor by what was taken, like the C
        // `sc.readPos += written`; a zero-progress or failed write (incl. the
        // 1 s timeout) drops the client, like the C `written == 0` check.
        match sock.write(&chunk[..n]) {
            Ok(0) | Err(_) => break,
            Ok(written) => {
                pos = pos.wrapping_add(written as u32);
                stats.bytes_out.fetch_add(written as u32, Ordering::Relaxed);
            }
        }
    }
    let _ = sock.shutdown(Shutdown::Both);
}
