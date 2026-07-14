//! Backend ingest uplink, port of the C `uplink.h`.
//!
//! The configured base URL's scheme picks the transport (the paths are fixed by
//! the bielefeld-live ingest contract):
//!   * `http(s)://host[:port][/prefix]` -> batched `POST /api/v1/ingest`,
//!   * `ws(s)://host[:port][/prefix]`   -> persistent WebSocket to `/ws/ingest`.
//!
//! Auth is a `Bearer` station key. TLS is encrypted but unverified
//! (`CONFIG_ESP_TLS_INSECURE` / `SKIP_SERVER_CERT_VERIFY` in `sdkconfig`,
//! matching the C `setInsecure()` posture — there is no room for a CA bundle and
//! the endpoint is user-configurable).
//!
//! Records are the only retry buffer: the bounded frame queue is drop-oldest, so
//! while the uplink is down fresh telegrams win and stale ones are discarded
//! (identical to the C v1 behavior). The config-generation counter
//! (`gen.uplink`) drives hot-reconfigure exactly like the C `upCfgGen`.
//!
//! # Remote control (WebSocket mode only)
//!
//! The ingest socket is bidirectional, so it doubles as the downlink a rooftop
//! node with no physical access is administered over. The backend pushes
//! `{"type":"command",...}` documents (see [`crate::control`]); this task applies
//! them and answers with `command_result`. It also pushes a `status` report on
//! connect and every [`STATUS_EVERY`] — that report is what lets the backend
//! render a channel picker for a node it has never been told the config of.
//!
//! Commands are *queued* by the WebSocket event callback and *applied* here on
//! purpose: the callback runs on the client's own 6 kB ESP-IDF task, which has
//! no room for a config apply (NVS write + radio re-tune).
//!
//! HTTP uplink mode has no downlink and therefore no remote control — a node
//! must be on a `ws(s)://` uplink to be administered from the backend.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::SyncSender;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use esp_idf_svc::http::client::{Configuration as HttpConfig, EspHttpConnection};
use esp_idf_svc::http::Method;
use esp_idf_svc::sys;
use esp_idf_svc::ws::client::{
    EspWebSocketClient, EspWebSocketClientConfig, EspWebSocketTransport, FrameType,
    WebSocketEventType,
};
use serde_json::json;

use crate::control;
use crate::frames::{FrameRecord, Frames, FRAME_RAW_MAX};
use crate::SharedState;

/// How often a connected node re-reports its full state to the backend.
pub const STATUS_EVERY: Duration = Duration::from_secs(30);

/// Depth of the callback -> task command queue. Commands are operator actions,
/// never bulk traffic; a backlog this deep already means the task is wedged.
const CMD_QUEUE_DEPTH: usize = 8;

/// Uplink status surfaced to `/api/status`, mirroring the C globals
/// (`upMode`/`uplinkConnected()`/`upWsConnected`/`upHelloAcked`/`upLastError`).
/// The send/accept/drop counters live in [`crate::frames::FrameStats`], shared
/// with the decoder, so they are not duplicated here.
#[derive(Default)]
pub struct UplinkStatus {
    /// `"off"` / `"http"` / `"ws"` (C `UP_MODE_NAMES`).
    pub mode: Mutex<String>,
    /// C `uplinkConnected()`: always true for HTTP, else `hello_acked`.
    pub connected: AtomicBool,
    /// Raw TCP/TLS+upgrade state, pre-hello (C `upWsConnected`).
    pub ws_socket: AtomicBool,
    /// hello/hello_ack handshake done — ok to send bursts (C `upHelloAcked`).
    pub hello_acked: AtomicBool,
    /// Last error string, empty when none (C `upLastError`).
    pub last_error: Mutex<String>,
    /// OTA asks the uplink to drop its TLS connection so a manifest/image fetch
    /// can get a large-enough contiguous heap block for its own handshake — on a
    /// WROOM there isn't room for two live TLS sessions plus the decoder buffers.
    /// See [`suspended`](Self::suspended).
    pub suspend: AtomicBool,
    /// Set by the uplink task once it has actually torn its transport down and
    /// released the heap; OTA waits on this before opening its connection.
    pub suspended: AtomicBool,
}

impl UplinkStatus {
    fn set_err(&self, e: &str) {
        if let Ok(mut s) = self.last_error.lock() {
            s.clear();
            s.push_str(e);
        }
    }
    fn clear_err(&self) {
        if let Ok(mut s) = self.last_error.lock() {
            s.clear();
        }
    }
    fn set_mode(&self, m: &str) {
        if let Ok(mut s) = self.mode.lock() {
            s.clear();
            s.push_str(m);
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Off,
    Http,
    Https,
    Ws,
    Wss,
}

impl Mode {
    fn name(self) -> &'static str {
        match self {
            Mode::Off => "off",
            Mode::Http | Mode::Https => "http",
            Mode::Ws | Mode::Wss => "ws",
        }
    }
    fn is_ws(self) -> bool {
        matches!(self, Mode::Ws | Mode::Wss)
    }
}

struct Parsed {
    mode: Mode,
    host: String,
    port: u16,
    base_path: String,
}

/// Port of C `upParseUrl`: scheme -> mode/default-port, then host[:port] + path.
fn parse_url(url: &str) -> Option<Parsed> {
    let (mode, default_port, rest) = if let Some(r) = url.strip_prefix("https://") {
        (Mode::Https, 443u16, r)
    } else if let Some(r) = url.strip_prefix("http://") {
        (Mode::Http, 80, r)
    } else if let Some(r) = url.strip_prefix("wss://") {
        (Mode::Wss, 443, r)
    } else if let Some(r) = url.strip_prefix("ws://") {
        (Mode::Ws, 80, r)
    } else {
        return None;
    };

    let (host_port, path) = match rest.find('/') {
        Some(i) => (&rest[..i], &rest[i..]),
        None => (rest, ""),
    };
    let base_path = path.trim_end_matches('/').to_string();

    let (host, port) = match host_port.find(':') {
        Some(i) => (
            host_port[..i].to_string(),
            host_port[i + 1..].parse::<u16>().unwrap_or(default_port),
        ),
        None => (host_port.to_string(), default_port),
    };
    if host.is_empty() {
        return None;
    }
    Some(Parsed {
        mode,
        host,
        port,
        base_path,
    })
}

const HEXD: &[u8; 16] = b"0123456789abcdef";

/// ISO-8601 UTC timestamp (C `strftime("%Y-%m-%dT%H:%M:%SZ")`).
fn iso8601_utc(ts_unix: i64) -> String {
    unsafe {
        let t: sys::time_t = ts_unix as sys::time_t;
        let mut tmv: sys::tm = core::mem::zeroed();
        sys::gmtime_r(&t, &mut tmv);
        format!(
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
            tmv.tm_year + 1900,
            tmv.tm_mon + 1,
            tmv.tm_mday,
            tmv.tm_hour,
            tmv.tm_min,
            tmv.tm_sec
        )
    }
}

/// One burst object per the ingest contract; `ws_wrap` adds the WS envelope.
/// Port of C `upBuildBurstJson`.
fn burst_json(r: &FrameRecord, ws_wrap: bool) -> String {
    let raw = r.raw_bytes();
    let n = raw.len().min(FRAME_RAW_MAX);
    let mut hex = String::with_capacity(n * 2);
    for &b in &raw[..n] {
        hex.push(HEXD[(b >> 4) as usize] as char);
        hex.push(HEXD[(b & 0xf) as usize] as char);
    }
    let ts = if r.ts_unix > 0 {
        format!(",\"received_at\":\"{}\"", iso8601_utc(r.ts_unix))
    } else {
        String::new()
    };
    // Raw SA818 units (0-255), NOT dBm — see frames::RfRssi. Omitted entirely
    // when the module has no RSSI, which the backend reads as a null column
    // rather than a bogus 0.
    let rssi = match r.rssi {
        Some(v) => format!(",\"rssi\":{v}"),
        None => String::new(),
    };
    let proto = r.proto_str();
    if ws_wrap {
        format!("{{\"type\":\"burst\",\"raw_hex\":\"{hex}\",\"protocol\":\"{proto}\"{ts}{rssi}}}")
    } else {
        format!("{{\"raw_hex\":\"{hex}\",\"protocol\":\"{proto}\"{ts}{rssi}}}")
    }
}

/// Spawn the uplink task (core 0, stack 12288, priority 1 — C `uplinkStart`).
pub fn start(shared: SharedState) -> std::io::Result<()> {
    crate::rt::spawn(b"uplink\0", 12288, 1, esp_idf_svc::hal::cpu::Core::Core0, move || {
        uplink_task(shared);
    })
    .map(|_| ())
}

fn uplink_task(shared: SharedState) {
    let frames = shared.frames.clone();
    let status = shared.uplink.clone();
    let wifi_up = shared.wifi_connected.clone();

    let mut applied_gen = 0u32;
    let mut mode = Mode::Off;
    let mut token = String::new();
    let mut ingest_url = String::new(); // full http(s) POST url

    // HTTP(S) transport state.
    let mut http: Option<EspHttpConnection> = None;
    let mut last_post = Instant::now();

    // WS transport state (the client owns a hidden ESP-IDF task; keep it alive).
    let mut ws: Option<EspWebSocketClient<'static>> = None;
    let mut hello_sent_for_conn = false;
    let mut connect_start = Instant::now();
    let mut hello_sent_at = Instant::now();
    let mut connect_stall_reported = false;
    let mut hello_stall_reported = false;

    // Downlink commands, handed over from the WS callback (see module docs).
    let (cmd_tx, cmd_rx) = std::sync::mpsc::sync_channel::<String>(CMD_QUEUE_DEPTH);
    let mut status_sent_for_conn = false;
    let mut last_status = Instant::now();

    // OTA coordination: while OTA needs the heap, we hold no transport.
    let mut suspended_local = false;

    loop {
        // OTA wants the heap for its own TLS handshake. Drop ours (freeing the
        // WS client's ESP-IDF task, buffers and mbedTLS session) and idle until
        // OTA is done, then rebuild from config.
        if shared.uplink.suspend.load(Ordering::Relaxed) {
            if !suspended_local {
                ws = None;
                http = None;
                status.connected.store(false, Ordering::Relaxed);
                status.ws_socket.store(false, Ordering::Relaxed);
                status.hello_acked.store(false, Ordering::Relaxed);
                status.suspended.store(true, Ordering::Relaxed);
                suspended_local = true;
                log::info!("[uplink] suspended for OTA (transport released)");
            }
            std::thread::sleep(Duration::from_millis(200));
            continue;
        } else if suspended_local {
            suspended_local = false;
            status.suspended.store(false, Ordering::Relaxed);
            // Force the reconfigure block below to rebuild the transport: the
            // generation counter has not changed, so make applied_gen mismatch.
            applied_gen = applied_gen.wrapping_sub(1);
            log::info!("[uplink] resumed after OTA");
        }

        let g = shared.app.gen.uplink.load(Ordering::SeqCst);
        if g != applied_gen {
            applied_gen = g;
            // Tear down whatever is running and re-parse the config.
            ws = None;
            http = None;
            let (url, tok) = {
                let cfg = shared.app.config.read().unwrap();
                (cfg.uplink_url.clone(), cfg.uplink_token.clone())
            };
            token = tok;
            match (url.is_empty(), parse_url(&url)) {
                (false, Some(p)) => {
                    mode = p.mode;
                    status.clear_err();
                    status.set_mode(mode.name());
                    if mode.is_ws() {
                        let path = format!("{}/ws/ingest", p.base_path);
                        ws = build_ws_client(&p, &path, &token, &frames, &status, cmd_tx.clone());
                        hello_sent_for_conn = false;
                        status_sent_for_conn = false;
                        connect_start = Instant::now();
                        connect_stall_reported = false;
                        status.ws_socket.store(false, Ordering::Relaxed);
                        status.hello_acked.store(false, Ordering::Relaxed);
                        log::info!(
                            "[uplink] mode={} host={} port={} path={}",
                            mode.name(),
                            p.host,
                            p.port,
                            path
                        );
                    } else {
                        let scheme = if mode == Mode::Https { "https" } else { "http" };
                        ingest_url =
                            format!("{scheme}://{}:{}{}/api/v1/ingest", p.host, p.port, p.base_path);
                        http = EspHttpConnection::new(&HttpConfig {
                            timeout: Some(Duration::from_secs(6)),
                            ..Default::default()
                        })
                        .ok();
                        log::info!("[uplink] mode={} host={} port={}", mode.name(), p.host, p.port);
                    }
                }
                _ => {
                    mode = Mode::Off;
                    status.set_mode("off");
                    if !url.is_empty() {
                        status.set_err("bad uplink url");
                    }
                }
            }
            status.connected.store(false, Ordering::Relaxed);
        }

        if mode == Mode::Off || !wifi_up.load(Ordering::Relaxed) {
            if mode == Mode::Off {
                status.connected.store(false, Ordering::Relaxed);
            }
            std::thread::sleep(Duration::from_millis(250));
            continue;
        }

        if mode.is_ws() {
            let Some(client) = ws.as_mut() else {
                std::thread::sleep(Duration::from_millis(250));
                continue;
            };
            let connected = status.ws_socket.load(Ordering::Relaxed);

            // Rising edge: socket up -> (re)send hello. Falling edge: reset.
            if connected && !hello_sent_for_conn {
                let hello = format!("{{\"type\":\"hello\",\"station_key\":\"{token}\"}}");
                let _ = client.send(FrameType::Text(false), hello.as_bytes());
                hello_sent_for_conn = true;
                hello_sent_at = Instant::now();
                hello_stall_reported = false;
                log::info!("[uplink] ws connected, hello sent");
            } else if !connected && hello_sent_for_conn {
                hello_sent_for_conn = false;
                status_sent_for_conn = false;
                connect_start = Instant::now();
                connect_stall_reported = false;
            }

            // Stall detection (the underlying client silently retries forever on
            // a stuck lower-level handshake). Mirrors the C 12 s / 8 s grace.
            if !connected
                && !connect_stall_reported
                && connect_start.elapsed() > Duration::from_secs(12)
            {
                connect_stall_reported = true;
                status.set_err("ws: connect stuck (no handshake, check TLS/heap)");
                log::warn!("[uplink] ws connect stalled: no CONNECTED event within 12s");
            }
            if connected
                && !status.hello_acked.load(Ordering::Relaxed)
                && !hello_stall_reported
                && hello_sent_at.elapsed() > Duration::from_secs(8)
            {
                hello_stall_reported = true;
                status.set_err("ws: connected but no hello_ack (server silent)");
                log::warn!("[uplink] hello stalled: no hello_ack/error within 8s");
            }

            let acked = status.hello_acked.load(Ordering::Relaxed);
            status.connected.store(acked, Ordering::Relaxed);
            if acked {
                // Report on every fresh connection, then on the interval. The
                // backend has no other way to learn this node's channel table.
                if !status_sent_for_conn || last_status.elapsed() >= STATUS_EVERY {
                    let report = control::status_report_json(&shared);
                    if client.send(FrameType::Text(false), report.as_bytes()).is_ok() {
                        status_sent_for_conn = true;
                        last_status = Instant::now();
                    }
                }

                // Downlink commands. Applied here, not in the WS callback: this
                // task has the stack for an NVS write and a radio re-tune.
                while let Ok(text) = cmd_rx.try_recv() {
                    if let Some(reboot) = handle_command(&shared, client, &text) {
                        if reboot {
                            log::warn!("[uplink] rebooting on backend command");
                            std::thread::sleep(Duration::from_millis(500));
                            esp_idf_svc::hal::reset::restart();
                        }
                        // The command may have changed what we last reported;
                        // re-report on the next pass rather than let the backend
                        // show stale state for up to STATUS_EVERY.
                        status_sent_for_conn = false;
                    }
                }

                for r in frames.drain() {
                    let msg = burst_json(&r, true);
                    if client.send(FrameType::Text(false), msg.as_bytes()).is_ok() {
                        frames.stats.sent.fetch_add(1, Ordering::Relaxed);
                    } else {
                        frames.stats.dropped.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
            std::thread::sleep(Duration::from_millis(20));
            continue;
        }

        // HTTP(S): flush when several frames are waiting or the oldest has
        // waited ~1 s (C batch cadence).
        status.connected.store(true, Ordering::Relaxed);
        let waiting = frames.queued();
        if waiting >= 8 || (waiting > 0 && last_post.elapsed() > Duration::from_secs(1)) {
            let batch = frames.drain();
            if !batch.is_empty() {
                post_batch(&mut http, &ingest_url, &token, &batch, &frames, &status);
                last_post = Instant::now();
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Apply one queued downlink command and put its result on the wire. Returns
/// `Some(reboot)` when the command was handled, `None` when it was unparseable
/// (nothing to answer — there is no id to answer with).
fn handle_command(
    shared: &SharedState,
    client: &mut EspWebSocketClient<'static>,
    text: &str,
) -> Option<bool> {
    let doc: serde_json::Value = match serde_json::from_str(text) {
        Ok(v) => v,
        Err(e) => {
            log::warn!("[uplink] bad command json: {e}");
            return None;
        }
    };
    let id = doc.get("id").and_then(|v| v.as_i64()).unwrap_or(-1);
    let cmd = doc.get("cmd").and_then(|v| v.as_str()).unwrap_or("?").to_string();

    let outcome = control::dispatch(shared, &doc);
    let reply = if outcome.ok {
        log::info!("[uplink] command {id} ({cmd}) ok");
        json!({"type":"command_result","id":id,"ok":true})
    } else {
        let e = outcome.error.clone().unwrap_or_default();
        log::warn!("[uplink] command {id} ({cmd}) failed: {e}");
        json!({"type":"command_result","id":id,"ok":false,"error":e})
    };
    // Sent before any reboot the command triggers, so the backend records the
    // outcome instead of timing the command out.
    let _ = client.send(FrameType::Text(false), reply.to_string().as_bytes());
    Some(outcome.reboot)
}

/// Build (and start) the websocket client with a status-tracking event
/// callback. The callback runs on the client's hidden ESP-IDF task and only
/// captures `Arc` state plus the command queue's sender, so it is
/// `Send + 'static`.
fn build_ws_client(
    p: &Parsed,
    path: &str,
    token: &str,
    frames: &Arc<Frames>,
    status: &Arc<UplinkStatus>,
    cmd_tx: SyncSender<String>,
) -> Option<EspWebSocketClient<'static>> {
    let uri = format!(
        "{}://{}:{}{}",
        if p.mode == Mode::Wss { "wss" } else { "ws" },
        p.host,
        p.port,
        path
    );
    // The library wants headers terminated with CRLF (C set an extra
    // "Authorization: Bearer <token>" header for pre-auth).
    let headers = (!token.is_empty()).then(|| format!("Authorization: Bearer {token}\r\n"));

    let cb_frames = frames.clone();
    let cb_status = status.clone();
    let cb = move |ev: &Result<esp_idf_svc::ws::client::WebSocketEvent<'_>, esp_idf_svc::io::EspIOError>| {
        match ev {
            Ok(event) => match &event.event_type {
                WebSocketEventType::Connected => {
                    cb_status.ws_socket.store(true, Ordering::Relaxed);
                    cb_status.hello_acked.store(false, Ordering::Relaxed);
                }
                WebSocketEventType::Disconnected | WebSocketEventType::Closed => {
                    cb_status.ws_socket.store(false, Ordering::Relaxed);
                    cb_status.hello_acked.store(false, Ordering::Relaxed);
                    log::info!("[uplink] ws disconnected");
                }
                WebSocketEventType::Text(txt) => {
                    // Checked first: a command is the only message whose payload
                    // is attacker-ish free-form (a config patch could itself
                    // contain the substring `"ack"`).
                    if txt.contains("\"command\"") {
                        // Hand off to the uplink task — this callback runs on the
                        // client's 6 kB task and must not apply config itself.
                        if cmd_tx.try_send(txt.to_string()).is_err() {
                            log::warn!("[uplink] command queue full, dropping");
                        }
                    } else if txt.contains("\"hello_ack\"") {
                        cb_status.hello_acked.store(true, Ordering::Relaxed);
                        cb_status.clear_err();
                        log::info!("[uplink] hello acked");
                    } else if txt.contains("\"ack\"") {
                        if let Some(n) = parse_accepted(txt) {
                            cb_frames.stats.accepted.fetch_add(n, Ordering::Relaxed);
                        }
                    } else if txt.contains("\"error\"") {
                        let mut e = txt.to_string();
                        // Truncate on a char boundary: byte-index truncate
                        // panics if a server-sent multibyte char straddles it.
                        let mut cut = 63.min(e.len());
                        while cut > 0 && !e.is_char_boundary(cut) {
                            cut -= 1;
                        }
                        e.truncate(cut);
                        cb_status.set_err(&e);
                        log::warn!("[uplink] ws error: {txt}");
                    }
                }
                _ => {}
            },
            Err(_) => {
                cb_status.set_err("ws error");
            }
        }
    };

    let config = EspWebSocketClientConfig {
        transport: if p.mode == Mode::Wss {
            EspWebSocketTransport::TransportOverSSL
        } else {
            EspWebSocketTransport::TransportOverTCP
        },
        headers: headers.as_deref(),
        reconnect_timeout_ms: Duration::from_millis(5000),
        network_timeout_ms: Duration::from_millis(10000),
        ping_interval_sec: Duration::from_secs(15),
        task_stack: 6144,
        // 2 kB (was 1 kB) so a downlink command arrives in a single frame.
        // esp-idf-svc's event callback hands us one `Text` event per *frame*
        // and exposes no offset/total, so a fragmented command would reach
        // `handle_command` as truncated JSON and be rejected. Config patches and
        // per-slot `channel` patches are a few hundred bytes; a full 32-entry
        // `channels` table is ~2.5 kB and is the one command the backend must
        // not send over the WS (it edits slots one at a time instead).
        // Outbound is unaffected — the client chunks long sends itself, so the
        // multi-kB status report goes out fine.
        buffer_size: 2048,
        // TLS server-cert verification is skipped globally via sdkconfig
        // (CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY) — no CA bundle attached,
        // matching the C setInsecure() posture. Do NOT also set
        // skip_cert_common_name_check: in esp-tls that skips
        // mbedtls_ssl_set_hostname(), which is what sends SNI — servers with
        // ssl_reject_handshake (stadtbahn.live) then fatal-alert the handshake
        // (unrecognized_name, seen on hardware as -0x7780).
        ..Default::default()
    };

    match EspWebSocketClient::new(&uri, &config, Duration::from_secs(5), cb) {
        Ok(c) => Some(c),
        Err(e) => {
            status.set_err("ws init failed");
            log::error!("[uplink] ws client init failed: {e:?}");
            None
        }
    }
}

/// Extract the integer after `"accepted":` in a server message.
fn parse_accepted(txt: &str) -> Option<u32> {
    let idx = txt.find("\"accepted\":")? + "\"accepted\":".len();
    let rest = txt[idx..].trim_start();
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    rest[..end].parse::<u32>().ok()
}

/// POST a drained batch to `<base>/api/v1/ingest`. Records are dropped on
/// failure — the bounded queue is the only retry buffer (C `upPostBatch`).
fn post_batch(
    http: &mut Option<EspHttpConnection>,
    url: &str,
    token: &str,
    batch: &[FrameRecord],
    frames: &Frames,
    status: &UplinkStatus,
) {
    let n = batch.len() as u32;
    let mut body = String::from("{\"bursts\":[");
    for (i, r) in batch.iter().enumerate() {
        if i > 0 {
            body.push(',');
        }
        body.push_str(&burst_json(r, false));
    }
    body.push_str("]}");

    // Re-open the connection if a previous request tore it down.
    if http.is_none() {
        *http = EspHttpConnection::new(&HttpConfig {
            timeout: Some(Duration::from_secs(6)),
            ..Default::default()
        })
        .ok();
    }
    let Some(conn) = http.as_mut() else {
        frames.stats.dropped.fetch_add(n, Ordering::Relaxed);
        status.set_err("http begin failed");
        return;
    };

    let len = body.len().to_string();
    let auth = format!("Bearer {token}");
    let mut headers: Vec<(&str, &str)> = vec![
        ("Content-Type", "application/json"),
        ("Content-Length", &len),
    ];
    if !token.is_empty() {
        headers.push(("Authorization", &auth));
    }

    let result = (|| -> Result<u16, esp_idf_svc::sys::EspError> {
        conn.initiate_request(Method::Post, url, &headers)?;
        conn.write_all(body.as_bytes())?;
        conn.initiate_response()?;
        Ok(conn.status())
    })();

    match result {
        Ok(code) if (200..300).contains(&code) => {
            frames.stats.sent.fetch_add(n, Ordering::Relaxed);
            // Read {"accepted":N,"rejected":M} and add the acked count.
            let mut resp = String::new();
            let mut buf = [0u8; 128];
            while let Ok(got) = conn.read(&mut buf) {
                if got == 0 {
                    break;
                }
                resp.push_str(&String::from_utf8_lossy(&buf[..got]));
                if resp.len() > 512 {
                    break;
                }
            }
            if let Some(acc) = parse_accepted(&resp) {
                frames.stats.accepted.fetch_add(acc, Ordering::Relaxed);
            }
            status.clear_err();
        }
        Ok(code) => {
            frames.stats.dropped.fetch_add(n, Ordering::Relaxed);
            status.set_err(&format!("http {code}"));
            *http = None; // force a fresh connection next time
        }
        Err(_) => {
            frames.stats.dropped.fetch_add(n, Ordering::Relaxed);
            status.set_err("http request failed");
            *http = None;
        }
    }
}
