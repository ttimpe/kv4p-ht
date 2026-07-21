//! Web UI + JSON control API, port of the C `webui.h`.
//!
//! An [`EspHttpServer`] on port 80 serves the single-page UI (`assets/index.html`,
//! embedded via `include_str!`) and a small JSON API the embedded JS drives. The
//! JSON contract is kept faithful to the C so the same page script works
//! unchanged; the only functional change on the client side is that the firmware
//! upload posts a raw `application/octet-stream` body (streamed straight into
//! `EspOta`) instead of a multipart form.
//!
//! Mutating routes (`POST /api/config`, `PUT /api/channels`, `POST /update`) are
//! guarded by HTTP Basic auth against `admin:<admin_pass>` when an admin
//! password is set, exactly like the C `requireAuth()`.
//!
//! The state serialization and the config/channel apply logic live in
//! [`crate::control`], shared with the backend command downlink so a remote
//! retune and a local one go through exactly the same validation and NVS write.

use std::sync::atomic::Ordering;
use std::time::Duration;

use esp_idf_svc::http::server::{Configuration, Connection, EspHttpServer, Request};
use esp_idf_svc::http::Method;
use esp_idf_svc::ota::EspOta;
use serde_json::{json, Value};

use crate::control;
use crate::frames::uptime_ms;
use crate::SharedState;

const INDEX_HTML: &str = include_str!("../assets/index.html");
const JSON: &str = "application/json";

/// Start the web server on port 80. The returned handle must be kept alive for
/// the server to keep running (main binds it for the process lifetime).
pub fn start(shared: SharedState) -> anyhow::Result<EspHttpServer<'static>> {
    let mut server = EspHttpServer::new(&Configuration {
        http_port: 80,
        stack_size: 8192,
        max_uri_handlers: 12,
        ..Default::default()
    })?;

    server.fn_handler("/", Method::Get, |req| -> anyhow::Result<()> {
        resp(
            req,
            200,
            Some("OK"),
            &[("Content-Type", "text/html")],
            INDEX_HTML.as_bytes(),
        )
    })?;

    let s = shared.clone();
    server.fn_handler("/api/status", Method::Get, move |req| -> anyhow::Result<()> {
        resp_json(req, 200, &control::status_json(&s))
    })?;

    let s = shared.clone();
    server.fn_handler("/api/telegrams", Method::Get, move |req| -> anyhow::Result<()> {
        resp_json(req, 200, &telegrams_json(&s))
    })?;

    let s = shared.clone();
    server.fn_handler("/api/config", Method::Get, move |req| -> anyhow::Result<()> {
        resp_json(req, 200, &control::config_json(&s))
    })?;

    // Debug: the uplink's event ring (connect attempts, probes, real errors).
    let s = shared.clone();
    server.fn_handler("/api/uplinklog", Method::Get, move |req| -> anyhow::Result<()> {
        resp_json(req, 200, &json!({ "log": s.uplink.log.snapshot() }).to_string())
    })?;

    // Debug: last crash (from RTC memory, survives the reboot) + reset reason.
    server.fn_handler("/api/crashlog", Method::Get, move |req| -> anyhow::Result<()> {
        let last = crate::crashlog::last_crash().map(|(kind, up, msg)| {
            json!({
                "kind": crate::crashlog::kind_str(kind),
                "uptimeMs": up,
                "msg": msg,
            })
        });
        let body = json!({
            "resetReason": crate::crashlog::reset_reason(),
            "lastCrash": last,
        });
        resp_json(req, 200, &body.to_string())
    })?;

    let s = shared.clone();
    server.fn_handler("/api/config", Method::Post, move |mut req| -> anyhow::Result<()> {
        if !auth_ok(&req, &s) {
            return resp_401(req);
        }
        let body = read_body(&mut req, 4096);
        let doc: Value = match serde_json::from_slice(&body) {
            Ok(v) => v,
            Err(_) => return resp(req, 400, Some("Bad Request"), &[], b"bad json"),
        };
        match control::apply_config(&s, &doc) {
            Ok(reboot) => {
                resp_json(req, 200, &json!({ "ok": true, "reboot": reboot }).to_string())?;
                if reboot {
                    std::thread::sleep(Duration::from_millis(500));
                    esp_idf_svc::hal::reset::restart();
                }
                Ok(())
            }
            Err(msg) => resp(req, 400, Some("Bad Request"), &[], msg.as_bytes()),
        }
    })?;

    let s = shared.clone();
    server.fn_handler("/api/channels", Method::Get, move |req| -> anyhow::Result<()> {
        resp_json(req, 200, &control::channels_json(&s))
    })?;

    let s = shared.clone();
    server.fn_handler("/api/channels", Method::Put, move |mut req| -> anyhow::Result<()> {
        if !auth_ok(&req, &s) {
            return resp_401(req);
        }
        let body = read_body(&mut req, 8192);
        let doc: Value = match serde_json::from_slice(&body) {
            Ok(v) => v,
            Err(_) => return resp(req, 400, Some("Bad Request"), &[], b"bad json"),
        };
        match control::apply_channels(&s, &doc) {
            Ok(()) => resp_json(req, 200, &json!({ "ok": true }).to_string()),
            Err(msg) => resp(req, 400, Some("Bad Request"), &[], msg.as_bytes()),
        }
    })?;

    let s = shared.clone();
    server.fn_handler("/update", Method::Post, move |mut req| -> anyhow::Result<()> {
        if !auth_ok(&req, &s) {
            s.ota_in_progress.store(false, Ordering::Relaxed);
            return resp_401(req);
        }
        // Manual OTA: the raw request body IS the firmware image.
        s.ota_in_progress.store(true, Ordering::Relaxed);
        log::info!("[ota] receiving manual upload");
        match stream_update(&mut req) {
            Ok(()) => {
                resp(req, 200, Some("OK"), &[("Content-Type", "text/plain")], b"OK")?;
                log::info!("[ota] manual update ok, rebooting");
                std::thread::sleep(Duration::from_millis(500));
                esp_idf_svc::hal::reset::restart();
            }
            Err(msg) => {
                s.ota_in_progress.store(false, Ordering::Relaxed);
                log::warn!("[ota] manual update failed: {msg}");
                resp(req, 500, Some("Internal Error"), &[], msg.as_bytes())
            }
        }
    })?;

    log::info!("[web] server up on :80");
    Ok(server)
}

// --- response helpers ---

fn resp<C>(
    req: Request<C>,
    code: u16,
    message: Option<&str>,
    headers: &[(&str, &str)],
    body: &[u8],
) -> anyhow::Result<()>
where
    C: Connection,
    C::Error: core::fmt::Debug,
{
    let mut r = req
        .into_response(code, message, headers)
        .map_err(|e| anyhow::anyhow!("into_response: {e:?}"))?;
    r.write(body).map_err(|e| anyhow::anyhow!("write: {e:?}"))?;
    Ok(())
}

fn resp_json<C>(req: Request<C>, code: u16, body: &str) -> anyhow::Result<()>
where
    C: Connection,
    C::Error: core::fmt::Debug,
{
    resp(req, code, Some("OK"), &[("Content-Type", JSON)], body.as_bytes())
}

/// Reads the request body up to `cap` bytes (`httpd_req_recv` returns 0 at end).
fn read_body<C: Connection>(req: &mut Request<C>, cap: usize) -> Vec<u8> {
    let mut out = Vec::new();
    let mut buf = [0u8; 512];
    loop {
        match req.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                out.extend_from_slice(&buf[..n]);
                if out.len() >= cap {
                    break;
                }
            }
            Err(_) => break,
        }
    }
    out
}

// --- auth (HTTP Basic, C `requireAuth`) ---

/// True when the request may proceed: no admin password set, or the
/// `Authorization: Basic base64(admin:<pass>)` header matches. Only reads the
/// header (immutable borrow), so the caller keeps ownership of `req` and can
/// consume it to send a 401 when this returns false.
fn auth_ok<C: Connection>(req: &Request<C>, s: &SharedState) -> bool {
    let admin = s.app.config.read().unwrap().admin_pass.clone();
    if admin.is_empty() {
        return true;
    }
    req.header("Authorization")
        .and_then(|h| h.strip_prefix("Basic "))
        .and_then(|b| base64_decode(b.trim()))
        .map(|d| d == format!("admin:{admin}").into_bytes())
        .unwrap_or(false)
}

fn resp_401<C>(req: Request<C>) -> anyhow::Result<()>
where
    C: Connection,
    C::Error: core::fmt::Debug,
{
    resp(
        req,
        401,
        Some("Unauthorized"),
        &[("WWW-Authenticate", "Basic realm=\"kv4p\"")],
        b"auth required",
    )
}

fn base64_decode(s: &str) -> Option<Vec<u8>> {
    fn val(c: u8) -> Option<u8> {
        match c {
            b'A'..=b'Z' => Some(c - b'A'),
            b'a'..=b'z' => Some(c - b'a' + 26),
            b'0'..=b'9' => Some(c - b'0' + 52),
            b'+' => Some(62),
            b'/' => Some(63),
            _ => None,
        }
    }
    let mut acc = 0u32;
    let mut bits = 0u32;
    let mut out = Vec::new();
    for c in s.bytes().filter(|&c| c != b'=') {
        acc = (acc << 6) | val(c)? as u32;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
        }
    }
    Some(out)
}

// --- JSON builders (field-faithful to webui.h) ---

fn hex_str(bytes: &[u8]) -> String {
    const HEXD: &[u8; 16] = b"0123456789abcdef";
    let mut s = String::with_capacity(bytes.len() * 2);
    for &b in bytes {
        s.push(HEXD[(b >> 4) as usize] as char);
        s.push(HEXD[(b & 0xf) as usize] as char);
    }
    s
}

fn telegrams_json(s: &SharedState) -> String {
    let hist = s.frames.history_snapshot(); // newest-first, <= 50
    let now = uptime_ms();
    let arr: Vec<Value> = hist
        .iter()
        .map(|r| {
            let mut o = json!({
                "ageS": now.wrapping_sub(r.uptime_ms) / 1000,
                "protocol": r.proto_str(),
                "label": r.label,
                "rawHex": hex_str(r.raw_bytes()),
            });
            if let Some(v) = r.line {
                o["line"] = json!(v);
            }
            if let Some(v) = r.run {
                o["run"] = json!(v);
            }
            if let Some(v) = r.meldepunkt {
                o["meldepunkt"] = json!(v);
            }
            if let Some(v) = r.destination {
                o["destination"] = json!(v);
            }
            if let Some(v) = r.route {
                o["route"] = json!(v);
            }
            if let Some(v) = r.zuglaenge {
                o["zuglaenge"] = json!(v);
            }
            o
        })
        .collect();
    json!({ "telegrams": arr }).to_string()
}

// --- manual OTA (C handleUpdateUpload/Done) ---

/// Streams the raw request body into the inactive OTA slot. A bad/truncated
/// image fails `complete()` (or is aborted) and the device stays on the current
/// firmware — no brick.
fn stream_update<C: Connection>(req: &mut Request<C>) -> Result<(), String> {
    let mut ota = EspOta::new().map_err(|_| "ota busy".to_string())?;
    let mut update = ota
        .initiate_update()
        .map_err(|_| "update begin failed".to_string())?;
    let mut buf = [0u8; 1024];
    let mut total = 0usize;
    loop {
        let n = match req.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            Err(_) => {
                let _ = update.abort();
                return Err("recv failed".to_string());
            }
        };
        if update.write(&buf[..n]).is_err() {
            let _ = update.abort();
            return Err("write failed".to_string());
        }
        total += n;
    }
    if total == 0 {
        let _ = update.abort();
        return Err("empty upload".to_string());
    }
    update
        .complete()
        .map_err(|_| "update end failed".to_string())?;
    Ok(())
}
