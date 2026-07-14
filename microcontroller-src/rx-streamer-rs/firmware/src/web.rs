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

use std::sync::atomic::Ordering;
use std::time::Duration;

use esp_idf_svc::http::server::{Configuration, Connection, EspHttpServer, Request};
use esp_idf_svc::http::Method;
use esp_idf_svc::ota::EspOta;
use esp_idf_svc::sys;
use serde_json::{json, Value};

use crate::config::{self, CHANNEL_NAME_LEN, CH_MODE_DATA, CH_MODE_VOICE, MAX_CHANNELS};
use crate::frames::uptime_ms;
use crate::{radio, wifi, SharedState};

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
        resp_json(req, 200, &status_json(&s))
    })?;

    let s = shared.clone();
    server.fn_handler("/api/telegrams", Method::Get, move |req| -> anyhow::Result<()> {
        resp_json(req, 200, &telegrams_json(&s))
    })?;

    let s = shared.clone();
    server.fn_handler("/api/config", Method::Get, move |req| -> anyhow::Result<()> {
        resp_json(req, 200, &config_json(&s))
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
        match apply_config(&s, &doc) {
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
        resp_json(req, 200, &channels_json(&s))
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
        match apply_channels(&s, &doc) {
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

fn name_str(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

fn status_json(s: &SharedState) -> String {
    let heap = unsafe { sys::esp_get_free_heap_size() };
    let heap_max = unsafe {
        sys::heap_caps_get_largest_free_block((sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT) as u32)
    };
    let connected = s.wifi_connected.load(Ordering::Relaxed);

    let cfg = s.app.config.read().unwrap();
    let ch = s.app.channels.read().unwrap();
    let (freq_hz, bandwidth) = config::active_tuning(&cfg, &ch);
    let active = cfg.active_channel;

    let mut radio = json!({
        "moduleFound": radio::module_found(),
        "freqHz": freq_hz,
        "bandwidth": bandwidth,
        "activeChannel": active,
        "squelchOpen": s.squelch_open.load(Ordering::Relaxed),
    });
    if active >= 0 && (active as usize) < MAX_CHANNELS && ch.ch[active as usize].used != 0 {
        radio["channelName"] = json!(name_str(&ch.ch[active as usize].name));
        radio["channelMode"] = json!(ch.ch[active as usize].ch_mode);
    }

    let wifi = json!({
        "ssid": if connected { cfg.ssid.clone() } else { String::new() },
        "rssi": if connected { wifi::sta_rssi() } else { 0 },
        "ip": wifi::ip_string(connected),
    });

    let stream = json!({
        "port": cfg.stream_port,
        "clients": s.stream_stats.clients.load(Ordering::Relaxed),
        "overruns": s.stream_stats.overruns.load(Ordering::Relaxed),
        "bytesOut": s.stream_stats.bytes_out.load(Ordering::Relaxed),
    });

    let mut decoder = json!({
        "proto": s.decoder.requested_proto.load(Ordering::Relaxed),
        "bursts": s.frames.stats.bursts.load(Ordering::Relaxed),
        "frames": s.frames.stats.frames.load(Ordering::Relaxed),
        "feedDrops": s.decoder.feed_drops.load(Ordering::Relaxed),
    });
    {
        let last_label = s.frames.stats.last_label.lock().unwrap();
        if !last_label.is_empty() {
            decoder["lastLabel"] = json!(*last_label);
            let last_ms = s.frames.stats.last_ms.load(Ordering::Relaxed);
            decoder["lastAgeS"] = json!(uptime_ms().wrapping_sub(last_ms) / 1000);
        }
    }

    let mode = s
        .uplink
        .mode
        .lock()
        .map(|m| m.clone())
        .unwrap_or_else(|_| "off".to_string());
    let mut uplink = json!({
        "mode": mode,
        "connected": s.uplink.connected.load(Ordering::Relaxed),
        "wsSocket": s.uplink.ws_socket.load(Ordering::Relaxed),
        "helloAcked": s.uplink.hello_acked.load(Ordering::Relaxed),
        "sent": s.frames.stats.sent.load(Ordering::Relaxed),
        "accepted": s.frames.stats.accepted.load(Ordering::Relaxed),
        "dropped": s.frames.stats.dropped.load(Ordering::Relaxed),
        "queued": s.frames.queued(),
    });
    {
        let e = s.uplink.last_error.lock().unwrap();
        if !e.is_empty() {
            uplink["lastError"] = json!(*e);
        }
    }

    let last_check = s.ota.last_check_ms.load(Ordering::Relaxed);
    let mut update = json!({
        "enabled": cfg.auto_update_enabled,
        "currentBuild": s.ota.current_build.load(Ordering::Relaxed),
        "lastCheckAgeS": if last_check == 0 { -1i64 } else { (uptime_ms().wrapping_sub(last_check) / 1000) as i64 },
    });
    let pending = s.ota.pending_build.load(Ordering::Relaxed);
    if pending != 0 {
        update["pendingBuild"] = json!(pending);
        let apply_at = s.ota.apply_at_ms.load(Ordering::Relaxed);
        let remain = apply_at.wrapping_sub(uptime_ms()) as i32;
        update["applyInS"] = json!(if remain > 0 { remain / 1000 } else { 0 });
    }
    {
        let e = s.ota.last_error.lock().unwrap();
        if !e.is_empty() {
            update["lastError"] = json!(*e);
        }
    }

    json!({
        "version": config::FIRMWARE_VERSION,
        "uptime": uptime_ms() / 1000,
        "heap": heap,
        "heapMaxAlloc": heap_max,
        "wifi": wifi,
        "radio": radio,
        "stream": stream,
        "decoder": decoder,
        "uplink": uplink,
        "update": update,
    })
    .to_string()
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

fn config_json(s: &SharedState) -> String {
    let c = s.app.config.read().unwrap();
    json!({
        "ssid": c.ssid,
        "streamPort": c.stream_port,
        "volume": c.volume,
        "squelch": c.squelch,
        "activeChannel": c.active_channel,
        "vfoFreqHz": c.vfo_freq_hz,
        "vfoBandwidth": c.vfo_bandwidth,
        "muteWhenClosed": c.mute_when_closed,
        "vfoDataProto": c.vfo_data_proto,
        "nodeName": c.node_name,
        "nodeLat": c.node_lat as f64,
        "nodeLon": c.node_lon as f64,
        "uplinkUrl": c.uplink_url,
        "hasUplinkToken": !c.uplink_token.is_empty(),
        "hasAdminPass": !c.admin_pass.is_empty(),
        "autoUpdateEnabled": c.auto_update_enabled,
        "updateUrl": c.update_url,
        "updateCheckIntervalMin": c.update_check_interval_min,
    })
    .to_string()
}

fn channels_json(s: &SharedState) -> String {
    let ch = s.app.channels.read().unwrap();
    let arr: Vec<Value> = (0..MAX_CHANNELS)
        .filter(|&i| ch.ch[i].used != 0)
        .map(|i| {
            let c = ch.ch[i];
            // Copy packed fields into aligned locals — `json!` borrows each
            // value, and `&c.freq_hz` (a `repr(packed)` field) would be UB.
            let (number, freq_hz) = (c.number, c.freq_hz);
            let (bandwidth, ch_mode, data_proto) = (c.bandwidth, c.ch_mode, c.data_proto);
            let name = name_str(&c.name);
            json!({
                "number": number,
                "name": name,
                "freqHz": freq_hz,
                "bandwidth": bandwidth,
                "chMode": ch_mode,
                "dataProto": data_proto,
            })
        })
        .collect();
    json!({ "channels": arr }).to_string()
}

// --- mutation (C handlePostConfig / handlePutChannels) ---

/// Applies a config patch. Returns `Ok(needReboot)` or `Err(msg)` for a 400.
/// Validation (freq range / channel existence) runs before anything is written,
/// so a rejected request never leaves a partial change (a minor tightening of
/// the C, which mutated in place then early-returned).
fn apply_config(s: &SharedState, doc: &Value) -> Result<bool, String> {
    let hw = s.app.hw;

    if let Some(f) = doc.get("vfoFreqHz").and_then(|v| v.as_u64()) {
        if !config::freq_in_module_range(&hw, f as u32) {
            return Err("frequency outside module range".to_string());
        }
    }
    if let Some(a) = doc.get("activeChannel").and_then(|v| v.as_i64()) {
        let ch = s.app.channels.read().unwrap();
        if a >= MAX_CHANNELS as i64 || (a >= 0 && ch.ch[a as usize].used == 0) {
            return Err("no such channel".to_string());
        }
    }

    let mut reboot = false;
    // Targeted reconfigure fan-out (webui.h): tuning fields re-apply the radio
    // and decoder; only actual uplink/update changes touch those workers, so a
    // volume tweak can't tear down the live WebSocket.
    let mut tuning_changed = false;
    let mut uplink_changed = false;
    let mut ota_changed = false;
    {
        let mut c = s.app.config.write().unwrap();
        if let Some(v) = doc.get("ssid").and_then(|v| v.as_str()) {
            if v != c.ssid {
                c.ssid = v.to_string();
                reboot = true;
            }
        }
        if let Some(v) = doc.get("pass").and_then(|v| v.as_str()) {
            c.pass = v.to_string();
            reboot = true;
        }
        if let Some(v) = doc.get("adminPass").and_then(|v| v.as_str()) {
            c.admin_pass = v.to_string();
        }
        if let Some(v) = doc.get("streamPort").and_then(|v| v.as_u64()) {
            let p = v as u16;
            if p != c.stream_port {
                c.stream_port = p;
                reboot = true;
            }
        }
        if let Some(v) = doc.get("volume").and_then(|v| v.as_i64()) {
            let v = v.clamp(1, 8) as u8;
            tuning_changed |= v != c.volume;
            c.volume = v;
        }
        if let Some(v) = doc.get("squelch").and_then(|v| v.as_i64()) {
            let v = v.clamp(0, 8) as u8;
            tuning_changed |= v != c.squelch;
            c.squelch = v;
        }
        if let Some(v) = doc.get("muteWhenClosed").and_then(|v| v.as_bool()) {
            c.mute_when_closed = v; // audio pump reads this live, no bump needed
        }
        if let Some(v) = doc.get("vfoFreqHz").and_then(|v| v.as_u64()) {
            tuning_changed |= v as u32 != c.vfo_freq_hz;
            c.vfo_freq_hz = v as u32;
        }
        if let Some(v) = doc.get("vfoBandwidth").and_then(|v| v.as_i64()) {
            let v = if v != 0 { 1 } else { 0 };
            tuning_changed |= v != c.vfo_bandwidth;
            c.vfo_bandwidth = v;
        }
        if let Some(v) = doc.get("activeChannel").and_then(|v| v.as_i64()) {
            let v = if v < 0 { -1 } else { v as i8 };
            tuning_changed |= v != c.active_channel;
            c.active_channel = v;
        }
        if let Some(v) = doc.get("vfoDataProto").and_then(|v| v.as_i64()) {
            let v = v.clamp(0, 2) as u8;
            tuning_changed |= v != c.vfo_data_proto;
            c.vfo_data_proto = v;
        }
        if let Some(v) = doc.get("nodeName").and_then(|v| v.as_str()) {
            c.node_name = v.to_string();
        }
        if let Some(v) = doc.get("nodeLat").and_then(|v| v.as_f64()) {
            c.node_lat = v as f32;
        }
        if let Some(v) = doc.get("nodeLon").and_then(|v| v.as_f64()) {
            c.node_lon = v as f32;
        }
        if let Some(v) = doc.get("uplinkUrl").and_then(|v| v.as_str()) {
            uplink_changed |= v != c.uplink_url;
            c.uplink_url = v.to_string();
        }
        if let Some(v) = doc.get("uplinkToken").and_then(|v| v.as_str()) {
            uplink_changed |= v != c.uplink_token;
            c.uplink_token = v.to_string();
        }
        if let Some(v) = doc.get("autoUpdateEnabled").and_then(|v| v.as_bool()) {
            ota_changed |= v != c.auto_update_enabled;
            c.auto_update_enabled = v;
        }
        if let Some(v) = doc.get("updateUrl").and_then(|v| v.as_str()) {
            ota_changed |= v != c.update_url;
            c.update_url = v.to_string();
        }
        if let Some(v) = doc.get("updateCheckIntervalMin").and_then(|v| v.as_u64()) {
            let v = (v as u32).max(5); // avoid hammering the endpoint
            ota_changed |= v != c.update_check_interval_min;
            c.update_check_interval_min = v;
        }
    }

    // Persist, then bump only the workers whose fields changed (the webui.h
    // uplinkChanged/updateChanged gating).
    s.app.save_config();
    if tuning_changed {
        s.app.bump_radio();
        s.app.bump_decoder();
    }
    if uplink_changed {
        s.app.bump_uplink();
    }
    if ota_changed {
        s.app.bump_ota();
        s.ota.notify_wake();
    }
    Ok(reboot)
}

fn set_channel_name(dst: &mut [u8; CHANNEL_NAME_LEN + 1], s: &str) {
    *dst = [0; CHANNEL_NAME_LEN + 1];
    let b = s.as_bytes();
    let n = b.len().min(CHANNEL_NAME_LEN);
    dst[..n].copy_from_slice(&b[..n]);
}

fn apply_channels(s: &SharedState, doc: &Value) -> Result<(), String> {
    let arr = doc
        .get("channels")
        .and_then(|v| v.as_array())
        .ok_or_else(|| "channels must be an array of at most 32".to_string())?;
    if arr.len() > MAX_CHANNELS {
        return Err("channels must be an array of at most 32".to_string());
    }
    let hw = s.app.hw;
    for o in arr {
        let f = o.get("freqHz").and_then(|v| v.as_u64()).unwrap_or(0) as u32;
        if !config::freq_in_module_range(&hw, f) {
            return Err("frequency outside module range".to_string());
        }
    }

    let count = arr.len();
    {
        let mut ch = s.app.channels.write().unwrap();
        ch.reset();
        for (i, o) in arr.iter().enumerate() {
            // Build an aligned local, then copy it in — `Channel` is
            // `repr(C, packed)`, so a `&mut ch.ch[i]` field reference is UB.
            let mut c = config::Channel::ZERO;
            c.number = o.get("number").and_then(|v| v.as_u64()).unwrap_or(0) as u8;
            set_channel_name(&mut c.name, o.get("name").and_then(|v| v.as_str()).unwrap_or(""));
            c.freq_hz = o.get("freqHz").and_then(|v| v.as_u64()).unwrap_or(0) as u32;
            c.bandwidth = if o.get("bandwidth").and_then(|v| v.as_u64()).unwrap_or(0) != 0 {
                1
            } else {
                0
            };
            c.ch_mode = if o.get("chMode").and_then(|v| v.as_u64()).unwrap_or(0) != 0 {
                CH_MODE_DATA
            } else {
                CH_MODE_VOICE
            };
            c.data_proto = (o.get("dataProto").and_then(|v| v.as_u64()).unwrap_or(0) as u8).min(2);
            c.used = 1;
            ch.ch[i] = c;
        }
    }

    // The active channel may no longer exist; reset it to VFO and persist config.
    let mut active_reset = false;
    {
        let mut c = s.app.config.write().unwrap();
        if c.active_channel >= 0 && (c.active_channel as usize) >= count {
            c.active_channel = -1;
            active_reset = true;
        }
    }
    if active_reset {
        if let Ok(c) = s.app.config.read() {
            let _ = config::save(&s.app.nvs, &c);
        }
    }
    // Persist the channel table + bump radio/decoder generations.
    s.app.channels_changed();
    Ok(())
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
