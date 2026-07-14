//! Config/channel mutation, state serialization, and remote-command dispatch.
//!
//! This is the single place that knows what a settings change *means* — the
//! validation, the NVS write, and the targeted generation-bump fan-out that
//! wakes only the workers whose fields actually changed. Two front-ends drive
//! it:
//!
//!   * [`crate::web`] — the on-device HTTP UI (`POST /api/config`,
//!     `PUT /api/channels`), reachable only on the LAN the node sits on.
//!   * [`crate::uplink`] — commands pushed down the backend WebSocket, which is
//!     how a node installed on a rooftop with no physical access gets retuned.
//!
//! Both must behave identically, hence the shared entry points rather than a
//! second copy of the apply logic in the uplink.
//!
//! # Command protocol (backend -> node, over the ingest WebSocket)
//!
//! ```json
//! {"type":"command","id":42,"cmd":"config","config":{"activeChannel":2}}
//! {"type":"command","id":43,"cmd":"channel","channel":{"index":0,"freqHz":151512500,...}}
//! {"type":"command","id":44,"cmd":"channels","channels":[ ... ]}
//! {"type":"command","id":45,"cmd":"report"}
//! {"type":"command","id":46,"cmd":"update_now"}
//! {"type":"command","id":47,"cmd":"reboot"}
//! ```
//!
//! Every command is answered with
//! `{"type":"command_result","id":42,"ok":true}` (or `"ok":false,"error":"..."`),
//! sent *before* any reboot the command triggers so the backend can record the
//! outcome rather than time out.
//!
//! `channel` (single-slot patch) exists because the downlink is a small-buffer
//! WebSocket: a full 32-entry `channels` table is several kB and would arrive
//! fragmented, while the common remote operation — "retune slot 0" — fits in a
//! couple hundred bytes.

use std::sync::atomic::Ordering;

use esp_idf_svc::sys;
use serde_json::{json, Value};

use crate::config::{self, CHANNEL_NAME_LEN, CH_MODE_DATA, CH_MODE_VOICE, MAX_CHANNELS};
use crate::frames::uptime_ms;
use crate::{radio, wifi, SharedState};

/// What the caller must do after a command was applied.
pub struct Outcome {
    pub ok: bool,
    pub error: Option<String>,
    /// The change only takes effect after a restart (WiFi/port fields).
    pub reboot: bool,
}

impl Outcome {
    fn ok() -> Outcome {
        Outcome { ok: true, error: None, reboot: false }
    }
    fn err(msg: String) -> Outcome {
        Outcome { ok: false, error: Some(msg), reboot: false }
    }
}

/// Route one `{"type":"command",...}` document to the matching apply function.
///
/// `reboot`/`update_now` are *scheduled*, not performed here: this runs on the
/// uplink task, which must first put the `command_result` on the wire.
pub fn dispatch(s: &SharedState, doc: &Value) -> Outcome {
    let cmd = doc.get("cmd").and_then(|v| v.as_str()).unwrap_or("");
    match cmd {
        "config" => match doc.get("config") {
            Some(patch) => match apply_config(s, patch) {
                Ok(reboot) => Outcome { ok: true, error: None, reboot },
                Err(e) => Outcome::err(e),
            },
            None => Outcome::err("missing config".to_string()),
        },
        "channel" => match doc.get("channel") {
            Some(patch) => match apply_channel(s, patch) {
                Ok(()) => Outcome::ok(),
                Err(e) => Outcome::err(e),
            },
            None => Outcome::err("missing channel".to_string()),
        },
        // The full-table form takes the same `{"channels":[...]}` document the
        // HTTP PUT does, so the backend can mirror a table verbatim.
        "channels" => match apply_channels(s, doc) {
            Ok(()) => Outcome::ok(),
            Err(e) => Outcome::err(e),
        },
        // A no-op here: the uplink sends a fresh status report whenever it sees
        // an ok outcome for this command.
        "report" => Outcome::ok(),
        "update_now" => {
            // Skip the 0-6 h rollout jitter and re-check the manifest at once.
            s.ota.force.store(true, Ordering::Relaxed);
            s.ota.notify_wake();
            Outcome::ok()
        }
        "reboot" => Outcome { ok: true, error: None, reboot: true },
        "" => Outcome::err("missing cmd".to_string()),
        other => Outcome::err(format!("unknown cmd: {other}")),
    }
}

/// The node's full self-description, pushed to the backend on connect and every
/// [`crate::uplink::STATUS_EVERY`] thereafter. `channels` is included on every
/// report (a few hundred bytes) so the backend's channel picker never has to
/// ask for it separately.
pub fn status_report_json(s: &SharedState) -> String {
    // status/config/channels are already-serialized JSON documents; splice them
    // in as raw values rather than parse-and-rebuild.
    let status: Value = serde_json::from_str(&status_json(s)).unwrap_or(Value::Null);
    let cfg: Value = serde_json::from_str(&config_json(s)).unwrap_or(Value::Null);
    let channels: Value = serde_json::from_str(&channels_json(s))
        .ok()
        .and_then(|v: Value| v.get("channels").cloned())
        .unwrap_or(Value::Null);
    json!({
        "type": "status",
        "build": config::FIRMWARE_BUILD,
        "version": config::FIRMWARE_VERSION,
        "status": status,
        "config": cfg,
        "channels": channels,
    })
    .to_string()
}

// --- state serialization (the JSON contract the C webui.h defined) ---

pub fn name_str(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

pub fn status_json(s: &SharedState) -> String {
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
    // Live RF level in raw SA818 units (0-255, not dBm); absent when the module
    // has no RSSI? command (frames::RfRssi).
    if let Some(v) = s.frames.rssi.last() {
        radio["rssi"] = json!(v);
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

pub fn config_json(s: &SharedState) -> String {
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

pub fn channels_json(s: &SharedState) -> String {
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
                "index": i,
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
pub fn apply_config(s: &SharedState, doc: &Value) -> Result<bool, String> {
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

/// Read one channel object into an aligned `Channel`. Missing fields fall back
/// to `base`, so a patch can carry only the fields it means to change.
fn channel_from_json(o: &Value, base: config::Channel) -> config::Channel {
    let mut c = config::Channel::ZERO;
    c.number = o
        .get("number")
        .and_then(|v| v.as_u64())
        .map(|v| v as u8)
        .unwrap_or(base.number);
    match o.get("name").and_then(|v| v.as_str()) {
        Some(n) => set_channel_name(&mut c.name, n),
        None => c.name = base.name,
    }
    c.freq_hz = o
        .get("freqHz")
        .and_then(|v| v.as_u64())
        .map(|v| v as u32)
        .unwrap_or(base.freq_hz);
    c.bandwidth = o
        .get("bandwidth")
        .and_then(|v| v.as_u64())
        .map(|v| if v != 0 { 1 } else { 0 })
        .unwrap_or(base.bandwidth);
    c.ch_mode = o
        .get("chMode")
        .and_then(|v| v.as_u64())
        .map(|v| if v != 0 { CH_MODE_DATA } else { CH_MODE_VOICE })
        .unwrap_or(base.ch_mode);
    c.data_proto = o
        .get("dataProto")
        .and_then(|v| v.as_u64())
        .map(|v| (v as u8).min(2))
        .unwrap_or(base.data_proto);
    c.used = 1;
    c
}

/// Patch a single channel slot: `{"index":0,"freqHz":151512500,...}`. Unset
/// fields keep their current value, so the backend can retune a slot without
/// having to round-trip the whole table. `{"index":N,"used":false}` clears it.
///
/// Preferred over [`apply_channels`] on the WebSocket downlink — a full table is
/// several kB and would arrive fragmented.
pub fn apply_channel(s: &SharedState, doc: &Value) -> Result<(), String> {
    let idx = doc
        .get("index")
        .and_then(|v| v.as_u64())
        .ok_or_else(|| "channel needs an index".to_string())? as usize;
    if idx >= MAX_CHANNELS {
        return Err("index out of range".to_string());
    }

    // Clearing a slot: no frequency to validate.
    if doc.get("used").and_then(|v| v.as_bool()) == Some(false) {
        {
            let mut ch = s.app.channels.write().unwrap();
            ch.ch[idx] = config::Channel::ZERO;
        }
        clear_active_if_unused(s, idx);
        s.app.channels_changed();
        return Ok(());
    }

    let base = {
        let ch = s.app.channels.read().unwrap();
        ch.ch[idx]
    };
    let c = channel_from_json(doc, base);
    if !config::freq_in_module_range(&s.app.hw, c.freq_hz) {
        return Err("frequency outside module range".to_string());
    }
    {
        let mut ch = s.app.channels.write().unwrap();
        ch.ch[idx] = c;
    }
    s.app.channels_changed();
    Ok(())
}

/// Fall back to the VFO when the slot the radio is tuned to just went away.
fn clear_active_if_unused(s: &SharedState, idx: usize) {
    let mut reset = false;
    {
        let mut c = s.app.config.write().unwrap();
        if c.active_channel >= 0 && c.active_channel as usize == idx {
            c.active_channel = -1;
            reset = true;
        }
    }
    if reset {
        s.app.save_config();
    }
}

pub fn apply_channels(s: &SharedState, doc: &Value) -> Result<(), String> {
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
            ch.ch[i] = channel_from_json(o, config::Channel::ZERO);
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
        s.app.save_config();
    }
    // Persist the channel table + bump radio/decoder generations.
    s.app.channels_changed();
    Ok(())
}
