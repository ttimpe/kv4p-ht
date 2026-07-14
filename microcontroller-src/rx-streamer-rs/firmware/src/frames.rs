//! CRC-checked telegram frames handed from the decoder task to the uplink
//! task, plus the web UI history ring and the shared decode/uplink counters.
//! Port of the C `frames.h`.
//!
//! The queue is bounded and drop-oldest: while the uplink is down it doubles
//! as the (small) retry buffer, and fresh telegrams win. The 50-entry history
//! ring is display-only (never persisted to flash).

use std::collections::VecDeque;
use std::fmt::Write as _;
use std::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};
use std::sync::{Condvar, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use esp_idf_svc::sys;

use crate::config::{PROTO_FFSK_VDV, PROTO_NEMO_LIO};
use crate::wifi;

/// ffsk telegrams <= 21 B, g2 GPS beacons 19-20 B (C `FRAME_RAW_MAX`).
pub const FRAME_RAW_MAX: usize = 32;
pub const FRAME_QUEUE_DEPTH: usize = 16;
pub const TELEGRAM_HISTORY_SIZE: usize = 50;

/// Milliseconds since boot (the C `millis()`).
pub fn uptime_ms() -> u32 {
    unsafe { (sys::esp_timer_get_time() / 1000) as u32 }
}

/// One decoded, CRC-valid telegram. Mirrors the C `FrameRecord`; the
/// `bool has*` + value pairs become `Option`s.
#[derive(Debug, Clone)]
pub struct FrameRecord {
    /// `PROTO_FFSK_VDV` or `PROTO_NEMO_LIO` -> wire tag "ffsk" / "g2".
    pub proto: u8,
    pub uptime_ms: u32,
    /// Unix seconds; 0 when wall-clock time is not synced yet.
    pub ts_unix: i64,
    pub raw_len: u8,
    /// Checkable frame incl. CRC/FCS bytes.
    pub raw: [u8; FRAME_RAW_MAX],
    /// Local status display only (<= 23 chars, like the C `char label[24]`).
    pub label: String,
    /// ffsk only.
    pub repaired_bits: u8,

    // R09 field-level decode (VDV R09/ffsk only; g2 frames leave these unset).
    // No single telegram type carries all of these — see decoder.rs.
    /// Linie: R09.14/16 and R09.0.7
    pub line: Option<u16>,
    /// Kurs: R09.14/16 and R09.0.7
    pub run: Option<u8>,
    /// R09.14/16 (confirmed) or R09.0.7 (experimental)
    pub meldepunkt: Option<u16>,
    /// Ziel: R09.16 only
    pub destination: Option<u16>,
    /// R09.0.7 only, confirmed against timetable data
    pub route: Option<u16>,
    /// R09.0.7 only, hypothesis (see the C decoder_ffsk.h discussion)
    pub zuglaenge: Option<u8>,

    /// Peak RF signal level during this telegram's burst, raw SA818 units
    /// (0-255, **not** dBm — see [`RfRssi`]). `None` when the module does not
    /// answer `RSSI?`, or when no sample landed inside the burst.
    pub rssi: Option<u8>,
}

impl FrameRecord {
    pub fn new(proto: u8) -> FrameRecord {
        FrameRecord {
            proto,
            uptime_ms: 0,
            ts_unix: 0,
            raw_len: 0,
            raw: [0; FRAME_RAW_MAX],
            label: String::new(),
            repaired_bits: 0,
            line: None,
            run: None,
            meldepunkt: None,
            destination: None,
            route: None,
            zuglaenge: None,
            rssi: None,
        }
    }

    /// Store the checkable frame bytes (truncating at [`FRAME_RAW_MAX`],
    /// matching the C fixed buffer).
    pub fn set_raw(&mut self, bytes: &[u8]) {
        let n = bytes.len().min(FRAME_RAW_MAX);
        self.raw[..n].copy_from_slice(&bytes[..n]);
        self.raw_len = n as u8;
    }

    pub fn raw_bytes(&self) -> &[u8] {
        &self.raw[..self.raw_len as usize]
    }

    /// Wire protocol tag, as the C uplink/web mapped it.
    pub fn proto_str(&self) -> &'static str {
        match self.proto {
            PROTO_FFSK_VDV => "ffsk",
            PROTO_NEMO_LIO => "g2",
            _ => "?",
        }
    }
}

/// Log one CRC-valid telegram to the console, so reception is verifiable over
/// USB alone — no WiFi, no browser, no backend. The device has no other way to
/// show a decoded frame locally: the history ring is only readable over HTTP.
fn log_frame(r: &FrameRecord) {
    let mut hex = String::with_capacity(r.raw_len as usize * 2);
    for b in r.raw_bytes() {
        let _ = write!(hex, "{b:02X}");
    }

    // R09 fields, only the ones this telegram type actually carries.
    let mut fields = String::new();
    if let Some(v) = r.line {
        let _ = write!(fields, " line={v}");
    }
    if let Some(v) = r.run {
        let _ = write!(fields, " run={v}");
    }
    if let Some(v) = r.meldepunkt {
        let _ = write!(fields, " mp={v}");
    }
    if let Some(v) = r.destination {
        let _ = write!(fields, " dest={v}");
    }
    if let Some(v) = r.route {
        let _ = write!(fields, " route={v}");
    }
    if let Some(v) = r.zuglaenge {
        let _ = write!(fields, " zl={v}");
    }
    if let Some(v) = r.rssi {
        let _ = write!(fields, " rssi={v}");
    }

    log::info!(
        "[tg] {} {} rep={}{} ({})",
        r.proto_str(),
        hex,
        r.repaired_bits,
        fields,
        r.label
    );
}

/// RF signal level sampled from the SA818, shared between the poll task
/// (producer) and the decoder (consumer).
///
/// **Raw module units, 0-255 — not dBm.** The SA818 answers `RSSI?` with an
/// unscaled byte and documents no dBm mapping, so nothing here pretends to know
/// one: the value is only meaningful *relatively* (this telegram vs. that one,
/// this node vs. that node).
///
/// Peak-hold rather than "latest sample", because a telegram is short (~60 ms
/// for VDV R09) and the decoder only builds the record once the burst has
/// *ended* — an instantaneous read at that moment tends to land in the silence
/// after the transmission. [`Self::take_peak`] therefore returns the strongest
/// sample seen since the previous burst consumed it.
///
/// Both cells store `value + 1`, so 0 doubles as "no sample" and the whole thing
/// stays lock-free (a real 0 reading is a legitimate value, not an absence).
#[derive(Default)]
pub struct RfRssi {
    last: AtomicU16,
    peak: AtomicU16,
    /// Set once the module has proven it does not answer `RSSI?` (a genuine
    /// DRA818 has no such command). Polling stops; records keep `rssi: None`.
    pub unsupported: AtomicBool,
}

impl RfRssi {
    /// Record one sample (poll task).
    pub fn observe(&self, v: u8) {
        let enc = v as u16 + 1;
        self.last.store(enc, Ordering::Relaxed);
        self.peak.fetch_max(enc, Ordering::Relaxed);
    }

    /// Most recent sample, for live status display.
    pub fn last(&self) -> Option<u8> {
        match self.last.load(Ordering::Relaxed) {
            0 => None,
            enc => Some((enc - 1) as u8),
        }
    }

    /// Take (and clear) the peak since the last burst — call once per burst, so
    /// every record decoded out of that burst carries the same level.
    pub fn take_peak(&self) -> Option<u8> {
        match self.peak.swap(0, Ordering::Relaxed) {
            0 => None,
            enc => Some((enc - 1) as u8),
        }
    }
}

/// Shared decode/uplink counters (C `stBursts`..`stAccepted` + last label).
/// The decoder bumps `bursts`; `frames`/`dropped` are bumped here; the uplink
/// task (later module) bumps `sent`/`accepted` and `dropped` on send errors.
#[derive(Default)]
pub struct FrameStats {
    /// Decoder bursts examined.
    pub bursts: AtomicU32,
    /// CRC-valid frames decoded.
    pub frames: AtomicU32,
    /// Frames dropped (queue full / uplink error).
    pub dropped: AtomicU32,
    /// Frames handed to the transport.
    pub sent: AtomicU32,
    /// Frames the server acked as accepted.
    pub accepted: AtomicU32,
    /// Label of the most recent frame (C `stLastLabel`), for the status UI.
    pub last_label: Mutex<String>,
    /// `uptime_ms` of the most recent frame (C `stLastMs`).
    pub last_ms: AtomicU32,
}

/// Frame queue + history + stats, `Arc`-shared between the decoder (producer),
/// uplink (queue consumer) and web UI (history/stats reader).
pub struct Frames {
    queue: Mutex<VecDeque<FrameRecord>>,
    queue_cv: Condvar,
    history: Mutex<VecDeque<FrameRecord>>,
    pub stats: FrameStats,
    /// Lives here because `Frames` is the one object both the RSSI poll task and
    /// the decoder already hold — no extra plumbing needed to get a sample from
    /// the radio to the record it belongs to.
    pub rssi: RfRssi,
}

impl Frames {
    pub fn new() -> Frames {
        Frames {
            queue: Mutex::new(VecDeque::with_capacity(FRAME_QUEUE_DEPTH)),
            queue_cv: Condvar::new(),
            history: Mutex::new(VecDeque::with_capacity(TELEGRAM_HISTORY_SIZE)),
            stats: FrameStats::default(),
            rssi: RfRssi::default(),
        }
    }

    /// Called from the decoder task. Stamps time (unix time only once SNTP has
    /// synced) and drops the oldest queued frame when full so fresh telegrams
    /// win. Port of C `frameEnqueue`.
    pub fn enqueue(&self, mut r: FrameRecord) {
        r.uptime_ms = uptime_ms();
        r.ts_unix = if wifi::time_synced() {
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs() as i64)
                .unwrap_or(0)
        } else {
            0
        };

        log_frame(&r);

        {
            let mut q = self.queue.lock().unwrap();
            if q.len() >= FRAME_QUEUE_DEPTH {
                q.pop_front();
                self.stats.dropped.fetch_add(1, Ordering::Relaxed);
            }
            q.push_back(r.clone());
        }
        self.queue_cv.notify_one();

        self.stats.frames.fetch_add(1, Ordering::Relaxed);
        if let Ok(mut l) = self.stats.last_label.lock() {
            l.clear();
            l.push_str(&r.label);
        }
        self.stats.last_ms.store(r.uptime_ms, Ordering::Relaxed);

        let mut h = self.history.lock().unwrap();
        if h.len() >= TELEGRAM_HISTORY_SIZE {
            h.pop_front();
        }
        h.push_back(r);
    }

    /// Blocking pop with timeout, for the uplink task's per-frame path.
    pub fn pop_timeout(&self, timeout: Duration) -> Option<FrameRecord> {
        let q = self.queue.lock().unwrap();
        let (mut q, _) = self
            .queue_cv
            .wait_timeout_while(q, timeout, |q| q.is_empty())
            .unwrap();
        q.pop_front()
    }

    /// Drain everything currently queued (uplink batch path).
    pub fn drain(&self) -> Vec<FrameRecord> {
        self.queue.lock().unwrap().drain(..).collect()
    }

    pub fn queued(&self) -> usize {
        self.queue.lock().unwrap().len()
    }

    /// History snapshot, newest first (web UI `/api/telegrams`).
    pub fn history_snapshot(&self) -> Vec<FrameRecord> {
        self.history.lock().unwrap().iter().rev().cloned().collect()
    }
}

impl Default for Frames {
    fn default() -> Self {
        Frames::new()
    }
}
