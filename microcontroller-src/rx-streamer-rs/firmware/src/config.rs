//! Persistent configuration, byte-compatible with the C `config.h`.
//!
//! Everything is stored in the NVS namespace `rxstream` using the exact same
//! keys, value types and blob layouts the Arduino `Preferences` firmware wrote,
//! so a board can be reflashed from C to Rust (or back) without losing its
//! settings or channel table.
//!
//! Type mapping (Arduino `Preferences` -> NVS primitive), verified against
//! arduino-esp32 `Preferences.cpp`:
//!   * `putString`  -> NVS string
//!   * `putUShort`  -> NVS u16
//!   * `putUInt`    -> NVS u32
//!   * `putUChar`   -> NVS u8
//!   * `putChar`    -> NVS i8
//!   * `putBool`    -> NVS u8 (0/1)                     (Preferences::putBool calls putUChar)
//!   * `putFloat`   -> NVS blob of 4 bytes (LE f32)     (Preferences::putFloat calls putBytes)
//!   * `putBytes`   -> NVS blob                          (channel table)
//!
//! Runtime sharing: [`AppState`] wraps the live [`Config`] and [`ChannelTable`]
//! in `RwLock`s and carries a set of `AtomicU32` generation counters. A config
//! change bumps the relevant counter(s); each worker task caches the last
//! generation it applied and re-reads config when it differs — the same
//! hot-reload pattern the C firmware spreads across `cfgGen`/`upCfgGen`.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::RwLock;

use esp_idf_svc::nvs::{EspNvs, EspNvsPartition, NvsDefault};
use esp_idf_svc::sys::{self, EspError};

use crate::board::HwConfig;

pub const FIRMWARE_VERSION: &str = "0.2.0";
/// Monotonic OTA build number (independent of the human-readable
/// [`FIRMWARE_VERSION`]). The auto-updater flashes a manifest build only when
/// it is strictly greater than this. Starts at 100 per the migration plan so
/// it stays ahead of the C firmware's build counter.
pub const FIRMWARE_BUILD: u32 = 101;

// Stream format constants (config.h).
pub const CAPTURE_SAMPLE_RATE: u32 = 48000;
pub const STREAM_SAMPLE_RATE: u32 = 16000;
pub const DECIMATION_RATIO: usize = 3;
pub const FRAME_SAMPLES_48K: usize = 720;
pub const FRAME_SAMPLES_16K: usize = FRAME_SAMPLES_48K / DECIMATION_RATIO;

pub const MAX_CHANNELS: usize = 32;
pub const CHANNEL_NAME_LEN: usize = 16;

// chMode: metadata only (radio configured identically either way).
pub const CH_MODE_VOICE: u8 = 0;
pub const CH_MODE_DATA: u8 = 1;

// dataProto: which telegram decoder runs while a data channel is active.
pub const PROTO_NONE: u8 = 0;
pub const PROTO_FFSK_VDV: u8 = 1;
pub const PROTO_NEMO_LIO: u8 = 2;

pub const CHANNEL_TABLE_MAGIC: u16 = 0x4B56; // 'KV'
pub const CHANNEL_TABLE_VERSION: u8 = 2;

const NVS_NAMESPACE: &str = "rxstream";

// --- Channel table, v2 (current) and v1 (migration source) ---

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Channel {
    pub number: u8,
    pub name: [u8; CHANNEL_NAME_LEN + 1],
    pub freq_hz: u32,
    pub bandwidth: u8,
    pub ch_mode: u8,
    pub data_proto: u8,
    pub used: u8,
}

impl Channel {
    pub const ZERO: Channel = Channel {
        number: 0,
        name: [0; CHANNEL_NAME_LEN + 1],
        freq_hz: 0,
        bandwidth: 0,
        ch_mode: 0,
        data_proto: 0,
        used: 0,
    };
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct ChannelTable {
    pub magic: u16,
    pub version: u8,
    pub reserved: u8,
    pub ch: [Channel; MAX_CHANNELS],
}

// v1 layout (before dataProto). Kept only so stored tables migrate instead of
// being wiped on upgrade.
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct ChannelV1 {
    number: u8,
    name: [u8; CHANNEL_NAME_LEN + 1],
    freq_hz: u32,
    bandwidth: u8,
    ch_mode: u8,
    used: u8,
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct ChannelTableV1 {
    magic: u16,
    version: u8,
    reserved: u8,
    ch: [ChannelV1; MAX_CHANNELS],
}

// Compile-time layout guards — mirror the C static_asserts exactly.
const _: () = assert!(core::mem::size_of::<Channel>() == 26);
const _: () = assert!(core::mem::size_of::<ChannelV1>() == 25);
const _: () = assert!(core::mem::size_of::<ChannelTable>() == 4 + 26 * MAX_CHANNELS);
const _: () = assert!(core::mem::size_of::<ChannelTableV1>() == 4 + 25 * MAX_CHANNELS);
const _: () =
    assert!(core::mem::size_of::<ChannelTable>() != core::mem::size_of::<ChannelTableV1>());

impl ChannelTable {
    pub fn reset(&mut self) {
        *self = ChannelTable {
            magic: CHANNEL_TABLE_MAGIC,
            version: CHANNEL_TABLE_VERSION,
            reserved: 0,
            ch: [Channel::ZERO; MAX_CHANNELS],
        };
    }

    fn empty() -> ChannelTable {
        ChannelTable {
            magic: CHANNEL_TABLE_MAGIC,
            version: CHANNEL_TABLE_VERSION,
            reserved: 0,
            ch: [Channel::ZERO; MAX_CHANNELS],
        }
    }

    /// Byte view for NVS blob writes. Layout is `repr(C, packed)` POD, so the
    /// bytes are identical to the C `putBytes(&channels, sizeof(channels))`.
    fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self as *const _ as *const u8,
                core::mem::size_of::<ChannelTable>(),
            )
        }
    }

    fn from_bytes(b: &[u8]) -> ChannelTable {
        let mut t = ChannelTable::empty();
        let n = core::mem::size_of::<ChannelTable>();
        assert!(b.len() >= n);
        unsafe {
            core::ptr::copy_nonoverlapping(b.as_ptr(), &mut t as *mut _ as *mut u8, n);
        }
        t
    }
}

fn v1_from_bytes(b: &[u8]) -> ChannelTableV1 {
    let n = core::mem::size_of::<ChannelTableV1>();
    assert!(b.len() >= n);
    let mut t = ChannelTableV1 {
        magic: 0,
        version: 0,
        reserved: 0,
        ch: [ChannelV1 {
            number: 0,
            name: [0; CHANNEL_NAME_LEN + 1],
            freq_hz: 0,
            bandwidth: 0,
            ch_mode: 0,
            used: 0,
        }; MAX_CHANNELS],
    };
    unsafe {
        core::ptr::copy_nonoverlapping(b.as_ptr(), &mut t as *mut _ as *mut u8, n);
    }
    t
}

fn set_name(dst: &mut [u8; CHANNEL_NAME_LEN + 1], s: &str) {
    *dst = [0; CHANNEL_NAME_LEN + 1];
    let bytes = s.as_bytes();
    let n = bytes.len().min(CHANNEL_NAME_LEN); // leave room for the NUL, like snprintf
    dst[..n].copy_from_slice(&bytes[..n]);
}

fn add_default_channel(table: &mut ChannelTable, idx: usize, freq_hz: u32, prefix: &str) {
    let mut c = Channel::ZERO;
    c.number = (idx + 1) as u8;
    set_name(&mut c.name, &format!("{} {}", prefix, idx + 1));
    c.freq_hz = freq_hz;
    c.bandwidth = 0; // both band plans use 12.5 kHz
    c.ch_mode = CH_MODE_VOICE;
    c.data_proto = PROTO_NONE;
    c.used = 1;
    table.ch[idx] = c;
}

/// First-boot channel table: the license-free band plan matching the module.
/// UHF -> PMR446 (16 channels); VHF -> German Freenet (6 channels). Identical
/// to C `populateDefaultChannels()`.
fn populate_default_channels(table: &mut ChannelTable, hw: &HwConfig) {
    use crate::board::RfModuleType;
    if hw.rf_module_type == RfModuleType::Sa818Uhf {
        for i in 0..16 {
            add_default_channel(table, i, 446_006_250 + (i as u32) * 12_500, "PMR");
        }
    } else {
        const FREENET_HZ: [u32; 6] = [
            149_025_000,
            149_037_500,
            149_050_000,
            149_087_500,
            149_100_000,
            149_112_500,
        ];
        for (i, &hz) in FREENET_HZ.iter().enumerate() {
            add_default_channel(table, i, hz, "Freenet");
        }
    }
}

// --- Runtime configuration (ported from C `StreamerConfig`) ---

#[derive(Debug, Clone)]
pub struct Config {
    pub ssid: String,
    pub pass: String,
    /// If non-empty, HTTP basic auth (user "admin") guards mutating endpoints.
    pub admin_pass: String,
    pub stream_port: u16,
    pub volume: u8,
    pub squelch: u8,
    /// Index into the channel table, -1 = manual VFO.
    pub active_channel: i8,
    pub vfo_freq_hz: u32,
    pub vfo_bandwidth: u8,
    pub mute_when_closed: bool,
    pub vfo_data_proto: u8,
    pub node_name: String,
    pub node_lat: f32,
    pub node_lon: f32,
    pub uplink_url: String,
    pub uplink_token: String,
    pub auto_update_enabled: bool,
    pub update_url: String,
    pub update_check_interval_min: u32,
}

impl Config {
    /// Defaults matching C `loadConfig()` fall-backs. `hw` supplies the volume
    /// default (C uses `hw.volume`).
    pub fn defaults(hw: &HwConfig) -> Config {
        Config {
            ssid: String::new(),
            pass: String::new(),
            admin_pass: String::new(),
            stream_port: 8000,
            volume: hw.volume,
            squelch: 2,
            active_channel: -1,
            vfo_freq_hz: 146_520_000,
            vfo_bandwidth: 0,
            mute_when_closed: true,
            vfo_data_proto: PROTO_NONE,
            node_name: String::new(),
            node_lat: 0.0,
            node_lon: 0.0,
            uplink_url: String::new(),
            uplink_token: String::new(),
            auto_update_enabled: true,
            update_url: String::new(),
            update_check_interval_min: 60,
        }
    }
}

// --- NVS helpers (open a fresh handle per op, like C `begin`/`end`) ---

fn open_rw(nvs: &EspNvsPartition<NvsDefault>) -> Result<EspNvs<NvsDefault>, EspError> {
    EspNvs::new(nvs.clone(), NVS_NAMESPACE, true)
}
fn open_ro(nvs: &EspNvsPartition<NvsDefault>) -> Result<EspNvs<NvsDefault>, EspError> {
    // Read-only opens fail if the namespace was never written; fall back to a
    // read/write open so first boot still returns defaults.
    EspNvs::new(nvs.clone(), NVS_NAMESPACE, false)
        .or_else(|_| EspNvs::new(nvs.clone(), NVS_NAMESPACE, true))
}

fn get_string(store: &EspNvs<NvsDefault>, key: &str, default: &str) -> String {
    // Size the read from the stored length so a long value (NVS strings allow
    // ~4000 bytes, and save() happily writes them) round-trips instead of
    // silently reverting to the default. Errors are logged, not swallowed.
    let len = match store.str_len(key) {
        Ok(Some(l)) => l,
        Ok(None) => return default.to_string(),
        Err(e) => {
            log::error!("[config] str_len({key}) failed: {e:?}, using default");
            return default.to_string();
        }
    };
    let mut buf = vec![0u8; len + 1];
    match store.get_str(key, &mut buf) {
        Ok(Some(s)) => s.trim_end_matches('\0').to_string(),
        Ok(None) => default.to_string(),
        Err(e) => {
            log::error!("[config] get_str({key}) failed: {e:?}, using default");
            default.to_string()
        }
    }
}

fn get_float_blob(store: &EspNvs<NvsDefault>, key: &str, default: f32) -> f32 {
    // Arduino Preferences::putFloat stores a 4-byte blob (LE), not an NVS float.
    let mut buf = [0u8; 4];
    match store.get_blob(key, &mut buf) {
        Ok(Some(b)) if b.len() == 4 => f32::from_le_bytes([b[0], b[1], b[2], b[3]]),
        _ => default,
    }
}

/// Load the config and channel table from NVS, applying the same defaults and
/// the size-keyed v1->v2 channel-table migration as C `loadConfig()`.
pub fn load(nvs: &EspNvsPartition<NvsDefault>, hw: &HwConfig) -> (Config, ChannelTable) {
    let mut cfg = Config::defaults(hw);
    let mut channels = ChannelTable::empty();

    let Ok(store) = open_ro(nvs) else {
        channels.reset();
        populate_default_channels(&mut channels, hw);
        return (cfg, channels);
    };

    cfg.ssid = get_string(&store, "ssid", "");
    cfg.pass = get_string(&store, "pass", "");
    cfg.admin_pass = get_string(&store, "webpass", "");
    cfg.stream_port = store.get_u16("port").ok().flatten().unwrap_or(8000);
    cfg.volume = store.get_u8("volume").ok().flatten().unwrap_or(hw.volume);
    cfg.squelch = store.get_u8("squelch").ok().flatten().unwrap_or(2);
    cfg.active_channel = store.get_i8("active").ok().flatten().unwrap_or(-1);
    cfg.vfo_freq_hz = store
        .get_u32("vfoFreq")
        .ok()
        .flatten()
        .unwrap_or(146_520_000);
    cfg.vfo_bandwidth = store.get_u8("vfoBw").ok().flatten().unwrap_or(0);
    cfg.mute_when_closed = store.get_u8("muteSq").ok().flatten().unwrap_or(1) != 0;
    cfg.vfo_data_proto = store
        .get_u8("vfoProto")
        .ok()
        .flatten()
        .unwrap_or(PROTO_NONE);
    cfg.node_name = get_string(&store, "nodeName", "");
    cfg.node_lat = get_float_blob(&store, "nodeLat", 0.0);
    cfg.node_lon = get_float_blob(&store, "nodeLon", 0.0);
    cfg.uplink_url = get_string(&store, "upUrl", "");
    cfg.uplink_token = get_string(&store, "upTok", "");
    cfg.auto_update_enabled = store.get_u8("autoUpd").ok().flatten().unwrap_or(1) != 0;
    cfg.update_url = get_string(&store, "updUrl", "");
    cfg.update_check_interval_min = store.get_u32("updIvl").ok().flatten().unwrap_or(60);

    // Channel table: load the current layout, migrate a v1 blob (keyed on the
    // exact blob size), or fall back to the defaults. Never wipe on a mere
    // version bump.
    let mut migrated = false;
    // Buffer sized for the larger (v2) blob; get_blob returns the actual length.
    let mut blob = vec![0u8; core::mem::size_of::<ChannelTable>()];
    let blob_len = match store.get_blob("channels", &mut blob) {
        Ok(Some(b)) => b.len(),
        _ => 0,
    };

    if blob_len == core::mem::size_of::<ChannelTable>() {
        channels = ChannelTable::from_bytes(&blob);
        let magic = channels.magic;
        let version = channels.version;
        if magic != CHANNEL_TABLE_MAGIC || version != CHANNEL_TABLE_VERSION {
            channels.reset();
            populate_default_channels(&mut channels, hw);
        }
    } else if blob_len == core::mem::size_of::<ChannelTableV1>() {
        let old = v1_from_bytes(&blob);
        channels.reset();
        let old_magic = old.magic;
        let old_version = old.version;
        if old_magic == CHANNEL_TABLE_MAGIC && old_version == 1 {
            for i in 0..MAX_CHANNELS {
                let src = old.ch[i];
                let mut c = Channel::ZERO;
                c.number = src.number;
                c.name = src.name;
                c.freq_hz = src.freq_hz;
                c.bandwidth = src.bandwidth;
                c.ch_mode = src.ch_mode;
                c.data_proto = PROTO_NONE;
                c.used = src.used;
                channels.ch[i] = c;
            }
            migrated = true;
        } else {
            populate_default_channels(&mut channels, hw);
        }
    } else {
        channels.reset();
        populate_default_channels(&mut channels, hw);
    }

    drop(store);
    if migrated {
        let _ = save_channel_table(nvs, &channels);
        log::info!("[config] channel table migrated v1 -> v2");
    }

    (cfg, channels)
}

/// Persist the full config. Mirrors C `saveConfig()` key-for-key.
pub fn save(nvs: &EspNvsPartition<NvsDefault>, cfg: &Config) -> Result<(), EspError> {
    let mut store = open_rw(nvs)?;
    store.set_str("ssid", &cfg.ssid)?;
    store.set_str("pass", &cfg.pass)?;
    store.set_str("webpass", &cfg.admin_pass)?;
    store.set_u16("port", cfg.stream_port)?;
    store.set_u8("volume", cfg.volume)?;
    store.set_u8("squelch", cfg.squelch)?;
    store.set_i8("active", cfg.active_channel)?;
    store.set_u32("vfoFreq", cfg.vfo_freq_hz)?;
    store.set_u8("vfoBw", cfg.vfo_bandwidth)?;
    store.set_u8("muteSq", cfg.mute_when_closed as u8)?;
    store.set_u8("vfoProto", cfg.vfo_data_proto)?;
    store.set_str("nodeName", &cfg.node_name)?;
    store.set_blob("nodeLat", &cfg.node_lat.to_le_bytes())?;
    store.set_blob("nodeLon", &cfg.node_lon.to_le_bytes())?;
    store.set_str("upUrl", &cfg.uplink_url)?;
    store.set_str("upTok", &cfg.uplink_token)?;
    store.set_u8("autoUpd", cfg.auto_update_enabled as u8)?;
    store.set_str("updUrl", &cfg.update_url)?;
    store.set_u32("updIvl", cfg.update_check_interval_min)?;
    Ok(())
}

/// Persist just the channel-table blob. Mirrors C `saveChannelTable()`.
pub fn save_channel_table(
    nvs: &EspNvsPartition<NvsDefault>,
    channels: &ChannelTable,
) -> Result<(), EspError> {
    let mut store = open_rw(nvs)?;
    store.set_blob("channels", channels.as_bytes())?;
    Ok(())
}

/// Wipe every user setting in the `rxstream` namespace and reboot. Leaves the
/// `hwconfig` namespace (the board description) untouched. Mirrors C
/// `factoryReset()`.
pub fn factory_reset(_nvs: &EspNvsPartition<NvsDefault>) -> ! {
    // esp-idf-svc's EspNvs has no `clear`, so erase the namespace directly via
    // the C API (nvs_erase_all clears only this handle's namespace).
    // VERIFY(phase1): confirm EspNvs still lacks a namespace-clear helper.
    unsafe {
        let name = c"rxstream";
        let mut handle: sys::nvs_handle_t = 0;
        if sys::nvs_open(
            name.as_ptr(),
            sys::nvs_open_mode_t_NVS_READWRITE,
            &mut handle,
        ) == sys::ESP_OK
        {
            sys::nvs_erase_all(handle);
            sys::nvs_commit(handle);
            sys::nvs_close(handle);
        }
    }
    log::warn!("[config] factory reset, restarting");
    esp_idf_svc::hal::reset::restart();
}

// --- Derived queries (ported from config.h helpers) ---

/// The channel (or VFO) the radio should currently be tuned to.
pub fn active_tuning(cfg: &Config, channels: &ChannelTable) -> (u32, u8) {
    let idx = cfg.active_channel;
    if idx >= 0 && (idx as usize) < MAX_CHANNELS {
        let c = channels.ch[idx as usize];
        if c.used != 0 {
            return (c.freq_hz, c.bandwidth);
        }
    }
    (cfg.vfo_freq_hz, cfg.vfo_bandwidth)
}

/// Which telegram decoder should run right now (active data channel's protocol,
/// or the VFO setting when no channel is selected).
pub fn active_data_proto(cfg: &Config, channels: &ChannelTable) -> u8 {
    let idx = cfg.active_channel;
    if idx >= 0 && (idx as usize) < MAX_CHANNELS {
        let c = channels.ch[idx as usize];
        if c.used != 0 {
            return if c.ch_mode == CH_MODE_DATA {
                c.data_proto
            } else {
                PROTO_NONE
            };
        }
    }
    cfg.vfo_data_proto
}

pub fn freq_in_module_range(hw: &HwConfig, freq_hz: u32) -> bool {
    let mhz = freq_hz as f32 / 1e6;
    mhz >= hw.module_min_freq_mhz() && mhz <= hw.module_max_freq_mhz()
}

// --- Shared runtime state ---

/// Config-generation counters. A worker caches the last value it applied and
/// re-reads config when the counter changes (replaces the C `cfgGen`/`upCfgGen`
/// scheme). All four are bumped on a full config save; individual bumps are
/// available for targeted changes.
#[derive(Debug, Default)]
pub struct Generations {
    pub radio: AtomicU32,
    pub decoder: AtomicU32,
    pub uplink: AtomicU32,
    pub ota: AtomicU32,
}

/// Shared application state handed (as `Arc`) to every worker thread.
pub struct AppState {
    pub hw: HwConfig,
    pub config: RwLock<Config>,
    pub channels: RwLock<ChannelTable>,
    pub gen: Generations,
    pub nvs: EspNvsPartition<NvsDefault>,
}

impl AppState {
    pub fn new(
        hw: HwConfig,
        cfg: Config,
        channels: ChannelTable,
        nvs: EspNvsPartition<NvsDefault>,
    ) -> AppState {
        AppState {
            hw,
            config: RwLock::new(cfg),
            channels: RwLock::new(channels),
            gen: Generations {
                radio: AtomicU32::new(1),
                decoder: AtomicU32::new(1),
                uplink: AtomicU32::new(1),
                ota: AtomicU32::new(1),
            },
            nvs,
        }
    }

    pub fn bump_radio(&self) {
        self.gen.radio.fetch_add(1, Ordering::SeqCst);
    }
    pub fn bump_decoder(&self) {
        self.gen.decoder.fetch_add(1, Ordering::SeqCst);
    }
    pub fn bump_uplink(&self) {
        self.gen.uplink.fetch_add(1, Ordering::SeqCst);
    }
    pub fn bump_ota(&self) {
        self.gen.ota.fetch_add(1, Ordering::SeqCst);
    }

    /// Persist the current config to NVS. Generation bumps are the CALLER's
    /// job and must be targeted like the webui.h fan-out: C only called
    /// `uplinkReconfigure()` / `otaUpdateReconfigure()` when the respective
    /// fields actually changed — an unconditional uplink bump tears down the
    /// live WebSocket (multi-second TLS reconnect, ~40 kB contiguous heap) on
    /// every unrelated save.
    pub fn save_config(&self) {
        if let Ok(cfg) = self.config.read() {
            if let Err(e) = save(&self.nvs, &cfg) {
                log::error!("[config] save failed: {e:?}");
            }
        }
    }

    pub fn channels_changed(&self) {
        if let Ok(ch) = self.channels.read() {
            if let Err(e) = save_channel_table(&self.nvs, &ch) {
                log::error!("[config] channel save failed: {e:?}");
            }
        }
        self.bump_radio();
        self.bump_decoder();
    }
}
