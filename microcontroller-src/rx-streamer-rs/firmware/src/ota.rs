//! Automatic firmware over-the-air updater, port of the C `otaUpdate.h`.
//!
//! Devices are installed remotely with no physical/USB access, so this is the
//! only way most of them ever get a new build. On an interval the task pulls a
//! manifest from `cfg.update_url` (Bearer = the reused `uplink_token`); if it
//! advertises a `build` greater than [`FIRMWARE_BUILD`] the apply is scheduled
//! after a random 0-6 h jitter (fleet-wide rollout staggering), then the `.bin`
//! is streamed into the inactive OTA slot, its SHA-256 verified *before*
//! committing, and the device reboots. A bad/short download never bricks —
//! it just gets retried on a later interval.
//!
//! Runs on core 0, NOT watchdog-registered (network I/O may block), same as the
//! uplink/decoder/streamer tasks. The config-generation counter (`gen.ota`)
//! plus a condvar let a settings save wake the task immediately instead of
//! leaving it asleep for up to 30 s (C `xTaskNotifyGive`).

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Condvar, Mutex};
use std::time::Duration;

use esp_idf_svc::http::client::{Configuration as HttpConfig, EspHttpConnection};
use esp_idf_svc::http::Method;
use esp_idf_svc::ota::EspOta;
use esp_idf_svc::sys::{self, EspError};
use sha2::{Digest, Sha256};

use crate::config::FIRMWARE_BUILD;
use crate::frames::uptime_ms;
use crate::wifi;
use crate::SharedState;

/// OTA status surfaced to `/api/status` (C `ota*` globals). Also carries the
/// wake condvar used to interrupt the 30 s idle when settings change.
pub struct OtaStatus {
    pub last_error: Mutex<String>,
    /// `uptime_ms` of the last manifest check, 0 = never (C `otaLastCheckMs`).
    pub last_check_ms: AtomicU32,
    /// The build this firmware reports (C `otaCurrentBuild`).
    pub current_build: AtomicU32,
    /// Build pending a jittered apply, 0 = none (C `otaPendingBuild`).
    pub pending_build: AtomicU32,
    /// `uptime_ms` deadline for the jittered apply (C `otaApplyAtMs`).
    pub apply_at_ms: AtomicU32,
    /// Set by the backend's `update_now` command: check the manifest regardless
    /// of the interval and apply a newer build immediately, skipping the
    /// fleet-staggering jitter. Consumed (swapped false) by one task pass.
    pub force: AtomicBool,
    wake: Condvar,
    wake_flag: Mutex<bool>,
}

impl Default for OtaStatus {
    fn default() -> Self {
        OtaStatus {
            last_error: Mutex::new(String::new()),
            last_check_ms: AtomicU32::new(0),
            current_build: AtomicU32::new(FIRMWARE_BUILD),
            pending_build: AtomicU32::new(0),
            apply_at_ms: AtomicU32::new(0),
            force: AtomicBool::new(false),
            wake: Condvar::new(),
            wake_flag: Mutex::new(false),
        }
    }
}

impl OtaStatus {
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
    /// Wake the check task now (call after `gen.ota` is bumped). C
    /// `otaUpdateReconfigure`.
    pub fn notify_wake(&self) {
        if let Ok(mut f) = self.wake_flag.lock() {
            *f = true;
        }
        self.wake.notify_all();
    }
    /// Block up to `timeout`, returning early on `notify_wake`.
    fn wait(&self, timeout: Duration) {
        if let Ok(mut f) = self.wake_flag.lock() {
            if !*f {
                let (g, _) = self.wake.wait_timeout(f, timeout).unwrap();
                f = g;
            }
            *f = false;
        }
    }
}

/// Spawn the OTA check task (core 0, stack 12288, priority 1).
pub fn start(shared: SharedState) -> std::io::Result<()> {
    crate::rt::spawn(
        b"otaCheck\0",
        12288,
        1,
        esp_idf_svc::hal::cpu::Core::Core0,
        move || ota_task(shared),
    )
    .map(|_| ())
}

fn ota_task(shared: SharedState) {
    let status = shared.ota.clone();

    loop {
        let (auto, url, token, interval_min) = {
            let cfg = shared.app.config.read().unwrap();
            (
                cfg.auto_update_enabled,
                cfg.update_url.clone(),
                cfg.uplink_token.clone(),
                cfg.update_check_interval_min,
            )
        };

        // The backend's `update_now` command: check off-interval and apply
        // without the rollout jitter. It also overrides the auto-update opt-out
        // — it is an explicit per-node operator action, not the unattended
        // rollout that switch governs.
        let forced = status.force.swap(false, Ordering::Relaxed);

        let now = uptime_ms();
        let last = status.last_check_ms.load(Ordering::Relaxed);
        let due =
            forced || last == 0 || now.wrapping_sub(last) >= interval_min.saturating_mul(60_000);
        let ready = (auto || forced)
            && !url.is_empty()
            && shared.wifi_connected.load(Ordering::Relaxed)
            && wifi::time_synced();

        if ready && due {
            status.last_check_ms.store(now, Ordering::Relaxed);
            match fetch_manifest(&url, &token, &status) {
                Ok((build, version, bin_url, sha256)) => {
                    status.clear_err();
                    if build > FIRMWARE_BUILD {
                        if status.pending_build.load(Ordering::Relaxed) != build {
                            // First sighting: stagger an unattended fleet-wide
                            // rollout with 0-6 h of jitter. A forced update is
                            // one operator acting on one node — no stagger.
                            let jitter = if forced {
                                0
                            } else {
                                (unsafe { sys::esp_random() }) % (6 * 3600 * 1000)
                            };
                            status.pending_build.store(build, Ordering::Relaxed);
                            status
                                .apply_at_ms
                                .store(uptime_ms().wrapping_add(jitter), Ordering::Relaxed);
                            log::info!(
                                "[ota] build {build} (v{version}) available, applying in {} s",
                                jitter / 1000
                            );
                        } else if forced {
                            // Already pending behind a jitter deadline — bring it forward.
                            status.apply_at_ms.store(uptime_ms(), Ordering::Relaxed);
                        }

                        // Deadline check runs in the same pass as the scheduling
                        // above, so a zero jitter applies now instead of waiting
                        // for the next 30 s wakeup.
                        let apply_at = status.apply_at_ms.load(Ordering::Relaxed);
                        if (uptime_ms().wrapping_sub(apply_at) as i32) >= 0 {
                            log::info!("[ota] downloading build {build}");
                            shared.ota_in_progress.store(true, Ordering::Relaxed);
                            if download_and_flash(&bin_url, &token, &sha256, &shared, &status) {
                                log::info!("[ota] flashed, rebooting");
                                std::thread::sleep(Duration::from_millis(200));
                                esp_idf_svc::hal::reset::restart();
                            } else {
                                shared.ota_in_progress.store(false, Ordering::Relaxed);
                                log::warn!(
                                    "[ota] failed: {}",
                                    status.last_error.lock().map(|s| s.clone()).unwrap_or_default()
                                );
                            }
                        }
                    } else {
                        status.pending_build.store(0, Ordering::Relaxed); // no newer build
                    }
                }
                Err(()) => { /* last_error already set by fetch_manifest */ }
            }
        }

        status.wait(Duration::from_secs(30));
    }
}

/// GET the manifest and parse `{build,version?,url,sha256}` (C
/// `otaFetchManifest`). Returns `(build, version, bin_url, sha256_hex)`.
fn fetch_manifest(
    url: &str,
    token: &str,
    status: &OtaStatus,
) -> Result<(u32, String, String, String), ()> {
    let body = match http_get_string(url, token, 8192) {
        Ok(b) => b,
        Err(msg) => {
            status.set_err(msg);
            return Err(());
        }
    };
    let doc: serde_json::Value = match serde_json::from_str(&body) {
        Ok(v) => v,
        Err(_) => {
            status.set_err("manifest bad json");
            return Err(());
        }
    };
    let (Some(build), Some(bin_url), Some(sha256)) = (
        doc.get("build").and_then(|v| v.as_u64()),
        doc.get("url").and_then(|v| v.as_str()),
        doc.get("sha256").and_then(|v| v.as_str()),
    ) else {
        status.set_err("manifest bad json");
        return Err(());
    };
    let version = doc
        .get("version")
        .and_then(|v| v.as_str())
        .unwrap_or("")
        .to_string();
    Ok((
        build as u32,
        version,
        bin_url.to_string(),
        sha256.to_string(),
    ))
}

/// Minimal HTTP(S) GET returning the response body as a String (manifest only —
/// bounded by `cap` bytes). Bearer auth when `token` is non-empty.
fn http_get_string(url: &str, token: &str, cap: usize) -> Result<String, &'static str> {
    let mut conn = EspHttpConnection::new(&HttpConfig {
        timeout: Some(Duration::from_secs(8)),
        ..Default::default()
    })
    .map_err(|_| "manifest begin failed")?;

    let auth = format!("Bearer {token}");
    let headers: Vec<(&str, &str)> = if token.is_empty() {
        vec![]
    } else {
        vec![("Authorization", auth.as_str())]
    };

    conn.initiate_request(Method::Get, url, &headers)
        .map_err(|_| "manifest begin failed")?;
    conn.initiate_response().map_err(|_| "manifest begin failed")?;
    if conn.status() != 200 {
        return Err("manifest http error");
    }

    let mut out = String::new();
    let mut buf = [0u8; 256];
    while let Ok(n) = conn.read(&mut buf) {
        if n == 0 {
            break;
        }
        out.push_str(&String::from_utf8_lossy(&buf[..n]));
        if out.len() >= cap {
            break;
        }
    }
    Ok(out)
}

/// Streams `url` into the inactive OTA slot, verifying SHA-256 *before*
/// committing. Never reboots on failure (caller reboots on `true`). Port of C
/// `otaDownloadAndFlash`.
fn download_and_flash(
    url: &str,
    token: &str,
    sha256_hex: &str,
    shared: &SharedState,
    status: &OtaStatus,
) -> bool {
    let _ = shared; // ota_in_progress already set by the caller
    let mut conn = match EspHttpConnection::new(&HttpConfig {
        timeout: Some(Duration::from_secs(15)),
        buffer_size: Some(1024),
        ..Default::default()
    }) {
        Ok(c) => c,
        Err(_) => {
            status.set_err("download begin failed");
            return false;
        }
    };

    let auth = format!("Bearer {token}");
    let headers: Vec<(&str, &str)> = if token.is_empty() {
        vec![]
    } else {
        vec![("Authorization", auth.as_str())]
    };

    let opened = (|| -> Result<u16, EspError> {
        conn.initiate_request(Method::Get, url, &headers)?;
        conn.initiate_response()?;
        Ok(conn.status())
    })();
    match opened {
        Ok(200) => {}
        Ok(code) => {
            status.set_err(&format!("download http {code}"));
            return false;
        }
        Err(_) => {
            status.set_err("download begin failed");
            return false;
        }
    }

    let mut ota = match EspOta::new() {
        Ok(o) => o,
        Err(_) => {
            status.set_err("ota busy");
            return false;
        }
    };
    let mut update = match ota.initiate_update() {
        Ok(u) => u,
        Err(_) => {
            status.set_err("update begin failed");
            return false;
        }
    };

    let mut hasher = Sha256::new();
    let mut buf = [0u8; 1024];
    let mut total = 0usize;
    let mut write_failed = false;
    loop {
        let n = match conn.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            Err(_) => {
                write_failed = true;
                break;
            }
        };
        if update.write(&buf[..n]).is_err() {
            write_failed = true;
            break;
        }
        hasher.update(&buf[..n]);
        total += n;
    }

    if write_failed || total == 0 {
        let _ = update.abort();
        status.set_err("download incomplete");
        return false;
    }

    let digest = hasher.finalize();
    let mut digest_hex = String::with_capacity(64);
    for b in digest.iter() {
        digest_hex.push_str(&format!("{b:02x}"));
    }
    if !digest_hex.eq_ignore_ascii_case(sha256_hex.trim()) {
        let _ = update.abort();
        status.set_err("sha256 mismatch");
        return false;
    }

    // complete() runs esp_ota_end (validates the image) then sets the boot
    // partition; the caller reboots into it on success.
    match update.complete() {
        Ok(()) => true,
        Err(_) => {
            status.set_err("update end failed");
            false
        }
    }
}
