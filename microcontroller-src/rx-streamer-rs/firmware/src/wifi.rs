//! WiFi connectivity, mDNS and SNTP. Port of the C `wifiMgr.h`.
//!
//! Station mode with saved credentials; if none are saved (or the join hasn't
//! succeeded after 30 s) an open setup AP `kv4p-rx-setup` comes up alongside so
//! the device is always reachable for provisioning — it may be mounted out of
//! physical reach. mDNS advertises `kv4p-rx.local` + `_http._tcp` on port 80,
//! and SNTP (pool.ntp.org) starts on the first STA connect.

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::mdns::EspMdns;
use esp_idf_svc::nvs::{EspNvsPartition, NvsDefault};
use esp_idf_svc::sntp::EspSntp;
use esp_idf_svc::wifi::{
    AccessPointConfiguration, AuthMethod, ClientConfiguration, Configuration, EspWifi,
};

use crate::config::Config;

pub const SETUP_AP_SSID: &str = "kv4p-rx-setup";
const STA_FALLBACK: Duration = Duration::from_secs(30);
const RECONNECT_INTERVAL: Duration = Duration::from_secs(5);

/// SNTP has run once the system clock is past 2023. Mirrors C `timeSynced()`.
pub fn time_synced() -> bool {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs() > 1_700_000_000)
        .unwrap_or(false)
}

/// STA RSSI in dBm, or 0 if not associated. Mirrors C `WiFi.RSSI()` (web
/// status only reports it while connected). Free function so the web task can
/// read it without holding the [`WifiManager`].
pub fn sta_rssi() -> i32 {
    unsafe {
        let mut ap: esp_idf_svc::sys::wifi_ap_record_t = core::mem::zeroed();
        if esp_idf_svc::sys::esp_wifi_sta_get_ap_info(&mut ap) == esp_idf_svc::sys::ESP_OK {
            ap.rssi as i32
        } else {
            0
        }
    }
}

/// The device's own IPv4 as a dotted string — the STA address when connected,
/// else the setup AP address (C `WiFi.localIP()` / `WiFi.softAPIP()`).
pub fn ip_string(connected: bool) -> String {
    let key = if connected {
        c"WIFI_STA_DEF"
    } else {
        c"WIFI_AP_DEF"
    };
    unsafe {
        let netif = esp_idf_svc::sys::esp_netif_get_handle_from_ifkey(key.as_ptr());
        if !netif.is_null() {
            let mut info: esp_idf_svc::sys::esp_netif_ip_info_t = core::mem::zeroed();
            if esp_idf_svc::sys::esp_netif_get_ip_info(netif, &mut info) == esp_idf_svc::sys::ESP_OK
            {
                let a = info.ip.addr; // network byte order: bytes are addr[0..4] = a.b.c.d
                return format!(
                    "{}.{}.{}.{}",
                    a & 0xff,
                    (a >> 8) & 0xff,
                    (a >> 16) & 0xff,
                    (a >> 24) & 0xff
                );
            }
        }
    }
    "0.0.0.0".to_string()
}

fn hs<const N: usize>(s: &str) -> heapless::String<N> {
    // Char-wise so an over-long value truncates at a UTF-8 boundary instead of
    // panicking on a byte slice inside a multi-byte character (the C strlcpy
    // just cut bytes; a panic here would boot-loop on a saved credential).
    let mut out = heapless::String::<N>::new();
    for ch in s.chars() {
        if out.push(ch).is_err() {
            break;
        }
    }
    out
}

fn client_config(cfg: &Config) -> ClientConfiguration {
    // auth_method is the MINIMUM accepted authmode (threshold), not a
    // selection: Arduino's WiFi.begin left it open and let the supplicant
    // negotiate, so a WPA-only or mixed-mode AP still joins. WPA2Personal
    // here silently rejected such APs (STA stuck "connecting" forever).
    ClientConfiguration {
        ssid: hs::<32>(&cfg.ssid),
        password: hs::<64>(&cfg.pass),
        auth_method: AuthMethod::None,
        ..Default::default()
    }
}

fn ap_config() -> AccessPointConfiguration {
    AccessPointConfiguration {
        ssid: hs::<32>(SETUP_AP_SSID),
        auth_method: AuthMethod::None,
        ..Default::default()
    }
}

pub struct WifiManager {
    wifi: EspWifi<'static>,
    have_ssid: bool,
    start: Instant,
    last_reconnect: Instant,
    ap_started: bool,
    mdns: Option<EspMdns>,
    _sntp: Option<EspSntp<'static>>,
    ntp_started: bool,
}

impl WifiManager {
    /// Configure and start WiFi. Mirrors C `wifiSetup()`.
    pub fn new(
        modem: Modem,
        sysloop: EspSystemEventLoop,
        nvs: EspNvsPartition<NvsDefault>,
        cfg: &Config,
    ) -> anyhow::Result<WifiManager> {
        let mut wifi = EspWifi::new(modem, sysloop, Some(nvs))?;
        let have_ssid = !cfg.ssid.is_empty();

        let mgr = if have_ssid {
            wifi.set_configuration(&Configuration::Client(client_config(cfg)))?;
            wifi.start()?;
            // Non-blocking connect (like Arduino WiFi.begin); poll() drives retries.
            let _ = wifi.connect();
            log::info!("[wifi] joining '{}'...", cfg.ssid);
            WifiManager {
                wifi,
                have_ssid,
                start: Instant::now(),
                last_reconnect: Instant::now(),
                ap_started: false,
                mdns: None,
                _sntp: None,
                ntp_started: false,
            }
        } else {
            let mut mgr = WifiManager {
                wifi,
                have_ssid,
                start: Instant::now(),
                last_reconnect: Instant::now(),
                ap_started: false,
                mdns: None,
                _sntp: None,
                ntp_started: false,
            };
            mgr.start_setup_ap(cfg)?;
            mgr
        };
        // Modem sleep is disabled in main.rs (esp_wifi_set_ps(WIFI_PS_NONE),
        // the WiFi.setSleep(false) equivalent) right after this returns, so
        // the audio stream sees no power-save latency spikes.
        Ok(mgr)
    }

    pub fn is_connected(&self) -> bool {
        self.wifi.is_connected().unwrap_or(false)
    }

    fn start_setup_ap(&mut self, cfg: &Config) -> anyhow::Result<()> {
        let config = if self.have_ssid {
            Configuration::Mixed(client_config(cfg), ap_config())
        } else {
            Configuration::AccessPoint(ap_config())
        };
        self.wifi.set_configuration(&config)?;
        self.wifi.start()?;
        if self.have_ssid {
            let _ = self.wifi.connect();
        }
        self.ap_started = true;
        log::info!("[wifi] setup AP '{}' up", SETUP_AP_SSID);
        Ok(())
    }

    /// Drive fallback-AP, mDNS and SNTP bring-up. Call periodically from the
    /// supervisor loop. Mirrors C `wifiLoop()` plus an explicit STA reconnect
    /// (the Arduino stack auto-reconnected; esp-idf-svc does not).
    pub fn poll(&mut self, cfg: &Config) {
        let connected = self.is_connected();

        if self.have_ssid && !connected {
            if self.last_reconnect.elapsed() >= RECONNECT_INTERVAL {
                self.last_reconnect = Instant::now();
                if let Err(e) = self.wifi.connect() {
                    log::warn!("[wifi] connect attempt failed: {e:?}");
                }
            }
        }

        if !self.ap_started && self.have_ssid && !connected && self.start.elapsed() > STA_FALLBACK {
            log::warn!("[wifi] STA join timed out, raising setup AP (STA keeps retrying)");
            let _ = self.start_setup_ap(cfg);
        }

        if self.mdns.is_none() && connected {
            match EspMdns::take() {
                Ok(mut mdns) => {
                    let _ = mdns.set_hostname("kv4p-rx");
                    let _ = mdns.add_service(None, "_http", "_tcp", 80, &[]);
                    self.mdns = Some(mdns);
                    log::info!("[wifi] connected (kv4p-rx.local)");
                }
                Err(e) => log::warn!("[wifi] mdns init failed: {e:?}"),
            }
        }

        if !self.ntp_started && connected {
            match EspSntp::new_default() {
                Ok(sntp) => {
                    self._sntp = Some(sntp);
                    self.ntp_started = true;
                    log::info!("[wifi] SNTP started (pool.ntp.org)");
                }
                Err(e) => log::warn!("[wifi] sntp init failed: {e:?}"),
            }
        }
    }
}
