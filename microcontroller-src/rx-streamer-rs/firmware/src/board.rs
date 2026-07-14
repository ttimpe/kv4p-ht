//! Board description: pin map, ADC front-end settings and RF module type.
//!
//! Port of the C `hardware.h`. Every kv4p-ht PCB revision keeps working
//! unchanged: v1.x and v2.0c/d are told apart by two strap pins (GPIO39/36),
//! and a factory-written NVS `hwconfig` blob overrides everything when present.
//!
//! Unlike the upstream firmware this module never *writes* the `hwconfig`
//! namespace — this firmware only reads the board description, it does not own
//! it. The namespace is opened read-only.

use esp_idf_svc::hal::gpio::{Input, InputPin, PinDriver};
use esp_idf_svc::nvs::{EspNvs, EspNvsPartition, NvsDefault};
use esp_idf_svc::sys::{adc_atten_t, adc_atten_t_ADC_ATTEN_DB_0, adc_atten_t_ADC_ATTEN_DB_12};

// --- Pin defaults (match hardware.h DEFAULT_PIN_*) ---
pub const DEFAULT_PIN_RF_RXD: i8 = 16;
pub const DEFAULT_PIN_RF_TXD: i8 = 17;
pub const DEFAULT_PIN_AUDIO_OUT: i8 = 25;
pub const DEFAULT_PIN_AUDIO_IN: i8 = 34;
pub const DEFAULT_PIN_PTT: i8 = 18;
pub const DEFAULT_PIN_PD: i8 = 19;
pub const DEFAULT_PIN_SQ: i8 = 32;
pub const DEFAULT_PIN_PHYS_PTT1: i8 = 5;
pub const DEFAULT_PIN_PHYS_PTT2: i8 = 33;
pub const DEFAULT_PIN_LED: i8 = 2;
pub const DEFAULT_PIN_PIXELS: i8 = 13;
pub const DEFAULT_PIN_HL: i8 = -1;

pub const DEFAULT_ADC_BIAS_VOLTAGE: f32 = 1.75;
pub const DEFAULT_HW_VOLUME: u8 = 8;

// ADC1 channel 6 is GPIO34 — usable alongside WiFi (ADC1, not ADC2).
pub const I2S_ADC_CHANNEL: u8 = 6; // ADC1_CHANNEL_6

/// RF module fitted to the board. Governs the default band plan and the
/// tuning range accepted by the SA818.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RfModuleType {
    Sa818Vhf = 0,
    Sa818Uhf = 1,
}

impl RfModuleType {
    pub fn from_u8(v: u8) -> RfModuleType {
        match v {
            1 => RfModuleType::Sa818Uhf,
            _ => RfModuleType::Sa818Vhf,
        }
    }
}

pub const DEFAULT_RF_MODULE_TYPE: RfModuleType = RfModuleType::Sa818Vhf;

/// Pin assignments. A value of `-1` means "not fitted" (matches C `int8_t`
/// semantics — only `pin_hl` is optional in practice).
#[derive(Debug, Clone, Copy)]
pub struct PinConfig {
    pub pin_sq: i8,
    pub pin_rf_module_rxd: i8,
    pub pin_rf_module_txd: i8,
    pub pin_audio_out: i8,
    pub pin_audio_in: i8,
    pub pin_ptt: i8,
    pub pin_pd: i8,
    pub pin_ptt_phys1: i8,
    pub pin_ptt_phys2: i8,
    pub pin_led: i8,
    pub pin_pixels: i8,
    pub pin_hl: i8,
}

impl Default for PinConfig {
    fn default() -> PinConfig {
        PinConfig {
            pin_sq: DEFAULT_PIN_SQ,
            pin_rf_module_rxd: DEFAULT_PIN_RF_RXD,
            pin_rf_module_txd: DEFAULT_PIN_RF_TXD,
            pin_audio_out: DEFAULT_PIN_AUDIO_OUT,
            pin_audio_in: DEFAULT_PIN_AUDIO_IN,
            pin_ptt: DEFAULT_PIN_PTT,
            pin_pd: DEFAULT_PIN_PD,
            pin_ptt_phys1: DEFAULT_PIN_PHYS_PTT1,
            pin_ptt_phys2: DEFAULT_PIN_PHYS_PTT2,
            pin_led: DEFAULT_PIN_LED,
            pin_pixels: DEFAULT_PIN_PIXELS,
            pin_hl: DEFAULT_PIN_HL,
        }
    }
}

/// Full board description. Mirrors C `HWConfig`.
#[derive(Debug, Clone, Copy)]
pub struct HwConfig {
    pub pins: PinConfig,
    pub adc_bias: f32,
    /// Raw ESP-IDF `adc_atten_t`. 0 dB on v2.0c, 12 dB elsewhere.
    pub adc_attenuation: adc_atten_t,
    pub volume: u8,
    pub rf_module_type: RfModuleType,
}

impl Default for HwConfig {
    fn default() -> HwConfig {
        HwConfig {
            pins: PinConfig::default(),
            adc_bias: DEFAULT_ADC_BIAS_VOLTAGE,
            adc_attenuation: adc_atten_t_ADC_ATTEN_DB_12,
            volume: DEFAULT_HW_VOLUME,
            rf_module_type: DEFAULT_RF_MODULE_TYPE,
        }
    }
}

impl HwConfig {
    pub fn module_min_freq_mhz(&self) -> f32 {
        if self.rf_module_type == RfModuleType::Sa818Uhf {
            400.0
        } else {
            134.0
        }
    }

    pub fn module_max_freq_mhz(&self) -> f32 {
        if self.rf_module_type == RfModuleType::Sa818Uhf {
            480.0
        } else {
            174.0
        }
    }
}

// --- Legacy hardware-version strap detection (hardware.h HW_VER_*) ---
const HW_VER_V1: u8 = 0x00;
const HW_VER_V2_0C: u8 = 0xFF;
const HW_VER_V2_0D: u8 = 0xF0;

/// Read the two strap pins (GPIO39 = bit-group 0, GPIO36 = bit-group 1) into
/// the packed nibble byte the C firmware uses.
fn hardware_version<P0, P1>(pin39: PinDriver<'_, P0, Input>, pin36: PinDriver<'_, P1, Input>) -> u8
where
    P0: InputPin,
    P1: InputPin,
{
    let mut ver: u8 = 0x00;
    if pin39.is_high() {
        ver |= 0x0F;
    }
    if pin36.is_high() {
        ver |= 0xF0;
    }
    ver
}

/// Detect the board: prefer the NVS `hwconfig` override, else fall back to the
/// strap-pin revision table. `pin39`/`pin36` are the input-only strap GPIOs;
/// they are consumed here and released on return (they are not used elsewhere).
///
/// Mirrors C `boardSetup()`.
pub fn detect<P0, P1>(
    nvs: &EspNvsPartition<NvsDefault>,
    pin39: PinDriver<'_, P0, Input>,
    pin36: PinDriver<'_, P1, Input>,
) -> HwConfig
where
    P0: InputPin,
    P1: InputPin,
{
    let mut hw = HwConfig::default();

    if hardware_config_exists(nvs) {
        load_hardware_config(nvs, &mut hw);
        return hw;
    }

    match hardware_version(pin39, pin36) {
        HW_VER_V2_0C => {
            hw.pins.pin_sq = 4;
            hw.adc_attenuation = adc_atten_t_ADC_ATTEN_DB_0;
            hw.volume = 6;
            hw.pins.pin_hl = 23;
        }
        HW_VER_V2_0D => {
            hw.pins.pin_sq = 4;
            hw.pins.pin_hl = 23;
        }
        HW_VER_V1 | _ => {}
    }
    hw
}

fn hardware_config_exists(nvs: &EspNvsPartition<NvsDefault>) -> bool {
    // Read-only handle; matches C `hwPrefs.begin("hwconfig", true)`.
    let Ok(store) = EspNvs::new(nvs.clone(), "hwconfig", false) else {
        return false;
    };
    store.contains("HWCONFIG").unwrap_or(false)
}

/// Read every `hwconfig` key with the same defaults as C `loadHardwareConfig()`.
fn load_hardware_config(nvs: &EspNvsPartition<NvsDefault>, hw: &mut HwConfig) {
    let Ok(store) = EspNvs::new(nvs.clone(), "hwconfig", false) else {
        return;
    };

    let get_char =
        |key: &str, default: i8| -> i8 { store.get_i8(key).ok().flatten().unwrap_or(default) };
    let get_uchar =
        |key: &str, default: u8| -> u8 { store.get_u8(key).ok().flatten().unwrap_or(default) };

    hw.pins.pin_rf_module_rxd = get_char("PIN_RF_RXD", DEFAULT_PIN_RF_RXD);
    hw.pins.pin_rf_module_txd = get_char("PIN_RF_TXD", DEFAULT_PIN_RF_TXD);
    hw.pins.pin_audio_out = get_char("PIN_AUDIO_OUT", DEFAULT_PIN_AUDIO_OUT);
    hw.pins.pin_audio_in = get_char("PIN_AUDIO_IN", DEFAULT_PIN_AUDIO_IN);
    hw.pins.pin_ptt = get_char("PIN_PTT", DEFAULT_PIN_PTT);
    hw.pins.pin_pd = get_char("PIN_PD", DEFAULT_PIN_PD);
    hw.pins.pin_sq = get_char("PIN_SQ", DEFAULT_PIN_SQ);
    hw.pins.pin_ptt_phys1 = get_char("PIN_PHYS_PTT1", DEFAULT_PIN_PHYS_PTT1);
    hw.pins.pin_ptt_phys2 = get_char("PIN_PHYS_PTT2", DEFAULT_PIN_PHYS_PTT2);
    hw.pins.pin_pixels = get_char("PIN_PIXELS", DEFAULT_PIN_PIXELS);
    hw.pins.pin_led = get_char("PIN_LED", DEFAULT_PIN_LED);
    hw.pins.pin_hl = get_char("PIN_HL", DEFAULT_PIN_HL);

    // ADC attenuation is stored as an i8 in C (`getChar`); widen to adc_atten_t.
    let atten = get_char("ADC_ATTEN", adc_atten_t_ADC_ATTEN_DB_12 as i8);
    hw.adc_attenuation = atten as adc_atten_t;

    // ADC_BIAS is a string in NVS (C stores TOSTRING(1.75) and calls .toFloat()).
    let mut buf = [0u8; 32];
    if let Ok(Some(s)) = store.get_str("ADC_BIAS", &mut buf) {
        if let Ok(v) = s.trim_end_matches('\0').trim().parse::<f32>() {
            hw.adc_bias = v;
        }
    }

    hw.volume = get_uchar("VOLUME", DEFAULT_HW_VOLUME);
    hw.rf_module_type =
        RfModuleType::from_u8(get_uchar("RF_MODULE_TYPE", DEFAULT_RF_MODULE_TYPE as u8));
}
