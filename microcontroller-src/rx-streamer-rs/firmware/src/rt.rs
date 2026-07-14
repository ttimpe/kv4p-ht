//! Runtime helpers shared by the worker modules: pinned/sized thread spawning,
//! task-watchdog subscription, and dynamic (config-driven) GPIO access.
//!
//! GPIO is done through the raw `gpio_*` FFI rather than the typed HAL: the C
//! firmware drives pins by number (`digitalRead`/`digitalWrite`) read from the
//! board description, and the pin numbers change per PCB revision, which the
//! typed peripheral model can't express without owning every possible pin.

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;

/// Spawn a std thread pinned to `core` with an explicit stack size and FreeRTOS
/// priority. Wraps `std::thread::Builder` with a one-shot
/// [`ThreadSpawnConfiguration`], which the very next spawn picks up; the config
/// is reset afterwards so unrelated spawns get defaults.
pub fn spawn<F>(
    name: &'static [u8],
    stack_size: usize,
    priority: u8,
    core: Core,
    f: F,
) -> std::io::Result<std::thread::JoinHandle<()>>
where
    F: FnOnce() + Send + 'static,
{
    // VERIFY(phase1): ThreadSpawnConfiguration field set matches esp-idf-hal 0.51.
    let _ = ThreadSpawnConfiguration {
        name: Some(name),
        stack_size,
        priority,
        inherit: false,
        pin_to_core: Some(core),
        ..Default::default()
    }
    .set();
    let handle = std::thread::Builder::new().stack_size(stack_size).spawn(f);
    let _ = ThreadSpawnConfiguration::default().set();
    handle
}

/// Subscribe the calling task to the Task Watchdog (the TWDT is initialized by
/// ESP-IDF from `sdkconfig`). Mirrors C `esp_task_wdt_add(NULL)`.
pub fn wdt_subscribe_current() {
    // VERIFY(phase1): esp-idf-hal's task::watchdog::TWDTDriver could replace this
    // raw call if the pinned hal version exposes it cleanly.
    unsafe {
        let _ = sys::esp_task_wdt_add(core::ptr::null_mut());
    }
}

/// Feed the watchdog from the calling (subscribed) task.
pub fn wdt_reset() {
    unsafe {
        let _ = sys::esp_task_wdt_reset();
    }
}

/// Dynamic GPIO by pin number (matching the C `pinMode`/`digitalRead`/
/// `digitalWrite` usage). All are no-ops for a negative pin (`-1` = not fitted).
pub mod gpio {
    use super::sys;

    pub fn input(pin: i8, pullup: bool) {
        if pin < 0 {
            return;
        }
        unsafe {
            let n = pin as sys::gpio_num_t;
            sys::gpio_reset_pin(n);
            sys::gpio_set_direction(n, sys::gpio_mode_t_GPIO_MODE_INPUT);
            if pullup {
                sys::gpio_set_pull_mode(n, sys::gpio_pull_mode_t_GPIO_PULLUP_ONLY);
            }
        }
    }

    pub fn output(pin: i8) {
        if pin < 0 {
            return;
        }
        unsafe {
            let n = pin as sys::gpio_num_t;
            sys::gpio_reset_pin(n);
            sys::gpio_set_direction(n, sys::gpio_mode_t_GPIO_MODE_OUTPUT);
        }
    }

    pub fn set(pin: i8, high: bool) {
        if pin < 0 {
            return;
        }
        unsafe {
            sys::gpio_set_level(pin as sys::gpio_num_t, high as u32);
        }
    }

    pub fn get(pin: i8) -> bool {
        if pin < 0 {
            return false;
        }
        unsafe { sys::gpio_get_level(pin as sys::gpio_num_t) != 0 }
    }
}
