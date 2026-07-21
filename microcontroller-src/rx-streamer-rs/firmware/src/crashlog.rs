//! Crash forensics that survive a reboot, for nodes reachable only over HTTP.
//!
//! A Rust panic (and an allocation failure, the prime suspect for the NEMO
//! crash loop) is captured into a fixed buffer in RTC slow memory, which the
//! ESP32 preserves across soft resets (panic/abort/watchdog — not power
//! cycles). `GET /api/crashlog` serves the last record plus the hardware
//! reset reason of the current boot, so a crash-looping device can be
//! diagnosed without serial access.

use core::fmt::Write as _;
use core::mem::MaybeUninit;

use esp_idf_svc::sys;

const MAGIC: u32 = 0xC0DE_FA11;
const MSG_CAP: usize = 448;

pub const KIND_PANIC: u8 = 1;
pub const KIND_ALLOC: u8 = 2;

#[repr(C)]
struct CrashBuf {
    magic: u32,
    kind: u8,
    _pad: u8,
    len: u16,
    uptime_ms: u32,
    msg: [u8; MSG_CAP],
}

// NOLOAD section: never initialized by startup, so the record written by the
// dying boot is still there when the next boot serves it. Garbage after a
// power cycle — hence the magic check.
#[link_section = ".rtc_noinit"]
static mut CRASH: MaybeUninit<CrashBuf> = MaybeUninit::uninit();

/// `core::fmt::Write` into a fixed stack buffer — no allocation, usable in a
/// panic hook and (critically) in the alloc-error hook.
struct SliceWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl core::fmt::Write for SliceWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

fn commit(kind: u8, msg: &[u8]) {
    unsafe {
        let p = core::ptr::addr_of_mut!(CRASH) as *mut CrashBuf;
        let n = msg.len().min(MSG_CAP);
        core::ptr::copy_nonoverlapping(msg.as_ptr(), (*p).msg.as_mut_ptr(), n);
        (*p).len = n as u16;
        (*p).kind = kind;
        (*p)._pad = 0;
        (*p).uptime_ms = (sys::esp_timer_get_time() / 1000) as u32;
        (*p).magic = MAGIC;
    }
}

/// Install the panic and alloc-error hooks. Call once, right after the logger.
pub fn init() {
    // Chain the previous hook so the message still reaches the serial console
    // before the runtime aborts.
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let mut buf = [0u8; MSG_CAP];
        let mut w = SliceWriter { buf: &mut buf, len: 0 };
        let _ = write!(w, "{info}");
        let n = w.len;
        commit(KIND_PANIC, &buf[..n]);
        prev(info);
    }));

    // Allocation failures abort without running the panic hook; this hook is
    // the only way to see the failing size. Records heap state at the moment
    // of death — exactly the evidence the NEMO OOM hypothesis needs.
    std::alloc::set_alloc_error_hook(|layout| {
        let mut buf = [0u8; 128];
        let mut w = SliceWriter { buf: &mut buf, len: 0 };
        let _ = write!(
            w,
            "alloc failed: {} bytes (align {}), free={} maxblk={}",
            layout.size(),
            layout.align(),
            unsafe { sys::esp_get_free_heap_size() },
            unsafe { sys::heap_caps_get_largest_free_block(sys::MALLOC_CAP_8BIT) },
        );
        let n = w.len;
        commit(KIND_ALLOC, &buf[..n]);
    });
}

/// The record left by the previous crash, if any: `(kind, uptime_ms, msg)`.
/// Persists until overwritten by the next crash or a power cycle.
pub fn last_crash() -> Option<(u8, u32, String)> {
    unsafe {
        let p = core::ptr::addr_of!(CRASH) as *const CrashBuf;
        if (*p).magic != MAGIC {
            return None;
        }
        let len = ((*p).len as usize).min(MSG_CAP);
        let mut tmp = vec![0u8; len];
        core::ptr::copy_nonoverlapping(
            core::ptr::addr_of!((*p).msg) as *const u8,
            tmp.as_mut_ptr(),
            len,
        );
        let msg = String::from_utf8_lossy(&tmp).into_owned();
        Some(((*p).kind, (*p).uptime_ms, msg))
    }
}

pub fn kind_str(kind: u8) -> &'static str {
    match kind {
        KIND_PANIC => "panic",
        KIND_ALLOC => "alloc",
        _ => "?",
    }
}

/// Why the current boot happened, from the hardware.
pub fn reset_reason() -> &'static str {
    #[allow(non_upper_case_globals)]
    match unsafe { sys::esp_reset_reason() } {
        sys::esp_reset_reason_t_ESP_RST_POWERON => "poweron",
        sys::esp_reset_reason_t_ESP_RST_EXT => "external",
        sys::esp_reset_reason_t_ESP_RST_SW => "sw_restart",
        sys::esp_reset_reason_t_ESP_RST_PANIC => "panic/abort",
        sys::esp_reset_reason_t_ESP_RST_INT_WDT => "int_wdt",
        sys::esp_reset_reason_t_ESP_RST_TASK_WDT => "task_wdt",
        sys::esp_reset_reason_t_ESP_RST_WDT => "other_wdt",
        sys::esp_reset_reason_t_ESP_RST_DEEPSLEEP => "deepsleep",
        sys::esp_reset_reason_t_ESP_RST_BROWNOUT => "brownout",
        sys::esp_reset_reason_t_ESP_RST_SDIO => "sdio",
        _ => "unknown",
    }
}
