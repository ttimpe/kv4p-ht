//! Single-producer, N-reader broadcast byte ring for the audio stream. Port of
//! the ring in the C `streamer.h` (`ring` / `ringWrite` / `ringRead`).
//!
//! One writer (the audio pump) appends PCM bytes and publishes a monotonic
//! `write_pos` (wraps at u32 like the C `ringWritePos`); any number of readers
//! keep their own cursor and copy out with [`BroadcastRing::read_at`], which
//! validates seqlock-style *after* the copy that the writer has not lapped the
//! region. Readers that fall behind resync near live (see `RESYNC_*`); the
//! writer never blocks and never allocates.
//!
//! The buffer is lazily heap-allocated on the first client connect (mirrors
//! the C: nobody pays the 32 kB unless someone is listening, leaving a larger
//! contiguous free block for the uplink's TLS handshake). `write` is a no-op
//! until then, exactly like the C `if (!ring) return;`.

use std::sync::atomic::{fence, AtomicPtr, AtomicU32, Ordering};

/// Power of two; ~1 s of 32 kB/s PCM (C `RING_SIZE`).
pub const RING_SIZE: usize = 32768;
/// A reader whose backlog exceeds this has hit TCP backpressure and must
/// resync instead of stalling the ring (C `RING_SIZE - 4096`).
pub const RESYNC_THRESHOLD: u32 = (RING_SIZE - 4096) as u32;
/// Resynced readers restart this far behind live (C `ringWritePos - 8192`).
pub const RESYNC_BACKOFF: u32 = 8192;

pub struct BroadcastRing {
    /// Null until [`ensure_allocated`](Self::ensure_allocated); then a leaked
    /// `RING_SIZE` allocation that lives for the rest of the program.
    buf: AtomicPtr<u8>,
    /// Monotonic (wrapping) count of bytes ever written.
    write_pos: AtomicU32,
}

impl BroadcastRing {
    pub const fn new() -> BroadcastRing {
        BroadcastRing {
            buf: AtomicPtr::new(core::ptr::null_mut()),
            write_pos: AtomicU32::new(0),
        }
    }

    /// Allocate the buffer if not yet done (called from the stream accept
    /// path, never from the producer). Returns false only on OOM.
    pub fn ensure_allocated(&self) -> bool {
        if !self.buf.load(Ordering::Acquire).is_null() {
            return true;
        }
        // Fallible allocation: `vec![0; N]` aborts on OOM, but this runs under
        // heap pressure by design (lazy alloc next to ~40 kB TLS handshakes) —
        // the C checked heap_caps_malloc for NULL and served 503 instead.
        let mut v: Vec<u8> = Vec::new();
        if v.try_reserve_exact(RING_SIZE).is_err() {
            return false;
        }
        v.resize(RING_SIZE, 0);
        let boxed: Box<[u8]> = v.into_boxed_slice();
        let ptr = Box::into_raw(boxed) as *mut u8;
        match self.buf.compare_exchange(
            core::ptr::null_mut(),
            ptr,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => true,
            Err(_) => {
                // Lost the allocation race; free ours, keep the winner's.
                // SAFETY: `ptr` came from Box::into_raw above with this exact
                // layout and was never published.
                unsafe {
                    drop(Box::from_raw(core::ptr::slice_from_raw_parts_mut(
                        ptr, RING_SIZE,
                    )));
                }
                true
            }
        }
    }

    /// Current monotonic write position (new readers start here).
    pub fn write_pos(&self) -> u32 {
        self.write_pos.load(Ordering::Acquire)
    }

    /// Append `data`. Single producer only. No-op until the buffer exists;
    /// never blocks, never allocates. Port of C `ringWrite`.
    pub fn write(&self, data: &[u8]) {
        debug_assert!(data.len() <= RING_SIZE);
        let buf = self.buf.load(Ordering::Acquire);
        if buf.is_null() {
            return;
        }
        let pos = self.write_pos.load(Ordering::Relaxed); // sole writer
        let idx = pos as usize & (RING_SIZE - 1);
        let first = data.len().min(RING_SIZE - idx);
        // SAFETY: `buf` points to a live RING_SIZE allocation (published once,
        // never freed) and both copies stay in bounds. Readers may copy these
        // bytes concurrently — a deliberate seqlock-style race: a reader that
        // observes torn data also observes `write_pos` having moved past its
        // window and discards the copy (`read_at` returns false). u8 has no
        // invalid representations, so the torn bytes themselves are benign.
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), buf.add(idx), first);
            if data.len() > first {
                core::ptr::copy_nonoverlapping(data.as_ptr().add(first), buf, data.len() - first);
            }
        }
        // Release-publish: a reader that sees the new position also sees the
        // bytes written above.
        self.write_pos
            .store(pos.wrapping_add(data.len() as u32), Ordering::Release);
    }

    /// Seqlock-style read of `dst.len()` bytes at monotonic position `pos`.
    /// Returns false if the writer lapped the region meanwhile (the copy is
    /// then torn and must be discarded) or the ring is unallocated. Port of C
    /// `ringRead`, including its `(writePos - pos) <= RING_SIZE` validation.
    pub fn read_at(&self, pos: u32, dst: &mut [u8]) -> bool {
        debug_assert!(dst.len() <= RING_SIZE);
        let buf = self.buf.load(Ordering::Acquire);
        if buf.is_null() {
            return false;
        }
        let idx = pos as usize & (RING_SIZE - 1);
        let first = dst.len().min(RING_SIZE - idx);
        // SAFETY: same live allocation and bounds as in `write`; racing with
        // the producer is intended and resolved by the validation below.
        unsafe {
            core::ptr::copy_nonoverlapping(buf.add(idx), dst.as_mut_ptr(), first);
            if dst.len() > first {
                core::ptr::copy_nonoverlapping(buf, dst.as_mut_ptr().add(first), dst.len() - first);
            }
        }
        // The copy must complete before the validation load: an Acquire fence
        // keeps the load from being reordered before the reads above.
        fence(Ordering::Acquire);
        self.write_pos.load(Ordering::Relaxed).wrapping_sub(pos) <= RING_SIZE as u32
    }

    /// Position a lapped/laggard reader restarts from (near live).
    pub fn resync_pos(&self) -> u32 {
        self.write_pos().wrapping_sub(RESYNC_BACKOFF)
    }
}

impl Default for BroadcastRing {
    fn default() -> Self {
        BroadcastRing::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn filled_ring() -> BroadcastRing {
        let r = BroadcastRing::new();
        assert!(r.ensure_allocated());
        r
    }

    #[test]
    fn write_before_alloc_is_noop() {
        let r = BroadcastRing::new();
        r.write(&[1, 2, 3]);
        assert_eq!(r.write_pos(), 0);
        let mut out = [0u8; 3];
        assert!(!r.read_at(0, &mut out));
    }

    #[test]
    fn roundtrip_and_wraparound() {
        let r = filled_ring();
        // Fill up to 100 bytes before the wrap point, then write 200 across it.
        let pre = vec![0xAAu8; RING_SIZE - 100];
        r.write(&pre);
        let data: Vec<u8> = (0..200u32).map(|i| i as u8).collect();
        let pos = r.write_pos();
        r.write(&data);
        assert_eq!(r.write_pos(), (RING_SIZE + 100) as u32);
        let mut out = vec![0u8; 200];
        assert!(r.read_at(pos, &mut out));
        assert_eq!(out, data);
    }

    #[test]
    fn lap_detection() {
        let r = filled_ring();
        r.write(&[7u8; 64]);
        let stale = 0u32; // reader cursor at the very start
        // Lap the reader: write a full ring plus one byte past its window.
        let lap = vec![1u8; RING_SIZE];
        r.write(&lap);
        r.write(&[2u8]);
        // write_pos - stale = RING_SIZE + 65 > RING_SIZE -> torn
        let mut out = [0u8; 64];
        assert!(!r.read_at(stale, &mut out));
        // Exactly RING_SIZE behind is still (just) valid, like the C `<=`.
        let edge = r.write_pos().wrapping_sub(RING_SIZE as u32);
        assert!(r.read_at(edge, &mut out));
    }

    #[test]
    fn resync_semantics() {
        let r = filled_ring();
        let big = vec![3u8; RING_SIZE];
        r.write(&big);
        r.write(&big);
        let laggard = r.write_pos().wrapping_sub(RESYNC_THRESHOLD + 1);
        // The stream task's laggard check fires strictly above the threshold...
        assert!(r.write_pos().wrapping_sub(laggard) > RESYNC_THRESHOLD);
        // ...and the resync position lands RESYNC_BACKOFF behind live, well
        // inside the valid window.
        let pos = r.resync_pos();
        assert_eq!(r.write_pos().wrapping_sub(pos), RESYNC_BACKOFF);
        let mut out = vec![0u8; RESYNC_BACKOFF as usize];
        assert!(r.read_at(pos, &mut out));
    }

    #[test]
    fn monotonic_pos_wraps_u32() {
        let r = filled_ring();
        // Simulate a cursor comparison across the u32 wrap.
        let pos = u32::MAX - 10;
        let write = pos.wrapping_add(100);
        assert_eq!(write.wrapping_sub(pos), 100);
        assert!(write.wrapping_sub(pos) <= RING_SIZE as u32);
        let _ = r;
    }
}
