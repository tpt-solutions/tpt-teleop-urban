//! Lock-free single-producer, single-consumer ring buffer: the inter-crate bus.
//!
//! Capacity is fixed at compile time and must be a power of two. Nothing
//! allocates after `channel` returns, and neither side ever blocks: a full ring
//! hands the value back, and an empty ring returns `None`.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

struct Ring<T, const N: usize> {
    slots: [UnsafeCell<MaybeUninit<T>>; N],
    /// Next index to read. Written only by the receiver.
    head: AtomicUsize,
    /// Next index to write. Written only by the sender.
    tail: AtomicUsize,
}

// SAFETY: a slot is written only by the sender, before it publishes the new
// `tail` with Release, and read only by the receiver, after it observes that
// `tail` with Acquire. The two sides never touch the same slot concurrently.
unsafe impl<T: Send, const N: usize> Sync for Ring<T, N> {}

/// Producing half. Not `Clone`, so there is exactly one producer.
pub struct Sender<T, const N: usize> {
    ring: Arc<Ring<T, N>>,
}

/// Consuming half. Not `Clone`, so there is exactly one consumer.
pub struct Receiver<T, const N: usize> {
    ring: Arc<Ring<T, N>>,
}

/// Creates a bus with capacity `N`.
///
/// # Panics
///
/// Fails to compile unless `N` is a power of two.
#[must_use]
pub fn channel<T: Copy + Send, const N: usize>() -> (Sender<T, N>, Receiver<T, N>) {
    const { assert!(N.is_power_of_two(), "bus capacity must be a power of two") };
    let ring = Arc::new(Ring {
        slots: [const { UnsafeCell::new(MaybeUninit::uninit()) }; N],
        head: AtomicUsize::new(0),
        tail: AtomicUsize::new(0),
    });
    (
        Sender {
            ring: Arc::clone(&ring),
        },
        Receiver { ring },
    )
}

impl<T: Copy, const N: usize> Sender<T, N> {
    /// Enqueues `value`, or returns it unchanged if the ring is full.
    pub fn try_send(&mut self, value: T) -> Result<(), T> {
        let ring = &*self.ring;
        let tail = ring.tail.load(Ordering::Relaxed);
        let head = ring.head.load(Ordering::Acquire);
        if tail.wrapping_sub(head) == N {
            return Err(value);
        }
        // SAFETY: `tail - head < N`, so this slot is free. The receiver's
        // Acquire load of `head` above orders its earlier read of the slot
        // before this write. Only this sender writes slots.
        unsafe {
            (*ring.slots[tail & (N - 1)].get()).write(value);
        }
        ring.tail.store(tail.wrapping_add(1), Ordering::Release);
        Ok(())
    }
}

impl<T: Copy, const N: usize> Receiver<T, N> {
    /// Dequeues the oldest value, or `None` if the ring is empty.
    pub fn try_recv(&mut self) -> Option<T> {
        let ring = &*self.ring;
        let head = ring.head.load(Ordering::Relaxed);
        let tail = ring.tail.load(Ordering::Acquire);
        if head == tail {
            return None;
        }
        // SAFETY: `head != tail`, so the sender has published this slot with a
        // Release store of `tail`, which the Acquire load above synchronizes
        // with. Only this receiver reads slots, and `T: Copy` means reading
        // without taking ownership is sound.
        let value = unsafe { (*ring.slots[head & (N - 1)].get()).assume_init_read() };
        ring.head.store(head.wrapping_add(1), Ordering::Release);
        Some(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn values_come_out_in_send_order() {
        let (mut tx, mut rx) = channel::<u32, 4>();
        assert_eq!(rx.try_recv(), None);
        for v in 1..=4 {
            tx.try_send(v).expect("ring has room");
        }
        let got: Vec<u32> = std::iter::from_fn(|| rx.try_recv()).collect();
        assert_eq!(got, vec![1, 2, 3, 4]);
    }

    #[test]
    fn full_ring_returns_the_value() {
        let (mut tx, _rx) = channel::<u8, 2>();
        tx.try_send(10).unwrap();
        tx.try_send(11).unwrap();
        assert_eq!(tx.try_send(12), Err(12));
    }

    #[test]
    fn indices_wrap_around_capacity() {
        let (mut tx, mut rx) = channel::<u64, 2>();
        for v in 0..1_000 {
            tx.try_send(v).unwrap();
            assert_eq!(rx.try_recv(), Some(v));
        }
    }

    #[test]
    fn delivers_across_threads_in_order() {
        const COUNT: u64 = 100_000;
        let (mut tx, mut rx) = channel::<u64, 64>();
        let producer = std::thread::spawn(move || {
            for mut v in 0..COUNT {
                while let Err(back) = tx.try_send(v) {
                    v = back;
                    std::hint::spin_loop();
                }
            }
        });
        let mut expected = 0;
        while expected < COUNT {
            match rx.try_recv() {
                Some(v) => {
                    assert_eq!(v, expected);
                    expected += 1;
                }
                None => std::hint::spin_loop(),
            }
        }
        producer.join().expect("producer thread");
    }
}
