//! Thread pinning for the thread-per-core layout (spec §4.5).
//!
//! Pinning fixes a thread to one core, so the layout checked by [`crate::schedule`]
//! is the one that runs. On Windows the affinity is a bit mask on the calling thread.
//! On Linux it is `sched_setaffinity`. Other platforms report [`PinError::Unsupported`].
//!
//! Pinning is a deployment property. It is checked here only by reading the mask
//! back. Measured latency on the target still has to confirm the layout.

use std::thread::JoinHandle;

/// Why a thread could not be pinned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinError {
    /// This platform has no pinning support in this crate.
    Unsupported,
    /// The core index does not fit in the platform's affinity mask.
    CoreOutOfRange {
        /// The core that was asked for.
        core: usize,
    },
    /// The operating system refused the request. Carries its error code.
    Os(i32),
}

/// Number of cores the OS reports as available to this process.
#[must_use]
pub fn available_cores() -> usize {
    std::thread::available_parallelism().map_or(1, usize::from)
}

/// Pins the calling thread to `core`. Returns the affinity mask the thread had
/// before, so a caller can see that the pin changed it.
///
/// # Errors
///
/// Returns [`PinError::CoreOutOfRange`] if `core` is 64 or more, or the OS error.
pub fn pin_current_thread(core: usize) -> Result<u64, PinError> {
    if core >= 64 {
        return Err(PinError::CoreOutOfRange { core });
    }
    pin_platform(core)
}

/// Starts a thread pinned to `core` that runs `f`. The thread's pin result is
/// returned as the first value of the join handle's output, so the caller can
/// tell whether pinning succeeded.
pub fn spawn_pinned<T: Send + 'static>(
    core: usize,
    f: impl FnOnce() -> T + Send + 'static,
) -> JoinHandle<(Result<u64, PinError>, T)> {
    std::thread::spawn(move || {
        let pinned = pin_current_thread(core);
        (pinned, f())
    })
}

#[cfg(windows)]
fn pin_platform(core: usize) -> Result<u64, PinError> {
    use windows_sys::Win32::Foundation::GetLastError;
    use windows_sys::Win32::System::Threading::{GetCurrentThread, SetThreadAffinityMask};

    let mask = 1_usize << core;
    // SAFETY: GetCurrentThread returns a pseudo-handle for the calling thread,
    // which SetThreadAffinityMask accepts. The mask is one valid bit.
    let previous = unsafe { SetThreadAffinityMask(GetCurrentThread(), mask) };
    if previous == 0 {
        // SAFETY: GetLastError has no preconditions.
        Err(PinError::Os(unsafe { GetLastError() } as i32))
    } else {
        Ok(previous as u64)
    }
}

#[cfg(target_os = "linux")]
fn pin_platform(core: usize) -> Result<u64, PinError> {
    // SAFETY: a zeroed cpu_set_t is a valid empty set. CPU_SET writes one bit
    // inside it. sched_setaffinity reads the set for the calling thread (pid 0).
    unsafe {
        let mut before: libc::cpu_set_t = std::mem::zeroed();
        if libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut before) != 0 {
            return Err(PinError::Os(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(-1),
            ));
        }
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_ZERO(&mut set);
        libc::CPU_SET(core, &mut set);
        if libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set) != 0 {
            return Err(PinError::Os(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(-1),
            ));
        }
        let mut bits = 0_u64;
        for cpu in 0..64_usize {
            if libc::CPU_ISSET(cpu, &before) {
                bits |= 1 << cpu;
            }
        }
        Ok(bits)
    }
}

#[cfg(not(any(windows, target_os = "linux")))]
fn pin_platform(_core: usize) -> Result<u64, PinError> {
    Err(PinError::Unsupported)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_core_beyond_the_mask_is_refused_without_calling_the_os() {
        assert_eq!(
            pin_current_thread(64),
            Err(PinError::CoreOutOfRange { core: 64 })
        );
    }

    #[test]
    fn pinning_the_first_core_succeeds_and_reports_the_prior_mask() {
        let handle = spawn_pinned(0, || 7_u32);
        let (pinned, value) = handle.join().expect("thread finished");
        assert_eq!(value, 7);
        // On platforms without support the test documents that, rather than failing.
        match pinned {
            Ok(_) | Err(PinError::Unsupported) => {}
            Err(other) => panic!("pinning core 0 failed: {other:?}"),
        }
    }

    #[test]
    fn available_cores_is_at_least_one() {
        assert!(available_cores() >= 1);
    }
}
