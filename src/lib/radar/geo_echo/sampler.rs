//! Optional geographic echo source for viewport geo_echo (emulator world).
//!
//! When registered on [`crate::radar::SharedRadars`], viewport PNG intensities
//! are filled by sampling this source at geographic (bearing, range) — not by
//! remapping the GUI acquisition polar (whose pitch follows ControlId::Range).

use std::sync::Arc;

/// Viewport world sampler: prepare once per publish, then `echo_at` per sample.
///
/// `echo_at(bearing_deg, range_m)` — bearing North = 0° clockwise, range metres
/// from the radar asset. Returns a legend pixel index / intensity (0 = empty).
#[derive(Clone)]
pub struct ViewportWorldSampler {
    prepare: Arc<dyn Fn() + Send + Sync>,
    echo_at: Arc<dyn Fn(f64, f64) -> u8 + Send + Sync>,
}

impl ViewportWorldSampler {
    pub fn new(
        prepare: Arc<dyn Fn() + Send + Sync>,
        echo_at: Arc<dyn Fn(f64, f64) -> u8 + Send + Sync>,
    ) -> Self {
        Self { prepare, echo_at }
    }

    /// Refresh any brand-side cache before a burst of [`Self::echo_at`] calls.
    pub fn prepare(&self) {
        (self.prepare)();
    }

    /// Intensity at geographic bearing (N=0 CW degrees) and range (metres).
    pub fn echo_at(&self, bearing_deg: f64, range_m: f64) -> u8 {
        (self.echo_at)(bearing_deg, range_m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn prepare_then_echo_at() {
        let prepares = Arc::new(AtomicUsize::new(0));
        let p = Arc::clone(&prepares);
        let sampler = ViewportWorldSampler::new(
            Arc::new(move || {
                p.fetch_add(1, Ordering::SeqCst);
            }),
            Arc::new(|bearing, range| {
                if (bearing - 90.0).abs() < 1.0 && (range - 500.0).abs() < 1.0 {
                    12
                } else {
                    0
                }
            }),
        );
        sampler.prepare();
        assert_eq!(prepares.load(Ordering::SeqCst), 1);
        assert_eq!(sampler.echo_at(90.0, 500.0), 12);
        assert_eq!(sampler.echo_at(0.0, 500.0), 0);
    }
}
