//! Optional geographic echo source for viewport geo_echo (emulator world).
//!
//! When registered on [`crate::radar::SharedRadars`], polar MEPO intensities
//! are filled by sampling this source at geographic (bearing, range) — not by
//! remapping the GUI acquisition polar (whose pitch follows ControlId::Range).
//! Paint stays on the geo-echo-renderer; the core only builds the polar.

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

/// Build a polar buffer by sampling the world at each bin centre.
///
/// `spoke_range_m` is the full radial extent (typically `usableRadiusM`) so the
/// MEPO pushed to the renderer covers operational range, independent of GUI
/// Control Range. Call [`ViewportWorldSampler::prepare`] before this.
pub fn fill_polar_from_world(
    sampler: &ViewportWorldSampler,
    spokes: usize,
    spoke_len: usize,
    spoke_range_m: f64,
) -> Vec<u8> {
    let spokes = spokes.max(1);
    let spoke_len = spoke_len.max(1);
    let spoke_range_m = if spoke_range_m.is_finite() && spoke_range_m > 0.0 {
        spoke_range_m
    } else {
        1e-6
    };
    let mut data = vec![0u8; spokes * spoke_len];
    for i in 0..spokes {
        let bearing = (i as f64) * 360.0 / spokes as f64;
        let row_start = i * spoke_len;
        for j in 0..spoke_len {
            let range = (j as f64 + 0.5) / spoke_len as f64 * spoke_range_m;
            data[row_start + j] = sampler.echo_at(bearing, range);
        }
    }
    data
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

    /// World fill with large coverage: bins past a short "GUI" range still have data.
    #[test]
    fn fill_polar_from_world_covers_beyond_short_gui_range() {
        // Echo only beyond 1000 m — past a hypothetical ½ NM GUI (~926 m).
        let sampler = ViewportWorldSampler::new(
            Arc::new(|| {}),
            Arc::new(|_bearing, range| if range > 1000.0 { 20 } else { 0 }),
        );
        let spokes = 8;
        let spoke_len = 40;
        let usable_m = 2000.0;
        sampler.prepare();
        let data = fill_polar_from_world(&sampler, spokes, spoke_len, usable_m);
        assert_eq!(data.len(), spokes * spoke_len);

        let mut outer_hit = false;
        for i in 0..spokes {
            for j in 0..spoke_len {
                let range = (j as f64 + 0.5) / spoke_len as f64 * usable_m;
                let v = data[i * spoke_len + j];
                if range > 1000.0 {
                    assert_eq!(v, 20, "outer bin i={i} j={j} range={range}");
                    outer_hit = true;
                } else {
                    assert_eq!(v, 0, "inner bin i={i} j={j} range={range}");
                }
            }
        }
        assert!(outer_hit, "expected at least one bin beyond 1000 m");
        // Spoke range of the filled polar equals usable coverage (not GUI short).
        assert!((usable_m - 2000.0).abs() < 1e-9);
    }
}
