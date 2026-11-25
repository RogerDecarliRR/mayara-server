//! Annular viewport geometry and Mayara range-step snap (geo_echo viewport).

use super::raster::{bearing_in_sector, BBox};
use super::super::target::{meters_per_degree_longitude, METERS_PER_DEGREE_LATITUDE};
use crate::radar::{FRAC_NM_2, FRAC_NM_4, FRAC_NM_8, FRAC_NM_16, FRAC_NM_32, NM};

/// Viewport paint range steps (metres), ascending — NM ladder (FRAC_NM_* / NM)
/// used for LocalView-style snap-up (e.g. 0.4 NM → ½ NM). Metric GUI dial
/// intermediates (75, 250, 750, …) are intentionally omitted so the PPI
/// resolution steps match nautical-mile range stops.
pub const VIEWPORT_RANGE_STEPS_M: &[f64] = &[
    50.0,
    FRAC_NM_32 as f64,
    FRAC_NM_16 as f64,
    FRAC_NM_8 as f64,
    FRAC_NM_4 as f64,
    FRAC_NM_2 as f64,
    (NM * 3 / 4) as f64,
    NM as f64,
    (NM * 3 / 2) as f64,
    (NM * 2) as f64,
    (NM * 3) as f64,
    (NM * 4) as f64,
    (NM * 6) as f64,
    (NM * 8) as f64,
    (NM * 12) as f64,
    (NM * 14) as f64,
    (NM * 16) as f64,
    (NM * 24) as f64,
    (NM * 32) as f64,
    (NM * 36) as f64,
    (NM * 40) as f64,
    (NM * 48) as f64,
];

/// Annular sector in polar coordinates about the radar asset.
///
/// Bearings are North = 0°, clockwise. When `full_circle` is set, θ is ignored.
/// Otherwise the sector is `[theta_min_deg, theta_max_deg)` and may wrap
/// (`theta_min_deg > theta_max_deg`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Annulus {
    pub r_min_m: f64,
    pub r_max_m: f64,
    pub theta_min_deg: f64,
    pub theta_max_deg: f64,
    pub full_circle: bool,
}

impl Annulus {
    /// True when range `r_m` and bearing (N=0 CW) lie in this annulus.
    pub fn contains(&self, r_m: f64, bearing_deg: f64) -> bool {
        if !r_m.is_finite() || r_m < 0.0 {
            return false;
        }
        if r_m + 1e-9 < self.r_min_m || r_m > self.r_max_m + 1e-9 {
            return false;
        }
        if self.full_circle {
            return true;
        }
        bearing_in_sector(bearing_deg, self.theta_min_deg, self.theta_max_deg)
    }
}

/// Geographic diagonal of `bbox` in metres (at bbox centre latitude).
pub fn bbox_diagonal_m(bbox: BBox) -> f64 {
    let (ew, ns) = bbox.spans_m();
    (ew * ew + ns * ns).sqrt()
}

/// Half of the **smaller** geographic span — viewport "radius" used to snap
/// the Mayara range step (not the diagonal; diagonal was over-estimating).
pub fn bbox_half_min_span_m(bbox: BBox) -> f64 {
    let (ew, ns) = bbox.spans_m();
    ew.min(ns).max(0.0) * 0.5
}

/// Snap `need_m` **up** to the nearest step in `steps` (ascending), not above `max_m`.
///
/// If `need_m` exceeds every step ≤ `max_m`, returns the largest such step
/// (or `max_m` when no step qualifies). Steps below ½ NM are valid.
pub fn snap_range_step_up(need_m: f64, steps: &[f64], max_m: f64) -> f64 {
    let max_m = if max_m.is_finite() && max_m > 0.0 {
        max_m
    } else {
        return 0.0;
    };
    let need = if need_m.is_finite() && need_m > 0.0 {
        need_m
    } else {
        0.0
    };
    let mut best_ge: Option<f64> = None;
    let mut best_le: Option<f64> = None;
    for &s in steps {
        if !(s.is_finite() && s > 0.0) || s > max_m + 1e-9 {
            continue;
        }
        best_le = Some(s);
        if s + 1e-9 >= need {
            best_ge = Some(s);
            break;
        }
    }
    best_ge.or(best_le).unwrap_or(max_m.min(need.max(max_m)))
}

/// Bearing degrees North=0 clockwise from local east/north metres.
pub fn bearing_deg_from_east_north(east_m: f64, north_m: f64) -> f64 {
    east_m.atan2(north_m).to_degrees().rem_euclid(360.0)
}

/// Annular sector covering `bbox` as seen from the radar asset `(radar_lat, radar_lon)`.
pub fn annulus_from_bbox(radar_lat: f64, radar_lon: f64, bbox: BBox) -> Annulus {
    let m_lon = meters_per_degree_longitude(&radar_lat).max(1e-12);
    let to_en = |lat: f64, lon: f64| -> (f64, f64) {
        let north = (lat - radar_lat) * METERS_PER_DEGREE_LATITUDE;
        let east = (lon - radar_lon) * m_lon;
        (east, north)
    };

    let corners = [
        (bbox.south, bbox.west),
        (bbox.south, bbox.east),
        (bbox.north, bbox.west),
        (bbox.north, bbox.east),
    ];
    let mut corner_en = [(0.0_f64, 0.0_f64); 4];
    let mut bearings = [0.0_f64; 4];
    let mut r_max = 0.0_f64;
    for (i, &(lat, lon)) in corners.iter().enumerate() {
        let (e, n) = to_en(lat, lon);
        corner_en[i] = (e, n);
        let r = (e * e + n * n).sqrt();
        r_max = r_max.max(r);
        bearings[i] = bearing_deg_from_east_north(e, n);
    }

    // ENU AABB of the geographic bbox (local tangent plane at the asset).
    let mut e_min = f64::INFINITY;
    let mut e_max = f64::NEG_INFINITY;
    let mut n_min = f64::INFINITY;
    let mut n_max = f64::NEG_INFINITY;
    for &(e, n) in &corner_en {
        e_min = e_min.min(e);
        e_max = e_max.max(e);
        n_min = n_min.min(n);
        n_max = n_max.max(n);
    }

    let inside = e_min <= 0.0 && e_max >= 0.0 && n_min <= 0.0 && n_max >= 0.0;
    let r_min = if inside {
        0.0
    } else {
        distance_point_to_aabb(0.0, 0.0, e_min, e_max, n_min, n_max)
    };

    if inside || r_max <= 1e-9 {
        return Annulus {
            r_min_m: 0.0,
            r_max_m: r_max.max(r_min),
            theta_min_deg: 0.0,
            theta_max_deg: 360.0,
            full_circle: true,
        };
    }

    let (theta_min, theta_max, full) = covering_bearing_arc(&bearings);
    Annulus {
        r_min_m: r_min,
        r_max_m: r_max,
        theta_min_deg: theta_min,
        theta_max_deg: theta_max,
        full_circle: full,
    }
}

fn distance_point_to_aabb(
    px: f64,
    py: f64,
    xmin: f64,
    xmax: f64,
    ymin: f64,
    ymax: f64,
) -> f64 {
    let dx = if px < xmin {
        xmin - px
    } else if px > xmax {
        px - xmax
    } else {
        0.0
    };
    let dy = if py < ymin {
        ymin - py
    } else if py > ymax {
        py - ymax
    } else {
        0.0
    };
    (dx * dx + dy * dy).sqrt()
}

/// Minimal arc covering `bearings_deg` (N=0 CW). Returns `(start, end, full_circle)`.
fn covering_bearing_arc(bearings_deg: &[f64]) -> (f64, f64, bool) {
    if bearings_deg.is_empty() {
        return (0.0, 360.0, true);
    }
    let mut a: Vec<f64> = bearings_deg
        .iter()
        .map(|b| b.rem_euclid(360.0))
        .collect();
    a.sort_by(|x, y| x.total_cmp(y));
    // Dedup near-identical bearings.
    a.dedup_by(|x, y| (*x - *y).abs() < 1e-9);
    if a.len() == 1 {
        let b = a[0];
        return (b, (b + 1e-6) % 360.0, false);
    }
    let n = a.len();
    let mut max_gap = 0.0_f64;
    let mut gap_after = 0usize;
    for i in 0..n {
        let next = if i + 1 < n {
            a[i + 1]
        } else {
            a[0] + 360.0
        };
        let gap = next - a[i];
        if gap > max_gap {
            max_gap = gap;
            gap_after = i;
        }
    }
    // Almost no gap → points surround the origin angularly → full circle.
    if max_gap < 1e-6 {
        return (0.0, 360.0, true);
    }
    // Covering arc is the complement of the largest gap.
    let theta_min = if gap_after + 1 < n {
        a[gap_after + 1]
    } else {
        a[0]
    };
    let theta_max = a[gap_after];
    // If the covering span is ≥ 360° − ε, treat as full.
    let span = if theta_min <= theta_max {
        theta_max - theta_min
    } else {
        (360.0 - theta_min) + theta_max
    };
    if span >= 360.0 - 1e-6 {
        return (0.0, 360.0, true);
    }
    (theta_min, theta_max, false)
}

/// HALO-like metres-per-bin for a snapped view step (`1.5 × step / spoke_len`).
pub fn view_m_per_bin(view_step_m: f64, spoke_len: usize) -> f64 {
    let cov = view_step_m.max(0.0) * 1.5;
    let n = spoke_len.max(1) as f64;
    cov / n
}

/// Sample acquisition polar at geographic `(range_m, bearing_deg)` with
/// nearest-neighbour (legend indices are discrete — no bilinear dilute).
pub fn sample_polar_nearest(
    polar: &[u8],
    spokes: usize,
    spoke_len: usize,
    spoke_range_m: f64,
    range_m: f64,
    bearing_deg: f64,
) -> u8 {
    if spokes == 0
        || spoke_len == 0
        || polar.len() < spokes * spoke_len
        || !(spoke_range_m.is_finite() && spoke_range_m > 0.0)
        || !(range_m.is_finite() && range_m >= 0.0)
        || range_m > spoke_range_m + 1e-9
    {
        return 0;
    }
    // Bin i covers [(i)/N, (i+1)/N) × spoke_range; centre ≈ (i+0.5)/N.
    let r_idx = ((range_m / spoke_range_m) * spoke_len as f64).floor() as usize;
    let r_idx = r_idx.min(spoke_len - 1);
    let b_idx = ((bearing_deg.rem_euclid(360.0) / 360.0) * spokes as f64).floor() as usize % spokes;
    polar[b_idx * spoke_len + r_idx]
}

/// Build a view-polar strip by sampling a geographic world source at each
/// bin centre — pitch is `m_per_bin` only (no acquisition / GUI Range polar).
pub fn build_view_polar_from_sampler(
    spokes: usize,
    r_min_m: f64,
    r_max_m: f64,
    m_per_bin: f64,
    echo_at: &dyn Fn(f64, f64) -> u8,
) -> (Vec<u8>, usize) {
    if spokes == 0 || m_per_bin <= 0.0 || !(r_max_m > r_min_m) {
        return (Vec::new(), 0);
    }
    let span = (r_max_m - r_min_m).max(0.0);
    let n_radial = ((span / m_per_bin).ceil() as usize).max(1);
    let mut out = vec![0u8; spokes * n_radial];
    for s in 0..spokes {
        let bearing = (s as f64) * 360.0 / (spokes as f64);
        for i in 0..n_radial {
            let r_c = r_min_m + (i as f64 + 0.5) * m_per_bin;
            if r_c > r_max_m + m_per_bin {
                continue;
            }
            out[s * n_radial + i] = echo_at(bearing, r_c);
        }
    }
    (out, n_radial)
}

/// Build a view-polar strip covering `[r_min, r_max]` at `m_per_bin` density.
///
/// Each view bin takes the **max** acquisition sample over its geographic
/// window so thin acq bins are not missed between view-bin centres (LocalView
/// block magnification when acq is coarse; fine acq still resolves per bin).
pub fn build_view_polar(
    polar: &[u8],
    spokes: usize,
    spoke_len: usize,
    spoke_range_m: f64,
    r_min_m: f64,
    r_max_m: f64,
    m_per_bin: f64,
) -> (Vec<u8>, usize) {
    if spokes == 0
        || spoke_len == 0
        || m_per_bin <= 0.0
        || !(r_max_m > r_min_m)
        || !(spoke_range_m.is_finite() && spoke_range_m > 0.0)
        || polar.len() < spokes * spoke_len
    {
        return (Vec::new(), 0);
    }
    let span = (r_max_m - r_min_m).max(0.0);
    let n_radial = ((span / m_per_bin).ceil() as usize).max(1);
    let mut out = vec![0u8; spokes * n_radial];
    for s in 0..spokes {
        let spoke = &polar[s * spoke_len..(s + 1) * spoke_len];
        for i in 0..n_radial {
            let r0 = r_min_m + i as f64 * m_per_bin;
            let r1 = (r0 + m_per_bin).min(r_max_m + m_per_bin);
            if r0 > r_max_m + 1e-9 {
                continue;
            }
            let i0 = ((r0 / spoke_range_m) * spoke_len as f64).floor() as isize;
            let i1 = ((r1 / spoke_range_m) * spoke_len as f64).ceil() as isize;
            let mut mx = 0u8;
            let lo = i0.max(0) as usize;
            let hi = i1.min(spoke_len as isize).max(0) as usize;
            for bi in lo..hi.min(spoke_len) {
                mx = mx.max(spoke[bi]);
            }
            // Degenerate window: fall back to nearest at bin centre.
            if lo >= hi {
                let r_c = r0 + 0.5 * m_per_bin;
                mx = sample_polar_nearest(
                    polar,
                    spokes,
                    spoke_len,
                    spoke_range_m,
                    r_c,
                    (s as f64) * 360.0 / (spokes as f64),
                );
            }
            out[s * n_radial + i] = mx;
        }
    }
    (out, n_radial)
}

/// Nearest-neighbour sample of a view-polar strip (`n_radial` bins from `r_min`).
pub fn sample_view_polar_nearest(
    view: &[u8],
    spokes: usize,
    n_radial: usize,
    r_min_m: f64,
    m_per_bin: f64,
    range_m: f64,
    bearing_deg: f64,
) -> u8 {
    if spokes == 0 || n_radial == 0 || view.len() < spokes * n_radial || m_per_bin <= 0.0 {
        return 0;
    }
    let u = (range_m - r_min_m) / m_per_bin;
    if u < 0.0 || u >= n_radial as f64 {
        return 0;
    }
    let r_idx = (u.floor() as usize).min(n_radial - 1);
    let b_idx = ((bearing_deg.rem_euclid(360.0) / 360.0) * spokes as f64).floor() as usize % spokes;
    view[b_idx * n_radial + r_idx]
}

/// Bilinear sample of a view-polar strip (bearing wraps, range clamps at edges).
pub fn sample_view_polar_bilinear(
    view: &[u8],
    spokes: usize,
    n_radial: usize,
    r_min_m: f64,
    m_per_bin: f64,
    range_m: f64,
    bearing_deg: f64,
) -> u8 {
    if spokes == 0 || n_radial == 0 || view.len() < spokes * n_radial || m_per_bin <= 0.0 {
        return 0;
    }
    let u = (range_m - r_min_m) / m_per_bin;
    if u < 0.0 || u >= n_radial as f64 {
        return 0;
    }
    let uf = u.clamp(0.0, (n_radial - 1) as f64);
    let r0 = uf.floor() as usize;
    let r1 = (r0 + 1).min(n_radial - 1);
    let dr = (uf - r0 as f64) as f32;

    let bf = (bearing_deg.rem_euclid(360.0) / 360.0) * spokes as f64;
    let s0 = bf.floor() as usize % spokes;
    let s1 = (s0 + 1) % spokes;
    let ds = (bf - bf.floor()) as f32;

    let at = |sp: usize, rd: usize| -> f32 { view[sp * n_radial + rd] as f32 };
    let v00 = at(s0, r0);
    let v10 = at(s1, r0);
    let v01 = at(s0, r1);
    let v11 = at(s1, r1);
    let top = v00 * (1.0_f32 - ds) + v10 * ds;
    let bot = v01 * (1.0_f32 - ds) + v11 * ds;
    let v = top * (1.0_f32 - dr) + bot * dr;
    if v < 0.5 {
        0
    } else {
        v.round().min(255.0) as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::NM_F64;

    #[test]
    fn snap_0_4_nm_up_to_half_nm() {
        let diag = 0.4 * NM_F64;
        let step = snap_range_step_up(diag, VIEWPORT_RANGE_STEPS_M, 14.0 * NM_F64);
        assert!(
            (step - FRAC_NM_2 as f64).abs() < 1e-9,
            "0.4 NM must snap up to ½ NM, got {step}"
        );
    }

    #[test]
    fn snap_respects_max_ceiling() {
        let diag = 20.0 * NM_F64;
        let max_m = 6.0 * NM_F64;
        let step = snap_range_step_up(diag, VIEWPORT_RANGE_STEPS_M, max_m);
        assert!(step <= max_m + 1e-9, "step {step} exceeds max {max_m}");
        assert!((step - (NM * 6) as f64).abs() < 1e-9);
    }

    #[test]
    fn bbox_diagonal_matches_spans() {
        let lat = -19.85;
        let lon = -40.06;
        let dlat = 1000.0 / METERS_PER_DEGREE_LATITUDE;
        let dlon = 1000.0 / meters_per_degree_longitude(&lat);
        let bbox = BBox {
            west: lon - dlon,
            south: lat - dlat,
            east: lon + dlon,
            north: lat + dlat,
        };
        let (ew, ns) = bbox.spans_m();
        let expected = (ew * ew + ns * ns).sqrt();
        assert!((bbox_diagonal_m(bbox) - expected).abs() < 1.0);
    }

    #[test]
    fn bbox_half_min_square_2km_is_about_1km() {
        // Square ~2 km side → half of the smaller span ≈ 1000 m.
        let lat = -19.85;
        let lon = -40.06;
        let half_side_m = 1000.0;
        let dlat = half_side_m / METERS_PER_DEGREE_LATITUDE;
        let dlon = half_side_m / meters_per_degree_longitude(&lat);
        let bbox = BBox {
            west: lon - dlon,
            south: lat - dlat,
            east: lon + dlon,
            north: lat + dlat,
        };
        let half = bbox_half_min_span_m(bbox);
        assert!(
            (half - 1000.0).abs() < 5.0,
            "half_min for ~2 km square must be ≈1000 m, got {half}"
        );
    }

    #[test]
    fn annulus_asset_inside_is_full_disk() {
        let lat = -19.85;
        let lon = -40.06;
        let dlat = 0.01;
        let dlon = 0.01;
        let bbox = BBox {
            west: lon - dlon,
            south: lat - dlat,
            east: lon + dlon,
            north: lat + dlat,
        };
        let a = annulus_from_bbox(lat, lon, bbox);
        assert!(a.full_circle);
        assert!(a.r_min_m < 1e-6);
        assert!(a.r_max_m > 500.0);
        assert!(a.contains(10.0, 0.0));
        assert!(a.contains(10.0, 180.0));
    }

    #[test]
    fn annulus_asset_outside_is_sector() {
        let radar_lat = -19.85;
        let radar_lon = -40.06;
        // Bbox due north of the asset.
        let dlat = 2000.0 / METERS_PER_DEGREE_LATITUDE;
        let half = 500.0 / METERS_PER_DEGREE_LATITUDE;
        let dlon = 500.0 / meters_per_degree_longitude(&radar_lat);
        let bbox = BBox {
            west: radar_lon - dlon,
            south: radar_lat + dlat - half,
            east: radar_lon + dlon,
            north: radar_lat + dlat + half,
        };
        let a = annulus_from_bbox(radar_lat, radar_lon, bbox);
        assert!(!a.full_circle, "off-center viewport must be a sector");
        assert!(a.r_min_m > 1000.0, "r_min={}" , a.r_min_m);
        assert!(a.r_max_m > a.r_min_m);
        assert!(
            a.contains((a.r_min_m + a.r_max_m) * 0.5, 0.0),
            "north mid-range must be inside"
        );
        assert!(
            !a.contains((a.r_min_m + a.r_max_m) * 0.5, 180.0),
            "south must be outside sector"
        );
    }

    #[test]
    fn sample_polar_respects_geographic_range() {
        let spokes = 4;
        let spoke_len = 10;
        let mut polar = vec![0u8; spokes * spoke_len];
        // North spoke, bin 5 → centre at (5.5/10)*range.
        polar[5] = 100;
        let range = 1000.0;
        let r_hit = (5.5 / 10.0) * range;
        let v = sample_polar_nearest(&polar, spokes, spoke_len, range, r_hit, 0.0);
        assert!(v > 50, "hit at geographic range, got {v}");
        let v_far = sample_polar_nearest(&polar, spokes, spoke_len, range, range + 10.0, 0.0);
        assert_eq!(v_far, 0);
    }

    #[test]
    fn view_polar_from_sampler_is_independent_of_acq_polar() {
        let echo = |bearing: f64, range: f64| {
            if (bearing - 0.0).abs() < 1.0 && (range - 500.0).abs() < 25.0 {
                9
            } else {
                0
            }
        };
        let (view, n) = build_view_polar_from_sampler(8, 400.0, 600.0, 20.0, &echo);
        assert!(n > 0);
        assert!(view.iter().any(|&v| v == 9), "sampler hit must appear");
        // Empty acq polar would yield zeros via build_view_polar — sampler does not.
        let (empty, _) = build_view_polar(&[0u8; 8 * 16], 8, 16, 1000.0, 400.0, 600.0, 20.0);
        assert!(empty.iter().all(|&v| v == 0));
    }
}
