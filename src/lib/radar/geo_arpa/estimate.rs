//! Estimate LOA/beam and heading from polar echo extent (RM-132 / RM-138).
//!
//! Radar returns are blobs with radial and cross-range span. LOA = max of the
//! two spans; beam = LOA/6 (fixed aspect — not measured from the minor span).
//! Emit `lengthM`/`beamM` only when the pair passes RM-102 validation; never
//! pass sim truth.
//!
//! Heading (RM-138): PCA major axis of the polar imprint in ENU (east/north)
//! relative to the antenna. Ambiguous by ±π (bow vs stern); tracker picks the
//! sense nearest the previous heading. No LOS heuristic, axis hysteresis, EMA,
//! or SOG freeze.

use std::f64::consts::{PI, TAU};

/// RM-102 / symbology: same bounds as imevts `valid_dimensions`.
pub const HULL_LENGTH_MIN_M: f64 = 3.0;
pub const HULL_LENGTH_MAX_M: f64 = 460.0;

/// Cap on polar pixels fed to PCA (stride subsample when larger).
pub const HEADING_PCA_MAX_POINTS: usize = 512;

/// Absolute floor on the dominant eigenvalue (degenerate cloud).
const HEADING_PCA_LAMBDA_MIN: f64 = 1e-6;

/// Relative |λ1−λ2|/max(1,λ1) below which the cloud is treated as circular.
const HEADING_PCA_CIRCULAR_REL: f64 = 1e-6;

/// True when LOA ∈ [3, 460] m and 0 < beam < length.
pub fn valid_dimensions(length_m: f64, beam_m: f64) -> bool {
    if !length_m.is_finite() || !beam_m.is_finite() {
        return false;
    }
    if length_m < HULL_LENGTH_MIN_M || length_m > HULL_LENGTH_MAX_M {
        return false;
    }
    // 0 < beam < length (RM-102); exclusive lower bound — `0.0..len` would accept 0.
    beam_m > 0.0 && beam_m < length_m
}

/// LOA = max(span), beam = LOA/6. `None` when outside RM-102.
pub fn estimate_dims_from_spans(range_span_m: f64, cross_span_m: f64) -> Option<(f64, f64)> {
    if !range_span_m.is_finite() || !cross_span_m.is_finite() {
        return None;
    }
    if range_span_m <= 0.0 || cross_span_m <= 0.0 {
        return None;
    }
    let length = range_span_m.max(cross_span_m);
    let beam = length / 6.0;
    if valid_dimensions(length, beam) {
        Some((length, beam))
    } else {
        None
    }
}

/// Radial extent (m) × bearing span (rad) at `distance_m` → optional LOA/beam.
pub fn estimate_dims_from_polar(
    range_span_m: f64,
    bearing_span_rad: f64,
    distance_m: f64,
) -> Option<(f64, f64)> {
    if !bearing_span_rad.is_finite() || !distance_m.is_finite() || distance_m < 0.0 {
        return None;
    }
    let cross_span_m = bearing_span_rad.abs() * distance_m;
    estimate_dims_from_spans(range_span_m, cross_span_m)
}

/// Blob pixel/spoke extents → optional dims (tracker / geo-arpa path).
pub fn estimate_dims_from_blob_extent(
    pixel_extent: usize,
    spoke_extent: u16,
    range_m: f64,
    spoke_len: usize,
    spokes_per_revolution: u16,
    distance_m: f64,
) -> Option<(f64, f64)> {
    let (range_span_m, cross_span_m) = blob_spans_m(
        pixel_extent,
        spoke_extent,
        range_m,
        spoke_len,
        spokes_per_revolution,
        distance_m,
    )?;
    estimate_dims_from_spans(range_span_m, cross_span_m)
}

/// Radial and cross-range spans (m) from blob pixel/spoke extents.
pub fn blob_spans_m(
    pixel_extent: usize,
    spoke_extent: u16,
    range_m: f64,
    spoke_len: usize,
    spokes_per_revolution: u16,
    distance_m: f64,
) -> Option<(f64, f64)> {
    if spoke_len == 0 || spokes_per_revolution == 0 || range_m <= 0.0 {
        return None;
    }
    if !distance_m.is_finite() || distance_m < 0.0 {
        return None;
    }
    let meters_per_pixel = range_m / spoke_len as f64;
    let range_span_m = pixel_extent.max(1) as f64 * meters_per_pixel;
    let bearing_span_rad =
        (spoke_extent.max(1) as f64) * TAU / spokes_per_revolution as f64;
    if !bearing_span_rad.is_finite() {
        return None;
    }
    let cross_span_m = bearing_span_rad.abs() * distance_m;
    Some((range_span_m, cross_span_m))
}

/// Resolve bow/stern ambiguity: pick `raw` or `raw+π`, whichever is closer to `prev`.
pub fn stabilize_heading_rad(raw: f64, prev: Option<f64>) -> f64 {
    let raw = raw.rem_euclid(TAU);
    let Some(prev) = prev.filter(|p| p.is_finite()) else {
        return raw;
    };
    let alt = (raw + PI).rem_euclid(TAU);
    if circular_abs_delta(prev, alt) < circular_abs_delta(prev, raw) {
        alt
    } else {
        raw
    }
}

/// PCA major-axis heading (rad, N=0 CW) from ENU points `(east, north)`.
///
/// `None` when fewer than 3 points, λ1 degenerated, or the cloud is nearly
/// circular (λ1 ≈ λ2). Ambiguous by ±π.
pub fn estimate_heading_rad_from_enu(points: &[(f64, f64)]) -> Option<f64> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let mut mean_e = 0.0;
    let mut mean_n = 0.0;
    for &(e, north) in points {
        if !e.is_finite() || !north.is_finite() {
            return None;
        }
        mean_e += e;
        mean_n += north;
    }
    mean_e /= n;
    mean_n /= n;

    let mut s_ee = 0.0;
    let mut s_nn = 0.0;
    let mut s_en = 0.0;
    for &(e, north) in points {
        let de = e - mean_e;
        let dn = north - mean_n;
        s_ee += de * de;
        s_nn += dn * dn;
        s_en += de * dn;
    }

    // Eigenvalues of the 2×2 (unnormalized) scatter / cov matrix.
    let trace = s_ee + s_nn;
    let det = s_ee * s_nn - s_en * s_en;
    let disc = (trace * trace * 0.25 - det).max(0.0);
    let root = disc.sqrt();
    let lam1 = trace * 0.5 + root;
    let lam2 = trace * 0.5 - root;

    if lam1 < HEADING_PCA_LAMBDA_MIN {
        return None;
    }
    if (lam1 - lam2).abs() < HEADING_PCA_CIRCULAR_REL * lam1.max(1.0) {
        return None;
    }

    // Eigenvector for λ1: (λ1 − s_nn, s_en), with fallback.
    let (mut vx, mut vy) = if s_en.abs() > 1e-12 || (s_ee - lam1).abs() > 1e-12 {
        let mut vx = lam1 - s_nn;
        let mut vy = s_en;
        if vx.abs() < 1e-12 && vy.abs() < 1e-12 {
            vx = s_en;
            vy = lam1 - s_ee;
        }
        (vx, vy)
    } else {
        (1.0, 0.0)
    };
    if vx.abs() < 1e-12 && vy.abs() < 1e-12 {
        return None;
    }
    // Normalize for numerical hygiene (direction only).
    let norm = (vx * vx + vy * vy).sqrt();
    if norm < 1e-12 {
        return None;
    }
    vx /= norm;
    vy /= norm;

    // N=0 CW: atan2(east, north).
    Some(vx.atan2(vy).rem_euclid(TAU))
}

/// `(spoke, pixel)` → ENU `(east, north)` relative to the antenna.
///
/// `heading_offset` = `ctx.bearing − ctx.angle` in spokes (same offset as
/// blob centre polarisation). `None` when geometry is invalid or range ≤ 0.
pub fn spoke_pixel_to_enu(
    spoke: u16,
    pixel: usize,
    heading_offset: i32,
    spokes_per_revolution: u16,
    spoke_len: usize,
    range_m: f64,
) -> Option<(f64, f64)> {
    if spokes_per_revolution == 0 || spoke_len == 0 || range_m <= 0.0 || !range_m.is_finite() {
        return None;
    }
    let true_spoke =
        (spoke as i32 + heading_offset).rem_euclid(spokes_per_revolution as i32) as u16;
    let bearing = (true_spoke as f64 / spokes_per_revolution as f64) * TAU;
    let range = (pixel as f64 / spoke_len as f64) * range_m;
    if !bearing.is_finite() || !range.is_finite() || range < 0.0 {
        return None;
    }
    let (sin_b, cos_b) = bearing.sin_cos();
    Some((range * sin_b, range * cos_b))
}

/// Subsample polar pixels with stride when longer than [`HEADING_PCA_MAX_POINTS`].
pub fn sample_polar_pixels(pixels: &[(u16, usize)]) -> Vec<(u16, usize)> {
    let n = pixels.len();
    if n <= HEADING_PCA_MAX_POINTS {
        return pixels.to_vec();
    }
    let stride = (n + HEADING_PCA_MAX_POINTS - 1) / HEADING_PCA_MAX_POINTS;
    pixels.iter().step_by(stride).copied().collect()
}

/// Heading from polar imprint pixels via ENU PCA (RM-138).
///
/// Prefer contour; caller may pass `all_pixels` when contour is empty.
/// Samples with stride when `pixels.len() > HEADING_PCA_MAX_POINTS`.
pub fn estimate_heading_rad_from_polar_pixels(
    pixels: &[(u16, usize)],
    heading_offset: i32,
    spokes_per_revolution: u16,
    spoke_len: usize,
    range_m: f64,
) -> Option<f64> {
    let sampled = sample_polar_pixels(pixels);
    if sampled.len() < 3 {
        return None;
    }
    let mut enu = Vec::with_capacity(sampled.len());
    for &(spoke, pixel) in &sampled {
        enu.push(spoke_pixel_to_enu(
            spoke,
            pixel,
            heading_offset,
            spokes_per_revolution,
            spoke_len,
            range_m,
        )?);
    }
    estimate_heading_rad_from_enu(&enu)
}

fn circular_signed_delta(from: f64, to: f64) -> f64 {
    let mut d = (to - from).rem_euclid(TAU);
    if d > PI {
        d -= TAU;
    }
    d
}

fn circular_abs_delta(a: f64, b: f64) -> f64 {
    circular_signed_delta(a, b).abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_dimensions_rm102() {
        assert!(valid_dimensions(100.0, 16.0));
        assert!(!valid_dimensions(2.0, 1.0)); // LOA too small
        assert!(!valid_dimensions(500.0, 16.0)); // LOA too large
        assert!(!valid_dimensions(100.0, 100.0)); // beam == length
        assert!(!valid_dimensions(100.0, 0.0));
        assert!(!valid_dimensions(100.0, -1.0));
    }

    #[test]
    fn estimate_from_distinct_spans() {
        let (loa, beam) = estimate_dims_from_spans(80.0, 16.0).expect("ok");
        assert!((loa - 80.0).abs() < 1e-9);
        assert!((beam - 80.0 / 6.0).abs() < 1e-9);
        let (loa2, beam2) = estimate_dims_from_spans(16.0, 80.0).expect("swap");
        assert!((loa2 - 80.0).abs() < 1e-9);
        assert!((beam2 - 80.0 / 6.0).abs() < 1e-9);
    }

    #[test]
    fn equal_spans_emit_l_over_6() {
        let (loa, beam) = estimate_dims_from_spans(30.0, 30.0).expect("equal spans ok");
        assert!((loa - 30.0).abs() < 1e-9);
        assert!((beam - 5.0).abs() < 1e-9);
        assert!(valid_dimensions(loa, beam));
    }

    #[test]
    fn polar_estimate_uses_cross_range() {
        let range_span = 40.0;
        let bearing = 0.1; // rad
        let distance = 200.0; // cross = 20 m → LOA = max(40, 20)
        let (loa, beam) = estimate_dims_from_polar(range_span, bearing, distance).expect("ok");
        assert!((loa - 40.0).abs() < 1e-6);
        assert!((beam - 40.0 / 6.0).abs() < 1e-6);
        assert!(valid_dimensions(loa, beam));
    }

    #[test]
    fn blob_extent_helper_rejects_tiny() {
        // 1 pixel × 1 spoke at short range → too small for RM-102
        assert!(estimate_dims_from_blob_extent(1, 1, 1000.0, 1024, 2048, 500.0).is_none());
    }

    fn assert_heading_axis_aligned(h: f64, a: f64, b: f64) {
        let da = circular_abs_delta(h, a);
        let db = circular_abs_delta(h, b);
        assert!(
            da.min(db) < 1e-6,
            "heading {h} not near {a} or {b} (Δ={})",
            da.min(db)
        );
    }

    #[test]
    fn pca_points_along_east_west() {
        // Line along east → heading π/2 or 3π/2
        let pts = [(-40.0, 0.0), (-20.0, 0.0), (0.0, 0.0), (20.0, 0.0), (40.0, 0.0)];
        let h = estimate_heading_rad_from_enu(&pts).expect("heading");
        assert_heading_axis_aligned(h, FRAC_PI_2_LOCAL, 3.0 * FRAC_PI_2_LOCAL);
    }

    #[test]
    fn pca_points_along_north_south() {
        let pts = [(0.0, -40.0), (0.0, -20.0), (0.0, 0.0), (0.0, 20.0), (0.0, 40.0)];
        let h = estimate_heading_rad_from_enu(&pts).expect("heading");
        assert_heading_axis_aligned(h, 0.0, PI);
    }

    #[test]
    fn pca_nearly_circular_omitted() {
        // Regular octagon — λ1 ≈ λ2
        let mut pts = Vec::new();
        for i in 0..8 {
            let a = (i as f64) * TAU / 8.0;
            pts.push((10.0 * a.sin(), 10.0 * a.cos()));
        }
        assert!(estimate_heading_rad_from_enu(&pts).is_none());
    }

    #[test]
    fn pca_diagonal_forty_five() {
        // Line east=north → heading π/4 or π/4+π
        let pts = [
            (-40.0, -40.0),
            (-20.0, -20.0),
            (0.0, 0.0),
            (20.0, 20.0),
            (40.0, 40.0),
        ];
        let h = estimate_heading_rad_from_enu(&pts).expect("heading");
        assert_heading_axis_aligned(h, FRAC_PI_4_LOCAL, FRAC_PI_4_LOCAL + PI);
    }

    #[test]
    fn pca_too_few_points_omitted() {
        assert!(estimate_heading_rad_from_enu(&[(0.0, 0.0), (1.0, 0.0)]).is_none());
        assert!(estimate_heading_rad_from_enu(&[]).is_none());
    }

    #[test]
    fn stabilize_picks_closer_pi_sense() {
        let prev = 0.1;
        let raw = PI + 0.1; // opposite sense
        let s = stabilize_heading_rad(raw, Some(prev));
        assert!((s - prev).abs() < 1e-12);
    }

    #[test]
    fn stabilize_keeps_raw_when_closer() {
        let prev = 0.1;
        let raw = 0.2;
        let s = stabilize_heading_rad(raw, Some(prev));
        assert!((s - raw).abs() < 1e-12);
    }

    #[test]
    fn polar_pixels_east_west_via_north_bearing_offset_zero() {
        // Spokes around north (spoke 0), varying range → radial N-S line.
        // heading_offset=0, 2048 spokes, spoke_len=1000, range=1000 m.
        let pixels: Vec<(u16, usize)> = (100..=500).step_by(50).map(|p| (0u16, p)).collect();
        let h = estimate_heading_rad_from_polar_pixels(&pixels, 0, 2048, 1000, 1000.0).expect("h");
        assert_heading_axis_aligned(h, 0.0, PI);
    }

    const FRAC_PI_2_LOCAL: f64 = PI / 2.0;
    const FRAC_PI_4_LOCAL: f64 = PI / 4.0;
}
