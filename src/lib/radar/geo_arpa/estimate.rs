//! Estimate LOA/beam from polar echo extent (RM-132).
//!
//! Radar returns are blobs with radial and cross-range span. When both spans
//! are usable and pass RM-102 validation, expose them as `lengthM`/`beamM`.
//! Equal or invalid spans → omit (never invent aspect ratio or pass sim truth).

/// RM-102 / symbology: same bounds as imevts `valid_dimensions`.
pub const HULL_LENGTH_MIN_M: f64 = 3.0;
pub const HULL_LENGTH_MAX_M: f64 = 460.0;

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

/// LOA = max(span), beam = min(span). `None` when equal or outside RM-102.
pub fn estimate_dims_from_spans(range_span_m: f64, cross_span_m: f64) -> Option<(f64, f64)> {
    if !range_span_m.is_finite() || !cross_span_m.is_finite() {
        return None;
    }
    if range_span_m <= 0.0 || cross_span_m <= 0.0 {
        return None;
    }
    let length = range_span_m.max(cross_span_m);
    let beam = range_span_m.min(cross_span_m);
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
    if spoke_len == 0 || spokes_per_revolution == 0 || range_m <= 0.0 {
        return None;
    }
    let meters_per_pixel = range_m / spoke_len as f64;
    let range_span_m = pixel_extent.max(1) as f64 * meters_per_pixel;
    let bearing_span_rad =
        (spoke_extent.max(1) as f64) * std::f64::consts::TAU / spokes_per_revolution as f64;
    estimate_dims_from_polar(range_span_m, bearing_span_rad, distance_m)
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
        assert!((beam - 16.0).abs() < 1e-9);
        let (loa2, beam2) = estimate_dims_from_spans(16.0, 80.0).expect("swap");
        assert!((loa2 - 80.0).abs() < 1e-9);
        assert!((beam2 - 16.0).abs() < 1e-9);
    }

    #[test]
    fn equal_spans_omitted() {
        assert!(estimate_dims_from_spans(30.0, 30.0).is_none());
    }

    #[test]
    fn polar_estimate_uses_cross_range() {
        let range_span = 40.0;
        let bearing = 0.1; // rad
        let distance = 200.0;
        let (loa, beam) = estimate_dims_from_polar(range_span, bearing, distance).expect("ok");
        assert!((loa - 40.0).abs() < 1e-6 || (beam - 40.0).abs() < 1e-6);
        assert!(valid_dimensions(loa, beam));
    }

    #[test]
    fn blob_extent_helper_rejects_tiny() {
        // 1 pixel × 1 spoke at short range → too small for RM-102
        assert!(estimate_dims_from_blob_extent(1, 1, 1000.0, 1024, 2048, 500.0).is_none());
    }
}
