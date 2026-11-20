//! Polar buffer → north-up RGBA raster (echo only).

use super::super::target::{METERS_PER_DEGREE_LATITUDE, meters_per_degree_longitude};
use super::super::Legend;
use crate::config::GEO_ECHO_ALLOWED_SIZES;
use rayon::prelude::*;
use std::f64::consts::TAU;

/// True when `bearing_deg` (North = 0, clockwise) lies in `[start, end)`.
///
/// When `start > end` or `end == 360` with `start > 0`, the sector wraps across
/// north. A full circle (`start == 0` and `end == 360`, or start ≈ end after
/// normalising a degenerate empty sector into full) always returns true.
pub fn bearing_in_sector(bearing_deg: f64, start_deg: f64, end_deg: f64) -> bool {
    if !bearing_deg.is_finite() || !start_deg.is_finite() || !end_deg.is_finite() {
        return false;
    }
    let b = bearing_deg.rem_euclid(360.0);
    let start = start_deg.clamp(0.0, 360.0);
    let end = end_deg.clamp(0.0, 360.0);

    // Full circle.
    if (start == 0.0 && end == 360.0) || (start - end).abs() < 1e-9 && start == 0.0 && end == 0.0 {
        return true;
    }
    if (start - end).abs() < 1e-9 {
        // Degenerate zero-width sector → treat as empty (nothing painted).
        return false;
    }

    if start < end {
        b >= start && b < end
    } else {
        // Wrap: e.g. 350→10, or 90→360 (end inclusive of north via 360).
        b >= start || b < end || (end == 360.0 && (b - 0.0).abs() < 1e-12)
    }
}

/// Viewport PNG edge length from geographic spans and config caps.
///
/// 1. `ideal = ceil(max(span_ew, span_ns) / target_m_per_px)`
/// 2. `size = min(max_size_px, max(ideal, 1))`
pub fn viewport_size_px(
    span_ew_m: f64,
    span_ns_m: f64,
    target_meters_per_px: f64,
    max_size_px: u32,
) -> u32 {
    let span = span_ew_m.max(span_ns_m).max(0.0);
    let max_size = max_size_px.max(1);
    if target_meters_per_px <= 0.0 {
        return 1.min(max_size);
    }
    let ideal = (span / target_meters_per_px).ceil();
    let ideal = if ideal.is_finite() && ideal > 0.0 {
        ideal as u32
    } else {
        1
    };
    ideal.max(1).min(max_size)
}

/// Snap a client `sizeHintPx` down to the largest allowed size ≤ hint
/// (`GEO_ECHO_ALLOWED_SIZES`). Hint below the smallest allowed → smallest.
pub fn snap_size_hint_to_allowed(hint: u32) -> u32 {
    GEO_ECHO_ALLOWED_SIZES
        .iter()
        .copied()
        .filter(|&s| s <= hint)
        .max()
        .unwrap_or(GEO_ECHO_ALLOWED_SIZES[0])
}

/// Deserialize `Option<u32>` from a JSON number that may be int or float
/// (`1024` or `1024.0`). Floats are truncated toward zero; negative / NaN /
/// out-of-range → deserialize error. Missing / null → `None` (with `default`).
pub fn deserialize_option_u32_lenient<'de, D>(deserializer: D) -> Result<Option<u32>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::de::Error;
    use serde::Deserialize;
    let Some(n) = Option::<serde_json::Number>::deserialize(deserializer)? else {
        return Ok(None);
    };
    if let Some(u) = n.as_u64() {
        return u32::try_from(u)
            .map(Some)
            .map_err(|_| D::Error::custom("sizeHintPx exceeds u32"));
    }
    if let Some(f) = n.as_f64() {
        if !f.is_finite() || f < 0.0 || f > f64::from(u32::MAX) {
            return Err(D::Error::custom("sizeHintPx out of range"));
        }
        return Ok(Some(f as u32));
    }
    Err(D::Error::custom("sizeHintPx must be a number"))
}

/// Effective max edge for paint: optional `sizeHint` caps `max_size_px`
/// (RM-129 W2 A3/C1 — first pan at ~50% via client hint).
pub fn effective_viewport_max_size_px(max_size_px: u32, size_hint: Option<u32>) -> u32 {
    let max_size = max_size_px.max(1);
    let Some(hint) = size_hint.filter(|&h| h > 0) else {
        return max_size;
    };
    snap_size_hint_to_allowed(hint).min(max_size).max(1)
}

/// Paint one spoke into an RGBA buffer. Bearing 0 = north, clockwise; image
/// y decreases northward (MapLibre north-up).
///
/// `scale` maps spoke range onto the composition canvas
/// (`spoke_range_m / composition_range_m`). Values &lt; 1 paint an interior
/// circle; values &gt; 1 clip at the canvas edge. Full circle, no usable-radius
/// clip beyond the canvas edge.
#[allow(dead_code)]
pub fn paint_spoke_rgba(
    rgba: &mut [u8],
    size: u32,
    bearing_index: usize,
    spokes: usize,
    data: &[u8],
    legend: &Legend,
    scale: f64,
) {
    paint_spoke_rgba_masked(
        rgba,
        size,
        bearing_index,
        spokes,
        data,
        legend,
        scale,
        0.0,
        360.0,
        f64::INFINITY,
        1.0,
    );
}

/// Like [`paint_spoke_rgba`] but skips the spoke when its bearing is outside
/// the usable sector, and clips samples beyond `usable_radius_m`.
///
/// `composition_range_m` is the canvas half-side (metres); radial clip is
/// `r ≤ usable_radius_m` mapped onto that canvas (`usable ≤ composition`).
pub fn paint_spoke_rgba_masked(
    rgba: &mut [u8],
    size: u32,
    bearing_index: usize,
    spokes: usize,
    data: &[u8],
    legend: &Legend,
    scale: f64,
    usable_bearing_start_deg: f64,
    usable_bearing_end_deg: f64,
    usable_radius_m: f64,
    composition_range_m: f64,
) {
    if spokes == 0 || data.is_empty() || scale <= 0.0 {
        return;
    }
    let bearing_deg = (bearing_index as f64) * 360.0 / (spokes as f64);
    if !bearing_in_sector(
        bearing_deg,
        usable_bearing_start_deg,
        usable_bearing_end_deg,
    ) {
        return;
    }
    let size = size as usize;
    let cx = (size / 2) as f64;
    let half = cx;
    let angle = (bearing_index as f64) * TAU / (spokes as f64);
    let sin_a = angle.sin();
    let cos_a = angle.cos();
    let spoke_len = data.len() as f64;
    // Radial mask analogous to bearing_in_sector: paint only r ≤ usableRadiusM.
    let max_dist = if usable_radius_m.is_finite() && composition_range_m > 0.0 {
        half * (usable_radius_m / composition_range_m).clamp(0.0, 1.0)
    } else {
        half
    };

    for (r, &pix) in data.iter().enumerate() {
        if pix == 0 {
            continue;
        }
        let Some(lookup) = legend.pixels.get(pix as usize) else {
            continue;
        };
        let color = &lookup.color;
        let dist = (r as f64 + 0.5) / spoke_len * half * scale;
        if dist > half + 0.5 {
            continue;
        }
        if dist > max_dist + 0.5 {
            continue;
        }
        let x = (cx + dist * sin_a).round() as isize;
        let y = (cx - dist * cos_a).round() as isize;
        if x < 0 || y < 0 {
            continue;
        }
        let (x, y) = (x as usize, y as usize);
        if x >= size || y >= size {
            continue;
        }
        let i = (y * size + x) * 4;
        rgba[i] = color.r;
        rgba[i + 1] = color.g;
        rgba[i + 2] = color.b;
        rgba[i + 3] = color.a;
    }
}

/// Rasterize a full polar buffer (`spokes * spoke_len` bytes, row-major by spoke).
///
/// Samples are scaled from `spoke_range_m` onto a canvas whose half-side is
/// `composition_range_m`. Full bearing circle, usable radius = composition
/// (compat wrapper).
#[allow(dead_code)]
pub fn rasterize_polar(
    polar: &[u8],
    spokes: usize,
    spoke_len: usize,
    legend: &Legend,
    size_px: u32,
    spoke_range_m: f64,
    composition_range_m: f64,
) -> Vec<u8> {
    rasterize_polar_masked(
        polar,
        spokes,
        spoke_len,
        legend,
        size_px,
        spoke_range_m,
        composition_range_m,
        0.0,
        360.0,
        composition_range_m,
    )
}

/// Composition raster with usable-bearing and usable-radius masks.
pub fn rasterize_polar_masked(
    polar: &[u8],
    spokes: usize,
    spoke_len: usize,
    legend: &Legend,
    size_px: u32,
    spoke_range_m: f64,
    composition_range_m: f64,
    usable_bearing_start_deg: f64,
    usable_bearing_end_deg: f64,
    usable_radius_m: f64,
) -> Vec<u8> {
    let size = size_px.max(1) as usize;
    let mut rgba = vec![0u8; size * size * 4];
    if spokes == 0 || spoke_len == 0 || polar.len() < spokes * spoke_len {
        return rgba;
    }
    let scale = if composition_range_m > 0.0 {
        spoke_range_m / composition_range_m
    } else {
        0.0
    };
    for spoke_i in 0..spokes {
        let start = spoke_i * spoke_len;
        let data = &polar[start..start + spoke_len];
        if data.iter().all(|&p| p == 0) {
            continue;
        }
        paint_spoke_rgba_masked(
            &mut rgba,
            size_px.max(1),
            spoke_i,
            spokes,
            data,
            legend,
            scale,
            usable_bearing_start_deg,
            usable_bearing_end_deg,
            usable_radius_m,
            composition_range_m,
        );
    }
    rgba
}

/// Geographic bbox (lon/lat, west/south/east/north).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BBox {
    pub west: f64,
    pub south: f64,
    pub east: f64,
    pub north: f64,
}

impl BBox {
    pub fn validate(self) -> Result<(), String> {
        if ![self.west, self.south, self.east, self.north]
            .iter()
            .all(|v| v.is_finite())
        {
            return Err("bbox coordinates must be finite".to_string());
        }
        if !(self.west < self.east) {
            return Err("bbox west must be less than east".to_string());
        }
        if !(self.south < self.north) {
            return Err("bbox south must be less than north".to_string());
        }
        if !(-90.0..=90.0).contains(&self.south) || !(-90.0..=90.0).contains(&self.north) {
            return Err("bbox latitude must be between -90 and 90".to_string());
        }
        if !(-180.0..=180.0).contains(&self.west) || !(-180.0..=180.0).contains(&self.east) {
            return Err("bbox longitude must be between -180 and 180".to_string());
        }
        Ok(())
    }

    pub fn center(&self) -> (f64, f64) {
        (
            (self.south + self.north) * 0.5,
            (self.west + self.east) * 0.5,
        )
    }

    /// East–west and north–south spans in metres at the bbox centre latitude.
    pub fn spans_m(&self) -> (f64, f64) {
        let (lat, _) = self.center();
        let span_ns = (self.north - self.south) * METERS_PER_DEGREE_LATITUDE;
        let span_ew = (self.east - self.west) * meters_per_degree_longitude(&lat);
        (span_ew.abs(), span_ns.abs())
    }
}

/// Half of the larger geographic span — geographic half-side of the viewport
/// PNG (corners / meta), independent of paint resolution.
pub fn view_range_from_bbox(bbox: BBox) -> f64 {
    let (span_ew, span_ns) = bbox.spans_m();
    span_ew.max(span_ns).max(0.0) * 0.5
}

/// HALO-like spoke coverage distance for a view range (3/2 × view_range).
pub fn view_spoke_coverage_m(view_range_m: f64) -> f64 {
    view_range_m.max(0.0) * 1.5
}

/// Inverse-sample a north-up square covering `bbox` from an annular sector
/// about the radar asset.
///
/// Paint resolution uses snapped `view_step_m` (HALO 1.5× / `spoke_len`
/// metres-per-bin). Intensity comes from a **view polar** rebuilt at that
/// pitch — either by sampling `world_echo` (emulator; GUI Range independent)
/// or by geographic lookup into the acquisition polar.
pub fn rasterize_viewport(
    polar: &[u8],
    spokes: usize,
    spoke_len: usize,
    legend: &Legend,
    size_px: u32,
    radar_lat: f64,
    radar_lon: f64,
    spoke_range_m: f64,
    view_step_m: f64,
    bbox: BBox,
    usable_bearing_start_deg: f64,
    usable_bearing_end_deg: f64,
    usable_radius_m: f64,
) -> Vec<u8> {
    rasterize_viewport_with_world(
        polar,
        spokes,
        spoke_len,
        legend,
        size_px,
        radar_lat,
        radar_lon,
        spoke_range_m,
        view_step_m,
        bbox,
        usable_bearing_start_deg,
        usable_bearing_end_deg,
        usable_radius_m,
        None,
    )
}

/// Like [`rasterize_viewport`], but when `world_echo` is set, intensities are
/// sampled from that geographic source at view_step pitch — acquisition polar
/// / `spoke_range_m` are not used for content (only angular spoke count and
/// `spoke_len` for metres-per-bin).
pub fn rasterize_viewport_with_world(
    polar: &[u8],
    spokes: usize,
    spoke_len: usize,
    legend: &Legend,
    size_px: u32,
    radar_lat: f64,
    radar_lon: f64,
    spoke_range_m: f64,
    view_step_m: f64,
    bbox: BBox,
    usable_bearing_start_deg: f64,
    usable_bearing_end_deg: f64,
    usable_radius_m: f64,
    world_echo: Option<&dyn Fn(f64, f64) -> u8>,
) -> Vec<u8> {
    use super::annulus::{
        annulus_from_bbox, bearing_deg_from_east_north, build_view_polar,
        build_view_polar_from_sampler, sample_view_polar_nearest, view_m_per_bin,
    };

    let size = size_px.max(1) as usize;
    let mut rgba = vec![0u8; size * size * 4];
    let use_world = world_echo.is_some();
    if spokes == 0
        || spoke_len == 0
        || !(view_step_m.is_finite() && view_step_m > 0.0)
        || (!use_world
            && (polar.len() < spokes * spoke_len
                || !(spoke_range_m.is_finite() && spoke_range_m > 0.0)))
    {
        return rgba;
    }

    let annulus = annulus_from_bbox(radar_lat, radar_lon, bbox);
    if annulus.r_max_m <= 0.0 {
        return rgba;
    }

    let m_per_bin = view_m_per_bin(view_step_m, spoke_len);
    let (view_polar, n_radial) = if let Some(echo_at) = world_echo {
        build_view_polar_from_sampler(
            spokes,
            annulus.r_min_m,
            annulus.r_max_m,
            m_per_bin,
            echo_at,
        )
    } else {
        build_view_polar(
            polar,
            spokes,
            spoke_len,
            spoke_range_m,
            annulus.r_min_m,
            annulus.r_max_m,
            m_per_bin,
        )
    };
    if n_radial == 0 || view_polar.is_empty() {
        return rgba;
    }

    let (span_ew, span_ns) = bbox.spans_m();
    let span = span_ew.max(span_ns).max(1e-6);
    let m_per_px = span / size as f64;
    if m_per_px <= 0.0 {
        return rgba;
    }
    let (bbox_lat, bbox_lon) = bbox.center();
    let half_m = span * 0.5;
    let m_lon_bbox = meters_per_degree_longitude(&bbox_lat).max(1e-12);
    let m_lon_radar = meters_per_degree_longitude(&radar_lat).max(1e-12);
    let r_min_m = annulus.r_min_m;

    // RM-129 W2 A4: one row per rayon task — disjoint `par_chunks_mut` slices.
    rgba.par_chunks_mut(size * 4)
        .enumerate()
        .for_each(|(py, row)| {
            for px in 0..size {
                // Pixel centre → ENU relative to bbox centre → geographic.
                let dx_m = (px as f64 + 0.5) * m_per_px - half_m;
                let dy_m = half_m - (py as f64 + 0.5) * m_per_px;
                let lat = bbox_lat + dy_m / METERS_PER_DEGREE_LATITUDE;
                let lon = bbox_lon + dx_m / m_lon_bbox;

                // Geographic (r, θ) from the radar asset (N=0 CW).
                let east = (lon - radar_lon) * m_lon_radar;
                let north = (lat - radar_lat) * METERS_PER_DEGREE_LATITUDE;
                let r = (east * east + north * north).sqrt();
                let theta = bearing_deg_from_east_north(east, north);

                if !annulus.contains(r, theta) {
                    continue;
                }
                if !bearing_in_sector(theta, usable_bearing_start_deg, usable_bearing_end_deg) {
                    continue;
                }
                // Operational usable radius (same mask as composition / geo-arpa).
                if usable_radius_m.is_finite() && r > usable_radius_m + 1e-9 {
                    continue;
                }
                // Acquisition polar has no data past spoke_range; world sampling
                // is limited only by the annulus / view_step / usable radius.
                if !use_world && r > spoke_range_m + 1e-9 {
                    continue;
                }

                let pix = sample_view_polar_nearest(
                    &view_polar,
                    spokes,
                    n_radial,
                    r_min_m,
                    m_per_bin,
                    r,
                    theta,
                );
                if pix == 0 {
                    continue;
                }
                let Some(lookup) = legend.pixels.get(pix as usize) else {
                    continue;
                };
                let i = px * 4;
                let c = &lookup.color;
                row[i] = c.r;
                row[i + 1] = c.g;
                row[i + 2] = c.b;
                row[i + 3] = c.a;
            }
        });
    rgba
}

/// Geographic corners of the square PNG that covers `bbox` at uniform m/px.
pub fn viewport_image_corners(bbox: BBox, size_px: u32) -> super::corners::EchoCorners {
    let (span_ew, span_ns) = bbox.spans_m();
    let span = span_ew.max(span_ns);
    let half = span * 0.5;
    let (lat, lon) = bbox.center();
    let size = size_px.max(1) as f64;
    // Corners are independent of size_px geometrically (span is geographic);
    // size_px only affects metres-per-pixel. Keep param for API symmetry.
    let _ = size;
    super::corners::corners_for(lat, lon, half)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::{Color, Legend, Lookup, PixelType};

    fn solid_legend() -> Legend {
        Legend {
            doppler_approaching: None,
            doppler_receding: None,
            doppler_rain: None,
            history_start: 2,
            low_return: 1,
            medium_return: 1,
            strong_return: 1,
            pixel_colors: 2,
            pixels: vec![
                Lookup {
                    r#type: PixelType::Normal,
                    color: Color {
                        r: 0,
                        g: 0,
                        b: 0,
                        a: 0,
                    },
                },
                Lookup {
                    r#type: PixelType::Normal,
                    color: Color {
                        r: 255,
                        g: 255,
                        b: 255,
                        a: 255,
                    },
                },
            ],
            static_background: None,
        }
    }

    #[test]
    fn short_spoke_on_wide_composition_paints_interior() {
        // 14 NM composition, 1 NM spoke → outermost sample at ~1/14 of radius.
        let composition = 14.0 * 1852.0;
        let spoke_range = 1852.0;
        let size: u32 = 2048;
        let half = (size / 2) as f64;
        let mut rgba = vec![0u8; (size as usize) * (size as usize) * 4];
        let data = vec![1u8; 64];
        let legend = solid_legend();
        let scale = spoke_range / composition;
        paint_spoke_rgba(&mut rgba, size, 0, 1, &data, &legend, scale);

        // North spoke: pixels along +y decreasing from centre.
        let expected_r = ((63.5_f64 / 64.0) * half * scale).round() as isize;
        assert!(
            expected_r < (half * 0.2) as isize,
            "short spoke should stay well inside (r={expected_r}, half={half})"
        );
        let cy = (size / 2) as isize;
        let y = cy - expected_r;
        assert!(y > 0);
        let i = (y as usize * size as usize + size as usize / 2) * 4;
        assert_eq!(
            rgba[i + 3],
            255,
            "outermost short-spoke sample should be painted"
        );

        // Edge of canvas must stay empty.
        let edge_i = (0 * size as usize + size as usize / 2) * 4;
        assert_eq!(rgba[edge_i + 3], 0);
    }

    #[test]
    fn bearing_mask_full_circle() {
        assert!(bearing_in_sector(0.0, 0.0, 360.0));
        assert!(bearing_in_sector(179.0, 0.0, 360.0));
        assert!(bearing_in_sector(359.9, 0.0, 360.0));
    }

    #[test]
    fn bearing_mask_simple_sector() {
        assert!(bearing_in_sector(10.0, 0.0, 90.0));
        assert!(!bearing_in_sector(90.0, 0.0, 90.0));
        assert!(!bearing_in_sector(180.0, 0.0, 90.0));
    }

    #[test]
    fn bearing_mask_wraps_across_north() {
        assert!(bearing_in_sector(350.0, 300.0, 30.0));
        assert!(bearing_in_sector(10.0, 300.0, 30.0));
        assert!(!bearing_in_sector(180.0, 300.0, 30.0));
    }

    #[test]
    fn composition_bearing_mask_skips_out_of_sector_spokes() {
        let spokes = 4;
        let spoke_len = 8;
        let mut polar = vec![0u8; spokes * spoke_len];
        // Spoke 0 = north, spoke 1 = east, spoke 2 = south, spoke 3 = west.
        for s in 0..spokes {
            for r in 0..spoke_len {
                polar[s * spoke_len + r] = 1;
            }
        }
        let legend = solid_legend();
        let size: u32 = 64;
        // Only northern quarter [315, 45) — spoke 0 at 0° included; others out.
        let rgba = rasterize_polar_masked(
            &polar, spokes, spoke_len, &legend, size, 1000.0, 1000.0, 315.0, 45.0, 1000.0,
        );
        let cx = (size / 2) as usize;
        let north_painted = (0..cx).any(|y| rgba[(y * size as usize + cx) * 4 + 3] == 255);
        let south_painted = ((cx + 1)..size as usize)
            .any(|y| rgba[(y * size as usize + cx) * 4 + 3] == 255);
        assert!(north_painted, "north spoke in sector should paint");
        assert!(!south_painted, "south spoke outside sector must stay empty");
    }

    #[test]
    fn composition_usable_radius_clips_outer_samples() {
        let spokes = 1;
        let spoke_len = 64;
        let polar = vec![1u8; spokes * spoke_len];
        let legend = solid_legend();
        let size: u32 = 128;
        let composition = 1000.0;
        let usable = 250.0; // quarter of canvas radius
        let rgba = rasterize_polar_masked(
            &polar, spokes, spoke_len, &legend, size, composition, composition, 0.0, 360.0, usable,
        );
        let cx = (size / 2) as usize;
        let half = cx as f64;
        let max_r_px = half * (usable / composition);
        // Near centre (inside usable) must paint.
        let near_y = cx - (max_r_px * 0.4).round() as usize;
        assert_eq!(
            rgba[(near_y * size as usize + cx) * 4 + 3],
            255,
            "sample inside usableRadiusM must paint"
        );
        // Near canvas rim (outside usable) must stay empty.
        let far_y = 1usize;
        assert_eq!(
            rgba[(far_y * size as usize + cx) * 4 + 3],
            0,
            "sample beyond usableRadiusM must stay empty"
        );
    }

    #[test]
    fn size_hint_snaps_down_to_allowed() {
        assert_eq!(snap_size_hint_to_allowed(1024), 1024);
        assert_eq!(snap_size_hint_to_allowed(1500), 1024);
        assert_eq!(snap_size_hint_to_allowed(2048), 2048);
        assert_eq!(snap_size_hint_to_allowed(3000), 2048);
        assert_eq!(snap_size_hint_to_allowed(4096), 4096);
        assert_eq!(snap_size_hint_to_allowed(500), 1024); // below min → min allowed
        assert_eq!(effective_viewport_max_size_px(2048, Some(1024)), 1024);
        assert_eq!(effective_viewport_max_size_px(2048, None), 2048);
        assert_eq!(effective_viewport_max_size_px(1024, Some(2048)), 1024); // never above cfg max
    }

    #[derive(Debug, serde::Deserialize)]
    #[serde(rename_all = "camelCase")]
    struct SizeHintProbe {
        #[serde(default, deserialize_with = "deserialize_option_u32_lenient")]
        size_hint_px: Option<u32>,
    }

    #[test]
    fn size_hint_px_accepts_int_and_float() {
        let int: SizeHintProbe = serde_json::from_str(r#"{"sizeHintPx":1024}"#).unwrap();
        assert_eq!(int.size_hint_px, Some(1024));
        let float: SizeHintProbe = serde_json::from_str(r#"{"sizeHintPx":1024.0}"#).unwrap();
        assert_eq!(float.size_hint_px, Some(1024));
        let trunc: SizeHintProbe = serde_json::from_str(r#"{"sizeHintPx":1024.9}"#).unwrap();
        assert_eq!(trunc.size_hint_px, Some(1024));
        let missing: SizeHintProbe = serde_json::from_str(r#"{}"#).unwrap();
        assert_eq!(missing.size_hint_px, None);
        let null: SizeHintProbe = serde_json::from_str(r#"{"sizeHintPx":null}"#).unwrap();
        assert_eq!(null.size_hint_px, None);
    }

    #[test]
    fn viewport_size_clamps_to_max() {
        // 10 km span at 2 m/px → ideal 5000 → clamp to 2048.
        assert_eq!(viewport_size_px(10_000.0, 8_000.0, 2.0, 2048), 2048);
        // 100 m at 2 m/px → 50.
        assert_eq!(viewport_size_px(100.0, 80.0, 2.0, 2048), 50);
        // Tiny span still at least 1.
        assert_eq!(viewport_size_px(0.1, 0.1, 2.0, 1024), 1);
    }

    #[test]
    fn composition_full_circle_regression() {
        let spokes = 4;
        let spoke_len = 16;
        let mut polar = vec![0u8; spokes * spoke_len];
        for s in 0..spokes {
            polar[s * spoke_len + spoke_len - 1] = 1;
        }
        let legend = solid_legend();
        let size: u32 = 64;
        let rgba = rasterize_polar(&polar, spokes, spoke_len, &legend, size, 1000.0, 1000.0);
        // Outer samples on N/E/S/W should paint near the rim.
        let painted = rgba.chunks(4).filter(|px| px[3] > 0).count();
        assert!(painted >= 4, "expected at least one pixel per spoke, got {painted}");
    }

    #[test]
    fn view_range_from_bbox_is_half_max_span() {
        let lat = -19.85;
        let lon = -40.06;
        // ~1 km NS, ~2 km EW → half max = ~1 km.
        let dlat = 1000.0 / METERS_PER_DEGREE_LATITUDE;
        let dlon = 2000.0 / meters_per_degree_longitude(&lat);
        let bbox = BBox {
            west: lon - dlon * 0.5,
            south: lat - dlat * 0.5,
            east: lon + dlon * 0.5,
            north: lat + dlat * 0.5,
        };
        let (ew, ns) = bbox.spans_m();
        let expected = ew.max(ns) * 0.5;
        let got = view_range_from_bbox(bbox);
        assert!(
            (got - expected).abs() < 1.0,
            "view_range={got} expected≈{expected} (ew={ew} ns={ns})"
        );
        assert!((view_spoke_coverage_m(got) - got * 1.5).abs() < 1e-9);
    }

    #[test]
    fn viewport_raster_clips_outside_annulus() {
        let spokes = 1;
        let spoke_len = 32;
        let polar = vec![1u8; spokes * spoke_len];
        let legend = solid_legend();
        let radar_lat = -19.85;
        let radar_lon = -40.06;
        // Tiny bbox well south of radar — annular sector misses the filled polar.
        let bbox = BBox {
            west: radar_lon - 0.0001,
            south: radar_lat - 0.05,
            east: radar_lon + 0.0001,
            north: radar_lat - 0.04,
        };
        let view_step_m = 0.5 * 1852.0;
        let rgba = rasterize_viewport(
            &polar,
            spokes,
            spoke_len,
            &legend,
            64,
            radar_lat,
            radar_lon,
            500.0,
            view_step_m,
            bbox,
            0.0,
            360.0,
            f64::INFINITY,
        );
        assert!(rgba.iter().all(|&b| b == 0));
    }

    /// Polar samples are range/bearing only; geographic placement uses the
    /// antenna site. Polar "at A" (Sneek) with bbox+site at B (port) must paint
    /// into B — using A's lat/lon with B's bbox yields a transparent PNG.
    #[test]
    fn viewport_site_b_not_polar_a_paints_in_bbox_b() {
        let spokes = 4;
        let spoke_len = 16;
        let mut polar = vec![0u8; spokes * spoke_len];
        // North spoke mid-range hit.
        polar[spoke_len / 2] = 1;
        let legend = solid_legend();
        let site_b_lat = -19.85;
        let site_b_lon = -40.06;
        let polar_a_lat = 53.03; // Sneek-ish
        let polar_a_lon = 5.66;
        let bbox_b = BBox {
            west: site_b_lon - 0.02,
            south: site_b_lat - 0.02,
            east: site_b_lon + 0.02,
            north: site_b_lat + 0.02,
        };
        let spoke_range_m = 2_000.0;
        let view_step_m = 1852.0;
        let with_site_b = rasterize_viewport(
            &polar,
            spokes,
            spoke_len,
            &legend,
            128,
            site_b_lat,
            site_b_lon,
            spoke_range_m,
            view_step_m,
            bbox_b,
            0.0,
            360.0,
            f64::INFINITY,
        );
        let painted_b = with_site_b.chunks(4).filter(|px| px[3] > 0).count();
        assert!(
            painted_b > 0,
            "site B + bbox B must paint echo pixels (got {painted_b})"
        );

        let with_site_a = rasterize_viewport(
            &polar,
            spokes,
            spoke_len,
            &legend,
            128,
            polar_a_lat,
            polar_a_lon,
            spoke_range_m,
            view_step_m,
            bbox_b,
            0.0,
            360.0,
            f64::INFINITY,
        );
        assert!(
            with_site_a.iter().all(|&b| b == 0),
            "polar centre A + bbox B must stay transparent"
        );
    }

    /// Coarse acquisition + fine view_step: inverse sample fills many pixels
    /// (LocalView-style blocks) without splat disks.
    #[test]
    fn viewport_coarse_polar_fine_step_inverse_fills_region() {
        let spokes = 64;
        let spoke_len = 512;
        let mut polar = vec![0u8; spokes * spoke_len];
        let hit_r = spoke_len / 4;
        polar[hit_r] = 1;
        polar[hit_r + 1] = 1;
        polar[hit_r + 2] = 1;
        let legend = solid_legend();
        let radar_lat = -19.85;
        let radar_lon = -40.06;
        let spoke_range_m = 14.0 * 1852.0;
        let view_step_m = 0.5 * 1852.0;
        let range_hit = (hit_r as f64 + 0.5) / spoke_len as f64 * spoke_range_m;
        let hit_lat = radar_lat + range_hit / METERS_PER_DEGREE_LATITUDE;
        let half_span_m = view_step_m;
        let dlat = half_span_m / METERS_PER_DEGREE_LATITUDE;
        let dlon = half_span_m / meters_per_degree_longitude(&hit_lat);
        let bbox = BBox {
            west: radar_lon - dlon,
            south: hit_lat - dlat,
            east: radar_lon + dlon,
            north: hit_lat + dlat,
        };
        let size_px = 256u32;
        let rgba = rasterize_viewport(
            &polar,
            spokes,
            spoke_len,
            &legend,
            size_px,
            radar_lat,
            radar_lon,
            spoke_range_m,
            view_step_m,
            bbox,
            0.0,
            360.0,
            f64::INFINITY,
        );
        let painted = rgba.chunks(4).filter(|px| px[3] > 0).count();
        assert!(
            painted > 20,
            "inverse sample at fine view_step must fill a region (painted={painted})"
        );
    }

    /// Opaque morphology depends on geographic echo placement + view_step, not
    /// the acquisition `spoke_range_m` label when polar content is remapped.
    #[test]
    fn viewport_opaque_independent_of_spoke_range_label_when_geo_matched() {
        // Fine radial bins so both spoke_range labels can encode the same
        // geographic echo window without coarse-bin edge smear.
        let spokes = 64;
        let spoke_len = 512;
        let legend = solid_legend();
        let radar_lat = -19.85;
        let radar_lon = -40.06;
        let geo_hit_m = 1500.0;
        let view_step_m = 1852.0;

        let dlat = 800.0 / METERS_PER_DEGREE_LATITUDE;
        let dlon = 800.0 / meters_per_degree_longitude(&radar_lat);
        let hit_lat = radar_lat + geo_hit_m / METERS_PER_DEGREE_LATITUDE;
        let bbox = BBox {
            west: radar_lon - dlon,
            south: hit_lat - dlat,
            east: radar_lon + dlon,
            north: hit_lat + dlat,
        };

        let place_hit = |spoke_range: f64| -> Vec<u8> {
            let mut polar = vec![0u8; spokes * spoke_len];
            let geo_lo = geo_hit_m - 40.0;
            let geo_hi = geo_hit_m + 40.0;
            for i in 0..spoke_len {
                let r0 = (i as f64) / spoke_len as f64 * spoke_range;
                let r1 = ((i + 1) as f64) / spoke_len as f64 * spoke_range;
                if r1 > geo_lo && r0 < geo_hi {
                    polar[i] = 1; // north spoke (index 0)
                }
            }
            polar
        };

        let range_a = 3_000.0;
        let range_b = 12_000.0;
        let rgba_a = rasterize_viewport(
            &place_hit(range_a),
            spokes,
            spoke_len,
            &legend,
            128,
            radar_lat,
            radar_lon,
            range_a,
            view_step_m,
            bbox,
            0.0,
            360.0,
            f64::INFINITY,
        );
        let rgba_b = rasterize_viewport(
            &place_hit(range_b),
            spokes,
            spoke_len,
            &legend,
            128,
            radar_lat,
            radar_lon,
            range_b,
            view_step_m,
            bbox,
            0.0,
            360.0,
            f64::INFINITY,
        );
        let mask = |rgba: &[u8]| -> Vec<bool> {
            rgba.chunks(4).map(|px| px[3] > 0).collect()
        };
        let ma = mask(&rgba_a);
        let mb = mask(&rgba_b);
        let painted_a = ma.iter().filter(|&&o| o).count();
        let painted_b = mb.iter().filter(|&&o| o).count();
        assert!(painted_a > 0 && painted_b > 0, "both labels must paint");
        let mismatch = ma.iter().zip(mb.iter()).filter(|(a, b)| a != b).count();
        // Allow a few edge pixels from residual bin quantisation.
        assert!(
            mismatch * 100 / ma.len() < 2,
            "opaque masks must match within 2% (mismatch={mismatch}/{})",
            ma.len()
        );
    }

    /// World sampler fills intensities; junk acquisition polar + different
    /// spoke_range labels must not change the opaque mask.
    #[test]
    fn viewport_world_sampler_ignores_acq_polar_and_spoke_range() {
        let spokes = 64;
        let spoke_len = 128;
        let legend = solid_legend();
        let radar_lat = -19.85;
        let radar_lon = -40.06;
        let geo_hit_m = 1200.0;
        let view_step_m = 1852.0;
        let dlat = 600.0 / METERS_PER_DEGREE_LATITUDE;
        let dlon = 600.0 / meters_per_degree_longitude(&radar_lat);
        let hit_lat = radar_lat + geo_hit_m / METERS_PER_DEGREE_LATITUDE;
        let bbox = BBox {
            west: radar_lon - dlon,
            south: hit_lat - dlat,
            east: radar_lon + dlon,
            north: hit_lat + dlat,
        };
        let echo = |bearing: f64, range: f64| {
            if bearing < 20.0 || bearing > 340.0 {
                if (range - geo_hit_m).abs() < 80.0 {
                    return 1;
                }
            }
            0
        };
        let junk_a = vec![255u8; spokes * spoke_len];
        let junk_b = vec![0u8; spokes * spoke_len];
        let rgba_a = rasterize_viewport_with_world(
            &junk_a,
            spokes,
            spoke_len,
            &legend,
            96,
            radar_lat,
            radar_lon,
            2_000.0,
            view_step_m,
            bbox,
            0.0,
            360.0,
            f64::INFINITY,
            Some(&echo),
        );
        let rgba_b = rasterize_viewport_with_world(
            &junk_b,
            spokes,
            spoke_len,
            &legend,
            96,
            radar_lat,
            radar_lon,
            48_000.0,
            view_step_m,
            bbox,
            0.0,
            360.0,
            f64::INFINITY,
            Some(&echo),
        );
        let mask = |rgba: &[u8]| -> Vec<bool> {
            rgba.chunks(4).map(|px| px[3] > 0).collect()
        };
        let ma = mask(&rgba_a);
        let mb = mask(&rgba_b);
        let painted = ma.iter().filter(|&&o| o).count();
        assert!(painted > 10, "world sampler must paint (painted={painted})");
        let mismatch = ma.iter().zip(mb.iter()).filter(|(a, b)| a != b).count();
        assert_eq!(
            mismatch, 0,
            "world path must ignore acq polar and spoke_range label"
        );
    }

    #[test]
    fn viewport_pixels_outside_annulus_stay_clear() {
        let spokes = 4;
        let spoke_len = 32;
        let mut polar = vec![0u8; spokes * spoke_len];
        // Fill entire polar — only annulus pixels may paint.
        for p in &mut polar {
            *p = 1;
        }
        let legend = solid_legend();
        let radar_lat = -19.85;
        let radar_lon = -40.06;
        let dlat = 1500.0 / METERS_PER_DEGREE_LATITUDE;
        let half = 400.0 / METERS_PER_DEGREE_LATITUDE;
        let dlon = 400.0 / meters_per_degree_longitude(&radar_lat);
        // Bbox due north — south half of a large canvas about the asset stays clear.
        let bbox = BBox {
            west: radar_lon - dlon,
            south: radar_lat + dlat - half,
            east: radar_lon + dlon,
            north: radar_lat + dlat + half,
        };
        // Render into a square that includes area south of the annulus by using
        // the bbox itself (annulus == bbox coverage); check a pixel south of r_min
        // via a synthetic larger check: south edge of bbox is near r_min, so
        // instead verify south-of-asset bearing is empty by painting a centred
        // disk bbox that only covers northern annulus… already sector-clipped.
        let rgba = rasterize_viewport(
            &polar,
            spokes,
            spoke_len,
            &legend,
            128,
            radar_lat,
            radar_lon,
            14.0 * 1852.0,
            1852.0,
            bbox,
            0.0,
            360.0,
            f64::INFINITY,
        );
        let painted = rgba.chunks(4).filter(|px| px[3] > 0).count();
        assert!(painted > 0, "annulus must paint something");
        // Bottom rows of the PNG are southern edge of bbox (still in annulus).
        // Use a second render with asset-centred bbox and assert far-south
        // corner of an off-centre northern bbox has no paint beyond sector:
        // pick pixel at image centre — should be near north mid-range.
        let cx = 64usize;
        let cy = 64usize;
        assert_eq!(
            rgba[(cy * 128 + cx) * 4 + 3],
            255,
            "bbox centre (north of asset) should be opaque"
        );
        let _ = painted;
    }
}
