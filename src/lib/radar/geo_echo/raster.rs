//! Polar buffer → north-up RGBA raster (echo only).

use super::super::target::{METERS_PER_DEGREE_LATITUDE, meters_per_degree_longitude};
use super::super::Legend;
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

/// Paint one spoke into an RGBA buffer. Bearing 0 = north, clockwise; image
/// y decreases northward (MapLibre north-up).
///
/// `scale` maps spoke range onto the composition canvas
/// (`spoke_range_m / composition_range_m`). Values &lt; 1 paint an interior
/// circle; values &gt; 1 clip at the canvas edge.
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
    );
}

/// Like [`paint_spoke_rgba`] but skips the spoke when its bearing is outside
/// the usable sector.
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
/// `composition_range_m`. Full bearing circle (compat wrapper).
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
    )
}

/// Composition raster with a usable-bearing mask.
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

/// Project polar samples into a north-up square covering `bbox` (uniform m/px
/// from the larger span, clipped to the bbox). Bearing mask applied.
pub fn rasterize_viewport(
    polar: &[u8],
    spokes: usize,
    spoke_len: usize,
    legend: &Legend,
    size_px: u32,
    radar_lat: f64,
    radar_lon: f64,
    spoke_range_m: f64,
    bbox: BBox,
    usable_bearing_start_deg: f64,
    usable_bearing_end_deg: f64,
) -> Vec<u8> {
    let size = size_px.max(1) as usize;
    let mut rgba = vec![0u8; size * size * 4];
    if spokes == 0 || spoke_len == 0 || polar.len() < spokes * spoke_len || spoke_range_m <= 0.0 {
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
    let m_per_deg_lon = meters_per_degree_longitude(&bbox_lat);

    for spoke_i in 0..spokes {
        let bearing_deg = (spoke_i as f64) * 360.0 / (spokes as f64);
        if !bearing_in_sector(
            bearing_deg,
            usable_bearing_start_deg,
            usable_bearing_end_deg,
        ) {
            continue;
        }
        let angle = bearing_deg.to_radians();
        let sin_a = angle.sin();
        let cos_a = angle.cos();
        let start = spoke_i * spoke_len;
        let data = &polar[start..start + spoke_len];
        for (r, &pix) in data.iter().enumerate() {
            if pix == 0 {
                continue;
            }
            let Some(lookup) = legend.pixels.get(pix as usize) else {
                continue;
            };
            let range_m = (r as f64 + 0.5) / (spoke_len as f64) * spoke_range_m;
            let east_m = range_m * sin_a;
            let north_m = range_m * cos_a;
            let lat = radar_lat + north_m / METERS_PER_DEGREE_LATITUDE;
            let lon = radar_lon + east_m / meters_per_degree_longitude(&radar_lat);
            if lat < bbox.south || lat > bbox.north || lon < bbox.west || lon > bbox.east {
                continue;
            }
            // ENU relative to bbox centre → pixel in square covering max span.
            let dx_m = (lon - bbox_lon) * m_per_deg_lon;
            let dy_m = (lat - bbox_lat) * METERS_PER_DEGREE_LATITUDE;
            if dx_m.abs() > half_m + m_per_px || dy_m.abs() > half_m + m_per_px {
                continue;
            }
            let x = ((dx_m + half_m) / m_per_px).round() as isize;
            let y = ((half_m - dy_m) / m_per_px).round() as isize;
            if x < 0 || y < 0 {
                continue;
            }
            let (x, y) = (x as usize, y as usize);
            if x >= size || y >= size {
                continue;
            }
            let i = (y * size + x) * 4;
            let color = &lookup.color;
            rgba[i] = color.r;
            rgba[i + 1] = color.g;
            rgba[i + 2] = color.b;
            rgba[i + 3] = color.a;
        }
    }
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
            &polar, spokes, spoke_len, &legend, size, 1000.0, 1000.0, 315.0, 45.0,
        );
        let cx = (size / 2) as usize;
        let north_painted = (0..cx).any(|y| rgba[(y * size as usize + cx) * 4 + 3] == 255);
        let south_painted = ((cx + 1)..size as usize)
            .any(|y| rgba[(y * size as usize + cx) * 4 + 3] == 255);
        assert!(north_painted, "north spoke in sector should paint");
        assert!(!south_painted, "south spoke outside sector must stay empty");
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
    fn viewport_raster_clips_outside_bbox() {
        let spokes = 1;
        let spoke_len = 32;
        let polar = vec![1u8; spokes * spoke_len];
        let legend = solid_legend();
        let radar_lat = -19.85;
        let radar_lon = -40.06;
        // Tiny bbox well south of radar — all samples should miss.
        let bbox = BBox {
            west: radar_lon - 0.0001,
            south: radar_lat - 0.05,
            east: radar_lon + 0.0001,
            north: radar_lat - 0.04,
        };
        let rgba = rasterize_viewport(
            &polar, spokes, spoke_len, &legend, 64, radar_lat, radar_lon, 500.0, bbox, 0.0, 360.0,
        );
        assert!(rgba.iter().all(|&b| b == 0));
    }
}
