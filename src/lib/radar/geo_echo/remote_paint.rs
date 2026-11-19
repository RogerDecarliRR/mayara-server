//! Viewport paint from a decoded polar blob (no world sampler).
//!
//! Used by `geo-echo-renderer` so heavy raster work can run out-of-process.

use super::annulus::{
    bbox_half_min_span_m, snap_range_step_up, view_m_per_bin, VIEWPORT_RANGE_STEPS_M,
};
use super::corners::refresh_interval_ms;
use super::frame::{now_unix_ms, Center, EchoFrame, EchoMeta, SizePx};
use super::png::encode_rgba;
use super::polar_blob::DecodedPolar;
use super::raster::{
    effective_viewport_max_size_px, rasterize_viewport, viewport_image_corners, viewport_size_px,
    BBox,
};
use crate::config::{GeoEchoConfig, GEO_ECHO_MIN_TARGET_M_PER_PX};

/// Inputs for painting a map viewport from polar data.
#[derive(Clone, Debug)]
pub struct ViewportPaintRequest {
    pub west: f64,
    pub south: f64,
    pub east: f64,
    pub north: f64,
    /// Antenna site; both set or both omitted.
    pub radar_lat: Option<f64>,
    pub radar_lon: Option<f64>,
    pub usable_bearing_start_deg: f64,
    pub usable_bearing_end_deg: f64,
    pub usable_radius_m: f64,
    pub max_size_px: u32,
    pub target_meters_per_px: f64,
    /// Range-step snap ceiling; `None` → [`GeoEchoConfig::default`] ceiling.
    pub max_viewport_range_m: Option<f64>,
    /// Optional paint max edge (`sizeHintPx` / `sizeHint`); caps `max_size_px` (W2 A3/C1).
    pub size_hint: Option<u32>,
}

impl ViewportPaintRequest {
    /// Fill paint knobs from [`GeoEchoConfig`] (defaults when omitted by caller).
    pub fn from_bbox_and_config(
        west: f64,
        south: f64,
        east: f64,
        north: f64,
        radar_lat: Option<f64>,
        radar_lon: Option<f64>,
        geo: &GeoEchoConfig,
    ) -> Self {
        Self {
            west,
            south,
            east,
            north,
            radar_lat,
            radar_lon,
            usable_bearing_start_deg: geo.usable_bearing_start_deg,
            usable_bearing_end_deg: geo.usable_bearing_end_deg,
            usable_radius_m: geo.effective_usable_radius_m(),
            max_size_px: geo.max_size_px,
            target_meters_per_px: geo.target_meters_per_px,
            max_viewport_range_m: geo.max_viewport_range_m,
            size_hint: None,
        }
    }
}

/// Painted viewport: SignalK-compatible meta + PNG bytes.
#[derive(Clone, Debug)]
pub struct ViewportPaintResult {
    pub meta: EchoMeta,
    pub png: Vec<u8>,
    pub meters_per_px: f64,
}

/// Rasterize `polar` into a north-up viewport PNG (no world sampler).
///
/// Mirrors [`super::compositor::publish_viewport`] geometry / snap logic with
/// [`DecodedPolar`] instead of a live [`super::compositor::PolarSnapshot`].
pub fn paint_viewport(
    polar: &DecodedPolar,
    req: &ViewportPaintRequest,
) -> Result<ViewportPaintResult, String> {
    let bbox = BBox {
        west: req.west,
        south: req.south,
        east: req.east,
        north: req.north,
    };
    bbox.validate()?;

    let (span_ew, span_ns) = bbox.spans_m();
    let half_min_m = bbox_half_min_span_m(bbox);

    let ceiling = match req.max_viewport_range_m {
        Some(m) if m.is_finite() && m > 0.0 => m,
        _ => GeoEchoConfig::default().max_viewport_range_ceiling(),
    };
    let view_step_m = snap_range_step_up(half_min_m, VIEWPORT_RANGE_STEPS_M, ceiling);

    let spokes = (polar.spokes as usize).max(1);
    let spoke_len = (polar.spoke_len as usize).max(1);

    let m_bin = view_m_per_bin(view_step_m, spoke_len);
    let effective_target = req
        .target_meters_per_px
        .min(m_bin.max(GEO_ECHO_MIN_TARGET_M_PER_PX));
    // A3/C1: sizeHintPx → max_size_for_paint (snapped to allowed ≤ hint).
    let max_for_paint = effective_viewport_max_size_px(req.max_size_px, req.size_hint);
    let size_px = viewport_size_px(span_ew, span_ns, effective_target, max_for_paint);
    let refresh = refresh_interval_ms(polar.period_ms);
    let meters_per_px = {
        let span = span_ew.max(span_ns).max(1e-6);
        span / size_px.max(1) as f64
    };

    let site = match (req.radar_lat, req.radar_lon) {
        (Some(lat), Some(lon)) => Some((lat, lon)),
        (None, None) => None,
        _ => {
            return Err("radar_lat and radar_lon must both be set or both omitted".into());
        }
    };
    let Some((lat, lon)) = site.or(polar.center) else {
        let mut cold = EchoFrame::unavailable_sized(size_px);
        cold.meta.revolution_id = polar.revolution_id;
        cold.meta.refresh_interval_ms = refresh;
        cold.meta.corners = viewport_image_corners(bbox, size_px);
        return Ok(ViewportPaintResult {
            meta: cold.meta,
            png: cold.png.to_vec(),
            meters_per_px,
        });
    };

    let rgba = rasterize_viewport(
        &polar.data,
        spokes,
        spoke_len,
        &polar.legend,
        size_px,
        lat,
        lon,
        polar.spoke_range_m,
        view_step_m,
        bbox,
        req.usable_bearing_start_deg,
        req.usable_bearing_end_deg,
        req.usable_radius_m,
    );
    let png = encode_rgba(size_px, size_px, &rgba)
        .map_err(|e| format!("png encode failed: {e}"))?;

    let corners = viewport_image_corners(bbox, size_px);
    let meta = EchoMeta {
        version: 1,
        available: true,
        generated_ms: now_unix_ms(),
        revolution_id: polar.revolution_id,
        refresh_interval_ms: refresh,
        center: Center {
            lat: (bbox.south + bbox.north) * 0.5,
            lon: (bbox.west + bbox.east) * 0.5,
        },
        range_m: view_step_m,
        north_up: true,
        size_px: SizePx {
            width: size_px,
            height: size_px,
        },
        corners,
    };

    Ok(ViewportPaintResult {
        meta,
        png,
        meters_per_px,
    })
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
                        g: 0,
                        b: 0,
                        a: 255,
                    },
                },
            ],
            static_background: None,
        }
    }

    fn tiny_polar_filled() -> DecodedPolar {
        let spokes = 4u32;
        let spoke_len = 16u32;
        let mut data = vec![0u8; (spokes * spoke_len) as usize];
        // North spoke mid-range hit.
        data[spoke_len as usize / 2] = 1;
        DecodedPolar {
            revolution_id: 9,
            period_ms: 1500,
            spokes,
            spoke_len,
            spoke_range_m: 2_000.0,
            center: Some((-19.85, -40.06)),
            legend: solid_legend(),
            data,
        }
    }

    #[test]
    fn paint_viewport_marks_available_with_site() {
        let polar = tiny_polar_filled();
        let site_lat = -19.85;
        let site_lon = -40.06;
        let req = ViewportPaintRequest {
            west: site_lon - 0.02,
            south: site_lat - 0.02,
            east: site_lon + 0.02,
            north: site_lat + 0.02,
            radar_lat: Some(site_lat),
            radar_lon: Some(site_lon),
            usable_bearing_start_deg: 0.0,
            usable_bearing_end_deg: 360.0,
            usable_radius_m: 14.0 * 1852.0,
            max_size_px: 256,
            target_meters_per_px: 20.0,
            max_viewport_range_m: Some(14.0 * 1852.0),
            size_hint: None,
        };
        let result = paint_viewport(&polar, &req).expect("paint");
        assert!(result.meta.available);
        assert_eq!(result.meta.revolution_id, 9);
        assert_eq!(result.meta.size_px.width, result.meta.size_px.height);
        assert!(!result.png.is_empty());
        assert_eq!(&result.png[0..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
        assert!(result.meters_per_px > 0.0);
    }

    #[test]
    fn paint_viewport_unavailable_without_center() {
        let mut polar = tiny_polar_filled();
        polar.center = None;
        let req = ViewportPaintRequest {
            west: -40.08,
            south: -19.87,
            east: -40.04,
            north: -19.83,
            radar_lat: None,
            radar_lon: None,
            usable_bearing_start_deg: 0.0,
            usable_bearing_end_deg: 360.0,
            usable_radius_m: 14.0 * 1852.0,
            max_size_px: 128,
            target_meters_per_px: 50.0,
            max_viewport_range_m: None,
            size_hint: None,
        };
        let result = paint_viewport(&polar, &req).expect("paint");
        assert!(!result.meta.available);
        assert_eq!(result.meta.revolution_id, 9);
        assert!(!result.png.is_empty());
    }

    #[test]
    fn paint_rejects_partial_radar_site() {
        let polar = tiny_polar_filled();
        let req = ViewportPaintRequest {
            west: -40.08,
            south: -19.87,
            east: -40.04,
            north: -19.83,
            radar_lat: Some(-19.85),
            radar_lon: None,
            usable_bearing_start_deg: 0.0,
            usable_bearing_end_deg: 360.0,
            usable_radius_m: 1000.0,
            max_size_px: 128,
            target_meters_per_px: 50.0,
            max_viewport_range_m: None,
            size_hint: None,
        };
        let err = paint_viewport(&polar, &req).unwrap_err();
        assert!(err.contains("radar_lat"));
    }

    #[test]
    fn size_hint_caps_max_to_half_allowed() {
        // Client first-pan hint = half of default 2048 → 1024 allowed.
        assert_eq!(effective_viewport_max_size_px(2048, Some(1024)), 1024);
        assert_eq!(effective_viewport_max_size_px(2048, Some(1500)), 1024);
        assert_eq!(effective_viewport_max_size_px(2048, Some(2048)), 2048);
        assert_eq!(effective_viewport_max_size_px(2048, None), 2048);

        let polar = tiny_polar_filled();
        let site_lat = -19.85;
        let site_lon = -40.06;
        // Huge bbox + fine target would hit max; hint must cap the edge.
        let req_full = ViewportPaintRequest {
            west: site_lon - 0.05,
            south: site_lat - 0.05,
            east: site_lon + 0.05,
            north: site_lat + 0.05,
            radar_lat: Some(site_lat),
            radar_lon: Some(site_lon),
            usable_bearing_start_deg: 0.0,
            usable_bearing_end_deg: 360.0,
            usable_radius_m: 14.0 * 1852.0,
            max_size_px: 2048,
            target_meters_per_px: 1.0,
            max_viewport_range_m: Some(14.0 * 1852.0),
            size_hint: None,
        };
        let full = paint_viewport(&polar, &req_full).expect("full");
        let mut req_half = req_full.clone();
        req_half.size_hint = Some(1024);
        let half = paint_viewport(&polar, &req_half).expect("half");
        assert!(full.meta.size_px.width >= half.meta.size_px.width);
        assert!(half.meta.size_px.width <= 1024);
        assert!(!half.png.is_empty());
    }

    #[test]
    fn rayon_paint_small_grid_ok() {
        let polar = tiny_polar_filled();
        let req = ViewportPaintRequest {
            west: -40.07,
            south: -19.86,
            east: -40.05,
            north: -19.84,
            radar_lat: Some(-19.85),
            radar_lon: Some(-40.06),
            usable_bearing_start_deg: 0.0,
            usable_bearing_end_deg: 360.0,
            usable_radius_m: 2000.0,
            max_size_px: 64,
            target_meters_per_px: 50.0,
            max_viewport_range_m: Some(2000.0),
            size_hint: None,
        };
        let result = paint_viewport(&polar, &req).expect("small paint");
        assert_eq!(result.meta.size_px.width, result.meta.size_px.height);
        assert!(result.meta.size_px.width >= 1);
        assert!(!result.png.is_empty());
    }
}
