//! Subscribes to radar spokes and publishes a north-up geo echo frame.

use super::super::settings::{ControlId, ControlValue};
use super::super::{Legend, RadarInfo, SharedRadars};
use super::corners::{corners_for, refresh_interval_ms};
use super::frame::{now_unix_ms, Center, EchoFrame, EchoMeta, SizePx};
use super::png::encode_rgba;
use super::annulus::{
    bbox_half_min_span_m, snap_range_step_up, view_m_per_bin, VIEWPORT_RANGE_STEPS_M,
};
use super::raster::{
    rasterize_polar_masked, rasterize_viewport, rasterize_viewport_with_world,
    view_range_from_bbox, viewport_image_corners, viewport_size_px, BBox,
};
use super::state::{PolarBuffer, RevClock};
use crate::navdata;
use crate::protos::RadarMessage::RadarMessage;
use bytes::Bytes;
use log::{debug, warn};
use protobuf::Message;
use std::collections::HashMap;
use std::sync::{Arc, LazyLock, Mutex, RwLock};
use tokio::sync::broadcast;

/// Metres: treat Range as already aligned when within this of the snapped target.
const RANGE_ALIGN_EPS_M: f64 = 0.5;

/// Latest polar snapshot for on-demand viewport renders.
#[derive(Clone)]
pub struct PolarSnapshot {
    pub data: Vec<u8>,
    pub spokes: usize,
    pub spoke_len: usize,
    pub center: Option<(f64, f64)>,
    pub spoke_range_m: f64,
    pub legend: Legend,
    pub revolution_id: u64,
    pub period_ms: u64,
}

impl PolarSnapshot {
    fn empty(legend: Legend) -> Self {
        Self {
            data: Vec::new(),
            spokes: 0,
            spoke_len: 0,
            center: None,
            spoke_range_m: 0.0,
            legend,
            revolution_id: 0,
            period_ms: 2_000,
        }
    }
}

struct RegistryEntry {
    frame: Arc<RwLock<EchoFrame>>,
    viewport: Arc<RwLock<EchoFrame>>,
    snapshot: Arc<RwLock<PolarSnapshot>>,
}

static REGISTRY: LazyLock<Mutex<HashMap<String, RegistryEntry>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// Start the geo-echo compositor for `radar_id` on first request (lazy).
/// Returns `None` when the radar key is unknown.
pub fn ensure_running(radars: &SharedRadars, radar_id: &str) -> Option<Arc<RwLock<EchoFrame>>> {
    let (frame, _, _) = ensure_entry(radars, radar_id)?;
    Some(frame)
}

/// Composition + viewport frame slots and polar snapshot for `radar_id`.
#[allow(clippy::type_complexity)]
pub fn ensure_entry(
    radars: &SharedRadars,
    radar_id: &str,
) -> Option<(
    Arc<RwLock<EchoFrame>>,
    Arc<RwLock<EchoFrame>>,
    Arc<RwLock<PolarSnapshot>>,
)> {
    {
        let reg = REGISTRY.lock().unwrap();
        if let Some(entry) = reg.get(radar_id) {
            return Some((
                Arc::clone(&entry.frame),
                Arc::clone(&entry.viewport),
                Arc::clone(&entry.snapshot),
            ));
        }
    }

    let info = radars.get_by_key(radar_id)?;
    let size_px = radars.geo_echo_config().echo_size_px;
    let frame = Arc::new(RwLock::new(EchoFrame::unavailable_sized(size_px)));
    let viewport = Arc::new(RwLock::new(EchoFrame::unavailable_sized(size_px)));
    let snapshot = Arc::new(RwLock::new(PolarSnapshot::empty(info.get_legend())));

    {
        let mut reg = REGISTRY.lock().unwrap();
        if let Some(entry) = reg.get(radar_id) {
            return Some((
                Arc::clone(&entry.frame),
                Arc::clone(&entry.viewport),
                Arc::clone(&entry.snapshot),
            ));
        }
        reg.insert(
            radar_id.to_string(),
            RegistryEntry {
                frame: Arc::clone(&frame),
                viewport: Arc::clone(&viewport),
                snapshot: Arc::clone(&snapshot),
            },
        );
    }

    let key = radar_id.to_string();
    let frame_task = Arc::clone(&frame);
    let snapshot_task = Arc::clone(&snapshot);
    let radars_task = radars.clone();
    tokio::spawn(async move {
        run_compositor(key, info, radars_task, frame_task, snapshot_task).await;
    });

    Some((frame, viewport, snapshot))
}

/// Render a viewport frame from the latest polar snapshot into the viewport
/// slot. Returns the published meta, or an error string.
///
/// `site`: optional antenna (lat, lon). When `Some`, used as `radar_lat`/`radar_lon`
/// in [`rasterize_viewport`] instead of the polar snapshot centre — so a polar
/// buffer still tagged at Sneek can paint into a port bbox.
pub fn publish_viewport(
    radars: &SharedRadars,
    radar_id: &str,
    bbox: BBox,
    site: Option<(f64, f64)>,
) -> Result<EchoMeta, String> {
    bbox.validate()?;
    let (_, viewport_slot, snapshot_slot) = ensure_entry(radars, radar_id)
        .ok_or_else(|| format!("radar '{radar_id}' not found"))?;

    let snap = snapshot_slot
        .read()
        .map_err(|_| "viewport snapshot lock poisoned".to_string())?
        .clone();

    let geo = radars.geo_echo_config();
    // Acquisition horizon (ARPA/blob): spokes only exist out to Control Range.
    // Keep Range on composition_range_m in viewport mode too — map zoom must
    // not shrink the detector's horizon (PNG pitch still uses view_step snap).
    ensure_composition_radar_range(radars, radar_id, geo.composition_range_m);

    let (span_ew, span_ns) = bbox.spans_m();
    // Snap metric = half of the smaller span (not diagonal — that over-snapped).
    let half_min_m = bbox_half_min_span_m(bbox);
    let view_half_m = view_range_from_bbox(bbox);
    let max_m = geo.max_viewport_range_ceiling();
    let view_step_m = snap_range_step_up(half_min_m, VIEWPORT_RANGE_STEPS_M, max_m);

    // Prefer live radar geometry; snapshot may still be empty pre-revolution.
    let (spokes, spoke_len, legend_live) = radars
        .get_by_key(radar_id)
        .map(|info| {
            (
                info.spokes_per_revolution.max(1) as usize,
                info.max_spoke_len.max(1) as usize,
                info.get_legend(),
            )
        })
        .unwrap_or((
            snap.spokes.max(1),
            snap.spoke_len.max(1),
            snap.legend.clone(),
        ));

    // size_px after snap: effective target ≤ m_bin (floor GEO_ECHO_MIN_TARGET_M_PER_PX)
    // so fine zoom yields a denser PNG instead of stretching a ~92px raster.
    let m_bin = view_m_per_bin(view_step_m, spoke_len);
    let effective_target = geo
        .target_meters_per_px
        .min(m_bin.max(crate::config::GEO_ECHO_MIN_TARGET_M_PER_PX));
    let size_px = viewport_size_px(span_ew, span_ns, effective_target, geo.max_size_px);
    let refresh = refresh_interval_ms(snap.period_ms);
    let meters_per_px = {
        let span = span_ew.max(span_ns).max(1e-6);
        span / size_px.max(1) as f64
    };

    let Some((lat, lon)) =
        resolve_viewport_site(site, snap.center, resolve_nav_center())
    else {
        let mut cold = EchoFrame::unavailable_sized(size_px);
        cold.meta.revolution_id = snap.revolution_id;
        cold.meta.refresh_interval_ms = refresh;
        cold.meta.corners = viewport_image_corners(bbox, size_px);
        if let Ok(mut slot) = viewport_slot.write() {
            *slot = cold.clone();
        }
        return Ok(cold.meta);
    };

    // Annular inverse-sample at snapped view_step. Range is forced above for
    // ARPA acquisition; PNG pitch still follows view_step. Emulator / world
    // sampler fills intensities from geography — same bbox+asset ⇒ same PNG
    // regardless of the operator's previous Range dial.
    let world = radars.viewport_world_sampler(radar_id);
    let rgba = if let Some(ref sampler) = world {
        sampler.prepare();
        let echo = |bearing: f64, range: f64| sampler.echo_at(bearing, range);
        rasterize_viewport_with_world(
            &[],
            spokes,
            spoke_len,
            &legend_live,
            size_px,
            lat,
            lon,
            0.0,
            view_step_m,
            bbox,
            geo.usable_bearing_start_deg,
            geo.usable_bearing_end_deg,
            Some(&echo),
        )
    } else {
        rasterize_viewport(
            &snap.data,
            snap.spokes.max(1),
            snap.spoke_len.max(1),
            &snap.legend,
            size_px,
            lat,
            lon,
            snap.spoke_range_m,
            view_step_m,
            bbox,
            geo.usable_bearing_start_deg,
            geo.usable_bearing_end_deg,
        )
    };
    let png_bytes = encode_rgba(size_px, size_px, &rgba)
        .map_err(|e| format!("png encode failed: {e}"))?;

    let corners = viewport_image_corners(bbox, size_px);
    let meta = EchoMeta {
        version: 1,
        available: true,
        generated_ms: now_unix_ms(),
        revolution_id: snap.revolution_id,
        refresh_interval_ms: refresh,
        center: Center {
            lat: (bbox.south + bbox.north) * 0.5,
            lon: (bbox.west + bbox.east) * 0.5,
        },
        // Snapped Mayara range step (half_min metric) — client shows this degrau.
        range_m: view_step_m,
        north_up: true,
        size_px: SizePx {
            width: size_px,
            height: size_px,
        },
        corners,
    };
    if let Ok(mut slot) = viewport_slot.write() {
        *slot = EchoFrame {
            meta: meta.clone(),
            png: Bytes::from(png_bytes),
        };
    }
    debug!(
        "geo_echo {radar_id}: viewport size={size_px} m_per_px={meters_per_px:.3} \
         view_step_m={view_step_m:.1} view_half_m={view_half_m:.1} spoke_range_m={:.1} rev={}",
        snap.spoke_range_m,
        snap.revolution_id
    );
    Ok(meta)
}

/// Read the last viewport frame (transparent / unavailable until first POST).
pub fn viewport_frame(radar_id: &str) -> Option<Arc<RwLock<EchoFrame>>> {
    let reg = REGISTRY.lock().unwrap();
    reg.get(radar_id).map(|e| Arc::clone(&e.viewport))
}

async fn run_compositor(
    key: String,
    info: RadarInfo,
    radars: SharedRadars,
    frame: Arc<RwLock<EchoFrame>>,
    snapshot: Arc<RwLock<PolarSnapshot>>,
) {
    info.wake_up();
    let mut rx: broadcast::Receiver<Bytes> = info.message_tx.subscribe();
    info.wake_up();
    radars.hold_up_antenna(&key);

    let spokes = info.spokes_per_revolution as usize;
    let spoke_len = info.max_spoke_len.max(1) as usize;
    let mut polar = PolarBuffer::new(spokes, spoke_len);
    let mut clock = RevClock::new();
    let mut legend = info.get_legend();

    debug!("geo_echo compositor started for {key}");

    loop {
        let wait = clock.wait_remaining();
        let sleep_deadline = wait.map(|d| tokio::time::Instant::now() + d);
        tokio::select! {
            biased;
            _ = async {
                match sleep_deadline {
                    Some(deadline) => tokio::time::sleep_until(deadline).await,
                    None => std::future::pending::<()>().await,
                }
            } => {
                if clock.should_publish_now() {
                    publish_frame(&key, &radars, &polar, &mut clock, &legend, &frame, &snapshot);
                }
            }
            msg = rx.recv() => {
                match msg {
                    Ok(bytes) => {
                        if let Ok(message) = RadarMessage::parse_from_bytes(&bytes) {
                            ingest_message(
                                &key,
                                &radars,
                                &message,
                                &mut polar,
                                &mut clock,
                                &mut legend,
                                &frame,
                                &snapshot,
                            );
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        debug!("geo_echo {key}: lagged by {n} messages");
                    }
                    Err(broadcast::error::RecvError::Closed) => {
                        warn!("geo_echo {key}: message channel closed");
                        break;
                    }
                }
            }
        }
    }
}

fn ingest_message(
    key: &str,
    radars: &SharedRadars,
    message: &RadarMessage,
    polar: &mut PolarBuffer,
    clock: &mut RevClock,
    legend: &mut Legend,
    frame: &Arc<RwLock<EchoFrame>>,
    snapshot: &Arc<RwLock<PolarSnapshot>>,
) {
    for spoke in &message.spokes {
        let bearing_idx = spoke
            .bearing
            .map(|b| b as usize)
            .unwrap_or(spoke.angle as usize);
        let spokes = polar.spokes;
        let idx = bearing_idx % spokes.max(1);

        if let (Some(lat), Some(lon)) = (spoke.lat, spoke.lon) {
            polar.last_center = Some((lat, lon));
        }
        if spoke.range > 0 {
            polar.last_range_m = spoke.range as f64;
        }

        // Wrap closes the previous revolution before overwriting its spoke slot.
        if clock.is_wrap(idx) {
            clock.close_revolution();
            refresh_legend(key, radars, polar, legend, spoke.data.len());
            if clock.should_publish_now() {
                publish_frame(key, radars, polar, clock, legend, frame, snapshot);
            }
        }

        polar.write_spoke(idx, &spoke.data);

        if clock.after_spoke_written(idx, spokes) {
            refresh_legend(key, radars, polar, legend, spoke.data.len());
            if clock.should_publish_now() {
                publish_frame(key, radars, polar, clock, legend, frame, snapshot);
            }
        }
    }
}

fn refresh_legend(
    key: &str,
    radars: &SharedRadars,
    polar: &mut PolarBuffer,
    legend: &mut Legend,
    spoke_data_len: usize,
) {
    if let Some(live) = radars.get_by_key(key) {
        *legend = live.get_legend();
        let s = live.spokes_per_revolution as usize;
        let sl = live.max_spoke_len.max(1) as usize;
        polar.resize_if_needed(s, sl.max(spoke_data_len.max(1)));
    }
}

fn resolve_center(polar: &PolarBuffer) -> Option<(f64, f64)> {
    if let Some(c) = polar.last_center {
        return Some(c);
    }
    resolve_nav_center()
}

fn resolve_nav_center() -> Option<(f64, f64)> {
    if let Some(p) = navdata::get_radar_position() {
        return Some((p.lat(), p.lon()));
    }
    let (lat, lon) = navdata::get_position();
    match (lat, lon) {
        (Some(la), Some(lo)) => Some((la, lo)),
        _ => None,
    }
}

/// Physical range of the current spoke data (for raster scale only — never
/// used as `echo.json` `rangeM`).
fn resolve_spoke_range_m(radars: &SharedRadars, key: &str, polar: &PolarBuffer) -> f64 {
    if polar.last_range_m > 0.0 {
        return polar.last_range_m;
    }
    if let Some(info) = radars.get_by_key(key)
        && let Some(control) = info.controls.get(&ControlId::Range)
        && let Some(v) = control.value()
        && let Some(f) = v.as_f64()
        && f > 0.0
    {
        return f;
    }
    0.0
}

/// Nearest settable Range (metres) for `target_m`, or `target_m` if unconstrained.
/// Mirrors `Control::snap_to_valid_value` so composition align matches the GUI path.
pub(crate) fn nearest_valid_range_m(valid: Option<&[i32]>, target_m: f64) -> f64 {
    let Some(valid) = valid else {
        return target_m;
    };
    valid
        .iter()
        .min_by(|a, b| {
            let da = (**a as f64 - target_m).abs();
            let db = (**b as f64 - target_m).abs();
            da.total_cmp(&db).then(a.cmp(b))
        })
        .map(|v| *v as f64)
        .unwrap_or(target_m)
}

fn range_needs_align(current_m: Option<f64>, desired_m: f64) -> bool {
    match current_m {
        Some(v) => (v - desired_m).abs() > RANGE_ALIGN_EPS_M,
        None => true,
    }
}

/// Keep radar `ControlId::Range` on `composition_range_m` whenever geo-echo is
/// active (composition **and** viewport).
///
/// ARPA / blob acquisition only sees spokes out to the GUI Control Range; if
/// viewport left Range on the operator zoom, detection would be clipped short
/// of the configured horizon. Composition also needs full-disk spoke cover
/// (`scale ≈ spoke_range/composition`). Viewport PNG pitch still uses the
/// annular `view_step` snap — this only widens hardware/emulator acquisition.
pub(crate) fn ensure_composition_radar_range(
    radars: &SharedRadars,
    key: &str,
    composition_range_m: f64,
) {
    if composition_range_m <= 0.0 {
        return;
    }
    let Some(info) = radars.get_by_key(key) else {
        return;
    };
    let Some(control) = info.controls.get(&ControlId::Range) else {
        return;
    };
    let desired_m = nearest_valid_range_m(
        control.item().valid_values.as_deref(),
        composition_range_m,
    );
    let current_m = control.value().and_then(|v| v.as_f64());
    if !range_needs_align(current_m, desired_m) {
        return;
    }
    // Same path the GUI uses: snap + Command destination → brand set_control.
    let (reply_tx, _reply_rx) = tokio::sync::mpsc::channel(1);
    let cv = ControlValue::new(ControlId::Range, composition_range_m.into());
    match info.controls.process_client_request(cv, reply_tx) {
        Ok(()) => debug!(
            "geo_echo {key}: forced Range → {desired_m:.0} m (was {:?}; acquisition=composition_range_m)",
            current_m
        ),
        Err(e) => debug!("geo_echo {key}: Range align skipped: {e}"),
    }
}

fn store_snapshot(
    snapshot: &Arc<RwLock<PolarSnapshot>>,
    polar: &PolarBuffer,
    clock: &RevClock,
    legend: &Legend,
    spoke_range_m: f64,
) {
    if let Ok(mut slot) = snapshot.write() {
        *slot = PolarSnapshot {
            data: polar.data.clone(),
            spokes: polar.spokes,
            spoke_len: polar.spoke_len,
            center: polar.last_center.or_else(resolve_nav_center),
            spoke_range_m,
            legend: legend.clone(),
            revolution_id: clock.revolution_id,
            period_ms: clock.period_ms(),
        };
    }
}

fn publish_frame(
    key: &str,
    radars: &SharedRadars,
    polar: &PolarBuffer,
    clock: &mut RevClock,
    legend: &Legend,
    frame: &Arc<RwLock<EchoFrame>>,
    snapshot: &Arc<RwLock<PolarSnapshot>>,
) {
    let geo = radars.geo_echo_config();
    let composition_range_m = geo.composition_range_m;
    let size_px = geo.echo_size_px;
    let refresh = refresh_interval_ms(clock.period_ms());

    // Both modes: align GUI/hardware Range to compositionRangeM so spokes
    // (and ARPA/blob) cover the configured horizon, not the operator zoom.
    ensure_composition_radar_range(radars, key, composition_range_m);

    let spoke_range_m = resolve_spoke_range_m(radars, key, polar);

    store_snapshot(snapshot, polar, clock, legend, spoke_range_m);

    let Some((lat, lon)) = resolve_center(polar) else {
        let mut cold = EchoFrame::unavailable_sized(size_px);
        cold.meta.revolution_id = clock.revolution_id;
        cold.meta.refresh_interval_ms = refresh;
        if let Ok(mut slot) = frame.write() {
            *slot = cold;
        }
        clock.mark_published();
        return;
    };

    let rgba = rasterize_polar_masked(
        &polar.data,
        polar.spokes,
        polar.spoke_len,
        legend,
        size_px,
        spoke_range_m,
        composition_range_m,
        geo.usable_bearing_start_deg,
        geo.usable_bearing_end_deg,
    );
    let png_bytes = match encode_rgba(size_px, size_px, &rgba) {
        Ok(b) => b,
        Err(e) => {
            warn!("geo_echo {key}: png encode failed: {e}");
            return;
        }
    };

    let corners = corners_for(lat, lon, composition_range_m);
    let meta = EchoMeta {
        version: 1,
        available: true,
        generated_ms: now_unix_ms(),
        revolution_id: clock.revolution_id,
        refresh_interval_ms: refresh,
        center: Center { lat, lon },
        range_m: composition_range_m,
        north_up: true,
        size_px: SizePx {
            width: size_px,
            height: size_px,
        },
        corners,
    };

    if let Ok(mut slot) = frame.write() {
        *slot = EchoFrame {
            meta,
            png: Bytes::from(png_bytes),
        };
    }
    clock.mark_published();
    debug!(
        "geo_echo {key}: published rev {} refresh_ms={} composition_m={composition_range_m} size={size_px}",
        clock.revolution_id, refresh
    );
}

/// Prefer explicit antenna site over polar/nav centre (viewport remap).
pub(crate) fn resolve_viewport_site(
    site: Option<(f64, f64)>,
    snap_center: Option<(f64, f64)>,
    nav: Option<(f64, f64)>,
) -> Option<(f64, f64)> {
    site.or(snap_center).or(nav)
}

#[cfg(test)]
mod tests {
    use super::{nearest_valid_range_m, range_needs_align, resolve_viewport_site};

    #[test]
    fn nearest_valid_range_picks_closest_settable() {
        let valid = [7408, 11112, 14816, 25928];
        assert_eq!(nearest_valid_range_m(Some(&valid), 14.0 * 1852.0), 25928.0);
        assert_eq!(nearest_valid_range_m(Some(&valid), 9888.0), 11112.0);
        assert_eq!(nearest_valid_range_m(None, 1234.0), 1234.0);
    }

    /// Viewport + ARPA share the same acquisition horizon: composition_range_m
    /// (snapped to a settable Range). Operator zoom (e.g. 1 NM) must still
    /// realign — otherwise blob/ARPA only sees spokes inside the GUI zoom.
    #[test]
    fn range_align_skips_when_already_on_composition() {
        let composition_horizon_m = 25928.0; // ~14 NM settable
        assert!(!range_needs_align(
            Some(composition_horizon_m),
            composition_horizon_m
        ));
        // Viewport zoom at 1 NM must still force Range up to composition.
        assert!(range_needs_align(Some(1852.0), composition_horizon_m));
        assert!(range_needs_align(None, composition_horizon_m));
    }

    #[test]
    fn viewport_site_overrides_polar_center() {
        let polar_a = (53.03, 5.66);
        let site_b = (-19.85, -40.06);
        assert_eq!(
            resolve_viewport_site(Some(site_b), Some(polar_a), None),
            Some(site_b)
        );
        assert_eq!(
            resolve_viewport_site(None, Some(polar_a), None),
            Some(polar_a)
        );
    }
}
