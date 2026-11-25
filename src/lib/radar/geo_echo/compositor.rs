//! Subscribes to radar spokes and publishes a north-up geo echo frame.

use super::super::settings::ControlId;
use super::super::{Legend, RadarInfo, SharedRadars};
use super::corners::{corners_for, refresh_interval_ms};
use super::frame::{now_unix_ms, Center, EchoFrame, EchoMeta, SizePx};
use super::png::encode_rgba;
use super::annulus::{
    bbox_half_min_span_m, snap_range_step_up, view_m_per_bin, VIEWPORT_RANGE_STEPS_M,
};
use super::polar_blob;
use super::polar_display_reduce::{PolarDisplayMode, reduce_for_display};
use super::polar_viewport_preprocess::{preprocess_viewport_polar, ViewportPaintOptions};
use super::raster::{
    rasterize_polar_masked, rasterize_viewport_with_world, effective_viewport_max_size_px,
    view_range_from_bbox, viewport_image_corners, viewport_size_px, ViewportSampleMode, BBox,
};
use super::sampler::{fill_polar_from_world, ViewportWorldSampler};
use super::state::{PolarBuffer, RevClock};
use crate::config::EchoPaint;
use crate::navdata;
use crate::protos::RadarMessage::RadarMessage;
use bytes::Bytes;
use log::{debug, warn};
use protobuf::Message;
use std::borrow::Cow;
use std::collections::HashMap;
use std::sync::{Arc, LazyLock, Mutex, RwLock};
use tokio::sync::broadcast;

/// Latest polar snapshot for on-demand viewport renders.
#[derive(Clone)]
pub struct PolarSnapshot {
    pub data: Vec<u8>,
    pub spokes: usize,
    pub spoke_len: usize,
    /// Spokes counted in a revolution (sparse radar); PPI reduce display axis.
    pub observed_spokes_per_rev: usize,
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
            observed_spokes_per_rev: 0,
            center: None,
            spoke_range_m: 0.0,
            legend,
            revolution_id: 0,
            period_ms: 2_000,
        }
    }

    /// True after at least one published revolution with spoke data.
    pub fn is_ready(&self) -> bool {
        self.revolution_id > 0
            && self.spokes > 0
            && self.spoke_len > 0
            && !self.data.is_empty()
    }
}

/// Capacity of the MEPO WebSocket broadcast (lagged clients skip intermediate revs).
const MEPO_BROADCAST_CAP: usize = 4;

struct RegistryEntry {
    frame: Arc<RwLock<EchoFrame>>,
    viewport: Arc<RwLock<EchoFrame>>,
    snapshot: Arc<RwLock<PolarSnapshot>>,
    mepo_tx: broadcast::Sender<Bytes>,
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
    let (frame, viewport, snapshot, _) = ensure_entry_full(radars, radar_id)?;
    Some((frame, viewport, snapshot))
}

/// Polar snapshot slot + MEPO broadcast subscribe for GET/WS handlers.
pub fn mepo_subscribe(
    radars: &SharedRadars,
    radar_id: &str,
) -> Option<(Arc<RwLock<PolarSnapshot>>, broadcast::Receiver<Bytes>)> {
    let (_, _, snapshot, mepo_tx) = ensure_entry_full(radars, radar_id)?;
    Some((snapshot, mepo_tx.subscribe()))
}

#[allow(clippy::type_complexity)]
fn ensure_entry_full(
    radars: &SharedRadars,
    radar_id: &str,
) -> Option<(
    Arc<RwLock<EchoFrame>>,
    Arc<RwLock<EchoFrame>>,
    Arc<RwLock<PolarSnapshot>>,
    broadcast::Sender<Bytes>,
)> {
    {
        let reg = REGISTRY.lock().unwrap();
        if let Some(entry) = reg.get(radar_id) {
            return Some((
                Arc::clone(&entry.frame),
                Arc::clone(&entry.viewport),
                Arc::clone(&entry.snapshot),
                entry.mepo_tx.clone(),
            ));
        }
    }

    let info = radars.get_by_key(radar_id)?;
    let size_px = radars.geo_echo_config().echo_size_px;
    let frame = Arc::new(RwLock::new(EchoFrame::unavailable_sized(size_px)));
    let viewport = Arc::new(RwLock::new(EchoFrame::unavailable_sized(size_px)));
    let snapshot = Arc::new(RwLock::new(PolarSnapshot::empty(info.get_legend())));
    let (mepo_tx, _) = broadcast::channel(MEPO_BROADCAST_CAP);

    {
        let mut reg = REGISTRY.lock().unwrap();
        if let Some(entry) = reg.get(radar_id) {
            return Some((
                Arc::clone(&entry.frame),
                Arc::clone(&entry.viewport),
                Arc::clone(&entry.snapshot),
                entry.mepo_tx.clone(),
            ));
        }
        reg.insert(
            radar_id.to_string(),
            RegistryEntry {
                frame: Arc::clone(&frame),
                viewport: Arc::clone(&viewport),
                snapshot: Arc::clone(&snapshot),
                mepo_tx: mepo_tx.clone(),
            },
        );
    }

    let key = radar_id.to_string();
    let frame_task = Arc::clone(&frame);
    let snapshot_task = Arc::clone(&snapshot);
    let mepo_tx_task = mepo_tx.clone();
    let radars_task = radars.clone();
    tokio::spawn(async move {
        run_compositor(
            key,
            info,
            radars_task,
            frame_task,
            snapshot_task,
            mepo_tx_task,
        )
        .await;
    });

    Some((frame, viewport, snapshot, mepo_tx))
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
    size_hint: Option<u32>,
    polar_display: PolarDisplayMode,
    paint_opts: ViewportPaintOptions,
) -> Result<EchoMeta, String> {
    bbox.validate()?;
    let (_, viewport_slot, snapshot_slot) = ensure_entry(radars, radar_id)
        .ok_or_else(|| format!("radar '{radar_id}' not found"))?;

    let snap = snapshot_slot
        .read()
        .map_err(|_| "viewport snapshot lock poisoned".to_string())?
        .clone();

    let geo = radars.geo_echo_config();

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
    let max_for_paint = effective_viewport_max_size_px(geo.max_size_px, size_hint);
    let size_px = viewport_size_px(span_ew, span_ns, effective_target, max_for_paint);
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

    // Annular inverse-sample at snapped view_step. PNG pitch follows view_step;
    // operator Control Range is independent of composition coverage.
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
            geo.effective_usable_radius_m(),
            Some(&echo),
            ViewportSampleMode::Nearest,
        )
    } else {
        let spokes_n = spokes;
        let spoke_len_n = spoke_len;
        let (polar_data, raster_spokes): (Vec<u8>, usize) = match polar_display {
            PolarDisplayMode::Default => (snap.data.clone(), spokes_n),
            PolarDisplayMode::Reduce => {
                let observed = (snap.observed_spokes_per_rev > 0).then_some(snap.observed_spokes_per_rev);
                let (reduced, display_spokes) =
                    reduce_for_display(&snap.data, spokes_n, spoke_len_n, observed);
                (reduced, display_spokes)
            }
        };
        let smooth_on =
            paint_opts.effective_smooth(polar_display == PolarDisplayMode::Reduce);
        let mut opts = paint_opts;
        opts.viewport_smooth = smooth_on;
        let polar_data =
            preprocess_viewport_polar(polar_data, raster_spokes, spoke_len_n, opts);
        let sample_mode = if smooth_on {
            ViewportSampleMode::Bilinear
        } else {
            ViewportSampleMode::Nearest
        };
        rasterize_viewport_with_world(
            &polar_data,
            raster_spokes,
            spoke_len_n,
            &snap.legend,
            size_px,
            lat,
            lon,
            snap.spoke_range_m,
            view_step_m,
            bbox,
            geo.usable_bearing_start_deg,
            geo.usable_bearing_end_deg,
            geo.effective_usable_radius_m(),
            None,
            sample_mode,
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
    mepo_tx: broadcast::Sender<Bytes>,
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
                    publish_frame(
                        &key,
                        &radars,
                        &polar,
                        &mut clock,
                        &legend,
                        &frame,
                        &snapshot,
                        &mepo_tx,
                    );
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
                                &mepo_tx,
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
    mepo_tx: &broadcast::Sender<Bytes>,
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
                publish_frame(key, radars, polar, clock, legend, frame, snapshot, mepo_tx);
            }
        }

        polar.write_spoke(idx, &spoke.data);

        if clock.after_spoke_written(idx, spokes) {
            refresh_legend(key, radars, polar, legend, spoke.data.len());
            if clock.should_publish_now() {
                publish_frame(key, radars, polar, clock, legend, frame, snapshot, mepo_tx);
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

/// Physical range of the current acquisition spoke data (GUI Control Range /
/// spoke TAG). Used for Mayara heritage composition PNG — not for MEPO when a
/// [`ViewportWorldSampler`] supplies world-filled usable coverage.
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

/// MEPO polar for `store_snapshot` / renderer push.
///
/// With a world sampler (emulator): fill at `usable_radius_m` so spoke pitch is
/// operational coverage, independent of GUI Control Range. Without sampler
/// (hardware): borrow the acquisition buffer and its spoke range.
fn resolve_mepo_polar<'a>(
    sampler: Option<&ViewportWorldSampler>,
    polar_data: &'a [u8],
    spokes: usize,
    spoke_len: usize,
    acq_spoke_range_m: f64,
    usable_radius_m: f64,
) -> (Cow<'a, [u8]>, f64) {
    if let Some(sampler) = sampler {
        sampler.prepare();
        let data = fill_polar_from_world(sampler, spokes, spoke_len, usable_radius_m);
        (Cow::Owned(data), usable_radius_m)
    } else {
        (Cow::Borrowed(polar_data), acq_spoke_range_m)
    }
}

fn store_snapshot(
    snapshot: &Arc<RwLock<PolarSnapshot>>,
    mepo_tx: &broadcast::Sender<Bytes>,
    echo_paint: EchoPaint,
    polar: &PolarBuffer,
    data: &[u8],
    clock: &RevClock,
    legend: &Legend,
    spoke_range_m: f64,
) {
    let snap = PolarSnapshot {
        data: data.to_vec(),
        spokes: polar.spokes,
        spoke_len: polar.spoke_len,
        observed_spokes_per_rev: clock.observed_spokes_per_rev,
        center: polar.last_center.or_else(resolve_nav_center),
        spoke_range_m,
        legend: legend.clone(),
        revolution_id: clock.revolution_id,
        period_ms: clock.period_ms(),
    };
    if let Ok(mut slot) = snapshot.write() {
        *slot = snap.clone();
    }
    // Notify MEPO WS subscribers (binary blob). Lagged receivers skip revs.
    if let Ok(bytes) = polar_blob::encode_snapshot(&snap) {
        let _ = mepo_tx.send(Bytes::from(bytes));
    }
    // W1 D1: push MEPO to geo-echo-renderer only when echoPaint=sidecar + URL set.
    super::renderer_client::push_polar_snapshot(&snap, echo_paint);
}

fn publish_frame(
    key: &str,
    radars: &SharedRadars,
    polar: &PolarBuffer,
    clock: &mut RevClock,
    legend: &Legend,
    frame: &Arc<RwLock<EchoFrame>>,
    snapshot: &Arc<RwLock<PolarSnapshot>>,
    mepo_tx: &broadcast::Sender<Bytes>,
) {
    let geo = radars.geo_echo_config();
    let composition_range_m = geo.composition_range_m;
    let size_px = geo.echo_size_px;
    let refresh = refresh_interval_ms(clock.period_ms());
    let usable_radius_m = geo.effective_usable_radius_m();

    // Acquisition pitch (GUI Control Range) — composition PNG only.
    let acq_spoke_range_m = resolve_spoke_range_m(radars, key, polar);

    // H1 / GUI ⟂: when emulator registers a ViewportWorldSampler, MEPO is
    // world-filled at usableRadiusM before store_snapshot → renderer push.
    // Viewport paint stays forwarded to the sidecar (no fill_polar there).
    let world = radars.viewport_world_sampler(key);
    let (mepo_data, mepo_spoke_range_m) = resolve_mepo_polar(
        world.as_ref(),
        &polar.data,
        polar.spokes,
        polar.spoke_len,
        acq_spoke_range_m,
        usable_radius_m,
    );
    store_snapshot(
        snapshot,
        mepo_tx,
        geo.echo_paint,
        polar,
        mepo_data.as_ref(),
        clock,
        legend,
        mepo_spoke_range_m,
    );

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

    // Cheap local composition PNG from the acquisition polar (Mayara heritage).
    // Viewport paint stays in the sidecar when GEO_ECHO_RENDERER_URL is set.
    let corners = corners_for(lat, lon, composition_range_m);
    let rgba = rasterize_polar_masked(
        &polar.data,
        polar.spokes,
        polar.spoke_len,
        legend,
        size_px,
        acq_spoke_range_m,
        composition_range_m,
        geo.usable_bearing_start_deg,
        geo.usable_bearing_end_deg,
        usable_radius_m,
    );
    let png_bytes = match encode_rgba(size_px, size_px, &rgba) {
        Ok(b) => b,
        Err(e) => {
            warn!("geo_echo {key}: png encode failed: {e}");
            return;
        }
    };
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
    use super::{resolve_mepo_polar, resolve_viewport_site, ViewportWorldSampler};
    use std::sync::Arc;

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

    /// With a world sampler, MEPO spoke_range is usable coverage — not GUI acq.
    #[test]
    fn mepo_with_sampler_uses_usable_spoke_range() {
        let acq: Vec<u8> = vec![1u8; 4 * 8];
        let acq_range = 926.0; // ~½ NM GUI Control Range
        let usable = 3704.0;
        let sampler = ViewportWorldSampler::new(
            Arc::new(|| {}),
            Arc::new(|_b, range| if range > 1000.0 { 9 } else { 0 }),
        );
        let (data, spoke_range) =
            resolve_mepo_polar(Some(&sampler), &acq, 4, 8, acq_range, usable);
        assert!((spoke_range - usable).abs() < 1e-9);
        assert_eq!(data.len(), 4 * 8);
        // Outer bins past short GUI range must be world-filled, not acq bytes.
        assert_ne!(&data[..], &acq[..]);
        let mut outer = false;
        for j in 0..8 {
            let range = (j as f64 + 0.5) / 8.0 * usable;
            if range > 1000.0 {
                assert_eq!(data[j], 9);
                outer = true;
            }
        }
        assert!(outer);
    }

    #[test]
    fn mepo_without_sampler_keeps_acquisition() {
        let acq: Vec<u8> = vec![3u8; 16];
        let acq_range = 694.0;
        let (data, spoke_range) =
            resolve_mepo_polar(None, &acq, 4, 4, acq_range, 3704.0);
        assert!((spoke_range - acq_range).abs() < 1e-9);
        assert_eq!(data.as_ref(), &acq[..]);
    }
}
