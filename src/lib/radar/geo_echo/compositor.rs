//! Subscribes to radar spokes and publishes a north-up geo echo frame.

use super::corners::{corners_for, refresh_interval_ms};
use super::frame::{now_unix_ms, Center, EchoFrame, EchoMeta, SizePx};
use super::png::{encode_rgba, ECHO_SIZE_PX};
use super::raster::rasterize_polar;
use super::state::{PolarBuffer, RevClock};
use super::super::settings::ControlId;
use super::super::{Legend, RadarInfo, SharedRadars};
use crate::navdata;
use crate::protos::RadarMessage::RadarMessage;
use bytes::Bytes;
use log::{debug, warn};
use protobuf::Message;
use std::collections::HashMap;
use std::sync::{Arc, LazyLock, Mutex, RwLock};
use tokio::sync::broadcast;

struct RegistryEntry {
    frame: Arc<RwLock<EchoFrame>>,
}

static REGISTRY: LazyLock<Mutex<HashMap<String, RegistryEntry>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// Start the geo-echo compositor for `radar_id` on first request (lazy).
/// Returns `None` when the radar key is unknown.
pub fn ensure_running(radars: &SharedRadars, radar_id: &str) -> Option<Arc<RwLock<EchoFrame>>> {
    {
        let reg = REGISTRY.lock().unwrap();
        if let Some(entry) = reg.get(radar_id) {
            return Some(Arc::clone(&entry.frame));
        }
    }

    let info = radars.get_by_key(radar_id)?;
    let frame = Arc::new(RwLock::new(EchoFrame::unavailable()));

    {
        let mut reg = REGISTRY.lock().unwrap();
        if let Some(entry) = reg.get(radar_id) {
            return Some(Arc::clone(&entry.frame));
        }
        reg.insert(
            radar_id.to_string(),
            RegistryEntry {
                frame: Arc::clone(&frame),
            },
        );
    }

    let key = radar_id.to_string();
    let frame_task = Arc::clone(&frame);
    let radars_task = radars.clone();
    tokio::spawn(async move {
        run_compositor(key, info, radars_task, frame_task).await;
    });

    Some(frame)
}

async fn run_compositor(
    key: String,
    info: RadarInfo,
    radars: SharedRadars,
    frame: Arc<RwLock<EchoFrame>>,
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
                    publish_frame(&key, &radars, &polar, &mut clock, &legend, &frame);
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
                publish_frame(key, radars, polar, clock, legend, frame);
            }
        }

        polar.write_spoke(idx, &spoke.data);

        if clock.after_spoke_written(idx, spokes) {
            refresh_legend(key, radars, polar, legend, spoke.data.len());
            if clock.should_publish_now() {
                publish_frame(key, radars, polar, clock, legend, frame);
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
    if let Some(p) = navdata::get_radar_position() {
        return Some((p.lat(), p.lon()));
    }
    let (lat, lon) = navdata::get_position();
    match (lat, lon) {
        (Some(la), Some(lo)) => Some((la, lo)),
        _ => None,
    }
}

fn resolve_range_m(radars: &SharedRadars, key: &str, polar: &PolarBuffer) -> f64 {
    if let Some(info) = radars.get_by_key(key)
        && let Some(control) = info.controls.get(&ControlId::Range)
        && let Some(v) = control.value()
        && let Some(f) = v.as_f64()
        && f > 0.0
    {
        return f;
    }
    if polar.last_range_m > 0.0 {
        polar.last_range_m
    } else {
        0.0
    }
}

fn publish_frame(
    key: &str,
    radars: &SharedRadars,
    polar: &PolarBuffer,
    clock: &mut RevClock,
    legend: &Legend,
    frame: &Arc<RwLock<EchoFrame>>,
) {
    let refresh = refresh_interval_ms(clock.period_ms());
    let Some((lat, lon)) = resolve_center(polar) else {
        let mut cold = EchoFrame::unavailable();
        cold.meta.revolution_id = clock.revolution_id;
        cold.meta.refresh_interval_ms = refresh;
        if let Ok(mut slot) = frame.write() {
            *slot = cold;
        }
        clock.mark_published();
        return;
    };

    let range_m = resolve_range_m(radars, key, polar);
    if range_m <= 0.0 {
        let mut cold = EchoFrame::unavailable();
        cold.meta.revolution_id = clock.revolution_id;
        cold.meta.refresh_interval_ms = refresh;
        if let Ok(mut slot) = frame.write() {
            *slot = cold;
        }
        clock.mark_published();
        return;
    }

    let rgba = rasterize_polar(&polar.data, polar.spokes, polar.spoke_len, legend);
    let png_bytes = match encode_rgba(ECHO_SIZE_PX, ECHO_SIZE_PX, &rgba) {
        Ok(b) => b,
        Err(e) => {
            warn!("geo_echo {key}: png encode failed: {e}");
            return;
        }
    };

    let corners = corners_for(lat, lon, range_m);
    let meta = EchoMeta {
        version: 1,
        available: true,
        generated_ms: now_unix_ms(),
        revolution_id: clock.revolution_id,
        refresh_interval_ms: refresh,
        center: Center { lat, lon },
        range_m,
        north_up: true,
        size_px: SizePx {
            width: ECHO_SIZE_PX,
            height: ECHO_SIZE_PX,
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
        "geo_echo {key}: published rev {} refresh_ms={}",
        clock.revolution_id, refresh
    );
}
