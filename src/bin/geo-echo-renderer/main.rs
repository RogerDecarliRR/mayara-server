//! Standalone geo-echo renderer: MEPO polar ingest + viewport paint (W1).
//! Polar double-buffer (W3 / A6): paint reads stable slot only.

use axum::body::Bytes;
use axum::extract::{DefaultBodyLimit, State};
use axum::http::StatusCode;
use axum::response::IntoResponse;
use axum::routing::{get, post, put};
use axum::{Json, Router};
use clap::Parser;
use mayara::config::GeoEchoConfig;
use mayara::radar::geo_echo::{
    deserialize_option_u32_lenient, paint_viewport, polar_blob, EchoFrame, EchoMeta, PolarSlots,
    ViewportPaintRequest,
};
use serde::{Deserialize, Serialize};
use std::net::SocketAddr;
use std::sync::Arc;
use tokio::sync::RwLock;

#[derive(Parser, Debug)]
#[command(name = "geo-echo-renderer", about = "Mayara geo-echo renderer (polar + viewport)")]
struct Args {
    /// Listen address (host:port).
    #[arg(long, default_value = "0.0.0.0:6503")]
    bind: String,
}

#[derive(Clone)]
struct AppState {
    /// Double-buffer polar (A6): PUT writes back slot then publishes.
    polar: Arc<RwLock<PolarSlots>>,
    last_png: Arc<RwLock<Option<Vec<u8>>>>,
}

#[derive(Serialize)]
struct Health {
    ok: bool,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct PolarMeta {
    revolution_id: u64,
    spokes: u32,
    spoke_len: u32,
    period_ms: u64,
    spoke_range_m: f64,
    has_center: bool,
}

/// POST /v1/viewport body (camelCase) — Mayara ViewportRequest + paint knobs + radarId.
#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct ViewportPost {
    west: f64,
    south: f64,
    east: f64,
    north: f64,
    #[serde(default)]
    radar_lat: Option<f64>,
    #[serde(default)]
    radar_lon: Option<f64>,
    /// Used only to build SignalK `pngPath`.
    radar_id: String,
    #[serde(default)]
    usable_bearing_start_deg: Option<f64>,
    #[serde(default)]
    usable_bearing_end_deg: Option<f64>,
    #[serde(default)]
    usable_radius_m: Option<f64>,
    #[serde(default)]
    max_size_px: Option<u32>,
    #[serde(default)]
    target_meters_per_px: Option<f64>,
    #[serde(default)]
    max_viewport_range_m: Option<f64>,
    /// First-pan / low-res hint (`sizeHintPx` or alias `sizeHint`).
    /// Accepts JSON int or float (`1024` / `1024.0`); float is truncated.
    #[serde(
        default,
        alias = "sizeHint",
        deserialize_with = "deserialize_option_u32_lenient"
    )]
    size_hint_px: Option<u32>,
}

/// BBox echo in the response (same shape as SignalK ViewportRequest).
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct ViewportBBox {
    west: f64,
    south: f64,
    east: f64,
    north: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    radar_lat: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    radar_lon: Option<f64>,
}

/// Matches SignalK `ViewportResponse` so the core can pass JSON through.
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct ViewportResponse {
    #[serde(flatten)]
    meta: EchoMeta,
    bbox: ViewportBBox,
    meters_per_px: f64,
    png_path: String,
}

#[tokio::main]
async fn main() {
    let args = Args::parse();
    let addr: SocketAddr = args
        .bind
        .parse()
        .unwrap_or_else(|e| panic!("invalid --bind {}: {e}", args.bind));

    let state = AppState {
        polar: Arc::new(RwLock::new(PolarSlots::new())),
        last_png: Arc::new(RwLock::new(None)),
    };

    // MEPO polar ~0.5–2+ MB/rev (spokes×spoke_len); axum default 2 MiB → 413.
    const POLAR_BODY_LIMIT: usize = 32 * 1024 * 1024;

    let app = Router::new()
        .route("/health", get(health))
        .route(
            "/v1/polar",
            put(put_polar).layer(DefaultBodyLimit::max(POLAR_BODY_LIMIT)),
        )
        .route("/v1/polar/meta", get(get_polar_meta))
        .route("/v1/viewport", post(post_viewport))
        .route("/v1/viewport.png", get(get_viewport_png))
        .with_state(state);

    let listener = tokio::net::TcpListener::bind(addr)
        .await
        .unwrap_or_else(|e| panic!("bind {addr}: {e}"));
    eprintln!("geo-echo-renderer listening on http://{addr}");
    axum::serve(listener, app)
        .await
        .unwrap_or_else(|e| panic!("serve: {e}"));
}

async fn health() -> Json<Health> {
    Json(Health { ok: true })
}

async fn put_polar(State(state): State<AppState>, body: Bytes) -> impl IntoResponse {
    match polar_blob::decode(&body) {
        Ok(decoded) => {
            state.polar.write().await.publish(decoded);
            StatusCode::NO_CONTENT
        }
        Err(_) => StatusCode::BAD_REQUEST,
    }
}

async fn get_polar_meta(State(state): State<AppState>) -> impl IntoResponse {
    let guard = state.polar.read().await;
    match guard.stable() {
        Some(p) => (
            StatusCode::OK,
            Json(PolarMeta {
                revolution_id: p.revolution_id,
                spokes: p.spokes,
                spoke_len: p.spoke_len,
                period_ms: p.period_ms,
                spoke_range_m: p.spoke_range_m,
                has_center: p.center.is_some(),
            }),
        )
            .into_response(),
        None => StatusCode::NOT_FOUND.into_response(),
    }
}

fn paint_request_from_post(body: &ViewportPost) -> Result<ViewportPaintRequest, String> {
    match (body.radar_lat, body.radar_lon) {
        (Some(_), Some(_)) | (None, None) => {}
        _ => {
            return Err("radarLat and radarLon must both be set or both omitted".into());
        }
    }
    let defaults = GeoEchoConfig::default();
    let mut geo = defaults;
    if let Some(v) = body.usable_bearing_start_deg {
        geo.usable_bearing_start_deg = v;
    }
    if let Some(v) = body.usable_bearing_end_deg {
        geo.usable_bearing_end_deg = v;
    }
    if let Some(v) = body.usable_radius_m {
        geo.usable_radius_m = Some(v);
    }
    if let Some(v) = body.max_size_px {
        geo.max_size_px = v;
    }
    if let Some(v) = body.target_meters_per_px {
        geo.target_meters_per_px = v;
    }
    if let Some(v) = body.max_viewport_range_m {
        geo.max_viewport_range_m = Some(v);
    }
    let geo = geo.normalized();
    let mut req = ViewportPaintRequest::from_bbox_and_config(
        body.west,
        body.south,
        body.east,
        body.north,
        body.radar_lat,
        body.radar_lon,
        &geo,
    );
    req.size_hint = body.size_hint_px;
    Ok(req)
}

fn png_path_for(radar_id: &str) -> String {
    format!("/signalk/v2/api/vessels/self/radars/{radar_id}/echo/viewport.png")
}

fn unavailable_viewport_response(body: &ViewportPost, size_px: u32) -> (ViewportResponse, Vec<u8>) {
    let frame = EchoFrame::unavailable_sized(size_px.max(1));
    let mid_lat = (body.south + body.north) * 0.5;
    let dlat = (body.north - body.south).abs();
    let dlon = (body.east - body.west).abs();
    let ns = dlat * 111_320.0;
    let ew = dlon * 111_320.0 * mid_lat.to_radians().cos().abs().max(0.2);
    let span = ew.max(ns).max(1e-6);
    let meters_per_px = span / frame.meta.size_px.width.max(1) as f64;
    let resp = ViewportResponse {
        meta: frame.meta.clone(),
        bbox: ViewportBBox {
            west: body.west,
            south: body.south,
            east: body.east,
            north: body.north,
            radar_lat: body.radar_lat,
            radar_lon: body.radar_lon,
        },
        meters_per_px,
        png_path: png_path_for(&body.radar_id),
    };
    (resp, frame.png.to_vec())
}

async fn post_viewport(
    State(state): State<AppState>,
    Json(body): Json<ViewportPost>,
) -> impl IntoResponse {
    if body.radar_id.trim().is_empty() {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({"error": "radarId required"})),
        )
            .into_response();
    }

    let req = match paint_request_from_post(&body) {
        Ok(r) => r,
        Err(msg) => {
            return (
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({"error": msg})),
            )
                .into_response();
        }
    };

    let polar = {
        let guard = state.polar.read().await;
        guard.clone_stable()
    };

    let Some(polar) = polar else {
        let defaults = GeoEchoConfig::default();
        let (resp, png) = unavailable_viewport_response(&body, defaults.max_size_px);
        *state.last_png.write().await = Some(png);
        return (StatusCode::OK, Json(resp)).into_response();
    };

    let paint_req = req;
    let result = match tokio::task::spawn_blocking(move || paint_viewport(&polar, &paint_req)).await
    {
        Ok(Ok(r)) => r,
        Ok(Err(msg)) => {
            return (
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({"error": msg})),
            )
                .into_response();
        }
        Err(e) => {
            return (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(serde_json::json!({"error": format!("paint join: {e}")})),
            )
                .into_response();
        }
    };

    *state.last_png.write().await = Some(result.png.clone());

    let resp = ViewportResponse {
        meta: result.meta,
        bbox: ViewportBBox {
            west: body.west,
            south: body.south,
            east: body.east,
            north: body.north,
            radar_lat: body.radar_lat,
            radar_lon: body.radar_lon,
        },
        meters_per_px: result.meters_per_px,
        png_path: png_path_for(&body.radar_id),
    };
    (StatusCode::OK, Json(resp)).into_response()
}

async fn get_viewport_png(State(state): State<AppState>) -> impl IntoResponse {
    let guard = state.last_png.read().await;
    match guard.as_ref() {
        Some(png) => (
            StatusCode::OK,
            [(axum::http::header::CONTENT_TYPE, "image/png")],
            png.clone(),
        )
            .into_response(),
        None => StatusCode::NOT_FOUND.into_response(),
    }
}
