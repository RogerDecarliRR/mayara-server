//! North-up georeferenced radar echo raster (PNG + JSON metadata).
//!
//! Built once per radar revolution (gated to T = max(1s, P)) for MapLibre
//! image overlays. Echo only — no ARPA/AIS paint.
//!
//! Dual mode: fixed **composition** (`echo.json` / `echo.png`) and client
//! **viewport** (`POST …/echo/viewport` → `GET …/echo/viewport.png`).
//!
//! Composition coverage (`compositionRangeM` × usable bearings) is independent
//! of the operator's `ControlId::Range` (GUI zoom). Monitor ARPA uses `geo_arpa`.

mod annulus;
mod compositor;
mod corners;
mod frame;
mod png;
pub mod polar_blob;
pub mod polar_slots;
mod raster;
mod remote_paint;
mod renderer_client;
mod sampler;
mod state;

pub use annulus::{
    annulus_from_bbox, bbox_diagonal_m, bbox_half_min_span_m, snap_range_step_up, Annulus,
    VIEWPORT_RANGE_STEPS_M,
};
pub use compositor::{
    ensure_entry, ensure_running, mepo_subscribe, publish_viewport, viewport_frame, PolarSnapshot,
};
pub use corners::{corners_for, refresh_interval_ms, EchoCorners, LonLat};
pub use frame::{Center, EchoFrame, EchoMeta, SizePx};
pub use png::{transparent_png, DEFAULT_ECHO_SIZE_PX};
pub use polar_blob::{decode, encode, encode_snapshot, DecodedPolar, PolarBlobError};
pub use polar_slots::PolarSlots;
pub use raster::{
    bearing_in_sector, deserialize_option_u32_lenient, effective_viewport_max_size_px,
    snap_size_hint_to_allowed, view_range_from_bbox, view_spoke_coverage_m, viewport_size_px, BBox,
};
pub use remote_paint::{paint_viewport, ViewportPaintRequest, ViewportPaintResult};
pub use renderer_client::{push_polar_snapshot, renderer_base_url, should_push_to_renderer};
pub use sampler::ViewportWorldSampler;
