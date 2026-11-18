//! North-up georeferenced radar echo raster (PNG + JSON metadata).
//!
//! Built once per radar revolution (gated to T = max(1s, P)) for MapLibre
//! image overlays. Echo only — no ARPA/AIS paint.
//!
//! Dual mode: fixed **composition** (`echo.json` / `echo.png`) and client
//! **viewport** (`POST …/echo/viewport` → `GET …/echo/viewport.png`).

mod annulus;
mod compositor;
mod corners;
mod frame;
mod png;
mod raster;
mod sampler;
mod state;

pub use annulus::{
    annulus_from_bbox, bbox_diagonal_m, bbox_half_min_span_m, snap_range_step_up, Annulus,
    VIEWPORT_RANGE_STEPS_M,
};
pub use compositor::{ensure_entry, ensure_running, publish_viewport, viewport_frame};
pub(crate) use compositor::ensure_composition_radar_range;
pub use corners::{corners_for, refresh_interval_ms, EchoCorners, LonLat};
pub use frame::{Center, EchoFrame, EchoMeta, SizePx};
pub use png::{transparent_png, DEFAULT_ECHO_SIZE_PX};
pub use raster::{
    bearing_in_sector, view_range_from_bbox, view_spoke_coverage_m, viewport_size_px, BBox,
};
pub use sampler::ViewportWorldSampler;
