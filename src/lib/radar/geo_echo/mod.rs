//! North-up georeferenced radar echo raster (PNG + JSON metadata).
//!
//! Built once per radar revolution (gated to T = max(1s, P)) for MapLibre
//! image overlays. Echo only — no ARPA/AIS paint.
//!
//! Dual mode: fixed **composition** (`echo.json` / `echo.png`) and client
//! **viewport** (`POST …/echo/viewport` → `GET …/echo/viewport.png`).

mod compositor;
mod corners;
mod frame;
mod png;
mod raster;
mod state;

pub use compositor::{ensure_entry, ensure_running, publish_viewport, viewport_frame};
pub use corners::{corners_for, refresh_interval_ms, EchoCorners, LonLat};
pub use frame::{Center, EchoFrame, EchoMeta, SizePx};
pub use png::{transparent_png, DEFAULT_ECHO_SIZE_PX};
pub use raster::{bearing_in_sector, viewport_size_px, BBox};
