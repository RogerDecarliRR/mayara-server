//! North-up georeferenced radar echo raster (PNG + JSON metadata).
//!
//! Built once per radar revolution (gated to T = max(1s, P)) for MapLibre
//! image overlays. Echo only — no ARPA/AIS paint.

mod compositor;
mod corners;
mod frame;
mod png;
mod raster;
mod state;

pub use compositor::ensure_running;
pub use corners::{corners_for, refresh_interval_ms, EchoCorners, LonLat};
pub use frame::{Center, EchoFrame, EchoMeta, SizePx};
pub use png::{transparent_png, ECHO_SIZE_PX};
