//! Shared echo snapshot types.

use super::corners::{EchoCorners, LonLat};
use super::png::{transparent_png, ECHO_SIZE_PX};
use bytes::Bytes;
use serde::Serialize;
use std::time::{SystemTime, UNIX_EPOCH};
use utoipa::ToSchema;

#[derive(Clone, Debug, Serialize, ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct SizePx {
    pub width: u32,
    pub height: u32,
}

#[derive(Clone, Debug, Serialize, ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct Center {
    pub lat: f64,
    pub lon: f64,
}

/// Contract for `GET …/echo.json` (camelCase).
#[derive(Clone, Debug, Serialize, ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct EchoMeta {
    pub version: u32,
    pub available: bool,
    pub generated_ms: u64,
    pub revolution_id: u64,
    pub refresh_interval_ms: u64,
    pub center: Center,
    pub range_m: f64,
    pub north_up: bool,
    pub size_px: SizePx,
    pub corners: EchoCorners,
}

#[derive(Clone)]
pub struct EchoFrame {
    pub meta: EchoMeta,
    pub png: Bytes,
}

impl EchoFrame {
    pub fn unavailable() -> Self {
        Self {
            meta: EchoMeta {
                version: 1,
                available: false,
                generated_ms: now_unix_ms(),
                revolution_id: 0,
                refresh_interval_ms: 1_000,
                center: Center { lat: 0.0, lon: 0.0 },
                range_m: 0.0,
                north_up: true,
                size_px: SizePx {
                    width: ECHO_SIZE_PX,
                    height: ECHO_SIZE_PX,
                },
                corners: EchoCorners {
                    tl: LonLat { lon: 0.0, lat: 0.0 },
                    tr: LonLat { lon: 0.0, lat: 0.0 },
                    br: LonLat { lon: 0.0, lat: 0.0 },
                    bl: LonLat { lon: 0.0, lat: 0.0 },
                },
            },
            png: Bytes::from_static(transparent_png()),
        }
    }
}

pub(crate) fn now_unix_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unavailable_frame_reports_available_false() {
        let f = EchoFrame::unavailable();
        assert!(!f.meta.available);
        assert_eq!(f.meta.version, 1);
        assert_eq!(f.meta.size_px.width, 1024);
        assert_eq!(f.meta.size_px.height, 1024);
        assert!(f.meta.north_up);
        assert!(!f.png.is_empty());
        assert_eq!(&f.png[0..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
    }
}
