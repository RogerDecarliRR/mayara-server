//! Geographic corners of a north-up square radar echo frame.

use super::super::target::{METERS_PER_DEGREE_LATITUDE, meters_per_degree_longitude};
use serde::Serialize;
use utoipa::ToSchema;

/// MapLibre image-source corner order: top-left, top-right, bottom-right, bottom-left.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct LonLat {
    pub lon: f64,
    pub lat: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct EchoCorners {
    pub tl: LonLat,
    pub tr: LonLat,
    pub br: LonLat,
    pub bl: LonLat,
}

/// Corners of a square centred on `(lat, lon)` with half-side `range_m` metres
/// (image edge to centre), north-up.
pub fn corners_for(lat: f64, lon: f64, range_m: f64) -> EchoCorners {
    let d_lat = range_m / METERS_PER_DEGREE_LATITUDE;
    let d_lon = range_m / meters_per_degree_longitude(&lat);
    EchoCorners {
        tl: LonLat {
            lon: lon - d_lon,
            lat: lat + d_lat,
        },
        tr: LonLat {
            lon: lon + d_lon,
            lat: lat + d_lat,
        },
        br: LonLat {
            lon: lon + d_lon,
            lat: lat - d_lat,
        },
        bl: LonLat {
            lon: lon - d_lon,
            lat: lat - d_lat,
        },
    }
}

/// `refreshIntervalMs` advertised to clients: max(1000, period of last revolution).
pub fn refresh_interval_ms(period_ms: u64) -> u64 {
    period_ms.max(1_000)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corners_are_north_up_square() {
        let c = corners_for(-19.85, -40.06, 1852.0);
        assert!(c.tl.lat > c.bl.lat);
        assert!(c.tr.lat > c.br.lat);
        assert!(c.tl.lon < c.tr.lon);
        assert!(c.bl.lon < c.br.lon);
        assert!((c.tl.lat - c.tr.lat).abs() < 1e-12);
        assert!((c.bl.lat - c.br.lat).abs() < 1e-12);
    }

    #[test]
    fn refresh_interval_is_at_least_one_second() {
        assert_eq!(refresh_interval_ms(400), 1_000);
        assert_eq!(refresh_interval_ms(1_000), 1_000);
        assert_eq!(refresh_interval_ms(2_500), 2_500);
    }
}
