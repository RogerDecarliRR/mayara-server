//! Build `GET …/geo-arpa.json` from a registered contact enumerator.

use super::super::geo_echo::bearing_in_sector;
use super::super::{GeoPosition, SharedRadars};
use serde::Serialize;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};
use utoipa::ToSchema;

/// One mobile contact from the brand world (emulator), before sector filter.
#[derive(Clone, Debug, PartialEq)]
pub struct MonitorContact {
    pub id: u64,
    pub position: GeoPosition,
    /// Bearing from the boat used when listing, radians North = 0 CW.
    pub bearing_rad: f64,
    /// Distance from that boat, metres.
    pub distance_m: f64,
    /// Heading in degrees (North = 0, clockwise).
    pub heading_deg: f64,
    /// Speed over ground in m/s.
    pub speed_m_s: f64,
}

/// Callback that lists monitor contacts for a boat position and max range.
#[derive(Clone)]
pub struct MonitorContactEnumerator {
    list: Arc<dyn Fn(GeoPosition, f64) -> Vec<MonitorContact> + Send + Sync>,
}

impl MonitorContactEnumerator {
    pub fn new(list: Arc<dyn Fn(GeoPosition, f64) -> Vec<MonitorContact> + Send + Sync>) -> Self {
        Self { list }
    }

    pub fn list(&self, boat: GeoPosition, max_range_m: f64) -> Vec<MonitorContact> {
        (self.list)(boat, max_range_m)
    }
}

/// One target in the `geo-arpa.json` payload (camelCase).
#[derive(Clone, Debug, Serialize, ToSchema, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GeoArpaTarget {
    pub id: u64,
    /// Bearing from radar, radians, North = 0 clockwise.
    pub bearing: f64,
    /// Slant range metres.
    pub distance: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lat: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lon: Option<f64>,
    /// Course over ground, radians North = 0 CW.
    pub cog: f64,
    /// Speed over ground, m/s.
    pub sog: f64,
}

/// Contract for `GET …/geo-arpa.json` (camelCase).
#[derive(Clone, Debug, Serialize, ToSchema, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GeoArpaMeta {
    pub version: u32,
    pub available: bool,
    pub generated_ms: u64,
    pub radar_id: String,
    pub max_range_m: f64,
    pub targets: Vec<GeoArpaTarget>,
}

impl GeoArpaMeta {
    pub fn unavailable(radar_id: &str, max_range_m: f64) -> Self {
        Self {
            version: 1,
            available: false,
            generated_ms: now_unix_ms(),
            radar_id: radar_id.to_string(),
            max_range_m,
            targets: Vec::new(),
        }
    }
}

fn now_unix_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// Flat-earth distance (m) and bearing (rad N CW) from `from` to `to`.
pub fn distance_and_bearing(from: &GeoPosition, to: &GeoPosition) -> (f64, f64) {
    const METERS_PER_DEGREE_LAT: f64 = 111_320.0;
    let lat_rad = from.lat().to_radians();
    let meters_per_degree_lon = METERS_PER_DEGREE_LAT * lat_rad.cos();
    let dx = (to.lon() - from.lon()) * meters_per_degree_lon;
    let dy = (to.lat() - from.lat()) * METERS_PER_DEGREE_LAT;
    let distance = dx.hypot(dy);
    let bearing = dx.atan2(dy).rem_euclid(std::f64::consts::TAU);
    (distance, bearing)
}

/// Publish monitor ARPA for `radar_id`. Without an enumerator → unavailable.
pub fn publish_geo_arpa(radars: &SharedRadars, radar_id: &str) -> GeoArpaMeta {
    let geo = radars.geo_echo_config();
    let max_range_m = geo.effective_usable_radius_m();
    let Some(_info) = radars.get_by_key(radar_id) else {
        return GeoArpaMeta::unavailable(radar_id, max_range_m);
    };
    let Some(enumerator) = radars.monitor_contact_enumerator(radar_id) else {
        return GeoArpaMeta::unavailable(radar_id, max_range_m);
    };

    let boat = resolve_boat_position().unwrap_or_else(|| GeoPosition::new(0.0, 0.0));
    let contacts = enumerator.list(boat, max_range_m);

    let start = geo.usable_bearing_start_deg;
    let end = geo.usable_bearing_end_deg;
    let mut targets = Vec::with_capacity(contacts.len());
    for c in contacts {
        if c.distance_m > max_range_m {
            continue;
        }
        let bearing_deg = c.bearing_rad.to_degrees();
        if !bearing_in_sector(bearing_deg, start, end) {
            continue;
        }
        targets.push(GeoArpaTarget {
            id: c.id,
            bearing: c.bearing_rad,
            distance: c.distance_m,
            lat: Some(c.position.lat()),
            lon: Some(c.position.lon()),
            cog: c.heading_deg.to_radians(),
            sog: c.speed_m_s,
        });
    }

    GeoArpaMeta {
        version: 1,
        available: true,
        generated_ms: now_unix_ms(),
        radar_id: radar_id.to_string(),
        max_range_m,
        targets,
    }
}

fn resolve_boat_position() -> Option<GeoPosition> {
    if let Some(p) = crate::navdata::get_radar_position() {
        return Some(p);
    }
    let (lat, lon) = crate::navdata::get_position();
    match (lat, lon) {
        (Some(la), Some(lo)) => Some(GeoPosition::new(la, lo)),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::geo_echo::bearing_in_sector;

    #[test]
    fn distance_and_bearing_east_is_pi_over_two() {
        let boat = GeoPosition::new(0.0, 0.0);
        // ~1113.2 m east at equator (0.01 deg lon)
        let east = GeoPosition::new(0.0, 0.01);
        let (d, b) = distance_and_bearing(&boat, &east);
        assert!((d - 1113.2).abs() < 2.0, "d={d}");
        assert!((b - std::f64::consts::FRAC_PI_2).abs() < 0.02, "b={b}");
    }

    #[test]
    fn sector_filter_excludes_outside() {
        assert!(bearing_in_sector(10.0, 0.0, 90.0));
        assert!(!bearing_in_sector(180.0, 0.0, 90.0));
    }

    #[test]
    fn enumerator_lists_then_sector_applied() {
        let boat = GeoPosition::new(53.0, 5.0);
        let near = boat.position_from_bearing(0.0, 500.0); // north
        let far = boat.position_from_bearing(0.0, 5_000.0);
        let west = boat.position_from_bearing(std::f64::consts::FRAC_PI_2 * 3.0, 400.0);

        let enumerator = MonitorContactEnumerator::new(Arc::new(move |_, max| {
            [near, far, west]
                .into_iter()
                .enumerate()
                .filter_map(|(i, pos)| {
                    let (distance_m, bearing_rad) = distance_and_bearing(&boat, &pos);
                    if distance_m > max {
                        return None;
                    }
                    Some(MonitorContact {
                        id: (i + 1) as u64,
                        position: pos,
                        bearing_rad,
                        distance_m,
                        heading_deg: 90.0,
                        speed_m_s: 2.0,
                    })
                })
                .collect()
        }));

        let listed = enumerator.list(boat, 1_000.0);
        assert_eq!(listed.len(), 2); // near + west; far out of range
        assert!(listed.iter().any(|c| c.id == 1));
        assert!(listed.iter().any(|c| c.id == 3));
        assert!(!listed.iter().any(|c| c.id == 2));

        // Sector east-only would keep nothing of near/west — check filter helper.
        let (d_near, b_near) = distance_and_bearing(&boat, &near);
        assert!(d_near <= 1_000.0);
        assert!(bearing_in_sector(b_near.to_degrees(), 0.0, 360.0));
        assert!(!bearing_in_sector(b_near.to_degrees(), 90.0, 180.0));
    }
}
