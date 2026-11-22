//! Shared tracker → geo-arpa contact snapshot (ImevtsSim / hardware ARPA).

use super::{distance_and_bearing, MonitorContact, MonitorContactEnumerator};
use crate::radar::GeoPosition;
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// Per-radar list of monitor contacts derived from the ARPA tracker.
pub type MonitorContactSnapshot = Arc<RwLock<HashMap<String, Vec<MonitorContact>>>>;

pub fn empty_snapshot() -> MonitorContactSnapshot {
    Arc::new(RwLock::new(HashMap::new()))
}

/// Replace the contact list for `radar_key` (called each revolution).
pub fn store_contacts(
    snapshot: &MonitorContactSnapshot,
    radar_key: &str,
    contacts: Vec<MonitorContact>,
) {
    if let Ok(mut map) = snapshot.write() {
        map.insert(radar_key.to_string(), contacts);
    }
}

/// Enumerator that reads live tracker contacts for one radar key.
pub fn enumerator_from_snapshot(
    snapshot: MonitorContactSnapshot,
    radar_key: String,
) -> MonitorContactEnumerator {
    MonitorContactEnumerator::new(Arc::new(move |boat, max_range_m| {
        let Ok(map) = snapshot.read() else {
            return Vec::new();
        };
        let Some(contacts) = map.get(&radar_key) else {
            return Vec::new();
        };
        contacts
            .iter()
            .filter_map(|c| {
                // Recompute range/bearing from current boat pose when available.
                let (distance_m, bearing_rad) = if boat.lat().abs() > 1e-12
                    || boat.lon().abs() > 1e-12
                {
                    distance_and_bearing(&boat, &c.position)
                } else {
                    (c.distance_m, c.bearing_rad)
                };
                if distance_m > max_range_m {
                    return None;
                }
                Some(MonitorContact {
                    id: c.id,
                    position: c.position,
                    bearing_rad,
                    distance_m,
                    heading_deg: c.heading_deg,
                    speed_m_s: c.speed_m_s,
                    length_m: c.length_m,
                    beam_m: c.beam_m,
                    blob_heading_rad: c.blob_heading_rad,
                })
            })
            .collect()
    }))
}

/// Build a [`MonitorContact`] from tracker state (geo + optional estimated dims/heading).
pub fn contact_from_track(
    id: u64,
    position: GeoPosition,
    boat: Option<&GeoPosition>,
    heading_deg: f64,
    speed_m_s: f64,
    length_m: Option<f64>,
    beam_m: Option<f64>,
    blob_heading_rad: Option<f64>,
) -> MonitorContact {
    let (distance_m, bearing_rad) = match boat {
        Some(b) => distance_and_bearing(b, &position),
        None => (0.0, 0.0),
    };
    MonitorContact {
        id,
        position,
        bearing_rad,
        distance_m,
        heading_deg,
        speed_m_s,
        length_m,
        beam_m,
        blob_heading_rad,
    }
}
