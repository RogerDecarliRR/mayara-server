//! Monitor-only ARPA feed (`geo-arpa.json`), independent of GUI Range / blob tracker.
//!
//! Coverage reuses [`crate::config::GeoEchoConfig`] (`usableRadiusM` × usable
//! bearings; composition is canvas only). Lab source: emulator world contacts via
//! [`MonitorContactEnumerator`], or ARPA tracker snapshot (ImevtsSim / hardware).
//! Hardware without an enumerator returns `available: false` and an empty target list.
//! Optional `lengthM`/`beamM` are estimated from echo extent (RM-132), never sim truth.

mod estimate;
mod publish;
mod snapshot;

pub use estimate::{
    estimate_dims_from_blob_extent, estimate_dims_from_polar, estimate_dims_from_spans,
    valid_dimensions, HULL_LENGTH_MAX_M, HULL_LENGTH_MIN_M,
};
pub use publish::{
    distance_and_bearing, publish_geo_arpa, GeoArpaMeta, GeoArpaTarget, MonitorContact,
    MonitorContactEnumerator,
};
pub use snapshot::{
    contact_from_track, empty_snapshot, enumerator_from_snapshot, store_contacts,
    MonitorContactSnapshot,
};
