//! Monitor-only ARPA feed (`geo-arpa.json`), independent of GUI Range / blob tracker.
//!
//! Coverage reuses [`crate::config::GeoEchoConfig`] (`usableRadiusM` × usable
//! bearings; composition is canvas only). Lab source: emulator world contacts via
//! [`MonitorContactEnumerator`], or ARPA tracker snapshot (ImevtsSim / hardware).
//! Hardware without an enumerator returns `available: false` and an empty target list.
//! Optional `lengthM`/`beamM` are estimated from echo extent (RM-132), never sim truth.
//! Optional `heading` (rad) from PCA of the polar imprint (RM-138), separate from `cog`.

mod estimate;
mod publish;
mod snapshot;

pub use estimate::{
    blob_spans_m, estimate_dims_from_blob_extent, estimate_dims_from_polar, estimate_dims_from_spans,
    estimate_heading_rad_from_enu, estimate_heading_rad_from_polar_pixels, sample_polar_pixels,
    spoke_pixel_to_enu, stabilize_heading_rad, valid_dimensions, HEADING_PCA_MAX_POINTS,
    HULL_LENGTH_MAX_M, HULL_LENGTH_MIN_M,
};
pub use publish::{
    distance_and_bearing, publish_geo_arpa, GeoArpaMeta, GeoArpaTarget, MonitorContact,
    MonitorContactEnumerator,
};
pub use snapshot::{
    contact_from_track, empty_snapshot, enumerator_from_snapshot, store_contacts,
    MonitorContactSnapshot,
};
