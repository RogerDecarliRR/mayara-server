//! Monitor-only ARPA feed (`geo-arpa.json`), independent of GUI Range / blob tracker.
//!
//! Coverage reuses [`crate::config::GeoEchoConfig`] (`compositionRangeM` × usable
//! bearings). Lab source: emulator world contacts via
//! [`MonitorContactEnumerator`]. Hardware without an enumerator returns
//! `available: false` and an empty target list.

mod publish;

pub use publish::{
    distance_and_bearing, publish_geo_arpa, GeoArpaMeta, GeoArpaTarget, MonitorContact,
    MonitorContactEnumerator,
};
