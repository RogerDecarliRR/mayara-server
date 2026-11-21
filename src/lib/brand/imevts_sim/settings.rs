use std::collections::HashMap;

use super::IMEVTS_SIM_RANGES;
use crate::Cli;
use crate::radar::settings::{
    ControlId, HAS_AUTO_NOT_ADJUSTABLE, SharedControls, new_auto, new_list, new_numeric, new_string,
};
use crate::radar::units::Units;
use crate::stream::SignalKDelta;

pub(crate) fn new(
    radar_id: String,
    sk_client_tx: tokio::sync::broadcast::Sender<SignalKDelta>,
    args: &Cli,
    site_key: &str,
) -> SharedControls {
    let mut controls = HashMap::new();
    let label = format!("ImevtsSim {site_key}");

    new_string(ControlId::UserName).build(&mut controls);
    controls
        .get_mut(&ControlId::UserName)
        .unwrap()
        .set_string(label.clone());

    new_string(ControlId::ModelName).build(&mut controls);
    controls
        .get_mut(&ControlId::ModelName)
        .unwrap()
        .set_string("ImevtsSim".to_string());

    new_string(ControlId::SerialNumber).build(&mut controls);
    controls
        .get_mut(&ControlId::SerialNumber)
        .unwrap()
        .set_string(site_key.to_string());

    new_string(ControlId::FirmwareVersion).build(&mut controls);
    controls
        .get_mut(&ControlId::FirmwareVersion)
        .unwrap()
        .set_string("1.0.0 (imevts-sim)".to_string());

    let max_range = *IMEVTS_SIM_RANGES.last().unwrap() as f64;
    new_numeric(ControlId::Range, 0., max_range)
        .wire_units(Units::Meters)
        .build(&mut controls);

    // No Navico-style wire protocol: set_value / Command use GUI units directly.
    // A wire_scale_factor would make Control::set divide (e.g. Gain 55→22, height 35→0).
    new_auto(ControlId::Gain, 0., 100., HAS_AUTO_NOT_ADJUSTABLE).build(&mut controls);
    new_auto(ControlId::Sea, 0., 100., HAS_AUTO_NOT_ADJUSTABLE).build(&mut controls);
    new_auto(ControlId::Rain, 0., 100., HAS_AUTO_NOT_ADJUSTABLE).build(&mut controls);

    new_numeric(ControlId::AntennaHeight, 0., 60.)
        .wire_units(Units::Meters)
        .build(&mut controls);

    new_numeric(ControlId::BearingAlignment, -180., 180.)
        .wire_scale_factor(10., true)
        .wire_offset(-1.)
        .wire_units(Units::Degrees)
        .build(&mut controls);

    new_list(ControlId::RangeUnits, &["Nautical", "Metric", "Mixed"]).build(&mut controls);

    SharedControls::new(radar_id, sk_client_tx, args, controls)
}
