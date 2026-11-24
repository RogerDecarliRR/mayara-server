//! Brand that listens for UDP IMRS spokes from the imevts simulator (RM-124).
//!
//! Spokes enter the normal `CommonRadar::add_spoke` path (MEPO / geo-echo
//! unchanged). No viewport paint or GUI Range coupling lives here.

use tokio_graceful_shutdown::{SubsystemBuilder, SubsystemHandle};

use crate::radar::SharedRadars;
use crate::Cli;

mod command;
mod gain;
mod receiver;
mod settings;

/// Like HALO / emulator: 2048 spokes × 1024 bins, 16 legend levels (nibble).
pub(crate) const PIXEL_VALUES: u8 = 16;

/// Supported ranges in meters (metric subset sufficient for lab).
pub(crate) const IMEVTS_SIM_RANGES: &[i32] = &[
    50, 75, 100, 250, 500, 750, 1000, 1500, 2000, 3000, 4000, 6000, 8000, 12000, 16000, 24000,
    36000, 48000, 64000, 72000,
];

/// Default UDP listen port when CLI/env omit one.
pub(crate) const DEFAULT_LISTEN_PORT: u16 = 6510;

/// Start the IMRS UDP listener (one radar per distinct `site_key`).
pub(crate) fn start(args: &Cli, radars: &SharedRadars, subsys: &SubsystemHandle) {
    let port = args.imevts_sim_listen_port();
    log::info!("ImevtsSim: listening for IMRS UDP on 0.0.0.0:{port}");
    let listener = receiver::ImevtsSimListener::new(args, radars.clone(), port);
    subsys.start(SubsystemBuilder::new(
        "imevts-sim-imrs",
        async move |s: &mut SubsystemHandle| listener.run(s).await,
    ));
}
