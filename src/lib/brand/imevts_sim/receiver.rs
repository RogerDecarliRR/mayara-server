//! UDP IMRS v1 receiver → `CommonRadar::add_spoke`.

use std::collections::{HashMap, HashSet};
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};

use tokio::net::UdpSocket;
use tokio_graceful_shutdown::SubsystemHandle;

use super::command::Command;
use super::gain::ReceiverLevels;
use super::settings;
use super::{DEFAULT_LISTEN_PORT, IMEVTS_SIM_RANGES, PIXEL_VALUES};
use crate::config::GuardZone;
use crate::radar::range::Ranges;
use crate::radar::settings::{ControlId, RANGE_UNITS_METRIC};
use crate::radar::spoke::GenericSpoke;
use crate::radar::{CommonRadar, Power, RadarError, RadarInfo, SharedRadars};
use crate::{Brand, Cli};

pub(crate) const MAGIC: &[u8] = b"IMRS";
pub(crate) const VERSION: u8 = 1;
pub(crate) const MAX_SITE_KEY: usize = 32;
/// Default spokes/rev (HALO-like lab density). Override via [`imrs_spokes`].
pub(crate) const IMRS_SPOKES_DEFAULT: usize = 2048;
pub(crate) const IMRS_SPOKES_HIRES: usize = 8192;
pub(crate) const IMRS_SPOKE_LEN: usize = 1024;
pub(crate) const HEADING_NONE: u16 = 0xFFFF;

/// Spokes per revolution from `IMEVTS_SIM_SPOKES` or alias `MAYARA_IMRS_SPOKES`
/// (only `2048` or `8192`; default [`IMRS_SPOKES_DEFAULT`]).
pub(crate) fn imrs_spokes() -> usize {
    imrs_spokes_with_env(|k| std::env::var(k).ok())
}

/// Injected env source for unit tests (no `set_var`).
pub(crate) fn imrs_spokes_with_env<F>(env: F) -> usize
where
    F: Fn(&str) -> Option<String>,
{
    for key in ["IMEVTS_SIM_SPOKES", "MAYARA_IMRS_SPOKES"] {
        if let Some(v) = env(key) {
            match v.trim() {
                "8192" => return IMRS_SPOKES_HIRES,
                "2048" => return IMRS_SPOKES_DEFAULT,
                _ => {}
            }
        }
    }
    IMRS_SPOKES_DEFAULT
}

/// Batch ~1/64 of a revolution before WS flush (one UDP datagram = one spoke).
const SPOKE_BATCH: usize = 32;
const NAV_SOURCE: &str = "imevts-sim";

/// Decoded IMRS v1 spoke (lab / unit tests).
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ImrsSpoke {
    pub site_key: String,
    pub lat: f64,
    pub lon: f64,
    pub height_m: f32,
    pub spoke_range_m: u32,
    pub angle: u16,
    pub heading: Option<u16>,
    pub samples_nibble: Vec<u8>,
}

/// Parse one IMRS v1 datagram. Returns `None` on malformed / unsupported input.
pub(crate) fn parse_imrs(packet: &[u8]) -> Option<ImrsSpoke> {
    parse_imrs_with_spokes(packet, imrs_spokes())
}

/// Like [`parse_imrs`] with an explicit spokes count (tests / config).
pub(crate) fn parse_imrs_with_spokes(packet: &[u8], spokes: usize) -> Option<ImrsSpoke> {
    if packet.len() < 6 || &packet[..4] != MAGIC {
        return None;
    }
    if packet[4] != VERSION {
        return None;
    }
    let key_len = packet[5] as usize;
    if key_len == 0 || key_len > MAX_SITE_KEY {
        return None;
    }
    let mut off = 6;
    if packet.len() < off + key_len {
        return None;
    }
    let site_key = std::str::from_utf8(&packet[off..off + key_len])
        .ok()?
        .to_string();
    off += key_len;
    // lat f64, lon f64, height f32, range u32, angle u16, heading u16, n_bins u16
    const TAIL: usize = 8 + 8 + 4 + 4 + 2 + 2 + 2;
    if packet.len() < off + TAIL + IMRS_SPOKE_LEN {
        return None;
    }
    let lat = f64::from_le_bytes(packet[off..off + 8].try_into().ok()?);
    off += 8;
    let lon = f64::from_le_bytes(packet[off..off + 8].try_into().ok()?);
    off += 8;
    let height_m = f32::from_le_bytes(packet[off..off + 4].try_into().ok()?);
    off += 4;
    let spoke_range_m = u32::from_le_bytes(packet[off..off + 4].try_into().ok()?);
    off += 4;
    let angle = u16::from_le_bytes(packet[off..off + 2].try_into().ok()?);
    off += 2;
    let heading_raw = u16::from_le_bytes(packet[off..off + 2].try_into().ok()?);
    off += 2;
    let n_bins = u16::from_le_bytes(packet[off..off + 2].try_into().ok()?);
    off += 2;
    if n_bins as usize != IMRS_SPOKE_LEN {
        return None;
    }
    let spokes_u16 = spokes.min(u16::MAX as usize) as u16;
    if angle >= spokes_u16 {
        return None;
    }
    let heading = if heading_raw == HEADING_NONE {
        None
    } else if heading_raw < spokes_u16 {
        Some(heading_raw)
    } else {
        None
    };
    let mut samples_nibble = Vec::with_capacity(IMRS_SPOKE_LEN);
    for &b in &packet[off..off + IMRS_SPOKE_LEN] {
        samples_nibble.push(b.min(15));
    }
    Some(ImrsSpoke {
        site_key,
        lat,
        lon,
        height_m,
        spoke_range_m,
        angle,
        heading,
        samples_nibble,
    })
}

struct SiteRadar {
    common: CommonRadar,
    command: Option<Command>,
    /// Gain / Sea / Rain snapshot; refreshed only when a control update arrives.
    levels: ReceiverLevels,
}

pub(crate) struct ImevtsSimListener {
    args: Cli,
    radars: SharedRadars,
    port: u16,
    /// Publish antenna position into global navdata when the emulator is not
    /// also owning the boat pose.
    publish_nav: bool,
    sites: HashMap<String, SiteRadar>,
    /// `site_key`s that failed to register (e.g. key collision on last-4 serial).
    rejected: HashSet<String>,
}

impl ImevtsSimListener {
    pub(crate) fn new(args: &Cli, radars: SharedRadars, port: u16) -> Self {
        let publish_nav = !args.emulator_enabled();
        Self {
            args: args.clone(),
            radars,
            port: if port == 0 {
                DEFAULT_LISTEN_PORT
            } else {
                port
            },
            publish_nav,
            sites: HashMap::new(),
            rejected: HashSet::new(),
        }
    }

    pub(crate) async fn run(mut self, subsys: &mut SubsystemHandle) -> Result<(), RadarError> {
        let bind = SocketAddr::from((Ipv4Addr::UNSPECIFIED, self.port));
        // FL / RM-143: 8192 spokes/rev ≈ 3k datagrams/s; default rmem (~212 KiB)
        // drops bursts (ARPA no sítio errado + eco vazio). socket2 → 16 MiB
        // (capped by net.core.rmem_max); depois Tokio UdpSocket.
        const IMRS_RECV_BUF: usize = 16 * 1024 * 1024;
        let socket = {
            use socket2::{Domain, Protocol, Type};
            let sock = socket2::Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))
                .map_err(|e| {
                    log::error!("ImevtsSim: socket new failed: {e}");
                    RadarError::Io(e)
                })?;
            sock.set_nonblocking(true).map_err(RadarError::Io)?;
            sock.set_reuse_address(true).map_err(RadarError::Io)?;
            match sock.set_recv_buffer_size(IMRS_RECV_BUF) {
                Ok(()) => match sock.recv_buffer_size() {
                    Ok(n) => log::info!(
                        "ImevtsSim: UDP recv buffer {n} bytes (requested {IMRS_RECV_BUF})"
                    ),
                    Err(e) => log::warn!("ImevtsSim: recv_buffer_size: {e}"),
                },
                Err(e) => log::warn!("ImevtsSim: set_recv_buffer_size({IMRS_RECV_BUF}): {e}"),
            }
            sock.bind(&bind.into()).map_err(|e| {
                log::error!("ImevtsSim: bind {bind} failed: {e}");
                RadarError::Io(e)
            })?;
            UdpSocket::from_std(sock.into()).map_err(|e| {
                log::error!("ImevtsSim: tokio from_std failed: {e}");
                RadarError::Io(e)
            })?
        };
        log::info!("ImevtsSim: UDP bound on {bind}");

        let mut buf = vec![0u8; 6 + MAX_SITE_KEY + 30 + IMRS_SPOKE_LEN + 64];
        loop {
            tokio::select! {
                _ = subsys.on_shutdown_requested() => {
                    log::debug!("ImevtsSim: shutdown");
                    return Ok(());
                }
                recv = socket.recv_from(&mut buf) => {
                    match recv {
                        Ok((n, _from)) => self.handle_datagram(&buf[..n]).await,
                        Err(e) => {
                            log::warn!("ImevtsSim: recv error: {e}");
                        }
                    }
                }
            }
        }
    }

    async fn handle_datagram(&mut self, packet: &[u8]) {
        let Some(spoke) = parse_imrs(packet) else {
            return;
        };
        if self.rejected.contains(&spoke.site_key) {
            return;
        }
        if !self.sites.contains_key(&spoke.site_key) {
            match self.create_site(&spoke.site_key, spoke.lat, spoke.lon, spoke.height_m) {
                Some(site) => {
                    self.sites.insert(spoke.site_key.clone(), site);
                }
                None => {
                    log::warn!(
                        "ImevtsSim: could not register site_key='{}' (radar key collision?)",
                        spoke.site_key
                    );
                    self.rejected.insert(spoke.site_key.clone());
                    return;
                }
            }
        }
        let Some(site) = self.sites.get_mut(&spoke.site_key) else {
            return;
        };
        site.common.info.mark_input();
        // Asset may relocate; keep CommonRadar + SharedRadars antenna in sync
        // without touching global navdata (that would move the Emulator boat).
        let antenna_moved = site.common.info.antenna_lat != Some(spoke.lat)
            || site.common.info.antenna_lon != Some(spoke.lon);
        site.common.info.antenna_lat = Some(spoke.lat);
        site.common.info.antenna_lon = Some(spoke.lon);
        if antenna_moved {
            site.common.update();
        }
        let mut controls_changed = false;
        while let Ok(cu) = site.common.control_update_rx.try_recv() {
            let _ = site
                .common
                .process_control_update(cu, &mut site.command)
                .await;
            controls_changed = true;
        }
        if controls_changed {
            site.levels = ReceiverLevels::from_controls(&site.common.info.controls);
        }
        if self.publish_nav {
            crate::navdata::set_position(Some(spoke.lat), Some(spoke.lon), NAV_SOURCE);
        }
        // AntennaHeight SoT = first-packet height_m in create_site; do not
        // overwrite per spoke (GUI edits must stick). Never set Range per spoke
        // (GUI ⟂). spoke.spoke_range_m still drives polar metres/bin via add_spoke.

        let mut data: GenericSpoke = spoke.samples_nibble;
        // Before add_spoke so GUI, MEPO and the ARPA blob detector see the same echo.
        if !site.levels.is_neutral() {
            site.levels.apply(&mut data, spoke.spoke_range_m);
        }
        // IMRS `angle` is already geographic north-up. Pass heading Some(0) so
        // bearing = (0+angle)%spokes = angle. None would fall back to
        // navdata::get_heading_true() (emulator boat heading) and rotate the
        // MEPO polar by that heading.
        site.common
            .add_spoke(spoke.spoke_range_m, spoke.angle, Some(0), data);
        // Threshold SPOKE_BATCH (set in create_site) flushes only when ready.
        site.common.send_spoke_message();
    }

    fn create_site(
        &self,
        site_key: &str,
        lat: f64,
        lon: f64,
        height_m: f32,
    ) -> Option<SiteRadar> {
        let fake_addr = SocketAddrV4::new(Ipv4Addr::LOCALHOST, self.port);
        let args_for_controls = self.args.clone();
        let site_for_controls = site_key.to_string();
        let info = RadarInfo::new(
            &self.radars,
            &self.args,
            Brand::ImevtsSim,
            Some(site_key),
            None,
            None,
            PIXEL_VALUES,
            imrs_spokes(),
            IMRS_SPOKE_LEN,
            fake_addr,
            Ipv4Addr::LOCALHOST,
            fake_addr,
            fake_addr,
            fake_addr,
            |id, tx| settings::new(id, tx, &args_for_controls, &site_for_controls),
            false,
            false,
        );

        let mut info = self.radars.add(info)?;
        log::info!(
            "ImevtsSim: radar '{}' for site_key='{}' (lat={lat:.5}, lon={lon:.5})",
            info.key(),
            site_key
        );

        // Shore-fixed antenna at the IMRS asset pose — not the Emulator boat.
        // Keeps ARPA SOG/COG absolute when lab runs --emulator --imevts-sim.
        info.stationary = true;
        info.antenna_lat = Some(lat);
        info.antenna_lon = Some(lon);

        // IMEVTS_SIM_RANGES is metric-only; default RangeUnits=Nautical leaves
        // update_valid_ranges() with an empty list → GUI +/- zoom no-ops.
        let _ = info.controls.set_value(
            &ControlId::RangeUnits,
            (RANGE_UNITS_METRIC as f64).into(),
        );
        let ranges = Ranges::new_by_distance(IMEVTS_SIM_RANGES);
        info.set_ranges(ranges);
        let _ = info
            .controls
            .set_value(&ControlId::Power, (Power::Transmit as i32 as f64).into());
        let _ = info
            .controls
            .set_value(&ControlId::AntennaHeight, (height_m as f64).into());
        // Mid step (4000 m); last()=72000 is too wide for useful PPI zoom.
        let _ = info
            .controls
            .set_value(&ControlId::Range, 4000_f64.into());

        // Align GZ1 to geo-echo usable sector × composition range so Auto-ARPA
        // acquires even when persistence left a disabled zone (same as Emulator).
        let gz1 = self.radars.geo_echo_config().arpa_acquisition_guard_zone();
        info.controls
            .set_guard_zone(&ControlId::GuardZone1, &gz1);
        log::info!(
            "ImevtsSim: Guard zone 1 from geo-echo ({:.0}–{:.0} deg, 0–{:.0} m)",
            gz1.start_angle.to_degrees(),
            gz1.end_angle.to_degrees(),
            gz1.end_distance
        );
        info.controls.set_guard_zone(
            &ControlId::GuardZone2,
            &GuardZone {
                start_angle: 0.0,
                end_angle: 0.0,
                start_distance: 0.0,
                end_distance: 0.0,
                enabled: false,
            },
        );

        self.radars.update(&mut info);

        if self.publish_nav {
            crate::navdata::set_position(Some(lat), Some(lon), NAV_SOURCE);
        }

        let control_update_rx = info.control_update_subscribe();
        let arpa_tx = self.radars.get_arpa_tx();
        let key = info.key();
        let command = Some(Command::new(&info));
        let mut common = CommonRadar::new(
            &self.args,
            key.clone(),
            info,
            self.radars.clone(),
            control_update_rx,
            false,
            arpa_tx,
        );
        common.set_spoke_batch_threshold(SPOKE_BATCH);
        // geo-arpa: expose ARPA tracker tracks (dims estimated from blob extent).
        self.radars.ensure_tracker_monitor_enumerator(&key);
        let levels = ReceiverLevels::from_controls(&common.info.controls);
        Some(SiteRadar {
            common,
            command,
            levels,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode_spoke(
        site_key: &str,
        lat: f64,
        lon: f64,
        height_m: f32,
        spoke_range_m: u32,
        angle: u16,
        heading: u16,
        nibbles: &[u8],
    ) -> Vec<u8> {
        let key = site_key.as_bytes();
        assert!(key.len() <= MAX_SITE_KEY);
        let mut out = Vec::new();
        out.extend_from_slice(MAGIC);
        out.push(VERSION);
        out.push(key.len() as u8);
        out.extend_from_slice(key);
        out.extend_from_slice(&lat.to_le_bytes());
        out.extend_from_slice(&lon.to_le_bytes());
        out.extend_from_slice(&height_m.to_le_bytes());
        out.extend_from_slice(&spoke_range_m.to_le_bytes());
        out.extend_from_slice(&angle.to_le_bytes());
        out.extend_from_slice(&heading.to_le_bytes());
        out.extend_from_slice(&(IMRS_SPOKE_LEN as u16).to_le_bytes());
        let mut data = vec![0u8; IMRS_SPOKE_LEN];
        for (i, &v) in nibbles.iter().enumerate().take(IMRS_SPOKE_LEN) {
            data[i] = v;
        }
        out.extend_from_slice(&data);
        out
    }

    #[test]
    fn parse_roundtrip_basic() {
        let mut nibbles = vec![0u8; IMRS_SPOKE_LEN];
        nibbles[0] = 15;
        nibbles[1] = 7;
        nibbles[10] = 3;
        let pkt = encode_spoke(
            "porto",
            -19.85,
            -40.12,
            12.5,
            6000,
            100,
            HEADING_NONE,
            &nibbles,
        );
        let spoke = parse_imrs(&pkt).expect("parse");
        assert_eq!(spoke.site_key, "porto");
        assert!((spoke.lat - (-19.85)).abs() < 1e-9);
        assert!((spoke.lon - (-40.12)).abs() < 1e-9);
        assert!((spoke.height_m - 12.5).abs() < 1e-3);
        assert_eq!(spoke.spoke_range_m, 6000);
        assert_eq!(spoke.angle, 100);
        assert_eq!(spoke.heading, None);
        assert_eq!(spoke.samples_nibble[0], 15);
        assert_eq!(spoke.samples_nibble[1], 7);
        assert_eq!(spoke.samples_nibble[10], 3);
    }

    #[test]
    fn parse_clamps_nibble_above_15() {
        let mut nibbles = vec![0u8; IMRS_SPOKE_LEN];
        nibbles[0] = 255;
        let pkt = encode_spoke("a", 0.0, 0.0, 0.0, 1000, 0, 42, &nibbles);
        let spoke = parse_imrs(&pkt).unwrap();
        assert_eq!(spoke.samples_nibble[0], 15);
        assert_eq!(spoke.heading, Some(42));
    }

    #[test]
    fn parse_rejects_bad_magic_and_version() {
        let mut pkt = encode_spoke("x", 0.0, 0.0, 0.0, 1, 0, HEADING_NONE, &[0]);
        pkt[0] = b'X';
        assert!(parse_imrs(&pkt).is_none());
        let mut pkt = encode_spoke("x", 0.0, 0.0, 0.0, 1, 0, HEADING_NONE, &[0]);
        pkt[4] = 99;
        assert!(parse_imrs(&pkt).is_none());
    }

    #[test]
    fn parse_rejects_empty_site_key() {
        let mut pkt = encode_spoke("ab", 0.0, 0.0, 0.0, 1, 0, HEADING_NONE, &[0]);
        // Corrupt key_len to 0 (byte after version).
        pkt[5] = 0;
        assert!(parse_imrs(&pkt).is_none());
    }

    #[test]
    fn parse_rejects_truncated() {
        let pkt = encode_spoke("site", 1.0, 2.0, 3.0, 4, 5, HEADING_NONE, &[1, 2, 3]);
        assert!(parse_imrs(&pkt[..20]).is_none());
    }

    #[test]
    fn parse_rejects_angle_at_or_above_spokes() {
        let pkt = encode_spoke(
            "x",
            0.0,
            0.0,
            0.0,
            1000,
            IMRS_SPOKES_DEFAULT as u16,
            HEADING_NONE,
            &[0],
        );
        assert!(parse_imrs_with_spokes(&pkt, IMRS_SPOKES_DEFAULT).is_none());
        assert!(parse_imrs_with_spokes(&pkt, IMRS_SPOKES_HIRES).is_some());
    }

    #[test]
    fn parse_accepts_high_angle_when_spokes_8192() {
        let pkt = encode_spoke("x", 0.0, 0.0, 0.0, 1000, 5000, HEADING_NONE, &[0]);
        assert!(parse_imrs_with_spokes(&pkt, IMRS_SPOKES_DEFAULT).is_none());
        let spoke = parse_imrs_with_spokes(&pkt, IMRS_SPOKES_HIRES).expect("8192");
        assert_eq!(spoke.angle, 5000);
    }

    #[test]
    fn imrs_spokes_from_injected_env() {
        assert_eq!(
            imrs_spokes_with_env(|_| Some("8192".into())),
            IMRS_SPOKES_HIRES
        );
        assert_eq!(
            imrs_spokes_with_env(|_| Some("2048".into())),
            IMRS_SPOKES_DEFAULT
        );
        assert_eq!(
            imrs_spokes_with_env(|_| Some("999".into())),
            IMRS_SPOKES_DEFAULT
        );
        assert_eq!(
            imrs_spokes_with_env(|k| {
                if k == "MAYARA_IMRS_SPOKES" {
                    Some("8192".into())
                } else {
                    None
                }
            }),
            IMRS_SPOKES_HIRES
        );
        assert_eq!(
            imrs_spokes_with_env(|k| {
                if k == "IMEVTS_SIM_SPOKES" {
                    Some("2048".into())
                } else if k == "MAYARA_IMRS_SPOKES" {
                    Some("8192".into())
                } else {
                    None
                }
            }),
            IMRS_SPOKES_DEFAULT,
            "IMEVTS_SIM_SPOKES wins over alias"
        );
    }
}
