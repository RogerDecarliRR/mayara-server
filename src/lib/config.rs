use directories::ProjectDirs;
use log::{debug, error, info, warn};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fs;
use std::fs::File;
use std::io::{BufReader, BufWriter, Write};
use std::path::PathBuf;
use std::time::SystemTime;
use utoipa::ToSchema;

use crate::Brand;
use crate::radar::RadarInfo;
use crate::radar::range::Ranges;
use crate::radar::settings::{ControlId, DEFAULT_AUTO_STANDBY};

pub(crate) fn get_project_dirs() -> ProjectDirs {
    directories::ProjectDirs::from("net", "verruijt", "mayara")
        .expect("Cannot find project directories")
}

pub(crate) fn default_range_units() -> i32 {
    0 // Nautical (default)
}

fn default_auto_standby() -> i32 {
    DEFAULT_AUTO_STANDBY as i32
}

#[derive(Serialize, Deserialize, Debug, Default, Clone, PartialEq)]
pub struct GuardZone {
    pub start_angle: f64,    // Start angle in radians (SI)
    pub end_angle: f64,      // End angle in radians (SI)
    pub start_distance: f64, // Inner distance in meters
    pub end_distance: f64,   // Outer distance in meters
    pub enabled: bool,
}

/// Exclusion zone for stationary radar installations.
/// Areas within exclusion zones are displayed differently and targets are not tracked.
pub type ExclusionZone = GuardZone;

/// Rectangular exclusion zone for stationary radar installations.
/// Defined by two corners (x1,y1) and (x2,y2) that form one edge, plus a width.
/// Coordinates are in meters from radar position (x=east, y=north).
#[derive(Serialize, Deserialize, Debug, Default, Clone, PartialEq)]
pub struct ExclusionRect {
    pub x1: f64,    // First corner X (meters from radar, positive = east)
    pub y1: f64,    // First corner Y (meters from radar, positive = north)
    pub x2: f64,    // Second corner X (defines one edge with first corner)
    pub y2: f64,    // Second corner Y (defines one edge with first corner)
    pub width: f64, // Perpendicular width of rectangle (always positive)
    pub enabled: bool,
}

#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub(crate) struct Radar {
    pub id: usize,
    pub user_name: String,
    #[serde(default)]
    pub spoke_processing: i32, // 0 = Clean, 1 = Fill, 2 = Reduce, 3 = Smooth
    #[serde(default = "default_range_units")]
    pub range_units: i32, // 0 = Nautical, 1 = Metric, 2 = Mixed, 3 = Statute, 4 = Kiloyards
    #[serde(default = "default_auto_standby")]
    pub auto_standby: i32, // Index into the AutoStandby control's choices

    // Data that is computed and not immediately known when starting
    pub model_name: Option<String>, // Descriptive model name (4G, HALO)
    pub ranges: Option<Vec<i32>>,   // Detected ranges

    // Guard zones
    #[serde(default)]
    pub guard_zone_1: Option<GuardZone>,
    #[serde(default)]
    pub guard_zone_2: Option<GuardZone>,

    // Exclusion zones (stationary installations only)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclusion_zone_1: Option<ExclusionZone>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclusion_zone_2: Option<ExclusionZone>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclusion_zone_3: Option<ExclusionZone>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclusion_zone_4: Option<ExclusionZone>,

    // Rectangular exclusion zones (stationary installations only)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclusion_rect_1: Option<ExclusionRect>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclusion_rect_2: Option<ExclusionRect>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclusion_rect_3: Option<ExclusionRect>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclusion_rect_4: Option<ExclusionRect>,

    // ARPA/Target tracking settings
    #[serde(default)]
    pub arpa_max_speed: i32, // 0 = Normal (25kn), 1 = Medium (40kn), 2 = Fast (50kn)
    #[serde(default)]
    pub doppler_auto_track: bool,
}

/// Where the answer to the telemetry question lives, so the user is asked
/// once and never again. A run that cannot store settings cannot remember an
/// answer either, and so never asks and never reports.
#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub(crate) struct TelemetryConfig {
    /// Random id for this install, created when consent is first given so
    /// repeat runs are not counted as separate installs. Never derived from
    /// anything about the boat, the radar or the network.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub install_id: Option<String>,
    /// `None` until the user has answered.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub consent: Option<bool>,
}

/// Fixed composition window for the north-up geo echo overlay.
/// Independent of the operator's `ControlId::Range`.
///
/// `composition_range_m` = canvas/disco half-side; operational emission uses
/// `usable_radius_m` (≤ composition) × usable bearings.
pub const GEO_ECHO_MIN_RANGE_M: f64 = 50.0;
pub const GEO_ECHO_MAX_RANGE_M: f64 = 120.0 * 1852.0;
pub const GEO_ECHO_ALLOWED_SIZES: &[u32] = &[1024, 2048, 4096];
pub const GEO_ECHO_MIN_TARGET_M_PER_PX: f64 = 0.25;
pub const GEO_ECHO_MAX_TARGET_M_PER_PX: f64 = 100.0;

fn default_composition_range_m() -> f64 {
    14.0 * 1852.0
}

fn default_echo_size_px() -> u32 {
    2048
}

fn default_target_meters_per_px() -> f64 {
    2.0
}

fn default_max_size_px() -> u32 {
    2048
}

fn default_usable_bearing_start_deg() -> f64 {
    0.0
}

fn default_usable_bearing_end_deg() -> f64 {
    360.0
}

/// How the geo-echo overlay is produced for MapLibre.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Default, ToSchema)]
#[serde(rename_all = "camelCase")]
pub enum GeoEchoMode {
    /// Fixed square centred on the radar (compositionRangeM × echoSizePx).
    Composition,
    /// Client-driven map bbox via `POST …/echo/viewport` (recommended).
    #[default]
    Viewport,
}

/// Where polar MEPO is painted for the monitor overlay.
///
/// `sidecar` (default): push MEPO to `GEO_ECHO_RENDERER_URL` for PNG paint.
/// `gpu`: browser/GPU samples MEPO directly — skip renderer PUT.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Default, ToSchema)]
#[serde(rename_all = "camelCase")]
pub enum EchoPaint {
    /// Paint via geo-echo-renderer (D1 PNG path).
    #[default]
    Sidecar,
    /// Client samples MEPO on GPU; no renderer PUT.
    Gpu,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct GeoEchoConfig {
    /// Active overlay workflow (`viewport` recommended).
    #[serde(default)]
    pub mode: GeoEchoMode,
    /// Half-side of the composition square in metres (image edge to centre).
    /// Canvas / disco only; GUI zoom is separate. Operational emission uses
    /// [`Self::usable_radius_m`].
    #[serde(default = "default_composition_range_m")]
    pub composition_range_m: f64,
    /// PNG edge length in pixels for composition mode; only 1024, 2048 or 4096.
    #[serde(default = "default_echo_size_px")]
    pub echo_size_px: u32,
    /// Target metres per pixel for viewport mode (ideal scale).
    #[serde(default = "default_target_meters_per_px")]
    pub target_meters_per_px: f64,
    /// Cap on viewport PNG edge length; only 1024, 2048 or 4096.
    #[serde(default = "default_max_size_px")]
    pub max_size_px: u32,
    /// Ceiling for viewport range-step snap (metres). `None` → use
    /// [`Self::composition_range_m`] (default 14 NM).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_viewport_range_m: Option<f64>,
    /// Usable sector start bearing in degrees (North = 0, clockwise).
    #[serde(default = "default_usable_bearing_start_deg")]
    pub usable_bearing_start_deg: f64,
    /// Usable sector end bearing in degrees (North = 0, clockwise; 360 = full).
    #[serde(default = "default_usable_bearing_end_deg")]
    pub usable_bearing_end_deg: f64,
    /// Operational usable radius in metres (echo + geo-arpa emission).
    /// `None` / omitted / legacy → equals [`Self::composition_range_m`].
    /// Must be ≤ composition; [`Self::normalized`] clamps when larger.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub usable_radius_m: Option<f64>,
    /// Monitor paint path: `sidecar` (PUT renderer) or `gpu` (MEPO to client).
    #[serde(default, rename = "echoPaint", alias = "echo_paint")]
    pub echo_paint: EchoPaint,
    /// ARPA blob acquisition scale: higher = more sensitive (lower threshold,
    /// fewer pixels required). Clamped to [`ARPA_SENSITIVITY_MIN`]..=[`ARPA_SENSITIVITY_MAX`].
    #[serde(default = "default_arpa_sensitivity", rename = "arpaSensitivity")]
    pub arpa_sensitivity: f64,
}

/// Lower bound for [`GeoEchoConfig::arpa_sensitivity`].
pub const ARPA_SENSITIVITY_MIN: f64 = 0.5;
/// Upper bound for [`GeoEchoConfig::arpa_sensitivity`].
pub const ARPA_SENSITIVITY_MAX: f64 = 2.0;

fn default_arpa_sensitivity() -> f64 {
    1.0
}

impl Default for GeoEchoConfig {
    fn default() -> Self {
        Self {
            mode: GeoEchoMode::Viewport,
            composition_range_m: default_composition_range_m(),
            echo_size_px: default_echo_size_px(),
            target_meters_per_px: default_target_meters_per_px(),
            max_size_px: default_max_size_px(),
            max_viewport_range_m: None,
            usable_bearing_start_deg: default_usable_bearing_start_deg(),
            usable_bearing_end_deg: default_usable_bearing_end_deg(),
            usable_radius_m: None,
            echo_paint: EchoPaint::Sidecar,
            arpa_sensitivity: default_arpa_sensitivity(),
        }
    }
}

impl GeoEchoConfig {
    /// Range-step snap ceiling: explicit `max_viewport_range_m` or composition.
    pub fn max_viewport_range_ceiling(&self) -> f64 {
        self.max_viewport_range_m
            .filter(|v| v.is_finite() && *v > 0.0)
            .unwrap_or(self.composition_range_m)
    }

    /// Effective usable radius (metres): explicit value or composition canvas.
    ///
    /// Always ≤ [`Self::composition_range_m`] after [`Self::normalized`].
    pub fn effective_usable_radius_m(&self) -> f64 {
        let composition = self.composition_range_m;
        match self.usable_radius_m {
            Some(m)
                if m.is_finite()
                    && (GEO_ECHO_MIN_RANGE_M..=GEO_ECHO_MAX_RANGE_M).contains(&m) =>
            {
                m.min(composition)
            }
            _ => composition,
        }
    }

    /// Clamp / repair values loaded from disk so the compositor always sees a
    /// usable window.
    pub fn normalized(mut self) -> Self {
        if !(GEO_ECHO_MIN_RANGE_M..=GEO_ECHO_MAX_RANGE_M).contains(&self.composition_range_m) {
            self.composition_range_m = default_composition_range_m();
        }
        if !GEO_ECHO_ALLOWED_SIZES.contains(&self.echo_size_px) {
            self.echo_size_px = default_echo_size_px();
        }
        if !(GEO_ECHO_MIN_TARGET_M_PER_PX..=GEO_ECHO_MAX_TARGET_M_PER_PX)
            .contains(&self.target_meters_per_px)
        {
            self.target_meters_per_px = default_target_meters_per_px();
        }
        if !GEO_ECHO_ALLOWED_SIZES.contains(&self.max_size_px) {
            self.max_size_px = default_max_size_px();
        }
        if let Some(m) = self.max_viewport_range_m {
            if !(GEO_ECHO_MIN_RANGE_M..=GEO_ECHO_MAX_RANGE_M).contains(&m) {
                self.max_viewport_range_m = None;
            }
        }
        self.usable_bearing_start_deg = normalize_bearing_deg(self.usable_bearing_start_deg);
        self.usable_bearing_end_deg = normalize_bearing_end_deg(self.usable_bearing_end_deg);
        self.usable_radius_m = match self.usable_radius_m {
            None => None,
            Some(m) if !m.is_finite() => None,
            Some(m) if !(GEO_ECHO_MIN_RANGE_M..=GEO_ECHO_MAX_RANGE_M).contains(&m) => None,
            Some(m) if m > self.composition_range_m => Some(self.composition_range_m),
            Some(m) => Some(m),
        };
        self.arpa_sensitivity = self.clamped_arpa_sensitivity();
        self
    }

    /// ARPA sensitivity clamped to the supported range.
    pub fn clamped_arpa_sensitivity(&self) -> f64 {
        if !self.arpa_sensitivity.is_finite() {
            return default_arpa_sensitivity();
        }
        self.arpa_sensitivity
            .clamp(ARPA_SENSITIVITY_MIN, ARPA_SENSITIVITY_MAX)
    }

    /// Guard zone 1 for Auto-ARPA acquisition: usable bearings × usable radius.
    ///
    /// Angles are converted from degrees (N=0 CW) to radians; equal start/end
    /// means a full circle (see blob detector). `enabled` is always true.
    /// `end_distance` is the operational usable radius, not the composition canvas.
    pub fn arpa_acquisition_guard_zone(&self) -> GuardZone {
        let n = self.clone().normalized();
        GuardZone {
            start_angle: n.usable_bearing_start_deg.to_radians(),
            end_angle: n.usable_bearing_end_deg.to_radians(),
            start_distance: 0.0,
            end_distance: n.effective_usable_radius_m(),
            enabled: true,
        }
    }

    /// Validate a value coming from the API.
    pub fn validate(&self) -> Result<(), String> {
        if !(GEO_ECHO_MIN_RANGE_M..=GEO_ECHO_MAX_RANGE_M).contains(&self.composition_range_m) {
            return Err(format!(
                "compositionRangeM must be between {} and {} metres",
                GEO_ECHO_MIN_RANGE_M, GEO_ECHO_MAX_RANGE_M
            ));
        }
        if !GEO_ECHO_ALLOWED_SIZES.contains(&self.echo_size_px) {
            return Err(format!(
                "echoSizePx must be one of {:?}",
                GEO_ECHO_ALLOWED_SIZES
            ));
        }
        if !(GEO_ECHO_MIN_TARGET_M_PER_PX..=GEO_ECHO_MAX_TARGET_M_PER_PX)
            .contains(&self.target_meters_per_px)
        {
            return Err(format!(
                "targetMetersPerPx must be between {} and {}",
                GEO_ECHO_MIN_TARGET_M_PER_PX, GEO_ECHO_MAX_TARGET_M_PER_PX
            ));
        }
        if !GEO_ECHO_ALLOWED_SIZES.contains(&self.max_size_px) {
            return Err(format!(
                "maxSizePx must be one of {:?}",
                GEO_ECHO_ALLOWED_SIZES
            ));
        }
        if let Some(m) = self.max_viewport_range_m {
            if !(GEO_ECHO_MIN_RANGE_M..=GEO_ECHO_MAX_RANGE_M).contains(&m) {
                return Err(format!(
                    "maxViewportRangeM must be between {} and {} metres",
                    GEO_ECHO_MIN_RANGE_M, GEO_ECHO_MAX_RANGE_M
                ));
            }
        }
        if let Some(m) = self.usable_radius_m {
            if !(GEO_ECHO_MIN_RANGE_M..=GEO_ECHO_MAX_RANGE_M).contains(&m) {
                return Err(format!(
                    "usableRadiusM must be between {} and {} metres",
                    GEO_ECHO_MIN_RANGE_M, GEO_ECHO_MAX_RANGE_M
                ));
            }
        }
        if !bearing_in_closed_range(self.usable_bearing_start_deg) {
            return Err(
                "usableBearingStartDeg must be between 0 and 360 (inclusive)".to_string(),
            );
        }
        if !bearing_in_closed_range(self.usable_bearing_end_deg) {
            return Err("usableBearingEndDeg must be between 0 and 360 (inclusive)".to_string());
        }
        if !self.arpa_sensitivity.is_finite()
            || self.arpa_sensitivity < ARPA_SENSITIVITY_MIN
            || self.arpa_sensitivity > ARPA_SENSITIVITY_MAX
        {
            return Err(format!(
                "arpaSensitivity must be between {} and {}",
                ARPA_SENSITIVITY_MIN, ARPA_SENSITIVITY_MAX
            ));
        }
        Ok(())
    }
}

fn normalize_bearing_deg(deg: f64) -> f64 {
    if !deg.is_finite() {
        return 0.0;
    }
    let mut d = deg % 360.0;
    if d < 0.0 {
        d += 360.0;
    }
    if (d - 360.0).abs() < 1e-9 {
        0.0
    } else {
        d
    }
}

fn normalize_bearing_end_deg(deg: f64) -> f64 {
    if !deg.is_finite() {
        return 360.0;
    }
    if (deg - 360.0).abs() < 1e-9 || deg > 360.0 && (deg % 360.0).abs() < 1e-9 {
        return 360.0;
    }
    normalize_bearing_deg(deg)
}

fn bearing_in_closed_range(deg: f64) -> bool {
    deg.is_finite() && (0.0..=360.0).contains(&deg)
}

#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub(crate) struct Config {
    pub radars: HashMap<String, Radar>,
    #[serde(default)]
    pub telemetry: TelemetryConfig,
    #[serde(default)]
    pub geo_echo: GeoEchoConfig,
}

/// What the server may do about telemetry, and whether the GUI should put the
/// question to the user.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Consent {
    /// Settings are not being stored, so an answer could not be remembered.
    /// Never ask, never report.
    Unavailable,
    /// The user has not answered yet.
    Unasked,
    Granted,
    Denied,
}

#[derive(Debug, Clone)]
pub(crate) struct Persistence {
    pub config: Config,
    timestamp: SystemTime,
    path: PathBuf,
    /// The settings file this run wanted but cannot write. `None` when
    /// settings are stored, and when a replay run wants none.
    unwritable: Option<PathBuf>,
    /// Set when an answer to the telemetry question could not be written.
    /// The stored answer no longer matches what the user asked for, so this
    /// run neither reports nor asks again.
    consent_unwritable: bool,
    /// Radars whose model-dependent controls have taken their saved values.
    model_controls_restored: HashSet<String>,
}

const SETTINGS_FILE: &str = "settings.json";

/// A radar this install has seen before, remembered across restarts.
///
/// Discovery is the same for everyone, but the advice is not: someone whose
/// radar worked last week has a different problem from someone setting one up
/// for the first time, and the page can only say so if it knows.
#[derive(Debug, Clone, PartialEq)]
pub struct KnownRadar {
    pub brand: Option<Brand>,
    /// The name the user gave it, or the one the radar reported.
    pub name: String,
    /// Model as last detected, e.g. `HALO24`.
    pub model: Option<String>,
}

/// What this run does with radar settings. Reported by the status endpoint
/// and, when settings are going nowhere, sent to every connecting client as
/// a Signal K notification.
#[derive(Debug, Clone, PartialEq)]
pub enum SettingsStorage {
    /// Settings are read from and written to this file.
    Stored(PathBuf),
    /// This file cannot be written, so nothing the user sets up is
    /// remembered. Named, so they know which permissions to fix.
    Unwritable(PathBuf),
    /// A replay run keeps nothing on purpose.
    NotWanted,
}

/// What a run can do with the settings file it found.
enum Loaded {
    /// Settings are read and written normally.
    Writable,
    /// Existing settings were read but cannot be changed -- a file that
    /// belongs to someone else.
    ReadOnly,
    /// Nothing could be read and nothing can be written.
    Unusable,
}

/// The settings file to read and write, or `None` when the config directory
/// cannot be created. A radar works fine without one, so an unwritable
/// config directory -- a read-only mount, a host directory owned by another
/// user -- costs the user their remembered settings, never their radar.
fn settings_path(config_dir: &std::path::Path) -> Option<PathBuf> {
    if let Err(e) = fs::create_dir_all(config_dir) {
        warn!(
            "Cannot create settings directory '{}': {}",
            config_dir.display(),
            e
        );
        warn!("Radar names, guard zones and other settings will not be remembered");
        return None;
    }
    Some(config_dir.join(SETTINGS_FILE))
}

impl Persistence {
    /// A persistence that remembers nothing. An empty path is the marker
    /// every write path already checks; `unwritable` names the file this run
    /// failed to write, and stays `None` when it never wanted one.
    fn disabled(unwritable: Option<PathBuf>) -> Self {
        Persistence {
            config: Config::default(),
            timestamp: SystemTime::UNIX_EPOCH,
            path: PathBuf::new(),
            unwritable,
            consent_unwritable: false,
            model_controls_restored: HashSet::new(),
        }
    }

    /// Every radar in the settings file, whether or not it is on the air now.
    pub(crate) fn known_radars(&self) -> Vec<KnownRadar> {
        let mut known: Vec<KnownRadar> = self
            .config
            .radars
            .iter()
            .map(|(key, radar)| KnownRadar {
                brand: Brand::from_key(key),
                name: if radar.user_name.is_empty() {
                    key.clone()
                } else {
                    radar.user_name.clone()
                },
                model: radar.model_name.clone(),
            })
            .collect();
        // A map's order is nobody's idea of an order; the page shows these to
        // a person.
        known.sort_by(|a, b| a.name.cmp(&b.name));
        known
    }

    pub(crate) fn storage(&self) -> SettingsStorage {
        match &self.unwritable {
            Some(path) => SettingsStorage::Unwritable(path.clone()),
            None if self.path.as_os_str().is_empty() => SettingsStorage::NotWanted,
            None => SettingsStorage::Stored(self.path.clone()),
        }
    }

    pub(crate) fn new() -> Self {
        if crate::replay::is_active() {
            debug!("persistence disabled in pcap replay mode");
            return Self::disabled(None);
        }
        // A unit test that stores a radar must not write into the settings
        // of whoever runs the tests.
        if cfg!(test) {
            return Self::disabled(None);
        }

        let config_dir = get_project_dirs().config_dir().to_owned();
        let Some(settings_path) = settings_path(&config_dir) else {
            return Self::disabled(Some(config_dir.join(SETTINGS_FILE)));
        };

        let mut this = Persistence {
            config: Config::default(),
            timestamp: SystemTime::UNIX_EPOCH,
            path: settings_path,
            unwritable: None,
            consent_unwritable: false,
            model_controls_restored: HashSet::new(),
        };

        match this.load() {
            Loaded::Writable => {
                debug!("persistence loaded: {:?}", this);
                this
            }
            Loaded::ReadOnly => {
                warn!("Radar names, guard zones and other settings will not be remembered");
                this.read_only()
            }
            Loaded::Unusable => {
                warn!("Radar names, guard zones and other settings will not be remembered");
                Self::disabled(Some(this.path))
            }
        }
    }

    /// Keep the settings that were read, write no more of them. A file owned
    /// by another user still describes this radar correctly -- the user keeps
    /// their zones and names for this run -- it just cannot take changes.
    fn read_only(self) -> Self {
        Persistence {
            config: self.config,
            timestamp: self.timestamp,
            unwritable: Some(self.path),
            path: PathBuf::new(),
            consent_unwritable: self.consent_unwritable,
            model_controls_restored: self.model_controls_restored,
        }
    }

    /// Whether the settings file will take a change. Existing settings can be
    /// readable and still belong to another user, which is what a container
    /// mount produces after the uid it runs as changes; opening for writing
    /// is the only way to find out, and it leaves the contents alone.
    fn is_writable(&self) -> bool {
        match fs::OpenOptions::new().write(true).open(&self.path) {
            Ok(_) => true,
            Err(e) => {
                warn!("cannot write config '{}': {}", self.path.display(), e);
                false
            }
        }
    }

    fn get_file_time(&self) -> SystemTime {
        let metadata = fs::metadata(&self.path);

        match metadata {
            Ok(data) => {
                if let Ok(time) = data.modified() {
                    return time;
                }
            }
            Err(e) => {
                error!("{e}");
            }
        }

        panic!(
            "Cannot check file modification of '{}' on this platform",
            self.path.display()
        );
    }

    /// What this run can do with the settings file. A first run writes an
    /// empty one, which doubles as the check that this run can write at all:
    /// `create_dir_all` succeeds on a directory that already exists but
    /// belongs to another user, so the write is what finds that out. An
    /// existing file gets the same question asked of it directly.
    fn load(&mut self) -> Loaded {
        let file = match File::open(&self.path) {
            Err(e) => {
                warn!(
                    "no config '{}' yet; starting fresh: {}",
                    self.path.display(),
                    e
                );

                return if self.save() {
                    Loaded::Writable
                } else {
                    Loaded::Unusable
                };
            }
            Ok(f) => f,
        };

        let reader = BufReader::new(file);

        match serde_json::from_reader(reader) {
            Ok(u) => {
                self.config = u;
                info!("Loaded config from '{}'", self.path.display());
            }
            Err(e) => {
                warn!(
                    "Config '{}' corrupted; starting fresh: {}",
                    self.path.display(),
                    e
                );
            }
        };

        self.timestamp = self.get_file_time();

        if self.is_writable() {
            Loaded::Writable
        } else {
            Loaded::ReadOnly
        }
    }

    fn saver(&mut self) -> Result<(), Box<dyn Error>> {
        let file = File::create(&self.path)?;

        let mut writer = BufWriter::new(&file);

        serde_json::to_writer_pretty(writer.by_ref(), &self.config)?;
        writeln!(writer)?;
        writer.flush()?;

        info!("Written config file '{}'", self.path.display());
        self.timestamp = self.get_file_time();
        Ok(())
    }

    fn save(&mut self) -> bool {
        match self.saver() {
            Err(e) => {
                warn!("cannot store config '{}': {}", self.path.display(), e);
                false
            }
            Ok(()) => true,
        }
    }

    /// Whether this run can remember anything at all.
    fn is_storing(&self) -> bool {
        matches!(self.storage(), SettingsStorage::Stored(_))
    }

    pub(crate) fn consent(&self) -> Consent {
        if !self.is_storing() || self.consent_unwritable {
            return Consent::Unavailable;
        }
        match self.config.telemetry.consent {
            None => Consent::Unasked,
            Some(true) => Consent::Granted,
            Some(false) => Consent::Denied,
        }
    }

    /// Record the user's answer. Withdrawing consent drops the install id, so
    /// a later yes is a new install rather than the old one resurfacing.
    ///
    /// An answer that cannot be written is not an answer: the file still
    /// holds the previous one, which a restart would restore -- turning a
    /// withdrawn consent back into a granted one behind the user's back. So a
    /// failed write puts this run in the same place as having no settings
    /// file at all: nothing is reported, and nothing more is asked.
    pub(crate) fn set_consent(&mut self, granted: bool) -> Consent {
        if !self.is_storing() || self.consent_unwritable {
            return Consent::Unavailable;
        }

        let previous = self.config.telemetry.clone();
        self.config.telemetry.consent = Some(granted);
        if !granted {
            self.config.telemetry.install_id = None;
        }

        if !self.save() {
            self.config.telemetry = previous;
            self.consent_unwritable = true;
            return Consent::Unavailable;
        }

        self.consent()
    }

    /// The id this install reports under, created on first use. Only ever
    /// called once reporting is allowed, so creating one here is not a
    /// decision about whether to report -- that has already been made.
    pub(crate) fn ensure_install_id(&mut self) -> Option<String> {
        if self.consent() == Consent::Unavailable {
            return None;
        }

        if self.config.telemetry.install_id.is_none() {
            self.config.telemetry.install_id = Some(uuid::Uuid::new_v4().to_string());
            // An id that cannot be stored would be a fresh one on every
            // start, counting one install many times over.
            if !self.save() {
                self.config.telemetry.install_id = None;
                return None;
            }
        }
        self.config.telemetry.install_id.clone()
    }

    /// Live geo-echo composition settings (normalized).
    pub(crate) fn geo_echo(&self) -> GeoEchoConfig {
        self.config.geo_echo.clone().normalized()
    }

    /// Persist geo-echo settings. Returns the stored (validated + normalized)
    /// config, or an error when validation fails or the settings file cannot
    /// be written.
    pub(crate) fn set_geo_echo(&mut self, cfg: GeoEchoConfig) -> Result<GeoEchoConfig, String> {
        cfg.validate()?;
        let stored = cfg.normalized();
        let previous = self.config.geo_echo.clone();
        self.config.geo_echo = stored.clone();
        if self.is_storing() && !self.save() {
            self.config.geo_echo = previous;
            return Err("cannot store geo-echo settings".to_string());
        }
        Ok(stored)
    }

    pub(crate) fn store(&mut self, radar_info: &RadarInfo) {
        self.restore_model_controls(radar_info);
        if self.path.as_os_str().is_empty() {
            return; // Pcap replay mode — no persistence
        }
        let mut modified = false;

        let radar = self.config.radars.entry(radar_info.key()).or_default();

        let user_name = radar_info.controls.user_name();
        if radar.user_name != user_name {
            radar.user_name = user_name;
            modified = true;
        }
        let spoke_processing = radar_info.controls.spoke_processing();
        if radar.spoke_processing != spoke_processing {
            radar.spoke_processing = spoke_processing;
            modified = true;
        }
        let range_units = radar_info.controls.range_units();
        if radar.range_units != range_units {
            radar.range_units = range_units;
            modified = true;
        }
        if let Some(auto_standby) = radar_info.controls.auto_standby_index()
            && radar.auto_standby != auto_standby
        {
            radar.auto_standby = auto_standby;
            modified = true;
        }
        let ranges = Some(radar_info.ranges.all.iter().map(|r| r.distance()).collect());
        if radar.ranges != ranges {
            radar.ranges = ranges;
            modified = true;
        }

        let model_name = radar_info.controls.model_name();
        if radar.model_name != model_name {
            radar.model_name = model_name;
            modified = true;
        }

        let guard_zone_1 = radar_info.controls.guard_zone(&ControlId::GuardZone1);
        if radar.guard_zone_1 != guard_zone_1 {
            radar.guard_zone_1 = guard_zone_1;
            modified = true;
        }

        let guard_zone_2 = radar_info.controls.guard_zone(&ControlId::GuardZone2);
        if radar.guard_zone_2 != guard_zone_2 {
            radar.guard_zone_2 = guard_zone_2;
            modified = true;
        }

        // Only persist enabled exclusion zones to keep settings.json clean
        let exclusion_zone_1 = radar_info
            .controls
            .exclusion_zone(&ControlId::ExclusionZone1)
            .filter(|z| z.enabled);
        if radar.exclusion_zone_1 != exclusion_zone_1 {
            radar.exclusion_zone_1 = exclusion_zone_1;
            modified = true;
        }

        let exclusion_zone_2 = radar_info
            .controls
            .exclusion_zone(&ControlId::ExclusionZone2)
            .filter(|z| z.enabled);
        if radar.exclusion_zone_2 != exclusion_zone_2 {
            radar.exclusion_zone_2 = exclusion_zone_2;
            modified = true;
        }

        let exclusion_zone_3 = radar_info
            .controls
            .exclusion_zone(&ControlId::ExclusionZone3)
            .filter(|z| z.enabled);
        if radar.exclusion_zone_3 != exclusion_zone_3 {
            radar.exclusion_zone_3 = exclusion_zone_3;
            modified = true;
        }

        let exclusion_zone_4 = radar_info
            .controls
            .exclusion_zone(&ControlId::ExclusionZone4)
            .filter(|z| z.enabled);
        if radar.exclusion_zone_4 != exclusion_zone_4 {
            radar.exclusion_zone_4 = exclusion_zone_4;
            modified = true;
        }

        // Only persist enabled exclusion rects to keep settings.json clean
        let exclusion_rect_1 = radar_info
            .controls
            .exclusion_rect(&ControlId::ExclusionRect1)
            .filter(|r| r.enabled);
        if radar.exclusion_rect_1 != exclusion_rect_1 {
            radar.exclusion_rect_1 = exclusion_rect_1;
            modified = true;
        }

        let exclusion_rect_2 = radar_info
            .controls
            .exclusion_rect(&ControlId::ExclusionRect2)
            .filter(|r| r.enabled);
        if radar.exclusion_rect_2 != exclusion_rect_2 {
            radar.exclusion_rect_2 = exclusion_rect_2;
            modified = true;
        }

        let exclusion_rect_3 = radar_info
            .controls
            .exclusion_rect(&ControlId::ExclusionRect3)
            .filter(|r| r.enabled);
        if radar.exclusion_rect_3 != exclusion_rect_3 {
            radar.exclusion_rect_3 = exclusion_rect_3;
            modified = true;
        }

        let exclusion_rect_4 = radar_info
            .controls
            .exclusion_rect(&ControlId::ExclusionRect4)
            .filter(|r| r.enabled);
        if radar.exclusion_rect_4 != exclusion_rect_4 {
            radar.exclusion_rect_4 = exclusion_rect_4;
            modified = true;
        }

        let arpa_max_speed = radar_info.controls.arpa_detect_max_speed();
        if radar.arpa_max_speed != arpa_max_speed {
            radar.arpa_max_speed = arpa_max_speed;
            modified = true;
        }

        // Until the model is known there is no Doppler Auto Track control,
        // and its absence must not overwrite the saved choice.
        if radar_info
            .controls
            .contains_key(&ControlId::DopplerAutoTrack)
        {
            let doppler_auto_track = radar_info.controls.doppler_auto_track();
            if radar.doppler_auto_track != doppler_auto_track {
                radar.doppler_auto_track = doppler_auto_track;
                modified = true;
            }
        }

        if modified {
            let _ = self.save();
        }
    }

    /// Give the controls a radar only gets once its model is known their
    /// saved values, the first time they are there. They do not exist yet
    /// when `update_info_from_persistence` runs as the radar is found.
    fn restore_model_controls(&mut self, radar_info: &RadarInfo) {
        let controls = &radar_info.controls;
        if !controls.contains_key(&ControlId::DopplerAutoTrack)
            || !self.model_controls_restored.insert(radar_info.key())
        {
            return;
        }
        if let Some(p) = self.config.radars.get(&radar_info.key()) {
            controls.set_doppler_auto_track(p.doppler_auto_track);
        }
    }

    /// Move settings saved under a radar's address-derived key across to
    /// the key it is known by now, once.
    ///
    /// The move matters as much as the copy: leaving the old entry behind
    /// would have every later start find it again, and would let a
    /// different radar that happens to take the same address inherit the
    /// settings of the one it replaced.
    fn take_legacy_entry(&mut self, key: &str, legacy: &str) -> bool {
        if legacy == key || self.config.radars.contains_key(key) {
            return false;
        }
        let Some(entry) = self.config.radars.remove(legacy) else {
            return false;
        };
        log::info!(
            "{}: adopting the settings saved under its previous key '{}'",
            key,
            legacy
        );
        self.config.radars.insert(key.to_string(), entry);
        if !self.path.as_os_str().is_empty() {
            let _ = self.save();
        }
        true
    }

    pub(crate) fn update_info_from_persistence(&mut self, info: &mut RadarInfo) {
        self.take_legacy_entry(&info.key(), &crate::radar::legacy_address_key(info));
        // A radar found again after it was removed gets its controls anew.
        self.model_controls_restored.remove(&info.key());

        if let Some(p) = self.config.radars.get(&info.key()) {
            if let Some(model_name) = p.model_name.as_ref() {
                info.controls.set_model_name(model_name.clone());
            }
            info.controls.set_user_name(p.user_name.clone());
            info.controls.set_spoke_processing(p.spoke_processing);
            info.controls.set_range_units(p.range_units);
            info.controls.set_auto_standby(p.auto_standby);
            if let Some(ranges) = &p.ranges
                && !ranges.is_empty()
            {
                info.set_ranges(Ranges::new_by_distance(ranges));
            }
            if let Some(zone) = &p.guard_zone_1 {
                info.controls.set_guard_zone(&ControlId::GuardZone1, zone);
            }
            if let Some(zone) = &p.guard_zone_2 {
                info.controls.set_guard_zone(&ControlId::GuardZone2, zone);
            }
            if let Some(zone) = &p.exclusion_zone_1 {
                info.controls
                    .set_exclusion_zone(&ControlId::ExclusionZone1, zone);
            }
            if let Some(zone) = &p.exclusion_zone_2 {
                info.controls
                    .set_exclusion_zone(&ControlId::ExclusionZone2, zone);
            }
            if let Some(zone) = &p.exclusion_zone_3 {
                info.controls
                    .set_exclusion_zone(&ControlId::ExclusionZone3, zone);
            }
            if let Some(zone) = &p.exclusion_zone_4 {
                info.controls
                    .set_exclusion_zone(&ControlId::ExclusionZone4, zone);
            }
            if let Some(rect) = &p.exclusion_rect_1 {
                info.controls
                    .set_exclusion_rect(&ControlId::ExclusionRect1, rect);
            }
            if let Some(rect) = &p.exclusion_rect_2 {
                info.controls
                    .set_exclusion_rect(&ControlId::ExclusionRect2, rect);
            }
            if let Some(rect) = &p.exclusion_rect_3 {
                info.controls
                    .set_exclusion_rect(&ControlId::ExclusionRect3, rect);
            }
            if let Some(rect) = &p.exclusion_rect_4 {
                info.controls
                    .set_exclusion_rect(&ControlId::ExclusionRect4, rect);
            }
            info.controls.set_arpa_max_speed(p.arpa_max_speed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geo_echo_defaults_are_viewport_14nm_and_2048() {
        let cfg = GeoEchoConfig::default();
        assert_eq!(cfg.mode, GeoEchoMode::Viewport);
        assert!((cfg.composition_range_m - 14.0 * 1852.0).abs() < 1e-9);
        assert_eq!(cfg.echo_size_px, 2048);
        assert!((cfg.target_meters_per_px - 2.0).abs() < 1e-9);
        assert_eq!(cfg.max_size_px, 2048);
        assert!((cfg.usable_bearing_start_deg - 0.0).abs() < 1e-9);
        assert!((cfg.usable_bearing_end_deg - 360.0).abs() < 1e-9);
        assert!(cfg.usable_radius_m.is_none());
        assert!((cfg.arpa_sensitivity - 1.0).abs() < 1e-9);
        assert!(
            (cfg.effective_usable_radius_m() - cfg.composition_range_m).abs() < 1e-9,
            "omit/legacy usableRadiusM follows composition"
        );
        cfg.validate().unwrap();
    }

    #[test]
    fn arpa_sensitivity_serde_round_trip_and_clamp() {
        let cfg = GeoEchoConfig {
            arpa_sensitivity: 1.75,
            ..GeoEchoConfig::default()
        };
        let json = serde_json::to_string(&cfg).unwrap();
        assert!(json.contains("arpaSensitivity"));
        let back: GeoEchoConfig = serde_json::from_str(&json).unwrap();
        assert!((back.arpa_sensitivity - 1.75).abs() < 1e-9);

        let legacy: GeoEchoConfig = serde_json::from_str(
            r#"{"mode":"viewport","composition_range_m":25928.0,"echo_size_px":2048,"target_meters_per_px":2.0,"max_size_px":2048,"usable_bearing_start_deg":0.0,"usable_bearing_end_deg":360.0}"#,
        )
        .unwrap();
        assert!((legacy.arpa_sensitivity - 1.0).abs() < 1e-9);

        let clamped = GeoEchoConfig {
            arpa_sensitivity: 9.0,
            ..GeoEchoConfig::default()
        }
        .normalized();
        assert!((clamped.arpa_sensitivity - ARPA_SENSITIVITY_MAX).abs() < 1e-9);
    }

    #[test]
    fn arpa_acquisition_guard_zone_maps_deg_to_rad_and_usable_radius() {
        let cfg = GeoEchoConfig {
            composition_range_m: 6.0 * 1852.0,
            usable_radius_m: Some(3.0 * 1852.0),
            usable_bearing_start_deg: 45.0,
            usable_bearing_end_deg: 135.0,
            ..GeoEchoConfig::default()
        };
        let zone = cfg.arpa_acquisition_guard_zone();
        assert!((zone.start_angle - 45.0_f64.to_radians()).abs() < 1e-12);
        assert!((zone.end_angle - 135.0_f64.to_radians()).abs() < 1e-12);
        assert_eq!(zone.start_distance, 0.0);
        assert!((zone.end_distance - 3.0 * 1852.0).abs() < 1e-9);
        assert!(zone.enabled);

        let full = GeoEchoConfig::default().arpa_acquisition_guard_zone();
        assert!((full.start_angle - 0.0).abs() < 1e-12);
        assert!((full.end_angle - 360.0_f64.to_radians()).abs() < 1e-12);
        assert!((full.end_distance - 14.0 * 1852.0).abs() < 1e-9);
        assert!(full.enabled);
    }

    #[test]
    fn usable_radius_normalize_clamps_above_composition() {
        let cfg = GeoEchoConfig {
            composition_range_m: 6.0 * 1852.0,
            usable_radius_m: Some(10.0 * 1852.0),
            ..GeoEchoConfig::default()
        }
        .normalized();
        assert!((cfg.usable_radius_m.unwrap() - 6.0 * 1852.0).abs() < 1e-9);
        assert!((cfg.effective_usable_radius_m() - 6.0 * 1852.0).abs() < 1e-9);

        let legacy = GeoEchoConfig {
            composition_range_m: 8.0 * 1852.0,
            usable_radius_m: None,
            ..GeoEchoConfig::default()
        }
        .normalized();
        assert!(legacy.usable_radius_m.is_none());
        assert!((legacy.effective_usable_radius_m() - 8.0 * 1852.0).abs() < 1e-9);

        let out_of_bounds = GeoEchoConfig {
            composition_range_m: 6.0 * 1852.0,
            usable_radius_m: Some(1.0), // below GEO_ECHO_MIN_RANGE_M
            ..GeoEchoConfig::default()
        }
        .normalized();
        assert!(out_of_bounds.usable_radius_m.is_none());
        assert!(
            (out_of_bounds.effective_usable_radius_m() - 6.0 * 1852.0).abs() < 1e-9
        );
    }

    #[test]
    fn geo_echo_rejects_invalid_size_and_range() {
        assert!(
            GeoEchoConfig {
                composition_range_m: 10.0,
                echo_size_px: 2048,
                ..GeoEchoConfig::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            GeoEchoConfig {
                composition_range_m: 1852.0,
                echo_size_px: 512,
                ..GeoEchoConfig::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            GeoEchoConfig {
                target_meters_per_px: 0.01,
                ..GeoEchoConfig::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            GeoEchoConfig {
                usable_bearing_start_deg: -1.0,
                ..GeoEchoConfig::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            GeoEchoConfig {
                usable_bearing_end_deg: 361.0,
                ..GeoEchoConfig::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            GeoEchoConfig {
                usable_radius_m: Some(10.0),
                ..GeoEchoConfig::default()
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn geo_echo_echo_paint_defaults_to_sidecar() {
        assert_eq!(GeoEchoConfig::default().echo_paint, EchoPaint::Sidecar);
        let legacy: GeoEchoConfig = serde_json::from_str(
            r#"{"mode":"viewport","composition_range_m":25928.0,"echo_size_px":2048,"target_meters_per_px":2.0,"max_size_px":2048,"usable_bearing_start_deg":0.0,"usable_bearing_end_deg":360.0}"#,
        )
        .unwrap();
        assert_eq!(legacy.echo_paint, EchoPaint::Sidecar);
        let gpu: GeoEchoConfig = serde_json::from_str(
            r#"{"mode":"viewport","composition_range_m":25928.0,"echo_size_px":2048,"target_meters_per_px":2.0,"max_size_px":2048,"usable_bearing_start_deg":0.0,"usable_bearing_end_deg":360.0,"echoPaint":"gpu"}"#,
        )
        .unwrap();
        assert_eq!(gpu.echo_paint, EchoPaint::Gpu);
        let snake: GeoEchoConfig = serde_json::from_str(
            r#"{"mode":"viewport","composition_range_m":25928.0,"echo_size_px":2048,"target_meters_per_px":2.0,"max_size_px":2048,"usable_bearing_start_deg":0.0,"usable_bearing_end_deg":360.0,"echo_paint":"sidecar"}"#,
        )
        .unwrap();
        assert_eq!(snake.echo_paint, EchoPaint::Sidecar);
    }

    #[test]
    fn geo_echo_config_round_trip_serde() {
        let cfg = GeoEchoConfig {
            mode: GeoEchoMode::Composition,
            composition_range_m: 6.0 * 1852.0,
            echo_size_px: 1024,
            target_meters_per_px: 1.5,
            max_size_px: 4096,
            max_viewport_range_m: Some(8.0 * 1852.0),
            usable_bearing_start_deg: 10.0,
            usable_bearing_end_deg: 350.0,
            usable_radius_m: Some(4.0 * 1852.0),
            echo_paint: EchoPaint::Gpu,
            arpa_sensitivity: default_arpa_sensitivity(),
        };
        let json = serde_json::to_string(&cfg).unwrap();
        assert!(json.contains("usable_radius_m") || json.contains("usableRadiusM"));
        assert!(json.contains("echoPaint"));
        let back: GeoEchoConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(back, cfg);
        assert!(
            (cfg.max_viewport_range_ceiling() - 8.0 * 1852.0).abs() < 1e-9
        );
        assert!(
            (GeoEchoConfig::default().max_viewport_range_ceiling()
                - default_composition_range_m())
            .abs()
                < 1e-9
        );
        // Legacy JSON without usableRadiusM → None → effective = composition.
        let legacy: GeoEchoConfig = serde_json::from_str(
            r#"{"mode":"viewport","composition_range_m":25928.0,"echo_size_px":2048,"target_meters_per_px":2.0,"max_size_px":2048,"usable_bearing_start_deg":0.0,"usable_bearing_end_deg":360.0}"#,
        )
        .unwrap();
        assert!(legacy.usable_radius_m.is_none());
        assert!((legacy.effective_usable_radius_m() - 25928.0).abs() < 1e-6);
    }

    #[test]
    fn set_geo_echo_persists_in_memory_when_not_storing() {
        let mut p = Persistence::disabled(None);
        let cfg = GeoEchoConfig {
            mode: GeoEchoMode::Composition,
            composition_range_m: 6.0 * 1852.0,
            echo_size_px: 1024,
            ..GeoEchoConfig::default()
        };
        let saved = p.set_geo_echo(cfg.clone()).unwrap();
        assert_eq!(saved, cfg);
        assert_eq!(p.geo_echo(), cfg);
    }

    fn persistence_with(key: &str, user_name: &str) -> Persistence {
        let mut radars = HashMap::new();
        radars.insert(
            key.to_string(),
            Radar {
                user_name: user_name.to_string(),
                ..Default::default()
            },
        );
        Persistence {
            config: Config {
                radars,
                ..Config::default()
            },
            ..Persistence::disabled(None)
        }
    }

    /// Tests that store radars must leave the real settings file alone.
    #[test]
    fn unit_tests_use_no_settings_file() {
        assert!(matches!(
            Persistence::new().storage(),
            SettingsStorage::NotWanted
        ));
    }

    /// Settings follow a radar to its new key exactly once, and the old
    /// entry does not linger: leaving it would have every later start find
    /// it again, and would let a different radar that later takes the same
    /// address inherit the settings of the one it replaced.
    #[test]
    fn legacy_settings_move_to_the_stable_key() {
        let mut p = persistence_with("kod0102", "Bow Radar");

        assert!(p.take_legacy_entry("kod3456", "kod0102"));
        assert_eq!(
            p.config.radars.get("kod3456").map(|r| r.user_name.as_str()),
            Some("Bow Radar")
        );
        assert!(
            !p.config.radars.contains_key("kod0102"),
            "the old entry must not linger"
        );

        assert!(
            !p.take_legacy_entry("kod3456", "kod0102"),
            "a second start has nothing left to move"
        );
    }

    /// A radar that already has settings under its stable key keeps them:
    /// whatever is under an address key is older and must not win.
    #[test]
    fn an_existing_stable_entry_is_not_overwritten() {
        let mut p = persistence_with("kod0102", "Old Name");
        p.config.radars.insert(
            "kod3456".to_string(),
            Radar {
                user_name: "Current Name".to_string(),
                ..Default::default()
            },
        );

        assert!(!p.take_legacy_entry("kod3456", "kod0102"));
        assert_eq!(
            p.config.radars.get("kod3456").map(|r| r.user_name.as_str()),
            Some("Current Name")
        );
    }

    /// The discovery page names the radars this install has had working, so
    /// someone whose radar worked last week is told something different from
    /// someone setting one up for the first time. A radar the user renamed is
    /// named the way they named it.
    #[test]
    fn known_radars_are_listed_by_the_name_the_user_would_recognise() {
        let mut p = persistence_with("nav1034A", "Bow Radar");
        p.config.radars.insert(
            "fur6424A".to_string(),
            Radar {
                user_name: String::new(),
                model_name: Some("DRS4D-NXT".to_string()),
                ..Default::default()
            },
        );

        let known = p.known_radars();

        assert_eq!(known.len(), 2);
        // Sorted, so the page does not reshuffle them between visits.
        assert_eq!(known[0].name, "Bow Radar");
        assert_eq!(known[0].brand, Some(Brand::Navico));
        // No name of its own falls back to the key, which at least identifies it.
        assert_eq!(known[1].name, "fur6424A");
        assert_eq!(known[1].brand, Some(Brand::Furuno));
        assert_eq!(known[1].model.as_deref(), Some("DRS4D-NXT"));
    }

    /// A first run has nothing to say about radars seen before.
    #[test]
    fn a_run_with_no_stored_radars_knows_of_none() {
        assert!(Persistence::disabled(None).known_radars().is_empty());
    }

    /// The question is only ever put to a user whose answer can be kept.
    #[test]
    fn a_run_that_stores_nothing_is_never_asked() {
        let mut p = Persistence::disabled(None);

        assert_eq!(p.consent(), Consent::Unavailable);
        assert_eq!(p.set_consent(true), Consent::Unavailable);
        assert_eq!(p.ensure_install_id(), None);
    }

    /// The answer and the id outlive the run that produced them, which is the
    /// whole reason the question is asked only once.
    #[test]
    fn consent_and_install_id_are_written_to_the_settings_file() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("settings.json");

        let mut p = persistence_at(path.clone());
        assert_eq!(p.consent(), Consent::Unasked);
        assert_eq!(p.set_consent(true), Consent::Granted);

        let id = p.ensure_install_id().expect("consent means an id");
        assert!(uuid::Uuid::parse_str(&id).is_ok());
        assert_eq!(p.ensure_install_id(), Some(id.clone()), "id is stable");

        let saved: Config = serde_json::from_reader(File::open(&path).unwrap()).unwrap();
        assert_eq!(saved.telemetry.consent, Some(true));
        assert_eq!(saved.telemetry.install_id, Some(id));
    }

    /// An answer that cannot be written is worse than no answer: the file
    /// still holds the previous one, so a restart would restore it -- turning
    /// a withdrawn consent back into a granted one. The run must then behave
    /// as if it could store nothing at all.
    #[test]
    fn an_answer_that_cannot_be_written_is_not_acknowledged() {
        let dir = tempfile::tempdir().unwrap();
        let mut p = persistence_at(dir.path().join("settings.json"));
        p.set_consent(true);
        assert_eq!(p.consent(), Consent::Granted);

        // Take the file away from under it: the parent is now a plain file,
        // which no user can write through.
        let blocked = dir.path().join("blocker");
        fs::write(&blocked, b"not a directory").unwrap();
        p.path = blocked.join("settings.json");

        assert_eq!(p.set_consent(false), Consent::Unavailable);
        assert_eq!(
            p.consent(),
            Consent::Unavailable,
            "a run that cannot record the answer must not keep reporting under the old one"
        );
        assert_eq!(p.ensure_install_id(), None);
    }

    /// Saying no drops the id, so a later yes counts as a new install rather
    /// than resurrecting the old one.
    #[test]
    fn declining_forgets_the_install_id() {
        let dir = tempfile::tempdir().unwrap();
        let mut p = persistence_at(dir.path().join("settings.json"));

        p.set_consent(true);
        let first = p.ensure_install_id().unwrap();
        assert_eq!(p.set_consent(false), Consent::Denied);
        assert_eq!(p.config.telemetry.install_id, None);

        p.set_consent(true);
        assert_ne!(p.ensure_install_id().unwrap(), first);
    }

    /// Brands that never had an identity key on their address as before,
    /// so there is nothing to move.
    #[test]
    fn a_radar_still_keyed_on_its_address_moves_nothing() {
        let mut p = persistence_with("gar0102", "Radar");
        assert!(!p.take_legacy_entry("gar0102", "gar0102"));
        assert!(p.config.radars.contains_key("gar0102"));
    }

    /// The usual case: the config directory does not exist yet on first run.
    #[test]
    fn a_missing_config_directory_is_created() {
        let dir = tempfile::tempdir().unwrap();
        let config_dir = dir.path().join("mayara");

        let path = settings_path(&config_dir).expect("first run must get a settings file");

        assert_eq!(path, config_dir.join(SETTINGS_FILE));
        assert!(config_dir.is_dir());
    }

    /// A radar must keep working when its settings cannot be stored: the
    /// config directory may be a read-only mount, or a host directory owned
    /// by another user. Blocked by a plain file standing where the directory
    /// should be, which no user -- root included -- can create through.
    #[test]
    fn an_unusable_config_directory_disables_persistence_instead_of_failing() {
        let dir = tempfile::tempdir().unwrap();
        let blocker = dir.path().join("blocker");
        fs::write(&blocker, b"not a directory").unwrap();

        assert!(settings_path(&blocker.join("mayara")).is_none());
    }

    /// The status endpoint and the connect-time warning both read this, and
    /// they must tell apart "cannot write" from "a replay run wants none" --
    /// only the first is worth warning a user about.
    #[test]
    fn storage_tells_a_failed_run_from_one_that_wants_nothing() {
        let path = PathBuf::from("/config/mayara/settings.json");

        assert_eq!(
            persistence_at(path.clone()).storage(),
            SettingsStorage::Stored(path.clone())
        );
        assert_eq!(
            Persistence::disabled(Some(path.clone())).storage(),
            SettingsStorage::Unwritable(path)
        );
        assert_eq!(
            Persistence::disabled(None).storage(),
            SettingsStorage::NotWanted
        );
    }

    /// A radar as discovery finds it: with only the controls every radar of
    /// its brand has, before its model is known.
    fn found_radar() -> RadarInfo {
        use crate::radar::SharedRadars;
        use crate::radar::settings::SharedControls;
        use clap::Parser;
        use std::net::{Ipv4Addr, SocketAddrV4};

        const PIXEL_VALUES: u8 = 64;
        const SPOKES: usize = 2048;
        const SPOKE_LEN: usize = 1024;
        let args = crate::Cli::parse_from(["mayara-server"]);
        let addr = SocketAddrV4::new(Ipv4Addr::new(10, 0, 0, 2), 10000);
        RadarInfo::new(
            &SharedRadars::new(),
            &args,
            Brand::Furuno,
            Some("TEST0001"),
            None,
            None,
            PIXEL_VALUES,
            SPOKES,
            SPOKE_LEN,
            addr,
            Ipv4Addr::new(10, 0, 0, 1),
            addr,
            addr,
            addr,
            |id, tx| SharedControls::new(id, tx, &args, HashMap::new()),
            true,
            true,
        )
    }

    /// Doppler Auto Track only exists once the radar has told its model,
    /// which is after the saved settings were applied to the radar found.
    /// A restart must still bring the switch back the way the user left it.
    #[test]
    fn doppler_auto_track_survives_a_restart() {
        use crate::radar::settings::new_list;

        let dir = tempfile::tempdir().unwrap();
        let mut p = persistence_at(dir.path().join("settings.json"));
        let mut info = found_radar();
        p.config.radars.insert(
            info.key(),
            Radar {
                doppler_auto_track: true,
                ..Default::default()
            },
        );

        p.update_info_from_persistence(&mut info);
        p.store(&info);
        assert!(
            p.config.radars[&info.key()].doppler_auto_track,
            "a radar without the control must not overwrite the saved choice"
        );

        info.controls
            .add(new_list(ControlId::DopplerAutoTrack, &["Off", "On"]));
        p.store(&info);
        assert!(info.controls.doppler_auto_track());

        info.controls.set_doppler_auto_track(false);
        p.store(&info);
        assert!(
            !p.config.radars[&info.key()].doppler_auto_track,
            "the user's later choice is saved, not undone"
        );
    }

    /// A radar removed and found again under the same key, as when an
    /// interface loses its address, gets a new switch that must take the
    /// saved value too.
    #[test]
    fn doppler_auto_track_survives_finding_the_radar_again() {
        use crate::radar::settings::new_list;

        let dir = tempfile::tempdir().unwrap();
        let mut p = persistence_at(dir.path().join("settings.json"));
        let key = found_radar().key();
        p.config.radars.insert(
            key.clone(),
            Radar {
                doppler_auto_track: true,
                ..Default::default()
            },
        );

        for _ in 0..2 {
            let mut info = found_radar();
            p.update_info_from_persistence(&mut info);
            info.controls
                .add(new_list(ControlId::DopplerAutoTrack, &["Off", "On"]));
            p.store(&info);
            assert!(info.controls.doppler_auto_track());
        }
        assert!(p.config.radars[&key].doppler_auto_track);
    }

    fn persistence_at(path: PathBuf) -> Persistence {
        Persistence {
            path,
            ..Persistence::disabled(None)
        }
    }

    /// First run: no settings file yet, so one is written and kept.
    #[test]
    fn a_first_run_writes_the_settings_file_it_found_missing() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(SETTINGS_FILE);

        assert!(matches!(
            persistence_at(path.clone()).load(),
            Loaded::Writable
        ));
        assert!(path.is_file());
    }

    /// A settings file that opens for reading may still belong to another
    /// user -- exactly what a container mount produces once the uid it runs
    /// as changes. Opening it for writing is the only way to find out.
    /// A directory standing in for the file is refused by every user, root
    /// included, which no chmod can claim.
    #[test]
    fn a_settings_file_that_rejects_writes_is_not_storage() {
        let dir = tempfile::tempdir().unwrap();

        let writable = dir.path().join("settings.json");
        fs::write(&writable, b"{}").unwrap();
        assert!(persistence_at(writable).is_writable());

        let refuses_writes = dir.path().join("unwritable.json");
        fs::create_dir(&refuses_writes).unwrap();
        assert!(!persistence_at(refuses_writes).is_writable());
    }

    /// Settings that cannot be updated are still worth having: the user keeps
    /// their radar names and zones for this run, and is told they will not
    /// survive it.
    #[test]
    fn settings_that_cannot_be_updated_are_still_read() {
        let path = PathBuf::from("/config/mayara/settings.json");
        let mut p = persistence_with("nav1034A", "Bow Radar");
        p.path = path.clone();

        let p = p.read_only();

        assert_eq!(
            p.config
                .radars
                .get("nav1034A")
                .map(|r| r.user_name.as_str()),
            Some("Bow Radar"),
            "what was read stays available for this run"
        );
        assert_eq!(p.storage(), SettingsStorage::Unwritable(path));
    }

    /// An existing directory says nothing about being able to write in it --
    /// it may belong to another user -- so a settings file that cannot be
    /// created has to disable persistence too, or every later save retries a
    /// write that cannot succeed.
    #[test]
    fn a_settings_file_that_cannot_be_written_disables_persistence() {
        let dir = tempfile::tempdir().unwrap();
        let blocker = dir.path().join("blocker");
        fs::write(&blocker, b"not a directory").unwrap();

        assert!(matches!(
            persistence_at(blocker.join(SETTINGS_FILE)).load(),
            Loaded::Unusable
        ));
    }

    /// A settings file written before auto standby existed enables it at its
    /// default, so an upgrade stands an unwatched radar down like a fresh
    /// install does.
    #[test]
    fn auto_standby_defaults_to_one_minute_for_old_settings() {
        let radar: Radar = serde_json::from_str(r#"{"id": 1, "user_name": "Bow"}"#).unwrap();
        assert_eq!(radar.auto_standby, DEFAULT_AUTO_STANDBY as i32);
    }

    #[test]
    fn auto_standby_round_trips() {
        let radar = Radar {
            auto_standby: 3,
            ..Default::default()
        };
        let json = serde_json::to_string(&radar).unwrap();
        let back: Radar = serde_json::from_str(&json).unwrap();
        assert_eq!(back.auto_standby, 3);
    }
}
