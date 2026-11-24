//! Gain / Sea / Rain applied to IMRS spokes before `CommonRadar::add_spoke`.
//!
//! The simulator already sends final nibbles; these controls only attenuate or
//! lift what arrives. Echoes are never created from empty bins (no noise floor).

use crate::radar::settings::{ControlId, SharedControls};

/// GUI value (0..100) at which Gain leaves the spoke untouched.
pub(crate) const NEUTRAL_GAIN: f64 = 50.0;
/// Nibble levels added at Gain = 100.
const GAIN_LIFT: f64 = 6.0;
/// Nibble levels removed at Gain = 0.
const GAIN_CUT: f64 = 10.0;
/// Nibble levels removed next to the antenna at Sea = 100 (STC).
const SEA_CUT_AT_ANTENNA: f64 = 10.0;
/// e-folding distance of the sea clutter cut.
const SEA_STC_RANGE_M: f64 = 500.0;
/// Nibble levels removed behind the leading edge of an extended echo at Rain = 100 (FTC).
const RAIN_TRAIL_CUT: f64 = 6.0;
/// Highest value of a 16-level nibble.
const NIBBLE_MAX: i32 = 15;
const CONTROL_MAX: f64 = 100.0;

/// Manual receiver settings; `None` means auto or unset, i.e. neutral.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct ReceiverLevels {
    gain: Option<f64>,
    sea: Option<f64>,
    rain: Option<f64>,
}

impl ReceiverLevels {
    pub(crate) fn from_controls(controls: &SharedControls) -> Self {
        let manual = |id: ControlId| {
            let control = controls.get(&id)?;
            if control.auto == Some(true) {
                return None;
            }
            control.value.map(|v| v.clamp(0.0, CONTROL_MAX))
        };
        Self {
            gain: manual(ControlId::Gain),
            sea: manual(ControlId::Sea),
            rain: manual(ControlId::Rain),
        }
    }

    pub(crate) fn is_neutral(&self) -> bool {
        self.gain
            .is_none_or(|g| (g - NEUTRAL_GAIN).abs() < f64::EPSILON)
            && self.sea.is_none_or(|s| s <= 0.0)
            && self.rain.is_none_or(|r| r <= 0.0)
    }

    pub(crate) fn apply(&self, samples: &mut [u8], spoke_range_m: u32) {
        let len = samples.len();
        if len == 0 {
            return;
        }
        let gain_shift = self.gain_shift();
        let rain_cut = self.rain_cut();
        let sea_amplitude = self.sea.unwrap_or(0.0) / CONTROL_MAX * SEA_CUT_AT_ANTENNA;
        let metres_per_bin = f64::from(spoke_range_m) / len as f64;

        // Reverse order: `samples[i - 1]` must still hold the original value.
        for i in (0..len).rev() {
            let original = samples[i];
            if original == 0 {
                continue;
            }
            let distance_m = (i as f64 + 0.5) * metres_per_bin;
            let sea_cut = sea_cut(sea_amplitude, distance_m);
            let ftc_cut = if i > 0 && samples[i - 1] > 0 {
                rain_cut
            } else {
                0
            };
            let out = i32::from(original) - sea_cut - ftc_cut + gain_shift;
            samples[i] = out.clamp(0, NIBBLE_MAX) as u8;
        }
    }

    fn gain_shift(&self) -> i32 {
        let Some(gain) = self.gain else {
            return 0;
        };
        let normalized = (gain - NEUTRAL_GAIN) / NEUTRAL_GAIN;
        let scale = if gain >= NEUTRAL_GAIN {
            GAIN_LIFT
        } else {
            GAIN_CUT
        };
        (normalized * scale).round() as i32
    }

    fn rain_cut(&self) -> i32 {
        (self.rain.unwrap_or(0.0) / CONTROL_MAX * RAIN_TRAIL_CUT).round() as i32
    }
}

/// Sea clutter cut in nibble levels at `distance_m` from the antenna.
fn sea_cut(amplitude: f64, distance_m: f64) -> i32 {
    if amplitude <= 0.0 {
        return 0;
    }
    (amplitude * (-distance_m / SEA_STC_RANGE_M).exp()).floor() as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPOKE_LEN: usize = 1024;
    const RANGE_M: u32 = 6144; // 6 m per bin

    fn levels(gain: Option<f64>, sea: Option<f64>, rain: Option<f64>) -> ReceiverLevels {
        ReceiverLevels { gain, sea, rain }
    }

    fn spoke_with(bin: usize, value: u8) -> Vec<u8> {
        let mut data = vec![0u8; SPOKE_LEN];
        data[bin] = value;
        data
    }

    #[test]
    fn neutral_is_byte_identical() {
        let original: Vec<u8> = (0..SPOKE_LEN).map(|i| (i % 16) as u8).collect();
        for lv in [
            ReceiverLevels::default(),
            levels(Some(NEUTRAL_GAIN), Some(0.0), Some(0.0)),
        ] {
            assert!(lv.is_neutral());
            let mut data = original.clone();
            lv.apply(&mut data, RANGE_M);
            assert_eq!(data, original);
        }
    }

    #[test]
    fn manual_levels_are_not_neutral() {
        assert!(!levels(Some(80.0), None, None).is_neutral());
        assert!(!levels(None, Some(10.0), None).is_neutral());
        assert!(!levels(None, None, Some(10.0)).is_neutral());
    }

    #[test]
    fn gain_100_lifts_weak_echo_across_strong_return() {
        let mut data = spoke_with(300, 4);
        levels(Some(100.0), None, None).apply(&mut data, RANGE_M);
        assert_eq!(data[300], 10);
    }

    #[test]
    fn gain_0_erases_weak_echoes() {
        let mut data = spoke_with(300, 8);
        data[301] = 15;
        levels(Some(0.0), None, None).apply(&mut data, RANGE_M);
        assert_eq!(data[300], 0);
        assert_eq!(data[301], 5);
    }

    #[test]
    fn gain_100_never_creates_echo_from_zero() {
        let mut data = vec![0u8; SPOKE_LEN];
        data[10] = 3;
        levels(Some(100.0), Some(100.0), Some(100.0)).apply(&mut data, RANGE_M);
        assert!(data.iter().enumerate().all(|(i, &v)| i == 10 || v == 0));
    }

    #[test]
    fn sea_100_cuts_near_antenna_but_not_far() {
        let mut near = spoke_with(8, 12); // ~51 m
        levels(None, Some(100.0), None).apply(&mut near, RANGE_M);
        assert_eq!(near[8], 3);

        let mut far = spoke_with(500, 12); // ~3003 m
        levels(None, Some(100.0), None).apply(&mut far, RANGE_M);
        assert_eq!(far[500], 12);
    }

    #[test]
    fn rain_100_keeps_leading_edge_and_thins_trail() {
        let mut data = vec![0u8; SPOKE_LEN];
        data[100..110].fill(12);
        levels(None, None, Some(100.0)).apply(&mut data, RANGE_M);
        assert_eq!(data[100], 12);
        assert!(data[101..110].iter().all(|&v| v == 6));
        assert_eq!(data[110], 0);
    }

    #[test]
    fn output_is_clamped_to_nibble_max() {
        let mut data = spoke_with(300, 14);
        levels(Some(100.0), None, None).apply(&mut data, RANGE_M);
        assert_eq!(data[300], 15);
    }
}
