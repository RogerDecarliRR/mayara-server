use async_trait::async_trait;
use serde_json::Value;

use super::gain::NEUTRAL_GAIN;
use crate::brand::CommandSender;
use crate::radar::settings::{ControlId, ControlValue, SharedControls};
use crate::radar::{RadarError, RadarInfo};

/// Acknowledges UI control writes; spoke data comes from UDP IMRS.
pub(crate) struct Command {
    key: String,
}

impl Command {
    pub(crate) fn new(info: &RadarInfo) -> Self {
        Command {
            key: info.key(),
        }
    }
}

/// Parse control value as f64 from JSON Number or numeric String (`"42"`).
/// ImevtsSim has no wire scale — persist GUI units as-is (see settings.rs).
fn value_as_f64(v: &Value) -> Option<f64> {
    match v {
        Value::Number(n) => n.as_f64(),
        Value::String(s) => s.parse().ok(),
        _ => None,
    }
}

/// Value + auto for a Gain/Sea/Rain write; omitted parts keep the current state.
fn resolve_level(
    requested_auto: Option<bool>,
    requested_value: Option<f64>,
    current_auto: Option<bool>,
    current_value: Option<f64>,
    neutral: f64,
) -> (bool, f64) {
    let auto = requested_auto.or(current_auto).unwrap_or(false);
    let value = requested_value.or(current_value).unwrap_or(neutral);
    (auto, value)
}

fn neutral_level(id: &ControlId) -> f64 {
    if *id == ControlId::Gain {
        NEUTRAL_GAIN
    } else {
        0.0
    }
}

fn persist_level(cv: &ControlValue, controls: &SharedControls) -> Result<(), RadarError> {
    let current = controls.get(&cv.id);
    let (auto, value) = resolve_level(
        cv.auto,
        cv.value.as_ref().and_then(value_as_f64),
        current.as_ref().and_then(|c| c.auto),
        current.as_ref().and_then(|c| c.value),
        neutral_level(&cv.id),
    );
    controls
        .set_value_auto(&cv.id, auto, value)
        .map_err(RadarError::ControlError)?;
    Ok(())
}

#[async_trait]
impl CommandSender for Command {
    async fn set_control(
        &mut self,
        cv: &ControlValue,
        controls: &SharedControls,
    ) -> Result<(), RadarError> {
        log::debug!(
            "{}: ImevtsSim set_control {:?} = {:?}",
            self.key,
            cv.id,
            cv.value
        );
        // Range/Gain/Sea/Rain/RangeUnits/AntennaHeight are Command destination —
        // core only calls us; we must persist. Gain/Sea/Rain (value + auto) are
        // read back by the receiver and shape every spoke (see gain.rs); the rest
        // only needs to stick in the GUI.
        match cv.id {
            ControlId::Gain | ControlId::Sea | ControlId::Rain => {
                persist_level(cv, controls)?;
            }
            ControlId::Range | ControlId::RangeUnits | ControlId::AntennaHeight => {
                if let Some(v) = cv.value.as_ref().and_then(value_as_f64) {
                    controls
                        .set_value(&cv.id, v.into())
                        .map_err(RadarError::ControlError)?;
                }
            }
            _ => {}
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{resolve_level, value_as_f64};
    use serde_json::json;

    #[test]
    fn resolve_level_keeps_omitted_parts() {
        assert_eq!(
            resolve_level(None, Some(70.0), Some(true), Some(40.0), 50.0),
            (true, 70.0)
        );
        assert_eq!(
            resolve_level(Some(false), None, Some(true), Some(40.0), 50.0),
            (false, 40.0)
        );
        assert_eq!(resolve_level(None, None, None, None, 50.0), (false, 50.0));
    }

    #[test]
    fn value_as_f64_number_and_string() {
        assert_eq!(value_as_f64(&json!(42.5)), Some(42.5));
        assert_eq!(value_as_f64(&json!("42")), Some(42.0));
        assert_eq!(value_as_f64(&json!("3.14")), Some(3.14));
        assert_eq!(value_as_f64(&json!("x")), None);
        assert_eq!(value_as_f64(&json!(true)), None);
    }
}
