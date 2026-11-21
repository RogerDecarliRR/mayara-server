use async_trait::async_trait;
use serde_json::Value;

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
        // core only calls us; we must persist. Polar IMRS already comes from the
        // sim; Gain/Sea/Rain are cosmetic/threshold in Mayara but must stick in GUI.
        match cv.id {
            ControlId::Range
            | ControlId::RangeUnits
            | ControlId::Gain
            | ControlId::Sea
            | ControlId::Rain
            | ControlId::AntennaHeight => {
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
    use super::value_as_f64;
    use serde_json::json;

    #[test]
    fn value_as_f64_number_and_string() {
        assert_eq!(value_as_f64(&json!(42.5)), Some(42.5));
        assert_eq!(value_as_f64(&json!("42")), Some(42.0));
        assert_eq!(value_as_f64(&json!("3.14")), Some(3.14));
        assert_eq!(value_as_f64(&json!("x")), None);
        assert_eq!(value_as_f64(&json!(true)), None);
    }
}
