use async_trait::async_trait;

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
        if cv.id == ControlId::RangeUnits
            && let Some(v) = cv.value.as_ref().and_then(|v| v.as_f64())
        {
            controls
                .set_value(&ControlId::RangeUnits, v.into())
                .map_err(RadarError::ControlError)?;
        }
        Ok(())
    }
}
