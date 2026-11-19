//! Double-buffer polar slots (RM-129 W3 / A6).
//!
//! Writers fill the inactive slot, then publish by flipping `stable_idx`.
//! Readers always clone from the stable slot — never the mid-write buffer.

use super::polar_blob::DecodedPolar;

/// Two polar slots with a stable index for coherent remaps.
#[derive(Clone, Debug, Default)]
pub struct PolarSlots {
    slots: [Option<DecodedPolar>; 2],
    /// Index (0 or 1) that paint / meta read. Updated only after a full write.
    stable_idx: usize,
}

impl PolarSlots {
    pub fn new() -> Self {
        Self::default()
    }

    /// Slot index paint and meta currently read.
    pub fn stable_idx(&self) -> usize {
        self.stable_idx
    }

    /// Index of the inactive slot (write target before publish).
    pub fn write_idx(&self) -> usize {
        1 - self.stable_idx
    }

    /// Borrow the published polar, if any.
    pub fn stable(&self) -> Option<&DecodedPolar> {
        self.slots[self.stable_idx].as_ref()
    }

    /// Clone the published polar for paint outside the lock.
    pub fn clone_stable(&self) -> Option<DecodedPolar> {
        self.slots[self.stable_idx].clone()
    }

    /// Write into the inactive slot **without** flipping `stable_idx`.
    ///
    /// Callers that need a mid-update invariant (tests, staged fill) use this
    /// then [`publish_written`]. Normal ingest uses [`publish`].
    pub fn write_back(&mut self, polar: DecodedPolar) {
        let idx = self.write_idx();
        self.slots[idx] = Some(polar);
    }

    /// Flip `stable_idx` to the inactive slot after it has been filled.
    pub fn publish_written(&mut self) {
        self.stable_idx = self.write_idx();
    }

    /// Decode-path helper: fill inactive slot then publish atomically.
    pub fn publish(&mut self, polar: DecodedPolar) {
        self.write_back(polar);
        self.publish_written();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::{Color, Legend, Lookup, PixelType};

    fn stub_legend() -> Legend {
        Legend {
            doppler_approaching: None,
            doppler_receding: None,
            doppler_rain: None,
            history_start: 19,
            low_return: 1,
            medium_return: 5,
            strong_return: 10,
            pixel_colors: 1,
            pixels: vec![Lookup {
                r#type: PixelType::Normal,
                color: Color {
                    r: 0,
                    g: 0,
                    b: 0,
                    a: 0xff,
                },
            }],
            static_background: None,
        }
    }

    fn stub_polar(rev: u64) -> DecodedPolar {
        DecodedPolar {
            revolution_id: rev,
            period_ms: 1000,
            spokes: 1,
            spoke_len: 1,
            spoke_range_m: 100.0,
            center: None,
            legend: stub_legend(),
            data: vec![rev as u8],
        }
    }

    #[test]
    fn write_to_other_slot_does_not_change_stable_read_mid_update() {
        let mut slots = PolarSlots::new();
        slots.publish(stub_polar(1));
        assert_eq!(slots.stable().map(|p| p.revolution_id), Some(1));
        let stable_before = slots.stable_idx();

        // Simulate mid-write: fill back buffer, do not publish yet.
        slots.write_back(stub_polar(2));
        assert_eq!(slots.stable_idx(), stable_before);
        assert_eq!(
            slots.stable().map(|p| p.revolution_id),
            Some(1),
            "stable read must stay on previous revolution while back slot is written"
        );

        slots.publish_written();
        assert_eq!(slots.stable().map(|p| p.revolution_id), Some(2));
        assert_ne!(slots.stable_idx(), stable_before);
    }

    #[test]
    fn empty_slots_stable_is_none() {
        let slots = PolarSlots::new();
        assert!(slots.stable().is_none());
        assert!(slots.clone_stable().is_none());
    }
}
