//! Polar echo buffer and revolution timing for the geo compositor.

use super::corners::refresh_interval_ms;
use std::time::{Duration, Instant};

pub(super) struct PolarBuffer {
    pub spokes: usize,
    pub spoke_len: usize,
    pub data: Vec<u8>,
    pub last_center: Option<(f64, f64)>,
    pub last_range_m: f64,
}

impl PolarBuffer {
    pub fn new(spokes: usize, spoke_len: usize) -> Self {
        let spoke_len = spoke_len.max(1);
        let spokes = spokes.max(1);
        Self {
            spokes,
            spoke_len,
            data: vec![0u8; spokes * spoke_len],
            last_center: None,
            last_range_m: 0.0,
        }
    }

    pub fn resize_if_needed(&mut self, spokes: usize, spoke_len: usize) {
        let spokes = spokes.max(1);
        let spoke_len = spoke_len.max(1);
        if spokes != self.spokes || spoke_len != self.spoke_len {
            self.spokes = spokes;
            self.spoke_len = spoke_len;
            self.data = vec![0u8; spokes * spoke_len];
        }
    }

    pub fn write_spoke(&mut self, index: usize, samples: &[u8]) {
        if samples.len() > self.spoke_len {
            self.resize_if_needed(self.spokes, samples.len());
        }
        if index >= self.spokes {
            return;
        }
        let len = samples.len().min(self.spoke_len);
        let start = index * self.spoke_len;
        let row = &mut self.data[start..start + self.spoke_len];
        row.fill(0);
        row[..len].copy_from_slice(&samples[..len]);
    }
}

pub(super) struct RevClock {
    prev_index: Option<usize>,
    spokes_this_rev: u32,
    last_rev_end: Option<Instant>,
    last_period: Duration,
    last_publish: Option<Instant>,
    pub revolution_id: u64,
    /// Snapshot waiting for the min-interval gate.
    pending: bool,
}

impl RevClock {
    pub fn new() -> Self {
        Self {
            prev_index: None,
            spokes_this_rev: 0,
            last_rev_end: None,
            last_period: Duration::from_secs(2),
            last_publish: None,
            revolution_id: 0,
            pending: false,
        }
    }

    pub fn period_ms(&self) -> u64 {
        self.last_period.as_millis() as u64
    }

    pub fn interval(&self) -> Duration {
        Duration::from_millis(refresh_interval_ms(self.period_ms()))
    }

    pub fn close_revolution(&mut self) {
        let now = Instant::now();
        if let Some(prev_end) = self.last_rev_end {
            let p = now.saturating_duration_since(prev_end);
            if p > Duration::from_millis(100) && p < Duration::from_secs(30) {
                self.last_period = p;
            }
        }
        self.last_rev_end = Some(now);
        self.spokes_this_rev = 0;
        self.revolution_id = self.revolution_id.saturating_add(1);
        self.pending = true;
    }

    /// Angle wrap: previous revolution ends *before* the new spoke is written.
    pub fn is_wrap(&self, index: usize) -> bool {
        self.prev_index.is_some_and(|prev| index < prev)
    }

    /// After writing a spoke: advance counters; return true if the count closed a rev.
    pub fn after_spoke_written(&mut self, index: usize, spokes: usize) -> bool {
        self.prev_index = Some(index);
        self.spokes_this_rev = self.spokes_this_rev.saturating_add(1);
        if spokes > 0 && self.spokes_this_rev >= spokes as u32 {
            self.close_revolution();
            true
        } else {
            false
        }
    }

    pub fn should_publish_now(&self) -> bool {
        if !self.pending {
            return false;
        }
        match self.last_publish {
            None => true,
            Some(lp) => Instant::now().saturating_duration_since(lp) >= self.interval(),
        }
    }

    pub fn mark_published(&mut self) {
        self.pending = false;
        self.last_publish = Some(Instant::now());
    }

    pub fn wait_remaining(&self) -> Option<Duration> {
        if !self.pending {
            return None;
        }
        let Some(lp) = self.last_publish else {
            return Some(Duration::ZERO);
        };
        let elapsed = Instant::now().saturating_duration_since(lp);
        let interval = self.interval();
        if elapsed >= interval {
            Some(Duration::ZERO)
        } else {
            Some(interval - elapsed)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rev_clock_interval_respects_fast_spin() {
        let mut clock = RevClock::new();
        clock.last_period = Duration::from_millis(400);
        assert_eq!(clock.interval(), Duration::from_secs(1));
        clock.last_period = Duration::from_millis(2500);
        assert_eq!(clock.interval(), Duration::from_millis(2500));
    }
}
