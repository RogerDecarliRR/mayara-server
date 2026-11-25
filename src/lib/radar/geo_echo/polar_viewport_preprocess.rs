//! Viewport polar preprocess (RM-145): weak gain + separable smooth before raster.

/// Knobs shared by in-process [`super::compositor::publish_viewport`] and sidecar paint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewportPaintOptions {
    pub viewport_smooth: bool,
    pub weak_gain: f32,
}

impl Default for ViewportPaintOptions {
    fn default() -> Self {
        Self {
            viewport_smooth: false,
            weak_gain: 1.0,
        }
    }
}

impl ViewportPaintOptions {
    pub fn from_request(smooth: bool, weak_gain: f32) -> Self {
        let gain = if weak_gain.is_finite() && weak_gain > 0.0 {
            weak_gain
        } else {
            1.0
        };
        Self {
            viewport_smooth: smooth,
            weak_gain: gain,
        }
    }

    /// Smooth só com PPI reduzido (paridade RM-145).
    pub fn effective_smooth(&self, polar_reduce: bool) -> bool {
        self.viewport_smooth && polar_reduce
    }
}

/// Scale non-zero legend indices by `gain` (< 1 attenuates weak returns).
pub fn apply_weak_gain(data: &mut [u8], gain: f32) {
    if !(gain.is_finite() && gain > 0.0) || (gain - 1.0).abs() < 1e-6 {
        return;
    }
    for v in data.iter_mut() {
        if *v == 0 {
            continue;
        }
        let scaled = (*v as f32 * gain).round();
        *v = scaled.max(1.0).min(255.0) as u8;
    }
}

/// Separable 3-tap smooth with angular wrap and radial clamp; each axis:
/// dilate (max) then box average (max-preserve footprint).
pub fn smooth_polar_separable(data: &[u8], spokes: usize, spoke_len: usize) -> Vec<u8> {
    if spokes == 0 || spoke_len == 0 || data.len() < spokes * spoke_len {
        return data.to_vec();
    }
    let mut cur = data.to_vec();
    cur = angular_dilate_then_avg(&cur, spokes, spoke_len);
    radial_dilate_then_avg(&cur, spokes, spoke_len)
}

fn angular_dilate_then_avg(src: &[u8], spokes: usize, spoke_len: usize) -> Vec<u8> {
    let mut dilated = vec![0u8; spokes * spoke_len];
    for r in 0..spoke_len {
        for s in 0..spokes {
            let sp_m1 = (s + spokes - 1) % spokes;
            let sp_p1 = (s + 1) % spokes;
            let v0 = src[sp_m1 * spoke_len + r];
            let v1 = src[s * spoke_len + r];
            let v2 = src[sp_p1 * spoke_len + r];
            dilated[s * spoke_len + r] = v0.max(v1).max(v2);
        }
    }
    let mut out = vec![0u8; spokes * spoke_len];
    for r in 0..spoke_len {
        for s in 0..spokes {
            let sp_m1 = (s + spokes - 1) % spokes;
            let sp_p1 = (s + 1) % spokes;
            let sum = dilated[sp_m1 * spoke_len + r] as u16
                + dilated[s * spoke_len + r] as u16
                + dilated[sp_p1 * spoke_len + r] as u16;
            out[s * spoke_len + r] = ((sum + 1) / 3) as u8;
        }
    }
    out
}

fn radial_dilate_then_avg(src: &[u8], spokes: usize, spoke_len: usize) -> Vec<u8> {
    let mut dilated = vec![0u8; spokes * spoke_len];
    for s in 0..spokes {
        let base = s * spoke_len;
        for r in 0..spoke_len {
            let r_m1 = r.saturating_sub(1);
            let r_p1 = (r + 1).min(spoke_len - 1);
            let v0 = src[base + r_m1];
            let v1 = src[base + r];
            let v2 = src[base + r_p1];
            dilated[base + r] = v0.max(v1).max(v2);
        }
    }
    let mut out = vec![0u8; spokes * spoke_len];
    for s in 0..spokes {
        let base = s * spoke_len;
        for r in 0..spoke_len {
            let r_m1 = r.saturating_sub(1);
            let r_p1 = (r + 1).min(spoke_len - 1);
            let sum = dilated[base + r_m1] as u16 + dilated[base + r] as u16 + dilated[base + r_p1] as u16;
            out[base + r] = ((sum + 1) / 3) as u8;
        }
    }
    out
}

/// Owned polar ready for viewport raster (gain then optional smooth).
pub fn preprocess_viewport_polar(
    mut data: Vec<u8>,
    spokes: usize,
    spoke_len: usize,
    opts: ViewportPaintOptions,
) -> Vec<u8> {
    if data.len() != spokes.saturating_mul(spoke_len) {
        return data;
    }
    apply_weak_gain(&mut data, opts.weak_gain);
    if opts.viewport_smooth {
        return smooth_polar_separable(&data, spokes, spoke_len);
    }
    data
}

/// Accept JSON `true`/`false`, `1`/`0`, or strings `true`/`on`/`yes`/`1`.
pub fn deserialize_option_bool_lenient<'de, D>(deserializer: D) -> Result<Option<bool>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::de::Error;
    use serde::Deserialize;
    let value = Option::<serde_json::Value>::deserialize(deserializer)?;
    let Some(v) = value else {
        return Ok(None);
    };
    match v {
        serde_json::Value::Bool(b) => Ok(Some(b)),
        serde_json::Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                Ok(Some(i != 0))
            } else if let Some(f) = n.as_f64() {
                Ok(Some(f != 0.0))
            } else {
                Err(D::Error::custom("echoViewportSmooth must be boolean or number"))
            }
        }
        serde_json::Value::String(s) => {
            Ok(Some(matches!(
                s.trim().to_ascii_lowercase().as_str(),
                "true" | "1" | "on" | "yes"
            )))
        }
        _ => Err(D::Error::custom("echoViewportSmooth must be boolean, number, or string")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn weak_gain_scales_nonzero() {
        let mut data = vec![0, 10, 20, 0];
        apply_weak_gain(&mut data, 0.5);
        assert_eq!(data[0], 0);
        assert_eq!(data[1], 5);
        assert_eq!(data[2], 10);
    }

    #[test]
    fn weak_gain_unity_noop() {
        let mut data = vec![0, 7, 9];
        let before = data.clone();
        apply_weak_gain(&mut data, 1.0);
        assert_eq!(data, before);
    }

    #[test]
    fn smooth_spreads_energy_with_wrap() {
        let spokes = 4usize;
        let spoke_len = 3usize;
        let mut data = vec![0u8; spokes * spoke_len];
        data[0 * spoke_len + 1] = 100; // spoke 0, mid range
        let out = smooth_polar_separable(&data, spokes, spoke_len);
        assert!(out[0 * spoke_len + 1] >= 30);
        // Angular wrap: neighbour spoke 3 should pick up energy.
        assert!(out[3 * spoke_len + 1] > 0);
    }

    #[test]
    fn preprocess_respects_smooth_flag() {
        let spokes = 2usize;
        let spoke_len = 2usize;
        let data = vec![0, 80, 0, 0];
        let plain = preprocess_viewport_polar(
            data.clone(),
            spokes,
            spoke_len,
            ViewportPaintOptions {
                viewport_smooth: false,
                weak_gain: 1.0,
            },
        );
        assert_eq!(plain[1], 80);
        let smooth = preprocess_viewport_polar(
            data,
            spokes,
            spoke_len,
            ViewportPaintOptions {
                viewport_smooth: true,
                weak_gain: 1.0,
            },
        );
        assert!(smooth[1] >= 20);
    }

    #[test]
    fn bool_lenient_deserializes_string() {
        #[derive(serde::Deserialize)]
        #[serde(rename_all = "camelCase")]
        struct Probe {
            #[serde(default, deserialize_with = "deserialize_option_bool_lenient")]
            echo_viewport_smooth: Option<bool>,
        }
        let on: Probe =
            serde_json::from_str(r#"{"echoViewportSmooth":"on"}"#).expect("string on");
        assert_eq!(on.echo_viewport_smooth, Some(true));
        let off: Probe = serde_json::from_str(r#"{"echoViewportSmooth":false}"#).unwrap();
        assert_eq!(off.echo_viewport_smooth, Some(false));
    }
}