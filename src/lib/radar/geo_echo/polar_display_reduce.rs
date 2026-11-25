//! Remap filled polar rows for viewport PPI when `echoPolarDisplay=reduce`.

/// How viewport paint interprets polar row density (`echoPolarDisplay` on POST).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PolarDisplayMode {
    #[default]
    Default,
    Reduce,
}

impl PolarDisplayMode {
    /// Parse API string (`default` | `reduce`).
    pub fn from_str(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "default" => Some(Self::Default),
            "reduce" => Some(Self::Reduce),
            _ => None,
        }
    }

    /// `None` or unknown → [`Default`].
    pub fn from_option(s: Option<&str>) -> Self {
        s.and_then(Self::from_str).unwrap_or(Self::Default)
    }
}

/// When [`PolarDisplayMode::Reduce`]: count rows with any non-zero sample (`actual`);
/// for each filled reported row index `i`, copy into compact row `(i * actual) / reported_spokes`
/// (integer floor; later `i` wins on collision) — paridade GUI `ReduceSpokeProcessor`:
/// `scaledAngle = floor(angle * actual / reported)`.
///
/// Returns compact `actual × spoke_len` and `actual` for raster / MEPO GPU (not reported 8192).
///
/// `observed_spokes_per_rev`: spokes counted in a full revolution (RevClock / GUI calibrate);
/// when set, used as `actual` instead of counting non-zero rows (sea clutter skewed bearing).
pub fn reduce_for_display(
    data: &[u8],
    reported_spokes: usize,
    spoke_len: usize,
    observed_spokes_per_rev: Option<usize>,
) -> (Vec<u8>, usize) {
    let reported = reported_spokes.max(1);
    let sl = spoke_len.max(1);
    let expected = reported.saturating_mul(sl);
    if data.len() != expected {
        return (vec![0u8; sl], 1);
    }

    let mut actual = 0usize;
    for row in 0..reported {
        let start = row * sl;
        if data[start..start + sl].iter().any(|&b| b != 0) {
            actual += 1;
        }
    }
    if let Some(observed) = observed_spokes_per_rev {
        if observed > 0 && observed <= reported {
            actual = observed;
        }
    }

    if actual == 0 {
        return (vec![0u8; sl], 1);
    }

    let mut out = vec![0u8; actual * sl];
    for i in 0..reported {
        let start = i * sl;
        let row = &data[start..start + sl];
        if row.iter().any(|&b| b != 0) {
            let dst = (i * actual) / reported;
            let dst_start = dst * sl;
            out[dst_start..dst_start + sl].copy_from_slice(row);
        }
    }
    (out, actual)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(reported: usize, sl: usize, rows: &[&[u8]]) -> Vec<u8> {
        let mut data = vec![0u8; reported * sl];
        for (row, slice) in rows.iter().enumerate() {
            let start = row * sl;
            let end = (start + slice.len()).min(data.len());
            data[start..end].copy_from_slice(&slice[..end - start]);
        }
        data
    }

    #[test]
    fn from_str_parses_default_and_reduce() {
        assert_eq!(PolarDisplayMode::from_str("default"), Some(PolarDisplayMode::Default));
        assert_eq!(PolarDisplayMode::from_str("REDUCE"), Some(PolarDisplayMode::Reduce));
        assert_eq!(PolarDisplayMode::from_str("full"), None);
        assert_eq!(PolarDisplayMode::from_option(None), PolarDisplayMode::Default);
        assert_eq!(
            PolarDisplayMode::from_option(Some("reduce")),
            PolarDisplayMode::Reduce
        );
    }

    #[test]
    fn reduce_maps_filled_rows_into_actual_slots() {
        // reported=4, actual=2 filled at rows 0 and 2 → compact 2 rows
        let data = grid(4, 2, &[&[1, 0], &[], &[2, 3], &[]]);
        let (out, spokes) = reduce_for_display(&data, 4, 2, None);
        assert_eq!(spokes, 2);
        assert_eq!(out.len(), 4);
        assert_eq!(&out[0..2], &[1, 0]);
        assert_eq!(&out[2..4], &[2, 3]);
    }

    #[test]
    fn reduce_last_row_wins_on_collision() {
        // reported=4, actual=2 filled at rows 2 and 3 → both dst = 1
        let data = grid(4, 1, &[&[], &[], &[5], &[9]]);
        let (out, spokes) = reduce_for_display(&data, 4, 1, None);
        assert_eq!(spokes, 2);
        assert_eq!(out, [0, 9]);
    }

    #[test]
    fn reduce_all_zero_returns_zeros() {
        let data = vec![0u8; 8];
        let (out, spokes) = reduce_for_display(&data, 4, 2, None);
        assert_eq!(spokes, 1);
        assert_eq!(out.len(), 2);
        assert!(out.iter().all(|&b| b == 0));
    }

    #[test]
    fn reduce_wrong_len_returns_zeros() {
        let (out, spokes) = reduce_for_display(&[1, 2, 3], 4, 2, None);
        assert_eq!(spokes, 1);
        assert_eq!(out.len(), 2);
        assert!(out.iter().all(|&b| b == 0));
    }

    #[test]
    fn reduce_sampling_row_matches_gui_scale() {
        // reported=8, actual=4; echo only at reported angle index 4 (180°)
        let data = grid(8, 1, &[&[], &[], &[], &[], &[7], &[], &[], &[]]);
        let (out, spokes) = reduce_for_display(&data, 8, 1, Some(4));
        assert_eq!(spokes, 4);
        let dst = (4 * 4) / 8;
        assert_eq!(dst, 2);
        assert_eq!(out[dst], 7);
    }
}