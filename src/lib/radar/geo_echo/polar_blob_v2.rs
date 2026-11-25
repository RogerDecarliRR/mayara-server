//! MEPO v2 — compactação sem perda: só linhas com algum byte ≠ 0 + índices de ângulo.
//!
//! Após o mesmo header de v1 (com `version = 2`) e `legend_json`:
//! - `filled_count: u32` LE
//! - `indices: filled_count × u16` LE (ângulo `0..spokes-1`)
//! - `data: filled_count × spoke_len` bytes (linha `i` = `indices[i]`)
//!
//! Decode expande para grade cheia `spokes × spoke_len` com zero nas ausentes.

use super::polar_blob::{
    write_header_fields, DecodedPolar, PolarBlobError, FLAG_HAS_CENTER,
    FLAG_HAS_OBSERVED_SPOKES, HEADER_AFTER_MAGIC, MAGIC,
};
use crate::radar::Legend;
use std::io::Write;

/// Wire format version for sparse polar blobs.
pub const VERSION: u16 = 2;

/// True when at least one spoke row is entirely zero.
pub fn has_empty_row(data: &[u8], spokes: usize, spoke_len: usize) -> bool {
    if spokes == 0 || spoke_len == 0 {
        return false;
    }
    if data.len() != spokes.saturating_mul(spoke_len) {
        return false;
    }
    for row in 0..spokes {
        let start = row * spoke_len;
        let end = start + spoke_len;
        if data[start..end].iter().all(|&b| b == 0) {
            return true;
        }
    }
    false
}

/// Indices of rows that contain at least one non-zero byte.
pub fn non_zero_row_indices(data: &[u8], spokes: usize, spoke_len: usize) -> Vec<u16> {
    let mut out = Vec::new();
    if spoke_len == 0 {
        return out;
    }
    for row in 0..spokes {
        let start = row * spoke_len;
        let end = start + spoke_len;
        if data.get(start..end).is_some_and(|r| r.iter().any(|&b| b != 0)) {
            if let Ok(idx) = u16::try_from(row) {
                out.push(idx);
            }
        }
    }
    out
}

/// Encode a dense polar grid as MEPO v2 (sparse rows).
pub fn encode(
    revolution_id: u64,
    period_ms: u64,
    spokes: u32,
    spoke_len: u32,
    spoke_range_m: f64,
    center: Option<(f64, f64)>,
    legend: &Legend,
    data: &[u8],
    observed_spokes_per_rev: usize,
    received_indices: &[u16],
) -> Result<Vec<u8>, PolarBlobError> {
    let spokes_usize = spokes as usize;
    let spoke_len_usize = spoke_len as usize;
    let expected = spokes_usize.saturating_mul(spoke_len_usize);
    if data.len() != expected {
        return Err(PolarBlobError::DataLen {
            expected,
            got: data.len(),
        });
    }
    if spokes > u32::from(u16::MAX) {
        return Err(PolarBlobError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "spokes exceed u16 index range",
        )));
    }

    let indices = if received_indices.is_empty() {
        non_zero_row_indices(data, spokes_usize, spoke_len_usize)
    } else {
        received_indices.to_vec()
    };
    let filled_count = u32::try_from(indices.len()).map_err(|_| {
        PolarBlobError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "filled_count too large",
        ))
    })?;

    let legend_json = serde_json::to_vec(legend)?;
    let mut flags = if center.is_some() {
        FLAG_HAS_CENTER
    } else {
        0
    };
    let (center_lat, center_lon) = center.unwrap_or((0.0, 0.0));
    let observed_u32 = u32::try_from(observed_spokes_per_rev).unwrap_or(0);
    if observed_u32 > 0 {
        flags |= FLAG_HAS_OBSERVED_SPOKES;
    }

    let sparse_bytes = (filled_count as usize).saturating_mul(spoke_len_usize);
    let capacity = 4
        + HEADER_AFTER_MAGIC
        + legend_json.len()
        + 4
        + 4
        + (filled_count as usize) * 2
        + sparse_bytes;
    let mut out = Vec::with_capacity(capacity);
    out.write_all(MAGIC)?;
    write_header_fields(
        &mut out,
        VERSION,
        flags,
        revolution_id,
        period_ms,
        spokes,
        spoke_len,
        spoke_range_m,
        center_lat,
        center_lon,
        &legend_json,
    )?;
    if observed_u32 > 0 {
        out.write_all(&observed_u32.to_le_bytes())?;
    }

    out.write_all(&filled_count.to_le_bytes())?;
    for &idx in &indices {
        out.write_all(&idx.to_le_bytes())?;
    }
    for &idx in &indices {
        let start = (idx as usize) * spoke_len_usize;
        out.write_all(&data[start..start + spoke_len_usize])?;
    }
    Ok(out)
}

/// Expand v2 payload (after legend) into a full `spokes × spoke_len` buffer.
pub fn expand_payload(
    payload: &[u8],
    spokes: u32,
    spoke_len: u32,
) -> Result<Vec<u8>, PolarBlobError> {
    let spokes_usize = spokes as usize;
    let spoke_len_usize = spoke_len as usize;
    if payload.len() < 4 {
        return Err(PolarBlobError::Truncated(payload.len()));
    }
    let filled_count = u32::from_le_bytes(payload[0..4].try_into().unwrap()) as usize;
    let indices_end = 4usize
        .checked_add(filled_count.checked_mul(2).ok_or(PolarBlobError::Truncated(payload.len()))?)
        .ok_or(PolarBlobError::Truncated(payload.len()))?;
    let data_end = indices_end
        .checked_add(
            filled_count
                .checked_mul(spoke_len_usize)
                .ok_or(PolarBlobError::Truncated(payload.len()))?,
        )
        .ok_or(PolarBlobError::Truncated(payload.len()))?;
    if payload.len() != data_end {
        return Err(PolarBlobError::DataLen {
            expected: data_end,
            got: payload.len(),
        });
    }

    let mut data = vec![0u8; spokes_usize.saturating_mul(spoke_len_usize)];
    for i in 0..filled_count {
        let idx_off = 4 + i * 2;
        let row = u16::from_le_bytes(payload[idx_off..idx_off + 2].try_into().unwrap()) as usize;
        if row >= spokes_usize {
            return Err(PolarBlobError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "v2 row index out of range",
            )));
        }
        let src = indices_end + i * spoke_len_usize;
        let dst = row * spoke_len_usize;
        data[dst..dst + spoke_len_usize]
            .copy_from_slice(&payload[src..src + spoke_len_usize]);
    }
    Ok(data)
}

/// Finish decode after magic+header+legend have been validated (`version == 2`).
pub fn decode_from_parts(
    revolution_id: u64,
    period_ms: u64,
    spokes: u32,
    spoke_len: u32,
    observed_spokes_per_rev: usize,
    spoke_range_m: f64,
    center: Option<(f64, f64)>,
    legend: Legend,
    payload: &[u8],
) -> Result<DecodedPolar, PolarBlobError> {
  let indices = read_v2_indices(payload)?;
    let data = expand_payload(payload, spokes, spoke_len)?;
    Ok(DecodedPolar {
        revolution_id,
        period_ms,
        spokes,
        spoke_len,
        observed_spokes_per_rev,
        received_indices: indices,
        spoke_range_m,
        center,
        legend,
        data,
    })
}

fn read_v2_indices(payload: &[u8]) -> Result<Vec<u16>, PolarBlobError> {
    if payload.len() < 4 {
        return Err(PolarBlobError::Truncated(payload.len()));
    }
    let filled_count = u32::from_le_bytes(payload[0..4].try_into().unwrap()) as usize;
    let mut out = Vec::with_capacity(filled_count);
    for i in 0..filled_count {
        let idx_off = 4 + i * 2;
        if payload.len() < idx_off + 2 {
            return Err(PolarBlobError::Truncated(payload.len()));
        }
        out.push(u16::from_le_bytes(payload[idx_off..idx_off + 2].try_into().unwrap()));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::geo_echo::compositor::PolarSnapshot;
    use crate::radar::geo_echo::polar_blob::{
        decode, encode as encode_v1, encode_snapshot, PolarBlobError, HEADER_AFTER_MAGIC,
    };
    use crate::radar::{Color, Legend, Lookup, PixelType};

    fn sample_legend() -> Legend {
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
                    r: 0x11,
                    g: 0x22,
                    b: 0x33,
                    a: 0xff,
                },
            }],
            static_background: Some(0),
        }
    }

    #[test]
    fn sparse_encodes_v2_and_decodes_equal() {
        let legend = sample_legend();
        let spokes = 8u32;
        let spoke_len = 4u32;
        let mut data = vec![0u8; (spokes * spoke_len) as usize];
        // Rows 1 and 5 non-zero; others empty → v2.
        data[4..8].copy_from_slice(&[1, 2, 3, 4]);
        data[20..24].copy_from_slice(&[9, 8, 7, 6]);

        let blob = encode(
            99,
            1200,
            spokes,
            spoke_len,
            11112.0,
            Some((-19.8, -40.1)),
            &legend,
            &data,
            0,
            &[],
        )
        .expect("encode v2");
        assert_eq!(&blob[0..4], b"MEPO");
        assert_eq!(u16::from_le_bytes(blob[4..6].try_into().unwrap()), VERSION);

        let decoded = decode(&blob).expect("decode");
        assert_eq!(decoded.revolution_id, 99);
        assert_eq!(decoded.spokes, spokes);
        assert_eq!(decoded.spoke_len, spoke_len);
        assert_eq!(decoded.data, data);
        assert_eq!(decoded.center, Some((-19.8, -40.1)));
    }

    #[test]
    fn dense_encode_snapshot_stays_v1() {
        let legend = sample_legend();
        let spokes = 4usize;
        let spoke_len = 3usize;
        let data: Vec<u8> = (1..=(spokes * spoke_len) as u8).collect();
        assert!(!has_empty_row(&data, spokes, spoke_len));

        let snap = PolarSnapshot {
            data: data.clone(),
            spokes,
            spoke_len,
            observed_spokes_per_rev: 0,
            received_indices: Vec::new(),
            center: None,
            spoke_range_m: 500.0,
            legend: legend.clone(),
            revolution_id: 3,
            period_ms: 1000,
        };
        let blob = encode_snapshot(&snap).expect("encode_snapshot");
        assert_eq!(u16::from_le_bytes(blob[4..6].try_into().unwrap()), 1);
        let decoded = decode(&blob).expect("decode");
        assert_eq!(decoded.data, data);
    }

    #[test]
    fn sparse_payload_smaller_than_v1() {
        let legend = sample_legend();
        let spokes = 64u32;
        let spoke_len = 16u32;
        let mut data = vec![0u8; (spokes * spoke_len) as usize];
        for row in [0usize, 10, 30, 63] {
            let s = row * spoke_len as usize;
            data[s..s + spoke_len as usize].fill(7);
        }

        let v2 = encode(1, 1000, spokes, spoke_len, 100.0, None, &legend, &data, 0, &[]).expect("v2");
        let v1 = encode_v1(1, 1000, spokes, spoke_len, 100.0, None, &legend, &data, 0).expect("v1");
        assert!(v2.len() < v1.len(), "v2 {} >= v1 {}", v2.len(), v1.len());
        assert_eq!(decode(&v2).unwrap().data, data);
    }

    #[test]
    fn all_zero_grid_is_v2_empty_filled() {
        let legend = sample_legend();
        let data = vec![0u8; 8];
        assert!(has_empty_row(&data, 2, 4));
        let blob = encode(1, 1, 2, 4, 1.0, None, &legend, &data, 0, &[]).expect("encode");
        assert_eq!(u16::from_le_bytes(blob[4..6].try_into().unwrap()), VERSION);
        let decoded = decode(&blob).expect("decode");
        assert_eq!(decoded.data, data);
    }

    #[test]
    fn v1_roundtrip_with_center() {
        let legend = sample_legend();
        let spokes = 4u32;
        let spoke_len = 8u32;
        let data: Vec<u8> = (0..(spokes * spoke_len) as u8).collect();
        let blob = encode_v1(
            42,
            1500,
            spokes,
            spoke_len,
            1852.0,
            Some((-19.85, -40.08)),
            &legend,
            &data,
            0,
        )
        .expect("encode");
        assert_eq!(&blob[0..4], b"MEPO");
        let decoded = decode(&blob).expect("decode");
        assert_eq!(decoded.revolution_id, 42);
        assert_eq!(decoded.center, Some((-19.85, -40.08)));
        assert_eq!(decoded.data, data);
    }

    #[test]
    fn encode_snapshot_dense_roundtrip() {
        let legend = sample_legend();
        let snapshot = PolarSnapshot {
            data: vec![1, 2, 3, 4],
            spokes: 2,
            spoke_len: 2,
            observed_spokes_per_rev: 0,
            received_indices: Vec::new(),
            center: Some((1.0, 2.0)),
            spoke_range_m: 500.0,
            legend: legend.clone(),
            revolution_id: 7,
            period_ms: 1100,
        };
        let blob = encode_snapshot(&snapshot).expect("encode_snapshot");
        assert_eq!(u16::from_le_bytes(blob[4..6].try_into().unwrap()), 1);
        let back = decode(&blob).expect("decode").into_snapshot();
        assert_eq!(back.data, snapshot.data);
        assert_eq!(back.center, snapshot.center);
    }

    #[test]
    fn rejects_bad_magic_and_truncated() {
        let mut buf = vec![0u8; 4 + HEADER_AFTER_MAGIC];
        buf[0..4].copy_from_slice(b"XXXX");
        assert!(matches!(decode(&buf).unwrap_err(), PolarBlobError::BadMagic));
        assert!(matches!(
            decode(b"ME").unwrap_err(),
            PolarBlobError::Truncated(_)
        ));
    }
}
