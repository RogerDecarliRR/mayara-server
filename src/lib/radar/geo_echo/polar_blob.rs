//! Binary polar snapshot blob (MEPO) shared by mayara-server and geo-echo-renderer.
//!
//! Layout (little-endian), then raw spoke data:
//! - magic: 4 bytes `b"MEPO"`
//! - version: u16 = 1
//! - flags: u16 (bit0 = has_center)
//! - revolution_id: u64
//! - period_ms: u64
//! - spokes: u32
//! - spoke_len: u32
//! - spoke_range_m: f64
//! - center_lat: f64, center_lon: f64 (ignored when !has_center)
//! - legend_json_len: u32
//! - legend_json: UTF-8 JSON of [`Legend`] (serde camelCase)
//! - data: spokes × spoke_len bytes, row-major

use super::compositor::PolarSnapshot;
use crate::radar::Legend;
use std::io::{Cursor, Read, Write};
use thiserror::Error;

/// Wire magic for Mayara Echo POlar blobs.
pub const MAGIC: &[u8; 4] = b"MEPO";
/// Current blob format version.
pub const VERSION: u16 = 1;
/// `flags` bit0: center_lat/lon are meaningful.
pub const FLAG_HAS_CENTER: u16 = 1;

/// Fixed-size header before variable legend JSON + spoke data (without magic).
/// version + flags + 2×u64 + 2×u32 + 3×f64 + legend_json_len = 2+2+8+8+4+4+8+8+8+4 = 56
const HEADER_AFTER_MAGIC: usize = 56;

#[derive(Debug, Error)]
pub enum PolarBlobError {
    #[error("blob too short ({0} bytes)")]
    Truncated(usize),
    #[error("bad magic (expected MEPO)")]
    BadMagic,
    #[error("unsupported version {0}")]
    BadVersion(u16),
    #[error("spoke data length mismatch: expected {expected}, got {got}")]
    DataLen { expected: usize, got: usize },
    #[error("legend JSON: {0}")]
    LegendJson(#[from] serde_json::Error),
    #[error("I/O: {0}")]
    Io(#[from] std::io::Error),
}

/// Decoded polar blob aligned with [`PolarSnapshot`] fields.
#[derive(Clone, Debug, PartialEq)]
pub struct DecodedPolar {
    pub revolution_id: u64,
    pub period_ms: u64,
    pub spokes: u32,
    pub spoke_len: u32,
    pub spoke_range_m: f64,
    pub center: Option<(f64, f64)>,
    pub legend: Legend,
    pub data: Vec<u8>,
}

impl DecodedPolar {
    /// Convert into a [`PolarSnapshot`] for raster / compositor use.
    pub fn into_snapshot(self) -> PolarSnapshot {
        PolarSnapshot {
            data: self.data,
            spokes: self.spokes as usize,
            spoke_len: self.spoke_len as usize,
            center: self.center,
            spoke_range_m: self.spoke_range_m,
            legend: self.legend,
            revolution_id: self.revolution_id,
            period_ms: self.period_ms,
        }
    }
}

/// Encode polar fields + legend into a MEPO blob.
pub fn encode(
    revolution_id: u64,
    period_ms: u64,
    spokes: u32,
    spoke_len: u32,
    spoke_range_m: f64,
    center: Option<(f64, f64)>,
    legend: &Legend,
    data: &[u8],
) -> Result<Vec<u8>, PolarBlobError> {
    let expected = (spokes as usize).saturating_mul(spoke_len as usize);
    if data.len() != expected {
        return Err(PolarBlobError::DataLen {
            expected,
            got: data.len(),
        });
    }

    let legend_json = serde_json::to_vec(legend)?;
    let legend_json_len = u32::try_from(legend_json.len()).map_err(|_| {
        PolarBlobError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "legend JSON too large",
        ))
    })?;

    let flags = if center.is_some() {
        FLAG_HAS_CENTER
    } else {
        0
    };
    let (center_lat, center_lon) = center.unwrap_or((0.0, 0.0));

    let capacity = 4 + HEADER_AFTER_MAGIC + legend_json.len() + data.len();
    let mut out = Vec::with_capacity(capacity);
    out.write_all(MAGIC)?;
    out.write_all(&VERSION.to_le_bytes())?;
    out.write_all(&flags.to_le_bytes())?;
    out.write_all(&revolution_id.to_le_bytes())?;
    out.write_all(&period_ms.to_le_bytes())?;
    out.write_all(&spokes.to_le_bytes())?;
    out.write_all(&spoke_len.to_le_bytes())?;
    out.write_all(&spoke_range_m.to_le_bytes())?;
    out.write_all(&center_lat.to_le_bytes())?;
    out.write_all(&center_lon.to_le_bytes())?;
    out.write_all(&legend_json_len.to_le_bytes())?;
    out.write_all(&legend_json)?;
    out.write_all(data)?;
    Ok(out)
}

/// Encode a [`PolarSnapshot`] into a MEPO blob.
pub fn encode_snapshot(snapshot: &PolarSnapshot) -> Result<Vec<u8>, PolarBlobError> {
    encode(
        snapshot.revolution_id,
        snapshot.period_ms,
        snapshot.spokes as u32,
        snapshot.spoke_len as u32,
        snapshot.spoke_range_m,
        snapshot.center,
        &snapshot.legend,
        &snapshot.data,
    )
}

/// Decode a MEPO blob.
pub fn decode(bytes: &[u8]) -> Result<DecodedPolar, PolarBlobError> {
    if bytes.len() < 4 + HEADER_AFTER_MAGIC {
        return Err(PolarBlobError::Truncated(bytes.len()));
    }
    if &bytes[0..4] != MAGIC.as_slice() {
        return Err(PolarBlobError::BadMagic);
    }

    let mut cur = Cursor::new(&bytes[4..]);
    let version = read_u16(&mut cur)?;
    if version != VERSION {
        return Err(PolarBlobError::BadVersion(version));
    }
    let flags = read_u16(&mut cur)?;
    let revolution_id = read_u64(&mut cur)?;
    let period_ms = read_u64(&mut cur)?;
    let spokes = read_u32(&mut cur)?;
    let spoke_len = read_u32(&mut cur)?;
    let spoke_range_m = read_f64(&mut cur)?;
    let center_lat = read_f64(&mut cur)?;
    let center_lon = read_f64(&mut cur)?;
    let legend_json_len = read_u32(&mut cur)? as usize;

    let header_end = 4 + HEADER_AFTER_MAGIC;
    let legend_end = header_end
        .checked_add(legend_json_len)
        .ok_or(PolarBlobError::Truncated(bytes.len()))?;
    if bytes.len() < legend_end {
        return Err(PolarBlobError::Truncated(bytes.len()));
    }
    let legend: Legend = serde_json::from_slice(&bytes[header_end..legend_end])?;

    let data = bytes[legend_end..].to_vec();
    let expected = (spokes as usize).saturating_mul(spoke_len as usize);
    if data.len() != expected {
        return Err(PolarBlobError::DataLen {
            expected,
            got: data.len(),
        });
    }

    let center = if flags & FLAG_HAS_CENTER != 0 {
        Some((center_lat, center_lon))
    } else {
        None
    };

    Ok(DecodedPolar {
        revolution_id,
        period_ms,
        spokes,
        spoke_len,
        spoke_range_m,
        center,
        legend,
        data,
    })
}

fn read_u16(r: &mut Cursor<&[u8]>) -> Result<u16, PolarBlobError> {
    let mut buf = [0u8; 2];
    r.read_exact(&mut buf)?;
    Ok(u16::from_le_bytes(buf))
}

fn read_u32(r: &mut Cursor<&[u8]>) -> Result<u32, PolarBlobError> {
    let mut buf = [0u8; 4];
    r.read_exact(&mut buf)?;
    Ok(u32::from_le_bytes(buf))
}

fn read_u64(r: &mut Cursor<&[u8]>) -> Result<u64, PolarBlobError> {
    let mut buf = [0u8; 8];
    r.read_exact(&mut buf)?;
    Ok(u64::from_le_bytes(buf))
}

fn read_f64(r: &mut Cursor<&[u8]>) -> Result<f64, PolarBlobError> {
    let mut buf = [0u8; 8];
    r.read_exact(&mut buf)?;
    Ok(f64::from_le_bytes(buf))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::{Color, Legend, Lookup, PixelType};

    fn sample_legend() -> Legend {
        Legend {
            doppler_approaching: Some((17, 1)),
            doppler_receding: None,
            doppler_rain: None,
            history_start: 19,
            low_return: 1,
            medium_return: 5,
            strong_return: 10,
            pixel_colors: 2,
            pixels: vec![Lookup {
                r#type: PixelType::Normal,
                color: Color {
                    r: 0x33,
                    g: 0x44,
                    b: 0x55,
                    a: 0xff,
                },
            }],
            static_background: Some(0),
        }
    }

    #[test]
    fn roundtrip_with_center() {
        let legend = sample_legend();
        let spokes = 4u32;
        let spoke_len = 8u32;
        let data: Vec<u8> = (0..(spokes * spoke_len) as u8).collect();
        let blob = encode(
            42,
            1500,
            spokes,
            spoke_len,
            1852.0,
            Some((-19.85, -40.08)),
            &legend,
            &data,
        )
        .expect("encode");
        assert_eq!(&blob[0..4], b"MEPO");
        let decoded = decode(&blob).expect("decode");
        assert_eq!(decoded.revolution_id, 42);
        assert_eq!(decoded.period_ms, 1500);
        assert_eq!(decoded.spokes, spokes);
        assert_eq!(decoded.spoke_len, spoke_len);
        assert_eq!(decoded.spoke_range_m, 1852.0);
        assert_eq!(decoded.center, Some((-19.85, -40.08)));
        assert_eq!(decoded.legend, legend);
        assert_eq!(decoded.data, data);
    }

    #[test]
    fn roundtrip_without_center() {
        let legend = sample_legend();
        let data = vec![9u8; 16];
        let blob = encode(1, 2000, 2, 8, 100.0, None, &legend, &data).expect("encode");
        let decoded = decode(&blob).expect("decode");
        assert_eq!(decoded.center, None);
        assert_eq!(decoded.data, data);
    }

    #[test]
    fn encode_snapshot_roundtrip() {
        let legend = sample_legend();
        let snapshot = PolarSnapshot {
            data: vec![1, 2, 3, 4],
            spokes: 2,
            spoke_len: 2,
            center: Some((1.0, 2.0)),
            spoke_range_m: 500.0,
            legend: legend.clone(),
            revolution_id: 7,
            period_ms: 1100,
        };
        let blob = encode_snapshot(&snapshot).expect("encode_snapshot");
        let decoded = decode(&blob).expect("decode");
        let back = decoded.into_snapshot();
        assert_eq!(back.revolution_id, snapshot.revolution_id);
        assert_eq!(back.period_ms, snapshot.period_ms);
        assert_eq!(back.spokes, snapshot.spokes);
        assert_eq!(back.spoke_len, snapshot.spoke_len);
        assert_eq!(back.spoke_range_m, snapshot.spoke_range_m);
        assert_eq!(back.center, snapshot.center);
        assert_eq!(back.legend, legend);
        assert_eq!(back.data, snapshot.data);
    }

    #[test]
    fn rejects_bad_magic() {
        let mut buf = vec![0u8; 4 + HEADER_AFTER_MAGIC];
        buf[0..4].copy_from_slice(b"XXXX");
        let err = decode(&buf).unwrap_err();
        assert!(matches!(err, PolarBlobError::BadMagic));
    }

    #[test]
    fn rejects_truncated() {
        let err = decode(b"ME").unwrap_err();
        assert!(matches!(err, PolarBlobError::Truncated(_)));
    }
}