//! Binary polar snapshot blob (MEPO) shared by mayara-server and geo-echo-renderer.
//!
//! Layout (little-endian), then payload:
//! - magic: 4 bytes `b"MEPO"`
//! - version: u16 (`1` dense, `2` sparse — see [`super::polar_blob_v2`])
//! - flags: u16 (bit0 = has_center)
//! - revolution_id: u64
//! - period_ms: u64
//! - spokes: u32
//! - spoke_len: u32
//! - spoke_range_m: f64
//! - center_lat: f64, center_lon: f64 (ignored when !has_center)
//! - legend_json_len: u32
//! - legend_json: UTF-8 JSON of [`Legend`] (serde camelCase)
//! - v1 data: spokes × spoke_len bytes, row-major
//! - v2 data: filled_count + indices + sparse rows (expanded on decode)

use super::compositor::PolarSnapshot;
use super::polar_blob_v2;
use crate::radar::Legend;
use std::io::{Cursor, Read, Write};
use thiserror::Error;

/// Wire magic for Mayara Echo POlar blobs.
pub const MAGIC: &[u8; 4] = b"MEPO";
/// Dense full-grid format version.
pub const VERSION: u16 = 1;
/// `flags` bit0: center_lat/lon are meaningful.
pub const FLAG_HAS_CENTER: u16 = 1;
/// `flags` bit1: u32 `observed_spokes_per_rev` after `legend_json` (PPI reduce).
pub const FLAG_HAS_OBSERVED_SPOKES: u16 = 2;

/// Fixed-size header before variable legend JSON + spoke data (without magic).
/// version + flags + 2×u64 + 2×u32 + 3×f64 + legend_json_len = 2+2+8+8+4+4+8+8+8+4 = 56
pub(crate) const HEADER_AFTER_MAGIC: usize = 56;

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
/// `data` is always the full dense grid (`spokes × spoke_len`), including for v2.
#[derive(Clone, Debug, PartialEq)]
pub struct DecodedPolar {
    pub revolution_id: u64,
    pub period_ms: u64,
    pub spokes: u32,
    pub spoke_len: u32,
    pub observed_spokes_per_rev: usize,
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
            observed_spokes_per_rev: self.observed_spokes_per_rev,
            center: self.center,
            spoke_range_m: self.spoke_range_m,
            legend: self.legend,
            revolution_id: self.revolution_id,
            period_ms: self.period_ms,
        }
    }
}

/// Write version…legend_json (after magic already written).
pub(crate) fn write_header_fields(
    out: &mut Vec<u8>,
    version: u16,
    flags: u16,
    revolution_id: u64,
    period_ms: u64,
    spokes: u32,
    spoke_len: u32,
    spoke_range_m: f64,
    center_lat: f64,
    center_lon: f64,
    legend_json: &[u8],
) -> Result<(), PolarBlobError> {
    let legend_json_len = u32::try_from(legend_json.len()).map_err(|_| {
        PolarBlobError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "legend JSON too large",
        ))
    })?;
    out.write_all(&version.to_le_bytes())?;
    out.write_all(&flags.to_le_bytes())?;
    out.write_all(&revolution_id.to_le_bytes())?;
    out.write_all(&period_ms.to_le_bytes())?;
    out.write_all(&spokes.to_le_bytes())?;
    out.write_all(&spoke_len.to_le_bytes())?;
    out.write_all(&spoke_range_m.to_le_bytes())?;
    out.write_all(&center_lat.to_le_bytes())?;
    out.write_all(&center_lon.to_le_bytes())?;
    out.write_all(&legend_json_len.to_le_bytes())?;
    out.write_all(legend_json)?;
    Ok(())
}

/// Encode polar fields + legend into a MEPO **v1** (dense) blob.
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
) -> Result<Vec<u8>, PolarBlobError> {
    let expected = (spokes as usize).saturating_mul(spoke_len as usize);
    if data.len() != expected {
        return Err(PolarBlobError::DataLen {
            expected,
            got: data.len(),
        });
    }

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

    let capacity = 4 + HEADER_AFTER_MAGIC + legend_json.len() + data.len() + 4;
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
    out.write_all(data)?;
    Ok(out)
}

/// Whether MEPO v2 compact encode is allowed.
///
/// Env `MAYARA_MEPO_COMPACT`: `0` → always dense v1 (full `spokes×spoke_len`,
/// ~8 MiB at Furuno 8192). Any other / unset → compact when there are empty rows.
pub fn mepo_compact_enabled() -> bool {
    match std::env::var("MAYARA_MEPO_COMPACT") {
        Ok(v) => v.trim() != "0",
        Err(_) => true,
    }
}

/// Encode a [`PolarSnapshot`]: v2 when compact on and any spoke row is all-zero, else v1.
pub fn encode_snapshot(snapshot: &PolarSnapshot) -> Result<Vec<u8>, PolarBlobError> {
    let use_v2 = mepo_compact_enabled()
        && polar_blob_v2::has_empty_row(&snapshot.data, snapshot.spokes, snapshot.spoke_len);
    if use_v2 {
        polar_blob_v2::encode(
            snapshot.revolution_id,
            snapshot.period_ms,
            snapshot.spokes as u32,
            snapshot.spoke_len as u32,
            snapshot.spoke_range_m,
            snapshot.center,
            &snapshot.legend,
            &snapshot.data,
            snapshot.observed_spokes_per_rev,
        )
    } else {
        encode(
            snapshot.revolution_id,
            snapshot.period_ms,
            snapshot.spokes as u32,
            snapshot.spoke_len as u32,
            snapshot.spoke_range_m,
            snapshot.center,
            &snapshot.legend,
            &snapshot.data,
            snapshot.observed_spokes_per_rev,
        )
    }
}

/// Decode a MEPO blob (v1 dense or v2 sparse → always full grid in [`DecodedPolar::data`]).
pub fn decode(bytes: &[u8]) -> Result<DecodedPolar, PolarBlobError> {
    if bytes.len() < 4 + HEADER_AFTER_MAGIC {
        return Err(PolarBlobError::Truncated(bytes.len()));
    }
    if &bytes[0..4] != MAGIC.as_slice() {
        return Err(PolarBlobError::BadMagic);
    }

    let mut cur = Cursor::new(&bytes[4..]);
    let version = read_u16(&mut cur)?;
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
    let center = if flags & FLAG_HAS_CENTER != 0 {
        Some((center_lat, center_lon))
    } else {
        None
    };
    let mut payload_off = legend_end;
    let mut observed_spokes_per_rev = 0usize;
    if flags & FLAG_HAS_OBSERVED_SPOKES != 0 {
        if bytes.len() < payload_off + 4 {
            return Err(PolarBlobError::Truncated(bytes.len()));
        }
        observed_spokes_per_rev =
            u32::from_le_bytes(bytes[payload_off..payload_off + 4].try_into().unwrap()) as usize;
        payload_off += 4;
    }
    let payload = &bytes[payload_off..];

    match version {
        VERSION => {
            let expected = (spokes as usize).saturating_mul(spoke_len as usize);
            if payload.len() != expected {
                return Err(PolarBlobError::DataLen {
                    expected,
                    got: payload.len(),
                });
            }
            Ok(DecodedPolar {
                revolution_id,
                period_ms,
                spokes,
                spoke_len,
                observed_spokes_per_rev,
                spoke_range_m,
                center,
                legend,
                data: payload.to_vec(),
            })
        }
        polar_blob_v2::VERSION => polar_blob_v2::decode_from_parts(
            revolution_id,
            period_ms,
            spokes,
            spoke_len,
            observed_spokes_per_rev,
            spoke_range_m,
            center,
            legend,
            payload,
        ),
        other => Err(PolarBlobError::BadVersion(other)),
    }
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
