//! Polar buffer → north-up RGBA raster (echo only).

use super::png::ECHO_SIZE_PX;
use super::super::Legend;
use std::f64::consts::TAU;

/// Paint one spoke into an RGBA buffer. Bearing 0 = north, clockwise; image
/// y decreases northward (MapLibre north-up).
pub fn paint_spoke_rgba(
    rgba: &mut [u8],
    size: u32,
    bearing_index: usize,
    spokes: usize,
    data: &[u8],
    legend: &Legend,
) {
    if spokes == 0 || data.is_empty() {
        return;
    }
    let size = size as usize;
    let cx = (size / 2) as f64;
    let half = cx;
    let angle = (bearing_index as f64) * TAU / (spokes as f64);
    let sin_a = angle.sin();
    let cos_a = angle.cos();
    let spoke_len = data.len() as f64;

    for (r, &pix) in data.iter().enumerate() {
        if pix == 0 {
            continue;
        }
        let Some(lookup) = legend.pixels.get(pix as usize) else {
            continue;
        };
        let color = &lookup.color;
        let dist = (r as f64 + 0.5) / spoke_len * half;
        let x = (cx + dist * sin_a).round() as isize;
        let y = (cx - dist * cos_a).round() as isize;
        if x < 0 || y < 0 {
            continue;
        }
        let (x, y) = (x as usize, y as usize);
        if x >= size || y >= size {
            continue;
        }
        let i = (y * size + x) * 4;
        rgba[i] = color.r;
        rgba[i + 1] = color.g;
        rgba[i + 2] = color.b;
        rgba[i + 3] = color.a;
    }
}

/// Rasterize a full polar buffer (`spokes * spoke_len` bytes, row-major by spoke).
pub fn rasterize_polar(
    polar: &[u8],
    spokes: usize,
    spoke_len: usize,
    legend: &Legend,
) -> Vec<u8> {
    let size = ECHO_SIZE_PX as usize;
    let mut rgba = vec![0u8; size * size * 4];
    if spokes == 0 || spoke_len == 0 || polar.len() < spokes * spoke_len {
        return rgba;
    }
    for spoke_i in 0..spokes {
        let start = spoke_i * spoke_len;
        let data = &polar[start..start + spoke_len];
        if data.iter().all(|&p| p == 0) {
            continue;
        }
        paint_spoke_rgba(&mut rgba, ECHO_SIZE_PX, spoke_i, spokes, data, legend);
    }
    rgba
}
