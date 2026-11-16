//! RGBA → PNG encode for geo echo frames.

pub const ECHO_SIZE_PX: u32 = 1024;

/// Encode a tightly-packed RGBA8 buffer as PNG bytes.
pub fn encode_rgba(width: u32, height: u32, rgba: &[u8]) -> Result<Vec<u8>, png::EncodingError> {
    let expected = (width as usize)
        .saturating_mul(height as usize)
        .saturating_mul(4);
    debug_assert_eq!(rgba.len(), expected);

    let mut out = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut out, width, height);
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        encoder.set_compression(png::Compression::Fast);
        let mut writer = encoder.write_header()?;
        writer.write_image_data(rgba)?;
    }
    Ok(out)
}

/// Fully transparent 1024×1024 PNG (cached).
pub fn transparent_png() -> &'static [u8] {
    use std::sync::OnceLock;
    static PNG: OnceLock<Vec<u8>> = OnceLock::new();
    PNG.get_or_init(|| {
        let rgba = vec![0u8; (ECHO_SIZE_PX as usize) * (ECHO_SIZE_PX as usize) * 4];
        encode_rgba(ECHO_SIZE_PX, ECHO_SIZE_PX, &rgba).expect("encode empty echo png")
    })
    .as_slice()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transparent_png_has_png_signature() {
        let bytes = transparent_png();
        assert!(bytes.len() > 8);
        assert_eq!(&bytes[0..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
    }
}
