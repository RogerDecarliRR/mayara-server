//! RGBA → PNG encode for geo echo frames.

/// Default PNG edge length when no config is available yet.
pub const DEFAULT_ECHO_SIZE_PX: u32 = 2048;

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

/// Fully transparent PNG of `size`×`size` (cached per allowed size).
pub fn transparent_png(size: u32) -> Vec<u8> {
    use std::sync::OnceLock;
    static PNG_1024: OnceLock<Vec<u8>> = OnceLock::new();
    static PNG_2048: OnceLock<Vec<u8>> = OnceLock::new();
    static PNG_4096: OnceLock<Vec<u8>> = OnceLock::new();

    let slot = match size {
        1024 => &PNG_1024,
        4096 => &PNG_4096,
        _ => &PNG_2048,
    };
    let edge = match size {
        1024 => 1024,
        4096 => 4096,
        _ => DEFAULT_ECHO_SIZE_PX,
    };
    slot.get_or_init(|| {
        let rgba = vec![0u8; (edge as usize) * (edge as usize) * 4];
        encode_rgba(edge, edge, &rgba).expect("encode empty echo png")
    })
    .clone()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transparent_png_has_png_signature() {
        let bytes = transparent_png(DEFAULT_ECHO_SIZE_PX);
        assert!(bytes.len() > 8);
        assert_eq!(&bytes[0..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
    }

    #[test]
    fn default_size_is_2048() {
        assert_eq!(DEFAULT_ECHO_SIZE_PX, 2048);
    }
}
