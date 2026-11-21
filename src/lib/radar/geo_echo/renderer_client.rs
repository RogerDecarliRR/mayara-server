//! Optional out-of-process geo-echo renderer (`GEO_ECHO_RENDERER_URL`).

use super::compositor::PolarSnapshot;
use super::polar_blob;
use crate::config::EchoPaint;
use log::warn;
use std::sync::OnceLock;

/// Base URL for the geo-echo-renderer process, or `None` when unset / empty.
///
/// Read once from `GEO_ECHO_RENDERER_URL` (trailing slash stripped).
pub fn renderer_base_url() -> Option<&'static str> {
    static URL: OnceLock<Option<String>> = OnceLock::new();
    URL.get_or_init(|| {
        std::env::var("GEO_ECHO_RENDERER_URL")
            .ok()
            .map(|s| s.trim().trim_end_matches('/').to_string())
            .filter(|s| !s.is_empty())
    })
    .as_deref()
}

/// Whether MEPO should be PUTted to the sidecar renderer.
///
/// Requires `echoPaint=sidecar` **and** `GEO_ECHO_RENDERER_URL` set.
pub fn should_push_to_renderer(echo_paint: EchoPaint) -> bool {
    echo_paint == EchoPaint::Sidecar && renderer_base_url().is_some()
}

fn http_client() -> &'static reqwest::Client {
    static CLIENT: OnceLock<reqwest::Client> = OnceLock::new();
    CLIENT.get_or_init(|| {
        reqwest::Client::builder()
            .timeout(std::time::Duration::from_secs(5))
            .build()
            .unwrap_or_else(|_| reqwest::Client::new())
    })
}

/// Fire-and-forget MEPO PUT to `{GEO_ECHO_RENDERER_URL}/v1/polar`.
///
/// No-op when the env var is unset or `echo_paint` is not [`EchoPaint::Sidecar`].
/// Does not block the compositor loop. Spawns on the current Tokio runtime;
/// skips if none.
pub fn push_polar_snapshot(snapshot: &PolarSnapshot, echo_paint: EchoPaint) {
    if !should_push_to_renderer(echo_paint) {
        return;
    }
    let Some(base) = renderer_base_url() else {
        return;
    };
    let bytes = match polar_blob::encode_snapshot(snapshot) {
        Ok(b) => b,
        Err(e) => {
            warn!("geo_echo renderer push: encode failed: {e}");
            return;
        }
    };
    let Ok(handle) = tokio::runtime::Handle::try_current() else {
        warn!("geo_echo renderer push: no Tokio runtime; skipping PUT");
        return;
    };
    let url = format!("{base}/v1/polar");
    let client = http_client().clone();
    handle.spawn(async move {
        match client
            .put(&url)
            .header(reqwest::header::CONTENT_TYPE, "application/octet-stream")
            .body(bytes)
            .send()
            .await
        {
            Ok(resp) if resp.status().is_success() || resp.status().as_u16() == 204 => {}
            Ok(resp) => {
                warn!(
                    "geo_echo renderer push: PUT {url} -> {}",
                    resp.status()
                );
            }
            Err(e) => {
                warn!("geo_echo renderer push: PUT {url} failed: {e}");
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn skip_push_when_gpu() {
        // URL may or may not be set in the test process; Gpu must never push.
        assert!(!should_push_to_renderer(EchoPaint::Gpu));
    }

    #[test]
    fn sidecar_push_depends_on_url() {
        let want = renderer_base_url().is_some();
        assert_eq!(should_push_to_renderer(EchoPaint::Sidecar), want);
    }
}
