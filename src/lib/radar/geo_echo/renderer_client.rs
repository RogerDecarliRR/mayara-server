//! Optional out-of-process geo-echo renderer (`GEO_ECHO_RENDERER_URL`).

use super::compositor::PolarSnapshot;
use super::polar_blob;
use log::debug;
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
/// No-op when the env var is unset. Does not block the compositor loop.
pub fn push_polar_snapshot(snapshot: &PolarSnapshot) {
    let Some(base) = renderer_base_url() else {
        return;
    };
    let bytes = match polar_blob::encode_snapshot(snapshot) {
        Ok(b) => b,
        Err(e) => {
            debug!("geo_echo renderer push: encode failed: {e}");
            return;
        }
    };
    let url = format!("{base}/v1/polar");
    let client = http_client().clone();
    tokio::spawn(async move {
        match client
            .put(&url)
            .header(reqwest::header::CONTENT_TYPE, "application/octet-stream")
            .body(bytes)
            .send()
            .await
        {
            Ok(resp) if resp.status().is_success() || resp.status().as_u16() == 204 => {}
            Ok(resp) => {
                debug!(
                    "geo_echo renderer push: PUT {url} -> {}",
                    resp.status()
                );
            }
            Err(e) => {
                debug!("geo_echo renderer push: PUT {url} failed: {e}");
            }
        }
    });
}
