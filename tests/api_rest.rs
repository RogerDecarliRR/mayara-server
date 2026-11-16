//! Integration tests for REST API endpoints
//!
//! These tests verify that the REST API responses match the documented format.
//! Run with: cargo test --test api_rest -- --ignored
//!
//! Prerequisites:
//! - mayara-server must be running with --emulator flag
//! - Default port 6502

use serde_json::Value;
use std::env;

fn base_url() -> String {
    env::var("MAYARA_TEST_URL").unwrap_or_else(|_| "http://localhost:6502".to_string())
}

async fn get_json(path: &str) -> Value {
    let url = format!("{}{}", base_url(), path);
    reqwest::Client::new()
        .get(&url)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap()
}

async fn get_response(path: &str) -> reqwest::Response {
    let url = format!("{}{}", base_url(), path);
    reqwest::Client::new().get(&url).send().await.unwrap()
}

async fn put_json(path: &str, body: &Value) -> reqwest::Response {
    let url = format!("{}{}", base_url(), path);
    reqwest::Client::new()
        .put(&url)
        .json(body)
        .send()
        .await
        .unwrap()
}

async fn first_radar_id() -> String {
    let json = get_json("/signalk/v2/api/vessels/self/radars").await;
    json["radars"]
        .as_object()
        .unwrap()
        .keys()
        .next()
        .unwrap()
        .clone()
}

// ============================================================================
// GET /
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_root_redirects_to_gui() {
    let url = format!("{}/", base_url());
    let client = reqwest::Client::builder()
        .redirect(reqwest::redirect::Policy::none())
        .build()
        .unwrap();
    let response = client.get(&url).send().await.unwrap();

    assert_eq!(response.status(), 303);
    let location = response
        .headers()
        .get("location")
        .unwrap()
        .to_str()
        .unwrap();
    assert_eq!(location, "/gui/");
}

// ============================================================================
// GET /signalk
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_signalk_endpoints() {
    let json = get_json("/signalk").await;

    assert!(json.get("endpoints").is_some(), "Missing 'endpoints'");
    assert!(json.get("server").is_some(), "Missing 'server'");

    let server = &json["server"];
    assert_eq!(server["id"], "mayara");
    assert!(server.get("version").is_some());

    let v2 = &json["endpoints"]["v2"];
    assert!(v2.get("version").is_some());
    assert!(v2.get("signalk-http").is_some());
    assert!(v2.get("signalk-ws").is_some());
}

// ============================================================================
// GET /signalk/v2/api/vessels/self/radars
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_get_radars() {
    let json = get_json("/signalk/v2/api/vessels/self/radars").await;

    assert!(json["version"].is_string(), "Missing 'version'");
    let radars = json["radars"].as_object().unwrap();
    assert!(!radars.is_empty(), "No radars found");

    for (id, radar) in radars {
        for field in ["name", "brand", "radarIpAddress"] {
            assert!(
                radar.get(field).is_some(),
                "Radar {} missing '{}'",
                id,
                field
            );
        }
    }
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_get_radars_returns_emulator() {
    let json = get_json("/signalk/v2/api/vessels/self/radars").await;
    let radars = json["radars"].as_object().unwrap();

    let (_, radar) = radars
        .iter()
        .find(|(id, _)| id.starts_with("emu"))
        .expect("No emulator radar found");
    assert_eq!(radar["brand"], "Emulator");
}

// ============================================================================
// GET /signalk/v2/api/vessels/self/radars/{radar_id}
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_get_radar_info() {
    let id = first_radar_id().await;
    let json = get_json(&format!("/signalk/v2/api/vessels/self/radars/{}", id)).await;

    for field in ["name", "brand", "radarIpAddress"] {
        assert!(
            json.get(field).is_some(),
            "Radar {} missing '{}'",
            id,
            field
        );
    }
}

/// The single-radar response is the same entry the list returns for that ID.
#[tokio::test]
#[ignore = "requires running server"]
async fn test_get_radar_info_matches_list_entry() {
    let id = first_radar_id().await;
    let list = get_json("/signalk/v2/api/vessels/self/radars").await;
    let single = get_json(&format!("/signalk/v2/api/vessels/self/radars/{}", id)).await;

    assert_eq!(list["radars"][&id], single);
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_get_radar_info_unknown_id_returns_404() {
    let response = get_response("/signalk/v2/api/vessels/self/radars/no-such-radar").await;
    assert_eq!(response.status(), 404);
}

// ============================================================================
// GET /signalk/v2/api/vessels/self/radars/{radar_id}/capabilities
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_get_capabilities() {
    let id = first_radar_id().await;
    let json = get_json(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/capabilities",
        id
    ))
    .await;

    // Response is bare capabilities object
    let caps = &json;

    for field in [
        "maxRange",
        "minRange",
        "supportedRanges",
        "spokesPerRevolution",
        "maxSpokeLength",
        "pixelValues",
        "hasDoppler",
        "hasDualRadar",
        "hasDualRange",
        "hasSparseSpokes",
        "noTransmitSectors",
        "controls",
        "legend",
    ] {
        assert!(
            caps.get(field).is_some(),
            "Missing capability field: {}",
            field
        );
    }

    assert!(caps["maxRange"].is_number());
    assert!(caps["minRange"].is_number());
    assert!(caps["supportedRanges"].is_array());
    assert!(caps["spokesPerRevolution"].is_number());
    assert!(caps["maxSpokeLength"].is_number());
    assert!(caps["pixelValues"].is_number());
    assert!(caps["hasDoppler"].is_boolean());
    assert!(caps["hasDualRadar"].is_boolean());
    assert!(caps["hasDualRange"].is_boolean());
    assert!(caps["hasSparseSpokes"].is_boolean());
    assert!(caps["noTransmitSectors"].is_number());
    assert!(caps["controls"].is_object());
    assert!(caps["legend"].is_object());
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_capabilities_controls_structure() {
    let id = first_radar_id().await;
    let json = get_json(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/capabilities",
        id
    ))
    .await;
    let controls = json["controls"].as_object().unwrap();

    assert!(controls.contains_key("power"), "Missing 'power' control");
    assert!(controls.contains_key("range"), "Missing 'range' control");

    let valid_types = [
        "number", "enum", "string", "button", "sector", "zone", "rect",
    ];
    for (cid, control) in controls {
        assert!(control.get("id").is_some(), "Control {} missing 'id'", cid);
        assert!(
            control.get("name").is_some(),
            "Control {} missing 'name'",
            cid
        );
        assert!(
            control.get("dataType").is_some(),
            "Control {} missing 'dataType'",
            cid
        );
        assert!(
            control.get("category").is_some(),
            "Control {} missing 'category'",
            cid
        );
        let dt = control["dataType"].as_str().unwrap();
        assert!(
            valid_types.contains(&dt),
            "Control {} has invalid dataType: {}",
            cid,
            dt
        );
    }
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_capabilities_legend_structure() {
    let id = first_radar_id().await;
    let json = get_json(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/capabilities",
        id
    ))
    .await;
    let legend = &json["legend"];

    for field in ["pixels", "lowReturn", "mediumReturn", "strongReturn"] {
        assert!(legend.get(field).is_some(), "Legend missing '{}'", field);
    }

    let pixels = legend["pixels"].as_array().unwrap();
    assert!(!pixels.is_empty());

    for (i, pixel) in pixels.iter().enumerate() {
        assert!(pixel.get("type").is_some(), "Pixel {} missing 'type'", i);
        assert!(pixel.get("color").is_some(), "Pixel {} missing 'color'", i);
    }
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_capabilities_units_are_si() {
    let id = first_radar_id().await;
    let json = get_json(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/capabilities",
        id
    ))
    .await;
    let controls = json["controls"].as_object().unwrap();

    let valid_si_units = ["m", "m/s", "rad", "rad/s", "s"];
    for (cid, control) in controls {
        if let Some(units) = control.get("units") {
            let u = units.as_str().unwrap();
            assert!(
                valid_si_units.contains(&u),
                "Control {} has non-SI unit: {}",
                cid,
                u
            );
        }
    }
}

// ============================================================================
// GET /signalk/v2/api/vessels/self/radars/{radar_id}/controls
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_get_all_controls() {
    let id = first_radar_id().await;
    let json = get_json(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/controls",
        id
    ))
    .await;

    // Response is bare controls object
    let controls = json.as_object().unwrap();
    assert!(!controls.is_empty());

    for (cid, control) in controls {
        assert!(
            control.is_object(),
            "Control {} value should be an object",
            cid
        );
    }
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_get_single_control_value() {
    let id = first_radar_id().await;
    let json = get_json(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/controls/power",
        id
    ))
    .await;

    // Single control returns bare value (not wrapped)
    assert!(json.get("value").is_some(), "Power should have 'value'");
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_get_single_control_invalid() {
    let id = first_radar_id().await;
    let response = get_response(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/controls/nonexistent",
        id
    ))
    .await;

    assert_eq!(response.status(), 404);
}

// ============================================================================
// PUT /signalk/v2/api/vessels/self/radars/{radar_id}/controls/{control_id}
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_set_control_value() {
    let id = first_radar_id().await;
    let path = format!("/signalk/v2/api/vessels/self/radars/{}/controls/gain", id);

    let response = put_json(&path, &serde_json::json!({"value": 75})).await;
    assert!(response.status().is_success(), "PUT should succeed");
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_set_power_standby_transmit() {
    let id = first_radar_id().await;
    let path = format!("/signalk/v2/api/vessels/self/radars/{}/controls/power", id);

    let response = put_json(&path, &serde_json::json!({"value": 1})).await;
    assert!(response.status().is_success(), "PUT standby should succeed");

    let response = put_json(&path, &serde_json::json!({"value": 2})).await;
    assert!(
        response.status().is_success(),
        "PUT transmit should succeed"
    );
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_set_invalid_control() {
    let id = first_radar_id().await;
    let response = put_json(
        &format!(
            "/signalk/v2/api/vessels/self/radars/{}/controls/nonexistent",
            id
        ),
        &serde_json::json!({"value": 50}),
    )
    .await;

    assert_eq!(response.status(), 404);
}

// ============================================================================
// GET /signalk/v2/api/vessels/self/radars/{radar_id}/targets
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_get_targets() {
    let id = first_radar_id().await;
    let json = get_json(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/targets",
        id
    ))
    .await;

    // Response is a bare array of targets
    assert!(json.is_array(), "Targets response should be an array");
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_acquire_target() {
    let id = first_radar_id().await;
    let url = format!(
        "{}/signalk/v2/api/vessels/self/radars/{}/targets",
        base_url(),
        id
    );
    let body = serde_json::json!({"bearing": 0.785, "distance": 2000});

    let response = reqwest::Client::new()
        .post(&url)
        .json(&body)
        .send()
        .await
        .unwrap();
    assert!(response.status().is_success());

    let json: Value = response.json().await.unwrap();
    let target_id = json["targetId"]
        .as_u64()
        .expect("Response should include a numeric 'targetId'");
    assert!(
        json.get("radarId").is_some(),
        "Response should include 'radarId'"
    );

    let targets = get_json(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/targets",
        id
    ))
    .await;
    assert!(
        targets
            .as_array()
            .unwrap()
            .iter()
            .any(|target| target["id"].as_u64() == Some(target_id)),
        "The acquired target should appear in the target list"
    );
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_delete_target() {
    let id = first_radar_id().await;
    let targets_url = format!(
        "{}/signalk/v2/api/vessels/self/radars/{}/targets",
        base_url(),
        id
    );

    // Delete a target this test acquired itself, so a parallel test's target
    // is never the one removed.
    let response = reqwest::Client::new()
        .post(&targets_url)
        .json(&serde_json::json!({"bearing": 1.57, "distance": 1000}))
        .send()
        .await
        .unwrap();
    assert!(response.status().is_success());
    let json: Value = response.json().await.unwrap();
    let target_id = json["targetId"]
        .as_u64()
        .expect("Response should include a numeric 'targetId'");

    let response = reqwest::Client::new()
        .delete(format!("{}/{}", targets_url, target_id))
        .send()
        .await
        .unwrap();
    assert!(response.status().is_success());
}

// ============================================================================
// GET /signalk/v2/api/vessels/self/radars/resources/openapi.json
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_openapi_spec() {
    let json = get_json("/signalk/v2/api/vessels/self/radars/resources/openapi.json").await;

    assert!(json.get("openapi").is_some());
    assert!(json.get("info").is_some());
    assert!(json.get("paths").is_some());

    let paths = json["paths"].as_object().unwrap();
    assert!(paths.contains_key("/signalk/v2/api/vessels/self/radars"));
}

// The bulk-PUT tests assert on controls the emulator actually applies (userName, range,
// targetTrails). Gain is deliberately avoided: the emulator accepts a gain PUT
// and reports success but never reflects the value, on the single-control path
// as much as this one, so asserting a gain round-trip would test the emulator
// rather than the endpoint.

#[tokio::test]
#[ignore = "requires running server"]
async fn test_set_control_values_bulk() {
    let id = first_radar_id().await;
    let path = format!("/signalk/v2/api/vessels/self/radars/{}/controls", id);

    let response = put_json(
        &path,
        &serde_json::json!({
            "userName": {"value": "BulkTest"},
            "range": {"value": 1852},
            "targetTrails": {"value": 2}
        }),
    )
    .await;
    assert!(
        response.status().is_success(),
        "bulk PUT should succeed, got {}",
        response.status()
    );

    let controls = get_json(&path).await;
    assert_eq!(controls["userName"]["value"], "BulkTest");
    assert_eq!(controls["range"]["value"], 1852);
    assert_eq!(controls["targetTrails"]["value"], 2);
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_set_control_values_rejects_unknown_control_atomically() {
    let id = first_radar_id().await;
    let path = format!("/signalk/v2/api/vessels/self/radars/{}/controls", id);

    put_json(&path, &serde_json::json!({"range": {"value": 1852}})).await;
    let before = get_json(&path).await;
    assert_eq!(before["range"]["value"], 1852, "test precondition");

    // One good control, one that does not exist. Nothing should be applied.
    let response = put_json(
        &path,
        &serde_json::json!({
            "range": {"value": 926},
            "notAControl": {"value": 1}
        }),
    )
    .await;
    assert_eq!(
        response.status(),
        404,
        "an unknown control id should be rejected with 404"
    );

    let after = get_json(&path).await;
    assert_eq!(
        after["range"]["value"], 1852,
        "the valid control must not be applied when another id is unknown"
    );
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_set_control_values_rejects_duplicate_control() {
    let id = first_radar_id().await;
    let path = format!("/signalk/v2/api/vessels/self/radars/{}/controls", id);

    // Control ids are parsed, not matched literally, so these two keys name the
    // same control and the result would otherwise depend on map ordering.
    let response = put_json(
        &path,
        &serde_json::json!({"range": {"value": 926}, "Range": {"value": 1852}}),
    )
    .await;
    assert_eq!(
        response.status(),
        400,
        "naming one control twice should be rejected"
    );
}

/// Covers the persistence branch of the bulk path — userName is one of the
/// controls `control_needs_persistence` matches, so this is the case that
/// reaches `save_persistence` — and checks the value is applied.
///
/// It does not prove the write reached disk: the read returns the live
/// in-memory value, which is set either way. Proving that needs a restart
/// between the PUT and the read, and these tests drive a server they did not
/// start (see `MAYARA_TEST_URL`), so they cannot restart it.
#[tokio::test]
#[ignore = "requires running server"]
async fn test_set_control_values_applies_a_persistent_control() {
    let id = first_radar_id().await;
    let path = format!("/signalk/v2/api/vessels/self/radars/{}/controls", id);

    let response = put_json(
        &path,
        &serde_json::json!({"userName": {"value": "BulkPersist"}}),
    )
    .await;
    assert!(
        response.status().is_success(),
        "setting a persistent control should succeed, got {}",
        response.status()
    );

    let controls = get_json(&path).await;
    assert_eq!(controls["userName"]["value"], "BulkPersist");
}

// ============================================================================
// Signal K response envelope
// ============================================================================

/// Every Signal K server answers a state-changing request in this shape, so a
/// client can read one thing whether it is talking to signalk-server or here.
fn assert_envelope(body: &Value, state: &str, status: u16) {
    assert_eq!(body["state"], state, "state in {}", body);
    assert_eq!(body["statusCode"], status, "statusCode in {}", body);
    assert!(
        body["message"].is_string(),
        "message must be a string in {}",
        body
    );
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_put_control_answers_with_completed_envelope() {
    let id = first_radar_id().await;
    let path = format!("/signalk/v2/api/vessels/self/radars/{}/controls/gain", id);

    let response = put_json(&path, &serde_json::json!({"value": 50})).await;

    assert_eq!(response.status(), 200);
    assert_envelope(&response.json::<Value>().await.unwrap(), "COMPLETED", 200);
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_unknown_radar_fails_with_envelope() {
    let response = put_json(
        "/signalk/v2/api/vessels/self/radars/nosuchradar/controls/gain",
        &serde_json::json!({"value": 50}),
    )
    .await;

    assert_eq!(response.status(), 404);
    assert_envelope(&response.json::<Value>().await.unwrap(), "FAILED", 404);
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_unknown_control_fails_with_envelope() {
    let id = first_radar_id().await;
    let response = put_json(
        &format!(
            "/signalk/v2/api/vessels/self/radars/{}/controls/nonexistent",
            id
        ),
        &serde_json::json!({"value": 50}),
    )
    .await;

    assert_eq!(response.status(), 404);
    assert_envelope(&response.json::<Value>().await.unwrap(), "FAILED", 404);
}

/// A body the server cannot read is a bad request, not an unprocessable one:
/// Signal K clients read 400 here, and the reason belongs in the envelope
/// rather than in a plain-text body they have to guess at.
#[tokio::test]
#[ignore = "requires running server"]
async fn test_malformed_body_fails_with_envelope() {
    let id = first_radar_id().await;
    let path = format!("/signalk/v2/api/vessels/self/radars/{}/controls/sea", id);

    let response = put_json(&path, &serde_json::json!({"autoValue": "abc"})).await;

    assert_eq!(response.status(), 400);
    assert_envelope(&response.json::<Value>().await.unwrap(), "FAILED", 400);
}

/// The radar API specifies a bare control value, so reading one must not pick
/// up the envelope that state-changing requests answer with.
#[tokio::test]
#[ignore = "requires running server"]
async fn test_reading_a_control_stays_bare() {
    let id = first_radar_id().await;
    let control = get_json(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/controls/gain",
        id
    ))
    .await;

    assert!(
        control.get("state").is_none() && control.get("statusCode").is_none(),
        "a control read must stay as the spec documents it, got {}",
        control
    );
}

/// A Signal K client looks for the stream under the version of the API it
/// speaks. mayara serves `/signalk/v1/stream`, so a client reading
/// `endpoints.v1` has to find it there — advertising it only under `v2` left
/// it undiscoverable to anything that follows the usual convention.
#[tokio::test]
#[ignore = "requires running server"]
async fn test_discovery_advertises_the_stream_under_v1() {
    let json = get_json("/signalk").await;
    let v1 = &json["endpoints"]["v1"];

    let ws = v1["signalk-ws"]
        .as_str()
        .expect("v1 must advertise the stream");
    assert!(
        ws.ends_with("/signalk/v1/stream"),
        "v1 stream should be the v1 stream, got {ws}"
    );
    assert!(v1.get("version").is_some());
}

/// mayara serves no v1 REST tree, so it must not name an address for one.
#[tokio::test]
#[ignore = "requires running server"]
async fn test_discovery_claims_no_v1_rest_tree() {
    let json = get_json("/signalk").await;

    assert!(
        json["endpoints"]["v1"].get("signalk-http").is_none(),
        "v1 must not advertise a REST tree mayara does not serve"
    );
}

// ============================================================================
// GET …/radars/{id}/echo.json and echo.png (geo echo overlay)
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_echo_json_contract() {
    let id = first_radar_id().await;
    let json = get_json(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/echo.json",
        id
    ))
    .await;

    assert_eq!(json["version"], 1);
    assert!(json["available"].is_boolean());
    assert!(json["generatedMs"].is_number());
    assert!(json["revolutionId"].is_number());
    assert!(json["refreshIntervalMs"].is_number());
    assert!(json["refreshIntervalMs"].as_u64().unwrap() >= 1000);
    assert!(json["northUp"].as_bool().unwrap());
    assert_eq!(json["sizePx"]["width"], 2048);
    assert_eq!(json["sizePx"]["height"], 2048);
    for corner in ["tl", "tr", "br", "bl"] {
        assert!(json["corners"][corner]["lat"].is_number());
        assert!(json["corners"][corner]["lon"].is_number());
    }
    assert!(json["center"]["lat"].is_number());
    assert!(json["center"]["lon"].is_number());
    assert!(json["rangeM"].is_number());
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_echo_png_is_png() {
    let id = first_radar_id().await;
    let response = get_response(&format!(
        "/signalk/v2/api/vessels/self/radars/{}/echo.png",
        id
    ))
    .await;

    assert_eq!(response.status(), 200);
    let ctype = response
        .headers()
        .get("content-type")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("");
    assert!(
        ctype.starts_with("image/png"),
        "expected image/png, got {ctype}"
    );
    let bytes = response.bytes().await.unwrap();
    assert!(bytes.len() > 8);
    assert_eq!(&bytes[0..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_echo_unknown_radar_returns_404() {
    let json_resp = get_response("/signalk/v2/api/vessels/self/radars/no-such-radar/echo.json").await;
    assert_eq!(json_resp.status(), 404);

    let png_resp = get_response("/signalk/v2/api/vessels/self/radars/no-such-radar/echo.png").await;
    assert_eq!(png_resp.status(), 404);
}

// ============================================================================
// GET / PUT /signalk/v2/api/geo-echo
// ============================================================================

#[tokio::test]
#[ignore = "requires running server"]
async fn test_geo_echo_get_defaults() {
    let json = get_json("/signalk/v2/api/geo-echo").await;
    assert_eq!(json["mode"], "viewport");
    assert!((json["compositionRangeM"].as_f64().unwrap() - 14.0 * 1852.0).abs() < 1.0);
    assert_eq!(json["echoSizePx"], 2048);
    assert!((json["targetMetersPerPx"].as_f64().unwrap() - 2.0).abs() < 1e-9);
    assert_eq!(json["maxSizePx"], 2048);
    assert!((json["usableBearingStartDeg"].as_f64().unwrap() - 0.0).abs() < 1e-9);
    assert!((json["usableBearingEndDeg"].as_f64().unwrap() - 360.0).abs() < 1e-9);
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_geo_echo_put_roundtrip() {
    let body = serde_json::json!({
        "mode": "composition",
        "compositionRangeM": 6.0 * 1852.0,
        "echoSizePx": 1024,
        "targetMetersPerPx": 1.5,
        "maxSizePx": 4096,
        "usableBearingStartDeg": 10.0,
        "usableBearingEndDeg": 350.0
    });
    let resp = put_json("/signalk/v2/api/geo-echo", &body).await;
    assert_eq!(resp.status(), 200);
    let saved: Value = resp.json().await.unwrap();
    assert_eq!(saved["mode"], "composition");
    assert!((saved["compositionRangeM"].as_f64().unwrap() - 6.0 * 1852.0).abs() < 1e-6);
    assert_eq!(saved["echoSizePx"], 1024);
    assert!((saved["targetMetersPerPx"].as_f64().unwrap() - 1.5).abs() < 1e-9);
    assert_eq!(saved["maxSizePx"], 4096);
    assert!((saved["usableBearingStartDeg"].as_f64().unwrap() - 10.0).abs() < 1e-9);
    assert!((saved["usableBearingEndDeg"].as_f64().unwrap() - 350.0).abs() < 1e-9);

    let got = get_json("/signalk/v2/api/geo-echo").await;
    assert_eq!(got["echoSizePx"], 1024);
    assert!((got["compositionRangeM"].as_f64().unwrap() - 6.0 * 1852.0).abs() < 1e-6);

    // Restore default for other tests.
    let restore = serde_json::json!({
        "mode": "viewport",
        "compositionRangeM": 14.0 * 1852.0,
        "echoSizePx": 2048,
        "targetMetersPerPx": 2.0,
        "maxSizePx": 2048,
        "usableBearingStartDeg": 0.0,
        "usableBearingEndDeg": 360.0
    });
    let resp = put_json("/signalk/v2/api/geo-echo", &restore).await;
    assert_eq!(resp.status(), 200);
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_geo_echo_put_rejects_bad_size() {
    let body = serde_json::json!({
        "mode": "viewport",
        "compositionRangeM": 1852.0,
        "echoSizePx": 512,
        "targetMetersPerPx": 2.0,
        "maxSizePx": 2048,
        "usableBearingStartDeg": 0.0,
        "usableBearingEndDeg": 360.0
    });
    let resp = put_json("/signalk/v2/api/geo-echo", &body).await;
    assert_eq!(resp.status(), 400);
}

#[tokio::test]
#[ignore = "requires running server"]
async fn test_echo_viewport_post_returns_png_path() {
    let id = first_radar_id().await;
    let body = serde_json::json!({
        "west": -40.08,
        "south": -19.87,
        "east": -40.04,
        "north": -19.83
    });
    let client = reqwest::Client::new();
    let url = format!(
        "{}/signalk/v2/api/vessels/self/radars/{id}/echo/viewport",
        base_url()
    );
    let resp = client.post(&url).json(&body).send().await.unwrap();
    assert_eq!(resp.status(), 200);
    let json: Value = resp.json().await.unwrap();
    assert!(json["version"].is_number());
    assert!(json["available"].is_boolean());
    assert!(json["pngPath"].as_str().unwrap().ends_with("/echo/viewport.png"));
    assert!(json["metersPerPx"].as_f64().unwrap() > 0.0);
    assert_eq!(json["bbox"]["west"], -40.08);

    let png_resp = get_response(&format!(
        "/signalk/v2/api/vessels/self/radars/{id}/echo/viewport.png"
    ))
    .await;
    assert_eq!(png_resp.status(), 200);
    let ctype = png_resp
        .headers()
        .get("content-type")
        .and_then(|v| v.to_str().ok())
        .unwrap_or("");
    assert!(ctype.starts_with("image/png"));
}
