/*
 * ESPectre - Shared Protocol Unit Tests
 *
 * Exercises JSON payload formatting and command parsing helpers used by the
 * runtime protocol surfaces.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include "direct_http_protocol.h"
#include "espectre_protocol.h"
#include "frontend/ota_protocol.h"
#include "frontend/ota_version.h"
#include "protocol_json.h"
#include "runtime_diagnostics.h"

#include <cmath>
#include <string>

using namespace espectre;

void test_frontend_protocol_extensions_share_capabilities_routing_and_validation(void) {
  static size_t validation_calls = 0U;
  validation_calls = 0U;
  const auto read_validator = +[](const std::vector<JsonObjectField> &fields, EspectreCommand *, std::string *) {
    return fields.empty();
  };
  const auto action_validator = +[](const std::vector<JsonObjectField> &fields, EspectreCommand *, std::string *) {
    ++validation_calls;
    bool mode_present = false;
    for (const auto &field : fields) {
      if (field.name == "command_id" || field.name == "command") continue;
      if (field.name != "mode" || field.type != JsonValueType::STRING || field.value != "fast") return false;
      mode_present = true;
    }
    return mode_present;
  };
  const EspectreProtocolExtension extension{
      {{"GET", "/espectre/v1/vendor-status", "vendor_status", "vendor_status",
        EspectreApiRouteKind::RESOURCE, false, false, true, read_validator},
       {"POST", "/espectre/v1/vendor-checks", "vendor_check", "vendor_check",
        EspectreApiRouteKind::OPERATION, true, true, false, action_validator}},
      {"vendor_done"}};
  TEST_ASSERT_TRUE(validate_protocol_extension(extension));
  EspectreCapabilityProfile profile;
  profile.extension = &extension;
  const std::string catalog = espectre_capabilities_payload({}, {}, profile);
  TEST_ASSERT_TRUE(catalog.find("\"vendor_status\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"vendor_check\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"vendor_done\"") != std::string::npos);
  std::string error;
  DirectRequest request;
  EspectreCommand direct;
  EspectreCommand mqtt;
  TEST_ASSERT_FALSE(parse_direct_http_request("POST", "/espectre/v1/vendor-checks", "{}", &request, &error));
  TEST_ASSERT_TRUE(parse_direct_http_request("POST", "/espectre/v1/vendor-checks", "{\"mode\":\"fast\"}",
                                           &request, &error, &extension));
  TEST_ASSERT_TRUE(request.asynchronous);
  TEST_ASSERT_TRUE(direct_http_request_to_command(request, &direct, &error, &extension));
  TEST_ASSERT_EQUAL(1U, validation_calls);
  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"vendor-1\",\"command\":\"vendor_check\",\"mode\":\"fast\"}", &mqtt, &error, &extension));
  TEST_ASSERT_EQUAL(2U, validation_calls);
  TEST_ASSERT_FALSE(parse_espectre_command("{", &mqtt, &error, &extension));
  TEST_ASSERT_EQUAL(2U, validation_calls);
  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"vendor-1\",\"command\":\"vendor_check\",\"mode\":\"fast\"}", &mqtt, &error, &extension));
  TEST_ASSERT_EQUAL_STRING(direct.command.c_str(), mqtt.command.c_str());
  TEST_ASSERT_EQUAL_STRING("fast", extract_json_string(direct.extension_parameters, "mode").c_str());
  TEST_ASSERT_EQUAL_STRING("fast", extract_json_string(mqtt.extension_parameters, "mode").c_str());
  request.params = "{\"mode\":true}";
  TEST_ASSERT_FALSE(direct_http_request_to_command(request, &direct, &error, &extension));
  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"vendor-1\",\"command\":\"vendor_check\",\"mode\":true}", &mqtt, &error, &extension));
  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"vendor-1\",\"command\":\"vendor_check\",\"mode\":\"fast\"}", &mqtt, &error));
  TEST_ASSERT_TRUE(frontend_command_allowed_during_raw_collection("vendor_status", &extension));
  TEST_ASSERT_FALSE(frontend_command_allowed_during_raw_collection("vendor_check", &extension));

  auto conflicting = extension;
  conflicting.routes[0].path = "/espectre/v1/device";
  TEST_ASSERT_FALSE(validate_protocol_extension(conflicting));
  TEST_ASSERT_NULL(find_extension_route(&conflicting, "vendor_check"));
  profile.extension = &conflicting;
  TEST_ASSERT_TRUE(espectre_capabilities_payload({}, {}, profile).find("vendor_check") == std::string::npos);
  conflicting = extension;
  conflicting.routes.push_back(conflicting.routes[0]);
  TEST_ASSERT_FALSE(validate_protocol_extension(conflicting));
  conflicting = extension;
  conflicting.events.push_back("motion");
  TEST_ASSERT_FALSE(validate_protocol_extension(conflicting));
  conflicting = extension;
  conflicting.routes[1].validate = nullptr;
  TEST_ASSERT_FALSE(validate_protocol_extension(conflicting));
}

void test_sdk_validators_reject_out_of_range_parameters_before_dispatch(void) {
  size_t route_count = 0U;
  const auto *routes = espectre_api_routes(&route_count);
  for (size_t i = 0U; i < route_count; ++i) {
    if (routes[i].kind != EspectreApiRouteKind::STREAM) TEST_ASSERT_TRUE(routes[i].validate != nullptr);
  }
  for (const char *parameters : {
           "{\"threshold\":-0.01}", "{\"threshold\":1.01}",
           "{\"motion_on_hits\":0,\"motion_off_hits\":1}",
           "{\"motion_on_hits\":1,\"motion_off_hits\":21}",
           "{\"detector\":\"high_accuracy\",\"threshold\":2}"}) {
    EspectreCommand command;
    std::string direct_error;
    std::string mqtt_error;
    TEST_ASSERT_FALSE(parse_espectre_command_request("bounds", "update_sensing", parameters,
                                                    &command, &direct_error));
    TEST_ASSERT_FALSE(parse_espectre_command(
        espectre_command_request_payload("bounds", "update_sensing", parameters), &command, &mqtt_error));
    TEST_ASSERT_EQUAL_STRING(direct_error.c_str(), mqtt_error.c_str());
  }
}

void test_ota_version_ordering_blocks_downgrades_and_divergent_builds(void) {
  TEST_ASSERT_TRUE(compare_ota_versions("2.8.0-280-gac7af68", "2.8.0-279-gc63eaed") ==
                   OtaVersionComparison::NEWER);
  TEST_ASSERT_TRUE(compare_ota_versions("2.8.0-279-gc63eaed", "2.8.0-280-gac7af68") ==
                   OtaVersionComparison::OLDER);
  TEST_ASSERT_TRUE(compare_ota_versions("2.8.0-280-gac7af68", "2.8.0-280-gac7af68-dirty") ==
                   OtaVersionComparison::SAME);
  TEST_ASSERT_TRUE(compare_ota_versions("2.8.0-280-gfffffff", "2.8.0-280-gac7af68") ==
                   OtaVersionComparison::UNORDERED);
  TEST_ASSERT_TRUE(compare_ota_versions("2.8.0-1-g0000001", "2.8.0") ==
                   OtaVersionComparison::NEWER);
  TEST_ASSERT_TRUE(compare_ota_versions("3.0.0", "3.0.0-rc.2") == OtaVersionComparison::NEWER);
  TEST_ASSERT_TRUE(compare_ota_versions("3.0.0-rc.2", "3.0.0-rc.1-5-gabcdef0") ==
                   OtaVersionComparison::NEWER);
  TEST_ASSERT_TRUE(compare_ota_versions("3.0.0", "2.8.0-999-gabcdef0") ==
                   OtaVersionComparison::NEWER);
  TEST_ASSERT_TRUE(compare_ota_versions("3.0.0", "unknown") == OtaVersionComparison::NEWER);
  TEST_ASSERT_TRUE(compare_ota_versions("snapshot", "3.0.0") == OtaVersionComparison::UNORDERED);
}

void test_device_id_helpers_format_and_parse_canonical_hex_consistently(void) {
  TEST_ASSERT_EQUAL_STRING("00007c2c6742bbac", format_espectre_device_id(0x00007C2C6742BBACULL).c_str());

  uint64_t parsed = 0U;
  TEST_ASSERT_TRUE(parse_espectre_device_id("00007c2c6742bbac", &parsed));
  TEST_ASSERT_EQUAL(0x00007C2C6742BBACULL, parsed);
  // Accept the legacy prefix while clients migrate to the canonical form.
  TEST_ASSERT_TRUE(parse_espectre_device_id("0x00007c2c6742bbac", &parsed));
  TEST_ASSERT_EQUAL(0x00007C2C6742BBACULL, parsed);
  TEST_ASSERT_TRUE(parse_espectre_device_id("124", &parsed));
  TEST_ASSERT_EQUAL(0x124ULL, parsed);
  TEST_ASSERT_FALSE(parse_espectre_device_id("bad-id", &parsed));

  const uint8_t mac[6] = {0x7C, 0x2C, 0x67, 0x42, 0xBB, 0xAC};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  TEST_ASSERT_EQUAL(0x00007C2C6742BBACULL, espectre_device_id_from_mac(mac, sizeof(mac)));
  TEST_ASSERT_EQUAL(ESPECTRE_DEFAULT_DEVICE_ID, espectre_device_id_from_mac(nullptr, 0));
#pragma GCC diagnostic pop
  TEST_ASSERT_EQUAL_STRING("ESPectre C6 42bbac", espectre_device_name(0x00007C2C6742BBACULL, "esp32c6").c_str());
  TEST_ASSERT_EQUAL_STRING("ESPectre S2 42bbac", espectre_device_name(0x00007C2C6742BBACULL, "esp32s2").c_str());
  TEST_ASSERT_EQUAL_STRING("ESPectre UNK 000000", espectre_device_name(ESPECTRE_DEFAULT_DEVICE_ID).c_str());
}

void test_effective_device_helpers_and_topic_generation_use_defaults(void) {
  EspectreDeviceConfig config;
  config.device_id = ESPECTRE_DEFAULT_DEVICE_ID;
  config.device_label.clear();
  config.topic_prefix = "custom/root/";

  TEST_ASSERT_EQUAL_STRING("0000000000000000", espectre_effective_device_id(config).c_str());
  TEST_ASSERT_EQUAL_STRING(ESPECTRE_DEFAULT_DEVICE_LABEL, espectre_effective_device_label(config).c_str());
  TEST_ASSERT_EQUAL_STRING("custom/root/0000000000000000/telemetry", espectre_topic(config, "telemetry").c_str());
  TEST_ASSERT_EQUAL_STRING("custom/root/0000000000000000/", espectre_topic(config, nullptr).c_str());
}

void test_clear_mqtt_config_resets_runtime_defaults(void) {
  EspectreDeviceConfig config;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "broker.local";
  config.mqtt_port = 2883;
  config.mqtt_username = "user";
  config.mqtt_password = "secret";
  config.topic_prefix = "custom/root";

  clear_espectre_mqtt_config(&config);

  TEST_ASSERT_TRUE(config.mqtt_scheme.empty());
  TEST_ASSERT_TRUE(config.mqtt_host.empty());
  TEST_ASSERT_EQUAL(0U, config.mqtt_port);
  TEST_ASSERT_TRUE(config.mqtt_username.empty());
  TEST_ASSERT_TRUE(config.mqtt_password.empty());
  TEST_ASSERT_EQUAL_STRING(ESPECTRE_TOPIC_PREFIX, config.topic_prefix.c_str());

  clear_espectre_mqtt_config(nullptr);
}

void test_parse_mqtt_batch_config_command_updates_all_fields(void) {
  EspectreDeviceConfig config;
  std::string error;

  TEST_ASSERT_TRUE(parse_espectre_mqtt_config_command(
      "SET_MQTT_CONFIG:scheme=mqtt&host=broker.local&port=2883&username=user%20name&password=s3cr%25t&topic_prefix=lab%2Froot",
      &config,
      &error));
  TEST_ASSERT_EQUAL_STRING("mqtt", config.mqtt_scheme.c_str());
  TEST_ASSERT_EQUAL_STRING("broker.local", config.mqtt_host.c_str());
  TEST_ASSERT_EQUAL(2883, config.mqtt_port);
  TEST_ASSERT_EQUAL_STRING("user name", config.mqtt_username.c_str());
  TEST_ASSERT_EQUAL_STRING("s3cr%t", config.mqtt_password.c_str());
  TEST_ASSERT_EQUAL_STRING("lab/root", config.topic_prefix.c_str());

  TEST_ASSERT_FALSE(parse_espectre_mqtt_config_command("SET_MQTT_CONFIG:host=broker.local", &config, &error));
  TEST_ASSERT_EQUAL_STRING("missing MQTT scheme", error.c_str());
  TEST_ASSERT_FALSE(parse_espectre_mqtt_config_command(
      "SET_MQTT_CONFIG:scheme=mqtt&host=broker.local&port=0", &config, &error));
  TEST_ASSERT_EQUAL_STRING("mqtt port must be 1..65535", error.c_str());
}

void test_mqtt_config_validation_rejects_uri_framing_and_preserves_the_previous_config(void) {
  EspectreDeviceConfig config;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "previous.local";
  config.mqtt_port = 1883U;
  std::string error;

  TEST_ASSERT_FALSE(parse_espectre_mqtt_config_command(
      "SET_MQTT_CONFIG:scheme=mqtts&host=mqtts%3A%2F%2Fbroker.example.com&port=8883",
      &config,
      &error));
  TEST_ASSERT_EQUAL_STRING("invalid MQTT host", error.c_str());
  TEST_ASSERT_EQUAL_STRING("mqtt", config.mqtt_scheme.c_str());
  TEST_ASSERT_EQUAL_STRING("previous.local", config.mqtt_host.c_str());
  TEST_ASSERT_EQUAL(1883U, config.mqtt_port);

  const char *invalid_hosts[] = {
      "user@broker.local", "broker.local/path", "broker.local?query", "broker.local#fragment",
      "broker.local:1883", "999.1.1.1", "bad_host.local", "[2001:db8::1]", "192.0.2.1::"};
  for (const char *host : invalid_hosts) {
    EspectreDeviceConfig invalid;
    invalid.mqtt_scheme = "mqtt";
    invalid.mqtt_host = host;
    invalid.mqtt_port = 1883U;
    TEST_ASSERT_FALSE(validate_espectre_mqtt_config(invalid, &error));
  }

  const char *valid_hosts[] = {"homeassistant.local", "192.168.1.10", "2001:db8::1", "::1"};
  for (const char *host : valid_hosts) {
    EspectreDeviceConfig valid;
    valid.mqtt_scheme = "mqtts";
    valid.mqtt_host = host;
    valid.mqtt_port = 8883U;
    TEST_ASSERT_TRUE(validate_espectre_mqtt_config(valid, &error));
    TEST_ASSERT_TRUE(espectre_mqtt_configured(valid));
  }

  config.mqtt_scheme = "ws";
  TEST_ASSERT_FALSE(validate_espectre_mqtt_config(config, &error));
  TEST_ASSERT_EQUAL_STRING("invalid MQTT scheme (accepted: mqtt and mqtts)", error.c_str());
}

void test_status_telemetry_and_diagnostics_payloads_include_expected_fields(void) {
  EspectreDeviceConfig config;
  config.device_id = 0x0000000000000007ULL;

  RuntimeSnapshot snapshot;
  snapshot.motion_state = MotionState::MOTION;
  snapshot.movement_metric = 2.75f;
  snapshot.threshold = 1.5f;
  snapshot.detector_name = "high_accuracy";

  const std::string status = espectre_health_payload(config, true, 1234);
  const std::string telemetry = espectre_motion_payload(config, snapshot, 222, 33, "native");
  const std::string diagnostics = espectre_diagnostics_payload(config, snapshot, 333, 44, 128.5f, 6.25f);

  TEST_ASSERT_TRUE(status.find("\"status\":\"ok\"") != std::string::npos);
  TEST_ASSERT_TRUE(status.find("\"online\":true") != std::string::npos);
  TEST_ASSERT_EQUAL_STRING("{\"timestamp_ms\":222,\"state\":\"motion\",\"score\":2.75}",
                           telemetry.c_str());
  snapshot.movement_metric = NAN;
  snapshot.threshold = NAN;
  const std::string telemetry_nan = espectre_motion_payload(config, snapshot, 222, 33, "native");
  TEST_ASSERT_TRUE(telemetry_nan.find("nan") == std::string::npos);
  TEST_ASSERT_TRUE(telemetry_nan.find("\"score\":0") != std::string::npos);
  TEST_ASSERT_EQUAL_STRING(
      "{\"timestamp_ms\":333,\"uptime\":44,\"free_memory_kb\":128.5,"
      "\"loop_time_ms\":6.25}",
      diagnostics.c_str());
  TEST_ASSERT_TRUE(diagnostics.find("\"uptime\":44") != std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"free_memory_kb\":128.5") != std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"loop_time_ms\":6.25") != std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"traffic_tx_pps\"") == std::string::npos);
}

void test_diagnostics_payload_includes_enabled_runtime_sample(void) {
  EspectreDeviceConfig config;
  RuntimeSnapshot snapshot;
  RuntimeDiagnosticsSample diagnostics;
  diagnostics.traffic_tx_pps = 100.0f;
  diagnostics.csi_callback_pps = 96.0f;
  diagnostics.csi_accepted_pps = 90.0f;
  diagnostics.csi_admitted_pps = 84.0f;
  diagnostics.csi_filtered_pps = 6.0f;
  diagnostics.csi_missing_slots_pps = 16.0f;
  diagnostics.csi_excess_pps = 7.0f;
  diagnostics.csi_stale_pps = 1.0f;
  diagnostics.csi_out_of_order_pps = 2.0f;
  diagnostics.csi_occupancy_ratio = 0.84f;
  diagnostics.wifi_channel = 10U;
  diagnostics.wifi_rssi_dbm = -55;

  const std::string payload =
      espectre_diagnostics_payload(config, snapshot, 333, 44, 128.5f, 6.25f, &diagnostics);

  TEST_ASSERT_TRUE(payload.find("\"traffic_tx_pps\":100") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_callback_pps\":96") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_accepted_pps\":90") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_admitted_pps\":84") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_filtered_pps\":6") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_missing_slots_pps\":16") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_excess_pps\":7") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_stale_pps\":1") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_out_of_order_pps\":2") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_occupancy\":0.84") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"wifi_channel\":10") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"wifi_rssi_dbm\":-55") != std::string::npos);

  diagnostics.wifi_rssi_dbm = INT8_MIN;
  const std::string payload_without_rssi =
      espectre_diagnostics_payload(config, snapshot, 333, 44, 128.5f, 6.25f, &diagnostics);
  TEST_ASSERT_TRUE(payload_without_rssi.find("\"wifi_rssi_dbm\":null") != std::string::npos);
}

void test_info_payload_uses_defaults_and_optional_sections(void) {
  EspectreDeviceConfig config;
  config.device_id = 0x0000000000000001ULL;
  config.device_label = "Kitchen \"node\"\nA";

  EspectreDeviceInfo info;
  info.frontend = "matter";
  info.firmware_version = "2026.7";
  info.chip = "esp32c6";
  info.detector = "lightweight";
  info.csi_profile = "ht20";
  info.supports_diagnostics = true;
  info.supports_device_config = true;
  info.supports_runtime_threshold = true;
  info.supports_runtime_motion_hits = true;
  info.supports_runtime_detector = true;
  info.supports_manual_recalibration = true;
  info.supports_traffic_control = true;
  info.csi_traffic_mode = "internal";
  info.traffic_mode = "ping";
  info.csi_target_pps = 100U;
  info.evaluation_interval_ms = 250U;
  info.network.ip_address = "192.168.1.10";
  info.network.mac_address = "AA:BB:CC:DD:EE:FF";
  info.network.channel = 6;

  const std::string payload = espectre_device_payload(config, info);

  TEST_ASSERT_TRUE(payload.find("\"device_id\":\"0000000000000001\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"name\":\"Kitchen \\\"node\\\"\\nA\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"label\":\"Kitchen \\\"node\\\"\\nA\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"frontend\":\"matter\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"firmware\":\"2026.7\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"chip\":\"esp32c6\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_profile\":\"ht20\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"supports_") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"network\":{") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"ip_address\"") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"mac_address\"") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"channel\"") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"detection\"") == std::string::npos);

  const std::string catalog =
      espectre_capabilities_payload(config, info, true, true, true, true, true, true, true, &frontend_ota_protocol());
  TEST_ASSERT_TRUE(catalog.find("\"device_id\"") == std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"resources\":[") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"traffic_udp_port\":5555") != std::string::npos);
  const std::string marker_property =
      std::string("\"marker\":\"") + RUNTIME_CSI_TRAFFIC_MARKER_UTF8 + "\"";
  TEST_ASSERT_TRUE(catalog.find(marker_property) != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"diagnostics\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"update_sensing\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"kind\"") == std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"access\"") == std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"params\"") == std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"result\"") == std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"wifi\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"scan_wifi\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"set_wifi_bssid\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"clear_wifi_credentials\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("set_wifi_config") == std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("band_policy") == std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"devices\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"name\":\"commands\"") == std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"name\":\"stats\"") == std::string::npos);
  EspectreCommand command;
  command.command_id = "capabilities";
  command.command = "capabilities";
  const std::string capability_response =
      espectre_command_result_payload(config, command, true, "ok", "capabilities returned", catalog);
  TEST_ASSERT_TRUE(catalog.size() <= ESPECTRE_DIRECT_MAX_REQUEST_SIZE);
  TEST_ASSERT_TRUE(capability_response.size() <= ESPECTRE_DIRECT_MAX_RESPONSE_SIZE);
}

void test_info_payload_omits_optional_sections_when_empty(void) {
  EspectreDeviceConfig config;
  config.device_id = ESPECTRE_DEFAULT_DEVICE_ID;
  config.device_label.clear();

  EspectreDeviceInfo info;
  info.frontend.clear();
  info.firmware_version.clear();
  info.chip.clear();
  info.detector.clear();

  const std::string payload = espectre_device_payload(config, info);

  TEST_ASSERT_TRUE(payload.find("\"device_id\":\"0000000000000000\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"name\":\"ESPectre UNK 000000\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"label\":\"\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"frontend\":\"native\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"firmware\":\"unknown\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"chip\":\"unknown\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"network\":{") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"detection\":{") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_traffic_mode\"") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"traffic_mode\"") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"csi_target_pps\"") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"evaluation_interval_ms\"") == std::string::npos);

  const std::string catalog = espectre_capabilities_payload(config, info);
  TEST_ASSERT_TRUE(catalog.find("\"protocol_version\":\"1.0\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"capabilities\"") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"diagnostics\"") == std::string::npos);
}

void test_info_normalization_reports_runtime_channel_and_csi_profile(void) {
  RuntimeSnapshot snapshot;
  snapshot.link_channel = 36U;
  snapshot.csi_capture_profile = CsiCaptureProfile::VHT20;

  const EspectreDeviceInfo info = normalize_protocol_device_info(
      EspectreDeviceInfo{}, &snapshot, "native", "esp32c5");

  TEST_ASSERT_EQUAL(36U, info.network.channel);
  TEST_ASSERT_EQUAL_STRING("vht20", info.csi_profile.c_str());
}

void test_command_result_payload_includes_acceptance_and_message(void) {
  EspectreDeviceConfig config;
  config.device_id = 0x0000000000000005ULL;

  EspectreCommand command;
  command.command_id = "abc123";
  command.command = "update_sensing";

  const std::string accepted =
      espectre_command_result_payload(config, command, true, "ok", "applied", "{\"threshold\":0.5}");
  const std::string rejected =
      espectre_command_result_payload(config, command, false, "invalid_params", "");

  TEST_ASSERT_TRUE(accepted.find("\"command_id\":\"abc123\"") != std::string::npos);
  TEST_ASSERT_TRUE(accepted.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_TRUE(accepted.find("\"code\":\"ok\"") != std::string::npos);
  TEST_ASSERT_TRUE(accepted.find("\"message\":\"applied\"") != std::string::npos);
  TEST_ASSERT_TRUE(accepted.find("\"data\":{\"threshold\":0.5}") != std::string::npos);
  TEST_ASSERT_TRUE(rejected.find("\"accepted\":false") != std::string::npos);
  TEST_ASSERT_TRUE(rejected.find("\"code\":\"invalid_params\"") != std::string::npos);
  TEST_ASSERT_TRUE(rejected.find("\"message\":\"\"") != std::string::npos);
  TEST_ASSERT_TRUE(rejected.find("\"data\"") == std::string::npos);
}

void test_parse_espectre_command_parses_info_and_threshold_commands(void) {
  EspectreCommand command;
  std::string error;

  TEST_ASSERT_TRUE(parse_espectre_command("{\"command_id\":\"x1\",\"command\":\"device\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("x1", command.command_id.c_str());
  TEST_ASSERT_EQUAL_STRING("device", command.command.c_str());
  TEST_ASSERT_FALSE(command.has_threshold);

  TEST_ASSERT_TRUE(
      parse_espectre_command("{\"command_id\":\"x-capabilities\",\"command\":\"capabilities\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("capabilities", command.command.c_str());

  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x-sensing\",\"command\":\"update_sensing\",\"enabled\":false}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_sensing_enabled);
  TEST_ASSERT_FALSE(command.sensing_enabled);

  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x-label\",\"command\":\"update_device\",\"label\":\"Kitchen\"}",
      &command,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_device_label);
  TEST_ASSERT_EQUAL_STRING("Kitchen", command.device_label.c_str());

  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x-label-clear\",\"command\":\"update_device\",\"label\":\"\"}",
      &command,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_device_label);
  TEST_ASSERT_TRUE(command.device_label.empty());
  const std::string ascii_label_at_limit(ESPECTRE_DEVICE_LABEL_MAX_LENGTH, 'a');
  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x-label-ascii\",\"command\":\"update_device\",\"label\":\"" +
          ascii_label_at_limit + "\"}",
      &command,
      &error, &frontend_ota_protocol()));
  const std::string ascii_label_over_limit = ascii_label_at_limit + "a";
  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"x-label-ascii-long\",\"command\":\"update_device\",\"label\":\"" +
          ascii_label_over_limit + "\"}",
      &command,
      &error, &frontend_ota_protocol()));
  const std::string utf8_label_at_limit = std::string(u8"éééééééééééééééé");
  TEST_ASSERT_EQUAL(ESPECTRE_DEVICE_LABEL_MAX_LENGTH, utf8_label_at_limit.size());
  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x-label-utf8\",\"command\":\"update_device\",\"label\":\"" +
          utf8_label_at_limit + "\"}",
      &command,
      &error, &frontend_ota_protocol()));
  const std::string utf8_label_over_limit = utf8_label_at_limit + u8"é";
  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"x-label-utf8-long\",\"command\":\"update_device\",\"label\":\"" +
          utf8_label_over_limit + "\"}",
      &command,
      &error, &frontend_ota_protocol()));

  TEST_ASSERT_TRUE(
      parse_espectre_command("{\"command_id\":\"x2\",\"command\":\"update_sensing\",\"threshold\":0.25}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("update_sensing", command.command.c_str());
  TEST_ASSERT_TRUE(command.has_threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.25f, command.threshold);

  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x-motion\",\"command\":\"update_sensing\",\"motion_on_hits\":6,\"motion_off_hits\":4}",
      &command,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_motion_hits);
  TEST_ASSERT_EQUAL_UINT8(6U, command.motion_on_hits);
  TEST_ASSERT_EQUAL_UINT8(4U, command.motion_off_hits);

  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x-detector\",\"command\":\"update_sensing\",\"detector\":\"high_accuracy\"}",
      &command,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_detector);
  TEST_ASSERT_EQUAL_STRING("high_accuracy", command.detector.c_str());

  TEST_ASSERT_TRUE(parse_espectre_command("{\"command_id\":\"x3\",\"command\":\"check_ota\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("check_ota", command.command.c_str());
  TEST_ASSERT_FALSE(has_json_key(command.extension_parameters, "channel"));

  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x3b\",\"command\":\"check_ota\",\"channel\":\"preview\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(has_json_key(command.extension_parameters, "channel"));
  TEST_ASSERT_EQUAL_STRING("preview", extract_json_string(command.extension_parameters, "channel").c_str());

  TEST_ASSERT_TRUE(parse_espectre_command("{\"command_id\":\"x4\",\"command\":\"start_ota\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("start_ota", command.command.c_str());
  TEST_ASSERT_FALSE(has_json_key(command.extension_parameters, "channel"));

  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x4b\",\"command\":\"start_ota\",\"channel\":\"develop\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(has_json_key(command.extension_parameters, "channel"));
  TEST_ASSERT_EQUAL_STRING("develop", extract_json_string(command.extension_parameters, "channel").c_str());

  TEST_ASSERT_TRUE(parse_espectre_command("{\"command_id\":\"x5\",\"command\":\"recalibrate\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("recalibrate", command.command.c_str());

  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x6\",\"command\":\"update_sensing\",\"csi_traffic_mode\":\"external\"}",
      &command,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_csi_traffic_mode);
  TEST_ASSERT_EQUAL_STRING("external", command.csi_traffic_mode.c_str());

  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x7\",\"command\":\"update_sensing\",\"traffic_generator_mode\":\"dns\"}",
      &command,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_traffic_generator_mode);
  TEST_ASSERT_EQUAL_STRING("dns", command.traffic_generator_mode.c_str());

  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"x8\",\"command\":\"update_sensing\",\"traffic_generator_mode\":\"dns_tcp\"}",
      &command,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("dns_tcp", command.traffic_generator_mode.c_str());

}

void test_parse_espectre_command_rejects_missing_command_and_invalid_threshold(void) {
  EspectreCommand command;
  std::string error;

  TEST_ASSERT_FALSE(parse_espectre_command("{\"command_id\":\"x3\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("missing command", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_command("{\"command_id\":\"test\",\"command\":\"update_device\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid device label (accepted: a single-line string)", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"test\",\"command\":\"update_device\",\"label\":123}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid device label (accepted: a single-line string)", error.c_str());

  TEST_ASSERT_FALSE(
      parse_espectre_command("{\"command_id\":\"test\",\"command\":\"update_sensing\",\"threshold\":\"abc\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid threshold (accepted: 0.0-1.0)", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_command("{\"command_id\":\"test\",\"command\":\"update_sensing\",\"threshold\":1e999}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid threshold (accepted: 0.0-1.0)", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"test\",\"command\":\"update_sensing\",\"motion_on_hits\":\"abc\",\"motion_off_hits\":2}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid motion hits (accepted: motion_on_hits and motion_off_hits in 1-20)", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"test\",\"command\":\"update_sensing\",\"detector\":\"pca\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid detector (accepted: lightweight and high_accuracy)", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"test\",\"command\":\"update_sensing\",\"csi_traffic_mode\":\"bogus\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid csi traffic mode (accepted: internal and external)", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"test\",\"command\":\"update_sensing\",\"csi_traffic_mode\":\"pacing\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid csi traffic mode (accepted: internal and external)", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"test\",\"command\":\"update_sensing\",\"traffic_generator_mode\":\"udp\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid traffic generator mode (accepted: ping, dns, and dns_tcp)", error.c_str());

  TEST_ASSERT_TRUE(parse_espectre_command("{\"command_id\":\"test\",\"command\":\"check_ota\"}", &command, &error, &frontend_ota_protocol()));

  TEST_ASSERT_TRUE(parse_espectre_command("{\"command_id\":\"test\",\"command\":\"start_ota\"}", &command, &error, &frontend_ota_protocol()));

  TEST_ASSERT_FALSE(parse_espectre_command(
      "{\"command_id\":\"test\",\"command\":\"start_ota\",\"image_url\":\"https://fw.example/native.bin\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("ota overrides are not supported (manifest_url, image_url, and version are not accepted)",
                           error.c_str());

  TEST_ASSERT_FALSE(
      parse_espectre_command("{\"command_id\":\"test\",\"command\":\"check_ota\",\"manifest_url\":\"\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("ota overrides are not supported (manifest_url, image_url, and version are not accepted)",
                           error.c_str());

  TEST_ASSERT_FALSE(
      parse_espectre_command("{\"command_id\":\"test\",\"command\":\"check_ota\",\"channel\":\"latest\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid ota channel (accepted: release, preview, and develop)", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_command("{\"command_id\":\"test\",\"command\":\"device\"}", nullptr, &error, &frontend_ota_protocol()));
}

void test_parse_espectre_command_rejects_oversized_payload_before_json_parsing(void) {
  EspectreCommand command;
  command.command_id = "stale";
  std::string error;
  const std::string oversized(ESPECTRE_COMMAND_MAX_PAYLOAD_SIZE + 1U, 'x');

  TEST_ASSERT_FALSE(parse_espectre_command(oversized, &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("command payload exceeds the size limit", error.c_str());
  TEST_ASSERT_TRUE(command.command_id.empty());
}

void test_ota_status_payload_includes_expected_fields(void) {
  EspectreDeviceConfig config;
  config.device_id = 0x000000000000000AULL;

  EspectreOtaStatus status;
  status.state = EspectreOtaState::UPDATE_AVAILABLE;
  status.current_version = "1.0.0";
  status.target_version = "1.1.0";
  status.manifest_url = "https://fw.example/manifest.json";
  status.image_url = "https://fw.example/native.bin";
  status.default_channel = "release";
  status.channel = "preview";
  status.message = "update available";
  status.busy = false;
  status.update_available = true;

  const std::string payload = espectre_ota_status_payload(config, status, 4321);

  TEST_ASSERT_TRUE(payload.find("\"device_id\"") == std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"state\":\"update_available\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"current_version\":\"1.0.0\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"target_version\":\"1.1.0\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"update_available\":true") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"manifest_url\":\"https://fw.example/manifest.json\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"default_channel\":\"release\"") != std::string::npos);
  TEST_ASSERT_TRUE(payload.find("\"channel\":\"preview\"") != std::string::npos);
}

void test_ota_channel_helpers(void) {
  TEST_ASSERT_TRUE(espectre_ota_channel_accepted("release"));
  TEST_ASSERT_TRUE(espectre_ota_channel_accepted("preview"));
  TEST_ASSERT_TRUE(espectre_ota_channel_accepted("develop"));
  TEST_ASSERT_FALSE(espectre_ota_channel_accepted(""));
  TEST_ASSERT_FALSE(espectre_ota_channel_accepted("latest"));
  TEST_ASSERT_EQUAL_STRING(
      "https://github.com/francescopace/espectre/releases/latest/download/firmware-manifest-release.json",
      espectre_ota_manifest_url("native", "esp32c3", "release").c_str());
  const std::string preview_url =
      std::string("https://github.com/francescopace/espectre/releases/download/") +
      ESPECTRE_OTA_RELEASE_TAG_PREVIEW + "/firmware-manifest-preview.json";
  const std::string develop_url =
      std::string("https://github.com/francescopace/espectre/releases/download/") +
      ESPECTRE_OTA_RELEASE_TAG_DEVELOP + "/firmware-manifest-develop.json";
  TEST_ASSERT_EQUAL_STRING(preview_url.c_str(),
                           espectre_ota_manifest_url("native", "esp32c6", "preview").c_str());
  TEST_ASSERT_EQUAL_STRING(develop_url.c_str(),
                           espectre_ota_manifest_url("native", "esp32s3", "develop").c_str());
  TEST_ASSERT_TRUE(espectre_ota_manifest_url("native", "esp32c3", "latest").empty());

}

void test_parse_espectre_config_command_updates_supported_fields(void) {
  EspectreDeviceConfig config;
  std::string error;

  TEST_ASSERT_TRUE(parse_espectre_config_command("SET_DEVICE_CONFIG:device_label=Office", &config, &error));

  TEST_ASSERT_EQUAL_STRING("Office", config.device_label.c_str());
}

void test_parse_espectre_config_command_rejects_invalid_inputs(void) {
  EspectreDeviceConfig config;
  std::string error;

  TEST_ASSERT_FALSE(parse_espectre_config_command("BAD_PREFIX:device_label=Office", &config, &error));
  TEST_ASSERT_EQUAL_STRING("invalid prefix", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_config_command("SET_DEVICE_CONFIG:device_label", &config, &error));
  TEST_ASSERT_EQUAL_STRING("expected key=value", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_config_command("SET_DEVICE_CONFIG:mqtt_port=2883", &config, &error));
  TEST_ASSERT_EQUAL_STRING("invalid config field", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_config_command("SET_DEVICE_CONFIG:unsupported=value", &config, &error));
  TEST_ASSERT_EQUAL_STRING("invalid config field", error.c_str());

  TEST_ASSERT_FALSE(parse_espectre_config_command("SET_DEVICE_CONFIG:device_label=test", nullptr, &error));
}

void test_direct_http_request_parses_canonical_message(void) {
  DirectRequest request;
  std::string error;

  TEST_ASSERT_TRUE(parse_direct_http_request(
      "PATCH", "/espectre/v1/sensing", "{\"threshold\":0.42}",
      &request,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(request.command_id.empty());
  TEST_ASSERT_EQUAL_STRING("update_sensing", request.command.c_str());
  TEST_ASSERT_EQUAL_STRING("{\"threshold\":0.42}", request.params.c_str());

  TEST_ASSERT_TRUE(parse_direct_http_request(
      "GET", "/espectre/v1/device", "",
      &request,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("device", request.command.c_str());
  TEST_ASSERT_EQUAL_STRING("{}", request.params.c_str());
}

void test_direct_http_request_separates_framing_from_canonical_validation(void) {
  DirectRequest request;
  std::string error;

  TEST_ASSERT_FALSE(parse_direct_http_request("GET", "/espectre/v1/missing", "", &request, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("unsupported Direct resource or method", error.c_str());
  TEST_ASSERT_FALSE(parse_direct_http_request("PATCH", "/espectre/v1/sensing", "{", &request, &error, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_direct_http_request(
      "PATCH", "/espectre/v1/sensing", "{\"threshold\":0.5,\"threshold\":0.6}",
      &request,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("duplicate JSON object field", error.c_str());
  TEST_ASSERT_TRUE(parse_direct_http_request(
      "PATCH", "/espectre/v1/sensing", "{\"protocol_version\":\"2.0\"}",
      &request,
      &error, &frontend_ota_protocol()));
  EspectreCommand command;
  TEST_ASSERT_FALSE(direct_http_request_to_command(request, &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("unknown command parameter", error.c_str());

  TEST_ASSERT_TRUE(parse_direct_http_request(
      "PATCH", "/espectre/v1/sensing", "{\"unexpected\":true}",
      &request,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(direct_http_request_to_command(request, &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("unknown command parameter", error.c_str());

  const std::string oversized(ESPECTRE_DIRECT_MAX_REQUEST_SIZE + 1U, 'x');
  TEST_ASSERT_FALSE(parse_direct_http_request("PATCH", "/espectre/v1/sensing", oversized, &request, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("Direct request exceeds the size limit", error.c_str());
  TEST_ASSERT_FALSE(parse_direct_http_request("GET", "/espectre/v1/health", "", nullptr, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("request output is required", error.c_str());
}

void test_canonical_message_builders_and_transport_catalog(void) {
  EspectreDeviceConfig config;
  EspectreCommand command;
  command.command_id = "req-1";
  command.command = "update_sensing";
  TEST_ASSERT_EQUAL_STRING(
      "{\"command_id\":\"req-1\",\"command\":\"update_sensing\",\"threshold\":0.5}",
      espectre_command_request_payload("req-1", "update_sensing", "{\"threshold\":0.5}").c_str());
  TEST_ASSERT_EQUAL_STRING(
      "{\"command_id\":\"req-1\",\"command\":\"update_sensing\",\"accepted\":true,\"code\":\"ok\",\"message\":\"updated\",\"data\":{\"threshold\":0.5}}",
      espectre_command_result_payload(config, command, true, "ok", "updated", "{\"threshold\":0.5}").c_str());
  const std::string messages = espectre_message_catalog_payload();
  TEST_ASSERT_TRUE(messages.find("\"txtvers\":\"1\"") != std::string::npos);
  TEST_ASSERT_TRUE(messages.find("\"protovers\":\"1.0\"") != std::string::npos);
  TEST_ASSERT_TRUE(messages.find("topic_suffix") == std::string::npos);
  const std::string mapping = espectre_transport_mapping_payload();
  TEST_ASSERT_TRUE(mapping.find("\"framing\":\"sse\"") != std::string::npos);
  TEST_ASSERT_TRUE(mapping.find("\"topic_suffix\":\"commands/result\"") != std::string::npos);
  const std::string catalog = espectre_protocol_catalog_payload();
  TEST_ASSERT_TRUE(catalog.find("\"message_model\":") != std::string::npos);
  TEST_ASSERT_TRUE(catalog.find("\"transport_mapping\":") != std::string::npos);
}

void test_direct_http_request_reuses_transport_neutral_command_validation(void) {
  DirectRequest request;
  EspectreCommand command;
  std::string error;

  TEST_ASSERT_TRUE(parse_direct_http_request(
      "PATCH", "/espectre/v1/sensing", "{\"motion_on_hits\":6,\"motion_off_hits\":4}",
      &request,
      &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(direct_http_request_to_command(request, &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.command_id.empty());
  TEST_ASSERT_EQUAL_STRING("update_sensing", command.command.c_str());
  TEST_ASSERT_EQUAL_UINT8(6U, command.motion_on_hits);
  TEST_ASSERT_EQUAL_UINT8(4U, command.motion_off_hits);

  request.command = "update_sensing";
  request.params = "{\"threshold\":\"0.5\"}";
  TEST_ASSERT_FALSE(direct_http_request_to_command(request, &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("invalid threshold (accepted: 0.0-1.0)", error.c_str());

  request.command = "unknown_method";
  request.params = "{}";
  TEST_ASSERT_TRUE(direct_http_request_to_command(request, &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("unknown_method", command.command.c_str());
}

void test_direct_http_preserves_embedded_nuls_for_canonical_validation(void) {
  const char *params[] = {
      R"({"label":"room\u0000hidden"})",
      R"({"label\u0000hidden":"room"})",
  };
  for (const char *payload : params) {
    DirectRequest request;
    EspectreCommand direct_command;
    EspectreCommand mqtt_command;
    std::string direct_error;
    std::string mqtt_error;
    TEST_ASSERT_TRUE(parse_direct_http_request(
        "PATCH", "/espectre/v1/device", payload, &request, &direct_error, &frontend_ota_protocol()));
    TEST_ASSERT_FALSE(direct_http_request_to_command(request, &direct_command, &direct_error, &frontend_ota_protocol()));
    std::string mqtt_payload = R"({"command_id":"nul-test","command":"update_device",)";
    mqtt_payload += payload + 1;
    TEST_ASSERT_FALSE(parse_espectre_command(mqtt_payload, &mqtt_command, &mqtt_error, &frontend_ota_protocol()));
    TEST_ASSERT_EQUAL_STRING(mqtt_error.c_str(), direct_error.c_str());
  }
}

void test_direct_http_configuration_commands_validate_write_only_fields(void) {
  EspectreCommand command;
  std::string error;

  TEST_ASSERT_TRUE(parse_espectre_command_request(
      "wifi-1",
      "set_wifi_bssid",
      "{\"bssid\":\"E6:FA:C4:20:19:DE\",\"force\":true}",
      &command,
      &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_wifi_bssid);
  TEST_ASSERT_EQUAL_STRING("E6:FA:C4:20:19:DE", command.wifi_bssid.c_str());
  TEST_ASSERT_TRUE(command.has_wifi_bssid_force);
  TEST_ASSERT_TRUE(command.wifi_bssid_force);

  TEST_ASSERT_TRUE(parse_espectre_command_request(
      "mqtt-1",
      "update_mqtt",
      "{\"scheme\":\"mqtt\",\"host\":\"homeassistant.local\",\"port\":1883,\"username\":\"mqtt\",\"password\":\"secret\"}",
      &command,
      &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_mqtt_scheme);
  TEST_ASSERT_EQUAL_STRING("mqtt", command.mqtt_scheme.c_str());
  TEST_ASSERT_EQUAL_STRING("homeassistant.local", command.mqtt_host.c_str());
  TEST_ASSERT_EQUAL(1883U, command.mqtt_port);
  TEST_ASSERT_EQUAL_STRING("mqtt", command.mqtt_username.c_str());
  TEST_ASSERT_TRUE(command.has_mqtt_password);

  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "wifi-bad", "set_wifi_bssid", "{\"bssid\":\"not-a-bssid\"}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "wifi-force-bad", "set_wifi_bssid",
      "{\"bssid\":\"E6:FA:C4:20:19:DE\",\"force\":\"true\"}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "wifi-credentials", "set_wifi_bssid", "{\"ssid\":\"Lab\",\"password\":\"secret\"}",
      &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "wifi-band", "set_wifi_bssid", "{\"bssid\":\"\",\"band_policy\":\"auto\"}",
      &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(parse_espectre_command_request(
      "clear-bssid", "clear_wifi_bssid", "{}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "clear-bssid-bad", "clear_wifi_bssid", "{\"bssid\":\"E6:FA:C4:20:19:DE\"}",
      &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(parse_espectre_command_request(
      "clear-wifi", "clear_wifi_config", "{}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "clear-wifi-bad", "clear_wifi_config", "{\"ssid\":\"Lab\"}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "mqtt-missing-scheme", "update_mqtt", "{\"host\":\"homeassistant.local\",\"port\":1883}",
      &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "mqtt-missing-port", "update_mqtt", "{\"scheme\":\"mqtt\",\"host\":\"homeassistant.local\"}",
      &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "mqtt-bad-scheme", "update_mqtt", "{\"scheme\":\"ws\",\"host\":\"broker.local\",\"port\":80}",
      &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "mqtt-bad-host", "update_mqtt",
      "{\"scheme\":\"mqtts\",\"host\":\"mqtts://broker.example.com\",\"port\":8883}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "mqtt-bad-port", "update_mqtt", "{\"scheme\":\"mqtt\",\"host\":\"homeassistant.local\",\"port\":0}",
      &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(parse_espectre_command_request("clear-2", "clear_mqtt", "{}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
}

void test_direct_http_read_and_sensing_methods_map_to_shared_commands(void) {
  const char *methods[] = {
      "capabilities", "device", "health", "sensing", "diagnostics", "ota", "wifi_access_points"};
  for (const char *method : methods) {
    EspectreCommand command;
    std::string error;
    TEST_ASSERT_TRUE(parse_espectre_command_request("direct-read", method, "{}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
    TEST_ASSERT_EQUAL_STRING(method, command.command.c_str());
    TEST_ASSERT_EQUAL_STRING("direct-read", command.command_id.c_str());
  }

  EspectreCommand command;
  std::string error;
  TEST_ASSERT_TRUE(
      parse_espectre_command_request("direct-sensing", "update_sensing", "{\"enabled\":true}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_sensing_enabled);
  TEST_ASSERT_TRUE(command.sensing_enabled);

  TEST_ASSERT_TRUE(parse_espectre_command_request("legacy", "start_sensing", "{}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("start_sensing", command.command.c_str());
  TEST_ASSERT_FALSE(parse_espectre_command_request(
      "extra", "update_sensing", "{\"enabled\":true,\"unexpected\":1}", &command, &error, ESPECTRE_PROTOCOL_VERSION, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("unknown command parameter", error.c_str());
}

void test_espectre_protocol_parses_config_and_rejects_bad_commands(void) {
  EspectreDeviceConfig config;
  std::string error;
  TEST_ASSERT_TRUE(parse_espectre_config_command("SET_DEVICE_CONFIG:device_label=Living Room", &config, &error));
  TEST_ASSERT_EQUAL_STRING("Living Room", config.device_label.c_str());
  TEST_ASSERT_FALSE(parse_espectre_config_command("SET_DEVICE_CONFIG:device_id=manual", &config, &error));
  TEST_ASSERT_FALSE(parse_espectre_config_command("SET_DEVICE_CONFIG:mqtt_port=1884", &config, &error));

  EspectreCommand command;
  TEST_ASSERT_TRUE(parse_espectre_command("{\"command_id\":\"test\",\"command\":\"update_sensing\",\"threshold\":0.325}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_EQUAL_STRING("update_sensing", command.command.c_str());
  TEST_ASSERT_TRUE(command.has_threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.325f, command.threshold);
  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"test\",\"command\":\"update_sensing\",\"motion_on_hits\":6,\"motion_off_hits\":4}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_motion_hits);
  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"test\",\"command\":\"update_sensing\",\"csi_traffic_mode\":\"external\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_csi_traffic_mode);
  TEST_ASSERT_TRUE(parse_espectre_command(
      "{\"command_id\":\"test\",\"command\":\"update_sensing\",\"traffic_generator_mode\":\"dns_tcp\"}", &command, &error, &frontend_ota_protocol()));
  TEST_ASSERT_TRUE(command.has_traffic_generator_mode);
  TEST_ASSERT_FALSE(parse_espectre_command("{\"command_id\":\"test\",\"command\":\"update_sensing\",\"threshold\":\"bad\"}", &command, &error, &frontend_ota_protocol()));
}

void test_protocol_json_writers_and_extractors_cover_edge_cases(void) {
  append_json_string(nullptr, "ignored");
  std::string json;
  append_json_string(&json, nullptr);
  TEST_ASSERT_EQUAL_STRING("\"\"", json.c_str());

  json.clear();
  append_json_string(&json, "quote\" slash\\ line\nreturn\rtab\t");
  TEST_ASSERT_EQUAL_STRING("\"quote\\\" slash\\\\ line\\nreturn\\rtab\\t\"", json.c_str());

  append_json_pair(nullptr, "ignored", "ignored");
  json = "{";
  append_json_pair(&json, "first", "one", true);
  append_json_pair(&json, "second", "two");
  json += "}";
  TEST_ASSERT_EQUAL_STRING("{\"first\":\"one\",\"second\":\"two\"}", json.c_str());

  TEST_ASSERT_FALSE(has_json_key(json, nullptr));
  TEST_ASSERT_FALSE(has_json_key(json, ""));
  TEST_ASSERT_FALSE(has_json_key("{\"first\"}", "first"));
  TEST_ASSERT_TRUE(has_json_key(json, "second"));

  TEST_ASSERT_TRUE(extract_json_string(json, nullptr).empty());
  TEST_ASSERT_TRUE(extract_json_string(json, "missing").empty());
  TEST_ASSERT_TRUE(extract_json_string("{\"key\"}", "key").empty());
  TEST_ASSERT_TRUE(extract_json_string("{\"key\":null}", "key").empty());
  TEST_ASSERT_TRUE(extract_json_string("{\"key\":\"unterminated}", "key").empty());
  TEST_ASSERT_EQUAL_STRING("escaped\"value",
                           extract_json_string("{\"key\":\"escaped\\\"value\"}", "key").c_str());

  TEST_ASSERT_TRUE(extract_json_number_token(json, nullptr).empty());
  TEST_ASSERT_TRUE(extract_json_number_token(json, "missing").empty());
  TEST_ASSERT_TRUE(extract_json_number_token("{\"number\"}", "number").empty());
  TEST_ASSERT_TRUE(extract_json_number_token("{\"number\":   }", "number").empty());
  TEST_ASSERT_EQUAL_STRING("-1.25e+3",
                           extract_json_number_token("{\"number\": -1.25e+3}", "number").c_str());
}

void test_protocol_json_url_encoding_validates_tokens(void) {
  std::string decoded;
  std::string error;
  TEST_ASSERT_FALSE(decode_urlencoded_component("value", nullptr, &error));
  TEST_ASSERT_EQUAL_STRING("decoded output is required", error.c_str());
  TEST_ASSERT_TRUE(decode_urlencoded_component("hello+world%21%af%AF", &decoded, &error));
  TEST_ASSERT_EQUAL_STRING("hello world!\xAF\xAF", decoded.c_str());
  TEST_ASSERT_EQUAL_STRING("AZaz09-_.~%20%2F%2B", encode_urlencoded_component("AZaz09-_.~ /+").c_str());

  TEST_ASSERT_FALSE(decode_urlencoded_component("%", &decoded, &error));
  TEST_ASSERT_EQUAL_STRING("truncated escape sequence", error.c_str());
  TEST_ASSERT_FALSE(decode_urlencoded_component("%GG", &decoded, &error));
  TEST_ASSERT_EQUAL_STRING("invalid escape sequence", error.c_str());

  std::vector<std::pair<std::string, std::string>> pairs;
  TEST_ASSERT_FALSE(parse_urlencoded_key_value_pairs("a=b", nullptr, &error));
  TEST_ASSERT_EQUAL_STRING("pairs output is required", error.c_str());
  TEST_ASSERT_FALSE(parse_urlencoded_key_value_pairs("", &pairs, &error));
  TEST_ASSERT_EQUAL_STRING("missing payload", error.c_str());
  TEST_ASSERT_FALSE(parse_urlencoded_key_value_pairs("a=b&&c=d", &pairs, &error));
  TEST_ASSERT_EQUAL_STRING("empty key-value token", error.c_str());
  TEST_ASSERT_FALSE(parse_urlencoded_key_value_pairs("missing", &pairs, &error));
  TEST_ASSERT_EQUAL_STRING("invalid key-value token", error.c_str());
  TEST_ASSERT_FALSE(parse_urlencoded_key_value_pairs("=value", &pairs, &error));
  TEST_ASSERT_FALSE(parse_urlencoded_key_value_pairs("bad%GG=value", &pairs, &error));
  TEST_ASSERT_FALSE(parse_urlencoded_key_value_pairs("key=bad%GG", &pairs, &error));
  TEST_ASSERT_TRUE(parse_urlencoded_key_value_pairs("name=Living+Room&empty=&encoded=%2Fapi", &pairs, &error));
  TEST_ASSERT_EQUAL(3U, pairs.size());
  TEST_ASSERT_EQUAL_STRING("Living Room", pairs[0].second.c_str());
  TEST_ASSERT_EQUAL_STRING("", pairs[1].second.c_str());
  TEST_ASSERT_EQUAL_STRING("/api", pairs[2].second.c_str());
}

void test_protocol_json_object_parser_decodes_every_value_type(void) {
  const std::string payload =
      " { \"text\":\"quote\\\" slash\\/ backslash\\\\ controls\\b\\f\\n\\r\\t\","
      "\"ascii\":\"\\u0041\",\"two_byte\":\"\\u00A2\",\"three_byte\":\"\\u20AC\","
      "\"four_byte\":\"\\uD83D\\uDE00\",\"number\":-12.5e+2,\"truth\":true,"
      "\"lie\":false,\"nothing\":null,\"object\":{\"nested\":1},\"array\":[1,\"two\",{}] } ";
  std::vector<JsonObjectField> fields;
  std::string error;
  TEST_ASSERT_TRUE(parse_json_object_fields(payload, &fields, &error));
  TEST_ASSERT_EQUAL(11U, fields.size());
  TEST_ASSERT_TRUE(find_json_object_field(fields, nullptr) == nullptr);
  TEST_ASSERT_TRUE(find_json_object_field(fields, "missing") == nullptr);

  const JsonObjectField *text = find_json_object_field(fields, "text");
  TEST_ASSERT_TRUE(text != nullptr);
  TEST_ASSERT_TRUE(text->type == JsonValueType::STRING);
  TEST_ASSERT_EQUAL_STRING("quote\" slash/ backslash\\ controls\b\f\n\r\t", text->value.c_str());
  TEST_ASSERT_EQUAL_STRING("A", find_json_object_field(fields, "ascii")->value.c_str());
  TEST_ASSERT_EQUAL_STRING("\xC2\xA2", find_json_object_field(fields, "two_byte")->value.c_str());
  TEST_ASSERT_EQUAL_STRING("\xE2\x82\xAC", find_json_object_field(fields, "three_byte")->value.c_str());
  TEST_ASSERT_EQUAL_STRING("\xF0\x9F\x98\x80", find_json_object_field(fields, "four_byte")->value.c_str());
  TEST_ASSERT_TRUE(find_json_object_field(fields, "number")->type == JsonValueType::NUMBER);
  TEST_ASSERT_EQUAL_STRING("-12.5e+2", find_json_object_field(fields, "number")->value.c_str());
  TEST_ASSERT_TRUE(find_json_object_field(fields, "truth")->type == JsonValueType::BOOLEAN);
  TEST_ASSERT_TRUE(find_json_object_field(fields, "nothing")->type == JsonValueType::NULL_VALUE);
  TEST_ASSERT_TRUE(find_json_object_field(fields, "object")->type == JsonValueType::OBJECT);
  TEST_ASSERT_TRUE(find_json_object_field(fields, "array")->type == JsonValueType::ARRAY);
}

void test_protocol_json_object_parser_rejects_invalid_documents(void) {
  std::vector<JsonObjectField> fields;
  std::string error;
  const auto rejects = [&fields, &error](const std::string &payload) {
    error.clear();
    return !parse_json_object_fields(payload, &fields, &error) && !error.empty();
  };

  TEST_ASSERT_FALSE(parse_json_object_fields("{}", nullptr, &error));
  TEST_ASSERT_TRUE(rejects("[]"));
  TEST_ASSERT_TRUE(rejects("{} trailing"));
  TEST_ASSERT_TRUE(rejects("{invalid:1}"));
  TEST_ASSERT_TRUE(rejects("{\"key\" 1}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":?}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":1 \"other\":2}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":1,}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":1,\"key\":2}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":\"line\nfeed\"}"));
  TEST_ASSERT_TRUE(rejects(std::string("{\"key\":\"trailing") + "\\"));
  TEST_ASSERT_TRUE(rejects("{\"key\":\"\\x\"}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":\"\\u12G4\"}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":\"\\uD800\"}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":\"\\uD800\\u0041\"}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":\"\\uDC00\"}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":01}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":-}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":1.}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":1e}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":1e+}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":tru}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":nul}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":[1 2]}"));
  TEST_ASSERT_TRUE(rejects("{\"key\":[1,]}"));

  std::string nested = "{\"key\":";
  nested.append(18U, '[');
  nested += "0";
  nested.append(18U, ']');
  nested += "}";
  TEST_ASSERT_TRUE(rejects(nested));
}

int process(void) {
  UNITY_BEGIN();
  RUN_TEST(test_frontend_protocol_extensions_share_capabilities_routing_and_validation);
  RUN_TEST(test_sdk_validators_reject_out_of_range_parameters_before_dispatch);
  RUN_TEST(test_ota_version_ordering_blocks_downgrades_and_divergent_builds);
  RUN_TEST(test_device_id_helpers_format_and_parse_canonical_hex_consistently);
  RUN_TEST(test_effective_device_helpers_and_topic_generation_use_defaults);
  RUN_TEST(test_clear_mqtt_config_resets_runtime_defaults);
  RUN_TEST(test_parse_mqtt_batch_config_command_updates_all_fields);
  RUN_TEST(test_mqtt_config_validation_rejects_uri_framing_and_preserves_the_previous_config);
  RUN_TEST(test_status_telemetry_and_diagnostics_payloads_include_expected_fields);
  RUN_TEST(test_diagnostics_payload_includes_enabled_runtime_sample);
  RUN_TEST(test_info_payload_uses_defaults_and_optional_sections);
  RUN_TEST(test_info_payload_omits_optional_sections_when_empty);
  RUN_TEST(test_info_normalization_reports_runtime_channel_and_csi_profile);
  RUN_TEST(test_command_result_payload_includes_acceptance_and_message);
  RUN_TEST(test_parse_espectre_command_parses_info_and_threshold_commands);
  RUN_TEST(test_parse_espectre_command_rejects_missing_command_and_invalid_threshold);
  RUN_TEST(test_parse_espectre_command_rejects_oversized_payload_before_json_parsing);
  RUN_TEST(test_ota_status_payload_includes_expected_fields);
  RUN_TEST(test_ota_channel_helpers);
  RUN_TEST(test_parse_espectre_config_command_updates_supported_fields);
  RUN_TEST(test_parse_espectre_config_command_rejects_invalid_inputs);
  RUN_TEST(test_direct_http_request_parses_canonical_message);
  RUN_TEST(test_direct_http_request_separates_framing_from_canonical_validation);
  RUN_TEST(test_canonical_message_builders_and_transport_catalog);
  RUN_TEST(test_direct_http_request_reuses_transport_neutral_command_validation);
  RUN_TEST(test_direct_http_preserves_embedded_nuls_for_canonical_validation);
  RUN_TEST(test_direct_http_configuration_commands_validate_write_only_fields);
  RUN_TEST(test_direct_http_read_and_sensing_methods_map_to_shared_commands);
  RUN_TEST(test_espectre_protocol_parses_config_and_rejects_bad_commands);
  RUN_TEST(test_protocol_json_writers_and_extractors_cover_edge_cases);
  RUN_TEST(test_protocol_json_url_encoding_validates_tokens);
  RUN_TEST(test_protocol_json_object_parser_decodes_every_value_type);
  RUN_TEST(test_protocol_json_object_parser_rejects_invalid_documents);
  return UNITY_END();
}

#if defined(ESP_PLATFORM)
extern "C" void app_main(void) { process(); }
#else
int main(int argc, char **argv) {
  (void) argc;
  (void) argv;
  return process();
}
#endif
