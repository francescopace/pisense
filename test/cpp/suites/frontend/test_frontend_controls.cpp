/*
 * ESPectre - Frontend Controls Unit Tests
 *
 * Unit tests for Frontend Controls.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include <algorithm>
#include <memory>
#define private public
#define protected public
#include "detector_select.h"
#include "diagnostics_button.h"
#include "espectre.h"
#include "motion_hits_number.h"
#include "recalibrate_button.h"
#include "sensing_switch.h"
#include "threshold_number.h"
#include "traffic_mode_select.h"
#undef protected
#undef private

#include "esphome/core/hal.h"
#include "esphome/core/preferences.h"
#include "direct_http_protocol.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "frontend_runtime_shim.h"
#include "mdns.h"
#include "runtime_direct_http_bridge.h"

using namespace esphome::espectre_component;

namespace {

class ESpectreComponentProbe : public ESpectreComponent {
 public:
  using ESpectreComponent::on_calibration_finished;
  using ESpectreComponent::on_calibration_started;
  using ESpectreComponent::on_live_telemetry;
  using ESpectreComponent::on_motion_state_changed;
  using ESpectreComponent::on_periodic_update;
  using ESpectreComponent::on_runtime_fault;
  using ESpectreComponent::on_threshold_changed;
  using ESpectreComponent::on_detector_changed;
};

class ThresholdNumberProbe : public ESpectreThresholdNumber {
 public:
  using ESpectreThresholdNumber::control;
};

class MotionHitsNumberProbe : public ESpectreMotionHitsNumber {
 public:
  using ESpectreMotionHitsNumber::control;
};

class SensingSwitchProbe : public ESpectreSensingSwitch {
 public:
  using ESpectreSensingSwitch::write_state;
};

class RecalibrateButtonProbe : public ESpectreRecalibrateButton {
 public:
  using ESpectreRecalibrateButton::press_action;
};

class DetectorSelectProbe : public ESpectreDetectorSelect {
 public:
  using ESpectreDetectorSelect::control;
};

class TrafficModeSelectProbe : public ESpectreTrafficModeSelect {
 public:
  using ESpectreTrafficModeSelect::control;
};

class DiagnosticsButtonProbe : public ESpectreDiagnosticsButton {
 public:
  using ESpectreDiagnosticsButton::press_action;
};

void complete_wifi_bssid_reconnect(ESpectreComponentProbe *component,
                                   const std::string &associated_bssid) {
  component->wifi_associated_bssid_.clear();
  component->wifi_has_ipv4_ = true;
  component->handle_wifi_bssid_association_(associated_bssid);
}

}  // namespace

void setUp(void) {
  frontend_runtime_shim::reset();
  httpd_mock_reset();
  mdns_mock_reset();
  esp_wifi_mock_reset();
  esphome::reset_mock_millis();
  esphome::reset_preference_store();
  esphome::global_preferences = nullptr;
}

void tearDown(void) {}

void test_espectre_component_setup_uses_mock_runtime_snapshot(void) {
  frontend_runtime_shim::state.snapshot.threshold = 4.5f;

  ESpectreComponentProbe component;
  component.setup();

  TEST_ASSERT_NOT_NULL(frontend_runtime_shim::state.last_listener);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_listener != &component);
  TEST_ASSERT_NOT_NULL(frontend_runtime_shim::state.last_instance);
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.live_telemetry_enabled);
  TEST_ASSERT_EQUAL_FLOAT(4.5f, component.get_threshold());
  TEST_ASSERT_EQUAL(1, g_httpd_mock.start_calls);
}

void test_espectre_component_direct_api_can_be_disabled(void) {
  ESpectreComponentProbe component;
  component.set_direct_api(false);
  component.setup();

  TEST_ASSERT_FALSE(component.is_failed());
  TEST_ASSERT_EQUAL(0, g_httpd_mock.start_calls);
  TEST_ASSERT_FALSE(component.mdns_bootstrap_responder_.configured_.load());

  component.loop();

  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.loop_calls);
  TEST_ASSERT_EQUAL(0, g_mdns_mock.service_add_call_count);
  TEST_ASSERT_EQUAL(0, g_mdns_mock.async_new_call_count);
}

void test_espectre_component_setup_marks_failed_when_runtime_setup_fails(void) {
  frontend_runtime_shim::state.setup_result = false;

  ESpectreComponentProbe component;
  component.setup();

  TEST_ASSERT_TRUE(component.is_failed());
}

void test_espectre_component_loop_and_destructor_forward_to_runtime(void) {
  {
    ESpectreComponentProbe component;
    component.setup();
    component.loop();
    TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.loop_calls);
  }

  TEST_ASSERT_TRUE(frontend_runtime_shim::state.shutdown_called);
}

void test_espectre_component_loop_does_not_poll_wifi_without_peer_discovery(void) {
  ESpectreComponentProbe component;
  component.setup();
  int wifi_snapshot_reads = 0;
  component.direct_bridge_.config_.wifi_snapshot_getter = [&wifi_snapshot_reads]() {
    wifi_snapshot_reads++;
    return DirectWifiSnapshot{};
  };

  component.loop();

  TEST_ASSERT_EQUAL(0, wifi_snapshot_reads);
  TEST_ASSERT_FALSE(component.peer_discovery_.wifi_ready_);
}

void test_espectre_component_direct_client_enables_live_telemetry(void) {
  ESpectreComponentProbe component;
  component.setup();
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.live_telemetry_enabled);

  component.direct_service_.notify_client_count_(1U);
  component.loop();
  component.loop();
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.live_telemetry_enabled);

  component.direct_service_.notify_client_count_(0U);
  component.loop();
  component.loop();
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.live_telemetry_enabled);
}

void test_espectre_component_raw_session_uses_shared_controller_and_recovers(void) {
  frontend_runtime_shim::state.capabilities.supports_raw_csi = true;
  ESpectreComponentProbe component;
  component.runtime_.config().device_id = 0x112233445566ULL;
  component.setup();
  TEST_ASSERT_FALSE(component.is_failed());
  TEST_ASSERT_EQUAL(espectre::ESPECTRE_DIRECT_HTTP_PORT, g_httpd_mock.last_config.server_port);

  const std::string capabilities = component.direct_bridge_.handle_request_(
      DirectRequest{"raw-capabilities", "capabilities", "{}"});
  TEST_ASSERT_TRUE(capabilities.find("\"csi\":true") != std::string::npos);
  TEST_ASSERT_TRUE(capabilities.find("\"endpoint\":\"/espectre/v1/csi\"") != std::string::npos);

  std::string message;
  TEST_ASSERT_TRUE(component.direct_service_.raw_session_requested_callback_(&message));
  TEST_ASSERT_TRUE(component.direct_service_.raw_diagnostics().active);
  TEST_ASSERT_EQUAL(RuntimeOperationState::RAW_COLLECTION,
                    component.runtime_.operation_state());

  const std::string busy = component.direct_bridge_.handle_request_(
      DirectRequest{"raw-busy", "update_sensing", "{\"enabled\":false}"});
  TEST_ASSERT_TRUE(busy.find("\"code\":\"busy_raw_collection\"") != std::string::npos);
  TEST_ASSERT_TRUE(component.runtime_.services_armed());

  TEST_ASSERT_TRUE(component.direct_service_.stop_raw_session(
      RawCsiStopReason::CHANNEL_CHANGED));
  TEST_ASSERT_FALSE(component.direct_service_.raw_diagnostics().active);
  component.loop();
  TEST_ASSERT_EQUAL(RuntimeOperationState::SENSING,
                    component.runtime_.operation_state());
}

void test_esphome_direct_exposes_common_wifi_and_label_capabilities(void) {
  esphome::ESPPreferences preferences;
  esphome::global_preferences = &preferences;
  ESpectreComponentProbe component;
  component.runtime_.config().device_id = 0x112233445566ULL;
  component.setup();
  TEST_ASSERT_FALSE(component.is_failed());
  const std::string device_id = format_espectre_device_id(component.runtime_.config().device_id);
  const std::string generated_name = espectre_device_name(component.runtime_.config().device_id, CONFIG_IDF_TARGET);
  TEST_ASSERT_EQUAL_STRING(generated_name.c_str(), component.device_name_().c_str());
  TEST_ASSERT_TRUE(component.device_label_().empty());
  TEST_ASSERT_EQUAL_STRING(generated_name.c_str(), component.display_name_().c_str());
  TEST_ASSERT_EQUAL_STRING(("ESPectre " + device_id).c_str(), component.mdns_instance_name_().c_str());
  TEST_ASSERT_EQUAL_STRING("espectre", component.peer_discovery_.local_candidate_.hostname.c_str());
  TEST_ASSERT_EQUAL_STRING(generated_name.c_str(), component.peer_discovery_.local_candidate_.name.c_str());
  TEST_ASSERT_EQUAL_STRING(("ESPectre " + device_id).c_str(),
                           component.peer_discovery_.local_candidate_.instance.c_str());

  const std::string default_info = component.direct_bridge_.handle_request_(
      DirectRequest{"default-info", "device", "{}"});
  TEST_ASSERT_TRUE(default_info.find("\"name\":\"" + generated_name + "\"") != std::string::npos);
  TEST_ASSERT_TRUE(default_info.find("\"label\":\"\"") != std::string::npos);

  const std::string capabilities = component.direct_bridge_.handle_request_(
      DirectRequest{"capabilities", "capabilities", "{}"});
  TEST_ASSERT_TRUE(capabilities.find("\"update_device\"") != std::string::npos);
  TEST_ASSERT_TRUE(capabilities.find("\"wifi\"") != std::string::npos);
  TEST_ASSERT_TRUE(capabilities.find("\"scan_wifi\"") != std::string::npos);
  TEST_ASSERT_TRUE(capabilities.find("\"set_wifi_bssid\"") != std::string::npos);
  TEST_ASSERT_TRUE(capabilities.find("\"clear_wifi_bssid\"") != std::string::npos);
  TEST_ASSERT_TRUE(capabilities.find("\"devices\"") != std::string::npos);
  TEST_ASSERT_TRUE(capabilities.find("\"clear_wifi_credentials\"") == std::string::npos);
  TEST_ASSERT_TRUE(capabilities.find("\"update_mqtt\"") == std::string::npos);
  TEST_ASSERT_TRUE(capabilities.find("\"start_ota\"") == std::string::npos);

  const std::string scan = component.direct_bridge_.handle_request_(
      DirectRequest{"scan", "scan_wifi", "{}"});
  const std::string pin = component.direct_bridge_.handle_request_(
      DirectRequest{"pin", "set_wifi_bssid", "{\"bssid\":\"E6:FA:C4:20:19:DE\"}"});
  TEST_ASSERT_TRUE(pin.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_TRUE(component.wifi_bssid_pin_.empty());
  complete_wifi_bssid_reconnect(&component, "E6:FA:C4:20:19:DE");
  TEST_ASSERT_EQUAL_STRING("E6:FA:C4:20:19:DE", component.wifi_bssid_pin_.c_str());
  const std::string unpin = component.direct_bridge_.handle_request_(
      DirectRequest{"unpin", "clear_wifi_bssid", "{}"});
  complete_wifi_bssid_reconnect(&component, "11:22:33:44:55:66");
  const std::string credential_reset = component.direct_bridge_.handle_request_(
      DirectRequest{"reset", "clear_wifi_credentials", "{}"});
  TEST_ASSERT_TRUE(scan.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_TRUE(pin.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_TRUE(unpin.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_TRUE(component.wifi_bssid_pin_.empty());
  TEST_ASSERT_TRUE(credential_reset.find("\"code\":\"unsupported\"") != std::string::npos);

  const std::string label = component.direct_bridge_.handle_request_(
      DirectRequest{"label", "update_device", "{\"label\":\"Kitchen ESPHome\"}"});
  TEST_ASSERT_TRUE(label.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_EQUAL_STRING(generated_name.c_str(), component.device_name_().c_str());
  TEST_ASSERT_EQUAL_STRING("Kitchen ESPHome", component.device_label_().c_str());
  TEST_ASSERT_EQUAL_STRING("Kitchen ESPHome", component.display_name_().c_str());
  const std::string info = component.direct_bridge_.handle_request_(
      DirectRequest{"", "device", "{}", "/espectre/v1/device", "GET"});
  const std::string config = component.direct_bridge_.handle_request_(
      DirectRequest{"", "sensing", "{}", "/espectre/v1/sensing", "GET"});
  TEST_ASSERT_TRUE(info.find("\"label\":\"Kitchen ESPHome\"") != std::string::npos);
  TEST_ASSERT_TRUE(config.find("Kitchen ESPHome") == std::string::npos);

  const std::string cleared = component.direct_bridge_.handle_request_(
      DirectRequest{"clear-label", "update_device", "{\"label\":\"\"}"});
  TEST_ASSERT_TRUE(cleared.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_TRUE(component.device_label_().empty());
  TEST_ASSERT_EQUAL_STRING(generated_name.c_str(), component.display_name_().c_str());
}

void test_wifi_bssid_pin_applies_config_during_station_reassociation(void) {
  std::memcpy(g_esp_wifi_mock.current_config.sta.ssid, "MatterLab", 9U);
  std::string message;
  bool station_transition_started = false;

  TEST_ASSERT_TRUE(espectre::apply_wifi_bssid_pin(
      "AA:BB:CC:DD:EE:FF", &message, &station_transition_started));
  TEST_ASSERT_TRUE(station_transition_started);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.get_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.deinit_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.init_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_storage_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_mode_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_promiscuous_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.start_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.current_config.sta.bssid_set);
  TEST_ASSERT_EQUAL_UINT8(0xAA, g_esp_wifi_mock.current_config.sta.bssid[0]);
  TEST_ASSERT_EQUAL_UINT8(0xFF, g_esp_wifi_mock.current_config.sta.bssid[5]);
}

void test_wifi_bssid_pin_reports_disconnect_failure(void) {
  std::memcpy(g_esp_wifi_mock.current_config.sta.ssid, "MatterLab", 9U);
  g_esp_wifi_mock.disconnect_results[0] = ESP_FAIL;
  g_esp_wifi_mock.disconnect_result_count = 1;
  std::string message;
  bool station_transition_started = false;

  TEST_ASSERT_FALSE(espectre::apply_wifi_bssid_pin(
      "AA:BB:CC:DD:EE:FF", &message, &station_transition_started));
  TEST_ASSERT_FALSE(station_transition_started);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.deinit_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.start_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_TRUE(message.find("disconnecting the station") != std::string::npos);
}

void test_wifi_bssid_pin_reports_no_transition_when_config_cannot_be_read(void) {
  g_esp_wifi_mock.get_config_result = ESP_FAIL;
  std::string message;
  bool station_transition_started = true;

  TEST_ASSERT_FALSE(espectre::apply_wifi_bssid_pin(
      "AA:BB:CC:DD:EE:FF", &message, &station_transition_started));
  TEST_ASSERT_FALSE(station_transition_started);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);
}

void test_wifi_bssid_pin_rejects_invalid_input_before_reading_station_config(void) {
  std::string message;
  bool station_transition_started = true;

  TEST_ASSERT_FALSE(espectre::apply_wifi_bssid_pin(
      "not-a-bssid", &message, &station_transition_started));
  TEST_ASSERT_FALSE(station_transition_started);
  TEST_ASSERT_EQUAL_STRING("BSSID must contain six hexadecimal octets", message.c_str());
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.get_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);
}

void test_wifi_bssid_pin_restarts_original_config_when_update_fails(void) {
  wifi_config_t original{};
  std::memcpy(original.sta.ssid, "MatterLab", 9U);
  original.sta.bssid_set = true;
  original.sta.bssid[0] = 0x11U;
  original.sta.bssid[5] = 0x66U;
  g_esp_wifi_mock.current_config = original;
  g_esp_wifi_mock.set_config_results[0] = ESP_FAIL;
  g_esp_wifi_mock.set_config_result_count = 1;
  std::string message;
  bool station_transition_started = false;

  TEST_ASSERT_FALSE(espectre::apply_wifi_bssid_pin(
      "AA:BB:CC:DD:EE:FF", &message, &station_transition_started));
  TEST_ASSERT_TRUE(station_transition_started);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.start_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_EQUAL(0, std::memcmp(&original, &g_esp_wifi_mock.current_config, sizeof(original)));
  TEST_ASSERT_TRUE(message.find("previous station association restarted") != std::string::npos);
}

void test_wifi_bssid_pin_restores_original_config_when_candidate_connect_fails(void) {
  wifi_config_t original{};
  std::memcpy(original.sta.ssid, "MatterLab", 9U);
  original.sta.bssid_set = true;
  original.sta.bssid[0] = 0x11U;
  original.sta.bssid[5] = 0x66U;
  g_esp_wifi_mock.current_config = original;
  g_esp_wifi_mock.connect_results[0] = ESP_FAIL;
  g_esp_wifi_mock.connect_results[1] = ESP_OK;
  g_esp_wifi_mock.connect_result_count = 2;
  std::string message;
  bool station_transition_started = false;

  TEST_ASSERT_FALSE(espectre::apply_wifi_bssid_pin(
      "AA:BB:CC:DD:EE:FF", &message, &station_transition_started));
  TEST_ASSERT_TRUE(station_transition_started);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.start_call_count);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_EQUAL(0, std::memcmp(&original, &g_esp_wifi_mock.current_config, sizeof(original)));
  TEST_ASSERT_TRUE(message.find("previous station association restarted") != std::string::npos);
}

void test_wifi_bssid_pin_rollback_restores_current_config_when_connect_fails(void) {
  wifi_config_t candidate{};
  std::memcpy(candidate.sta.ssid, "MatterLab", 9U);
  candidate.sta.bssid_set = true;
  candidate.sta.bssid[0] = 0xAAU;
  candidate.sta.bssid[5] = 0xFFU;
  g_esp_wifi_mock.current_config = candidate;
  g_esp_wifi_mock.connect_results[0] = ESP_FAIL;
  g_esp_wifi_mock.connect_results[1] = ESP_OK;
  g_esp_wifi_mock.connect_result_count = 2;
  std::string message;

  TEST_ASSERT_FALSE(espectre::apply_wifi_bssid_pin(
      "11:22:33:44:55:66", &message, nullptr));
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.start_call_count);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.current_config.sta.bssid_set);
  TEST_ASSERT_EQUAL_UINT8(0xAA, g_esp_wifi_mock.current_config.sta.bssid[0]);
  TEST_ASSERT_EQUAL_UINT8(0xFF, g_esp_wifi_mock.current_config.sta.bssid[5]);
}

void test_wifi_bssid_pin_rollback_restarts_current_config_when_update_fails(void) {
  wifi_config_t candidate{};
  std::memcpy(candidate.sta.ssid, "MatterLab", 9U);
  candidate.sta.bssid_set = true;
  candidate.sta.bssid[0] = 0xAAU;
  candidate.sta.bssid[5] = 0xFFU;
  g_esp_wifi_mock.current_config = candidate;
  g_esp_wifi_mock.set_config_results[0] = ESP_FAIL;
  g_esp_wifi_mock.set_config_result_count = 1;
  std::string message;

  TEST_ASSERT_FALSE(espectre::apply_wifi_bssid_pin(
      "11:22:33:44:55:66", &message, nullptr));
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.start_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_EQUAL(0, std::memcmp(&candidate,
                                  &g_esp_wifi_mock.current_config,
                                  sizeof(candidate)));
}

void test_espectre_component_publishes_cached_csi_diagnostics_on_demand(void) {
  ESpectreComponentProbe component;
  esphome::sensor::Sensor traffic_rate;
  esphome::sensor::Sensor callback_rate;
  esphome::sensor::Sensor accepted_rate;
  esphome::sensor::Sensor admitted_rate;
  esphome::sensor::Sensor filtered_rate;
  esphome::sensor::Sensor missing_rate;
  esphome::sensor::Sensor excess_rate;
  esphome::sensor::Sensor stale_rate;
  esphome::sensor::Sensor out_of_order_rate;
  esphome::sensor::Sensor occupancy;
  esphome::sensor::Sensor channel;
  esphome::sensor::Sensor rssi;
  component.set_traffic_rate_sensor(&traffic_rate);
  component.set_csi_callback_rate_sensor(&callback_rate);
  component.set_csi_accepted_rate_sensor(&accepted_rate);
  component.set_csi_admitted_rate_sensor(&admitted_rate);
  component.set_csi_filtered_rate_sensor(&filtered_rate);
  component.set_csi_missing_rate_sensor(&missing_rate);
  component.set_csi_excess_rate_sensor(&excess_rate);
  component.set_csi_stale_rate_sensor(&stale_rate);
  component.set_csi_out_of_order_rate_sensor(&out_of_order_rate);
  component.set_csi_occupancy_sensor(&occupancy);
  component.set_wifi_channel_sensor(&channel);
  component.set_wifi_rssi_sensor(&rssi);
  frontend_runtime_shim::state.diagnostics.wifi_channel = 8U;
  frontend_runtime_shim::state.diagnostics.wifi_rssi_dbm = -60;
  frontend_runtime_shim::state.diagnostics.traffic_packets_total = 100U;
  frontend_runtime_shim::state.diagnostics.csi_callbacks_total = 100U;
  frontend_runtime_shim::state.diagnostics.csi_accepted_total = 90U;
  frontend_runtime_shim::state.diagnostics.csi_admitted_total = 80U;
  frontend_runtime_shim::state.diagnostics.csi_filtered_total = 10U;
  frontend_runtime_shim::state.diagnostics.csi_pending_frame_drops_total = 5U;
  frontend_runtime_shim::state.diagnostics.csi_missing_slots_total = 20U;
  frontend_runtime_shim::state.diagnostics.csi_excess_total = 10U;
  frontend_runtime_shim::state.diagnostics.csi_stale_total = 2U;
  frontend_runtime_shim::state.diagnostics.csi_out_of_order_total = 1U;
  frontend_runtime_shim::state.diagnostics.csi_occupancy_slots = 80U;
  frontend_runtime_shim::state.diagnostics.csi_window_slots = 100U;
  component.setup();
  RuntimeSnapshot snapshot;
  snapshot.ready_to_publish = true;

  TEST_ASSERT_FALSE(traffic_rate.has_state());
  TEST_ASSERT_FALSE(channel.has_state());
  TEST_ASSERT_FALSE(rssi.has_state());

  frontend_runtime_shim::state.diagnostics.wifi_channel = 10U;
  frontend_runtime_shim::state.diagnostics.wifi_rssi_dbm = -55;
  frontend_runtime_shim::state.diagnostics.traffic_packets_total = 600U;
  frontend_runtime_shim::state.diagnostics.csi_callbacks_total = 580U;
  frontend_runtime_shim::state.diagnostics.csi_accepted_total = 540U;
  frontend_runtime_shim::state.diagnostics.csi_admitted_total = 480U;
  frontend_runtime_shim::state.diagnostics.csi_filtered_total = 40U;
  frontend_runtime_shim::state.diagnostics.csi_pending_frame_drops_total = 15U;
  frontend_runtime_shim::state.diagnostics.csi_missing_slots_total = 120U;
  frontend_runtime_shim::state.diagnostics.csi_excess_total = 60U;
  frontend_runtime_shim::state.diagnostics.csi_stale_total = 7U;
  frontend_runtime_shim::state.diagnostics.csi_out_of_order_total = 3U;
  frontend_runtime_shim::state.diagnostics.csi_occupancy_slots = 85U;
  RuntimeDiagnosticsSample &sample = frontend_runtime_shim::state.diagnostics_sample;
  sample.traffic_tx_pps = 100.0f;
  sample.csi_callback_pps = 96.0f;
  sample.csi_accepted_pps = 90.0f;
  sample.csi_admitted_pps = 80.0f;
  sample.csi_filtered_pps = 6.0f;
  sample.csi_pending_frame_drop_pps = 2.0f;
  sample.csi_missing_slots_pps = 20.0f;
  sample.csi_excess_pps = 10.0f;
  sample.csi_stale_pps = 1.0f;
  sample.csi_out_of_order_pps = 0.4f;
  sample.csi_occupancy_ratio = 0.85f;
  sample.wifi_channel = 10U;
  sample.wifi_rssi_dbm = -55;
  esphome::advance_mock_millis(5000U);
  component.on_periodic_update(snapshot, 100U);

  TEST_ASSERT_FALSE(traffic_rate.has_state());
  TEST_ASSERT_FALSE(channel.has_state());
  const std::string direct_diagnostics = component.direct_bridge_.handle_request_(
      DirectRequest{"diagnostics", "read_diagnostics", "{}"});
  TEST_ASSERT_TRUE(direct_diagnostics.find("\"traffic_tx_pps\":100") != std::string::npos);
  TEST_ASSERT_TRUE(direct_diagnostics.find("\"csi_callback_pps\":96") != std::string::npos);
  TEST_ASSERT_TRUE(direct_diagnostics.find("\"csi_pending_frame_drops_total\":15") !=
                   std::string::npos);
  TEST_ASSERT_TRUE(direct_diagnostics.find("\"csi_pending_frame_drop_pps\":2") !=
                   std::string::npos);

  DiagnosticsButtonProbe diagnostics_button;
  diagnostics_button.press_action();
  TEST_ASSERT_FALSE(traffic_rate.has_state());
  diagnostics_button.set_parent(&component);
  diagnostics_button.press_action();

  TEST_ASSERT_EQUAL_FLOAT(100.0f, traffic_rate.get_state());
  TEST_ASSERT_EQUAL_FLOAT(96.0f, callback_rate.get_state());
  TEST_ASSERT_EQUAL_FLOAT(90.0f, accepted_rate.get_state());
  TEST_ASSERT_EQUAL_FLOAT(80.0f, admitted_rate.get_state());
  TEST_ASSERT_EQUAL_FLOAT(6.0f, filtered_rate.get_state());
  TEST_ASSERT_EQUAL_FLOAT(20.0f, missing_rate.get_state());
  TEST_ASSERT_EQUAL_FLOAT(10.0f, excess_rate.get_state());
  TEST_ASSERT_EQUAL_FLOAT(1.0f, stale_rate.get_state());
  TEST_ASSERT_EQUAL_FLOAT(0.4f, out_of_order_rate.get_state());
  TEST_ASSERT_EQUAL_FLOAT(85.0f, occupancy.get_state());
  TEST_ASSERT_EQUAL_FLOAT(10.0f, channel.get_state());
  TEST_ASSERT_EQUAL_FLOAT(-55.0f, rssi.get_state());
}

void test_espectre_component_configuration_setters_update_runtime_config(void) {
  ESpectreComponentProbe component;
  esphome::sensor::Sensor movement_sensor;
  esphome::binary_sensor::BinarySensor binary_sensor;
  ThresholdNumberProbe threshold_number;
  MotionHitsNumberProbe motion_on_hits_number;
  MotionHitsNumberProbe motion_off_hits_number;
  SensingSwitchProbe sensing_switch;

  component.set_segmentation_window_size_ms(1500);
  component.set_direct_api(false);
  component.set_csi_target_pps(94);
  component.set_csi_traffic_mode("external");
  component.set_traffic_generator_mode("dns_tcp");
  TEST_ASSERT_TRUE(component.runtime_.config().traffic_generator_mode == RuntimeTrafficMode::DNS_TCP);
  component.set_traffic_generator_mode("ping");
  TEST_ASSERT_TRUE(component.runtime_.config().traffic_generator_mode == RuntimeTrafficMode::PING);
  component.set_detection_algorithm("high_accuracy");

  TEST_ASSERT_FALSE(component.direct_api_enabled_);
  TEST_ASSERT_TRUE(component.runtime_.config().detection_algorithm == DetectionAlgorithm::HIGH_ACCURACY);
  component.set_detection_algorithm("lightweight");
  TEST_ASSERT_TRUE(component.runtime_.config().detection_algorithm == DetectionAlgorithm::LIGHTWEIGHT);
  component.set_evaluation_interval_ms(500);
  component.set_motion_on_hits(4);
  component.set_motion_off_hits(5);
  component.set_lowpass_enabled(true);
  component.set_lowpass_cutoff(8.5f);
  component.set_hampel_enabled(false);
  component.set_hampel_window(9);
  component.set_hampel_threshold(4.5f);
  component.set_movement_sensor(&movement_sensor);
  component.set_motion_binary_sensor(&binary_sensor);
  component.set_threshold_number(&threshold_number);
  component.set_motion_on_hits_number(&motion_on_hits_number);
  component.set_motion_off_hits_number(&motion_off_hits_number);
  component.set_sensing_switch(&sensing_switch);

  component.set_traffic_generator_mode("dns");
  component.set_detection_algorithm("high_accuracy");

  TEST_ASSERT_EQUAL_FLOAT(RUNTIME_SEGMENTATION_THRESHOLD_DEFAULT,
                          component.runtime_.config().segmentation_threshold);
  TEST_ASSERT_EQUAL(1500U, component.runtime_.config().segmentation_window_size_ms);
  TEST_ASSERT_EQUAL(94, component.runtime_.config().csi_target_pps);
  TEST_ASSERT_TRUE(component.runtime_.config().csi_traffic_mode == CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_TRUE(component.runtime_.config().traffic_generator_mode == RuntimeTrafficMode::DNS);
  TEST_ASSERT_TRUE(component.runtime_.config().detection_algorithm == DetectionAlgorithm::HIGH_ACCURACY);
  TEST_ASSERT_EQUAL(500, component.runtime_.config().evaluation_interval_ms);
  TEST_ASSERT_EQUAL(4, component.runtime_.config().motion_on_hits);
  TEST_ASSERT_EQUAL(5, component.runtime_.config().motion_off_hits);
  TEST_ASSERT_TRUE(component.runtime_.config().lowpass_enabled);
  TEST_ASSERT_EQUAL_FLOAT(8.5f, component.runtime_.config().lowpass_cutoff);
  TEST_ASSERT_FALSE(component.runtime_.config().hampel_enabled);
  TEST_ASSERT_EQUAL(9, component.runtime_.config().hampel_window);
  TEST_ASSERT_EQUAL_FLOAT(4.5f, component.runtime_.config().hampel_threshold);
  TEST_ASSERT_TRUE(component.sensor_publisher_.has_movement_sensor());
  TEST_ASSERT_TRUE(component.sensor_publisher_.has_motion_binary_sensor());
  TEST_ASSERT_EQUAL_FLOAT(275.0f, component.get_setup_priority());
}

void test_threshold_number_behaviors_cover_parent_and_no_parent_paths(void) {
  ESpectreComponentProbe component;
  component.setup();
  ThresholdNumberProbe number;

  number.setup();
  number.dump_config();
  number.control(1.25f);
  number.republish_state();
  TEST_ASSERT_FALSE(number.has_state());

  number.set_parent(&component);
  number.control(0.625f);
  TEST_ASSERT_EQUAL_FLOAT(0.625f, component.get_threshold());
  component.set_threshold_runtime(0.375f);
  number.republish_state();

  TEST_ASSERT_EQUAL_FLOAT(0.375f, component.get_threshold());
  TEST_ASSERT_TRUE(number.has_state());
  TEST_ASSERT_EQUAL_FLOAT(0.375f, number.get_state());
}

void test_entity_commands_validate_parameters_before_runtime_callbacks(void) {
  ESpectreComponentProbe component;
  component.setup();
  const int threshold_calls = frontend_runtime_shim::state.set_threshold_calls;
  const int motion_calls = frontend_runtime_shim::state.set_motion_hits_calls;
  const int detector_calls = frontend_runtime_shim::state.set_detector_calls;
  TEST_ASSERT_FALSE(component.set_threshold_runtime(-0.1f));
  TEST_ASSERT_FALSE(component.set_threshold_runtime(1.1f));
  TEST_ASSERT_FALSE(component.set_motion_hits_runtime(0U, 1U));
  TEST_ASSERT_FALSE(component.set_motion_hits_runtime(1U, 21U));
  TEST_ASSERT_FALSE(component.set_detection_algorithm_runtime("invalid"));
  TEST_ASSERT_EQUAL(threshold_calls, frontend_runtime_shim::state.set_threshold_calls);
  TEST_ASSERT_EQUAL(motion_calls, frontend_runtime_shim::state.set_motion_hits_calls);
  TEST_ASSERT_EQUAL(detector_calls, frontend_runtime_shim::state.set_detector_calls);
}

void test_motion_hits_number_behaviors_cover_parent_and_no_parent_paths(void) {
  ESpectreComponentProbe component;
  MotionHitsNumberProbe motion_on_number;
  MotionHitsNumberProbe motion_off_number;

  motion_on_number.set_motion_on(true);
  motion_off_number.set_motion_on(false);
  motion_on_number.control(6.0f);
  motion_off_number.republish_state();
  TEST_ASSERT_FALSE(motion_on_number.has_state());
  TEST_ASSERT_FALSE(motion_off_number.has_state());

  component.setup();
  motion_on_number.set_parent(&component);
  motion_off_number.set_parent(&component);
  component.set_motion_on_hits_number(&motion_on_number);
  component.set_motion_off_hits_number(&motion_off_number);

  motion_on_number.control(6.0f);
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_motion_hits_calls);
  TEST_ASSERT_EQUAL_UINT8(6U, frontend_runtime_shim::state.last_motion_on_hits);

  motion_off_number.control(4.0f);
  TEST_ASSERT_EQUAL(2, frontend_runtime_shim::state.set_motion_hits_calls);
  TEST_ASSERT_EQUAL_UINT8(4U, frontend_runtime_shim::state.last_motion_off_hits);

  motion_on_number.republish_state();
  motion_off_number.republish_state();
  TEST_ASSERT_TRUE(motion_on_number.has_state());
  TEST_ASSERT_TRUE(motion_off_number.has_state());
  TEST_ASSERT_EQUAL_FLOAT(component.get_motion_on_hits(), motion_on_number.get_state());
  TEST_ASSERT_EQUAL_FLOAT(component.get_motion_off_hits(), motion_off_number.get_state());
}

void test_sensing_switch_and_recalibrate_button_use_the_command_engine(void) {
  ESpectreComponentProbe component;
  frontend_runtime_shim::state.capabilities.supports_manual_recalibration = true;
  component.setup();

  SensingSwitchProbe sensing_switch;
  sensing_switch.dump_config();
  sensing_switch.write_state(false);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.services_armed);

  sensing_switch.set_parent(&component);
  component.set_sensing_switch(&sensing_switch);
  sensing_switch.write_state(false);
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.services_armed);
  TEST_ASSERT_FALSE(sensing_switch.state);
  sensing_switch.write_state(true);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.services_armed);
  TEST_ASSERT_TRUE(sensing_switch.state);

  RecalibrateButtonProbe recalibrate_button;
  recalibrate_button.dump_config();
  recalibrate_button.press_action();
  TEST_ASSERT_EQUAL(0, frontend_runtime_shim::state.trigger_recalibration_calls);
  recalibrate_button.set_parent(&component);
  recalibrate_button.press_action();
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.trigger_recalibration_calls);
}

void test_sensing_switch_initial_state_matches_runtime(void) {
  for (bool direct_api : {true, false}) {
    for (bool armed : {true, false}) {
      ESpectreComponentProbe component;
      SensingSwitchProbe sensing_switch;
      sensing_switch.set_parent(&component);
      component.set_sensing_switch(&sensing_switch);
      component.set_direct_api(direct_api);
      component.runtime_.set_services_armed(armed);

      component.setup();

      TEST_ASSERT_FALSE(component.is_failed());
      TEST_ASSERT_EQUAL(armed, component.is_sensing_enabled());
      TEST_ASSERT_EQUAL(armed, sensing_switch.state);
    }
  }
}

void test_motion_reset_is_published_when_runtime_is_not_ready(void) {
  for (bool drain_motion_first : {true, false}) {
    ESpectreComponentProbe component;
    esphome::binary_sensor::BinarySensor motion_sensor;
    component.set_motion_binary_sensor(&motion_sensor);
    component.setup();

    RuntimeSnapshot snapshot{};
    snapshot.ready_to_publish = true;
    snapshot.motion_state = MotionState::MOTION;
    component.on_motion_state_changed(snapshot);
    if (drain_motion_first) {
      component.loop();
      TEST_ASSERT_TRUE(motion_sensor.get_state());
    }

    // Disarming sensing and restarting CSI both emit this reset snapshot.
    snapshot.ready_to_publish = false;
    snapshot.motion_state = MotionState::IDLE;
    component.on_motion_state_changed(snapshot);
    component.loop();

    TEST_ASSERT_TRUE(motion_sensor.has_state());
    TEST_ASSERT_FALSE(motion_sensor.get_state());
  }
}

void test_detector_select_switches_and_republishes_runtime_state(void) {
  ESpectreComponentProbe component;
  component.set_detection_algorithm("lightweight");
  component.setup();
  DetectorSelectProbe detector_select;
  ThresholdNumberProbe threshold_number;
  detector_select.set_parent(&component);
  component.set_detector_select(&detector_select);
  component.set_threshold_number(&threshold_number);

  detector_select.control("high_accuracy");
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_detector_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_detector == DetectionAlgorithm::HIGH_ACCURACY);
  detector_select.republish_state();
  TEST_ASSERT_EQUAL_STRING("high_accuracy", detector_select.get_state().c_str());

  RuntimeSnapshot snapshot = component.runtime_.snapshot();
  snapshot.detector_name = "lightweight";
  snapshot.threshold = RUNTIME_SEGMENTATION_THRESHOLD_DEFAULT;
  frontend_runtime_shim::state.last_listener->on_detector_changed(snapshot);
  TEST_ASSERT_EQUAL_STRING("lightweight", detector_select.get_state().c_str());
  TEST_ASSERT_EQUAL_FLOAT(LIGHTWEIGHT_MAX_THRESHOLD, threshold_number.traits.get_max_value());
}

void test_traffic_mode_selects_switch_and_republish_runtime_state(void) {
  ESpectreComponentProbe component;
  component.setup();

  TrafficModeSelectProbe csi_mode_select;
  csi_mode_select.set_parent(&component);
  csi_mode_select.set_csi_traffic_mode(true);
  component.set_csi_traffic_mode_select(&csi_mode_select);

  TrafficModeSelectProbe generator_mode_select;
  generator_mode_select.set_parent(&component);
  generator_mode_select.set_csi_traffic_mode(false);
  component.set_traffic_generator_mode_select(&generator_mode_select);

  csi_mode_select.control("external");
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_csi_traffic_mode_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_csi_traffic_mode == CsiTrafficMode::EXTERNAL);

  generator_mode_select.control("dns_tcp");
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_traffic_generator_mode_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_traffic_generator_mode == RuntimeTrafficMode::DNS_TCP);

  csi_mode_select.republish_state();
  generator_mode_select.republish_state();
  TEST_ASSERT_EQUAL_STRING("external", csi_mode_select.get_state().c_str());
  TEST_ASSERT_EQUAL_STRING("dns_tcp", generator_mode_select.get_state().c_str());
}

void test_motion_threshold_and_calibration_callbacks_publish_expected_state(void) {
  ESpectreComponentProbe component;
  esphome::sensor::Sensor movement_sensor;
  esphome::binary_sensor::BinarySensor binary_sensor;
  esphome::binary_sensor::BinarySensor calibration_active_sensor;
  ThresholdNumberProbe threshold_number;

  threshold_number.set_parent(&component);
  component.set_threshold_number(&threshold_number);
  component.set_calibration_active_sensor(&calibration_active_sensor);
  component.set_movement_sensor(&movement_sensor);
  component.set_motion_binary_sensor(&binary_sensor);
  component.setup();
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.live_telemetry_enabled);
  component.set_threshold_runtime(5.5f);

  component.threshold_republished_ = true;

  RuntimeSnapshot idle_snapshot{};
  idle_snapshot.ready_to_publish = false;
  idle_snapshot.motion_state = MotionState::IDLE;
  frontend_runtime_shim::state.last_listener->on_motion_state_changed(idle_snapshot);
  TEST_ASSERT_FALSE(component.threshold_republished_);
  TEST_ASSERT_FALSE(binary_sensor.has_state());
  frontend_runtime_shim::state.last_listener->on_live_telemetry(7.25f, 5.5f);
  TEST_ASSERT_EQUAL(0, movement_sensor.get_publish_count());

  RuntimeSnapshot motion_snapshot{};
  motion_snapshot.ready_to_publish = true;
  motion_snapshot.motion_state = MotionState::MOTION;
  motion_snapshot.threshold = 5.5f;
  motion_snapshot.movement_metric = 7.25f;
  frontend_runtime_shim::state.last_listener->on_motion_state_changed(motion_snapshot);
  TEST_ASSERT_FALSE(binary_sensor.has_state());
  TEST_ASSERT_EQUAL(0, movement_sensor.get_publish_count());

  frontend_runtime_shim::state.last_listener->on_periodic_update(idle_snapshot, 42);
  TEST_ASSERT_EQUAL(0, movement_sensor.get_publish_count());

  frontend_runtime_shim::state.last_listener->on_periodic_update(motion_snapshot, 42);
  frontend_runtime_shim::state.last_listener->on_periodic_update(motion_snapshot, 42);
  TEST_ASSERT_TRUE(threshold_number.has_state());
  TEST_ASSERT_EQUAL_FLOAT(5.5f, threshold_number.get_state());
  TEST_ASSERT_EQUAL(2, threshold_number.get_publish_count());
  TEST_ASSERT_EQUAL(0, movement_sensor.get_publish_count());

  frontend_runtime_shim::state.last_listener->on_live_telemetry(7.25f, 5.5f);
  TEST_ASSERT_EQUAL(0, movement_sensor.get_publish_count());
  component.loop();
  TEST_ASSERT_EQUAL(1, movement_sensor.get_publish_count());
  TEST_ASSERT_EQUAL_FLOAT(7.25f, movement_sensor.get_state());
  TEST_ASSERT_EQUAL(2, binary_sensor.get_publish_count());
  TEST_ASSERT_TRUE(binary_sensor.get_state());

  RuntimeSnapshot threshold_snapshot = motion_snapshot;
  threshold_snapshot.threshold = 6.75f;
  frontend_runtime_shim::state.last_listener->on_threshold_changed(threshold_snapshot);
  TEST_ASSERT_EQUAL_FLOAT(6.75f, component.runtime_.config().segmentation_threshold);
  TEST_ASSERT_EQUAL_FLOAT(5.5f, threshold_number.get_state());
  component.loop();
  TEST_ASSERT_EQUAL_FLOAT(6.75f, threshold_number.get_state());
  TEST_ASSERT_EQUAL(1, movement_sensor.get_publish_count());

  frontend_runtime_shim::state.last_listener->on_calibration_started(motion_snapshot);
  TEST_ASSERT_TRUE(calibration_active_sensor.get_state());
  TEST_ASSERT_TRUE(component.sensor_publisher_.has_motion_binary_sensor());
  TEST_ASSERT_TRUE(component.sensor_publisher_.has_movement_sensor());
  frontend_runtime_shim::state.last_listener->on_calibration_finished(motion_snapshot, false);
  TEST_ASSERT_FALSE(calibration_active_sensor.get_state());
}

void test_esphome_wifi_bssid_pin_persists_across_setup(void) {
  esphome::ESPPreferences preferences;
  esphome::global_preferences = &preferences;

  {
    ESpectreComponentProbe first;
    first.runtime_.config().device_id = 0x112233445566ULL;
    first.setup();
    TEST_ASSERT_FALSE(first.is_failed());
    const std::string pin = first.direct_bridge_.handle_request_(
        DirectRequest{"pin", "set_wifi_bssid", "{\"bssid\":\"E6:FA:C4:20:19:DE\"}"});
    TEST_ASSERT_TRUE(pin.find("\"accepted\":true") != std::string::npos);
    TEST_ASSERT_TRUE(first.wifi_bssid_pin_.empty());
    TEST_ASSERT_FALSE(first.runtime_.services_armed());
    first.handle_wifi_bssid_association_("E6:FA:C4:20:19:DE");
    TEST_ASSERT_TRUE(first.wifi_bssid_pin_.empty());
    first.wifi_has_ipv4_ = true;
    first.process_wifi_bssid_apply_();
    TEST_ASSERT_EQUAL_STRING("E6:FA:C4:20:19:DE", first.wifi_bssid_pin_.c_str());
    TEST_ASSERT_TRUE(first.runtime_.services_armed());
  }

  ESpectreComponentProbe reboot;
  reboot.runtime_.config().device_id = 0x112233445566ULL;
  reboot.setup();
  TEST_ASSERT_FALSE(reboot.is_failed());
  TEST_ASSERT_EQUAL_STRING("E6:FA:C4:20:19:DE", reboot.wifi_bssid_pin_.c_str());
}

void test_esphome_bssid_mutation_starts_only_after_direct_acknowledgement(void) {
  ESpectreComponentProbe component;
  component.runtime_.config().device_id = 0x112233445566ULL;
  component.setup();
  const int initial_set_config_calls = g_esp_wifi_mock.set_config_call_count;

  auto undelivered = component.direct_bridge_.handle_deferred_request_(
      1U, DirectRequest{"pin-lost", "set_wifi_bssid",
                        "{\"bssid\":\"AA:BB:CC:DD:EE:FF\"}"});
  TEST_ASSERT_TRUE(undelivered.response.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_EQUAL(initial_set_config_calls, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_TRUE(static_cast<bool>(undelivered.response_sent_callback));
  undelivered.response_sent_callback(false);
  TEST_ASSERT_EQUAL(initial_set_config_calls, g_esp_wifi_mock.set_config_call_count);

  auto delivered = component.direct_bridge_.handle_deferred_request_(
      2U, DirectRequest{"pin-ack", "set_wifi_bssid",
                        "{\"bssid\":\"AA:BB:CC:DD:EE:FF\"}"});
  TEST_ASSERT_EQUAL(initial_set_config_calls, g_esp_wifi_mock.set_config_call_count);
  auto concurrent = component.direct_bridge_.handle_deferred_request_(
      3U, DirectRequest{"pin-busy", "set_wifi_bssid",
                        "{\"bssid\":\"11:22:33:44:55:66\"}"});
  TEST_ASSERT_TRUE(concurrent.response.find("\"code\":\"unavailable\"") !=
                   std::string::npos);
  delivered.response_sent_callback(true);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.set_config_call_count > initial_set_config_calls);
}

void test_esphome_bssid_force_controls_reassociation_and_ack_reports_current_bssid(void) {
  ESpectreComponentProbe component;
  component.runtime_.config().device_id = 0x112233445566ULL;
  component.setup();
  component.handle_wifi_bssid_association_("AA:BB:CC:DD:EE:FF");
  const uint8_t current_bssid[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
  std::copy(std::begin(current_bssid), std::end(current_bssid),
            g_esp_wifi_mock.current_ap_info.bssid);
  const int initial_set_config_calls = g_esp_wifi_mock.set_config_call_count;

  auto unchanged = component.direct_bridge_.handle_deferred_request_(
      4U, DirectRequest{"pin-same", "set_wifi_bssid",
                        "{\"bssid\":\"AA:BB:CC:DD:EE:FF\"}"});
  TEST_ASSERT_TRUE(
      unchanged.response.find("\"current_bssid\":\"AA:BB:CC:DD:EE:FF\"") !=
      std::string::npos);
  unchanged.response_sent_callback(true);
  TEST_ASSERT_EQUAL(initial_set_config_calls, g_esp_wifi_mock.set_config_call_count);

  auto forced = component.direct_bridge_.handle_deferred_request_(
      5U, DirectRequest{"pin-force", "set_wifi_bssid",
                        "{\"bssid\":\"AA:BB:CC:DD:EE:FF\",\"force\":true}"});
  TEST_ASSERT_TRUE(
      forced.response.find("\"current_bssid\":\"AA:BB:CC:DD:EE:FF\"") !=
      std::string::npos);
  forced.response_sent_callback(true);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.set_config_call_count > initial_set_config_calls);
}

void test_esphome_wifi_bssid_pin_rolls_back_once_after_enforcement_failure(void) {
  esphome::ESPPreferences preferences;
  esphome::global_preferences = &preferences;
  ESpectreComponentProbe component;
  component.runtime_.config().device_id = 0x112233445566ULL;
  component.setup();
  component.handle_wifi_bssid_association_("E6:FA:C4:20:19:DE");
  const std::string pin = component.direct_bridge_.handle_request_(
      DirectRequest{"pin", "set_wifi_bssid", "{\"bssid\":\"E6:FA:C4:20:19:DE\"}"});
  TEST_ASSERT_TRUE(pin.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_EQUAL_STRING("E6:FA:C4:20:19:DE", component.wifi_bssid_pin_.c_str());

  const int initial_connect_calls = g_esp_wifi_mock.connect_call_count;
  component.handle_wifi_bssid_association_("11:22:33:44:55:66");
  TEST_ASSERT_EQUAL(ESpectreComponent::WifiBssidApplyMode::ENFORCE,
                    component.wifi_bssid_apply_mode_);
  TEST_ASSERT_EQUAL(initial_connect_calls + 1, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_FALSE(component.runtime_.services_armed());
  esphome::advance_mock_millis(35000);
  component.process_wifi_bssid_apply_();
  TEST_ASSERT_EQUAL(ESpectreComponent::WifiBssidApplyMode::NONE,
                    component.wifi_bssid_apply_mode_);
  TEST_ASSERT_TRUE(component.wifi_bssid_recovery_pending_);
  TEST_ASSERT_TRUE(component.wifi_bssid_pin_.empty());
  TEST_ASSERT_EQUAL(initial_connect_calls + 2, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_FALSE(component.runtime_.services_armed());

  complete_wifi_bssid_reconnect(&component, "11:22:33:44:55:66");
  component.process_wifi_bssid_apply_();
  TEST_ASSERT_FALSE(component.wifi_bssid_recovery_pending_);
  TEST_ASSERT_TRUE(component.runtime_.services_armed());
  component.handle_wifi_bssid_association_("11:22:33:44:55:66");
  TEST_ASSERT_EQUAL(ESpectreComponent::WifiBssidApplyMode::NONE,
                    component.wifi_bssid_apply_mode_);
}

void test_esphome_wifi_bssid_pin_rolls_back_when_persistence_fails(void) {
  esphome::ESPPreferences preferences;
  esphome::global_preferences = &preferences;
  {
    ESpectreComponentProbe component;
    component.setup();
    component.handle_wifi_bssid_association_("AA:BB:CC:DD:EE:FF");
    TEST_ASSERT_TRUE(component.begin_wifi_bssid_pin_update_("AA:BB:CC:DD:EE:FF", false, nullptr));
    TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", component.wifi_bssid_pin_.c_str());

    component.handle_wifi_bssid_association_("AA:BB:CC:DD:EE:FF");
    TEST_ASSERT_TRUE(component.begin_wifi_bssid_pin_update_("11:22:33:44:55:66", false, nullptr));
    esphome::g_esphome_preference_save_success = false;
    complete_wifi_bssid_reconnect(&component, "11:22:33:44:55:66");
    TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", component.wifi_bssid_pin_.c_str());
    TEST_ASSERT_TRUE(component.wifi_bssid_recovery_pending_);
    TEST_ASSERT_TRUE(component.wifi_bssid_recovery_journal_pending_);
    TEST_ASSERT_FALSE(component.runtime_.services_armed());

    component.handle_wifi_bssid_association_("AA:BB:CC:DD:EE:FF");
    TEST_ASSERT_FALSE(component.runtime_.services_armed());
    component.wifi_has_ipv4_ = true;
    component.loop();
    TEST_ASSERT_TRUE(component.wifi_bssid_recovery_pending_);
    TEST_ASSERT_TRUE(component.wifi_bssid_recovery_journal_pending_);
    TEST_ASSERT_FALSE(component.runtime_.services_armed());

    esphome::g_esphome_preference_save_success = true;
    component.loop();
    TEST_ASSERT_FALSE(component.wifi_bssid_recovery_pending_);
    TEST_ASSERT_FALSE(component.wifi_bssid_recovery_journal_pending_);
    TEST_ASSERT_TRUE(component.runtime_.services_armed());
  }
  ESpectreComponentProbe reboot;
  reboot.setup();
  TEST_ASSERT_FALSE(reboot.is_failed());
  TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", reboot.wifi_bssid_pin_.c_str());
}

int process(void) {
  UNITY_BEGIN();
  RUN_TEST(test_espectre_component_setup_uses_mock_runtime_snapshot);
  RUN_TEST(test_espectre_component_direct_api_can_be_disabled);
  RUN_TEST(test_espectre_component_setup_marks_failed_when_runtime_setup_fails);
  RUN_TEST(test_espectre_component_loop_and_destructor_forward_to_runtime);
  RUN_TEST(test_espectre_component_loop_does_not_poll_wifi_without_peer_discovery);
  RUN_TEST(test_espectre_component_direct_client_enables_live_telemetry);
  RUN_TEST(test_espectre_component_raw_session_uses_shared_controller_and_recovers);
  RUN_TEST(test_esphome_direct_exposes_common_wifi_and_label_capabilities);
  RUN_TEST(test_wifi_bssid_pin_applies_config_during_station_reassociation);
  RUN_TEST(test_wifi_bssid_pin_reports_disconnect_failure);
  RUN_TEST(test_wifi_bssid_pin_reports_no_transition_when_config_cannot_be_read);
  RUN_TEST(test_wifi_bssid_pin_rejects_invalid_input_before_reading_station_config);
  RUN_TEST(test_wifi_bssid_pin_restarts_original_config_when_update_fails);
  RUN_TEST(test_wifi_bssid_pin_restores_original_config_when_candidate_connect_fails);
  RUN_TEST(test_wifi_bssid_pin_rollback_restores_current_config_when_connect_fails);
  RUN_TEST(test_wifi_bssid_pin_rollback_restarts_current_config_when_update_fails);
  RUN_TEST(test_esphome_wifi_bssid_pin_persists_across_setup);
  RUN_TEST(test_esphome_bssid_mutation_starts_only_after_direct_acknowledgement);
  RUN_TEST(test_esphome_bssid_force_controls_reassociation_and_ack_reports_current_bssid);
  RUN_TEST(test_esphome_wifi_bssid_pin_rolls_back_once_after_enforcement_failure);
  RUN_TEST(test_esphome_wifi_bssid_pin_rolls_back_when_persistence_fails);
  RUN_TEST(test_espectre_component_publishes_cached_csi_diagnostics_on_demand);
  RUN_TEST(test_espectre_component_configuration_setters_update_runtime_config);
  RUN_TEST(test_threshold_number_behaviors_cover_parent_and_no_parent_paths);
  RUN_TEST(test_entity_commands_validate_parameters_before_runtime_callbacks);
  RUN_TEST(test_motion_hits_number_behaviors_cover_parent_and_no_parent_paths);
  RUN_TEST(test_sensing_switch_and_recalibrate_button_use_the_command_engine);
  RUN_TEST(test_sensing_switch_initial_state_matches_runtime);
  RUN_TEST(test_motion_reset_is_published_when_runtime_is_not_ready);
  RUN_TEST(test_detector_select_switches_and_republishes_runtime_state);
  RUN_TEST(test_traffic_mode_selects_switch_and_republish_runtime_state);
  RUN_TEST(test_motion_threshold_and_calibration_callbacks_publish_expected_state);
  return UNITY_END();
}

#if defined(ESP_PLATFORM)
extern "C" void app_main(void) { process(); }
#else
int main(int argc, char **argv) { return process(); }
#endif
