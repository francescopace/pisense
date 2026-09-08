/*
 * ESPectre - Matter Frontend Unit Tests
 *
 * Unit tests for Matter Frontend.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include <vector>

#define private public
#define protected public
#include "matter_frontend.h"
#undef protected
#undef private

#include "direct_http_service_mock.h"
#include "direct_http_protocol.h"
#include "frontend_runtime_shim.h"
#include "matter_bindings_mock.h"
#include "matter_surface.h"

using namespace espectre;
using espectre::direct_http_service_mock::MockDirectHttpService;
using espectre::matter_bindings_mock::MockMatterBindings;

namespace {

RuntimeSnapshot make_ready_snapshot(bool motion) {
  RuntimeSnapshot snapshot{};
  snapshot.ready_to_publish = true;
  snapshot.motion_state = motion ? MotionState::MOTION : MotionState::IDLE;
  snapshot.movement_metric = 2.75f;
  snapshot.threshold = 1.5f;
  snapshot.startup_threshold = 0.42f;
  snapshot.detector_name = "lightweight";
  return snapshot;
}

}  // namespace

void setUp(void) {
  frontend_runtime_shim::reset();
  direct_http_service_mock::reset();
  matter_bindings_mock::reset();
}

void tearDown(void) {}

void test_matter_frontend_setup_registers_runtime_listener(void) {
  frontend_runtime_shim::state.snapshot.threshold = 3.25f;

  MockMatterBindings bindings;
  MatterFrontend frontend(&bindings, 7);
  TEST_ASSERT_TRUE(frontend.setup());
  TEST_ASSERT_TRUE(frontend.is_setup_complete());
  TEST_ASSERT_NOT_NULL(frontend_runtime_shim::state.last_listener);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_listener != &frontend);
  TEST_ASSERT_EQUAL_FLOAT(3.25f, frontend.snapshot().threshold);
}

void test_matter_frontend_setup_fails_without_bindings(void) {
  MatterFrontend frontend(nullptr, 1);
  TEST_ASSERT_FALSE(frontend.setup());
}

void test_matter_frontend_setup_fails_when_runtime_setup_fails(void) {
  frontend_runtime_shim::state.setup_result = false;

  MockMatterBindings bindings;
  MatterFrontend frontend(&bindings, 1);
  TEST_ASSERT_FALSE(frontend.setup());
  TEST_ASSERT_FALSE(frontend.is_setup_complete());
}

void test_matter_frontend_loop_and_shutdown_forward_to_runtime(void) {
  MockMatterBindings bindings;
  MockDirectHttpService direct;
  {
    MatterFrontend frontend(&bindings, 2, &direct);
    TEST_ASSERT_TRUE(frontend.setup());
    TEST_ASSERT_FALSE(frontend_runtime_shim::state.live_telemetry_enabled);
    direct.emit_client_count(1U);
    frontend.loop();
    TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.loop_calls);
    TEST_ASSERT_TRUE(frontend_runtime_shim::state.live_telemetry_enabled);
    direct.emit_client_count(0U);
    frontend.loop();
    TEST_ASSERT_FALSE(frontend_runtime_shim::state.live_telemetry_enabled);
  }
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.shutdown_called);
}

void test_matter_frontend_defers_direct_until_runtime_services_are_armed(void) {
  MockMatterBindings bindings;
  MockDirectHttpService direct;
  MatterFrontend frontend(&bindings, 2, &direct);
  TEST_ASSERT_TRUE(frontend.set_runtime_services_armed(false));
  TEST_ASSERT_TRUE(frontend.setup());
  TEST_ASSERT_FALSE(frontend.runtime_services_armed());
  TEST_ASSERT_EQUAL(0, direct_http_service_mock::state.setup_calls);

  TEST_ASSERT_TRUE(frontend.set_runtime_services_armed(true));
  TEST_ASSERT_TRUE(frontend.runtime_services_armed());
  TEST_ASSERT_EQUAL(1, direct_http_service_mock::state.setup_calls);
  TEST_ASSERT_TRUE(direct_http_service_mock::state.running);

  TEST_ASSERT_TRUE(frontend.set_runtime_services_armed(false));
  TEST_ASSERT_FALSE(frontend.runtime_services_armed());
  TEST_ASSERT_TRUE(direct_http_service_mock::state.shutdown_called);
  TEST_ASSERT_FALSE(direct_http_service_mock::state.running);
}

void test_matter_frontend_defers_runtime_allocation_until_services_are_armed(void) {
  MockMatterBindings bindings;
  MockDirectHttpService direct;
  MatterFrontend frontend(&bindings, 2, &direct);

  TEST_ASSERT_TRUE(frontend.set_runtime_services_armed(false));
  TEST_ASSERT_FALSE(frontend.is_setup_complete());
  TEST_ASSERT_NULL(frontend_runtime_shim::state.last_listener);
  TEST_ASSERT_EQUAL(0, direct_http_service_mock::state.setup_calls);

  TEST_ASSERT_TRUE(frontend.set_runtime_services_armed(true));
  TEST_ASSERT_TRUE(frontend.is_setup_complete());
  TEST_ASSERT_NOT_NULL(frontend_runtime_shim::state.last_listener);
  TEST_ASSERT_EQUAL(1, direct_http_service_mock::state.setup_calls);
  TEST_ASSERT_TRUE(direct_http_service_mock::state.running);
}

void test_matter_frontend_keeps_runtime_disarmed_when_deferred_direct_setup_fails(void) {
  MockMatterBindings bindings;
  MockDirectHttpService direct;
  MatterFrontend frontend(&bindings, 2, &direct);
  TEST_ASSERT_TRUE(frontend.set_runtime_services_armed(false));
  TEST_ASSERT_TRUE(frontend.setup());
  direct_http_service_mock::state.setup_result = false;

  TEST_ASSERT_FALSE(frontend.set_runtime_services_armed(true));
  TEST_ASSERT_FALSE(frontend.runtime_services_armed());
  TEST_ASSERT_EQUAL(2, direct_http_service_mock::state.setup_calls);
}

void test_matter_frontend_motion_and_periodic_callbacks_publish_bindings(void) {
  MockMatterBindings bindings;
  MatterFrontend frontend(&bindings, 3);
  TEST_ASSERT_TRUE(frontend.setup());

  RuntimeSnapshot not_ready{};
  not_ready.ready_to_publish = false;
  frontend.on_motion_state_changed(not_ready);
  TEST_ASSERT_EQUAL(0, matter_bindings_mock::state.motion_events.size());

  RuntimeSnapshot ready_motion = make_ready_snapshot(true);
  frontend.on_motion_state_changed(ready_motion);
  TEST_ASSERT_EQUAL(0, matter_bindings_mock::state.motion_events.size());
  frontend.loop();
  TEST_ASSERT_EQUAL(1, matter_bindings_mock::state.motion_events.size());
  TEST_ASSERT_TRUE(matter_bindings_mock::state.motion_events[0].motion_detected);

  RuntimeSnapshot ready_idle = make_ready_snapshot(false);
  frontend.on_periodic_update(ready_idle, 128);
  TEST_ASSERT_EQUAL(1, matter_bindings_mock::state.motion_events.size());
}

void test_matter_frontend_defers_live_telemetry_serialization_until_after_runtime_loop(void) {
  RuntimeConfig config;
  config.device_id = 0x0123456789abcdefULL;
  MockMatterBindings bindings;
  MockDirectHttpService direct;
  MatterFrontend frontend(&bindings, 3, &direct);
  frontend.set_runtime_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  direct.emit_client_count(1U);
  frontend.loop();

  frontend.on_live_telemetry(7.5f, 2.25f);
  TEST_ASSERT_EQUAL(0, direct_http_service_mock::state.published_events.size());
  frontend.loop();
  TEST_ASSERT_EQUAL(1, direct_http_service_mock::state.published_events.size());
  TEST_ASSERT_TRUE(direct_http_service_mock::state.published_events[0].replaceable_telemetry);
  TEST_ASSERT_TRUE(direct_http_service_mock::state.published_events[0].data_json.find(
                       "\"score\":7.5") != std::string::npos);
  TEST_ASSERT_TRUE(direct_http_service_mock::state.published_events[0].data_json.find(
                       "\"threshold\"") == std::string::npos);
}

void test_matter_frontend_threshold_and_calibration_callbacks_update_runtime_snapshot(void) {
  MockMatterBindings bindings;
  MatterFrontend frontend(&bindings, 4);
  TEST_ASSERT_TRUE(frontend.setup());

  RuntimeSnapshot threshold_snapshot = make_ready_snapshot(false);
  threshold_snapshot.threshold = 4.25f;
  frontend_runtime_shim::state.last_listener->on_threshold_changed(threshold_snapshot);
  TEST_ASSERT_EQUAL_FLOAT(4.25f, frontend.snapshot().threshold);

  RuntimeSnapshot calibrating = make_ready_snapshot(false);
  calibrating.calibrating = true;
  frontend_runtime_shim::state.last_listener->on_calibration_started(calibrating);
  TEST_ASSERT_TRUE(frontend.snapshot().calibrating);

  RuntimeSnapshot finished = make_ready_snapshot(false);
  finished.calibrating = false;
  frontend_runtime_shim::state.last_listener->on_calibration_finished(finished, false);
  TEST_ASSERT_FALSE(frontend.snapshot().calibrating);
}

void test_matter_frontend_publishes_sensing_on_both_readiness_edges(void) {
  MockMatterBindings bindings;
  MockDirectHttpService direct;
  MatterFrontend frontend(&bindings, 8, &direct);
  TEST_ASSERT_TRUE(frontend.setup());
  direct.emit_client_count(1U);

  for (bool ready : {true, false}) {
    RuntimeSnapshot snapshot = make_ready_snapshot(false);
    snapshot.ready_to_publish = ready;
    frontend_runtime_shim::state.last_listener->on_motion_state_changed(snapshot);
    direct_http_service_mock::state.published_events.clear();
    frontend.on_sensing_readiness_changed(snapshot);
    TEST_ASSERT_EQUAL(1, direct_http_service_mock::state.published_events.size());
    const auto &event = direct_http_service_mock::state.published_events[0];
    TEST_ASSERT_EQUAL_STRING("sensing", event.event_name.c_str());
    TEST_ASSERT_TRUE(event.data_json.find(ready ? "\"ready\":true" : "\"ready\":false") !=
                     std::string::npos);
  }
}

void test_matter_frontend_runtime_fault_is_reported(void) {
  MockMatterBindings bindings;
  MockDirectHttpService direct;
  MatterFrontend frontend(&bindings, 8, &direct);
  RuntimeConfig config;
  config.device_id = 0x0123456789abcdefULL;
  frontend.set_runtime_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  direct.emit_client_count(1U);

  frontend.on_runtime_fault("wifi disconnected");
  TEST_ASSERT_EQUAL(1, matter_bindings_mock::state.faults.size());
  TEST_ASSERT_EQUAL_STRING("wifi disconnected", matter_bindings_mock::state.faults[0].c_str());
  TEST_ASSERT_EQUAL(1, direct_http_service_mock::state.published_events.size());
  const auto &event = direct_http_service_mock::state.published_events[0];
  TEST_ASSERT_EQUAL_STRING("fault", event.event_name.c_str());
  TEST_ASSERT_TRUE(event.data_json.find("\"protocol_version\"") == std::string::npos);
  TEST_ASSERT_TRUE(event.data_json.find("\"device_id\"") == std::string::npos);
  TEST_ASSERT_TRUE(event.data_json.find("\"message\":\"wifi disconnected\"") !=
                   std::string::npos);
}

void test_matter_frontend_exposes_runtime_tuning_over_direct_http(void) {
  RuntimeConfig config;
  config.device_id = 0x0123456789abcdefULL;
  config.runtime_detector_selection_enabled = true;
  frontend_runtime_shim::state.diagnostics.free_memory_bytes = 4096U;
  frontend_runtime_shim::state.diagnostics.minimum_free_memory_bytes = 2048U;
  frontend_runtime_shim::state.diagnostics.largest_free_memory_block_bytes = 1024U;
  frontend_runtime_shim::state.diagnostics.cpu_frequency_mhz = 160U;
  frontend_runtime_shim::state.diagnostics.performance_window_ready = true;
  frontend_runtime_shim::state.diagnostics.runtime_load_percent = 12.5f;
  frontend_runtime_shim::state.diagnostics.detection_timing_supported = true;
  frontend_runtime_shim::state.diagnostics.detection_samples = 4U;

  MockMatterBindings bindings;
  MockDirectHttpService direct;
  MatterFrontend frontend(&bindings, 9, &direct);
  frontend.set_runtime_config(config);

  TEST_ASSERT_TRUE(frontend.setup());
  TEST_ASSERT_EQUAL(1, direct_http_service_mock::state.setup_calls);
  TEST_ASSERT_EQUAL(ESPECTRE_DIRECT_HTTP_PORT, direct_http_service_mock::state.last_config.port);

  const std::string info = direct.emit_request(
      DirectRequest{"", "device", "{}", "/espectre/v1/device", "GET"});
  TEST_ASSERT_TRUE(info.find("\"frontend\":\"matter\"") != std::string::npos);
  TEST_ASSERT_TRUE(info.find("\"device_id\":\"0123456789abcdef\"") != std::string::npos);

  const std::string sensing = direct.emit_request(
      DirectRequest{"", "sensing", "{}", "/espectre/v1/sensing", "GET"});
  TEST_ASSERT_TRUE(sensing.find("\"enabled\":true") != std::string::npos);

  const std::string diagnostics = direct.emit_request(
      DirectRequest{"", "read_diagnostics", "{}", "/espectre/v1/diagnostics", "GET"});
  TEST_ASSERT_TRUE(diagnostics.find("\"traffic_packets_total\":0") != std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"traffic_tx_pps\":0") != std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"csi_callback_pps\":0") != std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"free_memory_kb\":4") != std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"runtime_load_percent\":12.5") != std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"detection_samples\":4") != std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"direct_event_clients\"") == std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"direct_rejected_connections\"") == std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"direct_dropped_telemetry_events\"") == std::string::npos);
  TEST_ASSERT_TRUE(diagnostics.find("\"direct_http\":{") != std::string::npos);

  frontend_runtime_shim::state.diagnostics.traffic_packets_total = 10U;
  frontend_runtime_shim::state.diagnostics.csi_callbacks_total = 8U;
  frontend_runtime_shim::state.diagnostics_sample.traffic_tx_pps = 10.0f;
  frontend_runtime_shim::state.diagnostics_sample.csi_callback_pps = 8.0f;
  frontend.on_periodic_update(make_ready_snapshot(false), 8U);
  const std::string sampled_diagnostics = direct.emit_request(
      DirectRequest{"", "read_diagnostics", "{}", "/espectre/v1/diagnostics", "GET"});
  TEST_ASSERT_TRUE(sampled_diagnostics.find("\"traffic_tx_pps\":0") == std::string::npos);
  TEST_ASSERT_TRUE(sampled_diagnostics.find("\"csi_callback_pps\":0") == std::string::npos);

  const std::string detector = direct.emit_request(
      DirectRequest{"detector-1", "update_sensing", "{\"detector\":\"high_accuracy\"}"});
  TEST_ASSERT_TRUE(detector.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_detector_calls);
  TEST_ASSERT_EQUAL(static_cast<int>(DetectionAlgorithm::HIGH_ACCURACY),
                    static_cast<int>(frontend_runtime_shim::state.last_detector));
}

void test_matter_frontend_raw_session_uses_shared_controller_and_recovers(void) {
  frontend_runtime_shim::state.capabilities.supports_raw_csi = true;
  RuntimeConfig config;
  config.device_id = 0x0123456789abcdefULL;
  MockMatterBindings bindings;
  MockDirectHttpService direct;
  MatterFrontend frontend(&bindings, 10, &direct);
  frontend.set_runtime_config(config);
  TEST_ASSERT_TRUE(frontend.setup());

  std::string raw_message;
  TEST_ASSERT_TRUE(direct.emit_raw_session_request(&raw_message));
  TEST_ASSERT_TRUE(direct_http_service_mock::state.raw_session_active);
  TEST_ASSERT_EQUAL(RuntimeOperationState::RAW_COLLECTION,
                    frontend.runtime_.operation_state());

  const std::string busy = direct.emit_request(
      DirectRequest{"raw-busy", "update_sensing", "{\"enabled\":false}"});
  TEST_ASSERT_TRUE(busy.find("\"code\":\"busy_raw_collection\"") != std::string::npos);
  TEST_ASSERT_TRUE(frontend.runtime_.services_armed());

  TEST_ASSERT_TRUE(direct.stop_raw_session(RawCsiStopReason::WIFI_LOST));
  TEST_ASSERT_FALSE(direct_http_service_mock::state.raw_session_active);
  TEST_ASSERT_EQUAL(RuntimeOperationState::SENSING,
                    frontend.runtime_.operation_state());
}

void test_matter_direct_exposes_common_wifi_and_node_label_capabilities(void) {
  RuntimeConfig config;
  config.device_id = 0x0123456789abcdefULL;
  MockMatterBindings bindings;
  MockDirectHttpService direct;
  MatterFrontend frontend(&bindings, 11, &direct);
  frontend.set_runtime_config(config);
  std::vector<std::string> bssid_updates;
  std::vector<bool> bssid_forces;
  frontend.set_wifi_bssid_pin_setter(
      [&bssid_updates, &bssid_forces](const std::string &bssid, bool force,
                                     std::string *message) {
        bssid_updates.push_back(bssid);
        bssid_forces.push_back(force);
        if (message != nullptr) *message = "Matter BSSID transaction started";
        return true;
      });
  TEST_ASSERT_TRUE(frontend.setup());

  const std::string default_info = direct.emit_request(
      DirectRequest{"", "device", "{}", "/espectre/v1/device", "GET"});
  TEST_ASSERT_TRUE(default_info.find("\"name\":\"ESPectre ESP32 abcdef\"") != std::string::npos);
  TEST_ASSERT_TRUE(default_info.find("\"label\":\"\"") != std::string::npos);
  TEST_ASSERT_EQUAL_STRING("ESPectre ESP32 abcdef", frontend.peer_discovery_.local_candidate_.name.c_str());
  TEST_ASSERT_EQUAL_STRING("ESPectre 0123456789abcdef",
                           frontend.peer_discovery_.local_candidate_.instance.c_str());

  const std::string capabilities = direct.emit_request(
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

  const std::string scan = direct.emit_request(
      DirectRequest{"scan", "scan_wifi", "{}"});
  auto pin_request = direct.emit_deferred_request(
      77U, DirectRequest{"pin", "set_wifi_bssid",
                         "{\"bssid\":\"E6:FA:C4:20:19:DE\",\"force\":true}"});
  TEST_ASSERT_TRUE(bssid_updates.empty());
  TEST_ASSERT_TRUE(pin_request.response.find("\"accepted\":true") != std::string::npos);
  pin_request.response_sent_callback(true);
  const std::string unpin = direct.emit_request(
      DirectRequest{"unpin", "clear_wifi_bssid", "{}"});
  const std::string credential_reset = direct.emit_request(
      DirectRequest{"reset", "clear_wifi_credentials", "{}"});
  TEST_ASSERT_TRUE(scan.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_TRUE(unpin.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_EQUAL(2, static_cast<int>(bssid_updates.size()));
  TEST_ASSERT_EQUAL_STRING("E6:FA:C4:20:19:DE", bssid_updates[0].c_str());
  TEST_ASSERT_TRUE(bssid_updates[1].empty());
  TEST_ASSERT_TRUE(bssid_forces[0]);
  TEST_ASSERT_FALSE(bssid_forces[1]);
  TEST_ASSERT_TRUE(credential_reset.find("\"code\":\"unsupported\"") != std::string::npos);

  const std::string label = direct.emit_request(
      DirectRequest{"label", "update_device", "{\"label\":\"Kitchen Matter\"}"});
  TEST_ASSERT_TRUE(label.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_EQUAL_STRING("Kitchen Matter", matter_bindings_mock::state.node_label.c_str());
  const std::string info = direct.emit_request(
      DirectRequest{"", "device", "{}", "/espectre/v1/device", "GET"});
  const std::string visible_config = direct.emit_request(
      DirectRequest{"", "sensing", "{}", "/espectre/v1/sensing", "GET"});
  TEST_ASSERT_TRUE(info.find("\"label\":\"Kitchen Matter\"") != std::string::npos);
  TEST_ASSERT_TRUE(visible_config.find("Kitchen Matter") == std::string::npos);
}

void test_matter_frontend_quiesces_sensing_while_wifi_is_reconfigured(void) {
  MockMatterBindings bindings;
  MockDirectHttpService direct;
  MatterFrontend frontend(&bindings, 12, &direct);
  TEST_ASSERT_TRUE(frontend.setup());
  TEST_ASSERT_TRUE(frontend.runtime_services_armed());
  TEST_ASSERT_TRUE(direct_http_service_mock::state.running);
  const int initial_set_services_armed_calls =
      frontend_runtime_shim::state.set_services_armed_calls;

  frontend.prepare_for_wifi_reconfigure();
  TEST_ASSERT_FALSE(frontend.runtime_services_armed());
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.services_armed);
  TEST_ASSERT_EQUAL(initial_set_services_armed_calls + 1,
                    frontend_runtime_shim::state.set_services_armed_calls);
  TEST_ASSERT_TRUE(direct_http_service_mock::state.running);

  frontend.resume_after_wifi_reconfigure();
  TEST_ASSERT_FALSE(frontend.runtime_services_armed());
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.services_armed);
  frontend.loop();
  TEST_ASSERT_TRUE(frontend.runtime_services_armed());
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.services_armed);
  TEST_ASSERT_EQUAL(initial_set_services_armed_calls + 2,
                    frontend_runtime_shim::state.set_services_armed_calls);

  TEST_ASSERT_TRUE(frontend.set_runtime_services_armed(false));
  frontend.prepare_for_wifi_reconfigure();
  frontend.resume_after_wifi_reconfigure();
  frontend.loop();
  TEST_ASSERT_FALSE(frontend.runtime_services_armed());
}

void test_matter_surface_mapping_helpers(void) {
  RuntimeSnapshot snapshot = make_ready_snapshot(true);

  TEST_ASSERT_TRUE(snapshot_to_motion_detected(snapshot));
}

int main(int argc, char **argv) {
  (void) argc;
  (void) argv;
  UNITY_BEGIN();
  RUN_TEST(test_matter_frontend_setup_registers_runtime_listener);
  RUN_TEST(test_matter_frontend_setup_fails_without_bindings);
  RUN_TEST(test_matter_frontend_setup_fails_when_runtime_setup_fails);
  RUN_TEST(test_matter_frontend_loop_and_shutdown_forward_to_runtime);
  RUN_TEST(test_matter_frontend_defers_direct_until_runtime_services_are_armed);
  RUN_TEST(test_matter_frontend_defers_runtime_allocation_until_services_are_armed);
  RUN_TEST(test_matter_frontend_keeps_runtime_disarmed_when_deferred_direct_setup_fails);
  RUN_TEST(test_matter_frontend_motion_and_periodic_callbacks_publish_bindings);
  RUN_TEST(test_matter_frontend_defers_live_telemetry_serialization_until_after_runtime_loop);
  RUN_TEST(test_matter_frontend_threshold_and_calibration_callbacks_update_runtime_snapshot);
  RUN_TEST(test_matter_frontend_runtime_fault_is_reported);
  RUN_TEST(test_matter_frontend_publishes_sensing_on_both_readiness_edges);
  RUN_TEST(test_matter_frontend_exposes_runtime_tuning_over_direct_http);
  RUN_TEST(test_matter_frontend_raw_session_uses_shared_controller_and_recovers);
  RUN_TEST(test_matter_direct_exposes_common_wifi_and_node_label_capabilities);
  RUN_TEST(test_matter_frontend_quiesces_sensing_while_wifi_is_reconfigured);
  RUN_TEST(test_matter_surface_mapping_helpers);
  return UNITY_END();
}
