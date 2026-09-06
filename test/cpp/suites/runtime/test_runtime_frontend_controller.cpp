/*
 * ESPectre - Runtime Frontend Controller Tests
 *
 * Covers the lightweight controller that wraps EspIdfRuntime for the host-side
 * frontend tests.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include "frontend_runtime_shim.h"
#include "runtime_config_utils.h"
#include "runtime_frontend_controller.h"
#include "frontend_bootstrap_helpers.h"
#include "device_config_store.h"
#include "nvs.h"

#include <string>

using namespace espectre;

namespace {

class DummyRuntimeListener : public IRuntimeListener {
 public:
  void on_threshold_changed(const RuntimeSnapshot &snapshot) override {
    threshold_count++;
    last_threshold = snapshot.threshold;
    if (controller != nullptr) {
      cached_threshold_during_callback = controller->snapshot().threshold;
      configured_threshold_during_callback = controller->config().segmentation_threshold;
      if (shutdown_on_threshold) {
        controller->shutdown();
      }
    }
  }

  void on_live_telemetry(float movement, float threshold) override {
    telemetry_count++;
    last_movement = movement;
    last_threshold = threshold;
  }

  void on_runtime_fault(const char *message) override {
    fault_count++;
    last_fault = message != nullptr ? message : "";
  }

  int fault_count{0};
  int threshold_count{0};
  int telemetry_count{0};
  float last_threshold{0.0f};
  float last_movement{0.0f};
  float cached_threshold_during_callback{0.0f};
  float configured_threshold_during_callback{0.0f};
  RuntimeFrontendController *controller{nullptr};
  bool shutdown_on_threshold{false};
  std::string last_fault;
};

}  // namespace

void setUp(void) {
  frontend_runtime_shim::reset();
  nvs_mock_reset();
  esp_event_mock_reset();
  esp_netif_mock_reset();
  esp_wifi_mock_reset();
}

void tearDown(void) {}

void test_frontend_bootstrap_loads_defaults_and_preserves_runtime_identity(void) {
  FrontendDeviceConfigDefaults defaults;
  defaults.runtime_device_id = 0x1234U;
  defaults.device_label = "Default device";
  defaults.mqtt_scheme = "mqtt";
  defaults.mqtt_host = "broker.local";
  defaults.mqtt_port = 1883U;
  defaults.mqtt_username = "user";
  defaults.mqtt_password = "test-password";
  defaults.topic_prefix = "test/devices";
  auto config = load_frontend_device_config(defaults, "test", "loaded", nullptr);
  TEST_ASSERT_TRUE(defaults.runtime_device_id == config.device_id);
  TEST_ASSERT_EQUAL_STRING(defaults.device_label, config.device_label.c_str());
  TEST_ASSERT_EQUAL_STRING(defaults.mqtt_host, config.mqtt_host.c_str());
  TEST_ASSERT_EQUAL_STRING(defaults.topic_prefix, config.topic_prefix.c_str());
  config.device_label = "Stored device";
  config.device_id = 0x5678U;
  TEST_ASSERT_EQUAL(ESP_OK, save_stored_device_config(config));
  const auto loaded = load_frontend_device_config(defaults, "test", "loaded", "load failed");
  TEST_ASSERT_EQUAL_STRING("Stored device", loaded.device_label.c_str());
  TEST_ASSERT_EQUAL_STRING("test-password", loaded.mqtt_password.c_str());
  TEST_ASSERT_TRUE(defaults.runtime_device_id == loaded.device_id);

  nvs_mock_set_open_result(ESP_FAIL);
  const auto fallback = load_frontend_device_config(defaults, "test", nullptr, nullptr);
  TEST_ASSERT_EQUAL_STRING(defaults.device_label, fallback.device_label.c_str());
  TEST_ASSERT_TRUE(defaults.runtime_device_id == fallback.device_id);
}

void test_frontend_bootstrap_accepts_optional_defaults_and_validates_wifi_options(void) {
  FrontendDeviceConfigDefaults defaults;
  defaults.device_label = defaults.mqtt_scheme = defaults.mqtt_host = nullptr;
  defaults.mqtt_username = defaults.mqtt_password = defaults.topic_prefix = nullptr;
  const auto config = load_frontend_device_config(defaults, "test", nullptr, nullptr);
  TEST_ASSERT_TRUE(derive_runtime_device_id() == config.device_id);
  TEST_ASSERT_EQUAL_STRING(ESPECTRE_DEFAULT_DEVICE_LABEL, config.device_label.c_str());
  TEST_ASSERT_TRUE(config.mqtt_host.empty());
  TEST_ASSERT_EQUAL_STRING(ESPECTRE_TOPIC_PREFIX, config.topic_prefix.c_str());

  StandaloneWifiService wifi;
  WifiProvisioningService provisioning(&wifi);
  FrontendWifiStationOptions options;
  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, setup_frontend_wifi_station(nullptr, &wifi, options, "test", nullptr));
  options.start_manager = true;
  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, setup_frontend_wifi_station(&provisioning, nullptr, options, "test", nullptr));
  options.configured_channel = 255;
  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, setup_frontend_wifi_station(&provisioning, &wifi, options, "test", nullptr));
  options.configured_channel = 0;
  options.band_policy = static_cast<WifiBandPolicy>(255);
  TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, setup_frontend_wifi_station(&provisioning, &wifi, options, "test", nullptr));
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.start_call_count);
}

void test_frontend_bootstrap_sets_up_wifi_and_propagates_start_failure(void) {
  StandaloneWifiService wifi;
  WifiProvisioningService provisioning(&wifi);
  FrontendWifiStationOptions options;
  options.ssid = "TestSSID";
  options.password = "test-password";
  options.configured_channel = 6;
  options.start_manager = true;
  g_esp_wifi_mock.start_result_count = 1;
  g_esp_wifi_mock.start_results[0] = ESP_FAIL;
  TEST_ASSERT_EQUAL(ESP_FAIL, setup_frontend_wifi_station(&provisioning, &wifi, options, "test", nullptr));
  TEST_ASSERT_EQUAL_STRING(options.ssid, provisioning.config().ssid.c_str());
  TEST_ASSERT_EQUAL_UINT8(6, provisioning.config().channel);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.start_call_count);
  wifi.shutdown();
}

void test_runtime_frontend_controller_preserves_pre_setup_config_and_snapshot(void) {
  RuntimeFrontendController controller;
  RuntimeConfig config;
  config.segmentation_threshold = 0.85f;

  controller.set_config(config);

  TEST_ASSERT_EQUAL_FLOAT(0.85f, controller.config().segmentation_threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.85f, controller.snapshot().threshold);

  frontend_runtime_shim::state.snapshot.threshold = 0.7f;
  DummyRuntimeListener listener;
  TEST_ASSERT_TRUE(controller.setup(&listener));

  RuntimeConfig updated = controller.config();
  updated.segmentation_threshold = 0.5f;
  controller.set_config(updated);
  TEST_ASSERT_EQUAL_FLOAT(0.85f, controller.config().segmentation_threshold);
}

void test_runtime_frontend_controller_rejects_invalid_config_before_backend_setup(void) {
  RuntimeFrontendController controller;
  RuntimeConfig config;
  config.evaluation_interval_ms = 0U;
  controller.set_config(config);
  DummyRuntimeListener listener;

  TEST_ASSERT_FALSE(controller.setup(&listener));
  TEST_ASSERT_FALSE(controller.is_setup_complete());
  TEST_ASSERT_NULL(frontend_runtime_shim::state.last_instance);
  TEST_ASSERT_EQUAL(1, listener.fault_count);
  TEST_ASSERT_EQUAL_STRING("invalid evaluation interval", listener.last_fault.c_str());
}

void test_runtime_frontend_controller_keeps_staged_mutations_out_of_live_validation(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;
  frontend_runtime_shim::state.capabilities.supports_runtime_threshold_updates = true;
  TEST_ASSERT_TRUE(controller.setup(&listener));

  controller.config().runtime_profile = static_cast<RuntimeProfile>(0x7f);
  TEST_ASSERT_TRUE(controller.set_threshold_runtime(0.55f));
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_threshold_calls);
  TEST_ASSERT_EQUAL_FLOAT(0.55f, controller.snapshot().threshold);

  controller.shutdown();
  TEST_ASSERT_FALSE(controller.setup(&listener));
  TEST_ASSERT_EQUAL(1, listener.fault_count);
}

void test_runtime_frontend_controller_preserves_staged_fields_across_live_callbacks(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;
  TEST_ASSERT_TRUE(controller.setup(&listener));

  controller.config().detection_algorithm = DetectionAlgorithm::HIGH_ACCURACY;
  controller.config().segmentation_threshold = 0.77f;

  RuntimeSnapshot current = controller.snapshot();
  current.detector_name = detection_algorithm_name(DetectionAlgorithm::LIGHTWEIGHT);
  current.threshold = 0.61f;
  frontend_runtime_shim::state.last_listener->on_detector_changed(current);
  frontend_runtime_shim::state.last_listener->on_threshold_changed(current);
  frontend_runtime_shim::state.last_listener->on_calibration_finished(current, true);
  frontend_runtime_shim::state.last_listener->on_live_telemetry(0.4f, 0.62f);

  TEST_ASSERT_TRUE(controller.config().detection_algorithm ==
                   DetectionAlgorithm::HIGH_ACCURACY);
  TEST_ASSERT_EQUAL_FLOAT(0.77f, controller.config().segmentation_threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.62f, controller.snapshot().threshold);

  controller.shutdown();
  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_TRUE(controller.config().detection_algorithm ==
                   DetectionAlgorithm::HIGH_ACCURACY);
  TEST_ASSERT_EQUAL_FLOAT(0.77f, controller.config().segmentation_threshold);
}

void test_runtime_frontend_controller_preserves_staged_fields_across_live_setters(void) {
  RuntimeFrontendController controller;
  RuntimeConfig initial;
  initial.runtime_detector_selection_enabled = true;
  controller.set_config(initial);
  DummyRuntimeListener listener;
  TEST_ASSERT_TRUE(controller.setup(&listener));

  controller.config().detection_algorithm = DetectionAlgorithm::HIGH_ACCURACY;
  controller.config().segmentation_threshold = 0.77f;
  controller.config().motion_on_hits = 8U;
  controller.config().motion_off_hits = 6U;
  controller.config().csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  controller.config().traffic_generator_mode = RuntimeTrafficMode::DNS_TCP;

  TEST_ASSERT_TRUE(controller.set_threshold_runtime(0.55f));
  TEST_ASSERT_TRUE(controller.set_motion_hits_runtime(5U, 2U));
  TEST_ASSERT_TRUE(controller.set_csi_traffic_mode_runtime(CsiTrafficMode::INTERNAL));
  TEST_ASSERT_TRUE(controller.set_traffic_generator_mode_runtime(RuntimeTrafficMode::DNS));
  TEST_ASSERT_TRUE(controller.set_detection_algorithm_runtime(DetectionAlgorithm::LIGHTWEIGHT));

  TEST_ASSERT_TRUE(controller.config().detection_algorithm == DetectionAlgorithm::HIGH_ACCURACY);
  TEST_ASSERT_EQUAL_FLOAT(0.77f, controller.config().segmentation_threshold);
  TEST_ASSERT_EQUAL_UINT8(8U, controller.config().motion_on_hits);
  TEST_ASSERT_EQUAL_UINT8(6U, controller.config().motion_off_hits);
  TEST_ASSERT_TRUE(controller.config().csi_traffic_mode == CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_TRUE(controller.config().traffic_generator_mode == RuntimeTrafficMode::DNS_TCP);

  controller.shutdown();
  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_TRUE(controller.config().detection_algorithm == DetectionAlgorithm::HIGH_ACCURACY);
  TEST_ASSERT_EQUAL_FLOAT(0.77f, controller.config().segmentation_threshold);
  TEST_ASSERT_EQUAL_UINT8(8U, controller.config().motion_on_hits);
  TEST_ASSERT_EQUAL_UINT8(6U, controller.config().motion_off_hits);
  TEST_ASSERT_TRUE(controller.config().csi_traffic_mode == CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_TRUE(controller.config().traffic_generator_mode == RuntimeTrafficMode::DNS_TCP);
}

void test_runtime_frontend_controller_setup_propagates_state_and_handles_failure(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;
  frontend_runtime_shim::state.snapshot.threshold = 3.0f;
  frontend_runtime_shim::state.capabilities.supports_manual_recalibration = false;

  controller.set_services_armed(false);
  controller.set_live_telemetry_enabled(false);

  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_TRUE(controller.is_setup_complete());
  TEST_ASSERT_NOT_NULL(frontend_runtime_shim::state.last_listener);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_listener != &listener);
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.services_armed);
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.live_telemetry_enabled);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, controller.snapshot().threshold);
  TEST_ASSERT_FALSE(controller.capabilities().supports_manual_recalibration);

  TEST_ASSERT_TRUE(controller.setup(&listener));

  RuntimeFrontendController failing;
  frontend_runtime_shim::reset();
  frontend_runtime_shim::state.setup_result = false;
  TEST_ASSERT_FALSE(failing.setup(&listener));
  TEST_ASSERT_FALSE(failing.is_setup_complete());
}

void test_runtime_frontend_controller_adopts_backend_effective_config(void) {
  RuntimeFrontendController controller;
  RuntimeConfig staged;
  staged.csi_traffic_mode = CsiTrafficMode::INTERNAL;
  staged.traffic_generator_mode = RuntimeTrafficMode::PING;
  controller.set_config(staged);

  frontend_runtime_shim::state.override_config_on_setup = true;
  frontend_runtime_shim::state.setup_config = staged;
  frontend_runtime_shim::state.setup_config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  frontend_runtime_shim::state.setup_config.traffic_generator_mode = RuntimeTrafficMode::DNS_TCP;
  frontend_runtime_shim::state.setup_config.motion_on_hits = 8U;
  frontend_runtime_shim::state.setup_config.motion_off_hits = 6U;

  DummyRuntimeListener listener;
  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_TRUE(controller.config().csi_traffic_mode == CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_TRUE(controller.config().traffic_generator_mode == RuntimeTrafficMode::DNS_TCP);
  TEST_ASSERT_EQUAL_UINT8(8U, controller.config().motion_on_hits);
  TEST_ASSERT_EQUAL_UINT8(6U, controller.config().motion_off_hits);
}

void test_runtime_frontend_controller_loop_shutdown_and_runtime_toggles_forward(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;

  controller.loop();
  controller.set_services_armed(false);
  controller.set_live_telemetry_enabled(false);

  TEST_ASSERT_TRUE(controller.setup(&listener));
  controller.loop();
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.loop_calls);

  controller.set_services_armed(true);
  controller.set_live_telemetry_enabled(true);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.services_armed);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.live_telemetry_enabled);

  controller.shutdown();
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.shutdown_called);
  TEST_ASSERT_FALSE(controller.is_setup_complete());
}

void test_runtime_frontend_controller_reads_diagnostics_from_backend(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;
  TEST_ASSERT_TRUE(controller.setup(&listener));

  frontend_runtime_shim::state.diagnostics.wifi_channel = 10U;
  frontend_runtime_shim::state.diagnostics.csi_callbacks_total = 123U;
  const RuntimeDiagnosticsSnapshot diagnostics = controller.diagnostics();

  TEST_ASSERT_EQUAL(10U, diagnostics.wifi_channel);
  TEST_ASSERT_EQUAL(123U, diagnostics.csi_callbacks_total);
}

void test_runtime_frontend_controller_exposes_runtime_owned_diagnostics_sample(void) {
  RuntimeFrontendController controller;
  TEST_ASSERT_NULL(controller.diagnostics_sample());

  DummyRuntimeListener listener;
  TEST_ASSERT_TRUE(controller.setup(&listener));
  frontend_runtime_shim::state.diagnostics_sample.csi_admitted_pps = 84.0f;

  const RuntimeDiagnosticsSample *sample = controller.diagnostics_sample();
  TEST_ASSERT_NOT_NULL(sample);
  TEST_ASSERT_TRUE(sample == &frontend_runtime_shim::state.diagnostics_sample);
  TEST_ASSERT_EQUAL_FLOAT(84.0f, sample->csi_admitted_pps);
}

void test_runtime_frontend_controller_threshold_runtime_updates_config_and_snapshot(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;

  TEST_ASSERT_FALSE(controller.set_threshold_runtime(-0.5f));
  TEST_ASSERT_TRUE(controller.set_threshold_runtime(0.75f));
  TEST_ASSERT_EQUAL_FLOAT(0.75f, controller.snapshot().threshold);

  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_TRUE(controller.set_threshold_runtime(0.5f));
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_threshold_calls);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, frontend_runtime_shim::state.last_threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, controller.snapshot().threshold);
}

void test_runtime_frontend_controller_threshold_requires_capability(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;
  frontend_runtime_shim::state.capabilities.supports_runtime_threshold_updates = false;

  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_FALSE(controller.set_threshold_runtime(0.5f));
  TEST_ASSERT_EQUAL(0, frontend_runtime_shim::state.set_threshold_calls);
  TEST_ASSERT_EQUAL_FLOAT(RUNTIME_SEGMENTATION_THRESHOLD_DEFAULT,
                          controller.config().segmentation_threshold);
}

void test_runtime_frontend_controller_motion_hits_runtime_updates_config(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;

  TEST_ASSERT_FALSE(controller.set_motion_hits_runtime(0U, 3U));
  TEST_ASSERT_TRUE(controller.set_motion_hits_runtime(6U, 4U));
  TEST_ASSERT_EQUAL_UINT8(6U, controller.config().motion_on_hits);
  TEST_ASSERT_EQUAL_UINT8(4U, controller.config().motion_off_hits);

  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_TRUE(controller.set_motion_hits_runtime(5U, 2U));
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_motion_hits_calls);
  TEST_ASSERT_EQUAL_UINT8(5U, frontend_runtime_shim::state.last_motion_on_hits);
  TEST_ASSERT_EQUAL_UINT8(2U, frontend_runtime_shim::state.last_motion_off_hits);
}

void test_runtime_frontend_controller_traffic_runtime_updates_config(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;

  TEST_ASSERT_TRUE(controller.set_csi_traffic_mode_runtime(CsiTrafficMode::EXTERNAL));
  TEST_ASSERT_TRUE(controller.config().csi_traffic_mode == CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_TRUE(controller.set_traffic_generator_mode_runtime(RuntimeTrafficMode::DNS_TCP));
  TEST_ASSERT_TRUE(controller.config().traffic_generator_mode == RuntimeTrafficMode::DNS_TCP);

  TEST_ASSERT_FALSE(controller.set_csi_traffic_mode_runtime(static_cast<CsiTrafficMode>(0x7f)));
  TEST_ASSERT_FALSE(controller.set_traffic_generator_mode_runtime(static_cast<RuntimeTrafficMode>(0x7f)));

  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_TRUE(controller.set_csi_traffic_mode_runtime(CsiTrafficMode::INTERNAL));
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_csi_traffic_mode_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_csi_traffic_mode == CsiTrafficMode::INTERNAL);

  TEST_ASSERT_TRUE(controller.set_traffic_generator_mode_runtime(RuntimeTrafficMode::PING));
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_traffic_generator_mode_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_traffic_generator_mode == RuntimeTrafficMode::PING);
}

void test_runtime_frontend_controller_recalibration_requires_capability_and_runtime(void) {
  RuntimeFrontendController controller;

  TEST_ASSERT_FALSE(controller.trigger_recalibration());

  DummyRuntimeListener listener;
  frontend_runtime_shim::state.capabilities.supports_manual_recalibration = true;
  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_FALSE(controller.is_calibrating());

  frontend_runtime_shim::state.calibrating = true;
  TEST_ASSERT_TRUE(controller.is_calibrating());
  TEST_ASSERT_TRUE(controller.trigger_recalibration());
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.trigger_recalibration_calls);

  controller.shutdown();
  TEST_ASSERT_FALSE(controller.is_calibrating());
}

void test_runtime_frontend_controller_refreshes_snapshot_across_raw_collection(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;
  frontend_runtime_shim::state.capabilities.supports_raw_csi = true;
  frontend_runtime_shim::state.snapshot.ready_to_publish = true;
  TEST_ASSERT_TRUE(controller.setup(&listener));

  TEST_ASSERT_TRUE(controller.start_raw_collection(
      [](void *, const RawCsiPacketView &) { return true; }, nullptr));
  TEST_ASSERT_EQUAL(RuntimeOperationState::RAW_COLLECTION, controller.operation_state());
  TEST_ASSERT_FALSE(controller.snapshot().ready_to_publish);

  TEST_ASSERT_TRUE(controller.stop_raw_collection());
  TEST_ASSERT_EQUAL(RuntimeOperationState::SENSING, controller.operation_state());
  TEST_ASSERT_TRUE(controller.snapshot().ready_to_publish);
}

void test_runtime_frontend_controller_applies_armed_state_staged_during_raw_collection(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;
  frontend_runtime_shim::state.capabilities.supports_raw_csi = true;
  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_TRUE(controller.start_raw_collection(
      [](void *, const RawCsiPacketView &) { return true; }, nullptr));

  controller.set_services_armed(false);
  TEST_ASSERT_FALSE(controller.services_armed());
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.services_armed);
  TEST_ASSERT_EQUAL(RuntimeOperationState::RAW_COLLECTION, controller.operation_state());

  TEST_ASSERT_TRUE(controller.stop_raw_collection());
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.services_armed);
}

void test_runtime_frontend_controller_quiesces_raw_collection(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;
  frontend_runtime_shim::state.capabilities.supports_raw_csi = true;
  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_TRUE(controller.start_raw_collection(
      [](void *, const RawCsiPacketView &) { return true; }, nullptr));

  controller.quiesce();

  TEST_ASSERT_EQUAL(RuntimeOperationState::SENSING, controller.operation_state());
  TEST_ASSERT_FALSE(controller.services_armed());
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.services_armed);
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.live_telemetry_enabled);
}

void test_runtime_frontend_controller_caches_and_forwards_listener_events(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;
  listener.controller = &controller;
  TEST_ASSERT_TRUE(controller.setup(&listener));

  RuntimeSnapshot snapshot = controller.snapshot();
  snapshot.threshold = 0.65f;
  frontend_runtime_shim::state.last_listener->on_threshold_changed(snapshot);

  TEST_ASSERT_EQUAL(1, listener.threshold_count);
  TEST_ASSERT_EQUAL_FLOAT(0.65f, listener.last_threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.65f, listener.cached_threshold_during_callback);
  TEST_ASSERT_EQUAL_FLOAT(0.65f, listener.configured_threshold_during_callback);

  frontend_runtime_shim::state.last_listener->on_live_telemetry(0.4f, 0.6f);
  TEST_ASSERT_EQUAL(1, listener.telemetry_count);
  TEST_ASSERT_EQUAL_FLOAT(0.4f, listener.last_movement);
  TEST_ASSERT_EQUAL_FLOAT(0.4f, controller.snapshot().movement_metric);
  TEST_ASSERT_EQUAL_FLOAT(0.6f, controller.snapshot().threshold);
}

void test_runtime_frontend_controller_defers_shutdown_requested_by_listener(void) {
  RuntimeFrontendController controller;
  DummyRuntimeListener listener;
  listener.controller = &controller;
  listener.shutdown_on_threshold = true;
  TEST_ASSERT_TRUE(controller.setup(&listener));

  frontend_runtime_shim::state.snapshot.threshold = 0.55f;
  frontend_runtime_shim::state.emit_threshold_on_next_loop = true;
  controller.loop();

  TEST_ASSERT_EQUAL(1, listener.threshold_count);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.shutdown_called);
  TEST_ASSERT_FALSE(controller.is_setup_complete());
}

void test_runtime_frontend_controller_switches_detector_and_resets_threshold(void) {
  RuntimeFrontendController controller;
  RuntimeConfig config;
  config.runtime_detector_selection_enabled = true;
  config.segmentation_threshold = 0.4f;
  controller.set_config(config);

  TEST_ASSERT_TRUE(controller.set_detection_algorithm_runtime(DetectionAlgorithm::HIGH_ACCURACY));
  TEST_ASSERT_TRUE(controller.config().detection_algorithm == DetectionAlgorithm::HIGH_ACCURACY);
  TEST_ASSERT_EQUAL_FLOAT(HIGH_ACCURACY_DEFAULT_THRESHOLD, controller.snapshot().threshold);

  frontend_runtime_shim::state.capabilities.supports_runtime_detector_selection = true;
  DummyRuntimeListener listener;
  TEST_ASSERT_TRUE(controller.setup(&listener));
  TEST_ASSERT_TRUE(controller.set_detection_algorithm_runtime(DetectionAlgorithm::LIGHTWEIGHT));
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_detector_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_detector == DetectionAlgorithm::LIGHTWEIGHT);
  TEST_ASSERT_EQUAL_FLOAT(LIGHTWEIGHT_DEFAULT_THRESHOLD, controller.snapshot().threshold);
}

int process(void) {
  UNITY_BEGIN();
  RUN_TEST(test_frontend_bootstrap_loads_defaults_and_preserves_runtime_identity);
  RUN_TEST(test_frontend_bootstrap_accepts_optional_defaults_and_validates_wifi_options);
  RUN_TEST(test_frontend_bootstrap_sets_up_wifi_and_propagates_start_failure);
  RUN_TEST(test_runtime_frontend_controller_preserves_pre_setup_config_and_snapshot);
  RUN_TEST(test_runtime_frontend_controller_rejects_invalid_config_before_backend_setup);
  RUN_TEST(test_runtime_frontend_controller_keeps_staged_mutations_out_of_live_validation);
  RUN_TEST(test_runtime_frontend_controller_preserves_staged_fields_across_live_callbacks);
  RUN_TEST(test_runtime_frontend_controller_preserves_staged_fields_across_live_setters);
  RUN_TEST(test_runtime_frontend_controller_setup_propagates_state_and_handles_failure);
  RUN_TEST(test_runtime_frontend_controller_adopts_backend_effective_config);
  RUN_TEST(test_runtime_frontend_controller_loop_shutdown_and_runtime_toggles_forward);
  RUN_TEST(test_runtime_frontend_controller_reads_diagnostics_from_backend);
  RUN_TEST(test_runtime_frontend_controller_exposes_runtime_owned_diagnostics_sample);
  RUN_TEST(test_runtime_frontend_controller_threshold_runtime_updates_config_and_snapshot);
  RUN_TEST(test_runtime_frontend_controller_threshold_requires_capability);
  RUN_TEST(test_runtime_frontend_controller_motion_hits_runtime_updates_config);
  RUN_TEST(test_runtime_frontend_controller_traffic_runtime_updates_config);
  RUN_TEST(test_runtime_frontend_controller_recalibration_requires_capability_and_runtime);
  RUN_TEST(test_runtime_frontend_controller_refreshes_snapshot_across_raw_collection);
  RUN_TEST(test_runtime_frontend_controller_applies_armed_state_staged_during_raw_collection);
  RUN_TEST(test_runtime_frontend_controller_quiesces_raw_collection);
  RUN_TEST(test_runtime_frontend_controller_caches_and_forwards_listener_events);
  RUN_TEST(test_runtime_frontend_controller_defers_shutdown_requested_by_listener);
  RUN_TEST(test_runtime_frontend_controller_switches_detector_and_resets_threshold);
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
