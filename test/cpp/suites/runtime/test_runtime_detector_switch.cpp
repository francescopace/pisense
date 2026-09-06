/*
 * ESPectre - Runtime Detector Switch Unit Tests
 *
 * Unit tests for Runtime Detector Switch.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>

#define private public
#define protected public
#include "esp_idf_runtime.h"
#undef protected
#undef private

#include "esp_timer.h"
#include "csi_format.h"
#include "csi_traffic_fakes.h"
#include "nvs.h"
#include "runtime_detector_store.h"
#include "runtime_motion_hits_store.h"
#include "runtime_traffic_mode_store.h"

using namespace espectre;
using namespace espectre::test;

namespace {

class DetectorListener : public IRuntimeListener {
 public:
  void on_detector_changed(const RuntimeSnapshot &snapshot) override {
    detector_changes++;
    last_detector = snapshot.detector_name;
  }
  void on_threshold_changed(const RuntimeSnapshot &snapshot) override {
    threshold_changes++;
    last_threshold = snapshot.threshold;
  }
  void on_calibration_started(const RuntimeSnapshot &) override { calibration_starts++; }
  void on_calibration_finished(const RuntimeSnapshot &, bool success) override {
    calibration_finishes++;
    last_calibration_success = success;
  }
  void on_runtime_fault(const char *) override { faults++; }

  int detector_changes{0};
  int threshold_changes{0};
  int calibration_starts{0};
  int calibration_finishes{0};
  int faults{0};
  std::string last_detector;
  float last_threshold{0.0f};
  bool last_calibration_success{true};
};

bool accept_raw_packet(void *, const RawCsiPacketView &) { return true; }

}  // namespace

void setUp(void) {
  nvs_mock_reset();
  esp_timer_mock::reset();
  esp_event_mock_reset();
  esp_netif_mock_reset();
  esp_wifi_mock_reset();
}
void tearDown(void) {}

void test_runtime_detector_switch_updates_pipeline_threshold_and_calibration(void) {
  RuntimeConfig config;
  config.runtime_detector_selection_enabled = true;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  EspIdfRuntime runtime(config);
  DetectorListener listener;
  runtime.set_listener(&listener);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());

  TEST_ASSERT_TRUE(runtime.set_detection_algorithm_runtime(DetectionAlgorithm::HIGH_ACCURACY));
  TEST_ASSERT_EQUAL_STRING("high_accuracy", runtime.get_snapshot().detector_name);
  TEST_ASSERT_EQUAL_FLOAT(HIGH_ACCURACY_DEFAULT_THRESHOLD, runtime.get_snapshot().threshold);
  TEST_ASSERT_FALSE(runtime.is_calibrating());
  TEST_ASSERT_EQUAL(1, listener.detector_changes);
  TEST_ASSERT_EQUAL(1, listener.threshold_changes);

  runtime.csi_pipeline_.enabled_ = true;
  TEST_ASSERT_TRUE(runtime.set_detection_algorithm_runtime(DetectionAlgorithm::LIGHTWEIGHT));
  TEST_ASSERT_EQUAL_STRING("lightweight", runtime.get_snapshot().detector_name);
  TEST_ASSERT_EQUAL_FLOAT(LIGHTWEIGHT_DEFAULT_THRESHOLD, runtime.get_snapshot().threshold);
  TEST_ASSERT_TRUE(runtime.is_calibrating());
  TEST_ASSERT_EQUAL(1, listener.calibration_starts);

  TEST_ASSERT_TRUE(runtime.set_detection_algorithm_runtime(DetectionAlgorithm::HIGH_ACCURACY));
  TEST_ASSERT_FALSE(runtime.is_calibrating());
  TEST_ASSERT_EQUAL(1, listener.calibration_finishes);
  TEST_ASSERT_FALSE(listener.last_calibration_success);

  TEST_ASSERT_TRUE(runtime.set_threshold_runtime(0.75f));
  TEST_ASSERT_EQUAL_FLOAT(0.75f, runtime.get_snapshot().threshold);
  TEST_ASSERT_TRUE(runtime.trigger_recalibration());
  TEST_ASSERT_EQUAL_FLOAT(HIGH_ACCURACY_DEFAULT_THRESHOLD, runtime.get_snapshot().threshold);
  TEST_ASSERT_TRUE(listener.last_calibration_success);
}

void test_runtime_detector_configuration_preserves_the_requested_threshold(void) {
  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::HIGH_ACCURACY;
  config.segmentation_threshold = 0.73f;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);

  TEST_ASSERT_TRUE(runtime.setup());
  TEST_ASSERT_EQUAL_FLOAT(0.73f, runtime.config_.segmentation_threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.73f, runtime.get_snapshot().threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.73f, runtime.detector_->get_threshold());

  esp_netif_ip_info_t ip_info{};
  ip_info.ip.addr = 0x0101A8C0U;
  ip_info.gw.addr = 0x0101A8C0U;
  runtime.on_wifi_connected_(ip_info);
  TEST_ASSERT_TRUE(runtime.get_snapshot().ready_to_publish);
  TEST_ASSERT_EQUAL_FLOAT(0.73f, runtime.get_snapshot().threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.73f, runtime.detector_->get_threshold());

  TEST_ASSERT_TRUE(runtime.set_threshold_runtime(0.68f));
  runtime.on_wifi_disconnected_();
  runtime.on_wifi_connected_(ip_info);
  wifi_event_sta_scan_done_t scan_done{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &scan_done);
  TEST_ASSERT_EQUAL(ESP_OK, runtime.wifi_lifecycle_.process_pending_events());
  TEST_ASSERT_TRUE(runtime.get_snapshot().ready_to_publish);
  TEST_ASSERT_EQUAL_FLOAT(0.68f, runtime.get_snapshot().threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.68f, runtime.detector_->get_threshold());

  TEST_ASSERT_TRUE(runtime.start_raw_collection(&accept_raw_packet, nullptr));
  TEST_ASSERT_TRUE(runtime.stop_raw_collection(RawCsiStopReason::REQUESTED));
  TEST_ASSERT_EQUAL_FLOAT(0.68f, runtime.get_snapshot().threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.68f, runtime.detector_->get_threshold());
  runtime.shutdown();
}

void test_runtime_calibration_consumes_evaluations_resets_on_gaps_and_finishes(void) {
  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);
  DetectorListener listener;
  runtime.set_listener(&listener);
  TEST_ASSERT_TRUE(runtime.setup());
  esp_netif_ip_info_t ip_info{};
  ip_info.ip.addr = 0x0101A8C0U;
  ip_info.gw.addr = 0x0101A8C0U;
  runtime.on_wifi_connected_(ip_info);
  TEST_ASSERT_TRUE(runtime.is_calibrating());
  const uint16_t target = runtime.get_snapshot().calibration_target_packets;
  int8_t csi[HT20_CSI_LEN];
  std::fill(std::begin(csi), std::end(csi), 8);
  TEST_ASSERT_FALSE(EspIdfRuntime::threshold_calibration_packet_callback_(
      nullptr, csi, sizeof(csi), -50, true, 1U, false));
  TEST_ASSERT_TRUE(runtime.handle_threshold_calibration_packet_(csi, sizeof(csi), -50, false, 1U, false));
  TEST_ASSERT_EQUAL(0, runtime.get_snapshot().calibration_packets);
  // An evaluation cannot count calibration packets before the detector is ready.
  TEST_ASSERT_TRUE(runtime.handle_threshold_calibration_packet_(csi, sizeof(csi), -50, true, 1U, false));
  TEST_ASSERT_EQUAL(0, runtime.get_snapshot().calibration_packets);
  for (uint32_t index = 0; index < target && runtime.get_snapshot().calibration_packets < 2U; ++index) {
    TEST_ASSERT_TRUE(EspIdfRuntime::threshold_calibration_packet_callback_(
        &runtime, csi, sizeof(csi), -50, true, 1U, false));
  }
  TEST_ASSERT_EQUAL(2, runtime.get_snapshot().calibration_packets);
  TEST_ASSERT_TRUE(runtime.handle_threshold_calibration_packet_(csi, sizeof(csi), -50, true, 1U, true));
  TEST_ASSERT_EQUAL(1, runtime.get_snapshot().calibration_packets);
  for (uint32_t index = 0; index < target && runtime.threshold_calibration_active_.load(); ++index) {
    TEST_ASSERT_TRUE(runtime.handle_threshold_calibration_packet_(csi, sizeof(csi), -50, true, 1U, false));
  }
  TEST_ASSERT_FALSE(runtime.threshold_calibration_active_.load());
  TEST_ASSERT_EQUAL(target, runtime.get_snapshot().calibration_packets);
  runtime.loop();
  TEST_ASSERT_FALSE(runtime.is_calibrating());
  TEST_ASSERT_EQUAL(1, listener.calibration_finishes);
  TEST_ASSERT_TRUE(listener.last_calibration_success);
  TEST_ASSERT_EQUAL_FLOAT(runtime.detector_->get_threshold(), runtime.get_snapshot().threshold);
  TEST_ASSERT_EQUAL_FLOAT(runtime.get_snapshot().threshold, runtime.get_snapshot().startup_threshold);
  TEST_ASSERT_EQUAL(0, runtime.get_snapshot().calibration_target_packets);
  TEST_ASSERT_FALSE(runtime.handle_threshold_calibration_packet_(csi, sizeof(csi), -50, true, 1U, false));
  runtime.loop();
  TEST_ASSERT_EQUAL(1, listener.calibration_finishes);
  runtime.shutdown();
}

void test_runtime_rejects_invalid_detector_geometry_before_starting_services(void) {
  for (int field = 0; field < 3; ++field) {
    RuntimeConfig config;
    if (field == 0) config.csi_target_pps = RUNTIME_CSI_TARGET_PPS_MIN - 1U;
    if (field == 1) config.segmentation_window_size_ms = RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_MAX + 1U;
    if (field == 2) config.segmentation_threshold = -1.0f;
    FakeCsiTrafficGenerator generator;
    FakeCsiTrafficIngress ingress;
    EspIdfRuntime runtime(config, generator, ingress);
    DetectorListener listener;
    runtime.set_listener(&listener);
    TEST_ASSERT_FALSE(runtime.setup());
    TEST_ASSERT_EQUAL(1, listener.faults);
    TEST_ASSERT_EQUAL(0U, generator.start_calls);
    TEST_ASSERT_EQUAL(0U, ingress.start_calls);
    runtime.shutdown();
  }
}

void test_runtime_rejects_invalid_or_unpersisted_controls_without_changing_config(void) {
  RuntimeConfig config;
  config.runtime_detector_selection_enabled = true;
  FakeCsiTrafficGenerator generator;
  FakeCsiTrafficIngress ingress;
  EspIdfRuntime runtime(config, generator, ingress);
  TEST_ASSERT_TRUE(runtime.setup());
  TEST_ASSERT_FALSE(runtime.set_motion_hits_runtime(RUNTIME_MOTION_HITS_MIN - 1U, config.motion_off_hits));
  TEST_ASSERT_FALSE(runtime.set_motion_hits_runtime(config.motion_on_hits, RUNTIME_MOTION_HITS_MAX + 1U));
  TEST_ASSERT_FALSE(runtime.set_csi_traffic_mode_runtime(static_cast<CsiTrafficMode>(255)));
  TEST_ASSERT_FALSE(runtime.set_traffic_generator_mode_runtime(static_cast<RuntimeTrafficMode>(255)));
  TEST_ASSERT_FALSE(runtime.set_detection_algorithm_runtime(static_cast<DetectionAlgorithm>(255)));
  TEST_ASSERT_FALSE(runtime.set_threshold_runtime(-1.0f));
  nvs_mock_set_open_result(ESP_FAIL);
  TEST_ASSERT_FALSE(runtime.set_motion_hits_runtime(7U, 5U));
  TEST_ASSERT_FALSE(runtime.set_traffic_generator_mode_runtime(RuntimeTrafficMode::DNS));
  TEST_ASSERT_FALSE(runtime.set_detection_algorithm_runtime(DetectionAlgorithm::HIGH_ACCURACY));
  TEST_ASSERT_TRUE(runtime.effective_config().traffic_generator_mode == config.traffic_generator_mode);
  TEST_ASSERT_TRUE(runtime.effective_config().detection_algorithm == config.detection_algorithm);
  TEST_ASSERT_EQUAL(config.motion_on_hits, runtime.effective_config().motion_on_hits);
  TEST_ASSERT_EQUAL(config.motion_off_hits, runtime.effective_config().motion_off_hits);
  runtime.shutdown();
}

void test_runtime_restores_internal_traffic_when_external_source_cannot_start(void) {
  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::HIGH_ACCURACY;
  FakeCsiTrafficGenerator generator;
  FakeCsiTrafficIngress ingress;
  EspIdfRuntime runtime(config, generator, ingress);
  DetectorListener listener;
  runtime.set_listener(&listener);
  TEST_ASSERT_TRUE(runtime.setup());
  esp_netif_ip_info_t ip_info{};
  ip_info.ip.addr = ip_info.gw.addr = 0x0101A8C0U;
  runtime.on_wifi_connected_(ip_info);
  ingress.start_result = false;
  TEST_ASSERT_FALSE(runtime.set_csi_traffic_mode_runtime(CsiTrafficMode::EXTERNAL));
  TEST_ASSERT_TRUE(runtime.effective_config().csi_traffic_mode == CsiTrafficMode::INTERNAL);
  TEST_ASSERT_TRUE(generator.is_running());
  TEST_ASSERT_FALSE(ingress.is_running());
  TEST_ASSERT_EQUAL(1, listener.faults);
  generator.start_result = false;
  TEST_ASSERT_FALSE(runtime.set_traffic_generator_mode_runtime(RuntimeTrafficMode::DNS));
  TEST_ASSERT_TRUE(runtime.effective_config().traffic_generator_mode == config.traffic_generator_mode);
  TEST_ASSERT_FALSE(generator.is_running());
  TEST_ASSERT_TRUE(listener.faults > 1);
  runtime.shutdown();
}

void test_runtime_traffic_updates_roll_back_when_persistence_fails(void) {
  RuntimeConfig config;
  config.csi_traffic_mode = CsiTrafficMode::INTERNAL;
  EspIdfRuntime runtime(config);
  nvs_mock_set_open_result(ESP_FAIL);

  TEST_ASSERT_FALSE(runtime.set_csi_traffic_mode_runtime(CsiTrafficMode::EXTERNAL));
  TEST_ASSERT_TRUE(runtime.config_.csi_traffic_mode == CsiTrafficMode::INTERNAL);
  TEST_ASSERT_FALSE(runtime.set_traffic_generator_mode_runtime(RuntimeTrafficMode::DNS_TCP));
  TEST_ASSERT_TRUE(runtime.config_.traffic_generator_mode == RuntimeTrafficMode::PING);
}

void test_runtime_detector_adaptation_emits_threshold_changed_without_live_telemetry(void) {
  RuntimeConfig config;
  config.segmentation_threshold = 0.80f;
  EspIdfRuntime runtime(config);
  DetectorListener listener;
  runtime.set_listener(&listener);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.set_live_telemetry_enabled(false);

  TEST_ASSERT_TRUE(runtime.detector_->set_threshold(0.42f));
  runtime.loop();
  TEST_ASSERT_EQUAL(1, listener.threshold_changes);
  TEST_ASSERT_EQUAL_FLOAT(0.42f, listener.last_threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.42f, runtime.get_snapshot().threshold);
  TEST_ASSERT_EQUAL_FLOAT(0.42f, runtime.config_.segmentation_threshold);

  runtime.loop();
  TEST_ASSERT_EQUAL(1, listener.threshold_changes);
}

void test_runtime_motion_hits_runtime_updates_pipeline_and_persists(void) {
  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  EspIdfRuntime runtime(config);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());

  TEST_ASSERT_TRUE(runtime.set_motion_hits_runtime(8U, 6U));
  TEST_ASSERT_EQUAL_UINT8(8U, runtime.csi_pipeline_.motion_on_hits_);
  TEST_ASSERT_EQUAL_UINT8(6U, runtime.csi_pipeline_.motion_off_hits_);

  uint8_t saved_motion_on_hits = 0U;
  uint8_t saved_motion_off_hits = 0U;
  bool has_saved_value = false;
  TEST_ASSERT_EQUAL(ESP_OK,
                    load_runtime_motion_hits(&saved_motion_on_hits, &saved_motion_off_hits, &has_saved_value));
  TEST_ASSERT_TRUE(has_saved_value);
  TEST_ASSERT_EQUAL_UINT8(8U, saved_motion_on_hits);
  TEST_ASSERT_EQUAL_UINT8(6U, saved_motion_off_hits);
}

void test_runtime_setup_loads_all_persisted_runtime_controls(void) {
  TEST_ASSERT_EQUAL(ESP_OK, save_runtime_detection_algorithm(DetectionAlgorithm::HIGH_ACCURACY));
  TEST_ASSERT_EQUAL(ESP_OK, save_runtime_motion_hits(8U, 6U));
  TEST_ASSERT_EQUAL(ESP_OK, save_runtime_csi_traffic_mode(CsiTrafficMode::EXTERNAL));
  TEST_ASSERT_EQUAL(ESP_OK, save_runtime_traffic_generator_mode(RuntimeTrafficMode::DNS));

  RuntimeConfig config;
  config.runtime_detector_selection_enabled = true;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  config.motion_on_hits = 4U;
  config.motion_off_hits = 3U;
  config.csi_traffic_mode = CsiTrafficMode::INTERNAL;
  config.traffic_generator_mode = RuntimeTrafficMode::PING;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);

  TEST_ASSERT_TRUE(runtime.setup());
  const RuntimeConfig &effective = runtime.effective_config();
  TEST_ASSERT_TRUE(effective.detection_algorithm == DetectionAlgorithm::HIGH_ACCURACY);
  TEST_ASSERT_EQUAL_FLOAT(HIGH_ACCURACY_DEFAULT_THRESHOLD, effective.segmentation_threshold);
  TEST_ASSERT_EQUAL_UINT8(8U, effective.motion_on_hits);
  TEST_ASSERT_EQUAL_UINT8(6U, effective.motion_off_hits);
  TEST_ASSERT_TRUE(effective.csi_traffic_mode == CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_TRUE(effective.traffic_generator_mode == RuntimeTrafficMode::DNS);
  TEST_ASSERT_TRUE(runtime.csi_traffic_service_.mode() == CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_TRUE(traffic_generator.mode == RuntimeTrafficMode::DNS);
  runtime.shutdown();
}

void test_runtime_diagnostics_cache_current_wifi_association(void) {
  esp_wifi_mock_reset();
  RuntimeConfig config;
  EspIdfRuntime runtime(config);
  runtime.services_armed_ = false;
  esp_netif_ip_info_t ip_info{};
  ip_info.ip.addr = 0x0101A8C0U;

  runtime.on_wifi_connected_(ip_info);

  RuntimeDiagnosticsSnapshot diagnostics = runtime.get_diagnostics();
  TEST_ASSERT_EQUAL_UINT8(6U, diagnostics.wifi_channel);
  TEST_ASSERT_EQUAL_INT8(-55, diagnostics.wifi_rssi_dbm);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.get_ap_info_call_count);

  runtime.csi_pipeline_.last_channel_ = 11U;
  runtime.csi_pipeline_.last_rssi_dbm_ = -42;
  runtime.refresh_wifi_association_from_csi_();
  runtime.csi_pipeline_.last_channel_ = 0U;
  runtime.csi_pipeline_.last_rssi_dbm_ = INT8_MIN;

  diagnostics = runtime.get_diagnostics();
  TEST_ASSERT_EQUAL_UINT8(11U, diagnostics.wifi_channel);
  TEST_ASSERT_EQUAL_INT8(-42, diagnostics.wifi_rssi_dbm);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.get_ap_info_call_count);

  runtime.on_wifi_disconnected_();
  diagnostics = runtime.get_diagnostics();
  TEST_ASSERT_EQUAL_UINT8(0U, diagnostics.wifi_channel);
  TEST_ASSERT_EQUAL_INT8(INT8_MIN, diagnostics.wifi_rssi_dbm);
}

void test_runtime_channel_change_rearms_csi_and_restarts_calibration(void) {
  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);
  DetectorListener listener;
  runtime.set_listener(&listener);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());
  runtime.csi_traffic_service_.init(to_csi_traffic_config(config));
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.enable());

  runtime.wifi_ready_ = true;
  runtime.wifi_ip_info_.ip.addr = 0x0101A8C0U;
  runtime.wifi_ip_info_.gw.addr = 0x0101A8C0U;
  runtime.on_csi_channel_changed_(8U, 10U);

  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_TRUE(runtime.is_calibrating());
  TEST_ASSERT_TRUE(runtime.get_snapshot().ready_to_publish);
  TEST_ASSERT_EQUAL(MotionState::IDLE, runtime.get_snapshot().motion_state);
  TEST_ASSERT_EQUAL(1, listener.calibration_starts);
  runtime.csi_traffic_service_.stop();
}

void test_runtime_services_armed_preserves_wifi_ip_and_restarts_capture(void) {
  esp_event_mock_reset();
  esp_wifi_mock_reset();
  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);
  DetectorListener listener;
  runtime.set_listener(&listener);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());
  runtime.csi_traffic_service_.init(to_csi_traffic_config(config));
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.enable());
  runtime.setup_complete_ = true;
  runtime.wifi_ready_ = true;
  runtime.wifi_ip_info_.ip.addr = 0x0101A8C0U;
  runtime.wifi_ip_info_.gw.addr = 0x0101A8C0U;
  const esp_netif_ip_info_t ip_info = runtime.wifi_ip_info_;
  runtime.snapshot_.ready_to_publish = true;

  runtime.set_services_armed(false);
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_FALSE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback == nullptr);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_required_);
  TEST_ASSERT_TRUE(runtime.wifi_ready_);
  TEST_ASSERT_EQUAL(0x0101A8C0U, runtime.wifi_ip_info_.ip.addr);
  TEST_ASSERT_FALSE(runtime.get_snapshot().ready_to_publish);

  runtime.set_services_armed(true);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.scan_start_call_count);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_in_progress_);
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_FALSE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback == nullptr);
  TEST_ASSERT_TRUE(runtime.wifi_ready_);
  TEST_ASSERT_EQUAL(0x0101A8C0U, runtime.wifi_ip_info_.ip.addr);

  wifi_event_sta_scan_done_t event{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &event);
  TEST_ASSERT_EQUAL(ESP_OK, runtime.wifi_lifecycle_.process_pending_events());
  TEST_ASSERT_FALSE(runtime.csi_receive_path_refresh_in_progress_);
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback != nullptr);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.promiscuous);
  TEST_ASSERT_TRUE(runtime.get_snapshot().ready_to_publish);
  TEST_ASSERT_TRUE(runtime.is_calibrating());
  TEST_ASSERT_EQUAL(1, listener.calibration_starts);

  runtime.on_wifi_disconnected_();
  TEST_ASSERT_FALSE(runtime.wifi_ready_);
  TEST_ASSERT_EQUAL(0U, runtime.wifi_ip_info_.ip.addr);
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_FALSE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback == nullptr);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_required_);

  runtime.on_wifi_connected_(ip_info);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.scan_start_call_count);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_in_progress_);
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &event);
  TEST_ASSERT_EQUAL(ESP_OK, runtime.wifi_lifecycle_.process_pending_events());
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback != nullptr);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.promiscuous);
  TEST_ASSERT_TRUE(runtime.get_snapshot().ready_to_publish);
  runtime.csi_traffic_service_.stop();
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.disable());
}

void test_runtime_sensing_reasserts_promiscuous_disabled_before_capture(void) {
  esp_wifi_mock_reset();
  g_esp_wifi_mock.promiscuous = true;

  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());
  runtime.csi_traffic_service_.init(to_csi_traffic_config(config));

  esp_netif_ip_info_t ip_info{};
  ip_info.ip.addr = 0x0101A8C0U;
  ip_info.gw.addr = 0x0101A8C0U;
  runtime.start_sensing_services_(ip_info);

  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_promiscuous_call_count);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.last_promiscuous);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.promiscuous);
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  runtime.csi_traffic_service_.stop();
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.disable());
}

void test_runtime_scans_before_rearming_csi_after_services_resume(void) {
  esp_event_mock_reset();
  esp_wifi_mock_reset();

  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());
  runtime.csi_traffic_service_.init(to_csi_traffic_config(config));
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.enable());
  runtime.setup_complete_ = true;
  runtime.wifi_ready_ = true;
  runtime.wifi_ip_info_.ip.addr = 0x0101A8C0U;
  runtime.wifi_ip_info_.gw.addr = 0x0101A8C0U;

  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback != nullptr);

  runtime.set_services_armed(false);
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_FALSE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback == nullptr);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_required_);

  runtime.set_services_armed(true);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.scan_start_call_count);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_in_progress_);
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_FALSE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback == nullptr);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.last_promiscuous);

  wifi_event_sta_scan_done_t event{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &event);
  TEST_ASSERT_EQUAL(ESP_OK, runtime.wifi_lifecycle_.process_pending_events());
  TEST_ASSERT_FALSE(runtime.csi_receive_path_refresh_in_progress_);
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback != nullptr);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.promiscuous);
  runtime.csi_traffic_service_.stop();
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.disable());
}

void test_runtime_restarts_csi_refresh_after_disconnect_during_scan(void) {
  esp_event_mock_reset();
  esp_wifi_mock_reset();

  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());
  runtime.csi_traffic_service_.init(to_csi_traffic_config(config));
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.enable());
  runtime.setup_complete_ = true;
  runtime.wifi_ready_ = true;
  runtime.wifi_ip_info_.ip.addr = 0x0101A8C0U;
  runtime.wifi_ip_info_.gw.addr = 0x0101A8C0U;
  const esp_netif_ip_info_t ip_info = runtime.wifi_ip_info_;

  runtime.set_services_armed(false);
  runtime.set_services_armed(true);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.scan_start_call_count);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_in_progress_);

  wifi_event_sta_scan_done_t stale_event{};
  stale_event.status = 1U;
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &stale_event);
  runtime.on_wifi_disconnected_();
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.scan_stop_call_count);
  TEST_ASSERT_FALSE(runtime.csi_receive_path_refresh_in_progress_);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_required_);
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());

  TEST_ASSERT_EQUAL(ESP_OK, runtime.wifi_lifecycle_.process_pending_events());
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());
  runtime.on_wifi_connected_(ip_info);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.scan_start_call_count);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_in_progress_);
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());

  wifi_event_sta_scan_done_t fresh_event{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &fresh_event);
  TEST_ASSERT_EQUAL(ESP_OK, runtime.wifi_lifecycle_.process_pending_events());
  TEST_ASSERT_FALSE(runtime.csi_receive_path_refresh_in_progress_);
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback != nullptr);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.promiscuous);
  runtime.csi_traffic_service_.stop();
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.disable());
}

void test_runtime_retries_failed_refresh_before_a_pending_disconnect(void) {
  esp_event_mock_reset();
  esp_wifi_mock_reset();

  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());
  runtime.csi_traffic_service_.init(to_csi_traffic_config(config));
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.enable());
  runtime.setup_complete_ = true;
  runtime.wifi_ready_ = true;
  runtime.wifi_ip_info_.ip.addr = 0x0101A8C0U;
  runtime.wifi_ip_info_.gw.addr = 0x0101A8C0U;
  const esp_netif_ip_info_t ip_info = runtime.wifi_ip_info_;

  runtime.set_services_armed(false);
  runtime.set_services_armed(true);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.scan_start_call_count);

  wifi_event_sta_scan_done_t failed_event{};
  failed_event.status = 1U;
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &failed_event);
  TEST_ASSERT_EQUAL(ESP_OK, runtime.wifi_lifecycle_.process_pending_events());
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.scan_start_call_count);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_in_progress_);
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());

  runtime.on_wifi_disconnected_();
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.scan_stop_call_count);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_required_);
  TEST_ASSERT_FALSE(runtime.csi_receive_path_refresh_in_progress_);
  runtime.on_wifi_connected_(ip_info);
  TEST_ASSERT_EQUAL(3, g_esp_wifi_mock.scan_start_call_count);
  TEST_ASSERT_TRUE(runtime.csi_receive_path_refresh_in_progress_);

  wifi_event_sta_scan_done_t fresh_event{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &fresh_event);
  TEST_ASSERT_EQUAL(ESP_OK, runtime.wifi_lifecycle_.process_pending_events());
  TEST_ASSERT_FALSE(runtime.csi_receive_path_refresh_in_progress_);
  TEST_ASSERT_FALSE(runtime.csi_receive_path_refresh_required_);
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_enabled);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.csi_callback != nullptr);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.promiscuous);
  runtime.csi_traffic_service_.stop();
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.disable());
}

void test_runtime_raw_collection_restores_armed_and_disarmed_sensing(void) {
  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
  config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);
  DetectorListener listener;
  runtime.set_listener(&listener);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());
  runtime.csi_traffic_service_.init(to_csi_traffic_config(config));
  runtime.capabilities_.supports_raw_csi = true;
  runtime.setup_complete_ = true;
  runtime.wifi_ready_ = true;
  runtime.wifi_ip_info_.ip.addr = 0x0101A8C0U;
  runtime.wifi_ip_info_.gw.addr = 0x0101A8C0U;
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.enable());
  runtime.snapshot_.calibrating = true;
  runtime.snapshot_.ready_to_publish = true;

  TEST_ASSERT_TRUE(runtime.start_raw_collection(&accept_raw_packet, nullptr));
  TEST_ASSERT_EQUAL(RuntimeOperationState::RAW_COLLECTION, runtime.operation_state());
  TEST_ASSERT_EQUAL(CsiTrafficMode::EXTERNAL, runtime.csi_traffic_service_.mode());
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_FALSE(runtime.snapshot_.calibrating);
  TEST_ASSERT_FALSE(runtime.snapshot_.ready_to_publish);
  TEST_ASSERT_FALSE(runtime.set_threshold_runtime(0.5f));

  runtime.set_services_armed(false);
  TEST_ASSERT_EQUAL(RuntimeOperationState::RAW_COLLECTION, runtime.operation_state());
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_TRUE(runtime.stop_raw_collection(RawCsiStopReason::REQUESTED));
  TEST_ASSERT_EQUAL(RuntimeOperationState::SENSING, runtime.operation_state());
  TEST_ASSERT_EQUAL(CsiTrafficMode::EXTERNAL, runtime.csi_traffic_service_.mode());
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_FALSE(runtime.snapshot_.ready_to_publish);

  const int calibration_starts = listener.calibration_starts;
  const int calibration_finishes = listener.calibration_finishes;
  TEST_ASSERT_FALSE(runtime.start_raw_collection(&accept_raw_packet, nullptr));
  TEST_ASSERT_EQUAL(RuntimeOperationState::SENSING, runtime.operation_state());
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.raw_capture_active());
  TEST_ASSERT_FALSE(runtime.csi_traffic_service_.is_running());
  TEST_ASSERT_EQUAL(0U, traffic_generator.start_calls);
  TEST_ASSERT_EQUAL(0U, traffic_ingress.start_calls);
  TEST_ASSERT_FALSE(runtime.snapshot_.calibrating);
  TEST_ASSERT_FALSE(runtime.snapshot_.ready_to_publish);
  TEST_ASSERT_EQUAL(calibration_starts, listener.calibration_starts);
  TEST_ASSERT_EQUAL(calibration_finishes, listener.calibration_finishes);

  runtime.set_services_armed(true);
  TEST_ASSERT_TRUE(runtime.start_raw_collection(&accept_raw_packet, nullptr));
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  runtime.set_services_armed(false);
  runtime.set_services_armed(true);
  TEST_ASSERT_EQUAL(RuntimeOperationState::RAW_COLLECTION, runtime.operation_state());
  TEST_ASSERT_TRUE(runtime.stop_raw_collection(RawCsiStopReason::REQUESTED));
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_TRUE(runtime.snapshot_.calibrating);
  TEST_ASSERT_TRUE(runtime.snapshot_.ready_to_publish);
  runtime.csi_traffic_service_.stop();
}

void test_runtime_raw_collection_terminates_on_wifi_loss_and_channel_change(void) {
  RuntimeConfig config;
  config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());
  runtime.csi_traffic_service_.init(to_csi_traffic_config(config));
  runtime.capabilities_.supports_raw_csi = true;
  runtime.setup_complete_ = true;
  runtime.wifi_ready_ = true;
  runtime.wifi_ip_info_.ip.addr = 0x0101A8C0U;
  runtime.wifi_ip_info_.gw.addr = 0x0101A8C0U;

  TEST_ASSERT_TRUE(runtime.start_raw_collection(&accept_raw_packet, nullptr));
  runtime.on_csi_channel_changed_(6U, 11U);
  TEST_ASSERT_EQUAL(RuntimeOperationState::SENSING, runtime.operation_state());
  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());

  TEST_ASSERT_TRUE(runtime.start_raw_collection(&accept_raw_packet, nullptr));
  runtime.on_wifi_disconnected_();
  TEST_ASSERT_EQUAL(RuntimeOperationState::SENSING, runtime.operation_state());
  TEST_ASSERT_FALSE(runtime.wifi_ready_);
  TEST_ASSERT_FALSE(runtime.csi_pipeline_.is_enabled());
  runtime.csi_traffic_service_.stop();
}

void test_runtime_channel_change_cold_resets_ml_without_calibration(void) {
  RuntimeConfig config;
  config.detection_algorithm = DetectionAlgorithm::HIGH_ACCURACY;
  config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  FakeCsiTrafficGenerator traffic_generator;
  FakeCsiTrafficIngress traffic_ingress;
  EspIdfRuntime runtime(config, traffic_generator, traffic_ingress);
  TEST_ASSERT_TRUE(runtime.configure_detector_());
  runtime.csi_pipeline_.init(runtime.detector_.get());
  runtime.csi_traffic_service_.init(to_csi_traffic_config(config));
  TEST_ASSERT_EQUAL(ESP_OK, runtime.csi_pipeline_.enable());

  int8_t csi_data[HT20_CSI_LEN] = {};
  runtime.detector_->process_packet(csi_data, sizeof(csi_data), DEFAULT_SUBCARRIERS,
                                    HT20_SELECTED_BAND_SIZE, -50);
  TEST_ASSERT_TRUE(runtime.detector_->get_buffer_count() > 0U);

  runtime.wifi_ready_ = true;
  runtime.wifi_ip_info_.ip.addr = 0x0101A8C0U;
  runtime.wifi_ip_info_.gw.addr = 0x0101A8C0U;
  runtime.on_csi_channel_changed_(8U, 10U);

  TEST_ASSERT_TRUE(runtime.csi_pipeline_.is_enabled());
  TEST_ASSERT_FALSE(runtime.is_calibrating());
  TEST_ASSERT_TRUE(runtime.get_snapshot().ready_to_publish);
  TEST_ASSERT_EQUAL(0U, runtime.detector_->get_buffer_count());
  runtime.csi_traffic_service_.stop();
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_runtime_calibration_consumes_evaluations_resets_on_gaps_and_finishes);
  RUN_TEST(test_runtime_rejects_invalid_detector_geometry_before_starting_services);
  RUN_TEST(test_runtime_rejects_invalid_or_unpersisted_controls_without_changing_config);
  RUN_TEST(test_runtime_restores_internal_traffic_when_external_source_cannot_start);
  RUN_TEST(test_runtime_detector_switch_updates_pipeline_threshold_and_calibration);
  RUN_TEST(test_runtime_detector_configuration_preserves_the_requested_threshold);
  RUN_TEST(test_runtime_traffic_updates_roll_back_when_persistence_fails);
  RUN_TEST(test_runtime_detector_adaptation_emits_threshold_changed_without_live_telemetry);
  RUN_TEST(test_runtime_motion_hits_runtime_updates_pipeline_and_persists);
  RUN_TEST(test_runtime_setup_loads_all_persisted_runtime_controls);
  RUN_TEST(test_runtime_diagnostics_cache_current_wifi_association);
  RUN_TEST(test_runtime_channel_change_rearms_csi_and_restarts_calibration);
  RUN_TEST(test_runtime_services_armed_preserves_wifi_ip_and_restarts_capture);
  RUN_TEST(test_runtime_sensing_reasserts_promiscuous_disabled_before_capture);
  RUN_TEST(test_runtime_scans_before_rearming_csi_after_services_resume);
  RUN_TEST(test_runtime_restarts_csi_refresh_after_disconnect_during_scan);
  RUN_TEST(test_runtime_retries_failed_refresh_before_a_pending_disconnect);
  RUN_TEST(test_runtime_raw_collection_restores_armed_and_disarmed_sensing);
  RUN_TEST(test_runtime_raw_collection_terminates_on_wifi_loss_and_channel_change);
  RUN_TEST(test_runtime_channel_change_cold_resets_ml_without_calibration);
  return UNITY_END();
}
