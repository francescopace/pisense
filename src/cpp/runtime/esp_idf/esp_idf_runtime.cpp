/*
 * ESPectre - ESP-IDF Runtime
 *
 * ESP-IDF runtime that wires Wi-Fi, CSI capture, calibration, and
 * detection together.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "esp_idf_runtime.h"

#include "csi_format.h"
#include "csi_platform_config.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "espectre_log.h"
#include "high_accuracy_detector.h"
#include "lightweight_detector.h"
#include "lwip/inet.h"
#include "runtime_config_utils.h"
#include "runtime_detector_store.h"
#include "runtime_motion_hits_store.h"
#include "runtime_time.h"
#include "runtime_traffic_mode_store.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>

namespace espectre {

namespace {

static const char *const RUNTIME_TAG = "espectre.runtime";
static constexpr uint32_t CSI_ENABLE_SETTLE_MS = 100U;

}  // namespace

void EspIdfRuntime::notify_threshold_if_changed_(float threshold) {
  if (threshold == snapshot_.threshold) {
    return;
  }
  snapshot_.threshold = threshold;
  config_.segmentation_threshold = threshold;
  if (listener_ != nullptr) {
    listener_->on_threshold_changed(snapshot_);
  }
}

void EspIdfRuntime::update_live_telemetry_callback_() {
  if (live_telemetry_enabled_) {
    csi_pipeline_.set_live_telemetry_callback([this](float movement, float threshold) {
      notify_threshold_if_changed_(threshold);
      if (listener_ != nullptr) {
        listener_->on_live_telemetry(movement, threshold);
      }
    });
  } else {
    csi_pipeline_.set_live_telemetry_callback({});
  }
}

EspIdfRuntime::EspIdfRuntime(const RuntimeConfig &config)
    : EspIdfRuntimeBase(config, RUNTIME_TAG, "Unknown runtime fault"),
      csi_traffic_service_(traffic_generator_, traffic_ingress_) {
  initialize_runtime_state_();
}

EspIdfRuntime::EspIdfRuntime(const RuntimeConfig &config,
                             ICsiTrafficGenerator &traffic_generator,
                             ICsiTrafficIngress &traffic_ingress)
    : EspIdfRuntimeBase(config, RUNTIME_TAG, "Unknown runtime fault"),
      csi_traffic_service_(traffic_generator, traffic_ingress) {
  initialize_runtime_state_();
}

void EspIdfRuntime::initialize_runtime_state_() {
  detection_timing_supported_ = true;
  snapshot_.threshold = config_.segmentation_threshold;
  snapshot_.subcarrier_source = RuntimeSubcarrierSource::FIXED_DEFAULT;
  // The sensing runtime owns a detector, so it can retune and recalibrate it,
  // and it drives the live-telemetry callback used by transport adapters.
  capabilities_.supports_runtime_threshold_updates = true;
  capabilities_.supports_runtime_motion_hits_updates = true;
  capabilities_.supports_manual_recalibration = true;
  capabilities_.supports_live_telemetry = true;
  capabilities_.supports_extended_diagnostics = true;
  capabilities_.supports_traffic_control = true;
  capabilities_.supports_runtime_detector_selection = config_.runtime_detector_selection_enabled;
  // Raw CSI uses the same ESP-IDF capture boundary on every supported chip.
  capabilities_.supports_raw_csi = true;
}

bool EspIdfRuntime::setup() {
  if (setup_complete_) {
    return true;
  }

  csi_receive_path_refresh_required_ = false;
  csi_receive_path_refresh_in_progress_ = false;

  const RuntimeConfigError config_error = validate_runtime_config(config_);
  if (config_error != RuntimeConfigError::NONE) {
    notify_fault_(runtime_config_error_message(config_error));
    return false;
  }

  ESPECTRE_LOGI(RUNTIME_TAG, "Initializing ESPectre runtime...");

  if (config_.runtime_detector_selection_enabled) {
    DetectionAlgorithm saved_algorithm = config_.detection_algorithm;
    bool has_saved_value = false;
    const esp_err_t err = load_runtime_detection_algorithm(&saved_algorithm, &has_saved_value);
    if (err != ESP_OK) {
      ESPECTRE_LOGW(RUNTIME_TAG, "Failed to load persisted detector: %s", esp_err_to_name(err));
    } else if (has_saved_value) {
      config_.detection_algorithm = saved_algorithm;
      config_.segmentation_threshold = runtime_default_threshold(saved_algorithm);
      snapshot_.threshold = config_.segmentation_threshold;
    }
  }

  bool has_saved_csi_traffic_mode = false;
  CsiTrafficMode saved_csi_traffic_mode = config_.csi_traffic_mode;
  const esp_err_t csi_traffic_err =
      load_runtime_csi_traffic_mode(&saved_csi_traffic_mode, &has_saved_csi_traffic_mode);
  if (csi_traffic_err != ESP_OK) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Failed to load persisted CSI traffic mode: %s", esp_err_to_name(csi_traffic_err));
  } else if (has_saved_csi_traffic_mode) {
    config_.csi_traffic_mode = saved_csi_traffic_mode;
  }

  bool has_saved_generator_mode = false;
  RuntimeTrafficMode saved_generator_mode = config_.traffic_generator_mode;
  const esp_err_t generator_err =
      load_runtime_traffic_generator_mode(&saved_generator_mode, &has_saved_generator_mode);
  if (generator_err != ESP_OK) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Failed to load persisted traffic generator mode: %s", esp_err_to_name(generator_err));
  } else if (has_saved_generator_mode) {
    config_.traffic_generator_mode = saved_generator_mode;
  }

  uint8_t saved_motion_on_hits = config_.motion_on_hits;
  uint8_t saved_motion_off_hits = config_.motion_off_hits;
  bool has_saved_motion_hits = false;
  const esp_err_t motion_hits_err =
      load_runtime_motion_hits(&saved_motion_on_hits, &saved_motion_off_hits, &has_saved_motion_hits);
  if (motion_hits_err != ESP_OK) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Failed to load persisted motion hits: %s", esp_err_to_name(motion_hits_err));
  } else if (has_saved_motion_hits) {
    config_.motion_on_hits = saved_motion_on_hits;
    config_.motion_off_hits = saved_motion_off_hits;
  }

  if (!configure_detector_()) {
    return false;
  }

  // Traffic ownership is explicit in csi_traffic_mode; the positive target
  // only defines cadence.
  csi_traffic_service_.init(to_csi_traffic_config(config_));

  csi_pipeline_.init(detector_.get());
  csi_pipeline_.set_evaluation_interval_ms(config_.evaluation_interval_ms);
  if (!csi_pipeline_.set_csi_target_pps(config_.csi_target_pps) ||
      !csi_pipeline_.set_segmentation_window_size_ms(config_.segmentation_window_size_ms)) {
    notify_fault_("Failed to allocate temporal CSI sampler");
    return false;
  }
  csi_pipeline_.set_motion_hit_thresholds(config_.motion_on_hits, config_.motion_off_hits);
  csi_pipeline_.set_channel_change_callback([this](uint8_t previous_channel, uint8_t current_channel) {
    on_csi_channel_changed_(previous_channel, current_channel);
  });
  update_live_telemetry_callback_();

  if (wifi_lifecycle_.register_handlers([this](const esp_netif_ip_info_t &ip_info) {
                                          on_wifi_connected_(ip_info);
                                        },
                                        [this]() { on_wifi_disconnected_(); },
                                        config_.wifi_band_policy) != ESP_OK) {
    notify_fault_("Failed to register Wi-Fi handlers");
    return false;
  }

  wifi_ready_ = false;
  wifi_ip_info_ = {};
  wifi_rssi_dbm_ = INT8_MIN;
  wifi_channel_ = 0U;
  setup_complete_ = true;
  performance_diagnostics_.reset();
  return true;
}

void EspIdfRuntime::shutdown() {
  if (!setup_complete_) {
    return;
  }

  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    (void) stop_raw_collection(RawCsiStopReason::SHUTDOWN);
  }
  set_services_armed(false);
  on_wifi_disconnected_();
  csi_receive_path_refresh_required_ = false;
  csi_receive_path_refresh_in_progress_ = false;
  wifi_lifecycle_.unregister_handlers();
  setup_complete_ = false;
}

void EspIdfRuntime::loop() {
  RuntimePerformanceLoopScope performance_scope(performance_diagnostics_);
  if (wifi_lifecycle_.process_pending_events() != ESP_OK) {
    notify_fault_("Wi-Fi lifecycle init failed");
  }
  bool calibration_success = false;
  if (calibration_finished_event_.take(calibration_success)) {
    finish_threshold_calibration_(calibration_success);
  }
  csi_pipeline_.loop();
  refresh_wifi_association_from_csi_();
  // Detector-owned adaptation is a control-plane change, so keep the runtime
  // snapshot and listener event current even when high-rate live telemetry is
  // disabled because no frontend consumer is watching it.
  if (detector_ != nullptr) {
    notify_threshold_if_changed_(detector_->get_threshold());
  }
  // Keep the active traffic source healthy while raw capture bypasses the
  // sensing sampler. In external mode this drains the non-blocking UDP socket;
  // otherwise its receive queue fills during long raw sessions and the marker
  // traffic path cannot recover cleanly.
  csi_traffic_service_.loop();
  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    return;
  }
  csi_pipeline_.heartbeat_if_due(monotonic_now_ms());
  DetectionTimingStats detection_timing;
  if (csi_pipeline_.take_detection_timing(&detection_timing)) {
    performance_diagnostics_.record_detection_timing(detection_timing.duration_sum_us,
                                                      detection_timing.samples,
                                                      detection_timing.minimum_us,
                                                      detection_timing.maximum_us);
  }
}

RuntimeDiagnosticsSnapshot EspIdfRuntime::get_diagnostics() const {
  RuntimeDiagnosticsSnapshot diagnostics = EspIdfRuntimeBase::get_diagnostics();
  diagnostics.wifi_rssi_dbm = wifi_rssi_dbm_;
  diagnostics.wifi_channel = wifi_channel_;
  diagnostics.traffic_packets_total = csi_traffic_service_.get_traffic_packets_total();
  diagnostics.csi_callbacks_total = csi_pipeline_.capture_callback_invocations_total();
  diagnostics.csi_classified_total = csi_pipeline_.traffic_classified_packets_total();
  diagnostics.csi_provenance_rejected_total = csi_pipeline_.traffic_rejected_packets_total();
  diagnostics.csi_accepted_total = csi_pipeline_.accepted_packets_total();
  diagnostics.csi_admitted_total = csi_pipeline_.detector_admitted_packets_total();
  diagnostics.csi_filtered_total = csi_pipeline_.capture_filtered_packets_total();
  diagnostics.csi_pending_frame_drops_total = csi_pipeline_.pending_frame_drops_total();
  diagnostics.csi_missing_slots_total = csi_pipeline_.detector_missing_slots_total();
  diagnostics.csi_excess_total = csi_pipeline_.detector_excess_packets_total();
  diagnostics.csi_stale_total = csi_pipeline_.detector_stale_packets_total();
  diagnostics.csi_out_of_order_total = csi_pipeline_.detector_out_of_order_packets_total();
  diagnostics.csi_occupancy_slots = csi_pipeline_.detector_window_occupancy_slots();
  diagnostics.csi_window_slots = csi_pipeline_.detector_window_slots();
  diagnostics.csi_pending_frames = static_cast<uint32_t>(csi_pipeline_.pending_frame_count());
  diagnostics.csi_pending_frame_capacity =
      static_cast<uint32_t>(csi_pipeline_.pending_frame_capacity());
  return diagnostics;
}

const RuntimeDiagnosticsSample *EspIdfRuntime::get_diagnostics_sample() const {
  return &latest_diagnostics_;
}

void EspIdfRuntime::set_services_armed(bool armed) {
  if (services_armed_ == armed) {
    return;
  }

  services_armed_ = armed;
  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    ESPECTRE_LOGI(RUNTIME_TAG, "Deferred sensing mutation until raw collection stops");
    return;
  }
  if (!setup_complete_) {
    return;
  }

  if (!services_armed_) {
    ESPECTRE_LOGI(RUNTIME_TAG, "CSI services disarmed");
    invalidate_csi_receive_path_refresh_();
    csi_receive_path_refresh_required_ =
        csi_receive_path_refresh_required_ || csi_pipeline_.is_enabled();
    stop_sensing_services_();
    return;
  }

  if (wifi_ready_ && wifi_ip_info_.ip.addr != 0U) {
    ESPECTRE_LOGI(RUNTIME_TAG, "CSI services armed, starting capture");
    maybe_resume_sensing_after_wifi_reconfigure_();
  } else {
    ESPECTRE_LOGI(RUNTIME_TAG, "CSI services armed, waiting for Wi-Fi IP");
  }
}

void EspIdfRuntime::set_live_telemetry_enabled(bool enabled) {
  live_telemetry_enabled_ = enabled;
  update_live_telemetry_callback_();
}

bool EspIdfRuntime::set_threshold_runtime(float threshold) {
  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    return false;
  }
  if (!validate_runtime_threshold_for_algorithm(threshold, config_.detection_algorithm)) {
    ESPECTRE_LOGW(RUNTIME_TAG,
             "Rejected invalid runtime threshold: %.6f (detector=%s max=%.3f)",
             threshold,
             detection_algorithm_name(config_.detection_algorithm),
             static_cast<double>(runtime_threshold_max(config_.detection_algorithm)));
    return false;
  }
  if (!csi_pipeline_.set_threshold(threshold)) {
    return false;
  }
  config_.segmentation_threshold = threshold;
  snapshot_.threshold = threshold;
  if (listener_ != nullptr) {
    listener_->on_threshold_changed(snapshot_);
  }
  ESPECTRE_LOGD(RUNTIME_TAG, "Threshold updated to %.6f (session-only, recalculated at boot)", threshold);
  return true;
}

bool EspIdfRuntime::set_motion_hits_runtime(uint8_t motion_on_hits, uint8_t motion_off_hits) {
  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    return false;
  }
  if (motion_on_hits < RUNTIME_MOTION_HITS_MIN || motion_on_hits > RUNTIME_MOTION_HITS_MAX ||
      motion_off_hits < RUNTIME_MOTION_HITS_MIN || motion_off_hits > RUNTIME_MOTION_HITS_MAX) {
    ESPECTRE_LOGW(RUNTIME_TAG,
             "Rejected invalid motion hits on=%u off=%u (range=%u-%u)",
             static_cast<unsigned>(motion_on_hits),
             static_cast<unsigned>(motion_off_hits),
             static_cast<unsigned>(RUNTIME_MOTION_HITS_MIN),
             static_cast<unsigned>(RUNTIME_MOTION_HITS_MAX));
    return false;
  }

  const esp_err_t persist_err = save_runtime_motion_hits(motion_on_hits, motion_off_hits);
  if (persist_err != ESP_OK) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Failed to persist motion hits: %s", esp_err_to_name(persist_err));
    return false;
  }

  csi_pipeline_.set_motion_hit_thresholds(motion_on_hits, motion_off_hits, true);
  config_.motion_on_hits = motion_on_hits;
  config_.motion_off_hits = motion_off_hits;
  ESPECTRE_LOGI(RUNTIME_TAG,
           "Motion hit thresholds updated: on=%u off=%u",
           static_cast<unsigned>(motion_on_hits),
           static_cast<unsigned>(motion_off_hits));
  return true;
}

bool EspIdfRuntime::set_csi_traffic_mode_runtime(CsiTrafficMode mode) {
  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    return false;
  }
  if (!runtime_csi_traffic_mode_valid_for_profile(RuntimeProfile::SENSING, mode)) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Invalid CSI traffic mode for sensing firmware");
    return false;
  }
  if (mode == config_.csi_traffic_mode) {
    return true;
  }
  const RuntimeConfig previous_config = config_;
  config_.csi_traffic_mode = mode;
  if (!apply_traffic_runtime_config_(true, false)) {
    restore_traffic_runtime_config_(previous_config);
    return false;
  }
  const esp_err_t persist_err = save_runtime_csi_traffic_mode(mode);
  if (persist_err != ESP_OK) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Failed to persist CSI traffic mode: %s", esp_err_to_name(persist_err));
    restore_traffic_runtime_config_(previous_config);
    return false;
  }
  if (config_.detection_algorithm == DetectionAlgorithm::LIGHTWEIGHT) {
    (void) trigger_recalibration();
  }
  ESPECTRE_LOGI(RUNTIME_TAG, "CSI traffic mode updated to %s", csi_traffic_mode_name(mode));
  return true;
}

bool EspIdfRuntime::set_traffic_generator_mode_runtime(RuntimeTrafficMode mode) {
  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    return false;
  }
  if (!runtime_traffic_mode_valid(mode)) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Invalid traffic generator mode");
    return false;
  }
  if (mode == config_.traffic_generator_mode) {
    return true;
  }
  const RuntimeConfig previous_config = config_;
  config_.traffic_generator_mode = mode;
  const bool generator_active = config_.csi_traffic_mode == CsiTrafficMode::INTERNAL;
  if (generator_active && !apply_traffic_runtime_config_(true, false)) {
    restore_traffic_runtime_config_(previous_config);
    return false;
  }
  const esp_err_t persist_err = save_runtime_traffic_generator_mode(mode);
  if (persist_err != ESP_OK) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Failed to persist traffic generator mode: %s", esp_err_to_name(persist_err));
    restore_traffic_runtime_config_(previous_config);
    return false;
  }
  if (generator_active && config_.detection_algorithm == DetectionAlgorithm::LIGHTWEIGHT) {
    (void) trigger_recalibration();
  }
  ESPECTRE_LOGI(RUNTIME_TAG, "Traffic generator mode updated to %s", traffic_mode_name(mode));
  return true;
}

bool EspIdfRuntime::set_detection_algorithm_runtime(DetectionAlgorithm algorithm) {
  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    return false;
  }
  if (!capabilities_.supports_runtime_detector_selection ||
      !runtime_detection_algorithm_valid(algorithm)) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Runtime detector selection is unavailable or invalid");
    return false;
  }
  if (algorithm == config_.detection_algorithm) {
    return true;
  }

  const float threshold = runtime_default_threshold(algorithm);
  std::unique_ptr<BaseDetector> next_detector =
      make_detector_(algorithm, threshold, resolved_window_packets_);
  if (next_detector == nullptr) {
    notify_fault_("Failed to configure runtime detector");
    return false;
  }
  const esp_err_t persist_err = save_runtime_detection_algorithm(algorithm);
  if (persist_err != ESP_OK) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Failed to persist detector: %s", esp_err_to_name(persist_err));
    return false;
  }

  cancel_calibration_(true);
  detector_ = std::move(next_detector);
  csi_pipeline_.set_detector(detector_.get());
  config_.detection_algorithm = algorithm;
  config_.segmentation_threshold = threshold;
  snapshot_.detector_name = detection_algorithm_name(algorithm);
  snapshot_.threshold = threshold;
  snapshot_.startup_threshold = 0.0f;
  snapshot_.movement_metric = 0.0f;
  snapshot_.motion_state = MotionState::IDLE;

  if (listener_ != nullptr) {
    listener_->on_detector_changed(snapshot_);
    listener_->on_threshold_changed(snapshot_);
  }
  ESPECTRE_LOGI(RUNTIME_TAG, "Detector changed to %s", detection_algorithm_name(algorithm));

  if (algorithm == DetectionAlgorithm::LIGHTWEIGHT && csi_pipeline_.is_enabled()) {
    return start_calibration_();
  }
  return true;
}

bool EspIdfRuntime::trigger_recalibration() {
  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    return false;
  }
  if (snapshot_.calibrating) {
    ESPECTRE_LOGW(RUNTIME_TAG, "Calibration already in progress");
    return false;
  }

  ESPECTRE_LOGI(RUNTIME_TAG, "Manual recalibration triggered");
  return start_calibration_();
}

bool EspIdfRuntime::is_calibrating() const { return snapshot_.calibrating; }

RuntimeOperationState EspIdfRuntime::operation_state() const {
  return operation_state_.load(std::memory_order_acquire);
}

bool EspIdfRuntime::start_raw_collection(raw_csi_packet_callback_t callback, void *context) {
  if (!capabilities_.supports_raw_csi || callback == nullptr || !setup_complete_ || !services_armed_ ||
      !wifi_ready_ || wifi_ip_info_.ip.addr == 0U ||
      operation_state() != RuntimeOperationState::SENSING) {
    return false;
  }

  operation_state_.store(RuntimeOperationState::RAW_COLLECTION, std::memory_order_release);
  cancel_calibration_(true);
  snapshot_.ready_to_publish = false;
  snapshot_.motion_state = MotionState::IDLE;
  csi_pipeline_.set_motion_state_callback({});
  csi_pipeline_.set_live_telemetry_callback({});
  performance_diagnostics_.reset();
  if (!csi_pipeline_.start_raw_capture(callback, context)) {
    operation_state_.store(RuntimeOperationState::SENSING, std::memory_order_release);
    csi_traffic_service_.stop();
    (void) csi_pipeline_.disable();
    update_live_telemetry_callback_();
    if (services_armed_) start_sensing_services_(wifi_ip_info_);
    return false;
  }
  refresh_csi_local_identity_(wifi_ip_info_.ip.addr);

  if (!csi_pipeline_.is_enabled()) {
    const CsiCaptureProfile profile = sensing_capture_profile_();
    snapshot_.csi_capture_profile = profile;
    const esp_err_t err = csi_pipeline_.enable({}, profile);
    if (err != ESP_OK) {
      csi_pipeline_.stop_raw_capture();
      update_live_telemetry_callback_();
      operation_state_.store(RuntimeOperationState::SENSING, std::memory_order_release);
      csi_traffic_service_.stop();
      (void) csi_pipeline_.disable();
      if (services_armed_) start_sensing_services_(wifi_ip_info_);
      char message[96];
      std::snprintf(message, sizeof(message), "Failed to enable raw CSI: %s", esp_err_to_name(err));
      notify_fault_(message);
      return false;
    }
  }

  ESPECTRE_LOGI(RUNTIME_TAG, "Entered raw CSI collection mode");
  return true;
}

bool EspIdfRuntime::stop_raw_collection(RawCsiStopReason reason) {
  if (operation_state() != RuntimeOperationState::RAW_COLLECTION) {
    return false;
  }

  operation_state_.store(RuntimeOperationState::SENSING, std::memory_order_release);
  csi_pipeline_.stop_raw_capture();
  csi_pipeline_.set_traffic_filter({});
  // Classic ESP32 can stop delivering CSI when traffic spans a CSI disable/
  // enable transition. Quiesce the source before disabling capture so every
  // new session starts with CSI armed before traffic resumes.
  csi_traffic_service_.stop();
  (void) csi_pipeline_.disable();
  cancel_calibration_(false);
  snapshot_.ready_to_publish = false;
  snapshot_.motion_state = MotionState::IDLE;
  update_live_telemetry_callback_();

  ESPECTRE_LOGI(RUNTIME_TAG, "Exited raw CSI collection mode: %u", static_cast<unsigned>(reason));
  if (services_armed_ && wifi_ready_ && wifi_ip_info_.ip.addr != 0U) {
    start_sensing_services_(wifi_ip_info_);
  } else if (listener_ != nullptr) {
    listener_->on_motion_state_changed(snapshot_);
  }
  return true;
}

CsiCaptureProfile EspIdfRuntime::sensing_capture_profile_() const {
  const bool requires_lltf = config_.csi_traffic_mode == CsiTrafficMode::INTERNAL &&
                             config_.traffic_generator_mode == RuntimeTrafficMode::WIFI_RAW;
  return select_csi_capture_profile(wifi_channel_, requires_lltf);
}

bool EspIdfRuntime::apply_traffic_runtime_config_(bool restart_service, bool recalibrate_if_active) {
  if (restart_service) {
    csi_traffic_service_.stop();
  }
  csi_traffic_service_.init(to_csi_traffic_config(config_));
  if (!setup_complete_ || !wifi_ready_ || !services_armed_ || wifi_ip_info_.ip.addr == 0U || !csi_pipeline_.is_enabled()) {
    return true;
  }
  const CsiCaptureProfile profile = sensing_capture_profile_();
  if (profile != csi_pipeline_.capture_profile()) {
    cancel_calibration_(true);
    const esp_err_t err = csi_pipeline_.reconfigure_capture(profile);
    if (err != ESP_OK) {
      notify_fault_("Failed to change CSI capture profile");
      return false;
    }
    snapshot_.csi_capture_profile = profile;
    snapshot_.motion_state = MotionState::IDLE;
    snapshot_.movement_metric = 0.0f;
    vTaskDelay(pdMS_TO_TICKS(CSI_ENABLE_SETTLE_MS));
  }
  refresh_csi_local_identity_(wifi_ip_info_.ip.addr);
  if (!csi_traffic_service_.start(wifi_ip_info_.gw.addr)) {
    notify_fault_("Failed to start CSI traffic service");
    return false;
  }
  if (recalibrate_if_active && config_.detection_algorithm == DetectionAlgorithm::LIGHTWEIGHT) {
    (void) trigger_recalibration();
  }
  return true;
}

void EspIdfRuntime::restore_traffic_runtime_config_(const RuntimeConfig &previous_config) {
  config_ = previous_config;
  if (!apply_traffic_runtime_config_(true, true)) {
    notify_fault_("Failed to restore CSI traffic configuration");
  }
}

bool EspIdfRuntime::configure_detector_() {
  if (!validate_runtime_uint32(config_.csi_target_pps,
                               RUNTIME_CSI_TARGET_PPS_MIN,
                               RUNTIME_CSI_TARGET_PPS_MAX)) {
    notify_fault_("Invalid CSI target PPS");
    return false;
  }
  if (!validate_runtime_uint32(config_.segmentation_window_size_ms,
                               RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_MIN,
                               RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_MAX)) {
    notify_fault_("Invalid detector window duration");
    return false;
  }
  if (!validate_runtime_threshold_for_algorithm(config_.segmentation_threshold,
                                                config_.detection_algorithm)) {
    notify_fault_("Invalid segmentation threshold");
    return false;
  }
  const float threshold = config_.segmentation_threshold;
  snapshot_.threshold = threshold;
  resolved_window_packets_ = static_cast<uint16_t>(temporal_window_slots(
      config_.csi_target_pps, config_.segmentation_window_size_ms));
  detector_ = make_detector_(config_.detection_algorithm, threshold, resolved_window_packets_);

  if (detector_ == nullptr) {
    notify_fault_("Failed to configure detector");
    return false;
  }

  snapshot_.detector_name = detection_algorithm_name(config_.detection_algorithm);
  return true;
}

std::unique_ptr<BaseDetector> EspIdfRuntime::make_detector_(DetectionAlgorithm algorithm,
                                                            float threshold,
                                                            uint16_t window_packets) {
  std::unique_ptr<BaseDetector> detector;
  if (algorithm == DetectionAlgorithm::HIGH_ACCURACY) {
    detector.reset(new (std::nothrow) HighAccuracyDetector(window_packets, threshold));
  } else if (algorithm == DetectionAlgorithm::LIGHTWEIGHT) {
    detector.reset(new (std::nothrow) LightweightDetector(window_packets, threshold));
  }
  if (detector != nullptr && !detector->is_valid()) {
    detector.reset();
  }
  if (detector != nullptr) {
    detector->configure_lowpass(config_.lowpass_enabled, config_.lowpass_cutoff);
    detector->configure_hampel(config_.hampel_enabled, config_.hampel_window, config_.hampel_threshold);
  }
  return detector;
}

void EspIdfRuntime::cancel_calibration_(bool notify_listener) {
  const bool was_calibrating = snapshot_.calibrating;
  threshold_calibration_active_.store(false, std::memory_order_relaxed);
  calibration_finished_event_.clear();
  csi_pipeline_.set_packet_interceptor(nullptr, nullptr);
  threshold_calibrator_.reset();
  snapshot_.calibrating = false;
  snapshot_.calibration_packets = 0U;
  snapshot_.calibration_target_packets = 0U;
  if (was_calibrating && notify_listener && listener_ != nullptr) {
    listener_->on_calibration_finished(snapshot_, false);
  }
}

void EspIdfRuntime::on_wifi_connected_(const esp_netif_ip_info_t &ip_info) {
  if (ip_info.ip.addr == 0U) {
    return;
  }

  wifi_ready_ = true;
  wifi_ip_info_ = ip_info;
  wifi_ap_record_t ap_info{};
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    wifi_rssi_dbm_ = ap_info.rssi;
    wifi_channel_ = ap_info.primary;
  } else {
    wifi_rssi_dbm_ = INT8_MIN;
    wifi_channel_ = 0U;
  }
  if (!services_armed_) {
    ESPECTRE_LOGI(RUNTIME_TAG, "Wi-Fi connected, CSI services not ready to resume");
    return;
  }

  maybe_resume_sensing_after_wifi_reconfigure_();
}

void EspIdfRuntime::on_wifi_disconnected_() {
  wifi_ready_ = false;
  wifi_ip_info_ = {};
  wifi_rssi_dbm_ = INT8_MIN;
  wifi_channel_ = 0U;
  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    (void) stop_raw_collection(RawCsiStopReason::WIFI_LOST);
    return;
  }
  invalidate_csi_receive_path_refresh_();
  csi_receive_path_refresh_required_ =
      csi_receive_path_refresh_required_ || csi_pipeline_.is_enabled();
  stop_sensing_services_();
}

void EspIdfRuntime::invalidate_csi_receive_path_refresh_() {
  if (!csi_receive_path_refresh_in_progress_) {
    return;
  }

  wifi_lifecycle_.cancel_csi_receive_path_refresh();
  csi_receive_path_refresh_in_progress_ = false;
  csi_receive_path_refresh_required_ = true;
}

void EspIdfRuntime::maybe_resume_sensing_after_wifi_reconfigure_() {
  if (!services_armed_ || !wifi_ready_ ||
      wifi_ip_info_.ip.addr == 0U || csi_receive_path_refresh_in_progress_) {
    return;
  }
  if (!csi_receive_path_refresh_required_) {
    start_sensing_services_(wifi_ip_info_);
    return;
  }

  csi_receive_path_refresh_required_ = false;
  csi_receive_path_refresh_in_progress_ = true;
  const esp_err_t refresh_err = wifi_lifecycle_.refresh_csi_receive_path(
      [this](esp_err_t result) { finish_csi_receive_path_refresh_(result); });
  if (refresh_err == ESP_OK) {
    ESPECTRE_LOGI(RUNTIME_TAG,
                  "Starting post-reassociation Wi-Fi scan to refresh the CSI receive path");
    return;
  }

  csi_receive_path_refresh_in_progress_ = false;
  ESPECTRE_LOGW(RUNTIME_TAG,
                "Could not start post-reassociation Wi-Fi scan: %s; resuming sensing",
                esp_err_to_name(refresh_err));
  start_sensing_services_(wifi_ip_info_);
}

void EspIdfRuntime::finish_csi_receive_path_refresh_(esp_err_t result) {
  csi_receive_path_refresh_in_progress_ = false;
  if (result == ESP_OK) {
    ESPECTRE_LOGI(RUNTIME_TAG, "Post-reassociation Wi-Fi scan completed; resuming sensing");
  } else {
    csi_receive_path_refresh_required_ = true;
    ESPECTRE_LOGW(RUNTIME_TAG,
                  "Post-reassociation Wi-Fi scan failed: %s; retrying refresh",
                  esp_err_to_name(result));
    if (services_armed_ && wifi_ready_ && wifi_ip_info_.ip.addr != 0U) {
      maybe_resume_sensing_after_wifi_reconfigure_();
    }
    return;
  }
  if (services_armed_ && wifi_ready_ && wifi_ip_info_.ip.addr != 0U) {
    start_sensing_services_(wifi_ip_info_);
  }
}

void EspIdfRuntime::refresh_wifi_association_from_csi_() {
  if (!wifi_ready_) return;
  const int8_t rssi_dbm = csi_pipeline_.last_rssi_dbm();
  const uint8_t channel = csi_pipeline_.last_channel();
  if (rssi_dbm != INT8_MIN) wifi_rssi_dbm_ = rssi_dbm;
  if (channel != 0U) wifi_channel_ = channel;
}

void EspIdfRuntime::start_sensing_services_(const esp_netif_ip_info_t &ip_info) {
  snapshot_.motion_state = MotionState::IDLE;
  snapshot_.ready_to_publish = false;

  csi_pipeline_.set_motion_state_callback([this](MotionState state) {
    snapshot_.motion_state = state;
    if (snapshot_.ready_to_publish && listener_ != nullptr) {
      listener_->on_motion_state_changed(snapshot_);
    }
  });
  refresh_csi_local_identity_(ip_info.ip.addr);

  if (!csi_pipeline_.is_enabled()) {
    // Keep associated CSI capture strictly non-promiscuous, including after a
    // station stop/start cycle has rebuilt the Wi-Fi control block.
    const esp_err_t promiscuous_err = esp_wifi_set_promiscuous(false);
    if (promiscuous_err != ESP_OK) {
      char message[112];
      std::snprintf(message, sizeof(message), "Failed to disable promiscuous mode before CSI: %s",
                    esp_err_to_name(promiscuous_err));
      notify_fault_(message);
      return;
    }

    const CsiCaptureProfile profile = sensing_capture_profile_();
    snapshot_.csi_capture_profile = profile;
    const esp_err_t err = csi_pipeline_.enable([this](MotionState state, uint32_t packets_received) {
      snapshot_.motion_state = state;
      snapshot_.movement_metric = detector_ != nullptr ? detector_->get_motion_metric() : 0.0f;
      if (detector_ != nullptr) {
        notify_threshold_if_changed_(detector_->get_threshold());
      }
      snapshot_.link_rssi_dbm = csi_pipeline_.last_rssi_dbm();
      snapshot_.link_channel = csi_pipeline_.last_channel();

      if (snapshot_.ready_to_publish) {
        log_periodic_status_(packets_received);
        if (listener_ != nullptr) {
          listener_->on_periodic_update(snapshot_, packets_received);
        }
      }
    }, profile);
    if (err != ESP_OK) {
      char message[96];
      std::snprintf(message, sizeof(message), "Failed to enable CSI: %s", esp_err_to_name(err));
      notify_fault_(message);
      return;
    }
  }

  // esp_wifi_set_csi(true) returns before the classic ESP32 receive path is
  // always ready to associate new traffic with the CSI callback. Starting the
  // source in the same scheduler slice reproduces esp-csi#247 intermittently.
  // Yield once after arming so managed traffic cannot predate driver readiness.
  vTaskDelay(pdMS_TO_TICKS(CSI_ENABLE_SETTLE_MS));

  if (!csi_traffic_service_.is_running() && !csi_traffic_service_.start(ip_info.gw.addr)) {
    notify_fault_("Failed to start CSI traffic service");
    return;
  }

  start_calibration_(false);
  snapshot_.ready_to_publish = true;
  reset_periodic_status_logger_();
}

void EspIdfRuntime::stop_sensing_services_() {
  cancel_calibration_(false);
  csi_pipeline_.set_traffic_filter({});
  csi_traffic_service_.stop();
  csi_pipeline_.disable();
  snapshot_.ready_to_publish = false;
  snapshot_.motion_state = MotionState::IDLE;
  if (listener_ != nullptr) {
    listener_->on_motion_state_changed(snapshot_);
  }
}

void EspIdfRuntime::on_csi_channel_changed_(uint8_t previous_channel, uint8_t current_channel) {
  if (operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    ESPECTRE_LOGW(RUNTIME_TAG,
             "Ending raw collection after Wi-Fi channel change: %u -> %u",
             static_cast<unsigned>(previous_channel),
             static_cast<unsigned>(current_channel));
    (void) stop_raw_collection(RawCsiStopReason::CHANNEL_CHANGED);
    return;
  }
  if (!wifi_ready_ || !services_armed_ || wifi_ip_info_.ip.addr == 0U || !csi_pipeline_.is_enabled()) {
    return;
  }

  ESPECTRE_LOGW(RUNTIME_TAG,
           "Rearming CSI session after Wi-Fi channel change: %u -> %u",
           static_cast<unsigned>(previous_channel),
           static_cast<unsigned>(current_channel));

  const esp_netif_ip_info_t ip_info = wifi_ip_info_;
  cancel_calibration_(false);
  csi_pipeline_.set_traffic_filter({});
  csi_traffic_service_.stop();
  const esp_err_t disable_err = csi_pipeline_.disable();
  snapshot_.ready_to_publish = false;
  snapshot_.motion_state = MotionState::IDLE;
  if (listener_ != nullptr) {
    listener_->on_motion_state_changed(snapshot_);
  }
  if (disable_err != ESP_OK) {
    char message[96];
    std::snprintf(message, sizeof(message), "Failed to rearm CSI after channel change: %s",
                  esp_err_to_name(disable_err));
    notify_fault_(message);
    return;
  }

  on_wifi_connected_(ip_info);
}

bool EspIdfRuntime::start_calibration_(bool reset_high_accuracy_threshold) {
  snapshot_.subcarrier_source = RuntimeSubcarrierSource::FIXED_DEFAULT;

  if (config_.detection_algorithm == DetectionAlgorithm::HIGH_ACCURACY) {
    const float threshold = reset_high_accuracy_threshold
                                ? runtime_default_threshold(DetectionAlgorithm::HIGH_ACCURACY)
                                : config_.segmentation_threshold;
    if (detector_ != nullptr) {
      detector_->set_threshold(threshold);
    }
    config_.segmentation_threshold = threshold;
    snapshot_.threshold = threshold;
    snapshot_.startup_threshold = threshold;
    snapshot_.calibrating = false;
    snapshot_.calibration_packets = 0U;
    snapshot_.calibration_target_packets = 0U;
    if (listener_ != nullptr) {
      listener_->on_threshold_changed(snapshot_);
      listener_->on_calibration_finished(snapshot_, true);
    }
    return true;
  }

  const uint32_t calibration_duration_ms =
      config_.segmentation_window_size_ms * CALIBRATION_NUM_WINDOWS;
  uint32_t calibration_target_packets = temporal_window_slots(
      config_.csi_target_pps, calibration_duration_ms);
  if (calibration_target_packets > UINT16_MAX) {
    calibration_target_packets = UINT16_MAX;
  }

  if (!snapshot_.calibrating) {
    snapshot_.calibrating = true;
    snapshot_.calibration_packets = 0U;
    snapshot_.calibration_target_packets = static_cast<uint16_t>(calibration_target_packets);
    if (listener_ != nullptr) {
      listener_->on_calibration_started(snapshot_);
    }
  }

  // Calibrate on the runtime detector itself (cold-cleared first), so the
  // observed metric matches the configured algorithm. Mirrors the Python
  // runtime calibration flow.
  threshold_calibrator_.reset(new (std::nothrow) StartupThresholdCalibrator());
  if (!threshold_calibrator_) {
    snapshot_.calibrating = false;
    snapshot_.calibration_packets = 0U;
    snapshot_.calibration_target_packets = 0U;
    notify_fault_("Failed to allocate startup calibrator");
    return false;
  }
  threshold_calibrator_->begin(static_cast<uint16_t>(calibration_target_packets),
                               detector_ != nullptr && detector_->startup_gate_enabled());
  calibration_finished_event_.clear();
  snapshot_.calibration_packets = 0U;
  snapshot_.calibration_target_packets = static_cast<uint16_t>(calibration_target_packets);
  threshold_calibration_active_.store(true, std::memory_order_relaxed);
  csi_pipeline_.clear_detector_buffer();
  if (detector_ != nullptr) {
    detector_->on_startup_calibration_begin();
  }
  csi_pipeline_.set_packet_interceptor(&EspIdfRuntime::threshold_calibration_packet_callback_, this);
  ESPECTRE_LOGI(RUNTIME_TAG, "Starting %s threshold calibration with fixed subcarriers",
           detector_ != nullptr ? detector_->get_name() : "detector");
  return true;
}

bool EspIdfRuntime::handle_threshold_calibration_packet_(const int8_t *csi_data, size_t csi_len,
                                                         int8_t rssi_dbm, bool evaluation_due,
                                                         uint32_t packets_in_window,
                                                         bool temporal_reset) {
  if (!threshold_calibration_active_.load(std::memory_order_relaxed) || detector_ == nullptr ||
      !threshold_calibrator_) {
    return false;
  }

  if (temporal_reset) {
    const uint16_t target_packets = threshold_calibrator_->target_packets();
    threshold_calibrator_->begin(target_packets, detector_->startup_gate_enabled());
    snapshot_.calibration_packets = 0U;
    snapshot_.calibration_target_packets = target_packets;
    detector_->on_startup_calibration_begin();
  }

  detector_->process_packet(csi_data, csi_len, snapshot_.fixed_subcarriers.data(),
                            HT20_SELECTED_BAND_SIZE, rssi_dbm);
  // The pipeline owns the cadence, so calibration evaluates exactly when
  // steady-state detection would and the threshold is fitted at the resolution
  // the detector will run at.
  if (!evaluation_due) {
    return true;
  }

  const uint16_t packet_weight =
      static_cast<uint16_t>(std::min<uint32_t>(std::max<uint32_t>(packets_in_window, 1U), UINT16_MAX));
  detector_->update_state();
  if (!detector_->is_ready()) {
    return true;
  }
  threshold_calibrator_->observe(true, detector_->get_motion_metric(), packet_weight);

  snapshot_.calibration_packets = threshold_calibrator_->packet_count();
  snapshot_.calibration_target_packets = threshold_calibrator_->target_packets();

  if (threshold_calibrator_->is_complete()) {
    snapshot_.calibration_packets = snapshot_.calibration_target_packets;
    threshold_calibration_active_.store(false, std::memory_order_relaxed);
    calibration_finished_event_.post(threshold_calibrator_->is_successful());
  }
  return true;
}

bool EspIdfRuntime::threshold_calibration_packet_callback_(void *context,
                                                           const int8_t *csi_data,
                                                           size_t csi_len,
                                                           int8_t rssi_dbm,
                                                           bool evaluation_due,
                                                           uint32_t packets_in_window,
                                                           bool temporal_reset) {
  auto *runtime = static_cast<EspIdfRuntime *>(context);
  return runtime != nullptr &&
         runtime->handle_threshold_calibration_packet_(csi_data, csi_len, rssi_dbm, evaluation_due,
                                                       packets_in_window, temporal_reset);
}

void EspIdfRuntime::finish_threshold_calibration_(bool success) {
  threshold_calibration_active_.store(false, std::memory_order_relaxed);
  csi_pipeline_.set_packet_interceptor(nullptr, nullptr);
  snapshot_.calibrating = false;
  snapshot_.calibration_packets = 0U;
  snapshot_.calibration_target_packets = 0U;

  if (success && threshold_calibrator_) {
    const float auto_factor = detector_ != nullptr
                                  ? detector_->get_startup_threshold_factor()
                                  : DEFAULT_ADAPTIVE_FACTOR;
    const float adaptive_threshold = threshold_calibrator_->threshold_metric() * auto_factor;
    snapshot_.startup_threshold = adaptive_threshold;
    if (detector_ != nullptr) {
      detector_->on_startup_calibration_complete();
    }

    if (detector_ != nullptr) {
      detector_->set_adaptive_threshold(adaptive_threshold);
      const float applied_threshold = detector_->get_threshold();
      config_.segmentation_threshold = applied_threshold;
      snapshot_.startup_threshold = applied_threshold;
      snapshot_.threshold = applied_threshold;
      if (listener_ != nullptr) {
        listener_->on_threshold_changed(snapshot_);
      }
      ESPECTRE_LOGD(RUNTIME_TAG, "Adaptive threshold: %.6f (shared proposal %.6f)",
               applied_threshold, adaptive_threshold);
    }
    csi_pipeline_.clear_detector_buffer();
  }

  if (listener_ != nullptr) {
    listener_->on_calibration_finished(snapshot_, success);
  }
  ESPECTRE_LOGD(RUNTIME_TAG, "Calibration %s", success ? "completed successfully" : "failed");
  threshold_calibrator_.reset();
  reset_periodic_status_logger_();
}

void EspIdfRuntime::log_periodic_status_(uint32_t packets_received) {
  latest_diagnostics_ = diagnostics_sampler_.sample(get_diagnostics(), monotonic_now_ms());
  status_logger_.log_status(RUNTIME_TAG, snapshot_, packets_received, &latest_diagnostics_);
}

void EspIdfRuntime::reset_periodic_status_logger_() {
  status_logger_.reset();
  const RuntimeDiagnosticsSnapshot diagnostics = get_diagnostics();
  const uint32_t now_ms = monotonic_now_ms();
  diagnostics_sampler_.reset(diagnostics, now_ms);
  latest_diagnostics_ = diagnostics_sampler_.sample(diagnostics, now_ms);
}

void EspIdfRuntime::refresh_csi_local_identity_(uint32_t local_ip_addr) {
  CsiFrameFilterConfig filter;
  filter.traffic_mode = config_.csi_traffic_mode;
  filter.internal_mode = config_.traffic_generator_mode;
  filter.local_ip_addr = local_ip_addr;
  filter.gateway_ip_addr = wifi_ip_info_.gw.addr;
  filter.multicast_ip_addr = config_.csi_traffic_multicast_group.empty()
                                 ? 0U
                                 : inet_addr(config_.csi_traffic_multicast_group.c_str());
  filter.external_udp_port = config_.csi_traffic_udp_port;
  filter.internal_icmp_identifier = csi_traffic_service_.internal_icmp_identifier();
  uint8_t mac[6] = {0U, 0U, 0U, 0U, 0U, 0U};
  if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
    std::memcpy(filter.local_mac_addr, mac, sizeof(filter.local_mac_addr));
  }
  csi_pipeline_.set_traffic_filter(filter);
}

}  // namespace espectre
