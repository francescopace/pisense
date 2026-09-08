/*
 * ESPectre - Runtime Frontend Controller
 *
 * Owns runtime lifecycle and exposes a frontend-friendly control surface.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "runtime_frontend_controller.h"

#include "espectre_log.h"
#include "esp_idf_runtime.h"
#include "runtime_config_utils.h"

#include <new>

namespace espectre {

namespace {

static const char *const TAG = "espectre.runtime";

}  // namespace

RuntimeFrontendController::~RuntimeFrontendController() { shutdown(); }

void RuntimeFrontendController::set_config(const RuntimeConfig &config) {
  if (runtime_) {
    return;
  }
  config_ = config;
  snapshot_.threshold = config_.segmentation_threshold;
}

bool RuntimeFrontendController::setup(IRuntimeListener *listener) {
  if (setup_complete_) {
    return true;
  }

  const RuntimeConfigError config_error = validate_runtime_config(config_);
  if (config_error != RuntimeConfigError::NONE) {
    const char *message = runtime_config_error_message(config_error);
    ESPECTRE_LOGE(TAG, "Rejected runtime configuration: %s", message);
    if (listener != nullptr) {
      listener->on_runtime_fault(message);
    }
    return false;
  }

  active_config_ = config_;
  auto *backend = new (std::nothrow) EspIdfRuntime(active_config_);
  if (backend == nullptr) {
    constexpr const char *message = "Failed to allocate runtime backend";
    ESPECTRE_LOGE(TAG, "%s", message);
    if (listener != nullptr) {
      listener->on_runtime_fault(message);
    }
    return false;
  }
  listener_ = listener;
  runtime_.reset(backend);
  runtime_->set_listener(this);
  runtime_->set_services_armed(services_armed_);
  runtime_->set_live_telemetry_enabled(live_telemetry_enabled_);
  if (!runtime_->setup()) {
    runtime_.reset();
    listener_ = nullptr;
    apply_deferred_shutdown_();
    return false;
  }

  active_config_ = backend->effective_config();
  config_ = active_config_;
  snapshot_ = runtime_->get_snapshot();
  last_sensing_ready_ = snapshot_.ready_to_publish;
  capabilities_ = runtime_->get_capabilities();
  setup_complete_ = true;
  apply_deferred_shutdown_();
  return setup_complete_;
}

void RuntimeFrontendController::loop() {
  if (runtime_) {
    runtime_->loop();
    cache_snapshot_(runtime_->get_snapshot());
    if (snapshot_.ready_to_publish != last_sensing_ready_) {
      last_sensing_ready_ = snapshot_.ready_to_publish;
      if (listener_ != nullptr) {
        begin_callback_();
        listener_->on_sensing_readiness_changed(snapshot_);
        end_callback_();
      }
    }
  }
  apply_deferred_shutdown_();
}

void RuntimeFrontendController::shutdown() {
  if (callback_depth_ > 0U) {
    shutdown_requested_ = true;
    return;
  }
  if (runtime_) {
    runtime_->shutdown();
    runtime_.reset();
  }
  listener_ = nullptr;
  setup_complete_ = false;
  shutdown_requested_ = false;
  capabilities_ = {};
  snapshot_.motion_state = MotionState::IDLE;
  snapshot_.calibrating = false;
  snapshot_.ready_to_publish = false;
}

void RuntimeFrontendController::set_services_armed(bool armed) {
  services_armed_ = armed;
  if (runtime_ && runtime_->operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    runtime_->set_services_armed(armed);
    ESPECTRE_LOGI(TAG, "Deferred sensing mutation until raw collection stops");
    return;
  }
  if (runtime_) {
    runtime_->set_services_armed(armed);
    snapshot_ = runtime_->get_snapshot();
  }
  apply_deferred_shutdown_();
}

void RuntimeFrontendController::set_live_telemetry_enabled(bool enabled) {
  live_telemetry_enabled_ = enabled;
  if (runtime_) {
    runtime_->set_live_telemetry_enabled(enabled);
  }
  apply_deferred_shutdown_();
}

void RuntimeFrontendController::quiesce() {
  set_live_telemetry_enabled(false);
  if (runtime_ && runtime_->operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    set_services_armed(false);
    if (!stop_raw_collection(RawCsiStopReason::SHUTDOWN)) {
      ESPECTRE_LOGE(TAG, "Failed to stop raw collection while quiescing the runtime");
    }
    return;
  }
  set_services_armed(false);
}

bool RuntimeFrontendController::set_threshold_runtime(float threshold) {
  const RuntimeConfig &effective_config = runtime_ ? active_config_ : config_;
  if (effective_config.runtime_profile != RuntimeProfile::SENSING ||
      !validate_runtime_threshold_for_algorithm(threshold, effective_config.detection_algorithm)) {
    return false;
  }
  if (runtime_) {
    if (!capabilities_.supports_runtime_threshold_updates ||
        !runtime_->set_threshold_runtime(threshold)) {
      apply_deferred_shutdown_();
      return false;
    }
  } else {
    snapshot_.threshold = threshold;
  }
  if (runtime_) {
    adopt_effective_threshold_(threshold);
  } else {
    config_.segmentation_threshold = threshold;
  }
  snapshot_.threshold = threshold;
  apply_deferred_shutdown_();
  return true;
}

bool RuntimeFrontendController::set_motion_hits_runtime(uint8_t motion_on_hits, uint8_t motion_off_hits) {
  const RuntimeConfig &effective_config = runtime_ ? active_config_ : config_;
  if (effective_config.runtime_profile != RuntimeProfile::SENSING ||
      motion_on_hits < RUNTIME_MOTION_HITS_MIN || motion_on_hits > RUNTIME_MOTION_HITS_MAX ||
      motion_off_hits < RUNTIME_MOTION_HITS_MIN || motion_off_hits > RUNTIME_MOTION_HITS_MAX) {
    return false;
  }
  const bool staged_motion_on_hits =
      runtime_ && config_.motion_on_hits != active_config_.motion_on_hits;
  const bool staged_motion_off_hits =
      runtime_ && config_.motion_off_hits != active_config_.motion_off_hits;
  if (runtime_) {
    if (!capabilities_.supports_runtime_motion_hits_updates ||
        !runtime_->set_motion_hits_runtime(motion_on_hits, motion_off_hits)) {
      apply_deferred_shutdown_();
      return false;
    }
  }
  if (!staged_motion_on_hits) config_.motion_on_hits = motion_on_hits;
  if (!staged_motion_off_hits) config_.motion_off_hits = motion_off_hits;
  if (runtime_) {
    active_config_.motion_on_hits = motion_on_hits;
    active_config_.motion_off_hits = motion_off_hits;
  }
  apply_deferred_shutdown_();
  return true;
}

bool RuntimeFrontendController::set_csi_traffic_mode_runtime(CsiTrafficMode mode) {
  const RuntimeConfig &effective_config = runtime_ ? active_config_ : config_;
  if (!runtime_csi_traffic_mode_valid_for_profile(effective_config.runtime_profile, mode)) {
    return false;
  }
  const bool staged_for_next_setup =
      runtime_ && config_.csi_traffic_mode != active_config_.csi_traffic_mode;
  if (runtime_) {
    if (!capabilities_.supports_traffic_control || !runtime_->set_csi_traffic_mode_runtime(mode)) {
      apply_deferred_shutdown_();
      return false;
    }
  }
  if (!staged_for_next_setup) config_.csi_traffic_mode = mode;
  if (runtime_) active_config_.csi_traffic_mode = mode;
  apply_deferred_shutdown_();
  return true;
}

bool RuntimeFrontendController::set_traffic_generator_mode_runtime(RuntimeTrafficMode mode) {
  if (!runtime_traffic_mode_valid(mode)) {
    return false;
  }
  const bool staged_for_next_setup =
      runtime_ && config_.traffic_generator_mode != active_config_.traffic_generator_mode;
  if (runtime_) {
    if (!capabilities_.supports_traffic_control || !runtime_->set_traffic_generator_mode_runtime(mode)) {
      apply_deferred_shutdown_();
      return false;
    }
  }
  if (!staged_for_next_setup) config_.traffic_generator_mode = mode;
  if (runtime_) active_config_.traffic_generator_mode = mode;
  apply_deferred_shutdown_();
  return true;
}

bool RuntimeFrontendController::set_detection_algorithm_runtime(DetectionAlgorithm algorithm) {
  const RuntimeConfig &effective_config = runtime_ ? active_config_ : config_;
  if (effective_config.runtime_profile != RuntimeProfile::SENSING ||
      !runtime_detection_algorithm_valid(algorithm)) {
    return false;
  }
  if (runtime_) {
    if (!capabilities_.supports_runtime_detector_selection ||
        !runtime_->set_detection_algorithm_runtime(algorithm)) {
      apply_deferred_shutdown_();
      return false;
    }
    snapshot_ = runtime_->get_snapshot();
  } else {
    config_.detection_algorithm = algorithm;
    config_.segmentation_threshold = runtime_default_threshold(algorithm);
    snapshot_.threshold = config_.segmentation_threshold;
    snapshot_.detector_name = detection_algorithm_name(algorithm);
  }
  if (runtime_) {
    adopt_effective_detector_(algorithm);
    adopt_effective_threshold_(snapshot_.threshold);
  } else {
    config_.detection_algorithm = algorithm;
    config_.segmentation_threshold = snapshot_.threshold;
  }
  apply_deferred_shutdown_();
  return true;
}

bool RuntimeFrontendController::trigger_recalibration() {
  if (!capabilities_.supports_manual_recalibration || !runtime_) {
    return false;
  }
  const bool started = runtime_->trigger_recalibration();
  apply_deferred_shutdown_();
  return started;
}

bool RuntimeFrontendController::is_calibrating() const {
  return runtime_ != nullptr && runtime_->is_calibrating();
}

bool RuntimeFrontendController::start_raw_collection(raw_csi_packet_callback_t callback,
                                                     void *context) {
  if (!runtime_ || !capabilities_.supports_raw_csi || callback == nullptr) {
    return false;
  }
  const bool started = runtime_->start_raw_collection(callback, context);
  if (started) {
    cache_snapshot_(runtime_->get_snapshot());
  }
  apply_deferred_shutdown_();
  return started;
}

bool RuntimeFrontendController::stop_raw_collection(RawCsiStopReason reason) {
  if (!runtime_ || runtime_->operation_state() != RuntimeOperationState::RAW_COLLECTION) {
    return false;
  }
  const bool stopped = runtime_->stop_raw_collection(reason);
  if (stopped) {
    runtime_->set_services_armed(services_armed_);
    cache_snapshot_(runtime_->get_snapshot());
  }
  apply_deferred_shutdown_();
  return stopped;
}

RuntimeOperationState RuntimeFrontendController::operation_state() const {
  return runtime_ != nullptr ? runtime_->operation_state() : RuntimeOperationState::SENSING;
}

RuntimeDiagnosticsSnapshot RuntimeFrontendController::diagnostics() const {
  return runtime_ != nullptr ? runtime_->get_diagnostics() : RuntimeDiagnosticsSnapshot{};
}

const RuntimeDiagnosticsSample *RuntimeFrontendController::diagnostics_sample() const {
  return runtime_ != nullptr ? runtime_->get_diagnostics_sample() : nullptr;
}

void RuntimeFrontendController::cache_snapshot_(const RuntimeSnapshot &snapshot) {
  snapshot_ = snapshot;
}

void RuntimeFrontendController::adopt_effective_threshold_(float threshold) {
  const bool staged_for_next_setup =
      config_.segmentation_threshold != active_config_.segmentation_threshold;
  active_config_.segmentation_threshold = threshold;
  if (!staged_for_next_setup) {
    config_.segmentation_threshold = threshold;
  }
}

void RuntimeFrontendController::adopt_effective_detector_(DetectionAlgorithm algorithm) {
  const bool staged_for_next_setup =
      config_.detection_algorithm != active_config_.detection_algorithm;
  active_config_.detection_algorithm = algorithm;
  if (!staged_for_next_setup) {
    config_.detection_algorithm = algorithm;
  }
}

void RuntimeFrontendController::begin_callback_() { ++callback_depth_; }

void RuntimeFrontendController::end_callback_() {
  if (callback_depth_ > 0U) {
    --callback_depth_;
  }
}

void RuntimeFrontendController::apply_deferred_shutdown_() {
  if (shutdown_requested_ && callback_depth_ == 0U) {
    shutdown();
  }
}

void RuntimeFrontendController::on_motion_state_changed(const RuntimeSnapshot &snapshot) {
  cache_snapshot_(snapshot);
  if (listener_ != nullptr) {
    begin_callback_();
    listener_->on_motion_state_changed(snapshot);
    end_callback_();
  }
}

void RuntimeFrontendController::on_periodic_update(const RuntimeSnapshot &snapshot,
                                                   uint32_t packets_received) {
  cache_snapshot_(snapshot);
  if (listener_ != nullptr) {
    begin_callback_();
    listener_->on_periodic_update(snapshot, packets_received);
    end_callback_();
  }
}

void RuntimeFrontendController::on_threshold_changed(const RuntimeSnapshot &snapshot) {
  cache_snapshot_(snapshot);
  adopt_effective_threshold_(snapshot.threshold);
  if (listener_ != nullptr) {
    begin_callback_();
    listener_->on_threshold_changed(snapshot);
    end_callback_();
  }
}

void RuntimeFrontendController::on_detector_changed(const RuntimeSnapshot &snapshot) {
  cache_snapshot_(snapshot);
  adopt_effective_detector_(parse_detection_algorithm(snapshot.detector_name));
  adopt_effective_threshold_(snapshot.threshold);
  if (listener_ != nullptr) {
    begin_callback_();
    listener_->on_detector_changed(snapshot);
    end_callback_();
  }
}

void RuntimeFrontendController::on_calibration_started(const RuntimeSnapshot &snapshot) {
  cache_snapshot_(snapshot);
  if (listener_ != nullptr) {
    begin_callback_();
    listener_->on_calibration_started(snapshot);
    end_callback_();
  }
}

void RuntimeFrontendController::on_calibration_finished(const RuntimeSnapshot &snapshot,
                                                        bool success) {
  cache_snapshot_(snapshot);
  adopt_effective_threshold_(snapshot.threshold);
  if (listener_ != nullptr) {
    begin_callback_();
    listener_->on_calibration_finished(snapshot, success);
    end_callback_();
  }
}

void RuntimeFrontendController::on_live_telemetry(float movement, float threshold) {
  snapshot_.movement_metric = movement;
  snapshot_.threshold = threshold;
  adopt_effective_threshold_(threshold);
  if (listener_ != nullptr) {
    begin_callback_();
    listener_->on_live_telemetry(movement, threshold);
    end_callback_();
  }
}

void RuntimeFrontendController::on_runtime_fault(const char *message) {
  if (listener_ != nullptr) {
    begin_callback_();
    listener_->on_runtime_fault(message);
    end_callback_();
  }
}

}  // namespace espectre
