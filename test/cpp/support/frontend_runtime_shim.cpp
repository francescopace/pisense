/*
 * ESPectre - Frontend Runtime Shim
 *
 * Host-side shim that exposes a configurable runtime to frontend tests.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "frontend_runtime_shim.h"

#include "esp_idf_runtime.h"
#include "espectre_protocol.h"
#include "runtime_config_utils.h"

namespace espectre {

uint64_t derive_runtime_device_id() { return 0x0123456789ABCDEFULL; }

std::string derive_runtime_device_id_string() {
  return format_espectre_device_id(derive_runtime_device_id());
}

namespace frontend_runtime_shim {

State state{};

void reset() { state = State{}; }

}  // namespace frontend_runtime_shim

EspIdfRuntime::EspIdfRuntime(const RuntimeConfig &config)
    : EspIdfRuntimeBase(config, "espectre.runtime.shim", "Unknown runtime fault"),
      detector_(nullptr),
      csi_traffic_service_(traffic_generator_, traffic_ingress_) {
  snapshot_ = frontend_runtime_shim::state.snapshot;
  capabilities_ = frontend_runtime_shim::state.capabilities;
  frontend_runtime_shim::state.last_instance = this;
  capabilities_.supports_runtime_detector_selection = config.runtime_detector_selection_enabled;
  if (frontend_runtime_shim::state.snapshot.threshold == RUNTIME_SEGMENTATION_THRESHOLD_DEFAULT) {
    snapshot_.threshold = config.segmentation_threshold;
  }
}

bool EspIdfRuntime::setup() {
  // The base owns set_listener() now, so the shim observes the registration
  // here instead. RuntimeFrontendController always calls set_listener() before
  // setup(), so listener_ is already the frontend by this point.
  frontend_runtime_shim::state.last_listener = listener_;
  if (frontend_runtime_shim::state.override_config_on_setup) {
    config_ = frontend_runtime_shim::state.setup_config;
  }
  return frontend_runtime_shim::state.setup_result;
}

void EspIdfRuntime::shutdown() { frontend_runtime_shim::state.shutdown_called = true; }

void EspIdfRuntime::loop() {
  frontend_runtime_shim::state.loop_calls++;
  if (frontend_runtime_shim::state.emit_threshold_on_next_loop && listener_ != nullptr) {
    frontend_runtime_shim::state.emit_threshold_on_next_loop = false;
    snapshot_ = frontend_runtime_shim::state.snapshot;
    listener_->on_threshold_changed(snapshot_);
  }
}

RuntimeSnapshot EspIdfRuntime::get_snapshot() const { return snapshot_; }

RuntimeDiagnosticsSnapshot EspIdfRuntime::get_diagnostics() const {
  return frontend_runtime_shim::state.diagnostics;
}

const RuntimeDiagnosticsSample *EspIdfRuntime::get_diagnostics_sample() const {
  return &frontend_runtime_shim::state.diagnostics_sample;
}

void EspIdfRuntime::set_services_armed(bool armed) {
  frontend_runtime_shim::state.services_armed = armed;
  frontend_runtime_shim::state.set_services_armed_calls++;
}

void EspIdfRuntime::set_live_telemetry_enabled(bool enabled) {
  frontend_runtime_shim::state.live_telemetry_enabled = enabled;
  frontend_runtime_shim::state.set_live_telemetry_enabled_calls++;
}

bool EspIdfRuntime::set_threshold_runtime(float threshold) {
  frontend_runtime_shim::state.set_threshold_calls++;
  frontend_runtime_shim::state.last_threshold = threshold;
  snapshot_.threshold = threshold;
  frontend_runtime_shim::state.snapshot.threshold = threshold;
  return true;
}

bool EspIdfRuntime::set_motion_hits_runtime(uint8_t motion_on_hits, uint8_t motion_off_hits) {
  frontend_runtime_shim::state.set_motion_hits_calls++;
  frontend_runtime_shim::state.last_motion_on_hits = motion_on_hits;
  frontend_runtime_shim::state.last_motion_off_hits = motion_off_hits;
  return true;
}

bool EspIdfRuntime::set_csi_traffic_mode_runtime(CsiTrafficMode mode) {
  frontend_runtime_shim::state.set_csi_traffic_mode_calls++;
  frontend_runtime_shim::state.last_csi_traffic_mode = mode;
  return true;
}

bool EspIdfRuntime::set_traffic_generator_mode_runtime(RuntimeTrafficMode mode) {
  frontend_runtime_shim::state.set_traffic_generator_mode_calls++;
  frontend_runtime_shim::state.last_traffic_generator_mode = mode;
  return true;
}

bool EspIdfRuntime::set_detection_algorithm_runtime(DetectionAlgorithm algorithm) {
  frontend_runtime_shim::state.set_detector_calls++;
  frontend_runtime_shim::state.last_detector = algorithm;
  snapshot_.detector_name = detection_algorithm_name(algorithm);
  snapshot_.threshold = runtime_default_threshold(algorithm);
  frontend_runtime_shim::state.snapshot = snapshot_;
  return true;
}

bool EspIdfRuntime::trigger_recalibration() {
  frontend_runtime_shim::state.trigger_recalibration_calls++;
  frontend_runtime_shim::state.calibrating = true;
  snapshot_.calibrating = true;
  frontend_runtime_shim::state.snapshot.calibrating = true;
  return true;
}

bool EspIdfRuntime::is_calibrating() const {
  return frontend_runtime_shim::state.calibrating;
}

bool EspIdfRuntime::start_raw_collection(raw_csi_packet_callback_t, void *) {
  operation_state_.store(RuntimeOperationState::RAW_COLLECTION, std::memory_order_release);
  snapshot_.ready_to_publish = false;
  frontend_runtime_shim::state.snapshot = snapshot_;
  return capabilities_.supports_raw_csi;
}

bool EspIdfRuntime::stop_raw_collection(RawCsiStopReason) {
  const bool was_active = operation_state() == RuntimeOperationState::RAW_COLLECTION;
  operation_state_.store(RuntimeOperationState::SENSING, std::memory_order_release);
  snapshot_.ready_to_publish = frontend_runtime_shim::state.services_armed;
  frontend_runtime_shim::state.snapshot = snapshot_;
  return was_active;
}

RuntimeOperationState EspIdfRuntime::operation_state() const {
  return operation_state_.load(std::memory_order_acquire);
}

}  // namespace espectre
