/*
 * ESPectre - Native Frontend Orchestrator
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

#include "native_frontend.h"

#include <esp_log.h>

#include <cmath>
#include <utility>

#include "esp_timer.h"
#include "frontend_mqtt_helpers.h"
#include "home_assistant_mqtt_frontend.h"
#include "native_command_bindings.h"
#include "native_mqtt_frontend.h"
#include "protocol_json.h"
#include "runtime_config_utils.h"
#include "runtime_time.h"
#include "sdkconfig.h"

namespace espectre {

namespace {

[[maybe_unused]] static const char *const TAG = "espectre.native";

}  // namespace

NativeFrontend::NativeFrontend(IMqttTransport *mqtt_transport, IOtaService *ota_service,
                               IDirectHttpService *direct_service)
    : ota_service_(ota_service),
      command_bindings_(std::make_unique<NativeCommandBindings>(*this)),
      direct_frontend_(std::make_unique<NativeDirectFrontend>(*this, direct_service)),
      mqtt_frontend_(std::make_unique<NativeMqttFrontend>(*this, mqtt_transport)) {}

NativeFrontend::~NativeFrontend() { shutdown(); }

void NativeFrontend::set_runtime_config(const RuntimeConfig &config) {
  RuntimeConfig native_config = config;
  native_config.runtime_detector_selection_enabled = true;
  runtime_.set_config(native_config);
}

void NativeFrontend::set_device_config(const EspectreDeviceConfig &config) {
  device_config_ = config;
  direct_frontend_->refresh_identity();
}

void NativeFrontend::set_device_info(const EspectreDeviceInfo &info) {
  device_info_ = info;
  direct_frontend_->refresh_identity();
}

void NativeFrontend::set_peer_discovery_service(IPeerDiscoveryService *service) {
  direct_frontend_->set_peer_discovery_service(service);
}

void NativeFrontend::set_wifi_provisioning_info(const WifiProvisioningInfo &info) {
  direct_frontend_->set_wifi_provisioning_info(info);
}

void NativeFrontend::set_provisioning_command_callback(ProvisioningCommandCallback callback) {
  provisioning_command_callback_ = std::move(callback);
}

void NativeFrontend::set_wifi_scan_callback(WifiScanCallback callback) { wifi_scan_callback_ = std::move(callback); }

void NativeFrontend::set_device_config_change_callback(DeviceConfigChangeCallback callback) {
  device_config_change_callback_ = std::move(callback);
}

size_t NativeFrontend::direct_client_count() const { return direct_frontend_->client_count(); }

void NativeFrontend::prepare_for_wifi_reconfigure() {
  if (wifi_reconfigure_quiesced_ || !runtime_.services_armed()) {
    return;
  }
  wifi_reconfigure_quiesced_ = true;
  wifi_reconfigure_resume_pending_ = false;
  runtime_.set_services_armed(false);
}

void NativeFrontend::resume_after_wifi_reconfigure() {
  if (!wifi_reconfigure_quiesced_) {
    return;
  }
  // The Wi-Fi service and runtime receive the same transition through
  // separate queues, so the runtime queue must drain before sensing resumes.
  wifi_reconfigure_resume_pending_ = true;
}

bool NativeFrontend::setup() {
  update_live_telemetry_enabled_();
  if (!runtime_.setup(this)) {
    ESP_LOGE(TAG, "ESPectre runtime setup failed");
    return false;
  }
  if (ota_service_ != nullptr) {
    ota_service_->set_prepare_for_update_callback([this]() { this->prepare_for_ota_(); });
    ota_service_->set_status_callback([this](const EspectreOtaStatus &status) {
      if (this->ota_frontend_quiesced_) {
        if (status.state == EspectreOtaState::ERROR) {
          this->resume_after_ota_error_();
        }
        return;
      }
      this->publish_ota_status_(status);
    });
  }
  mqtt_frontend_->setup();
  direct_frontend_->refresh();
  ESP_LOGI(TAG, "Native frontend initialized");
  return true;
}

void NativeFrontend::loop() {
  const int64_t loop_started_us = esp_timer_get_time();
  runtime_.loop();
  if (wifi_reconfigure_resume_pending_) {
    wifi_reconfigure_resume_pending_ = false;
    wifi_reconfigure_quiesced_ = false;
    if (!ota_frontend_quiesced_ && direct_frontend_->wifi_configured()) {
      runtime_.set_services_armed(true);
    }
  }
  drain_pending_runtime_events_();
  mqtt_frontend_->loop();
  direct_frontend_->loop();
  if (ota_service_ != nullptr) {
    ota_service_->loop();
  }
  last_loop_time_ms_ = static_cast<float>(esp_timer_get_time() - loop_started_us) / 1000.0f;
}

void NativeFrontend::shutdown() {
  runtime_events_.clear();
  wifi_reconfigure_resume_pending_ = false;
  wifi_reconfigure_quiesced_ = false;
  direct_frontend_->shutdown();
  direct_frontend_->shutdown_peer_discovery();
  update_live_telemetry_enabled_();
  mqtt_frontend_->publish_status(false);
  runtime_.shutdown();
  mqtt_frontend_->shutdown();
  if (ota_service_ != nullptr) {
    ota_service_->shutdown();
  }
}

void NativeFrontend::on_motion_state_changed(const RuntimeSnapshot &snapshot) {
  if (runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION || !snapshot.ready_to_publish) {
    return;
  }
  (void)runtime_events_.post_motion_state(snapshot);
}

void NativeFrontend::on_periodic_update(const RuntimeSnapshot &snapshot, uint32_t packets_received) {
  (void)snapshot;
  (void)packets_received;
}

void NativeFrontend::on_threshold_changed(const RuntimeSnapshot &snapshot) {
  if (runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION) return;
  mqtt_frontend_->publish_telemetry(snapshot, now_ms_());
  if (snapshot.ready_to_publish) {
    mqtt_frontend_->home_assistant().publish_threshold(snapshot.threshold);
  }
}

void NativeFrontend::on_detector_changed(const RuntimeSnapshot &snapshot) {
  if (runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION) return;
  const std::string sensing = direct_frontend_->sensing_payload();
  mqtt_frontend_->publish_message("sensing", sensing, true);
  direct_frontend_->publish_event("sensing", sensing);
  mqtt_frontend_->publish_telemetry(snapshot, now_ms_());
  if (snapshot.ready_to_publish) {
    if (snapshot.detector_name != nullptr) {
      mqtt_frontend_->home_assistant().publish_detector(snapshot.detector_name);
    }
    mqtt_frontend_->home_assistant().publish_threshold(snapshot.threshold);
  }
}

void NativeFrontend::on_calibration_started(const RuntimeSnapshot &snapshot) {
  if (runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION) return;
  calibration_started_ = true;
  calibration_start_threshold_ = snapshot.threshold;
  mqtt_frontend_->home_assistant().publish_calibrate(true);
  if (!protocol_recalibration_command_active_) {
    publish_runtime_status_state_();
  }
}

void NativeFrontend::on_calibration_finished(const RuntimeSnapshot &snapshot, bool success) {
  if (runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION) return;
  const bool threshold_changed =
      !calibration_started_ || std::fabs(snapshot.threshold - calibration_start_threshold_) > 1.0e-6f;
  calibration_started_ = false;
  if (!success) {
    ESP_LOGW(TAG, "Calibration finished without a valid update");
  }
  mqtt_frontend_->home_assistant().publish_calibrate(false);
  if (snapshot.ready_to_publish) {
    mqtt_frontend_->home_assistant().publish_threshold(snapshot.threshold);
  }
  publish_runtime_status_state_();
  if (success && threshold_changed) {
    publish_runtime_config_state_();
  }
}

void NativeFrontend::on_live_telemetry(float movement, float threshold) {
  if (runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION || !runtime_.snapshot().ready_to_publish) {
    return;
  }
  RuntimeSnapshot snapshot = runtime_.snapshot();
  snapshot.movement_metric = movement;
  snapshot.threshold = threshold;
  runtime_events_.post_live_telemetry(snapshot);
}

void NativeFrontend::drain_pending_runtime_events_() {
  if (runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    runtime_events_.clear();
    return;
  }
  RuntimeSnapshot snapshot;
  while (runtime_events_.take_motion_state(snapshot)) {
    mqtt_frontend_->home_assistant().publish_motion(snapshot.motion_state);
  }
  if (runtime_events_.take_live_telemetry(snapshot)) {
    const uint32_t now = now_ms_();
    const char *frontend = device_info_.frontend.empty() ? "native" : device_info_.frontend.c_str();
    const std::string payload = espectre_motion_payload(device_config_, snapshot, now, now / 1000U, frontend);
    fan_out_payload_("motion", "motion", payload, false, true);
    mqtt_frontend_->home_assistant().publish_movement(snapshot.movement_metric);
  }
}

void NativeFrontend::on_runtime_fault(const char *message) {
  direct_frontend_->publish_event("fault", espectre_fault_payload(device_config_, message, now_ms_()));
}

FrontendCommandResult NativeFrontend::dispatch_command_(const EspectreCommand &command, FrontendCommandOrigin origin,
                                                        bool allow_local_config, uint64_t connection_token) {
  return command_bindings_->execute(command, origin, allow_local_config, connection_token);
}

EspectreCapabilityProfile NativeFrontend::command_capability_profile_(bool allow_local_config) const {
  return command_bindings_->capability_profile(allow_local_config);
}

bool NativeFrontend::handle_threshold_write_(float threshold) {
  if (!runtime_.capabilities().supports_runtime_threshold_updates) {
    ESP_LOGW(TAG, "Runtime threshold updates are not supported");
    return false;
  }
  if (!runtime_.set_threshold_runtime(threshold)) {
    return false;
  }
  if (runtime_.snapshot().ready_to_publish) {
    mqtt_frontend_->home_assistant().publish_threshold(threshold);
  }
  return true;
}

bool NativeFrontend::handle_motion_hits_write_(uint8_t motion_on_hits, uint8_t motion_off_hits) {
  if (!runtime_.capabilities().supports_runtime_motion_hits_updates) {
    ESP_LOGW(TAG, "Runtime motion hit updates are not supported");
    return false;
  }
  if (!runtime_.set_motion_hits_runtime(motion_on_hits, motion_off_hits)) {
    return false;
  }
  mqtt_frontend_->home_assistant().publish_motion_hits(motion_on_hits, motion_off_hits);
  return true;
}

bool NativeFrontend::handle_csi_traffic_mode_write_(CsiTrafficMode mode) {
  if (!runtime_.capabilities().supports_traffic_control) {
    ESP_LOGW(TAG, "Runtime traffic control is not supported");
    return false;
  }
  if (!csi_traffic_mode_is_sensing_control(mode)) {
    ESP_LOGW(TAG, "CSI traffic mode is not selectable");
    return false;
  }
  if (!runtime_.set_csi_traffic_mode_runtime(mode)) {
    return false;
  }
  mqtt_frontend_->home_assistant().publish_traffic_control(runtime_.config().csi_traffic_mode,
                                                           runtime_.config().traffic_generator_mode);
  return true;
}

bool NativeFrontend::handle_traffic_generator_mode_write_(RuntimeTrafficMode mode) {
  if (!runtime_.capabilities().supports_traffic_control) {
    ESP_LOGW(TAG, "Runtime traffic control is not supported");
    return false;
  }
  if (!runtime_.set_traffic_generator_mode_runtime(mode)) {
    return false;
  }
  mqtt_frontend_->home_assistant().publish_traffic_control(runtime_.config().csi_traffic_mode,
                                                           runtime_.config().traffic_generator_mode);
  return true;
}

bool NativeFrontend::handle_detector_write_(DetectionAlgorithm algorithm) {
  if (!runtime_.capabilities().supports_runtime_detector_selection) {
    ESP_LOGW(TAG, "Runtime detector selection is not supported");
    return false;
  }
  return runtime_.set_detection_algorithm_runtime(algorithm);
}

bool NativeFrontend::handle_recalibration_write_() {
  if (!runtime_.capabilities().supports_manual_recalibration) {
    ESP_LOGW(TAG, "Manual recalibration is not supported");
    return false;
  }
  if (!runtime_.trigger_recalibration()) {
    return false;
  }
  mqtt_frontend_->home_assistant().publish_calibrate(runtime_.is_calibrating());
  return true;
}

void NativeFrontend::update_live_telemetry_enabled_() {
  runtime_.set_live_telemetry_enabled(mqtt_frontend_->connected() || direct_frontend_->client_count() > 0U);
}

void NativeFrontend::fan_out_payload_(const char *mqtt_suffix, const char *direct_event_name,
                                      const std::string &payload, bool mqtt_retain, bool replaceable_telemetry) {
  if (mqtt_suffix != nullptr) {
    (void)mqtt_frontend_->publish_message(mqtt_suffix, payload, mqtt_retain);
  }
  if (direct_event_name != nullptr) {
    direct_frontend_->publish_event(direct_event_name, payload, replaceable_telemetry);
  }
}

void NativeFrontend::publish_runtime_config_state_() {
  const std::string payload = direct_frontend_->sensing_payload();
  mqtt_frontend_->publish_message("sensing", payload, true);
  direct_frontend_->publish_event("sensing", payload);
}

void NativeFrontend::publish_runtime_status_state_() {
  const std::string payload = direct_frontend_->health_payload(!device_info_.network.ip_address.empty());
  mqtt_frontend_->publish_message("health", payload, true);
  direct_frontend_->publish_event("health", payload);
  const std::string sensing = direct_frontend_->sensing_payload();
  mqtt_frontend_->publish_message("sensing", sensing, true);
  direct_frontend_->publish_event("sensing", sensing);
}

EspectreDeviceInfo NativeFrontend::mqtt_protocol_device_info_() const {
  EspectreDeviceInfo info = normalize_protocol_device_info(device_info_, &runtime_.snapshot(),
                                                           "native", CONFIG_IDF_TARGET);
  info.supports_info = true;
  info.supports_diagnostics = true;
  info.supports_device_config = true;
  info.supports_runtime_threshold = runtime_.capabilities().supports_runtime_threshold_updates;
  info.supports_runtime_motion_hits = runtime_.capabilities().supports_runtime_motion_hits_updates;
  info.supports_runtime_detector = runtime_.capabilities().supports_runtime_detector_selection;
  info.supports_manual_recalibration = runtime_.capabilities().supports_manual_recalibration;
  info.supports_traffic_control = runtime_.capabilities().supports_traffic_control;
  info.csi_traffic_mode = csi_traffic_mode_name(runtime_.config().csi_traffic_mode);
  info.traffic_mode = traffic_mode_name(runtime_.config().traffic_generator_mode);
  info.csi_target_pps = runtime_.config().csi_target_pps;
  info.csi_traffic_udp_port = runtime_.config().csi_traffic_udp_port;
  info.csi_traffic_multicast_group = runtime_.config().csi_traffic_multicast_group;
  info.evaluation_interval_ms = runtime_.config().evaluation_interval_ms;
  return info;
}

EspectreOtaStatus NativeFrontend::current_ota_status_() const {
  EspectreOtaStatus status = ota_service_ != nullptr ? ota_service_->status() : EspectreOtaStatus{};
  if ((status.current_version.empty() || status.current_version == "unknown") &&
      !device_info_.firmware_version.empty()) {
    status.current_version = device_info_.firmware_version;
  }
  return status;
}

void NativeFrontend::publish_ota_status_(const EspectreOtaStatus &status) {
  EspectreOtaStatus normalized = status;
  if ((normalized.current_version.empty() || normalized.current_version == "unknown") &&
      !device_info_.firmware_version.empty()) {
    normalized.current_version = device_info_.firmware_version;
  }
  const std::string payload = espectre_ota_status_payload(device_config_, normalized, now_ms_());
  fan_out_payload_("ota", "ota", payload, true);
}

void NativeFrontend::prepare_for_ota_() {
  if (ota_frontend_quiesced_) {
    return;
  }
  ota_frontend_quiesced_ = true;
  ota_services_were_armed_ = runtime_.services_armed();
  mqtt_frontend_->shutdown();
  direct_frontend_->shutdown();
  runtime_.quiesce();
}

void NativeFrontend::resume_after_ota_error_() {
  if (!ota_frontend_quiesced_) {
    return;
  }
  ota_frontend_quiesced_ = false;
  if (ota_services_were_armed_ && direct_frontend_->wifi_configured()) {
    runtime_.set_services_armed(true);
  }
  ota_services_were_armed_ = false;
  mqtt_frontend_->setup();
  direct_frontend_->refresh();
}

uint32_t NativeFrontend::now_ms_() const { return monotonic_now_ms(); }

}  // namespace espectre
