/*
 * ESPectre - Matter Frontend Adapter
 *
 * Bridges runtime events to the standard Matter occupancy surface.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "matter_frontend.h"

#include <utility>

#include "device_identity.h"
#include "direct_http_protocol.h"
#include <esp_log.h>
#include "frontend/frontend_firmware_version.h"
#include "matter_surface.h"
#include "protocol_json.h"
#include "runtime_config_utils.h"
#include "runtime_time.h"
#include "sdkconfig.h"

namespace espectre {

static const char *const TAG = "espectre.matter";

MatterFrontend::MatterFrontend(IMatterBindings *bindings,
                               uint16_t endpoint_id,
                               IDirectHttpService *direct_service)
    : bindings_(bindings), endpoint_id_(endpoint_id), direct_service_(direct_service) {}

void MatterFrontend::set_runtime_config(const RuntimeConfig &config) { runtime_.set_config(config); }

void MatterFrontend::set_wifi_bssid_pin_setter(WifiBssidPinSetter setter) {
  wifi_bssid_pin_setter_ = std::move(setter);
}

void MatterFrontend::set_wifi_bssid_pin_preflight(WifiBssidPinPreflight preflight) {
  wifi_bssid_pin_preflight_ = std::move(preflight);
}

bool MatterFrontend::set_runtime_services_armed(bool armed) {
  operational_services_armed_ = armed;
  if (!armed) {
    wifi_reconfigure_quiesced_ = false;
    wifi_reconfigure_resume_pending_ = false;
    runtime_.set_services_armed(false);
    stop_direct_service_();
    return true;
  }
  runtime_.set_services_armed(true);
  if (!runtime_.is_setup_complete()) {
    if (!setup()) {
      operational_services_armed_ = false;
      runtime_.set_services_armed(false);
      ESP_LOGE(TAG, "Matter operational services remain disarmed because runtime setup failed");
      return false;
    }
    return true;
  }
  if (runtime_.is_setup_complete() && !start_direct_service_()) {
    operational_services_armed_ = false;
    runtime_.set_services_armed(false);
    ESP_LOGE(TAG, "Matter operational services remain disarmed because Direct HTTP failed to start");
    return false;
  }
  return true;
}

void MatterFrontend::prepare_for_wifi_reconfigure() {
  if (!operational_services_armed_ || wifi_reconfigure_quiesced_) return;
  wifi_reconfigure_quiesced_ = true;
  wifi_reconfigure_resume_pending_ = false;
  runtime_.set_services_armed(false);
}

void MatterFrontend::resume_after_wifi_reconfigure() {
  if (wifi_reconfigure_quiesced_) wifi_reconfigure_resume_pending_ = true;
}

bool MatterFrontend::setup() {
  if (runtime_.is_setup_complete()) {
    return true;
  }

  if (bindings_ == nullptr) {
    ESP_LOGE(TAG, "Matter bindings are not configured");
    return false;
  }

  update_live_telemetry_enabled_();
  if (!runtime_.setup(this)) {
    ESP_LOGE(TAG, "ESPectre runtime setup failed");
    return false;
  }

  if (runtime_.services_armed() && !start_direct_service_()) {
    runtime_.shutdown();
    return false;
  }

  ESP_LOGI(TAG, "Matter frontend initialized on endpoint %u", endpoint_id_);
  return true;
}

bool MatterFrontend::start_direct_service_() {
  if (direct_service_ == nullptr || direct_bridge_.running()) {
    return true;
  }
  const uint64_t device_id = runtime_.config().device_id != 0U
                                 ? runtime_.config().device_id
                                 : derive_runtime_device_id();
  if (!direct_bridge_.setup(
          direct_service_,
          &runtime_,
          RuntimeDirectHttpBridgeConfig{
              "matter",
              espectre_device_name(device_id, CONFIG_IDF_TARGET),
              "",
              frontend_firmware_version(),
              CONFIG_IDF_TARGET,
              device_id,
              ESPECTRE_DIRECT_HTTP_PORT,
              true,
              false,
              [this]() {
                std::string label;
                return bindings_->get_node_label(&label) ? label : fallback_device_label_;
              },
              [this](const std::string &label, std::string *message) {
                const bool accepted = bindings_->set_node_label(label);
                if (message != nullptr) {
                  *message = accepted ? "Matter NodeLabel updated" : "Matter NodeLabel update rejected";
                }
                return accepted;
              },
              {},
              &peer_discovery_,
              [this]() { return this->runtime_.diagnostics_sample(); },
              &runtime_events_,
              wifi_bssid_pin_setter_,
              wifi_bssid_pin_preflight_,
          })) {
    ESP_LOGE(TAG, "Matter Direct HTTP setup failed");
    return false;
  }
  update_live_telemetry_enabled_();
  return true;
}

void MatterFrontend::stop_direct_service_() {
  direct_bridge_.shutdown();
  runtime_events_.clear();
  update_live_telemetry_enabled_();
}

void MatterFrontend::sync_device_label() {
  (void) direct_bridge_.publish_changes(FrontendCommandChange::DEVICE);
}

void MatterFrontend::shutdown() {
  operational_services_armed_ = false;
  wifi_reconfigure_quiesced_ = false;
  wifi_reconfigure_resume_pending_ = false;
  stop_direct_service_();
  runtime_.shutdown();
}

MatterFrontend::~MatterFrontend() { shutdown(); }

void MatterFrontend::loop() {
  runtime_.loop();
  if (wifi_reconfigure_resume_pending_) {
    wifi_reconfigure_resume_pending_ = false;
    wifi_reconfigure_quiesced_ = false;
    if (operational_services_armed_) runtime_.set_services_armed(true);
  }
  drain_pending_runtime_events_();
  bindings_->flush_pending();
  direct_bridge_.loop();
  update_live_telemetry_enabled_();
}

void MatterFrontend::drain_pending_runtime_events_() {
  RuntimeSnapshot snapshot;
  while (runtime_events_.take_motion_state(snapshot)) {
    bindings_->publish_motion(endpoint_id_, snapshot_to_motion_detected(snapshot));
  }
  if (runtime_events_.take_live_telemetry(snapshot)) {
    (void) direct_bridge_.publish_motion(snapshot);
  }
}

void MatterFrontend::update_live_telemetry_enabled_() {
  const bool enabled = direct_bridge_.event_client_count() > 0U;
  if (enabled == live_telemetry_enabled_) {
    return;
  }
  live_telemetry_enabled_ = enabled;
  runtime_.set_live_telemetry_enabled(enabled);
}

void MatterFrontend::on_sensing_readiness_changed(const RuntimeSnapshot &snapshot) {
  (void) snapshot;
  (void) direct_bridge_.publish_changes(FrontendCommandChange::SENSING);
}

void MatterFrontend::on_motion_state_changed(const RuntimeSnapshot &snapshot) {
  if (!snapshot.ready_to_publish) {
    return;
  }

  (void) runtime_events_.post_motion_state(snapshot);
}

void MatterFrontend::on_periodic_update(const RuntimeSnapshot &snapshot, uint32_t packets_received) {
  (void) snapshot;
  (void) packets_received;
}

void MatterFrontend::on_threshold_changed(const RuntimeSnapshot &snapshot) {
  (void) snapshot;
  (void) direct_bridge_.publish_changes(FrontendCommandChange::SENSING);
}

void MatterFrontend::on_detector_changed(const RuntimeSnapshot &snapshot) {
  (void) snapshot;
  (void) direct_bridge_.publish_changes(FrontendCommandChange::SENSING);
}

void MatterFrontend::on_calibration_started(const RuntimeSnapshot &snapshot) {
  (void) snapshot;
  (void) direct_bridge_.publish_changes(FrontendCommandChange::SENSING);
}

void MatterFrontend::on_calibration_finished(const RuntimeSnapshot &snapshot, bool success) {
  (void) snapshot;
  (void) direct_bridge_.publish_changes(FrontendCommandChange::SENSING);
  if (!success) {
    ESP_LOGW(TAG, "Calibration finished without a valid update");
  }
}

void MatterFrontend::on_live_telemetry(float movement, float threshold) {
  RuntimeSnapshot snapshot = runtime_.snapshot();
  snapshot.movement_metric = movement;
  snapshot.threshold = threshold;
  runtime_events_.post_live_telemetry(snapshot);
}

void MatterFrontend::on_runtime_fault(const char *message) {
  // setup() refuses a null bindings_ and the runtime only calls back once
  // setup() succeeded, so the pointer is an invariant rather than something to
  // re-test per hook; on_motion_state_changed already relies on that.
  if (message != nullptr) {
    bindings_->report_fault(message);
  }
  EspectreDeviceConfig device;
  device.device_id = runtime_.config().device_id;
  (void) direct_bridge_.publish_event(
      "fault", espectre_fault_payload(device, message, monotonic_now_ms()));
}

}  // namespace espectre
