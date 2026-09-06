/*
 * ESPectre - Native Home Assistant MQTT Frontend Adapter
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

#include "home_assistant_mqtt_frontend.h"

#include <esp_log.h>

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "native_frontend.h"
#include "native_mqtt_frontend.h"
#include "runtime_diagnostics.h"

namespace espectre {

namespace {

[[maybe_unused]] static const char *const TAG = "espectre.native.ha";
constexpr const char *kHaOnlinePayload = "online";

const char *motion_state_payload(MotionState state) { return state == MotionState::MOTION ? "ON" : "OFF"; }

std::string float_payload(float value) {
  char buffer[24];
  std::snprintf(buffer, sizeof(buffer), "%.4f", static_cast<double>(value));
  return buffer;
}

std::string normalize_text_token(const std::string &value) {
  std::string normalized;
  normalized.reserve(value.size());
  for (const char ch : value) {
    if (!std::isspace(static_cast<unsigned char>(ch))) {
      normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
  }
  return normalized;
}

std::string diagnostic_state_payload(const std::string &key, const RuntimeDiagnosticsSample &sample) {
  char buffer[24];
  if (key == "traffic_tx_rate") {
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(sample.traffic_tx_pps));
  } else if (key == "csi_callback_rate") {
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(sample.csi_callback_pps));
  } else if (key == "csi_accepted_rate") {
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(sample.csi_accepted_pps));
  } else if (key == "csi_admitted_rate") {
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(sample.csi_admitted_pps));
  } else if (key == "csi_filtered_rate") {
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(sample.csi_filtered_pps));
  } else if (key == "csi_missing_rate") {
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(sample.csi_missing_slots_pps));
  } else if (key == "csi_excess_rate") {
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(sample.csi_excess_pps));
  } else if (key == "csi_stale_rate") {
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(sample.csi_stale_pps));
  } else if (key == "csi_out_of_order_rate") {
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(sample.csi_out_of_order_pps));
  } else if (key == "csi_occupancy") {
    std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(sample.csi_occupancy_ratio * 100.0f));
  } else if (key == "wifi_channel") {
    std::snprintf(buffer, sizeof(buffer), "%u", static_cast<unsigned>(sample.wifi_channel));
  } else if (key == "wifi_rssi") {
    if (sample.wifi_rssi_dbm == INT8_MIN) {
      return {};
    }
    std::snprintf(buffer, sizeof(buffer), "%d", static_cast<int>(sample.wifi_rssi_dbm));
  } else {
    return {};
  }
  return buffer;
}

}  // namespace

HomeAssistantMqttFrontend::HomeAssistantMqttFrontend(NativeFrontend &owner, IMqttTransport *transport)
    : owner_(owner), transport_(transport) {}

void HomeAssistantMqttFrontend::set_online(bool online) {
  online_ = online && frontend_ha_mqtt_enabled();
  if (!online_) {
    pending_discovery_ = false;
    pending_discovery_message_ = {};
    pending_discovery_index_ = 0U;
    pending_state_ = false;
  }
}

void HomeAssistantMqttFrontend::cancel_pending_state() { pending_state_ = false; }

void HomeAssistantMqttFrontend::setup() {
  if (!frontend_ha_mqtt_enabled() || transport_ == nullptr) {
    return;
  }
  settings_ = build_frontend_ha_mqtt_settings(owner_.device_config_, owner_.device_info_, "native");
  (void)transport_->subscribe(settings_.birth_topic, [this](const std::string &topic, const std::string &payload) {
    this->handle_birth_message_(topic, payload);
  });
  (void)transport_->subscribe(
      settings_.threshold_command_topic,
      [this](const std::string &, const std::string &payload) { this->handle_threshold_command_(payload); });
  if (owner_.runtime_.capabilities().supports_runtime_motion_hits_updates) {
    (void)transport_->subscribe(
        settings_.motion_on_hits_command_topic,
        [this](const std::string &, const std::string &payload) { this->handle_motion_hits_command_(true, payload); });
    (void)transport_->subscribe(
        settings_.motion_off_hits_command_topic,
        [this](const std::string &, const std::string &payload) { this->handle_motion_hits_command_(false, payload); });
  }
  (void)transport_->subscribe(
      settings_.calibrate_command_topic,
      [this](const std::string &, const std::string &payload) { this->handle_calibrate_command_(payload); });
  if (owner_.runtime_.capabilities().supports_runtime_detector_selection) {
    (void)transport_->subscribe(
        settings_.detector_command_topic, [this](const std::string &, const std::string &payload) {
          const std::string detector = normalize_text_token(payload);
          if (detector == RUNTIME_DETECTION_ALGORITHM_LIGHTWEIGHT_NAME ||
              detector == RUNTIME_DETECTION_ALGORITHM_HIGH_ACCURACY_NAME) {
            if (this->owner_.handle_detector_write_(parse_detection_algorithm(detector.c_str()))) {
              this->owner_.publish_runtime_config_state_();
            }
          }
        });
  }
  if (owner_.runtime_.capabilities().supports_traffic_control) {
    (void)transport_->subscribe(
        settings_.csi_traffic_mode_command_topic,
        [this](const std::string &, const std::string &payload) { this->handle_csi_traffic_mode_command_(payload); });
    (void)transport_->subscribe(settings_.traffic_generator_mode_command_topic,
                                [this](const std::string &, const std::string &payload) {
                                  this->handle_traffic_generator_mode_command_(payload);
                                });
  }
  (void)transport_->subscribe(settings_.diagnostics_command_topic,
                              [this](const std::string &, const std::string &) { this->publish_diagnostics(); });
}

void HomeAssistantMqttFrontend::schedule_discovery() {
  if (!frontend_ha_mqtt_enabled() || transport_ == nullptr || !transport_->connected()) {
    return;
  }
  settings_ = build_frontend_ha_mqtt_settings(owner_.device_config_, owner_.device_info_, "native");
  pending_discovery_ = true;
  pending_discovery_message_ = {};
  pending_discovery_index_ = 0U;
  pending_state_ = true;
  drain_pending_snapshot();
}

void HomeAssistantMqttFrontend::drain_pending_snapshot() {
  if (!online_ || transport_ == nullptr || !transport_->connected()) {
    return;
  }
  while (pending_discovery_) {
    const MqttTransportDiagnostics diagnostics = transport_->diagnostics();
    if (diagnostics.queue_capacity > 0U && diagnostics.queued_publishes >= diagnostics.queue_capacity) {
      return;
    }
    if (pending_discovery_message_.topic.empty() && !build_frontend_ha_discovery_message(
            settings_, owner_.device_info_,
            owner_.runtime_.capabilities().supports_runtime_detector_selection,
            owner_.runtime_.capabilities().supports_runtime_motion_hits_updates,
            owner_.runtime_.capabilities().supports_traffic_control,
            pending_discovery_index_, &pending_discovery_message_)) {
      pending_discovery_ = false;
      pending_discovery_index_ = 0U;
      break;
    }
    const FrontendHaDiscoveryMessage &message = pending_discovery_message_;
    if (!transport_->publish(message.topic, message.payload, true)) {
      return;
    }
    pending_discovery_message_ = {};
    pending_discovery_index_ += 1U;
  }

  if (pending_state_ && owner_.runtime_.snapshot().ready_to_publish &&
      transport_->diagnostics().queued_publishes == 0U) {
    pending_state_ = false;
    publish_current_state();
  }
}

bool HomeAssistantMqttFrontend::ready_() {
  if (!online_ || transport_ == nullptr) {
    return false;
  }
  if (settings_.movement_state_topic.empty()) {
    settings_ = build_frontend_ha_mqtt_settings(owner_.device_config_, owner_.device_info_, "native");
  }
  return !settings_.movement_state_topic.empty();
}

void HomeAssistantMqttFrontend::publish_motion(MotionState state) {
  if (ready_()) {
    (void)transport_->publish(settings_.motion_state_topic, motion_state_payload(state), false);
  }
}

void HomeAssistantMqttFrontend::publish_movement(float movement) {
  if (ready_()) {
    (void)transport_->publish(settings_.movement_state_topic, float_payload(movement), false);
  }
}

void HomeAssistantMqttFrontend::publish_threshold(float threshold) {
  if (ready_()) {
    (void)transport_->publish(settings_.threshold_state_topic, float_payload(threshold), false);
  }
}

void HomeAssistantMqttFrontend::publish_motion_hits(uint8_t motion_on_hits, uint8_t motion_off_hits) {
  if (!ready_() || !owner_.runtime_.capabilities().supports_runtime_motion_hits_updates) {
    return;
  }
  char buffer[8];
  std::snprintf(buffer, sizeof(buffer), "%u", static_cast<unsigned>(motion_on_hits));
  (void)transport_->publish(settings_.motion_on_hits_state_topic, buffer, false);
  std::snprintf(buffer, sizeof(buffer), "%u", static_cast<unsigned>(motion_off_hits));
  (void)transport_->publish(settings_.motion_off_hits_state_topic, buffer, false);
}

void HomeAssistantMqttFrontend::publish_calibrate(bool calibrating) {
  if (ready_()) {
    (void)transport_->publish(settings_.calibrate_state_topic, calibrating ? "ON" : "OFF", false);
  }
}

void HomeAssistantMqttFrontend::publish_detector(const char *detector_name) {
  if (!ready_() || detector_name == nullptr || !owner_.runtime_.capabilities().supports_runtime_detector_selection) {
    return;
  }
  (void)transport_->publish(settings_.detector_state_topic, detector_name, false);
}

void HomeAssistantMqttFrontend::publish_traffic_control(CsiTrafficMode csi_traffic_mode,
                                                        RuntimeTrafficMode traffic_generator_mode) {
  if (!ready_() || !owner_.runtime_.capabilities().supports_traffic_control) {
    return;
  }
  (void)transport_->publish(settings_.csi_traffic_mode_state_topic, csi_traffic_mode_name(csi_traffic_mode), false);
  (void)transport_->publish(settings_.traffic_generator_mode_state_topic, traffic_mode_name(traffic_generator_mode),
                            false);
}

void HomeAssistantMqttFrontend::publish_diagnostics() {
  if (!ready_()) {
    return;
  }
  const RuntimeDiagnosticsSample *sample = owner_.runtime_.diagnostics_sample();
  if (sample == nullptr) {
    return;
  }
  for (const FrontendHaDiagnosticSensor &sensor : settings_.diagnostic_sensors) {
    const std::string payload = diagnostic_state_payload(sensor.key, *sample);
    if (!payload.empty()) {
      (void)transport_->publish(sensor.state_topic, payload, false);
    }
  }
}

void HomeAssistantMqttFrontend::publish_state(const RuntimeSnapshot &snapshot) {
  if (!snapshot.ready_to_publish) {
    return;
  }
  publish_motion(snapshot.motion_state);
  publish_movement(snapshot.movement_metric);
  publish_threshold(snapshot.threshold);
  publish_motion_hits(owner_.runtime_.config().motion_on_hits, owner_.runtime_.config().motion_off_hits);
  publish_calibrate(owner_.runtime_.is_calibrating() || snapshot.calibrating);
  publish_detector(snapshot.detector_name);
  publish_traffic_control(owner_.runtime_.config().csi_traffic_mode, owner_.runtime_.config().traffic_generator_mode);
}

void HomeAssistantMqttFrontend::publish_current_state() { publish_state(owner_.runtime_.snapshot()); }

void HomeAssistantMqttFrontend::handle_birth_message_(const std::string &topic, const std::string &payload) {
  if (topic == settings_.birth_topic && frontend_ha_mqtt_enabled() &&
      normalize_text_token(payload) == kHaOnlinePayload) {
    schedule_discovery();
    owner_.mqtt_frontend_->publish_status(true);
  }
}

void HomeAssistantMqttFrontend::handle_threshold_command_(const std::string &payload) {
  const std::string token = normalize_text_token(payload);
  char *end_ptr = nullptr;
  errno = 0;
  const float threshold = strtof(token.c_str(), &end_ptr);
  const bool parse_ok =
      end_ptr != token.c_str() && end_ptr != nullptr && *end_ptr == '\0' && errno != ERANGE && std::isfinite(threshold);
  if (!parse_ok) {
    ESP_LOGW(TAG, "Invalid HA threshold command: %s", payload.c_str());
    return;
  }
  if (owner_.handle_threshold_write_(threshold)) {
    owner_.publish_runtime_config_state_();
  }
}

void HomeAssistantMqttFrontend::handle_motion_hits_command_(bool motion_on, const std::string &payload) {
  const std::string token = normalize_text_token(payload);
  char *end_ptr = nullptr;
  errno = 0;
  const unsigned long parsed = std::strtoul(token.c_str(), &end_ptr, 10);
  if (end_ptr == token.c_str() || end_ptr == nullptr || *end_ptr != '\0' || errno == ERANGE || parsed > UINT8_MAX) {
    ESP_LOGW(TAG, "Invalid HA motion hits command: %s", payload.c_str());
    return;
  }
  const uint8_t value = static_cast<uint8_t>(parsed);
  const uint8_t motion_on_hits = motion_on ? value : owner_.runtime_.config().motion_on_hits;
  const uint8_t motion_off_hits = motion_on ? owner_.runtime_.config().motion_off_hits : value;
  if (owner_.handle_motion_hits_write_(motion_on_hits, motion_off_hits)) {
    owner_.publish_runtime_config_state_();
  }
}

void HomeAssistantMqttFrontend::handle_calibrate_command_(const std::string &payload) {
  const std::string token = normalize_text_token(payload);
  if (token == "off") {
    publish_calibrate(owner_.runtime_.is_calibrating());
    return;
  }
  if (token != "on") {
    ESP_LOGW(TAG, "Invalid HA calibrate command: %s", payload.c_str());
    return;
  }
  if (owner_.runtime_.is_calibrating()) {
    publish_calibrate(true);
    return;
  }
  if (!owner_.handle_recalibration_write_()) {
    publish_calibrate(owner_.runtime_.is_calibrating());
  }
}

void HomeAssistantMqttFrontend::handle_csi_traffic_mode_command_(const std::string &payload) {
  const std::string mode = normalize_text_token(payload);
  if (mode != RUNTIME_CSI_TRAFFIC_MODE_INTERNAL_NAME && mode != RUNTIME_CSI_TRAFFIC_MODE_EXTERNAL_NAME) {
    ESP_LOGW(TAG, "Invalid HA CSI traffic mode command: %s", payload.c_str());
    return;
  }
  if (owner_.handle_csi_traffic_mode_write_(parse_csi_traffic_mode(mode.c_str()))) {
    owner_.publish_runtime_config_state_();
  }
}

void HomeAssistantMqttFrontend::handle_traffic_generator_mode_command_(const std::string &payload) {
  const std::string mode = normalize_text_token(payload);
  if (mode != RUNTIME_TRAFFIC_GENERATOR_MODE_PING_NAME && mode != RUNTIME_TRAFFIC_GENERATOR_MODE_DNS_NAME &&
      mode != RUNTIME_TRAFFIC_GENERATOR_MODE_DNS_TCP_NAME &&
      mode != RUNTIME_TRAFFIC_GENERATOR_MODE_WIFI_RAW_NAME) {
    ESP_LOGW(TAG, "Invalid HA traffic generator mode command: %s", payload.c_str());
    return;
  }
  if (owner_.handle_traffic_generator_mode_write_(parse_traffic_mode(mode.c_str()))) {
    owner_.publish_runtime_config_state_();
  }
}

}  // namespace espectre
