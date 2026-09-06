/*
 * ESPectre - Native MQTT Frontend Adapter
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

#include "native_mqtt_frontend.h"

#include <esp_log.h>

#include <utility>

#include "frontend_mqtt_helpers.h"
#include "native_direct_frontend.h"
#include "native_frontend.h"
#include "protocol_json.h"
#include "sdkconfig.h"

namespace espectre {

namespace {

static const char *const TAG = "espectre.native.mqtt";

}  // namespace

NativeMqttFrontend::NativeMqttFrontend(NativeFrontend &owner, IMqttTransport *transport)
    : owner_(owner), transport_(transport), home_assistant_(owner, transport) {}

void NativeMqttFrontend::setup() {
  const bool was_connected = connected_;
  connected_ = false;
  home_assistant_.set_online(false);
  if (was_connected) {
    owner_.update_live_telemetry_enabled_();
  }
  (void)setup_frontend_mqtt_transport(
      transport_, owner_.device_config_, [this](const std::string &payload) { this->handle_command_(payload); },
      [this](bool connected) {
        this->connected_ = connected;
        this->home_assistant_.set_online(connected);
        this->owner_.update_live_telemetry_enabled_();
        if (connected) {
          this->publish_capabilities();
          this->publish_info();
          this->publish_status(true);
          this->publish_config();
          this->publish_current_ota_status();
          this->home_assistant_.setup();
          this->home_assistant_.schedule_discovery();
        }
      },
      TAG);
}

void NativeMqttFrontend::loop() {
  if (transport_ != nullptr) {
    transport_->loop();
    home_assistant_.drain_pending_snapshot();
  }
}

void NativeMqttFrontend::shutdown() {
  connected_ = false;
  home_assistant_.set_online(false);
  owner_.update_live_telemetry_enabled_();
  if (transport_ != nullptr) {
    transport_->shutdown();
  }
}

MqttTransportDiagnostics NativeMqttFrontend::diagnostics() const {
  return transport_ != nullptr ? transport_->diagnostics() : MqttTransportDiagnostics{};
}

bool NativeMqttFrontend::publish_message(const char *suffix, const std::string &payload, bool retain) {
  return publish_frontend_mqtt_message(transport_, owner_.device_config_, suffix, payload, retain);
}

void NativeMqttFrontend::handle_command_(const std::string &payload) {
  EspectreCommand command;
  std::string parse_error;
  if (!parse_espectre_command(payload, &command, &parse_error, owner_.command_capability_profile_(false).extension)) {
    if (command.command.empty()) {
      command.command = "unknown";
    }
    FrontendCommandResult result;
    result.handled = true;
    result.command = std::move(command);
    result.code = frontend_command_parse_error_code(parse_error);
    result.message = std::move(parse_error);
    publish_command_result(result);
    return;
  }
  publish_command_result(owner_.dispatch_command_(command, FrontendCommandOrigin::MQTT, false));
}

void NativeMqttFrontend::publish_capabilities() {
  const EspectreDeviceInfo info = owner_.mqtt_protocol_device_info_();
  (void)publish_frontend_mqtt_message(
      transport_, owner_.device_config_, "capabilities",
      espectre_capabilities_payload(owner_.device_config_, info, owner_.command_capability_profile_(false)), true);
}

void NativeMqttFrontend::publish_info() {
  const EspectreDeviceInfo info = owner_.mqtt_protocol_device_info_();
  (void)publish_frontend_mqtt_message(transport_, owner_.device_config_, "device",
                                      espectre_device_payload(owner_.device_config_, info), true);
}

void NativeMqttFrontend::publish_status(bool online) {
  (void)publish_frontend_mqtt_message(transport_, owner_.device_config_, "health",
                                      owner_.direct_frontend_->health_payload(online), true);
}

void NativeMqttFrontend::publish_telemetry(const RuntimeSnapshot &snapshot, uint32_t now_ms) {
  const char *frontend = owner_.device_info_.frontend.empty() ? "native" : owner_.device_info_.frontend.c_str();
  (void)publish_frontend_mqtt_message(
      transport_, owner_.device_config_, "motion",
      espectre_motion_payload(owner_.device_config_, snapshot, now_ms, now_ms / 1000U, frontend), false);
}

void NativeMqttFrontend::publish_config() {
  (void)publish_frontend_mqtt_message(transport_, owner_.device_config_, "sensing",
                                      owner_.direct_frontend_->sensing_payload(), true);
  (void)publish_frontend_mqtt_message(transport_, owner_.device_config_, "wifi",
                                      owner_.direct_frontend_->wifi_payload(true), true);
}

void NativeMqttFrontend::publish_ota_status(const EspectreOtaStatus &status) {
  (void)publish_frontend_mqtt_message(transport_, owner_.device_config_, "ota",
      espectre_ota_status_payload(owner_.device_config_, status, owner_.now_ms_()), true);
}

void NativeMqttFrontend::publish_current_ota_status() {
  if (owner_.ota_service_ != nullptr) {
    publish_ota_status(owner_.current_ota_status_());
  }
}

void NativeMqttFrontend::publish_command_result(const FrontendCommandResult &result) {
  (void)publish_frontend_mqtt_command_result(transport_, owner_.device_config_, result);
}

}  // namespace espectre
