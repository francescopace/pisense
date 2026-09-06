/*
 * ESPectre - Native MQTT Frontend Adapter
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstdint>
#include <string>

#include "frontend_command_engine.h"
#include "home_assistant_mqtt_frontend.h"
#include "mqtt_transport.h"
#include "frontend/ota_service.h"
#include "runtime_events.h"

namespace espectre {

class NativeFrontend;

class NativeMqttFrontend {
 public:
  NativeMqttFrontend(NativeFrontend &owner, IMqttTransport *transport);

  void setup();
  void loop();
  void shutdown();
  bool connected() const { return connected_; }
  bool transport_connected() const { return transport_ != nullptr && transport_->connected(); }
  MqttTransportDiagnostics diagnostics() const;
  bool publish_message(const char *suffix, const std::string &payload, bool retain = false);

  void publish_capabilities();
  void publish_info();
  void publish_status(bool online);
  void publish_telemetry(const RuntimeSnapshot &snapshot, uint32_t now_ms);
  void publish_config();
  void publish_ota_status(const EspectreOtaStatus &status);
  void publish_current_ota_status();
  void publish_command_result(const FrontendCommandResult &result);
  HomeAssistantMqttFrontend &home_assistant() { return home_assistant_; }

 private:
  void handle_command_(const std::string &payload);

  NativeFrontend &owner_;
  IMqttTransport *transport_{nullptr};
  HomeAssistantMqttFrontend home_assistant_;
  bool connected_{false};
};

}  // namespace espectre
