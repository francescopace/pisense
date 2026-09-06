/*
 * ESPectre - Frontend MQTT Helpers
 *
 * Sets up frontend MQTT transport and maps MQTT payloads to the shared
 * frontend command dispatcher.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "frontend_mqtt_helpers.h"

#include <utility>

#include "espectre_log.h"

namespace espectre {

bool setup_frontend_mqtt_transport(IMqttTransport *transport,
                                   const EspectreDeviceConfig &config,
                                   IMqttTransport::CommandCallback command_callback,
                                   FrontendMqttConnectedCallback connected_callback,
                                   const char *log_tag) {
  if (transport == nullptr) {
    return false;
  }
  if (!espectre_mqtt_configured(config)) {
    transport->shutdown();
    return false;
  }

  transport->set_command_callback(std::move(command_callback));
  transport->set_connection_callback([connected_callback = std::move(connected_callback)](bool connected) {
    if (connected_callback) {
      connected_callback(connected);
    }
  });
  if (!transport->setup(config)) {
    ESPECTRE_LOGW(log_tag != nullptr ? log_tag : "espectre.mqtt", "MQTT transport setup failed");
    return false;
  }
  return true;
}

bool publish_frontend_mqtt_message(IMqttTransport *transport,
                                   const EspectreDeviceConfig &config,
                                   const char *suffix,
                                   const std::string &payload,
                                   bool retain) {
  if (transport == nullptr || !transport->connected()) {
    return false;
  }
  (void) config;
  return transport->publish_suffix(suffix, payload, retain);
}

bool publish_frontend_mqtt_status(IMqttTransport *transport,
                                  const EspectreDeviceConfig &config,
                                  bool online,
                                  uint32_t timestamp_ms) {
  return publish_frontend_mqtt_message(
      transport, config, "health", espectre_health_payload(config, online, timestamp_ms), true);
}

bool publish_frontend_mqtt_command_result(IMqttTransport *transport,
                                          const EspectreDeviceConfig &config,
                                          const FrontendCommandResult &result) {
  return publish_frontend_mqtt_message(transport,
                                       config,
                                       "commands/result",
                                       espectre_command_result_payload(config,
                                                                       result.command,
                                                                       result.accepted,
                                                                       result.code.c_str(),
                                                                       result.message.c_str(),
                                                                       result.data_json),
                                       false);
}

}  // namespace espectre
