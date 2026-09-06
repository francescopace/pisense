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
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "frontend_command_engine.h"
#include "mqtt_transport.h"

namespace espectre {

using FrontendMqttConnectedCallback = std::function<void(bool)>;
bool setup_frontend_mqtt_transport(IMqttTransport *transport,
                                   const EspectreDeviceConfig &config,
                                   IMqttTransport::CommandCallback command_callback,
                                   FrontendMqttConnectedCallback connected_callback,
                                   const char *log_tag);

bool publish_frontend_mqtt_message(IMqttTransport *transport,
                                   const EspectreDeviceConfig &config,
                                   const char *suffix,
                                   const std::string &payload,
                                   bool retain);

bool publish_frontend_mqtt_status(IMqttTransport *transport,
                                  const EspectreDeviceConfig &config,
                                  bool online,
                                  uint32_t timestamp_ms);

bool publish_frontend_mqtt_command_result(IMqttTransport *transport,
                                          const EspectreDeviceConfig &config,
                                          const FrontendCommandResult &result);

}  // namespace espectre
