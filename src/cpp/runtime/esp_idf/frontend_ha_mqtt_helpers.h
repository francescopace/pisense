/*
 * ESPectre - Frontend Home Assistant MQTT Helpers
 *
 * Builds Home Assistant MQTT discovery and simple state topics for
 * standalone frontends while preserving the canonical ESPectre protocol.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <string>
#include <vector>

#include "espectre_protocol.h"

namespace espectre {

struct FrontendHaDiagnosticSensor {
  std::string name;
  std::string key;
  std::string object_id;
  std::string state_topic;
  const char *unit_of_measurement{nullptr};
  const char *icon{nullptr};
  const char *device_class{nullptr};
  bool state_class_measurement{true};
};

struct FrontendHaMqttSettings {
  std::string discovery_prefix;
  std::string birth_topic;
  std::string availability_topic;
  std::string availability_template;
  std::string motion_state_topic;
  std::string movement_state_topic;
  std::string threshold_state_topic;
  std::string threshold_command_topic;
  std::string motion_on_hits_state_topic;
  std::string motion_on_hits_command_topic;
  std::string motion_off_hits_state_topic;
  std::string motion_off_hits_command_topic;
  std::string calibrate_state_topic;
  std::string calibrate_command_topic;
  std::string detector_state_topic;
  std::string detector_command_topic;
  std::string csi_traffic_mode_state_topic;
  std::string csi_traffic_mode_command_topic;
  std::string traffic_generator_mode_state_topic;
  std::string traffic_generator_mode_command_topic;
  std::string diagnostics_command_topic;
  std::string motion_object_id;
  std::string movement_object_id;
  std::string threshold_object_id;
  std::string motion_on_hits_object_id;
  std::string motion_off_hits_object_id;
  std::string recalibrate_object_id;
  std::string calibration_active_object_id;
  std::string detector_object_id;
  std::string csi_traffic_mode_object_id;
  std::string traffic_generator_mode_object_id;
  std::string diagnostics_object_id;
  std::string ha_object_prefix;
  std::vector<FrontendHaDiagnosticSensor> diagnostic_sensors;
  std::string device_id;
  std::string device_name;
  std::string model;
};

struct FrontendHaDiscoveryMessage {
  std::string topic;
  std::string payload;
};

bool frontend_ha_mqtt_enabled();
FrontendHaMqttSettings build_frontend_ha_mqtt_settings(const EspectreDeviceConfig &config,
                                                       const EspectreDeviceInfo &info,
                                                       const char *frontend_name);
/// Build one discovery message in publication order; false marks the end.
bool build_frontend_ha_discovery_message(
    const FrontendHaMqttSettings &settings,
    const EspectreDeviceInfo &info,
    bool supports_detector,
    bool supports_motion_hits,
    bool supports_traffic_control, size_t index, FrontendHaDiscoveryMessage *message);
std::vector<FrontendHaDiscoveryMessage> build_frontend_ha_discovery_messages(
    const FrontendHaMqttSettings &settings,
    const EspectreDeviceInfo &info,
    bool supports_detector,
    bool supports_motion_hits,
    bool supports_traffic_control);

}  // namespace espectre
