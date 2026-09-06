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
#include "frontend_ha_mqtt_helpers.h"

#include <cctype>
#include <utility>

#include "protocol_json.h"
#include "sdkconfig.h"

#ifndef CONFIG_ESPECTRE_HA_DISCOVERY_PREFIX
#define CONFIG_ESPECTRE_HA_DISCOVERY_PREFIX "homeassistant"
#endif

namespace espectre {

namespace {

constexpr const char *kHomeAssistantBirthTopic = "homeassistant/status";
constexpr const char *kStatusAvailabilityTemplate = "{{ 'online' if value_json.online else 'offline' }}";

std::string ha_entity_base_topic(const EspectreDeviceConfig &config, const char *entity) {
  return espectre_topic(config, (std::string("ha/") + entity).c_str());
}

std::string sanitize_identifier(const std::string &value) {
  std::string sanitized;
  sanitized.reserve(value.size());
  for (const char ch : value) {
    if (std::isalnum(static_cast<unsigned char>(ch))) {
      sanitized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    } else {
      sanitized.push_back('_');
    }
  }
  return sanitized;
}

std::string build_discovery_topic(const char *component, const std::string &prefix, const std::string &object_id) {
  return prefix + "/" + component + "/" + object_id + "/config";
}

void append_discovery_device(std::string *out, const FrontendHaMqttSettings &settings, const EspectreDeviceInfo &info) {
  if (out == nullptr) {
    return;
  }
  out->append(",\"device\":{");
  append_json_string(out, "identifiers");
  out->append(":[");
  append_json_string(out, settings.device_id.c_str());
  out->append("],");
  append_json_pair(out, "name", settings.device_name.c_str(), true);
  append_json_pair(out, "manufacturer", "ESPectre");
  append_json_pair(out, "model", settings.model.c_str());
  append_json_pair(out, "sw_version", info.firmware_version.empty() ? "unknown" : info.firmware_version.c_str());
  out->append("}");
}

void append_discovery_availability(std::string *out, const FrontendHaMqttSettings &settings) {
  append_json_pair(out, "availability_topic", settings.availability_topic.c_str());
  if (!settings.availability_template.empty()) {
    append_json_pair(out, "availability_template", settings.availability_template.c_str());
  }
  append_json_pair(out, "payload_available", "online");
  append_json_pair(out, "payload_not_available", "offline");
}

std::string build_motion_discovery_payload(const FrontendHaMqttSettings &settings, const EspectreDeviceInfo &info) {
  std::string out = "{";
  append_json_pair(&out, "name", "Motion Detected", true);
  append_json_pair(&out, "unique_id", settings.motion_object_id.c_str());
  append_json_pair(&out, "object_id", settings.motion_object_id.c_str());
  append_json_pair(&out, "state_topic", settings.motion_state_topic.c_str());
  append_discovery_availability(&out, settings);
  append_json_pair(&out, "payload_on", "ON");
  append_json_pair(&out, "payload_off", "OFF");
  append_json_pair(&out, "device_class", "motion");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

std::string build_movement_discovery_payload(const FrontendHaMqttSettings &settings, const EspectreDeviceInfo &info) {
  std::string out = "{";
  append_json_pair(&out, "name", "Movement Score", true);
  append_json_pair(&out, "unique_id", settings.movement_object_id.c_str());
  append_json_pair(&out, "object_id", settings.movement_object_id.c_str());
  append_json_pair(&out, "state_topic", settings.movement_state_topic.c_str());
  append_discovery_availability(&out, settings);
  append_json_pair(&out, "state_class", "measurement");
  append_json_pair(&out, "icon", "mdi:sine-wave");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

struct DiagnosticSensorDef {
  const char *name;
  const char *key;
  const char *object_suffix;
  const char *unit;
  const char *icon;
  const char *device_class;
  bool state_class_measurement;
};

constexpr DiagnosticSensorDef kDiagnosticSensors[] = {
    {"Traffic TX Rate", "traffic_tx_rate", "traffic_tx_rate", "pkt/s", "mdi:upload-network", nullptr, true},
    {"CSI Callback Rate", "csi_callback_rate", "csi_callback_rate", "pkt/s", "mdi:access-point", nullptr, true},
    {"CSI Accepted Rate", "csi_accepted_rate", "csi_accepted_rate", "pkt/s", "mdi:check-network", nullptr, true},
    {"CSI Admitted Rate", "csi_admitted_rate", "csi_admitted_rate", "pkt/s", "mdi:timeline-check-outline", nullptr, true},
    {"CSI Filtered Rate", "csi_filtered_rate", "csi_filtered_rate", "pkt/s", "mdi:filter-outline", nullptr, true},
    {"CSI Missing Slot Rate", "csi_missing_rate", "csi_missing_slot_rate", "slot/s", "mdi:timeline-minus-outline",
     nullptr, true},
    {"CSI Excess Rate", "csi_excess_rate", "csi_excess_rate", "pkt/s", "mdi:timeline-plus-outline", nullptr, true},
    {"CSI Stale Rate", "csi_stale_rate", "csi_stale_rate", "pkt/s", "mdi:timer-sand", nullptr, true},
    {"CSI Out-of-order Rate", "csi_out_of_order_rate", "csi_out_of_order_rate", "pkt/s", "mdi:swap-vertical", nullptr,
     true},
    {"CSI Temporal Occupancy", "csi_occupancy", "csi_temporal_occupancy", "%", "mdi:view-grid-outline", nullptr, true},
    {"WiFi Channel", "wifi_channel", "wifi_channel", nullptr, "mdi:wifi-marker", nullptr, false},
    {"WiFi RSSI", "wifi_rssi", "wifi_rssi", "dBm", nullptr, "signal_strength", true},
};

struct RetiredHaDiscovery {
  const char *component;
  const char *suffix;
};

constexpr RetiredHaDiscovery kRetiredHaDiscoveries[] = {
    {"sensor", "intensity"},
    {"binary_sensor", "motion"},
    {"sensor", "movement"},
    {"switch", "calibrate"},
    {"switch", "trigger_calibration"},
    {"select", "detector"},
    {"select", "csi_traffic_mode"},
    {"select", "traffic_generator_mode"},
    {"button", "diagnostics"},
    {"sensor", "csi_missing_rate"},
    {"sensor", "csi_occupancy"},
};

std::string build_diagnostic_sensor_discovery_payload(const FrontendHaMqttSettings &settings,
                                                      const EspectreDeviceInfo &info,
                                                      const FrontendHaDiagnosticSensor &sensor) {
  std::string out = "{";
  append_json_pair(&out, "name", sensor.name.c_str(), true);
  append_json_pair(&out, "unique_id", sensor.object_id.c_str());
  append_json_pair(&out, "object_id", sensor.object_id.c_str());
  append_json_pair(&out, "state_topic", sensor.state_topic.c_str());
  append_discovery_availability(&out, settings);
  if (sensor.unit_of_measurement != nullptr) {
    append_json_pair(&out, "unit_of_measurement", sensor.unit_of_measurement);
  }
  if (sensor.state_class_measurement) {
    append_json_pair(&out, "state_class", "measurement");
  }
  if (sensor.device_class != nullptr) {
    append_json_pair(&out, "device_class", sensor.device_class);
  }
  if (sensor.icon != nullptr) {
    append_json_pair(&out, "icon", sensor.icon);
  }
  append_json_pair(&out, "entity_category", "diagnostic");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

std::string build_diagnostics_button_discovery_payload(const FrontendHaMqttSettings &settings,
                                                       const EspectreDeviceInfo &info) {
  std::string out = "{";
  append_json_pair(&out, "name", "Refresh Diagnostics", true);
  append_json_pair(&out, "unique_id", settings.diagnostics_object_id.c_str());
  append_json_pair(&out, "object_id", settings.diagnostics_object_id.c_str());
  append_json_pair(&out, "command_topic", settings.diagnostics_command_topic.c_str());
  append_discovery_availability(&out, settings);
  append_json_pair(&out, "payload_press", "PRESS");
  append_json_pair(&out, "entity_category", "diagnostic");
  append_json_pair(&out, "icon", "mdi:refresh");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

void append_json_raw_pair(std::string *out, const char *key, const char *raw, bool first = false) {
  if (out == nullptr || key == nullptr || raw == nullptr) {
    return;
  }
  if (!first) {
    out->append(",");
  }
  append_json_string(out, key);
  out->append(":");
  out->append(raw);
}

std::string build_threshold_discovery_payload(const FrontendHaMqttSettings &settings, const EspectreDeviceInfo &info) {
  std::string out = "{";
  append_json_pair(&out, "name", "Threshold", true);
  append_json_pair(&out, "unique_id", settings.threshold_object_id.c_str());
  append_json_pair(&out, "object_id", settings.threshold_object_id.c_str());
  append_json_pair(&out, "state_topic", settings.threshold_state_topic.c_str());
  append_json_pair(&out, "command_topic", settings.threshold_command_topic.c_str());
  append_discovery_availability(&out, settings);
  append_json_raw_pair(&out, "min", "0");
  append_json_raw_pair(&out, "max", "1");
  append_json_raw_pair(&out, "step", "0.01");
  append_json_pair(&out, "mode", "box");
  append_json_pair(&out, "entity_category", "config");
  append_json_pair(&out, "icon", "mdi:pulse");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

std::string build_motion_hits_discovery_payload(const FrontendHaMqttSettings &settings,
                                                const EspectreDeviceInfo &info,
                                                bool motion_on) {
  const std::string &object_id = motion_on ? settings.motion_on_hits_object_id : settings.motion_off_hits_object_id;
  const std::string &state_topic = motion_on ? settings.motion_on_hits_state_topic : settings.motion_off_hits_state_topic;
  const std::string &command_topic =
      motion_on ? settings.motion_on_hits_command_topic : settings.motion_off_hits_command_topic;
  std::string out = "{";
  append_json_pair(&out, "name", motion_on ? "Motion On Hits" : "Motion Off Hits", true);
  append_json_pair(&out, "unique_id", object_id.c_str());
  append_json_pair(&out, "object_id", object_id.c_str());
  append_json_pair(&out, "state_topic", state_topic.c_str());
  append_json_pair(&out, "command_topic", command_topic.c_str());
  append_discovery_availability(&out, settings);
  append_json_raw_pair(&out, "min", "1");
  append_json_raw_pair(&out, "max", "20");
  append_json_raw_pair(&out, "step", "1");
  append_json_pair(&out, "mode", "box");
  append_json_pair(&out, "entity_category", "config");
  append_json_pair(&out, "icon", motion_on ? "mdi:motion-play-outline" : "mdi:motion-pause-outline");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

std::string build_recalibrate_button_discovery_payload(const FrontendHaMqttSettings &settings,
                                                       const EspectreDeviceInfo &info) {
  std::string out = "{";
  append_json_pair(&out, "name", "Recalibrate", true);
  append_json_pair(&out, "unique_id", settings.recalibrate_object_id.c_str());
  append_json_pair(&out, "object_id", settings.recalibrate_object_id.c_str());
  append_json_pair(&out, "command_topic", settings.calibrate_command_topic.c_str());
  append_discovery_availability(&out, settings);
  append_json_pair(&out, "payload_press", "ON");
  append_json_pair(&out, "entity_category", "config");
  append_json_pair(&out, "icon", "mdi:tune-vertical");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

std::string build_calibration_active_discovery_payload(const FrontendHaMqttSettings &settings,
                                                       const EspectreDeviceInfo &info) {
  std::string out = "{";
  append_json_pair(&out, "name", "Calibration Active", true);
  append_json_pair(&out, "unique_id", settings.calibration_active_object_id.c_str());
  append_json_pair(&out, "object_id", settings.calibration_active_object_id.c_str());
  append_json_pair(&out, "state_topic", settings.calibrate_state_topic.c_str());
  append_discovery_availability(&out, settings);
  append_json_pair(&out, "payload_on", "ON");
  append_json_pair(&out, "payload_off", "OFF");
  append_json_pair(&out, "entity_category", "diagnostic");
  append_json_pair(&out, "icon", "mdi:tune-vertical");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

std::string build_detector_discovery_payload(const FrontendHaMqttSettings &settings, const EspectreDeviceInfo &info) {
  std::string out = "{";
  append_json_pair(&out, "name", "Detection Profile", true);
  append_json_pair(&out, "unique_id", settings.detector_object_id.c_str());
  append_json_pair(&out, "object_id", settings.detector_object_id.c_str());
  append_json_pair(&out, "state_topic", settings.detector_state_topic.c_str());
  append_json_pair(&out, "command_topic", settings.detector_command_topic.c_str());
  append_discovery_availability(&out, settings);
  out.append(",\"options\":[");
  append_json_string(&out, "lightweight");
  out.push_back(',');
  append_json_string(&out, "high_accuracy");
  out.push_back(']');
  append_json_pair(&out, "entity_category", "config");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

std::string build_csi_traffic_mode_discovery_payload(const FrontendHaMqttSettings &settings,
                                                     const EspectreDeviceInfo &info) {
  std::string out = "{";
  append_json_pair(&out, "name", "CSI Traffic Ownership", true);
  append_json_pair(&out, "unique_id", settings.csi_traffic_mode_object_id.c_str());
  append_json_pair(&out, "object_id", settings.csi_traffic_mode_object_id.c_str());
  append_json_pair(&out, "state_topic", settings.csi_traffic_mode_state_topic.c_str());
  append_json_pair(&out, "command_topic", settings.csi_traffic_mode_command_topic.c_str());
  append_discovery_availability(&out, settings);
  out.append(",\"options\":[");
  append_json_string(&out, "internal");
  out.push_back(',');
  append_json_string(&out, "external");
  out.push_back(']');
  append_json_pair(&out, "entity_category", "config");
  append_json_pair(&out, "icon", "mdi:wifi-cog");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

std::string build_traffic_generator_mode_discovery_payload(const FrontendHaMqttSettings &settings,
                                                           const EspectreDeviceInfo &info) {
  std::string out = "{";
  append_json_pair(&out, "name", "CSI Traffic Source", true);
  append_json_pair(&out, "unique_id", settings.traffic_generator_mode_object_id.c_str());
  append_json_pair(&out, "object_id", settings.traffic_generator_mode_object_id.c_str());
  append_json_pair(&out, "state_topic", settings.traffic_generator_mode_state_topic.c_str());
  append_json_pair(&out, "command_topic", settings.traffic_generator_mode_command_topic.c_str());
  append_discovery_availability(&out, settings);
  out.append(",\"options\":[");
  append_json_string(&out, "ping");
  out.push_back(',');
  append_json_string(&out, "dns");
  out.push_back(',');
  append_json_string(&out, "dns_tcp");
  out.push_back(']');
  append_json_pair(&out, "entity_category", "config");
  append_json_pair(&out, "icon", "mdi:swap-horizontal");
  append_discovery_device(&out, settings, info);
  out.push_back('}');
  return out;
}

}  // namespace

bool frontend_ha_mqtt_enabled() {
#ifdef CONFIG_ESPECTRE_HA_DISCOVERY_ENABLED
  return CONFIG_ESPECTRE_HA_DISCOVERY_ENABLED;
#else
  return false;
#endif
}

FrontendHaMqttSettings build_frontend_ha_mqtt_settings(const EspectreDeviceConfig &config,
                                                       const EspectreDeviceInfo &info,
                                                       const char *frontend_name) {
  const std::string device_id = espectre_effective_device_id(config);
  const std::string effective_name = espectre_effective_device_label(config).empty()
                                         ? espectre_device_name(espectre_effective_device_id_u64(config),
                                                                info.chip.empty() ? nullptr : info.chip.c_str())
                                         : espectre_effective_device_label(config);
  const std::string frontend = frontend_name == nullptr || frontend_name[0] == '\0' ? "device" : frontend_name;
  const std::string device_key = sanitize_identifier(frontend + "_" + device_id);
  FrontendHaMqttSettings settings{};
  settings.discovery_prefix = CONFIG_ESPECTRE_HA_DISCOVERY_PREFIX;
  settings.birth_topic = kHomeAssistantBirthTopic;
  settings.availability_topic = espectre_topic(config, "health");
  settings.availability_template = kStatusAvailabilityTemplate;
  settings.motion_state_topic = ha_entity_base_topic(config, "motion/state");
  settings.movement_state_topic = ha_entity_base_topic(config, "movement/state");
  settings.threshold_state_topic = ha_entity_base_topic(config, "threshold/state");
  settings.threshold_command_topic = ha_entity_base_topic(config, "threshold/set");
  settings.motion_on_hits_state_topic = ha_entity_base_topic(config, "motion_on_hits/state");
  settings.motion_on_hits_command_topic = ha_entity_base_topic(config, "motion_on_hits/set");
  settings.motion_off_hits_state_topic = ha_entity_base_topic(config, "motion_off_hits/state");
  settings.motion_off_hits_command_topic = ha_entity_base_topic(config, "motion_off_hits/set");
  settings.calibrate_state_topic = ha_entity_base_topic(config, "calibrate/state");
  settings.calibrate_command_topic = ha_entity_base_topic(config, "calibrate/set");
  settings.detector_state_topic = ha_entity_base_topic(config, "detector/state");
  settings.detector_command_topic = ha_entity_base_topic(config, "detector/set");
  settings.csi_traffic_mode_state_topic = ha_entity_base_topic(config, "csi_traffic_mode/state");
  settings.csi_traffic_mode_command_topic = ha_entity_base_topic(config, "csi_traffic_mode/set");
  settings.traffic_generator_mode_state_topic = ha_entity_base_topic(config, "traffic_generator_mode/state");
  settings.traffic_generator_mode_command_topic = ha_entity_base_topic(config, "traffic_generator_mode/set");
  settings.diagnostics_command_topic = ha_entity_base_topic(config, "diagnostics/set");
  settings.ha_object_prefix = device_key;
  settings.motion_object_id = device_key + "_motion_detected";
  settings.movement_object_id = device_key + "_movement_score";
  settings.threshold_object_id = device_key + "_threshold";
  settings.motion_on_hits_object_id = device_key + "_motion_on_hits";
  settings.motion_off_hits_object_id = device_key + "_motion_off_hits";
  settings.recalibrate_object_id = device_key + "_recalibrate";
  settings.calibration_active_object_id = device_key + "_calibration_active";
  settings.detector_object_id = device_key + "_detection_profile";
  settings.csi_traffic_mode_object_id = device_key + "_csi_traffic_ownership";
  settings.traffic_generator_mode_object_id = device_key + "_csi_traffic_source";
  settings.diagnostics_object_id = device_key + "_refresh_diagnostics";
  settings.diagnostic_sensors.clear();
  settings.diagnostic_sensors.reserve(sizeof(kDiagnosticSensors) / sizeof(kDiagnosticSensors[0]));
  for (const DiagnosticSensorDef &def : kDiagnosticSensors) {
    FrontendHaDiagnosticSensor sensor;
    sensor.name = def.name;
    sensor.key = def.key;
    sensor.object_id = device_key + "_" + def.object_suffix;
    sensor.state_topic = ha_entity_base_topic(config, (std::string(def.key) + "/state").c_str());
    sensor.unit_of_measurement = def.unit;
    sensor.icon = def.icon;
    sensor.device_class = def.device_class;
    sensor.state_class_measurement = def.state_class_measurement;
    settings.diagnostic_sensors.push_back(std::move(sensor));
  }
  settings.device_id = device_id;
  settings.device_name = effective_name;
  settings.model = std::string("ESPectre ") + frontend;
  return settings;
}

bool build_frontend_ha_discovery_message(
    const FrontendHaMqttSettings &settings,
    const EspectreDeviceInfo &info,
    bool supports_detector,
    bool supports_motion_hits,
    bool supports_traffic_control, size_t index, FrontendHaDiscoveryMessage *message) {
  if (message == nullptr) return false;
  if (index-- == 0U) {
    *message = FrontendHaDiscoveryMessage{
        build_discovery_topic("binary_sensor", settings.discovery_prefix, settings.motion_object_id),
        build_motion_discovery_payload(settings, info),
    };
    return true;
  }
  if (index-- == 0U) {
    *message = FrontendHaDiscoveryMessage{
        build_discovery_topic("sensor", settings.discovery_prefix, settings.movement_object_id),
        build_movement_discovery_payload(settings, info),
    };
    return true;
  }
  for (const FrontendHaDiagnosticSensor &sensor : settings.diagnostic_sensors) {
    if (index-- == 0U) {
      *message = FrontendHaDiscoveryMessage{
          build_discovery_topic("sensor", settings.discovery_prefix, sensor.object_id),
          build_diagnostic_sensor_discovery_payload(settings, info, sensor),
      };
      return true;
    }
  }
  if (index-- == 0U) {
    *message = FrontendHaDiscoveryMessage{
        build_discovery_topic("button", settings.discovery_prefix, settings.diagnostics_object_id),
        build_diagnostics_button_discovery_payload(settings, info),
    };
    return true;
  }
  if (index-- == 0U) {
    *message = FrontendHaDiscoveryMessage{
        build_discovery_topic("number", settings.discovery_prefix, settings.threshold_object_id),
        build_threshold_discovery_payload(settings, info),
    };
    return true;
  }
  if (supports_motion_hits) {
    if (index-- == 0U) {
      *message = FrontendHaDiscoveryMessage{
          build_discovery_topic("number", settings.discovery_prefix, settings.motion_on_hits_object_id),
          build_motion_hits_discovery_payload(settings, info, true),
      };
      return true;
    }
    if (index-- == 0U) {
      *message = FrontendHaDiscoveryMessage{
          build_discovery_topic("number", settings.discovery_prefix, settings.motion_off_hits_object_id),
          build_motion_hits_discovery_payload(settings, info, false),
      };
      return true;
    }
  }
  if (supports_detector) {
    if (index-- == 0U) {
      *message = FrontendHaDiscoveryMessage{
          build_discovery_topic("select", settings.discovery_prefix, settings.detector_object_id),
          build_detector_discovery_payload(settings, info),
      };
      return true;
    }
  }
  if (supports_traffic_control) {
    if (index-- == 0U) {
      *message = FrontendHaDiscoveryMessage{
          build_discovery_topic("select", settings.discovery_prefix, settings.csi_traffic_mode_object_id),
          build_csi_traffic_mode_discovery_payload(settings, info),
      };
      return true;
    }
    if (index-- == 0U) {
      *message = FrontendHaDiscoveryMessage{
          build_discovery_topic("select", settings.discovery_prefix, settings.traffic_generator_mode_object_id),
          build_traffic_generator_mode_discovery_payload(settings, info),
      };
      return true;
    }
  }
  if (index-- == 0U) {
    *message = FrontendHaDiscoveryMessage{
        build_discovery_topic("button", settings.discovery_prefix, settings.recalibrate_object_id),
        build_recalibrate_button_discovery_payload(settings, info),
    };
    return true;
  }
  if (index-- == 0U) {
    *message = FrontendHaDiscoveryMessage{
        build_discovery_topic("binary_sensor", settings.discovery_prefix, settings.calibration_active_object_id),
        build_calibration_active_discovery_payload(settings, info),
    };
    return true;
  }
  for (const RetiredHaDiscovery &retired : kRetiredHaDiscoveries) {
    if (index-- == 0U) {
      *message = FrontendHaDiscoveryMessage{
          build_discovery_topic(retired.component, settings.discovery_prefix,
                                settings.ha_object_prefix + "_" + retired.suffix),
          "",
      };
      return true;
    }
  }
  return false;
}

std::vector<FrontendHaDiscoveryMessage> build_frontend_ha_discovery_messages(
    const FrontendHaMqttSettings &settings,
    const EspectreDeviceInfo &info,
    bool supports_detector,
    bool supports_motion_hits,
    bool supports_traffic_control) {
  std::vector<FrontendHaDiscoveryMessage> messages;
  FrontendHaDiscoveryMessage message;
  for (size_t index = 0U; build_frontend_ha_discovery_message(
           settings, info, supports_detector, supports_motion_hits,
           supports_traffic_control, index, &message); ++index) {
    messages.push_back(std::move(message));
  }
  return messages;
}

}  // namespace espectre
