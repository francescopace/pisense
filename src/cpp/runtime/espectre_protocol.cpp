/*
 * ESPectre - ESPectre Protocol
 *
 * Shared device and extensible command protocol types used by frontend
 * transports.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "espectre_protocol.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <utility>

#include "base_detector.h"
#include "protocol_json.h"
#include "raw_csi.h"
#include "runtime_diagnostics.h"
#include "runtime_sensing_schema.h"
#include "runtime_config_utils.h"

namespace espectre {

namespace {

bool validate_sdk_command_parameters(const std::vector<JsonObjectField> &fields,
                                     EspectreCommand *command, std::string *error);

constexpr EspectreApiRoute kApiRoutes[] = {
    {"GET", "/espectre/v1/health", "health", "health", EspectreDirectMethod::STATUS,
     EspectreApiRouteKind::RESOURCE, false, validate_sdk_command_parameters},
    {"GET", "/espectre/v1/device", "device", "device", EspectreDirectMethod::INFO,
     EspectreApiRouteKind::RESOURCE, false, validate_sdk_command_parameters},
    {"GET", "/espectre/v1/capabilities", "capabilities", "capabilities",
     EspectreDirectMethod::CAPABILITIES, EspectreApiRouteKind::RESOURCE, false, validate_sdk_command_parameters},
    {"GET", "/espectre/v1/sensing", "sensing", "sensing", EspectreDirectMethod::CONFIG,
     EspectreApiRouteKind::RESOURCE, false, validate_sdk_command_parameters},
    {"GET", "/espectre/v1/wifi", "wifi", "wifi", EspectreDirectMethod::CONFIG,
     EspectreApiRouteKind::RESOURCE, false, validate_sdk_command_parameters},
    {"GET", "/espectre/v1/wifi/access-points", "wifi/access-points", "wifi_access_points",
     EspectreDirectMethod::WIFI_ACCESS_POINTS, EspectreApiRouteKind::RESOURCE, false, validate_sdk_command_parameters},
    {"GET", "/espectre/v1/mqtt", "mqtt", "mqtt", EspectreDirectMethod::CONFIG,
     EspectreApiRouteKind::RESOURCE, false, validate_sdk_command_parameters},
    {"GET", "/espectre/v1/diagnostics", "diagnostics", "read_diagnostics",
     EspectreDirectMethod::DIAGNOSTICS, EspectreApiRouteKind::RESOURCE, false, validate_sdk_command_parameters},
    {"GET", "/espectre/v1/devices", "devices", "devices", EspectreDirectMethod::DISCOVER_PEERS,
     EspectreApiRouteKind::RESOURCE, false, validate_sdk_command_parameters},
    {"PATCH", "/espectre/v1/device", "update_device", "update_device",
     EspectreDirectMethod::SET_DEVICE_LABEL, EspectreApiRouteKind::OPERATION, false, validate_sdk_command_parameters},
    {"PATCH", "/espectre/v1/sensing", "update_sensing", "update_sensing",
     EspectreDirectMethod::SET_SENSING, EspectreApiRouteKind::OPERATION, false, validate_sdk_command_parameters},
    {"PATCH", "/espectre/v1/mqtt", "update_mqtt", "update_mqtt",
     EspectreDirectMethod::SET_MQTT_CONFIG, EspectreApiRouteKind::OPERATION, false, validate_sdk_command_parameters},
    {"POST", "/espectre/v1/sensing/calibrations", "recalibrate", "recalibrate",
     EspectreDirectMethod::RECALIBRATE, EspectreApiRouteKind::OPERATION, true, validate_sdk_command_parameters},
    {"POST", "/espectre/v1/wifi/scans", "scan_wifi", "scan_wifi",
     EspectreDirectMethod::SCAN_WIFI_ACCESS_POINTS, EspectreApiRouteKind::OPERATION, true, validate_sdk_command_parameters},
    {"PUT", "/espectre/v1/wifi/bssid", "set_wifi_bssid", "set_wifi_bssid",
     EspectreDirectMethod::SET_WIFI_BSSID, EspectreApiRouteKind::OPERATION, true, validate_sdk_command_parameters},
    {"DELETE", "/espectre/v1/wifi/bssid", "clear_wifi_bssid", "clear_wifi_bssid",
     EspectreDirectMethod::CLEAR_WIFI_BSSID, EspectreApiRouteKind::OPERATION, true, validate_sdk_command_parameters},
    {"DELETE", "/espectre/v1/wifi/credentials", "clear_wifi_credentials", "clear_wifi_credentials",
     EspectreDirectMethod::CLEAR_WIFI_CONFIG, EspectreApiRouteKind::OPERATION, true, validate_sdk_command_parameters},
    {"DELETE", "/espectre/v1/mqtt", "clear_mqtt", "clear_mqtt",
     EspectreDirectMethod::CLEAR_MQTT_CONFIG, EspectreApiRouteKind::OPERATION, false, validate_sdk_command_parameters},
    {"GET", "/espectre/v1/events", "events", "", EspectreDirectMethod::STATUS,
     EspectreApiRouteKind::STREAM, false},
    {"GET", "/espectre/v1/csi", "csi", "", EspectreDirectMethod::START_RAW_STREAM,
     EspectreApiRouteKind::STREAM, false},
};

constexpr EspectreApiEventDescriptor kApiEvents[] = {
    {"motion", EspectreEvent::TELEMETRY, EspectreDirectMethod::STATUS},
    {"health", EspectreEvent::STATUS, EspectreDirectMethod::STATUS},
    {"device", EspectreEvent::INFO, EspectreDirectMethod::INFO},
    {"sensing", EspectreEvent::CONFIG, EspectreDirectMethod::CONFIG},
    {"wifi", EspectreEvent::CONFIG, EspectreDirectMethod::WIFI_ACCESS_POINTS},
    {"fault", EspectreEvent::FAULT, EspectreDirectMethod::STATUS},
};

}  // namespace

const EspectreApiRoute *espectre_api_routes(size_t *count) {
  if (count != nullptr) *count = sizeof(kApiRoutes) / sizeof(kApiRoutes[0]);
  return kApiRoutes;
}

const EspectreApiEventDescriptor *espectre_api_events(size_t *count) {
  if (count != nullptr) *count = sizeof(kApiEvents) / sizeof(kApiEvents[0]);
  return kApiEvents;
}

bool validate_protocol_extension(const EspectreProtocolExtension &extension, std::string *error) {
  const auto reject = [error]() {
    if (error != nullptr) *error = "invalid or conflicting protocol extension";
    return false;
  };
  const auto identifier = [](const char *text, bool path) {
    if (text == nullptr || text[0] == '\0') return false;
    for (const unsigned char *ch = reinterpret_cast<const unsigned char *>(text); *ch; ++ch) {
      if (!std::isalnum(*ch) && *ch != '_' && !(path && (*ch == '/' || *ch == '-'))) return false;
    }
    return true;
  };
  for (size_t index = 0; index < extension.routes.size(); ++index) {
    const auto &route = extension.routes[index];
    if (route.http_method == nullptr || route.path == nullptr || route.validate == nullptr ||
        !identifier(route.command, false) || !identifier(route.name, true) ||
        !identifier(route.path, true) || std::string(route.path).rfind("/espectre/v1/", 0) != 0) return reject();
    const std::string method = route.http_method;
    if ((route.kind == EspectreApiRouteKind::RESOURCE && method != "GET") ||
        (route.kind == EspectreApiRouteKind::OPERATION && method != "POST" && method != "PUT" &&
         method != "PATCH" && method != "DELETE") ||
        (route.kind != EspectreApiRouteKind::RESOURCE && route.kind != EspectreApiRouteKind::OPERATION)) return reject();
    for (const auto &base : kApiRoutes) {
      if (std::string(route.path) == base.path || std::string(route.command) == base.command ||
          std::string(route.name) == base.name) return reject();
    }
    for (size_t previous = 0; previous < index; ++previous) {
      const auto &other = extension.routes[previous];
      if ((method == other.http_method && std::string(route.path) == other.path) ||
          std::string(route.command) == other.command || std::string(route.name) == other.name) return reject();
    }
  }
  for (size_t index = 0; index < extension.events.size(); ++index) {
    const auto &event = extension.events[index];
    if (!identifier(event.c_str(), false)) return reject();
    for (const auto &base : kApiEvents) if (event == base.name) return reject();
    for (size_t previous = 0; previous < index; ++previous) {
      if (event == extension.events[previous]) return reject();
    }
  }
  return true;
}

const EspectreExtensionRoute *find_extension_route(const EspectreProtocolExtension *extension,
                                                 const std::string &command) {
  if (extension == nullptr || !validate_protocol_extension(*extension)) return nullptr;
  for (const auto &route : extension->routes) {
    if (command == route.command) return &route;
  }
  return nullptr;
}

namespace {

const char *motion_state_name(MotionState state) {
  return state == MotionState::MOTION ? "motion" : "idle";
}

double json_finite(float value) {
  return std::isfinite(value) ? static_cast<double>(value) : 0.0;
}

void append_json_field_prefix(std::string *out, const char *key) {
  out->append(",\"");
  out->append(key);
  out->append("\":");
}

void append_json_uint_field(std::string *out, const char *key, uint64_t value) {
  append_json_field_prefix(out, key);
  out->append(std::to_string(value));
}

void append_json_float_field(std::string *out, const char *key, float value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.6g", static_cast<double>(value));
  append_json_field_prefix(out, key);
  out->append(text);
}

void append_json_null_field(std::string *out, const char *key) {
  append_json_field_prefix(out, key);
  out->append("null");
}

bool command_id_accepted(const std::string &value) {
  if (value.empty() || value.size() > ESPECTRE_COMMAND_ID_MAX_LENGTH) return false;
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) || character == '_' || character == '-' || character == '.' ||
           character == ':';
  });
}

bool api_route_supported(const EspectreApiRoute &route,
                         const EspectreCapabilityProfile &capabilities) {
  if (std::strcmp(route.name, "wifi") == 0) {
    return capabilities.has(EspectreConfigSection::WIFI);
  }
  if (std::strcmp(route.name, "mqtt") == 0) {
    return capabilities.has(EspectreConfigSection::MQTT);
  }
  if (std::strcmp(route.name, "csi") == 0) {
    return capabilities.supports(EspectreDirectMethod::START_RAW_STREAM) &&
           capabilities.supports(EspectreDirectMethod::STOP_RAW_STREAM);
  }
  return capabilities.supports(route.capability);
}

template <typename Route>
void append_operation_descriptor(std::string *out,
                                 bool *first,
                                 const Route &route,
                                 const char *name = nullptr) {
  if (out == nullptr || first == nullptr) {
    return;
  }
  if (!*first) {
    out->append(",");
  }
  *first = false;
  out->append("{\"name\":");
  append_json_string(out, name != nullptr ? name : route.name);
  out->append(",\"method\":");
  append_json_string(out, route.http_method);
  out->append(",\"path\":");
  append_json_string(out, route.path);
  out->append("}");
}

void append_capability_commands(std::string *out,
                                const EspectreCapabilityProfile &capabilities) {
  if (out == nullptr) {
    return;
  }
  bool first = true;
  size_t route_count = 0U;
  const EspectreApiRoute *routes = espectre_api_routes(&route_count);
  for (size_t index = 0U; index < route_count; ++index) {
    const EspectreApiRoute &route = routes[index];
    const bool correlated_diagnostics =
        route.kind == EspectreApiRouteKind::RESOURCE &&
        std::strcmp(route.command, "read_diagnostics") == 0;
    if ((route.kind == EspectreApiRouteKind::OPERATION || correlated_diagnostics) &&
        api_route_supported(route, capabilities)) {
      append_operation_descriptor(out, &first, route,
                                  correlated_diagnostics ? route.command : nullptr);
    }
  }
  if (capabilities.extension != nullptr && validate_protocol_extension(*capabilities.extension)) {
    for (const auto &route : capabilities.extension->routes) {
      if (route.kind == EspectreApiRouteKind::OPERATION) append_operation_descriptor(out, &first, route);
    }
  }
}

bool parse_float_value(const std::string &value, float *out) {
  if (out == nullptr || value.empty()) {
    return false;
  }
  char *end_ptr = nullptr;
  errno = 0;
  const float parsed = std::strtof(value.c_str(), &end_ptr);
  if (end_ptr == value.c_str() || end_ptr == nullptr || *end_ptr != '\0' || errno == ERANGE ||
      !std::isfinite(parsed)) {
    return false;
  }
  *out = parsed;
  return true;
}

bool parse_uint16_value(const std::string &value, uint16_t *out) {
  if (out == nullptr || value.empty()) {
    return false;
  }
  char *end_ptr = nullptr;
  errno = 0;
  const unsigned long parsed = std::strtoul(value.c_str(), &end_ptr, 10);
  if (end_ptr == value.c_str() || end_ptr == nullptr || *end_ptr != '\0' || errno == ERANGE || parsed > 65535UL) {
    return false;
  }
  *out = static_cast<uint16_t>(parsed);
  return true;
}

bool parse_uint8_value(const std::string &value, uint8_t *out) {
  uint16_t parsed = 0U;
  if (!parse_uint16_value(value, &parsed) || parsed > UINT8_MAX) {
    return false;
  }
  *out = static_cast<uint8_t>(parsed);
  return true;
}

std::string normalize_chip_label(const char *chip) {
  if (chip == nullptr || chip[0] == '\0') {
    return "UNK";
  }
  std::string normalized;
  normalized.reserve(8);
  for (const char *p = chip; *p != '\0'; ++p) {
    const unsigned char ch = static_cast<unsigned char>(*p);
    if (std::isalnum(ch)) {
      normalized.push_back(static_cast<char>(std::toupper(ch)));
    }
  }
  if (normalized == "ESP32C3") return "C3";
  if (normalized == "ESP32C5") return "C5";
  if (normalized == "ESP32C6") return "C6";
  if (normalized == "ESP32S2") return "S2";
  if (normalized == "ESP32S3") return "S3";
  if (normalized == "ESP32") return "ESP32";
  return normalized.empty() ? "UNK" : normalized;
}


bool assign_config_field(const std::string &field, const std::string &value, EspectreDeviceConfig *config) {
  if (field == "device_label") {
    config->device_label = value;
    return true;
  }
  return false;
}

bool parse_mqtt_port_value(const std::string &value, uint16_t *port) {
  return parse_uint16_value(value, port) && port != nullptr && *port > 0U;
}

bool ipv4_literal_accepted(const std::string &value) {
  size_t start = 0U;
  unsigned octets = 0U;
  while (start < value.size()) {
    const size_t end = value.find('.', start);
    const size_t length = (end == std::string::npos ? value.size() : end) - start;
    if (length == 0U || length > 3U || ++octets > 4U) {
      return false;
    }
    unsigned octet = 0U;
    for (size_t index = start; index < start + length; ++index) {
      const unsigned char character = static_cast<unsigned char>(value[index]);
      if (!std::isdigit(character)) {
        return false;
      }
      octet = octet * 10U + static_cast<unsigned>(character - '0');
    }
    if (octet > 255U) {
      return false;
    }
    if (end == std::string::npos) {
      start = value.size();
    } else {
      start = end + 1U;
    }
  }
  return octets == 4U && !value.empty() && value.back() != '.';
}

bool ipv6_side_groups_accepted(const std::string &side, bool allow_ipv4, unsigned *groups) {
  if (groups == nullptr) {
    return false;
  }
  *groups = 0U;
  if (side.empty()) {
    return true;
  }
  size_t start = 0U;
  while (start < side.size()) {
    const size_t end = side.find(':', start);
    const std::string group = side.substr(start, (end == std::string::npos ? side.size() : end) - start);
    if (group.empty()) {
      return false;
    }
    if (group.find('.') != std::string::npos) {
      if (!allow_ipv4 || end != std::string::npos || !ipv4_literal_accepted(group)) {
        return false;
      }
      *groups += 2U;
    } else {
      if (group.size() > 4U || !std::all_of(group.begin(), group.end(), [](unsigned char character) {
            return std::isxdigit(character) != 0;
          })) {
        return false;
      }
      *groups += 1U;
    }
    if (*groups > 8U) {
      return false;
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1U;
  }
  return !side.empty() && side.back() != ':';
}

bool ipv6_literal_accepted(const std::string &value) {
  const size_t compression = value.find("::");
  if (compression != std::string::npos && value.find("::", compression + 2U) != std::string::npos) {
    return false;
  }
  unsigned left_groups = 0U;
  unsigned right_groups = 0U;
  if (compression == std::string::npos) {
    return ipv6_side_groups_accepted(value, true, &left_groups) && left_groups == 8U;
  }
  const std::string left = value.substr(0U, compression);
  const std::string right = value.substr(compression + 2U);
  if (!ipv6_side_groups_accepted(left, false, &left_groups) ||
      !ipv6_side_groups_accepted(right, true, &right_groups)) {
    return false;
  }
  return left_groups + right_groups < 8U;
}

bool dns_hostname_accepted(const std::string &value) {
  if (value.empty() || value.size() > 253U || value.front() == '.' || value.back() == '.') {
    return false;
  }
  size_t label_start = 0U;
  while (label_start < value.size()) {
    const size_t label_end = value.find('.', label_start);
    const size_t label_size = (label_end == std::string::npos ? value.size() : label_end) - label_start;
    if (label_size == 0U || label_size > 63U || value[label_start] == '-' ||
        value[label_start + label_size - 1U] == '-') {
      return false;
    }
    for (size_t index = label_start; index < label_start + label_size; ++index) {
      const unsigned char character = static_cast<unsigned char>(value[index]);
      if (!std::isalnum(character) && character != '-') {
        return false;
      }
    }
    if (label_end == std::string::npos) {
      break;
    }
    label_start = label_end + 1U;
  }
  return true;
}

bool mqtt_host_accepted(const std::string &value) {
  if (value.empty() || value.size() > 253U || value.find_first_of("/?#@[]") != std::string::npos) {
    return false;
  }
  if (std::any_of(value.begin(), value.end(), [](unsigned char character) {
        return std::iscntrl(character) != 0 || std::isspace(character) != 0;
      })) {
    return false;
  }
  if (value.find(':') != std::string::npos) {
    return ipv6_literal_accepted(value);
  }
  const bool numeric_address = std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isdigit(character) != 0 || character == '.';
  });
  return numeric_address ? ipv4_literal_accepted(value) : dns_hostname_accepted(value);
}

bool single_line_string(const std::string &value, size_t max_size) {
  return value.size() <= max_size && value.find_first_of("\r\n\0", 0U, 3U) == std::string::npos;
}

bool bssid_string_accepted(const std::string &value) {
  if (value.empty()) {
    return true;
  }
  if (value.size() != 17U) {
    return false;
  }
  for (size_t index = 0U; index < value.size(); ++index) {
    if (index % 3U == 2U) {
      if (value[index] != ':') {
        return false;
      }
    } else if (!std::isxdigit(static_cast<unsigned char>(value[index]))) {
      return false;
    }
  }
  return true;
}

bool validate_sdk_command_parameters(const std::vector<JsonObjectField> &fields,
                                     EspectreCommand *command, std::string *error) {
  EspectreCommand parsed = *command;
  const auto reject = [&](const char *message) {
    if (error != nullptr) *error = message;
    *command = parsed;
    return false;
  };
  const auto string_field = [&](const char *name, std::string *value) {
    const JsonObjectField *field = find_json_object_field(fields, name);
    if (field == nullptr || field->type != JsonValueType::STRING || value == nullptr) {
      return false;
    }
    *value = field->value;
    return true;
  };
  const auto number_field = [&](const char *name, std::string *value) {
    const JsonObjectField *field = find_json_object_field(fields, name);
    if (field == nullptr || field->type != JsonValueType::NUMBER || value == nullptr) {
      return false;
    }
    *value = field->value;
    return true;
  };
  const auto bool_field = [&](const char *name, bool *value) {
    const JsonObjectField *field = find_json_object_field(fields, name);
    if (field == nullptr || field->type != JsonValueType::BOOLEAN || value == nullptr) {
      return false;
    }
    *value = field->value == "true";
    return true;
  };

  const auto field_allowed = [&parsed](const std::string &name) {
    if (name == "command_id" || name == "command") return true;
    if (parsed.command == "update_sensing") {
      return name == "enabled" || name == "threshold" || name == "motion_on_hits" ||
             name == "motion_off_hits" || name == "detector" ||
             name == "csi_traffic_mode" || name == "traffic_generator_mode";
    }
    if (parsed.command == "update_device") return name == "label";
    if (parsed.command == "set_wifi_bssid") return name == "bssid" || name == "force";
    if (parsed.command == "update_mqtt") {
      return name == "scheme" || name == "host" || name == "port" ||
             name == "username" || name == "password" || name == "topic_prefix";
    }
    return false;
  };
  for (const JsonObjectField &field : fields) {
    if (!field_allowed(field.name)) return reject("unknown command parameter");
  }
  if (parsed.command == "update_sensing") {
    if (find_json_object_field(fields, "enabled") != nullptr) {
      if (!bool_field("enabled", &parsed.sensing_enabled)) {
        return reject("invalid sensing state (accepted: boolean enabled)");
      }
      parsed.has_sensing_enabled = true;
    }
    if (find_json_object_field(fields, "threshold") != nullptr) {
      std::string token;
      if (!number_field("threshold", &token) || !parse_float_value(token, &parsed.threshold) ||
          !validate_runtime_threshold(parsed.threshold)) {
        return reject("invalid threshold (accepted: 0.0-1.0)");
      }
      parsed.has_threshold = true;
    }
    const bool has_on = find_json_object_field(fields, "motion_on_hits") != nullptr;
    const bool has_off = find_json_object_field(fields, "motion_off_hits") != nullptr;
    if (has_on != has_off) return reject("motion_on_hits and motion_off_hits must be updated together");
    if (has_on) {
      std::string on_token;
      std::string off_token;
      if (!number_field("motion_on_hits", &on_token) || !number_field("motion_off_hits", &off_token) ||
          !parse_uint8_value(on_token, &parsed.motion_on_hits) ||
          !parse_uint8_value(off_token, &parsed.motion_off_hits) ||
          parsed.motion_on_hits < RUNTIME_MOTION_HITS_MIN || parsed.motion_on_hits > RUNTIME_MOTION_HITS_MAX ||
          parsed.motion_off_hits < RUNTIME_MOTION_HITS_MIN || parsed.motion_off_hits > RUNTIME_MOTION_HITS_MAX) {
        return reject("invalid motion hits (accepted: motion_on_hits and motion_off_hits in 1-20)");
      }
      parsed.has_motion_hits = true;
    }
    if (find_json_object_field(fields, "detector") != nullptr) {
      if (!string_field("detector", &parsed.detector) ||
          (parsed.detector != RUNTIME_DETECTION_ALGORITHM_LIGHTWEIGHT_NAME &&
           parsed.detector != RUNTIME_DETECTION_ALGORITHM_HIGH_ACCURACY_NAME)) {
        return reject("invalid detector (accepted: lightweight and high_accuracy)");
      }
      parsed.has_detector = true;
    }
    if (find_json_object_field(fields, "csi_traffic_mode") != nullptr) {
      if (!string_field("csi_traffic_mode", &parsed.csi_traffic_mode) ||
          (parsed.csi_traffic_mode != RUNTIME_CSI_TRAFFIC_MODE_INTERNAL_NAME &&
           parsed.csi_traffic_mode != RUNTIME_CSI_TRAFFIC_MODE_EXTERNAL_NAME)) {
        return reject("invalid csi traffic mode (accepted: internal and external)");
      }
      parsed.has_csi_traffic_mode = true;
    }
    if (find_json_object_field(fields, "traffic_generator_mode") != nullptr) {
      if (!string_field("traffic_generator_mode", &parsed.traffic_generator_mode) ||
          (parsed.traffic_generator_mode != RUNTIME_TRAFFIC_GENERATOR_MODE_PING_NAME &&
           parsed.traffic_generator_mode != RUNTIME_TRAFFIC_GENERATOR_MODE_DNS_NAME &&
           parsed.traffic_generator_mode != RUNTIME_TRAFFIC_GENERATOR_MODE_DNS_TCP_NAME)) {
        return reject("invalid traffic generator mode (accepted: ping, dns, and dns_tcp)");
      }
      parsed.has_traffic_generator_mode = true;
    }
    if (!parsed.has_sensing_enabled && !parsed.has_threshold && !parsed.has_motion_hits &&
        !parsed.has_detector && !parsed.has_csi_traffic_mode && !parsed.has_traffic_generator_mode) {
      return reject("sensing update is empty");
    }
  } else if (parsed.command == "update_device") {
    if (!string_field("label", &parsed.device_label) ||
        parsed.device_label.size() > ESPECTRE_DEVICE_LABEL_MAX_LENGTH ||
        parsed.device_label.find_first_of("\r\n\0", 0U, 3U) != std::string::npos) {
      return reject("invalid device label (accepted: a single-line string)");
    }
    parsed.has_device_label = true;
  } else if (parsed.command == "set_wifi_bssid") {
    if (!string_field("bssid", &parsed.wifi_bssid) || parsed.wifi_bssid.empty() ||
        !bssid_string_accepted(parsed.wifi_bssid)) {
      return reject("invalid BSSID (accepted: six hexadecimal octets)");
    }
    parsed.has_wifi_bssid = true;
    if (find_json_object_field(fields, "force") != nullptr) {
      if (!bool_field("force", &parsed.wifi_bssid_force)) {
        return reject("invalid force flag (accepted: boolean)");
      }
      parsed.has_wifi_bssid_force = true;
    }
  } else if (parsed.command == "clear_wifi_credentials") {
    // No additional payload required.
  } else if (parsed.command == "update_mqtt") {
    if (!string_field("scheme", &parsed.mqtt_scheme)) {
      return reject("missing or invalid MQTT scheme");
    }
    parsed.has_mqtt_scheme = true;
    if (!string_field("host", &parsed.mqtt_host)) return reject("missing or invalid MQTT host");
    parsed.has_mqtt_host = true;
    std::string port_token;
    if (!number_field("port", &port_token) || !parse_mqtt_port_value(port_token, &parsed.mqtt_port)) {
      return reject("missing or invalid MQTT port (accepted: 1..65535)");
    }
    parsed.has_mqtt_port = true;
    EspectreDeviceConfig mqtt_config;
    mqtt_config.mqtt_scheme = parsed.mqtt_scheme;
    mqtt_config.mqtt_host = parsed.mqtt_host;
    mqtt_config.mqtt_port = parsed.mqtt_port;
    std::string mqtt_error;
    if (!validate_espectre_mqtt_config(mqtt_config, &mqtt_error)) {
      return reject(mqtt_error.c_str());
    }
    if (find_json_object_field(fields, "username") != nullptr) {
      if (!string_field("username", &parsed.mqtt_username) || !single_line_string(parsed.mqtt_username, 128U)) {
        return reject("invalid MQTT username");
      }
      parsed.has_mqtt_username = true;
    }
    if (find_json_object_field(fields, "password") != nullptr) {
      if (!string_field("password", &parsed.mqtt_password) || !single_line_string(parsed.mqtt_password, 256U)) {
        return reject("invalid MQTT password");
      }
      parsed.has_mqtt_password = true;
    }
    if (find_json_object_field(fields, "topic_prefix") != nullptr) {
      if (!string_field("topic_prefix", &parsed.mqtt_topic_prefix) ||
          !single_line_string(parsed.mqtt_topic_prefix, 128U)) {
        return reject("invalid MQTT topic prefix");
      }
      parsed.has_mqtt_topic_prefix = true;
    }
  } else if (parsed.command == "clear_mqtt") {
    // No additional payload required.
  } else if (parsed.command == "recalibrate" || parsed.command == "scan_wifi" ||
             parsed.command == "clear_wifi_bssid") {
    // No additional payload required.
  } else if (parsed.command == "device" ||
             parsed.command == "capabilities" || parsed.command == "health" ||
             parsed.command == "sensing" || parsed.command == "wifi" ||
             parsed.command == "mqtt" ||
             parsed.command == "devices" || parsed.command == "wifi_access_points" ||
             parsed.command == "read_diagnostics") {
    // No additional payload required.
  } else {
    // The command engine owns registry filtering and returns the stable
    // `unsupported` code. The transport parser only validates the envelope and
    // parameters it knows how to decode.
  }
  *command = std::move(parsed);
  return true;
}

bool parse_command_fields(const std::string &command_id,
                          const std::string &command_name,
                          const std::vector<JsonObjectField> &fields,
                          const std::string &parameters,
                          const EspectreProtocolExtension *extension,
                          EspectreCommand *command,
                          std::string *error) {
  if (command == nullptr) {
    if (error != nullptr) *error = "command output is required";
    return false;
  }
  *command = EspectreCommand{};
  command->command_id = command_id;
  command->command = command_name;
  if (command_name.empty()) {
    if (error != nullptr) *error = "missing command";
    return false;
  }
  // Unknown commands retain the existing unsupported-command dispatch contract.
  EspectreCommandValidator validator = validate_sdk_command_parameters;
  if (const auto *route = find_extension_route(extension, command_name)) {
    command->extension_parameters = parameters;
    validator = route->validate;
  } else {
    for (const auto &route : kApiRoutes) {
      if (command_name == route.command) {
        validator = route.validate;
        break;
      }
    }
  }
  return validator(fields, command, error);
}

}  // namespace

std::string format_espectre_device_id(uint64_t device_id) {
  char text[sizeof("0123456789abcdef")] = {0};
  std::snprintf(text, sizeof(text), "%016" PRIx64, device_id);
  return text;
}

bool parse_espectre_device_id(const std::string &value, uint64_t *device_id) {
  if (device_id == nullptr || value.empty()) {
    return false;
  }
  char *end_ptr = nullptr;
  errno = 0;
  const unsigned long long parsed = std::strtoull(value.c_str(), &end_ptr, 16);
  if (end_ptr == value.c_str() || end_ptr == nullptr || *end_ptr != '\0' || errno == ERANGE) {
    return false;
  }
  *device_id = static_cast<uint64_t>(parsed);
  return true;
}

uint64_t espectre_device_id_from_mac(const uint8_t *mac, size_t mac_len) {
  if (mac == nullptr || mac_len < 6U) {
    return ESPECTRE_DEFAULT_DEVICE_ID;
  }
  uint64_t device_id = 0U;
  for (size_t i = 0U; i < 6U; ++i) {
    device_id = (device_id << 8U) | static_cast<uint64_t>(mac[i]);
  }
  return device_id;
}

std::string espectre_device_name(uint64_t device_id, const char *chip) {
  const std::string chip_label = normalize_chip_label(chip);
  const std::string formatted_id = format_espectre_device_id(device_id);
  const std::string suffix = formatted_id.size() >= 6 ? formatted_id.substr(formatted_id.size() - 6) : formatted_id;
  return std::string("ESPectre ") + chip_label + " " + suffix;
}

uint64_t espectre_effective_device_id_u64(const EspectreDeviceConfig &config) {
  return config.device_id;
}

std::string espectre_effective_device_id(const EspectreDeviceConfig &config) {
  return format_espectre_device_id(espectre_effective_device_id_u64(config));
}

std::string espectre_effective_device_label(const EspectreDeviceConfig &config) {
  return config.device_label;
}

EspectreDeviceInfo normalize_protocol_device_info(const EspectreDeviceInfo &info,
                                                  const RuntimeSnapshot *snapshot,
                                                  const char *default_frontend,
                                                  const char *default_chip) {
  EspectreDeviceInfo normalized = info;
  normalized.frontend =
      normalized.frontend.empty() ? (default_frontend != nullptr ? default_frontend : "native") : normalized.frontend;
  normalized.firmware_version = normalized.firmware_version.empty() ? "unknown" : normalized.firmware_version;
  normalized.chip = normalized.chip.empty() ? (default_chip != nullptr ? default_chip : "unknown") : normalized.chip;
  if (normalized.detector.empty() && snapshot != nullptr && snapshot->detector_name != nullptr) {
    normalized.detector = snapshot->detector_name;
  }
  if (snapshot != nullptr) {
    if (normalized.csi_profile.empty()) {
      normalized.csi_profile =
          csi_capture_profile_name(snapshot->csi_capture_profile);
    }
    if (normalized.network.channel == 0U) {
      normalized.network.channel = snapshot->link_channel;
    }
  }
  return normalized;
}

void clear_espectre_mqtt_config(EspectreDeviceConfig *config) {
  if (config == nullptr) {
    return;
  }
  config->mqtt_scheme.clear();
  config->mqtt_host.clear();
  config->mqtt_port = 0U;
  config->mqtt_username.clear();
  config->mqtt_password.clear();
  config->topic_prefix = ESPECTRE_TOPIC_PREFIX;
}

bool validate_espectre_mqtt_config(const EspectreDeviceConfig &config, std::string *error) {
  const auto reject = [error](const char *message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  if (config.mqtt_scheme.empty()) {
    return reject("missing MQTT scheme");
  }
  if (config.mqtt_scheme != "mqtt" && config.mqtt_scheme != "mqtts") {
    return reject("invalid MQTT scheme (accepted: mqtt and mqtts)");
  }
  if (config.mqtt_host.empty()) {
    return reject("missing MQTT host");
  }
  if (!mqtt_host_accepted(config.mqtt_host)) {
    return reject("invalid MQTT host");
  }
  if (config.mqtt_port == 0U) {
    return reject("missing MQTT port (accepted: 1..65535)");
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool espectre_mqtt_configured(const EspectreDeviceConfig &config) {
  return validate_espectre_mqtt_config(config, nullptr);
}

std::string espectre_topic(const EspectreDeviceConfig &config, const char *suffix) {
  std::string topic = config.topic_prefix.empty() ? ESPECTRE_TOPIC_PREFIX : config.topic_prefix;
  if (!topic.empty() && topic.back() == '/') {
    topic.pop_back();
  }
  topic.append("/");
  topic.append(espectre_effective_device_id(config));
  topic.append("/");
  topic.append(suffix != nullptr ? suffix : "");
  return topic;
}

std::string espectre_health_payload(const EspectreDeviceConfig &config, bool online, uint32_t timestamp_ms) {
  (void) config;
  char line[128];
  std::snprintf(line,
                sizeof(line),
                "{\"status\":\"%s\",\"online\":%s,\"uptime_s\":%u,\"timestamp_ms\":%u}",
                online ? "ok" : "offline",
                online ? "true" : "false",
                static_cast<unsigned>(timestamp_ms / 1000U),
                static_cast<unsigned>(timestamp_ms));
  return line;
}

std::string espectre_device_payload(const EspectreDeviceConfig &config, const EspectreDeviceInfo &info) {
  const std::string device_id = espectre_effective_device_id(config);
  const std::string device_name = espectre_device_name(espectre_effective_device_id_u64(config),
                                                       info.chip.empty() ? nullptr : info.chip.c_str());
  const std::string device_label = espectre_effective_device_label(config);
  std::string out;
  out.reserve(192U + device_id.size() + device_name.size() + device_label.size() +
              info.frontend.size() + info.firmware_version.size() + info.chip.size() +
              info.detector.size() + info.csi_profile.size() +
              info.csi_traffic_mode.size() + info.traffic_mode.size());
  out = "{";
  append_json_pair(&out, "device_id", device_id.c_str(), true);
  append_json_pair(&out, "name", device_label.empty() ? device_name.c_str() : device_label.c_str());
  append_json_pair(&out, "label", device_label.c_str());
  append_json_pair(&out, "frontend", info.frontend.empty() ? "native" : info.frontend.c_str());
  append_json_pair(&out, "firmware", info.firmware_version.empty() ? "unknown" : info.firmware_version.c_str());
  append_json_pair(&out, "chip", info.chip.empty() ? "unknown" : info.chip.c_str());
  if (!info.csi_profile.empty()) {
    append_json_pair(&out, "csi_profile", info.csi_profile.c_str());
  }
  out += "}";
  return out;
}

std::string espectre_capabilities_payload(const EspectreDeviceConfig &config,
                                          const EspectreDeviceInfo &info,
                                          const EspectreCapabilityProfile &capabilities) {
  (void) config;
  std::string out;
  out.reserve(1536U);
  out = "{";
  append_json_pair(&out, "protocol_version", ESPECTRE_PROTOCOL_VERSION, true);
  out += ",\"operations\":[";
  append_capability_commands(&out, capabilities);
  out += "],\"events\":[";
  bool first_event = true;
  size_t event_count = 0U;
  const EspectreApiEventDescriptor *events = espectre_api_events(&event_count);
  for (size_t index = 0U; index < event_count; ++index) {
    const EspectreApiEventDescriptor &event = events[index];
    if (!capabilities.publishes(event.event) ||
        !capabilities.supports(event.capability)) {
      continue;
    }
    if (!first_event) out += ',';
    append_json_string(&out, event.name);
    first_event = false;
  }

  if (capabilities.extension != nullptr && validate_protocol_extension(*capabilities.extension)) {
    for (const auto &event : capabilities.extension->events) {
      if (!first_event) out += ',';
      append_json_string(&out, event.c_str());
      first_event = false;
    }
  }
  out += "],\"resources\":[";
  bool first_resource = true;
  size_t route_count = 0U;
  const EspectreApiRoute *routes = espectre_api_routes(&route_count);
  for (size_t index = 0U; index < route_count; ++index) {
    const EspectreApiRoute &route = routes[index];
    if (route.kind != EspectreApiRouteKind::RESOURCE ||
        !api_route_supported(route, capabilities)) {
      continue;
    }
    if (!first_resource) out += ',';
    append_json_string(&out, route.name);
    first_resource = false;
  }

  if (capabilities.extension != nullptr && validate_protocol_extension(*capabilities.extension)) {
    for (const auto &route : capabilities.extension->routes) {
      if (route.kind != EspectreApiRouteKind::RESOURCE) continue;
      if (!first_resource) out += ',';
      append_json_string(&out, route.name);
      first_resource = false;
    }
  }
  const bool supports_raw_csi =
      capabilities.supports(EspectreDirectMethod::START_RAW_STREAM) &&
      capabilities.supports(EspectreDirectMethod::STOP_RAW_STREAM);
  out += "],\"features\":{\"csi\":";
  out += supports_raw_csi ? "true" : "false";
  out += "}";
  if (supports_raw_csi) {
    out += ",\"csi\":{\"endpoint\":\"/espectre/v1/csi\","
           "\"transport\":\"http\",\"protocol_version\":";
    out += std::to_string(static_cast<unsigned>(ESPECTRE_RAW_CSI_PROTOCOL_VERSION));
    out += ",\"record_version\":8,\"frame_prefix_bytes\":60,"
           "\"queue_depth\":16,\"batch_records\":4,"
           "\"traffic_udp_port\":";
    out += std::to_string(info.csi_traffic_udp_port == 0U ? RUNTIME_CSI_TRAFFIC_UDP_PORT_DEFAULT
                                                          : info.csi_traffic_udp_port);
    out += ",\"traffic_multicast_group\":";
    append_json_string(&out, info.csi_traffic_multicast_group.c_str());
    out += ",\"marker\":";
    append_json_string(&out, RUNTIME_CSI_TRAFFIC_MARKER_UTF8);
    out += "}";
  }
  out += "}";
  return out;
}

std::string espectre_capabilities_payload(const EspectreDeviceConfig &config,
                                          const EspectreDeviceInfo &info,
                                          bool supports_status,
                                          bool supports_config,
                                          bool supports_sensing_control,
                                          bool supports_wifi_bssid,
                                          bool supports_mqtt_config,
                                          bool supports_peer_discovery,
                                          bool supports_raw_csi,
                                          const EspectreProtocolExtension *extension) {
  EspectreCapabilityProfile capabilities;
  capabilities.extension = extension;
  using Method = EspectreDirectMethod;
  capabilities.set(Method::CAPABILITIES);
  capabilities.set(Method::INFO, info.supports_info);
  capabilities.set(Method::STATUS, supports_status);
  capabilities.set(Method::CONFIG, supports_config);
  capabilities.set(Method::DIAGNOSTICS, info.supports_diagnostics);
  capabilities.set(Method::SET_SENSING, supports_sensing_control);
  capabilities.set(Method::SET_DEVICE_LABEL, info.supports_device_config);
  capabilities.set(Method::SET_THRESHOLD, info.supports_runtime_threshold);
  capabilities.set(Method::SET_MOTION_HITS, info.supports_runtime_motion_hits);
  capabilities.set(Method::SET_DETECTOR, info.supports_runtime_detector);
  capabilities.set(Method::RECALIBRATE, info.supports_manual_recalibration);
  capabilities.set(Method::SET_CSI_TRAFFIC_MODE, info.supports_traffic_control);
  capabilities.set(Method::SET_TRAFFIC_GENERATOR_MODE, info.supports_traffic_control);
  capabilities.set(Method::WIFI_ACCESS_POINTS, supports_wifi_bssid);
  capabilities.set(Method::SCAN_WIFI_ACCESS_POINTS, supports_wifi_bssid);
  capabilities.set(Method::SET_WIFI_BSSID, supports_wifi_bssid);
  capabilities.set(Method::CLEAR_WIFI_BSSID, supports_wifi_bssid);
  capabilities.set(Method::CLEAR_WIFI_CONFIG, supports_wifi_bssid);
  capabilities.set(Method::SET_MQTT_CONFIG, supports_mqtt_config);
  capabilities.set(Method::CLEAR_MQTT_CONFIG, supports_mqtt_config);
  capabilities.set(Method::DISCOVER_PEERS, supports_peer_discovery);
  capabilities.set(Method::START_RAW_STREAM, supports_raw_csi);
  capabilities.set(Method::STOP_RAW_STREAM, supports_raw_csi);
  // The compatibility overload historically reported the runtime section even
  // when callers hid the config command.
  capabilities.set(EspectreConfigSection::RUNTIME);
  capabilities.set(EspectreConfigSection::DEVICE, info.supports_device_config);
  capabilities.set(EspectreConfigSection::WIFI, supports_wifi_bssid);
  capabilities.set(EspectreConfigSection::MQTT, supports_mqtt_config);
  return espectre_capabilities_payload(config, info, capabilities);
}

std::string espectre_motion_payload(const EspectreDeviceConfig &config,
                                    const RuntimeSnapshot &snapshot,
                                    uint32_t timestamp_ms,
                                    uint32_t uptime_s,
                                    const char *frontend) {
  (void) config;
  (void) uptime_s;
  (void) frontend;
  char line[160];
  std::snprintf(line,
                sizeof(line),
                "{\"timestamp_ms\":%u,\"state\":\"%s\",\"score\":%.6g}",
                static_cast<unsigned>(timestamp_ms),
                motion_state_name(snapshot.motion_state),
                json_finite(snapshot.movement_metric));
  return line;
}

std::string espectre_diagnostics_payload(const EspectreDeviceConfig &config,
                                         const RuntimeSnapshot &snapshot,
                                         uint32_t timestamp_ms,
                                         uint32_t uptime_s,
                                         float free_memory_kb,
                                         float loop_time_ms,
                                         const RuntimeDiagnosticsSample *diagnostics) {
  (void) snapshot;
  (void) config;
  std::string out;
  out.reserve(diagnostics != nullptr ? 640U : 160U);
  out = "{";
  out += "\"timestamp_ms\":" + std::to_string(timestamp_ms);
  append_json_uint_field(&out, "uptime", uptime_s);
  append_json_float_field(&out, "free_memory_kb", free_memory_kb);
  append_json_float_field(&out, "loop_time_ms", loop_time_ms);
  if (diagnostics != nullptr) {
    append_json_float_field(&out, "traffic_tx_pps", diagnostics->traffic_tx_pps);
    append_json_float_field(&out, "csi_callback_pps", diagnostics->csi_callback_pps);
    append_json_float_field(&out, "csi_accepted_pps", diagnostics->csi_accepted_pps);
    append_json_float_field(&out, "csi_admitted_pps", diagnostics->csi_admitted_pps);
    append_json_float_field(&out, "csi_filtered_pps", diagnostics->csi_filtered_pps);
    append_json_float_field(
        &out, "csi_pending_frame_drop_pps", diagnostics->csi_pending_frame_drop_pps);
    append_json_float_field(&out, "csi_missing_slots_pps", diagnostics->csi_missing_slots_pps);
    append_json_float_field(&out, "csi_excess_pps", diagnostics->csi_excess_pps);
    append_json_float_field(&out, "csi_stale_pps", diagnostics->csi_stale_pps);
    append_json_float_field(&out, "csi_out_of_order_pps", diagnostics->csi_out_of_order_pps);
    append_json_float_field(&out, "csi_occupancy", diagnostics->csi_occupancy_ratio);
    append_json_uint_field(&out, "wifi_channel", diagnostics->wifi_channel);
    if (diagnostics->wifi_rssi_dbm == INT8_MIN) {
      append_json_null_field(&out, "wifi_rssi_dbm");
    } else {
      append_json_field_prefix(&out, "wifi_rssi_dbm");
      out.append(std::to_string(diagnostics->wifi_rssi_dbm));
    }
  }
  out += "}";
  return out;
}

std::string espectre_command_result_payload(const EspectreDeviceConfig &config,
                                            const EspectreCommand &command,
                                            bool accepted,
                                            const char *code,
                                            const char *message,
                                            const std::string &data_json) {
  (void) config;
  std::string out;
  out.reserve(128U + command.command_id.size() + command.command.size() +
              (message != nullptr ? std::strlen(message) : 0U) + data_json.size());
  out = "{";
  bool first = true;
  if (!command.command_id.empty()) {
    append_json_pair(&out, "command_id", command.command_id.c_str(), first);
    first = false;
  }
  if (!command.command_id.empty() && !command.command.empty()) {
    append_json_pair(&out, "command", command.command.c_str(), first);
    first = false;
  }
  if (!first) out += ',';
  out += "\"accepted\":";
  out += accepted ? "true" : "false";
  append_json_pair(&out, "code", code != nullptr ? code : (accepted ? "ok" : "internal_error"));
  append_json_pair(&out, "message", message != nullptr ? message : "");
  if (!data_json.empty()) {
    std::vector<JsonObjectField> fields;
    if (parse_json_object_fields(data_json, &fields, nullptr)) {
      out += ",\"data\":";
      out += data_json;
    }
  }
  out += "}";
  return out;
}

std::string espectre_command_request_payload(const std::string &command_id,
                                             const std::string &command,
                                             const std::string &params_json) {
  if (!command_id_accepted(command_id) || command.empty()) return {};
  std::vector<JsonObjectField> fields;
  if (!parse_json_object_fields(params_json, &fields, nullptr)) return {};
  std::string out{"{"};
  append_json_pair(&out, "command_id", command_id.c_str(), true);
  append_json_pair(&out, "command", command.c_str());
  for (const JsonObjectField &field : fields) {
    if (field.name == "command_id" || field.name == "command" || field.name == "protocol_version") return {};
    out += ',';
    append_json_string(&out, field.name.c_str());
    out += ':';
    if (field.type == JsonValueType::STRING) {
      append_json_string(&out, field.value.c_str());
    } else {
      out += field.value;
    }
  }
  out += "}";
  return out;
}

std::string espectre_fault_payload(const EspectreDeviceConfig &config,
                                   const char *message,
                                   uint32_t timestamp_ms) {
  std::string out{"{"};
  (void) config;
  out += "\"timestamp_ms\":" + std::to_string(static_cast<unsigned>(timestamp_ms));
  append_json_pair(&out, "message", message != nullptr ? message : "runtime fault");
  out += "}";
  return out;
}

std::string espectre_message_catalog_payload(const EspectreProtocolExtension *extension) {
  EspectreDeviceConfig config;
  EspectreCommand command;
  command.command_id = "contract-1";
  command.command = "update_sensing";
  RuntimeSnapshot snapshot;
  snapshot.motion_state = MotionState::MOTION;
  snapshot.movement_metric = 0.25f;
  snapshot.threshold = 0.5f;
  snapshot.detector_name = "lightweight";

  std::string out{"{"};
  append_json_pair(&out, "protocol_version", ESPECTRE_PROTOCOL_VERSION, true);
  out += ",\"dns_sd\":{";
  append_json_pair(&out, "txtvers", ESPECTRE_DNS_SD_TXT_SCHEMA_VERSION, true);
  append_json_pair(&out, "protovers", ESPECTRE_PROTOCOL_VERSION);
  out += "},\"messages\":{\"request\":";
  out += espectre_command_request_payload(command.command_id, command.command, "{\"threshold\":0.5}");
  out += ",\"result\":";
  out += espectre_command_result_payload(config, command, true, "ok", "threshold updated",
                                         "{\"threshold\":0.5}");
  out += ",\"error\":";
  out += espectre_command_result_payload(config, command, false, "invalid_params",
                                         "threshold is invalid");
  out += ",\"events\":{\"names\":[\"motion\",\"health\",\"device\",\"sensing\"";
  if (extension != nullptr && validate_protocol_extension(*extension)) {
    for (const auto &event : extension->events) {
      out += ',';
      append_json_string(&out, event.c_str());
    }
  }
  out += ",\"fault\"],\"health\":";
  out += espectre_health_payload(config, true, 1000U);
  out += ",\"motion\":";
  out += espectre_motion_payload(config, snapshot, 1000U, 1U, "micro");
  out += ",\"fault\":";
  out += espectre_fault_payload(config, "runtime fault", 1000U);
  out += "}}}";
  return out;
}


bool parse_espectre_command(const std::string &payload, EspectreCommand *command, std::string *error,
                            const EspectreProtocolExtension *extension) {
  if (command == nullptr) {
    if (error != nullptr) {
      *error = "command output is required";
    }
    return false;
  }
  if (payload.size() > ESPECTRE_COMMAND_MAX_PAYLOAD_SIZE) {
    if (error != nullptr) {
      *error = "command payload exceeds the size limit";
    }
    *command = EspectreCommand{};
    return false;
  }
  std::vector<JsonObjectField> fields;
  std::string json_error;
  if (!parse_json_object_fields(payload, &fields, &json_error)) {
    if (error != nullptr) {
      *error = json_error.empty() ? "invalid command JSON" : json_error;
    }
    *command = EspectreCommand{};
    return false;
  }
  std::string command_id;
  const JsonObjectField *id_field = find_json_object_field(fields, "command_id");
  if (id_field == nullptr || id_field->type != JsonValueType::STRING ||
      !command_id_accepted(id_field->value)) {
    if (error != nullptr) {
      *error = "invalid command_id (accepted: non-empty string up to 64 characters)";
    }
    *command = EspectreCommand{};
    return false;
  }
  command_id = id_field->value;
  const JsonObjectField *command_field = find_json_object_field(fields, "command");
  if (command_field == nullptr || command_field->type != JsonValueType::STRING) {
    if (error != nullptr) {
      *error = "missing command";
    }
    *command = EspectreCommand{};
    command->command_id = std::move(command_id);
    return false;
  }
  return parse_command_fields(command_id, command_field->value, fields, payload, extension, command, error);
}

bool parse_espectre_command_request(const std::string &command_id,
                                    const std::string &command_name,
                                    const std::string &params_json,
                                    EspectreCommand *command,
                                    std::string *error,
                                    const std::string &protocol_version,
                                    const EspectreProtocolExtension *extension) {
  if (!command_id.empty() && !command_id_accepted(command_id)) {
    if (error != nullptr) {
      *error = "invalid command_id (accepted: non-empty string up to 64 characters)";
    }
    if (command != nullptr) {
      *command = EspectreCommand{};
      command->command_id = command_id;
      command->command = command_name;
    }
    return false;
  }
  if (protocol_version != ESPECTRE_PROTOCOL_VERSION) {
    if (error != nullptr) *error = "unsupported protocol_version";
    if (command != nullptr) {
      *command = EspectreCommand{};
      command->command_id = command_id;
      command->command = command_name;
    }
    return false;
  }
  std::vector<JsonObjectField> fields;
  std::string json_error;
  if (!parse_json_object_fields(params_json, &fields, &json_error)) {
    if (error != nullptr) {
      *error = json_error.empty() ? "invalid command params" : json_error;
    }
    if (command != nullptr) {
      *command = EspectreCommand{};
      command->command_id = command_id;
      command->command = command_name;
    }
    return false;
  }
  return parse_command_fields(command_id, command_name, fields, params_json, extension, command, error);
}



bool parse_espectre_config_command(const std::string &command, EspectreDeviceConfig *config, std::string *error) {
  if (config == nullptr) {
    return false;
  }
  constexpr const char *prefix = "SET_DEVICE_CONFIG:";
  if (command.rfind(prefix, 0) != 0) {
    if (error != nullptr) {
      *error = "invalid prefix";
    }
    return false;
  }
  const std::string body = command.substr(std::string(prefix).size());
  const size_t equal = body.find('=');
  if (equal == std::string::npos) {
    if (error != nullptr) {
      *error = "expected key=value";
    }
    return false;
  }
  const std::string field = body.substr(0, equal);
  const std::string value = body.substr(equal + 1);
  if (!assign_config_field(field, value, config)) {
    if (error != nullptr) {
      *error = "invalid config field";
    }
    return false;
  }
  return true;
}

bool parse_espectre_mqtt_config_command(const std::string &command, EspectreDeviceConfig *config, std::string *error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "config output is required";
    }
    return false;
  }
  constexpr const char *prefix = "SET_MQTT_CONFIG:";
  if (command.rfind(prefix, 0) != 0) {
    if (error != nullptr) {
      *error = "invalid mqtt config command";
    }
    return false;
  }

  std::vector<std::pair<std::string, std::string>> pairs;
  if (!parse_urlencoded_key_value_pairs(command.substr(std::strlen(prefix)), &pairs, error)) {
    return false;
  }

  EspectreDeviceConfig parsed = *config;
  bool has_scheme = false;
  bool has_host = false;
  bool has_port = false;
  for (const auto &pair : pairs) {
    if (pair.first == "scheme") {
      parsed.mqtt_scheme = pair.second;
      has_scheme = true;
      continue;
    }
    if (pair.first == "host") {
      parsed.mqtt_host = pair.second;
      has_host = true;
      continue;
    }
    if (pair.first == "port") {
      uint16_t port = 0U;
      if (!parse_mqtt_port_value(pair.second, &port)) {
        if (error != nullptr) {
          *error = "mqtt port must be 1..65535";
        }
        return false;
      }
      parsed.mqtt_port = port;
      has_port = true;
      continue;
    }
    if (pair.first == "username") {
      parsed.mqtt_username = pair.second;
      continue;
    }
    if (pair.first == "password") {
      parsed.mqtt_password = pair.second;
      continue;
    }
    if (pair.first == "topic_prefix") {
      parsed.topic_prefix = pair.second.empty() ? ESPECTRE_TOPIC_PREFIX : pair.second;
      continue;
    }
    if (error != nullptr) {
      *error = "unsupported mqtt config field";
    }
    return false;
  }

  if (!has_scheme) {
    if (error != nullptr) {
      *error = "missing MQTT scheme";
    }
    return false;
  }
  if (!has_host) {
    if (error != nullptr) {
      *error = "missing MQTT host";
    }
    return false;
  }
  if (!has_port) {
    if (error != nullptr) {
      *error = "missing MQTT port";
    }
    return false;
  }
  if (!validate_espectre_mqtt_config(parsed, error)) {
    return false;
  }
  *config = std::move(parsed);
  return true;
}

}  // namespace espectre
