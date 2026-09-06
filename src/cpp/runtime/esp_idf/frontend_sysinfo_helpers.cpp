/*
 * ESPectre - Frontend Sysinfo Helpers
 *
 * Builds shared sysinfo lines for frontend status surfaces.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "frontend_sysinfo_helpers.h"

#include <cstdio>

#include "runtime_config_utils.h"

namespace espectre {

namespace {

void append_line(std::vector<std::string> *lines, const char *line) {
  if (lines == nullptr || line == nullptr) {
    return;
  }
  lines->emplace_back(line);
}

void append_kv_line(std::vector<std::string> *lines, const char *key, const char *value) {
  if (lines == nullptr || key == nullptr || value == nullptr) {
    return;
  }
  char line[192];
  std::snprintf(line, sizeof(line), "%s=%s", key, value);
  lines->emplace_back(line);
}

void append_u32_line(std::vector<std::string> *lines, const char *key, unsigned value) {
  if (lines == nullptr || key == nullptr) {
    return;
  }
  char line[192];
  std::snprintf(line, sizeof(line), "%s=%u", key, value);
  lines->emplace_back(line);
}

void append_bool_line(std::vector<std::string> *lines, const char *key, bool value) {
  append_kv_line(lines, key, value ? "true" : "false");
}

}  // namespace

void append_sysinfo_protocol_lines(std::vector<std::string> *lines,
                                   const char *frontend,
                                   const SysinfoCapabilities &capabilities,
                                   bool include_proto_version,
                                   const char *protocol_version) {
  if (include_proto_version) {
    append_line(lines, "proto_version=1");
  }
  append_kv_line(lines, "frontend", frontend != nullptr ? frontend : "unknown");
  append_kv_line(lines, "espectre_protocol_version", protocol_version != nullptr ? protocol_version : ESPECTRE_PROTOCOL_VERSION);
  append_bool_line(lines, "supports_wifi_provisioning", capabilities.supports_wifi_provisioning);
  append_bool_line(lines, "supports_mqtt_config", capabilities.supports_mqtt_config);
  append_bool_line(lines, "supports_device_config", capabilities.supports_device_config);
  append_bool_line(lines, "supports_runtime_threshold", capabilities.supports_runtime_threshold);
  append_bool_line(lines, "supports_runtime_motion_hits", capabilities.supports_runtime_motion_hits);
  append_bool_line(lines, "supports_runtime_detector", capabilities.supports_runtime_detector);
  append_bool_line(lines, "supports_manual_recalibration", capabilities.supports_manual_recalibration);
  append_bool_line(lines, "supports_traffic_control", capabilities.supports_traffic_control);
  append_bool_line(lines, "supports_live_telemetry", capabilities.supports_live_telemetry);
  append_bool_line(lines, "supports_extended_diagnostics", capabilities.supports_extended_diagnostics);
  append_bool_line(lines, "supports_wifi_5ghz", capabilities.supports_wifi_5ghz);
}

void append_sysinfo_identity_lines(std::vector<std::string> *lines,
                                   const EspectreDeviceConfig &device_config,
                                   const EspectreDeviceInfo &device_info,
                                   bool include_firmware_version) {
  if (include_firmware_version) {
    append_kv_line(lines, "firmware_version", device_info.firmware_version.c_str());
  }
  append_kv_line(lines, "chip", device_info.chip.c_str());
  append_kv_line(lines, "device_id", espectre_effective_device_id(device_config).c_str());
  append_kv_line(lines, "device_label", espectre_effective_device_label(device_config).c_str());
  const std::string device_name = espectre_device_name(espectre_effective_device_id_u64(device_config),
                                                       device_info.chip.empty() ? nullptr : device_info.chip.c_str());
  append_kv_line(lines, "device_name", device_name.c_str());
}

void append_sysinfo_mqtt_lines(std::vector<std::string> *lines,
                               const EspectreDeviceConfig &device_config,
                               bool mqtt_connected) {
  append_bool_line(lines, "mqtt_connected", mqtt_connected);
  append_kv_line(lines, "mqtt_scheme", device_config.mqtt_scheme.c_str());
  append_kv_line(lines, "mqtt_host", device_config.mqtt_host.c_str());
  append_u32_line(lines, "mqtt_port", static_cast<unsigned>(device_config.mqtt_port));
  append_kv_line(lines, "mqtt_username", device_config.mqtt_username.c_str());
  append_kv_line(lines, "topic_prefix", device_config.topic_prefix.c_str());
}

void append_sysinfo_wifi_lines(std::vector<std::string> *lines, const SysinfoWifiState &wifi) {
  append_bool_line(lines, "wifi_connected", wifi.connected);
  append_kv_line(lines, "wifi_ssid", wifi.ssid.c_str());
  append_kv_line(lines, "wifi_bssid", wifi.bssid.c_str());
  append_u32_line(lines, "wifi_channel", static_cast<unsigned>(wifi.channel));
  append_kv_line(lines, "wifi_band_policy", wifi_band_policy_name(wifi.band_policy));
}

void append_sysinfo_network_lines(std::vector<std::string> *lines,
                                  const char *ip_address,
                                  const char *mac_address) {
  if (ip_address != nullptr && ip_address[0] != '\0') {
    append_kv_line(lines, "ip_address", ip_address);
  }
  if (mac_address != nullptr && mac_address[0] != '\0') {
    append_kv_line(lines, "mac_address", mac_address);
  }
}

std::vector<std::string> build_frontend_sysinfo_lines(const FrontendSysinfoBase &base) {
  std::vector<std::string> lines;
  lines.reserve(24U);
  append_sysinfo_protocol_lines(
      &lines, base.frontend, base.capabilities, base.include_proto_version, ESPECTRE_PROTOCOL_VERSION);
  append_sysinfo_identity_lines(&lines, base.device_config, base.device_info, base.include_firmware_version);
  append_sysinfo_wifi_lines(&lines, base.wifi);
  append_sysinfo_mqtt_lines(&lines, base.device_config, base.mqtt_connected);
  return lines;
}

void append_sysinfo_end_line(std::vector<std::string> *lines) { append_line(lines, "END"); }

}  // namespace espectre
