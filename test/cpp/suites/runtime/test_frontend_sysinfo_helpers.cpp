/*
 * ESPectre - Frontend Sysinfo Helper Tests
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include "frontend_sysinfo_helpers.h"

#include <algorithm>
#include <string>
#include <vector>

using namespace espectre;

namespace {

bool contains_line(const std::vector<std::string> &lines, const char *expected) {
  return std::find(lines.begin(), lines.end(), expected) != lines.end();
}

void test_frontend_sysinfo_builder_emits_shared_runtime_state(void) {
  FrontendSysinfoBase base;
  base.frontend = "native";
  base.include_proto_version = true;
  base.include_firmware_version = true;
  base.mqtt_connected = true;
  base.capabilities.supports_runtime_threshold = true;
  base.capabilities.supports_runtime_motion_hits = true;
  base.capabilities.supports_runtime_detector = true;
  base.capabilities.supports_manual_recalibration = true;
  base.capabilities.supports_traffic_control = true;
  base.capabilities.supports_live_telemetry = true;
  base.capabilities.supports_extended_diagnostics = true;
  base.capabilities.supports_wifi_5ghz = true;
  base.device_config.device_id = 0x112233445566ULL;
  base.device_config.device_label = "Kitchen";
  base.device_config.mqtt_scheme = "mqtts";
  base.device_config.mqtt_host = "broker.example";
  base.device_config.mqtt_port = 8883U;
  base.device_config.mqtt_username = "sensor";
  base.device_config.topic_prefix = "building/west";
  base.device_info.chip = "ESP32-C5";
  base.device_info.firmware_version = "3.0.0";
  base.wifi.connected = true;
  base.wifi.ssid = "lab";
  base.wifi.bssid = "00:11:22:33:44:55";
  base.wifi.channel = 36U;
  base.wifi.band_policy = WifiBandPolicy::BAND_5G;

  std::vector<std::string> lines = build_frontend_sysinfo_lines(base);
  TEST_ASSERT_TRUE(contains_line(lines, "proto_version=1"));
  TEST_ASSERT_TRUE(contains_line(lines, "frontend=native"));
  TEST_ASSERT_TRUE(contains_line(lines, "supports_runtime_threshold=true"));
  TEST_ASSERT_TRUE(contains_line(lines, "supports_wifi_provisioning=true"));
  TEST_ASSERT_TRUE(contains_line(lines, "supports_wifi_5ghz=true"));
  TEST_ASSERT_TRUE(contains_line(lines, "firmware_version=3.0.0"));
  TEST_ASSERT_TRUE(contains_line(lines, "chip=ESP32-C5"));
  TEST_ASSERT_TRUE(contains_line(lines, "device_label=Kitchen"));
  TEST_ASSERT_TRUE(contains_line(lines, "mqtt_connected=true"));
  TEST_ASSERT_TRUE(contains_line(lines, "mqtt_scheme=mqtts"));
  TEST_ASSERT_TRUE(contains_line(lines, "mqtt_host=broker.example"));
  TEST_ASSERT_TRUE(contains_line(lines, "mqtt_port=8883"));
  TEST_ASSERT_TRUE(contains_line(lines, "mqtt_username=sensor"));
  TEST_ASSERT_TRUE(contains_line(lines, "topic_prefix=building/west"));
  TEST_ASSERT_TRUE(contains_line(lines, "wifi_connected=true"));
  TEST_ASSERT_TRUE(contains_line(lines, "wifi_ssid=lab"));
  TEST_ASSERT_TRUE(contains_line(lines, "wifi_bssid=00:11:22:33:44:55"));
  TEST_ASSERT_TRUE(contains_line(lines, "wifi_channel=36"));
  TEST_ASSERT_TRUE(contains_line(lines, "wifi_band_policy=5g"));
}

void test_frontend_sysinfo_helpers_handle_optional_and_null_outputs(void) {
  SysinfoCapabilities capabilities;
  capabilities.supports_wifi_provisioning = false;
  capabilities.supports_mqtt_config = false;
  capabilities.supports_device_config = false;
  std::vector<std::string> lines;

  append_sysinfo_protocol_lines(&lines, nullptr, capabilities, false, nullptr);
  TEST_ASSERT_TRUE(contains_line(lines, "frontend=unknown"));
  TEST_ASSERT_TRUE(contains_line(lines, "supports_wifi_provisioning=false"));
  TEST_ASSERT_FALSE(contains_line(lines, "proto_version=1"));

  append_sysinfo_network_lines(&lines, "", nullptr);
  TEST_ASSERT_FALSE(contains_line(lines, "ip_address="));
  append_sysinfo_network_lines(&lines, "192.0.2.5", "AA:BB:CC:DD:EE:FF");
  TEST_ASSERT_TRUE(contains_line(lines, "ip_address=192.0.2.5"));
  TEST_ASSERT_TRUE(contains_line(lines, "mac_address=AA:BB:CC:DD:EE:FF"));
  append_sysinfo_end_line(&lines);
  TEST_ASSERT_EQUAL_STRING("END", lines.back().c_str());

  append_sysinfo_protocol_lines(nullptr, nullptr, capabilities, true, nullptr);
  append_sysinfo_identity_lines(nullptr, EspectreDeviceConfig{}, EspectreDeviceInfo{}, true);
  append_sysinfo_mqtt_lines(nullptr, EspectreDeviceConfig{}, false);
  append_sysinfo_wifi_lines(nullptr, SysinfoWifiState{});
  append_sysinfo_network_lines(nullptr, "192.0.2.5", "AA:BB:CC:DD:EE:FF");
  append_sysinfo_end_line(nullptr);
}

}  // namespace

int process(void) {
  UNITY_BEGIN();
  RUN_TEST(test_frontend_sysinfo_builder_emits_shared_runtime_state);
  RUN_TEST(test_frontend_sysinfo_helpers_handle_optional_and_null_outputs);
  return UNITY_END();
}

#if defined(ESP_PLATFORM)
extern "C" void app_main(void) { process(); }
#else
int main(int argc, char **argv) {
  (void) argc;
  (void) argv;
  return process();
}
#endif
