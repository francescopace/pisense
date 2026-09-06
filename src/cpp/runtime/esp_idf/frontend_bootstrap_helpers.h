/*
 * ESPectre - Frontend Bootstrap Helpers
 *
 * Loads persisted frontend config and initializes shared Wi-Fi station
 * setup.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstdint>

#include "device_identity.h"
#include "esp_err.h"
#include "espectre_protocol.h"
#include "standalone_wifi_service.h"
#include "wifi_provisioning_service.h"

namespace espectre {

struct FrontendDeviceConfigDefaults {
  const char *device_label{ESPECTRE_DEFAULT_DEVICE_LABEL};
  const char *mqtt_scheme{""};
  const char *mqtt_host{""};
  uint16_t mqtt_port{0U};
  const char *mqtt_username{""};
  const char *mqtt_password{""};
  const char *topic_prefix{ESPECTRE_TOPIC_PREFIX};
  uint64_t runtime_device_id{0U};
};

struct FrontendWifiStationOptions {
  const char *ssid{nullptr};
  const char *password{nullptr};
  const char *bssid{nullptr};
  int configured_channel{0};
  int max_retry{8};
  bool manage_csi_lifecycle{false};
  bool start_manager{false};
  WifiProvisioningService::ChangeCallback change_callback{};
  standalone_wifi_callback_t connected_callback{};
  standalone_wifi_callback_t disconnected_callback{};
  WifiBandPolicy band_policy{WifiBandPolicy::BAND_2G};
};

EspectreDeviceConfig load_frontend_device_config(const FrontendDeviceConfigDefaults &defaults,
                                                 const char *log_tag,
                                                 const char *stored_config_message,
                                                 const char *load_error_prefix);

esp_err_t setup_frontend_wifi_station(WifiProvisioningService *provisioning,
                                      StandaloneWifiService *wifi_manager,
                                      const FrontendWifiStationOptions &options,
                                      const char *log_tag,
                                      const char *stored_config_message);

}  // namespace espectre
