/*
 * ESPectre - Device Config Store
 *
 * Persists Wi-Fi and device configuration in ESP-IDF non-volatile storage.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstdint>
#include <string>

#include "esp_err.h"
#include "espectre_protocol.h"
#include "runtime_interface.h"

namespace espectre {

struct StoredWifiConfig {
  std::string ssid;
  std::string password;
  std::string bssid;
  uint8_t channel{0U};
  WifiBandPolicy band_policy{WifiBandPolicy::BAND_2G};
  bool has_saved_band_policy{false};
  bool has_saved_config{false};
};

esp_err_t load_stored_wifi_config(StoredWifiConfig *config);
esp_err_t save_stored_wifi_config(const StoredWifiConfig &config);
esp_err_t clear_stored_wifi_config();
esp_err_t load_pending_wifi_config(StoredWifiConfig *config, bool *has_pending);
esp_err_t save_pending_wifi_config(const StoredWifiConfig &config);
esp_err_t clear_pending_wifi_config();

esp_err_t load_stored_device_config(EspectreDeviceConfig *config, bool *has_saved_config);
esp_err_t save_stored_device_config(const EspectreDeviceConfig &config);
esp_err_t clear_stored_device_config();

}  // namespace espectre
