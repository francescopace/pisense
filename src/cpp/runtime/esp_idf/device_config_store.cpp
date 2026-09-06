/*
 * ESPectre - Device Config Store
 *
 * Persists Wi-Fi and device configuration in ESP-IDF non-volatile storage.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "device_config_store.h"

#include <utility>

#include "nvs.h"

namespace espectre {

namespace {

constexpr const char *kNamespace = "espectre";
constexpr const char *kWifiSsidKey = "wifi_ssid";
constexpr const char *kWifiPasswordKey = "wifi_pass";
constexpr const char *kWifiBssidKey = "wifi_bssid";
constexpr const char *kWifiChannelKey = "wifi_chan";
constexpr const char *kWifiBandPolicyKey = "wifi_band";
constexpr const char *kWifiPendingKey = "wifi_pending";
constexpr const char *kPendingWifiSsidKey = "wifi_p_ssid";
constexpr const char *kPendingWifiPasswordKey = "wifi_p_pass";
constexpr const char *kPendingWifiBssidKey = "wifi_p_bssid";
constexpr const char *kPendingWifiChannelKey = "wifi_p_chan";
constexpr const char *kPendingWifiBandPolicyKey = "wifi_p_band";
constexpr const char *kDeviceLabelKey = "device_label";
constexpr const char *kMqttSchemeKey = "mqtt_scheme";
constexpr const char *kMqttHostKey = "mqtt_host";
constexpr const char *kMqttPortKey = "mqtt_port";
constexpr const char *kMqttUserKey = "mqtt_user";
constexpr const char *kMqttPasswordKey = "mqtt_pass";
constexpr const char *kTopicPrefixKey = "topic_prefix";

esp_err_t read_string(nvs_handle_t handle, const char *key, std::string *value) {
  size_t length = 0;
  esp_err_t err = nvs_get_str(handle, key, nullptr, &length);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    value->clear();
    return ESP_OK;
  }
  if (err != ESP_OK) {
    return err;
  }

  std::string buffer(length, '\0');
  err = nvs_get_str(handle, key, buffer.data(), &length);
  if (err != ESP_OK) {
    return err;
  }
  if (!buffer.empty() && buffer.back() == '\0') {
    buffer.pop_back();
  }
  *value = buffer;
  return ESP_OK;
}

esp_err_t write_string(nvs_handle_t handle, const char *key, const std::string &value) {
  if (value.empty()) {
    const esp_err_t err = nvs_erase_key(handle, key);
    return err == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : err;
  }
  return nvs_set_str(handle, key, value.c_str());
}

esp_err_t erase_key(nvs_handle_t handle, const char *key) {
  const esp_err_t err = nvs_erase_key(handle, key);
  return err == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : err;
}

esp_err_t load_wifi_config(nvs_handle_t handle,
                           const char *ssid_key,
                           const char *password_key,
                           const char *bssid_key,
                           const char *channel_key,
                           const char *band_policy_key,
                           StoredWifiConfig *config) {
  StoredWifiConfig loaded;
  esp_err_t err = read_string(handle, ssid_key, &loaded.ssid);
  if (err == ESP_OK) {
    err = read_string(handle, password_key, &loaded.password);
  }
  if (err == ESP_OK) {
    err = read_string(handle, bssid_key, &loaded.bssid);
  }
  uint8_t channel = 0U;
  const esp_err_t channel_err = nvs_get_u8(handle, channel_key, &channel);
  if (err == ESP_OK && channel_err != ESP_OK && channel_err != ESP_ERR_NVS_NOT_FOUND) {
    err = channel_err;
  }
  uint8_t band_policy = 0U;
  const esp_err_t band_err = nvs_get_u8(handle, band_policy_key, &band_policy);
  if (err == ESP_OK && band_err != ESP_OK && band_err != ESP_ERR_NVS_NOT_FOUND) {
    err = band_err;
  }
  if (err == ESP_OK && band_err == ESP_OK &&
      band_policy > static_cast<uint8_t>(WifiBandPolicy::AUTO)) {
    err = ESP_ERR_INVALID_STATE;
  }
  if (err != ESP_OK) {
    return err;
  }
  loaded.channel = channel;
  if (band_err == ESP_OK) {
    loaded.band_policy = static_cast<WifiBandPolicy>(band_policy);
    loaded.has_saved_band_policy = true;
  }
  loaded.has_saved_config = !loaded.ssid.empty();
  *config = std::move(loaded);
  return ESP_OK;
}

esp_err_t save_wifi_config(nvs_handle_t handle,
                           const char *ssid_key,
                           const char *password_key,
                           const char *bssid_key,
                           const char *channel_key,
                           const char *band_policy_key,
                           const StoredWifiConfig &config) {
  esp_err_t err = write_string(handle, ssid_key, config.ssid);
  if (err == ESP_OK) {
    err = write_string(handle, password_key, config.password);
  }
  if (err == ESP_OK) {
    err = write_string(handle, bssid_key, config.bssid);
  }
  if (err == ESP_OK) {
    err = nvs_set_u8(handle, channel_key, config.channel);
  }
  if (err == ESP_OK) {
    err = nvs_set_u8(handle, band_policy_key, static_cast<uint8_t>(config.band_policy));
  }
  return err;
}

esp_err_t clear_wifi_config(nvs_handle_t handle,
                            const char *ssid_key,
                            const char *password_key,
                            const char *bssid_key,
                            const char *channel_key,
                            const char *band_policy_key) {
  esp_err_t result = erase_key(handle, ssid_key);
  const char *keys[] = {password_key, bssid_key, channel_key, band_policy_key};
  for (const char *key : keys) {
    const esp_err_t err = erase_key(handle, key);
    if (result == ESP_OK) {
      result = err;
    }
  }
  return result;
}

}  // namespace

esp_err_t load_stored_wifi_config(StoredWifiConfig *config) {
  if (config == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READONLY, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    *config = StoredWifiConfig{};
    return ESP_OK;
  }
  if (err != ESP_OK) {
    return err;
  }

  StoredWifiConfig loaded;
  err = load_wifi_config(handle, kWifiSsidKey, kWifiPasswordKey, kWifiBssidKey,
                         kWifiChannelKey, kWifiBandPolicyKey, &loaded);
  nvs_close(handle);

  if (err != ESP_OK) {
    return err;
  }

  *config = loaded;
  return ESP_OK;
}

esp_err_t save_stored_wifi_config(const StoredWifiConfig &config) {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    return err;
  }

  err = save_wifi_config(handle, kWifiSsidKey, kWifiPasswordKey, kWifiBssidKey,
                         kWifiChannelKey, kWifiBandPolicyKey, config);
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }
  nvs_close(handle);
  return err;
}

esp_err_t clear_stored_wifi_config() {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    return ESP_OK;
  }
  if (err != ESP_OK) {
    return err;
  }

  esp_err_t result = clear_wifi_config(handle, kWifiSsidKey, kWifiPasswordKey,
                                       kWifiBssidKey, kWifiChannelKey, kWifiBandPolicyKey);
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

esp_err_t load_pending_wifi_config(StoredWifiConfig *config, bool *has_pending) {
  if (config == nullptr || has_pending == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  *config = StoredWifiConfig{};
  *has_pending = false;

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READONLY, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    return ESP_OK;
  }
  if (err != ESP_OK) {
    return err;
  }

  uint8_t pending = 0U;
  err = nvs_get_u8(handle, kWifiPendingKey, &pending);
  if (err == ESP_ERR_NVS_NOT_FOUND || pending == 0U) {
    nvs_close(handle);
    return ESP_OK;
  }
  if (err == ESP_OK) {
    err = load_wifi_config(handle, kPendingWifiSsidKey, kPendingWifiPasswordKey,
                           kPendingWifiBssidKey, kPendingWifiChannelKey,
                           kPendingWifiBandPolicyKey, config);
  }
  nvs_close(handle);
  if (err == ESP_OK) {
    *has_pending = true;
  }
  return err;
}

esp_err_t save_pending_wifi_config(const StoredWifiConfig &config) {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    return err;
  }
  err = save_wifi_config(handle, kPendingWifiSsidKey, kPendingWifiPasswordKey,
                         kPendingWifiBssidKey, kPendingWifiChannelKey,
                         kPendingWifiBandPolicyKey, config);
  if (err == ESP_OK) {
    err = nvs_set_u8(handle, kWifiPendingKey, 1U);
  }
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }
  nvs_close(handle);
  return err;
}

esp_err_t clear_pending_wifi_config() {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    return ESP_OK;
  }
  if (err != ESP_OK) {
    return err;
  }
  esp_err_t result = clear_wifi_config(handle, kPendingWifiSsidKey,
                                       kPendingWifiPasswordKey, kPendingWifiBssidKey,
                                       kPendingWifiChannelKey, kPendingWifiBandPolicyKey);
  const esp_err_t pending_err = erase_key(handle, kWifiPendingKey);
  if (result == ESP_OK) {
    result = pending_err;
  }
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

esp_err_t load_stored_device_config(EspectreDeviceConfig *config, bool *has_saved_config) {
  if (config == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READONLY, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    if (has_saved_config != nullptr) {
      *has_saved_config = false;
    }
    return ESP_OK;
  }
  if (err != ESP_OK) {
    return err;
  }

  EspectreDeviceConfig loaded;
  err = read_string(handle, kDeviceLabelKey, &loaded.device_label);
  if (err == ESP_OK) {
    err = read_string(handle, kMqttSchemeKey, &loaded.mqtt_scheme);
  }
  if (err == ESP_OK) {
    err = read_string(handle, kMqttHostKey, &loaded.mqtt_host);
  }
  if (err == ESP_OK) {
    err = read_string(handle, kMqttUserKey, &loaded.mqtt_username);
  }
  if (err == ESP_OK) {
    err = read_string(handle, kMqttPasswordKey, &loaded.mqtt_password);
  }
  if (err == ESP_OK) {
    err = read_string(handle, kTopicPrefixKey, &loaded.topic_prefix);
  }

  uint16_t port = 0;
  const esp_err_t port_err = nvs_get_u16(handle, kMqttPortKey, &port);
  if (err == ESP_OK && port_err != ESP_OK && port_err != ESP_ERR_NVS_NOT_FOUND) {
    err = port_err;
  }

  nvs_close(handle);

  if (err != ESP_OK) {
    return err;
  }

  const bool has_config = !loaded.device_label.empty() || !loaded.mqtt_scheme.empty() || !loaded.mqtt_host.empty() ||
                          !loaded.mqtt_username.empty() || !loaded.mqtt_password.empty() ||
                          !loaded.topic_prefix.empty() || port_err == ESP_OK;
  if (!has_config) {
    if (has_saved_config != nullptr) {
      *has_saved_config = false;
    }
    return ESP_OK;
  }

  if (loaded.topic_prefix.empty()) {
    loaded.topic_prefix = ESPECTRE_TOPIC_PREFIX;
  }
  if (port_err == ESP_OK && port != 0) {
    loaded.mqtt_port = port;
  }
  *config = loaded;
  if (has_saved_config != nullptr) {
    *has_saved_config = true;
  }
  return ESP_OK;
}

esp_err_t save_stored_device_config(const EspectreDeviceConfig &config) {
  const bool has_mqtt_settings = !config.mqtt_scheme.empty() || !config.mqtt_host.empty() ||
                                 config.mqtt_port != 0U || !config.mqtt_username.empty() ||
                                 !config.mqtt_password.empty();
  if (has_mqtt_settings && !validate_espectre_mqtt_config(config)) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    return err;
  }

  err = write_string(handle, kDeviceLabelKey, config.device_label);
  if (err == ESP_OK) {
    err = write_string(handle, kMqttSchemeKey, config.mqtt_scheme);
  }
  if (err == ESP_OK) {
    err = write_string(handle, kMqttHostKey, config.mqtt_host);
  }
  if (err == ESP_OK) {
    err = write_string(handle, kMqttUserKey, config.mqtt_username);
  }
  if (err == ESP_OK) {
    err = write_string(handle, kMqttPasswordKey, config.mqtt_password);
  }
  if (err == ESP_OK) {
    err = write_string(handle, kTopicPrefixKey, config.topic_prefix);
  }
  if (err == ESP_OK) {
    err = nvs_set_u16(handle, kMqttPortKey, config.mqtt_port);
  }
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }
  nvs_close(handle);
  return err;
}

esp_err_t clear_stored_device_config() {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    return ESP_OK;
  }
  if (err != ESP_OK) {
    return err;
  }

  const char *keys[] = {kDeviceLabelKey,
                        kMqttSchemeKey,
                        kMqttHostKey,
                        kMqttPortKey,
                        kMqttUserKey,
                        kMqttPasswordKey,
                        kTopicPrefixKey};
  esp_err_t result = ESP_OK;
  for (const char *key : keys) {
    const esp_err_t erase_err = nvs_erase_key(handle, key);
    if (result == ESP_OK && erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) {
      result = erase_err;
    }
  }
  if (result == ESP_OK) {
    result = nvs_commit(handle);
  }
  nvs_close(handle);
  return result;
}

}  // namespace espectre
