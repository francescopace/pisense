/*
 * ESPectre - Wi-Fi Provisioning Service
 *
 * Stores Wi-Fi credentials and applies live station provisioning changes.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "device_config_store.h"
#include "esp_err.h"
#include "standalone_wifi_service.h"

namespace espectre {

struct WifiProvisioningDefaults {
  const char *ssid{nullptr};
  const char *password{nullptr};
  const char *bssid{nullptr};
  uint8_t channel{0U};
  int max_retry{8};
  bool manage_csi_lifecycle{false};
  WifiBandPolicy band_policy{WifiBandPolicy::BAND_2G};
  /** Per-association verification window for a staged candidate. */
  uint32_t candidate_timeout_ms{30000U};
};

enum class WifiProvisioningApplyState : uint8_t {
  IDLE = 0,
  VERIFYING,
  ROLLING_BACK,
  APPLIED,
  ROLLED_BACK,
  RECOVERY_REQUIRED,
};

const char *wifi_provisioning_apply_state_name(WifiProvisioningApplyState state);

class WifiProvisioningService {
 public:
  using ChangeCallback = std::function<void()>;

  explicit WifiProvisioningService(StandaloneWifiService *wifi_manager);

  void set_change_callback(ChangeCallback callback);
  void set_reconfigure_callbacks(ChangeCallback prepare_callback,
                                 ChangeCallback resume_callback);
  void set_scan_callbacks(ChangeCallback prepare_callback,
                          ChangeCallback resume_callback);
  void set_apply_completed_callback(ChangeCallback callback);
  esp_err_t load_or_set_defaults(const WifiProvisioningDefaults &defaults);
  esp_err_t setup_station(const WifiProvisioningDefaults &defaults,
                          standalone_wifi_callback_t connected_cb = {},
                          standalone_wifi_callback_t disconnected_cb = {});
  bool handle_command(const std::string &command, std::string *message);
  /** Start an asynchronous scan; results are limited to the provisioned SSID. */
  bool request_access_point_scan(std::string *message);
  /** Stage credentials received through the standard Improv Serial RPC. */
  bool begin_serial_provisioning(const std::string &ssid,
                                 const std::string &password,
                                 std::string *message);
  /** Advance candidate verification and bounded rollback after Wi-Fi events. */
  void loop();
  bool apply_live(std::string *message);

  const StoredWifiConfig &config() const { return wifi_config_; }
  bool password_set() const { return !wifi_config_.password.empty(); }
  esp_err_t last_load_result() const { return last_load_result_; }
  WifiProvisioningApplyState apply_state() const { return apply_state_; }
  const std::string &apply_message() const { return apply_message_; }
  bool apply_pending() const;
  bool scan_pending() const { return scan_active_; }
  const std::vector<StandaloneWifiAccessPoint> &access_points() const { return access_points_; }
  const std::string &scan_message() const { return scan_message_; }

 private:
  bool apply_config_live_(const StoredWifiConfig &config, std::string *message);
  bool begin_candidate_apply_(StoredWifiConfig candidate, std::string *message);
  void handle_connected_();
  void begin_rollback_(const char *reason);
  void set_apply_state_(WifiProvisioningApplyState state, const char *message);
  void resume_reconfigure_();
  void refresh_cached_strings_();
  void notify_changed_();

  StandaloneWifiService *wifi_manager_;
  ChangeCallback change_callback_;
  ChangeCallback prepare_reconfigure_callback_;
  ChangeCallback resume_reconfigure_callback_;
  ChangeCallback prepare_scan_callback_;
  ChangeCallback resume_scan_callback_;
  ChangeCallback apply_completed_callback_;
  StoredWifiConfig wifi_config_;
  StoredWifiConfig last_good_config_;
  StoredWifiConfig candidate_config_;
  WifiProvisioningDefaults defaults_;
  esp_err_t last_load_result_{ESP_OK};
  std::string wifi_ssid_;
  std::string wifi_password_;
  std::string wifi_bssid_;
  std::vector<StandaloneWifiAccessPoint> access_points_;
  std::string scan_message_;
  WifiProvisioningApplyState apply_state_{WifiProvisioningApplyState::IDLE};
  std::string apply_message_;
  bool candidate_apply_pending_{false};
  bool reconfigure_active_{false};
  bool scan_active_{false};
  uint32_t apply_started_ms_{0U};
};

}  // namespace espectre
