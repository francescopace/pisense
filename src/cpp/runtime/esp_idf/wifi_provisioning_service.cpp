/*
 * ESPectre - Wi-Fi Provisioning Service
 *
 * Stores Wi-Fi credentials and applies live station provisioning changes.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "wifi_provisioning_service.h"

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

#include "espectre_log.h"
#include "direct_wifi_snapshot_esp_idf.h"
#include "protocol_json.h"
#include "runtime_config_utils.h"
#include "runtime_time.h"
#include "wifi_band_helpers.h"

namespace espectre {

namespace {

static const char *const TAG = "espectre.wifi_prov";

bool assign_wifi_config_field(const std::string &field,
                              const std::string &value,
                              StoredWifiConfig *config,
                              std::string *error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "wifi config output is required";
    }
    return false;
  }
  if (field == "ssid") {
    if (value.empty() || value.size() > 32) {
      if (error != nullptr) {
        *error = "SSID must be 1..32 bytes";
      }
      return false;
    }
    config->ssid = value;
    return true;
  }
  if (field == "password") {
    if (value.size() > 63) {
      if (error != nullptr) {
        *error = "password must be 0..63 bytes";
      }
      return false;
    }
    config->password = value;
    return true;
  }
  if (field == "bssid") {
    if (!value.empty() && value.size() != 17U) {
      if (error != nullptr) {
        *error = "BSSID must be empty or 17 chars";
      }
      return false;
    }
    std::string normalized = value;
    for (size_t index = 0U; index < normalized.size(); ++index) {
      if (index % 3U == 2U) {
        if (normalized[index] != ':') {
          if (error != nullptr) *error = "BSSID must contain six hexadecimal octets";
          return false;
        }
      } else if (!std::isxdigit(static_cast<unsigned char>(normalized[index]))) {
        if (error != nullptr) *error = "BSSID must contain six hexadecimal octets";
        return false;
      } else {
        normalized[index] = static_cast<char>(std::toupper(static_cast<unsigned char>(normalized[index])));
      }
    }
    config->bssid = std::move(normalized);
    return true;
  }
  if (field == "channel") {
    char *end_ptr = nullptr;
    const long parsed = std::strtol(value.c_str(), &end_ptr, 10);
    // The range check bounds the narrowing cast; the channel plan itself is
    // what actually decides which of those values is a channel.
    if (end_ptr == value.c_str() || end_ptr == nullptr || *end_ptr != '\0' ||
        parsed < 0 || parsed > 255 ||
        !wifi_channel_is_supported(static_cast<int>(parsed))) {
      if (error != nullptr) {
        *error = std::string("channel must be ") + wifi_channel_supported_description();
      }
      return false;
    }
    config->channel = static_cast<uint8_t>(parsed);
    return true;
  }
  if (field == "band_policy") {
    if (value != "2g" && value != "5g" && value != "auto") {
      if (error != nullptr) {
        *error = "band_policy must be 2g, 5g, or auto";
      }
      return false;
    }
    const WifiBandPolicy policy = parse_wifi_band_policy(value.c_str());
    if (!wifi_band_policy_is_supported(policy)) {
      if (error != nullptr) {
        *error = "5 GHz Wi-Fi is not supported by this device";
      }
      return false;
    }
    config->band_policy = policy;
    config->has_saved_band_policy = true;
    return true;
  }
  if (error != nullptr) {
    *error = "unsupported wifi config field";
  }
  return false;
}

}  // namespace

const char *wifi_provisioning_apply_state_name(WifiProvisioningApplyState state) {
  switch (state) {
    case WifiProvisioningApplyState::VERIFYING:
      return "verifying";
    case WifiProvisioningApplyState::ROLLING_BACK:
      return "rolling_back";
    case WifiProvisioningApplyState::APPLIED:
      return "applied";
    case WifiProvisioningApplyState::ROLLED_BACK:
      return "rolled_back";
    case WifiProvisioningApplyState::RECOVERY_REQUIRED:
      return "recovery_required";
    case WifiProvisioningApplyState::IDLE:
    default:
      return "idle";
  }
}

WifiProvisioningService::WifiProvisioningService(StandaloneWifiService *wifi_manager) : wifi_manager_(wifi_manager) {}

void WifiProvisioningService::set_change_callback(ChangeCallback callback) { change_callback_ = std::move(callback); }

void WifiProvisioningService::set_reconfigure_callbacks(ChangeCallback prepare_callback,
                                                        ChangeCallback resume_callback) {
  prepare_reconfigure_callback_ = std::move(prepare_callback);
  resume_reconfigure_callback_ = std::move(resume_callback);
}

void WifiProvisioningService::set_scan_callbacks(ChangeCallback prepare_callback,
                                                 ChangeCallback resume_callback) {
  prepare_scan_callback_ = std::move(prepare_callback);
  resume_scan_callback_ = std::move(resume_callback);
}

void WifiProvisioningService::set_apply_completed_callback(ChangeCallback callback) {
  apply_completed_callback_ = std::move(callback);
}

esp_err_t WifiProvisioningService::load_or_set_defaults(const WifiProvisioningDefaults &defaults) {
  defaults_ = defaults;
  StoredWifiConfig stored_config;
  const esp_err_t load_err = load_stored_wifi_config(&stored_config);
  last_load_result_ = load_err;
  if (load_err == ESP_OK && stored_config.has_saved_config) {
    wifi_config_ = stored_config;
    if (!wifi_config_.has_saved_band_policy) {
      wifi_config_.band_policy = defaults.band_policy;
    }
  } else {
    if (load_err != ESP_OK) {
      ESPECTRE_LOGW(TAG, "Failed to load stored Wi-Fi config: %s; using build defaults", esp_err_to_name(load_err));
    }
    wifi_config_.ssid = defaults.ssid != nullptr ? defaults.ssid : "";
    wifi_config_.password = defaults.password != nullptr ? defaults.password : "";
    wifi_config_.bssid = defaults.bssid != nullptr ? defaults.bssid : "";
    wifi_config_.channel = defaults.channel;
    wifi_config_.band_policy = defaults.band_policy;
    wifi_config_.has_saved_band_policy = false;
    wifi_config_.has_saved_config = false;
  }
  if (!wifi_band_policy_is_supported(wifi_config_.band_policy)) {
    ESPECTRE_LOGW(TAG, "Stored Wi-Fi band policy is unsupported; restoring build default");
    wifi_config_.band_policy = defaults_.band_policy;
    wifi_config_.has_saved_band_policy = false;
  }
  if (!wifi_channel_is_supported(wifi_config_.channel) ||
      !wifi_channel_matches_band_policy(wifi_config_.channel, wifi_config_.band_policy)) {
    ESPECTRE_LOGW(TAG, "Stored Wi-Fi channel %u does not match band policy; restoring automatic scan",
             static_cast<unsigned>(wifi_config_.channel));
    wifi_config_.channel = WIFI_CHANNEL_AUTO;
    if (wifi_config_.has_saved_config) {
      const esp_err_t save_err = save_stored_wifi_config(wifi_config_);
      if (save_err != ESP_OK) {
        ESPECTRE_LOGW(TAG, "Failed to persist the normalized Wi-Fi channel: %s", esp_err_to_name(save_err));
      }
    }
  }
  last_good_config_ = wifi_config_;
  candidate_config_ = StoredWifiConfig{};
  apply_state_ = WifiProvisioningApplyState::IDLE;
  apply_message_.clear();
  candidate_apply_pending_ = false;
  apply_started_ms_ = 0U;
  StoredWifiConfig pending_config;
  bool has_pending = false;
  const esp_err_t pending_err = load_pending_wifi_config(&pending_config, &has_pending);
  if (pending_err != ESP_OK) {
    ESPECTRE_LOGW(TAG, "Failed to load pending Wi-Fi config: %s", esp_err_to_name(pending_err));
    last_load_result_ = pending_err;
  } else if (has_pending && !pending_config.ssid.empty()) {
    candidate_config_ = std::move(pending_config);
    candidate_apply_pending_ = true;
    apply_state_ = WifiProvisioningApplyState::VERIFYING;
    apply_message_ = "Resuming pending Wi-Fi candidate from non-volatile storage";
  }
  refresh_cached_strings_();
  notify_changed_();
  return ESP_OK;
}

esp_err_t WifiProvisioningService::setup_station(const WifiProvisioningDefaults &defaults,
                                                 standalone_wifi_callback_t connected_cb,
                                                 standalone_wifi_callback_t disconnected_cb) {
  if (wifi_manager_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  const esp_err_t load_err = load_or_set_defaults(defaults);
  if (load_err != ESP_OK) {
    return load_err;
  }

  StandaloneWifiConfig wifi_config;
  wifi_config.ssid = wifi_ssid_.c_str();
  wifi_config.password = wifi_password_.c_str();
  wifi_config.bssid = wifi_bssid_.c_str();
  wifi_config.channel = wifi_config_.channel;
  wifi_config.max_retry = defaults_.max_retry;
  wifi_config.manage_csi_lifecycle = defaults_.manage_csi_lifecycle;
  wifi_config.band_policy = wifi_config_.band_policy;
  auto on_connected = [this, connected_cb]() {
    const bool transaction_pending = this->apply_pending();
    this->handle_connected_();
    if (connected_cb) {
      connected_cb();
    }
    this->resume_reconfigure_();
    if (!transaction_pending) {
      this->notify_changed_();
    }
  };
  auto on_disconnected = [this, disconnected_cb]() {
    if (disconnected_cb) {
      disconnected_cb();
    }
    this->notify_changed_();
  };
  return wifi_manager_->setup(wifi_config, on_connected, on_disconnected);
}

bool WifiProvisioningService::handle_command(const std::string &command, std::string *message) {
  auto set_message = [message](const char *value) {
    if (message != nullptr) {
      *message = value;
    }
  };

  constexpr const char *kBssidPrefix = "SET_WIFI_BSSID:";

  if (command.rfind(kBssidPrefix, 0) == 0) {
    if (apply_pending() || scan_active_) {
      set_message("Wi-Fi configuration change already in progress");
      return false;
    }
    std::vector<std::pair<std::string, std::string>> pairs;
    std::string error;
    if (!parse_urlencoded_key_value_pairs(command.substr(std::strlen(kBssidPrefix)), &pairs, &error)) {
      set_message(error.c_str());
      return false;
    }
    std::string requested_bssid;
    bool force = false;
    bool has_bssid = false;
    bool has_force = false;
    for (const auto &pair : pairs) {
      if (pair.first == "bssid" && !has_bssid) {
        requested_bssid = pair.second;
        has_bssid = true;
      } else if (pair.first == "force" && !has_force &&
                 (pair.second == "true" || pair.second == "false")) {
        force = pair.second == "true";
        has_force = true;
      } else {
        set_message("set Wi-Fi BSSID requires bssid and optional boolean force fields");
        return false;
      }
    }
    if (!has_bssid) {
      set_message("set Wi-Fi BSSID requires bssid and optional boolean force fields");
      return false;
    }
    StoredWifiConfig updated = wifi_config_;
    if (!assign_wifi_config_field("bssid", requested_bssid, &updated, &error)) {
      set_message(error.c_str());
      return false;
    }
    if (updated.ssid.empty()) {
      set_message("provision Wi-Fi over Improv Serial before selecting a BSSID");
      return false;
    }
    updated.channel = WIFI_CHANNEL_AUTO;
    for (const StandaloneWifiAccessPoint &access_point : access_points_) {
      if (!updated.bssid.empty() && access_point.bssid == updated.bssid) {
        updated.channel = access_point.channel;
        break;
      }
    }
    updated.has_saved_config = true;
    const DirectWifiSnapshot wifi = read_direct_wifi_snapshot();
    if (!force && !updated.bssid.empty() && wifi.connected &&
        wifi.bssid == updated.bssid) {
      const esp_err_t save_err = save_stored_wifi_config(updated);
      if (save_err != ESP_OK) {
        set_message(esp_err_to_name(save_err));
        return false;
      }
      const esp_err_t clear_err = clear_pending_wifi_config();
      if (clear_err != ESP_OK) {
        set_message(esp_err_to_name(clear_err));
        return false;
      }
      wifi_config_ = std::move(updated);
      last_good_config_ = wifi_config_;
      candidate_config_ = StoredWifiConfig{};
      refresh_cached_strings_();
      set_apply_state_(WifiProvisioningApplyState::APPLIED,
                       "Wi-Fi BSSID pin verified and saved");
      notify_changed_();
      set_message(apply_message_.c_str());
      return true;
    }
    return begin_candidate_apply_(std::move(updated), message);
  }

  if (command == "CLEAR_WIFI") {
    if (apply_pending()) {
      set_message("Wi-Fi configuration change already in progress");
      return false;
    }
    const esp_err_t err = clear_stored_wifi_config();
    if (err != ESP_OK) {
      set_message(esp_err_to_name(err));
      return false;
    }
    const esp_err_t pending_err = clear_pending_wifi_config();
    if (pending_err != ESP_OK) {
      set_message(esp_err_to_name(pending_err));
      return false;
    }
    const WifiBandPolicy previous_band_policy = wifi_config_.band_policy;
    wifi_config_ = StoredWifiConfig{};
    wifi_config_.band_policy = defaults_.band_policy;
    candidate_config_ = StoredWifiConfig{};
    last_good_config_ = wifi_config_;
    set_apply_state_(WifiProvisioningApplyState::IDLE, "Wi-Fi configuration cleared");
    refresh_cached_strings_();
    notify_changed_();
    if (wifi_config_.band_policy != previous_band_policy) {
      const WifiBandPolicy restored_band_policy = wifi_config_.band_policy;
      wifi_config_.band_policy = previous_band_policy;
      const bool applied = apply_live(message);
      wifi_config_.band_policy = restored_band_policy;
      notify_changed_();
      if (!applied) {
        return false;
      }
      set_message("Wi-Fi config cleared; restart required to restore the default band policy");
      return true;
    }
    return apply_live(message);
  }

  set_message("unknown provisioning command");
  return false;
}

bool WifiProvisioningService::request_access_point_scan(std::string *message) {
  if (wifi_manager_ == nullptr) {
    if (message != nullptr) *message = "Wi-Fi manager is not configured";
    return false;
  }
  if (wifi_config_.ssid.empty()) {
    if (message != nullptr) *message = "provision Wi-Fi over Improv Serial before scanning";
    return false;
  }
  if (apply_pending() || scan_active_) {
    if (message != nullptr) *message = "Wi-Fi operation already in progress";
    return false;
  }

  scan_active_ = true;
  scan_message_ = "Wi-Fi access point scan in progress";
  access_points_.clear();
  if (prepare_scan_callback_) prepare_scan_callback_();
  notify_changed_();
  const esp_err_t err = wifi_manager_->request_scan(
      [this](esp_err_t result, const std::vector<StandaloneWifiAccessPoint> &scanned) {
        access_points_.clear();
        if (result == ESP_OK) {
          for (const StandaloneWifiAccessPoint &access_point : scanned) {
            if (access_point.ssid == wifi_config_.ssid) {
              access_points_.push_back(access_point);
            }
          }
          scan_message_ = access_points_.empty()
                              ? "No access points found for the provisioned Wi-Fi network"
                              : "Wi-Fi access point scan complete";
        } else {
          scan_message_ = esp_err_to_name(result);
        }
        scan_active_ = false;
        if (resume_scan_callback_) resume_scan_callback_();
        notify_changed_();
      });
  if (err != ESP_OK) {
    scan_active_ = false;
    scan_message_ = esp_err_to_name(err);
    if (resume_scan_callback_) resume_scan_callback_();
    notify_changed_();
    if (message != nullptr) *message = scan_message_;
    return false;
  }
  if (message != nullptr) *message = scan_message_;
  return true;
}

bool WifiProvisioningService::apply_live(std::string *message) {
  return apply_config_live_(wifi_config_, message);
}

bool WifiProvisioningService::apply_config_live_(const StoredWifiConfig &config, std::string *message) {
  if (wifi_manager_ == nullptr) {
    if (message != nullptr) {
      *message = "Wi-Fi manager is not configured";
    }
    return false;
  }

  wifi_ssid_ = config.ssid;
  wifi_password_ = config.password;
  wifi_bssid_ = config.bssid;
  StandaloneWifiConfig wifi_config;
  wifi_config.ssid = wifi_ssid_.c_str();
  wifi_config.password = wifi_password_.c_str();
  wifi_config.bssid = wifi_bssid_.c_str();
  wifi_config.channel = config.channel;
  wifi_config.max_retry = defaults_.max_retry;
  wifi_config.manage_csi_lifecycle = defaults_.manage_csi_lifecycle;
  wifi_config.band_policy = config.band_policy;

  if (!reconfigure_active_) {
    reconfigure_active_ = true;
    if (prepare_reconfigure_callback_) {
      prepare_reconfigure_callback_();
    }
  }
  const esp_err_t err = wifi_manager_->update_station_config(wifi_config);
  if (err != ESP_OK) {
    resume_reconfigure_();
  }
  notify_changed_();
  if (message != nullptr) {
    *message = err == ESP_OK ? "Wi-Fi config applied" : esp_err_to_name(err);
  }
  return err == ESP_OK;
}

bool WifiProvisioningService::begin_candidate_apply_(StoredWifiConfig candidate, std::string *message) {
  if (apply_pending()) {
    if (message != nullptr) {
      *message = "Wi-Fi configuration change already in progress";
    }
    return false;
  }
  const esp_err_t save_err = save_pending_wifi_config(candidate);
  if (save_err != ESP_OK) {
    if (message != nullptr) {
      *message = esp_err_to_name(save_err);
    }
    return false;
  }
  last_good_config_ = wifi_config_;
  candidate_config_ = std::move(candidate);
  candidate_apply_pending_ = true;
  apply_started_ms_ = 0U;
  set_apply_state_(WifiProvisioningApplyState::VERIFYING,
                   "Wi-Fi candidate accepted; reconnect after address verification");
  if (message != nullptr) {
    *message = apply_message_;
  }
  return true;
}

bool WifiProvisioningService::begin_serial_provisioning(const std::string &ssid,
                                                        const std::string &password,
                                                        std::string *message) {
  StoredWifiConfig candidate = wifi_config_;
  std::string validation_error;
  if (!assign_wifi_config_field("ssid", ssid, &candidate, &validation_error) ||
      !assign_wifi_config_field("password", password, &candidate, &validation_error)) {
    if (message != nullptr) {
      *message = validation_error;
    }
    return false;
  }
  candidate.bssid.clear();
  candidate.channel = WIFI_CHANNEL_AUTO;
  candidate.has_saved_config = true;
  return begin_candidate_apply_(std::move(candidate), message);
}

void WifiProvisioningService::handle_connected_() {
  if (candidate_apply_pending_) {
    return;
  }
  if (apply_state_ == WifiProvisioningApplyState::VERIFYING) {
    if (!candidate_config_.bssid.empty()) {
      const DirectWifiSnapshot wifi = read_direct_wifi_snapshot();
      if (!wifi.connected || wifi.bssid != candidate_config_.bssid) return;
    }
    const esp_err_t save_err = save_stored_wifi_config(candidate_config_);
    if (save_err != ESP_OK) {
      begin_rollback_(esp_err_to_name(save_err));
      return;
    }
    const esp_err_t clear_err = clear_pending_wifi_config();
    if (clear_err != ESP_OK) {
      wifi_config_ = candidate_config_;
      last_good_config_ = wifi_config_;
      refresh_cached_strings_();
      set_apply_state_(WifiProvisioningApplyState::RECOVERY_REQUIRED,
                       "Wi-Fi candidate was saved, but the pending journal could not be cleared");
      return;
    }
    wifi_config_ = candidate_config_;
    refresh_cached_strings_();
    set_apply_state_(WifiProvisioningApplyState::APPLIED,
                     "Wi-Fi candidate verified and saved");
    if (apply_completed_callback_) {
      apply_completed_callback_();
    }
    return;
  }
  if (apply_state_ == WifiProvisioningApplyState::ROLLING_BACK) {
    wifi_config_ = last_good_config_;
    refresh_cached_strings_();
    set_apply_state_(WifiProvisioningApplyState::ROLLED_BACK,
                     "Wi-Fi candidate failed; last-known-good configuration restored");
    if (apply_completed_callback_) {
      apply_completed_callback_();
    }
  }
}

void WifiProvisioningService::loop() {
  if (!apply_pending()) {
    return;
  }
  if (candidate_apply_pending_) {
    candidate_apply_pending_ = false;
    apply_started_ms_ = monotonic_now_ms();
    std::string apply_error;
    if (!apply_config_live_(candidate_config_, &apply_error)) {
      begin_rollback_(apply_error.c_str());
    }
    return;
  }
  const uint32_t elapsed_ms = monotonic_now_ms() - apply_started_ms_;
  if (elapsed_ms < defaults_.candidate_timeout_ms) {
    return;
  }

  if (apply_state_ == WifiProvisioningApplyState::VERIFYING) {
    begin_rollback_("candidate association or address acquisition timed out");
    return;
  }

  set_apply_state_(WifiProvisioningApplyState::RECOVERY_REQUIRED,
                   "last-known-good Wi-Fi rollback timed out; use Improv Serial recovery");
}

void WifiProvisioningService::begin_rollback_(const char *reason) {
  candidate_apply_pending_ = false;
  const esp_err_t clear_err = clear_pending_wifi_config();
  apply_started_ms_ = monotonic_now_ms();
  std::string rollback_message = "Wi-Fi candidate failed";
  if (reason != nullptr && reason[0] != '\0') {
    rollback_message += ": ";
    rollback_message += reason;
  }
  rollback_message += "; restoring last-known-good configuration";
  set_apply_state_(WifiProvisioningApplyState::ROLLING_BACK, rollback_message.c_str());
  std::string apply_error;
  if (!apply_config_live_(last_good_config_, &apply_error)) {
    set_apply_state_(WifiProvisioningApplyState::RECOVERY_REQUIRED,
                     "last-known-good Wi-Fi rollback could not be applied; use Improv Serial recovery");
  } else if (clear_err != ESP_OK) {
    set_apply_state_(WifiProvisioningApplyState::RECOVERY_REQUIRED,
                     "last-known-good Wi-Fi rollback started, but the pending journal could not be cleared");
  }
}

void WifiProvisioningService::set_apply_state_(WifiProvisioningApplyState state, const char *message) {
  apply_state_ = state;
  apply_message_ = message != nullptr ? message : "";
  notify_changed_();
}

bool WifiProvisioningService::apply_pending() const {
  return apply_state_ == WifiProvisioningApplyState::VERIFYING ||
         apply_state_ == WifiProvisioningApplyState::ROLLING_BACK;
}

void WifiProvisioningService::resume_reconfigure_() {
  if (!reconfigure_active_) {
    return;
  }
  reconfigure_active_ = false;
  if (resume_reconfigure_callback_) {
    resume_reconfigure_callback_();
  }
}

void WifiProvisioningService::refresh_cached_strings_() {
  wifi_ssid_ = wifi_config_.ssid;
  wifi_password_ = wifi_config_.password;
  wifi_bssid_ = wifi_config_.bssid;
}

void WifiProvisioningService::notify_changed_() {
  if (change_callback_) {
    change_callback_();
  }
}

}  // namespace espectre
