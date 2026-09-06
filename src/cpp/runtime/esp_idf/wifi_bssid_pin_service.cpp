/*
 * ESPectre - Wi-Fi BSSID Pin Service
 *
 * Persists an SSID-bound BSSID override and applies it transactionally without
 * taking ownership of frontend-managed Wi-Fi credentials.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "wifi_bssid_pin_service.h"

#include <cctype>
#include <utility>

#include "espectre_log.h"
#include "nvs.h"
#include "runtime_time.h"

namespace espectre {

namespace {

static const char *const TAG = "espectre.bssid_pin";
constexpr const char *kNamespace = "espectre";
constexpr const char *kSsidKey = "pin_ssid";
constexpr const char *kBssidKey = "pin_bssid";
constexpr const char *kPendingKey = "pin_pending";
constexpr const char *kPendingSsidKey = "pin_p_ssid";
constexpr const char *kPendingBssidKey = "pin_p_bssid";

esp_err_t read_string(nvs_handle_t handle, const char *key, std::string *value) {
  size_t length = 0U;
  esp_err_t err = nvs_get_str(handle, key, nullptr, &length);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    value->clear();
    return ESP_OK;
  }
  if (err != ESP_OK) return err;

  std::string buffer(length, '\0');
  err = nvs_get_str(handle, key, buffer.data(), &length);
  if (err != ESP_OK) return err;
  if (!buffer.empty() && buffer.back() == '\0') buffer.pop_back();
  *value = std::move(buffer);
  return ESP_OK;
}

bool normalize_bssid(const std::string &value, std::string *normalized) {
  if (normalized == nullptr || value.size() != 17U) return false;
  *normalized = value;
  for (size_t index = 0U; index < normalized->size(); ++index) {
    char &current = (*normalized)[index];
    if (index % 3U == 2U) {
      if (current != ':') return false;
      continue;
    }
    if (!std::isxdigit(static_cast<unsigned char>(current))) return false;
    current = static_cast<char>(std::toupper(static_cast<unsigned char>(current)));
  }
  return true;
}

}  // namespace

const char *wifi_bssid_pin_apply_state_name(WifiBssidPinApplyState state) {
  switch (state) {
    case WifiBssidPinApplyState::VERIFYING:
      return "verifying";
    case WifiBssidPinApplyState::ROLLING_BACK:
      return "rolling_back";
    case WifiBssidPinApplyState::APPLIED:
      return "applied";
    case WifiBssidPinApplyState::ROLLED_BACK:
      return "rolled_back";
    case WifiBssidPinApplyState::RECOVERY_REQUIRED:
      return "recovery_required";
    case WifiBssidPinApplyState::IDLE:
    default:
      return "idle";
  }
}

esp_err_t WifiBssidPinService::setup(WifiBssidPinServiceConfig config) {
  if (!config.apply_callback || !config.station_state_getter ||
      config.candidate_timeout_ms == 0U) {
    return ESP_ERR_INVALID_ARG;
  }
  config_ = std::move(config);
  const esp_err_t load_err = load_stored_pin_();
  if (load_err != ESP_OK) return load_err;
  initialized_ = true;
  return ESP_OK;
}

bool WifiBssidPinService::request_update(const std::string &bssid,
                                         std::string *message,
                                         bool force) {
  if (!initialized_) {
    if (message != nullptr) *message = "Wi-Fi BSSID persistence is unavailable";
    return false;
  }
  if (apply_pending()) {
    if (message != nullptr) *message = "Wi-Fi BSSID update already in progress";
    return false;
  }

  std::string normalized;
  if (!bssid.empty() && !normalize_bssid(bssid, &normalized)) {
    if (message != nullptr) *message = "BSSID must contain six hexadecimal octets";
    return false;
  }
  const WifiBssidPinStationState station = config_.station_state_getter();
  if (!station.configured || station.ssid.empty()) {
    if (message != nullptr) *message = "provision Wi-Fi through Matter before selecting a BSSID";
    return false;
  }

  if (!force && !normalized.empty() && station.connected && station.has_ipv4 &&
      station.bssid == normalized) {
    const esp_err_t save_err = persist_pin_(station.ssid, normalized);
    if (save_err != ESP_OK) {
      if (message != nullptr) *message = esp_err_to_name(save_err);
      return false;
    }
    const esp_err_t clear_err = clear_pending_pin_();
    if (clear_err != ESP_OK) {
      if (message != nullptr) *message = esp_err_to_name(clear_err);
      return false;
    }
    apply_state_ = WifiBssidPinApplyState::APPLIED;
    apply_message_ = "Wi-Fi BSSID pin verified and saved";
    if (message != nullptr) *message = apply_message_;
    return true;
  }

  return begin_apply_(station, normalized, true, message);
}

void WifiBssidPinService::notify_station_changed() { station_refresh_pending_ = true; }

void WifiBssidPinService::loop() {
  if (!initialized_) return;

  if (apply_pending() &&
      monotonic_now_ms() - apply_started_ms_ >= config_.candidate_timeout_ms) {
    if (apply_state_ == WifiBssidPinApplyState::VERIFYING) {
      begin_rollback_("association or address acquisition timed out");
    } else {
      finish_apply_(WifiBssidPinApplyState::RECOVERY_REQUIRED,
                    "Wi-Fi BSSID rollback timed out; automatic association may require recovery");
    }
  }

  if (!station_refresh_pending_) return;
  station_refresh_pending_ = false;
  process_station_state_(config_.station_state_getter());
}

bool WifiBssidPinService::apply_pending() const {
  return apply_state_ == WifiBssidPinApplyState::VERIFYING ||
         apply_state_ == WifiBssidPinApplyState::ROLLING_BACK;
}

bool WifiBssidPinService::begin_apply_(const WifiBssidPinStationState &station,
                                       const std::string &target_bssid,
                                       bool persist_on_success,
                                       std::string *message) {
  if (persist_on_success) {
    const esp_err_t persist_err = persist_pending_pin_(station.ssid, target_bssid);
    if (persist_err != ESP_OK) {
      if (message != nullptr) *message = esp_err_to_name(persist_err);
      return false;
    }
  }
  candidate_ssid_ = station.ssid;
  candidate_bssid_ = target_bssid;
  previous_bssid_ = stored_ssid_ == candidate_ssid_ ? stored_bssid_ : std::string{};
  persist_on_success_ = persist_on_success;
  apply_started_ms_ = monotonic_now_ms();
  apply_state_ = WifiBssidPinApplyState::VERIFYING;
  apply_message_ = target_bssid.empty() ? "Wi-Fi BSSID clear started"
                                        : "Wi-Fi BSSID update started";
  if (config_.prepare_callback) {
    config_.prepare_callback();
    reconfigure_active_ = true;
  }

  std::string apply_error;
  bool station_transition_started = false;
  if (!config_.apply_callback(target_bssid, &apply_error, &station_transition_started)) {
    if (station_transition_started) {
      if (persist_on_success) {
        const esp_err_t clear_err = clear_pending_pin_();
        if (clear_err != ESP_OK) {
          finish_apply_(WifiBssidPinApplyState::RECOVERY_REQUIRED,
                        "Wi-Fi BSSID rollback started, but the pending journal could not be cleared");
          if (message != nullptr) *message = apply_message_;
          return false;
        }
      }
      apply_state_ = WifiBssidPinApplyState::ROLLING_BACK;
      apply_message_ = apply_error.empty()
                           ? "Wi-Fi BSSID update failed; waiting for the previous station config"
                           : apply_error;
      persist_on_success_ = false;
      if (message != nullptr) *message = apply_message_;
      return false;
    }
    const esp_err_t clear_err = persist_on_success ? clear_pending_pin_() : ESP_OK;
    if (clear_err != ESP_OK) {
      finish_apply_(WifiBssidPinApplyState::RECOVERY_REQUIRED,
                    "Wi-Fi BSSID update did not start, and the pending journal could not be cleared");
    } else {
      finish_apply_(WifiBssidPinApplyState::IDLE,
                    apply_error.empty() ? "Wi-Fi BSSID update could not be started"
                                        : apply_error.c_str());
    }
    if (message != nullptr) *message = apply_message_;
    return false;
  }
  if (message != nullptr) *message = apply_message_;
  return true;
}

void WifiBssidPinService::process_station_state_(const WifiBssidPinStationState &station) {
  if (apply_pending() && station.configured && !station.ssid.empty() &&
      station.ssid != candidate_ssid_) {
    const esp_err_t clear_err = clear_pending_pin_();
    if (clear_err != ESP_OK) {
      finish_apply_(WifiBssidPinApplyState::RECOVERY_REQUIRED,
                    "commissioned SSID changed, but the pending Wi-Fi BSSID journal could not be cleared");
    } else {
      finish_apply_(WifiBssidPinApplyState::ROLLED_BACK,
                    "Wi-Fi BSSID candidate discarded after the commissioned SSID changed");
    }
    return;
  }

  if (apply_state_ == WifiBssidPinApplyState::VERIFYING) {
    const bool target_matches = candidate_bssid_.empty() || station.bssid == candidate_bssid_;
    if (!station.connected || !station.has_ipv4 || !target_matches) return;

    if (persist_on_success_) {
      const esp_err_t persist_err = commit_candidate_pin_();
      if (persist_err != ESP_OK) {
        begin_rollback_(esp_err_to_name(persist_err));
        return;
      }
    }
    finish_apply_(WifiBssidPinApplyState::APPLIED,
                  candidate_bssid_.empty() ? "Wi-Fi BSSID pin cleared"
                                           : "Wi-Fi BSSID pin verified and saved");
    return;
  }

  if (pending_pin_loaded_) {
    if (!station.configured || !station.connected || !station.has_ipv4) return;
    if (station.ssid != pending_ssid_) {
      const esp_err_t clear_err = clear_pending_pin_();
      if (clear_err != ESP_OK) {
        finish_apply_(WifiBssidPinApplyState::RECOVERY_REQUIRED,
                      "commissioned SSID changed, but the pending Wi-Fi BSSID journal could not be cleared");
      }
      return;
    }
    const std::string target_bssid = pending_bssid_;
    pending_pin_loaded_ = false;
    std::string message;
    if (!begin_apply_(station, target_bssid, true, &message)) {
      ESPECTRE_LOGW(TAG, "Pending Wi-Fi BSSID pin could not be resumed: %s", message.c_str());
    }
    return;
  }

  if (apply_state_ == WifiBssidPinApplyState::ROLLING_BACK) {
    const bool rollback_matches = previous_bssid_.empty() || station.bssid == previous_bssid_;
    if (station.connected && station.has_ipv4 && rollback_matches) {
      finish_apply_(WifiBssidPinApplyState::ROLLED_BACK,
                    "previous Wi-Fi BSSID configuration restored");
    }
    return;
  }

  if (stored_bssid_.empty() || !station.connected || !station.has_ipv4) return;
  if (station.ssid != stored_ssid_) return;
  if (station.bssid == stored_bssid_) return;

  std::string message;
  if (!begin_apply_(station, stored_bssid_, false, &message)) {
    ESPECTRE_LOGW(TAG, "Stored Wi-Fi BSSID pin could not be applied: %s", message.c_str());
  }
}

void WifiBssidPinService::begin_rollback_(const char *reason) {
  const esp_err_t clear_err = clear_pending_pin_();
  apply_state_ = WifiBssidPinApplyState::ROLLING_BACK;
  apply_message_ = "Wi-Fi BSSID candidate failed";
  if (reason != nullptr && reason[0] != '\0') {
    apply_message_ += ": ";
    apply_message_ += reason;
  }
  apply_message_ += "; restoring the previous pin";
  apply_started_ms_ = monotonic_now_ms();
  persist_on_success_ = false;

  std::string rollback_error;
  if (!config_.apply_callback(previous_bssid_, &rollback_error, nullptr)) {
    finish_apply_(WifiBssidPinApplyState::RECOVERY_REQUIRED,
                  rollback_error.empty() ? "previous Wi-Fi BSSID configuration could not be restored"
                                         : rollback_error.c_str());
  } else if (clear_err != ESP_OK) {
    finish_apply_(WifiBssidPinApplyState::RECOVERY_REQUIRED,
                  "previous Wi-Fi BSSID configuration was restored, but the pending journal could not be cleared");
  }
}

void WifiBssidPinService::finish_apply_(WifiBssidPinApplyState state, const char *message) {
  apply_state_ = state;
  apply_message_ = message != nullptr ? message : "";
  candidate_ssid_.clear();
  candidate_bssid_.clear();
  previous_bssid_.clear();
  apply_started_ms_ = 0U;
  persist_on_success_ = false;
  if (state != WifiBssidPinApplyState::RECOVERY_REQUIRED && reconfigure_active_ &&
      config_.resume_callback) {
    config_.resume_callback();
  }
  reconfigure_active_ = false;
}

esp_err_t WifiBssidPinService::load_stored_pin_() {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READONLY, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
  if (err != ESP_OK) return err;

  std::string ssid;
  std::string bssid;
  err = read_string(handle, kSsidKey, &ssid);
  if (err == ESP_OK) err = read_string(handle, kBssidKey, &bssid);
  uint8_t has_pending = 0U;
  const esp_err_t pending_flag_err = nvs_get_u8(handle, kPendingKey, &has_pending);
  if (err == ESP_OK && pending_flag_err != ESP_OK && pending_flag_err != ESP_ERR_NVS_NOT_FOUND) {
    err = pending_flag_err;
  }
  if (err == ESP_OK && has_pending != 0U) {
    err = read_string(handle, kPendingSsidKey, &pending_ssid_);
    if (err == ESP_OK) err = read_string(handle, kPendingBssidKey, &pending_bssid_);
  }
  nvs_close(handle);
  if (err != ESP_OK) return err;

  if (has_pending != 0U) {
    std::string normalized_pending;
    if (pending_ssid_.empty() || pending_ssid_.size() > 32U ||
        (!pending_bssid_.empty() && !normalize_bssid(pending_bssid_, &normalized_pending))) {
      ESPECTRE_LOGW(TAG, "Discarding an invalid pending Wi-Fi BSSID pin");
      const esp_err_t clear_err = clear_pending_pin_();
      if (clear_err != ESP_OK) return clear_err;
    } else {
      if (!pending_bssid_.empty()) pending_bssid_ = std::move(normalized_pending);
      pending_pin_loaded_ = true;
    }
  }

  if (ssid.empty() && bssid.empty()) return ESP_OK;

  std::string normalized;
  if (ssid.empty() || ssid.size() > 32U || !normalize_bssid(bssid, &normalized)) {
    ESPECTRE_LOGW(TAG, "Discarding an invalid stored Wi-Fi BSSID pin");
    return clear_stored_pin_();
  }
  stored_ssid_ = std::move(ssid);
  stored_bssid_ = std::move(normalized);
  return ESP_OK;
}

esp_err_t WifiBssidPinService::persist_pending_pin_(const std::string &ssid,
                                                    const std::string &bssid) {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) return err;
  err = nvs_set_str(handle, kPendingSsidKey, ssid.c_str());
  if (err == ESP_OK) err = nvs_set_str(handle, kPendingBssidKey, bssid.c_str());
  if (err == ESP_OK) err = nvs_set_u8(handle, kPendingKey, 1U);
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  if (err == ESP_OK) {
    pending_ssid_ = ssid;
    pending_bssid_ = bssid;
  }
  return err;
}

esp_err_t WifiBssidPinService::clear_pending_pin_() {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
  if (err != ESP_OK) return err;
  const char *keys[] = {kPendingKey, kPendingSsidKey, kPendingBssidKey};
  for (const char *key : keys) {
    const esp_err_t erase_err = nvs_erase_key(handle, key);
    if (err == ESP_OK && erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) {
      err = erase_err;
    }
  }
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  if (err == ESP_OK) {
    pending_ssid_.clear();
    pending_bssid_.clear();
    pending_pin_loaded_ = false;
  }
  return err;
}

esp_err_t WifiBssidPinService::commit_candidate_pin_() {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) return err;
  if (candidate_bssid_.empty()) {
    const esp_err_t ssid_err = nvs_erase_key(handle, kSsidKey);
    const esp_err_t bssid_err = nvs_erase_key(handle, kBssidKey);
    if (ssid_err != ESP_OK && ssid_err != ESP_ERR_NVS_NOT_FOUND) err = ssid_err;
    if (err == ESP_OK && bssid_err != ESP_OK && bssid_err != ESP_ERR_NVS_NOT_FOUND) err = bssid_err;
  } else {
    err = nvs_set_str(handle, kSsidKey, candidate_ssid_.c_str());
    if (err == ESP_OK) err = nvs_set_str(handle, kBssidKey, candidate_bssid_.c_str());
  }
  const char *pending_keys[] = {kPendingKey, kPendingSsidKey, kPendingBssidKey};
  for (const char *key : pending_keys) {
    const esp_err_t erase_err = nvs_erase_key(handle, key);
    if (err == ESP_OK && erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) err = erase_err;
  }
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  if (err == ESP_OK) {
    stored_ssid_ = candidate_bssid_.empty() ? std::string{} : candidate_ssid_;
    stored_bssid_ = candidate_bssid_;
    pending_ssid_.clear();
    pending_bssid_.clear();
    pending_pin_loaded_ = false;
  }
  return err;
}

esp_err_t WifiBssidPinService::persist_pin_(const std::string &ssid, const std::string &bssid) {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) return err;
  err = nvs_set_str(handle, kSsidKey, ssid.c_str());
  if (err == ESP_OK) err = nvs_set_str(handle, kBssidKey, bssid.c_str());
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  if (err == ESP_OK) {
    stored_ssid_ = ssid;
    stored_bssid_ = bssid;
  }
  return err;
}

esp_err_t WifiBssidPinService::clear_stored_pin_() {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    stored_ssid_.clear();
    stored_bssid_.clear();
    return ESP_OK;
  }
  if (err != ESP_OK) return err;

  const esp_err_t ssid_err = nvs_erase_key(handle, kSsidKey);
  const esp_err_t bssid_err = nvs_erase_key(handle, kBssidKey);
  if (ssid_err != ESP_OK && ssid_err != ESP_ERR_NVS_NOT_FOUND) {
    err = ssid_err;
  } else if (bssid_err != ESP_OK && bssid_err != ESP_ERR_NVS_NOT_FOUND) {
    err = bssid_err;
  } else {
    err = nvs_commit(handle);
  }
  nvs_close(handle);
  if (err == ESP_OK) {
    stored_ssid_.clear();
    stored_bssid_.clear();
  }
  return err;
}

}  // namespace espectre
