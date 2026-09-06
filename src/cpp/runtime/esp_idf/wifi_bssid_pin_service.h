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
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "esp_err.h"

namespace espectre {

struct WifiBssidPinStationState {
  bool configured{false};
  bool connected{false};
  bool has_ipv4{false};
  std::string ssid;
  std::string bssid;
};

enum class WifiBssidPinApplyState : uint8_t {
  IDLE = 0,
  VERIFYING,
  ROLLING_BACK,
  APPLIED,
  ROLLED_BACK,
  RECOVERY_REQUIRED,
};

const char *wifi_bssid_pin_apply_state_name(WifiBssidPinApplyState state);

struct WifiBssidPinServiceConfig {
  using ApplyCallback = std::function<bool(const std::string &bssid,
                                           std::string *message,
                                           bool *station_transition_started)>;
  using StationStateGetter = std::function<WifiBssidPinStationState()>;
  using ChangeCallback = std::function<void()>;

  ApplyCallback apply_callback;
  StationStateGetter station_state_getter;
  ChangeCallback prepare_callback;
  ChangeCallback resume_callback;
  uint32_t candidate_timeout_ms{60000U};
};

class WifiBssidPinService {
 public:
  esp_err_t setup(WifiBssidPinServiceConfig config);

  /** Stage a pin update. An empty BSSID clears the current override. */
  bool request_update(const std::string &bssid, std::string *message, bool force = false);
  /** Schedule a station snapshot refresh after a Wi-Fi or IP event. */
  void notify_station_changed();
  /** Advance verification, rollback, and boot-time enforcement. */
  void loop();

  WifiBssidPinApplyState apply_state() const { return apply_state_; }
  const std::string &apply_message() const { return apply_message_; }
  const std::string &stored_ssid() const { return stored_ssid_; }
  const std::string &stored_bssid() const { return stored_bssid_; }
  bool apply_pending() const;

 private:
  bool begin_apply_(const WifiBssidPinStationState &station,
                    const std::string &target_bssid,
                    bool persist_on_success,
                    std::string *message);
  void process_station_state_(const WifiBssidPinStationState &station);
  void begin_rollback_(const char *reason);
  void finish_apply_(WifiBssidPinApplyState state, const char *message);
  esp_err_t load_stored_pin_();
  esp_err_t persist_pending_pin_(const std::string &ssid, const std::string &bssid);
  esp_err_t clear_pending_pin_();
  esp_err_t commit_candidate_pin_();
  esp_err_t persist_pin_(const std::string &ssid, const std::string &bssid);
  esp_err_t clear_stored_pin_();

  WifiBssidPinServiceConfig config_;
  WifiBssidPinApplyState apply_state_{WifiBssidPinApplyState::IDLE};
  std::string apply_message_;
  std::string stored_ssid_;
  std::string stored_bssid_;
  std::string candidate_ssid_;
  std::string candidate_bssid_;
  std::string previous_bssid_;
  std::string pending_ssid_;
  std::string pending_bssid_;
  uint32_t apply_started_ms_{0U};
  bool initialized_{false};
  bool station_refresh_pending_{false};
  bool persist_on_success_{false};
  bool reconfigure_active_{false};
  bool pending_pin_loaded_{false};
};

}  // namespace espectre
