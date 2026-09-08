/*
 * ESPectre - Main Component Implementation
 *
 * Main ESPHome component that orchestrates all ESPectre subsystems.
 * Integrates CSI processing, calibration, and Home Assistant publishing.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "espectre.h"
#include "threshold_number.h"
#include "motion_hits_number.h"
#include "sensing_switch.h"
#include "detector_select.h"
#include "traffic_mode_select.h"
#include "esphome_log_sink.h"

#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#include "device_identity.h"
#include "direct_http_protocol.h"
#include "espectre_banner.h"
#include "espectre_protocol.h"
#include "frontend/frontend_firmware_version.h"
#include "primary_console.h"
#include "protocol_json.h"
#include "sdkconfig.h"

#include <cctype>
#include <cmath>
#include <cstdio>

namespace esphome {
namespace espectre_component {

namespace {

constexpr uint32_t kMdnsRetryIntervalMs = 5000U;
constexpr uint32_t kWifiBssidPinApplyTimeoutMs = 35000U;

bool bssid_equals(const std::string &left, const std::string &right) {
  if (left.size() != right.size()) {
    return false;
  }
  for (size_t index = 0U; index < left.size(); ++index) {
    if (std::tolower(static_cast<unsigned char>(left[index])) !=
        std::tolower(static_cast<unsigned char>(right[index]))) {
      return false;
    }
  }
  return true;
}

}  // namespace

void ESpectreComponent::setup() {
  const esp_err_t console_result = espectre::initialize_primary_console();
  if (console_result != ESP_OK) {
    ESP_LOGE(TAG, "Primary console initialization failed: %s", esp_err_to_name(console_result));
    this->mark_failed();
    return;
  }
  if (!::espectre::set_log_sink(make_esphome_log_sink())) {
    ESP_LOGE(TAG, "Failed to register the ESPHome log sink");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Initializing ESPectre component...");
  this->runtime_.config().device_id = espectre::derive_runtime_device_id();
  if (global_preferences != nullptr) {
    this->device_label_preference_ =
        global_preferences->make_preference<StoredDeviceLabel>(fnv1_hash("espectre_device_label"));
    StoredDeviceLabel stored;
    if (this->device_label_preference_.load(&stored) && stored.version == 1U) {
      stored.value.back() = '\0';
      this->device_label_override_ = stored.value.data();
    }
    this->wifi_bssid_preference_ =
        global_preferences->make_preference<StoredWifiBssid>(fnv1_hash("espectre_wifi_bssid_v2"));
    StoredWifiBssid wifi_bssid;
    if (this->wifi_bssid_preference_.load(&wifi_bssid) && wifi_bssid.version == 2U) {
      if (wifi_bssid.pinned == 1U) {
        wifi_bssid.value.back() = '\0';
        const std::string pin = wifi_bssid.value.data();
        if (pin.size() == 17U) this->wifi_bssid_pin_ = pin;
      }
      if (wifi_bssid.pending == 1U) {
        wifi_bssid.pending_value.back() = '\0';
        const std::string pending = wifi_bssid.pending_value.data();
        if (pending.empty() || pending.size() == 17U) {
          this->wifi_bssid_pending_target_ = pending;
          this->wifi_bssid_pending_loaded_ = true;
        }
      }
    } else {
      struct LegacyStoredWifiBssid {
        uint8_t version{1U};
        uint8_t pinned{0U};
        std::array<char, 18> value{};
      };
      auto legacy_preference = global_preferences->make_preference<LegacyStoredWifiBssid>(
          fnv1_hash("espectre_wifi_bssid"));
      LegacyStoredWifiBssid legacy;
      if (legacy_preference.load(&legacy) && legacy.version == 1U && legacy.pinned == 1U) {
        legacy.value.back() = '\0';
        const std::string pin = legacy.value.data();
        if (pin.size() == 17U) {
          this->wifi_bssid_pin_ = pin;
          std::string migration_message;
          (void) this->persist_wifi_bssid_pin_(pin, &migration_message);
        }
      }
    }
  }

  this->update_live_telemetry_enabled_();
  if (!this->runtime_.setup(this)) {
    ESP_LOGE(TAG, "ESPectre runtime setup failed");
    this->mark_failed();
    return;
  }
  if (this->threshold_number_ != nullptr) {
    static_cast<ESpectreThresholdNumber *>(this->threshold_number_)
        ->update_detector_range(this->runtime_.config().detection_algorithm);
  }
  if (this->sensing_switch_ != nullptr) {
    static_cast<ESpectreSensingSwitch *>(this->sensing_switch_)->republish_state();
  }

  if (!this->direct_api_enabled_) {
    ESP_LOGI(TAG, "Direct API disabled by ESPHome configuration");
    ESP_LOGI(TAG, "ESPectre initialized successfully");
    return;
  }

  if (!this->direct_bridge_.setup(
          &this->direct_service_,
          &this->runtime_,
          RuntimeDirectHttpBridgeConfig{
              "esphome",
              this->device_name_(),
              std::string(App.get_name()),
              frontend_firmware_version(),
              CONFIG_IDF_TARGET,
              this->runtime_.config().device_id,
              espectre::ESPECTRE_DIRECT_HTTP_PORT,
              true,
              false,
              [this]() { return this->device_label_(); },
              [this](const std::string &label, std::string *message) {
                return this->set_device_label_(label, message);
              },
              {},
              &this->peer_discovery_,
              [this]() { return this->runtime_.diagnostics_sample(); },
              &this->runtime_events_,
              [this](const std::string &bssid, bool force, std::string *message) {
                return this->begin_wifi_bssid_pin_update_(bssid, force, message);
              },
              [this](std::string *message) {
                const bool available =
                    this->wifi_bssid_apply_mode_ == WifiBssidApplyMode::NONE &&
                    !this->wifi_bssid_recovery_pending_;
                if (!available && message != nullptr) {
                  *message = "Wi-Fi BSSID update already in progress";
                }
                return available;
              },
          },
          [this]() { this->sync_direct_config_(); })) {
    ESP_LOGE(TAG, "ESPHome Direct HTTP setup failed");
    this->runtime_.shutdown();
    this->mark_failed();
    return;
  }
  if (!this->mdns_bootstrap_responder_.setup()) {
    ESP_LOGE(TAG, "ESPHome mDNS bootstrap responder setup failed");
    this->direct_bridge_.shutdown();
    this->runtime_.shutdown();
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "ESPectre initialized successfully");
}

ESpectreComponent::~ESpectreComponent() {
  this->mdns_bootstrap_responder_.shutdown();
  this->mdns_discovery_.shutdown();
  this->direct_bridge_.shutdown();
  this->runtime_.shutdown();
  ::espectre::clear_log_sink();
}

void ESpectreComponent::loop() {
  this->runtime_.loop();
  this->process_wifi_bssid_apply_();
  this->drain_pending_runtime_events_();
  this->update_live_telemetry_enabled_();
  if (!this->direct_api_enabled_) return;

  this->direct_bridge_.loop();
  if (this->mdns_ipv4_pending_) {
    this->mdns_ipv4_pending_ = false;
    (void) this->mdns_bootstrap_responder_.update(this->pending_mdns_ipv4_);
  }
  this->mdns_bootstrap_responder_.loop();
  if (!this->mdns_discovery_.service_enabled() && millis() >= this->next_mdns_setup_ms_) {
    this->setup_mdns_discovery_();
  }
}

void ESpectreComponent::update_live_telemetry_enabled_() {
  const bool enabled = this->sensor_publisher_.has_movement_sensor() ||
                       (this->direct_api_enabled_ && this->direct_bridge_.event_client_count() > 0U);
  if (enabled == this->live_telemetry_enabled_) {
    return;
  }
  this->live_telemetry_enabled_ = enabled;
  this->runtime_.set_live_telemetry_enabled(enabled);
}

std::string ESpectreComponent::device_name_() const {
  return espectre_device_name(this->runtime_.config().device_id, CONFIG_IDF_TARGET);
}

const std::string &ESpectreComponent::device_label_() const {
  return this->device_label_override_;
}

std::string ESpectreComponent::display_name_() const {
  return this->device_label_override_.empty() ? this->device_name_() : this->device_label_override_;
}

std::string ESpectreComponent::mdns_instance_name_() const {
  const std::string device_id = format_espectre_device_id(this->runtime_.config().device_id);
  return this->device_label_override_.empty()
             ? "ESPectre " + device_id
             : this->device_label_override_ + " " + device_id;
}

MdnsTxtRecords ESpectreComponent::mdns_txt_records_() const {
  return {
      {"device_id", format_espectre_device_id(this->runtime_.config().device_id)},
      {"name", this->display_name_()},
      {"frontend", "esphome"},
      {"txtvers", ESPECTRE_DNS_SD_TXT_SCHEMA_VERSION},
      {"protovers", ESPECTRE_PROTOCOL_VERSION},
      {"transport", ESPECTRE_DIRECT_HTTP_TRANSPORT},
      {"path", ESPECTRE_DIRECT_HTTP_BASE_ENDPOINT},
      {"firmware", frontend_firmware_version()},
      {"chip", CONFIG_IDF_TARGET},
      {"capabilities", "config,monitor,csi"},
  };
}

bool ESpectreComponent::set_device_label_(const std::string &device_label, std::string *message) {
  if (device_label.size() > ESPECTRE_DEVICE_LABEL_MAX_LENGTH ||
      device_label.find_first_of("\r\n") != std::string::npos) {
    if (message != nullptr) *message = "device label must be at most 32 bytes and one line";
    return false;
  }
  StoredDeviceLabel stored;
  std::copy(device_label.begin(), device_label.end(), stored.value.begin());
  if (!this->device_label_preference_.save(&stored)) {
    if (message != nullptr) *message = "device label could not be persisted";
    return false;
  }
  this->device_label_override_ = device_label;
  if (this->mdns_discovery_.initialized()) {
    (void) this->mdns_discovery_.update_txt(this->mdns_txt_records_());
  }
  if (message != nullptr) *message = "ESPectre label updated";
  return true;
}

bool ESpectreComponent::persist_wifi_bssid_pin_(const std::string &bssid, std::string *message) {
  StoredWifiBssid stored;
  stored.pinned = bssid.empty() ? 0U : 1U;
  if (!bssid.empty()) {
    if (bssid.size() >= stored.value.size()) {
      if (message != nullptr) *message = "Wi-Fi BSSID pin could not be persisted";
      return false;
    }
    std::copy(bssid.begin(), bssid.end(), stored.value.begin());
  }
  if (!this->wifi_bssid_preference_.save(&stored)) {
    if (message != nullptr) *message = "Wi-Fi BSSID pin could not be persisted";
    return false;
  }
  this->wifi_bssid_pin_ = bssid;
  this->wifi_bssid_pending_loaded_ = false;
  this->wifi_bssid_pending_target_.clear();
  if (message != nullptr) {
    *message = bssid.empty() ? "Wi-Fi BSSID pin cleared" : "Wi-Fi BSSID pin persisted";
  }
  return true;
}

bool ESpectreComponent::stage_wifi_bssid_pin_(const std::string &bssid, std::string *message) {
  StoredWifiBssid stored;
  stored.pinned = this->wifi_bssid_pin_.empty() ? 0U : 1U;
  stored.pending = 1U;
  if (!this->wifi_bssid_pin_.empty()) {
    std::copy(this->wifi_bssid_pin_.begin(), this->wifi_bssid_pin_.end(), stored.value.begin());
  }
  if (!bssid.empty()) {
    if (bssid.size() >= stored.pending_value.size()) {
      if (message != nullptr) *message = "Wi-Fi BSSID candidate could not be persisted";
      return false;
    }
    std::copy(bssid.begin(), bssid.end(), stored.pending_value.begin());
  }
  if (!this->wifi_bssid_preference_.save(&stored)) {
    if (message != nullptr) *message = "Wi-Fi BSSID candidate could not be persisted";
    return false;
  }
  this->wifi_bssid_pending_loaded_ = true;
  this->wifi_bssid_pending_target_ = bssid;
  return true;
}

bool ESpectreComponent::apply_esphome_wifi_bssid_pin_(const std::string &bssid,
                                                       std::string *message) {
#if defined(ESP_PLATFORM) && defined(USE_WIFI)
  if (wifi::global_wifi_component == nullptr) {
    if (message != nullptr) *message = "ESPHome Wi-Fi component is unavailable";
    return false;
  }

  const wifi::WiFiAP previous_station = wifi::global_wifi_component->get_sta();
  wifi::WiFiAP station = previous_station;
  if (station.get_ssid().size() == 0U) {
    if (message != nullptr) *message = "ESPHome Wi-Fi station is not configured";
    return false;
  }
  if (bssid.empty()) {
    station.clear_bssid();
  } else {
    unsigned int octets[6]{};
    if (std::sscanf(bssid.c_str(),
                    "%2x:%2x:%2x:%2x:%2x:%2x",
                    &octets[0],
                    &octets[1],
                    &octets[2],
                    &octets[3],
                    &octets[4],
                    &octets[5]) != 6) {
      if (message != nullptr) *message = "Wi-Fi BSSID pin is invalid";
      return false;
    }
    wifi::bssid_t parsed{};
    for (size_t index = 0U; index < parsed.size(); ++index) {
      parsed[index] = static_cast<uint8_t>(octets[index]);
    }
    station.set_bssid(parsed);
  }
  station.clear_channel();
  wifi::global_wifi_component->set_sta(station);
  wifi::global_wifi_component->retry_connect();
  this->wifi_bssid_apply_transition_started_ = true;
  if (message != nullptr) {
    *message = bssid.empty() ? "Wi-Fi BSSID clear queued through ESPHome"
                             : "Wi-Fi BSSID update queued through ESPHome";
  }
  return true;
#else
  return apply_wifi_bssid_pin(bssid, message);
#endif
}

bool ESpectreComponent::begin_wifi_bssid_pin_update_(const std::string &bssid,
                                                     bool force,
                                                     std::string *message) {
  if (this->wifi_bssid_apply_mode_ != WifiBssidApplyMode::NONE ||
      this->wifi_bssid_recovery_pending_) {
    if (message != nullptr) *message = "Wi-Fi BSSID update already in progress";
    return false;
  }
  if (!force && !bssid.empty() && bssid_equals(this->wifi_associated_bssid_, bssid)) {
    return this->persist_wifi_bssid_pin_(bssid, message);
  }
  if (!bssid.empty() && bssid_equals(this->wifi_bssid_pin_, bssid)) {
    this->wifi_bssid_apply_mode_ = WifiBssidApplyMode::ENFORCE;
  } else {
    this->wifi_bssid_apply_mode_ = WifiBssidApplyMode::UPDATE;
  }
  this->wifi_bssid_apply_target_ = bssid;
  this->wifi_bssid_apply_previous_pin_ = this->wifi_bssid_pin_;
  this->wifi_bssid_apply_started_ms_ = millis();
  this->wifi_bssid_apply_resume_sensing_ = this->runtime_.services_armed();
  this->wifi_bssid_apply_transition_started_ = false;
  if (this->wifi_bssid_apply_mode_ == WifiBssidApplyMode::UPDATE &&
      !this->stage_wifi_bssid_pin_(bssid, message)) {
    this->wifi_bssid_apply_mode_ = WifiBssidApplyMode::NONE;
    this->wifi_bssid_apply_target_.clear();
    this->wifi_bssid_apply_previous_pin_.clear();
    this->wifi_bssid_apply_started_ms_ = 0U;
    return false;
  }
  if (this->wifi_bssid_apply_resume_sensing_) {
    this->runtime_.set_services_armed(false);
  }

  std::string apply_message;
  if (!this->apply_esphome_wifi_bssid_pin_(bssid, &apply_message)) {
    this->fail_wifi_bssid_apply_(apply_message.c_str());
    if (message != nullptr) *message = apply_message;
    return false;
  }
  this->wifi_bssid_apply_transition_started_ = false;
  this->wifi_associated_bssid_.clear();
  this->wifi_has_ipv4_ = false;
  if (message != nullptr) {
    *message = bssid.empty() ? "Wi-Fi BSSID clear started" : "Wi-Fi BSSID update started";
  }
  return true;
}

void ESpectreComponent::finish_wifi_bssid_apply_() {
  const bool resume_sensing = this->wifi_bssid_apply_resume_sensing_;
  this->wifi_bssid_apply_mode_ = WifiBssidApplyMode::NONE;
  this->wifi_bssid_apply_target_.clear();
  this->wifi_bssid_apply_previous_pin_.clear();
  this->wifi_bssid_apply_started_ms_ = 0U;
  this->wifi_bssid_apply_resume_sensing_ = false;
  this->wifi_bssid_apply_transition_started_ = false;
  if (resume_sensing) this->runtime_.set_services_armed(true);
}

void ESpectreComponent::fail_wifi_bssid_apply_(const char *reason) {
  const WifiBssidApplyMode mode = this->wifi_bssid_apply_mode_;
  const std::string recovery_pin =
      mode == WifiBssidApplyMode::UPDATE ? this->wifi_bssid_apply_previous_pin_ : std::string{};
  ESP_LOGW(TAG,
           "Wi-Fi BSSID %s failed: %s; restoring %s association",
           mode == WifiBssidApplyMode::UPDATE ? "update" : "enforcement",
           reason != nullptr ? reason : "unknown error",
           recovery_pin.empty() ? "automatic" : "the previous pinned");
  std::string persist_message;
  const bool recovery_persisted = this->persist_wifi_bssid_pin_(recovery_pin, &persist_message);
  if (!recovery_persisted) {
    ESP_LOGW(TAG, "Wi-Fi BSSID recovery journal could not be persisted: %s",
             persist_message.c_str());
  }
  std::string recovery_message;
  const bool recovery_started = this->wifi_bssid_apply_transition_started_ ||
                                this->apply_esphome_wifi_bssid_pin_(recovery_pin, &recovery_message);
  if (!recovery_started) {
    ESP_LOGW(TAG, "Wi-Fi BSSID recovery could not be started: %s", recovery_message.c_str());
  }
  const bool resume_sensing = this->wifi_bssid_apply_resume_sensing_;
  this->wifi_bssid_apply_mode_ = WifiBssidApplyMode::NONE;
  this->wifi_bssid_apply_target_.clear();
  this->wifi_bssid_apply_previous_pin_.clear();
  this->wifi_bssid_apply_started_ms_ = 0U;
  this->wifi_bssid_apply_resume_sensing_ = false;
  this->wifi_bssid_apply_transition_started_ = false;
  if (!recovery_started && recovery_persisted) {
    if (resume_sensing) this->runtime_.set_services_armed(true);
  } else {
    this->wifi_bssid_recovery_pending_ = true;
    this->wifi_bssid_recovery_journal_pending_ = !recovery_persisted;
    this->wifi_bssid_recovery_resume_sensing_ = resume_sensing;
    this->wifi_bssid_recovery_target_ = recovery_pin;
    this->wifi_bssid_recovery_started_ms_ = millis();
  }
}

void ESpectreComponent::process_wifi_bssid_apply_() {
  if (this->wifi_bssid_apply_mode_ == WifiBssidApplyMode::NONE) {
    if (this->wifi_bssid_recovery_pending_ &&
        this->wifi_bssid_recovery_journal_pending_) {
      std::string persist_message;
      if (!this->persist_wifi_bssid_pin_(this->wifi_bssid_recovery_target_,
                                         &persist_message)) {
        return;
      }
      this->wifi_bssid_recovery_journal_pending_ = false;
    }
    if (this->wifi_bssid_recovery_pending_ &&
        !this->wifi_associated_bssid_.empty() && this->wifi_has_ipv4_ &&
        (this->wifi_bssid_recovery_target_.empty() ||
         bssid_equals(this->wifi_associated_bssid_, this->wifi_bssid_recovery_target_))) {
      const bool resume_sensing = this->wifi_bssid_recovery_resume_sensing_;
      this->wifi_bssid_recovery_pending_ = false;
      this->wifi_bssid_recovery_journal_pending_ = false;
      this->wifi_bssid_recovery_resume_sensing_ = false;
      this->wifi_bssid_recovery_target_.clear();
      this->wifi_bssid_recovery_started_ms_ = 0U;
      if (resume_sensing)
        this->runtime_.set_services_armed(true);
      return;
    }
    if (this->wifi_bssid_recovery_pending_ &&
        millis() - this->wifi_bssid_recovery_started_ms_ >= kWifiBssidPinApplyTimeoutMs) {
      ESP_LOGW(TAG, "Wi-Fi BSSID recovery timed out");
      const bool resume_sensing = this->wifi_bssid_recovery_resume_sensing_;
      this->wifi_bssid_recovery_pending_ = false;
      this->wifi_bssid_recovery_journal_pending_ = false;
      this->wifi_bssid_recovery_resume_sensing_ = false;
      this->wifi_bssid_recovery_target_.clear();
      this->wifi_bssid_recovery_started_ms_ = 0U;
      if (resume_sensing) this->runtime_.set_services_armed(true);
    }
    return;
  }

  const bool associated_after_reconnect = !this->wifi_associated_bssid_.empty();
  const bool target_matches = this->wifi_bssid_apply_target_.empty() ||
                              bssid_equals(this->wifi_associated_bssid_,
                                           this->wifi_bssid_apply_target_);
  if (associated_after_reconnect && target_matches && this->wifi_has_ipv4_) {
    if (this->wifi_bssid_apply_mode_ == WifiBssidApplyMode::UPDATE) {
      std::string persist_message;
      if (!this->persist_wifi_bssid_pin_(this->wifi_bssid_apply_target_, &persist_message)) {
        this->fail_wifi_bssid_apply_(persist_message.c_str());
        return;
      }
    }
    ESP_LOGI(TAG,
             "Wi-Fi BSSID %s completed on %s",
             this->wifi_bssid_apply_mode_ == WifiBssidApplyMode::UPDATE ? "update" : "enforcement",
             this->wifi_associated_bssid_.c_str());
    this->finish_wifi_bssid_apply_();
    return;
  }

  if (millis() - this->wifi_bssid_apply_started_ms_ >= kWifiBssidPinApplyTimeoutMs) {
    this->fail_wifi_bssid_apply_("association verification timed out");
  }
}

void ESpectreComponent::handle_wifi_bssid_association_(const std::string &associated_bssid) {
  this->wifi_associated_bssid_ = associated_bssid;
  if (this->wifi_bssid_recovery_pending_) {
    // Wait for the component loop to drain the runtime's Wi-Fi transition and
    // observe IPv4 before rearming CSI.
    return;
  }
  if (this->wifi_bssid_apply_mode_ != WifiBssidApplyMode::NONE) {
    this->process_wifi_bssid_apply_();
    return;
  }
  if (this->wifi_bssid_pending_loaded_) {
    const std::string pending_target = this->wifi_bssid_pending_target_;
    if (!pending_target.empty() && bssid_equals(pending_target, this->wifi_bssid_pin_)) {
      std::string persist_message;
      if (!this->persist_wifi_bssid_pin_(pending_target, &persist_message)) {
        ESP_LOGW(TAG, "Pending Wi-Fi BSSID journal could not be cleared: %s",
                 persist_message.c_str());
        return;
      }
    } else {
      std::string apply_message;
      if (!this->begin_wifi_bssid_pin_update_(pending_target, true, &apply_message)) {
        ESP_LOGW(TAG, "Pending Wi-Fi BSSID update could not be resumed: %s", apply_message.c_str());
        return;
      }
      this->wifi_bssid_pending_loaded_ = false;
      this->wifi_bssid_pending_target_.clear();
      return;
    }
  }
  if (this->wifi_bssid_pin_.empty() ||
      bssid_equals(this->wifi_associated_bssid_, this->wifi_bssid_pin_)) {
    return;
  }

  std::string apply_message;
  this->wifi_bssid_apply_mode_ = WifiBssidApplyMode::ENFORCE;
  this->wifi_bssid_apply_target_ = this->wifi_bssid_pin_;
  this->wifi_bssid_apply_previous_pin_ = this->wifi_bssid_pin_;
  this->wifi_bssid_apply_started_ms_ = millis();
  this->wifi_bssid_apply_resume_sensing_ = this->runtime_.services_armed();
  this->wifi_bssid_apply_transition_started_ = false;
  if (this->wifi_bssid_apply_resume_sensing_) {
    this->runtime_.set_services_armed(false);
  }
  if (!this->apply_esphome_wifi_bssid_pin_(this->wifi_bssid_pin_, &apply_message)) {
    this->fail_wifi_bssid_apply_(apply_message.c_str());
  } else {
    this->wifi_bssid_apply_transition_started_ = false;
    this->wifi_associated_bssid_.clear();
    this->wifi_has_ipv4_ = false;
  }
}

void ESpectreComponent::setup_mdns_discovery_() {
  if (!this->direct_api_enabled_) return;

  const MdnsTxtRecords txt_records = this->mdns_txt_records_();
  if (!this->mdns_discovery_.setup(MdnsDiscoveryServiceConfig{
          "",
          this->mdns_instance_name_(),
          "_espectre",
          "_tcp",
          espectre::ESPECTRE_DIRECT_HTTP_PORT,
          txt_records,
          MdnsResponderMode::USE_EXISTING_RESPONDER,
      })) {
    this->next_mdns_setup_ms_ = millis() + kMdnsRetryIntervalMs;
    return;
  }
  ESP_LOGI(TAG, "Direct HTTP discovery published on port %u", espectre::ESPECTRE_DIRECT_HTTP_PORT);
}

void ESpectreComponent::sync_direct_config_() {
  if (this->threshold_number_ != nullptr) {
    this->threshold_number_->publish_state(this->runtime_.snapshot().threshold);
  }
  if (this->detector_select_ != nullptr) {
    this->detector_select_->publish_state(detection_algorithm_name(this->runtime_.config().detection_algorithm));
  }
  if (this->motion_on_hits_number_ != nullptr) {
    this->motion_on_hits_number_->publish_state(this->runtime_.config().motion_on_hits);
  }
  if (this->motion_off_hits_number_ != nullptr) {
    this->motion_off_hits_number_->publish_state(this->runtime_.config().motion_off_hits);
  }
  if (this->csi_traffic_mode_select_ != nullptr) {
    this->csi_traffic_mode_select_->publish_state(csi_traffic_mode_name(this->runtime_.config().csi_traffic_mode));
  }
  if (this->traffic_generator_mode_select_ != nullptr) {
    this->traffic_generator_mode_select_->publish_state(
        traffic_mode_name(this->runtime_.config().traffic_generator_mode));
  }
  if (this->sensing_switch_ != nullptr) {
    static_cast<ESpectreSensingSwitch *>(this->sensing_switch_)->republish_state();
  }
  if (this->calibration_active_sensor_ != nullptr) {
    this->calibration_active_sensor_->publish_state(this->runtime_.is_calibrating());
  }
}

void ESpectreComponent::publish_cached_diagnostics_() {
  const RuntimeDiagnosticsSample *latest = this->runtime_.diagnostics_sample();
  if (latest == nullptr) {
    return;
  }
  const RuntimeDiagnosticsSample &sample = *latest;

  if (this->traffic_rate_sensor_ != nullptr) {
    this->traffic_rate_sensor_->publish_state(sample.traffic_tx_pps);
  }
  if (this->csi_callback_rate_sensor_ != nullptr) {
    this->csi_callback_rate_sensor_->publish_state(sample.csi_callback_pps);
  }
  if (this->csi_accepted_rate_sensor_ != nullptr) {
    this->csi_accepted_rate_sensor_->publish_state(sample.csi_accepted_pps);
  }
  if (this->csi_admitted_rate_sensor_ != nullptr) {
    this->csi_admitted_rate_sensor_->publish_state(sample.csi_admitted_pps);
  }
  if (this->csi_filtered_rate_sensor_ != nullptr) {
    this->csi_filtered_rate_sensor_->publish_state(sample.csi_filtered_pps);
  }
  if (this->csi_missing_rate_sensor_ != nullptr) {
    this->csi_missing_rate_sensor_->publish_state(sample.csi_missing_slots_pps);
  }
  if (this->csi_excess_rate_sensor_ != nullptr) {
    this->csi_excess_rate_sensor_->publish_state(sample.csi_excess_pps);
  }
  if (this->csi_stale_rate_sensor_ != nullptr) {
    this->csi_stale_rate_sensor_->publish_state(sample.csi_stale_pps);
  }
  if (this->csi_out_of_order_rate_sensor_ != nullptr) {
    this->csi_out_of_order_rate_sensor_->publish_state(sample.csi_out_of_order_pps);
  }
  if (this->csi_occupancy_sensor_ != nullptr) {
    this->csi_occupancy_sensor_->publish_state(sample.csi_occupancy_ratio * 100.0f);
  }
  if (this->wifi_channel_sensor_ != nullptr) this->wifi_channel_sensor_->publish_state(sample.wifi_channel);
  if (this->wifi_rssi_sensor_ != nullptr) {
    this->wifi_rssi_sensor_->publish_state(sample.wifi_rssi_dbm == INT8_MIN
                                               ? NAN
                                               : static_cast<float>(sample.wifi_rssi_dbm));
  }
}

void ESpectreComponent::publish_diagnostics_on_demand() {
  const FrontendCommandResult result = this->execute_entity_command_("read_diagnostics");
  if (result.accepted) this->publish_cached_diagnostics_();
}

FrontendCommandResult ESpectreComponent::execute_entity_command_(const std::string &name,
                                                                const std::string &parameters) {
  EspectreCommand command;
  std::string error;
  if (!parse_espectre_command_request("", name, parameters, &command, &error)) {
    FrontendCommandResult rejected;
    rejected.handled = true;
    rejected.command = command;
    rejected.code = frontend_command_parse_error_code(error);
    rejected.message = error;
    return rejected;
  }
  const RuntimeCapabilities &runtime_capabilities = this->runtime_.capabilities();
  FrontendCommandCapabilities capabilities;
  using Method = EspectreDirectMethod;
  capabilities.set(Method::CAPABILITIES);
  capabilities.set(Method::INFO);
  capabilities.set(Method::STATUS);
  capabilities.set(Method::CONFIG);
  capabilities.set(Method::DIAGNOSTICS, runtime_capabilities.supports_extended_diagnostics);
  capabilities.set(Method::SET_SENSING);
  capabilities.set(Method::SET_THRESHOLD, runtime_capabilities.supports_runtime_threshold_updates);
  capabilities.set(Method::SET_MOTION_HITS, runtime_capabilities.supports_runtime_motion_hits_updates);
  capabilities.set(Method::SET_DETECTOR, runtime_capabilities.supports_runtime_detector_selection);
  capabilities.set(Method::RECALIBRATE, runtime_capabilities.supports_manual_recalibration);
  capabilities.set(Method::SET_CSI_TRAFFIC_MODE, runtime_capabilities.supports_traffic_control);
  capabilities.set(Method::SET_TRAFFIC_GENERATOR_MODE, runtime_capabilities.supports_traffic_control);
  capabilities.set(EspectreConfigSection::RUNTIME);
  FrontendCommandResult result = this->command_engine_.execute(
      command,
      FrontendCommandContext{FrontendCommandOrigin::ESPHOME},
      capabilities,
      [this, capabilities](const EspectreCommand &read) {
        EspectreDeviceConfig device;
        device.device_id = this->runtime_.config().device_id;
        EspectreDeviceInfo info;
        info.frontend = "esphome";
        info.firmware_version = frontend_firmware_version();
        info.chip = CONFIG_IDF_TARGET;
        info.detector = detection_algorithm_name(this->runtime_.config().detection_algorithm);
        info.supports_diagnostics = capabilities.supports(Method::DIAGNOSTICS);
        info.supports_runtime_threshold = capabilities.supports(Method::SET_THRESHOLD);
        info.supports_runtime_motion_hits = capabilities.supports(Method::SET_MOTION_HITS);
        info.supports_runtime_detector = capabilities.supports(Method::SET_DETECTOR);
        info.supports_manual_recalibration = capabilities.supports(Method::RECALIBRATE);
        info.supports_traffic_control = capabilities.supports(Method::SET_CSI_TRAFFIC_MODE) &&
                                        capabilities.supports(Method::SET_TRAFFIC_GENERATOR_MODE);
        info = normalize_protocol_device_info(
            info, &this->runtime_.snapshot(), "esphome", CONFIG_IDF_TARGET);
        if (read.command == "capabilities") {
          return espectre_capabilities_payload(device, info, capabilities);
        }
        if (read.command == "device") return espectre_device_payload(device, info);
        if (read.command == "read_diagnostics") {
          std::string payload = espectre_diagnostics_payload(device,
                                                             this->runtime_.snapshot(),
                                                             millis(),
                                                             millis() / 1000U,
                                                             0.0f,
                                                             0.0f,
                                                             this->runtime_.diagnostics_sample());
          payload.pop_back();
          append_runtime_csi_quality_diagnostics_json(&payload, this->runtime_.diagnostics());
          payload += "}";
          return payload;
        }
        if (read.command == "health") {
          return espectre_health_payload(device, true, millis());
        }
        if (read.command == "sensing") {
          const RuntimeConfig &config = this->runtime_.config();
          const bool collecting = this->runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION;
          std::string out{"{\"enabled\":"};
          out += this->runtime_.services_armed() ? "true" : "false";
          out += ",\"ready\":";
          out += this->runtime_.snapshot().ready_to_publish && !collecting ? "true" : "false";
          out += ",\"calibrating\":";
          out += this->runtime_.is_calibrating() ? "true" : "false";
          append_json_pair(&out, "mode", collecting ? "csi_collection" : "sensing");
          out += ",\"derived_events_paused\":";
          out += collecting ? "true" : "false";
          out += ",\"threshold\":" + std::to_string(this->runtime_.snapshot().threshold);
          append_json_pair(&out, "detector", detection_algorithm_name(config.detection_algorithm));
          out += ",\"motion_on_hits\":" + std::to_string(config.motion_on_hits);
          out += ",\"motion_off_hits\":" + std::to_string(config.motion_off_hits);
          append_json_pair(&out, "csi_traffic_mode", csi_traffic_mode_name(config.csi_traffic_mode));
          append_json_pair(&out, "traffic_generator_mode", traffic_mode_name(config.traffic_generator_mode));
          out += "}";
          return out;
        }
        return std::string{};
      },
      {},
      [this](float value, std::string *) { return this->runtime_.set_threshold_runtime(value); },
      [this](uint8_t on, uint8_t off, std::string *) { return this->runtime_.set_motion_hits_runtime(on, off); },
      [this](CsiTrafficMode mode, std::string *) { return this->runtime_.set_csi_traffic_mode_runtime(mode); },
      [this](RuntimeTrafficMode mode, std::string *) {
        return this->runtime_.set_traffic_generator_mode_runtime(mode);
      },
      [this](DetectionAlgorithm algorithm, std::string *) {
        return this->runtime_.set_detection_algorithm_runtime(algorithm);
      },
      [this](std::string *) { return this->runtime_.trigger_recalibration(); },
      {},
      {},
      [this](bool enabled, std::string *) {
        this->runtime_.set_services_armed(enabled);
        return true;
      });
  if (result.accepted && result.changes != FrontendCommandChange::NONE) {
    (void) this->direct_bridge_.publish_changes(result.changes);
    this->sync_direct_config_();
    this->threshold_republished_ = true;
    this->detector_republished_ = true;
    this->motion_hits_republished_ = true;
    this->traffic_mode_republished_ = true;
  }
  return result;
}

bool ESpectreComponent::set_threshold_runtime(float threshold) {
  char parameters[64];
  std::snprintf(parameters, sizeof(parameters), "{\"threshold\":%.9g}", static_cast<double>(threshold));
  const FrontendCommandResult result = this->execute_entity_command_("update_sensing", parameters);
  if (!result.accepted) this->sync_direct_config_();
  return result.accepted;
}

bool ESpectreComponent::set_motion_hits_runtime(uint8_t motion_on_hits, uint8_t motion_off_hits) {
  const std::string parameters = "{\"motion_on_hits\":" + std::to_string(motion_on_hits) +
                                 ",\"motion_off_hits\":" + std::to_string(motion_off_hits) + "}";
  const FrontendCommandResult result = this->execute_entity_command_("update_sensing", parameters);
  if (!result.accepted) this->sync_direct_config_();
  return result.accepted;
}

bool ESpectreComponent::set_detection_algorithm_runtime(const std::string &algorithm) {
  std::string parameters{"{"};
  append_json_pair(&parameters, "detector", algorithm.c_str(), true);
  parameters += "}";
  const FrontendCommandResult result = this->execute_entity_command_("update_sensing", parameters);
  if (!result.accepted) this->sync_direct_config_();
  return result.accepted;
}

bool ESpectreComponent::set_sensing_runtime(bool enabled) {
  const FrontendCommandResult result = this->execute_entity_command_(
      "update_sensing", enabled ? "{\"enabled\":true}" : "{\"enabled\":false}");
  if (!result.accepted) this->sync_direct_config_();
  return result.accepted;
}

bool ESpectreComponent::set_csi_traffic_mode_runtime(const std::string &mode) {
  std::string parameters{"{"};
  append_json_pair(&parameters, "csi_traffic_mode", mode.c_str(), true);
  parameters += "}";
  const FrontendCommandResult result = this->execute_entity_command_("update_sensing", parameters);
  if (!result.accepted) this->sync_direct_config_();
  return result.accepted;
}

bool ESpectreComponent::set_traffic_generator_mode_runtime(const std::string &mode) {
  std::string parameters{"{"};
  append_json_pair(&parameters, "traffic_generator_mode", mode.c_str(), true);
  parameters += "}";
  const FrontendCommandResult result = this->execute_entity_command_("update_sensing", parameters);
  if (!result.accepted) this->sync_direct_config_();
  return result.accepted;
}

void ESpectreComponent::trigger_recalibration() {
  (void) this->execute_entity_command_("recalibrate");
}

void ESpectreComponent::on_sensing_readiness_changed(const RuntimeSnapshot &snapshot) {
  (void) snapshot;
  (void) this->direct_bridge_.publish_changes(FrontendCommandChange::SENSING);
}

void ESpectreComponent::on_motion_state_changed(const RuntimeSnapshot &snapshot) {
  if (!snapshot.ready_to_publish) {
    this->threshold_republished_ = false;
    this->detector_republished_ = false;
    this->motion_hits_republished_ = false;
    this->traffic_mode_republished_ = false;
  }
  // Runtime stop and CSI restart events reset motion before sensing is ready.
  (void) this->runtime_events_.post_motion_state(snapshot);
}

void ESpectreComponent::on_periodic_update(const RuntimeSnapshot &snapshot, uint32_t packets_received) {
  (void) packets_received;
  if (!snapshot.ready_to_publish) {
    this->threshold_republished_ = false;
    this->detector_republished_ = false;
    this->motion_hits_republished_ = false;
    this->traffic_mode_republished_ = false;
  }
  if (!snapshot.ready_to_publish) {
    return;
  }

  if (!this->threshold_republished_ && this->threshold_number_ != nullptr) {
    auto *threshold_num = static_cast<ESpectreThresholdNumber *>(this->threshold_number_);
    threshold_num->republish_state();
    this->threshold_republished_ = true;
  }
  if (!this->detector_republished_ && this->detector_select_ != nullptr) {
    static_cast<ESpectreDetectorSelect *>(this->detector_select_)->republish_state();
    this->detector_republished_ = true;
  }
  if (!this->motion_hits_republished_ && (this->motion_on_hits_number_ != nullptr || this->motion_off_hits_number_ != nullptr)) {
    if (this->motion_on_hits_number_ != nullptr) {
      static_cast<ESpectreMotionHitsNumber *>(this->motion_on_hits_number_)->republish_state();
    }
    if (this->motion_off_hits_number_ != nullptr) {
      static_cast<ESpectreMotionHitsNumber *>(this->motion_off_hits_number_)->republish_state();
    }
    this->motion_hits_republished_ = true;
  }
  if (!this->traffic_mode_republished_ &&
      (this->csi_traffic_mode_select_ != nullptr || this->traffic_generator_mode_select_ != nullptr)) {
    if (this->csi_traffic_mode_select_ != nullptr) {
      static_cast<ESpectreTrafficModeSelect *>(this->csi_traffic_mode_select_)->republish_state();
    }
    if (this->traffic_generator_mode_select_ != nullptr) {
      static_cast<ESpectreTrafficModeSelect *>(this->traffic_generator_mode_select_)->republish_state();
    }
    this->traffic_mode_republished_ = true;
  }

}

void ESpectreComponent::on_live_telemetry(float movement, float threshold) {
  RuntimeSnapshot snapshot = this->runtime_.snapshot();
  if (!snapshot.ready_to_publish) {
    return;
  }
  snapshot.movement_metric = movement;
  snapshot.threshold = threshold;
  this->runtime_events_.post_live_telemetry(snapshot);
}

void ESpectreComponent::drain_pending_runtime_events_() {
  RuntimeSnapshot snapshot;
  while (this->runtime_events_.take_motion_state(snapshot)) {
    this->sensor_publisher_.publish_motion_binary(snapshot.motion_state);
  }
  if (this->runtime_events_.take_live_telemetry(snapshot)) {
    this->sensor_publisher_.publish_movement_metric(snapshot.movement_metric);
    (void) this->direct_bridge_.publish_motion(snapshot);
  }
  float threshold = 0.0f;
  if (this->runtime_events_.take_threshold(threshold)) {
    if (this->threshold_number_ != nullptr) {
      this->threshold_number_->publish_state(threshold);
    }
    (void) this->direct_bridge_.publish_changes(FrontendCommandChange::SENSING);
  }
}

void ESpectreComponent::on_threshold_changed(const RuntimeSnapshot &snapshot) {
  this->runtime_events_.post_threshold(snapshot.threshold);
}

#ifdef USE_WIFI_IP_STATE_LISTENERS
void ESpectreComponent::on_ip_state(const network::IPAddresses &ips,
                                    const network::IPAddress &dns1,
                                    const network::IPAddress &dns2) {
  (void) dns1;
  (void) dns2;
  uint32_t ipv4 = 0U;
  for (const network::IPAddress &address : ips) {
    if (!address.is_set() || !address.is_ip4()) continue;
    ipv4 = static_cast<esp_ip4_addr_t>(address).addr;
    break;
  }
  this->pending_mdns_ipv4_ = ipv4;
  this->mdns_ipv4_pending_ = true;
  this->wifi_has_ipv4_ = ipv4 != 0U;
}
#endif

#ifdef USE_WIFI_CONNECT_STATE_LISTENERS
void ESpectreComponent::on_wifi_connect_state(StringRef ssid,
                                              std::span<const uint8_t, 6> bssid) {
  if (ssid.size() == 0U) {
    this->pending_mdns_ipv4_ = 0U;
    this->mdns_ipv4_pending_ = true;
    this->wifi_associated_bssid_.clear();
    this->wifi_has_ipv4_ = false;
    return;
  }
  char associated[18]{};
  std::snprintf(associated,
                sizeof(associated),
                "%02X:%02X:%02X:%02X:%02X:%02X",
                bssid[0],
                bssid[1],
                bssid[2],
                bssid[3],
                bssid[4],
                bssid[5]);
  this->handle_wifi_bssid_association_(associated);
}
#endif

void ESpectreComponent::on_detector_changed(const RuntimeSnapshot &snapshot) {
  if (this->detector_select_ != nullptr) {
    this->detector_select_->publish_state(detection_algorithm_name(this->runtime_.config().detection_algorithm));
  }
  if (this->threshold_number_ != nullptr) {
    static_cast<ESpectreThresholdNumber *>(this->threshold_number_)
        ->update_detector_range(this->runtime_.config().detection_algorithm);
  }
  (void) snapshot;
  (void) this->direct_bridge_.publish_changes(FrontendCommandChange::SENSING);
}

void ESpectreComponent::on_calibration_started(const RuntimeSnapshot &snapshot) {
  (void) snapshot;
  if (this->calibration_active_sensor_ != nullptr) this->calibration_active_sensor_->publish_state(true);
  (void) this->direct_bridge_.publish_changes(FrontendCommandChange::SENSING);
}

void ESpectreComponent::on_calibration_finished(const RuntimeSnapshot &snapshot, bool success) {
  (void) snapshot;
  if (this->calibration_active_sensor_ != nullptr) this->calibration_active_sensor_->publish_state(false);
  (void) success;
  (void) this->direct_bridge_.publish_changes(FrontendCommandChange::SENSING);
  if (!success) {
    ESP_LOGW(TAG, "Calibration finished without a valid update");
  }
}

void ESpectreComponent::on_runtime_fault(const char *message) {
  EspectreDeviceConfig device;
  device.device_id = this->runtime_.config().device_id;
  (void) this->direct_bridge_.publish_event(
      "fault", espectre_fault_payload(device, message, millis()));
}

void ESpectreComponent::dump_config() {
  log_espectre_banner([](const char *line) { ESP_LOGCONFIG(TAG, "%s", line); });
  const RuntimeConfig &config = this->runtime_.config();
  const RuntimeSnapshot &snapshot = this->runtime_.snapshot();
  ESP_LOGCONFIG(TAG, " MOTION DETECTION");
  ESP_LOGCONFIG(TAG, " ├─ Wi-Fi band ......... %s", wifi_band_policy_name(config.wifi_band_policy));
  ESP_LOGCONFIG(TAG, " ├─ Detector ........... %s", snapshot.detector_name);
  ESP_LOGCONFIG(TAG, " ├─ Threshold .......... %.6f", snapshot.threshold);
  ESP_LOGCONFIG(TAG, " ├─ Window ............. %u ms",
                static_cast<unsigned>(config.segmentation_window_size_ms));
  ESP_LOGCONFIG(TAG, " └─ Startup threshold .. %.6f", snapshot.startup_threshold);
  ESP_LOGCONFIG(TAG, " ");
  ESP_LOGCONFIG(TAG, " SUBCARRIERS [%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d]",
                snapshot.fixed_subcarriers[0], snapshot.fixed_subcarriers[1],
                snapshot.fixed_subcarriers[2], snapshot.fixed_subcarriers[3],
                snapshot.fixed_subcarriers[4], snapshot.fixed_subcarriers[5],
                snapshot.fixed_subcarriers[6], snapshot.fixed_subcarriers[7],
                snapshot.fixed_subcarriers[8], snapshot.fixed_subcarriers[9],
                snapshot.fixed_subcarriers[10], snapshot.fixed_subcarriers[11]);
  ESP_LOGCONFIG(TAG, " └─ Source ............. %s", subcarrier_source_name(snapshot.subcarrier_source));
  ESP_LOGCONFIG(TAG, " ");
  ESP_LOGCONFIG(TAG, " TRAFFIC GENERATOR");
  ESP_LOGCONFIG(TAG, " ├─ Mode ............... %s", traffic_mode_name(config.traffic_generator_mode));
  ESP_LOGCONFIG(TAG, " ├─ CSI target ......... %u pps",
                static_cast<unsigned>(config.csi_target_pps));
  ESP_LOGCONFIG(TAG, " ├─ CSI traffic ........ %s", csi_traffic_mode_name(config.csi_traffic_mode));
  ESP_LOGCONFIG(TAG, " ├─ Multicast join ..... %s",
                config.csi_traffic_multicast_group.empty() ? "[disabled]"
                                                          : config.csi_traffic_multicast_group.c_str());
  ESP_LOGCONFIG(TAG, " └─ Status ............. %s", snapshot.ready_to_publish ? "[ACTIVE]" : "[IDLE]");
  ESP_LOGCONFIG(TAG, " ");
  ESP_LOGCONFIG(TAG, " EVALUATION");
  ESP_LOGCONFIG(TAG, " ├─ Interval ........... %u ms",
                static_cast<unsigned>(config.evaluation_interval_ms));
  ESP_LOGCONFIG(TAG, " └─ Hits on/off ........ %u / %u",
                static_cast<unsigned>(config.motion_on_hits),
                static_cast<unsigned>(config.motion_off_hits));
  ESP_LOGCONFIG(TAG, " ");
  ESP_LOGCONFIG(TAG, " LOW-PASS FILTER");
  ESP_LOGCONFIG(TAG, " ├─ Status ............. %s", config.lowpass_enabled ? "[ENABLED]" : "[DISABLED]");
  if (config.lowpass_enabled) {
    ESP_LOGCONFIG(TAG, " └─ Cutoff ............. %.1f Hz", config.lowpass_cutoff);
  }
  ESP_LOGCONFIG(TAG, " ");
  ESP_LOGCONFIG(TAG, " HAMPEL FILTER");
  ESP_LOGCONFIG(TAG, " ├─ Status ............. %s", config.hampel_enabled ? "[ENABLED]" : "[DISABLED]");
  if (config.hampel_enabled) {
    ESP_LOGCONFIG(TAG, " ├─ Window ............. %d pkts", config.hampel_window);
    ESP_LOGCONFIG(TAG, " └─ Threshold .......... %.1f MAD", config.hampel_threshold);
  }
  ESP_LOGCONFIG(TAG, " ");
  ESP_LOGCONFIG(TAG, " SENSORS");
  ESP_LOGCONFIG(TAG, " ├─ Movement ........... %s",
                this->sensor_publisher_.has_movement_sensor() ? "[OK]" : "[--]");
  ESP_LOGCONFIG(TAG, " └─ Motion Binary ...... %s",
                this->sensor_publisher_.has_motion_binary_sensor() ? "[OK]" : "[--]");
  ESP_LOGCONFIG(TAG, " ");
  ESP_LOGCONFIG(TAG, " DIRECT API ............ %s",
                this->direct_api_enabled_ ? "[ENABLED]" : "[DISABLED]");
  ESP_LOGCONFIG(TAG, " ");
}

}  // namespace espectre_component
}  // namespace esphome
