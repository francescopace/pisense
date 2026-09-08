/*
 * ESPectre - Main Component
 *
 * Main ESPHome component that orchestrates all ESPectre subsystems.
 * Integrates CSI processing, calibration, and Home Assistant publishing.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#if defined(USE_WIFI) || defined(USE_WIFI_IP_STATE_LISTENERS) || defined(USE_WIFI_CONNECT_STATE_LISTENERS)
#include "esphome/components/wifi/wifi_component.h"
#endif

#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include "sensor_publisher.h"
#include "direct_http_service_esp_idf.h"
#include "frontend_command_engine.h"
#include "mdns_discovery_service.h"
#include "mdns_bootstrap_responder.h"
#include "peer_discovery_service_esp_idf.h"
#include "runtime_config_utils.h"
#include "runtime_direct_http_bridge.h"
#include "runtime_diagnostics.h"
#include "runtime_event_mailbox.h"
#include "runtime_events.h"
#include "runtime_frontend_controller.h"
#include "sdkconfig.h"

namespace esphome {
namespace espectre_component {

using namespace ::espectre;

static const char *const TAG = "espectre";

class ESpectreComponent : public Component, public IRuntimeListener
#ifdef USE_WIFI_IP_STATE_LISTENERS
    , public wifi::WiFiIPStateListener
#endif
#ifdef USE_WIFI_CONNECT_STATE_LISTENERS
    , public wifi::WiFiConnectStateListener
#endif
{
  friend class ESpectreDetectorSelect;
  friend class ESpectreTrafficModeSelect;
 public:
  ESpectreComponent() {
    // ESPHome always generates the detector select entity, so this frontend
    // always wants the NVS-backed runtime detector store behind it.
    this->runtime_.config().runtime_detector_selection_enabled = true;
  }

  void setup() override;
  void loop() override;
  ~ESpectreComponent();
  void dump_config() override;
  // Register STA_START after ESPHome creates the event loop, but before its
  // Wi-Fi component starts scanning and associating.
  float get_setup_priority() const override { return 275.0f; }

  // Setters for YAML configuration
  void set_direct_api(bool enabled) { this->direct_api_enabled_ = enabled; }
  void set_segmentation_window_size_ms(uint32_t size_ms) {
    this->runtime_.config().segmentation_window_size_ms = size_ms;
  }
  void set_wifi_band_policy(const std::string &policy) {
    this->runtime_.config().wifi_band_policy = parse_wifi_band_policy(policy.c_str());
  }
  void set_csi_target_pps(uint32_t target_pps) { this->runtime_.config().csi_target_pps = target_pps; }
  void set_csi_traffic_mode(const std::string &mode) {
    this->runtime_.config().csi_traffic_mode = parse_csi_traffic_mode(mode.c_str());
  }
  void set_csi_traffic_multicast_group(const std::string &group) {
    this->runtime_.config().csi_traffic_multicast_group = group;
  }
  void set_traffic_generator_mode(const std::string &mode) { 
    this->runtime_.config().traffic_generator_mode = parse_traffic_mode(mode.c_str());
  }
  // Picking the startup algorithm and enabling runtime selection are separate
  // concerns; only the first is a YAML choice. ESPHome always exposes the
  // detector select entity and the NVS-backed store behind it, so the second is
  // declared once in the constructor rather than as a side effect here.
  void set_detection_algorithm(const std::string &algo) {
    this->runtime_.config().detection_algorithm = parse_detection_algorithm(algo.c_str());
  }
  void set_evaluation_interval_ms(uint32_t interval_ms) {
    this->runtime_.config().evaluation_interval_ms = interval_ms;
  }
  void set_motion_on_hits(uint8_t hits) { this->runtime_.config().motion_on_hits = hits; }
  void set_motion_off_hits(uint8_t hits) { this->runtime_.config().motion_off_hits = hits; }
  void set_lowpass_enabled(bool enabled) { this->runtime_.config().lowpass_enabled = enabled; }
  void set_lowpass_cutoff(float cutoff) { this->runtime_.config().lowpass_cutoff = cutoff; }
  void set_hampel_enabled(bool enabled) { this->runtime_.config().hampel_enabled = enabled; }
  void set_hampel_window(uint8_t window) { this->runtime_.config().hampel_window = window; }
  void set_hampel_threshold(float threshold) { this->runtime_.config().hampel_threshold = threshold; }
  
  // Setters for ESPHome sensors (delegated to SensorPublisher)
  void set_movement_sensor(sensor::Sensor *sensor) { this->sensor_publisher_.set_movement_sensor(sensor); }
  void set_motion_binary_sensor(binary_sensor::BinarySensor *sensor) { this->sensor_publisher_.set_motion_binary_sensor(sensor); }
  void set_calibration_active_sensor(binary_sensor::BinarySensor *sensor) { this->calibration_active_sensor_ = sensor; }
  void set_traffic_rate_sensor(sensor::Sensor *sensor) { this->traffic_rate_sensor_ = sensor; }
  void set_csi_callback_rate_sensor(sensor::Sensor *sensor) { this->csi_callback_rate_sensor_ = sensor; }
  void set_csi_accepted_rate_sensor(sensor::Sensor *sensor) { this->csi_accepted_rate_sensor_ = sensor; }
  void set_csi_admitted_rate_sensor(sensor::Sensor *sensor) { this->csi_admitted_rate_sensor_ = sensor; }
  void set_csi_filtered_rate_sensor(sensor::Sensor *sensor) { this->csi_filtered_rate_sensor_ = sensor; }
  void set_csi_missing_rate_sensor(sensor::Sensor *sensor) { this->csi_missing_rate_sensor_ = sensor; }
  void set_csi_excess_rate_sensor(sensor::Sensor *sensor) { this->csi_excess_rate_sensor_ = sensor; }
  void set_csi_stale_rate_sensor(sensor::Sensor *sensor) { this->csi_stale_rate_sensor_ = sensor; }
  void set_csi_out_of_order_rate_sensor(sensor::Sensor *sensor) { this->csi_out_of_order_rate_sensor_ = sensor; }
  void set_csi_occupancy_sensor(sensor::Sensor *sensor) { this->csi_occupancy_sensor_ = sensor; }
  void set_wifi_channel_sensor(sensor::Sensor *sensor) { this->wifi_channel_sensor_ = sensor; }
  void set_wifi_rssi_sensor(sensor::Sensor *sensor) { this->wifi_rssi_sensor_ = sensor; }
  
  // Setter for threshold number control
  void set_threshold_number(number::Number *num) { this->threshold_number_ = num; }
  void set_motion_on_hits_number(number::Number *num) { this->motion_on_hits_number_ = num; }
  void set_motion_off_hits_number(number::Number *num) { this->motion_off_hits_number_ = num; }
  
  // Runtime threshold adjustment (called from HA via number component)
  bool set_threshold_runtime(float threshold);
  bool set_motion_hits_runtime(uint8_t motion_on_hits, uint8_t motion_off_hits);
  bool set_detection_algorithm_runtime(const std::string &algorithm);
  bool set_sensing_runtime(bool enabled);
  bool is_sensing_enabled() const { return this->runtime_.services_armed(); }
  float get_threshold() const { return this->runtime_.snapshot().threshold; }
  uint8_t get_motion_on_hits() const { return this->runtime_.config().motion_on_hits; }
  uint8_t get_motion_off_hits() const { return this->runtime_.config().motion_off_hits; }
  
  // Runtime calibration trigger (called from HA via button component)
  void trigger_recalibration();
  void publish_diagnostics_on_demand();
  
  // Check if calibration is in progress
  bool is_calibrating() const { return this->runtime_.is_calibrating(); }
  
  void set_sensing_switch(switch_::Switch *value) { this->sensing_switch_ = value; }
  void set_detector_select(select::Select *value) { this->detector_select_ = value; }
  void set_csi_traffic_mode_select(select::Select *value) { this->csi_traffic_mode_select_ = value; }
  void set_traffic_generator_mode_select(select::Select *value) { this->traffic_generator_mode_select_ = value; }
  bool set_csi_traffic_mode_runtime(const std::string &mode);
  bool set_traffic_generator_mode_runtime(const std::string &mode);
  
 protected:
  void on_motion_state_changed(const RuntimeSnapshot &snapshot) override;
  void on_sensing_readiness_changed(const RuntimeSnapshot &snapshot) override;
  void on_periodic_update(const RuntimeSnapshot &snapshot, uint32_t packets_received) override;
  void on_live_telemetry(float movement, float threshold) override;
  void on_threshold_changed(const RuntimeSnapshot &snapshot) override;
  void on_detector_changed(const RuntimeSnapshot &snapshot) override;
  void on_calibration_started(const RuntimeSnapshot &snapshot) override;
  void on_calibration_finished(const RuntimeSnapshot &snapshot, bool success) override;
  void on_runtime_fault(const char *message) override;
#ifdef USE_WIFI_IP_STATE_LISTENERS
  void on_ip_state(const network::IPAddresses &ips,
                   const network::IPAddress &dns1,
                   const network::IPAddress &dns2) override;
#endif
#ifdef USE_WIFI_CONNECT_STATE_LISTENERS
  void on_wifi_connect_state(StringRef ssid, std::span<const uint8_t, 6> bssid) override;
#endif
  void publish_cached_diagnostics_();
  void drain_pending_runtime_events_();
  void update_live_telemetry_enabled_();
  FrontendCommandResult execute_entity_command_(const std::string &name, const std::string &parameters = "{}");
  void sync_direct_config_();
  void setup_mdns_discovery_();
  std::string device_name_() const;
  const std::string &device_label_() const;
  std::string display_name_() const;
  std::string mdns_instance_name_() const;
  MdnsTxtRecords mdns_txt_records_() const;
  bool set_device_label_(const std::string &device_label, std::string *message);
  bool apply_esphome_wifi_bssid_pin_(const std::string &bssid, std::string *message);
  bool begin_wifi_bssid_pin_update_(const std::string &bssid, bool force, std::string *message);
  bool persist_wifi_bssid_pin_(const std::string &bssid, std::string *message);
  bool stage_wifi_bssid_pin_(const std::string &bssid, std::string *message);
  void handle_wifi_bssid_association_(const std::string &associated_bssid);
  void process_wifi_bssid_apply_();
  void finish_wifi_bssid_apply_();
  void fail_wifi_bssid_apply_(const char *reason);

  RuntimeFrontendController runtime_;
  FrontendCommandEngine command_engine_;
  // Keep the bridge alive while the service drains deferred callbacks during
  // destruction. The component destructor disconnects the bridge explicitly
  // before reverse member destruction begins.
  RuntimeDirectHttpBridge direct_bridge_;
  EspIdfDirectHttpService direct_service_;
  MdnsDiscoveryService mdns_discovery_;
  MdnsBootstrapResponder mdns_bootstrap_responder_;
  EspIdfPeerDiscoveryService peer_discovery_;
  struct StoredDeviceLabel {
    uint8_t version{1U};
    std::array<char, ESPECTRE_DEVICE_LABEL_MAX_LENGTH + 1U> value{};
  };
  ESPPreferenceObject device_label_preference_;
  std::string device_label_override_;
  struct StoredWifiBssid {
    uint8_t version{2U};
    uint8_t pinned{0U};
    uint8_t pending{0U};
    uint8_t reserved{0U};
    std::array<char, 18> value{};
    std::array<char, 18> pending_value{};
  };
  ESPPreferenceObject wifi_bssid_preference_;
  std::string wifi_bssid_pin_;
  enum class WifiBssidApplyMode : uint8_t {
    NONE = 0U,
    UPDATE,
    ENFORCE,
  };
  WifiBssidApplyMode wifi_bssid_apply_mode_{WifiBssidApplyMode::NONE};
  std::string wifi_bssid_apply_target_;
  std::string wifi_bssid_apply_previous_pin_;
  std::string wifi_associated_bssid_;
  bool wifi_has_ipv4_{false};
  uint32_t wifi_bssid_apply_started_ms_{0U};
  bool wifi_bssid_apply_resume_sensing_{false};
  bool wifi_bssid_apply_transition_started_{false};
  bool wifi_bssid_pending_loaded_{false};
  std::string wifi_bssid_pending_target_;
  bool wifi_bssid_recovery_pending_{false};
  bool wifi_bssid_recovery_journal_pending_{false};
  bool wifi_bssid_recovery_resume_sensing_{false};
  std::string wifi_bssid_recovery_target_;
  uint32_t wifi_bssid_recovery_started_ms_{0U};

  SensorPublisher sensor_publisher_;

  // Number controls
  number::Number *threshold_number_{nullptr};
  number::Number *motion_on_hits_number_{nullptr};
  number::Number *motion_off_hits_number_{nullptr};
  
  switch_::Switch *sensing_switch_{nullptr};
  select::Select *detector_select_{nullptr};
  select::Select *csi_traffic_mode_select_{nullptr};
  select::Select *traffic_generator_mode_select_{nullptr};

  sensor::Sensor *traffic_rate_sensor_{nullptr};
  sensor::Sensor *csi_callback_rate_sensor_{nullptr};
  sensor::Sensor *csi_accepted_rate_sensor_{nullptr};
  sensor::Sensor *csi_admitted_rate_sensor_{nullptr};
  sensor::Sensor *csi_filtered_rate_sensor_{nullptr};
  sensor::Sensor *csi_missing_rate_sensor_{nullptr};
  sensor::Sensor *csi_excess_rate_sensor_{nullptr};
  sensor::Sensor *csi_stale_rate_sensor_{nullptr};
  sensor::Sensor *csi_out_of_order_rate_sensor_{nullptr};
  sensor::Sensor *csi_occupancy_sensor_{nullptr};
  sensor::Sensor *wifi_channel_sensor_{nullptr};
  sensor::Sensor *wifi_rssi_sensor_{nullptr};
  binary_sensor::BinarySensor *calibration_active_sensor_{nullptr};

  RuntimeEventMailbox runtime_events_{};
  bool direct_api_enabled_{true};
  bool live_telemetry_enabled_{true};

  bool threshold_republished_{false};
  bool detector_republished_{false};
  bool motion_hits_republished_{false};
  bool traffic_mode_republished_{false};
  uint32_t pending_mdns_ipv4_{0U};
  uint32_t next_mdns_setup_ms_{0U};
  bool mdns_ipv4_pending_{false};
};

}  // namespace espectre_component
}  // namespace esphome
