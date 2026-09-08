/*
 * ESPectre - Native Frontend Adapter
 *
 * Composes the Native Direct, MQTT, Home Assistant, and OTA adapters around
 * the shared runtime and command engine.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "frontend_command_engine.h"
#include "native_direct_frontend.h"
#include "frontend/ota_service.h"
#include "runtime_event_mailbox.h"
#include "runtime_frontend_controller.h"

namespace espectre {

class HomeAssistantMqttFrontend;
class IMqttTransport;
class NativeCommandBindings;
class NativeMqttFrontend;

class NativeFrontend : public IRuntimeListener {
 public:
  using WifiProvisioningInfo = NativeWifiProvisioningInfo;
  using ProvisioningCommandCallback = std::function<bool(const std::string &command, std::string *message)>;
  using WifiScanCallback = std::function<bool(std::string *message)>;
  using DeviceConfigChangeCallback =
      std::function<bool(const EspectreDeviceConfig &config, bool clear, std::string *message)>;

  explicit NativeFrontend(IMqttTransport *mqtt_transport = nullptr, IOtaService *ota_service = nullptr,
                          IDirectHttpService *direct_service = nullptr);
  ~NativeFrontend();

  void set_runtime_config(const RuntimeConfig &config);
  void set_device_config(const EspectreDeviceConfig &config);
  void set_device_info(const EspectreDeviceInfo &info);
  void set_peer_discovery_service(IPeerDiscoveryService *service);
  void set_wifi_provisioning_info(const WifiProvisioningInfo &info);
  void set_provisioning_command_callback(ProvisioningCommandCallback callback);
  void set_wifi_scan_callback(WifiScanCallback callback);
  void set_device_config_change_callback(DeviceConfigChangeCallback callback);
  void prepare_for_wifi_reconfigure();
  void resume_after_wifi_reconfigure();

  const EspectreDeviceConfig &device_config() const { return device_config_; }
  const RuntimeConfig &runtime_config() const { return runtime_.config(); }
  const RuntimeSnapshot &snapshot() const { return runtime_.snapshot(); }
  const RuntimeCapabilities &capabilities() const { return runtime_.capabilities(); }
  bool is_setup_complete() const { return runtime_.is_setup_complete(); }
  size_t direct_client_count() const;

  bool setup();
  void loop();
  void shutdown();

 protected:
  void on_motion_state_changed(const RuntimeSnapshot &snapshot) override;
  void on_sensing_readiness_changed(const RuntimeSnapshot &snapshot) override;
  void on_periodic_update(const RuntimeSnapshot &snapshot, uint32_t packets_received) override;
  void on_threshold_changed(const RuntimeSnapshot &snapshot) override;
  void on_detector_changed(const RuntimeSnapshot &snapshot) override;
  void on_calibration_started(const RuntimeSnapshot &snapshot) override;
  void on_calibration_finished(const RuntimeSnapshot &snapshot, bool success) override;
  void on_live_telemetry(float movement, float threshold) override;
  void on_runtime_fault(const char *message) override;

 private:
  friend class HomeAssistantMqttFrontend;
  friend class NativeCommandBindings;
  friend class NativeDirectFrontend;
  friend class NativeMqttFrontend;

  FrontendCommandResult dispatch_command_(const EspectreCommand &command, FrontendCommandOrigin origin,
                                          bool allow_local_config, uint64_t connection_token = 0U);
  EspectreCapabilityProfile command_capability_profile_(bool allow_local_config) const;
  bool handle_threshold_write_(float threshold);
  bool handle_motion_hits_write_(uint8_t motion_on_hits, uint8_t motion_off_hits);
  bool handle_csi_traffic_mode_write_(CsiTrafficMode mode);
  bool handle_traffic_generator_mode_write_(RuntimeTrafficMode mode);
  bool handle_detector_write_(DetectionAlgorithm algorithm);
  bool handle_recalibration_write_();
  void drain_pending_runtime_events_();
  void update_live_telemetry_enabled_();
  void fan_out_payload_(const char *mqtt_suffix, const char *direct_event_name, const std::string &payload,
                        bool mqtt_retain = false, bool replaceable_telemetry = false);
  void publish_runtime_config_state_();
  void publish_runtime_status_state_();
  EspectreDeviceInfo mqtt_protocol_device_info_() const;
  EspectreOtaStatus current_ota_status_() const;
  void publish_ota_status_(const EspectreOtaStatus &status);
  void prepare_for_ota_();
  void resume_after_ota_error_();
  uint32_t now_ms_() const;

  IOtaService *ota_service_{nullptr};
  ProvisioningCommandCallback provisioning_command_callback_{};
  WifiScanCallback wifi_scan_callback_{};
  DeviceConfigChangeCallback device_config_change_callback_{};
  RuntimeFrontendController runtime_;
  EspectreDeviceConfig device_config_{};
  EspectreDeviceInfo device_info_{};
  RuntimeEventMailbox runtime_events_{};
  std::unique_ptr<NativeCommandBindings> command_bindings_;
  std::unique_ptr<NativeDirectFrontend> direct_frontend_;
  std::unique_ptr<NativeMqttFrontend> mqtt_frontend_;
  bool ota_frontend_quiesced_{false};
  bool ota_services_were_armed_{false};
  bool wifi_reconfigure_quiesced_{false};
  bool wifi_reconfigure_resume_pending_{false};
  bool protocol_recalibration_command_active_{false};
  bool calibration_started_{false};
  float calibration_start_threshold_{0.0f};
  float last_loop_time_ms_{0.0f};
};

}  // namespace espectre
