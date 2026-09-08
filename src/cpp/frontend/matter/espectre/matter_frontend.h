/*
 * ESPectre - Matter Frontend Adapter
 *
 * Bridges runtime events to the standard Matter occupancy surface.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <functional>
#include <memory>

#include "direct_http_service.h"
#include "matter_bindings.h"
#include "peer_discovery_service_esp_idf.h"
#include "runtime_events.h"
#include "runtime_direct_http_bridge.h"
#include "runtime_diagnostics.h"
#include "runtime_event_mailbox.h"
#include "runtime_frontend_controller.h"

namespace espectre {

class MatterFrontend : public IRuntimeListener {
 public:
  using WifiBssidPinSetter =
      std::function<bool(const std::string &bssid, bool force, std::string *message)>;
  using WifiBssidPinPreflight = std::function<bool(std::string *message)>;

  MatterFrontend(IMatterBindings *bindings,
                 uint16_t endpoint_id,
                 IDirectHttpService *direct_service = nullptr);

  void set_runtime_config(const RuntimeConfig &config);
  void set_wifi_bssid_pin_setter(WifiBssidPinSetter setter);
  void set_wifi_bssid_pin_preflight(WifiBssidPinPreflight preflight);
  bool set_runtime_services_armed(bool armed);
  void prepare_for_wifi_reconfigure();
  void resume_after_wifi_reconfigure();
  const RuntimeConfig &runtime_config() const { return runtime_.config(); }
  bool runtime_services_armed() const { return runtime_.services_armed(); }

  bool setup();
  void loop();
  void shutdown();
  ~MatterFrontend();

  const RuntimeSnapshot &snapshot() const { return runtime_.snapshot(); }
  const RuntimeCapabilities &capabilities() const { return runtime_.capabilities(); }
  bool is_setup_complete() const { return runtime_.is_setup_complete(); }
  void sync_device_label();

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
  bool start_direct_service_();
  void stop_direct_service_();
  void drain_pending_runtime_events_();
  void update_live_telemetry_enabled_();

  IMatterBindings *bindings_;
  uint16_t endpoint_id_;
  RuntimeFrontendController runtime_;
  IDirectHttpService *direct_service_{nullptr};
  RuntimeDirectHttpBridge direct_bridge_;
  EspIdfPeerDiscoveryService peer_discovery_;
  RuntimeEventMailbox runtime_events_{};
  WifiBssidPinSetter wifi_bssid_pin_setter_{};
  WifiBssidPinPreflight wifi_bssid_pin_preflight_{};
  bool live_telemetry_enabled_{true};
  bool operational_services_armed_{true};
  bool wifi_reconfigure_quiesced_{false};
  bool wifi_reconfigure_resume_pending_{false};
  std::string fallback_device_label_{};
};

}  // namespace espectre
