/*
 * ESPectre - Runtime Direct HTTP Bridge
 *
 * Shared Direct HTTP control surface for firmware frontends.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "direct_http_service.h"
#include "direct_wifi_snapshot_esp_idf.h"
#include "frontend_command_engine.h"
#include "peer_discovery.h"
#include "raw_csi_session_controller.h"
#include "runtime_diagnostics.h"
#include "runtime_event_mailbox.h"
#include "runtime_frontend_controller.h"

#include <atomic>

namespace espectre {

struct RuntimeDirectHttpBridgeConfig {
  std::string frontend;
  std::string device_name;
  std::string hostname;
  std::string firmware_version;
  std::string chip;
  uint64_t device_id{0U};
  uint16_t port{ESPECTRE_DIRECT_HTTP_PORT};
  bool raw_csi{false};
  bool allow_missing_origin{false};
  std::function<std::string()> device_label_getter;
  FrontendDeviceLabelCallback device_label_setter;
  std::function<DirectWifiSnapshot()> wifi_snapshot_getter;
  IPeerDiscoveryService *peer_discovery{nullptr};
  std::function<const RuntimeDiagnosticsSample *()> diagnostics_sample_getter;
  const RuntimeEventMailbox *runtime_events{nullptr};
  // Empty BSSID clears the pin. Frontends that provide this callback own the
  // complete live-apply and persistence transaction; frontends that omit it
  // apply the pin to the current station session only.
  std::function<bool(const std::string &bssid, bool force, std::string *message)> wifi_bssid_pin_setter;
  std::function<bool(std::string *message)> wifi_bssid_pin_preflight;
};

/** Apply or clear the ESP-IDF station BSSID pin through a Wi-Fi state-machine restart. */
bool apply_wifi_bssid_pin(const std::string &bssid,
                          std::string *message,
                          bool *station_transition_started = nullptr);

/**
 * Exposes the common runtime controls over the versioned Direct HTTP API.
 *
 * Frontends retain ownership of their runtime and transport. The optional
 * callback lets an adapter republish frontend-native entities after a Direct
 * mutation, for example ESPHome number and select entities.
 */
class RuntimeDirectHttpBridge {
 public:
  using ConfigChangedCallback = std::function<void()>;

  bool setup(IDirectHttpService *service,
             RuntimeFrontendController *runtime,
             const RuntimeDirectHttpBridgeConfig &config,
             ConfigChangedCallback config_changed = {});
  void loop();
  void shutdown();
  bool running() const;
  size_t event_client_count() const;
  bool publish_event(const char *event_name, const std::string &data_json, bool replaceable_telemetry = false);
  bool publish_motion(const RuntimeSnapshot &snapshot);
  bool publish_changes(FrontendCommandChange changes);

 private:
  IDirectHttpService::DeferredRequestResult handle_deferred_request_(uint64_t request_token,
                                                                     const DirectRequest &request);
  std::string handle_request_(const DirectRequest &request);
  EspectreCapabilityProfile capability_profile_() const;
  std::string device_label_() const;
  DirectWifiSnapshot wifi_snapshot_() const;
  std::string capabilities_payload_() const;
  std::string device_payload_() const;
  std::string health_payload_() const;
  std::string sensing_payload_() const;
  std::string wifi_payload_() const;
  std::string diagnostics_payload_() const;
  std::string wifi_access_points_payload_() const;
  bool handle_wifi_control_(const EspectreCommand &command, std::string *message);
  bool handle_raw_stream_(const EspectreCommand &command,
                          const FrontendCommandContext &context,
                          std::string *code,
                          std::string *message,
                          std::string *data_json);
  void refresh_peer_candidate_();
  void notify_config_changed_();

  IDirectHttpService *service_{nullptr};
  RuntimeFrontendController *runtime_{nullptr};
  FrontendCommandEngine command_engine_{};
  RuntimeDirectHttpBridgeConfig config_{};
  ConfigChangedCallback config_changed_{};
  RawCsiSessionController raw_session_controller_{};
  std::atomic<size_t> event_client_count_{0U};
  bool deferred_requests_enabled_{false};
  bool wifi_response_pending_{false};
};

}  // namespace espectre
