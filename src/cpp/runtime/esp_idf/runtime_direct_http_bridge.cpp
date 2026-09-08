/*
 * ESPectre - Runtime Direct HTTP Bridge
 *
 * Shared Direct HTTP control surface for firmware frontends.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "runtime_direct_http_bridge.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>

#include "direct_http_protocol.h"
#include "espectre_protocol.h"
#include "protocol_json.h"
#include "runtime_config_utils.h"
#include "runtime_diagnostics.h"
#include "runtime_time.h"
#include "wifi_lifecycle.h"

#if defined(ESP_PLATFORM) || defined(ESPECTRE_HOST_WIFI_CONTROL_TEST)
#include "esp_wifi.h"
#endif

namespace espectre {

namespace {

std::string wifi_bssid_ack_data(const std::string &current_bssid) {
  std::string data{"{"};
  append_json_pair(&data, "current_bssid", current_bssid.c_str(), true);
  data += "}";
  return data;
}

void append_bool(std::string *out, const char *key, bool value, bool first = false) {
  if (out == nullptr || key == nullptr) {
    return;
  }
  *out += first ? "\"" : ",\"";
  *out += key;
  *out += "\":";
  *out += value ? "true" : "false";
}

void append_uint(std::string *out, const char *key, uint64_t value, bool first = false) {
  if (out == nullptr || key == nullptr) {
    return;
  }
  *out += first ? "\"" : ",\"";
  *out += key;
  *out += "\":" + std::to_string(value);
}

void append_float(std::string *out, const char *key, float value) {
  if (out == nullptr || key == nullptr) {
    return;
  }
  char text[32];
  std::snprintf(text, sizeof(text), "%.6g", static_cast<double>(value));
  *out += ",\"";
  *out += key;
  *out += "\":";
  *out += text;
}

}  // namespace

bool RuntimeDirectHttpBridge::setup(IDirectHttpService *service,
                                         RuntimeFrontendController *runtime,
                                         const RuntimeDirectHttpBridgeConfig &config,
                                         ConfigChangedCallback config_changed) {
  shutdown();
  if (service == nullptr || runtime == nullptr || !runtime->is_setup_complete() || config.frontend.empty() ||
      config.device_id == 0U || config.port == 0U) {
    return false;
  }
  service_ = service;
  runtime_ = runtime;
  config_ = config;
  config_changed_ = std::move(config_changed);
  refresh_peer_candidate_();

  DirectHttpServiceConfig service_config = DirectHttpServiceConfig::for_first_party_portals();
  service_config.device_id = config.device_id;
  service_config.port = config.port;
  service_config.allow_missing_origin = config.allow_missing_origin;
#if defined(CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED) && CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED
  service_config.allow_http_loopback_origins = true;
#endif
  deferred_requests_enabled_ = false;
  const IDirectHttpService::ClientCountCallback client_count_changed =
      [this](size_t client_count) {
        this->event_client_count_.store(client_count, std::memory_order_relaxed);
      };
  bool setup = service_->setup_deferred(
      service_config,
      [this](uint64_t token, const DirectRequest &request) {
        return this->handle_deferred_request_(token, request);
      },
      client_count_changed);
  deferred_requests_enabled_ = setup;
  if (!setup) {
    setup = service_->setup(
        service_config,
        [this](const DirectRequest &request) { return this->handle_request_(request); },
        client_count_changed);
  }
  if (!setup) {
    service_ = nullptr;
    runtime_ = nullptr;
    config_changed_ = {};
    event_client_count_.store(0U, std::memory_order_relaxed);
    deferred_requests_enabled_ = false;
    return false;
  }
  raw_session_controller_.configure(
      service_, runtime_, config_.device_id, config_.chip,
      [this](RawCsiStopReason) { (void) this->publish_event("sensing", this->sensing_payload_()); },
      [this]() { (void) this->publish_event("sensing", this->sensing_payload_()); });
  return true;
}

void RuntimeDirectHttpBridge::loop() {
  if (service_ == nullptr) return;
  if (service_->running()) {
    raw_session_controller_.ensure_runtime_consistency();
    if (config_.peer_discovery != nullptr && config_.peer_discovery->active()) {
      // Refresh link readiness only while discovery is active. Even the cheap
      // netif query may contend with the Wi-Fi task on single-core targets and
      // must not run in every frontend loop.
      config_.peer_discovery->set_wifi_ready(read_direct_wifi_connected());
      config_.peer_discovery->loop();
    }
  }
  service_->loop();
}

void RuntimeDirectHttpBridge::shutdown() {
  raw_session_controller_.shutdown();
  if (config_.peer_discovery != nullptr) config_.peer_discovery->shutdown();
  if (service_ != nullptr) {
    service_->shutdown();
  }
  service_ = nullptr;
  runtime_ = nullptr;
  config_changed_ = {};
  event_client_count_.store(0U, std::memory_order_relaxed);
  deferred_requests_enabled_ = false;
  wifi_response_pending_ = false;
}

bool RuntimeDirectHttpBridge::running() const { return service_ != nullptr && service_->running(); }

size_t RuntimeDirectHttpBridge::event_client_count() const {
  return event_client_count_.load(std::memory_order_relaxed);
}

bool RuntimeDirectHttpBridge::publish_event(const char *event_name,
                                                 const std::string &data_json,
                                                 bool replaceable_telemetry) {
  return event_name != nullptr && running() && service_->publish_event(event_name, data_json, replaceable_telemetry);
}

bool RuntimeDirectHttpBridge::publish_motion(const RuntimeSnapshot &snapshot) {
  if (event_client_count() == 0U) {
    return false;
  }
  EspectreDeviceConfig device;
  device.device_id = config_.device_id;
  return publish_event("motion",
                       espectre_motion_payload(device,
                                                  snapshot,
                                                  monotonic_now_ms(),
                                                  monotonic_now_ms() / 1000U,
                                                  config_.frontend.c_str()),
                       true);
}

bool RuntimeDirectHttpBridge::publish_changes(FrontendCommandChange changes) {
  if (!running() || runtime_ == nullptr) {
    return false;
  }
  bool published = true;
  const uint8_t flags = static_cast<uint8_t>(changes);
  if ((flags & static_cast<uint8_t>(FrontendCommandChange::HEALTH)) != 0U) {
    published = publish_event("health", health_payload_()) && published;
  }
  if ((flags & static_cast<uint8_t>(FrontendCommandChange::DEVICE)) != 0U) {
    refresh_peer_candidate_();
    published = publish_event("device", device_payload_()) && published;
  }
  if ((flags & static_cast<uint8_t>(FrontendCommandChange::SENSING)) != 0U) {
    published = publish_event("sensing", sensing_payload_()) && published;
  }
  if ((flags & static_cast<uint8_t>(FrontendCommandChange::WIFI)) != 0U) {
    published = publish_event("wifi", wifi_payload_()) && published;
  }
  return published;
}

std::string RuntimeDirectHttpBridge::handle_request_(const DirectRequest &request) {
  EspectreDeviceConfig device;
  device.device_id = config_.device_id;
  EspectreCommand command;
  std::string parse_error;
  if (!direct_http_request_to_command(request, &command, &parse_error)) {
    command.command_id = request.command_id;
    command.command = request.command;
    return espectre_command_result_payload(
        device, command, false, frontend_command_parse_error_code(parse_error), parse_error.c_str());
  }
  if (runtime_->operation_state() == RuntimeOperationState::RAW_COLLECTION &&
      !frontend_command_allowed_during_raw_collection(command.command)) {
    return espectre_command_result_payload(device,
                                           command,
                                           false,
                                           "busy_raw_collection",
                                           "mutation is unavailable during raw CSI collection");
  }
  const FrontendCommandCapabilities capabilities = capability_profile_();
  FrontendCommandResult result = command_engine_.execute(
      command,
      FrontendCommandContext{FrontendCommandOrigin::DIRECT, 0U},
      capabilities,
      [this](const EspectreCommand &read) {
        if (read.command == "capabilities") return capabilities_payload_();
        if (read.command == "device") return device_payload_();
        if (read.command == "health") return health_payload_();
        if (read.command == "sensing") return sensing_payload_();
        if (read.command == "wifi") return wifi_payload_();
        if (read.command == "read_diagnostics") return diagnostics_payload_();
        if (read.command == "wifi_access_points") return wifi_access_points_payload_();
        return std::string{};
      },
      config_.device_label_setter,
      [this](float value, std::string *) { return runtime_->set_threshold_runtime(value); },
      [this](uint8_t on, uint8_t off, std::string *) { return runtime_->set_motion_hits_runtime(on, off); },
      [this](CsiTrafficMode mode, std::string *) { return runtime_->set_csi_traffic_mode_runtime(mode); },
      [this](RuntimeTrafficMode mode, std::string *) { return runtime_->set_traffic_generator_mode_runtime(mode); },
      [this](DetectionAlgorithm algorithm, std::string *) {
        return runtime_->set_detection_algorithm_runtime(algorithm);
      },
      [this](std::string *) { return runtime_->trigger_recalibration(); },
      [this](const EspectreCommand &wifi, std::string *message) {
        return handle_wifi_control_(wifi, message);
      },
      {},
      [this](bool enabled, std::string *message) {
        runtime_->set_services_armed(enabled);
        const bool applied = runtime_->services_armed() == enabled;
        if (!applied && message != nullptr) {
          *message = "sensing state could not be changed";
        }
        return applied;
      },
      [this](const EspectreCommand &raw,
             const FrontendCommandContext &context,
             std::string *code,
             std::string *message,
             std::string *data_json) {
        return handle_raw_stream_(raw, context, code, message, data_json);
      });
  if (!result.accepted) {
    return espectre_command_result_payload(device,
                                           result.command,
                                           false,
                                           result.code.c_str(),
                                           result.message.c_str(),
                                           result.data_json);
  }
  if (result.changes != FrontendCommandChange::NONE) {
    (void) publish_changes(result.changes);
    notify_config_changed_();
  }
  if (request.http_method == "GET") {
    return result.data_json;
  }
  return espectre_command_result_payload(device,
                                         result.command,
                                         true,
                                         result.code.c_str(),
                                         result.message.c_str(),
                                         result.data_json);
}

IDirectHttpService::DeferredRequestResult RuntimeDirectHttpBridge::handle_deferred_request_(
    uint64_t request_token,
    const DirectRequest &request) {
  if (request.command == "set_wifi_bssid" || request.command == "clear_wifi_bssid") {
    EspectreDeviceConfig device;
    device.device_id = config_.device_id;
    EspectreCommand command;
    std::string parse_error;
    if (!direct_http_request_to_command(request, &command, &parse_error)) {
      command.command_id = request.command_id;
      command.command = request.command;
      return {false,
              espectre_command_result_payload(
                  device, command, false,
                  frontend_command_parse_error_code(parse_error), parse_error.c_str()),
              {}};
    }
    if (wifi_response_pending_) {
      return {false,
              espectre_command_result_payload(
                  device, command, false, "unavailable",
                  "Wi-Fi BSSID update already in progress"),
              {}};
    }
    std::string preflight_message;
    if (config_.wifi_bssid_pin_preflight &&
        !config_.wifi_bssid_pin_preflight(&preflight_message)) {
      return {false,
              espectre_command_result_payload(
                  device, command, false, "unavailable",
                  preflight_message.empty()
                      ? "Wi-Fi BSSID update is unavailable"
                      : preflight_message.c_str()),
              {}};
    }
    const DirectWifiSnapshot wifi = wifi_snapshot_();
    wifi_response_pending_ = true;
    const char *message = request.command == "set_wifi_bssid"
                              ? "Wi-Fi BSSID update accepted"
                              : "Wi-Fi BSSID clear accepted";
    return {
        false,
        espectre_command_result_payload(
            device, command, true, "ok", message, wifi_bssid_ack_data(wifi.bssid)),
        [this, request](bool sent) {
          this->wifi_response_pending_ = false;
          if (sent) (void) this->handle_request_(request);
        },
    };
  }
  if (request.command != "devices") {
    return {false, handle_request_(request), {}};
  }
  EspectreDeviceConfig device;
  device.device_id = config_.device_id;
  EspectreCommand command;
  command.command_id = request.command_id;
  command.command = request.command;
  if (!deferred_requests_enabled_ || config_.peer_discovery == nullptr) {
    return {false, espectre_command_result_payload(device, command, false, "unsupported",
                                                   "peer discovery is unavailable"),
            {}};
  }
  std::vector<JsonObjectField> params;
  if (!parse_json_object_fields(request.params, &params) || !params.empty()) {
    return {false, espectre_command_result_payload(device, command, false, "invalid_params",
                                                   "devices does not accept parameters"),
            {}};
  }
  if (config_.peer_discovery->active()) {
    return {false, espectre_command_result_payload(device, command, false, "conflict",
                                                   "a peer discovery request is already active"),
            {}};
  }
  config_.peer_discovery->set_wifi_ready(read_direct_wifi_connected());
  const bool started = config_.peer_discovery->start(
      [this, request_token, request_id = request.command_id, command_name = request.command](PeerDiscoverySnapshot snapshot) {
        if (service_ == nullptr) return;
        EspectreDeviceConfig response_device;
        response_device.device_id = config_.device_id;
        EspectreCommand response_command;
        response_command.command_id = request_id;
        response_command.command = command_name;
        (void) service_->complete_deferred_response(
            request_token,
            peer_discovery_snapshot_json(snapshot));
      });
  if (!started) {
    return {false, espectre_command_result_payload(device, command, false, "unavailable",
                                                   "peer discovery could not be started"),
            {}};
  }
  return {true, {}, {}};
}

EspectreCapabilityProfile RuntimeDirectHttpBridge::capability_profile_() const {
  EspectreCapabilityProfile profile;
  if (runtime_ == nullptr) return profile;
  const RuntimeCapabilities &runtime_capabilities = runtime_->capabilities();
  using Method = EspectreDirectMethod;
  profile.set(Method::CAPABILITIES);
  profile.set(Method::INFO);
  profile.set(Method::STATUS);
  profile.set(Method::CONFIG);
  profile.set(Method::DIAGNOSTICS, runtime_capabilities.supports_extended_diagnostics);
  profile.set(Method::SET_SENSING);
  profile.set(Method::SET_DEVICE_LABEL, static_cast<bool>(config_.device_label_setter));
  profile.set(Method::SET_THRESHOLD, runtime_capabilities.supports_runtime_threshold_updates);
  profile.set(Method::SET_MOTION_HITS, runtime_capabilities.supports_runtime_motion_hits_updates);
  profile.set(Method::SET_DETECTOR, runtime_capabilities.supports_runtime_detector_selection);
  profile.set(Method::RECALIBRATE, runtime_capabilities.supports_manual_recalibration);
  profile.set(Method::SET_CSI_TRAFFIC_MODE, runtime_capabilities.supports_traffic_control);
  profile.set(Method::SET_TRAFFIC_GENERATOR_MODE, runtime_capabilities.supports_traffic_control);
  profile.set(Method::WIFI_ACCESS_POINTS);
  profile.set(Method::SCAN_WIFI_ACCESS_POINTS);
  profile.set(Method::SET_WIFI_BSSID);
  profile.set(Method::CLEAR_WIFI_BSSID);
  const bool raw_csi = config_.raw_csi && runtime_capabilities.supports_raw_csi;
  profile.set(Method::START_RAW_STREAM, raw_csi);
  profile.set(Method::STOP_RAW_STREAM, raw_csi);
  profile.set(Method::DISCOVER_PEERS,
              deferred_requests_enabled_ && config_.peer_discovery != nullptr);
  profile.set(EspectreConfigSection::RUNTIME);
  profile.set(EspectreConfigSection::DEVICE);
  profile.set(EspectreConfigSection::WIFI);
  return profile;
}

std::string RuntimeDirectHttpBridge::device_label_() const {
  return config_.device_label_getter ? config_.device_label_getter() : config_.device_name;
}

DirectWifiSnapshot RuntimeDirectHttpBridge::wifi_snapshot_() const {
  return config_.wifi_snapshot_getter ? config_.wifi_snapshot_getter() : read_direct_wifi_snapshot();
}

std::string RuntimeDirectHttpBridge::wifi_access_points_payload_() const {
  std::string out{"{\"scanning\":false,\"message\":\"\",\"access_points\":["};
#if defined(ESP_PLATFORM)
  const std::string configured_ssid = wifi_snapshot_().ssid;
  uint16_t count = 0U;
  const esp_err_t count_result = esp_wifi_scan_get_ap_num(&count);
  if (!configured_ssid.empty() && count_result == ESP_OK && count > 0U) {
    count = std::min<uint16_t>(count, 32U);
    std::vector<wifi_ap_record_t> records(count);
    if (esp_wifi_scan_get_ap_records(&count, records.data()) == ESP_OK) {
      bool first = true;
      for (uint16_t index = 0U; index < count; ++index) {
        const uint8_t *ssid_begin = records[index].ssid;
        const uint8_t *ssid_end =
            std::find(ssid_begin, ssid_begin + sizeof(records[index].ssid), static_cast<uint8_t>(0U));
        const size_t ssid_length = static_cast<size_t>(ssid_end - ssid_begin);
        if (ssid_length != configured_ssid.size() ||
            std::memcmp(ssid_begin, configured_ssid.data(), ssid_length) != 0) {
          continue;
        }
        if (!first) out += ',';
        first = false;
        char bssid[18]{};
        std::snprintf(bssid,
                      sizeof(bssid),
                      "%02X:%02X:%02X:%02X:%02X:%02X",
                      records[index].bssid[0],
                      records[index].bssid[1],
                      records[index].bssid[2],
                      records[index].bssid[3],
                      records[index].bssid[4],
                      records[index].bssid[5]);
        out += '{';
        append_json_pair(&out, "bssid", bssid, true);
        out += ",\"rssi_dbm\":" + std::to_string(static_cast<int>(records[index].rssi));
        append_uint(&out, "channel", records[index].primary);
        out += '}';
      }
    }
  } else if (count_result == ESP_ERR_WIFI_STATE) {
    out.replace(12U, 5U, "true");
  }
#endif
  out += "]}";
  return out;
}

bool apply_wifi_bssid_pin(const std::string &bssid,
                          std::string *message,
                          bool *station_transition_started) {
  if (station_transition_started != nullptr) *station_transition_started = false;
#if defined(ESP_PLATFORM) || defined(ESPECTRE_HOST_WIFI_CONTROL_TEST)
  unsigned int octets[6]{};
  if (!bssid.empty()) {
    bool valid = bssid.size() == 17U;
    for (size_t index = 0U; valid && index < bssid.size(); ++index) {
      valid = index % 3U == 2U
                  ? bssid[index] == ':'
                  : std::isxdigit(static_cast<unsigned char>(bssid[index])) != 0;
    }
    if (!valid || std::sscanf(bssid.c_str(),
                              "%2x:%2x:%2x:%2x:%2x:%2x",
                              &octets[0],
                              &octets[1],
                              &octets[2],
                              &octets[3],
                              &octets[4],
                              &octets[5]) != 6) {
      if (message != nullptr) *message = "BSSID must contain six hexadecimal octets";
      return false;
    }
  }

  wifi_config_t original_config{};
  const esp_err_t read_err = esp_wifi_get_config(WIFI_IF_STA, &original_config);
  if (read_err != ESP_OK) {
    if (message != nullptr) {
      *message = std::string("Wi-Fi BSSID pin update failed while reading the station config: ") +
                 esp_err_to_name(read_err);
    }
    return false;
  }
  wifi_config_t config = original_config;
  if (bssid.empty()) {
    config.sta.bssid_set = false;
    std::memset(config.sta.bssid, 0, sizeof(config.sta.bssid));
    config.sta.channel = 0U;
  } else {
    for (size_t index = 0U; index < 6U; ++index) {
      config.sta.bssid[index] = static_cast<uint8_t>(octets[index]);
    }
    config.sta.bssid_set = true;
    config.sta.channel = 0U;
  }
  const esp_err_t disconnect_err = esp_wifi_disconnect();
  if (disconnect_err != ESP_OK && disconnect_err != ESP_ERR_WIFI_NOT_CONNECT) {
    if (message != nullptr) {
      *message = std::string("Wi-Fi BSSID pin update failed while disconnecting the station: ") +
                 esp_err_to_name(disconnect_err);
    }
    return false;
  }
  if (disconnect_err == ESP_OK && station_transition_started != nullptr) {
    *station_transition_started = true;
  }

  const esp_err_t update_err = esp_wifi_set_config(WIFI_IF_STA, &config);
  if (update_err != ESP_OK) {
    const esp_err_t restore_err = esp_wifi_set_config(WIFI_IF_STA, &original_config);
    const esp_err_t reconnect_err = restore_err == ESP_OK ? esp_wifi_connect() : restore_err;
    if (message != nullptr) {
      *message = std::string("Wi-Fi BSSID pin update failed while setting the station config: ") +
                 esp_err_to_name(update_err);
      *message += reconnect_err == ESP_OK
                      ? "; previous station association restarted"
                      : std::string("; previous station association restart failed: ") +
                            esp_err_to_name(reconnect_err);
    }
    return false;
  }

  const esp_err_t connect_err = esp_wifi_connect();
  if (connect_err != ESP_OK) {
    const esp_err_t rollback_config_err = esp_wifi_set_config(WIFI_IF_STA, &original_config);
    const esp_err_t rollback_connect_err =
        rollback_config_err == ESP_OK ? esp_wifi_connect() : rollback_config_err;
    if (message != nullptr) {
      *message = std::string("Wi-Fi BSSID pin update failed while reconnecting the station: ") +
                 esp_err_to_name(connect_err);
      if (rollback_config_err != ESP_OK) {
        *message += std::string("; previous station config restore failed: ") +
                    esp_err_to_name(rollback_config_err);
      } else if (rollback_connect_err != ESP_OK) {
        *message += std::string("; previous station reconnect failed: ") +
                    esp_err_to_name(rollback_connect_err);
      } else {
        *message += "; previous station association restarted";
      }
    }
    return false;
  }
  if (station_transition_started != nullptr) *station_transition_started = true;

  if (message != nullptr) {
    *message = bssid.empty() ? "Wi-Fi BSSID pin cleared; station reassociation started"
                             : "Wi-Fi BSSID pin updated; station reassociation started";
  }
  return true;
#else
  (void) bssid;
  (void) station_transition_started;
  if (message != nullptr) *message = "Wi-Fi control accepted by host test adapter";
  return true;
#endif
}

bool RuntimeDirectHttpBridge::handle_wifi_control_(const EspectreCommand &command,
                                                    std::string *message) {
  if (command.command == "scan_wifi") {
#if defined(ESP_PLATFORM)
    const std::string configured_ssid = wifi_snapshot_().ssid;
    if (configured_ssid.empty()) {
      if (message != nullptr) *message = "Wi-Fi access point scan requires a configured SSID";
      return false;
    }
    std::vector<uint8_t> scan_ssid(configured_ssid.begin(), configured_ssid.end());
    scan_ssid.push_back(0U);
    wifi_scan_config_t scan{};
    scan.ssid = scan_ssid.data();
    const esp_err_t result = esp_wifi_scan_start(&scan, false);
    if (message != nullptr) {
      *message = result == ESP_OK ? "Wi-Fi access point scan started"
                                  : "Wi-Fi access point scan could not be started";
    }
    return result == ESP_OK;
#else
    if (message != nullptr) *message = "Wi-Fi control accepted by host test adapter";
    return true;
#endif
  }
  if (command.command != "set_wifi_bssid" && command.command != "clear_wifi_bssid") {
    return false;
  }
  const std::string pin = command.command == "clear_wifi_bssid" ? std::string{} : command.wifi_bssid;
  if (config_.wifi_bssid_pin_setter) {
    return config_.wifi_bssid_pin_setter(pin, command.wifi_bssid_force, message);
  }
  return apply_wifi_bssid_pin(pin, message);
}

std::string RuntimeDirectHttpBridge::capabilities_payload_() const {
  const RuntimeCapabilities &capabilities = runtime_->capabilities();
  EspectreDeviceConfig device;
  device.device_id = config_.device_id;
  EspectreDeviceInfo info;
  info.frontend = config_.frontend;
  info.firmware_version = config_.firmware_version;
  info.chip = config_.chip;
  info.supports_info = true;
  info.supports_diagnostics = capabilities.supports_extended_diagnostics;
  info.supports_runtime_threshold = capabilities.supports_runtime_threshold_updates;
  info.supports_runtime_motion_hits = capabilities.supports_runtime_motion_hits_updates;
  info.supports_runtime_detector = capabilities.supports_runtime_detector_selection;
  info.supports_manual_recalibration = capabilities.supports_manual_recalibration;
  info.supports_traffic_control = capabilities.supports_traffic_control;
  info.csi_traffic_udp_port = runtime_->config().csi_traffic_udp_port;
  info.csi_traffic_multicast_group = runtime_->config().csi_traffic_multicast_group;
  return espectre_capabilities_payload(device, info, capability_profile_());
}

bool RuntimeDirectHttpBridge::handle_raw_stream_(const EspectreCommand &command,
                                                  const FrontendCommandContext &context,
                                                  std::string *code,
                                                  std::string *message,
                                                  std::string *data_json) {
  if (!config_.raw_csi) {
    if (code != nullptr) *code = "unsupported";
    if (message != nullptr) *message = "raw CSI collection is unavailable";
    return false;
  }
  return raw_session_controller_.handle_command(command, context, code, message, data_json);
}

std::string RuntimeDirectHttpBridge::device_payload_() const {
  EspectreDeviceConfig device;
  device.device_id = config_.device_id;
  device.device_label = device_label_();
  EspectreDeviceInfo info;
  info.frontend = config_.frontend;
  info.firmware_version = config_.firmware_version;
  info.chip = config_.chip;
  info.evaluation_interval_ms = runtime_->config().evaluation_interval_ms;
  info = normalize_protocol_device_info(info, &runtime_->snapshot(),
                                        config_.frontend.c_str(),
                                        config_.chip.c_str());
  return espectre_device_payload(device, info);
}

std::string RuntimeDirectHttpBridge::health_payload_() const {
  EspectreDeviceConfig device;
  device.device_id = config_.device_id;
  return espectre_health_payload(device, true, monotonic_now_ms());
}

std::string RuntimeDirectHttpBridge::sensing_payload_() const {
  const RuntimeConfig &config = runtime_->config();
  std::string out{"{"};
  const RuntimeSnapshot &snapshot = runtime_->snapshot();
  const bool collecting = runtime_->operation_state() == RuntimeOperationState::RAW_COLLECTION;
  append_bool(&out, "enabled", runtime_->services_armed(), true);
  append_bool(&out, "ready", snapshot.ready_to_publish && !collecting);
  append_bool(&out, "calibrating", runtime_->is_calibrating());
  append_json_pair(&out, "mode", collecting ? "csi_collection" : "sensing");
  append_bool(&out, "derived_events_paused", collecting);
  append_json_pair(&out, "detector", detection_algorithm_name(config.detection_algorithm));
  append_float(&out, "threshold", snapshot.threshold);
  append_uint(&out, "motion_on_hits", config.motion_on_hits);
  append_uint(&out, "motion_off_hits", config.motion_off_hits);
  append_json_pair(&out, "csi_traffic_mode", csi_traffic_mode_name(config.csi_traffic_mode));
  append_json_pair(&out, "traffic_generator_mode", traffic_mode_name(config.traffic_generator_mode));
  append_uint(&out, "csi_target_pps", config.csi_target_pps);
  append_uint(&out, "csi_traffic_udp_port", config.csi_traffic_udp_port);
  append_json_pair(&out, "csi_traffic_multicast_group", config.csi_traffic_multicast_group.c_str());
  out += "}";
  return out;
}

std::string RuntimeDirectHttpBridge::wifi_payload_() const {
  const DirectWifiSnapshot wifi = wifi_snapshot_();
  std::string out{"{"};
  append_bool(&out, "configured", wifi.configured, true);
  append_bool(&out, "connected", wifi.connected);
  append_json_pair(&out, "ssid", wifi.ssid.c_str());
  append_json_pair(&out, "bssid", wifi.bssid.c_str());
  append_json_pair(&out, "band", wifi.band.c_str());
  append_uint(&out, "channel", wifi.channel);
  out += ",\"rssi_dbm\":";
  out += wifi.rssi_dbm == INT16_MIN ? "null" : std::to_string(wifi.rssi_dbm);
  out += "}";
  return out;
}

std::string RuntimeDirectHttpBridge::diagnostics_payload_() const {
  const RuntimeDiagnosticsSnapshot diagnostics = runtime_->diagnostics();
  const uint32_t now_ms = monotonic_now_ms();
  std::string out{"{"};
  append_uint(&out, "timestamp_ms", now_ms, true);
  append_uint(&out, "uptime", now_ms / 1000U);
  append_uint(&out, "traffic_packets_total", diagnostics.traffic_packets_total);
  append_uint(&out, "csi_callbacks_total", diagnostics.csi_callbacks_total);
  append_uint(&out, "csi_classified_total", diagnostics.csi_classified_total);
  append_uint(&out,
              "csi_provenance_rejected_total",
              diagnostics.csi_provenance_rejected_total);
  append_uint(&out, "csi_accepted_total", diagnostics.csi_accepted_total);
  append_uint(&out, "csi_admitted_total", diagnostics.csi_admitted_total);
  append_uint(&out, "csi_filtered_total", diagnostics.csi_filtered_total);
  append_uint(&out,
              "csi_pending_frame_drops_total",
              diagnostics.csi_pending_frame_drops_total);
  append_uint(&out, "csi_missing_slots_total", diagnostics.csi_missing_slots_total);
  append_uint(&out, "csi_excess_total", diagnostics.csi_excess_total);
  append_uint(&out, "csi_stale_total", diagnostics.csi_stale_total);
  append_uint(&out, "csi_out_of_order_total", diagnostics.csi_out_of_order_total);
  append_uint(&out, "csi_occupancy_slots", diagnostics.csi_occupancy_slots);
  append_uint(&out, "csi_window_slots", diagnostics.csi_window_slots);
  append_uint(&out, "csi_pending_frames", diagnostics.csi_pending_frames);
  append_uint(&out,
              "csi_pending_frame_capacity",
              diagnostics.csi_pending_frame_capacity);
  const RuntimeDiagnosticsSample *sample = config_.diagnostics_sample_getter
                                               ? config_.diagnostics_sample_getter()
                                               : nullptr;
  if (sample != nullptr) {
    append_float(&out, "traffic_tx_pps", sample->traffic_tx_pps);
    append_float(&out, "csi_callback_pps", sample->csi_callback_pps);
    append_float(&out, "csi_accepted_pps", sample->csi_accepted_pps);
    append_float(&out, "csi_admitted_pps", sample->csi_admitted_pps);
    append_float(&out, "csi_filtered_pps", sample->csi_filtered_pps);
    append_float(&out,
                 "csi_pending_frame_drop_pps",
                 sample->csi_pending_frame_drop_pps);
    append_float(&out, "csi_missing_slots_pps", sample->csi_missing_slots_pps);
    append_float(&out, "csi_excess_pps", sample->csi_excess_pps);
    append_float(&out, "csi_stale_pps", sample->csi_stale_pps);
    append_float(&out, "csi_out_of_order_pps", sample->csi_out_of_order_pps);
    append_float(&out, "csi_occupancy", sample->csi_occupancy_ratio);
  }
  out += ",\"wifi_rssi_dbm\":";
  const int8_t wifi_rssi_dbm = sample != nullptr ? sample->wifi_rssi_dbm : diagnostics.wifi_rssi_dbm;
  out += wifi_rssi_dbm == INT8_MIN
             ? "null"
             : std::to_string(static_cast<int>(wifi_rssi_dbm));
  append_uint(&out,
              "wifi_channel",
              sample != nullptr ? sample->wifi_channel : diagnostics.wifi_channel);
  append_runtime_csi_quality_diagnostics_json(&out, diagnostics);
  append_runtime_performance_diagnostics_json(&out, diagnostics);
  if (service_ != nullptr) {
    const DirectHttpServiceDiagnostics direct = service_->diagnostics();
    const size_t event_client_count = event_client_count_.load(std::memory_order_relaxed);
    if (config_.runtime_events != nullptr) {
      append_uint(&out,
                  "runtime_motion_event_drops_total",
                  config_.runtime_events->motion_state_drops_total());
    }
    out += ",\"direct_http\":{";
    append_uint(&out, "event_clients", event_client_count, true);
    append_uint(&out, "event_client_limit", direct.event_client_limit);
    append_uint(&out, "queue_capacity", direct.queue_capacity);
    append_uint(&out, "queued_messages", direct.queued_messages);
    append_uint(&out, "accepted_connections", direct.accepted_connections);
    append_uint(&out, "rejected_connections", direct.rejected_connections);
    append_uint(&out, "malformed_requests", direct.malformed_requests);
    append_uint(&out, "oversized_requests", direct.oversized_requests);
    append_uint(&out, "rate_limited_requests", direct.rate_limited_requests);
    append_uint(&out, "dropped_motion_events", direct.dropped_motion_events);
    append_uint(&out, "send_failures", direct.send_failures);
    out += "}";
    const RawCsiSessionDiagnostics raw = service_->raw_diagnostics();
    out += ",\"raw_csi\":{\"active\":";
    out += raw.active ? "true" : "false";
    out += ",\"binary_bound\":";
    out += raw.binary_bound ? "true" : "false";
    append_uint(&out, "raw_drop_total", raw.raw_drop_total);
    append_uint(&out, "send_backpressure_total", raw.raw_send_backpressure_total);
    append_uint(&out, "fresh_record_total", raw.fresh_record_total);
    append_uint(&out, "stream_sequence", raw.stream_sequence);
    out += "}";
  }
  out += "}";
  return out;
}

void RuntimeDirectHttpBridge::refresh_peer_candidate_() {
  if (config_.peer_discovery == nullptr) return;
  const std::string device_id = format_espectre_device_id(config_.device_id);
  const std::string device_label = device_label_();
  const std::string display_name = device_label.empty() ? config_.device_name : device_label;
  PeerDiscoveryCandidate local;
  local.instance = device_label.empty() ? "ESPectre " + device_id : device_label + " " + device_id;
  local.hostname = config_.hostname;
  local.device_id = device_id;
  local.name = display_name;
  local.frontend = config_.frontend;
  local.txt_version = ESPECTRE_DNS_SD_TXT_SCHEMA_VERSION;
  local.protocol_version = ESPECTRE_PROTOCOL_VERSION;
  local.transport = ESPECTRE_DIRECT_HTTP_TRANSPORT;
  local.path = ESPECTRE_DIRECT_HTTP_BASE_ENDPOINT;
  local.firmware = config_.firmware_version;
  local.chip = config_.chip;
  local.capabilities = "sensing,motion,devices,csi";
  local.port = config_.port;
  config_.peer_discovery->set_local_candidate(std::move(local));
}

void RuntimeDirectHttpBridge::notify_config_changed_() {
  if (config_changed_) {
    config_changed_();
  }
}

}  // namespace espectre
