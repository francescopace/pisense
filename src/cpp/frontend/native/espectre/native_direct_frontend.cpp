/*
 * ESPectre - Native Direct Frontend Adapter
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

#include "native_direct_frontend.h"

#include <esp_log.h>

#include <cstdint>
#include <utility>

#include "direct_wifi_snapshot_esp_idf.h"
#include "native_frontend.h"
#include "native_mqtt_frontend.h"
#include "protocol_json.h"
#include "runtime_diagnostics.h"
#include "sdkconfig.h"
#include "wifi_band_helpers.h"

#if __has_include("esp_heap_caps.h")
#include "esp_heap_caps.h"
#define ESPECTRE_HAVE_ESP_HEAP_CAPS 1
#endif

#if defined(ESP_PLATFORM) && __has_include("freertos/FreeRTOS.h") && __has_include("freertos/task.h")
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace espectre {

namespace {

[[maybe_unused]] static const char *const TAG = "espectre.native.direct";

float current_free_memory_kb() {
#ifdef ESPECTRE_HAVE_ESP_HEAP_CAPS
  return static_cast<float>(heap_caps_get_free_size(MALLOC_CAP_DEFAULT)) / 1024.0f;
#else
  return 0.0f;
#endif
}

uint32_t current_task_stack_high_water_bytes() {
#if defined(ESP_PLATFORM) && INCLUDE_uxTaskGetStackHighWaterMark
  return static_cast<uint32_t>(uxTaskGetStackHighWaterMark(nullptr));
#else
  return 0U;
#endif
}

std::string wifi_bssid_ack_data(const std::string &current_bssid) {
  std::string data{"{"};
  append_json_pair(&data, "current_bssid", current_bssid.c_str(), true);
  data += "}";
  return data;
}

}  // namespace

NativeDirectFrontend::NativeDirectFrontend(NativeFrontend &owner, IDirectHttpService *service)
    : owner_(owner), service_(service) {}

void NativeDirectFrontend::set_peer_discovery_service(IPeerDiscoveryService *service) {
  peer_discovery_ = service;
  if (peer_discovery_ != nullptr) {
    peer_discovery_->set_wifi_ready(!owner_.device_info_.network.ip_address.empty());
  }
  refresh_peer_candidate_();
  refresh();
}

void NativeDirectFrontend::set_wifi_provisioning_info(const NativeWifiProvisioningInfo &info) {
  wifi_info_ = info;
  refresh();
}

void NativeDirectFrontend::refresh_identity() {
  if (peer_discovery_ != nullptr) {
    peer_discovery_->set_wifi_ready(!owner_.device_info_.network.ip_address.empty());
  }
  refresh_peer_candidate_();
  refresh();
}

void NativeDirectFrontend::refresh() {
  if (service_ == nullptr) {
    return;
  }
  const bool has_station_address = !owner_.device_info_.network.ip_address.empty();
  if (!owner_.runtime_.is_setup_complete() || !has_station_address || owner_.ota_frontend_quiesced_) {
    shutdown();
    return;
  }
  if (service_->running()) {
    return;
  }

  peer_discovery_enabled_ = false;
  session_tokens_enabled_ = false;
  wifi_response_pending_ = false;
  DirectHttpServiceConfig config = DirectHttpServiceConfig::for_first_party_portals();
  config.protocol_extension = owner_.command_capability_profile_(true).extension;
  config.device_id = owner_.device_config_.device_id;
#if defined(CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED) && CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED
  config.allow_http_loopback_origins = true;
#endif
  const auto client_count_changed = [this](size_t client_count) {
    this->client_count_ = client_count;
    this->owner_.update_live_telemetry_enabled_();
  };
  bool setup = service_->setup_deferred(
      config,
      [this](uint64_t token, const DirectRequest &request) { return this->handle_deferred_request_(token, request); },
      client_count_changed);
  session_tokens_enabled_ = setup;
  peer_discovery_enabled_ = setup && peer_discovery_ != nullptr;
  if (!setup) {
    setup = service_->setup(
        config, [this](const DirectRequest &request) { return this->handle_request_(request, 0U); },
        client_count_changed);
  }
  if (!setup) {
    ESP_LOGE(TAG, "Direct HTTP service setup failed");
    return;
  }
  uint64_t device_id = 0U;
  if (parse_espectre_device_id(espectre_effective_device_id(owner_.device_config_), &device_id)) {
    raw_session_controller_.configure(service_, &owner_.runtime_, device_id, owner_.device_info_.chip,
                                      [this](RawCsiStopReason) {
                                        this->owner_.runtime_events_.clear();
                                        this->owner_.mqtt_frontend_->home_assistant().cancel_pending_state();
                                        this->owner_.publish_runtime_status_state_();
                                      },
                                      [this]() {
                                        this->owner_.runtime_events_.clear();
                                        this->owner_.mqtt_frontend_->home_assistant().cancel_pending_state();
                                        this->owner_.publish_runtime_status_state_();
                                      });
  }
}

void NativeDirectFrontend::loop() {
  if (service_ != nullptr && service_->running()) {
    raw_session_controller_.ensure_runtime_consistency();
    service_->loop();
  }
  if (peer_discovery_ != nullptr) {
    peer_discovery_->loop();
  }
}

void NativeDirectFrontend::shutdown() {
  if (service_ != nullptr && service_->running()) {
    service_->shutdown();
  }
  const bool had_clients = client_count_ > 0U;
  client_count_ = 0U;
  peer_discovery_enabled_ = false;
  session_tokens_enabled_ = false;
  wifi_response_pending_ = false;
  if (had_clients) {
    owner_.update_live_telemetry_enabled_();
  }
}

void NativeDirectFrontend::shutdown_peer_discovery() {
  if (peer_discovery_ != nullptr) {
    peer_discovery_->shutdown();
  }
}

void NativeDirectFrontend::publish_event(const char *event_name, const std::string &data_json,
                                         bool replaceable_telemetry) {
  if (service_ == nullptr || !service_->running() || client_count_ == 0U || event_name == nullptr) {
    return;
  }
  (void)service_->publish_event(event_name, data_json, replaceable_telemetry);
}

std::string NativeDirectFrontend::handle_request_(const DirectRequest &request, uint64_t connection_token) {
  EspectreCommand command;
  std::string parse_error;
  if (!direct_http_request_to_command(request, &command, &parse_error, owner_.command_capability_profile_(true).extension)) {
    command.command_id = request.command_id;
    command.command = request.command;
    return espectre_command_result_payload(owner_.device_config_, command, false,
                                           frontend_command_parse_error_code(parse_error), parse_error.c_str());
  }
  const FrontendCommandResult result =
      owner_.dispatch_command_(command, FrontendCommandOrigin::DIRECT, true, connection_token);
  if (result.accepted && request.http_method == "GET") {
    return result.data_json;
  }
  return espectre_command_result_payload(owner_.device_config_, result.command, result.accepted, result.code.c_str(),
                                         result.message.c_str(), result.data_json);
}

IDirectHttpService::DeferredRequestResult NativeDirectFrontend::handle_deferred_request_(uint64_t connection_token,
                                                                                         const DirectRequest &request) {
  if (request.command == "set_wifi_bssid" || request.command == "clear_wifi_bssid" ||
      request.command == "clear_wifi_credentials") {
    if (owner_.runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION) {
      return {false, handle_request_(request, connection_token), {}};
    }
    EspectreCommand command;
    std::string parse_error;
    if (!direct_http_request_to_command(request, &command, &parse_error, owner_.command_capability_profile_(true).extension)) {
      command.command_id = request.command_id;
      command.command = request.command;
      return {false,
              espectre_command_result_payload(
                  owner_.device_config_, command, false,
                  frontend_command_parse_error_code(parse_error), parse_error.c_str()),
              {}};
    }
    if (!owner_.provisioning_command_callback_) {
      return {false,
              espectre_command_result_payload(
                  owner_.device_config_, command, false, "unavailable",
                  "Wi-Fi configuration changes are unavailable"),
              {}};
    }
    if (wifi_response_pending_ || wifi_info_.apply_state == "verifying" ||
        wifi_info_.apply_state == "rolling_back" || wifi_info_.scan_pending) {
      return {false,
              espectre_command_result_payload(
                  owner_.device_config_, command, false, "unavailable",
                  "Wi-Fi BSSID update already in progress"),
              {}};
    }
    if (request.command != "clear_wifi_credentials" && wifi_info_.ssid.empty()) {
      return {false,
              espectre_command_result_payload(
                  owner_.device_config_, command, false, "unavailable",
                  "provision Wi-Fi over Improv Serial before selecting a BSSID"),
              {}};
    }
    const DirectWifiSnapshot wifi = read_direct_wifi_snapshot();
    wifi_response_pending_ = true;
    const char *message = request.command == "set_wifi_bssid"
                              ? "Wi-Fi BSSID update accepted"
                              : request.command == "clear_wifi_bssid"
                                    ? "Wi-Fi BSSID clear accepted"
                                    : "Wi-Fi configuration clear accepted";
    return {
        false,
        espectre_command_result_payload(
            owner_.device_config_, command, true, "ok", message,
            wifi_bssid_ack_data(wifi.bssid)),
        [this, request, connection_token](bool sent) {
          this->wifi_response_pending_ = false;
          if (sent) (void) this->handle_request_(request, connection_token);
        },
    };
  }
  if (request.command != "devices") {
    return {false, handle_request_(request, connection_token), {}};
  }
  EspectreCommand command;
  command.command_id = request.command_id;
  command.command = request.command;
  if (!peer_discovery_enabled_ || peer_discovery_ == nullptr) {
    return {false, espectre_command_result_payload(owner_.device_config_, command, false, "unsupported",
                                                   "peer discovery is unavailable"),
            {}};
  }
  std::vector<JsonObjectField> params;
  if (!parse_json_object_fields(request.params, &params) || !params.empty()) {
    return {false, espectre_command_result_payload(owner_.device_config_, command, false, "invalid_params",
                                                   "devices does not accept parameters"),
            {}};
  }
  if (peer_discovery_->active()) {
    return {false, espectre_command_result_payload(owner_.device_config_, command, false, "conflict",
                                                   "a peer discovery request is already active"),
            {}};
  }
  const bool started = peer_discovery_->start([this, connection_token, request_id = request.command_id,
                                               command_name = request.command](PeerDiscoverySnapshot snapshot) {
    if (this->service_ == nullptr) {
      return;
    }
    EspectreCommand completed;
    completed.command_id = request_id;
    completed.command = command_name;
    (void)this->service_->complete_deferred_response(
        connection_token,
        peer_discovery_snapshot_json(snapshot));
  });
  if (!started) {
    return {false, espectre_command_result_payload(owner_.device_config_, command, false, "unavailable",
                                                   "peer discovery could not be started"),
            {}};
  }
  return {true, {}, {}};
}

std::string NativeDirectFrontend::capabilities_payload() const {
  return espectre_capabilities_payload(owner_.device_config_, owner_.mqtt_protocol_device_info_(),
                                       owner_.command_capability_profile_(true));
}

std::string NativeDirectFrontend::device_payload() const {
  return espectre_device_payload(owner_.device_config_, owner_.mqtt_protocol_device_info_());
}

std::string NativeDirectFrontend::health_payload(bool online) const {
  const uint32_t now = owner_.now_ms_();
  return espectre_health_payload(owner_.device_config_, online, now);
}

std::string NativeDirectFrontend::sensing_payload() const {
  const RuntimeConfig &runtime_config = owner_.runtime_.config();
  std::string out{"{"};
  const bool collecting = owner_.runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION;
  out += "\"enabled\":";
  out += owner_.runtime_.services_armed() ? "true" : "false";
  out += ",\"ready\":";
  out += owner_.runtime_.snapshot().ready_to_publish && !collecting ? "true" : "false";
  out += ",\"calibrating\":";
  out += owner_.runtime_.is_calibrating() ? "true" : "false";
  append_json_pair(&out, "mode", collecting ? "csi_collection" : "sensing");
  out += ",\"derived_events_paused\":";
  out += collecting ? "true" : "false";
  append_json_pair(&out, "detector", detection_algorithm_name(runtime_config.detection_algorithm));
  out += ",\"threshold\":" + std::to_string(owner_.runtime_.snapshot().threshold);
  out += ",\"motion_on_hits\":" + std::to_string(runtime_config.motion_on_hits);
  out += ",\"motion_off_hits\":" + std::to_string(runtime_config.motion_off_hits);
  append_json_pair(&out, "csi_traffic_mode", csi_traffic_mode_name(runtime_config.csi_traffic_mode));
  append_json_pair(&out, "traffic_generator_mode", traffic_mode_name(runtime_config.traffic_generator_mode));
  out += ",\"csi_target_pps\":" + std::to_string(runtime_config.csi_target_pps);
  out += ",\"csi_traffic_udp_port\":" + std::to_string(runtime_config.csi_traffic_udp_port);
  append_json_pair(&out, "csi_traffic_multicast_group", runtime_config.csi_traffic_multicast_group.c_str());
  out += "}";
  return out;
}

std::string NativeDirectFrontend::wifi_payload(bool mqtt_safe) const {
  DirectWifiSnapshot wifi = read_direct_wifi_snapshot();
  wifi.configured = wifi.configured || wifi_configured();
  wifi.connected = wifi.connected || !owner_.device_info_.network.ip_address.empty();
  if (wifi.ssid.empty()) wifi.ssid = wifi_info_.ssid;
  if (wifi.bssid.empty()) wifi.bssid = wifi_info_.bssid;
  if (wifi.channel == 0U) {
    wifi.channel = owner_.device_info_.network.channel != 0U ? owner_.device_info_.network.channel : wifi_info_.channel;
  }
  if (wifi.band.empty() && wifi.channel > 0U) {
    wifi.band = wifi.channel <= WIFI_CHANNEL_2G_MAX ? "2g" : "5g";
  }
  std::string out{"{\"configured\":"};
  out += wifi.configured ? "true" : "false";
  out += ",\"connected\":";
  out += wifi.connected ? "true" : "false";
  if (!mqtt_safe) {
    append_json_pair(&out, "ssid", wifi.ssid.c_str());
    append_json_pair(&out, "bssid", wifi.bssid.c_str());
    append_json_pair(&out, "ip", owner_.device_info_.network.ip_address.c_str());
  }
  append_json_pair(&out, "band", wifi.band.c_str());
  out += ",\"channel\":" + std::to_string(static_cast<unsigned>(wifi.channel));
  out += ",\"rssi_dbm\":";
  out += wifi.rssi_dbm == INT16_MIN ? "null" : std::to_string(wifi.rssi_dbm);
  append_json_pair(&out, "apply_state", wifi_info_.apply_state.c_str());
  append_json_pair(&out, "apply_message", wifi_info_.apply_message.c_str());
  out += "}";
  return out;
}

std::string NativeDirectFrontend::mqtt_payload() const {
  std::string out{"{\"configured\":"};
  out += espectre_mqtt_configured(owner_.device_config_) ? "true" : "false";
  append_json_pair(&out, "scheme", owner_.device_config_.mqtt_scheme.c_str());
  append_json_pair(&out, "host", owner_.device_config_.mqtt_host.c_str());
  out += ",\"port\":" + std::to_string(static_cast<unsigned>(owner_.device_config_.mqtt_port));
  out += ",\"username_configured\":";
  out += owner_.device_config_.mqtt_username.empty() ? "false" : "true";
  append_json_pair(&out, "topic_prefix", owner_.device_config_.topic_prefix.c_str());
  out += "}";
  return out;
}

std::string NativeDirectFrontend::ota_payload() const {
  return espectre_ota_status_payload(owner_.device_config_, owner_.current_ota_status_(), owner_.now_ms_());
}

std::string NativeDirectFrontend::wifi_access_points_payload() const {
  std::string out{"{\"scanning\":"};
  out += wifi_info_.scan_pending ? "true" : "false";
  append_json_pair(&out, "message", wifi_info_.scan_message.c_str());
  out += ",\"access_points\":[";
  bool first = true;
  for (const NativeWifiProvisioningInfo::AccessPoint &access_point : wifi_info_.access_points) {
    if (!first) out += ",";
    first = false;
    out += "{";
    append_json_pair(&out, "bssid", access_point.bssid.c_str(), true);
    out += ",\"rssi_dbm\":" + std::to_string(static_cast<int>(access_point.rssi_dbm));
    out += ",\"channel\":" + std::to_string(static_cast<unsigned>(access_point.channel));
    out += "}";
  }
  out += "]}";
  return out;
}

std::string NativeDirectFrontend::diagnostics_payload() const {
  const uint32_t now = owner_.now_ms_();
  std::string out = espectre_diagnostics_payload(owner_.device_config_, owner_.runtime_.snapshot(), now, now / 1000U,
                                                 current_free_memory_kb(), owner_.last_loop_time_ms_,
                                                 owner_.runtime_.diagnostics_sample());
  if (!out.empty() && out.back() == '}') {
    out.pop_back();
  }
  const RuntimeDiagnosticsSnapshot runtime_diagnostics = owner_.runtime_.diagnostics();
  out += ",\"csi_classified_total\":" + std::to_string(runtime_diagnostics.csi_classified_total);
  out += ",\"csi_provenance_rejected_total\":" + std::to_string(runtime_diagnostics.csi_provenance_rejected_total);
  out += ",\"csi_pending_frame_drops_total\":" + std::to_string(runtime_diagnostics.csi_pending_frame_drops_total);
  out += ",\"csi_pending_frames\":" + std::to_string(runtime_diagnostics.csi_pending_frames);
  out += ",\"csi_pending_frame_capacity\":" + std::to_string(runtime_diagnostics.csi_pending_frame_capacity);
  out += ",\"runtime_motion_event_drops_total\":" + std::to_string(owner_.runtime_events_.motion_state_drops_total());
  append_runtime_csi_quality_diagnostics_json(&out, runtime_diagnostics);
  append_runtime_performance_diagnostics_json(&out, runtime_diagnostics, false);
  out += ",\"task_stack_high_water_bytes\":" + std::to_string(current_task_stack_high_water_bytes());

  const DirectHttpServiceDiagnostics direct =
      service_ != nullptr ? service_->diagnostics() : DirectHttpServiceDiagnostics{};
  out += ",\"direct_http\":{\"event_clients\":" + std::to_string(client_count_);
  out += ",\"event_client_limit\":" + std::to_string(direct.event_client_limit);
  out += ",\"queue_capacity\":" + std::to_string(direct.queue_capacity);
  out += ",\"queued_messages\":" + std::to_string(direct.queued_messages);
  out += ",\"accepted_connections\":" + std::to_string(direct.accepted_connections);
  out += ",\"rejected_connections\":" + std::to_string(direct.rejected_connections);
  out += ",\"malformed_requests\":" + std::to_string(direct.malformed_requests);
  out += ",\"oversized_requests\":" + std::to_string(direct.oversized_requests);
  out += ",\"rate_limited_requests\":" + std::to_string(direct.rate_limited_requests);
  out += ",\"dropped_motion_events\":" + std::to_string(direct.dropped_motion_events);
  out += ",\"send_failures\":" + std::to_string(direct.send_failures) + "}";

  const RawCsiSessionDiagnostics raw = service_ != nullptr ? service_->raw_diagnostics() : RawCsiSessionDiagnostics{};
  out += ",\"raw_csi\":{\"active\":";
  out += raw.active ? "true" : "false";
  out += ",\"binary_bound\":";
  out += raw.binary_bound ? "true" : "false";
  out += ",\"raw_drop_total\":" + std::to_string(raw.raw_drop_total);
  out += ",\"send_backpressure_total\":" + std::to_string(raw.raw_send_backpressure_total);
  out += ",\"fresh_record_total\":" + std::to_string(raw.fresh_record_total);
  out += ",\"stream_sequence\":" + std::to_string(raw.stream_sequence) + "}";

  const MqttTransportDiagnostics mqtt = owner_.mqtt_frontend_->diagnostics();
  out += ",\"mqtt\":{\"connected\":";
  out += owner_.mqtt_frontend_->transport_connected() ? "true" : "false";
  out += ",\"queue_capacity\":" + std::to_string(mqtt.queue_capacity);
  out += ",\"outbox_capacity_bytes\":" + std::to_string(mqtt.outbox_capacity_bytes);
  out += ",\"queued_publishes\":" + std::to_string(mqtt.queued_publishes);
  out += ",\"dropped_publishes\":" + std::to_string(mqtt.dropped_publishes);
  out += ",\"publish_failures\":" + std::to_string(mqtt.publish_failures);
  out += ",\"reconnects\":" + std::to_string(mqtt.reconnects) + "}}";
  return out;
}

void NativeDirectFrontend::refresh_peer_candidate_() {
  if (peer_discovery_ == nullptr) return;
  const std::string device_id = espectre_effective_device_id(owner_.device_config_);
  const std::string generated_name =
      espectre_device_name(espectre_effective_device_id_u64(owner_.device_config_),
                           owner_.device_info_.chip.empty() ? nullptr : owner_.device_info_.chip.c_str());
  const std::string display_name =
      owner_.device_config_.device_label.empty() ? generated_name : owner_.device_config_.device_label;
  PeerDiscoveryCandidate candidate;
  candidate.instance = owner_.device_config_.device_label.empty()
                           ? "ESPectre " + device_id
                           : owner_.device_config_.device_label + " " + device_id;
  candidate.hostname = "espectre-" + device_id;
  candidate.device_id = device_id;
  candidate.name = display_name;
  candidate.frontend = "native";
  candidate.txt_version = ESPECTRE_DNS_SD_TXT_SCHEMA_VERSION;
  candidate.protocol_version = ESPECTRE_PROTOCOL_VERSION;
  candidate.transport = ESPECTRE_DIRECT_HTTP_TRANSPORT;
  candidate.path = ESPECTRE_DIRECT_HTTP_BASE_ENDPOINT;
  candidate.firmware = owner_.device_info_.firmware_version;
  candidate.chip = owner_.device_info_.chip;
  candidate.capabilities = "config,monitor,csi";
  candidate.port = ESPECTRE_DIRECT_HTTP_PORT;
  peer_discovery_->set_local_candidate(std::move(candidate));
}

bool NativeDirectFrontend::handle_raw_stream_command(const EspectreCommand &command,
                                                     const FrontendCommandContext &context, std::string *code,
                                                     std::string *message, std::string *data_json) {
  uint64_t device_id = 0U;
  if (!parse_espectre_device_id(espectre_effective_device_id(owner_.device_config_), &device_id)) {
    if (code != nullptr) *code = "internal_error";
    if (message != nullptr) *message = "device identity is unavailable";
    return false;
  }
  return raw_session_controller_.handle_command(command, context, code, message, data_json);
}

}  // namespace espectre
