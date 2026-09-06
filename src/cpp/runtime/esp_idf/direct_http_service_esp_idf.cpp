/*
 * ESPectre - ESP-IDF Direct HTTP Service
 *
 * Bounded local HTTP, SSE, and binary streaming transport.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "direct_http_service_esp_idf.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <new>
#include <cerrno>
#include <cstring>
#include <limits>
#include <utility>

#include "espectre_log.h"
#include <esp_timer.h>
#include <lwip/sockets.h>
#if defined(ESP_PLATFORM)
#include <lwip/tcp.h>
#else
#include <netinet/tcp.h>
#endif

#include "protocol_json.h"
#include "espectre_protocol.h"
#include "sta_socket_helpers.h"
#include "task_scheduling_config.h"

namespace espectre {

namespace {

[[maybe_unused]] const char *const TAG = "espectre.http";
constexpr char kApiWildcard[] = "/espectre/v1/*";
constexpr size_t kHeaderBufferSize = 256U;
constexpr uint64_t kMutationWindowUs = 60ULL * 1000ULL * 1000ULL;
constexpr uint64_t kEventHeartbeatUs = 10ULL * 1000ULL * 1000ULL;
constexpr uint8_t kMaxConsecutiveSendFailures = 3U;
constexpr uint64_t kRequestWindowUs = 1000000U;
constexpr const char *kHttp400 = "400 Bad Request";
constexpr const char *kHttp403 = "403 Forbidden";
constexpr const char *kHttp404 = "404 Not Found";
constexpr const char *kHttp409 = "409 Conflict";
constexpr const char *kHttp413 = "413 Content Too Large";
constexpr const char *kHttp415 = "415 Unsupported Media Type";
constexpr const char *kHttp429 = "429 Too Many Requests";
constexpr const char *kHttp503 = "503 Service Unavailable";
#if defined(ESP_PLATFORM)
constexpr TickType_t kWorkerShutdownPollTicks = pdMS_TO_TICKS(1U);
constexpr uint32_t kWorkerShutdownTimeoutMs = 1500U;
#endif

const char *http_method_name(int method) {
  switch (method) {
    case HTTP_GET: return "GET";
    case HTTP_POST: return "POST";
    case HTTP_PUT: return "PUT";
    case HTTP_DELETE: return "DELETE";
    case HTTP_PATCH: return "PATCH";
    default: return "";
  }
}

const char *peer_disconnect_reason(int error) {
  switch (error) {
    case ECONNRESET:
      return "connection reset";
    case ENOTCONN:
      return "socket not connected";
    case EPIPE:
      return "broken pipe";
    default:
      return nullptr;
  }
}

bool read_only_method(const std::string &method, const EspectreProtocolExtension *extension) {
  if (const auto *route = find_extension_route(extension, method)) return route->kind == EspectreApiRouteKind::RESOURCE;
  return method == "capabilities" || method == "device" || method == "health" ||
         method == "sensing" || method == "wifi" || method == "mqtt" ||
         method == "read_diagnostics" || method == "devices" ||
         method == "wifi_access_points";
}

bool valid_loopback_port_suffix(const std::string &suffix) {
  if (suffix.empty()) return true;
  if (suffix.size() < 2U || suffix.front() != ':') return false;
  uint32_t port = 0U;
  for (size_t index = 1U; index < suffix.size(); ++index) {
    const unsigned char character = static_cast<unsigned char>(suffix[index]);
    if (!std::isdigit(character)) return false;
    port = port * 10U + static_cast<uint32_t>(character - static_cast<unsigned char>('0'));
    if (port > 65535U) return false;
  }
  return port > 0U;
}

bool http_loopback_origin(const std::string &origin) {
  std::string normalized = origin;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  constexpr const char *prefixes[] = {
      "http://localhost", "http://127.0.0.1", "http://[::1]",
  };
  for (const char *prefix : prefixes) {
    const size_t length = std::strlen(prefix);
    if (normalized.compare(0U, length, prefix) == 0 &&
        valid_loopback_port_suffix(normalized.substr(length))) {
      return true;
    }
  }
  return false;
}

bool session_id_present(const uint8_t *session_id) {
  if (session_id == nullptr) return false;
  for (size_t index = 0U; index < ESPECTRE_RAW_CSI_SESSION_ID_BYTES; ++index) {
    if (session_id[index] != 0U) return true;
  }
  return false;
}

bool conflicts_with_csi_collection(const std::string &command, const EspectreProtocolExtension *extension) {
  if (const auto *route = find_extension_route(extension, command)) return !route->allowed_during_raw_collection;
  return command == "update_sensing" || command == "recalibrate" ||
         command == "scan_wifi" || command == "set_wifi_bssid" ||
         command == "clear_wifi_bssid" || command == "clear_wifi_credentials";
}

std::string sse_payload(const std::string &event_name, const std::string &data_json) {
  return "event: " + event_name + "\ndata: " + data_json + "\n\n";
}

esp_err_t send_http_error(httpd_req_t *request, const char *status, const char *message) {
  if (request == nullptr || status == nullptr || message == nullptr) return ESP_ERR_INVALID_ARG;
  (void) httpd_resp_set_status(request, status);
  (void) httpd_resp_set_type(request, "text/plain; charset=utf-8");
  (void) httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  return httpd_resp_send(request, message, std::strlen(message));
}

}  // namespace

EspIdfDirectHttpService::EspIdfDirectHttpService() {
  mutex_ = xSemaphoreCreateMutex();
  raw_send_mutex_ = xSemaphoreCreateMutex();
}

EspIdfDirectHttpService::~EspIdfDirectHttpService() {
  shutdown_(false);
  if (mutex_ != nullptr) {
    vSemaphoreDelete(mutex_);
    mutex_ = nullptr;
  }
  if (raw_send_mutex_ != nullptr) {
    vSemaphoreDelete(raw_send_mutex_);
    raw_send_mutex_ = nullptr;
  }
}

bool EspIdfDirectHttpService::setup(const DirectHttpServiceConfig &config,
                                    RequestHandler request_handler,
                                    ClientCountCallback client_count_callback) {
  if (mutex_ == nullptr || raw_send_mutex_ == nullptr || !request_handler ||
      (config.allowed_origins.empty() && !config.allow_missing_origin) ||
      config.max_event_clients == 0U ||
      config.max_event_clients > 2U || config.max_pending_requests == 0U ||
      config.outbound_queue_depth == 0U || config.max_requests_per_second == 0U) {
    return false;
  }
  shutdown();
  if (lock_()) {
    diagnostics_ = {};
    next_request_token_ = 1U;
    request_window_started_us_ = 0U;
    request_count_ = 0U;
    mutation_window_started_us_ = 0U;
    mutation_count_ = 0U;
    pending_event_connections_ = 0U;
    unlock_();
  }
  if (config.protocol_extension != nullptr && !validate_protocol_extension(*config.protocol_extension)) return false;
  config_ = config;
  request_handler_ = std::move(request_handler);
  deferred_request_handler_ = {};
  client_count_callback_ = std::move(client_count_callback);

  httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
  http_config.task_priority = task_scheduling::kDirectHttpdPriority;
  http_config.server_port = config_.port;
  http_config.ctrl_port = static_cast<uint16_t>(http_config.ctrl_port + 1U);
  http_config.max_open_sockets = static_cast<uint16_t>(config_.max_event_clients + 5U);
  http_config.max_uri_handlers = 8U;
  http_config.uri_match_fn = httpd_uri_match_wildcard;
  http_config.lru_purge_enable = false;
  // Direct clients commonly poll diagnostics once per second. Leave enough
  // margin for scheduling and Wi-Fi jitter so a healthy keep-alive session is
  // not closed immediately before the next request arrives.
  http_config.recv_wait_timeout = 5U;
  http_config.send_wait_timeout = 1U;
  http_config.open_fn = &open_session_;
  if (httpd_start(&server_, &http_config) != ESP_OK) {
    server_ = nullptr;
    ESPECTRE_LOGE(TAG, "Failed to start Direct HTTP on port %u", static_cast<unsigned>(config_.port));
    return false;
  }

  const auto register_uri = [this](const char *uri, httpd_method_t method,
                                   esp_err_t (*handler)(httpd_req_t *)) {
    httpd_uri_t descriptor{};
    descriptor.uri = uri;
    descriptor.method = method;
    descriptor.handler = handler;
    descriptor.user_ctx = this;
    return httpd_register_uri_handler(server_, &descriptor) == ESP_OK;
  };
  const bool registered =
      register_uri(kApiWildcard, HTTP_GET, &request_uri_handler_) &&
      register_uri(kApiWildcard, HTTP_POST, &request_uri_handler_) &&
      register_uri(kApiWildcard, HTTP_PUT, &request_uri_handler_) &&
      register_uri(kApiWildcard, HTTP_DELETE, &request_uri_handler_) &&
      register_uri(kApiWildcard, HTTP_PATCH, &request_uri_handler_) &&
      register_uri(kApiWildcard, HTTP_OPTIONS, &options_handler_);
  if (!registered) {
    httpd_stop(server_);
    server_ = nullptr;
    ESPECTRE_LOGE(TAG, "Failed to register Direct HTTP endpoints");
    return false;
  }

  worker_running_.store(true, std::memory_order_release);
#if defined(ESP_PLATFORM)
  TaskHandle_t worker_task = nullptr;
  if (xTaskCreate(&worker_entry_, "espectre_http", 4096U, this,
                  task_scheduling::kDirectWorkerPriority, &worker_task) != pdPASS) {
    worker_running_.store(false, std::memory_order_release);
    httpd_stop(server_);
    server_ = nullptr;
    ESPECTRE_LOGE(TAG, "Failed to start Direct HTTP streaming worker");
    return false;
  }
  worker_task_.store(worker_task, std::memory_order_release);
#endif
  stopping_.store(false, std::memory_order_release);
  ESPECTRE_LOGI(TAG,
           "Direct HTTP listening on port %u (httpd=%u worker=%u raw=%u core=%d)",
           static_cast<unsigned>(config_.port),
           static_cast<unsigned>(http_config.task_priority),
           static_cast<unsigned>(task_scheduling::kDirectWorkerPriority),
           static_cast<unsigned>(task_scheduling::kRawWorkerPriority),
           http_config.core_id);
  return true;
}

bool EspIdfDirectHttpService::setup_deferred(const DirectHttpServiceConfig &config,
                                             DeferredRequestHandler request_handler,
                                             ClientCountCallback client_count_callback) {
  if (!request_handler) return false;
  const bool started = setup(config, [](const DirectRequest &) { return std::string{}; },
                             std::move(client_count_callback));
  if (started) {
    request_handler_ = {};
    deferred_request_handler_ = std::move(request_handler);
  }
  return started;
}

bool EspIdfDirectHttpService::complete_deferred_response(uint64_t request_token,
                                                         std::string response) {
  if (response.empty() || response.size() > ESPECTRE_DIRECT_MAX_RESPONSE_SIZE || !lock_()) {
    return false;
  }
  if (stopping_.load(std::memory_order_acquire)) {
    unlock_();
    return false;
  }
  const auto stored = std::find_if(deferred_.begin(), deferred_.end(),
                                   [request_token](const PendingRequest &request) {
                                     return request.token == request_token;
                                   });
  if (stored == deferred_.end()) {
    unlock_();
    return false;
  }
  PendingRequest pending = std::move(*stored);
  deferred_.erase(stored);
  enqueue_completed_response_locked_(std::move(pending), std::move(response));
  unlock_();
  notify_worker_();
  return true;
}

void EspIdfDirectHttpService::loop() {
  dispatch_pending_callbacks_();
  if (stopping_.load(std::memory_order_acquire)) return;

  PendingRequest pending;
  bool have_request = false;
  if (lock_()) {
    if (!inbound_.empty()) {
      pending = std::move(inbound_.front());
      inbound_.pop_front();
      have_request = true;
    }
    diagnostics_.queued_messages = inbound_.size() + deferred_.size() + completed_.size();
    for (const EventClient &client : event_clients_) diagnostics_.queued_messages += client.outbound.size();
    unlock_();
  }
  if (have_request) {
    bool deferred = false;
    std::string response;
    ResponseSentCallback response_sent_callback;
    if (deferred_request_handler_) {
      DeferredRequestResult result = deferred_request_handler_(pending.token, pending.direct);
      deferred = result.deferred;
      response = std::move(result.response);
      response_sent_callback = std::move(result.response_sent_callback);
    } else if (request_handler_) {
      response = request_handler_(pending.direct);
    }
    if (deferred) {
      if (lock_()) {
        deferred_.push_back(std::move(pending));
        unlock_();
      } else {
        release_request_(std::move(pending));
      }
    } else {
      if (response.empty()) {
        EspectreDeviceConfig device;
        device.device_id = config_.device_id;
        EspectreCommand command;
        command.command_id = pending.direct.command_id;
        command.command = pending.direct.command;
        response = espectre_command_result_payload(
            device, command, false, "internal_error", "empty Direct response");
      } else if (response.size() > ESPECTRE_DIRECT_MAX_RESPONSE_SIZE) {
        EspectreDeviceConfig device;
        device.device_id = config_.device_id;
        EspectreCommand command;
        command.command_id = pending.direct.command_id;
        command.command = pending.direct.command;
        response = espectre_command_result_payload(
            device, command, false, "internal_error", "Direct response exceeds the size limit");
      }
      (void) enqueue_completed_response_(std::move(pending),
                                         std::move(response),
                                         std::move(response_sent_callback));
    }
  }
#if !defined(ESP_PLATFORM)
  worker_loop_();
#endif
  dispatch_pending_callbacks_();
}

void EspIdfDirectHttpService::shutdown() { shutdown_(true); }

void EspIdfDirectHttpService::shutdown_(bool dispatch_callbacks) {
  stopping_.store(true, std::memory_order_release);
  (void) stop_raw_session(RawCsiStopReason::SHUTDOWN);
#if defined(ESP_PLATFORM)
  while (worker_notifications_active_.load(std::memory_order_acquire) != 0U ||
         raw_worker_notifications_active_.load(std::memory_order_acquire) != 0U) {
    vTaskDelay(kWorkerShutdownPollTicks);
  }
#endif
  worker_running_.store(false, std::memory_order_release);
  request_raw_worker_stop_();
#if defined(ESP_PLATFORM)
  TaskHandle_t worker_task = worker_task_.load(std::memory_order_acquire);
  TaskHandle_t raw_worker_task = raw_worker_task_.load(std::memory_order_acquire);
  if (worker_task != nullptr) xTaskNotifyGive(worker_task);
  uint32_t waited_ms = 0U;
  while ((worker_task_.load(std::memory_order_acquire) != nullptr ||
          raw_worker_task_.load(std::memory_order_acquire) != nullptr) &&
         waited_ms < kWorkerShutdownTimeoutMs) {
    vTaskDelay(kWorkerShutdownPollTicks);
    waited_ms += 1U;
  }
  worker_task = worker_task_.exchange(nullptr, std::memory_order_acq_rel);
  if (worker_task != nullptr) {
    ESPECTRE_LOGW(TAG, "Direct HTTP worker did not stop within %u ms",
             static_cast<unsigned>(kWorkerShutdownTimeoutMs));
    vTaskDelete(worker_task);
  }
  raw_worker_task = raw_worker_task_.exchange(nullptr, std::memory_order_acq_rel);
  if (raw_worker_task != nullptr) {
    ESPECTRE_LOGW(TAG, "Direct raw worker did not stop within %u ms",
             static_cast<unsigned>(kWorkerShutdownTimeoutMs));
    vTaskDelete(raw_worker_task);
  }
#endif

  std::vector<httpd_req_t *> requests;
  if (lock_()) {
    for (EventClient &client : event_clients_) requests.push_back(client.request);
    for (PendingRequest &pending : inbound_) requests.push_back(pending.request);
    for (PendingRequest &pending : deferred_) requests.push_back(pending.request);
    for (CompletedResponse &completed : completed_) {
      requests.push_back(completed.request.request);
      if (dispatch_callbacks && completed.response_sent_callback) {
        response_completions_.push_back(
            ResponseCompletion{std::move(completed.response_sent_callback), false});
      }
    }
    if (pending_raw_open_.request != nullptr) {
      requests.push_back(pending_raw_open_.request);
      pending_raw_open_ = {};
    }
    event_clients_.clear();
    pending_event_connections_ = 0U;
    inbound_.clear();
    deferred_.clear();
    completed_.clear();
    diagnostics_.queued_messages = 0U;
    unlock_();
  }
  for (httpd_req_t *request : requests) {
    if (request != nullptr) {
      (void) httpd_resp_send_chunk(request, nullptr, 0U);
      (void) httpd_req_async_handler_complete(request);
    }
  }

  httpd_handle_t server = server_;
  server_ = nullptr;
  if (server != nullptr) (void) httpd_stop(server);
  request_handler_ = {};
  deferred_request_handler_ = {};
  raw_session_requested_callback_ = {};
  if (dispatch_callbacks) {
    notify_client_count_(0U);
    dispatch_pending_callbacks_();
  } else {
    pending_client_count_event_.clear();
    if (lock_()) {
      pending_raw_stopped_callback_ = {};
      response_completions_.clear();
      unlock_();
    }
  }
  client_count_callback_ = {};
}

bool EspIdfDirectHttpService::running() const {
  return !stopping_.load(std::memory_order_acquire);
}

size_t EspIdfDirectHttpService::event_client_count() const {
  size_t count = 0U;
  if (lock_()) {
    count = event_clients_.size();
    unlock_();
  }
  return count;
}

bool EspIdfDirectHttpService::publish_event(const std::string &event_name,
                                            const std::string &data_json,
                                            bool replaceable_telemetry) {
  if (stopping_.load(std::memory_order_acquire) || event_name.empty()) {
    return false;
  }
  std::vector<JsonObjectField> fields;
  if (!parse_json_object_fields(data_json, &fields, nullptr) ||
      data_json.size() > ESPECTRE_DIRECT_MAX_RESPONSE_SIZE) {
    return false;
  }
  const OutboundEvent event{sse_payload(event_name, data_json), event_name, replaceable_telemetry};
  bool accepted = false;
  if (lock_()) {
    if (!stopping_.load(std::memory_order_acquire)) {
      for (EventClient &client : event_clients_) {
        accepted = enqueue_event_locked_(&client, event) || accepted;
      }
    }
    unlock_();
  }
  if (accepted) notify_worker_();
  return accepted;
}

DirectHttpServiceDiagnostics EspIdfDirectHttpService::diagnostics() const {
  DirectHttpServiceDiagnostics snapshot;
  if (lock_()) {
    snapshot = diagnostics_;
    snapshot.event_client_limit = config_.max_event_clients;
    snapshot.queue_capacity = config_.outbound_queue_depth;
    unlock_();
  }
  return snapshot;
}

void EspIdfDirectHttpService::set_raw_session_requested_callback(
    RawSessionRequestedCallback callback) {
  if (lock_()) {
    raw_session_requested_callback_ = std::move(callback);
    unlock_();
  }
}

bool EspIdfDirectHttpService::start_raw_session(
    const RawCsiSessionConfig &config,
    RawSessionStoppedCallback stopped_callback) {
  if (stopping_.load(std::memory_order_acquire) ||
      !session_id_present(config.session_id) || !lock_()) {
    return false;
  }
  if (stopping_.load(std::memory_order_acquire) ||
      raw_session_active_.load(std::memory_order_acquire)) {
    unlock_();
    return false;
  }
  if (raw_buffers_ != nullptr || pending_raw_stopped_callback_ ||
      raw_worker_task_.load(std::memory_order_acquire) != nullptr) {
    unlock_();
    return false;
  }
  raw_buffers_.reset(new (std::nothrow) RawBuffers);
  if (raw_buffers_ == nullptr) {
    unlock_();
    return false;
  }
#if defined(ESP_PLATFORM)
  raw_worker_running_.store(true, std::memory_order_release);
  TaskHandle_t task = nullptr;
  if (xTaskCreate(&raw_worker_entry_, "espectre_raw", 4096U, this,
                 task_scheduling::kRawWorkerPriority, &task) != pdPASS) {
    raw_worker_running_.store(false, std::memory_order_release);
    raw_buffers_.reset();
    unlock_();
    return false;
  }
  raw_worker_task_.store(task, std::memory_order_release);
#endif
  raw_session_ = {};
  raw_session_.config = config;
  raw_session_.stopped_callback = std::move(stopped_callback);
  raw_session_.generation = next_raw_session_generation_++;
  if (next_raw_session_generation_ == 0U) next_raw_session_generation_ = 1U;
  raw_session_.opened_at_us = static_cast<uint64_t>(esp_timer_get_time());
  raw_sample_head_.store(0U, std::memory_order_relaxed);
  raw_sample_tail_.store(0U, std::memory_order_relaxed);
  raw_offer_sequence_.store(0U, std::memory_order_relaxed);
  raw_drop_total_.store(0U, std::memory_order_relaxed);
  raw_send_backpressure_total_.store(0U, std::memory_order_relaxed);
  raw_fresh_record_total_.store(0U, std::memory_order_relaxed);
  raw_session_active_.store(true, std::memory_order_release);
  unlock_();
  return true;
}

bool EspIdfDirectHttpService::stop_raw_session(RawCsiStopReason reason) {
  bool expected_active = true;
  if (!raw_session_active_.compare_exchange_strong(
          expected_active, false, std::memory_order_acq_rel)) return false;
  while (raw_producer_active_.load(std::memory_order_acquire) != 0U) {
    vTaskDelay(1U);
  }
  if (xSemaphoreTake(raw_send_mutex_, portMAX_DELAY) != pdTRUE) return false;
  httpd_req_t *request = nullptr;
  std::string origin;
  bool headers_pending = false;
  if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
    xSemaphoreGive(raw_send_mutex_);
    return false;
  }
  const uint64_t head = raw_sample_head_.load(std::memory_order_acquire);
  const uint64_t tail = raw_sample_tail_.load(std::memory_order_acquire);
  if (tail > head) {
    raw_drop_total_.fetch_add(tail - head, std::memory_order_relaxed);
  }
  request = raw_session_.request;
  headers_pending = raw_session_.stream_sequence == 0U;
  if (headers_pending) origin = raw_session_.origin;
  pending_raw_stopped_callback_ = std::move(raw_session_.stopped_callback);
  pending_raw_stop_reason_ = reason;
  request_raw_worker_stop_();
  reset_raw_session_locked_();
  raw_buffers_.reset();
  unlock_();
  xSemaphoreGive(raw_send_mutex_);
  if (request != nullptr) {
    if (headers_pending) set_response_headers_(request, origin);
    (void) httpd_resp_send_chunk(request, nullptr, 0U);
    (void) httpd_req_async_handler_complete(request);
  }
  return true;
}

bool EspIdfDirectHttpService::offer_raw_packet(const RawCsiPacketView &packet) {
  if (stopping_.load(std::memory_order_acquire) ||
      !raw_session_active_.load(std::memory_order_acquire)) return false;
  raw_producer_active_.fetch_add(1U, std::memory_order_acq_rel);
  if (stopping_.load(std::memory_order_acquire) ||
      !raw_session_active_.load(std::memory_order_acquire)) {
    raw_producer_active_.fetch_sub(1U, std::memory_order_release);
    return false;
  }
  const uint64_t sequence = raw_offer_sequence_.fetch_add(1U, std::memory_order_relaxed) + 1U;
  if (packet.csi == nullptr || packet.csi_len == 0U ||
      packet.csi_len > RAW_CSI_MAX_PAYLOAD_BYTES || (packet.csi_len & 1U) != 0U) {
    raw_drop_total_.fetch_add(1U, std::memory_order_relaxed);
    raw_producer_active_.fetch_sub(1U, std::memory_order_release);
    return false;
  }
  const uint64_t tail = raw_sample_tail_.load(std::memory_order_relaxed);
  const uint64_t head = raw_sample_head_.load(std::memory_order_acquire);
  if (tail - head >= kRawQueueDepth) {
    raw_drop_total_.fetch_add(1U, std::memory_order_relaxed);
    raw_producer_active_.fetch_sub(1U, std::memory_order_release);
    return false;
  }
  RawSampleSlot &slot = raw_buffers_->samples[tail % kRawQueueDepth];
  slot.metadata = packet;
  std::memcpy(slot.csi.data(), packet.csi, packet.csi_len);
  slot.metadata.csi = slot.csi.data();
  slot.stream_sequence = sequence;
  raw_sample_tail_.store(tail + 1U, std::memory_order_release);
  notify_raw_worker_();
  raw_producer_active_.fetch_sub(1U, std::memory_order_release);
  return true;
}

RawCsiSessionDiagnostics EspIdfDirectHttpService::raw_diagnostics() const {
  RawCsiSessionDiagnostics snapshot;
  snapshot.active = raw_session_active_.load(std::memory_order_acquire);
  snapshot.raw_drop_total = raw_drop_total_.load(std::memory_order_relaxed);
  snapshot.raw_send_backpressure_total = raw_send_backpressure_total_.load(std::memory_order_relaxed);
  snapshot.fresh_record_total = raw_fresh_record_total_.load(std::memory_order_relaxed);
  if (lock_()) {
    snapshot.binary_bound = raw_session_.binary_bound;
    snapshot.stream_sequence = raw_offer_sequence_.load(std::memory_order_relaxed);
    unlock_();
  }
  return snapshot;
}

esp_err_t EspIdfDirectHttpService::request_uri_handler_(httpd_req_t *request) {
  if (request == nullptr || request->user_ctx == nullptr) return ESP_ERR_INVALID_ARG;
  auto *service = static_cast<EspIdfDirectHttpService *>(request->user_ctx);
  if (std::strcmp(request->uri, ESPECTRE_DIRECT_HTTP_EVENTS_ENDPOINT) == 0) {
    return service->handle_events_(request);
  }
  if (std::strcmp(request->uri, ESPECTRE_RAW_CSI_ENDPOINT) == 0) {
    return service->handle_raw_(request);
  }
  return service->handle_request_(request);
}

esp_err_t EspIdfDirectHttpService::events_handler_(httpd_req_t *request) {
  if (request == nullptr || request->user_ctx == nullptr) return ESP_ERR_INVALID_ARG;
  return static_cast<EspIdfDirectHttpService *>(request->user_ctx)->handle_events_(request);
}

esp_err_t EspIdfDirectHttpService::raw_handler_(httpd_req_t *request) {
  if (request == nullptr || request->user_ctx == nullptr) return ESP_ERR_INVALID_ARG;
  return static_cast<EspIdfDirectHttpService *>(request->user_ctx)->handle_raw_(request);
}

esp_err_t EspIdfDirectHttpService::options_handler_(httpd_req_t *request) {
  if (request == nullptr || request->user_ctx == nullptr) return ESP_ERR_INVALID_ARG;
  return static_cast<EspIdfDirectHttpService *>(request->user_ctx)->handle_options_(request);
}

esp_err_t EspIdfDirectHttpService::open_session_(httpd_handle_t, int socket) {
  return bind_socket_to_sta_interface(socket, TAG, "Direct HTTP") ? ESP_OK : ESP_FAIL;
}

void EspIdfDirectHttpService::worker_entry_(void *context) {
  auto *service = static_cast<EspIdfDirectHttpService *>(context);
  while (service != nullptr && service->worker_running_.load(std::memory_order_acquire)) {
    service->worker_loop_();
    // Event publication and completed requests wake the worker immediately;
    // the timeout keeps SSE heartbeats progressing without a 1 ms busy poll.
#if defined(ESP_PLATFORM)
    // Consume one wake-up per bounded worker iteration. Clearing the entire
    // count here can strand burst backlog until successive 100 ms timeouts.
    (void) ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(100));
#else
    vTaskDelay(pdMS_TO_TICKS(100));
#endif
  }
  if (service != nullptr) service->worker_task_.store(nullptr, std::memory_order_release);
  vTaskDelete(nullptr);
}

void EspIdfDirectHttpService::raw_worker_entry_(void *context) {
#if defined(ESP_PLATFORM)
  auto *service = static_cast<EspIdfDirectHttpService *>(context);
  while (service != nullptr && service->raw_worker_running_.load(std::memory_order_acquire)) {
    (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (service->raw_worker_running_.load(std::memory_order_acquire) &&
           service->service_raw_stream_()) {
    }
  }
  if (service != nullptr) {
    while (service->raw_worker_notifications_active_.load(std::memory_order_acquire) != 0U) {
      vTaskDelay(1U);
    }
    service->raw_worker_task_.store(nullptr, std::memory_order_release);
  }
  vTaskDelete(nullptr);
#else
  (void) context;
#endif
}

esp_err_t EspIdfDirectHttpService::handle_request_(httpd_req_t *request) {
  std::string origin;
  if (!validate_origin_(request, &origin)) return ESP_FAIL;
  bool service_available = false;
  bool request_allowed = false;
  if (lock_()) {
    service_available = !stopping_.load(std::memory_order_acquire);
    request_allowed = service_available &&
                      request_allowed_locked_(static_cast<uint64_t>(esp_timer_get_time()));
    if (service_available && !request_allowed) diagnostics_.rate_limited_requests += 1U;
    unlock_();
  }
  if (!service_available) {
    (void) send_error_(request, kHttp503, "Direct service is stopping", origin);
    return ESP_FAIL;
  }
  if (!request_allowed) {
    (void) send_error_(request, kHttp429, "Direct request rate limit reached", origin);
    return ESP_FAIL;
  }
  std::string content_type;
  if (request->content_len > 0U &&
      (!read_header_(request, "Content-Type", &content_type) ||
       content_type.compare(0U, std::strlen("application/json"), "application/json") != 0)) {
    (void) send_error_(request, kHttp415, "application/json required", origin);
    return ESP_FAIL;
  }
  if (request->content_len > ESPECTRE_DIRECT_MAX_REQUEST_SIZE) {
    if (lock_()) {
      diagnostics_.oversized_requests += request->content_len > ESPECTRE_DIRECT_MAX_REQUEST_SIZE ? 1U : 0U;
      unlock_();
    }
    (void) send_error_(request,
                       request->content_len > ESPECTRE_DIRECT_MAX_REQUEST_SIZE
                           ? kHttp413 : kHttp400,
                       "invalid Direct request size", origin);
    return ESP_FAIL;
  }
  std::string payload(request->content_len, '\0');
  size_t received = 0U;
  while (received < payload.size()) {
    const int result = httpd_req_recv(request, payload.data() + received, payload.size() - received);
    if (result <= 0) {
      (void) send_error_(request, kHttp400, "incomplete Direct request", origin);
      return ESP_FAIL;
    }
    received += static_cast<size_t>(result);
  }
  DirectRequest direct;
  std::string error;
  if (!parse_direct_http_request(http_method_name(request->method), request->uri,
                                 payload, &direct, &error, config_.protocol_extension)) {
    if (lock_()) {
      diagnostics_.malformed_requests += 1U;
      unlock_();
    }
    const char *status = error == "unsupported Direct resource or method" ? kHttp404 : kHttp400;
    (void) send_error_(request, status, error.c_str(), origin);
    return ESP_FAIL;
  }

  bool csi_conflict = false;
  if (conflicts_with_csi_collection(direct.command, config_.protocol_extension) && lock_()) {
    csi_conflict = pending_raw_open_.request != nullptr ||
                   raw_session_active_.load(std::memory_order_acquire);
    unlock_();
  }
  if (csi_conflict) {
    (void) send_error_(request, kHttp409,
                       "mutation is unavailable during CSI collection", origin);
    return ESP_FAIL;
  }

  bool accepted = false;
  bool rate_limited = false;
  bool queue_full = false;
  uint64_t token = 0U;
  if (lock_()) {
    service_available = !stopping_.load(std::memory_order_acquire);
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    const bool mutation_allowed = service_available &&
                                  mutation_allowed_locked_(direct.command, now_us);
    queue_full = inbound_.size() + deferred_.size() + completed_.size() >=
                 config_.max_pending_requests;
    if (mutation_allowed && !queue_full) {
      token = next_request_token_++;
      if (next_request_token_ == 0U) next_request_token_ = 1U;
      accepted = true;
    } else {
      rate_limited = service_available && !mutation_allowed;
      if (rate_limited) diagnostics_.rate_limited_requests += 1U;
    }
    unlock_();
  }
  if (!accepted) {
    const char *message = !service_available
                              ? "Direct service is stopping"
                              : queue_full ? "Direct request queue is full"
                                           : "Direct request rate limit reached";
    (void) send_error_(request, rate_limited ? kHttp429 : kHttp503, message, origin);
    return ESP_FAIL;
  }
  httpd_req_t *async_request = nullptr;
  if (httpd_req_async_handler_begin(request, &async_request) != ESP_OK || async_request == nullptr) {
    (void) send_error_(request, kHttp503, "Direct request unavailable", origin);
    return ESP_FAIL;
  }
  if (!lock_()) {
    (void) send_error_(async_request, kHttp503, "Direct request unavailable", origin);
    (void) httpd_req_async_handler_complete(async_request);
    return ESP_FAIL;
  }
  if (stopping_.load(std::memory_order_acquire)) {
    unlock_();
    (void) send_error_(async_request, kHttp503, "Direct service is stopping", origin);
    (void) httpd_req_async_handler_complete(async_request);
    return ESP_FAIL;
  }
  inbound_.push_back(PendingRequest{token, async_request, std::move(direct), std::move(origin)});
  unlock_();
  return ESP_OK;
}

esp_err_t EspIdfDirectHttpService::handle_events_(httpd_req_t *request) {
  std::string origin;
  if (!validate_origin_(request, &origin)) return ESP_FAIL;
  if (!lock_()) return ESP_FAIL;
  const bool service_available = !stopping_.load(std::memory_order_acquire);
  const bool available = service_available &&
                         event_clients_.size() + pending_event_connections_ <
                             config_.max_event_clients;
  if (available) {
    pending_event_connections_ += 1U;
  } else {
    diagnostics_.rejected_connections += 1U;
  }
  unlock_();
  if (!available) {
    (void) send_error_(request, kHttp503,
                       service_available ? "Direct event client limit reached"
                                         : "Direct service is stopping", origin);
    return ESP_FAIL;
  }
  (void) httpd_resp_set_type(request, "text/event-stream; charset=utf-8");
  (void) httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  (void) httpd_resp_set_hdr(request, "Connection", "keep-alive");
  httpd_req_t *async_request = nullptr;
  if (httpd_req_async_handler_begin(request, &async_request) != ESP_OK || async_request == nullptr) {
    if (lock_()) {
      if (pending_event_connections_ > 0U) pending_event_connections_ -= 1U;
      diagnostics_.rejected_connections += 1U;
      unlock_();
    }
    (void) send_error_(request, kHttp503, "Direct event stream unavailable", origin);
    return ESP_FAIL;
  }
  bool connection_available = false;
  if (lock_()) {
    connection_available = !stopping_.load(std::memory_order_acquire);
    if (!connection_available) {
      if (pending_event_connections_ > 0U) pending_event_connections_ -= 1U;
      diagnostics_.rejected_connections += 1U;
    }
    unlock_();
  }
  if (!connection_available) {
    (void) httpd_req_async_handler_complete(async_request);
    return ESP_FAIL;
  }
  set_response_headers_(async_request, origin);
  static constexpr char kConnected[] = "retry: 3000\n: connected\n\n";
  if (httpd_resp_send_chunk(async_request, kConnected, sizeof(kConnected) - 1U) != ESP_OK) {
    if (lock_()) {
      if (pending_event_connections_ > 0U) pending_event_connections_ -= 1U;
      diagnostics_.rejected_connections += 1U;
      diagnostics_.send_failures += 1U;
      unlock_();
    }
    (void) httpd_req_async_handler_complete(async_request);
    return ESP_FAIL;
  }
  const int fd = httpd_req_to_sockfd(async_request);
  size_t count = 0U;
  bool registered = false;
  if (lock_()) {
    if (pending_event_connections_ > 0U) pending_event_connections_ -= 1U;
    if (!stopping_.load(std::memory_order_acquire)) {
      event_clients_.push_back(EventClient{async_request, fd, 0U,
                                           static_cast<uint64_t>(esp_timer_get_time()), {}});
      diagnostics_.accepted_connections += 1U;
      count = event_clients_.size();
      registered = true;
      notify_client_count_(count);
    } else {
      diagnostics_.rejected_connections += 1U;
    }
    unlock_();
  }
  if (!registered) {
    (void) httpd_req_async_handler_complete(async_request);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t EspIdfDirectHttpService::handle_raw_(httpd_req_t *request) {
  std::string origin;
  if (!validate_origin_(request, &origin)) return ESP_FAIL;
  if (stopping_.load(std::memory_order_acquire)) {
    (void) send_error_(request, kHttp503, "Direct service is stopping", origin);
    return ESP_FAIL;
  }
  bool available = false;
  if (lock_()) {
    available = !stopping_.load(std::memory_order_acquire) &&
                pending_raw_open_.request == nullptr &&
                !raw_session_active_.load(std::memory_order_acquire) &&
                static_cast<bool>(raw_session_requested_callback_);
    unlock_();
  }
  if (!available) {
    (void) send_error_(request, kHttp409, "CSI collection is already active or unavailable", origin);
    return ESP_FAIL;
  }
  (void) httpd_resp_set_type(request, "application/octet-stream");
  (void) httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  httpd_req_t *async_request = nullptr;
  if (httpd_req_async_handler_begin(request, &async_request) != ESP_OK || async_request == nullptr) {
    (void) send_error_(request, kHttp503, "raw CSI stream unavailable", origin);
    return ESP_FAIL;
  }
  if (!lock_()) {
    (void) httpd_req_async_handler_complete(async_request);
    return ESP_FAIL;
  }
  if (stopping_.load(std::memory_order_acquire) || pending_raw_open_.request != nullptr ||
      raw_session_active_.load(std::memory_order_acquire)) {
    unlock_();
    (void) send_error_(async_request, kHttp409, "CSI collection is already active", origin);
    (void) httpd_req_async_handler_complete(async_request);
    return ESP_FAIL;
  }
  pending_raw_open_.request = async_request;
  pending_raw_open_.origin = std::move(origin);
  unlock_();
  return ESP_OK;
}

esp_err_t EspIdfDirectHttpService::handle_options_(httpd_req_t *request) {
  std::string origin;
  if (!validate_origin_(request, &origin)) return ESP_FAIL;
  if (stopping_.load(std::memory_order_acquire)) {
    (void) send_error_(request, kHttp503, "Direct service is stopping", origin);
    return ESP_FAIL;
  }
  set_response_headers_(request, origin);
  (void) httpd_resp_set_hdr(request, "Access-Control-Allow-Methods", "GET, PATCH, POST, PUT, DELETE, OPTIONS");
  (void) httpd_resp_set_hdr(request, "Access-Control-Allow-Headers", "Authorization, Content-Type");
  (void) httpd_resp_set_hdr(request, "Access-Control-Max-Age", "600");
  (void) httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  (void) httpd_resp_set_status(request, "204 No Content");
  return httpd_resp_send(request, nullptr, 0U);
}

bool EspIdfDirectHttpService::validate_origin_(httpd_req_t *request, std::string *origin) {
  if (origin == nullptr) return false;
  origin->clear();
  if (!read_header_(request, "Origin", origin)) {
    if (!config_.allow_missing_origin) {
      (void) send_http_error(request, kHttp403, "Origin required");
      return false;
    }
    return true;
  }
  const bool exact = std::find(config_.allowed_origins.begin(), config_.allowed_origins.end(), *origin) !=
                     config_.allowed_origins.end();
  if (!exact && !(config_.allow_http_loopback_origins && http_loopback_origin(*origin))) {
    (void) send_http_error(request, kHttp403, "Origin rejected");
    return false;
  }
  return true;
}

void EspIdfDirectHttpService::set_response_headers_(httpd_req_t *request,
                                                     const std::string &origin) const {
  if (!origin.empty()) {
    const auto configured =
        std::find(config_.allowed_origins.begin(), config_.allowed_origins.end(), origin);
    const char *response_origin = configured != config_.allowed_origins.end()
                                      ? configured->c_str()
                                      : origin.c_str();
    (void) httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", response_origin);
  }
  (void) httpd_resp_set_hdr(request, "Vary", "Origin");
  std::string private_network;
  if (read_header_(request, "Access-Control-Request-Private-Network", &private_network) &&
      private_network == "true") {
    (void) httpd_resp_set_hdr(request, "Access-Control-Allow-Private-Network", "true");
  }
}

esp_err_t EspIdfDirectHttpService::send_error_(httpd_req_t *request,
                                                const char *status,
                                                const char *message,
                                                const std::string &origin) const {
  set_response_headers_(request, origin);
  return send_http_error(request, status, message);
}

bool EspIdfDirectHttpService::read_header_(httpd_req_t *request,
                                           const char *name,
                                           std::string *value) const {
  if (request == nullptr || name == nullptr || value == nullptr) return false;
  const size_t length = httpd_req_get_hdr_value_len(request, name);
  if (length == 0U || length >= kHeaderBufferSize) return false;
  std::array<char, kHeaderBufferSize> buffer{};
  if (httpd_req_get_hdr_value_str(request, name, buffer.data(), buffer.size()) != ESP_OK) return false;
  value->assign(buffer.data(), length);
  return true;
}

bool EspIdfDirectHttpService::request_allowed_locked_(uint64_t now_us) {
  if (request_window_started_us_ == 0U ||
      now_us - request_window_started_us_ >= kRequestWindowUs) {
    request_window_started_us_ = now_us;
    request_count_ = 0U;
  }
  if (request_count_ >= config_.max_requests_per_second) return false;
  request_count_ += 1U;
  return true;
}

bool EspIdfDirectHttpService::mutation_allowed_locked_(const std::string &method, uint64_t now_us) {
  if (read_only_method(method, config_.protocol_extension)) return true;
  if (mutation_window_started_us_ == 0U || now_us - mutation_window_started_us_ >= kMutationWindowUs) {
    mutation_window_started_us_ = now_us;
    mutation_count_ = 0U;
  }
  if (mutation_count_ >= config_.max_mutations_per_minute) return false;
  mutation_count_ += 1U;
  return true;
}

bool EspIdfDirectHttpService::enqueue_event_locked_(EventClient *client, OutboundEvent event) {
  if (client == nullptr) return false;
  if (event.replaceable_telemetry) {
    const auto replace = std::find_if(client->outbound.rbegin(), client->outbound.rend(),
                                      [&event](const OutboundEvent &queued) {
                                        return queued.replaceable_telemetry && queued.event_name == event.event_name;
                                      });
    if (replace != client->outbound.rend()) {
      *replace = std::move(event);
      return true;
    }
  }
  if (client->outbound.size() >= config_.outbound_queue_depth) {
    if (event.replaceable_telemetry) {
      diagnostics_.dropped_motion_events += 1U;
      return false;
    }
    const auto stale = std::find_if(client->outbound.begin(), client->outbound.end(),
                                    [](const OutboundEvent &queued) {
                                      return queued.replaceable_telemetry;
                                    });
    if (stale == client->outbound.end()) return false;
    client->outbound.erase(stale);
    diagnostics_.dropped_motion_events += 1U;
  }
  client->outbound.push_back(std::move(event));
  return true;
}

void EspIdfDirectHttpService::enqueue_completed_response_locked_(PendingRequest request,
                                                                 std::string response,
                                                                 ResponseSentCallback response_sent_callback) {
  completed_.push_back(CompletedResponse{
      std::move(request), std::move(response), std::move(response_sent_callback)});
  diagnostics_.queued_messages = inbound_.size() + deferred_.size() + completed_.size();
}

bool EspIdfDirectHttpService::finish_request_(PendingRequest request, const std::string &response) {
  if (request.request == nullptr) return false;
  set_response_headers_(request.request, request.origin);
  (void) httpd_resp_set_type(request.request, "application/json; charset=utf-8");
  const bool rejected = response.find("\"accepted\":false") != std::string::npos;
  const bool conflict = response.find("\"code\":\"busy\"") != std::string::npos ||
                        response.find("\"code\":\"busy_raw_collection\"") != std::string::npos ||
                        response.find("\"code\":\"conflict\"") != std::string::npos;
  const bool unsupported = response.find("\"code\":\"unsupported\"") != std::string::npos;
  const bool invalid = response.find("\"code\":\"invalid_params\"") != std::string::npos;
  if (rejected && conflict) {
    (void) httpd_resp_set_status(request.request, "409 Conflict");
  } else if (rejected && unsupported) {
    (void) httpd_resp_set_status(request.request, "404 Not Found");
  } else if (rejected && invalid) {
    (void) httpd_resp_set_status(request.request, "400 Bad Request");
  } else if (request.direct.asynchronous) {
    (void) httpd_resp_set_status(request.request, "202 Accepted");
  }
  (void) httpd_resp_set_hdr(request.request, "Cache-Control", "no-store");
  const esp_err_t result = httpd_resp_send(request.request, response.data(), response.size());
  (void) httpd_req_async_handler_complete(request.request);
  if (result != ESP_OK && lock_()) {
    diagnostics_.send_failures += 1U;
    unlock_();
  }
  return result == ESP_OK;
}

bool EspIdfDirectHttpService::enqueue_completed_response_(PendingRequest request,
                                                          std::string response,
                                                          ResponseSentCallback response_sent_callback) {
  if (request.request == nullptr || response.empty()) return false;
  if (!lock_()) {
    release_request_(std::move(request));
    return false;
  }
  enqueue_completed_response_locked_(std::move(request),
                                     std::move(response),
                                     std::move(response_sent_callback));
  unlock_();
  notify_worker_();
  return true;
}

void EspIdfDirectHttpService::release_request_(PendingRequest request) {
  if (request.request != nullptr) {
    (void) httpd_req_async_handler_complete(request.request);
  }
}

void EspIdfDirectHttpService::service_event_streams_() {
  struct Send {
    int fd{-1};
    httpd_req_t *request{nullptr};
    std::string payload;
  };
  std::vector<Send> sends;
  const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
  if (lock_()) {
    // Most worker iterations have nothing to send. Reserving for the active
    // clients here would allocate and free on every 1 ms poll while an SSE
    // connection is open, eventually starving the single-core frontend loop.
    for (EventClient &client : event_clients_) {
      if (!client.outbound.empty()) {
        sends.push_back(Send{client.fd, client.request, std::move(client.outbound.front().payload)});
        client.outbound.pop_front();
      } else if (now_us - client.last_send_us >= kEventHeartbeatUs) {
        sends.push_back(Send{client.fd, client.request, ": ping\n\n"});
      }
    }
    unlock_();
  }
  size_t previous_count = event_client_count();
  for (const Send &send : sends) {
    const esp_err_t result = httpd_resp_send_chunk(send.request, send.payload.data(), send.payload.size());
    const int send_errno = result == ESP_OK ? 0 : errno;
    const char *disconnect_reason = peer_disconnect_reason(send_errno);
    httpd_req_t *request_to_complete = nullptr;
    if (lock_()) {
      const auto client = std::find_if(event_clients_.begin(), event_clients_.end(),
                                       [&send](const EventClient &candidate) {
                                         return candidate.fd == send.fd;
                                       });
      if (client != event_clients_.end()) {
        if (result == ESP_OK) {
          client->last_send_us = now_us;
          client->consecutive_send_failures = 0U;
        } else if (disconnect_reason != nullptr) {
          request_to_complete = client->request;
          event_clients_.erase(client);
        } else {
          diagnostics_.send_failures += 1U;
          client->consecutive_send_failures += 1U;
          if (client->consecutive_send_failures >= kMaxConsecutiveSendFailures) {
            request_to_complete = client->request;
            event_clients_.erase(client);
          }
        }
      }
      unlock_();
    }
    if (request_to_complete != nullptr) {
      if (disconnect_reason != nullptr) {
        ESPECTRE_LOGI(TAG, "Direct SSE peer disconnected (fd=%d, reason=%s, errno=%d)",
                 send.fd, disconnect_reason, send_errno);
      }
      (void) httpd_req_async_handler_complete(request_to_complete);
    }
  }
  const size_t current_count = event_client_count();
  if (previous_count != current_count) notify_client_count_(current_count);
}

bool EspIdfDirectHttpService::service_raw_stream_() {
  if (xSemaphoreTake(raw_send_mutex_, portMAX_DELAY) != pdTRUE) return false;
  RawCsiSessionConfig config;
  httpd_req_t *request = nullptr;
  std::string origin;
  bool headers_pending = false;
  if (!lock_()) {
    xSemaphoreGive(raw_send_mutex_);
    return false;
  }
  if (!raw_session_active_.load(std::memory_order_acquire) || !raw_session_.binary_bound ||
      raw_session_.request == nullptr) {
    unlock_();
    xSemaphoreGive(raw_send_mutex_);
    return false;
  }
  config = raw_session_.config;
  request = raw_session_.request;
  headers_pending = raw_session_.stream_sequence == 0U;
  if (headers_pending) origin = raw_session_.origin;
  unlock_();

  size_t length = 0U;
  size_t records = 0U;
  uint64_t last_sequence = 0U;
  RawSampleSlot sample;
  while (records < kRawBatchRecords && pop_raw_sample_(&sample)) {
    const uint64_t sequence = sample.stream_sequence;
    last_sequence = sequence;
    const uint64_t fresh_total =
        raw_fresh_record_total_.load(std::memory_order_relaxed) + records + 1U;
    RawCsiHttpFramePrefix prefix{};
    prefix.magic = ESPECTRE_RAW_CSI_RESPONSE_MAGIC;
    prefix.version = ESPECTRE_RAW_CSI_PROTOCOL_VERSION;
    prefix.record_version = ESPECTRE_RAW_CSI_RECORD_VERSION;
    prefix.header_len = sizeof(prefix);
    std::memcpy(prefix.session_id, config.session_id, sizeof(prefix.session_id));
    prefix.stream_sequence = sequence;
    prefix.record_len = static_cast<uint16_t>(sizeof(RawCsiRecordHeaderV8) + sample.metadata.csi_len);
    prefix.flags = 0U;
    prefix.fresh_record_total = fresh_total;
    prefix.raw_drop_total = raw_drop_total_.load(std::memory_order_relaxed);
    prefix.raw_send_backpressure_total = raw_send_backpressure_total_.load(std::memory_order_relaxed);
    std::memcpy(raw_buffers_->send.data() + length, &prefix, sizeof(prefix));
    length += sizeof(prefix);

    RawCsiRecordHeaderV8 header{};
    header.magic = RAW_CSI_RECORD_MAGIC;
    header.version = RAW_CSI_RECORD_VERSION_V8;
    header.header_len = sizeof(header);
    header.chip = static_cast<uint8_t>(config.chip);
    header.flags = sample.metadata.record_flags;
    header.seq_num = static_cast<uint32_t>(std::min<uint64_t>(sequence, UINT32_MAX));
    header.num_subcarriers = static_cast<uint16_t>(sample.metadata.csi_len / 2U);
    header.csi_len_bytes = sample.metadata.csi_len;
    header.device_id = config.device_id;
    header.device_ticks_us = sample.metadata.captured_at_us;
    header.wifi_rx_ts_us = sample.metadata.wifi_rx_ts_us;
    header.wifi_rx_start_ts_ns = sample.metadata.wifi_rx_start_ts_ns;
    header.channel = sample.metadata.channel;
    header.rssi_dbm = sample.metadata.rssi_dbm;
    header.noise_floor_dbm = sample.metadata.noise_floor_dbm;
    header.transport_backpressure_total = prefix.raw_send_backpressure_total;
    header.fresh_record_total = static_cast<uint32_t>(std::min<uint64_t>(fresh_total, UINT32_MAX));
    header.request_accepted_total =
        static_cast<uint32_t>(std::min<uint64_t>(sequence, UINT32_MAX));
    header.phy_mode = static_cast<uint8_t>(sample.metadata.phy_mode);
    header.ltf_type = static_cast<uint8_t>(sample.metadata.ltf_type);
    header.channel_width = static_cast<uint8_t>(sample.metadata.channel_width);
    std::memcpy(raw_buffers_->send.data() + length, &header, sizeof(header));
    length += sizeof(header);
    std::memcpy(raw_buffers_->send.data() + length, sample.csi.data(), sample.metadata.csi_len);
    length += sample.metadata.csi_len;
    records += 1U;
  }
  if (records == 0U) {
    xSemaphoreGive(raw_send_mutex_);
    return false;
  }

  // The HTTP server borrows header strings until the first chunk is sent.
  if (headers_pending) set_response_headers_(request, origin);
  const esp_err_t result = httpd_resp_send_chunk(request,
                                                  reinterpret_cast<const char *>(raw_buffers_->send.data()),
                                                  length);
  if (result != ESP_OK) {
    raw_send_backpressure_total_.fetch_add(1U, std::memory_order_relaxed);
    raw_drop_total_.fetch_add(records, std::memory_order_relaxed);
    xSemaphoreGive(raw_send_mutex_);
    (void) stop_raw_session(RawCsiStopReason::SLOW_CLIENT);
    return false;
  }
  raw_fresh_record_total_.fetch_add(records, std::memory_order_relaxed);
  if (lock_()) {
    raw_session_.last_send_us = static_cast<uint64_t>(esp_timer_get_time());
    raw_session_.stream_sequence = last_sequence;
    unlock_();
  }
  xSemaphoreGive(raw_send_mutex_);
  return true;
}

void EspIdfDirectHttpService::dispatch_pending_callbacks_() {
  bool raw_open_pending = false;
  RawSessionRequestedCallback raw_open_callback;
  if (lock_()) {
    raw_open_pending = pending_raw_open_.request != nullptr;
    raw_open_callback = raw_session_requested_callback_;
    unlock_();
  }
  if (raw_open_pending) {
    std::string message;
    const bool started = raw_open_callback && raw_open_callback(&message);
    PendingRawOpen pending;
    bool bound = false;
    if (lock_()) {
      pending = std::move(pending_raw_open_);
      pending_raw_open_ = {};
      if (started && pending.request != nullptr &&
          raw_session_active_.load(std::memory_order_acquire)) {
        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
        raw_session_.request = pending.request;
        raw_session_.fd = httpd_req_to_sockfd(pending.request);
        raw_session_.last_send_us = now_us;
        raw_session_.origin = pending.origin;
        const int keepalive = 1;
        (void) setsockopt(raw_session_.fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        raw_session_.binary_bound = true;
        bound = true;
      }
      unlock_();
    }
    if (!bound) {
      if (pending.request != nullptr) {
        (void) send_error_(pending.request, kHttp409,
                           message.empty() ? "CSI collection is unavailable" : message.c_str(),
                           pending.origin);
        (void) httpd_req_async_handler_complete(pending.request);
      }
    } else {
      notify_raw_worker_();
    }
  }

  size_t client_count = 0U;
  if (pending_client_count_event_.take(client_count) && client_count_callback_) {
    client_count_callback_(client_count);
  }

  RawSessionStoppedCallback stopped_callback;
  RawCsiStopReason stop_reason = RawCsiStopReason::INTERNAL_ERROR;
  if (lock_()) {
    if (raw_worker_task_.load(std::memory_order_acquire) == nullptr) {
      stopped_callback = std::move(pending_raw_stopped_callback_);
    }
    if (stopped_callback) {
      stop_reason = pending_raw_stop_reason_;
    }
    unlock_();
  }
  if (stopped_callback) stopped_callback(stop_reason);

  // libstdc++ allocates even an empty deque. Leave the idle loop allocation-free.
  if (!lock_()) return;
  const bool have_completions = !response_completions_.empty();
  unlock_();
  if (!have_completions) return;
  std::deque<ResponseCompletion> response_completions;
  if (lock_()) {
    response_completions.swap(response_completions_);
    unlock_();
  }
  for (ResponseCompletion &completion : response_completions) {
    if (completion.callback) completion.callback(completion.sent);
  }
}

void EspIdfDirectHttpService::worker_loop_() {
  if (server_ == nullptr) return;
  CompletedResponse completed;
  bool have_response = false;
  if (lock_()) {
    if (!completed_.empty()) {
      completed = std::move(completed_.front());
      completed_.pop_front();
      diagnostics_.queued_messages = inbound_.size() + deferred_.size() + completed_.size();
      have_response = true;
    }
    unlock_();
  }
  if (have_response) {
    const bool sent = finish_request_(std::move(completed.request), completed.response);
    if (completed.response_sent_callback && lock_()) {
      response_completions_.push_back(
          ResponseCompletion{std::move(completed.response_sent_callback), sent});
      unlock_();
    }
  }
  service_event_streams_();
#if !defined(ESP_PLATFORM)
  (void) service_raw_stream_();
#endif
}

bool EspIdfDirectHttpService::pop_raw_sample_(RawSampleSlot *sample) {
  if (sample == nullptr) return false;
  const uint64_t head = raw_sample_head_.load(std::memory_order_relaxed);
  const uint64_t tail = raw_sample_tail_.load(std::memory_order_acquire);
  if (head == tail) return false;
  const RawSampleSlot &slot = raw_buffers_->samples[head % kRawQueueDepth];
  sample->metadata = slot.metadata;
  std::memcpy(sample->csi.data(), slot.csi.data(), slot.metadata.csi_len);
  sample->metadata.csi = sample->csi.data();
  sample->stream_sequence = slot.stream_sequence;
  raw_sample_head_.store(head + 1U, std::memory_order_release);
  return true;
}

void EspIdfDirectHttpService::reset_raw_session_locked_() {
  raw_session_ = {};
  raw_sample_head_.store(0U, std::memory_order_relaxed);
  raw_sample_tail_.store(0U, std::memory_order_relaxed);
}

void EspIdfDirectHttpService::notify_client_count_(size_t count) {
  pending_client_count_event_.post(count);
}

void EspIdfDirectHttpService::notify_worker_() {
#if defined(ESP_PLATFORM)
  worker_notifications_active_.fetch_add(1U, std::memory_order_acq_rel);
  if (!stopping_.load(std::memory_order_acquire)) {
    TaskHandle_t task = worker_task_.load(std::memory_order_acquire);
    if (task != nullptr) xTaskNotifyGive(task);
  }
  worker_notifications_active_.fetch_sub(1U, std::memory_order_release);
#endif
}

void EspIdfDirectHttpService::notify_raw_worker_() {
#if defined(ESP_PLATFORM)
  raw_worker_notifications_active_.fetch_add(1U, std::memory_order_acq_rel);
  if (!stopping_.load(std::memory_order_acquire) &&
      raw_worker_running_.load(std::memory_order_acquire)) {
    TaskHandle_t task = raw_worker_task_.load(std::memory_order_acquire);
    if (task != nullptr) xTaskNotifyGive(task);
  }
  raw_worker_notifications_active_.fetch_sub(1U, std::memory_order_release);
#endif
}

void EspIdfDirectHttpService::request_raw_worker_stop_() {
#if defined(ESP_PLATFORM)
  // Publish the notification reference before stopping the worker, so its task
  // handle cannot disappear while the final wakeup is in progress.
  raw_worker_notifications_active_.fetch_add(1U, std::memory_order_acq_rel);
  if (raw_worker_running_.exchange(false, std::memory_order_acq_rel)) {
    TaskHandle_t task = raw_worker_task_.load(std::memory_order_acquire);
    if (task != nullptr) xTaskNotifyGive(task);
  }
  raw_worker_notifications_active_.fetch_sub(1U, std::memory_order_release);
#endif
}

bool EspIdfDirectHttpService::lock_() const {
  return mutex_ != nullptr && xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE;
}

void EspIdfDirectHttpService::unlock_() const { xSemaphoreGive(mutex_); }

}  // namespace espectre
