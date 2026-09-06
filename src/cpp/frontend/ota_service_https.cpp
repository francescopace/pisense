/*
 * ESPectre - HTTPS OTA Service
 *
 * Checks OTA manifests and applies HTTPS firmware updates.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "ota_service_https.h"

#include <algorithm>
#include <memory>
#include <new>
#include <utility>

#include "esp_crt_bundle.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "espectre_log.h"
#include "esp_system.h"
#include "ota_version.h"
#include "protocol_json.h"

namespace espectre {

namespace {

static const char *const TAG = "espectre.ota";
constexpr size_t kMaxManifestBytes = 64U * 1024U;
constexpr uint32_t kHttpTimeoutMs = 30000U;
constexpr uint32_t kPostSuccessDelayMs = 500U;
constexpr uint32_t kWorkerStackSize = 8192U;
constexpr UBaseType_t kWorkerPriority = 5U;
// GitHub Releases 302 responses include a multi-kilobyte Content-Security-Policy
// header and a JWT Location URL. The ESP-IDF default 512-byte HTTP buffer fails
// with "Out of buffer" / ESP_FAIL before the body is read.
constexpr int kHttpRxBufferBytes = 8192;
constexpr int kHttpTxBufferBytes = 1024;

void fill_https_client_config(esp_http_client_config_t *config, const char *url) {
  config->url = url;
  config->timeout_ms = static_cast<int>(kHttpTimeoutMs);
  config->crt_bundle_attach = esp_crt_bundle_attach;
  config->buffer_size = kHttpRxBufferBytes;
  config->buffer_size_tx = kHttpTxBufferBytes;
}

struct ManifestFetchContext {
  std::string *body{nullptr};
  std::string *error{nullptr};
};

esp_err_t manifest_http_event(esp_http_client_event_t *event) {
  auto *context = static_cast<ManifestFetchContext *>(event->user_data);
  if (context == nullptr || context->body == nullptr || event->event_id != HTTP_EVENT_ON_DATA ||
      event->data_len <= 0) {
    return ESP_OK;
  }
  if (context->body->size() + static_cast<size_t>(event->data_len) > kMaxManifestBytes) {
    if (context->error != nullptr) {
      *context->error = "manifest too large";
    }
    return ESP_FAIL;
  }
  context->body->append(static_cast<const char *>(event->data), static_cast<size_t>(event->data_len));
  return ESP_OK;
}

}  // namespace

HttpsOtaService::HttpsOtaService(const char *frontend, const char *chip, OtaReleaseChannel channel) {
  lock_ = xSemaphoreCreateMutex();
  worker_done_ = xSemaphoreCreateBinary();
  prepare_done_ = xSemaphoreCreateBinary();
  frontend_ = frontend != nullptr ? frontend : "";
  chip_ = chip != nullptr ? chip : "";
  if (channel == OtaReleaseChannel::PREVIEW) {
    default_channel_ = ESPECTRE_OTA_CHANNEL_PREVIEW;
  } else if (channel == OtaReleaseChannel::DEVELOP) {
    default_channel_ = ESPECTRE_OTA_CHANNEL_DEVELOP;
  } else {
    default_channel_ = ESPECTRE_OTA_CHANNEL_RELEASE;
  }
  status_.default_channel = default_channel_;
  status_.channel = default_channel_;
  status_.manifest_url = espectre_ota_manifest_url(frontend_.c_str(), chip_.c_str(), default_channel_);
}

HttpsOtaService::~HttpsOtaService() { shutdown(); }

void HttpsOtaService::shutdown() {
  shutdown_requested_.store(true, std::memory_order_release);
  if (prepare_done_ != nullptr) {
    xSemaphoreGive(prepare_done_);
  }

  bool worker_active = false;
  if (lock_ != nullptr) {
    xSemaphoreTake(lock_, portMAX_DELAY);
    worker_active = worker_active_;
    xSemaphoreGive(lock_);
  }
  if (worker_active && worker_done_ != nullptr) {
    // HTTP and OTA APIs are synchronous. Let their owner unwind normally so
    // client handles and TLS state are released before this service disappears.
    xSemaphoreTake(worker_done_, portMAX_DELAY);
  }

  if (lock_ != nullptr) {
    // A completion signal is posted while the worker still holds the mutex.
    // This barrier guarantees finish_worker_ has left the critical section
    // before the synchronization objects are destroyed.
    xSemaphoreTake(lock_, portMAX_DELAY);
    xSemaphoreGive(lock_);
    vSemaphoreDelete(lock_);
    lock_ = nullptr;
  }
  if (worker_done_ != nullptr) {
    vSemaphoreDelete(worker_done_);
    worker_done_ = nullptr;
  }
  if (prepare_done_ != nullptr) {
    vSemaphoreDelete(prepare_done_);
    prepare_done_ = nullptr;
  }
}

void HttpsOtaService::loop() {
  StatusCallback status_callback;
  PrepareForUpdateCallback prepare_callback;
  EspectreOtaStatus pending_status;
  bool deliver_status = false;
  bool prepare = false;

  if (lock_ == nullptr) {
    return;
  }
  xSemaphoreTake(lock_, portMAX_DELAY);
  prepare = prepare_callback_pending_;
  prepare_callback_pending_ = false;
  prepare_callback = prepare_for_update_callback_;
  deliver_status = status_callback_pending_;
  status_callback_pending_ = false;
  if (deliver_status) {
    pending_status = pending_status_;
    status_callback = status_callback_;
  }
  xSemaphoreGive(lock_);

  if (prepare) {
    if (prepare_callback) {
      prepare_callback();
    }
    if (prepare_done_ != nullptr) {
      xSemaphoreGive(prepare_done_);
    }
  }
  if (deliver_status && status_callback) {
    status_callback(pending_status);
  }
}

bool HttpsOtaService::start_check(const std::string &current_version) {
  return start_check(current_version, std::string{});
}

bool HttpsOtaService::start_check(const std::string &current_version, const std::string &channel) {
  WorkerRequest request;
  request.action = WorkerAction::CHECK;
  request.current_version = current_version;
  request.channel = channel;
  return begin_request_(request);
}

bool HttpsOtaService::start_update(const std::string &current_version) {
  return start_update(current_version, std::string{});
}

bool HttpsOtaService::start_update(const std::string &current_version, const std::string &channel) {
  WorkerRequest request;
  request.action = WorkerAction::START_UPDATE;
  request.current_version = current_version;
  request.channel = channel;
  return begin_request_(request);
}

EspectreOtaStatus HttpsOtaService::status() const {
  if (!ensure_lock_()) {
    return status_;
  }
  xSemaphoreTake(lock_, portMAX_DELAY);
  const EspectreOtaStatus snapshot = status_;
  xSemaphoreGive(lock_);
  return snapshot;
}

void HttpsOtaService::set_status_callback(StatusCallback callback) {
  if (lock_ == nullptr) {
    return;
  }
  xSemaphoreTake(lock_, portMAX_DELAY);
  status_callback_ = std::move(callback);
  xSemaphoreGive(lock_);
}

void HttpsOtaService::set_prepare_for_update_callback(PrepareForUpdateCallback callback) {
  if (lock_ == nullptr) {
    return;
  }
  xSemaphoreTake(lock_, portMAX_DELAY);
  prepare_for_update_callback_ = std::move(callback);
  xSemaphoreGive(lock_);
}

void HttpsOtaService::worker_entry_(void *ctx) {
  std::unique_ptr<WorkerContext> context(static_cast<WorkerContext *>(ctx));
  if (context != nullptr && context->service != nullptr) {
    context->service->run_worker_(context->request);
    context->service->finish_worker_();
  }
  vTaskDelete(nullptr);
}

void HttpsOtaService::run_worker_(const WorkerRequest &request) {
  const std::string current_version = request.current_version.empty() ? "unknown" : request.current_version;
  const std::string channel = request.channel.empty() ? default_channel_ : request.channel;
  const std::string manifest_url = espectre_ota_manifest_url(frontend_.c_str(), chip_.c_str(), channel);
  ManifestInfo manifest;

  EspectreOtaStatus checking;
  checking.state = EspectreOtaState::CHECKING;
  checking.busy = true;
  checking.current_version = current_version;
  checking.channel = channel;
  checking.manifest_url = manifest_url;
  update_status_(checking);
  ESPECTRE_LOGI(TAG, "%s channel=%s url=%s", request.action == WorkerAction::CHECK ? "checking" : "updating",
           channel.c_str(), manifest_url.c_str());

  if (manifest_url.empty()) {
    set_error_status_("invalid ota channel", current_version, "", "", "", channel);
    return;
  }

  std::string body;
  std::string error;
  if (!fetch_https_text_(manifest_url, &body, &error) ||
      !parse_manifest_(std::move(body), channel, &manifest, &error)) {
    set_error_status_(error.empty() ? "manifest fetch failed" : error, current_version, "", manifest_url, "",
                      channel);
    return;
  }

  if (request.action == WorkerAction::CHECK) {
    EspectreOtaStatus result;
    result.current_version = current_version;
    result.target_version = manifest.version;
    result.channel = channel;
    result.manifest_url = manifest_url;
    result.image_url = manifest.image_url;
    const OtaVersionComparison comparison = compare_ota_versions(manifest.version, current_version);
    if (comparison == OtaVersionComparison::UNORDERED) {
      set_error_status_("unrecognized or divergent firmware version", current_version, manifest.version,
                        manifest_url, manifest.image_url, channel);
      return;
    }
    result.update_available = comparison == OtaVersionComparison::NEWER;
    result.busy = false;
    result.state = result.update_available ? EspectreOtaState::UPDATE_AVAILABLE : EspectreOtaState::UP_TO_DATE;
    result.message = result.update_available ? "update available" : "already up to date";
    ESPECTRE_LOGI(TAG, "%s current=%s target=%s", result.message.c_str(), current_version.c_str(),
             manifest.version.c_str());
    update_status_(result);
    return;
  }

  const std::string &image_url = manifest.image_url;
  const std::string &target_version = manifest.version;
  const OtaVersionComparison comparison = compare_ota_versions(target_version, current_version);
  if (comparison == OtaVersionComparison::UNORDERED) {
    set_error_status_("unrecognized or divergent firmware version", current_version, target_version,
                      manifest_url, image_url, channel);
    return;
  }
  if (comparison != OtaVersionComparison::NEWER) {
    EspectreOtaStatus result;
    result.state = EspectreOtaState::UP_TO_DATE;
    result.current_version = current_version;
    result.target_version = target_version;
    result.channel = channel;
    result.manifest_url = manifest_url;
    result.image_url = image_url;
    result.update_available = false;
    result.busy = false;
    result.message = comparison == OtaVersionComparison::SAME ? "already up to date" : "target is not newer";
    update_status_(result);
    return;
  }
  if (image_url.empty()) {
    set_error_status_("missing image_url", current_version, target_version, manifest_url, image_url, channel);
    return;
  }

  if (!request_prepare_for_update_()) {
    if (!shutdown_requested_.load(std::memory_order_acquire)) {
      set_error_status_("frontend did not quiesce for ota", current_version, target_version,
                        manifest_url, image_url, channel);
    }
    return;
  }

  EspectreOtaStatus downloading;
  downloading.state = EspectreOtaState::DOWNLOADING;
  downloading.busy = true;
  downloading.current_version = current_version;
  downloading.target_version = target_version;
  downloading.channel = channel;
  downloading.manifest_url = manifest_url;
  downloading.image_url = image_url;
  downloading.update_available = true;
  downloading.message = "starting https ota";
  update_status_(downloading);
  ESPECTRE_LOGI(TAG, "downloading %s", image_url.c_str());

  esp_http_client_config_t http_config{};
  fill_https_client_config(&http_config, image_url.c_str());

  esp_https_ota_config_t ota_config{};
  ota_config.http_config = &http_config;

  const esp_err_t err = esp_https_ota(&ota_config);
  if (err != ESP_OK) {
    set_error_status_(esp_err_to_name(err), current_version, target_version, manifest_url, image_url, channel);
    return;
  }

  EspectreOtaStatus ready;
  ready.state = EspectreOtaState::REBOOT_SCHEDULED;
  ready.busy = false;
  ready.current_version = current_version;
  ready.target_version = target_version;
  ready.channel = channel;
  ready.manifest_url = manifest_url;
  ready.image_url = image_url;
  ready.update_available = false;
  ready.message = "ota applied, rebooting";
  ESPECTRE_LOGI(TAG, "ota applied, rebooting to %s", target_version.c_str());
  update_status_(ready);

  vTaskDelay(pdMS_TO_TICKS(kPostSuccessDelayMs));
  esp_restart();
}

bool HttpsOtaService::begin_request_(const WorkerRequest &request) {
  if (!request.channel.empty() && !espectre_ota_channel_accepted(request.channel)) {
    ESPECTRE_LOGW(TAG, "invalid ota channel: %s", request.channel.c_str());
    return false;
  }

  if (!ensure_lock_() || worker_done_ == nullptr || prepare_done_ == nullptr ||
      shutdown_requested_.load(std::memory_order_acquire)) {
    return false;
  }

  xSemaphoreTake(lock_, portMAX_DELAY);
  if (worker_active_ || status_.busy) {
    xSemaphoreGive(lock_);
    return false;
  }
  worker_active_ = true;
  xSemaphoreGive(lock_);
  (void)xSemaphoreTake(worker_done_, 0);

  auto *context = new (std::nothrow) WorkerContext{this, request};
  if (context == nullptr) {
    xSemaphoreTake(lock_, portMAX_DELAY);
    worker_active_ = false;
    xSemaphoreGive(lock_);
    return false;
  }

  if (xTaskCreate(&HttpsOtaService::worker_entry_,
                  "espectre_ota",
                  kWorkerStackSize,
                  context,
                  kWorkerPriority,
                  nullptr) != pdPASS) {
    delete context;
    xSemaphoreTake(lock_, portMAX_DELAY);
    worker_active_ = false;
    xSemaphoreGive(lock_);
    return false;
  }
  return true;
}

bool HttpsOtaService::ensure_lock_() const {
  return lock_ != nullptr;
}

void HttpsOtaService::finish_worker_() {
  if (lock_ != nullptr) {
    xSemaphoreTake(lock_, portMAX_DELAY);
    if (worker_done_ != nullptr) {
      xSemaphoreGive(worker_done_);
    }
    worker_active_ = false;
    xSemaphoreGive(lock_);
  } else if (worker_done_ != nullptr) {
    xSemaphoreGive(worker_done_);
  }
}

bool HttpsOtaService::request_prepare_for_update_() {
  if (lock_ == nullptr || prepare_done_ == nullptr) {
    return false;
  }
  (void)xSemaphoreTake(prepare_done_, 0);
  xSemaphoreTake(lock_, portMAX_DELAY);
  prepare_callback_pending_ = true;
  xSemaphoreGive(lock_);

  while (!shutdown_requested_.load(std::memory_order_acquire)) {
    if (xSemaphoreTake(prepare_done_, pdMS_TO_TICKS(100U)) == pdTRUE) {
      return !shutdown_requested_.load(std::memory_order_acquire);
    }
  }
  return false;
}

void HttpsOtaService::update_status_(const EspectreOtaStatus &status) {
  EspectreOtaStatus normalized = status;
  normalized.default_channel = default_channel_;
  if (ensure_lock_()) {
    xSemaphoreTake(lock_, portMAX_DELAY);
    status_ = normalized;
    pending_status_ = normalized;
    status_callback_pending_ = true;
    xSemaphoreGive(lock_);
  } else {
    status_ = normalized;
  }
}

void HttpsOtaService::set_error_status_(const std::string &message,
                                        const std::string &current_version,
                                        const std::string &target_version,
                                        const std::string &manifest_url,
                                        const std::string &image_url,
                                        const std::string &channel) {
  EspectreOtaStatus status;
  status.state = EspectreOtaState::ERROR;
  status.busy = false;
  status.current_version = current_version;
  status.target_version = target_version;
  status.manifest_url = manifest_url;
  status.image_url = image_url;
  status.channel = channel;
  status.message = message;
  status.update_available = false;
  ESPECTRE_LOGE(TAG, "failed: %s channel=%s url=%s", message.c_str(), channel.c_str(),
           image_url.empty() ? manifest_url.c_str() : image_url.c_str());
  update_status_(status);
}

bool HttpsOtaService::fetch_https_text_(const std::string &url, std::string *body, std::string *error) const {
  if (body == nullptr) {
    return false;
  }
  body->clear();
  if (url.empty()) {
    if (error != nullptr) {
      *error = "empty url";
    }
    return false;
  }

  ManifestFetchContext context{body, error};
  esp_http_client_config_t config{};
  fill_https_client_config(&config, url.c_str());
  config.event_handler = manifest_http_event;
  config.user_data = &context;

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    if (error != nullptr) {
      *error = "esp_http_client_init failed";
    }
    return false;
  }

  const esp_err_t err = esp_http_client_perform(client);
  if (err != ESP_OK) {
    if (error != nullptr && error->empty()) {
      *error = esp_err_to_name(err);
    }
    esp_http_client_cleanup(client);
    return false;
  }

  const int status_code = esp_http_client_get_status_code(client);
  if (status_code < 200 || status_code >= 300) {
    if (error != nullptr) {
      *error = "manifest http status " + std::to_string(status_code);
    }
    esp_http_client_cleanup(client);
    return false;
  }
  esp_http_client_cleanup(client);
  return true;
}

bool HttpsOtaService::parse_manifest_(std::string body, const std::string &channel,
                                     ManifestInfo *manifest, std::string *error) const {
  if (manifest == nullptr) {
    return false;
  }
  *manifest = {};
  const auto fail = [error](const char *message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  std::vector<JsonObjectField> fields;
  if (!parse_json_object_fields(body, &fields, error)) {
    return false;
  }
  const auto *schema = find_json_object_field(fields, "schema_version");
  const auto *published_channel = find_json_object_field(fields, "channel");
  const auto *version = find_json_object_field(fields, "version");
  if (schema == nullptr || schema->type != JsonValueType::NUMBER || schema->value != "1" ||
      published_channel == nullptr || published_channel->type != JsonValueType::STRING ||
      published_channel->value != channel || version == nullptr ||
      version->type != JsonValueType::STRING || version->value.empty()) {
    return fail("invalid manifest metadata");
  }
  const std::string target_version = version->value;
  // Retain only the selected subtree at each level, releasing the full catalog.
  const auto take_field = [&fields, &body](const char *name, JsonValueType type) {
    for (auto &field : fields) {
      if (field.name == name && field.type == type) {
        body = std::move(field.value);
        fields.clear();
        return true;
      }
    }
    return false;
  };
  if (!take_field("frontends", JsonValueType::OBJECT) ||
      !parse_json_object_fields(body, &fields, error) ||
      !take_field(frontend_.c_str(), JsonValueType::OBJECT) ||
      !parse_json_object_fields(body, &fields, error) ||
      !take_field("artifacts", JsonValueType::ARRAY)) {
    return fail("missing frontend artifacts");
  }
  std::vector<std::vector<JsonObjectField>> artifacts;
  if (!parse_json_array_objects(body, &artifacts, error)) {
    return false;
  }
  std::string image_url;
  for (const auto &artifact : artifacts) {
    const auto *chip = find_json_object_field(artifact, "chip");
    const auto *build_type = find_json_object_field(artifact, "build_type");
    if (chip == nullptr || chip->type != JsonValueType::STRING || chip->value != chip_ ||
        build_type == nullptr || build_type->type != JsonValueType::STRING || build_type->value != "ota") {
      continue;
    }
    const auto *url = find_json_object_field(artifact, "url");
    if (url == nullptr || url->type != JsonValueType::STRING || url->value.compare(0, 8, "https://") != 0) {
      return fail("invalid ota image url");
    }
    if (!image_url.empty()) {
      return fail("ambiguous ota image");
    }
    image_url = url->value;
  }
  if (image_url.empty()) {
    return fail("missing ota image for chip");
  }
  manifest->version = target_version;
  manifest->image_url = std::move(image_url);
  return true;
}

}  // namespace espectre
