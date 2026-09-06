// SPDX-License-Identifier: GPL-3.0-only
// Commercial licensing available under separate agreement; see LICENSING.md.

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mdns.h"
#include "py/mperrno.h"
#include "py/mpprint.h"
#include "py/objstr.h"
#include "py/runtime.h"
#include "sdkconfig.h"

#define DIRECT_BASE_PATH "/espectre/v1"
#define DIRECT_RESOURCE_PATH "/espectre/v1/*"
#define DIRECT_EVENTS_PATH "/espectre/v1/events"
#define DIRECT_MAX_REQUEST_BYTES (512)
#define DIRECT_MAX_EVENT_BYTES (4096)
#define DIRECT_MAX_STATUS_BYTES (384)
#define DIRECT_MAX_DIAGNOSTICS_BYTES (2048)
#define DIRECT_MAX_PROTOCOL_VERSION_BYTES (16)
#define DIRECT_MAX_DNS_SD_SCHEMA_VERSION_BYTES (8)
#define DIRECT_MAX_DEVICE_ID_BYTES (64)
#define DIRECT_MAX_COMMAND_ID_BYTES (64)
#define DIRECT_MAX_COMMAND_BYTES (48)
#define DIRECT_MAX_EVENT_NAME_BYTES (32)
#define DIRECT_MAX_FIRMWARE_VERSION_BYTES (48)
#define DIRECT_DIAGNOSTICS_BUFFER_COUNT (2)
#define DIRECT_EVENT_FRAME_CAPACITY (DIRECT_MAX_EVENT_BYTES + DIRECT_MAX_EVENT_NAME_BYTES + 50)
#define DIRECT_MAX_REQUESTS_PER_SECOND (20)
#define DIRECT_REQUEST_WINDOW_US (1000000ULL)
#define DIRECT_HEARTBEAT_INTERVAL_US (10000000ULL)

typedef struct {
  httpd_handle_t server;
  httpd_req_t *event_request;
  esp_timer_handle_t heartbeat_timer;
  SemaphoreHandle_t lock;
  char *capabilities;
  char *info;
  char status[DIRECT_MAX_STATUS_BYTES + 1];
  char status_staging[DIRECT_MAX_STATUS_BYTES + 1];
  char *config;
  char *wifi;
  char diagnostics[DIRECT_DIAGNOSTICS_BUFFER_COUNT][DIRECT_MAX_DIAGNOSTICS_BYTES + 1];
  uint8_t diagnostics_active;
  uint8_t diagnostics_readers[DIRECT_DIAGNOSTICS_BUFFER_COUNT];
  char *device_id;
  char *protocol_version;
  char *dns_sd_schema_version;
  char event_frame[DIRECT_EVENT_FRAME_CAPACITY];
  size_t event_frame_length;
  bool event_work_pending;
  bool heartbeat_work_pending;
  bool recalibration_pending;
  bool recalibration_active;
  bool stopping;
  bool mdns_service_added;
  uint64_t request_window_started_us;
  uint16_t request_count;
  volatile uint32_t accepted_connections;
  volatile uint32_t rejected_connections;
  volatile uint32_t malformed_requests;
  volatile uint32_t oversized_requests;
  volatile uint32_t rate_limited_requests;
  volatile uint32_t dropped_motion_events;
  volatile uint32_t send_failures;
} native_direct_state_t;

static native_direct_state_t direct_state;
static const char *const NATIVE_DIRECT_TAG = "MicroDirect";
static const char *const DIRECT_HEARTBEAT_FRAME = ": heartbeat\n\n";

static char *direct_replace_string(char **target, const char *source, size_t length);
static void direct_close_event_stream(bool finish_response);

static void direct_increment(volatile uint32_t *counter) {
  __atomic_fetch_add(counter, 1U, __ATOMIC_RELAXED);
}

static uint32_t direct_counter(const volatile uint32_t *counter) {
  return __atomic_load_n(counter, __ATOMIC_RELAXED);
}

static const char *direct_peer_disconnect_reason(int error) {
  switch (error) {
    case ECONNRESET:
      return "connection reset";
    case ENOTCONN:
      return "socket not connected";
    case EPIPE:
      return "broken pipe";
    default:
      return NULL;
  }
}

static void direct_handle_event_send_failure(int send_errno) {
  const char *reason = direct_peer_disconnect_reason(send_errno);
  if (reason != NULL) {
    ESP_LOGI(
        NATIVE_DIRECT_TAG,
        "Direct SSE peer disconnected (reason=%s, errno=%d)",
        reason,
        send_errno);
    return;
  }
  direct_increment(&direct_state.send_failures);
}

static void direct_reset_counters(void) {
  __atomic_store_n(&direct_state.accepted_connections, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&direct_state.rejected_connections, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&direct_state.malformed_requests, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&direct_state.oversized_requests, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&direct_state.rate_limited_requests, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&direct_state.dropped_motion_events, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&direct_state.send_failures, 0U, __ATOMIC_RELAXED);
  direct_state.heartbeat_work_pending = false;
  direct_state.recalibration_pending = false;
  direct_state.recalibration_active = false;
  direct_state.request_window_started_us = 0U;
  direct_state.request_count = 0U;
}

static bool direct_request_allowed(void) {
  uint64_t now_us = (uint64_t) esp_timer_get_time();
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  if (
      direct_state.request_window_started_us == 0U ||
      now_us - direct_state.request_window_started_us >= DIRECT_REQUEST_WINDOW_US) {
    direct_state.request_window_started_us = now_us;
    direct_state.request_count = 0U;
  }
  bool allowed = direct_state.request_count < DIRECT_MAX_REQUESTS_PER_SECOND;
  if (allowed) {
    direct_state.request_count += 1U;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  return allowed;
}

typedef struct {
  mp_print_ext_t print;
  char *buffer;
  size_t capacity;
  size_t length;
  bool overflow;
} direct_json_buffer_t;

typedef enum {
  DIRECT_DIAGNOSTICS_UPDATED,
  DIRECT_DIAGNOSTICS_BUSY,
  DIRECT_DIAGNOSTICS_TOO_LARGE,
} direct_diagnostics_update_t;

typedef struct {
  const char *data;
  char *owned;
  uint8_t diagnostics_index;
  bool diagnostics_pinned;
} direct_command_snapshot_t;

static void direct_json_write(void *data, const char *source, size_t length) {
  direct_json_buffer_t *writer = data;
  if (writer->overflow || length > writer->capacity - writer->length) {
    writer->overflow = true;
    return;
  }
  if (writer->buffer != NULL) {
    memcpy(writer->buffer + writer->length, source, length);
  }
  writer->length += length;
}

static bool direct_encode_json(char *target, size_t capacity, mp_obj_t payload, size_t *length) {
  direct_json_buffer_t writer = {
      .print = {
          .base = {
              .data = NULL,
              .print_strn = direct_json_write,
          },
          .item_separator = ",",
          .key_separator = ":",
      },
      .buffer = target,
      .capacity = capacity,
      .length = 0,
      .overflow = false,
  };
  writer.print.base.data = &writer;
  mp_obj_print_helper(&writer.print.base, payload, PRINT_JSON);
  if (writer.overflow) {
    target[0] = '\0';
    return false;
  }
  target[writer.length] = '\0';
  if (length != NULL) {
    *length = writer.length;
  }
  return true;
}

static bool direct_encode_status_object(mp_obj_t payload) {
  size_t length = 0;
  if (!direct_encode_json(
          direct_state.status_staging,
          DIRECT_MAX_STATUS_BYTES,
          payload,
          &length)) {
    return false;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  memcpy(direct_state.status, direct_state.status_staging, length + 1);
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  return true;
}

static char *direct_replace_json_string(char **target, mp_obj_t payload) {
  if (mp_obj_is_str_or_bytes(payload)) {
    size_t length;
    const char *source = mp_obj_str_get_data(payload, &length);
    return direct_replace_string(target, source, length);
  }

  direct_json_buffer_t counter = {
      .print = {
          .base = {
              .data = NULL,
              .print_strn = direct_json_write,
          },
          .item_separator = ",",
          .key_separator = ":",
      },
      .buffer = NULL,
      .capacity = SIZE_MAX,
      .length = 0,
      .overflow = false,
  };
  counter.print.base.data = &counter;
  mp_obj_print_helper(&counter.print.base, payload, PRINT_JSON);
  char *replacement = malloc(counter.length + 1);
  if (replacement == NULL ||
      !direct_encode_json(replacement, counter.length, payload, NULL)) {
    free(replacement);
    return NULL;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  char *previous = *target;
  *target = replacement;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  free(previous);
  return replacement;
}

static bool direct_identifier_valid(const char *value, size_t max_length, bool command) {
  if (value == NULL) {
    return false;
  }
  size_t length = strlen(value);
  if (length == 0 || length > max_length) {
    return false;
  }
  for (size_t index = 0; index < length; ++index) {
    char character = value[index];
    bool alphanumeric =
        (character >= '0' && character <= '9') ||
        (character >= 'A' && character <= 'Z') ||
        (character >= 'a' && character <= 'z');
    if (
        !alphanumeric && character != '_' && character != '-' && character != '.' &&
        (command || character != ':')) {
      return false;
    }
  }
  return true;
}

#if defined(CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED) && CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED
static bool direct_valid_loopback_port_suffix(const char *suffix) {
  if (suffix == NULL || suffix[0] == '\0') {
    return true;
  }
  if (suffix[0] != ':' || suffix[1] == '\0') {
    return false;
  }
  uint32_t port = 0U;
  for (size_t index = 1U; suffix[index] != '\0'; ++index) {
    unsigned char character = (unsigned char) suffix[index];
    if (!isdigit(character)) {
      return false;
    }
    port = port * 10U + (uint32_t) (character - (unsigned char) '0');
    if (port > 65535U) {
      return false;
    }
  }
  return port > 0U;
}
#endif

static bool direct_http_loopback_origin(const char *origin) {
#if defined(CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED) && CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED
  if (origin == NULL) {
    return false;
  }
  char normalized[96];
  size_t length = strlen(origin);
  if (length == 0U || length >= sizeof(normalized)) {
    return false;
  }
  for (size_t index = 0U; index <= length; ++index) {
    normalized[index] = (char) tolower((unsigned char) origin[index]);
  }
  const char *prefixes[] = {
      "http://localhost", "http://127.0.0.1", "http://[::1]",
  };
  for (size_t index = 0U; index < MP_ARRAY_SIZE(prefixes); ++index) {
    size_t prefix_length = strlen(prefixes[index]);
    if (
        strncmp(normalized, prefixes[index], prefix_length) == 0 &&
        direct_valid_loopback_port_suffix(normalized + prefix_length)) {
      return true;
    }
  }
#else
  (void) origin;
#endif
  return false;
}

static bool direct_origin_allowed(const char *origin) {
  return origin != NULL && origin[0] != '\0' &&
      (strcmp(origin, "https://espectre.dev") == 0 ||
      strcmp(origin, "https://www.espectre.dev") == 0 ||
      strcmp(origin, "https://test.espectre.dev") == 0 ||
      direct_http_loopback_origin(origin));
}

static bool direct_read_origin(httpd_req_t *request, char *origin, size_t capacity) {
  size_t length = httpd_req_get_hdr_value_len(request, "Origin");
  if (length == 0) {
    return false;
  }
  if (length >= capacity || httpd_req_get_hdr_value_str(request, "Origin", origin, capacity) != ESP_OK) {
    return false;
  }
  return direct_origin_allowed(origin);
}

static bool direct_set_cors(httpd_req_t *request) {
  char origin[96];
  if (!direct_read_origin(request, origin, sizeof(origin))) {
    return false;
  }
  if (origin[0] != '\0') {
    (void) httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", origin);
  }
  (void) httpd_resp_set_hdr(request, "Vary", "Origin");
  return true;
}

static char *direct_replace_string(char **target, const char *source, size_t length) {
  char *replacement = malloc(length + 1);
  if (replacement == NULL) {
    return NULL;
  }
  memcpy(replacement, source, length);
  replacement[length] = '\0';
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  char *previous = *target;
  *target = replacement;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  free(previous);
  return replacement;
}

static bool direct_replace_status(const char *source, size_t length) {
  if (length > DIRECT_MAX_STATUS_BYTES) {
    return false;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  memcpy(direct_state.status, source, length);
  direct_state.status[length] = '\0';
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  return true;
}

static bool direct_reserve_diagnostics_buffer(uint8_t *index) {
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  uint8_t candidate = direct_state.diagnostics_active ^ 1U;
  bool available = direct_state.diagnostics_readers[candidate] == 0U;
  if (available) {
    *index = candidate;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  return available;
}

static void direct_commit_diagnostics_buffer(uint8_t index) {
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  direct_state.diagnostics_active = index;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
}

static direct_diagnostics_update_t direct_replace_diagnostics(
    const char *source,
    size_t length) {
  if (length > DIRECT_MAX_DIAGNOSTICS_BYTES) {
    return DIRECT_DIAGNOSTICS_TOO_LARGE;
  }
  uint8_t index = 0U;
  if (!direct_reserve_diagnostics_buffer(&index)) {
    return DIRECT_DIAGNOSTICS_BUSY;
  }
  memcpy(direct_state.diagnostics[index], source, length);
  direct_state.diagnostics[index][length] = '\0';
  direct_commit_diagnostics_buffer(index);
  return DIRECT_DIAGNOSTICS_UPDATED;
}

static bool direct_replace_status_object(mp_obj_t payload) {
  if (mp_obj_is_str_or_bytes(payload)) {
    size_t length;
    const char *source = mp_obj_str_get_data(payload, &length);
    return direct_replace_status(source, length);
  }
  return direct_encode_status_object(payload);
}

static direct_diagnostics_update_t direct_replace_diagnostics_object(mp_obj_t payload) {
  if (mp_obj_is_str_or_bytes(payload)) {
    size_t length;
    const char *source = mp_obj_str_get_data(payload, &length);
    return direct_replace_diagnostics(source, length);
  }
  uint8_t index = 0U;
  if (!direct_reserve_diagnostics_buffer(&index)) {
    return DIRECT_DIAGNOSTICS_BUSY;
  }
  if (!direct_encode_json(
          direct_state.diagnostics[index],
          DIRECT_MAX_DIAGNOSTICS_BYTES,
          payload,
          NULL)) {
    return DIRECT_DIAGNOSTICS_TOO_LARGE;
  }
  direct_commit_diagnostics_buffer(index);
  return DIRECT_DIAGNOSTICS_UPDATED;
}

static esp_err_t direct_send_result(
    httpd_req_t *request,
    const char *command_id,
    const char *command,
    bool accepted,
    const char *code,
    const char *message,
    const char *snapshot,
    const char *http_status) {
  (void) command_id;
  (void) command;
  char prefix[512];
  int prefix_length = snprintf(
      prefix,
      sizeof(prefix),
      "{\"accepted\":%s,"
      "\"code\":\"%s\",\"message\":\"%s\"%s",
      accepted ? "true" : "false",
      code,
      message,
      snapshot == NULL ? "}" : ",\"data\":");
  if (prefix_length < 0 || (size_t) prefix_length >= sizeof(prefix)) {
    return ESP_FAIL;
  }
  if (!direct_set_cors(request)) {
    httpd_resp_set_status(request, "403 Forbidden");
    return httpd_resp_sendstr(request, "Forbidden");
  }
  httpd_resp_set_status(request, http_status);
  httpd_resp_set_type(request, "application/json");
  (void) httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  esp_err_t result = httpd_resp_send_chunk(request, prefix, (size_t) prefix_length);
  if (result == ESP_OK && snapshot != NULL) {
    result = httpd_resp_send_chunk(request, snapshot, strlen(snapshot));
  }
  if (result == ESP_OK && snapshot != NULL) {
    result = httpd_resp_send_chunk(request, "}", 1);
  }
  if (result == ESP_OK) {
    result = httpd_resp_send_chunk(request, NULL, 0);
  }
  return result;
}

static direct_command_snapshot_t direct_acquire_command_snapshot(const char *command) {
  direct_command_snapshot_t result = {0};
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  const char *snapshot = NULL;
  if (strcmp(command, "capabilities") == 0) {
    snapshot = direct_state.capabilities;
  } else if (strcmp(command, "device") == 0) {
    snapshot = direct_state.info;
  } else if (strcmp(command, "health") == 0) {
    snapshot = direct_state.status;
  } else if (strcmp(command, "sensing") == 0) {
    snapshot = direct_state.config;
  } else if (strcmp(command, "wifi") == 0) {
    snapshot = direct_state.wifi;
  } else if (strcmp(command, "diagnostics") == 0) {
    uint8_t index = direct_state.diagnostics_active;
    direct_state.diagnostics_readers[index] += 1U;
    result.data = direct_state.diagnostics[index];
    result.diagnostics_index = index;
    result.diagnostics_pinned = true;
  }
  if (!result.diagnostics_pinned && snapshot != NULL) {
    result.owned = strdup(snapshot);
    result.data = result.owned;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  return result;
}

static void direct_release_command_snapshot(direct_command_snapshot_t *snapshot) {
  if (snapshot->diagnostics_pinned) {
    if (direct_state.lock != NULL) {
      xSemaphoreTake(direct_state.lock, portMAX_DELAY);
    }
    uint8_t index = snapshot->diagnostics_index;
    if (direct_state.diagnostics_readers[index] > 0U) {
      direct_state.diagnostics_readers[index] -= 1U;
    }
    if (direct_state.lock != NULL) {
      xSemaphoreGive(direct_state.lock);
    }
  }
  free(snapshot->owned);
  *snapshot = (direct_command_snapshot_t) {0};
}

static bool direct_queue_recalibration(void) {
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  bool accepted = !direct_state.recalibration_pending && !direct_state.recalibration_active;
  if (accepted) {
    direct_state.recalibration_pending = true;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  return accepted;
}

static bool direct_request_size_allowed(httpd_req_t *request) {
  if (request->content_len <= DIRECT_MAX_REQUEST_BYTES) {
    return true;
  }
  direct_increment(&direct_state.oversized_requests);
  httpd_resp_set_status(request, "413 Payload Too Large");
  (void) httpd_resp_set_hdr(request, "Connection", "close");
  (void) httpd_resp_sendstr(request, "Direct request body is too large");
  return false;
}

typedef bool (*direct_parameter_validator_t)(const cJSON *parameters, const char **error);

typedef struct {
  httpd_method_t method;
  const char *path;
  const char *command;
  direct_parameter_validator_t validate;
} direct_route_t;

static bool direct_validate_no_parameters(const cJSON *parameters, const char **error) {
  if (parameters->child != NULL) {
    *error = "unknown command parameter";
    return false;
  }
  return true;
}

static const direct_route_t direct_routes[] = {
    {HTTP_GET, "/espectre/v1/capabilities", "capabilities", direct_validate_no_parameters},
    {HTTP_GET, "/espectre/v1/device", "device", direct_validate_no_parameters},
    {HTTP_GET, "/espectre/v1/health", "health", direct_validate_no_parameters},
    {HTTP_GET, "/espectre/v1/sensing", "sensing", direct_validate_no_parameters},
    {HTTP_GET, "/espectre/v1/wifi", "wifi", direct_validate_no_parameters},
    {HTTP_GET, "/espectre/v1/diagnostics", "diagnostics", direct_validate_no_parameters},
    {HTTP_POST, "/espectre/v1/sensing/calibrations", "recalibrate", direct_validate_no_parameters},
};

static bool direct_json_input_allowed(const char *body, size_t length) {
  unsigned depth = 0U;
  bool quoted = false;
  bool escaped = false;
  for (size_t i = 0U; i < length; ++i) {
    char token = body[i];
    if ((unsigned char) token < 0x20U &&
        (quoted || (token != '\t' && token != '\r' && token != '\n'))) return false;
    if (quoted) {
      if (escaped) escaped = false;
      else if (token == '\\') escaped = true;
      else if (token == '"') quoted = false;
    } else if (token == '"') {
      quoted = true;
    } else if (token == '{' || token == '[') {
      // Bound cJSON recursion on the HTTP server task's stack.
      if (++depth > 8U) return false;
    } else if ((token == '}' || token == ']') && depth > 0U) {
      --depth;
    }
  }
  return true;
}

static bool direct_validate_request(httpd_req_t *request, const direct_route_t *route) {
  char body[DIRECT_MAX_REQUEST_BYTES + 1];
  size_t received = 0U;
  while (received < request->content_len) {
    int count = httpd_req_recv(request, body + received, request->content_len - received);
    if (count <= 0) {
      direct_increment(&direct_state.malformed_requests);
      httpd_resp_set_status(request, "400 Bad Request");
      (void) httpd_resp_set_hdr(request, "Connection", "close");
      (void) httpd_resp_sendstr(request, "incomplete Direct request");
      return false;
    }
    received += (size_t) count;
  }
  body[received] = '\0';
  if (received > 0U) {
    char content_type[96];
    if (httpd_req_get_hdr_value_str(request, "Content-Type", content_type, sizeof(content_type)) != ESP_OK ||
        strncmp(content_type, "application/json", strlen("application/json")) != 0) {
      httpd_resp_set_status(request, "415 Unsupported Media Type");
      (void) httpd_resp_sendstr(request, "application/json required");
      return false;
    }
  }
  // Empty bodies represent an empty parameter object, as in the C++ protocol.
  const char *payload = received == 0U ? "{}" : body;
  size_t length = received == 0U ? 2U : received;
  cJSON *parameters = memchr(payload, '\0', length) == NULL && direct_json_input_allowed(payload, length)
      ? cJSON_ParseWithLengthOpts(payload, length + 1U, NULL, true) : NULL;
  const char *error = "invalid Direct JSON object";
  bool valid = cJSON_IsObject(parameters);
  if (valid) valid = route->validate(parameters, &error);
  cJSON_Delete(parameters);
  if (!valid) {
    direct_increment(&direct_state.malformed_requests);
    (void) direct_send_result(request, "", route->command, false, "invalid_params",
                              error, NULL, "400 Bad Request");
  }
  return valid;
}

static esp_err_t direct_request_handler(httpd_req_t *request) {
  if (!direct_set_cors(request)) {
    httpd_resp_set_status(request, "403 Forbidden");
    return httpd_resp_sendstr(request, "Forbidden");
  }
  if (!direct_request_size_allowed(request)) {
    // Returning an error closes the session without draining an oversized body.
    return ESP_FAIL;
  }
  if (!direct_request_allowed()) {
    direct_increment(&direct_state.rate_limited_requests);
    httpd_resp_set_status(request, "429 Too Many Requests");
    return httpd_resp_sendstr(request, "Direct request rate limit reached");
  }
  const direct_route_t *route = NULL;
  for (size_t i = 0U; i < sizeof(direct_routes) / sizeof(direct_routes[0]); ++i) {
    if (request->method == direct_routes[i].method && strcmp(request->uri, direct_routes[i].path) == 0) {
      route = &direct_routes[i];
      break;
    }
  }
  if (route != NULL && !direct_validate_request(request, route)) return ESP_FAIL;
  const char *command = route != NULL ? route->command : NULL;
  bool recalibrate = route != NULL && request->method == HTTP_POST;
  bool query = route != NULL && request->method == HTTP_GET;
  direct_command_snapshot_t snapshot = query
      ? direct_acquire_command_snapshot(command)
      : (direct_command_snapshot_t) {0};
  esp_err_t result;
  if (recalibrate && direct_queue_recalibration()) {
    result = direct_send_result(
        request, "", "recalibrate", true, "ok",
        "recalibration queued", NULL, "202 Accepted");
  } else if (recalibrate) {
    result = direct_send_result(
        request, "", "recalibrate", false, "busy",
        "recalibration is already pending or active", NULL, "409 Conflict");
  } else if (!query) {
    result = direct_send_result(
        request, "", "", false, "unsupported",
        "resource or method is not supported", NULL, "404 Not Found");
  } else if (snapshot.data == NULL) {
    result = direct_send_result(
        request, "", command, false, "unavailable",
        "snapshot is unavailable", NULL, "503 Service Unavailable");
  } else {
    httpd_resp_set_type(request, "application/json");
    (void) httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    result = httpd_resp_sendstr(request, snapshot.data);
  }
  direct_release_command_snapshot(&snapshot);
  return result;
}

static esp_err_t direct_options_handler(httpd_req_t *request) {
  if (!direct_set_cors(request)) {
    httpd_resp_set_status(request, "403 Forbidden");
    return httpd_resp_sendstr(request, "Forbidden");
  }
  if (!direct_request_size_allowed(request)) {
    // Returning an error closes the session without draining an oversized body.
    return ESP_FAIL;
  }
  (void) httpd_resp_set_hdr(request, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  (void) httpd_resp_set_hdr(request, "Access-Control-Allow-Headers", "Content-Type");
  char private_network[8];
  if (
      httpd_req_get_hdr_value_str(
          request,
          "Access-Control-Request-Private-Network",
          private_network,
          sizeof(private_network)) == ESP_OK &&
      strcmp(private_network, "true") == 0) {
    (void) httpd_resp_set_hdr(request, "Access-Control-Allow-Private-Network", "true");
  }
  httpd_resp_set_status(request, "204 No Content");
  return httpd_resp_send(request, NULL, 0);
}

static void direct_stop_heartbeat(void) {
  if (direct_state.heartbeat_timer != NULL) {
    (void) esp_timer_stop(direct_state.heartbeat_timer);
  }
}

static void direct_send_heartbeat_work(void *argument) {
  (void) argument;
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  httpd_req_t *request = direct_state.event_request;
  bool active = request != NULL && !direct_state.stopping;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  esp_err_t result = active
      ? httpd_resp_send_chunk(request, DIRECT_HEARTBEAT_FRAME, strlen(DIRECT_HEARTBEAT_FRAME))
      : ESP_ERR_INVALID_STATE;
  int send_errno = result == ESP_OK ? 0 : errno;
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  active = active && direct_state.event_request == request;
  direct_state.heartbeat_work_pending = false;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  if (active && result != ESP_OK) {
    direct_handle_event_send_failure(send_errno);
    direct_close_event_stream(false);
  }
}

static void direct_heartbeat_timer_callback(void *argument) {
  (void) argument;
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  httpd_handle_t server = direct_state.server;
  bool queue = server != NULL && direct_state.event_request != NULL &&
      !direct_state.stopping && !direct_state.heartbeat_work_pending;
  if (queue) {
    direct_state.heartbeat_work_pending = true;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  if (!queue) {
    return;
  }
  if (httpd_queue_work(server, direct_send_heartbeat_work, NULL) != ESP_OK) {
    if (direct_state.lock != NULL) {
      xSemaphoreTake(direct_state.lock, portMAX_DELAY);
    }
    direct_state.heartbeat_work_pending = false;
    if (direct_state.lock != NULL) {
      xSemaphoreGive(direct_state.lock);
    }
    direct_increment(&direct_state.send_failures);
  }
}

static bool direct_start_heartbeat(void) {
  if (direct_state.heartbeat_timer == NULL) {
    return false;
  }
  direct_stop_heartbeat();
  return esp_timer_start_periodic(
             direct_state.heartbeat_timer,
             DIRECT_HEARTBEAT_INTERVAL_US) == ESP_OK;
}

static esp_err_t direct_events_handler(httpd_req_t *request) {
  if (!direct_set_cors(request)) {
    httpd_resp_set_status(request, "403 Forbidden");
    return httpd_resp_sendstr(request, "Forbidden");
  }
  if (!direct_request_size_allowed(request)) {
    // Returning an error closes the session without draining an oversized body.
    return ESP_FAIL;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  bool unavailable = direct_state.stopping || direct_state.event_request != NULL;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  if (unavailable) {
    direct_increment(&direct_state.rejected_connections);
    httpd_resp_set_status(request, "503 Service Unavailable");
    return httpd_resp_sendstr(request, "event stream is already in use");
  }

  httpd_req_t *async_request = NULL;
  if (httpd_req_async_handler_begin(request, &async_request) != ESP_OK || async_request == NULL) {
    direct_increment(&direct_state.rejected_connections);
    return ESP_FAIL;
  }
  httpd_resp_set_type(async_request, "text/event-stream");
  (void) httpd_resp_set_hdr(async_request, "Cache-Control", "no-store");
  (void) httpd_resp_set_hdr(async_request, "Connection", "keep-alive");
  if (!direct_set_cors(async_request) ||
      httpd_resp_send_chunk(async_request, ": connected\n\n", strlen(": connected\n\n")) != ESP_OK) {
    direct_increment(&direct_state.send_failures);
    httpd_req_async_handler_complete(async_request);
    return ESP_FAIL;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  if (direct_state.stopping || direct_state.event_request != NULL) {
    if (direct_state.lock != NULL) {
      xSemaphoreGive(direct_state.lock);
    }
    direct_increment(&direct_state.rejected_connections);
    httpd_req_async_handler_complete(async_request);
    return ESP_ERR_INVALID_STATE;
  }
  direct_state.event_request = async_request;
  direct_increment(&direct_state.accepted_connections);
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  if (!direct_start_heartbeat()) {
    direct_increment(&direct_state.send_failures);
    direct_close_event_stream(true);
    return ESP_FAIL;
  }
  return ESP_OK;
}

static void direct_close_event_stream(bool finish_response) {
  direct_stop_heartbeat();
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  httpd_req_t *request = direct_state.event_request;
  direct_state.event_request = NULL;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  if (request != NULL) {
    if (finish_response) {
      (void) httpd_resp_send_chunk(request, NULL, 0);
    }
    httpd_req_async_handler_complete(request);
  }
}

static void direct_send_event_work(void *argument) {
  (void) argument;
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  httpd_req_t *request = direct_state.event_request;
  size_t length = direct_state.event_frame_length;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }

  // event_work_pending prevents the producer from changing the static frame
  // until this send completes, avoiding a heap allocation on every sample.
  esp_err_t result = request == NULL || length == 0
      ? ESP_ERR_INVALID_STATE
      : httpd_resp_send_chunk(request, direct_state.event_frame, length);
  int send_errno = result == ESP_OK ? 0 : errno;

  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  bool active = direct_state.event_request == request;
  direct_state.event_work_pending = false;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  if (active && result != ESP_OK) {
    direct_handle_event_send_failure(send_errno);
    direct_close_event_stream(false);
  }
}

static bool direct_wait_for_event_work(void) {
  for (size_t attempt = 0; attempt < 200; ++attempt) {
    if (direct_state.lock != NULL) {
      xSemaphoreTake(direct_state.lock, portMAX_DELAY);
    }
    bool pending = direct_state.event_work_pending || direct_state.heartbeat_work_pending;
    if (direct_state.lock != NULL) {
      xSemaphoreGive(direct_state.lock);
    }
    if (!pending) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return false;
}

static void direct_free_snapshots(void) {
  free(direct_state.capabilities);
  free(direct_state.info);
  free(direct_state.config);
  free(direct_state.wifi);
  free(direct_state.device_id);
  free(direct_state.protocol_version);
  free(direct_state.dns_sd_schema_version);
  direct_state.capabilities = NULL;
  direct_state.info = NULL;
  direct_state.status[0] = '\0';
  direct_state.status_staging[0] = '\0';
  direct_state.config = NULL;
  direct_state.wifi = NULL;
  for (size_t index = 0; index < DIRECT_DIAGNOSTICS_BUFFER_COUNT; ++index) {
    direct_state.diagnostics[index][0] = '\0';
    direct_state.diagnostics_readers[index] = 0U;
  }
  direct_state.diagnostics_active = 0U;
  direct_state.device_id = NULL;
  direct_state.protocol_version = NULL;
  direct_state.dns_sd_schema_version = NULL;
  direct_state.event_frame_length = 0;
  direct_state.event_work_pending = false;
  direct_state.heartbeat_work_pending = false;
  direct_state.recalibration_pending = false;
  direct_state.recalibration_active = false;
}

static void direct_stop_native(void) {
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  direct_state.stopping = true;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  direct_stop_heartbeat();
  bool event_idle = direct_wait_for_event_work();
  if (event_idle) {
    direct_close_event_stream(true);
  } else {
    ESP_LOGW(NATIVE_DIRECT_TAG, "Timed out waiting for Direct event send to finish");
  }
  if (direct_state.server != NULL) {
    httpd_stop(direct_state.server);
    direct_state.server = NULL;
  }
  if (direct_state.heartbeat_timer != NULL) {
    (void) esp_timer_delete(direct_state.heartbeat_timer);
    direct_state.heartbeat_timer = NULL;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  direct_state.event_request = NULL;
  direct_state.event_work_pending = false;
  direct_state.heartbeat_work_pending = false;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  if (direct_state.mdns_service_added) {
    esp_err_t remove_result = mdns_service_remove("_espectre", "_tcp");
    if (remove_result == ESP_OK) {
      // The mDNS component queues removals. Wait for that bounded action
      // before allowing Direct to add the same service after recovery.
      for (size_t attempt = 0; attempt < 50; ++attempt) {
        if (!mdns_service_exists("_espectre", "_tcp", NULL)) {
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
    direct_state.mdns_service_added = false;
  }
  direct_free_snapshots();
}

static bool direct_register_http_handlers(void) {
  const httpd_uri_t get_handler = {
      .uri = DIRECT_RESOURCE_PATH,
      .method = HTTP_GET,
      .handler = direct_request_handler,
  };
  const httpd_uri_t post_handler = {
      .uri = DIRECT_RESOURCE_PATH,
      .method = HTTP_POST,
      .handler = direct_request_handler,
  };
  const httpd_uri_t options_handler = {
      .uri = DIRECT_RESOURCE_PATH,
      .method = HTTP_OPTIONS,
      .handler = direct_options_handler,
  };
  const httpd_uri_t events_handler = {
      .uri = DIRECT_EVENTS_PATH,
      .method = HTTP_GET,
      .handler = direct_events_handler,
  };
  const httpd_uri_t events_options_handler = {
      .uri = DIRECT_EVENTS_PATH,
      .method = HTTP_OPTIONS,
      .handler = direct_options_handler,
  };
  return httpd_register_uri_handler(direct_state.server, &events_handler) == ESP_OK &&
      httpd_register_uri_handler(direct_state.server, &events_options_handler) == ESP_OK &&
      httpd_register_uri_handler(direct_state.server, &get_handler) == ESP_OK &&
      httpd_register_uri_handler(direct_state.server, &post_handler) == ESP_OK &&
      httpd_register_uri_handler(direct_state.server, &options_handler) == ESP_OK;
}

static esp_err_t direct_add_mdns_service(
    const char *hostname,
    const char *instance,
    const char *device_id,
    const char *chip,
    const char *firmware_version,
    uint16_t port) {
  esp_err_t init_result = mdns_init();
  if (init_result != ESP_OK && init_result != ESP_ERR_INVALID_STATE) {
    return init_result;
  }
  esp_err_t result = mdns_hostname_set(hostname);
  if (result != ESP_OK) {
    return result;
  }
  result = mdns_instance_name_set(instance);
  if (result != ESP_OK) {
    return result;
  }
  mdns_txt_item_t txt[] = {
      {"txtvers", direct_state.dns_sd_schema_version},
      {"protovers", direct_state.protocol_version},
      {"device_id", device_id},
      {"name", instance},
      {"frontend", "micro"},
      {"transport", "http"},
      {"path", DIRECT_BASE_PATH},
      {"firmware", firmware_version},
      {"chip", chip},
      {"capabilities", "monitor"},
  };
  result = mdns_service_add(instance, "_espectre", "_tcp", port, txt, MP_ARRAY_SIZE(txt));
  if (result != ESP_OK) {
    return result;
  }
  direct_state.mdns_service_added = true;
  return ESP_OK;
}

static mp_obj_t native_direct_start(
    size_t n_pos_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args) {
  enum {
    ARG_port,
    ARG_hostname,
    ARG_instance,
    ARG_device_id,
    ARG_chip,
    ARG_firmware_version,
    ARG_protocol_version,
    ARG_dns_sd_schema_version,
    ARG_capabilities,
    ARG_info,
    ARG_config,
    ARG_wifi,
    ARG_status,
    ARG_diagnostics,
  };
  static const mp_arg_t allowed_args[] = {
      {MP_QSTR_port, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0}},
      {MP_QSTR_hostname, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_instance, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_device_id, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_chip, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_firmware_version, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_protocol_version, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_dns_sd_schema_version, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_capabilities, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_info, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_config, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_wifi, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_status, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
      {MP_QSTR_diagnostics, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
  };
  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all(n_pos_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
  if (direct_state.server != NULL) {
    mp_raise_OSError(MP_EBUSY);
  }
  int port = args[ARG_port].u_int;
  if (port <= 0 || port > 65535) {
    mp_raise_ValueError(MP_ERROR_TEXT("invalid Direct HTTP port"));
  }
  const char *hostname = mp_obj_str_get_str(args[ARG_hostname].u_obj);
  const char *instance = mp_obj_str_get_str(args[ARG_instance].u_obj);
  const char *device_id = mp_obj_str_get_str(args[ARG_device_id].u_obj);
  const char *chip = mp_obj_str_get_str(args[ARG_chip].u_obj);
  const char *firmware_version = mp_obj_str_get_str(args[ARG_firmware_version].u_obj);
  const char *protocol_version = mp_obj_str_get_str(args[ARG_protocol_version].u_obj);
  const char *dns_sd_schema_version = mp_obj_str_get_str(args[ARG_dns_sd_schema_version].u_obj);
  size_t protocol_version_len = strlen(protocol_version);
  size_t dns_sd_schema_version_len = strlen(dns_sd_schema_version);
  if (
      !direct_identifier_valid(device_id, DIRECT_MAX_DEVICE_ID_BYTES, false) ||
      !direct_identifier_valid(firmware_version, DIRECT_MAX_FIRMWARE_VERSION_BYTES, true) ||
      !direct_identifier_valid(protocol_version, DIRECT_MAX_PROTOCOL_VERSION_BYTES, true) ||
      dns_sd_schema_version_len == 0 ||
      dns_sd_schema_version_len > DIRECT_MAX_DNS_SD_SCHEMA_VERSION_BYTES) {
    mp_raise_ValueError(MP_ERROR_TEXT("invalid protocol version"));
  }

  if (direct_state.lock == NULL) {
    direct_state.lock = xSemaphoreCreateMutex();
  }
  direct_state.stopping = false;
  direct_reset_counters();
  if (direct_state.lock == NULL ||
      direct_replace_json_string(
          &direct_state.capabilities,
          args[ARG_capabilities].u_obj) == NULL ||
      direct_replace_json_string(&direct_state.info, args[ARG_info].u_obj) == NULL ||
      direct_replace_json_string(&direct_state.config, args[ARG_config].u_obj) == NULL ||
      direct_replace_json_string(&direct_state.wifi, args[ARG_wifi].u_obj) == NULL ||
      direct_replace_diagnostics_object(args[ARG_diagnostics].u_obj) !=
          DIRECT_DIAGNOSTICS_UPDATED ||
      !direct_replace_status_object(args[ARG_status].u_obj) ||
      direct_replace_string(&direct_state.device_id, device_id, strlen(device_id)) == NULL ||
      direct_replace_string(
          &direct_state.protocol_version,
          protocol_version,
          protocol_version_len) == NULL ||
      direct_replace_string(
          &direct_state.dns_sd_schema_version,
          dns_sd_schema_version,
          dns_sd_schema_version_len) == NULL) {
    direct_free_snapshots();
    mp_raise_OSError(MP_ENOMEM);
  }

  const esp_timer_create_args_t heartbeat_timer_args = {
      .callback = direct_heartbeat_timer_callback,
      .name = "micro_direct_heartbeat",
  };
  if (esp_timer_create(&heartbeat_timer_args, &direct_state.heartbeat_timer) != ESP_OK) {
    direct_stop_native();
    mp_raise_OSError(MP_ENOMEM);
  }

  httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
  server_config.server_port = (uint16_t) port;
  server_config.task_priority = CONFIG_ESPECTRE_DIRECT_HTTPD_TASK_PRIORITY;
  server_config.max_open_sockets = 4;
  server_config.max_uri_handlers = 5;
  server_config.uri_match_fn = httpd_uri_match_wildcard;
  server_config.lru_purge_enable = true;
  server_config.enable_so_linger = true;
  server_config.linger_timeout = 0;
  server_config.recv_wait_timeout = 5;
  // SSE and command responses share the HTTP server task. Do not let a slow
  // event client block canonical command handling for multiple sample ticks.
  server_config.send_wait_timeout = 1;
  esp_err_t server_result = httpd_start(&direct_state.server, &server_config);
  if (server_result != ESP_OK) {
    ESP_LOGE(NATIVE_DIRECT_TAG, "HTTP server start failed: %s", esp_err_to_name(server_result));
    direct_stop_native();
    mp_raise_OSError(MP_EIO);
  }
  if (!direct_register_http_handlers()) {
    ESP_LOGE(NATIVE_DIRECT_TAG, "HTTP handler registration failed");
    direct_stop_native();
    mp_raise_OSError(MP_EIO);
  }
  esp_err_t mdns_result = direct_add_mdns_service(
      hostname,
      instance,
      device_id,
      chip,
      firmware_version,
      (uint16_t) port);
  if (mdns_result != ESP_OK) {
    ESP_LOGE(
        NATIVE_DIRECT_TAG,
        "mDNS service registration failed: %s",
        esp_err_to_name(mdns_result));
    direct_stop_native();
    mp_raise_OSError(
        mdns_result == ESP_ERR_NO_MEM ? MP_ENOMEM : mdns_result == ESP_ERR_INVALID_STATE ? MP_EBUSY
                                                                                         : MP_EIO);
  }
  return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(native_direct_start_obj, 0, native_direct_start);

static mp_obj_t native_direct_update_status(mp_obj_t status_obj) {
  if (!direct_replace_status_object(status_obj)) {
    mp_raise_ValueError(MP_ERROR_TEXT("Direct status is too large"));
  }
  return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(native_direct_update_status_obj, native_direct_update_status);

static mp_obj_t native_direct_update_diagnostics(mp_obj_t diagnostics_obj) {
  direct_diagnostics_update_t result = direct_replace_diagnostics_object(diagnostics_obj);
  if (result == DIRECT_DIAGNOSTICS_TOO_LARGE) {
    mp_raise_ValueError(MP_ERROR_TEXT("Direct diagnostics are too large"));
  }
  if (result == DIRECT_DIAGNOSTICS_BUSY) {
    return mp_const_false;
  }
  return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(native_direct_update_diagnostics_obj, native_direct_update_diagnostics);

static mp_obj_t native_direct_update_config(mp_obj_t config_obj) {
  if (direct_replace_json_string(&direct_state.config, config_obj) == NULL) {
    mp_raise_OSError(MP_ENOMEM);
  }
  return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(native_direct_update_config_obj, native_direct_update_config);

static mp_obj_t native_direct_update_wifi(mp_obj_t wifi_obj) {
  if (direct_replace_json_string(&direct_state.wifi, wifi_obj) == NULL) {
    mp_raise_OSError(MP_ENOMEM);
  }
  return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(native_direct_update_wifi_obj, native_direct_update_wifi);

static mp_obj_t native_direct_has_event_client(void) {
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  bool connected = direct_state.event_request != NULL;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  return mp_obj_new_bool(connected);
}
static MP_DEFINE_CONST_FUN_OBJ_0(native_direct_has_event_client_obj, native_direct_has_event_client);

static mp_obj_t native_direct_take_recalibration_request(void) {
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  bool pending = direct_state.recalibration_pending;
  if (pending) {
    direct_state.recalibration_pending = false;
    direct_state.recalibration_active = true;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  return mp_obj_new_bool(pending);
}
static MP_DEFINE_CONST_FUN_OBJ_0(
    native_direct_take_recalibration_request_obj,
    native_direct_take_recalibration_request);

static mp_obj_t native_direct_complete_recalibration(void) {
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  direct_state.recalibration_active = false;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(
    native_direct_complete_recalibration_obj,
    native_direct_complete_recalibration);

static mp_obj_t native_direct_firmware_version(void) {
  const esp_app_desc_t *description = esp_app_get_description();
  const char *version = description != NULL && description->version[0] != '\0'
      ? description->version
      : "unknown";
  return mp_obj_new_str(version, strlen(version));
}
static MP_DEFINE_CONST_FUN_OBJ_0(
    native_direct_firmware_version_obj,
    native_direct_firmware_version);

static mp_obj_t native_direct_chip_target(void) {
  return mp_obj_new_str(CONFIG_IDF_TARGET, strlen(CONFIG_IDF_TARGET));
}
static MP_DEFINE_CONST_FUN_OBJ_0(
    native_direct_chip_target_obj,
    native_direct_chip_target);

static void direct_release_event_slot(void) {
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  direct_state.event_work_pending = false;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
}

static mp_obj_t native_direct_publish(mp_obj_t event_obj, mp_obj_t payload_obj) {
  size_t event_length;
  const char *event = mp_obj_str_get_data(event_obj, &event_length);
  if (event_length == 0 || event_length > DIRECT_MAX_EVENT_NAME_BYTES) {
    mp_raise_ValueError(MP_ERROR_TEXT("Direct event is too large"));
  }
  for (size_t index = 0; index < event_length; ++index) {
    char value = event[index];
    if (!((value >= 'a' && value <= 'z') || value == '_')) {
      mp_raise_ValueError(MP_ERROR_TEXT("invalid Direct event name"));
    }
  }
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  bool unavailable = direct_state.stopping ||
      direct_state.event_request == NULL || direct_state.server == NULL;
  bool busy = !unavailable && direct_state.event_work_pending;
  if (!unavailable && !busy) {
    // Reserve the sole static frame before JSON serialization. The HTTP
    // worker cannot consume it until queue_work succeeds below.
    direct_state.event_work_pending = true;
  }
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  if (busy) {
    direct_increment(&direct_state.dropped_motion_events);
    return mp_const_false;
  }
  if (unavailable) {
    return mp_const_false;
  }

  char prefix[80];
  int prefix_length = snprintf(
      prefix,
      sizeof(prefix),
      "event: %.*s\ndata: ",
      (int) event_length,
      event);
  if (prefix_length <= 0 || (size_t) prefix_length >= sizeof(prefix)) {
    direct_release_event_slot();
    return mp_const_false;
  }
  memcpy(direct_state.event_frame, prefix, (size_t) prefix_length);
  char *payload_target = direct_state.event_frame + prefix_length;
  size_t payload_length = 0;
  if (mp_obj_is_str_or_bytes(payload_obj)) {
    const char *payload = mp_obj_str_get_data(payload_obj, &payload_length);
    if (payload_length > DIRECT_MAX_EVENT_BYTES) {
      direct_release_event_slot();
      mp_raise_ValueError(MP_ERROR_TEXT("Direct event is too large"));
    }
    memcpy(payload_target, payload, payload_length);
  } else if (!direct_encode_json(
                 payload_target,
                 DIRECT_MAX_EVENT_BYTES,
                 payload_obj,
                 &payload_length)) {
    direct_release_event_slot();
    mp_raise_ValueError(MP_ERROR_TEXT("Direct event is too large"));
  }
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  if (direct_state.stopping || direct_state.event_request == NULL || direct_state.server == NULL) {
    direct_state.event_work_pending = false;
    if (direct_state.lock != NULL) {
      xSemaphoreGive(direct_state.lock);
    }
    return mp_const_false;
  }
  size_t frame_length = (size_t) prefix_length + payload_length + 2;
  memcpy(direct_state.event_frame + prefix_length + payload_length, "\n\n", 2);
  direct_state.event_frame_length = frame_length;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }

  // Keep one bounded in-flight frame. A later detector evaluation retries
  // after completion, while skipped events are visible in Direct diagnostics.
  if (
      httpd_queue_work(direct_state.server, direct_send_event_work, NULL) != ESP_OK) {
    direct_release_event_slot();
    direct_increment(&direct_state.send_failures);
    return mp_const_false;
  }
  return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_2(native_direct_publish_obj, native_direct_publish);

static void direct_store_uint(mp_obj_t target, qstr key, uint32_t value) {
  mp_obj_dict_store(
      target,
      MP_OBJ_NEW_QSTR(key),
      mp_obj_new_int_from_uint(value));
}

static mp_obj_t native_direct_diagnostics(mp_obj_t target) {
  if (!mp_obj_is_type(target, &mp_type_dict)) {
    mp_raise_TypeError(MP_ERROR_TEXT("Direct diagnostics target must be a dict"));
  }
  if (direct_state.lock != NULL) {
    xSemaphoreTake(direct_state.lock, portMAX_DELAY);
  }
  uint32_t event_clients = direct_state.event_request == NULL ? 0U : 1U;
  uint32_t queued_messages =
      direct_state.event_work_pending || direct_state.heartbeat_work_pending ? 1U : 0U;
  if (direct_state.lock != NULL) {
    xSemaphoreGive(direct_state.lock);
  }
  direct_store_uint(target, MP_QSTR_event_clients, event_clients);
  direct_store_uint(target, MP_QSTR_event_client_limit, 1U);
  direct_store_uint(target, MP_QSTR_queue_capacity, 1U);
  direct_store_uint(target, MP_QSTR_queued_messages, queued_messages);
  direct_store_uint(
      target,
      MP_QSTR_accepted_connections,
      direct_counter(&direct_state.accepted_connections));
  direct_store_uint(
      target,
      MP_QSTR_rejected_connections,
      direct_counter(&direct_state.rejected_connections));
  direct_store_uint(
      target,
      MP_QSTR_malformed_requests,
      direct_counter(&direct_state.malformed_requests));
  direct_store_uint(
      target,
      MP_QSTR_oversized_requests,
      direct_counter(&direct_state.oversized_requests));
  direct_store_uint(
      target,
      MP_QSTR_rate_limited_requests,
      direct_counter(&direct_state.rate_limited_requests));
  direct_store_uint(
      target,
      MP_QSTR_dropped_motion_events,
      direct_counter(&direct_state.dropped_motion_events));
  direct_store_uint(
      target,
      MP_QSTR_send_failures,
      direct_counter(&direct_state.send_failures));
  return target;
}
static MP_DEFINE_CONST_FUN_OBJ_1(native_direct_diagnostics_obj, native_direct_diagnostics);

static mp_obj_t native_direct_stop(void) {
  direct_stop_native();
  return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(native_direct_stop_obj, native_direct_stop);

static const mp_rom_map_elem_t native_direct_module_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_espectre_native_direct)},
    {MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&native_direct_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_update_status), MP_ROM_PTR(&native_direct_update_status_obj)},
    {MP_ROM_QSTR(MP_QSTR_update_diagnostics), MP_ROM_PTR(&native_direct_update_diagnostics_obj)},
    {MP_ROM_QSTR(MP_QSTR_update_config), MP_ROM_PTR(&native_direct_update_config_obj)},
    {MP_ROM_QSTR(MP_QSTR_update_wifi), MP_ROM_PTR(&native_direct_update_wifi_obj)},
    {MP_ROM_QSTR(MP_QSTR_has_event_client), MP_ROM_PTR(&native_direct_has_event_client_obj)},
    {MP_ROM_QSTR(MP_QSTR_take_recalibration_request), MP_ROM_PTR(&native_direct_take_recalibration_request_obj)},
    {MP_ROM_QSTR(MP_QSTR_complete_recalibration), MP_ROM_PTR(&native_direct_complete_recalibration_obj)},
    {MP_ROM_QSTR(MP_QSTR_firmware_version), MP_ROM_PTR(&native_direct_firmware_version_obj)},
    {MP_ROM_QSTR(MP_QSTR_chip_target), MP_ROM_PTR(&native_direct_chip_target_obj)},
    {MP_ROM_QSTR(MP_QSTR_publish), MP_ROM_PTR(&native_direct_publish_obj)},
    {MP_ROM_QSTR(MP_QSTR_diagnostics), MP_ROM_PTR(&native_direct_diagnostics_obj)},
    {MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&native_direct_stop_obj)},
};
static MP_DEFINE_CONST_DICT(native_direct_module_globals, native_direct_module_globals_table);

const mp_obj_module_t native_direct_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *) &native_direct_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_espectre_native_direct, native_direct_module);
