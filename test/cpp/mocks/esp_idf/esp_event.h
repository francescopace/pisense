/*
 * ESPectre - Mock esp_event.h
 *
 * Host-side mock of esp_event.h for native C++ tests.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#ifndef ESP_EVENT_H
#define ESP_EVENT_H

#include "esp_err.h"
#include "esp_netif.h"
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Event loop handle
typedef void *esp_event_loop_handle_t;

// Event base type
typedef const char *esp_event_base_t;

// Event handler
typedef void (*esp_event_handler_t)(void *event_handler_arg,
                                    esp_event_base_t event_base,
                                    int32_t event_id, void *event_data);

// Event handler instance
typedef void *esp_event_handler_instance_t;

#define ESP_EVENT_ANY_ID -1

// WiFi events
#define WIFI_EVENT "WIFI_EVENT"

typedef enum {
  WIFI_EVENT_STA_START,
  WIFI_EVENT_STA_STOP,
  WIFI_EVENT_STA_CONNECTED,
  WIFI_EVENT_STA_DISCONNECTED,
  WIFI_EVENT_SCAN_DONE,
  WIFI_EVENT_AP_START,
  WIFI_EVENT_AP_STOP,
} wifi_event_t;

typedef struct {
  uint8_t status;
  uint8_t number;
  uint8_t scan_id;
} wifi_event_sta_scan_done_t;

// IP events
#define IP_EVENT "IP_EVENT"

typedef enum {
  IP_EVENT_STA_GOT_IP,
  IP_EVENT_STA_LOST_IP,
} ip_event_t;

typedef struct {
  esp_netif_t *esp_netif;
  esp_netif_ip_info_t ip_info;
  bool ip_changed;
} ip_event_got_ip_t;

typedef struct {
  esp_event_base_t event_base;
  int32_t event_id;
  esp_event_handler_t handler;
  void *handler_arg;
  esp_event_handler_instance_t instance;
  int active;
} esp_event_mock_slot_t;

typedef struct {
  esp_err_t register_results[8];
  int register_result_count;
  int register_call_count;
  int unregister_call_count;
  esp_event_mock_slot_t slots[8];
} esp_event_mock_state_t;

extern esp_event_mock_state_t g_esp_event_mock;

void esp_event_mock_reset(void);
void esp_event_mock_emit(esp_event_base_t event_base, int32_t event_id,
                         void *event_data);

// Mock event functions
static inline esp_err_t esp_event_loop_create_default(void) { return ESP_OK; }

static inline esp_err_t
esp_event_handler_register(esp_event_base_t event_base, int32_t event_id,
                           esp_event_handler_t event_handler,
                           void *event_handler_arg) {
  (void)event_base;
  (void)event_id;
  (void)event_handler;
  (void)event_handler_arg;
  return ESP_OK;
}

static inline esp_err_t
esp_event_handler_unregister(esp_event_base_t event_base, int32_t event_id,
                             esp_event_handler_t event_handler) {
  (void)event_base;
  (void)event_id;
  (void)event_handler;
  return ESP_OK;
}

static inline esp_err_t esp_event_handler_instance_register(
    esp_event_base_t event_base, int32_t event_id,
    esp_event_handler_t event_handler, void *event_handler_arg,
    esp_event_handler_instance_t *instance) {
  esp_err_t result = ESP_OK;
  if (g_esp_event_mock.register_call_count < g_esp_event_mock.register_result_count) {
    result = g_esp_event_mock.register_results[g_esp_event_mock.register_call_count];
  }

  const int call_index = g_esp_event_mock.register_call_count++;
  if (result != ESP_OK) {
    if (instance) {
      *instance = NULL;
    }
    return result;
  }

  if (call_index < 8) {
    g_esp_event_mock.slots[call_index].event_base = event_base;
    g_esp_event_mock.slots[call_index].event_id = event_id;
    g_esp_event_mock.slots[call_index].handler = event_handler;
    g_esp_event_mock.slots[call_index].handler_arg = event_handler_arg;
    g_esp_event_mock.slots[call_index].instance =
        (esp_event_handler_instance_t)(size_t)(call_index + 1);
    g_esp_event_mock.slots[call_index].active = 1;
    if (instance) {
      *instance = g_esp_event_mock.slots[call_index].instance;
    }
    return ESP_OK;
  }

  if (instance) {
    *instance = NULL;
  }
  return ESP_FAIL;
}

static inline esp_err_t
esp_event_handler_instance_unregister(esp_event_base_t event_base,
                                      int32_t event_id,
                                      esp_event_handler_instance_t instance) {
  g_esp_event_mock.unregister_call_count++;
  for (size_t i = 0; i < sizeof(g_esp_event_mock.slots) / sizeof(g_esp_event_mock.slots[0]); i++) {
    esp_event_mock_slot_t *slot = &g_esp_event_mock.slots[i];
    if (!slot->active || slot->instance != instance) {
      continue;
    }
    const int same_base =
        (slot->event_base == event_base) ||
        (slot->event_base != NULL && event_base != NULL &&
         strcmp(slot->event_base, event_base) == 0);
    if (same_base && (slot->event_id == event_id || slot->event_id == ESP_EVENT_ANY_ID)) {
      slot->active = 0;
    }
  }
  return ESP_OK;
}

static inline esp_err_t esp_event_post(esp_event_base_t event_base,
                                       int32_t event_id, void *event_data,
                                       size_t event_data_size,
                                       uint32_t ticks_to_wait) {
  (void)event_base;
  (void)event_id;
  (void)event_data;
  (void)event_data_size;
  (void)ticks_to_wait;
  return ESP_OK;
}

#ifdef __cplusplus
}
#endif

#endif // ESP_EVENT_H
