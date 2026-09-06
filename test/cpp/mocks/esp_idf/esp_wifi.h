/*
 * ESPectre - Mock esp_wifi.h
 *
 * Host-side mock of esp_wifi.h for native C++ tests.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#ifndef ESP_WIFI_H
#define ESP_WIFI_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// WiFi mode
typedef enum {
  WIFI_MODE_NULL = 0,
  WIFI_MODE_STA,
  WIFI_MODE_AP,
  WIFI_MODE_APSTA,
  WIFI_MODE_MAX
} wifi_mode_t;

// WiFi interface
typedef enum { WIFI_IF_STA = 0, WIFI_IF_AP, WIFI_IF_MAX } wifi_interface_t;

typedef enum { WIFI_STORAGE_FLASH = 0, WIFI_STORAGE_RAM } wifi_storage_t;

typedef struct {
  int dummy;
} wifi_init_config_t;

#define WIFI_INIT_CONFIG_DEFAULT() wifi_init_config_t{}

typedef enum {
  WIFI_ALL_CHANNEL_SCAN = 0,
  WIFI_FAST_SCAN = 1,
} wifi_scan_method_t;

typedef enum {
  WIFI_CONNECT_AP_BY_SIGNAL = 0,
  WIFI_CONNECT_AP_BY_SECURITY = 1,
} wifi_sort_method_t;

typedef enum {
  WIFI_AUTH_OPEN = 0,
  WIFI_AUTH_WEP,
  WIFI_AUTH_WPA_PSK,
  WIFI_AUTH_WPA2_PSK,
  WIFI_AUTH_WPA_WPA2_PSK,
  WIFI_AUTH_WPA2_ENTERPRISE,
  WIFI_AUTH_WPA3_PSK,
  WIFI_AUTH_WPA2_WPA3_PSK,
} wifi_auth_mode_t;

typedef enum {
  WPA3_SAE_PWE_UNSPECIFIED = 0,
  WPA3_SAE_PWE_HUNT_AND_PECK,
  WPA3_SAE_PWE_HASH_TO_ELEMENT,
  WPA3_SAE_PWE_BOTH,
} wifi_sae_pwe_method_t;

typedef struct {
  bool capable;
  bool required;
} wifi_pmf_config_t;

typedef struct {
  uint8_t ssid[32];
  uint8_t password[64];
  wifi_scan_method_t scan_method;
  wifi_sort_method_t sort_method;
  struct {
    wifi_auth_mode_t authmode;
  } threshold;
  wifi_sae_pwe_method_t sae_pwe_h2e;
  wifi_pmf_config_t pmf_cfg;
  uint8_t channel;
  uint8_t bssid[6];
  bool bssid_set;
} wifi_sta_config_t;

typedef union {
  wifi_sta_config_t sta;
} wifi_config_t;

typedef struct {
  uint8_t *ssid;
  uint8_t *bssid;
  uint8_t channel;
} wifi_scan_config_t;

typedef struct {
  uint8_t reason;
} wifi_event_sta_disconnected_t;

#ifndef WIFI_REASON_ROAMING
#define WIFI_REASON_ROAMING 207
#endif

#ifndef WIFI_REASON_BEACON_TIMEOUT
#define WIFI_REASON_BEACON_TIMEOUT 200
#endif
#ifndef WIFI_REASON_NO_AP_FOUND
#define WIFI_REASON_NO_AP_FOUND 201
#endif
#ifndef WIFI_REASON_AUTH_FAIL
#define WIFI_REASON_AUTH_FAIL 202
#endif
#ifndef WIFI_REASON_ASSOC_FAIL
#define WIFI_REASON_ASSOC_FAIL 203
#endif
#ifndef WIFI_REASON_HANDSHAKE_TIMEOUT
#define WIFI_REASON_HANDSHAKE_TIMEOUT 204
#endif
#ifndef WIFI_REASON_CONNECTION_FAIL
#define WIFI_REASON_CONNECTION_FAIL 205
#endif
#ifndef WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY
#define WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY 206
#endif
#ifndef WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD
#define WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD 207
#endif
#ifndef WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD
#define WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD 208
#endif

// CSI configuration
// Note: field order must match designated initializer order in csi_pipeline.cpp
typedef struct {
  // ESP32-S3 fields (used in non-C6 builds)
  bool lltf_en;
  bool htltf_en;
  bool stbc_htltf2_en;
  bool ltf_merge_en;
  bool channel_filter_en;
  bool manu_scale;
  uint8_t shift;
  // ESP32-C6 fields
  bool enable;
  bool acquire_csi_legacy;
  bool acquire_csi_ht20;
  bool acquire_csi_ht40;
  bool acquire_csi_su;
  uint8_t acquire_csi_mu;
  uint8_t acquire_csi_dcm;
  uint8_t acquire_csi_beamformed;
  uint8_t acquire_csi_he_stbc;
  uint8_t val_scale_cfg;
  uint8_t dump_ack_en;
} wifi_csi_config_t;

// RX control structure (matches ESP-IDF wifi_pkt_rx_ctrl_t)
typedef struct {
  int8_t rssi;
  uint8_t rate;
  uint8_t sig_mode;
  uint8_t mcs;
  uint8_t cwb;
  uint8_t smoothing;
  uint8_t not_sounding;
  uint8_t aggregation;
  uint8_t stbc;
  uint8_t fec_coding;
  uint8_t sgi;
  int8_t noise_floor;
  uint8_t ampdu_cnt;
  uint8_t channel;
  uint8_t secondary_channel;
  uint32_t timestamp;
  uint8_t ant;
  uint16_t sig_len;
  uint8_t rx_state;
} wifi_pkt_rx_ctrl_t;

// CSI info structure (matches ESP-IDF wifi_csi_info_t)
typedef struct {
  wifi_pkt_rx_ctrl_t rx_ctrl;
  uint8_t mac[6];
  uint8_t dmac[6];
  bool first_word_invalid;
  int8_t *buf;
  uint16_t len;
  uint8_t *hdr;
  uint8_t *payload;
  uint16_t payload_len;
  uint16_t rx_seq;
} wifi_csi_info_t;

// WiFi Bandwidth
typedef enum {
  WIFI_BW_HT20 = 1,
  WIFI_BW_HT40,
} wifi_bandwidth_t;

typedef enum {
  WIFI_SECOND_CHAN_NONE = 0,
  WIFI_SECOND_CHAN_ABOVE,
  WIFI_SECOND_CHAN_BELOW,
} wifi_second_chan_t;

// WiFi Band mode (dual-band capable targets)
typedef enum {
  WIFI_BAND_MODE_2G_ONLY = 1,
  WIFI_BAND_MODE_5G_ONLY = 2,
  WIFI_BAND_MODE_AUTO = 3,
} wifi_band_mode_t;

// WiFi Protocols
#define WIFI_PROTOCOL_11B 1
#define WIFI_PROTOCOL_11G 2
#define WIFI_PROTOCOL_11N 4
#define WIFI_PROTOCOL_LR 8
#define WIFI_PROTOCOL_11A 16
#define WIFI_PROTOCOL_11AC 32
#define WIFI_PROTOCOL_11AX 64

typedef struct {
  uint16_t ghz_2g;
  uint16_t ghz_5g;
} wifi_protocols_t;

typedef struct {
  wifi_bandwidth_t ghz_2g;
  wifi_bandwidth_t ghz_5g;
} wifi_bandwidths_t;

// WiFi Power Save
typedef enum {
  WIFI_PS_NONE,
  WIFI_PS_MIN_MODEM,
  WIFI_PS_MAX_MODEM,
} wifi_ps_type_t;

// CSI callback type
typedef void (*wifi_csi_cb_t)(void *ctx, wifi_csi_info_t *data);

typedef struct {
  uint8_t ssid[33];
  int8_t rssi;
  uint8_t primary;
  uint8_t bssid[6];
} wifi_ap_record_t;

typedef struct {
  esp_err_t set_protocol_results[4];
  int set_protocol_result_count;
  int set_protocol_call_count;
  uint8_t last_protocol_bitmap;

  esp_err_t set_protocols_result;
  int set_protocols_call_count;
  wifi_protocols_t last_protocols;

  esp_err_t get_protocol_result;
  uint8_t protocol_bitmap;

  esp_err_t get_protocols_result;
  wifi_protocols_t protocols;

  esp_err_t set_bandwidth_result;
  int set_bandwidth_call_count;
  int set_bandwidth_sequence;
  wifi_bandwidth_t last_bandwidth;

  esp_err_t set_bandwidths_result;
  int set_bandwidths_call_count;
  wifi_bandwidths_t last_bandwidths;

  esp_err_t get_bandwidth_result;
  wifi_bandwidth_t bandwidth;

  esp_err_t get_bandwidths_result;
  wifi_bandwidths_t bandwidths;

  esp_err_t set_promiscuous_result;
  int set_promiscuous_call_count;
  bool last_promiscuous;

  esp_err_t get_promiscuous_result;
  bool promiscuous;

  int set_csi_call_count;
  bool csi_enabled;
  int set_csi_rx_cb_call_count;
  wifi_csi_cb_t csi_callback;
  void *csi_callback_context;

  esp_err_t get_ps_result;
  wifi_ps_type_t ps_type;
  int set_ps_call_count;
  wifi_ps_type_t last_set_ps_type;

  int init_call_count;
  esp_err_t deinit_result;
  int deinit_call_count;
  int set_storage_call_count;
  int set_mode_call_count;
  esp_err_t start_results[4];
  int start_result_count;
  int start_call_count;
  esp_err_t stop_result;
  int stop_call_count;
  esp_err_t connect_results[4];
  int connect_result_count;
  int connect_call_count;
  int connect_sequences[4];
  esp_err_t disconnect_results[4];
  int disconnect_result_count;
  int disconnect_call_count;
  int disconnect_sequences[4];
  esp_err_t get_config_result;
  int get_config_call_count;
  esp_err_t set_config_results[4];
  int set_config_result_count;
  int set_config_call_count;
  int set_config_sequences[4];
  int get_ap_info_call_count;
  esp_err_t get_ap_info_result;
  wifi_ap_record_t current_ap_info;
  esp_err_t scan_start_result;
  int scan_start_call_count;
  esp_err_t scan_stop_result;
  int scan_stop_call_count;
  bool last_scan_block;
  bool last_scan_configured;
  char last_scan_ssid[33];
  uint8_t last_scan_channel;
  esp_err_t scan_get_ap_num_result;
  esp_err_t scan_get_ap_records_result;
  uint16_t scan_ap_count;
  wifi_ap_record_t scan_ap_records[32];
  wifi_config_t current_config;
  wifi_config_t last_config;
  esp_err_t get_mac_result;
  uint8_t mac[6];

  esp_err_t get_channel_result;
  uint8_t primary_channel;
  wifi_second_chan_t second_channel;

  esp_err_t set_band_mode_result;
  int set_band_mode_call_count;
  wifi_band_mode_t last_band_mode;

  esp_err_t get_band_mode_result;
  wifi_band_mode_t band_mode;

  // Monotonic tick stamped on the radio-policy setters. The band mode must be
  // widened before the per-band protocol is pinned, and only the relative order
  // of those calls proves it; call counts alone cannot.
  int call_sequence;
  int set_band_mode_sequence;
  int set_protocols_sequence;
} esp_wifi_mock_state_t;

extern esp_wifi_mock_state_t g_esp_wifi_mock;

void esp_wifi_mock_reset(void);

// Mock WiFi functions
static inline esp_err_t esp_wifi_init(const wifi_init_config_t *config) {
  (void)config;
  g_esp_wifi_mock.init_call_count++;
  return ESP_OK;
}

static inline esp_err_t esp_wifi_deinit(void) {
  g_esp_wifi_mock.deinit_call_count++;
  return g_esp_wifi_mock.deinit_result;
}

static inline esp_err_t esp_wifi_set_storage(wifi_storage_t storage) {
  (void)storage;
  g_esp_wifi_mock.set_storage_call_count++;
  return ESP_OK;
}

static inline esp_err_t esp_wifi_set_mode(wifi_mode_t mode) {
  (void)mode;
  g_esp_wifi_mock.set_mode_call_count++;
  return ESP_OK;
}

static inline esp_err_t esp_wifi_get_mode(wifi_mode_t *mode) {
  if (mode)
    *mode = WIFI_MODE_STA;
  return ESP_OK;
}

static inline esp_err_t esp_wifi_start(void) {
  const int index = g_esp_wifi_mock.start_call_count;
  const esp_err_t result = index < g_esp_wifi_mock.start_result_count
                               ? g_esp_wifi_mock.start_results[index]
                               : ESP_OK;
  g_esp_wifi_mock.start_call_count++;
  return result;
}

static inline esp_err_t esp_wifi_stop(void) {
  g_esp_wifi_mock.stop_call_count++;
  return g_esp_wifi_mock.stop_result;
}

static inline esp_err_t esp_wifi_connect(void) {
  const int index = g_esp_wifi_mock.connect_call_count;
  const esp_err_t result = index < g_esp_wifi_mock.connect_result_count
                               ? g_esp_wifi_mock.connect_results[index]
                               : ESP_OK;
  if (index < 4) {
    g_esp_wifi_mock.connect_sequences[index] = ++g_esp_wifi_mock.call_sequence;
  }
  g_esp_wifi_mock.connect_call_count++;
  return result;
}

static inline esp_err_t esp_wifi_disconnect(void) {
  const int index = g_esp_wifi_mock.disconnect_call_count;
  const esp_err_t result = index < g_esp_wifi_mock.disconnect_result_count
                               ? g_esp_wifi_mock.disconnect_results[index]
                               : ESP_OK;
  if (index < 4) {
    g_esp_wifi_mock.disconnect_sequences[index] = ++g_esp_wifi_mock.call_sequence;
  }
  g_esp_wifi_mock.disconnect_call_count++;
  return result;
}

static inline esp_err_t esp_wifi_scan_start(const wifi_scan_config_t *config, bool block) {
  g_esp_wifi_mock.scan_start_call_count++;
  g_esp_wifi_mock.last_scan_block = block;
  g_esp_wifi_mock.last_scan_configured = config != NULL;
  memset(g_esp_wifi_mock.last_scan_ssid, 0, sizeof(g_esp_wifi_mock.last_scan_ssid));
  if (config != NULL) {
    g_esp_wifi_mock.last_scan_channel = config->channel;
    if (config->ssid != NULL) {
      strncpy(g_esp_wifi_mock.last_scan_ssid,
              (const char *)config->ssid,
              sizeof(g_esp_wifi_mock.last_scan_ssid) - 1U);
    }
  }
  return g_esp_wifi_mock.scan_start_result;
}

static inline esp_err_t esp_wifi_scan_stop(void) {
  g_esp_wifi_mock.scan_stop_call_count++;
  return g_esp_wifi_mock.scan_stop_result;
}

static inline esp_err_t esp_wifi_scan_get_ap_num(uint16_t *number) {
  if (number != nullptr) {
    *number = g_esp_wifi_mock.scan_ap_count;
  }
  return g_esp_wifi_mock.scan_get_ap_num_result;
}

static inline esp_err_t esp_wifi_scan_get_ap_records(uint16_t *number,
                                                     wifi_ap_record_t *records) {
  if (number != nullptr && records != nullptr && g_esp_wifi_mock.scan_get_ap_records_result == ESP_OK) {
    const uint16_t count = *number < g_esp_wifi_mock.scan_ap_count ? *number : g_esp_wifi_mock.scan_ap_count;
    memcpy(records, g_esp_wifi_mock.scan_ap_records, count * sizeof(wifi_ap_record_t));
    *number = count;
  }
  return g_esp_wifi_mock.scan_get_ap_records_result;
}

static inline esp_err_t esp_wifi_set_config(wifi_interface_t ifx, wifi_config_t *config) {
  (void)ifx;
  const int index = g_esp_wifi_mock.set_config_call_count;
  const esp_err_t result = index < g_esp_wifi_mock.set_config_result_count
                               ? g_esp_wifi_mock.set_config_results[index]
                               : ESP_OK;
  if (index < 4) {
    g_esp_wifi_mock.set_config_sequences[index] = ++g_esp_wifi_mock.call_sequence;
  }
  g_esp_wifi_mock.set_config_call_count++;
  if (config != nullptr) {
    g_esp_wifi_mock.last_config = *config;
    if (result == ESP_OK) {
      g_esp_wifi_mock.current_config = *config;
    }
  } else {
    memset(&g_esp_wifi_mock.last_config, 0, sizeof(g_esp_wifi_mock.last_config));
  }
  return result;
}

static inline esp_err_t esp_wifi_get_config(wifi_interface_t ifx, wifi_config_t *config) {
  (void)ifx;
  g_esp_wifi_mock.get_config_call_count++;
  if (config != nullptr && g_esp_wifi_mock.get_config_result == ESP_OK) {
    *config = g_esp_wifi_mock.current_config;
  }
  return g_esp_wifi_mock.get_config_result;
}

static inline esp_err_t esp_wifi_get_mac(wifi_interface_t ifx, uint8_t mac[6]) {
  (void)ifx;
  if (mac != nullptr) {
    memcpy(mac, g_esp_wifi_mock.mac, 6);
  }
  return g_esp_wifi_mock.get_mac_result;
}

static inline esp_err_t
esp_wifi_set_csi_config(const wifi_csi_config_t *config) {
  (void)config;
  return ESP_OK;
}

static inline esp_err_t esp_wifi_set_csi_rx_cb(wifi_csi_cb_t cb, void *ctx) {
  g_esp_wifi_mock.set_csi_rx_cb_call_count++;
  g_esp_wifi_mock.csi_callback = cb;
  g_esp_wifi_mock.csi_callback_context = ctx;
  return ESP_OK;
}

static inline esp_err_t esp_wifi_set_csi(bool en) {
  g_esp_wifi_mock.set_csi_call_count++;
  g_esp_wifi_mock.csi_enabled = en;
  return ESP_OK;
}

static inline esp_err_t esp_wifi_set_promiscuous(bool en) {
  g_esp_wifi_mock.set_promiscuous_call_count++;
  g_esp_wifi_mock.last_promiscuous = en;
  if (g_esp_wifi_mock.set_promiscuous_result == ESP_OK) {
    g_esp_wifi_mock.promiscuous = en;
  }
  return g_esp_wifi_mock.set_promiscuous_result;
}

static inline esp_err_t esp_wifi_get_promiscuous(bool *en) {
  if (en) {
    *en = g_esp_wifi_mock.promiscuous;
  }
  return g_esp_wifi_mock.get_promiscuous_result;
}

static inline esp_err_t esp_wifi_set_bandwidth(wifi_interface_t ifx,
                                               wifi_bandwidth_t bw) {
  (void)ifx;
  g_esp_wifi_mock.set_bandwidth_call_count++;
  g_esp_wifi_mock.set_bandwidth_sequence = ++g_esp_wifi_mock.call_sequence;
  g_esp_wifi_mock.last_bandwidth = bw;
  if (g_esp_wifi_mock.set_bandwidth_result == ESP_OK) {
    g_esp_wifi_mock.bandwidth = bw;
  }
  return g_esp_wifi_mock.set_bandwidth_result;
}

static inline esp_err_t esp_wifi_set_band_mode(wifi_band_mode_t band_mode) {
  g_esp_wifi_mock.set_band_mode_call_count++;
  g_esp_wifi_mock.set_band_mode_sequence = ++g_esp_wifi_mock.call_sequence;
  g_esp_wifi_mock.last_band_mode = band_mode;
  if (g_esp_wifi_mock.set_band_mode_result == ESP_OK) {
    g_esp_wifi_mock.band_mode = band_mode;
  }
  return g_esp_wifi_mock.set_band_mode_result;
}

static inline esp_err_t esp_wifi_get_band_mode(wifi_band_mode_t *band_mode) {
  if (band_mode) {
    *band_mode = g_esp_wifi_mock.band_mode;
  }
  return g_esp_wifi_mock.get_band_mode_result;
}

static inline esp_err_t esp_wifi_set_protocol(wifi_interface_t ifx,
                                              uint8_t protocol_bitmap) {
  (void)ifx;
  esp_err_t result = ESP_OK;
  if (g_esp_wifi_mock.set_protocol_call_count < g_esp_wifi_mock.set_protocol_result_count) {
    result =
        g_esp_wifi_mock.set_protocol_results[g_esp_wifi_mock.set_protocol_call_count];
  }
  g_esp_wifi_mock.set_protocol_call_count++;
  g_esp_wifi_mock.last_protocol_bitmap = protocol_bitmap;
  if (result == ESP_OK) {
    g_esp_wifi_mock.protocol_bitmap = protocol_bitmap;
  }
  return result;
}

static inline esp_err_t esp_wifi_set_protocols(wifi_interface_t ifx,
                                               wifi_protocols_t *protocols) {
  (void)ifx;
  g_esp_wifi_mock.set_protocols_call_count++;
  g_esp_wifi_mock.set_protocols_sequence = ++g_esp_wifi_mock.call_sequence;
  if (protocols) {
    g_esp_wifi_mock.last_protocols = *protocols;
    if (g_esp_wifi_mock.set_protocols_result == ESP_OK) {
      g_esp_wifi_mock.protocols = *protocols;
    }
  }
  return g_esp_wifi_mock.set_protocols_result;
}

static inline esp_err_t esp_wifi_get_protocol(wifi_interface_t ifx,
                                              uint8_t *protocol_bitmap) {
  (void)ifx;
  if (protocol_bitmap) {
    *protocol_bitmap = g_esp_wifi_mock.protocol_bitmap;
  }
  return g_esp_wifi_mock.get_protocol_result;
}

static inline esp_err_t esp_wifi_get_protocols(wifi_interface_t ifx,
                                               wifi_protocols_t *protocols) {
  (void)ifx;
  if (protocols) {
    *protocols = g_esp_wifi_mock.protocols;
  }
  return g_esp_wifi_mock.get_protocols_result;
}

static inline esp_err_t esp_wifi_get_ps(wifi_ps_type_t *ps_type) {
  if (ps_type) {
    *ps_type = g_esp_wifi_mock.ps_type;
  }
  return g_esp_wifi_mock.get_ps_result;
}

static inline esp_err_t esp_wifi_set_ps(wifi_ps_type_t ps_type) {
  g_esp_wifi_mock.set_ps_call_count++;
  g_esp_wifi_mock.last_set_ps_type = ps_type;
  g_esp_wifi_mock.ps_type = ps_type;
  return ESP_OK;
}

static inline esp_err_t esp_wifi_set_bandwidths(wifi_interface_t ifx,
                                                wifi_bandwidths_t *bw) {
  (void)ifx;
  g_esp_wifi_mock.set_bandwidths_call_count++;
  if (bw) {
    g_esp_wifi_mock.last_bandwidths = *bw;
    if (g_esp_wifi_mock.set_bandwidths_result == ESP_OK) {
      g_esp_wifi_mock.bandwidths = *bw;
    }
  }
  return g_esp_wifi_mock.set_bandwidths_result;
}

static inline esp_err_t esp_wifi_get_bandwidth(wifi_interface_t ifx,
                                                wifi_bandwidth_t *bw) {
  (void)ifx;
  if (bw) {
    *bw = g_esp_wifi_mock.bandwidth;
  }
  return g_esp_wifi_mock.get_bandwidth_result;
}

static inline esp_err_t esp_wifi_get_bandwidths(wifi_interface_t ifx,
                                                wifi_bandwidths_t *bw) {
  (void)ifx;
  if (bw) {
    *bw = g_esp_wifi_mock.bandwidths;
  }
  return g_esp_wifi_mock.get_bandwidths_result;
}

static inline esp_err_t esp_wifi_get_channel(uint8_t *primary,
                                             wifi_second_chan_t *second) {
  if (primary) {
    *primary = g_esp_wifi_mock.primary_channel;
  }
  if (second) {
    *second = g_esp_wifi_mock.second_channel;
  }
  return g_esp_wifi_mock.get_channel_result;
}

static inline esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t *ap_info) {
  g_esp_wifi_mock.get_ap_info_call_count++;
  if (ap_info && g_esp_wifi_mock.get_ap_info_result == ESP_OK) {
    *ap_info = g_esp_wifi_mock.current_ap_info;
  }
  return g_esp_wifi_mock.get_ap_info_result;
}

#ifdef __cplusplus
}
#endif

#endif // ESP_WIFI_H
