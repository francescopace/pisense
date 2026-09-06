/*
 * ESPectre - Wi-Fi Lifecycle Manager
 *
 * Controls STA lifecycle and HT20 CSI compatibility for sensing runtimes.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "wifi_lifecycle.h"
#include "espectre_log.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "wifi_band_helpers.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#if defined(CONFIG_ESP_COEX_SW_COEXIST_ENABLE) && __has_include("esp_coexist.h")
#include "esp_coexist.h"
#define ESPECTRE_HAVE_ESP_COEXIST 1
#endif

namespace espectre {

static const char *WIFI_LIFECYCLE_TAG = "WiFiLifecycle";

namespace {

std::atomic<uint32_t> wifi_driver_generation{0U};
#ifdef ESP_PLATFORM
constexpr uint32_t WIFI_DRIVER_POWER_CYCLE_SETTLE_MS = 250U;
#endif

enum class WiFiProtocolPolicyPath : uint8_t {
  ALREADY_PINNED,
  PINNED,
};

struct WiFiProtocolPolicyResult {
  esp_err_t err;
  WiFiProtocolPolicyPath path;
};

// The sensing contract uses one 20 MHz, 64-bin geometry across bands. The
// lifecycle permits HT on 2.4 GHz and VHT on 5 GHz while keeping HE disabled.
//
// ESP-IDF exposes 802.11n on 2.4 GHz through the supported b/g/n protocol
// combination; WIFI_PROTOCOL_11N alone is not a valid station mode on the
// published ESPectre targets. 802.11b/g do not exist on 5 GHz, where 802.11a is
// the legacy OFDM floor under 802.11n.
constexpr uint16_t WIFI_PROTOCOL_CSI_2G = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
constexpr wifi_bandwidth_t WIFI_BANDWIDTH_CSI = WIFI_BW_HT20;
#if ESPECTRE_WIFI_DUAL_BAND
constexpr uint16_t WIFI_PROTOCOL_CSI_5G =
    WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AC;
#endif

const char *wifi_csi_policy_target_(WifiBandPolicy policy) {
  switch (policy) {
#if ESPECTRE_WIFI_DUAL_BAND
    case WifiBandPolicy::BAND_5G:
      return "5 GHz VHT20 11a/n/ac (11ax disabled)";
    case WifiBandPolicy::AUTO:
      return "20 MHz on both bands (2.4 GHz 11b/g/n, 5 GHz 11a/n/ac; 11ax disabled)";
#endif
    case WifiBandPolicy::BAND_2G:
    default:
      return "2.4 GHz HT20 11b/g/n (802.11ax disabled)";
  }
}

const char *wifi_band_policy_to_str_(WifiBandPolicy policy) {
  switch (policy) {
    case WifiBandPolicy::BAND_2G:
      return "2g";
    case WifiBandPolicy::BAND_5G:
      return "5g";
    case WifiBandPolicy::AUTO:
      return "auto";
    default:
      return "unknown";
  }
}

const char *bandwidth_to_str_(wifi_bandwidth_t bw) {
  switch (bw) {
    case WIFI_BW_HT20:
      return "HT20";
    case WIFI_BW_HT40:
      return "HT40";
#ifdef WIFI_BW80
    case WIFI_BW80:
      return "BW80";
#endif
#ifdef WIFI_BW160
    case WIFI_BW160:
      return "BW160";
#endif
#ifdef WIFI_BW80_BW80
    case WIFI_BW80_BW80:
      return "BW80+80";
#endif
    default:
      return "UNKNOWN";
  }
}

const char *protocol_policy_path_to_str_(WiFiProtocolPolicyPath path) {
  switch (path) {
    case WiFiProtocolPolicyPath::ALREADY_PINNED:
      return "sensing protocol set already active";
    case WiFiProtocolPolicyPath::PINNED:
      return "pinned the sensing protocol set explicitly";
    default:
      return "unknown";
  }
}

#if ESPECTRE_WIFI_DUAL_BAND
wifi_band_mode_t band_mode_for_policy_(WifiBandPolicy policy) {
  switch (policy) {
    case WifiBandPolicy::BAND_5G:
      return WIFI_BAND_MODE_5G_ONLY;
    case WifiBandPolicy::AUTO:
      return WIFI_BAND_MODE_AUTO;
    case WifiBandPolicy::BAND_2G:
    default:
      return WIFI_BAND_MODE_2G_ONLY;
  }
}

// Must run before the protocol and bandwidth policy. It selects which legacy
// or per-band ESP-IDF API is valid and prevents an integrator's explicit band
// choice from being overwritten by the sensing runtime.
esp_err_t set_wifi_band_mode_for_csi_(WifiBandPolicy policy) {
  const wifi_band_mode_t requested = band_mode_for_policy_(policy);
  wifi_band_mode_t current = requested;
  if (esp_wifi_get_band_mode(&current) == ESP_OK && current == requested) {
    return ESP_OK;
  }
  return esp_wifi_set_band_mode(requested);
}
#endif

WiFiProtocolPolicyResult set_wifi_protocol_for_csi_(WifiBandPolicy policy) {
#if ESPECTRE_WIFI_DUAL_BAND
  if (policy == WifiBandPolicy::AUTO) {
    wifi_protocols_t current{};
    if (esp_wifi_get_protocols(WIFI_IF_STA, &current) == ESP_OK &&
        current.ghz_2g == WIFI_PROTOCOL_CSI_2G && current.ghz_5g == WIFI_PROTOCOL_CSI_5G) {
      return WiFiProtocolPolicyResult{ESP_OK, WiFiProtocolPolicyPath::ALREADY_PINNED};
    }
    wifi_protocols_t protocols{WIFI_PROTOCOL_CSI_2G, WIFI_PROTOCOL_CSI_5G};
    const esp_err_t ret = esp_wifi_set_protocols(WIFI_IF_STA, &protocols);
    return WiFiProtocolPolicyResult{ret, WiFiProtocolPolicyPath::PINNED};
  }
#endif
  const uint8_t requested_protocol =
#if ESPECTRE_WIFI_DUAL_BAND
      policy == WifiBandPolicy::BAND_5G ? WIFI_PROTOCOL_CSI_5G : WIFI_PROTOCOL_CSI_2G;
#else
      WIFI_PROTOCOL_CSI_2G;
#endif
  uint8_t current_protocol = 0U;
  if (esp_wifi_get_protocol(WIFI_IF_STA, &current_protocol) == ESP_OK) {
    if (current_protocol == requested_protocol) {
      return WiFiProtocolPolicyResult{ESP_OK, WiFiProtocolPolicyPath::ALREADY_PINNED};
    }
  }
  const esp_err_t ret = esp_wifi_set_protocol(WIFI_IF_STA, requested_protocol);
  return WiFiProtocolPolicyResult{ret, WiFiProtocolPolicyPath::PINNED};
}

esp_err_t set_wifi_bandwidth_for_csi_(WifiBandPolicy policy) {
#if ESPECTRE_WIFI_DUAL_BAND
  if (policy == WifiBandPolicy::AUTO) {
    wifi_bandwidths_t current{};
    if (esp_wifi_get_bandwidths(WIFI_IF_STA, &current) == ESP_OK &&
        current.ghz_2g == WIFI_BANDWIDTH_CSI && current.ghz_5g == WIFI_BANDWIDTH_CSI) {
      return ESP_OK;
    }
    wifi_bandwidths_t bandwidths{WIFI_BANDWIDTH_CSI, WIFI_BANDWIDTH_CSI};
    return esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths);
  }
#endif
  wifi_bandwidth_t current_bandwidth = WIFI_BW_HT20;
  if (esp_wifi_get_bandwidth(WIFI_IF_STA, &current_bandwidth) == ESP_OK &&
      current_bandwidth == WIFI_BANDWIDTH_CSI) {
    return ESP_OK;
  }
  return esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BANDWIDTH_CSI);
}

bool wifi_csi_policy_is_active_(WifiBandPolicy policy) {
#if ESPECTRE_WIFI_DUAL_BAND
  if (policy == WifiBandPolicy::AUTO) {
    wifi_band_mode_t band_mode = WIFI_BAND_MODE_AUTO;
    wifi_protocols_t protocols{};
    wifi_bandwidths_t bandwidths{};
    return esp_wifi_get_band_mode(&band_mode) == ESP_OK &&
           band_mode == WIFI_BAND_MODE_AUTO &&
           esp_wifi_get_protocols(WIFI_IF_STA, &protocols) == ESP_OK &&
           protocols.ghz_2g == WIFI_PROTOCOL_CSI_2G &&
           protocols.ghz_5g == WIFI_PROTOCOL_CSI_5G &&
           esp_wifi_get_bandwidths(WIFI_IF_STA, &bandwidths) == ESP_OK &&
           bandwidths.ghz_2g == WIFI_BANDWIDTH_CSI &&
           bandwidths.ghz_5g == WIFI_BANDWIDTH_CSI;
  }
  wifi_band_mode_t band_mode = WIFI_BAND_MODE_AUTO;
  if (esp_wifi_get_band_mode(&band_mode) != ESP_OK ||
      band_mode != band_mode_for_policy_(policy)) {
    return false;
  }
#endif

  const uint8_t requested_protocol =
#if ESPECTRE_WIFI_DUAL_BAND
      policy == WifiBandPolicy::BAND_5G ? WIFI_PROTOCOL_CSI_5G : WIFI_PROTOCOL_CSI_2G;
#else
      WIFI_PROTOCOL_CSI_2G;
#endif
  uint8_t protocol = 0U;
  wifi_bandwidth_t bandwidth = WIFI_BW_HT20;
  return esp_wifi_get_protocol(WIFI_IF_STA, &protocol) == ESP_OK &&
         protocol == requested_protocol &&
         esp_wifi_get_bandwidth(WIFI_IF_STA, &bandwidth) == ESP_OK &&
         bandwidth == WIFI_BANDWIDTH_CSI;
}

void log_wifi_protocol_state_(const char *log_tag, WifiBandPolicy policy) {
#if ESPECTRE_WIFI_DUAL_BAND
  if (policy == WifiBandPolicy::AUTO) {
    wifi_protocols_t protocols{};
    const esp_err_t err = esp_wifi_get_protocols(WIFI_IF_STA, &protocols);
    if (err != ESP_OK) {
      ESPECTRE_LOGW(log_tag, "Wi-Fi protocol: unavailable (%s)", esp_err_to_name(err));
      return;
    }
    ESPECTRE_LOGD(log_tag, "Wi-Fi protocol: 2.4 GHz 0x%04X, 5 GHz 0x%04X",
             static_cast<unsigned>(protocols.ghz_2g), static_cast<unsigned>(protocols.ghz_5g));
    if ((protocols.ghz_2g & WIFI_PROTOCOL_11N) == 0U ||
        (protocols.ghz_5g & WIFI_PROTOCOL_11N) == 0U) {
      ESPECTRE_LOGW(log_tag, "Wi-Fi protocol does not include 11n on both bands");
    }
    return;
  }
#endif
  uint8_t protocol = 0U;
  const esp_err_t err = esp_wifi_get_protocol(WIFI_IF_STA, &protocol);
  if (err != ESP_OK) {
    ESPECTRE_LOGW(log_tag, "Wi-Fi protocol: unavailable (%s)", esp_err_to_name(err));
    return;
  }
  const int has_11n = (protocol & WIFI_PROTOCOL_11N) ? 1 : 0;
  ESPECTRE_LOGD(log_tag, "Wi-Fi protocol: band=%s bitmap=0x%04X (802.11n=%d)",
           wifi_band_policy_to_str_(policy), static_cast<unsigned>(protocol), has_11n);
  if (has_11n == 0) {
    ESPECTRE_LOGW(log_tag, "Wi-Fi protocol does not include 11n support: 0x%04X",
             static_cast<unsigned>(protocol));
  }
}

void log_wifi_bandwidth_state_(const char *log_tag, WifiBandPolicy policy) {
#if ESPECTRE_WIFI_DUAL_BAND
  if (policy == WifiBandPolicy::AUTO) {
    wifi_bandwidths_t bandwidths{};
    const esp_err_t err = esp_wifi_get_bandwidths(WIFI_IF_STA, &bandwidths);
    if (err != ESP_OK) {
      ESPECTRE_LOGW(log_tag, "Wi-Fi bandwidth: unavailable (%s)", esp_err_to_name(err));
      return;
    }
    ESPECTRE_LOGD(log_tag, "Wi-Fi bandwidth: 2.4 GHz %s, 5 GHz %s",
             bandwidth_to_str_(bandwidths.ghz_2g), bandwidth_to_str_(bandwidths.ghz_5g));
    return;
  }
#endif
  wifi_bandwidth_t bw = WIFI_BW_HT20;
  const esp_err_t err = esp_wifi_get_bandwidth(WIFI_IF_STA, &bw);
  if (err != ESP_OK) {
    ESPECTRE_LOGW(log_tag, "Wi-Fi bandwidth: unavailable (%s)", esp_err_to_name(err));
    return;
  }
  ESPECTRE_LOGD(log_tag, "Wi-Fi bandwidth: band=%s width=%s",
           wifi_band_policy_to_str_(policy), bandwidth_to_str_(bw));
}

}  // namespace

esp_err_t WiFiLifecycleManager::apply_csi_wifi_policy(WifiBandPolicy band_policy) {
  if (!wifi_band_policy_is_supported(band_policy)) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Wi-Fi band policy is not supported by this target: %s",
             wifi_band_policy_to_str_(band_policy));
    return ESP_ERR_NOT_SUPPORTED;
  }
#if ESPECTRE_WIFI_DUAL_BAND
  const esp_err_t band_err = set_wifi_band_mode_for_csi_(band_policy);
  if (band_err != ESP_OK) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Failed to apply Wi-Fi band policy %s: %s",
             wifi_band_policy_to_str_(band_policy), esp_err_to_name(band_err));
    return band_err;
  }
#endif

  // Configure WiFi protocol mode (MUST be done before CSI configuration)
  // This initializes internal WiFi structures required for CSI.
  // Keep HE disabled while permitting VHT on the 5 GHz C5 path.
  const WiFiProtocolPolicyResult protocol_result = set_wifi_protocol_for_csi_(band_policy);
  esp_err_t ret = protocol_result.err;
  if (ret != ESP_OK) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Failed to set Wi-Fi protocol: %s", esp_err_to_name(ret));
    return ret;
  }
  ESPECTRE_LOGI(WIFI_LIFECYCLE_TAG, "Wi-Fi CSI protocol policy: path=%s target=%s",
           protocol_policy_path_to_str_(protocol_result.path), wifi_csi_policy_target_(band_policy));
  // HT20 bandwidth for 64 subcarriers
  ret = set_wifi_bandwidth_for_csi_(band_policy);
  if (ret != ESP_OK) {
    ESPECTRE_LOGW(WIFI_LIFECYCLE_TAG, "Failed to set bandwidth: %s", esp_err_to_name(ret));
    // Non-fatal: continue anyway
  }

  return ESP_OK;
}

esp_err_t WiFiLifecycleManager::apply_started_csi_policy(WifiBandPolicy band_policy) {
#ifdef ESPECTRE_HAVE_ESP_COEXIST
  const esp_err_t coex_err = esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
  if (coex_err != ESP_OK) {
    ESPECTRE_LOGW(WIFI_LIFECYCLE_TAG, "Failed to bias Wi-Fi/BT coexistence toward Wi-Fi: %s",
             esp_err_to_name(coex_err));
  }
#endif

  const esp_err_t policy_err = apply_csi_wifi_policy(band_policy);
  if (policy_err != ESP_OK) {
    ESPECTRE_LOGW(WIFI_LIFECYCLE_TAG, "Failed to apply started Wi-Fi CSI policy: %s",
             esp_err_to_name(policy_err));
    return policy_err;
  }
  log_csi_runtime_state(WIFI_LIFECYCLE_TAG, band_policy);
  return ESP_OK;
}

esp_err_t WiFiLifecycleManager::reinitialize_stopped_station_driver(wifi_storage_t storage) {
  esp_err_t err = esp_wifi_deinit();
  if (err != ESP_OK) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Failed to deinitialize Wi-Fi driver: %s",
                  esp_err_to_name(err));
    return err;
  }

#ifdef ESP_PLATFORM
  // esp_wifi_deinit() powers down the shared Wi-Fi PHY domain. Give the radio
  // a real off interval so the CSI measurement block cannot retain its final
  // sample when the driver is initialized again in the same application.
  vTaskDelay(pdMS_TO_TICKS(WIFI_DRIVER_POWER_CYCLE_SETTLE_MS));
#endif

  wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&wifi_config);
  if (err == ESP_OK) err = esp_wifi_set_promiscuous(false);
  if (err == ESP_OK) err = esp_wifi_set_storage(storage);
  if (err == ESP_OK) err = esp_wifi_set_mode(WIFI_MODE_STA);
  if (err != ESP_OK) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Failed to rebuild Wi-Fi station driver: %s",
                  esp_err_to_name(err));
    return err;
  }

  ESPECTRE_LOGI(WIFI_LIFECYCLE_TAG,
                "Wi-Fi station driver rebuilt with promiscuous mode disabled");
  wifi_driver_generation.fetch_add(1U, std::memory_order_release);
  return ESP_OK;
}

// Configure WiFi for optimal CSI capture
esp_err_t WiFiLifecycleManager::init() {
  if (ready_) {
    return ESP_OK;
  }

  ESPECTRE_LOGI(WIFI_LIFECYCLE_TAG, "Initializing Wi-Fi CSI lifecycle");
  if (!started_policy_attempted_.load(std::memory_order_relaxed)) {
    // STA_START fired before the handlers were registered, so the policy was
    // never even attempted. Failing here consumed the GOT_IP event and left
    // CSI off with nothing to retry until the next reconnect. The station is
    // up by now, so applying it late is valid and strictly better than
    // dropping the connection.
    ESPECTRE_LOGW(WIFI_LIFECYCLE_TAG, "STA start was not observed; applying Wi-Fi CSI policy late");
    const esp_err_t late_err = apply_started_csi_policy(band_policy_);
    started_policy_attempted_.store(true, std::memory_order_relaxed);
    started_policy_err_.store(late_err, std::memory_order_relaxed);
    started_policy_applied_.store(late_err == ESP_OK, std::memory_order_relaxed);
    if (late_err == ESP_OK) {
      started_policy_driver_generation_.store(
          wifi_driver_generation.load(std::memory_order_acquire), std::memory_order_relaxed);
    }
  }

  // A policy that was attempted and failed is a real radio failure, not an
  // ordering accident, so it propagates instead of being retried.
  const esp_err_t policy_err = started_policy_err_.load(std::memory_order_relaxed);
  if (policy_err != ESP_OK) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Wi-Fi CSI policy was not applied at STA start: %s",
             esp_err_to_name(policy_err));
    return policy_err;
  }

  // Reassert the runtime power-save policy only after the station is up.
  // This matches the older CSI lifecycle that behaved correctly on ESP32 and
  // avoids toggling the PS mode during the early STA bootstrap.
  const esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
  if (ps_err != ESP_OK) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Failed to disable Wi-Fi power save: %s",
             esp_err_to_name(ps_err));
    return ps_err;
  }

  ESPECTRE_LOGI(WIFI_LIFECYCLE_TAG, "Wi-Fi CSI lifecycle ready");
  log_csi_runtime_state(WIFI_LIFECYCLE_TAG, band_policy_);
  ready_ = true;

  return ESP_OK;
}

esp_err_t WiFiLifecycleManager::register_handlers(wifi_connected_callback_t connected_cb,
                                                  wifi_disconnected_callback_t disconnected_cb,
                                                  WifiBandPolicy band_policy) {
  if (!wifi_band_policy_is_supported(band_policy)) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Wi-Fi band policy is not supported by this target: %s",
             wifi_band_policy_to_str_(band_policy));
    return ESP_ERR_NOT_SUPPORTED;
  }
  connected_callback_ = connected_cb;
  disconnected_callback_ = disconnected_cb;
  band_policy_ = band_policy;
  
  started_policy_err_.store(ESP_ERR_INVALID_STATE, std::memory_order_relaxed);
  started_policy_applied_.store(false, std::memory_order_relaxed);
  started_policy_attempted_.store(false, std::memory_order_relaxed);
  esp_err_t err = esp_event_handler_instance_register(
      WIFI_EVENT,
      WIFI_EVENT_STA_START,
      &WiFiLifecycleManager::wifi_event_handler_,
      this,
      &started_instance_
  );
  if (err != ESP_OK) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Failed to register started handler: %s", esp_err_to_name(err));
    return err;
  }

  // Register WiFi connected event (IP_EVENT_STA_GOT_IP)
  err = esp_event_handler_instance_register(
      IP_EVENT,
      IP_EVENT_STA_GOT_IP,
      &WiFiLifecycleManager::ip_event_handler_,
      this,
      &connected_instance_
  );
  
  if (err != ESP_OK) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Failed to register connected handler: %s", esp_err_to_name(err));
    esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_START, started_instance_);
    started_instance_ = nullptr;
    return err;
  }
  
  // Register WiFi disconnected event
  err = esp_event_handler_instance_register(
      WIFI_EVENT,
      WIFI_EVENT_STA_DISCONNECTED,
      &WiFiLifecycleManager::wifi_event_handler_,
      this,
      &disconnected_instance_
  );
  
  if (err != ESP_OK) {
    ESPECTRE_LOGE(WIFI_LIFECYCLE_TAG, "Failed to register disconnected handler: %s", esp_err_to_name(err));
    // Cleanup connected handler
    if (connected_instance_) {
      esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, connected_instance_);
      connected_instance_ = nullptr;
    }
    if (started_instance_) {
      esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_START, started_instance_);
      started_instance_ = nullptr;
    }
    return err;
  }

  err = esp_event_handler_instance_register(
      WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &WiFiLifecycleManager::wifi_event_handler_,
      this, &associated_instance_);
  if (err != ESP_OK) {
    unregister_handlers();
    return err;
  }

  // Registration may happen after the product has already joined Wi-Fi. In
  // that case GOT_IP is historical and no event will wake the runtime. Apply
  // a missed STA-start policy before restoring that state: changing protocol
  // or bandwidth can asynchronously reconnect the station, and the cached IP
  // and AP record remain readable during that transition. When a setting did
  // change, wait for the next real GOT_IP instead of starting CSI from stale
  // association data.
  esp_netif_t *station = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  esp_netif_ip_info_t current_ip{};
  wifi_ap_record_t current_ap{};
  const bool has_current_station =
      station != nullptr && esp_netif_get_ip_info(station, &current_ip) == ESP_OK &&
      current_ip.ip.addr != 0U && esp_wifi_sta_get_ap_info(&current_ap) == ESP_OK;
  bool restore_current_station = has_current_station;
  if (has_current_station && !started_policy_attempted_.load(std::memory_order_relaxed)) {
    const bool policy_was_active = wifi_csi_policy_is_active_(band_policy_);
    const esp_err_t late_err = apply_started_csi_policy(band_policy_);
    started_policy_attempted_.store(true, std::memory_order_relaxed);
    started_policy_err_.store(late_err, std::memory_order_relaxed);
    started_policy_applied_.store(late_err == ESP_OK, std::memory_order_relaxed);
    if (late_err == ESP_OK) {
      started_policy_driver_generation_.store(
          wifi_driver_generation.load(std::memory_order_acquire), std::memory_order_relaxed);
    }
    if (late_err != ESP_OK) {
      unregister_handlers();
      return late_err;
    }

    restore_current_station = policy_was_active;
    if (!restore_current_station) {
      // A failed getter is indistinguishable from a setting that changed.
      // Force a complete station transition so either DISCONNECTED or a fresh
      // GOT_IP advances the deferred state instead of waiting for an event
      // that the policy setters are not required to emit.
      const esp_err_t disconnect_err = esp_wifi_disconnect();
      if (disconnect_err != ESP_OK && disconnect_err != ESP_ERR_WIFI_NOT_CONNECT) {
        unregister_handlers();
        return disconnect_err;
      }
      const esp_err_t connect_err = esp_wifi_connect();
      if (connect_err != ESP_OK) {
        unregister_handlers();
        return connect_err;
      }
      ESPECTRE_LOGI(WIFI_LIFECYCLE_TAG,
                    "Waiting for fresh GOT_IP after restarting the station for the late Wi-Fi CSI policy");
    }
  }
  if (restore_current_station &&
      !pending_events_.post_overwrite_oldest(
          PendingWifiEvent{PendingWifiEventType::CONNECTED, current_ip})) {
    ESPECTRE_LOGW(WIFI_LIFECYCLE_TAG,
             "Wi-Fi event queue overflowed while restoring the current station state");
  }
  
  ESPECTRE_LOGI(WIFI_LIFECYCLE_TAG, "Wi-Fi event handlers registered");
  return ESP_OK;
}

void WiFiLifecycleManager::unregister_handlers() {
  cancel_csi_receive_path_refresh();
  if (associated_instance_) {
    esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, associated_instance_);
    associated_instance_ = nullptr;
  }
  if (started_instance_) {
    esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_START, started_instance_);
    started_instance_ = nullptr;
  }
  if (connected_instance_) {
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, connected_instance_);
    connected_instance_ = nullptr;
  }

  if (disconnected_instance_) {
    esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, disconnected_instance_);
    disconnected_instance_ = nullptr;
  }

  pending_events_.clear();
  csi_rx_refresh_callback_ = {};
  started_policy_err_.store(ESP_ERR_INVALID_STATE, std::memory_order_relaxed);
  started_policy_applied_.store(false, std::memory_order_relaxed);
  started_policy_attempted_.store(false, std::memory_order_relaxed);
  started_policy_driver_generation_.store(
      wifi_driver_generation.load(std::memory_order_acquire), std::memory_order_relaxed);
  ready_ = false;
  roaming_ = false;
  active_ip_info_ = {};
  ESPECTRE_LOGI(WIFI_LIFECYCLE_TAG, "Wi-Fi event handlers unregistered");
}

esp_err_t WiFiLifecycleManager::process_pending_events() {
  PendingWifiEvent event;
  while (pending_events_.take(event)) {
    if (event.type == PendingWifiEventType::CSI_RX_REFRESHED) {
      if (event.refresh_generation !=
          csi_rx_refresh_generation_.load(std::memory_order_acquire)) {
        continue;
      }
      if (scan_done_instance_) {
        esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, scan_done_instance_);
        scan_done_instance_ = nullptr;
      }
      wifi_csi_rx_refresh_callback_t callback = std::move(csi_rx_refresh_callback_);
      csi_rx_refresh_callback_ = {};
      if (callback) {
        callback(event.result);
      }
      continue;
    }
    if (event.type == PendingWifiEventType::DISCONNECTED) {
      roaming_ = event.roaming && (ready_ || roaming_);
      ready_ = false;
      active_ip_info_ = {};
      if (disconnected_callback_) {
        disconnected_callback_();
      }
      continue;
    }

    if (event.type == PendingWifiEventType::ASSOCIATED) {
      // Initial association and ordinary reconnects must wait for GOT_IP.
      // Roaming can retain the IP stack and omit that event altogether.
      if (!ready_ && !roaming_) {
        continue;
      }
      if (ready_ && disconnected_callback_) {
        disconnected_callback_();
      }
      ready_ = false;
      roaming_ = false;
      active_ip_info_ = {};
      esp_netif_t *station = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
      wifi_ap_record_t ap{};
      if (station == nullptr || esp_netif_get_ip_info(station, &event.ip_info) != ESP_OK ||
          event.ip_info.ip.addr == 0U || esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        continue;
      }
    }

    const bool duplicate = ready_ &&
        active_ip_info_.ip.addr == event.ip_info.ip.addr &&
        active_ip_info_.netmask.addr == event.ip_info.netmask.addr &&
        active_ip_info_.gw.addr == event.ip_info.gw.addr;
    if (duplicate) {
      continue;
    }

    ESPECTRE_LOGD(WIFI_LIFECYCLE_TAG, "Wi-Fi connected event received");
    const esp_err_t err = init();
    if (err != ESP_OK) {
      return err;
    }
    if (connected_callback_) {
      connected_callback_(event.ip_info);
    }
    active_ip_info_ = event.ip_info;
    roaming_ = false;
  }
  return ESP_OK;
}

esp_err_t WiFiLifecycleManager::refresh_csi_receive_path(
    wifi_csi_rx_refresh_callback_t callback) {
  if (!callback) {
    return ESP_ERR_INVALID_ARG;
  }
  if (scan_done_instance_ != nullptr || csi_rx_refresh_callback_) {
    return ESP_ERR_INVALID_STATE;
  }

  const esp_err_t promiscuous_err = esp_wifi_set_promiscuous(false);
  if (promiscuous_err != ESP_OK) {
    return promiscuous_err;
  }

  csi_rx_refresh_generation_.fetch_add(1U, std::memory_order_acq_rel);
  esp_err_t err = esp_event_handler_instance_register(
      WIFI_EVENT,
      WIFI_EVENT_SCAN_DONE,
      &WiFiLifecycleManager::wifi_event_handler_,
      this,
      &scan_done_instance_);
  if (err != ESP_OK) {
    return err;
  }

  csi_rx_refresh_callback_ = std::move(callback);
  err = esp_wifi_scan_start(nullptr, false);
  if (err != ESP_OK) {
    csi_rx_refresh_generation_.fetch_add(1U, std::memory_order_acq_rel);
    esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, scan_done_instance_);
    scan_done_instance_ = nullptr;
    csi_rx_refresh_callback_ = {};
  }
  return err;
}

void WiFiLifecycleManager::cancel_csi_receive_path_refresh() {
  if (scan_done_instance_ == nullptr && !csi_rx_refresh_callback_) {
    return;
  }

  csi_rx_refresh_generation_.fetch_add(1U, std::memory_order_acq_rel);
  csi_rx_refresh_callback_ = {};
  const esp_err_t stop_err = esp_wifi_scan_stop();
  if (stop_err != ESP_OK) {
    ESPECTRE_LOGD(WIFI_LIFECYCLE_TAG,
                  "CSI receive-path refresh scan was already stopped: %s",
                  esp_err_to_name(stop_err));
  }
  if (scan_done_instance_) {
    esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, scan_done_instance_);
    scan_done_instance_ = nullptr;
  }
}

void WiFiLifecycleManager::log_csi_runtime_state(const char *tag, WifiBandPolicy band_policy) {
  const char *log_tag = tag != nullptr ? tag : WIFI_LIFECYCLE_TAG;
  bool promiscuous = false;
  esp_wifi_get_promiscuous(&promiscuous);
  ESPECTRE_LOGD(log_tag, "Wi-Fi promiscuous mode: %s", promiscuous ? "ENABLED" : "DISABLED");

  wifi_ps_type_t ps_type;
  esp_err_t ps_err = esp_wifi_get_ps(&ps_type);
  if (ps_err == ESP_OK) {
    const char* ps_str = (ps_type == WIFI_PS_NONE) ? "NONE" :
                         (ps_type == WIFI_PS_MIN_MODEM) ? "MIN_MODEM" : "MAX_MODEM";
    ESPECTRE_LOGD(log_tag, "Wi-Fi power save: %s", ps_str);
  } else {
    ESPECTRE_LOGW(log_tag, "Wi-Fi power save: unavailable (%s)", esp_err_to_name(ps_err));
  }

  log_wifi_protocol_state_(log_tag, band_policy);
  log_wifi_bandwidth_state_(log_tag, band_policy);

  // The associated band is the one operational fact the per-band settings do
  // not reveal, and it decides which of them is actually in force.
  uint8_t primary_channel = 0U;
  wifi_second_chan_t second_channel = WIFI_SECOND_CHAN_NONE;
  if (esp_wifi_get_channel(&primary_channel, &second_channel) == ESP_OK && primary_channel != 0U) {
    ESPECTRE_LOGD(log_tag, "Wi-Fi channel: %u (%s)", static_cast<unsigned>(primary_channel),
             primary_channel > WIFI_CHANNEL_2G_MAX ? "5 GHz" : "2.4 GHz");
  }
}

// The event handlers run on the default event loop task (sys_evt), so they
// must remain short and non-blocking. STA start applies the radio policy before
// association; IP and disconnect events are drained from the runtime loop.
void WiFiLifecycleManager::ip_event_handler_(void* arg, esp_event_base_t event_base,
                                             int32_t event_id, void* event_data) {
  (void)event_base;
  WiFiLifecycleManager* manager = static_cast<WiFiLifecycleManager*>(arg);

  if (manager != nullptr && event_id == IP_EVENT_STA_GOT_IP && event_data != nullptr) {
    const auto *event = static_cast<const ip_event_got_ip_t *>(event_data);
    if (!manager->pending_events_.post_overwrite_oldest(
            PendingWifiEvent{PendingWifiEventType::CONNECTED, event->ip_info})) {
      ESPECTRE_LOGW(WIFI_LIFECYCLE_TAG, "Wi-Fi event queue overflowed; oldest transition discarded");
    }
  }
}

void WiFiLifecycleManager::wifi_event_handler_(void* arg, esp_event_base_t event_base,
                                               int32_t event_id, void* event_data) {

  (void)event_base;
  (void)event_data;

  WiFiLifecycleManager* manager = static_cast<WiFiLifecycleManager*>(arg);

  if (manager == nullptr) {
    return;
  }
  if (event_id == WIFI_EVENT_STA_START) {
    const uint32_t current_driver_generation =
        wifi_driver_generation.load(std::memory_order_acquire);
    if (!manager->started_policy_applied_.load(std::memory_order_relaxed) ||
        manager->started_policy_driver_generation_.load(std::memory_order_relaxed) !=
            current_driver_generation) {
      const esp_err_t err = apply_started_csi_policy(manager->band_policy_);
      manager->started_policy_attempted_.store(true, std::memory_order_relaxed);
      manager->started_policy_err_.store(err, std::memory_order_relaxed);
      if (err == ESP_OK) {
        manager->started_policy_applied_.store(true, std::memory_order_relaxed);
        manager->started_policy_driver_generation_.store(
            current_driver_generation, std::memory_order_relaxed);
      }
    }
  } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
    if (!manager->pending_events_.post_overwrite_oldest(
            PendingWifiEvent{PendingWifiEventType::ASSOCIATED, {}})) {
      ESPECTRE_LOGW(WIFI_LIFECYCLE_TAG, "Wi-Fi event queue overflowed; oldest transition discarded");
    }
  } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
    const auto *event = static_cast<const wifi_event_sta_disconnected_t *>(event_data);
    PendingWifiEvent pending{PendingWifiEventType::DISCONNECTED, {}};
    pending.roaming = event != nullptr && event->reason == WIFI_REASON_ROAMING;
    if (!manager->pending_events_.post_overwrite_oldest(pending)) {
      ESPECTRE_LOGW(WIFI_LIFECYCLE_TAG, "Wi-Fi event queue overflowed; oldest transition discarded");
    }
  } else if (event_id == WIFI_EVENT_SCAN_DONE) {
    const auto *event = static_cast<const wifi_event_sta_scan_done_t *>(event_data);
    const esp_err_t result = event != nullptr && event->status == 0U ? ESP_OK : ESP_FAIL;
    if (!manager->pending_events_.post_overwrite_oldest(
            PendingWifiEvent{PendingWifiEventType::CSI_RX_REFRESHED, {}, result,
                             manager->csi_rx_refresh_generation_.load(
                                 std::memory_order_acquire)})) {
      ESPECTRE_LOGW(WIFI_LIFECYCLE_TAG,
                    "Wi-Fi event queue overflowed; CSI receive refresh discarded");
    }
  }
}

}  // namespace espectre
