/*
 * ESPectre - Wi-Fi Lifecycle Manager
 *
 * Controls STA lifecycle and HT20 CSI compatibility for sensing runtimes.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include "esp_event.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include <functional>

#include "pending_queue.h"
#include "runtime_interface.h"

namespace espectre {

// Callback types
using wifi_connected_callback_t = std::function<void(const esp_netif_ip_info_t &)>;
using wifi_disconnected_callback_t = std::function<void()>;
using wifi_csi_rx_refresh_callback_t = std::function<void(esp_err_t)>;

/**
 * WiFi Lifecycle Manager
 *
 * Manages WiFi connection events and coordinates service lifecycle.
 * Handles startup sequence: CSI → Traffic Generator → Band Calibration
 *
 * The STA-start handler applies the short radio policy synchronously, before
 * association. Connect/disconnect callbacks run from process_pending_events(),
 * which the runtime must call from its loop task. This keeps service startup
 * and log formatting off the small default event loop task (sys_evt) stack.
 */
class WiFiLifecycleManager {
 public:
  /**
   * Register WiFi event handlers
   * 
   * @param connected_cb Callback when WiFi obtains or retains an IPv4 configuration;
   *        receives the address, netmask, and gateway after GOT_IP or reassociation
   * @param disconnected_cb Callback when WiFi disconnects
   * @return ESP_OK on success. If the default station already has an IPv4
   *         address, its current state is queued for process_pending_events().
   */
  esp_err_t register_handlers(wifi_connected_callback_t connected_cb,
                              wifi_disconnected_callback_t disconnected_cb,
                              WifiBandPolicy band_policy = WifiBandPolicy::BAND_2G);
  
  /**
   * Unregister WiFi event handlers
   */
  void unregister_handlers();

  /**
   * Invoke the registered callbacks for events recorded by the handlers.
   *
   * Must be called periodically from the runtime loop task. Events are
   * processed in the same order in which the ESP event loop recorded them.
   */
  esp_err_t process_pending_events();

  /**
   * Run the asynchronous station scan required to refresh the CSI receive
   * path after reassociation. Promiscuous mode is kept disabled throughout.
   * The completion callback runs from process_pending_events().
   */
  esp_err_t refresh_csi_receive_path(wifi_csi_rx_refresh_callback_t callback);

  /**
   * Cancel an in-flight CSI receive-path refresh. Late scan completion events
   * are invalidated and cannot invoke the canceled callback.
   */
  void cancel_csi_receive_path_refresh();

  /**
   * Apply the short CSI radio policy that must run after WIFI_EVENT_STA_START
   * and before association. Safe to call more than once; later calls are
   * no-ops once protocol, bandwidth, and power-save already match.
   */
  static esp_err_t apply_started_csi_policy(WifiBandPolicy band_policy = WifiBandPolicy::BAND_2G);

  /**
   * Reinitialize an already-stopped station driver and restore the invariants
   * required by every CSI frontend. The caller remains responsible for
   * applying its station configuration and starting the driver.
   */
  static esp_err_t reinitialize_stopped_station_driver(
      wifi_storage_t storage = WIFI_STORAGE_RAM);

 private:
  esp_err_t init();
  static esp_err_t apply_csi_wifi_policy(WifiBandPolicy band_policy);
  static void log_csi_runtime_state(const char *tag, WifiBandPolicy band_policy);

  // Static handlers for ESP-IDF C API (separated by event type)
  static void ip_event_handler_(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
  static void wifi_event_handler_(void* arg, esp_event_base_t event_base,
                                  int32_t event_id, void* event_data);
  
  // Callbacks
  wifi_connected_callback_t connected_callback_;
  wifi_disconnected_callback_t disconnected_callback_;
  wifi_csi_rx_refresh_callback_t csi_rx_refresh_callback_;
  
  // Event handler instances
  esp_event_handler_instance_t connected_instance_{nullptr};
  esp_event_handler_instance_t disconnected_instance_{nullptr};
  esp_event_handler_instance_t associated_instance_{nullptr};
  esp_event_handler_instance_t started_instance_{nullptr};
  esp_event_handler_instance_t scan_done_instance_{nullptr};

  enum class PendingWifiEventType : uint8_t {
    CONNECTED,
    DISCONNECTED,
    ASSOCIATED,
    CSI_RX_REFRESHED,
  };

  struct PendingWifiEvent {
    PendingWifiEventType type{PendingWifiEventType::DISCONNECTED};
    esp_netif_ip_info_t ip_info{};
    esp_err_t result{ESP_OK};
    uint32_t refresh_generation{0U};
    bool roaming{false};
  };

  // Wi-Fi transitions are infrequent; this absorbs a short event-loop burst
  // while preserving the latest state if the owner loop is delayed.
  PendingQueue<PendingWifiEvent, 8U> pending_events_;
  std::atomic<esp_err_t> started_policy_err_{ESP_ERR_INVALID_STATE};
  std::atomic<bool> started_policy_applied_{false};
  std::atomic<uint32_t> started_policy_driver_generation_{0U};
  std::atomic<uint32_t> csi_rx_refresh_generation_{0U};
  // Distinguishes "STA_START never reached us" from "it did and the policy
  // failed". Only the first is recoverable at GOT_IP; the second is a real
  // radio failure that must propagate. The error code alone cannot tell them
  // apart, because esp_wifi_set_protocol can itself return the same
  // ESP_ERR_INVALID_STATE this is seeded with.
  std::atomic<bool> started_policy_attempted_{false};
  WifiBandPolicy band_policy_{WifiBandPolicy::BAND_2G};
  bool ready_{false};
  bool roaming_{false};
  esp_netif_ip_info_t active_ip_info_{};
};

}  // namespace espectre
