/*
 * ESPectre - WiFi Lifecycle Unit Tests
 *
 * Unit tests for WiFi Lifecycle.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include <cstring>
#include <string>

#include "esp_event.h"
#include "esp_wifi.h"
#include "standalone_wifi_service.h"
#include "wifi_lifecycle.h"

using namespace espectre;

namespace espectre {

struct StandaloneWifiServiceTestAccess {
  static bool deferred_connect_fallback_pending(const StandaloneWifiService &service) {
    return service.deferred_connect_fallback_pending_;
  }

  static void expire_deferred_connect_fallback(StandaloneWifiService &service) {
    service.deferred_connect_fallback_deadline_us_ = 0U;
  }

  static bool station_reconfigure_pending(const StandaloneWifiService &service) {
    return service.station_reconfigure_pending_;
  }

  static bool station_disconnect_pending(const StandaloneWifiService &service) {
    return service.station_disconnect_pending_;
  }
};

}  // namespace espectre

void setUp(void) {
  esp_event_mock_reset();
  esp_netif_mock_reset();
  // Most lifecycle tests start before association. Tests for late setup opt
  // into an already configured station explicitly.
  g_esp_netif_mock.ip_addr = 0U;
  esp_wifi_mock_reset();
}

void tearDown(void) {}

void test_wifi_lifecycle_init_configures_protocol_bandwidth_and_promiscuous(void) {
  WiFiLifecycleManager manager;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT40;

  TEST_ASSERT_EQUAL(ESP_OK, manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);

  ip_event_got_ip_t event{};
  event.ip_info.ip.addr = 0x0101A8C0U;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &event);
  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_protocol_call_count);
  TEST_ASSERT_EQUAL(WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
                    g_esp_wifi_mock.last_protocol_bitmap);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_bandwidth_call_count);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.last_bandwidth == WIFI_BW_HT20);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_promiscuous_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_ps_call_count);
  TEST_ASSERT_EQUAL(WIFI_PS_NONE, g_esp_wifi_mock.last_set_ps_type);
}

void test_wifi_lifecycle_init_reports_bgn_configuration_failure(void) {
  WiFiLifecycleManager manager;
  g_esp_wifi_mock.set_protocol_results[0] = ESP_FAIL;
  g_esp_wifi_mock.set_protocol_result_count = 1;

  TEST_ASSERT_EQUAL(ESP_OK, manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);

  ip_event_got_ip_t event{};
  event.ip_info.ip.addr = 0x0101A8C0U;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &event);
  TEST_ASSERT_EQUAL(ESP_FAIL, manager.process_pending_events());
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_protocol_call_count);
  TEST_ASSERT_EQUAL(WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
                    g_esp_wifi_mock.last_protocol_bitmap);
}

void test_wifi_lifecycle_rejects_dual_band_policies_on_single_band_targets(void) {
  WiFiLifecycleManager manager;
  TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED,
                    manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {},
                                              WifiBandPolicy::BAND_5G));
  TEST_ASSERT_EQUAL(0, g_esp_event_mock.register_call_count);

  TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED,
                    manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {},
                                              WifiBandPolicy::AUTO));
  TEST_ASSERT_EQUAL(0, g_esp_event_mock.register_call_count);
}

// STA_START can fire before the handlers are registered. If no connected
// station can be restored yet, the next real GOT_IP applies the missed policy.
void test_wifi_lifecycle_applies_policy_late_when_sta_start_was_missed(void) {
  WiFiLifecycleManager manager;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT40;

  TEST_ASSERT_EQUAL(ESP_OK, manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));
  // Deliberately no WIFI_EVENT_STA_START.
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_protocol_call_count);

  ip_event_got_ip_t event{};
  event.ip_info.ip.addr = 0x0101A8C0U;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &event);

  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_protocol_call_count);
  TEST_ASSERT_EQUAL(WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
                    g_esp_wifi_mock.last_protocol_bitmap);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.last_bandwidth == WIFI_BW_HT20);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_ps_call_count);
}

// A policy that was attempted and failed is a radio failure, not an ordering
// accident, so it must still propagate rather than be retried at GOT_IP.
void test_wifi_lifecycle_does_not_retry_a_policy_that_actually_failed(void) {
  WiFiLifecycleManager manager;
  g_esp_wifi_mock.set_protocol_results[0] = ESP_FAIL;
  g_esp_wifi_mock.set_protocol_result_count = 1;

  TEST_ASSERT_EQUAL(ESP_OK, manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);

  ip_event_got_ip_t event{};
  event.ip_info.ip.addr = 0x0101A8C0U;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &event);

  TEST_ASSERT_EQUAL(ESP_FAIL, manager.process_pending_events());
  // One attempt only: the mock would have succeeded on a second call.
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_protocol_call_count);
}

void test_wifi_lifecycle_started_policy_skips_matching_radio_settings(void) {
  WiFiLifecycleManager manager;
  g_esp_wifi_mock.protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT20;

  TEST_ASSERT_EQUAL(ESP_OK, manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_protocol_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_bandwidth_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_ps_call_count);
}

void test_wifi_lifecycle_register_handlers_dispatches_and_unregisters(void) {
  WiFiLifecycleManager manager;
  int connected_calls = 0;
  int disconnected_calls = 0;
  std::string callback_order;
  esp_netif_ip_info_t observed_ip_info{};

  TEST_ASSERT_EQUAL(
      ESP_OK, manager.register_handlers([&](const esp_netif_ip_info_t &ip_info) {
                                          connected_calls++;
                                          callback_order += "connected ";
                                          observed_ip_info = ip_info;
                                        },
                                        [&disconnected_calls, &callback_order]() {
                                          disconnected_calls++;
                                          callback_order += "disconnected";
                                        }));
  TEST_ASSERT_EQUAL(3, g_esp_event_mock.register_call_count);

  ip_event_got_ip_t event{};
  event.ip_info.ip.addr = 0x0101A8C0U;
  event.ip_info.gw.addr = 0x0101A8C0U;
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &event);
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, nullptr);

  // Handlers only record events; callbacks fire from process_pending_events().
  TEST_ASSERT_EQUAL(0, connected_calls);
  TEST_ASSERT_EQUAL(0, disconnected_calls);

  manager.process_pending_events();
  TEST_ASSERT_EQUAL(1, connected_calls);
  TEST_ASSERT_EQUAL(1, disconnected_calls);
  TEST_ASSERT_EQUAL_STRING("connected disconnected", callback_order.c_str());
  TEST_ASSERT_EQUAL(event.ip_info.gw.addr, observed_ip_info.gw.addr);

  manager.unregister_handlers();
  TEST_ASSERT_EQUAL(3, g_esp_event_mock.unregister_call_count);

  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &event);
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, nullptr);

  manager.process_pending_events();
  TEST_ASSERT_EQUAL(1, connected_calls);
  TEST_ASSERT_EQUAL(1, disconnected_calls);
}

void test_wifi_lifecycle_refreshes_csi_receive_path_without_promiscuous_mode(void) {
  WiFiLifecycleManager manager;
  esp_err_t observed_result = ESP_FAIL;
  int callback_count = 0;

  TEST_ASSERT_EQUAL(
      ESP_OK,
      manager.refresh_csi_receive_path([&](esp_err_t result) {
        observed_result = result;
        callback_count++;
      }));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_promiscuous_call_count);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.last_promiscuous);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.promiscuous);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.scan_start_call_count);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.last_scan_block);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.last_scan_configured);
  TEST_ASSERT_EQUAL(
      ESP_ERR_INVALID_STATE,
      manager.refresh_csi_receive_path([](esp_err_t) {}));

  wifi_event_sta_scan_done_t event{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &event);
  TEST_ASSERT_EQUAL(0, callback_count);
  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(1, callback_count);
  TEST_ASSERT_EQUAL(ESP_OK, observed_result);
}

void test_wifi_lifecycle_csi_receive_refresh_requires_promiscuous_disabled(void) {
  WiFiLifecycleManager manager;
  g_esp_wifi_mock.set_promiscuous_result = ESP_FAIL;

  TEST_ASSERT_EQUAL(
      ESP_FAIL,
      manager.refresh_csi_receive_path([](esp_err_t) {}));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_promiscuous_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.scan_start_call_count);
}

void test_wifi_lifecycle_cancel_invalidates_pending_csi_refresh_completion(void) {
  WiFiLifecycleManager manager;
  int canceled_callback_count = 0;
  int completed_callback_count = 0;

  TEST_ASSERT_EQUAL(
      ESP_OK,
      manager.refresh_csi_receive_path(
          [&](esp_err_t) { canceled_callback_count++; }));
  wifi_event_sta_scan_done_t event{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &event);

  manager.cancel_csi_receive_path_refresh();
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.scan_stop_call_count);
  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(0, canceled_callback_count);

  TEST_ASSERT_EQUAL(
      ESP_OK,
      manager.refresh_csi_receive_path(
          [&](esp_err_t result) {
            TEST_ASSERT_EQUAL(ESP_OK, result);
            completed_callback_count++;
          }));
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &event);
  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(1, completed_callback_count);
}

void test_wifi_lifecycle_restores_an_already_connected_station(void) {
  WiFiLifecycleManager manager;
  int connected_calls = 0;
  esp_netif_ip_info_t observed_ip_info{};
  g_esp_netif_mock.ip_addr = 0x3701A8C0U;
  g_esp_netif_mock.netmask_addr = 0x00FFFFFFU;
  g_esp_netif_mock.gw_addr = 0x0101A8C0U;
  g_esp_wifi_mock.protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT20;

  TEST_ASSERT_EQUAL(
      ESP_OK, manager.register_handlers([&](const esp_netif_ip_info_t &ip_info) {
                                          connected_calls++;
                                          observed_ip_info = ip_info;
                                        },
                                        []() {}));
  TEST_ASSERT_EQUAL(0, connected_calls);

  ip_event_got_ip_t duplicate{};
  duplicate.ip_info.ip.addr = g_esp_netif_mock.ip_addr;
  duplicate.ip_info.netmask.addr = g_esp_netif_mock.netmask_addr;
  duplicate.ip_info.gw.addr = g_esp_netif_mock.gw_addr;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &duplicate);

  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(1, connected_calls);
  TEST_ASSERT_EQUAL(g_esp_netif_mock.ip_addr, observed_ip_info.ip.addr);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_protocol_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_ps_call_count);
}

void test_wifi_lifecycle_waits_for_fresh_ip_after_late_policy_change(void) {
  WiFiLifecycleManager manager;
  int connected_calls = 0;
  g_esp_netif_mock.ip_addr = 0x3701A8C0U;
  g_esp_netif_mock.netmask_addr = 0x00FFFFFFU;
  g_esp_netif_mock.gw_addr = 0x0101A8C0U;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT40;

  TEST_ASSERT_EQUAL(
      ESP_OK, manager.register_handlers([&](const esp_netif_ip_info_t &) { connected_calls++; },
                                        []() {}));
  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(0, connected_calls);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_bandwidth_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.set_bandwidth_sequence <
                   g_esp_wifi_mock.disconnect_sequences[0]);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.disconnect_sequences[0] <
                   g_esp_wifi_mock.connect_sequences[0]);

  ip_event_got_ip_t fresh{};
  fresh.ip_info.ip.addr = g_esp_netif_mock.ip_addr;
  fresh.ip_info.netmask.addr = g_esp_netif_mock.netmask_addr;
  fresh.ip_info.gw.addr = g_esp_netif_mock.gw_addr;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &fresh);
  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(1, connected_calls);
}

void test_wifi_lifecycle_restarts_station_when_late_policy_getter_fails(void) {
  WiFiLifecycleManager manager;
  int connected_calls = 0;
  g_esp_netif_mock.ip_addr = 0x3701A8C0U;
  g_esp_netif_mock.netmask_addr = 0x00FFFFFFU;
  g_esp_netif_mock.gw_addr = 0x0101A8C0U;
  g_esp_wifi_mock.protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT20;
  g_esp_wifi_mock.get_protocol_result = ESP_FAIL;

  TEST_ASSERT_EQUAL(
      ESP_OK, manager.register_handlers([&](const esp_netif_ip_info_t &) { connected_calls++; },
                                        []() {}));
  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(0, connected_calls);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_protocol_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
}

void test_wifi_lifecycle_restarts_station_after_nonfatal_bandwidth_failure(void) {
  WiFiLifecycleManager manager;
  int connected_calls = 0;
  g_esp_netif_mock.ip_addr = 0x3701A8C0U;
  g_esp_netif_mock.netmask_addr = 0x00FFFFFFU;
  g_esp_netif_mock.gw_addr = 0x0101A8C0U;
  g_esp_wifi_mock.protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT40;
  g_esp_wifi_mock.set_bandwidth_result = ESP_FAIL;

  TEST_ASSERT_EQUAL(
      ESP_OK, manager.register_handlers([&](const esp_netif_ip_info_t &) { connected_calls++; },
                                        []() {}));
  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(0, connected_calls);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_bandwidth_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
}

void test_wifi_lifecycle_propagates_fatal_late_policy_failure(void) {
  WiFiLifecycleManager manager;
  g_esp_netif_mock.ip_addr = 0x3701A8C0U;
  g_esp_netif_mock.netmask_addr = 0x00FFFFFFU;
  g_esp_netif_mock.gw_addr = 0x0101A8C0U;
  g_esp_wifi_mock.set_protocol_results[0] = ESP_FAIL;
  g_esp_wifi_mock.set_protocol_result_count = 1;

  TEST_ASSERT_EQUAL(
      ESP_FAIL, manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_protocol_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_EQUAL(3, g_esp_event_mock.unregister_call_count);
}

void test_wifi_lifecycle_propagates_late_station_restart_failure(void) {
  WiFiLifecycleManager manager;
  g_esp_netif_mock.ip_addr = 0x3701A8C0U;
  g_esp_netif_mock.netmask_addr = 0x00FFFFFFU;
  g_esp_netif_mock.gw_addr = 0x0101A8C0U;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT40;
  g_esp_wifi_mock.connect_results[0] = ESP_FAIL;
  g_esp_wifi_mock.connect_result_count = 1;

  TEST_ASSERT_EQUAL(
      ESP_FAIL, manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_EQUAL(3, g_esp_event_mock.unregister_call_count);
}

void test_wifi_lifecycle_does_not_restore_stale_ip_while_station_reconnects(void) {
  WiFiLifecycleManager manager;
  int connected_calls = 0;
  g_esp_netif_mock.ip_addr = 0x3701A8C0U;
  g_esp_netif_mock.netmask_addr = 0x00FFFFFFU;
  g_esp_netif_mock.gw_addr = 0x0101A8C0U;
  g_esp_wifi_mock.protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT20;
  g_esp_wifi_mock.get_ap_info_result = ESP_ERR_WIFI_NOT_CONNECT;

  TEST_ASSERT_EQUAL(
      ESP_OK, manager.register_handlers([&](const esp_netif_ip_info_t &) { connected_calls++; },
                                        []() {}));
  TEST_ASSERT_EQUAL(ESP_OK, manager.process_pending_events());
  TEST_ASSERT_EQUAL(0, connected_calls);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.get_ap_info_call_count);
}

void test_wifi_lifecycle_register_handlers_cleans_up_when_second_registration_fails(void) {
  WiFiLifecycleManager manager;
  g_esp_event_mock.register_results[0] = ESP_OK;
  g_esp_event_mock.register_results[1] = ESP_FAIL;
  g_esp_event_mock.register_result_count = 2;

  TEST_ASSERT_EQUAL(
      ESP_FAIL, manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));
  TEST_ASSERT_EQUAL(2, g_esp_event_mock.register_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_event_mock.unregister_call_count);
}

void test_standalone_wifi_service_configures_fast_scan_bssid_and_channel(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";
  config.password = "secret";
  config.bssid = "aa:bb:cc:dd:ee:ff";
  config.channel = 10;

  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.init_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_storage_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_mode_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_ps_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(WIFI_FAST_SCAN, g_esp_wifi_mock.last_config.sta.scan_method);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.last_config.sta.bssid_set);
  TEST_ASSERT_EQUAL_UINT8(10, g_esp_wifi_mock.last_config.sta.channel);
  TEST_ASSERT_EQUAL_UINT8(0xaa, g_esp_wifi_mock.last_config.sta.bssid[0]);
  TEST_ASSERT_EQUAL_UINT8(0xff, g_esp_wifi_mock.last_config.sta.bssid[5]);
}

void test_standalone_wifi_service_applies_policy_and_connects_on_start(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";
  config.password = "secret";
  config.manage_csi_lifecycle = true;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT40;

  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(ESP_OK, service.start());
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.start_call_count);

  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_ps_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_protocol_call_count);
  TEST_ASSERT_EQUAL(WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
                    g_esp_wifi_mock.last_protocol_bitmap);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_bandwidth_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);

  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_protocol_call_count);
}

void test_wifi_lifecycle_reapplies_policy_after_shared_driver_reinitialize(void) {
  WiFiLifecycleManager manager;
  TEST_ASSERT_EQUAL(ESP_OK, manager.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));

  g_esp_wifi_mock.bandwidth = WIFI_BW_HT40;
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_bandwidth_call_count);

  TEST_ASSERT_EQUAL(
      ESP_OK,
      WiFiLifecycleManager::reinitialize_stopped_station_driver(WIFI_STORAGE_RAM));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.deinit_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.init_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_storage_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_mode_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_promiscuous_call_count);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.last_promiscuous);

  // The mock driver does not reset radio fields during init, so model the
  // post-init default explicitly and verify that the generation change makes
  // the same manager apply its CSI policy again.
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT40;
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.set_bandwidth_call_count);
  manager.unregister_handlers();
}

void test_standalone_wifi_service_unmanaged_applies_policy_before_connect(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";
  config.password = "secret";
  config.manage_csi_lifecycle = false;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT40;

  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(ESP_OK, service.start());

  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_protocol_call_count);
  service.loop();
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_ps_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_protocol_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_bandwidth_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);
}

void test_standalone_wifi_service_reconnects_after_sta_stop(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";
  config.password = "secret";

  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(ESP_OK, service.start());

  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  service.loop();
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);

  // A later STA_START without STOP must not double-connect.
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  service.loop();
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);

  // Protocol/coexistence restarts clear the latch so association can resume.
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_STOP, nullptr);
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  service.loop();
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
}

void test_standalone_wifi_service_runs_deferred_connect_fallback_once(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";
  config.password = "secret";

  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(ESP_OK, service.start());

  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);
  service.loop();
  TEST_ASSERT_TRUE(StandaloneWifiServiceTestAccess::deferred_connect_fallback_pending(service));

  StandaloneWifiServiceTestAccess::expire_deferred_connect_fallback(service);
  service.loop();
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_FALSE(StandaloneWifiServiceTestAccess::deferred_connect_fallback_pending(service));

  service.loop();
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
}

void test_standalone_wifi_service_managed_lifecycle_dispatches_after_csi_init(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";
  config.manage_csi_lifecycle = true;
  int connected_calls = 0;
  int disconnected_calls = 0;

  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config,
                                         [&connected_calls]() { connected_calls++; },
                                         [&disconnected_calls]() { disconnected_calls++; }));

  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  ip_event_got_ip_t event{};
  event.ip_info.ip.addr = 0x0101A8C0U;
  event.ip_info.gw.addr = 0x0101A8C0U;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &event);
  TEST_ASSERT_EQUAL(0, connected_calls);

  service.loop();
  TEST_ASSERT_EQUAL(1, connected_calls);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_ps_call_count);
  TEST_ASSERT_EQUAL(WIFI_PS_NONE, g_esp_wifi_mock.last_set_ps_type);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_promiscuous_call_count);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.last_promiscuous);

  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, nullptr);
  service.loop();
  TEST_ASSERT_EQUAL(1, disconnected_calls);
}

void test_standalone_wifi_service_get_info_reports_station_details(void) {
  StandaloneWifiService service;
  StandaloneWifiInfo info{};
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";

  TEST_ASSERT_FALSE(service.get_info(nullptr));
  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  ip_event_got_ip_t event{};
  g_esp_netif_mock.ip_addr = 0x6401A8C0U;
  event.ip_info.ip.addr = g_esp_netif_mock.ip_addr;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &event);
  service.loop();
  TEST_ASSERT_TRUE(service.get_info(&info));
  TEST_ASSERT_TRUE(info.connected);
  TEST_ASSERT_EQUAL_UINT8(6, info.channel);
  TEST_ASSERT_TRUE(std::string(info.ip_address).find('.') != std::string::npos);
  TEST_ASSERT_EQUAL_STRING("7C:2C:67:42:BB:AC", info.mac_address);
}

void test_standalone_wifi_service_get_info_uses_cached_ip_from_got_ip_event(void) {
  StandaloneWifiService service;
  StandaloneWifiInfo info{};
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";

  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));

  StandaloneWifiInfo before_ip{};
  TEST_ASSERT_TRUE(service.get_info(&before_ip));
  TEST_ASSERT_FALSE(before_ip.connected);
  TEST_ASSERT_EQUAL_STRING("", before_ip.ip_address);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.get_ap_info_call_count);

  g_esp_netif_mock.ip_addr = 0U;

  ip_event_got_ip_t event{};
  event.ip_info.ip.addr =
      ((uint32_t)192U << 0U) | ((uint32_t)168U << 8U) | ((uint32_t)1U << 16U) | ((uint32_t)55U << 24U);
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &event);
  service.loop();

  TEST_ASSERT_TRUE(service.get_info(&info));
  TEST_ASSERT_EQUAL_STRING("192.168.1.55", info.ip_address);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.get_ap_info_call_count);

  wifi_event_sta_disconnected_t disconnect_event{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_event);
  service.loop();

  StandaloneWifiInfo after_disconnect{};
  TEST_ASSERT_TRUE(service.get_info(&after_disconnect));
  TEST_ASSERT_EQUAL_STRING("", after_disconnect.ip_address);
}

void test_standalone_wifi_service_update_station_config_handles_setup_and_reconnect_paths(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "InitialSSID";
  config.password = "secret";

  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, service.update_station_config(config));

  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(ESP_OK, service.update_station_config(config));
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);

  TEST_ASSERT_EQUAL(ESP_OK, service.start());
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  ip_event_got_ip_t got_ip_event{};
  got_ip_event.ip_info.ip.addr = 0x3701A8C0U;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip_event);
  service.loop();
  StandaloneWifiConfig empty = config;
  empty.ssid = "";
  TEST_ASSERT_EQUAL(ESP_OK, service.update_station_config(empty));
  TEST_ASSERT_TRUE(StandaloneWifiServiceTestAccess::station_reconfigure_pending(service));
  TEST_ASSERT_TRUE(StandaloneWifiServiceTestAccess::station_disconnect_pending(service));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.start_call_count);
  wifi_event_sta_disconnected_t disconnect_event{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_event);
  service.loop();
  TEST_ASSERT_FALSE(StandaloneWifiServiceTestAccess::station_reconfigure_pending(service));
  TEST_ASSERT_FALSE(StandaloneWifiServiceTestAccess::station_disconnect_pending(service));
  TEST_ASSERT_EQUAL(3, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.deinit_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.init_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_promiscuous_call_count);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.last_promiscuous);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);

  TEST_ASSERT_EQUAL(ESP_OK, service.update_station_config(config));
  TEST_ASSERT_FALSE(StandaloneWifiServiceTestAccess::station_reconfigure_pending(service));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(4, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.set_config_sequences[3] <
                   g_esp_wifi_mock.connect_sequences[0]);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.start_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_ps_call_count);
}

void test_standalone_wifi_service_update_station_config_handles_idle_station(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;

  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(ESP_OK, service.start());

  StandaloneWifiConfig updated = config;
  updated.ssid = "ProvisionedSSID";
  updated.password = "secret";
  TEST_ASSERT_EQUAL(ESP_OK, service.update_station_config(updated));
  TEST_ASSERT_FALSE(StandaloneWifiServiceTestAccess::station_reconfigure_pending(service));
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.set_config_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.set_config_sequences[1] <
                   g_esp_wifi_mock.connect_sequences[0]);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);
}

void test_standalone_wifi_service_leaves_csi_refresh_to_shared_runtime(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";
  config.password = "secret";
  int connected_count = 0;

  TEST_ASSERT_EQUAL(
      ESP_OK,
      service.setup(config, [&connected_count]() { connected_count++; }));
  TEST_ASSERT_EQUAL(ESP_OK, service.start());
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  ip_event_got_ip_t got_ip{};
  got_ip.ip_info.ip.addr = 0x3701A8C0U;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip);
  service.loop();
  TEST_ASSERT_EQUAL(1, connected_count);

  StandaloneWifiConfig updated = config;
  updated.bssid = "AA:BB:CC:DD:EE:FF";
  updated.channel = 6U;
  TEST_ASSERT_EQUAL(ESP_OK, service.update_station_config(updated));
  wifi_event_sta_disconnected_t disconnected{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnected);
  service.loop();
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.disconnect_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.connect_call_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.stop_call_count);

  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip);
  service.loop();
  TEST_ASSERT_EQUAL(2, connected_count);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.scan_start_call_count);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.last_promiscuous);
}

void test_standalone_wifi_service_update_station_config_rejects_invalid_bssid(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "SSID";

  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  config.bssid = "not-a-bssid";
  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, service.update_station_config(config));
}

void test_standalone_wifi_service_reports_asynchronous_scan_snapshot(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";
  std::vector<StandaloneWifiAccessPoint> observed;
  esp_err_t observed_result = ESP_FAIL;

  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, service.request_scan({}));
  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, service.request_scan({}));
  TEST_ASSERT_EQUAL(ESP_OK, service.start());

  g_esp_wifi_mock.scan_ap_count = 2U;
  std::memcpy(g_esp_wifi_mock.scan_ap_records[0].ssid, "TestSSID", 9U);
  g_esp_wifi_mock.scan_ap_records[0].rssi = -70;
  g_esp_wifi_mock.scan_ap_records[0].primary = 11U;
  const uint8_t weak[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
  std::memcpy(g_esp_wifi_mock.scan_ap_records[0].bssid, weak, sizeof(weak));
  std::memcpy(g_esp_wifi_mock.scan_ap_records[1].ssid, "TestSSID", 9U);
  g_esp_wifi_mock.scan_ap_records[1].rssi = -40;
  g_esp_wifi_mock.scan_ap_records[1].primary = 6U;
  const uint8_t strong[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
  std::memcpy(g_esp_wifi_mock.scan_ap_records[1].bssid, strong, sizeof(strong));

  TEST_ASSERT_EQUAL(ESP_OK, service.request_scan(
      [&observed_result, &observed](esp_err_t result,
                                    const std::vector<StandaloneWifiAccessPoint> &access_points) {
        observed_result = result;
        observed = access_points;
      }));
  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, service.request_scan({}));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.scan_start_call_count);
  TEST_ASSERT_FALSE(g_esp_wifi_mock.last_scan_block);
  TEST_ASSERT_TRUE(g_esp_wifi_mock.last_scan_configured);
  TEST_ASSERT_EQUAL_STRING("TestSSID", g_esp_wifi_mock.last_scan_ssid);
  TEST_ASSERT_EQUAL_UINT8(0U, g_esp_wifi_mock.last_scan_channel);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.disconnect_call_count);

  wifi_event_sta_scan_done_t event{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &event);
  TEST_ASSERT_TRUE(observed.empty());
  service.loop();

  TEST_ASSERT_EQUAL(ESP_OK, observed_result);
  TEST_ASSERT_EQUAL(2U, observed.size());
  TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", observed[0].bssid.c_str());
  TEST_ASSERT_EQUAL_INT8(-40, observed[0].rssi_dbm);
  TEST_ASSERT_EQUAL_UINT8(6U, observed[0].channel);
}

void test_standalone_wifi_service_setup_failure_unregisters_partial_handlers(void) {
  for (bool managed : {false, true}) {
    setUp();
    StandaloneWifiConfig config;
    config.ssid = "TestSSID";
    config.manage_csi_lifecycle = managed;
    StandaloneWifiService probe;
    TEST_ASSERT_EQUAL(ESP_OK, probe.setup(config));
    const int registrations = g_esp_event_mock.register_call_count;
    probe.shutdown();
    for (int failed_registration = 0; failed_registration <= registrations; ++failed_registration) {
      setUp();
      StandaloneWifiService service;
      if (failed_registration < registrations) {
        g_esp_event_mock.register_result_count = registrations;
        for (int index = 0; index < registrations; ++index) {
          g_esp_event_mock.register_results[index] = index == failed_registration ? ESP_FAIL : ESP_OK;
        }
      } else {
        g_esp_wifi_mock.set_config_result_count = 1;
        g_esp_wifi_mock.set_config_results[0] = ESP_FAIL;
      }
      TEST_ASSERT_EQUAL(ESP_FAIL, service.setup(config));
      TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.start_call_count);
      for (const auto &slot : g_esp_event_mock.slots) TEST_ASSERT_FALSE(slot.active);
      service.shutdown();
    }
  }
}

void test_standalone_wifi_service_shutdown_clears_pending_events_and_scan(void) {
  for (bool managed : {false, true}) {
    setUp();
    StandaloneWifiService service;
    StandaloneWifiConfig config;
    config.ssid = "TestSSID";
    config.manage_csi_lifecycle = managed;
    int callbacks = 0;
    TEST_ASSERT_EQUAL(ESP_OK, service.setup(config, [&]() { callbacks++; }));
    TEST_ASSERT_EQUAL(ESP_OK, service.start());
    TEST_ASSERT_EQUAL(ESP_OK, service.request_scan(
        [&](esp_err_t, const std::vector<StandaloneWifiAccessPoint> &) { callbacks++; }));
    ip_event_got_ip_t got_ip{};
    got_ip.ip_info.ip.addr = 0x3701A8C0U;
    esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip);
    esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, nullptr);
    g_esp_wifi_mock.stop_result = ESP_FAIL;
    service.shutdown();
    service.shutdown();
    service.loop();
    TEST_ASSERT_EQUAL(0, callbacks);
    TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.stop_call_count);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, service.request_scan({}));
    StandaloneWifiInfo info;
    service.get_info(&info);
    TEST_ASSERT_FALSE(info.connected);
    TEST_ASSERT_EQUAL_STRING("", info.ip_address);
  }
}

void test_standalone_wifi_service_recovers_after_scan_failures(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "TestSSID";
  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(ESP_OK, service.start());
  g_esp_wifi_mock.scan_start_result = ESP_FAIL;
  TEST_ASSERT_EQUAL(ESP_FAIL, service.request_scan({}));
  g_esp_wifi_mock.scan_start_result = ESP_OK;
  for (int failure = 0; failure < 4; ++failure) {
    int callbacks = 0;
    esp_err_t observed = ESP_OK;
    g_esp_wifi_mock.scan_ap_count = failure == 2 ? 1U : 0U;
    g_esp_wifi_mock.scan_get_ap_num_result = failure == 1 ? ESP_FAIL : ESP_OK;
    g_esp_wifi_mock.scan_get_ap_records_result = failure == 2 ? ESP_FAIL : ESP_OK;
    TEST_ASSERT_EQUAL(ESP_OK, service.request_scan(
        [&](esp_err_t result, const std::vector<StandaloneWifiAccessPoint> &access_points) {
          callbacks++;
          observed = result;
          TEST_ASSERT_TRUE(access_points.empty());
        }));
    wifi_event_sta_scan_done_t event{};
    event.status = failure == 0 ? 1U : 0U;
    esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &event);
    service.loop();
    TEST_ASSERT_EQUAL(1, callbacks);
    TEST_ASSERT_EQUAL(failure == 3 ? ESP_OK : ESP_FAIL, observed);
    esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &event);
    service.loop();
    TEST_ASSERT_EQUAL(1, callbacks);
  }
  service.shutdown();
}

void test_standalone_wifi_service_restarts_driver_when_disconnect_fails(void) {
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "InitialSSID";
  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(ESP_OK, service.start());
  ip_event_got_ip_t got_ip{};
  got_ip.ip_info.ip.addr = 0x3701A8C0U;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip);
  service.loop();
  g_esp_wifi_mock.disconnect_result_count = 1;
  g_esp_wifi_mock.disconnect_results[0] = ESP_ERR_WIFI_CONN;
  config.ssid = "ReplacementSSID";
  TEST_ASSERT_EQUAL(ESP_OK, service.update_station_config(config));
  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, service.update_station_config(config));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.stop_call_count);
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_STOP, nullptr);
  service.loop();
  TEST_ASSERT_FALSE(StandaloneWifiServiceTestAccess::station_reconfigure_pending(service));
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.deinit_call_count);
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.start_call_count);
  TEST_ASSERT_EQUAL_STRING("ReplacementSSID", reinterpret_cast<const char *>(g_esp_wifi_mock.last_config.sta.ssid));
  service.shutdown();
}

void test_standalone_wifi_service_apply_started_policy_and_reconnect_logic(void) {
  WiFiLifecycleManager lifecycle;
  g_esp_wifi_mock.bandwidth = WIFI_BW_HT40;
  TEST_ASSERT_EQUAL(ESP_OK,
                    lifecycle.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.set_ps_call_count);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_protocol_call_count);
  TEST_ASSERT_EQUAL(WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
                    g_esp_wifi_mock.last_protocol_bitmap);
  TEST_ASSERT_EQUAL(1, g_esp_wifi_mock.set_bandwidth_call_count);

  lifecycle.unregister_handlers();
  esp_wifi_mock_reset();
  g_esp_wifi_mock.set_protocol_results[0] = ESP_FAIL;
  g_esp_wifi_mock.set_protocol_result_count = 1;
  WiFiLifecycleManager failing_lifecycle;
  TEST_ASSERT_EQUAL(ESP_OK,
                    failing_lifecycle.register_handlers([](const esp_netif_ip_info_t &) {}, []() {}));
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  ip_event_got_ip_t got_ip{};
  got_ip.ip_info.ip.addr = 0x0101A8C0U;
  esp_event_mock_emit(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip);
  TEST_ASSERT_EQUAL(ESP_FAIL, failing_lifecycle.process_pending_events());

  failing_lifecycle.unregister_handlers();
  esp_wifi_mock_reset();
  StandaloneWifiService service;
  StandaloneWifiConfig config;
  config.ssid = "SSID";
  config.max_retry = 2;
  TEST_ASSERT_EQUAL(ESP_OK, service.setup(config));
  TEST_ASSERT_EQUAL(ESP_OK, service.start());

  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_START, nullptr);
  service.loop();
  TEST_ASSERT_EQUAL(0, g_esp_wifi_mock.connect_call_count);

  wifi_event_sta_disconnected_t event{};
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event);
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event);
  esp_event_mock_emit(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event);
  service.loop();
  TEST_ASSERT_EQUAL(2, g_esp_wifi_mock.connect_call_count);
}

int process(void) {
  UNITY_BEGIN();
  RUN_TEST(test_wifi_lifecycle_init_configures_protocol_bandwidth_and_promiscuous);
  RUN_TEST(test_wifi_lifecycle_init_reports_bgn_configuration_failure);
  RUN_TEST(test_wifi_lifecycle_rejects_dual_band_policies_on_single_band_targets);
  RUN_TEST(test_wifi_lifecycle_applies_policy_late_when_sta_start_was_missed);
  RUN_TEST(test_wifi_lifecycle_does_not_retry_a_policy_that_actually_failed);
  RUN_TEST(test_wifi_lifecycle_started_policy_skips_matching_radio_settings);
  RUN_TEST(test_wifi_lifecycle_register_handlers_dispatches_and_unregisters);
  RUN_TEST(test_wifi_lifecycle_refreshes_csi_receive_path_without_promiscuous_mode);
  RUN_TEST(test_wifi_lifecycle_csi_receive_refresh_requires_promiscuous_disabled);
  RUN_TEST(test_wifi_lifecycle_cancel_invalidates_pending_csi_refresh_completion);
  RUN_TEST(test_wifi_lifecycle_restores_an_already_connected_station);
  RUN_TEST(test_wifi_lifecycle_waits_for_fresh_ip_after_late_policy_change);
  RUN_TEST(test_wifi_lifecycle_restarts_station_when_late_policy_getter_fails);
  RUN_TEST(test_wifi_lifecycle_restarts_station_after_nonfatal_bandwidth_failure);
  RUN_TEST(test_wifi_lifecycle_propagates_fatal_late_policy_failure);
  RUN_TEST(test_wifi_lifecycle_propagates_late_station_restart_failure);
  RUN_TEST(test_wifi_lifecycle_does_not_restore_stale_ip_while_station_reconnects);
  RUN_TEST(test_wifi_lifecycle_register_handlers_cleans_up_when_second_registration_fails);
  RUN_TEST(test_standalone_wifi_service_configures_fast_scan_bssid_and_channel);
  RUN_TEST(test_standalone_wifi_service_applies_policy_and_connects_on_start);
  RUN_TEST(test_wifi_lifecycle_reapplies_policy_after_shared_driver_reinitialize);
  RUN_TEST(test_standalone_wifi_service_unmanaged_applies_policy_before_connect);
  RUN_TEST(test_standalone_wifi_service_reconnects_after_sta_stop);
  RUN_TEST(test_standalone_wifi_service_runs_deferred_connect_fallback_once);
  RUN_TEST(test_standalone_wifi_service_managed_lifecycle_dispatches_after_csi_init);
  RUN_TEST(test_standalone_wifi_service_get_info_reports_station_details);
  RUN_TEST(test_standalone_wifi_service_get_info_uses_cached_ip_from_got_ip_event);
  RUN_TEST(test_standalone_wifi_service_update_station_config_handles_setup_and_reconnect_paths);
  RUN_TEST(test_standalone_wifi_service_update_station_config_handles_idle_station);
  RUN_TEST(test_standalone_wifi_service_leaves_csi_refresh_to_shared_runtime);
  RUN_TEST(test_standalone_wifi_service_update_station_config_rejects_invalid_bssid);
  RUN_TEST(test_standalone_wifi_service_reports_asynchronous_scan_snapshot);
  RUN_TEST(test_standalone_wifi_service_shutdown_clears_pending_events_and_scan);
  RUN_TEST(test_standalone_wifi_service_setup_failure_unregisters_partial_handlers);
  RUN_TEST(test_standalone_wifi_service_recovers_after_scan_failures);
  RUN_TEST(test_standalone_wifi_service_restarts_driver_when_disconnect_fails);
  RUN_TEST(test_standalone_wifi_service_apply_started_policy_and_reconnect_logic);
  return UNITY_END();
}

#if defined(ESP_PLATFORM)
extern "C" void app_main(void) { process(); }
#else
int main(int argc, char **argv) { return process(); }
#endif
