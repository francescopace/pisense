/*
 * ESPectre - Matter Firmware Entrypoint
 *
 * Matter firmware application entrypoint.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include <esp_err.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include <cstdarg>
#include <cstdio>
#include <string>
#include <utility>

#include <app/server/Server.h>
#include <esp_matter.h>
#include <esp_matter_attribute.h>
#include <esp_matter_core.h>
#include <esp_matter_endpoint.h>
#include <esp_matter_providers.h>
#include <platform/CHIPDeviceLayer.h>
#include <setup_payload/OnboardingCodesUtil.h>

#include "espectre_banner.h"
#include "espectre_log.h"
#include "device_identity.h"
#include "direct_http_protocol.h"
#include "direct_http_service_esp_idf.h"
#include "direct_wifi_snapshot_esp_idf.h"
#include "espectre_protocol.h"
#include "frontend/frontend_firmware_version.h"
#include "improv_serial_service.h"
#include "matter_bindings_esp_matter.h"
#include "matter_commissioning_data.h"
#include "matter_frontend.h"
#include "mdns_discovery_service.h"
#include "mdns_bootstrap_responder.h"
#include "nvs_helpers.h"
#include "pending_event.h"
#include "primary_console.h"
#include "runtime_config_utils.h"
#include "runtime_direct_http_bridge.h"
#include "runtime_sensing_kconfig.h"
#include "wifi_bssid_pin_service.h"

static const char *TAG = "espectre.matter.app";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

namespace {

espectre::MatterEspBindings g_bindings;
espectre::MatterCommissioningDataProvider g_commissioning_data;
espectre::MatterFrontend *g_frontend = nullptr;
espectre::ImprovSerialService *g_improv_serial = nullptr;
espectre::MdnsDiscoveryService *g_mdns_discovery = nullptr;
espectre::MdnsBootstrapResponder *g_mdns_bootstrap_responder = nullptr;
espectre::WifiBssidPinService *g_wifi_bssid_pin_service = nullptr;
espectre::MdnsDiscoveryServiceConfig g_mdns_config;

esp_log_level_t idf_log_level(espectre::LogLevel level) {
  switch (level) {
    case espectre::LogLevel::ERROR:
      return ESP_LOG_ERROR;
    case espectre::LogLevel::WARNING:
      return ESP_LOG_WARN;
    case espectre::LogLevel::INFO:
      return ESP_LOG_INFO;
    case espectre::LogLevel::DEBUG:
      return ESP_LOG_DEBUG;
    case espectre::LogLevel::VERBOSE:
      return ESP_LOG_VERBOSE;
  }
  return ESP_LOG_NONE;
}

bool idf_log_enabled(void *, espectre::LogLevel level, const char *tag) {
  return tag != nullptr && idf_log_level(level) <= esp_log_level_get(tag);
}

void idf_log_write(void *, espectre::LogLevel level, const char *tag, int,
                   const char *format, va_list args) {
  esp_log_va(ESP_LOG_CONFIG_INIT(idf_log_level(level) | ESP_LOG_CONFIGS_DEFAULT),
             tag, format, args);
}
uint16_t g_motion_endpoint_id = 0;
uint64_t g_device_id = 0U;
espectre::PendingEvent<bool> g_commissioned_event;
espectre::PendingEvent<> g_dnssd_initialized_event;
espectre::PendingEvent<uint32_t> g_station_ipv4_event;
espectre::PendingEvent<> g_wifi_station_state_event;
espectre::PendingEvent<> g_node_label_event;
bool g_commissioned = false;
bool g_dnssd_initialized = false;
bool g_bootstrap_configured = false;
uint32_t g_station_ipv4 = 0U;
bool g_operational_start_pending = false;
TickType_t g_operational_start_retry_at = 0U;
TickType_t g_operational_start_not_before = 0U;
std::string g_matter_qr;
std::string g_matter_manual_code;

constexpr TickType_t kOperationalStartRetryDelay = pdMS_TO_TICKS(1000);
constexpr TickType_t kPostCommissioningRuntimeGrace = pdMS_TO_TICKS(10000);

espectre::WifiBssidPinStationState matter_wifi_station_state() {
  const espectre::DirectWifiSnapshot snapshot = espectre::read_direct_wifi_snapshot();
  return espectre::WifiBssidPinStationState{
      snapshot.configured,
      snapshot.connected,
      espectre::read_direct_wifi_connected(),
      snapshot.ssid,
      snapshot.bssid,
  };
}

bool apply_matter_wifi_bssid_pin(const std::string &bssid,
                                 std::string *message,
                                 bool *station_transition_started) {
  const esp_err_t ram_err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (ram_err != ESP_OK) {
    if (message != nullptr) *message = esp_err_to_name(ram_err);
    return false;
  }

  const bool applied =
      espectre::apply_wifi_bssid_pin(bssid, message, station_transition_started);
  const esp_err_t flash_err = esp_wifi_set_storage(WIFI_STORAGE_FLASH);
  if (flash_err != ESP_OK) {
    const std::string storage_error =
        std::string("Wi-Fi storage policy could not be restored: ") + esp_err_to_name(flash_err);
    ESP_LOGE(TAG, "%s", storage_error.c_str());
    if (applied) {
      // The station is already reconnecting with the candidate. No rollback
      // was issued, so keep verifying that transition instead of reporting a
      // failed transition to the pin service.
      if (message != nullptr) *message = storage_error;
      return true;
    }
  }
  return applied;
}

bool handle_wifi_bssid_pin_update(const std::string &bssid,
                                  bool force,
                                  std::string *message) {
  if (g_wifi_bssid_pin_service == nullptr) {
    if (message != nullptr) *message = "Wi-Fi BSSID persistence is unavailable";
    return false;
  }
  return g_wifi_bssid_pin_service->request_update(bssid, message, force);
}

bool wifi_bssid_pin_update_available(std::string *message) {
  const bool available = g_wifi_bssid_pin_service != nullptr &&
                         !g_wifi_bssid_pin_service->apply_pending();
  if (!available && message != nullptr) {
    *message = "Wi-Fi BSSID update already in progress";
  }
  return available;
}

espectre::RuntimeConfig build_runtime_config() {
  espectre::RuntimeConfig config = espectre::make_runtime_sensing_config_from_kconfig();
  config.device_id = espectre::derive_runtime_device_id();
  config.runtime_detector_selection_enabled = true;
  return config;
}

std::string matter_generated_name(uint64_t device_id) {
  return espectre::espectre_device_name(device_id, CONFIG_IDF_TARGET);
}

std::string matter_display_name(uint64_t device_id, const std::string &device_label) {
  return device_label.empty() ? matter_generated_name(device_id) : device_label;
}

std::string matter_instance_name(uint64_t device_id, const std::string &device_label) {
  const std::string formatted_id = espectre::format_espectre_device_id(device_id);
  return device_label.empty() ? "ESPectre " + formatted_id : device_label + " " + formatted_id;
}

std::string matter_mdns_hostname() {
  uint8_t mac[6]{};
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) return {};
  char hostname[13]{};
  std::snprintf(hostname,
                sizeof(hostname),
                "%02X%02X%02X%02X%02X%02X",
                mac[0],
                mac[1],
                mac[2],
                mac[3],
                mac[4],
                mac[5]);
  return hostname;
}

espectre::MdnsTxtRecords matter_mdns_txt(uint64_t device_id, const std::string &device_label) {
  return {
      {"device_id", espectre::format_espectre_device_id(device_id)},
      {"name", matter_display_name(device_id, device_label)},
      {"frontend", "matter"},
      {"txtvers", espectre::ESPECTRE_DNS_SD_TXT_SCHEMA_VERSION},
      {"protovers", espectre::ESPECTRE_PROTOCOL_VERSION},
      {"transport", espectre::ESPECTRE_DIRECT_HTTP_TRANSPORT},
      {"path", espectre::ESPECTRE_DIRECT_HTTP_BASE_ENDPOINT},
      {"firmware", espectre::frontend_firmware_version()},
      {"chip", CONFIG_IDF_TARGET},
      {"capabilities", "config,monitor,csi"},
  };
}

bool has_commissioned_fabric_on_chip_thread() {
  return chip::Server::GetInstance().GetFabricTable().FabricCount() != 0;
}

bool configure_logging() {
  if (!espectre::set_log_sink({nullptr, &idf_log_enabled, &idf_log_write})) {
    return false;
  }
  // CHIP logs are reduced at build time; mute esp-matter attribute chatter at runtime.
  esp_log_level_set("esp_matter_attribute", ESP_LOG_WARN);
  return true;
}

void restart_for_commissioning(chip::System::Layer *, void *) {
  if (has_commissioned_fabric_on_chip_thread()) return;
  // BLE memory was released after commissioning; only a reboot restores it.
  ESP_LOGI(TAG, "Restarting to restore BLE commissioning");
  esp_restart();
}

bool refresh_matter_onboarding_codes() {
  constexpr auto rendezvous =
      chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE);
  char qr_code[128] = {};
  char manual_code[32] = {};
  chip::MutableCharSpan qr_span(qr_code, sizeof(qr_code));
  chip::MutableCharSpan manual_span(manual_code, sizeof(manual_code));

  CHIP_ERROR qr_error = GetQRCode(qr_span, rendezvous);
  CHIP_ERROR manual_error = GetManualPairingCode(manual_span, rendezvous);
  if (qr_error != CHIP_NO_ERROR || manual_error != CHIP_NO_ERROR) {
    return false;
  }

  g_matter_qr.assign(qr_span.data(), qr_span.size());
  g_matter_manual_code.assign(manual_span.data(), manual_span.size());
  return true;
}

bool matter_onboarding_codes(std::string *qr, std::string *manual_code) {
  if (qr == nullptr || manual_code == nullptr || g_matter_qr.empty() ||
      g_matter_manual_code.empty()) {
    return false;
  }
  *qr = g_matter_qr;
  *manual_code = g_matter_manual_code;
  return true;
}

[[gnu::noinline]] bool setup_matter_improv_serial(
    const espectre::RuntimeConfig &runtime_config) {
  espectre::ImprovSerialServiceConfig improv_config;
  improv_config.firmware_name = "ESPectre Matter";
  improv_config.firmware_version = espectre::frontend_firmware_version();
  improv_config.hardware_variant = CONFIG_IDF_TARGET;
  improv_config.device_name = matter_display_name(
      runtime_config.device_id, CONFIG_ESPECTRE_MATTER_NODE_LABEL);
  improv_config.matter_onboarding = matter_onboarding_codes;
  static espectre::ImprovSerialService improv_serial;
  if (!improv_serial.setup(std::move(improv_config))) {
    return false;
  }
  g_improv_serial = &improv_serial;
  return true;
}

void log_onboarding_codes() {
  if (g_matter_qr.empty() || g_matter_manual_code.empty()) {
    ESP_LOGE(TAG, "Matter onboarding codes are unavailable");
    return;
  }
  ESP_LOGI(TAG, "MATTER_QR=%s", g_matter_qr.c_str());
  ESP_LOGI(TAG, "MATTER_MANUAL_CODE=%s", g_matter_manual_code.c_str());
  ESP_LOGI(TAG, "MATTER_DISCRIMINATOR=%u",
           static_cast<unsigned>(g_commissioning_data.setup_discriminator()));
}

void sync_post_start_state_on_chip_thread(intptr_t arg) {
  (void) arg;

  const bool commissioned = has_commissioned_fabric_on_chip_thread();
  g_bindings.refresh_node_label_on_chip_thread();
  log_onboarding_codes();
  g_commissioned_event.post(commissioned);

  ESP_LOGI(TAG, "ESPectre Matter CSI services: %s", commissioned ? "armed" : "waiting for commissioning");
}

void app_event_cb(const ChipDeviceEvent *event, intptr_t arg) {
  switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kDnssdInitialized:
      g_dnssd_initialized_event.post();
      break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
      ESP_LOGI(TAG, "Commissioning complete");
      g_operational_start_not_before = xTaskGetTickCount() + kPostCommissioningRuntimeGrace;
      g_commissioned_event.post(true);
      g_wifi_station_state_event.post();
      break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
      ESP_LOGW(TAG, "Commissioning failed, fail safe timer expired");
      g_wifi_station_state_event.post();
      break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
      ESP_LOGI(TAG, "Fabric removed");
      if (!has_commissioned_fabric_on_chip_thread()) {
        g_commissioned_event.post(false);
        // Allow the RemoveFabric response to leave before restarting.
        const CHIP_ERROR err = chip::DeviceLayer::SystemLayer().StartTimer(
            chip::System::Clock::Seconds32(2), restart_for_commissioning, nullptr);
        if (err != CHIP_NO_ERROR) {
          ESP_LOGE(TAG, "Failed to schedule commissioning restart: %s", err.AsString());
        }
      }
      g_wifi_station_state_event.post();
      break;
    case chip::DeviceLayer::DeviceEventType::kWiFiConnectivityChange:
      g_wifi_station_state_event.post();
      break;
    default:
      break;
  }
}

esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                uint8_t effect_variant, void *priv_data) {
  ESP_LOGI(TAG, "Identify endpoint %u", endpoint_id);
  return ESP_OK;
}

esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                  uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data) {
  (void) priv_data;
  if (type == attribute::POST_UPDATE && endpoint_id == 0 &&
      cluster_id == BasicInformation::Id &&
      attribute_id == BasicInformation::Attributes::NodeLabel::Id && val != nullptr &&
      val->type == ESP_MATTER_VAL_TYPE_CHAR_STRING) {
    g_bindings.cache_node_label(val->val.a.s == 0U
                                   ? std::string{}
                                   : std::string(reinterpret_cast<const char *>(val->val.a.b), val->val.a.s));
    g_node_label_event.post();
  }
  return ESP_OK;
}

void ip_event_cb(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  (void) arg;
  (void) event_base;
  if (event_id == IP_EVENT_STA_GOT_IP && event_data != nullptr) {
    const auto *event = static_cast<const ip_event_got_ip_t *>(event_data);
    g_station_ipv4_event.post(event->ip_info.ip.addr);
    g_wifi_station_state_event.post();
  } else if (event_id == IP_EVENT_STA_LOST_IP) {
    g_station_ipv4_event.post(0U);
    g_wifi_station_state_event.post();
  }
}

bool start_operational_services() {
  const TickType_t now = xTaskGetTickCount();
  if (static_cast<int32_t>(now - g_operational_start_not_before) < 0) {
    return false;
  }
  if (g_frontend == nullptr || !g_frontend->set_runtime_services_armed(true)) {
    return false;
  }
  if (!g_bootstrap_configured && g_mdns_bootstrap_responder != nullptr) {
    if (!g_mdns_bootstrap_responder->setup()) {
      ESP_LOGE(TAG, "Failed to initialize the mDNS bootstrap responder");
      (void) g_frontend->set_runtime_services_armed(false);
      return false;
    }
    g_bootstrap_configured = true;
    (void) g_mdns_bootstrap_responder->update(g_station_ipv4);
  }
  if (g_dnssd_initialized && g_mdns_discovery != nullptr && !g_mdns_discovery->initialized()) {
    if (g_mdns_discovery->setup(g_mdns_config)) {
      if (g_station_ipv4 != 0U) {
        g_mdns_discovery->on_wifi_connected();
      }
      ESP_LOGI(TAG, "ESPectre Direct discovery registered with Matter mDNS");
    } else {
      ESP_LOGE(TAG, "Failed to register ESPectre Direct discovery with Matter mDNS");
      return false;
    }
  }
  return true;
}

void start_operational_services_or_schedule_retry() {
  if (start_operational_services()) {
    g_operational_start_pending = false;
    return;
  }
  g_operational_start_pending = true;
  g_operational_start_retry_at = xTaskGetTickCount() + kOperationalStartRetryDelay;
}

void retry_operational_services_if_due() {
  if (!g_operational_start_pending) return;
  const TickType_t now = xTaskGetTickCount();
  if (static_cast<int32_t>(now - g_operational_start_retry_at) < 0) return;
  start_operational_services_or_schedule_retry();
}

void stop_operational_services() {
  g_operational_start_pending = false;
  if (g_frontend != nullptr) {
    (void) g_frontend->set_runtime_services_armed(false);
  }
  if (g_mdns_discovery != nullptr) {
    g_mdns_discovery->shutdown();
  }
  if (g_mdns_bootstrap_responder != nullptr && g_bootstrap_configured) {
    g_mdns_bootstrap_responder->shutdown();
    g_bootstrap_configured = false;
  }
}

void process_pending_platform_events() {
  bool operational_state_changed = false;
  bool commissioned = false;
  if (g_commissioned_event.take(commissioned)) {
    g_commissioned = commissioned;
    operational_state_changed = true;
  }
  if (g_dnssd_initialized_event.take()) {
    g_dnssd_initialized = true;
    operational_state_changed = true;
  }
  uint32_t ipv4 = 0U;
  if (g_station_ipv4_event.take(ipv4)) {
    g_station_ipv4 = ipv4;
    if (g_bootstrap_configured && g_mdns_bootstrap_responder != nullptr) {
      (void) g_mdns_bootstrap_responder->update(g_station_ipv4);
    }
    if (g_mdns_discovery != nullptr && g_mdns_discovery->initialized()) {
      if (g_station_ipv4 != 0U) {
        g_mdns_discovery->on_wifi_connected();
      } else {
        g_mdns_discovery->on_wifi_disconnected();
      }
    }
  }
  if (g_node_label_event.take()) {
    std::string label;
    if (g_bindings.get_node_label(&label)) {
      for (auto &record : g_mdns_config.txt_records) {
        if (record.first == "name") record.second = matter_display_name(g_device_id, label);
      }
      if (g_frontend != nullptr) g_frontend->sync_device_label();
      if (g_mdns_discovery != nullptr && g_mdns_discovery->initialized()) {
        (void) g_mdns_discovery->update_txt(g_mdns_config.txt_records);
      }
    }
  }
  if (g_wifi_station_state_event.take() && g_wifi_bssid_pin_service != nullptr) {
    g_wifi_bssid_pin_service->notify_station_changed();
  }
  if (operational_state_changed) {
    if (g_commissioned) {
      start_operational_services_or_schedule_retry();
    } else {
      stop_operational_services();
    }
  }
  retry_operational_services_if_due();
}

void espectre_loop_task(void *arg) {
  (void) arg;
  while (true) {
    if (g_wifi_bssid_pin_service != nullptr) {
      g_wifi_bssid_pin_service->loop();
    }
    if (g_improv_serial != nullptr) {
      g_improv_serial->loop();
    }
    if (g_frontend != nullptr) {
      g_frontend->loop();
    }
    process_pending_platform_events();
    if (g_mdns_bootstrap_responder != nullptr && g_bootstrap_configured) {
      g_mdns_bootstrap_responder->loop();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

}  // namespace

extern "C" void app_main() {
  ESP_ERROR_CHECK(espectre::initialize_primary_console());
  if (!configure_logging()) {
    ESP_LOGE(TAG, "Failed to register the Matter log sink");
    return;
  }
  ESP_ERROR_CHECK(espectre::nvs_init_with_erase_fallback());
  espectre::log_espectre_banner([](const char *line) { ESP_LOGI(TAG, "%s", line); });

  CHIP_ERROR commissioning_error = g_commissioning_data.initialize();
  if (commissioning_error != CHIP_NO_ERROR) {
    ESP_LOGE(TAG, "Failed to initialize per-device Matter commissioning data");
    return;
  }
  esp_matter::set_custom_commissionable_data_provider(&g_commissioning_data);
  ESP_LOGI(TAG, "Matter factory data: %s",
           g_commissioning_data.generated_on_boot() ? "generated" : "loaded");

  node::config_t node_config;
  std::snprintf(node_config.root_node.basic_information.node_label,
                sizeof(node_config.root_node.basic_information.node_label),
                "%s",
                CONFIG_ESPECTRE_MATTER_NODE_LABEL);
  node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
  if (node == nullptr) {
    ESP_LOGE(TAG, "Failed to create Matter node");
    return;
  }

  occupancy_sensor::config_t occupancy_config;
  occupancy_config.occupancy_sensing.feature_flags =
      chip::to_underlying(OccupancySensing::Feature::kOther);

  endpoint_t *motion_endpoint = occupancy_sensor::create(node, &occupancy_config, ENDPOINT_FLAG_NONE, nullptr);
  if (motion_endpoint == nullptr) {
    ESP_LOGE(TAG, "Failed to create occupancy endpoint");
    return;
  }

  g_motion_endpoint_id = endpoint::get_id(motion_endpoint);

  static espectre::EspIdfDirectHttpService direct_service;
  static espectre::MdnsDiscoveryService mdns_discovery;
  static espectre::MdnsBootstrapResponder mdns_bootstrap_responder;
  static espectre::MatterFrontend frontend(&g_bindings, g_motion_endpoint_id, &direct_service);
  const espectre::RuntimeConfig runtime_config = build_runtime_config();
  g_device_id = runtime_config.device_id;
  if (!setup_matter_improv_serial(runtime_config)) {
    ESP_LOGE(TAG, "Failed to initialize Matter Improv Serial identity service");
    return;
  }
  frontend.set_runtime_config(runtime_config);
  frontend.set_runtime_services_armed(false);
  g_frontend = &frontend;
  static espectre::WifiBssidPinService wifi_bssid_pin_service;
  espectre::WifiBssidPinServiceConfig wifi_bssid_pin_config;
  wifi_bssid_pin_config.apply_callback = apply_matter_wifi_bssid_pin;
  wifi_bssid_pin_config.station_state_getter = matter_wifi_station_state;
  wifi_bssid_pin_config.prepare_callback = []() {
    if (g_frontend != nullptr) g_frontend->prepare_for_wifi_reconfigure();
  };
  wifi_bssid_pin_config.resume_callback = []() {
    if (g_frontend != nullptr) g_frontend->resume_after_wifi_reconfigure();
  };
  ESP_ERROR_CHECK(wifi_bssid_pin_service.setup(std::move(wifi_bssid_pin_config)));
  g_wifi_bssid_pin_service = &wifi_bssid_pin_service;
  frontend.set_wifi_bssid_pin_setter(handle_wifi_bssid_pin_update);
  frontend.set_wifi_bssid_pin_preflight(wifi_bssid_pin_update_available);
  g_mdns_discovery = &mdns_discovery;
  g_mdns_bootstrap_responder = &mdns_bootstrap_responder;
  const std::string initial_device_label = CONFIG_ESPECTRE_MATTER_NODE_LABEL;
  g_mdns_config = espectre::MdnsDiscoveryServiceConfig{
      matter_mdns_hostname(),
      matter_instance_name(runtime_config.device_id, initial_device_label),
      "_espectre",
      "_tcp",
      espectre::ESPECTRE_DIRECT_HTTP_PORT,
      matter_mdns_txt(runtime_config.device_id, initial_device_label),
      espectre::MdnsResponderMode::USE_EXISTING_RESPONDER,
  };
  esp_err_t err = esp_event_loop_create_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "Failed to create default event loop (%d)", err);
    return;
  }
  esp_event_handler_instance_t got_ip_instance = nullptr;
  esp_event_handler_instance_t lost_ip_instance = nullptr;
  err = esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_cb, nullptr, &got_ip_instance);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register the station IPv4 handler (%d)", err);
    return;
  }
  err = esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_LOST_IP, &ip_event_cb, nullptr, &lost_ip_instance);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register the station IPv4 loss handler (%d)", err);
    (void) esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_instance);
    return;
  }
  err = esp_matter::start(app_event_cb);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start Matter (%d)", err);
    return;
  }
  if (!refresh_matter_onboarding_codes()) {
    ESP_LOGE(TAG, "Failed to generate Matter onboarding codes");
    return;
  }
  g_wifi_station_state_event.post();
  const CHIP_ERROR sync_error = chip::DeviceLayer::PlatformMgr().ScheduleWork(
      sync_post_start_state_on_chip_thread, 0);
  if (sync_error != CHIP_NO_ERROR) {
    ESP_LOGE(TAG, "Failed to schedule Matter post-start synchronization: %s", sync_error.AsString());
    return;
  }

  ESP_LOGI(TAG, "ESPectre Matter detector: %s", espectre::detection_algorithm_name(frontend.runtime_config().detection_algorithm));
  ESP_LOGI(TAG, "ESPectre runtime initialization deferred until Matter commissioning completes");

  xTaskCreate(espectre_loop_task, "espectre_loop", 8192, nullptr, 5, nullptr);

  ESP_LOGI(TAG, "ESPectre Matter firmware started on endpoint %u", g_motion_endpoint_id);
}
