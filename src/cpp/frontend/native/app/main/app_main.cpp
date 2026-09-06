/*
 * ESPectre - Native Firmware Entrypoint
 *
 * Native firmware application entrypoint.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include <cstdarg>
#include <string>
#include <utility>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <driver/gpio.h>

#include "native_frontend.h"
#include "mdns_bootstrap_responder.h"
#include "recovery_button_service.h"
#include "device_config_store.h"
#include "direct_http_protocol.h"
#include "direct_http_service_esp_idf.h"
#include "nvs_helpers.h"
#include "device_identity.h"
#include "espectre_banner.h"
#include "espectre_log.h"
#include "espectre_protocol.h"
#include "frontend/frontend_firmware_version.h"
#include "frontend_bootstrap_helpers.h"
#include "improv_serial_service.h"
#include "mdns_discovery_service.h"
#include "frontend/ota_service_https.h"
#include "peer_discovery_service_esp_idf.h"
#include "primary_console.h"
#include "mqtt_transport_esp_idf.h"
#include "runtime_sensing_kconfig.h"
#include "standalone_wifi_service.h"
#include "task_scheduling_config.h"
#include "wifi_provisioning_service.h"

static const char *TAG = "espectre.native.app";

namespace {

#ifdef ESPECTRE_OTA_DEVELOP_BUILD
constexpr espectre::OtaReleaseChannel kOtaReleaseChannel = espectre::OtaReleaseChannel::DEVELOP;
#elif defined(ESPECTRE_OTA_PREVIEW_BUILD)
constexpr espectre::OtaReleaseChannel kOtaReleaseChannel = espectre::OtaReleaseChannel::PREVIEW;
#else
constexpr espectre::OtaReleaseChannel kOtaReleaseChannel = espectre::OtaReleaseChannel::RELEASE;
#endif

constexpr int kWifiConnectMaxRetry = 8;

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

bool register_espectre_log_sink() {
  return espectre::set_log_sink({nullptr, &idf_log_enabled, &idf_log_write});
}

espectre::NativeFrontend *g_frontend = nullptr;
espectre::RecoveryButtonService *g_recovery_button = nullptr;
espectre::ImprovSerialService *g_improv_serial = nullptr;
espectre::MdnsDiscoveryService *g_mdns_discovery = nullptr;
espectre::MdnsBootstrapResponder *g_mdns_bootstrap_responder = nullptr;
espectre::StandaloneWifiService g_wifi_manager;
espectre::WifiProvisioningService g_wifi_provisioning(&g_wifi_manager);

const char *native_capabilities() {
  return "config,monitor,ota,peer_discovery,csi";
}

std::string native_generated_name(const espectre::EspectreDeviceConfig &config) {
  return espectre::espectre_device_name(config.device_id, CONFIG_IDF_TARGET);
}

std::string native_display_name(const espectre::EspectreDeviceConfig &config) {
  return config.device_label.empty() ? native_generated_name(config) : config.device_label;
}

std::string native_instance_name(const espectre::EspectreDeviceConfig &config) {
  const std::string device_id = espectre::format_espectre_device_id(config.device_id);
  return config.device_label.empty() ? "ESPectre " + device_id : config.device_label + " " + device_id;
}

espectre::MdnsTxtRecords native_mdns_txt(const espectre::EspectreDeviceConfig &config) {
  return {
      {"device_id", espectre::format_espectre_device_id(config.device_id)},
      {"name", native_display_name(config)},
      {"frontend", "native"},
      {"txtvers", espectre::ESPECTRE_DNS_SD_TXT_SCHEMA_VERSION},
      {"protovers", espectre::ESPECTRE_PROTOCOL_VERSION},
      {"transport", espectre::ESPECTRE_DIRECT_HTTP_TRANSPORT},
      {"path", espectre::ESPECTRE_DIRECT_HTTP_BASE_ENDPOINT},
      {"firmware", espectre::frontend_firmware_version()},
      {"chip", CONFIG_IDF_TARGET},
      {"capabilities", native_capabilities()},
  };
}

espectre::PeerDiscoveryCandidate native_peer_candidate(
    const espectre::EspectreDeviceConfig &config,
    const std::string &instance_name,
    const std::string &display_name) {
  const std::string device_id = espectre::format_espectre_device_id(config.device_id);
  espectre::PeerDiscoveryCandidate candidate;
  candidate.instance = instance_name;
  candidate.hostname = "espectre-" + device_id;
  candidate.device_id = device_id;
  candidate.name = display_name;
  candidate.frontend = "native";
  candidate.txt_version = espectre::ESPECTRE_DNS_SD_TXT_SCHEMA_VERSION;
  candidate.protocol_version = espectre::ESPECTRE_PROTOCOL_VERSION;
  candidate.transport = espectre::ESPECTRE_DIRECT_HTTP_TRANSPORT;
  candidate.path = espectre::ESPECTRE_DIRECT_HTTP_BASE_ENDPOINT;
  candidate.firmware = espectre::frontend_firmware_version();
  candidate.chip = CONFIG_IDF_TARGET;
  candidate.capabilities = native_capabilities();
  candidate.port = espectre::ESPECTRE_DIRECT_HTTP_PORT;
  return candidate;
}

std::string improv_device_url() {
  espectre::StandaloneWifiInfo wifi_info;
  if (!g_wifi_manager.get_info(&wifi_info) || !wifi_info.connected || wifi_info.ip_address[0] == '\0') {
    return {};
  }
  return std::string("https://espectre.dev/tools/device-settings/?target=") + wifi_info.ip_address;
}

bool improv_network_connected() {
  espectre::StandaloneWifiInfo wifi_info;
  return g_wifi_manager.get_info(&wifi_info) && wifi_info.connected &&
         wifi_info.ip_address[0] != '\0';
}

[[gnu::noinline]] bool setup_native_improv_serial(const std::string &display_name) {
  espectre::ImprovSerialServiceConfig improv_config;
  improv_config.firmware_name = "ESPectre Native";
  improv_config.firmware_version = espectre::frontend_firmware_version();
  improv_config.hardware_variant = CONFIG_IDF_TARGET;
  improv_config.device_name = display_name;
  improv_config.device_url = improv_device_url;
  improv_config.begin_provisioning = [](const std::string &ssid,
                                        const std::string &password,
                                        std::string *message) {
    return g_wifi_provisioning.begin_serial_provisioning(ssid, password, message);
  };
  improv_config.provisioning_state = []() {
    switch (g_wifi_provisioning.apply_state()) {
      case espectre::WifiProvisioningApplyState::APPLIED:
        return espectre::ImprovSerialProvisioningState::APPLIED;
      case espectre::WifiProvisioningApplyState::ROLLED_BACK:
      case espectre::WifiProvisioningApplyState::RECOVERY_REQUIRED:
        return espectre::ImprovSerialProvisioningState::FAILED;
      case espectre::WifiProvisioningApplyState::VERIFYING:
      case espectre::WifiProvisioningApplyState::ROLLING_BACK:
        return espectre::ImprovSerialProvisioningState::PENDING;
      case espectre::WifiProvisioningApplyState::IDLE:
      default:
        return espectre::ImprovSerialProvisioningState::IDLE;
    }
  };
  improv_config.network_connected = improv_network_connected;
  static espectre::ImprovSerialService improv_serial;
  if (!improv_serial.setup(std::move(improv_config))) {
    return false;
  }
  g_improv_serial = &improv_serial;
  return true;
}

void sync_frontend_wifi_info() {
  if (g_frontend == nullptr) {
    return;
  }
  espectre::NativeFrontend::WifiProvisioningInfo info;
  const espectre::StoredWifiConfig &wifi_config = g_wifi_provisioning.config();
  info.ssid = wifi_config.ssid;
  info.bssid = wifi_config.bssid;
  info.channel = wifi_config.channel;
  info.has_saved_config = wifi_config.has_saved_config;
  info.band_policy = wifi_config.band_policy;
  info.apply_state = espectre::wifi_provisioning_apply_state_name(g_wifi_provisioning.apply_state());
  info.apply_message = g_wifi_provisioning.apply_message();
  info.scan_pending = g_wifi_provisioning.scan_pending();
  info.scan_message = g_wifi_provisioning.scan_message();
  for (const espectre::StandaloneWifiAccessPoint &access_point : g_wifi_provisioning.access_points()) {
    info.access_points.push_back(espectre::NativeFrontend::WifiProvisioningInfo::AccessPoint{
        access_point.bssid,
        access_point.rssi_dbm,
        access_point.channel,
    });
  }
  g_frontend->set_wifi_provisioning_info(info);

  espectre::EspectreDeviceInfo device_info;
  device_info.frontend = "native";
  device_info.firmware_version = espectre::frontend_firmware_version();
  device_info.chip = CONFIG_IDF_TARGET;

  espectre::StandaloneWifiInfo wifi_info;
  if (g_wifi_manager.get_info(&wifi_info)) {
    device_info.network.ip_address = wifi_info.ip_address;
    device_info.network.mac_address = wifi_info.mac_address;
    device_info.network.channel = wifi_info.channel;
  }
  g_frontend->set_device_info(device_info);
  if (g_mdns_discovery != nullptr) {
    if (wifi_info.connected) {
      g_mdns_discovery->on_wifi_connected();
      if (g_mdns_bootstrap_responder != nullptr) {
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        esp_netif_ip_info_t ip_info{};
        if (netif != nullptr && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
          if (!g_mdns_bootstrap_responder->active()) {
            (void) g_mdns_bootstrap_responder->setup();
          }
          (void) g_mdns_bootstrap_responder->update(ip_info.ip.addr);
        }
      }
    } else {
      g_mdns_discovery->on_wifi_disconnected();
      if (g_mdns_bootstrap_responder != nullptr) {
        g_mdns_bootstrap_responder->shutdown();
      }
    }
  }
}

espectre::RuntimeConfig make_runtime_config() {
  espectre::RuntimeConfig config = espectre::make_runtime_sensing_config_from_kconfig();
  config.wifi_band_policy = g_wifi_provisioning.config().band_policy;
  return config;
}

espectre::EspectreDeviceConfig make_device_config() {
  return espectre::load_frontend_device_config(espectre::FrontendDeviceConfigDefaults{
                                                            CONFIG_ESPECTRE_DEVICE_LABEL,
                                                            CONFIG_ESPECTRE_MQTT_SCHEME,
                                                            CONFIG_ESPECTRE_MQTT_HOST,
                                                            CONFIG_ESPECTRE_MQTT_PORT,
                                                            CONFIG_ESPECTRE_MQTT_USERNAME,
                                                            CONFIG_ESPECTRE_MQTT_PASSWORD,
                                                            CONFIG_ESPECTRE_TOPIC_PREFIX,
                                                            espectre::derive_runtime_device_id(),
                                                        },
                                                        TAG,
                                                        "Using stored ESPectre Protocol device config",
                                                        "Failed to load stored device config");
}

void espectre_loop_task(void *arg) {
  (void) arg;
  while (true) {
    g_wifi_manager.loop();
    g_wifi_provisioning.loop();
    if (g_improv_serial != nullptr) {
      g_improv_serial->loop();
    }
    if (g_mdns_bootstrap_responder != nullptr) {
      g_mdns_bootstrap_responder->loop();
    }
    if (g_frontend != nullptr) {
      g_frontend->loop();
    }
#if CONFIG_ESPECTRE_RECOVERY_BUTTON_ENABLED
    if (g_recovery_button != nullptr) {
      const bool pressed = gpio_get_level(static_cast<gpio_num_t>(CONFIG_ESPECTRE_RECOVERY_BUTTON_GPIO)) == 0;
      g_recovery_button->update(pressed, static_cast<uint32_t>(esp_timer_get_time() / 1000));
    }
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

bool init_wifi_station() {
  const esp_err_t setup_err = espectre::setup_frontend_wifi_station(
      &g_wifi_provisioning,
      &g_wifi_manager,
      espectre::FrontendWifiStationOptions{CONFIG_ESPECTRE_WIFI_SSID,
                                                    CONFIG_ESPECTRE_WIFI_PASSWORD,
                                                    CONFIG_ESPECTRE_WIFI_BSSID,
                                                    CONFIG_ESPECTRE_WIFI_CHANNEL,
                                                    kWifiConnectMaxRetry,
                                                    false,
                                                    false,
                                                    sync_frontend_wifi_info,
                                                    sync_frontend_wifi_info,
                                                    sync_frontend_wifi_info,
                                                    espectre::make_runtime_sensing_config_from_kconfig().wifi_band_policy},
      TAG,
      "Using stored Wi-Fi credentials");
  if (setup_err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to initialize Wi-Fi provisioning service: %s", esp_err_to_name(setup_err));
    return false;
  }
  sync_frontend_wifi_info();
  return true;
}

bool handle_wifi_provisioning_command(const std::string &command, std::string *message) {
  return g_wifi_provisioning.handle_command(command, message);
}

bool handle_device_config_change(const espectre::EspectreDeviceConfig &config, bool clear, std::string *message) {
  if (clear) {
    const esp_err_t err = espectre::clear_stored_device_config();
    if (message != nullptr) {
      *message = err == ESP_OK ? "device config cleared" : esp_err_to_name(err);
    }
    return err == ESP_OK;
  }

  const esp_err_t err = espectre::save_stored_device_config(config);
  if (err == ESP_OK && g_mdns_discovery != nullptr) {
    (void) g_mdns_discovery->update_txt(native_mdns_txt(config));
  }
  if (message != nullptr) {
    *message = err == ESP_OK ? "device config saved" : esp_err_to_name(err);
  }
  return err == ESP_OK;
}

void request_wifi_recovery() {
  std::string message;
  if (g_wifi_provisioning.handle_command("CLEAR_WIFI", &message)) {
    ESP_LOGW(TAG, "Physical recovery cleared saved Wi-Fi configuration; use Improv Serial to provision again");
  } else {
    ESP_LOGE(TAG, "Physical Wi-Fi recovery failed: %s", message.c_str());
  }
}

}  // namespace

extern "C" void app_main() {
  ESP_ERROR_CHECK(espectre::initialize_primary_console());
  if (!register_espectre_log_sink()) {
    ESP_LOGE(TAG, "Failed to register the Native log sink");
    return;
  }
  ESP_ERROR_CHECK(espectre::nvs_init_with_erase_fallback());

  espectre::log_espectre_banner([](const char *line) { ESP_LOGI(TAG, "%s", line); });

  if (!init_wifi_station()) {
    return;
  }

  static espectre::EspIdfMqttTransport mqtt_transport;
  static espectre::EspIdfDirectHttpService direct_service;
  static espectre::MdnsDiscoveryService mdns_discovery;
  static espectre::MdnsBootstrapResponder mdns_bootstrap_responder;
  static espectre::EspIdfPeerDiscoveryService peer_discovery;
  static espectre::HttpsOtaService ota_service("native", CONFIG_IDF_TARGET, kOtaReleaseChannel);
  static espectre::NativeFrontend frontend(&mqtt_transport, &ota_service, &direct_service);
  const espectre::EspectreDeviceConfig device_config = make_device_config();
  const std::string device_id = espectre::format_espectre_device_id(device_config.device_id);
  const std::string display_name = native_display_name(device_config);
  const std::string instance_name = native_instance_name(device_config);
  if (!mdns_discovery.setup(espectre::MdnsDiscoveryServiceConfig{
          "espectre-" + device_id,
          instance_name,
          "_espectre",
          "_tcp",
          espectre::ESPECTRE_DIRECT_HTTP_PORT,
          native_mdns_txt(device_config),
      })) {
    ESP_LOGE(TAG, "Failed to initialize Native mDNS discovery");
    return;
  }
  g_mdns_discovery = &mdns_discovery;
  if (!mdns_bootstrap_responder.setup()) {
    ESP_LOGE(TAG, "Failed to initialize the mDNS bootstrap responder");
    return;
  }
  g_mdns_bootstrap_responder = &mdns_bootstrap_responder;
  peer_discovery.set_local_candidate(native_peer_candidate(device_config, instance_name, display_name));
  frontend.set_peer_discovery_service(&peer_discovery);
  frontend.set_runtime_config(make_runtime_config());
  frontend.set_device_config(device_config);
  g_frontend = &frontend;
  sync_frontend_wifi_info();
  frontend.set_provisioning_command_callback(handle_wifi_provisioning_command);
  frontend.set_wifi_scan_callback(
      [](std::string *message) { return g_wifi_provisioning.request_access_point_scan(message); });
  frontend.set_device_config_change_callback(handle_device_config_change);
  if (!frontend.setup()) {
    ESP_LOGE(TAG, "Failed to initialize ESPectre native frontend");
    return;
  }
  g_wifi_provisioning.set_reconfigure_callbacks(
      []() {
        // Cancel pending bootstrap answers before the station is reconfigured.
        if (g_mdns_bootstrap_responder != nullptr) {
          g_mdns_bootstrap_responder->shutdown();
        }
        if (g_frontend != nullptr) {
          g_frontend->prepare_for_wifi_reconfigure();
        }
      },
      []() {
        if (g_frontend != nullptr) {
          g_frontend->resume_after_wifi_reconfigure();
        }
      });
  g_wifi_provisioning.set_scan_callbacks(
      []() {
        if (g_frontend != nullptr) g_frontend->prepare_for_wifi_reconfigure();
      },
      []() {
        if (g_frontend != nullptr) g_frontend->resume_after_wifi_reconfigure();
      });
  if (!setup_native_improv_serial(display_name)) {
    ESP_LOGE(TAG, "Failed to initialize Improv Serial");
    return;
  }

#if CONFIG_ESPECTRE_RECOVERY_BUTTON_ENABLED
  gpio_config_t recovery_button_config{};
  recovery_button_config.pin_bit_mask = 1ULL << CONFIG_ESPECTRE_RECOVERY_BUTTON_GPIO;
  recovery_button_config.mode = GPIO_MODE_INPUT;
  recovery_button_config.pull_up_en = GPIO_PULLUP_ENABLE;
  recovery_button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  recovery_button_config.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&recovery_button_config));
  static espectre::RecoveryButtonService recovery_button(
      CONFIG_ESPECTRE_RECOVERY_BUTTON_HOLD_MS, request_wifi_recovery);
  g_recovery_button = &recovery_button;
  ESP_LOGI(TAG,
           "Hold BOOT on GPIO%d for %d ms to clear Wi-Fi and return to Improv Serial setup",
           CONFIG_ESPECTRE_RECOVERY_BUTTON_GPIO,
           CONFIG_ESPECTRE_RECOVERY_BUTTON_HOLD_MS);
#endif

  ESP_ERROR_CHECK(g_wifi_manager.start());
  xTaskCreate(espectre_loop_task, "espectre_native_loop", 8192, nullptr,
              espectre::task_scheduling::kNativeLoopPriority, nullptr);
  ESP_LOGI(TAG, "ESPectre native firmware started (loop priority=%u)",
           static_cast<unsigned>(espectre::task_scheduling::kNativeLoopPriority));
}
