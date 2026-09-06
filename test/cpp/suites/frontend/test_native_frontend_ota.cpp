/*
 * ESPectre - Native Frontend OTA Tests
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "native_frontend_test_support.h"

void test_native_frontend_mqtt_connect_publishes_current_ota_state(void) {
  MockMqttTransport mqtt;
  MockOtaService ota;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt, &ota);
  EspectreDeviceInfo info;
  info.frontend = "native";
  info.firmware_version = "1.2.3";
  frontend.set_device_config(config);
  frontend.set_device_info(info);
  TEST_ASSERT_TRUE(frontend.setup());

  mqtt_transport_mock::state.publishes.clear();
  mqtt.emit_connection(true);
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic == "espectre/v1/devices/0000abcdeffedcba/ota" &&
                                        publish.payload.find("\"state\":\"idle\"") != std::string::npos &&
                                        publish.payload.find("\"current_version\":\"1.2.3\"") != std::string::npos &&
                                        publish.retain;
                               }));
}

void test_native_frontend_mqtt_ota_commands_use_ota_service_and_publish_state(void) {
  MockMqttTransport mqtt;
  MockOtaService ota;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt, &ota);
  EspectreDeviceInfo info;
  info.frontend = "native";
  info.firmware_version = "1.0.0";
  frontend.set_device_config(config);
  frontend.set_device_info(info);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt_transport_mock::state.publishes.clear();

  mqtt.emit_command("{\"command_id\":\"cmd-ota-check\",\"command\":\"check_ota\"}");
  TEST_ASSERT_EQUAL(1, ota_service_mock::state.start_check_calls);
  TEST_ASSERT_EQUAL_STRING("1.0.0", ota_service_mock::state.last_current_version.c_str());
  TEST_ASSERT_TRUE(ota_service_mock::state.last_channel.empty());
  TEST_ASSERT_EQUAL_STRING("espectre/v1/devices/0000abcdeffedcba/commands/result",
                           mqtt_transport_mock::state.publishes.back().topic.c_str());

  mqtt_transport_mock::state.publishes.clear();
  mqtt.emit_command("{\"command_id\":\"cmd-ota-check-preview\",\"command\":\"check_ota\",\"channel\":\"preview\"}");
  TEST_ASSERT_EQUAL(2, ota_service_mock::state.start_check_calls);
  TEST_ASSERT_EQUAL_STRING("preview", ota_service_mock::state.last_channel.c_str());

  mqtt_transport_mock::state.publishes.clear();
  mqtt.emit_command("{\"command_id\":\"cmd-ota-start\",\"command\":\"start_ota\"}");
  TEST_ASSERT_EQUAL(1, ota_service_mock::state.start_update_calls);
  TEST_ASSERT_EQUAL_STRING("1.0.0", ota_service_mock::state.last_current_version.c_str());

  mqtt_transport_mock::state.publishes.clear();
  EspectreOtaStatus ota_status;
  ota_status.state = EspectreOtaState::UPDATE_AVAILABLE;
  ota_status.current_version = "1.0.0";
  ota_status.target_version = "1.1.0";
  ota_status.image_url = "https://fw.example/native.bin";
  ota.emit_status(ota_status);
  TEST_ASSERT_EQUAL(1, static_cast<int>(mqtt_transport_mock::state.publishes.size()));
  TEST_ASSERT_EQUAL_STRING("espectre/v1/devices/0000abcdeffedcba/ota",
                           mqtt_transport_mock::state.publishes[0].topic.c_str());
  TEST_ASSERT_TRUE(mqtt_transport_mock::state.publishes[0].payload.find("\"state\":\"update_available\"") !=
                   std::string::npos);
}

void test_native_frontend_ota_prepare_quiesces_transports_and_recovers_on_error(void) {
  MockMqttTransport mqtt;
  MockOtaService ota;
  EspectreDeviceConfig config;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;
  NativeFrontend::WifiProvisioningInfo wifi;
  wifi.ssid = "HomeNet";

  NativeFrontend frontend(&mqtt, &ota);
  frontend.set_device_config(config);
  frontend.set_wifi_provisioning_info(wifi);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt_transport_mock::state.publishes.clear();

  ota.emit_prepare();

  TEST_ASSERT_TRUE(frontend.ota_frontend_quiesced_);
  TEST_ASSERT_TRUE(mqtt_transport_mock::state.shutdown_called);
  TEST_ASSERT_FALSE(frontend_runtime_shim::state.services_armed);

  EspectreOtaStatus downloading;
  downloading.state = EspectreOtaState::DOWNLOADING;
  downloading.busy = true;
  ota.emit_status(downloading);
  TEST_ASSERT_TRUE(mqtt_transport_mock::state.publishes.empty());

  const int mqtt_setup_calls = mqtt_transport_mock::state.setup_calls;
  EspectreOtaStatus error;
  error.state = EspectreOtaState::ERROR;
  error.message = "download failed";
  ota.emit_status(error);

  TEST_ASSERT_FALSE(frontend.ota_frontend_quiesced_);
  TEST_ASSERT_EQUAL(mqtt_setup_calls + 1, mqtt_transport_mock::state.setup_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.services_armed);
}

void test_native_frontend_ota_error_restores_the_previous_sensing_state(void) {
  MockOtaService ota;
  MockDirectHttpService direct;
  NativeFrontend frontend(nullptr, &ota, &direct);
  NativeFrontend::WifiProvisioningInfo wifi;
  wifi.ssid = "Lab";
  frontend.set_wifi_provisioning_info(wifi);
  EspectreDeviceInfo info;
  info.network.ip_address = "192.168.1.42";
  frontend.set_device_info(info);
  TEST_ASSERT_TRUE(frontend.setup());

  for (bool enabled : {false, true, false}) {
    const std::string response = direct.emit_request(
        DirectRequest{"sensing", "update_sensing", enabled ? "{\"enabled\":true}" : "{\"enabled\":false}"});
    TEST_ASSERT_TRUE(response.find("\"accepted\":true") != std::string::npos);
    TEST_ASSERT_EQUAL(enabled, frontend_runtime_shim::state.services_armed);
    ota.emit_prepare();
    ota.emit_prepare();
    TEST_ASSERT_FALSE(frontend_runtime_shim::state.services_armed);
    TEST_ASSERT_FALSE(direct.running());

    EspectreOtaStatus error;
    error.state = EspectreOtaState::ERROR;
    ota.emit_status(error);
    TEST_ASSERT_TRUE(direct.running());
    TEST_ASSERT_EQUAL(enabled, frontend_runtime_shim::state.services_armed);
  }
}

void test_native_frontend_direct_ota_returns_status_and_streams_updates(void) {
  MockOtaService ota;
  MockDirectHttpService direct;
  ota_service_mock::state.status.default_channel = "develop";

  NativeFrontend frontend(nullptr, &ota, &direct);
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  frontend.set_device_config(config);
  EspectreDeviceInfo info;
  info.firmware_version = "1.2.3";
  info.network.ip_address = "192.168.1.42";
  frontend.set_device_info(info);
  TEST_ASSERT_TRUE(frontend.setup());
  direct.emit_client_count(1U);

  const std::string status =
      direct.emit_request(DirectRequest{"ota-status", "ota", "{}"});
  TEST_ASSERT_TRUE(status.find("\"command_id\":\"ota-status\"") != std::string::npos);
  TEST_ASSERT_TRUE(status.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_TRUE(status.find("\"current_version\":\"1.2.3\"") != std::string::npos);
  TEST_ASSERT_TRUE(status.find("\"default_channel\":\"develop\"") != std::string::npos);

  const std::string check = direct.emit_request(
      DirectRequest{"ota-check", "check_ota", "{\"channel\":\"preview\"}"});
  TEST_ASSERT_TRUE(check.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_EQUAL(1, ota_service_mock::state.start_check_calls);
  TEST_ASSERT_EQUAL_STRING("1.2.3", ota_service_mock::state.last_current_version.c_str());
  TEST_ASSERT_EQUAL_STRING("preview", ota_service_mock::state.last_channel.c_str());

  EspectreOtaStatus available;
  available.state = EspectreOtaState::UPDATE_AVAILABLE;
  available.current_version = "1.2.3";
  available.target_version = "1.3.0";
  available.update_available = true;
  ota.emit_status(available);
  TEST_ASSERT_EQUAL(1, static_cast<int>(direct_http_service_mock::state.published_events.size()));
  const direct_http_service_mock::PublishedEvent &event =
      direct_http_service_mock::state.published_events[0];
  TEST_ASSERT_EQUAL_STRING("ota", event.event_name.c_str());
  TEST_ASSERT_TRUE(event.data_json.find("\"state\":\"update_available\"") != std::string::npos);
  TEST_ASSERT_TRUE(event.data_json.find("\"target_version\":\"1.3.0\"") != std::string::npos);
  TEST_ASSERT_FALSE(event.replaceable_telemetry);

  const std::string update = direct.emit_request(
      DirectRequest{"ota-start", "start_ota", "{\"channel\":\"preview\"}"});
  TEST_ASSERT_TRUE(update.find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_EQUAL(1, ota_service_mock::state.start_update_calls);
  TEST_ASSERT_EQUAL_STRING("preview", ota_service_mock::state.last_channel.c_str());
}


void test_native_frontend_ota_dispatch_preserves_decoded_channels_on_both_transports(void) {
  MockMqttTransport mqtt;
  MockOtaService ota;
  MockDirectHttpService direct;
  NativeFrontend frontend(&mqtt, &ota, &direct);
  EspectreDeviceConfig config;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;
  frontend.set_device_config(config);
  EspectreDeviceInfo info;
  info.firmware_version = "1.2.3";
  info.network.ip_address = "192.168.1.42";
  frontend.set_device_info(info);
  TEST_ASSERT_TRUE(frontend.setup());

  const struct {
    const char *parameters;
    const char *channel;
  } cases[] = {
      {R"({"channel":"preview"})", ESPECTRE_OTA_CHANNEL_PREVIEW},
      {R"({"chan\u006eel":"preview"})", ESPECTRE_OTA_CHANNEL_PREVIEW},
      {R"({"channel":"\u0070review"})", ESPECTRE_OTA_CHANNEL_PREVIEW},
      {"{}", ""},
  };
  for (const char *name : {"check_ota", "start_ota"}) {
    for (const auto &test : cases) {
      for (bool over_mqtt : {true, false}) {
        const int checks_before = ota_service_mock::state.start_check_calls;
        const int updates_before = ota_service_mock::state.start_update_calls;
        if (over_mqtt) {
          std::string payload = "{\"command_id\":\"channel\",\"command\":\"";
          payload += name;
          payload += '"';
          const std::string parameters = test.parameters;
          if (parameters != "{}") payload += ',' + parameters.substr(1U, parameters.size() - 2U);
          payload += '}';
          mqtt.emit_command(payload);
        } else {
          const std::string result = direct.emit_request(DirectRequest{"channel", name, test.parameters});
          TEST_ASSERT_TRUE(result.find("\"accepted\":true") != std::string::npos);
        }
        TEST_ASSERT_EQUAL(checks_before + (std::string(name) == "check_ota" ? 1 : 0),
                          ota_service_mock::state.start_check_calls);
        TEST_ASSERT_EQUAL(updates_before + (std::string(name) == "start_ota" ? 1 : 0),
                          ota_service_mock::state.start_update_calls);
        TEST_ASSERT_EQUAL_STRING(test.channel, ota_service_mock::state.last_channel.c_str());
        TEST_ASSERT_EQUAL_STRING("1.2.3", ota_service_mock::state.last_current_version.c_str());
      }
    }
  }
}

int main(int argc, char **argv) {
  (void) argc;
  (void) argv;
  UNITY_BEGIN();
  RUN_TEST(test_native_frontend_mqtt_connect_publishes_current_ota_state);
  RUN_TEST(test_native_frontend_mqtt_ota_commands_use_ota_service_and_publish_state);
  RUN_TEST(test_native_frontend_ota_prepare_quiesces_transports_and_recovers_on_error);
  RUN_TEST(test_native_frontend_ota_error_restores_the_previous_sensing_state);
  RUN_TEST(test_native_frontend_direct_ota_returns_status_and_streams_updates);
  RUN_TEST(test_native_frontend_ota_dispatch_preserves_decoded_channels_on_both_transports);
  return UNITY_END();
}
