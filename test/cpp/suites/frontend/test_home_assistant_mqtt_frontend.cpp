/*
 * ESPectre - Home Assistant MQTT Frontend Tests
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "native_frontend_test_support.h"

void test_native_frontend_mqtt_connect_publishes_ha_discovery_and_subscribes_birth_topics(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000111122223333ULL;
  config.device_label = "Kitchen Sensor";
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  RuntimeConfig runtime_config;
  frontend.set_runtime_config(runtime_config);
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());

  mqtt_transport_mock::state.publishes.clear();
  mqtt.emit_connection(true);

  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.subscriptions.begin(),
                               mqtt_transport_mock::state.subscriptions.end(),
                               [](const mqtt_transport_mock::Subscription &subscription) {
                                 return subscription.topic == "homeassistant/status";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.subscriptions.begin(),
                               mqtt_transport_mock::state.subscriptions.end(),
                               [](const mqtt_transport_mock::Subscription &subscription) {
                                 return subscription.topic ==
                                        "espectre/v1/devices/0000111122223333/ha/threshold/set";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.subscriptions.begin(),
                               mqtt_transport_mock::state.subscriptions.end(),
                               [](const mqtt_transport_mock::Subscription &subscription) {
                                 return subscription.topic ==
                                        "espectre/v1/devices/0000111122223333/ha/motion_on_hits/set";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.subscriptions.begin(),
                               mqtt_transport_mock::state.subscriptions.end(),
                               [](const mqtt_transport_mock::Subscription &subscription) {
                                 return subscription.topic ==
                                        "espectre/v1/devices/0000111122223333/ha/motion_off_hits/set";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.subscriptions.begin(),
                               mqtt_transport_mock::state.subscriptions.end(),
                               [](const mqtt_transport_mock::Subscription &subscription) {
                                 return subscription.topic ==
                                        "espectre/v1/devices/0000111122223333/ha/calibrate/set";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.subscriptions.begin(),
                               mqtt_transport_mock::state.subscriptions.end(),
                               [](const mqtt_transport_mock::Subscription &subscription) {
                                 return subscription.topic ==
                                        "espectre/v1/devices/0000111122223333/ha/detector/set";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.subscriptions.begin(),
                               mqtt_transport_mock::state.subscriptions.end(),
                               [](const mqtt_transport_mock::Subscription &subscription) {
                                 return subscription.topic ==
                                        "espectre/v1/devices/0000111122223333/ha/csi_traffic_mode/set";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.subscriptions.begin(),
                               mqtt_transport_mock::state.subscriptions.end(),
                               [](const mqtt_transport_mock::Subscription &subscription) {
                                 return subscription.topic ==
                                        "espectre/v1/devices/0000111122223333/ha/traffic_generator_mode/set";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.subscriptions.begin(),
                               mqtt_transport_mock::state.subscriptions.end(),
                               [](const mqtt_transport_mock::Subscription &subscription) {
                                 return subscription.topic ==
                                        "espectre/v1/devices/0000111122223333/ha/diagnostics/set";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/binary_sensor/native_0000111122223333_motion_detected/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"Motion Detected\"") != std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/sensor/native_0000111122223333_movement_score/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"Movement Score\"") != std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/sensor/native_0000111122223333_intensity/config" &&
                                        publish.retain && publish.payload.empty();
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/binary_sensor/native_0000111122223333_motion/config" &&
                                        publish.retain && publish.payload.empty();
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/switch/native_0000111122223333_calibrate/config" &&
                                        publish.retain && publish.payload.empty();
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/switch/native_0000111122223333_trigger_calibration/config" &&
                                        publish.retain && publish.payload.empty();
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/sensor/native_0000111122223333_traffic_tx_rate/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"Traffic TX Rate\"") != std::string::npos &&
                                        publish.payload.find("\"entity_category\":\"diagnostic\"") != std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/button/native_0000111122223333_refresh_diagnostics/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"Refresh Diagnostics\"") != std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/number/native_0000111122223333_threshold/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"Threshold\"") != std::string::npos &&
                                        publish.payload.find("\"command_topic\"") != std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/number/native_0000111122223333_motion_on_hits/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"Motion On Hits\"") != std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/number/native_0000111122223333_motion_off_hits/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"Motion Off Hits\"") != std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/button/native_0000111122223333_recalibrate/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"Recalibrate\"") != std::string::npos &&
                                        publish.payload.find("\"command_topic\"") != std::string::npos &&
                                        publish.payload.find("\"payload_press\":\"ON\"") != std::string::npos &&
                                        publish.payload.find("\"entity_category\":\"config\"") != std::string::npos &&
                                        publish.payload.find("\"state_topic\"") == std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/binary_sensor/native_0000111122223333_calibration_active/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"Calibration Active\"") != std::string::npos &&
                                        publish.payload.find("\"state_topic\"") != std::string::npos &&
                                        publish.payload.find("\"entity_category\":\"diagnostic\"") !=
                                            std::string::npos &&
                                        publish.payload.find("\"command_topic\"") == std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/select/native_0000111122223333_detection_profile/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"Detection Profile\"") !=
                                            std::string::npos &&
                                        publish.payload.find("\"entity_category\":\"config\"") !=
                                            std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/select/native_0000111122223333_csi_traffic_ownership/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"CSI Traffic Ownership\"") !=
                                            std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/select/native_0000111122223333_csi_traffic_source/config" &&
                                        publish.retain &&
                                        publish.payload.find("\"name\":\"CSI Traffic Source\"") !=
                                            std::string::npos &&
                                        publish.payload.find(
                                            std::string("\"options\":[\"") +
                                            RUNTIME_TRAFFIC_GENERATOR_MODE_PING_NAME + "\",\"" +
                                            RUNTIME_TRAFFIC_GENERATOR_MODE_DNS_NAME + "\",\"" +
                                            RUNTIME_TRAFFIC_GENERATOR_MODE_DNS_TCP_NAME + "\",\"" +
                                            RUNTIME_TRAFFIC_GENERATOR_MODE_WIFI_RAW_NAME + "\"]") !=
                                            std::string::npos;
                               }));
  const int csi_traffic_discovery = mqtt_publish_index(
      "homeassistant/select/native_0000111122223333_csi_traffic_ownership/config");
  const int traffic_generator_discovery = mqtt_publish_index(
      "homeassistant/select/native_0000111122223333_csi_traffic_source/config");
  const int recalibrate_discovery =
      mqtt_publish_index("homeassistant/button/native_0000111122223333_recalibrate/config");
  const int calibration_active_discovery =
      mqtt_publish_index("homeassistant/binary_sensor/native_0000111122223333_calibration_active/config");
  TEST_ASSERT_TRUE(csi_traffic_discovery >= 0);
  TEST_ASSERT_TRUE(csi_traffic_discovery < traffic_generator_discovery);
  TEST_ASSERT_TRUE(traffic_generator_discovery < recalibrate_discovery);
  TEST_ASSERT_TRUE(recalibrate_discovery < calibration_active_discovery);
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/binary_sensor/native_0000111122223333_motion_detected/config" &&
                                        publish.payload.find(
                                            "\"availability_topic\":\"espectre/v1/devices/0000111122223333/health\"") !=
                                            std::string::npos &&
                                        publish.payload.find("\"availability_template\"") != std::string::npos;
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "espectre/v1/devices/0000111122223333/ha/motion/state" &&
                                        publish.payload == "ON";
                               }));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/motion_on_hits/state", "4"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/motion_off_hits/state", "3"));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "espectre/v1/devices/0000111122223333/ha/movement/state" &&
                                        publish.payload == "2.7500";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "espectre/v1/devices/0000111122223333/ha/threshold/state" &&
                                        publish.payload == "1.5000";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "espectre/v1/devices/0000111122223333/ha/calibrate/state" &&
                                        publish.payload == "OFF";
                               }));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/csi_traffic_mode/state", "internal"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/traffic_generator_mode/state", "ping"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/traffic_tx_rate/state"));

  mqtt_transport_mock::state.publishes.clear();
  frontend.shutdown();
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic == "espectre/v1/devices/0000111122223333/health" &&
                                        publish.payload.find("\"online\":false") != std::string::npos && publish.retain;
                               }));
}

void test_native_frontend_ha_birth_message_republishes_discovery_and_state(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000111122223333ULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  RuntimeConfig runtime_config;
  frontend.set_runtime_config(runtime_config);
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt.emit_connection(true);
  mqtt_transport_mock::state.publishes.clear();

  mqtt.emit_message("homeassistant/status", "online");

  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/binary_sensor/native_0000111122223333_motion_detected/config" &&
                                        publish.retain;
                               }));
  TEST_ASSERT_TRUE(has_mqtt_publish(
      "homeassistant/button/native_0000111122223333_recalibrate/config"));
  TEST_ASSERT_TRUE(has_mqtt_publish(
      "homeassistant/binary_sensor/native_0000111122223333_calibration_active/config"));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "espectre/v1/devices/0000111122223333/ha/movement/state";
                               }));
  TEST_ASSERT_TRUE(std::none_of(mqtt_transport_mock::state.publishes.begin(),
                                mqtt_transport_mock::state.publishes.end(),
                                [](const mqtt_transport_mock::Publish &publish) {
                                  return publish.topic.find("/ha/intensity/state") != std::string::npos;
                                }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "homeassistant/sensor/native_0000111122223333_traffic_tx_rate/config" &&
                                        publish.retain;
                               }));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/traffic_tx_rate/state"));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "espectre/v1/devices/0000111122223333/ha/threshold/state";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic ==
                                            "espectre/v1/devices/0000111122223333/ha/calibrate/state";
                               }));
  TEST_ASSERT_TRUE(std::any_of(mqtt_transport_mock::state.publishes.begin(),
                               mqtt_transport_mock::state.publishes.end(),
                               [](const mqtt_transport_mock::Publish &publish) {
                                 return publish.topic == "espectre/v1/devices/0000111122223333/health" &&
                                        publish.payload.find("\"online\":true") != std::string::npos && publish.retain;
                               }));
}

void test_native_frontend_retries_the_complete_ha_snapshot_after_queue_backpressure(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000111122223333ULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  frontend.set_runtime_config(RuntimeConfig{});
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt_transport_mock::state.publishes.clear();
  mqtt_transport_mock::state.diagnostics.queue_capacity = 16U;
  mqtt_transport_mock::state.diagnostics.queued_publishes = 0U;

  mqtt.emit_connection(true);
  TEST_ASSERT_EQUAL(16U, mqtt_transport_mock::state.publishes.size());
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/motion/state"));

  frontend.loop();
  TEST_ASSERT_EQUAL(16U, mqtt_transport_mock::state.publishes.size());

  mqtt_transport_mock::state.diagnostics.queued_publishes = 0U;
  frontend.loop();
  TEST_ASSERT_TRUE(mqtt_transport_mock::state.publishes.size() > 16U);
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/motion/state"));

  mqtt_transport_mock::state.diagnostics.queued_publishes = 0U;
  frontend.loop();
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/motion/state"));

  mqtt_transport_mock::state.diagnostics.queued_publishes = 0U;
  frontend.loop();
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/motion/state", "ON"));
  TEST_ASSERT_TRUE(has_mqtt_publish(
      "homeassistant/sensor/native_0000111122223333_csi_temporal_occupancy/config"));
  TEST_ASSERT_TRUE(has_mqtt_publish(
      "homeassistant/sensor/native_0000111122223333_csi_occupancy/config", ""));
}

void test_native_frontend_defers_initial_ha_state_until_sensing_is_ready(void) {
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000111122223333ULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  frontend.set_runtime_config(RuntimeConfig{});
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  TEST_ASSERT_FALSE(frontend.snapshot().ready_to_publish);
  mqtt.emit_connection(true);
  frontend.loop();
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000111122223333/ha/motion_on_hits/state"));

  frontend_runtime_shim::state.last_listener->on_periodic_update(make_ready_snapshot(), 100U);
  frontend.loop();
  for (const char *suffix : {"motion", "movement", "threshold", "motion_on_hits", "motion_off_hits",
                             "calibrate", "detector", "csi_traffic_mode", "traffic_generator_mode"}) {
    TEST_ASSERT_TRUE(has_mqtt_publish(std::string("espectre/v1/devices/0000111122223333/ha/") + suffix + "/state"));
  }
  mqtt_transport_mock::state.publishes.clear();
  frontend.loop();
  TEST_ASSERT_TRUE(mqtt_transport_mock::state.publishes.empty());
}

void test_native_frontend_ha_entities_follow_esphome_cadences(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt.emit_connection(true);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.live_telemetry_enabled);

  mqtt_transport_mock::state.publishes.clear();
  RuntimeSnapshot snapshot = make_ready_snapshot();
  frontend.on_live_telemetry(snapshot.movement_metric, snapshot.threshold);
  TEST_ASSERT_TRUE(mqtt_transport_mock::state.publishes.empty());
  frontend.loop();
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/movement/state", "2.7500"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/motion"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/motion/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/threshold/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/detector/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/intensity/state"));

  mqtt_transport_mock::state.publishes.clear();
  frontend.on_periodic_update(snapshot, 10);
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/movement/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/motion"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/intensity/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/motion/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/threshold/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/detector/state"));

  mqtt_transport_mock::state.publishes.clear();
  frontend.on_motion_state_changed(snapshot);
  TEST_ASSERT_TRUE(mqtt_transport_mock::state.publishes.empty());
  frontend.loop();
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/motion/state", "ON"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/motion"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/movement/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/intensity/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/threshold/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/state"));

  mqtt_transport_mock::state.publishes.clear();
  snapshot.threshold = 0.45f;
  frontend_runtime_shim::state.last_listener->on_threshold_changed(snapshot);
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/threshold/state", "0.4500"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/intensity/state"));
}

void test_native_frontend_ha_threshold_command_updates_runtime(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt.emit_connection(true);
  mqtt_transport_mock::state.publishes.clear();

  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/threshold/set", "0.45");

  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_threshold_calls);
  TEST_ASSERT_EQUAL_FLOAT(0.45f, frontend_runtime_shim::state.last_threshold);
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/threshold/state", "0.4500"));
  TEST_ASSERT_TRUE(has_mqtt_publish_containing("espectre/v1/devices/0000abcdeffedcba/sensing",
                                               "\"threshold\":0.450000"));
}

void test_native_frontend_ha_motion_hits_commands_update_runtime(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt.emit_connection(true);
  mqtt_transport_mock::state.publishes.clear();

  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/motion_on_hits/set", "6");
  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/motion_off_hits/set", "4");

  TEST_ASSERT_EQUAL(2, frontend_runtime_shim::state.set_motion_hits_calls);
  TEST_ASSERT_EQUAL_UINT8(6U, frontend_runtime_shim::state.last_motion_on_hits);
  TEST_ASSERT_EQUAL_UINT8(4U, frontend_runtime_shim::state.last_motion_off_hits);
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/motion_on_hits/state", "6"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/motion_off_hits/state", "4"));
  TEST_ASSERT_TRUE(has_mqtt_publish_containing("espectre/v1/devices/0000abcdeffedcba/sensing",
                                               "\"motion_on_hits\":6"));
  TEST_ASSERT_TRUE(has_mqtt_publish_containing("espectre/v1/devices/0000abcdeffedcba/sensing",
                                               "\"motion_off_hits\":4"));
}

void test_native_frontend_ha_calibrate_command_triggers_runtime(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt.emit_connection(true);
  mqtt_transport_mock::state.publishes.clear();

  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/set", "ON");

  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.trigger_recalibration_calls);
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/state", "ON"));

  mqtt_transport_mock::state.publishes.clear();
  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/set", "OFF");
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.trigger_recalibration_calls);
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/state", "ON"));

  mqtt_transport_mock::state.publishes.clear();
  RuntimeSnapshot snapshot = make_ready_snapshot();
  snapshot.calibrating = true;
  frontend_runtime_shim::state.last_listener->on_calibration_started(snapshot);
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/state", "ON"));
  TEST_ASSERT_TRUE(has_mqtt_publish_containing("espectre/v1/devices/0000abcdeffedcba/sensing",
                                               "\"calibrating\":true"));

  mqtt_transport_mock::state.publishes.clear();
  snapshot.calibrating = false;
  snapshot.threshold = 0.42f;
  frontend_runtime_shim::state.calibrating = false;
  frontend_runtime_shim::state.last_listener->on_calibration_finished(snapshot, true);
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/state", "OFF"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/threshold/state", "0.4200"));
  TEST_ASSERT_TRUE(has_mqtt_publish_containing("espectre/v1/devices/0000abcdeffedcba/sensing",
                                               "\"calibrating\":false"));
  TEST_ASSERT_TRUE(has_mqtt_publish_containing("espectre/v1/devices/0000abcdeffedcba/sensing",
                                               "\"threshold\":0.420000"));
}

void test_native_frontend_ha_calibrate_command_respects_manual_recalibration_capability(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  frontend_runtime_shim::state.capabilities.supports_manual_recalibration = false;
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt.emit_connection(true);
  mqtt_transport_mock::state.publishes.clear();

  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/set", "ON");

  TEST_ASSERT_EQUAL(0, frontend_runtime_shim::state.trigger_recalibration_calls);
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/calibrate/state", "OFF"));
}

void test_native_frontend_ha_traffic_control_commands_update_runtime(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt.emit_connection(true);
  mqtt_transport_mock::state.publishes.clear();

  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/csi_traffic_mode/set", "external");
  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/traffic_generator_mode/set", "dns_tcp");

  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_csi_traffic_mode_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_csi_traffic_mode == CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_traffic_generator_mode_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_traffic_generator_mode == RuntimeTrafficMode::DNS_TCP);
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_traffic_mode/state", "external"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/traffic_generator_mode/state", "dns_tcp"));
  TEST_ASSERT_TRUE(has_mqtt_publish_containing("espectre/v1/devices/0000abcdeffedcba/sensing",
                                               "\"csi_traffic_mode\":\"external\""));
  TEST_ASSERT_TRUE(has_mqtt_publish_containing("espectre/v1/devices/0000abcdeffedcba/sensing",
                                               "\"traffic_generator_mode\":\"dns_tcp\""));

  mqtt_transport_mock::state.publishes.clear();
  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/csi_traffic_mode/set", "pacing");
  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_csi_traffic_mode_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_csi_traffic_mode == CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_traffic_mode/state", "pacing"));
}

void test_native_frontend_ha_detector_command_updates_canonical_config(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;
  RuntimeConfig runtime_config;
  runtime_config.runtime_detector_selection_enabled = true;

  NativeFrontend frontend(&mqtt);
  frontend.set_device_config(config);
  frontend.set_runtime_config(runtime_config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt.emit_connection(true);
  mqtt_transport_mock::state.publishes.clear();

  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/detector/set", "high_accuracy");

  TEST_ASSERT_EQUAL(1, frontend_runtime_shim::state.set_detector_calls);
  TEST_ASSERT_TRUE(frontend_runtime_shim::state.last_detector == DetectionAlgorithm::HIGH_ACCURACY);
  TEST_ASSERT_TRUE(has_mqtt_publish_containing("espectre/v1/devices/0000abcdeffedcba/sensing",
                                               "\"detector\":\"high_accuracy\""));
}

void test_native_frontend_ha_diagnostics_button_publishes_cached_sample(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt.emit_connection(true);
  RuntimeDiagnosticsSample &sample = frontend_runtime_shim::state.diagnostics_sample;
  sample.traffic_tx_pps = 100.0f;
  sample.csi_callback_pps = 96.0f;
  sample.csi_accepted_pps = 90.0f;
  sample.csi_admitted_pps = 84.0f;
  sample.csi_filtered_pps = 6.0f;
  sample.csi_missing_slots_pps = 10.0f;
  sample.csi_excess_pps = 6.0f;
  sample.csi_stale_pps = 0.0f;
  sample.csi_out_of_order_pps = 0.0f;
  sample.csi_occupancy_ratio = 0.84f;
  sample.wifi_channel = 10U;
  sample.wifi_rssi_dbm = -55;
  mqtt_transport_mock::state.publishes.clear();

  mqtt.emit_message("espectre/v1/devices/0000abcdeffedcba/ha/diagnostics/set", "PRESS");

  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/traffic_tx_rate/state", "100.0"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_callback_rate/state", "96.0"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_accepted_rate/state", "90.0"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_admitted_rate/state", "84.0"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_filtered_rate/state", "6.0"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_missing_rate/state", "10.0"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_excess_rate/state", "6.0"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_stale_rate/state", "0.0"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_out_of_order_rate/state", "0.0"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/csi_occupancy/state", "84.0"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/wifi_channel/state", "10"));
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/wifi_rssi/state", "-55"));
}

void test_native_frontend_motion_edge_publishes_ready_ha_motion(void) {
  frontend_runtime_shim::state.snapshot = make_ready_snapshot();
  MockMqttTransport mqtt;
  EspectreDeviceConfig config;
  config.device_id = 0x0000abcdeffedcbaULL;
  config.mqtt_scheme = "mqtt";
  config.mqtt_host = "localhost";
  config.mqtt_port = 1883U;

  NativeFrontend frontend(&mqtt);
  frontend.set_device_config(config);
  TEST_ASSERT_TRUE(frontend.setup());
  mqtt.emit_connection(true);
  mqtt_transport_mock::state.publishes.clear();

  RuntimeSnapshot snapshot = make_ready_snapshot();
  snapshot.ready_to_publish = false;
  frontend.on_motion_state_changed(snapshot);
  frontend.loop();
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/motion/state"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/motion"));

  snapshot.ready_to_publish = true;
  frontend.on_motion_state_changed(snapshot);
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/motion/state"));
  frontend.loop();
  TEST_ASSERT_TRUE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/ha/motion/state", "ON"));
  TEST_ASSERT_FALSE(has_mqtt_publish("espectre/v1/devices/0000abcdeffedcba/motion"));
}


int main(int argc, char **argv) {
  (void) argc;
  (void) argv;
  UNITY_BEGIN();
  RUN_TEST(test_native_frontend_mqtt_connect_publishes_ha_discovery_and_subscribes_birth_topics);
  RUN_TEST(test_native_frontend_ha_birth_message_republishes_discovery_and_state);
  RUN_TEST(test_native_frontend_retries_the_complete_ha_snapshot_after_queue_backpressure);
  RUN_TEST(test_native_frontend_defers_initial_ha_state_until_sensing_is_ready);
  RUN_TEST(test_native_frontend_ha_entities_follow_esphome_cadences);
  RUN_TEST(test_native_frontend_ha_threshold_command_updates_runtime);
  RUN_TEST(test_native_frontend_ha_motion_hits_commands_update_runtime);
  RUN_TEST(test_native_frontend_ha_calibrate_command_triggers_runtime);
  RUN_TEST(test_native_frontend_ha_calibrate_command_respects_manual_recalibration_capability);
  RUN_TEST(test_native_frontend_ha_traffic_control_commands_update_runtime);
  RUN_TEST(test_native_frontend_ha_detector_command_updates_canonical_config);
  RUN_TEST(test_native_frontend_ha_diagnostics_button_publishes_cached_sample);
  RUN_TEST(test_native_frontend_motion_edge_publishes_ready_ha_motion);
  return UNITY_END();
}
