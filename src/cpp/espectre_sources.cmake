if(NOT DEFINED ESPECTRE_CPP_ROOT)
    get_filename_component(ESPECTRE_CPP_ROOT "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
endif()

set(ESPECTRE_CORE_SOURCES
    "${ESPECTRE_CPP_ROOT}/core/espectre_log.cpp"
    "${ESPECTRE_CPP_ROOT}/core/base_detector.cpp"
    "${ESPECTRE_CPP_ROOT}/core/temporal_csi_sampler.cpp"
    "${ESPECTRE_CPP_ROOT}/core/lightweight_detector.cpp"
    "${ESPECTRE_CPP_ROOT}/core/filters.cpp"
    "${ESPECTRE_CPP_ROOT}/core/filtered_turbulence_ring.cpp"
    "${ESPECTRE_CPP_ROOT}/core/high_accuracy_detector.cpp"
)

set(ESPECTRE_RUNTIME_COMMON_SOURCES
    "${ESPECTRE_CPP_ROOT}/runtime/csi_traffic_service.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/periodic_sensing_status_logger.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/espectre_protocol.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/direct_http_protocol.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/peer_discovery.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/protocol_json.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/runtime_config_utils.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/runtime_diagnostics.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/runtime_time.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/frontend_command_engine.cpp"
)

set(ESPECTRE_RUNTIME_FRONTEND_SUPPORT_SOURCES
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/frontend_bootstrap_helpers.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/frontend_ha_mqtt_helpers.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/frontend_mqtt_helpers.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/frontend_sysinfo_helpers.cpp"
)

set(ESPECTRE_RUNTIME_ESP_IDF_PLATFORM_SOURCES
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/primary_console.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/device_identity.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/csi_capture_service.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/csi_pipeline.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/csi_payload_normalizer.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/csi_platform_config.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/esp_idf_runtime.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/esp_idf_runtime_base.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/nvs_helpers.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/runtime_frontend_controller.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/runtime_detector_store.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/runtime_motion_hits_store.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/runtime_traffic_mode_store.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/runtime_performance_diagnostics.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/runtime_sensing_kconfig.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/standalone_wifi_service.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/sta_socket_helpers.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/csi_frame_identity.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/traffic_generator_manager.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/udp_datagram_socket_esp_idf.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/udp_listener.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/wifi_lifecycle.cpp"
)

set(ESPECTRE_RUNTIME_ESP_IDF_SOURCES
    ${ESPECTRE_RUNTIME_COMMON_SOURCES}
    ${ESPECTRE_RUNTIME_ESP_IDF_PLATFORM_SOURCES}
)

set(ESPECTRE_RUNTIME_ESP_IDF_IMPROV_SOURCES
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/improv_serial_service.cpp"
)

set(ESPECTRE_RUNTIME_ESP_IDF_PROVISIONING_SOURCES
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/device_config_store.cpp"
    ${ESPECTRE_RUNTIME_ESP_IDF_IMPROV_SOURCES}
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/wifi_provisioning_service.cpp"
)

set(ESPECTRE_RUNTIME_ESP_IDF_MQTT_SOURCES
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/mqtt_transport_esp_idf.cpp"
)

set(ESPECTRE_RUNTIME_ESP_IDF_DIRECT_SOURCES
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/direct_wifi_snapshot_esp_idf.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/direct_http_service_esp_idf.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/mdns_bootstrap_responder.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/mdns_discovery_service.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/peer_discovery_service_esp_idf.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/raw_csi_session_controller.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/runtime_direct_http_bridge.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/wifi_bssid_pin_service.cpp"
)

set(ESPECTRE_FRONTEND_OTA_PROTOCOL_SOURCES
    "${ESPECTRE_CPP_ROOT}/frontend/ota_protocol.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/ota_version.cpp"
)

set(ESPECTRE_FRONTEND_OTA_SOURCES
    ${ESPECTRE_FRONTEND_OTA_PROTOCOL_SOURCES}
    "${ESPECTRE_CPP_ROOT}/frontend/ota_service_https.cpp"
)

set(ESPECTRE_FRONTEND_COMMON_SOURCES
    "${ESPECTRE_CPP_ROOT}/frontend/frontend_firmware_version.cpp"
)

set(ESPECTRE_FRONTEND_ESPHOME_SOURCES
    ${ESPECTRE_FRONTEND_COMMON_SOURCES}
    "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre/esphome_log_sink.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre/recalibrate_button.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre/sensing_switch.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre/detector_select.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre/diagnostics_button.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre/espectre.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre/motion_hits_number.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre/sensor_publisher.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre/threshold_number.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre/traffic_mode_select.cpp"
)

set(ESPECTRE_FRONTEND_MATTER_SOURCES
    ${ESPECTRE_FRONTEND_COMMON_SOURCES}
    "${ESPECTRE_CPP_ROOT}/frontend/matter/espectre/matter_frontend.cpp"
)

set(ESPECTRE_FRONTEND_NATIVE_SOURCES
    ${ESPECTRE_FRONTEND_COMMON_SOURCES}
    "${ESPECTRE_CPP_ROOT}/frontend/native/espectre/home_assistant_mqtt_frontend.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/native/espectre/native_command_bindings.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/native/espectre/native_direct_frontend.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/native/espectre/recovery_button_service.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/native/espectre/native_mqtt_frontend.cpp"
    "${ESPECTRE_CPP_ROOT}/frontend/native/espectre/native_frontend.cpp"
)

set(ESPECTRE_FRONTEND_NATIVE_ESP_IDF_SOURCES)

# SDK root. Exposes the `espectre_sdk.h` facade and lets integrators use
# layer-prefixed includes such as "runtime/runtime_interface.h" instead of the
# flat header names, which is what keeps generic basenames like `utils.h` from
# colliding with the consuming project.
set(ESPECTRE_SDK_ROOT_INCLUDE_DIRS
    "${ESPECTRE_CPP_ROOT}"
)

set(ESPECTRE_CORE_INCLUDE_DIRS
    "${ESPECTRE_CPP_ROOT}/core"
)

set(ESPECTRE_RUNTIME_INCLUDE_DIRS
    "${ESPECTRE_CPP_ROOT}/runtime"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf"
)

set(ESPECTRE_SHARED_INCLUDE_DIRS
    ${ESPECTRE_SDK_ROOT_INCLUDE_DIRS}
    ${ESPECTRE_CORE_INCLUDE_DIRS}
    ${ESPECTRE_RUNTIME_INCLUDE_DIRS}
)
