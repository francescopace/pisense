include(FetchContent)
include("${ESPECTRE_CPP_ROOT}/espectre_sources.cmake")

find_package(ZLIB REQUIRED)

find_path(ARDUINOJSON_INCLUDE_DIR ArduinoJson.h)
if(NOT ARDUINOJSON_INCLUDE_DIR)
    FetchContent_Declare(
        ArduinoJson
        URL https://github.com/bblanchon/ArduinoJson/releases/download/v6.21.4/ArduinoJson-v6.21.4.hpp
        DOWNLOAD_NO_EXTRACT TRUE
        DOWNLOAD_NAME ArduinoJson.h
    )
    FetchContent_MakeAvailable(ArduinoJson)
    set(ARDUINOJSON_INCLUDE_DIR "${arduinojson_SOURCE_DIR}")
endif()

FetchContent_Declare(
    ImprovWifiSdk
    GIT_REPOSITORY https://github.com/improv-wifi/sdk-cpp.git
    GIT_TAG 17898613a1c17062ca5af295ceb639b16b4930bf
    GIT_SHALLOW FALSE
)
FetchContent_MakeAvailable(ImprovWifiSdk)

add_library(improv_wifi_testlib STATIC
    "${improvwifisdk_SOURCE_DIR}/src/improv.cpp"
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # The pinned upstream source has three signedness warnings under -Wall.
    # Keep project diagnostics enabled without applying them to third-party code.
    target_compile_options(improv_wifi_testlib PRIVATE -Wno-sign-compare)
endif()
target_include_directories(improv_wifi_testlib
    PUBLIC
        "${improvwifisdk_SOURCE_DIR}/src"
)

add_library(espectre_test_framework STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/support/test_harness.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mocks/esp_idf/esp_event_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mocks/esp_idf/esp_http_server_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mocks/esp_idf/mdns_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mocks/esp_idf/mqtt_client_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mocks/esp_idf/esp_netif_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mocks/esp_idf/nvs_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mocks/esp_idf/esp_wifi_mock.cpp"
)
target_include_directories(espectre_test_framework
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/support"
)
target_link_libraries(espectre_test_framework
    PUBLIC
        espectre_test_mocks
)

add_library(espectre_test_support STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/support/cnpy.cpp"
)
target_include_directories(espectre_test_support
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/support"
        "${ARDUINOJSON_INCLUDE_DIR}"
)
target_link_libraries(espectre_test_support
    PUBLIC
        espectre_test_mocks
        ZLIB::ZLIB
)

add_library(espectre_core_testlib STATIC
    ${ESPECTRE_CORE_SOURCES}
)
target_link_libraries(espectre_core_testlib
    PUBLIC
        espectre_test_framework
        espectre_test_mocks
)

add_library(espectre_runtime_testlib STATIC
    "${ESPECTRE_CPP_ROOT}/runtime/csi_traffic_service.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/primary_console.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/device_identity.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/periodic_sensing_status_logger.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/espectre_protocol.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/direct_http_protocol.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/peer_discovery.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/protocol_json.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/runtime_config_utils.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/runtime_diagnostics.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/runtime_time.cpp"
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
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/mdns_discovery_service.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/peer_discovery_service_esp_idf.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/standalone_wifi_service.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/sta_socket_helpers.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/csi_frame_identity.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/direct_wifi_snapshot_esp_idf.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/traffic_generator_manager.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/udp_datagram_socket_esp_idf.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/udp_listener.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/wifi_lifecycle.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/frontend_bootstrap_helpers.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/frontend_command_engine.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/frontend_ha_mqtt_helpers.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/frontend_mqtt_helpers.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/frontend_sysinfo_helpers.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/raw_csi_session_controller.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/device_config_store.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/improv_serial_service.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/wifi_provisioning_service.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/wifi_bssid_pin_service.cpp"
)
target_link_libraries(espectre_runtime_testlib
    PUBLIC
        espectre_core_testlib
        espectre_test_mocks
        improv_wifi_testlib
)
target_compile_definitions(espectre_runtime_testlib
    PRIVATE
        ESPECTRE_HOST_WIFI_CONTROL_TEST=1
)

add_library(espectre_runtime_invalid_kconfig_testlib STATIC
    "${ESPECTRE_CPP_ROOT}/runtime/runtime_config_utils.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/runtime_sensing_kconfig.cpp"
)
target_compile_definitions(espectre_runtime_invalid_kconfig_testlib
    PRIVATE
        CONFIG_ESPECTRE_MOTION_ON_HITS=260
        CONFIG_ESPECTRE_MOTION_OFF_HITS=258
        CONFIG_ESPECTRE_HAMPEL_WINDOW=263
        CONFIG_ESPECTRE_CSI_TRAFFIC_MULTICAST_GROUP="not-an-address"
)
target_link_libraries(espectre_runtime_invalid_kconfig_testlib
    PUBLIC
        espectre_core_testlib
        espectre_test_mocks
)

add_library(espectre_direct_service_testlib STATIC
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/direct_http_service_esp_idf.cpp"
)

add_library(espectre_native_mdns_bootstrap_testlib STATIC
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/mdns_bootstrap_responder.cpp"
)
target_link_libraries(espectre_native_mdns_bootstrap_testlib
    PUBLIC
        espectre_test_mocks
)
target_include_directories(espectre_native_mdns_bootstrap_testlib
    PUBLIC
        "${ESPECTRE_CPP_ROOT}/runtime/esp_idf"
)

add_library(espectre_mqtt_transport_testlib STATIC
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/mqtt_transport_esp_idf.cpp"
)
add_library(espectre_frontend_ota_protocol_testlib STATIC ${ESPECTRE_FRONTEND_OTA_PROTOCOL_SOURCES})
target_link_libraries(espectre_frontend_ota_protocol_testlib PUBLIC espectre_runtime_testlib)
add_library(espectre_ota_https_testlib STATIC
    "${ESPECTRE_CPP_ROOT}/frontend/ota_service_https.cpp"
)
target_link_libraries(espectre_ota_https_testlib
    PUBLIC
        espectre_frontend_ota_protocol_testlib
        espectre_test_mocks
)
target_link_libraries(espectre_mqtt_transport_testlib
    PUBLIC
        espectre_runtime_testlib
        espectre_test_mocks
)
target_link_libraries(espectre_direct_service_testlib
    PUBLIC
        espectre_runtime_testlib
        espectre_test_mocks
)
target_compile_definitions(espectre_runtime_testlib
    PUBLIC
        CONFIG_ESPECTRE_HA_DISCOVERY_ENABLED=1
)

# The dual-band radio policy is selected at compile time by CONFIG_SOC_WIFI_SUPPORT_5G,
# which the host sdkconfig mock leaves off because it models a 2.4 GHz-only part.
# Compiling the same lifecycle unit a second time with the capability forced on is
# what keeps the ESP32-C5 branch under host test.
add_library(espectre_runtime_dual_band_testlib STATIC
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/wifi_lifecycle.cpp"
)
target_link_libraries(espectre_runtime_dual_band_testlib
    PUBLIC
        espectre_core_testlib
)
target_compile_definitions(espectre_runtime_dual_band_testlib
    PUBLIC
        CONFIG_SOC_WIFI_SUPPORT_5G=1
)

add_library(espectre_frontend_esphome_testlib STATIC
    ${ESPECTRE_FRONTEND_ESPHOME_SOURCES}
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/direct_http_service_esp_idf.cpp"
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/runtime_direct_http_bridge.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/support/frontend_runtime_shim.cpp"
)
target_compile_definitions(espectre_frontend_esphome_testlib
    PRIVATE
        ESPECTRE_HOST_WIFI_CONTROL_TEST=1
)
target_link_libraries(espectre_frontend_esphome_testlib
    PUBLIC
        espectre_runtime_testlib
        espectre_native_mdns_bootstrap_testlib
        espectre_test_mocks
)
target_include_directories(espectre_frontend_esphome_testlib
    PUBLIC
        "${ESPECTRE_CPP_ROOT}/frontend/esphome/components/espectre"
)

add_library(espectre_frontend_matter_testlib STATIC
    ${ESPECTRE_FRONTEND_MATTER_SOURCES}
    "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/runtime_direct_http_bridge.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/support/direct_http_service_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/support/matter_bindings_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/support/frontend_runtime_shim.cpp"
)
target_compile_definitions(espectre_frontend_matter_testlib
    PRIVATE
        ESPECTRE_HOST_WIFI_CONTROL_TEST=1
)
target_link_libraries(espectre_frontend_matter_testlib
    PUBLIC
        espectre_runtime_testlib
        espectre_test_mocks
)
target_include_directories(espectre_frontend_matter_testlib
    PUBLIC
        "${ESPECTRE_CPP_ROOT}/frontend/matter/espectre"
)

add_library(espectre_frontend_native_testlib STATIC
    ${ESPECTRE_FRONTEND_NATIVE_SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/support/direct_http_service_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/support/mqtt_transport_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/support/ota_service_mock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/support/frontend_runtime_shim.cpp"
)
target_link_libraries(espectre_frontend_native_testlib
    PUBLIC
        espectre_frontend_ota_protocol_testlib
        espectre_test_mocks
)
target_include_directories(espectre_frontend_native_testlib
    PUBLIC
        "${ESPECTRE_CPP_ROOT}/frontend/native/espectre"
)

foreach(target_name
        espectre_test_framework
        espectre_test_support
        espectre_core_testlib
        espectre_runtime_testlib
        espectre_runtime_invalid_kconfig_testlib
        espectre_runtime_dual_band_testlib
        espectre_direct_service_testlib
        espectre_native_mdns_bootstrap_testlib
        espectre_mqtt_transport_testlib
        espectre_frontend_ota_protocol_testlib
        espectre_ota_https_testlib
        espectre_frontend_esphome_testlib
        espectre_frontend_native_testlib
        espectre_frontend_matter_testlib)
    espectre_apply_coverage("${target_name}")
endforeach()
