include("${CMAKE_CURRENT_LIST_DIR}/../../../espectre_sources.cmake")

set(espectre_native_sources
    ${ESPECTRE_FRONTEND_NATIVE_SOURCES}
    ${ESPECTRE_FRONTEND_NATIVE_ESP_IDF_SOURCES}
    ${ESPECTRE_CORE_SOURCES}
    ${ESPECTRE_RUNTIME_ESP_IDF_SOURCES}
    ${ESPECTRE_RUNTIME_FRONTEND_SUPPORT_SOURCES}
    ${ESPECTRE_RUNTIME_ESP_IDF_PROVISIONING_SOURCES}
    ${ESPECTRE_RUNTIME_ESP_IDF_MQTT_SOURCES}
    ${ESPECTRE_RUNTIME_ESP_IDF_DIRECT_SOURCES}
    ${ESPECTRE_FRONTEND_OTA_SOURCES}
)

set(espectre_native_requires
    esp_wifi
    esp_event
    esp_netif
    esp_timer
    mqtt
    app_update
    esp_http_client
    esp_http_server
    esp_https_ota
    esp-tls
    mbedtls
    nvs_flash
    lwip
    driver
    freertos
    heap
    improv
    mdns
    log
)

idf_component_register(
    SRCS
        ${espectre_native_sources}
    INCLUDE_DIRS
        "."
        ${ESPECTRE_SHARED_INCLUDE_DIRS}
    REQUIRES
        ${espectre_native_requires}
)

idf_component_optional_requires(PRIVATE esp_tinyusb espressif__esp_tinyusb)

if(CONFIG_ESPECTRE_TINYUSB_PRIMARY_CONSOLE)
    target_link_options(
        ${COMPONENT_LIB}
        INTERFACE
            "LINKER:--wrap=esp_usb_console_available_for_read"
            "LINKER:--wrap=esp_usb_console_read_buf"
            "LINKER:--wrap=esp_usb_console_write_buf"
    )
endif()

idf_component_get_property(espectre_mdns_dir espressif__mdns COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${espectre_mdns_dir}/private_include")
target_link_options(${COMPONENT_LIB} INTERFACE "LINKER:--wrap=mdns_priv_receive_action")
