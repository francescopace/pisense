# ESPHome compiles this directory's adapter sources in its generated `src`
# component. Register the canonical shared ESPectre sources as a separate,
# native ESP-IDF component through the SDK build definition.
set(ESPECTRE_SDK_ADDITIONAL_REQUIRES esp_http_server espressif__mdns esp_app_format)
set(ESPECTRE_SDK_EXCLUDED_REQUIRES improv)
include("${CMAKE_CURRENT_LIST_DIR}/../../../../CMakeLists.txt")

# ESPHome copies adapter sources from this directory only. Compile the shared
# frontend helper here, without adding it to the standalone SDK source groups.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    target_sources(${COMPONENT_LIB} PRIVATE ${ESPECTRE_FRONTEND_COMMON_SOURCES})
endif()
