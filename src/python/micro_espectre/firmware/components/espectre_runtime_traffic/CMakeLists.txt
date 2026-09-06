# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.

# Focused ESP-IDF runtime component for the shared managed traffic generator.
# Micro-ESPectre links this separately from the detector-only core component so
# platform socket and FreeRTOS ownership remain in runtime/esp_idf.
if(NOT ESPECTRE_CORE_SDK_ROOT AND DEFINED ENV{ESPECTRE_CORE_SDK_ROOT})
    set(ESPECTRE_CORE_SDK_ROOT "$ENV{ESPECTRE_CORE_SDK_ROOT}")
endif()

if(NOT ESPECTRE_CORE_SDK_ROOT)
    message(FATAL_ERROR "ESPECTRE_CORE_SDK_ROOT must point to the ESPectre C++ SDK")
endif()

include("${ESPECTRE_CORE_SDK_ROOT}/espectre_sources.cmake")

idf_component_register(
    SRCS
        "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/sta_socket_helpers.cpp"
        "${ESPECTRE_CPP_ROOT}/runtime/esp_idf/traffic_generator_manager.cpp"
    INCLUDE_DIRS
        ${ESPECTRE_SDK_ROOT_INCLUDE_DIRS}
        ${ESPECTRE_CORE_INCLUDE_DIRS}
        ${ESPECTRE_RUNTIME_INCLUDE_DIRS}
    REQUIRES
        espectre_core
        esp_netif
        esp_timer
        esp_wifi
        freertos
        lwip
)
