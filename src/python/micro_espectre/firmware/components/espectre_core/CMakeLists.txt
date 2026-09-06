# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.

# Core-only ESP-IDF component for the Micro-ESPectre firmware. Keep the source
# registry in the C++ SDK so the native binding cannot drift onto a copied subset.
if(NOT ESPECTRE_CORE_SDK_ROOT AND DEFINED ENV{ESPECTRE_CORE_SDK_ROOT})
    set(ESPECTRE_CORE_SDK_ROOT "$ENV{ESPECTRE_CORE_SDK_ROOT}")
endif()

if(NOT ESPECTRE_CORE_SDK_ROOT)
    message(FATAL_ERROR "ESPECTRE_CORE_SDK_ROOT must point to the ESPectre C++ SDK")
endif()

include("${ESPECTRE_CORE_SDK_ROOT}/espectre_sources.cmake")

idf_component_register(
    SRCS
        ${ESPECTRE_CORE_SOURCES}
    INCLUDE_DIRS
        ${ESPECTRE_SDK_ROOT_INCLUDE_DIRS}
        ${ESPECTRE_CORE_INCLUDE_DIRS}
)
