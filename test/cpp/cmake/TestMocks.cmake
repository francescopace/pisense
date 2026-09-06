add_library(espectre_test_mocks INTERFACE)

target_include_directories(espectre_test_mocks
    INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/mocks"
        "${CMAKE_CURRENT_SOURCE_DIR}/mocks/esp_idf"
        "${CMAKE_CURRENT_SOURCE_DIR}/mocks/esphome"
        "${CMAKE_CURRENT_SOURCE_DIR}/support"
        "${ESPECTRE_CPP_ROOT}"
        "${ESPECTRE_CPP_ROOT}/core"
        "${ESPECTRE_CPP_ROOT}/runtime"
        "${ESPECTRE_CPP_ROOT}/runtime/esp_idf"
)

target_compile_definitions(espectre_test_mocks
    INTERFACE
        CONFIG_FREERTOS_HZ=100
)
