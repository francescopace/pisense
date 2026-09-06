cmake_minimum_required(VERSION 3.22)

project(espectre_tests LANGUAGES C CXX)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Host-side test build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        Debug Release RelWithDebInfo MinSizeRel)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The host build is the one agents and CI actually run, so it must carry the
# diagnostics the shared core relies on. -Wswitch in particular is what reports
# an MLFeatureId enumerator that ml_feature_source() forgot to classify.
add_compile_options(-Wall)
if(MSVC)
    add_compile_options($<$<CONFIG:RelWithDebInfo>:/UNDEBUG>)
else()
    add_compile_options($<$<CONFIG:RelWithDebInfo>:-UNDEBUG>)
endif()

set(ESPECTRE_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(ESPECTRE_CPP_ROOT "${ESPECTRE_REPO_ROOT}/src/cpp")
set(ESPECTRE_TEST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

include(CTest)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(Coverage)
include(TestMocks)
include(EspectreTestLib)

add_subdirectory(suites)
