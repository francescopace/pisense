# ESPectre git-describe version
#
# Resolves ESPECTRE_GIT_VERSION from numeric git tags. Rolling GitHub tags such
# as `snapshot` and `snapshot-dev` are ignored. Keep the git arguments in sync with
# `.github/scripts/detect_git_version.py`.
#
# Used by first-party firmware builds only. SDK identity comes from its header.
# CI may supply ESPECTRE_GIT_VERSION explicitly; source archives default to 0.0.0.

if(NOT DEFINED ESPECTRE_GIT_VERSION OR ESPECTRE_GIT_VERSION STREQUAL "")
    if(DEFINED ENV{ESPECTRE_GIT_VERSION} AND NOT "$ENV{ESPECTRE_GIT_VERSION}" STREQUAL "")
        set(ESPECTRE_GIT_VERSION "$ENV{ESPECTRE_GIT_VERSION}")
    endif()
endif()

if(NOT DEFINED ESPECTRE_GIT_VERSION OR ESPECTRE_GIT_VERSION STREQUAL "")
    get_filename_component(_espectre_git_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
    set(_espectre_described "")
    # Only describe this checkout, not a containing Git project.
    if(EXISTS "${_espectre_git_root}/.git")
        execute_process(
            COMMAND git describe --tags --match "[0-9]*" --abbrev=7
            WORKING_DIRECTORY "${_espectre_git_root}"
            OUTPUT_VARIABLE _espectre_described
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE ESPECTRE_GIT_DESCRIBE_RESULT
        )
        if(NOT ESPECTRE_GIT_DESCRIBE_RESULT EQUAL 0)
            set(_espectre_described "")
        endif()
    endif()

    if(NOT _espectre_described STREQUAL "")
        set(ESPECTRE_GIT_VERSION "${_espectre_described}")
    else()
        set(ESPECTRE_GIT_VERSION "0.0.0")
    endif()
    unset(_espectre_git_root)
    unset(_espectre_described)
    unset(ESPECTRE_GIT_DESCRIBE_RESULT)
endif()

if(NOT ESPECTRE_GIT_VERSION)
    message(FATAL_ERROR "ESPECTRE_GIT_VERSION is empty")
endif()
if(NOT ESPECTRE_GIT_VERSION MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)")
    message(FATAL_ERROR "ESPECTRE_GIT_VERSION ${ESPECTRE_GIT_VERSION} is not MAJOR.MINOR.PATCH")
endif()
