/*
 * ESPectre - SDK Version
 *
 * Compile-time version of the embedded ESPectre SDK.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

/**
 * @file espectre_sdk_version.h
 * @brief Compile-time identity of the ESPectre SDK sources you compiled against.
 *
 * This is the version of the *SDK*, not of your firmware. The two are
 * deliberately separate:
 *
 * - `ESPECTRE_SDK_VERSION_STRING` is baked in at compile time and identifies
 *   the ESPectre sources in your build. Use it in diagnostics, bug reports,
 *   and to guard code against SDK releases.
 * - The *application* version is supplied to the runtime by the frontend or
 *   integrator. In an integration it is your product's version, not ESPectre's.
 *
 * Published SDK bundles stamp their identity into this header. Integrators can
 * supply all four version macros at compile time to override the package.
 * The SDK does not inspect Git, environment variables, or firmware metadata.
 * Missing or incomplete metadata defaults to 0.0.0, meaning an unknown SDK
 * version. Its numeric macros are zero, so checks for newer releases are false.
 */

#if !defined(ESPECTRE_SDK_VERSION_MAJOR) && !defined(ESPECTRE_SDK_VERSION_MINOR) && \
    !defined(ESPECTRE_SDK_VERSION_PATCH) && !defined(ESPECTRE_SDK_VERSION_STRING)
/* ESPECTRE_SDK_VERSION_VALUES_BEGIN */
/* ESPECTRE_SDK_VERSION_VALUES_END */
#endif

#if !defined(ESPECTRE_SDK_VERSION_MAJOR) || !defined(ESPECTRE_SDK_VERSION_MINOR) || \
    !defined(ESPECTRE_SDK_VERSION_PATCH) || !defined(ESPECTRE_SDK_VERSION_STRING)
#undef ESPECTRE_SDK_VERSION_MAJOR
#undef ESPECTRE_SDK_VERSION_MINOR
#undef ESPECTRE_SDK_VERSION_PATCH
#undef ESPECTRE_SDK_VERSION_STRING
#define ESPECTRE_SDK_VERSION_MAJOR 0
#define ESPECTRE_SDK_VERSION_MINOR 0
#define ESPECTRE_SDK_VERSION_PATCH 0
#define ESPECTRE_SDK_VERSION_STRING "0.0.0"
#endif

/**
 * Legacy packed numeric identity for the SDK version, as `MMmmpp`.
 *
 * Example: `3.0.0` becomes `30000`. Retained for compatibility and compact
 * telemetry; do not use it for ordering because components are not limited to
 * two digits. Use `ESPECTRE_SDK_VERSION_AT_LEAST()` for feature guards.
 */
#define ESPECTRE_SDK_VERSION_NUMBER \
  ((ESPECTRE_SDK_VERSION_MAJOR * 10000) + (ESPECTRE_SDK_VERSION_MINOR * 100) + ESPECTRE_SDK_VERSION_PATCH)

/**
 * Compile-time feature guard.
 *
 * Use it to keep one integration compiling against several SDK releases:
 * @code
 * #if ESPECTRE_SDK_VERSION_AT_LEAST(3, 1, 0)
 *   controller.set_motion_hits_runtime(3, 5);
 * #endif
 * @endcode
 */
#define ESPECTRE_SDK_VERSION_AT_LEAST(major, minor, patch)                  \
  ((ESPECTRE_SDK_VERSION_MAJOR > (major)) ||                               \
   (ESPECTRE_SDK_VERSION_MAJOR == (major) &&                              \
    (ESPECTRE_SDK_VERSION_MINOR > (minor) ||                              \
     (ESPECTRE_SDK_VERSION_MINOR == (minor) &&                            \
      ESPECTRE_SDK_VERSION_PATCH >= (patch)))))

namespace espectre {

/**
 * The SDK version as a string, usable where a macro is not.
 *
 * @return `ESPECTRE_SDK_VERSION_STRING`, or `"0.0.0"` when unknown. Never null;
 *         valid for the process lifetime.
 */
constexpr const char *espectre_sdk_version() { return ESPECTRE_SDK_VERSION_STRING; }

}  // namespace espectre
