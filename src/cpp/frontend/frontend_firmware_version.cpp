/*
 * ESPectre - Frontend Firmware Version
 *
 * Firmware version string helper for first-party frontends.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "frontend_firmware_version.h"

#if __has_include("esp_app_desc.h")
#include "esp_app_desc.h"
#endif

namespace espectre {

const char *frontend_firmware_version() {
#ifdef APP_PROJECT_VER
  return APP_PROJECT_VER;
#elif __has_include("esp_app_desc.h")
  const esp_app_desc_t *app_desc = esp_app_get_description();
  return (app_desc != nullptr && app_desc->version[0] != '\0') ? app_desc->version : "unknown";
#else
  return "unknown";
#endif
}

}  // namespace espectre
