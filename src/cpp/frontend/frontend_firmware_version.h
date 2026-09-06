/*
 * ESPectre - Frontend Firmware Version
 *
 * Firmware version string helper for first-party frontends.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

namespace espectre {

/**
 * The running frontend application's version, as its build system defined it.
 *
 * Resolved from `APP_PROJECT_VER` when set, otherwise from the ESP-IDF
 * application descriptor, and `"unknown"` on a host build. First-party
 * frontends pass this value to the runtime for device information, discovery,
 * provisioning, and OTA comparisons. SDK integrators supply their own product
 * version through the same runtime interfaces.
 *
 * @return A static string, valid for the process lifetime. Never `nullptr`.
 */
const char *frontend_firmware_version();

}  // namespace espectre
