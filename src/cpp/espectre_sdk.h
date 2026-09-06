/*
 * ESPectre - SDK Facade
 *
 * Single entry point for firmware integrating the ESPectre sensing engine.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

/**
 * @mainpage ESPectre SDK
 *
 * This reference covers the supported integration surface only. Every
 * declaration included in the reference follows the SDK version contract;
 * implementation dependencies that merely ship in the bundle are internal and
 * may change in any release.
 *
 * Start at espectre_sdk.h for the integration model, the threading contract,
 * and a working example. The
 * [full SDK guide](https://github.com/francescopace/espectre/blob/main/docs/SDK.md)
 * covers build integration, install surfaces, and release channels.
 */

/**
 * @file espectre_sdk.h
 * @brief The public ESPectre integration surface, in one include.
 *
 * ESPectre turns ordinary Wi-Fi traffic into a motion signal: it captures
 * Channel State Information from the radio, extracts features, and reports a
 * debounced motion state. This header is the supported entry point for
 * firmware that embeds that engine instead of flashing one of the published
 * frontends.
 *
 * @code
 * #include "espectre_sdk.h"
 *
 * class ProductFrontend : public espectre::IRuntimeListener {
 *  public:
 *   bool setup() {
 *     espectre::RuntimeConfig config;  // documented defaults, ready to use
 *     runtime_.set_config(config);
 *     return runtime_.setup(this);
 *   }
 *
 *   void loop() { runtime_.loop(); }
 *
 *   void on_motion_state_changed(const espectre::RuntimeSnapshot &snapshot) override {
 *     if (!snapshot.ready_to_publish) return;
 *     publish(snapshot.motion_state == espectre::MotionState::MOTION);
 *   }
 *
 *  private:
 *   espectre::RuntimeFrontendController runtime_;
 * };
 * @endcode
 *
 * @section sdk_paths Two integration paths
 *
 * - **Full runtime (recommended).** Your firmware owns boot, provisioning,
 *   networking, OTA, and the product surface. ESPectre owns Wi-Fi CSI capture,
 *   calibration, detection, and eventing behind
 *   `espectre::RuntimeFrontendController` and `espectre::IRuntimeListener`.
 *   Requires ESP-IDF >= 5.5.
 * - **Core-only.** Your firmware already captures CSI. Include
 *   `espectre_core_sdk.h` and drive `espectre::LightweightDetector` or
 *   `espectre::HighAccuracyDetector` directly. `runtime/esp_idf/csi_pipeline.cpp`
 *   is the reference for normalization, evaluation cadence, and hit filtering.
 *
 * @section sdk_threading Threading contract
 *
 * The control surface is single-owner. Internal bounded mailboxes protect
 * callback-to-loop handoff, but they do not make control calls thread-safe.
 *
 * - Run `setup()`, `loop()`, and `shutdown()` on one task. These are the calls
 *   that build and tear down runtime state, and they are not safe to race.
 * - Every `IRuntimeListener` callback is delivered on the caller's task: from
 *   `loop()` for sensing events, or inline on the task that invoked a control
 *   method. Work raised in the Wi-Fi CSI callback is deferred through an
 *   internal mailbox first, so no listener callback runs in interrupt or Wi-Fi
 *   driver context.
 * - Keep callbacks bounded and non-blocking. Slow work delays `loop()` and can
 *   fill the bounded CSI mailbox, dropping incoming frames. Queue network I/O,
 *   NVS writes, and other blocking work for another task.
 * - Call `set_*_runtime()` only from the owner task. The shipped MQTT, Direct
 *   HTTP, and OTA adapters queue stack events and deliver their application callbacks
 *   from the frontend loop, so Native follows this rule without external locks.
 * - Do not drive the controller from inside `on_runtime_fault()` beyond
 *   `shutdown()`.
 * - Raw CSI packet callbacks are the deliberate exception to listener delivery:
 *   they run synchronously in Wi-Fi capture context. Keep them bounded,
 *   non-blocking, and allocation-free; see `raw_csi_packet_callback_t`.
 *
 * @section sdk_versioning Versioning
 *
 * `ESPECTRE_SDK_VERSION_STRING` and `ESPECTRE_SDK_VERSION_AT_LEAST()` identify
 * the SDK sources you compiled against. See `runtime/espectre_sdk_version.h`
 * for how that differs from your firmware version.
 *
 * @section sdk_stability Stability tiers
 *
 * Everything reachable from this header is the stable runtime surface and
 * follows the SDK version contract. The opt-in `espectre_core_sdk.h` facade is
 * the lower-level detector extension. Other headers are internal and can
 * change in any release. `docs/SDK.md` defines the exact guarantees.
 *
 * @section sdk_licensing Licensing
 *
 * ESPectre is dual-licensed: GPLv3, or a separately offered commercial license
 * for proprietary firmware. See `LICENSING.md`.
 */

// SDK identity.
#include "runtime/espectre_sdk_version.h"

// Optional frontend-owned logging sink. No sink is installed by default.
#include "core/espectre_log.h"

// Runtime contracts. Platform-agnostic and host-testable.
#include "runtime/runtime_capabilities.h"
#include "runtime/runtime_config_utils.h"
#include "runtime/runtime_diagnostics.h"
#include "runtime/runtime_events.h"
#include "runtime/runtime_interface.h"
#include "runtime/raw_csi.h"
#include "runtime/runtime_sensing_schema.h"
#include "runtime/runtime_snapshot.h"

// Boundary interfaces you implement to reach your own transports.
#include "runtime/espectre_protocol.h"
#include "runtime/protocol_json.h"
#include "runtime/direct_http_service.h"
#include "runtime/mqtt_transport.h"

// Recommended entry point. The declaration is portable; linking it requires
// the ESP-IDF runtime sources.
#include "runtime/esp_idf/device_identity.h"
#include "runtime/esp_idf/runtime_frontend_controller.h"
#include "runtime/esp_idf/runtime_sensing_kconfig.h"
