# C++ SDK Guide

This guide is for firmware teams that want to integrate the ESPectre sensing engine into their own ESP32 firmware instead of shipping one of the published frontends. It complements [ARCHITECTURE.md](ARCHITECTURE.md), which describes the internal layering in detail.

It assumes C++17, an ESP-IDF application or equivalent host build, and familiarity with callbacks and task ownership. A **snapshot** is one immutable view of runtime state, a **listener** receives runtime events, and a **capability** reports whether the selected backend supports an optional control. If you only need an existing ESPectre firmware image, use [SETUP.md](SETUP.md) instead.

## Five-minute integration

The shortest supported integration uses `espectre_sdk.h` and `RuntimeFrontendController`:

```cpp
#include "espectre_sdk.h"

class ProductFrontend : public espectre::IRuntimeListener {
 public:
  bool setup() {
    espectre::RuntimeConfig config;  // documented defaults, ready to use
    runtime_.set_config(config);
    return runtime_.setup(this);
  }

  void loop() { runtime_.loop(); }

  void on_motion_state_changed(const espectre::RuntimeSnapshot &snapshot) override {
    if (!snapshot.ready_to_publish) {
      return;
    }
    publish_motion(snapshot.motion_state == espectre::MotionState::MOTION);
  }

 private:
  espectre::RuntimeFrontendController runtime_;
};
```

On ESP-IDF, replace the bare `RuntimeConfig` with `espectre::make_runtime_sensing_config_from_kconfig()` to drive the sensing settings from menuconfig.

Before adding product-specific behavior, enforce these runtime constraints:

- Gate everything user-visible on `snapshot.ready_to_publish`. The runtime emits snapshots while it calibrates, and motion state is not meaningful before that flag is true.
- Read `runtime_.snapshot()` for on-demand state. The controller refreshes it before forwarding each listener callback; frontends do not maintain a second cache.
- Run `setup()`, `loop()`, and `shutdown()` on one task.
- Ask `capabilities()` before exposing a control, rather than assuming the active runtime supports it.

## What you embed

| Layer | Contents | Dependencies |
|-------|----------|--------------|
| `src/cpp/espectre_sdk.h` | Stable full-runtime SDK facade | Single include; link the selected core and runtime sources |
| `src/cpp/espectre_core_sdk.h` | Optional core-only detector facade | C++17 standard library only |
| `src/cpp/core/` | Lightweight and High-Accuracy detectors, feature extraction, filters, CSI format | C++17 standard library only |
| `src/cpp/runtime/` | Runtime contracts, snapshots, events, ESPectre Protocol message and capability models, traffic generation | Portable, host-testable |
| `src/cpp/runtime/esp_idf/` | CSI capture, Wi-Fi lifecycle, sensing pipeline, traffic generation, NVS persistence | ESP-IDF `>= 5.5` |
| `src/cpp/frontend/` | ESPHome, Native Direct/MQTT, and Matter reference integrations | Frontend-specific stacks |

The layering is strict: `core` has no upward or SDK dependencies, and `runtime` contracts stay platform-agnostic, so the sensing logic can be compiled, tested, and simulated on a host machine without ESP-IDF.

### Stability tiers

| Tier | What it covers | Change policy |
|------|----------------|---------------|
| Stable runtime | Everything reachable from `espectre_sdk.h` | Follows the SDK version contract below |
| Core-only extension | Detector classes and documented public methods exposed by `espectre_core_sdk.h` | Follows source compatibility; algorithm internals and exact numeric output may evolve as documented below |
| Internal | Headers and declarations not identified as either facade's public API | May change in any release; they ship because the runtime and core detector definitions need them to compile |

The frontend layer is a set of reference integrations, not a supported API. Read it for patterns; do not link against it.

## Supported hardware

ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C5, and ESP32-C6, using standard single-antenna Wi-Fi CSI with AGC active and HT20 bandwidth. No extra sensors or radio hardware are required. See [SETUP.md](SETUP.md) for the current per-frontend target matrix.

Set `RuntimeConfig::wifi_band_policy` to choose `BAND_2G`, `BAND_5G`, or `AUTO`. Full-runtime builds default to `AUTO` on dual-band silicon, currently ESP32-C5 among the published targets, and to `BAND_2G` everywhere else. A directly constructed `RuntimeConfig` remains target-neutral and defaults to `BAND_2G`; source-list integrations can override it before setup. The runtime applies the selected policy and pins 20 MHz bandwidth on the active band or bands. Unsupported policies fail setup instead of falling back silently, and packets outside the selected capture profile are dropped and counted.

The full runtime selects the read-only CSI capture profile after Wi-Fi association. The original ESP32 and ESP32-S2 use `lltf20`; a VHT-capable dual-band target uses `vht20` on 5 GHz; and every other association uses `ht20`. The active value is reported as `csi_profile` in the canonical `device` resource and is not a writable setting. LLTF admits legacy OFDM traffic while preserving the canonical centered 64-bin geometry: raw capture marks the unavailable physical tones ±27 and ±28 as zero, records the observed PHY and LLTF metadata, and copies I/Q from the nearest live ±26 tone only in the private detector view.

## Choosing a detection profile

Choose Lightweight Detection when sensing must leave more CPU time and working memory for the rest of the product. Choose High Accuracy when detection quality is the priority and the product can afford additional feature state and neural inference. [ALGORITHMS.md](ALGORITHMS.md#why-two-detection-profiles) owns the detector behavior and resource rationale, while [TUNING.md](TUNING.md#startup-and-detection-profile) owns the operator-facing choice.

For SDK integration, gate output on `RuntimeSnapshot::ready_to_publish`, mirror threshold changes from `IRuntimeListener::on_threshold_changed()`, and budget flash separately from the CPU and working memory used by the active profile. A runtime-switching build may contain both detector implementations and ML weights even while Lightweight is active.

## Integration paths

### Full runtime (recommended)

Your firmware owns boot, provisioning, networking policy, OTA, and the product surface; the ESPectre runtime owns CSI capture, calibration, detection, and eventing behind two contracts:

- `IEspectreRuntime` (`runtime/runtime_interface.h`): `setup()`, `loop()`, runtime threshold/detector control, recalibration, and snapshot access.
- `IRuntimeListener` (`runtime/runtime_events.h`): callbacks for motion-state changes, periodic updates, threshold/detector changes (including Lightweight settled-level recovery), calibration lifecycle, live telemetry, and runtime faults. If you publish a writable threshold control, override `on_threshold_changed()` rather than inferring the live value from telemetry.

`RuntimeFrontendController` wires configuration, runtime-control persistence, and the runtime backend together. After `setup()`, `config()` reflects the backend's effective configuration, including persisted detector, motion-hit, and traffic overrides; direct writes to `config()` after setup only stage the next setup, while live changes use the capability-gated runtime setters. The Native and Matter frontends are compact reference integrations for this path.

Set `RuntimeConfig::device_id` to `derive_runtime_device_id()` before setup when the integration uses the ESPectre Protocol or CSI streaming. The helper returns a cached pseudonym derived from the station MAC; zero remains an unresolved sentinel and is not replaced by `RuntimeFrontendController`.

### Core-only

If your firmware already owns Wi-Fi and CSI capture, include `espectre_core_sdk.h` and consume the detectors directly. The `core` detectors accept normalized CSI payloads and expose motion state, movement metric, and threshold control. The same facade exposes `TemporalCsiSampler`, which applies the production fixed-grid admission before `process_packet()`.

Detector and sampler working buffers use non-throwing allocation. Check `detector.is_valid()` after construction and the result of `sampler.configure(...)` before starting a custom pipeline; a false result means the requested bounded storage was unavailable. These objects are movable and intentionally non-copyable because their buffers own live temporal state.

The sampler tracks timing and slots; your integration stores the selected CSI payload. Handle each input in this order:

1. Call `admit()` before replacing the stored payload.
2. If `admit()` returns `true`, consume the stored payload: clear detector history when `reset_required()` is true, call `advance_missing_slots(missing_slots_before())`, and then call `process_packet()`.
3. If `gap_reset_required()` is true, clear detector history again before admitting post-gap data.
4. If `selected_current()` is true, replace the stored payload with the current normalized CSI.

At the end of a finite stream, call `flush()` and consume the stored payload if it returns `true`.

After each `update_state()`, re-read `get_threshold()`: Lightweight can lower it without a setter call, and the core-only path has no `on_threshold_changed()` hook. The sampler owns admission only; use `runtime/esp_idf/csi_pipeline.cpp` as the reference for CSI normalization, evaluation cadence, and hit filtering before committing to custom wiring.

A core-only integration that captures LLTF owns the same two-view boundary. Normalize the payload into the centered HT20 convention, call `zero_ht20_lltf_missing_bins()` on the raw view, copy that payload into the detector buffer, and call `impute_ht20_lltf_detector_bins()` only on the detector copy.

### Logging

Both SDK facades expose the portable logging contract in `core/espectre_log.h`. ESPectre does not install a sink or fall back to `stdio`, so integrations that do not need logs have no logger dependency and do not evaluate filtered log arguments. To receive shared logs, register a complete `LogSink` before runtime setup:

```cpp
espectre::LogSink sink{
    product_context,
    &product_log_enabled,
    &product_log_write,
};
if (!espectre::set_log_sink(sink)) {
  return false;
}
```

The `enabled` callback decides whether a level and tag should be formatted. The `write` callback receives the level, tag, source line, format string, and a `va_list` that remains valid only for that call. ESPectre copies the callback value but does not own its context. Keep the context alive until `clear_log_sink()`, and register, replace, or clear the sink only while no runtime is active. Callbacks may arrive from the runtime owner task, ESP-IDF service tasks, or CSI capture paths, so they must be thread-safe, bounded, non-blocking, and must not call the ESPectre logger recursively.

The shipped frontends provide the reference adapters. ESPHome sends messages to its logger. Native, Matter, and the Micro-ESPectre native bindings use ESP-IDF Log v2 and pass each callback's `va_list` to `esp_log_va`. ESP-IDF adds the standard level, timestamp, tag, and line ending, so these adapters do not need their own formatting buffer. Micro-ESPectre keeps the ESP-IDF dependency in its frontend module; its core-only and focused traffic components remain logger-independent and share the sink implementation linked from core. The shared SDK components remain independent of `esp_log`.

## Header map

| Header | Use it for |
|--------|------------|
| `espectre_sdk.h` | Stable full-runtime facade and recommended integration entry point |
| `espectre_core_sdk.h` | Opt-in core-only facade for integrations that already own normalized CSI capture |
| `core/espectre_log.h` | Register an optional frontend-owned logging sink for shared SDK messages |
| `runtime/espectre_sdk_version.h` | Compile-time SDK version and the `ESPECTRE_SDK_VERSION_AT_LEAST()` guard |
| `runtime/runtime_interface.h` | `RuntimeConfig` and the backend contract |
| `runtime/runtime_events.h` | `IRuntimeListener` and the threading contract |
| `runtime/runtime_snapshot.h` | `RuntimeSnapshot`: what every callback delivers |
| `runtime/csi_capture_profile.h` | Read-only CSI capture profile names and automatic-selection policy |
| `runtime/runtime_capabilities.h` | Which controls the active runtime honors |
| `runtime/runtime_sensing_schema.h` | Defaults and valid ranges for every tunable |
| `runtime/runtime_config_utils.h` | Validators and name/enum conversion |
| `runtime/runtime_diagnostics.h` | Capture and link counters, plus the sampler that turns them into rates |
| `runtime/csi_traffic_types.h` | Runtime traffic-source and generator mode enums used by `RuntimeConfig` |
| `runtime/csi_raw_record.h` | Transport-neutral CSI V8 record layout and historical V7 capture parsing |
| `runtime/raw_csi.h` | Optional raw-collection runtime state, session configuration, diagnostics, and Direct binary framing |
| `runtime/esp_idf/device_identity.h` | Derive the stable device pseudonym used by protocol and CSI surfaces |
| `runtime/esp_idf/runtime_frontend_controller.h` | The recommended entry point |
| `runtime/esp_idf/runtime_sensing_kconfig.h` | Build a config from menuconfig |
| `runtime/espectre_protocol.h` | Wire types, payload builders, command parsers |
| `runtime/protocol_json.h` | Decoded JSON fields and helpers for command validators |
| `runtime/mqtt_transport.h` | Implement to reach your own MQTT client |
| `runtime/direct_http_protocol.h` | Canonical request parsing, Direct HTTP constants, and the executable Direct/MQTT mapping |
| `runtime/direct_http_service.h` | Implement to expose Direct HTTP resource methods, SSE events, and optional CSI streaming |
| `core/detector_types.h`, `core/csi_types.h`, `core/filter_config.h`, `core/detector_limits.h` | Stable value types, dimensions, defaults, and ranges shared by both facades |
| **Core-only extension** | **Headers below are reached only through `espectre_core_sdk.h`** |
| `core/lightweight_detector.h`, `core/high_accuracy_detector.h` | The supported core-only detector classes |
| `core/base_detector.h` | The shared detector lifecycle both detectors inherit |
| `core/csi_format.h` | CSI layout and the subcarrier band the detectors measure on |
| `core/temporal_csi_sampler.h` | Production fixed-grid admission for a custom capture pipeline |
| `core/detector_limits.h` | Detector dimensions and limits used by the supported classes |

`EspectreDeviceConfig` represents an MQTT endpoint with separate `mqtt_scheme`, `mqtt_host`, and `mqtt_port` members. All three are required for MQTT to be configured; the default empty scheme and host with port `0` disable it. Use `validate_espectre_mqtt_config()` before persisting or applying an endpoint. The shipped ESP-IDF adapter accepts `mqtt` for explicit plaintext TCP and `mqtts` for TLS with the public certificate bundle and hostname verification. Host values are not URIs, and WebSocket transports are not part of this contract.

Headers such as `core/filtered_turbulence_ring.h`, `core/filters.h`, `core/utils.h`, `core/csi_features.h`, `core/ml_feature_trackers.h`, `core/l1_delta_tracker.h`, and `core/threshold.h` ship because the detector definitions depend on them. They are implementation dependencies rather than independent extension points, do not appear in the generated API reference, and may change without a compatibility guarantee.

## Runtime contract

### Threading

The control surface is single-owner. Internal bounded mailboxes protect callback-to-loop handoff, but they do not make control calls thread-safe.

- Run `setup()`, `loop()`, and `shutdown()` on one task.
- Every `IRuntimeListener` callback is delivered on the caller's task: from `loop()` for sensing events, or inline on the task that invoked a control method. Work raised in the Wi-Fi CSI callback is deferred through an internal mailbox first, so no listener callback runs in interrupt or Wi-Fi driver context.
- Keep callbacks bounded and non-blocking. A slow callback delays the next `loop()` iteration; sufficiently long work can fill the bounded CSI mailbox and drop incoming frames. Queue network publication, NVS writes, and other potentially blocking work for a separate task.
- Call `set_*_runtime()` only from the owner task. The shipped MQTT, Direct HTTP, and OTA adapters queue stack events and deliver application callbacks from the frontend loop, so Native follows this rule without external locks.
- Raw CSI packet callbacks are the deliberate exception: they run synchronously in the Wi-Fi CSI capture context. Keep them bounded, non-blocking, and allocation-free, and copy accepted samples into a preallocated bounded queue when another task must process them. Returning false reports a caller-owned drop or backpressure event; it does not stop collection.

Stopping raw collection synchronizes with any packet callback already in progress before releasing its context. The caller can reclaim that context after `stop_raw_collection()` succeeds. Raw callbacks must follow the owner-task rule for runtime controls; they must not stop collection themselves.

### Lifecycle

`set_config()` -> `setup(listener)` -> `loop()` repeatedly -> `shutdown()`. Create the default station interface and ESP event loop before `setup()`. Prefer setup before association so the CSI radio policy is applied at `WIFI_EVENT_STA_START`; setup after association is also supported and restores the station's current IPv4 state. The controller is reusable after `shutdown()`: the configuration survives and `set_config()` becomes effective again. `setup()` is idempotent, and a failed `setup()` leaves the controller un-setup so you can fix the config and retry.

Register an optional `LogSink` before `setup()`. Do not replace or clear it until every runtime and callback source using it has shut down.

High Accuracy preserves the configured or live threshold when sensing starts, Wi-Fi reconnects, or raw collection ends. Explicit recalibration and switching detectors restore the detector's default threshold. Lightweight continues to derive its threshold through startup calibration.

### Errors

The control surface reports failure through `bool` returns and never throws. Runtime-backend, temporal-sampler, and detector storage allocations are non-throwing; an allocation failure makes `setup()` return false and reports the fault synchronously to the listener. A `false` means the call was rejected or could not be applied, and the runtime is unchanged. There are four reasons a control call returns false:

1. The value is outside the range published in `runtime_sensing_schema.h`.
2. The active runtime does not advertise the matching capability.
3. The backend refused the change.
4. `setup()` could not allocate the runtime's bounded working storage.

Asynchronous failures arrive instead through `IRuntimeListener::on_runtime_fault()`. Calibration outcome is reported by `on_calibration_finished(snapshot, success)`; a `false` there is not fatal, the runtime keeps sensing with the configured threshold.

### Capabilities

`RuntimeCapabilities` defaults every flag to false, so a runtime declares what it offers rather than inheriting a permissive default. Read `controller.capabilities()` after `setup()` and expose only what it advertises. The controller already refuses capability-gated calls; this check keeps unsupported controls out of the product interface.

### Diagnostics

The runtime exposes cumulative capture and link counters separately from the sensing snapshot. `RuntimeFrontendController::diagnostics()` reads the totals, and `RuntimeDiagnosticsSampler` turns two reads into rates without requiring a separate timer:

```cpp
// once, at frontend startup
sampler_.reset(runtime_.diagnostics(), now_ms);

// whenever the existing periodic sensing callback runs
latest_ = sampler_.sample(runtime_.diagnostics(), now_ms);
```

`RuntimeDiagnosticsSample::csi_admitted_pps` is the detector input rate after temporal admission. `csi_accepted_pps` is the identity-accepted supply. Compare admitted PPS with `RuntimeConfig::csi_target_pps` together with `csi_occupancy_ratio`, callback-queue overflow, same-slot excess, missing-slot, stale, and out-of-order rates when a deployment underperforms. `RuntimeDiagnosticsSnapshot` exposes the cumulative callback-queue drop counter and its current occupancy and capacity; `RuntimeDiagnosticsSample::csi_pending_frame_drop_pps` derives the overflow rate. Occupancy is diagnostic telemetry and does not change the device send rate. [API.md](API.md#diagnostics) owns the corresponding wire field names, units, and optionality.

SDK transport adapters should pass parsed requests through `FrontendCommandEngine` and preserve the canonical distinction between requester-scoped query results and state changes published to active transports. [API.md](API.md#contract-principles) owns the message fields and cross-transport semantics; [ARCHITECTURE.md](ARCHITECTURE.md#shared-protocol-and-transport-services) owns command-engine and adapter placement.

The shipped ESP-IDF runtime always collects these counters and bounded performance windows. `RuntimeDiagnosticsSnapshot` also reports heap, CPU frequency, loop load and timing, detector timing, CSI provenance classification, and provenance rejection. Architecture owns how first-party frontends collect and cache those samples, while Protocol owns their transport representation.

### Frontend extensions

All command parameters use `EspectreCommandValidator`: the parser checks JSON syntax and request structure, then invokes the callback registered on the SDK or frontend route. The callback receives decoded `JsonObjectField` values, validates parameters, and fills the command before dispatch. It must not change device state. For frontend commands, `extension_parameters` initially contains the original request, including the MQTT envelope; the validator may replace it with normalized JSON built from the decoded fields. Native's OTA validator uses this to preserve the selected channel independently of envelope values and JSON escapes. The SDK validator checks parameter types and ranges, including thresholds in `0–1` and motion hit counts in `1–20`. ESPHome entity controls use the same parser before calling the command engine.

`FrontendCommandEngine::execute()` and frontend extension handlers require a successfully parsed command. They check capabilities and operational state without repeating parameter validation. Direct runtime and service APIs retain their own argument checks because integrators can call them without a protocol parser.

`EspectreCapabilityProfile::extension` accepts an optional `EspectreProtocolExtension` supplied by the frontend. Each route declares its HTTP method and path, resource or operation name, command name, asynchronous behavior, MQTT availability, raw-collection policy, and parameter validator. The extension also lists its event names. `validate_protocol_extension()` rejects invalid descriptors and collisions with SDK routes, command names, resources, or events.

Use the same immutable catalog for capability output, `DirectHttpServiceConfig::protocol_extension`, `direct_http_request_to_command()`, and `parse_espectre_command()`. Keep it alive while the adapters use it. Direct and MQTT then validate parameters through the same callback, and an unregistered extension command is rejected. The frontend enforces each route's MQTT availability, supplies the command implementation, and publishes any extension events through the normal transports; the SDK does not execute frontend operations itself. Advertise only extensions that the frontend has enabled.

Firmware updates belong to the frontend. The reusable `frontend/ota_service.h`, `frontend/ota_service_https.h`, and `frontend/ota_protocol.h` live in the repository, outside the SDK bundle. Native opts into them with `ESPECTRE_FRONTEND_OTA_SOURCES`; the provided HTTPS implementation targets ESPectre's release catalogs. Other products can implement that frontend interface for their own update mechanism. `frontend_ota_protocol()` supplies Native's additional OTA resources, operations, validation, and events without adding OTA types or policy to the SDK.

`RuntimeFrontendController::quiesce()` is a generic suspension operation: it disables telemetry and sensing services and stops active raw collection while retaining the configured backend. The caller restores its desired service and telemetry gates when resuming. It is useful before a firmware update or another temporary activity that needs CSI and sensing traffic to stop.

### Versioning

The frontend or integrator supplies its application version explicitly through `EspectreDeviceInfo::firmware_version` and discovery or provisioning configuration. Frontend OTA services also receive the application version directly from their owner. The SDK does not read the ESP-IDF application descriptor to determine it. First-party Native, Matter, and ESPHome frontends use `frontend_firmware_version()` from their shared frontend code; this helper is not part of the SDK.

`ESPECTRE_SDK_VERSION_STRING` identifies the SDK sources you compiled against. Use the component-wise `ESPECTRE_SDK_VERSION_AT_LEAST(major, minor, patch)` to guard code that needs a given release. `ESPECTRE_SDK_VERSION_NUMBER` retains the historical `MMmmpp` packing for compatibility and compact telemetry, but it is not an ordering contract because Semantic Versioning components are not limited to two digits.

ESPectre uses Semantic Versioning for the published C++ source API:

- Patch releases preserve source compatibility and documented lifecycle, validation, ownership, threading, capability, and error semantics. Detector coefficients and generated model weights may change when validation gates demonstrate a compatible quality fix; exact floating-point telemetry is not a compatibility guarantee.
- Minor releases may append fields, add callbacks with default implementations, and add types, functions, or overloads. Existing calls keep their meaning, closed enums do not gain values, and removals require a prior deprecation in a released minor version.
- Major releases may remove deprecated APIs or otherwise break source compatibility, with migration notes in `CHANGELOG.md`.
- Prerelease and rolling `preview` or `develop` bundles may change before the corresponding final release. The compatibility promise begins at the final numeric release.

The SDK is distributed and consumed as source. It does not promise a stable binary ABI: rebuild the SDK and integration together with the same C++ standard library and ESP-IDF toolchain. Construct public configuration and snapshot structs with their defaults, then assign named fields as shown in this guide; positional aggregate initialization is outside the compatibility contract so new fields can be appended safely.

Everything reachable from `espectre_sdk.h` belongs to the stable runtime surface. `espectre_core_sdk.h` is a separate, explicit opt-in for custom capture pipelines: its detector classes and documented public methods follow the same source-compatibility rules, while feature trackers, generated weights, and other headers reached only as implementation dependencies are not independent extension points.

The SDK reads its identity only from `runtime/espectre_sdk_version.h` and explicit compiler definitions. It does not inspect Git, source refs, environment variables, or the application version. Published SDK bundles stamp the release identity into that header and `idf_component.yml`; the CI packaging tools determine the identity before producing the bundle.

Integrators can stamp the same header or override all four macros together: `ESPECTRE_SDK_VERSION_STRING`, `ESPECTRE_SDK_VERSION_MAJOR`, `ESPECTRE_SDK_VERSION_MINOR`, and `ESPECTRE_SDK_VERSION_PATCH`. A complete compiler override takes precedence over the packaged values. The string and numeric components must describe the same SDK release, and definitions must be consistent across the SDK and its consumers.

Without a complete identity, the SDK uses `"0.0.0"` and zero for all numeric version components. Incomplete overrides also fall back to these values rather than mixing metadata from different sources or failing compilation. `espectre_sdk_version()` always returns a non-null string; `"0.0.0"` means the SDK version is unknown, and version guards for newer releases evaluate to false. A source checkout without stamped metadata therefore reports `"0.0.0"`, even if it has Git tags.

The SDK manifest exposes the packaged identity once as `version`; `release_tag` names the GitHub release that carries the assets and may differ for rolling channels. The generated API index uses `sdk_version` to identify the source revision used to build that reference. Official release tooling still requires and validates the published version. Rolling GitHub tags remain `snapshot` for `preview` and `snapshot-dev` for `develop`. SDK identity is separate from the application version supplied by the integrator and `ESPECTRE_PROTOCOL_VERSION`, which versions the wire format.

## Build integration

Both surfaces are distributed as source, but they compile different source sets.

- **Core-only CMake**: include `src/cpp/espectre_sources.cmake`, compile `ESPECTRE_CORE_SOURCES`, and add `ESPECTRE_SHARED_INCLUDE_DIRS`. No ESP-IDF runtime sources are required.
- **Full-runtime CMake / ESP-IDF**: compile `ESPECTRE_CORE_SOURCES` and `ESPECTRE_RUNTIME_ESP_IDF_SOURCES`, then add `ESPECTRE_RUNTIME_FRONTEND_SUPPORT_SOURCES` or the per-capability Direct HTTP, MQTT, and provisioning lists only when the integration uses them. Add `ESPECTRE_SHARED_INCLUDE_DIRS`; the frontend `CMakeLists.txt` files show working combinations.
- **Vendored ESP-IDF component**: drop `src/cpp/` into your project's `components/` directory and add `espectre` to your own component's `REQUIRES`. The sensing runtime is always built; the optional groups are opt-in under the "ESPectre SDK" menuconfig menu.
- **Toolchain**: C++17, ESP-IDF `>= 5.5` for the `runtime/esp_idf` services. Repository builds use ESP-IDF `5.5.5`.

Source-list integrations and vendored ESP-IDF components use the version header directly; no version resolution step is required. A complete core-only target is:

```cmake
set(ESPECTRE_CPP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/espectre/src/cpp")
include("${ESPECTRE_CPP_ROOT}/espectre_sources.cmake")
add_library(espectre_core STATIC ${ESPECTRE_CORE_SOURCES})
target_compile_features(espectre_core PUBLIC cxx_std_17)
target_include_directories(espectre_core PUBLIC ${ESPECTRE_SHARED_INCLUDE_DIRS})
```

Link the application target to `espectre_core` to inherit the includes and C++ standard. Adjust `ESPECTRE_CPP_ROOT` to the SDK's location. If overriding the packaged SDK identity, apply all four version macros with `target_compile_definitions(espectre_core PUBLIC ...)` so consumers inherit them. The same approach works for a full-runtime source-list target.

The shared component does not require ESP-IDF's `log` component. A product that registers an `esp_log` adapter declares that dependency in its own frontend or application component.

`ESPECTRE_SHARED_INCLUDE_DIRS` puts the SDK root on the include path, so both the flat form (`#include "runtime_interface.h"`) and the layer-prefixed form (`#include "runtime/runtime_interface.h"`) work. Prefer the prefixed form: the shared tree contains generic basenames such as `utils.h` and `filters.h`, and the prefix keeps them from colliding with headers of your own.

### Advanced task scheduling

Full-runtime ESP-IDF integrations expose ESPectre-owned FreeRTOS priorities under the `Advanced task scheduling` menu. These settings are compile-time policies, not runtime controls. Values range from `1` to `10`; higher-priority tasks preempt lower-priority work. Change them only with workload-specific validation because an unsuitable priority can starve sensing, Direct delivery, managed traffic, or system networking. ESP-IDF continues to own the internal Wi-Fi and lwIP task priorities.

The shared runtime defines these priorities:

| Kconfig option | Default | Owner |
|----------------|---------|-------|
| `CONFIG_ESPECTRE_DIRECT_HTTPD_TASK_PRIORITY` | `4` on classic ESP32 and ESP32-S2, otherwise `1` | Direct HTTP server |
| `CONFIG_ESPECTRE_DIRECT_WORKER_TASK_PRIORITY` | `2` | Direct control responses and SSE delivery |
| `CONFIG_ESPECTRE_RAW_WORKER_TASK_PRIORITY` | `3` | Raw CSI HTTP delivery |
| `CONFIG_ESPECTRE_TRAFFIC_TASK_PRIORITY` | `1` | Managed PING or DNS traffic |

The Native frontend separately defines `CONFIG_ESPECTRE_NATIVE_LOOP_TASK_PRIORITY`, with a default of `5`, for its frontend and sensing loop. A custom integration owns the task that calls `RuntimeFrontendController::loop()` and must select that task's priority as part of its own scheduling policy.

Classic ESP32 and ESP32-S2 builds default the Direct HTTP server priority to `4`, keeping control requests above best-effort managed traffic while CSI is active. Other supported targets default it to `1`. This is an explicit validated target policy, not an inference from scheduler topology; custom integrations can still override it after workload-specific validation.

See [TUNING.md](TUNING.md) for how evaluation cadence, tick alignment, and hit filtering determine expected publish delay.

### Optional capability groups

| Menuconfig option | `espectre_sources.cmake` variable | Adds | Additional source-list requirements |
|-------------------|-----------------------------------|------|------------------------------------|
| `ESPECTRE_SDK_ENABLE_FRONTEND_SUPPORT` | `ESPECTRE_RUNTIME_FRONTEND_SUPPORT_SOURCES` | Shared bootstrap, control, sysinfo, and MQTT payload helpers | None beyond the base runtime |
| `ESPECTRE_SDK_ENABLE_MQTT` | `ESPECTRE_RUNTIME_ESP_IDF_MQTT_SOURCES` | `EspIdfMqttTransport` over `esp-mqtt` | `mqtt` |
| `ESPECTRE_SDK_ENABLE_PROVISIONING` | `ESPECTRE_RUNTIME_ESP_IDF_PROVISIONING_SOURCES` | Device config store and Wi-Fi provisioning | `improv` |
| `ESPECTRE_SDK_ENABLE_DIRECT` | `ESPECTRE_RUNTIME_ESP_IDF_DIRECT_SOURCES` | Direct HTTP, SSE, raw CSI streaming, peer discovery, and mDNS | `esp_http_server` and `mdns` |

Each group is off by default, so a minimal integration does not link transport code it never calls. ESP-IDF resolves component requirements before menuconfig, so the vendored component declares every optional stack dependency up front; the source switches still control what reaches the firmware image. Its manifest resolves the pinned Improv revision and constrained Espressif mDNS component so provisioning and Direct builds work when selected; source-list integrations must declare those dependencies themselves. Implementing `IMqttTransport` or `IDirectHttpService` yourself needs no group at all: the interfaces are header-only. `DirectHttpServiceConfig` keeps its generic Origin allowlist empty; `for_first_party_portals()` explicitly selects the official production and validation portals.

## Published SDK channels

ESPectre publishes source-first SDK bundles alongside the firmware release channels:

| Channel | Source | Intended use |
|---------|--------|--------------|
| `release` | latest tagged semver GitHub Release and `https://espectre.dev/artifacts/sdk/release/` | Final numeric versions are production candidates; prerelease tags are published explicitly as evaluation builds |
| `preview` | rolling `snapshot` GitHub prerelease and `https://espectre.dev/artifacts/sdk/preview/` | Validate `main` before the next release |
| `develop` | rolling `snapshot-dev` GitHub prerelease and `https://espectre.dev/artifacts/sdk/develop/` | Pre-main validation from `develop` |

Rolling releases publish `sdk-manifest-preview.json` and `sdk-manifest-develop.json`; the manifest filename follows the channel, while `release_tag` retains the GitHub tag shown above. Tagged releases publish `sdk-manifest-<release-tag>.json`.

Each SDK bundle includes:

- `docs/SDK.md`
- `src/cpp/espectre_sdk.h`
- `src/cpp/espectre_core_sdk.h`
- `src/cpp/core/`
- `src/cpp/runtime/`
- `src/cpp/runtime/esp_idf/espectre_config/`
- `src/cpp/espectre_sources.cmake`
- `src/cpp/CMakeLists.txt`
- `src/cpp/idf_component.yml`
- `src/cpp/Kconfig.projbuild`
- `src/cpp/Doxyfile`
- generated `src/cpp/core/ml_weights.h`
- `LICENSE`, `LICENSING.md`, and `THIRD_PARTY_NOTICES.md`

The published bundle is a versioned C++ source SDK with stamped packaging metadata, ready to vendor or unpack into your firmware tree. ESPectre is compiled together with the product firmware; the bundle does not include chip-specific precompiled libraries or promise binary ABI compatibility. In the bundled copy of this guide, repository-relative links point to GitHub URLs pinned to the commit or release tag used for that package. The `.tar.gz` and `.zip` archives are generated deterministically from the source commit timestamp, and the SDK manifest records a SHA-256 digest for each archive so consumers can verify downloaded bytes.

## Validation assets

- [README.md](performance/README.md) publishes the current benchmark and validation metrics per chip and detector.
- `test/cpp/` builds the full sensing stack on a host machine, including integration suites that replay real CSI recordings through the production pipeline; `test/python/` mirrors the algorithm behavior for parity checks.
- `test/cpp/suites/runtime/test_sdk_surface.cpp` compiles against `espectre_sdk.h` alone, so it fails if the facade stops reaching the documented surface or a published default drifts out of its range.
- `test/python/contracts/test_sdk_surface_invariants.py` checks the surface against its own documentation: every facade header appears in the API reference and this guide's header map, and no type reachable from the facade is left as an unresolved forward declaration.
- The dataset collection and quality workflow is documented in [ML_DATA_COLLECTION.md](ML_DATA_COLLECTION.md).

## Generated API reference

The headers carry Doxygen-compatible documentation. Generate the portal-ready reference for the supported surface from the repository root with:

```bash
python3 .github/scripts/generate_sdk_api.py
```

The generator requires Doxygen 1.17.0, stamps `PROJECT_NUMBER` from the same `git describe` identity used by SDK bundles, generates XML in an isolated build directory, and renders it through a pinned m.css revision. Only an index manifest and HTML fragments are written to `docs/web/artifacts/sdk/api/`; the existing `/sdk/api/` SPA owns navigation, search, styling, header, and footer. The generated output is not committed, so it never drifts from the headers.

An unpacked SDK bundle ships this guide and `src/cpp/Doxyfile` rewritten to write `output/xml/` and stamped with that bundle's version. Running `doxygen src/cpp/Doxyfile` from the bundle root therefore produces a tool-neutral XML reference without shipping the website or a second HTML shell. The browsable reference is integrated at `https://espectre.dev/sdk/api/` and rebuilt from source on every deploy.

## Licensing

ESPectre is dual-licensed: GPLv3 for open-source use, with commercial licenses available for embedding into proprietary firmware. See [LICENSING.md](../LICENSING.md).
