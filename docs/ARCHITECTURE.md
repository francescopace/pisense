# Architecture Guide

This reference is for contributors and firmware integrators who need the current code layout, dependency boundaries, and runtime contracts. It is not an installation guide; use [SETUP.md](SETUP.md) for deployment and [SDK.md](SDK.md) for the supported SDK path.

In this document, **core** means portable detector logic, **runtime** means the execution and event layer around it, and **frontend** means an ecosystem-specific adapter such as ESPHome or Matter. Historical rationale lives in the [ADR index](adr/README.md); this page describes only the current structure.

## Current Source Layout

```text
src/cpp/
├── core/
├── runtime/
│   └── esp_idf/
└── frontend/
    ├── esphome/
    ├── native/
    └── matter/
```

```text
Frontend -> Runtime contracts -> ESP-IDF runtime services -> Core
```

The contracts and detector logic compile on a host without ESP-IDF. Frontends select a backend through `RuntimeFrontendController` and do not call `core` directly.

## Layer Responsibilities

### `src/cpp/core/`

`core` contains reusable sensing logic and domain primitives:

- `LightweightDetector` and `HighAccuracyDetector`
- `TemporalCsiSampler`, which admits at most one packet per configured slot
- feature extraction and detector math
- filters and helper utilities
- exported ML artifacts and related constants

Rule of thumb: code in `core` should stay free of frontend-specific concerns such as ESPHome entities, Matter clusters, HTTP transport details, or MQTT topic handling.

Shared code also stays independent of a platform logger. `core/espectre_log.h` defines a portable sink contract used by `core` and `runtime`; the active frontend registers the backend before runtime setup. With no sink, shared logging is silent. ESPHome maps the sink to its logger, while Native, Matter, and the Micro-ESPectre native bindings map it to ESP-IDF without making `esp_log` a dependency of the shared SDK components. Micro-ESPectre's focused traffic component depends on its core component so both use the same registered sink.

### `src/cpp/runtime/`

`runtime` owns the execution environment around the shared detectors:

- CSI ingestion, normalization, and temporal admission before detector input
- AGC-active sensing path
- startup calibration orchestration
- traffic generation or packet ingress hooks
- runtime snapshots, capabilities, and events
- common runtime-facing configuration validation

The shared runtime layer owns the frontend-facing contract. The ESP-IDF implementation under `src/cpp/runtime/esp_idf/` runs sensing and capability-gated raw collection for every maintained C++ frontend.

Shared runtime services also live here, including:

- `RuntimeFrontendController`
- standalone Wi-Fi helpers for non-ESPHome firmware
- shared diagnostics helpers
- ESPectre Protocol model and shared Direct HTTP/MQTT transport support
- NVS-backed provisioning helpers reused by ESP-IDF frontends

### Shared Wi-Fi and CSI Lifecycle

`WiFiLifecycleManager` owns the CSI-specific ESP-IDF radio policy for every frontend. It applies the protocol and HT20 bandwidth policy synchronously on `WIFI_EVENT_STA_START`, before the first association, then completes the CSI prerequisites when `IP_EVENT_STA_GOT_IP` is drained from the runtime loop. ESPHome, Native, and Matter must not apply these radio settings in their frontend code.

Association changes are also drained by `WiFiLifecycleManager` from `WIFI_EVENT_STA_CONNECTED`. A reassociation while the session is active, or a roaming disconnect followed by association with valid retained IPv4 state, reuses the shared disconnect/connect callbacks to stop traffic, refresh the CSI receive path, and restart sensing. This also refreshes the `wifi_raw` BSSID target when IP and channel are unchanged, without requiring a new `GOT_IP` event. Initial association and ordinary reconnects still wait for `GOT_IP`; a later duplicate IP notification does not restart the session again.

The frontend or SDK integrator explicitly selects `2g`, `5g`, or `auto`; `2g` is the validated default, while `5g` and `auto` are available only on dual-band targets. The lifecycle applies that band mode first and pins 20 MHz bandwidth on the selected band or bands. After association, the runtime selects `lltf20` for internal `wifi_raw` or on ESP32 and ESP32-S2, `vht20` on a VHT-capable 5 GHz link, and `ht20` otherwise; HE capture remains disabled. The shared C++ `lltf20` capture profile enables ACK dumping and admits valid 802.11 ACKs addressed to the local station, independently of the selected traffic generator; IP traffic retains its configured provenance filter. Other capture profiles reject ACKs. Fixed-band policies use the single-band ESP-IDF APIs, while AUTO uses the per-band APIs. See [`2026-07-23-adopt-classifier-first-ht20-sensing-contract.md`](adr/2026-07-23-adopt-classifier-first-ht20-sensing-contract.md).

The shared C++ and Python normalizers support the compact 106-byte LLTF layout (53 signed 8-bit I/Q pairs) in addition to full-width LLTF. Legacy LLTF admission is required before accepting that compact layout for sensing. C5 hardware captures show centered ordering (`-26..+26`), with DC at pair 26. Normalization pads six bins on the left and five on the right, placing DC at bin 32 in the existing 128-byte payload and leaving absent bins zero-filled. The detector retains its separate LLTF edge-tone imputation. This mapping does not accept packed 12-bit samples; C5 LLTF capture selects 8-bit components. The centered ordering was confirmed on 2026-09-07 with ESP-IDF 5.5.5 on a C5 at 2.4 GHz: all eight pre-normalization samples (five ACKs and three legacy data frames) had their only zero I/Q pair at index 26. This differs from the ordering in the Espressif documentation; other chip and driver combinations still require hardware confirmation.

Supported first-party firmware uses an associated Wi-Fi station and does not enable promiscuous mode. Standalone ESP-IDF startup explicitly keeps promiscuous mode disabled, and the shared CSI pipeline filters frames against the local device identity where the relevant metadata is available. This is an intentional responsible-use boundary: a protected network requires valid credentials, which raises the barrier against passive collection by an unaffiliated device. It is not an authorization mechanism or proof of consent; open networks need no password, credentials can be misused, and downstream open-source builds can change the radio policy.

The `GOT_IP` payload is also the source of truth for the local address, netmask, and gateway used during service startup. The runtime passes that gateway directly to the internal traffic generator instead of querying the network interface again. Disconnect processing clears the shared ready state, so the same sequence is repeated after a genuine reconnect.

`runtime/csi_traffic_service` owns the platform-neutral traffic policy: configuration projection, internal-versus-external selection, lifecycle, callbacks, and counters. It depends only on the `ICsiTrafficGenerator` and `ICsiTrafficIngress` boundaries. The ESP-IDF layer implements those boundaries with `TrafficGeneratorManager` and `UDPListener`; the latter delegates socket creation, binding, multicast membership, and datagram reads to `UdpDatagramSocketEspIdf`. This keeps lwIP and FreeRTOS below the shared runtime contract and lets host tests inject deterministic in-memory adapters.

The CSI callback classifies packet provenance against the configured traffic mode before data reaches sensing or raw collection. Accepted sensing frames pass through `TemporalCsiSampler`; accepted raw frames bypass temporal sampling and enter a preallocated SPSC ring drained by a dedicated task-notified HTTP worker. The protocol specification owns the external marker, raw framing, session semantics, and observable drop accounting.

### Shared Protocol and Transport Services

`FrontendCommandEngine` is the C++ command owner below the frontend adapters. Native MQTT, Native Direct, the shared Direct bridge, and ESPHome entities construct the same typed request and receive the same structured result and change set; Matter inherits the same path through the shared bridge. Commands execute serially on the existing frontend task. Queries return only through the requesting adapter, while accepted mutations publish the affected state families to every active adapter. MQTT and each Direct client keep independent outbound queues because transport backpressure is independent of command semantics.

`EspectreCapabilityProfile` is the single C++ catalog for executable Direct methods, published event families, and visible configuration sections. Serialization and command enforcement consume the same profile.

The shared Direct service owns HTTP request lifetime, SSE delivery, deferred responses, and the owner-bound raw CSI session used by ESPectre. The ESP-IDF implementation assigns an opaque monotonically increasing token to each live connection, removes inbound work by token rather than file descriptor, and completes deferred work only while that token still identifies the originating client. The default interface implementation reports deferred delivery as unsupported, preserving source compatibility for transports that implement only synchronous requests.

Peer-assisted discovery keeps orchestration out of `core`. `runtime/peer_discovery` owns bounded validation, deterministic deduplication, sorting, and serialization; `runtime/esp_idf/peer_discovery_service_esp_idf` owns the asynchronous DNS-SD browse; and `runtime/esp_idf/mdns_bootstrap_responder` owns the shared IPv4 bootstrap response through the existing Espressif responder. Frontend shutdown and Wi-Fi reconfiguration release pending discovery work without retaining a peer inventory. [`DISCOVERY.md`](DISCOVERY.md#dns-sd-and-mdns) owns the advertisement, bootstrap wire behavior, request and result schemas, limits, and compatibility rules.

### `src/cpp/frontend/`

`frontend` maps the runtime into a concrete ecosystem or firmware surface.

Current frontends:

- `esphome`: Home Assistant-facing external component, Direct runtime controls, and packaging root
- `native`: standalone Direct HTTP/MQTT firmware surface
- `matter`: Matter-facing adapter with a separate Direct tuning plane

Rule of thumb: frontend-specific schemas, transport bindings, and ecosystem integration belong here, not in `core`.

Frontends also own logging integration. A sink must be registered before shared setup begins and remain unchanged until the runtime has shut down. Sink callbacks can run from the runtime owner task, ESP-IDF service tasks, or CSI capture paths, so they must be thread-safe, bounded, non-blocking, and non-reentrant.

## Frontend Notes

### ESPHome

`src/cpp/frontend/esphome/` maps the shared runtime into ESPHome entities, YAML/config-codegen, external-component packaging, and the common Direct HTTP bridge. Direct mutations republish matching entity state rather than creating a second configuration owner.

For the ESPHome workflow, see [`README.md` (esphome)](../src/cpp/frontend/esphome/README.md).

### Native

`src/cpp/frontend/native/` exposes the runtime through Improv Serial provisioning, local Direct HTTP, and optional MQTT. `NativeFrontend` composes dedicated Direct, MQTT, and Home Assistant adapters around the shared runtime and `FrontendCommandEngine`; transport lifecycle, queues, discovery, subscriptions, and framing remain in their owning adapters. It reuses the shared ESP-IDF services for staged Wi-Fi configuration, device configuration, mDNS, transport-independent commands, diagnostics, and raw CSI. Its OTA service and protocol extension live in the shared `src/cpp/frontend/` code. Native alone delegates credential reset to its provisioning owner and adds MQTT configuration and OTA to the common Direct surface.

For the native workflow and protocol surface, see:

- [`README.md` (native)](../src/cpp/frontend/native/README.md)
- [`API.md`](API.md)

### Matter

`src/cpp/frontend/matter/` maps occupancy into Matter without pulling Matter-specific concerns into the shared detector or runtime layers. Detector configuration is not represented by the standard occupancy clusters, so the frontend also exposes the shared Direct HTTP bridge as its local tuning plane.

For the Matter workflow, see [`README.md` (matter)](../src/cpp/frontend/matter/README.md).

## Runtime Contract

The shared runtime contract is the interface used by frontends.

Current key pieces:

- `runtime_interface.h`
- `runtime_snapshot.h`
- `runtime_events.h`
- `runtime_capabilities.h`

Frontend-facing operations include:

- `setup()`
- `shutdown()`
- `loop()`
- `set_threshold_runtime()`
- `set_detection_algorithm_runtime()`
- `trigger_recalibration()`
- `get_snapshot()`
- `get_capabilities()`

Normalized runtime events include:

- motion-state changes
- threshold changes, including Lightweight settled-level recovery
- detector changes
- calibration start and finish
- periodic status updates
- runtime faults

Frontends should use this surface instead of reaching directly into low-level Wi-Fi or CSI pipeline services.

Runtime detector selection is capability-gated. ESPHome and Native enable the shared ESP-IDF detector store, which persists `lightweight` or `high_accuracy` in NVS and restores it at boot. Matter enables runtime detector selection through Direct HTTP because its standard clusters do not expose detector configuration.

### Runtime Performance Diagnostics

C++ runtime implementations use `RuntimePerformanceDiagnostics` to aggregate runtime-loop load and timing plus sampled detector evaluation timing in bounded 10-second windows. `RuntimeDiagnosticsSnapshot` combines the latest complete window with current, minimum, and largest-block heap values and configured CPU frequency. The runtime also owns the single one-second `RuntimeDiagnosticsSample` derived from cumulative counters; Native, ESPHome, Matter, Direct, and ecosystem adapters read that shared sample instead of maintaining frontend-specific samplers. ESPectre frontends expose these production fields through Direct `diagnostics`; collection is unconditional and does not emit a periodic debug log.

`runtime_load_percent` measures wall time spent inside the ESPectre runtime loop, not whole-system CPU utilization. Wi-Fi callbacks only normalize and enqueue CSI; detector processing, inference, state transitions, and frontend callback delivery run in the owning loop task. MQTT, Direct HTTP, and OTA stacks may still perform transport work on private tasks, but their application events are drained by the frontend loop. Detector timing is sampled on an evaluation tick after approximately 1,000 detector packets. For High Accuracy, it covers ML feature extraction, inference, and state update.

The public field names, units, optionality, and transport objects are part of the additive [`diagnostics` contract](API.md#diagnostics). Architecture owns how the samples are collected and cached, not their wire schema.

## ESPectre Protocol In The Architecture

ESPectre Protocol is the shared device-facing message model used by the standalone ESP-IDF frontends and related tools. [`API.md`](API.md) owns resources, operations, payloads, Direct HTTP and MQTT mappings, public limits, and version semantics. [`DISCOVERY.md`](DISCOVERY.md) owns DNS-SD, mDNS, browser bootstrap, and peer-result contracts.

For every maintained C++ frontend, protocol adapters sit at the boundary between the frontend and shared runtime layers.

## Packaging Note For ESPHome

ESPHome still expects a component-shaped entry point under the external components root. For that reason, `src/cpp/frontend/esphome/components/espectre/` acts as the ESPHome packaging root even though the shared sources live under `src/cpp/core/` and `src/cpp/runtime/`.

## Related References

- Deployment and frontend selection: [SETUP.md](SETUP.md)
- Supported SDK surface: [SDK.md](SDK.md)
- Detector behavior and tuning: [ALGORITHMS.md](ALGORITHMS.md) and [TUNING.md](TUNING.md)
- Measured detector results: [docs/performance](performance/README.md)
- Frontend operation: the relevant README under `src/cpp/frontend/`
