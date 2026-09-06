# ESPectre Native Frontend

Native is the standalone ESP-IDF firmware for browser-based local setup, sensing over Direct HTTP, optional MQTT integration, Home Assistant MQTT Discovery, and HTTPS OTA. The shared message model is documented in [`API.md`](../../../../docs/API.md).

## Getting Started

The normal browser workflow is:

1. Open [Flash](https://espectre.dev/tools/flash/) in a supported Chromium browser and install the Native image for the detected chip.
2. Complete the standard Improv Serial prompt to provision Wi-Fi over USB.
3. Open Device settings with the returned device URL, or enter the private IP, device name, full 16-character device ID, or last 6 ID characters.
4. Use Direct HTTP to inspect status, reconcile or pin the associated BSSID, edit the device label, and add optional MQTT settings.
5. Open Monitor for broker-free sensing over Direct HTTP. Use MQTT for Home Assistant, automation, remote brokers, and other broker-based clients.

The `release`, `preview`, and `develop` channels publish a full-flash image and an application-only OTA image for ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C5, and ESP32-C6.

### Local ESP-IDF Workflow

Complete the shared [`Local Build Prerequisites`](../../../../docs/SETUP.md#local-build-prerequisites), then use the repository CLI:

```bash
./espectre native build --chip s2 --ota-channel develop --clean
./espectre native flash --chip s2 --port /dev/cu.usbmodemXXXX
./espectre monitor --port /dev/cu.usbmodemXXXX
```

Flashing and serial monitoring require local tooling. `--ota-channel` selects the default release channel used when an OTA request omits one.

The primary console transport follows the target's USB capability. UART and USB Serial/JTAG use ESP-IDF directly; maintained USB-OTG configurations that cannot rely on the ROM CDC path use the shared TinyUSB CDC console. Improv Serial keeps the same protocol and CLI workflow on every transport.

## Direct HTTP

Native exposes resource requests below `http://<device>:62587/espectre/v1`, plus `GET /events` and `GET /csi`, after Wi-Fi obtains an address. The production portal and `https://test.espectre.dev` validation origins are allowed by default; optional loopback development origins are controlled by Kconfig and remain disabled in published firmware. Resource mutations use JSON, and events use SSE read through streaming `fetch` so the browser can request local-network access explicitly.

Direct mode provides:

- capability negotiation and separate device, health, sensing, Wi-Fi, MQTT, OTA, and diagnostics resources
- Wi-Fi updates with optional BSSID and channel hints
- device-label and optional MQTT add, change, or clear operations
- sensing enable or pause through `PATCH /sensing`, recalibration, detector selection, thresholds, hit counts, and traffic controls
- per-evaluation motion plus resource and fault events
- supported OTA status and control operations

The endpoints never return stored Wi-Fi or MQTT passwords. They cap request and response size, total request rate, mutation rate, queued messages, and concurrent SSE subscribers. Telemetry may replace an older queued sample, while state transitions are preserved. MQTT uses its own 16-message frontend queue and bounded ESP-IDF outbox.

The per-evaluation motion callback runs only while MQTT is connected or a Direct SSE client is present. Runtime callbacks stage numeric sensing state; serialization and transport work run after detector evaluation returns.

Raw CSI packet buffers and the streaming worker are allocated only for an active collection session; stopping collection releases them after capture and sending are quiescent.

Native advertises the shared automatic CSI HTTP surface. Opening `GET /csi` starts the exclusive collection, and closing the response restores sensing without changing persisted traffic configuration. [`API.md`](../../../../docs/API.md#csi-collection) owns framing, queue limits, and recovery behavior; [`CLI.md`](../../../../docs/CLI.md#collect) owns the `./espectre collect` workflow.

The device advertises `_espectre._tcp` through mDNS with a stable `espectre-<device_id>.local` hostname. Run `./espectre devices --frontend native` to enumerate Native endpoints on an mDNS-visible LAN. [`DISCOVERY.md`](../../../../docs/DISCOVERY.md#dns-sd-and-mdns) owns the SRV, TXT, and peer-discovery contract; [`SETUP.md`](../../../../docs/SETUP.md#direct-http-connectivity) owns browser permissions, supported connection identifiers, and recovery when discovery fails.

## Wi-Fi Provisioning and Recovery

Standard Improv Serial remains available through the primary serial console. It owns the Wi-Fi SSID and password and returns `https://espectre.dev/tools/device-settings/?target=<device-ip>`; Device settings uses the target to prefill its Direct connection field. The same parameter also accepts a device name or ID when a browser link is shared. BSSID selection, Wi-Fi removal, device-label, MQTT, sensing, and OTA operations belong to Direct HTTP. Direct reports the current SSID and active band as read-only values but does not expose the Wi-Fi password or band selection.

Device settings can scan asynchronously for access points that advertise the provisioned SSID. The station remains associated and Direct HTTP stays active during the scan, but off-channel radio work can briefly pause sensing and network traffic. Each protocol result contains the BSSID, channel, and RSSI; Device settings displays the BSSID and signal strength, while retaining the channel only as an internal association hint. Choosing automatic selection clears both the BSSID pin and hint.

The Direct `clear_wifi_config` action removes the provisioned SSID and password, disconnects the station, and returns the device to Improv Serial provisioning. Device settings asks for confirmation before sending it because the active Direct session normally closes before a response can be observed.

BSSID changes are staged, and Direct acknowledges the mutation with the current BSSID before Native changes the station association. When the requested BSSID is already active, the default behavior only persists the pin; `force=true` performs the reassociation anyway. Native saves the candidate to non-volatile storage, suspends sensing, disables CSI, detaches its callback, disconnects, applies the station configuration, and reconnects with promiscuous mode disabled and the selected access point. Native commits the candidate only after association, BSSID verification, and address acquisition. A failure or timeout clears the candidate and reconnects once with the last-known-good settings. The verified reconnect runs a non-blocking scan to refresh the CSI receive path, registers a fresh CSI callback, resets capture-format state, and starts a fresh calibration. Direct and MQTT clients reconnect after the device address becomes reachable again, and a power loss during the transaction resumes the pending candidate on the next boot.

Holding BOOT for `ESPECTRE_RECOVERY_BUTTON_HOLD_MS` clears saved Wi-Fi configuration and returns the device to Improv Serial provisioning. The default hold is 3 seconds. The default active-low GPIO is GPIO0 on ESP32, ESP32-S2, and ESP32-S3, GPIO9 on ESP32-C3 and ESP32-C6, and GPIO28 on ESP32-C5. Override or disable the input for boards that route BOOT differently.

Frontend-owned defaults in [`Kconfig.projbuild`](espectre/Kconfig.projbuild) are useful for reproducible lab images. Runtime provisioning stored in NVS takes precedence.

| Option | Purpose |
| --- | --- |
| `ESPECTRE_WIFI_SSID` | Initial Wi-Fi SSID |
| `ESPECTRE_WIFI_PASSWORD` | Initial Wi-Fi password |
| `ESPECTRE_WIFI_BSSID` | Optional AP-radio pin |
| `ESPECTRE_WIFI_BAND_2G`, `ESPECTRE_WIFI_BAND_5G`, `ESPECTRE_WIFI_BAND_AUTO` | Build-time band policy |
| `ESPECTRE_WIFI_CHANNEL` | Optional channel hint (`0` scans normally) |
| `ESPECTRE_RECOVERY_BUTTON_*` | Physical recovery GPIO and hold policy |

ESP32-C5 defaults to `auto` and can be pinned to `2g` or `5g`; the other supported Native targets use `2g`. The runtime selects `vht20` after a 5 GHz C5 association and `ht20` otherwise.

## Optional MQTT and Home Assistant

MQTT is disabled until configured. Wi-Fi alone is sufficient for Native to start Direct HTTP and sense. Adding, losing, slowing, or clearing MQTT does not disable Direct mode. Receive buffers are allocated when MQTT starts and released when it shuts down. Home Assistant discovery messages are generated incrementally as the outbound queue accepts them.

Device settings requires an explicit scheme, host, and port. Select `mqtt`, a bare local hostname such as `homeassistant.local`, and port `1883` for a typical trusted-LAN Home Assistant or Mosquitto broker. Select `mqtts` and the broker's TLS port, commonly `8883`, for a public-CA-secured broker; Native verifies both the certificate chain and broker hostname. Do not put `mqtt://`, `mqtts://`, credentials, a port, or a path in the host field. WebSocket MQTT and private certificate authorities are not supported in this configuration version.

An older saved endpoint without an explicit scheme is retained for recovery but remains disconnected and reports `configured: false`. Open Device settings over Direct HTTP and save the endpoint again with the intended scheme. Native never guesses whether an existing broker should use plaintext or TLS.

When configured, MQTT runs concurrently with Direct HTTP and provides the canonical ESPectre topic surface, Home Assistant MQTT Discovery, retained availability, and integration with broker-based clients. Both transports invoke the same command engine: a query answers only its requester, while a mutation fans out the corresponding authoritative state event. Their outbound queues remain separate, so broker backpressure cannot delay Direct sensing.

The browser Monitor uses Direct HTTP and does not connect to MQTT. Device-to-broker MQTT configuration is independent of the browser connection.

The Native `diagnostics` request groups production metrics into:

- system state: uptime, current, minimum, and largest-block heap, CPU frequency, and frontend-task stack high-water;
- sensing and performance: bounded loop-load and detector-timing windows, plus cached traffic, CSI, and Wi-Fi diagnostics; and
- transports: Direct and MQTT diagnostics, including fixed client and queue budgets, the MQTT outbox budget, current occupancy, cumulative drops, and send failures.

Performance aggregation is unconditional production runtime state; it does not require a build option or periodic debug logger.

Home Assistant discovery is enabled in the published defaults and can be disabled with `CONFIG_ESPECTRE_HA_DISCOVERY_ENABLED`. It publishes the same primary sensing and tuning entities used by the ESPHome frontend:

| Entity | Behavior |
| --- | --- |
| Motion Detected | Filtered movement-state edges |
| Movement Score | Each detector evaluation |
| Threshold and hit counts | Retained state and writable control |
| Detection Profile | `lightweight` or `high_accuracy` |
| CSI Traffic Ownership and Source | Runtime traffic controls |
| Recalibrate | Configuration button that starts recalibration |
| Calibration Active | Diagnostic binary sensor that reports the authoritative runtime state |
| CSI and Wi-Fi diagnostics | Published on demand after Refresh Diagnostics |

Canonical topics under `espectre/v1/devices/{device_id}/...` remain available to standalone clients. See [`API.md`](../../../../docs/API.md) for the exact topic and payload contract.

## Detection and Traffic

Native selects its build-time sensing defaults through the shared ESP-IDF `sdkconfig` menu and exposes supported runtime controls through Direct HTTP, optional MQTT, and Home Assistant MQTT Discovery. The accepted detector and traffic selections persist across reboot. [`SETUP.md`](../../../../docs/SETUP.md#shared-sensing-options) owns option names, defaults, traffic modes, and external-source configuration; [`TUNING.md`](../../../../docs/TUNING.md) owns profile selection, calibration, packet health, placement, and detector troubleshooting.

## OTA

Native uses the reusable HTTPS OTA service in the shared `src/cpp/frontend/` code. `frontend_ota_protocol()` registers the additional OTA routes, parameter validation, and event in the SDK's generic extension catalog; OTA policy and implementation remain outside the SDK. Native compiles `ESPECTRE_FRONTEND_OTA_SOURCES`, and other frontend integrations can opt into the same source group. Before downloading, Native calls the generic runtime `quiesce()` operation and stops its transports. Direct exposes the `ota` resource plus `POST /ota/checks` and `POST /ota/updates`; MQTT uses `check_ota` and `start_ota` when its advertised capabilities include OTA.

- `release`, `preview`, and `develop` select the corresponding publication channel.
- Clients cannot override the manifest host, image URL, chip, or target version.
- The HTTPS service downloads only a strictly newer release, prerelease, or rolling `git describe` identity; stale manifests cannot trigger a downgrade.
- A successful update schedules a reboot into the new OTA slot.
- A failed update restores Direct HTTP and MQTT; sensing resumes only if it was enabled before the update.
- Reconnection republishes device identity, online status, and OTA state.
- USB reflashing with the full factory image remains the recovery path when OTA cannot complete.

OTA reads the shared `firmware-manifest-<channel>.json` catalog: `release` uses the latest stable GitHub release, `preview` uses `snapshot`, and `develop` uses `snapshot-dev`. The service validates the catalog schema and channel, then selects the unique Native artifact matching the device chip and `build_type: "ota"`. Its URL points to the application-only `-ota.bin` image. Missing or ambiguous matches fail the update check.

## Troubleshooting

Use [`SETUP.md`](../../../../docs/SETUP.md#direct-http-connectivity) for Direct HTTP, browser permission, address, and discovery failures. Use [`TUNING.md`](../../../../docs/TUNING.md#troubleshooting) for missing motion, false positives, calibration, packet health, placement, or unstable detection.

### The device does not join Wi-Fi

Reconnect over Improv Serial and provision the network again. If a BSSID pin is stale, configure the SSID without a pin. When remote configuration is unreachable, hold BOOT for the configured recovery interval and repeat Improv Serial provisioning.

### OTA failed or an older release is required

Reflash the full factory image over USB when OTA cannot complete. Downgrades are not a general compatibility promise: use an older factory image only when that release's migration notes explicitly allow it, and erase flash when its persisted configuration schema is incompatible. A full reflash and Improv Serial provisioning do not depend on MQTT, a remembered endpoint, or the original browser profile.

### MQTT clients do not receive data

Confirm that the endpoint reports `configured: true`, that the broker hostname resolves from the ESP32, that the selected scheme and port match the broker listener, that the credentials are valid, and that the intended broker client subscribes to the canonical topics. For `mqtts`, the certificate must chain to the ESP-IDF public bundle and identify the configured host. The browser Monitor uses Direct HTTP and should remain operational while broker issues are diagnosed.

## Implementation Map

- [`app/`](app/): standalone ESP-IDF entry point, Wi-Fi lifecycle, Improv Serial, mDNS, Direct service, and recovery wiring
- [`espectre/native_frontend.cpp`](espectre/native_frontend.cpp): lightweight lifecycle orchestrator, runtime listener, event fan-out, and OTA coordination
- [`espectre/native_command_bindings.cpp`](espectre/native_command_bindings.cpp): Native persistence, provisioning, and state-publication bindings around the shared `FrontendCommandEngine`
- [`espectre/native_direct_frontend.cpp`](espectre/native_direct_frontend.cpp): Direct HTTP lifecycle, local configuration reads, diagnostics, peer discovery, and raw CSI sessions
- [`espectre/native_mqtt_frontend.cpp`](espectre/native_mqtt_frontend.cpp): canonical MQTT command, state, telemetry, capability, and OTA topics
- [`espectre/home_assistant_mqtt_frontend.cpp`](espectre/home_assistant_mqtt_frontend.cpp): Home Assistant discovery, commands, diagnostics, and deferred state snapshots
- [`../../runtime/direct_http_protocol.cpp`](../../runtime/direct_http_protocol.cpp): canonical request parsing and Direct/MQTT protocol mapping
- [`../../runtime/esp_idf/direct_http_service_esp_idf.cpp`](../../runtime/esp_idf/direct_http_service_esp_idf.cpp): bounded ESP-IDF HTTP, SSE, and binary streaming server
- [`../../runtime/esp_idf/mdns_discovery_service.cpp`](../../runtime/esp_idf/mdns_discovery_service.cpp): shared Direct discovery lifecycle
- [`../../runtime/esp_idf/improv_serial_service.cpp`](../../runtime/esp_idf/improv_serial_service.cpp): standard Improv Serial adapter
- [`../../runtime/esp_idf/wifi_provisioning_service.cpp`](../../runtime/esp_idf/wifi_provisioning_service.cpp): staged Wi-Fi updates, commit, rollback, and BSSID fallback
- [`../../runtime/espectre_protocol.cpp`](../../runtime/espectre_protocol.cpp): shared command and application payload semantics
