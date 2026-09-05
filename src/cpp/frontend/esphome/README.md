# ESPectre ESPHome Frontend

Installing an ESPHome image? Start with [Getting Started](#getting-started). If the device is already adopted, [Configuration Surface](#configuration-surface) lists the YAML options and entity controls. The implementation and packaging sections are for component maintainers.

## Scope

The ESPHome frontend is responsible for:

- YAML schema and code generation
- packaging as an ESPHome `external_components` source
- Home Assistant-facing entities
- ESPHome provisioning and dashboard-oriented usage
- ESPHome-specific SDK configuration defaults and troubleshooting

## Getting Started

If you want the browser-flash path, start from [`SETUP.md`](../../../../docs/SETUP.md) and come back here after flashing `ESPHome`.

After flashing, configure Wi-Fi with one of these provisioning paths:

| Method | How |
|--------|-----|
| USB | Use Improv Serial with `./espectre provision --ssid MyNetwork` or any Improv Serial-compatible web flasher, such as the [ESPectre web flasher](https://espectre.dev/tools/flash/) |
| Captive portal | Connect to the `ESPectre Fallback` network and finish setup in the browser |

All maintained ESPHome example configurations enable Improv Serial and return a Device settings URL containing the provisioned device address.

The maintained examples provision Wi-Fi through Improv Serial or the `ESPectre Fallback` captive portal and do not embed SSID, password, or BSSID in YAML. After the device is on the LAN, use the mesh Wi-Fi procedure in [`TUNING.md`](../../../../docs/TUNING.md#mesh-wi-fi-instability) if its access-point association is unstable. `PUT /wifi/bssid` acknowledges the staged mutation with the current BSSID before changing the station association. When the requested BSSID is already active, the default behavior only persists the ESPectre pin; `force=true` performs the reassociation anyway. A new candidate is saved to non-volatile storage before sensing is suspended, CSI is disabled, and its callback is detached. The shared Wi-Fi lifecycle then disconnects, applies the RAM-backed station configuration, and reconnects with promiscuous mode disabled. After ESPectre verifies the selected association and IPv4 acquisition, it runs a non-blocking scan to refresh the CSI receive path, registers a fresh CSI callback, enables a fresh CSI session, commits the pin, and starts a new calibration without rebooting the device. A failed update reconnects once with the previous pin and clears the pending candidate. The operation does not rewrite the ESPHome YAML or saved SSID and password.

Once Wi-Fi is configured, the device is discovered automatically by Home Assistant through ESPHome.

## Integration Surface

The frontend maps runtime state and controls into the ESPHome entities listed under [Integrated Entities](#integrated-entities). Those entities are created automatically when the `espectre:` component is declared.

ESPHome continues to advertise its native API as `_esphomelib._tcp.local.`. ESPectre also publishes the canonical `_espectre._tcp.local.` record for its Direct HTTP endpoint on the shared port `62587`. Run `./espectre devices --frontend esphome` to list that record with the standard ESPectre `device_id`.

The CLI does not inspect or depend on ESPHome's upstream TXT schema. [`DISCOVERY.md`](../../../../docs/DISCOVERY.md#dns-sd-and-mdns) owns the shared discovery contract.

Direct HTTP and the ESPHome entities use the same command engine for the runtime controls advertised by the device. Direct also exposes shared local-management features such as Wi-Fi association inspection, BSSID selection, device labels, peer discovery, and CSI. The capability response is authoritative; [`API.md`](../../../../docs/API.md) owns the resource catalog, and [`DISCOVERY.md`](../../../../docs/DISCOVERY.md#browser-bootstrap) owns peer-assisted browser discovery.

Direct API is enabled by default. To expose only ESPHome's native API and Home Assistant entities, disable it in the component configuration:

```yaml
espectre:
  direct_api: false
```

This disables Direct HTTP requests, SSE telemetry, raw CSI streaming, `_espectre._tcp.local.` discovery, and the peer-assisted browser bootstrap responder. It does not disable ESPHome's native API or ESPectre entities.

A successful Direct mutation republishes the affected number or select state, so Home Assistant and Direct clients observe the same runtime configuration. Wi-Fi credentials, OTA, and ESPHome API encryption remain owned by ESPHome. Changing the ESPectre label does not alter the ESPHome hostname, adopted YAML, or entity IDs.

## Configuration Surface

The ESPHome YAML schema is defined in [`__init__.py`](components/espectre/__init__.py). This README covers ESPHome-specific syntax and entity mapping. See [`SETUP.md`](../../../../docs/SETUP.md) for the shared configuration overview and [`TUNING.md`](../../../../docs/TUNING.md) for the "when and why" of tuning.

### Core Parameters

The shared sensing options, with their defaults and ranges, are documented in the [`Shared Sensing Options`](../../../../docs/SETUP.md#shared-sensing-options) table in `SETUP.md`. In ESPHome, those options live under the `espectre:` section with the same names, as shown in the example below.

These options are applied from YAML during firmware configuration. The [Integrated Entities](#integrated-entities) table is the canonical ESPHome mapping for read-only state, writable runtime controls, and publication cadence.

### Diagnostic Telemetry

Diagnostic entities are always available in production builds. The shared runtime refreshes one cached rate sample from the existing sensing update that also feeds the periodic status log, without adding a diagnostic timer or periodically publishing new Home Assistant states. ESPHome, Direct, and the other C++ frontends read that same runtime-owned sample. Press `Refresh Diagnostics` to publish it to Home Assistant on demand:

| Entity | Meaning |
|--------|---------|
| `Traffic TX Rate` | Successful internal generator or observed external marker packets per second |
| `CSI Callback Rate` | Raw ESP-IDF CSI callbacks per second |
| `CSI Accepted Rate` | CSI packets per second accepted by the sensing pipeline |
| `CSI Admitted Rate` | CSI packets per second admitted to the detector's temporal grid |
| `CSI Filtered Rate` | CSI packets per second rejected by capture validation |
| `CSI Missing Slot Rate` | Detector slots without an admitted packet per second |
| `CSI Excess Rate` | Non-selected same-slot candidates per second |
| `CSI Stale Rate` | Stale packets discarded per second |
| `CSI Out-of-Order Rate` | Duplicate or backward-timestamp packets discarded per second |
| `CSI Temporal Occupancy` | Valid-slot occupancy of the active detector window |
| `WiFi Channel` | Current primary channel reported by the associated access point |
| `WiFi RSSI` | Current RSSI reported by the Wi-Fi association |

Use the diagnostic sequence in [`TUNING.md`](../../../../docs/TUNING.md#monitoring) to localize traffic, capture, temporal-admission, and detector failures.

Runtime performance, heap, load, and detector timing are collected as production diagnostics and are available through Direct HTTP without a build-time switch or periodic debug logs.

### Detection Profile Selection

```yaml
wifi:
  band_mode: AUTO  # ESP32-C5 only; optional because AUTO is the default

espectre:
  detection_algorithm: lightweight  # or high_accuracy
```

ESPHome owns Wi-Fi association policy through `wifi.band_mode`; it is not an `espectre:` property. On ESP32-C5 it accepts `2.4GHz`, `5GHz`, or `AUTO` and is optional; when omitted, ESPectre follows ESPHome's `AUTO` default. Other supported targets are single-band and remain fixed to 2.4 GHz. ESPectre mirrors the effective ESPHome selection into its runtime and selects `vht20` after a 5 GHz C5 association or `ht20` on 2.4 GHz. The ESP32-C5 example uses the `AUTO` default; detection quality on 5 GHz is not yet characterized.

The YAML value is the initial profile when no persisted selection exists. The Home Assistant `detector_select` changes it live and persists the choice across reboot. `high_accuracy -> lightweight` starts calibration automatically, and `calibration_active_sensor` reflects automatic and user-triggered calibration state.

See [`SETUP.md`](../../../../docs/SETUP.md#shared-sensing-options) for shared defaults and ranges, [`TUNING.md`](../../../../docs/TUNING.md#startup-and-detection-profile) for profile selection and startup, and [`ALGORITHMS.md`](../../../../docs/ALGORITHMS.md) for detector behavior.

### Example

```yaml
espectre:
  detection_algorithm: lightweight
  csi_target_pps: 100
  csi_traffic_mode: internal
  csi_traffic_multicast_group: "239.255.0.1"
  traffic_generator_mode: ping
  segmentation_window_size_ms: 1000
  motion_on_hits: 4
  motion_off_hits: 3
  direct_api: true
```

## Entity Customization

### Integrated Entities

| Sensor config | Type | Default name | Description |
|---------------|------|--------------|-------------|
| `movement_sensor` | sensor | `Movement Score` | Current movement score (0.0–1.0), published every `evaluation_interval_ms` |
| `motion_sensor` | binary_sensor | `Motion Detected` | Edge-driven motion state; resets to idle when sensing stops or CSI restarts |
| `threshold_number` | number | `Threshold` | Runtime probability threshold (0.0–1.0) |
| `motion_on_hits_number` | number | `Motion On Hits` | Runtime motion-on debounce count (1–20) |
| `motion_off_hits_number` | number | `Motion Off Hits` | Runtime motion-off debounce count (1–20) |
| `detector_select` | select | `Detection Profile` | Runtime `lightweight` / `high_accuracy` selection |
| `csi_traffic_mode_select` | select | `CSI Traffic Ownership` | Runtime `internal` / `external` selection |
| `traffic_generator_mode_select` | select | `CSI Traffic Source` | Runtime `ping` / `dns` (UDP) / `dns_tcp` selection |
| `sensing_switch` | switch | `Sensing Enabled` | Enables or pauses sensing through the common command engine; publishes the runtime state at startup |
| `recalibrate_button` | button | `Recalibrate` | Starts runtime recalibration |
| `calibration_active_sensor` | binary_sensor | `Calibration Active` | Read-only authoritative calibration state |
| `diagnostics_button` | button | `Refresh Diagnostics` | Publishes the latest cached diagnostic sample on demand |
| `traffic_rate_sensor` | sensor | `Traffic TX Rate` | Diagnostic traffic rate |
| `csi_callback_rate_sensor` | sensor | `CSI Callback Rate` | Raw CSI callback rate; diagnostic-only |
| `csi_accepted_rate_sensor` | sensor | `CSI Accepted Rate` | Raw identity-accepted capture rate before temporal admission; diagnostic-only |
| `csi_admitted_rate_sensor` | sensor | `CSI Admitted Rate` | Rate admitted to the detector's temporal grid; diagnostic-only |
| `csi_filtered_rate_sensor` | sensor | `CSI Filtered Rate` | Capture rejection rate; diagnostic-only |
| `csi_missing_rate_sensor` | sensor | `CSI Missing Slot Rate` | Missing detector slots per second; diagnostic-only |
| `csi_excess_rate_sensor` | sensor | `CSI Excess Rate` | Non-selected same-slot candidates per second, including candidates replaced by one nearer the slot center; diagnostic-only |
| `csi_stale_rate_sensor` | sensor | `CSI Stale Rate` | Packets discarded as stale per second; diagnostic-only |
| `csi_out_of_order_rate_sensor` | sensor | `CSI Out-of-Order Rate` | Duplicate or backward-timestamp packets discarded per second; diagnostic-only |
| `csi_occupancy_sensor` | sensor | `CSI Temporal Occupancy` | Valid-slot occupancy of the active detector window; diagnostic-only |
| `wifi_channel_sensor` | sensor | `WiFi Channel` | Current associated Wi-Fi channel; diagnostic-only |
| `wifi_rssi_sensor` | sensor | `WiFi RSSI` | Current associated Wi-Fi RSSI; diagnostic-only |

All entities support standard ESPHome options such as:

- `name`
- `internal`
- `icon`
- `disabled_by_default`

The `movement_sensor` also supports ESPHome [sensor filters](https://esphome.io/components/sensor/#sensor-filters).

Common filters:

| Filter | Example | Description |
|--------|---------|-------------|
| `multiply` | `multiply: 10` | Scale values |
| `round` | `round: 1` | Round to N decimals |
| `clamp` | `min_value: 0, max_value: 100` | Limit the value range |
| `offset` | `offset: -0.5` | Add or subtract a constant |
| `sliding_window_moving_average` | `window_size: 5` | Smooth noisy readings |

Example:

```yaml
espectre:
  movement_sensor:
    name: "Living Room Movement"
    internal: true
    icon: "mdi:sine-wave"
    filters:
      - multiply: 100
      - clamp:
          min_value: 0
          max_value: 100
      - round: 1
  motion_sensor:
    name: "Living Room Motion"
    icon: "mdi:motion-sensor"
  threshold_number:
    name: "Living Room Threshold"
```

Use `internal: true` on `movement_sensor` when you want to keep the binary motion entity for automations without publishing the raw score to Home Assistant.

## Home Assistant Integration

Once the device is flashed and connected to Wi-Fi:

1. Home Assistant discovers it through ESPHome
2. Go to **Settings** -> **Devices & Services** -> **ESPHome**
3. Configure the discovered device
4. The default entities are added automatically

The ESPHome frontend exposes movement, motion, sensing state, tuning controls, calibration, traffic controls, and on-demand diagnostics as Home Assistant entities. Every writable entity invokes the common command engine and republishes authoritative state when a command is rejected. Direct mutations use the same engine and immediately synchronize the affected entities.

Movement Score updates on the detector evaluation cadence, 250 ms by default. The high-rate path runs while the Movement Score entity exists or a Direct SSE client is connected, so Direct-only configurations do not need an unused Home Assistant sensor. Motion Detected publishes only on filtered state edges.

Threshold publishes after operator writes, calibration, and Lightweight settled-level recovery. Motion-hit controls publish on change, Calibration Active reports the read-only runtime state, and the traffic selects mirror runtime state on connect and after each accepted change. Diagnostic sensors publish only when Refresh Diagnostics runs the canonical `diagnostics` query.

If the Home Assistant recorder is a concern, exclude `sensor.*_movement_score` rather than lowering `evaluation_interval_ms`.

To manage configuration and OTA updates, install ESPHome Device Builder and adopt the discovered device. The adopted configuration uses the GitHub source profile, follows `main`, and identifies that rolling build as `0.0.0-main`. Repository builds launched through `./espectre esphome` resolve `project_version` from the same numeric `git describe` identity used by the other frontends. First-party CI and release builds override it with the detected build version or release tag.

To install a prebuilt OTA image from GitHub Releases instead, download the `espectre-esphome-<channel-or-version>-<chip>-ota.bin` asset and upload it over the network:

```bash
./espectre esphome flash --chip c6 --device espectre.local --firmware espectre-esphome-3.0.0-esp32c6-ota.bin
```

To stay on a released version, use the matching prebuilt image rather than the rolling `main` example.

### Dashboard Examples

Examples live in:

| File | Description |
|------|-------------|
| [`home-assistant-dashboard.yaml`](examples/home-assistant-dashboard.yaml) | Production dashboard with motion, movement score, history, controls, and diagnostics |

![ESPectre Home Assistant dashboard](../../../../docs/web/assets/images/guides/home-assistant-dashboard.png)

*Home Assistant dashboard with motion state, movement score, movement-versus-threshold history, detection profile, threshold, calibration, and diagnostics.*

To import a dashboard:

1. Go to **Settings** -> **Dashboards** -> **Add Dashboard**
2. Open the dashboard and choose **Edit**
3. Open the raw configuration editor
4. Replace the default content with the YAML from the example file
5. Save the dashboard

If you changed the device name from `espectre`, update the entity IDs in the YAML. If you enabled `name_add_mac_suffix: true`, include the MAC suffix in the entity names as well. Inspect the exact IDs under the Home Assistant device before adapting the dashboard because an existing registry collision can add a suffix such as `_2`.

## Traffic Configuration

The ESPHome surface exposes the shared runtime traffic settings under `espectre:`. [`SETUP.md`](../../../../docs/SETUP.md#traffic-generation) owns traffic modes, pacing, ports, and external marker behavior; [`TUNING.md`](../../../../docs/TUNING.md#traffic-health-and-target-rate) owns rate and occupancy guidance.

### Internal Traffic Generator

```yaml
espectre:
  csi_target_pps: 100
  csi_traffic_mode: internal
  traffic_generator_mode: ping
```

The `traffic_generator_mode_select` entity can change the internal source at runtime, and `csi_traffic_mode_select` can switch between internal and external ownership. Both selections persist after an accepted change.

The shared component default is `ping`. The published product examples use that default. `dns` and `dns_tcp` remain available where they perform better, and no mode silently falls back to another.

### External Traffic Mode

To disable the internal generator and rely on external traffic:

```yaml
espectre:
  csi_target_pps: 100
  csi_traffic_mode: external
  csi_traffic_multicast_group: "239.255.0.1"
  evaluation_interval_ms: 250
```

For raw collection, use `./espectre collect` with this device's IP, hostname, Direct endpoint, or device ID. ESPHome exposes Direct and raw HTTP on the shared port `62587`; [`CLI.md`](../../../../docs/CLI.md#collect) owns the collector workflow, including its persistent external-traffic selection. External mode also accepts ordinary unicast ICMP Echo Requests addressed to the device, which provides a dependency-free diagnostic source without changing the UDP collection workflow.

## Build and Consumption

The `release`, `preview`, and `develop` channels publish one full-flash image and one OTA image per supported chip, with `lightweight` as the initial detector. Both `lightweight` and `high_accuracy` are available in the image and can be selected through the persisted runtime detector entity. After adoption, ESPHome Device Builder can compile and install updates wirelessly from the device YAML; `detection_algorithm` sets the initial detector for a fresh configuration rather than limiting which detector the firmware supports.

### As an ESPHome external component

Each maintained chip has one canonical example. By default it includes `espectre-source-github.yaml`, which resolves the component from GitHub:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/francescopace/espectre
      path: src/cpp/frontend/esphome/components
    components: [espectre]
```

Repository development selects `espectre-source-local.yaml` instead, which resolves the same component from the local checkout:

```yaml
external_components:
  - source:
      type: local
      path: ../components
    components: [espectre]
```

### Repository CLI

See [`CLI.md`](../../../../docs/CLI.md) for shared CLI syntax, host-side tools, and wrapper behavior.

```bash
./espectre esphome build --chip c6 --clean
./espectre esphome flash --chip c6
./espectre esphome config --chip c6
./espectre esphome monitor --chip c6 --device /dev/cu.usbmodemXXXX
```

On Windows, use `.\espectre.cmd esphome ...` from the repository root and pass a COM port such as `COM5` to `--device` when serial access is needed.

The repository CLI keeps the selected canonical YAML and loads the ESPectre component from the local checkout. Use `flash` for upload-only and `monitor` for logs.

## Hardware and Packaging Notes

### Build Toolchain

The ESPHome examples use the native ESP-IDF backend from the ESPHome version pinned in [`requirements.txt`](../../../../requirements.txt). [`__init__.py`](components/espectre/__init__.py) registers this component directory with ESP-IDF's component manager. Its [`CMakeLists.txt`](components/espectre/CMakeLists.txt) reuses the canonical SDK build definition at [`CMakeLists.txt`](../../CMakeLists.txt), so ESPHome compiles `src/cpp/core/` and `src/cpp/runtime/esp_idf/` directly. No toolchain override or separate library package is required.

### Automatic SDK Configuration

The frontend automatically sets the ESP-IDF options required by the runtime, including CSI enablement, disabled Wi-Fi power save, TX AMPDU, the shared high-rate Wi-Fi buffer profile, lwIP IRAM optimization, and enlarged TCP/IP and UDP mailboxes. RX AMPDU remains disabled so sensing receives individual CSI frames. The component registers the shared SDK's portable log sink with the ESPHome logger, so `espectre` and `espectre.runtime` messages follow the YAML logger level without passing through ESP-IDF's log formatter first. On maintained USB-OTG configurations where the ROM CDC path is unreliable, the component selects the shared TinyUSB CDC console and routes ESPHome logging and Improv Serial through it. In most cases you do not need to set these options manually.

The supplied examples deliberately use Improv Serial instead of BLE provisioning. Omitting BLE keeps its provisioning stack out of the firmware, reducing flash and memory pressure. BLE and Wi-Fi share the ESP32's 2.4 GHz radio; while BLE is active, coexistence can interrupt the Wi-Fi packet flow and reduce the CSI occupancy required for reliable sensing. Disabling BLE after provisioning ends that radio contention, but it does not remove the compiled-in stack or the added provisioning lifecycle.

For board-specific tweaks, you can still add `sdkconfig_options` in YAML:

```yaml
esp32:
  variant: ESP32C6
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ: "160"
```

### Flash Size and Partitions

The ESPHome frontend fits in `4 MB` flash with OTA. It uses the board and framework default partition table unless you override it in your own project.

If you need a custom table:

```yaml
esp32:
  variant: ESP32C6
  partitions: /absolute/path/to/partitions_custom.csv
```

The frontend itself does not require a custom partition table.

## ESPHome-Specific Troubleshooting

Use [`TUNING.md`](../../../../docs/TUNING.md#troubleshooting) for missing motion, false positives, calibration, packet health, placement, or unstable detection. Use the shared [`SETUP.md`](../../../../docs/SETUP.md#web-flash-no-coding-required) recovery procedure when the board does not enter download mode.

### View logs

Home Assistant entity updates do not replace the serial status log. The shared runtime forwards its 1 Hz `IDLE | csi:` / `MOTION | csi:` heartbeats through the ESPHome sink, so they should appear on USB serial and in `esphome logs` when the `espectre.runtime` tag permits INFO messages.

```bash
esphome logs <your-config>.yaml
esphome logs <your-config>.yaml --device espectre.local
./espectre monitor --port /dev/cu.usbmodem*
```

## Implementation Map

This map is for component maintainers; it is not required for normal installation or tuning.

- [`__init__.py`](components/espectre/__init__.py): YAML schema, validation, codegen, native ESP-IDF component registration, and ESPHome build flags
- [`CMakeLists.txt`](components/espectre/CMakeLists.txt): native ESP-IDF bridge to the canonical shared SDK build definition
- [`espectre.cpp`](components/espectre/espectre.cpp), [`espectre.h`](components/espectre/espectre.h): ESPHome adapter over the shared runtime frontend controller
- [`sensor_publisher.cpp`](components/espectre/sensor_publisher.cpp): movement and motion publishing
- [`threshold_number.cpp`](components/espectre/threshold_number.cpp): runtime threshold control
- [`motion_hits_number.cpp`](components/espectre/motion_hits_number.cpp): runtime motion-hit debounce control
- [`detector_select.cpp`](components/espectre/detector_select.cpp): persisted runtime detector selection
- [`sensing_switch.cpp`](components/espectre/sensing_switch.cpp): sensing lifecycle control
- [`recalibrate_button.cpp`](components/espectre/recalibrate_button.cpp): runtime recalibration action
- [`traffic_mode_select.cpp`](components/espectre/traffic_mode_select.cpp): runtime CSI traffic ownership and generator control
- [`examples/`](examples/): production and local-development configurations for ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C5, and ESP32-C6, plus the Home Assistant dashboard
