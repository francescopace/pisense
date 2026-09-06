# Micro-ESPectre

Micro-ESPectre is the small, research-oriented MicroPython sensing frontend. It keeps the device path easy to modify while using native ESP-IDF components only where timing or transport work benefits from fixed memory and predictable scheduling.

This README is the source of truth for the Micro-ESPectre build, deployment, configuration, runtime, and Direct surface. The general architecture, setup, tuning, CLI, and protocol documents describe the maintained C++ frontends unless they explicitly link here. Changes to shared detector behavior and serialized messages must still remain aligned with the C++ implementation.

## Device profile

The deployed runtime intentionally contains only:

- the Lightweight CSI detector and its startup threshold calibration;
- the shared ESP-IDF managed traffic generator with ICMP, DNS/UDP, and DNS/TCP modes;
- a bounded Direct HTTP endpoint for monitoring and manual recalibration;
- one SSE motion-event client;
- mDNS/DNS-SD advertisement and a unique `.local` hostname; and
- serial logging and the MicroPython REPL.

The device does not deploy the High Accuracy ML detector, ML weights, MQTT, Home Assistant discovery, runtime detector switching, raw CSI streaming, OTA, or configuration mutations. The High Accuracy and pure-Python Lightweight implementations live under `tools/lib/` for host-side research and C++/Python validation; `micro deploy` does not copy them to the device.

ESPectre contributed direct ESP32 Wi-Fi CSI access to mainline MicroPython through [micropython/micropython#18460](https://github.com/micropython/micropython/pull/18460). Micro-ESPectre builds a pinned mainline revision with a lean ESPectre board profile rather than using the earlier CSI fork.

The project firmware supports classic ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C5, and ESP32-C6. It uses an associated 2.4 GHz Wi-Fi station with HT20 CSI and does not enable promiscuous mode. Valid network credentials are therefore required for protected networks, but they are not proof of consent or authorization to sense a space.

## Build and deploy

Complete the shared prerequisites in [SETUP.md](../../../docs/SETUP.md#local-build-prerequisites), then run the Micro-ESPectre workflow from the repository root:

```bash
cp src/python/micro_espectre/config_local.py.example src/python/micro_espectre/config_local.py
./espectre micro flash --chip c3 --erase
./espectre micro deploy
./espectre micro run
```

On Windows, replace `./espectre` with `.\espectre.cmd`; the namespace and flags are unchanged.

Set the Wi-Fi credentials in `config_local.py`; do not commit that file.

```python
WIFI_SSID = "YourWiFiSSID"
WIFI_PASSWORD = "YourWiFiPassword"
# WIFI_BSSID = "AA:BB:CC:DD:EE:FF"  # Optional AP lock
# WIFI_CHANNEL = 6  # Optional known channel used with WIFI_BSSID
```

Firmware builds require an ESP-IDF 5.5 host toolchain. The CLI prefers an active `IDF_PATH`, a standard local ESP-IDF installation, or the pinned toolchain managed by ESPHome, and falls back to the pinned Docker image when no local installation is available. Use `--backend auto|local|docker` and `--pull ask|missing|never` to control that selection. When `ccache` is available, the local build enables it automatically and reports `Compiler cache: ccache`; set `IDF_CCACHE_ENABLE=0` to disable it for one shell. Cached source, build trees, and firmware images live under `.firmware/` in this directory.

The firmware image freezes only MicroPython's upstream boot and filesystem helpers. The complete ESPectre application is compiled to optimized `.mpy -O3` bytecode and stored on the filesystem, so research changes require only `micro deploy`, not a firmware rebuild and flash. Deployment uploads the complete manifest to a staging directory and atomically activates it, restoring the previous directory after an interrupted swap. The device and `mpy-cross` use MPY ABI 6.3.

Project builds populate the ESP-IDF application descriptor with project name `micro-espectre` and the ESPectre Git version, using the same `espectre_git_version.cmake` resolver as Native and Matter. The build retains these fields in the image and startup logs, and packages `micro-espectre.bin` into the existing combined firmware artifact. This version identifies the ESPectre firmware build, including its native components; a later `micro deploy` can replace the filesystem application without changing the embedded descriptor. Older images may have empty descriptor fields and require log-based identification or an explicitly inferred runtime label in the USB installer. Rebuild and flash the firmware to populate these fields on an existing device.

The firmware links the core-only ESPectre SDK and the focused shared traffic runtime as separate ESP-IDF components. Neither component depends directly on ESP-IDF logging; the MicroPython frontend registers one ESP-IDF Log v2 sink before it creates a detector, sampler, or traffic generator. The sink passes each callback's `va_list` to `esp_log_va` and leaves line formatting to ESP-IDF. It does not allocate a formatting buffer in the runtime hot path. Its MicroPython bindings expose finalizable Lightweight `Detector` and `TemporalCsiSampler` objects through the public `espectre_core_sdk.h` facade and adapt the existing `TrafficGeneratorManager` without copying its protocol implementations. The production Lightweight detector, temporal admission, traffic pacing, and packet generation hot paths therefore run in C++, while MicroPython owns orchestration, calibration policy, diagnostics, and delivery. The application fails at startup if the core module is absent or incompatible; it does not silently fall back to the Python detector on the device. Equivalent Python detector implementations remain available under `tools/lib/` for replay, training, and host-side experimentation. The other native component is the Direct HTTP/mDNS service. Bluetooth, ESP-NOW, asyncio, Ethernet, unused peripheral bindings, and unused generic Python modules remain disabled.

The board profile prioritizes the Wi-Fi CSI path through performance optimization, balanced queues, a 1 kHz FreeRTOS tick, disabled power management, and Wi-Fi, PHY, and lwIP IRAM placement. Classic ESP32 uses reduced Wi-Fi queues and omits lwIP IRAM placement to preserve heap. RX AMPDU remains disabled so the device receives individual HT20 CSI frames.

## Runtime behavior

The runtime uses this fixed sensing path:

```text
Wi-Fi -> native managed traffic -> CSI temporal sampler -> Lightweight calibration -> detection -> Direct SSE and serial
```

Key settings live in `config.py`:

```python
DEVICE_LABEL = ""
CSI_TARGET_PPS = 100
TRAFFIC_GENERATOR_ENABLED = True
TRAFFIC_GENERATOR_MODE = "ping"
CSI_LINK_RECOVERY_TIMEOUT_MS = 5000
CSI_BUFFER_SIZE = 16
CSI_CAPTURE_MAX_DATA_LEN = 256
SEGMENTATION_WINDOW_SIZE_MS = 1000
EVALUATION_INTERVAL_MS = 250
MOTION_ON_HITS = 4
MOTION_OFF_HITS = 3
ENABLE_LOWPASS_FILTER = False
LOWPASS_CUTOFF = 11.0
ENABLE_HAMPEL_FILTER = False
HAMPEL_WINDOW = 7
HAMPEL_THRESHOLD = 5.0
```

These `config.py` values are deployment settings rather than runtime mutations. An empty `DEVICE_LABEL` keeps the generated device name. `CSI_TARGET_PPS` defines the detector grid and managed traffic rate. `MOTION_ON_HITS` and `MOTION_OFF_HITS` apply the same evaluation-tick filtering as the C++ runtime, but changing them requires another deployment. The low-pass filter calculates its coefficient against a nominal 100 Hz sample rate, so a different target rate or substantial missing-slot pattern requires validation. The committed device profile disables both optional filters by default. Calibration aborts after 15 seconds without admitted CSI or after twice `CALIBRATION_DURATION_MS` plus one segmentation window (at least 15 seconds), even if sparse packets continue arriving.

`TRAFFIC_GENERATOR_MODE` accepts `ping`, `dns`, or `dns_tcp`; the committed configuration selects `ping` for every supported chip, and `config_local.py` can select another mode for a deployment. Both DNS modes use the gateway resolver on port 53. Setting `TRAFFIC_GENERATOR_ENABLED = False` requires an external CSI traffic source. Micro-ESPectre has no runtime traffic mutation, external UDP marker listener, or multicast join. `CSI_CAPTURE_MAX_DATA_LEN` selects the fixed native ring-record stride: 256 supports the doubled HT20 layout, while 128 is suitable only when every captured frame uses the canonical payload because larger frames are truncated.

In `config_local.py`, `WIFI_CHANNEL` can accompany `WIFI_BSSID` to avoid a scan during association. If no CSI frame arrives for `CSI_LINK_RECOVERY_TIMEOUT_MS`, the runtime first rearms CSI. If the stall persists, it disables CSI, reconnects Wi-Fi, enables CSI with a fresh native ring, recalibrates, and republishes Direct discovery.

The production `TemporalCsiSampler` retains the packet nearest each slot center, preserves missing slots, and keeps the live detector geometry independent from observed network jitter. Classic-MAC targets rotate Espressif's native `0~31, -32~-1` CSI layout into the centered convention before feature processing. Collector-derived sensing, replay, training, Python validation, and C++ integration replay apply equivalent temporal admission before feature extraction. See [ALGORITHMS.md](../../../docs/ALGORITHMS.md) for the shared detector rationale.

## Direct HTTP surface

Micro-ESPectre listens on port `62587`, advertises `_espectre._tcp.local.`, and exposes:

- `GET /espectre/v1/{health,device,capabilities,sensing,wifi,diagnostics}`;
- `POST /espectre/v1/sensing/calibrations` for session-only recalibration;
- `GET /espectre/v1/events` with canonical `motion` SSE events; and
- CORS and Private Network Access preflight support for the ESPectre website.

At boot, the frontend derives `device_id` as the first 64 bits of `SHA-256("espectre-device-id-v1" || station_mac_bytes)` and uses the generated name `ESPectre <chip> <last-six-device-id-characters>` when `DEVICE_LABEL` is empty. Its stable hostname is `espectre-<device_id>.local`. The `firmware` TXT value comes from the running ESP-IDF application descriptor, and `chip` is the active `CONFIG_IDF_TARGET`; the same values appear in `device`.

The exact capability response is authoritative. Micro-ESPectre maintains its own bounded registry and advertises read-only resources plus manual recalibration, but no configuration mutations, MQTT, CSI, or peer discovery. The cross-language protocol probe verifies this exact capability intersection and the serialized shared-message parity. The Monitor site therefore displays sensing and can request a session-only recalibration without exposing unsupported controls. Enter the device IP or `espectre-<device-id>.local` in the site, or discover it with:

```bash
./espectre devices --frontend micro
```

Monitor Auto-discovery can also list this device when a Native, ESPHome, or Matter responder is already on the LAN. Micro-ESPectre does not answer the one-shot bootstrap hostname or `/devices`; without another eligible responder, use the private IP or unique `.local` hostname. Shared discovery is documented in [Peer-assisted browser discovery](../../../docs/DISCOVERY.md#browser-bootstrap).

Every resource and operation registers a parameter validator. Direct parses each bounded JSON body and runs that validator before reading a snapshot or queuing recalibration. The current routes accept no parameters: omit the body or send `{}`. A non-empty body requires `Content-Type: application/json`. Malformed JSON, non-object bodies, and unexpected fields return HTTP 400 with `invalid_params`; an unsupported media type returns HTTP 415. An incomplete body closes the connection without executing the request.

Only one SSE client and one in-flight event are retained to bound sockets, heap, and work queued on the HTTP server task. Request bodies are limited to 512 bytes; larger bodies receive HTTP 413, and the connection closes. SSE events are limited to 4,096 bytes. The stream sets `Cache-Control: no-store`, emits `: heartbeat` every 10 seconds, retains at most one queued heartbeat, and closes after a send error. A peer close or reset does not increment `send_failures`; timeout and backpressure failures do. Query snapshots are generated by the MicroPython runtime, while HTTP framing, request parsing, CORS, mDNS, and Direct transport counters run in the native firmware component. Telemetry follows `EVALUATION_INTERVAL_MS` only while an SSE client is connected. Manual recalibration is queued onto the main loop and remains session-only.

By default, the HTTP task runs at the MicroPython task priority, and the service accepts at most 20 Direct requests per one-second window. A lightweight health refresh remains on a one-second cadence, while full diagnostics normally wait for the native CSI ring to drain on a second cadence. Diagnostics and garbage collection run after at most another 500 ms under sustained backlog, making maintenance bounded while preserving the empty-ring fast path. The diagnostics payload uses the canonical fields Micro-ESPectre can measure plus the optional bounded `direct_http` transport object for queue and failure counters; unavailable C++ measurements are omitted.

Direct rejects requests without an `Origin` header and accepts only `https://espectre.dev`, `https://www.espectre.dev`, and `https://test.espectre.dev` in published firmware. Development builds may set `CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED=y` to accept exact HTTP loopback hosts with an optional valid port. The published board profiles leave this option disabled.

The native Direct HTTPD and managed traffic priorities are build policies under
the `Micro-ESPectre > Advanced task scheduling` Kconfig menu. They use
`CONFIG_ESPECTRE_DIRECT_HTTPD_TASK_PRIORITY` and
`CONFIG_ESPECTRE_TRAFFIC_TASK_PRIORITY`, respectively, with a safe range of
`1..10` and a shared default of `1`. Target board profiles may override these
values after validation. The MicroPython VM priority remains owned by
MicroPython and is not exposed by ESPectre.

## Commands

| Command | Purpose |
| --- | --- |
| `./espectre micro build [--chip <esp32\|c3\|s2\|s3\|c5\|c6>]` | Build the lean project firmware; the default chip is `esp32` |
| `./espectre micro flash --chip <chip> --erase` | Build and flash the canonical project image |
| `./espectre micro deploy` | Compile and upload the complete `.mpy -O3` manifest |
| `./espectre micro run` | Start the device application |
| `./espectre micro verify` | Check firmware, native modules, and deployed bytecode |
| `./espectre monitor --chip <chip> --reset` | Start the application through esptool, then follow serial output |

`--port` is optional for `flash`, `deploy`, `run`, and `verify`. `micro flash` requires `--chip`, builds the canonical project image, and flashes the files listed in its generated `flasher_args.json` metadata in one esptool session. Other device-facing commands accept an optional chip to resolve ambiguous candidates, while `build` defaults to `esp32`. `micro deploy --config <path>` compiles an alternate local override as device `config_local.mpy`, which is useful for isolated laboratory settings.

`micro build --json` emits final artifact metadata, `micro flash --json` adds the selected port after a successful flash, and `micro run --json` streams logs and emits a `direct_ready` event when the application reports its endpoint. Run `./espectre micro <command> --help` for the current flags.

## Troubleshooting

Use [SETUP.md](../../../docs/SETUP.md#direct-http-connectivity) for Direct HTTP, browser permission, address, and discovery failures. Use [TUNING.md](../../../docs/TUNING.md#troubleshooting) for missing CSI, calibration, placement, false positives, or unstable detection.

### Deployed changes do not appear

Run `./espectre micro deploy` again, then restart the application with `./espectre micro run`. If the application still does not start, use `./espectre micro verify` to check CSI firmware support, the MicroPython version, required bytecode, and `config_local.mpy`.

### Monitor cannot open the event stream

Micro-ESPectre retains one SSE client. Close any previous Monitor tab or client before reconnecting with the private IP or unique `.local` hostname.

### Wi-Fi never becomes ready

Confirm that `config_local.py` exists, contains the intended SSID and password, and does not retain a stale optional `WIFI_BSSID`. Deploy the updated configuration and run the application again.

## Validation

Run the focused host tests from the repository environment:

```bash
.venv/bin/pytest test/python/host/cli/test_traffic_generator.py test/python/micro/test_espectre_cli_micro.py test/python/micro/test_micro_protocol.py -q --tb=short
```

Firmware builds use the project wrapper:

```bash
./espectre micro build --chip c3
```

## Contributing

Keep device code compatible with MicroPython: do not use `asyncio`, CPython-only APIs, or heavy libraries, keep allocations bounded for ESP32-class memory, use `config.py` for shared runtime constants, and write code and comments in English. Host-only analysis and validation code belongs under `tools/`.

Use this header for Python modules in this directory:

```python
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
Micro-ESPectre - [Module Name]

[Brief description]

Author: [your name] <[your email]>
"""
```

For the full Python coverage gate, including device-runtime coverage, run:

```bash
.venv/bin/pytest test/python -q --tb=short --cov=src/python/micro_espectre --cov=src/python/espectre_cli --cov-branch --cov-report=json:python-coverage.json --cov-report=term-missing
.venv/bin/python test/python/check_coverage.py python-coverage.json
```
