# Setup Guide

Choose a frontend first. If it has a published image, [Web Flash](#web-flash-no-coding-required) is the shortest installation path. Local builds use the repository environment and the relevant frontend README. Firmware integrators should start with [SDK.md](SDK.md).

## Choose Your Frontend

| Frontend | Best starting point | Frontend README |
|----------|---------------------|-----------------|
| `ESPHome` | [Web Flash](#web-flash-no-coding-required), Home Assistant entities, and Direct HTTP runtime tuning | [`README.md` (esphome)](../src/cpp/frontend/esphome/README.md) |
| `Native` | [Web Flash](#web-flash-no-coding-required), Improv Serial Wi-Fi provisioning, Direct HTTP, and optional MQTT or Home Assistant MQTT Discovery | [`README.md` (native)](../src/cpp/frontend/native/README.md) |
| `Matter` | [Web Flash](#web-flash-no-coding-required), Matter commissioning, and Direct HTTP detector tuning | [`README.md (matter)`](../src/cpp/frontend/matter/README.md) |
| `Micro-ESPectre` | Research-oriented MicroPython frontend | [`README.md` (micro_espectre)](../src/python/micro_espectre/README.md) |

## Web Flash (no coding required)

Browser flashing uses Web Serial and works in desktop Chrome or Edge. Firefox, Safari, and mobile browsers do not support it; on those, [build and flash with the repository CLI](#local-build-prerequisites) instead.

Published Native and ESPHome images speak standard Improv Serial. The ESPectre installer uses that protocol directly after flashing: it opens Wi-Fi setup when the device is unconfigured and carries the returned private address into Device settings when one is available. Matter exposes a read-only Improv surface for firmware identification and its persisted setup QR and manual code; it rejects Improv Wi-Fi provisioning because the Matter controller owns network commissioning.

Use `Release` for official firmware, `Preview` for the latest build from `main`, or `Development` for the latest build from `develop`. Published ESPectre firmware images start with Lightweight Detection and support persisted runtime switching to High Accuracy through their advertised controls.

To flash:

1. Connect the board over USB
2. Open [espectre.dev/tools/flash](https://espectre.dev/tools/flash/), select **Connect USB device**, and choose the board from the browser's serial-port list
3. Wait for the installer to detect the chip and current firmware, then choose an update, reinstall, or another firmware type and channel
4. Review whether the installation preserves device data or erases the complete flash, then confirm
5. Keep the page open until the board restarts, and complete the displayed setup step

If the board does not enter download mode automatically, use its `BOOT` and `RESET` controls: hold `BOOT`, press and release `RESET`, release `BOOT`, and retry the flash. Board labels and automatic-reset behavior vary, so use the board documentation when those controls are named differently.

## Shared Prerequisites

### Hardware

- ESP32 board with CSI support
- USB cable for flashing
- Wi-Fi network on a band the board supports: 2.4 GHz on every supported chip, or 5 GHz on the dual-band ESP32-C5. Firmware defaults to automatic band selection on ESP32-C5 and to 2.4 GHz on single-band targets. The runtime pins the selected band or bands to 20 MHz. Detection quality on 5 GHz is not characterized yet

Current chip support for the published C++ frontends:

| Frontend | Supported chips | Delivery |
|----------|-----------------|----------|
| `ESPHome` | `ESP32-S3`, `ESP32-S2`, `ESP32-C6`, `ESP32-C5`, `ESP32-C3`, `ESP32` | Published web-flash images, Improv Serial and fallback-AP provisioning |
| `Native` | `ESP32`, `ESP32-S3`, `ESP32-S2`, `ESP32-C3`, `ESP32-C5`, `ESP32-C6` | Published web-flash images and Improv Serial |
| `Matter` | `ESP32`, `ESP32-S3`, `ESP32-C3`, `ESP32-C5`, `ESP32-C6` | Published web-flash images and Matter commissioning |

Use the selected C++ frontend README for workflow and surface details.

### Software

- Desktop Chrome or Edge for browser flashing; see [Web Flash](#web-flash-no-coding-required)
- For local workflows, use the repository [CLI.md](CLI.md) plus the relevant frontend README

### Local Build Prerequisites

Create the repository environment before any local firmware build:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

On Windows PowerShell, create the environment with `py -3 -m venv .venv`, activate `.\.venv\Scripts\Activate.ps1`, and run the same install command.

Native and Matter firmware builds use one shared backend policy: prefer an active `IDF_PATH` environment, a standard local ESP-IDF installation, or the pinned ESP-IDF toolchain already managed by ESPHome, and automatically fall back to the pinned ESP-IDF Docker image when none is available. Repository ESPHome commands explicitly select its native `esp-idf` toolchain and never use PlatformIO.

```bash
./espectre native build --chip c3
```

On Windows, use `.\espectre.cmd native build --chip c3`. The same pattern applies to Matter.

When the local environment is absent and Docker is running, a cached image is used without prompting. If the image is missing, an interactive build asks before downloading it; non-interactive builds must opt in with `--pull missing`. If Docker is installed but stopped, the CLI asks you to start it and retry. Use `--backend local` or `--backend docker` to require one path, and use `./espectre doctor` to inspect only the local ESP-IDF environment.

Docker covers firmware compilation only; flashing still uses host serial tooling. If neither build backend is available, build an ESPHome configuration once to provision its native toolchain, install Docker, or install ESP-IDF `5.5.5` with the official [ESP-IDF Get Started](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) flow.

#### Optional Compiler Cache

`ccache` is optional. It shortens repeat ESP-IDF builds, especially Matter builds, by reusing unchanged compiler output across build directories. The local `./espectre` backend enables it automatically when `ccache` is on `PATH`. Repository Docker builds enable a persistent cache automatically, so the Docker backend needs no host installation.

Install `ccache` for the local backend:

- macOS with Homebrew: `brew install ccache`
- Debian or Ubuntu Linux: `sudo apt update && sudo apt install ccache`; on other distributions, install the `ccache` package with the system package manager
- Windows: the official ESP-IDF Tools installation includes `ccache`; verify it from an ESP-IDF PowerShell with `ccache --version`. For a manually managed toolchain, install the [official Windows release](https://ccache.dev/download.html) or run `choco install ccache` when Chocolatey is available

Confirm the binary is on `PATH`:

```bash
ccache --version
```

`./espectre native build --chip c3`, `./espectre matter build --chip c3`, and `./espectre doctor` then print `Compiler cache: ccache` when the cache is active. Replace `c3` with the selected chip. Set `IDF_CCACHE_ENABLE=0` to disable it for one shell. An explicit `IDF_CCACHE_ENABLE=1` remains supported for toolchains that do not go through the repository wrapper.

Use the equivalent PowerShell environment variable on Windows to disable the cache:

```powershell
$env:IDF_CCACHE_ENABLE = "0"
ccache --version
```

Build cleanup, chip-matched flash selection, and namespace-specific flags are documented in [CLI.md](CLI.md#frontend-workflow-commands).

### Generated Files and Caches

Repository workflows keep generated files under `.cache/`: `firmware/` and `sdk/` contain distribution files, `reports/` contains audit and coverage reports, `build/` contains Docker toolchain homes, and `npz/`, `ruff/`, and `pytest/` contain reusable caches. The virtual environment remains in `.venv/`, and ESP-IDF and ESPHome retain their frontend-specific build directories. Public artifact filenames and URLs are unchanged.

Python bytecode follows the interpreter defaults or the user's `PYTHONPYCACHEPREFIX` setting; repository scripts do not set a bytecode cache location. NPZ tooling continues to support `ESPECTRE_NPZ_CACHE_DIR` for a cache on another volume. Moving an existing `.npz_cache/` to `.cache/npz/` preserves its contents; avoid moving caches while a build, test, or training process is using them.

## Local CLI Workflows

Use the repository CLI from the repository root for local build, flash, monitor, and host-tool tasks.

Matter generates a unique onboarding identity on first boot and stores it in a dedicated factory partition. Retrieve the same QR payload after either a web or CLI flash with:

```bash
./espectre matter qr --chip s3 --port /dev/cu.usbmodemXXXX
```

Normal flashes preserve the QR. Erasing the complete flash generates a new identity on the next boot.

See the repository [CLI.md](CLI.md) for:

- launcher syntax on each host
- namespace and command coverage
- shared host-tool behavior, including the interactive MQTT shell
- common wrapper patterns such as `doctor`, serial monitoring, and CLI examples

Use the frontend READMEs for frontend-specific prerequisites, examples, and chip-specific notes:

- [`README.md` (esphome)](../src/cpp/frontend/esphome/README.md)
- [`README.md` (native)](../src/cpp/frontend/native/README.md)
- [`README.md` (matter)](../src/cpp/frontend/matter/README.md)

## Browser Configuration and Monitoring

Browser tools such as Flash, Device settings, Monitor, and Theremin live on [espectre.dev](https://espectre.dev). Device settings offers starting device-to-broker presets for Home Assistant with the Mosquitto add-on, a broker on the LAN, EMQX Cloud, HiveMQ Cloud, Flespi, and a custom broker; credentials are never prefilled. Provider presets fill stable MQTT TLS ports and prefill editable `.emqxsl.com` and `.hivemq.cloud` endpoint templates. Provider-defined ports and the fixed Flespi hostname are read-only while their preset is selected; account-specific endpoints, credentials, and topic prefixes remain editable. Device settings adds the `mqtts://` scheme automatically when saving a secure preset. Monitor uses Direct HTTP rather than MQTT over WebSockets. To preview the same site from this repository, serve `docs/web` as described in [docs/web/README.md](web/README.md).

On Device settings, click the device ID in the connected-device banner to set the first user-facing name, or click the current name to edit it. The browser saves the value when the field loses focus; Enter saves immediately, and Escape cancels the edit.

## Direct HTTP Connectivity

The published ESP-IDF frontends expose Direct HTTP on the local network. If Device settings or Monitor cannot connect:

1. Confirm that the device and browser are on the same LAN.
2. Try the current private IPv4 address if the `.local` hostname does not resolve.
3. Grant the browser's local-network permission when prompted.
4. Use a desktop Chromium browser listed in the current [browser support matrix](https://espectre.dev/guides/setup/#setup-native-discovery) when another browser blocks hosted HTTPS-to-local-HTTP access.
5. Confirm that the hosted page uses `https://espectre.dev`, `https://www.espectre.dev`, or `https://test.espectre.dev`. A local website preview also requires firmware that accepts the corresponding loopback origin.

Device settings and Monitor accept a private IP, device name, full 16-character device ID, or the last 6 characters of that ID. A full ID maps to the device's unique local address; a name or short ID uses the same bounded discovery as the **Auto-discovery** button. One match connects directly, while multiple matches require an explicit selection. Names, short IDs, and `.local` addresses depend on working mDNS. If discovery fails, use `./espectre devices`, enter the current IP, or check the router's DHCP lease table. Remove a stale remembered endpoint before entering a replacement address. [`DISCOVERY.md`](DISCOVERY.md#browser-bootstrap) owns the peer-discovery contract.

Origin, mixed-content, and local-network permission errors come from the browser boundary rather than the detector. Grant local-network access only to the ESPectre portal, confirm that the device remains on the same trusted LAN, and retry with a browser in the support matrix. Once Direct connects, use [TUNING.md](TUNING.md) for missing CSI, calibration, or detection problems.

## Advanced: SDK Bundles

If you want to embed the sensing layers into your own firmware instead of flashing a published frontend, use the SDK bundle channels:

| Channel | Surface | Best for |
|---------|---------|----------|
| `release` | `https://espectre.dev/artifacts/sdk/release/` and semver GitHub Releases | production integrations and reproducible builds |
| `preview` | `https://espectre.dev/artifacts/sdk/preview/` and the rolling `snapshot` prerelease | validating the latest `main` changes before release |
| `develop` | `https://espectre.dev/artifacts/sdk/develop/` and the rolling `snapshot-dev` prerelease | pre-main validation from `develop` |

The bundle is source-first. It includes:

- `src/cpp/espectre_sdk.h`, the single include that reaches the supported integration surface
- `src/cpp/espectre_sources.cmake` for CMake / ESP-IDF integration
- a component-shaped `src/cpp/` root with `CMakeLists.txt`, `espectre_git_version.cmake`, `idf_component.yml`, and `Kconfig.projbuild`, where the optional MQTT, provisioning, OTA, and stream-runtime groups are selected under the "ESPectre SDK" menuconfig menu

Use [SDK.md](SDK.md) for the integration model, runtime contracts, optional capability groups, and task scheduling policies.

## After Installation

For a published C++ frontend, continue with its local README:

| Frontend | Continue here | What that README owns |
|----------|---------------|-----------------------|
| `ESPHome` | [`README.md`](../src/cpp/frontend/esphome/README.md) | Wi-Fi provisioning, YAML parameters, Home Assistant entities, dashboards, ESPHome-specific troubleshooting |
| `Native` | [`README.md`](../src/cpp/frontend/native/README.md) | Build/flash workflow, Wi-Fi and MQTT setup, Home Assistant MQTT Discovery, native control surface, and HTTPS OTA flow |
| `Matter` | [`README.md`](../src/cpp/frontend/matter/README.md) | Commissioning flow, Matter occupancy surface, and local ESP-IDF workflow |

## Reference: Shared Runtime Concepts

These concepts are shared across the maintained C++ sensing frontends, even though each frontend exposes them differently.

### Shared Sensing Options

These options belong to the shared C++ sensing runtime. This table is the canonical reference for names, defaults, and ranges; the exact user-facing syntax differs by frontend:

- `ESPHome`: YAML under `espectre:`, except the ESP32-C5 band policy, which uses ESPHome's native `wifi.band_mode`
- `Native`: shared ESP-IDF sensing `sdkconfig` menu, with frontend-local overrides in `app/sdkconfig.defaults`
- `Matter`: shared ESP-IDF sensing `sdkconfig` menu, with frontend-local overrides in `app/sdkconfig.defaults`

Frontend coverage:

| Frontend | Shared sensing options available |
|----------|----------------------------------|
| `ESPHome` | yes |
| `Native` | yes |
| `Matter` | yes |

| Option | Type / values | Default | Range / notes |
|--------|---------------|---------|---------------|
| `wifi.band_mode` (ESPHome) / `RuntimeConfig::wifi_band_policy` | `2.4GHz`, `5GHz`, or `AUTO` in ESPHome; `BAND_2G`, `BAND_5G`, or `AUTO` in the SDK | ESP32-C5 firmware: `AUTO`; single-band firmware: `2.4GHz` | `5GHz` and `AUTO` require the dual-band ESP32-C5. |
| `detection_algorithm` | `lightweight` or `high_accuracy` | `lightweight`, including Matter | Lightweight uses less detector CPU and working memory; High Accuracy improves detection quality and skips quiet-room threshold calibration |
| Runtime threshold | probability | detector-specific | Selected automatically at startup; session-adjustable through ESPHome entities, Native Direct HTTP or MQTT, and Matter Direct HTTP when advertised |
| `segmentation_window_size_ms` | int | `1000` | `1000-2000` milliseconds; combined with `csi_target_pps` to define a fixed temporal slot window |
| `csi_target_pps` | int | `100` | `1-500`; defines detector slot cadence and the managed-traffic target, but never enables or disables traffic |
| `csi_traffic_mode` | `internal` or `external` | `internal` | Selects device-generated traffic or externally supplied UDP markers and ICMP Echo Requests independently from `csi_target_pps`; persisted legacy `pacing` or `disabled` values migrate once to `internal` |
| `csi_traffic_multicast_group` | IPv4 multicast address, or empty | `239.255.0.1` | Joined by the UDP listener in `external`. Empty disables the join. Unicast to the device IP still works |
| `traffic_generator_mode` | `ping`, `dns`, or `dns_tcp` | `ping` | `dns` uses UDP, and `dns_tcp` uses persistent TCP |
| `evaluation_interval_ms` | int | `250` | `10-10000` milliseconds between detector evaluations |
| `motion_on_hits` | int | `4` | `1-20` consecutive evaluation hits for `IDLE -> MOTION` (about `0.75-1.0 s` from physical motion at the default `250 ms` interval) |
| `motion_off_hits` | int | `3` | `1-20` consecutive evaluation hits for `MOTION -> IDLE` (about `0.50-0.75 s` from physical idle at the same defaults) |
| `lowpass_enabled` | bool | `false` | Enables low-pass filtering |
| `lowpass_cutoff` | float | `11.0` | `5.0-20.0` Hz against a nominal regular `100 pps` cadence; other targets or substantial missing-slot patterns require filter revalidation |
| `hampel_enabled` | bool | `true` | Enables Hampel outlier filtering |
| `hampel_window` | int | `7` | `3-11` samples |
| `hampel_threshold` | float | `5.0` | `1.0-10.0` MAD units |

Migration from earlier v3 snapshots: replace `traffic_generator_rate: N` with `csi_target_pps: N` plus `csi_traffic_mode: internal`. Persisted `pacing` and `disabled` values are migrated once to `internal`; runtime requests using those removed values fail with `invalid_params`.

### Detection Profile Availability

ESPHome, Native, and Matter ship both `lightweight` and `high_accuracy`. They persist an accepted runtime selection and expose it through their advertised controls: ESPHome entities and Direct HTTP, Native Direct HTTP and optional MQTT, and Matter Direct HTTP. Published Matter firmware starts with `lightweight`.

Use [TUNING.md](TUNING.md#startup-and-detection-profile) to choose a profile and follow its startup procedure, [ALGORITHMS.md](ALGORITHMS.md) for detector behavior and formulas, and the frontend README for configuration syntax.

### Traffic Generation

The C++ motion-detection frontends depend on CSI packets. For the shared detection runtime, traffic is generated internally by default, but the way that traffic is configured or exposed belongs to each frontend surface.

| Path | Target owner | Traffic source | Detector admission | Pacing notes |
|------|--------------|----------------|--------------------|--------------|
| Native / Matter | `CONFIG_ESPECTRE_CSI_TARGET_PPS` | `csi_traffic_mode`; internal by default | yes | phase-preserving cadence without catch-up bursts; local socket backoff only |
| ESPHome | `csi_target_pps` | `csi_traffic_mode`; internal by default | yes | phase-preserving cadence without catch-up bursts; local socket backoff only |
| Collector detector, replay, training, and validation | recorded `csi_target_pps`, collector `--pps`, or a documented legacy fallback | recorded raw HTTP stream | yes | external generator owns rate; HTTP does not pace |

Raw HTTP collection is available on supported ESPectre frontends. [`CLI.md`](CLI.md#collect) owns the collection workflow, and [`API.md`](API.md#csi-collection) owns session behavior and framing.

In `external`, ESP-IDF frontends accept either paced UDP markers or ordinary ICMP Echo Requests sent to the device. UDP traffic can be unicast to each device IP, or sent to multicast group `239.255.0.1`; the frontends join that group automatically, unless `csi_traffic_multicast_group` is empty. Subnet and limited broadcast (`x.x.x.255`, `255.255.255.255`) do not produce reliable HT20 CSI. ESPHome, Native, and Matter listen on port `5555` and accept only the exact four-byte UTF-8 marker `"👻".encode("utf-8")` (`F0 9F 91 BB`); use [`espectre_traffic_generator.py`](../tools/espectre_traffic_generator.py) standalone or through `./espectre collect`. For a dependency-free diagnostic, send unicast ping requests directly to the device IP; Echo Requests become CSI candidates, and the normal IP stack sends the replies. Ping pacing remains the external host's responsibility.

Across Native, Matter, and ESPHome, internal `ping` mode sends ICMP echo requests, `dns` sends connectionless DNS root queries over UDP, and `dns_tcp` sends length-prefixed queries through a persistent, non-blocking TCP connection. Both DNS modes target the gateway resolver on port `53`, and `dns_tcp` requires that resolver to accept TCP queries. The three explicit modes allow deployments to select the protocol that behaves best for their device, Wi-Fi driver, AP, and resolver; there is no automatic fallback. The shared schema default is `ping`.

Use [TUNING.md](TUNING.md#traffic-health-and-target-rate) to evaluate packet occupancy or change `csi_target_pps`, and use the frontend README for configuration syntax.

## Where to Go Next

- For installation, discovery, or Direct connectivity, use this guide and the selected frontend README.
- For CSI health, calibration, placement, or detection quality, use [TUNING.md](TUNING.md).
- To study detector behavior and formulas, use [ALGORITHMS.md](ALGORITHMS.md).
- To change the shared code, use [ARCHITECTURE.md](ARCHITECTURE.md).
- To integrate the SDK into another firmware product, use [SDK.md](SDK.md).
