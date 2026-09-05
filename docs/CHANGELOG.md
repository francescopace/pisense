# Changelog

All notable changes to this project will be documented in this file.

---

## [3.0.0-rc2] - in progress

### Changes

- ESPHome firmware now builds against ESPHome `2026.8.2`. The matching host packages in `requirements.txt` are aligned with that pin.
- Website dependencies are updated to Rollup `4.63.1`, `@rollup/plugin-commonjs` `29.0.3`, `@rollup/plugin-node-resolve` `16.0.3`, and `improv-wifi-serial-sdk` `2.8.1`, including the fix for CVE-2026-27606 in Rollup. Dependabot now checks those packages weekly on `develop`, and website CI audits the committed lockfile, including build dependencies, failing for known vulnerabilities rated low or higher.
- Firmware builds use one Espressif-generated SBOM for distribution and dependency checks. Development audits report findings in job summaries and GitHub Code scanning; release gates block license violations, scanner failures, and applicable high, critical, or unscored CVEs.
- Local caches and CI output are consolidated under `.cache/`, including firmware and SDK packages, audit and coverage reports, NPZ data, pytest, and Docker toolchain homes.

---

## [3.0.0-rc1] - 2026-09-05 - Wi-Fi sensing SDK, multi-frontend support, ready for new runtimes

ESPectre 3.0 makes Wi-Fi sensing a building block for your own products. This first release candidate introduces a C++ SDK and separates the sensing engine from the runtime and frontend, so developers can bring it into their own firmware and extend it to new integrations.

Install, configure, and tune a sensor from the browser, or use one CLI from the first firmware build through deployment, diagnostics, and CSI collection. ESPHome is joined by standalone Native and Matter firmware, all built on the shared sensing engine. Matter controller validation remains limited in this release candidate.

### Highlights

- Embed ESPectre in your own product: the public C++ source SDK offers the sensing runtime or direct access to the detectors. Commercial licenses are available for proprietary firmware, alongside GPLv3 for open-source use.
- Add runtimes and frontends around the same sensing engine. Portable runtime contracts separate detection from platform services and product integrations. ESP-IDF is the supplied full runtime backend, with ESPHome, Native, and Matter reference frontends.
- Install and tune from the browser: the web suite at [espectre.dev](https://espectre.dev/tools/) covers firmware flashing, Wi-Fi provisioning, device configuration, live monitoring, and detection tuning. Controls adapt to each device's supported capabilities.
- Use one CLI throughout the device lifecycle. `./espectre` builds, flashes, provisions, deploys, discovers devices, and sends control requests. It also provides serial monitoring, diagnostics, raw CSI collection, and an interactive MQTT client.
- Choose the detector for your product: Lightweight keeps CPU and memory costs low; High Accuracy uses a neural model and starts without quiet-room calibration. Both report motion on the same `0.0-1.0` probability scale.

### SDK, firmware, and tools

- The SDK has two public entry points. `espectre_core_sdk.h` exposes the detectors and temporal sampler for products that already own CSI capture. `espectre_sdk.h` adds sensing, calibration, events, and the ESP-IDF runtime backend. The documented contract covers source compatibility, capabilities, ownership, threading, errors, optional components, and versioned release bundles. Logging is optional and frontend-owned, so the shared SDK does not depend on `esp_log` and remains silent unless the integration registers a sink.
- Lightweight is the default detection profile, with a two-feature weighted model and startup calibration. High Accuracy uses an eight-feature phaseless neural model and starts from a trained threshold. Both use gain- and scale-invariant production features, so maintained firmware keeps AGC enabled.
- One versioned application API spans Direct HTTP, SSE, and optional MQTT. HTTP and MQTT share resource payloads, operation names, validation, and result codes. Direct resources live below `/espectre/v1` on port `62587`, and `GET /events` carries resource snapshots and per-evaluation motion events. Native publishes its supported resource snapshots and control operations over MQTT; `health` provides retained availability and Last Will, while motion events remain unretained. Capability profiles declare the resources, operations, events, and features supported by each frontend.
- Local discovery now uses one first-party DNS-SD service across every networked frontend. Devices advertise `_espectre._tcp.local.` with their Direct endpoint and protocol metadata, and `./espectre devices` validates that record instead of relying on ESPHome or Matter service schemas. The hosted portal can discover devices without an extension, cloud relay, or address-range scan: it resolves a fresh nonce-scoped `.local` hostname, then an eligible Native, ESPHome, or Matter device performs the bounded browse and returns validated endpoints. Browser discovery requires IPv4, working mDNS, and Local Network Access.
- Native is standalone ESP-IDF firmware with Direct HTTP sensing and HTTPS OTA updates. MQTT and Home Assistant MQTT Discovery are optional. It uses standard Improv Serial for provisioning and has no BLE dependency.
- Matter adds a firmware frontend for the standard occupancy sensor device type. Each device generates its own onboarding data, and its QR code remains available from serial output, Improv Serial, the web flasher, or `./espectre matter qr`. Sensing starts only after commissioning. Direct stores a BSSID preference for the commissioned SSID without changing Matter-owned credentials. The hardware benchmark commissions the device with a CHIP Tool build that matches the firmware, scores both detectors, and does not retain onboarding data or fabric secrets. Current images use development VID/PID and example device attestation credentials, do not support Matter OTA, and have limited controller validation.
- Micro-ESPectre is a micropython, read-only sensing frontend. The deployed application uses Lightweight Detector, the shared managed traffic generator with deployment-time `ping`, `dns`, or `dns_tcp` selection, six read-only HTTP resources plus session-only recalibration, one motion SSE stream, mDNS discovery, and serial logging. Its profile enforces production Origin validation, a 10-second SSE heartbeat, a 512-byte request limit, a 4,096-byte event limit, one client, and one frame in flight. The High Accuracy ML model remains available in Python for host research and C++/Python validation but is not copied to the device.
- Temporal CSI sampling replaces packet-count windows. Packet timestamps define a stable slot grid that retains the candidate nearest each slot and leaves missing slots empty. Traffic generators avoid catch-up bursts, detector resets do not rephase the sampling grid, and ESP-IDF frontends expose explicit `ping`, DNS/UDP `dns`, and persistent `dns_tcp` sources. Ping remains the shared schema default and the published product configuration default.
- The runtime selects the CSI capture profile for each chip and band. ESP32-C5 defaults to automatic 2.4/5 GHz selection and can be pinned to either band, using HT20 on 2.4 GHz and VHT20 on 5 GHz while excluding HE capture. Detection quality on 5 GHz has not yet been characterized. ESP32 and ESP32-S2 select LLTF20 to admit legacy OFDM traffic while preserving the canonical centered 64-bin raw and detector views; other 2.4 GHz targets continue to use HT20.
- CSI collection runs over a single HTTP response across supported ESPectre frontends. `GET /espectre/v1/csi` starts the exclusive collection and TCP close ends it, without a bearer or explicit start/stop commands. `./espectre collect` starts the shared external UDP marker generator before opening the response and saves CSI V8 records with PHY and device provenance. Derived sensing events pause during collection and resume only after sensing is ready. HTTP does not pace or decimate records, and bounded firmware queues report drops explicitly.
- ML corpus curation and promotion keep selection and holdout distinct. New captures require explicit environment and dataset-role metadata before validation. `train_ml_model.py --evaluate-selection` runs candidate deployment gates without opening holdout, while `--evaluate-gates` remains the final read-only check for a fixed candidate.
- The web tools provision Native and ESPHome over standard Improv Serial, configure and monitor supported C++ frontends, manage Native MQTT and OTA settings, and read Matter onboarding data. The suite also includes a motion game and Wi-Fi Theremin. USB flashing, provisioning, and Matter QR reading require a Chromium-based browser. ESPHome images retain their fallback access point but no longer include BLE provisioning.
- Release downloads include factory and OTA images for ESPHome and Native, factory images for Matter, SDK bundles, and `firmware-compliance-<channel-or-version>.zip`. Commercial licensing covers the shared engine and eligible integrations; the ESPHome frontend remains GPLv3-only. Contributions are subject to CLA and DCO checks. Native OTA accepts only a strictly newer release, prerelease, or rolling `git describe` identity.

### Breaking changes and migration

- The unreleased Direct RPC contract is replaced without aliases. Replace `POST /espectre/v1/request` with the resource methods documented in `API.md`; replace telemetry with `motion`; replace explicit raw-session start, bearer-bound `/csi`, and stop with a single `GET /csi` response lifetime; and update mDNS consumers to use `path=/espectre/v1` without an `events` TXT field. MQTT clients must use the retained resource topics and consolidated `update_device`, `update_sensing`, `recalibrate`, `read_diagnostics`, `check_ota`, and `start_ota` commands.
- Detector identifiers, C++ names, and the metric scale changed without compatibility aliases. Replace `mvs` with `lightweight`, `ml` with `high_accuracy`, `MVSDetector` with `LightweightDetector`, `MLDetector` with `HighAccuracyDetector`, and the corresponding `DetectionAlgorithm::MVS` / `ML` values with `LIGHTWEIGHT` / `HIGH_ACCURACY`. Update movement and threshold integrations from the former `0–10` assumptions to the shared `0.0–1.0` probability scale.
- The repository command wrapper is now `./espectre`. Run MicroPython commands under `./espectre micro`, and run `collect` and `mqtt` from the repository root. Replace the former `micro-espectre/me` `ui`, `detect`, `stream`, and collection workflows with the browser tools, the MQTT client, or `./espectre collect` as appropriate. Raw collection now uses HTTP, and `--pps` controls the external generator.
- ESPHome configuration now follows the temporal sampler. Replace `segmentation_window_size`, `evaluation_interval`, and `traffic_generator_rate` with `segmentation_window_size_ms`, `evaluation_interval_ms`, a positive `csi_target_pps`, and an explicit `csi_traffic_mode`. Drop `segmentation_threshold`, `gain_lock`, `selected_subcarriers`, the configurable publish interval, and the legacy BLE channel settings `ble_channel_enabled`, `ble_server_id`, `ble_control_char_id`, `ble_sysinfo_char_id`, `ble_telemetry_char_id`, and `ble_telemetry_interval_ms`. Use Improv Serial for USB provisioning.
- Micro-ESPectre is now read-only at runtime. MQTT transport and commands, device-side High Accuracy deployment, and the UDP CSI streamer are removed. Flash the matching project firmware once, keep Wi-Fi settings in `config_local.py`, use `micro deploy` for `.mpy -O3` application updates, and monitor through Direct HTTP, SSE, or serial output.
- Micro-ESPectre now uses `espectre-<device_id>.local`. Update manually saved endpoints or rediscover the device; the former `espectre-micro-<suffix>.local` hostname has no compatibility alias. Its `device` resource and mDNS TXT record now use the ESP-IDF application version and target chip.
- The supported build baseline changed. ESPHome requires at least `2026.7.0`, host and ML workflows require Python `3.14`, ESP-IDF integrations require ESP-IDF `>= 5.5`, and PlatformIO-backed builds are no longer supported.
- Dataset metadata moved from format `1.0` to `1.2`. Consumers of 2.8.0 `.npz` captures must migrate to the versioned metadata schema.
- C++ integrations must move to the supported SDK facade. Include `espectre_sdk.h`, keep dependencies flowing from frontend to runtime to core, and update code that depends on the former `components/espectre/` layout or namespace.
- ESPHome examples and firmware names changed. Replace repository paths beginning with `examples/` with `src/cpp/frontend/esphome/examples/`, including package URLs that reference an example YAML file. Full-flash images now use `espectre-esphome-<channel-or-version>-<chip>.bin`, and OTA images use `espectre-esphome-<channel-or-version>-<chip>-ota.bin`.

---

## [2.8.0] - 2026-05-21 - Detection hardening, ML cross-chip reliability, and runtime motion policy

- Hardened detection and calibration across stacks with tighter NBVI defaults, Hampel enabled by default, a 100-packet default window, and a clearer edge-driven motion policy.
- Improved ML reliability across chips with shared CV-normalized turbulence, a refreshed 9-feature model, and stricter training/data quality controls.
- Made `ping` the default CSI traffic source, added `./me detect` for live ML inference, and expanded notebooks and CI/test coverage.

---

## [2.7.0] - 2026-03-17 - ESPectre configuration over BLE and subcarrier normalization

- Added BLE runtime control as a first-class standalone integration surface, including live threshold updates and a Web Bluetooth example client.
- Extended CSI normalization to `256->128`, `228->114`, and `114->128` payload remaps, with aligned behavior and tests across C++ and Micro-ESPectre.

---

## [2.6.0] - 2026-03-08 - ESP32-C5 support, context-aware calibration, and stricter validation targets

- Added ESP32-C5 support and hardened runtime handling on newer chips (`C5`/`C6`).
- Aligned calibration, thresholds, dataset metadata, and ML feature selection more strictly across C++ and Micro-ESPectre.
- Tightened validation targets to `Recall >95%` and `FP <5%` and improved the related tooling and deploy diagnostics.

---

## [2.5.1] - 2026-02-23 - HT STBC multi-antenna router fix

- Fixed HT STBC CSI handling on ESP32-C5/C6 with multi-antenna routers by accepting 256-byte packets and using the first HT20 estimate.
- Fixed Micro-ESPectre NBVI calibration memory issues on ESP32-C3, improved calibration speed, and refreshed performance/snapshot documentation.

---

## [2.5.0] - 2026-02-15 - ML detector, training pipeline, and pre-built firmware

- Added the first experimental ML detector in both ESPHome/C++ and Micro-ESPectre/Python, with a training and weight-export pipeline.
- Added pre-built firmware releases for all supported ESP32 variants.
- Removed the PCA detector and the older P95 calibrator, leaving MVS plus NBVI as the main non-ML path at the time.

---

## [2.4.0] - 2026-01-24 - Live recalibration, adaptive threshold, and PCA

- Added live recalibration, adaptive thresholds by default, and a choice between MVS and experimental PCA detection.
- Standardized the runtime around HT20 CSI, improved calibration/subcarrier handling, and expanded tooling, tests, and Micro-ESPectre support.

---

## [2.3.0] - 2025-12-31 - End-of-year edition

- Added `ESPectre - The Game`, a browser-based motion-controlled tuning and demo client.
- Added sensor customization, external traffic mode, `ping` traffic generation, and configurable gain-lock behavior.
- Improved channel-change handling, NBVI calibration, and board support, including tested ESP32-C3 and original ESP32 paths.

---

## [2.2.0] - 2025-12-19 - Gain lock, low-pass filter, and ML data collection

- Added gain-lock stabilization, low-pass filtering, and baseline variance normalization to make calibration more stable.
- Tightened NBVI behavior, moved variance evaluation to publish time, and auto-configured the required ESP-IDF options in the ESPHome path.
- Added the first labeled ML data-collection infrastructure (`me collect`, `.npz`, and `csi_utils.py`) plus broader testing/documentation.

---

## [2.1.0] - 2025-12-10 - Made for ESPHome compliance

- Updated all example configs to meet "Made for ESPHome" requirements, including provisioning, dashboard import, and project metadata.
- Unified and optimized variance and Hampel behavior across C++ and MicroPython.
- Expanded the test suite and coverage pipeline substantially.

---

## [2.0.0] - 2025-12-06 - ESPHome native integration

- Migrated the platform from standalone ESP-IDF firmware to an ESPHome native integration for Home Assistant.
- Established the dual-platform model: ESPHome/C++ for production, and Micro-ESPectre/MicroPython for R&D and rapid experimentation.
- Moved tests and CI to the ESPHome-oriented workflow with host-side CMake/CTest coverage.

---

## [1.5.0] - 2025-12-03 - Automatic subcarrier selection

- Added zero-configuration subcarrier selection using the Normalized Baseline Variability Index (NBVI) algorithm.
- Calibrated automatically at boot and after `factory_reset`.
- Defined NBVI as `NBVI = 0.3 × (σ/μ²) + 0.7 × (σ/μ)`.
- Achieved F1 97.6%, recall 95.3%, precision 100%, and FP 0%.

---

## [1.4.0] - 2025-11-28 - Major refactoring and technical debt reduction

- Extracted feature calculation into `csi_features.c/h`, reducing `csi_processor.c` by 50%.
- Centralized defaults in `espectre.h` and validation in `validation.h/c`.
- Replaced variance calculation with a numerically stable two-pass implementation.
- Increased the traffic generator maximum from 50 to 1000 pps, with a default of 100 pps.
- Migrated the CLI from Bash to Python for cross-platform use.
- Added `tools/web/espectre-theremin.html` for CSI sonification.
- Removed the redundant `min_length`, `max_length`, and `k_factor` segmentation parameters.

---

## [1.3.0] - 2025-11-22 - ESP32-C6 platform support

- Added Wi-Fi 6 (`802.11ax`) support and the corresponding CSI configuration.
- Made `threshold` and `window_size` configurable at runtime through MQTT.
- Added `tools/web/espectre-monitor.html` for real-time visualization.
- Added CPU and RAM usage to the `stats` command.
- Simplified the MQTT message format and removed segment tracking.

---

## [1.2.1] - 2025-11-17 - Wi-Fi optimization

- Applied ESP-IDF Wi-Fi practices by disabling power saving (`WIFI_PS_NONE`), making the country code configurable, and setting HT20 bandwidth.

---

## [1.2.0] - 2025-11-16 - Simplified architecture and MVS segmentation

- Added Moving Variance Segmentation (MVS) with an adaptive threshold.
- Switched to amplitude-based features, improving separation for skewness and kurtosis by 151%.
- Replaced UDP broadcast traffic generation with ICMP ping.
- Used all 64 available subcarriers instead of filtering the set to 52.
- Added `temporal_delta_mean` and `temporal_delta_variance`, bringing the feature count to 10.

---

## [1.1.0] - 2025-11-08 - Automatic calibration system

- Used Fisher's criterion to select four to six features automatically from a set of eight.
- Applied a fourth-order Butterworth filter with an 8 Hz cutoff.
- Added a Daubechies `db4` wavelet filter for high-noise environments.
- Persisted configuration in NVS across reboots.
- Split the implementation into 10 specialized modules.

---

## [1.0.0] - 2025-11-01 - Initial release

- Added CSI-based movement detection for ESP32-S3 with Hampel and Savitzky-Golay filters, 15 features, four detection states (`IDLE`, `MICRO`, `DETECTED`, and `INTENSE`), MQTT publishing, and a CLI tool.
- Supported traffic rates from 10 to 100 pps, latency below 50 ms, and a range of 3 to 8 m.
