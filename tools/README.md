# Analysis And Benchmark Tools

This directory contains host-side Python tools for CSI inspection, dataset validation, detector research, model training, and firmware benchmarking. It is written for contributors who already understand the basic ESPectre workflow; start with [ML_DATA_COLLECTION.md](../docs/ML_DATA_COLLECTION.md) if you need to collect data, or [ALGORITHMS.md](../docs/ALGORITHMS.md) if you need the detector concepts first.

Run commands from the repository root through the project virtual environment. Use `python tools/<tool>.py --help` for the complete and current option reference; this README explains which tool to choose and the recommended mainline workflows.

## Common Terms

- **CSI:** channel state information, the per-packet radio-channel measurement used by the detectors.
- **Replay:** running recorded CSI through production-aligned feature or detector logic.
- **Candidate:** a research-only feature, model, or detector configuration that has not been promoted to production.
- **Gate:** a validation requirement that must pass before generated runtime artifacts can change.
- **OOF:** out-of-fold metrics, computed on data excluded from the fold that fitted the model.

## Prerequisites

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Install `requirements-ml.txt` only for training and ML-specific analysis. When upgrading an existing ML environment, use `python -m pip install -r requirements-ml.txt` so pip resolves NumPy, Numba, SHAP, and PyTorch together. The main repository workflow targets Python `3.14`.

The tools support the original ESP32, ESP32-C3, ESP32-C5, ESP32-C6, ESP32-S2, and ESP32-S3 when the required datasets or connected hardware are available. A missing dataset or hardware report does not mean that a chip is unsupported.

## Tool Index

| Tool | Use it when you need to |
|---|---|
| `analyze_raw_data.py` | inspect registered CSI pairs and basic signal quality |
| `analyze_system_tuning.py` | grid-search Lightweight parameters on the fixed production band |
| `compare_detection_methods.py` | compare RSSI, Lightweight, and ML behavior on recorded data |
| `compare_chips.py` | compare CSI characteristics across available chip datasets |
| `plot_constellation.py` | visualize I/Q samples by subcarrier |
| `plot_heatmap.py` | render time-by-subcarrier CSI amplitude heatmaps |
| `validate_dataset_quality.py` | validate metadata, files, signal quality, pair consistency, and training readiness |
| `train_ml_model.py` | train, evaluate, and conditionally export the production ML model |
| `generate_performance_report.py` | regenerate the aggregate detector performance report and run its parity checks |
| `benchmark_firmware.py` | build, flash, monitor, and report representative live firmware cases |
| `analyze_seed_dispersion.py` | measure replay-metric variation across training seeds |
| `compare_reserved_selection.py` | compare one candidate on reserved selection roles with an explicit seed |
| `benchmark_subcarrier_aggregation.py` | evaluate adjacent-subcarrier aggregation as a host-side experiment |
| `sweep_occupancy_floor.py` | replay reserved pairs while thinning admitted CSI; `--always-evaluate` keeps occupancy holes scored |
| `benchmark_lightweight_candidate_pairs.py` | screen Lightweight features and combinations without threshold coupling |
| `test/cpp/support/benchmark_lightweight_iqr_resources.cpp` | compare host C++ RAM and hot-path cost for the normal- and aggregated-IQR Lightweight finalists |
| `test/cpp/support/benchmark_detector_resources.cpp` | measure current production Lightweight and High Accuracy host memory, packet cost, inference latency, and nominal CPU load |
| `replay_lightweight_candidates.py` | fit and replay research-only Lightweight candidates end to end |
| `fit_lightweight_detector.py` | fit production Lightweight coefficients and optionally apply an approved result |
| `prune_npz_cache.py` | remove cached analysis artifacts whose sources or implementation dependencies are no longer current |
| `espectre_traffic_generator.py` | send phase-paced, DSCP 46 unicast or local-link multicast UDP traffic to devices in `csi_traffic_mode: external` |

## Dataset Inspection And Validation

Generated quality reports record their chip filter. `--check-current` requires the same `--chip` scope as the generating run; a filtered report is not current for a full-corpus check. Reports created before scope tracking must be regenerated. A filtered run still replaces the default report, so use `--report-output` to retain a separate copy.

Use lightweight inspection before changing a detector:

```bash
python tools/analyze_raw_data.py
python tools/analyze_raw_data.py --chip C6
python tools/compare_detection_methods.py --chip C6 --plot
```

Run the quality validator before training or publishing dataset conclusions:

```bash
python tools/validate_dataset_quality.py
python tools/validate_dataset_quality.py --chip C6
python tools/validate_dataset_quality.py --no-report
python tools/validate_dataset_quality.py --check-current
python tools/validate_dataset_quality.py --data-dir data/untracked/example --preserve-pairs
python tools/validate_dataset_quality.py --data-dir data/untracked/example --diagnostic-all-phy
```

The validator checks catalog metadata, NPZ integrity, CSI shape, packet timing, stream continuity, explicit pair consistency, quiet recordings, and ML readiness. It refreshes derived pair metadata and normally regenerates `data/auto_generated/DATASET_QUALITY_CHECK.md`. Use `--data-dir` for a standalone ESPectre-format corpus; its report defaults to `<data-dir>/auto_generated/DATASET_QUALITY_CHECK.md`. Add `--preserve-pairs` when the external catalog already contains deliberate reciprocal pairs and its timestamps must not drive automatic re-pairing. `--report-output` overrides the generated report path when needed. `--diagnostic-all-phy` evaluates all explicitly tagged PHY rows while retaining the supported HT20/HT-LTF contract failure in the report; it is for external holdouts and never admits those rows into production training.

`dataset_role` remains a manual curation decision. The trainer treats a missing role as excluded, but the validator requires every catalog entry to declare its role explicitly, including `exclude`. It never promotes a recording to `train`, `selection`, or `holdout`. Review scores are diagnostic and do not replace admission gates: mean valid-slot occupancy warns below 85%, fails admission below 70%, and caps every affected score. Excluded idle captures that produce no usable feature rows after temporal admission are listed first in the generated report with `n/a ⚠️`. The generated report owns the detailed tables and definitions.

## ML Training

`clipped_standard` is available for host-side CV with `--no-export`. Runtime replay and export require an affine scaler (`standard`, `robust`, or `session_balanced_robust`), because the runtime model contract does not carry clipping bounds.

Read [ML_TRAINING.md](../docs/ML_TRAINING.md) before running a promotion workflow. Use this promotion sequence:

```bash
python tools/train_ml_model.py --info
python tools/train_ml_model.py --augment --no-export
python tools/train_ml_model.py --augment --seed SEED --evaluate-selection
python tools/train_ml_model.py --augment --seed SEED
```

`--no-export` runs CV without replacing runtime weights or opening deployment replays. `--evaluate-selection` evaluates a candidate against selection while keeping holdout sealed. A normal production run opens selection and holdout, then exports only after its promotion gates pass. Host-only candidate features remain research artifacts even when their metrics improve; use [FEATURES.md](../docs/FEATURES.md) to record the evidence and promotion status.

Use an explicit seed and the same corpus, roles, preprocessing, features, and augmentation when comparing two changes. Seed searches, cross-environment checks, cross-chip checks, gain stress, ablations, and feature-importance modes are advanced workflows documented by `--help` and [ML_TRAINING.md](../docs/ML_TRAINING.md).

## Generated Performance Report

`generate_performance_report.py` publishes Lightweight and High Accuracy replay tables only for the combined `selection + holdout` corpus, executes the current production C++ resource microbenchmark, and runs the host-side C++/Python parity checks before writing the report. Training-role recordings remain covered by the validation suites but are neither replayed nor summarized by the report generator. Detector replay summaries, augmented rows, and training matrices use the shared `.cache/npz`; a warm generation reuses them instead of replaying the corpus. Its augmentation diagnostic applies the production two-seed packet recipe to the same reserved pairs and compares Lightweight and High Accuracy on matching alternating replay positions; it never reads augmented training rows.

```bash
python tools/generate_performance_report.py
python tools/generate_performance_report.py --check-current
python tools/generate_performance_report.py --stdout
python tools/generate_performance_report.py --data-dir data/untracked/example
python tools/generate_performance_report.py --data-dir data/untracked/example --diagnostic-all-phy
```

Do not edit `docs/performance/README.md` manually. `--check-current` is a lightweight input-revision check; a normal warm regeneration measures resources again, loads the cached replay summary and robustness artifacts, runs parity, and renders the report. A replay-summary miss rebuilds only from the lower-level row cache and never starts ML training. `--data-dir` writes an external holdout report to `<data-dir>/auto_generated/PERFORMANCE_REPORT.md` by default and skips primary-corpus resource, augmentation, and C++ parity sections. Add `--diagnostic-all-phy` only for explicitly tagged external views such as LLTF or HT40; the report records that non-production evaluation view.

## Firmware Benchmark

### Firmware Benchmark Contract

The following rules are normative for `benchmark_firmware.py` and its owners under `tools/lib/firmware_benchmark/`:

- Build the canonical frontend configuration in its ordinary build directory, retain production defaults, and allow the normal incremental build system to reuse valid artifacts. Do not generate benchmark-specific YAML, sdkconfig overlays, or dedicated build directories, and do not force a clean build.
- Clear all device data as part of the frontend's normal `flash --erase` operation. Do not add partial-erasure exceptions.
- Provision Wi-Fi through standard Improv Serial on Native and ESPHome. Matter's read-only Improv surface exposes firmware identity and onboarding data but does not provision Wi-Fi; commission Matter through a revision-compatible CHIP Tool controller over BLE and Wi-Fi. Micro-ESPectre may inject only connectivity settings because it does not support Improv Serial.
- Apply and verify the optional target BSSID through Direct where supported. Send `force=true` so the setup exercises the Wi-Fi transition even when the device is already associated with that access point. Require the successful Direct acknowledgement, including its pre-apply `current_bssid`, before reconnecting; a timeout, reset, dropped response, failed association check, or observed device restart fails the benchmark.
- Use Direct responses, diagnostics, and events for runtime configuration, validation, and metrics. Use serial output only to detect fatal firmware errors, unexpected resets, or an unexpectedly terminated monitor.
- Reuse one keep-alive Direct control connection during readiness and scored sampling so the harness does not perturb lwIP socket and packet-buffer pressure. Non-persistent TCP timing is an explicit diagnostic opt-in, not the canonical matrix workload.
- Retry only GET requests after a persistent-connection transport failure. Never resend a mutation automatically after losing its response.
- Use one SSE connection per detector from readiness through scored sampling. Close it after the detector window, wait until the device releases the connection, and open a distinct connection for the next detector.
- Reuse capabilities and device identity within the same frontend session. Read fresh health, sensing, Wi-Fi, and diagnostics before each detector, apply only the required sensing changes in one PATCH, and confirm only the changed sensing resource. Reconnects and new firmware sessions require a fresh device identity check.
- During readiness, read sensing once and consume its complete SSE snapshots when the frontend advertises sensing events and the stream is active; otherwise, keep polling sensing. Always retain the one-second diagnostics cadence, consecutive-ready-sample requirement, and reboot checks. Poll for SSE teardown every 250 ms after scoring. The scored health and diagnostics request workload stays unchanged.
- Verify production runtime defaults before applying case-specific mutations. Reuse one canonical Lightweight image and select another supported detector through Direct instead of rebuilding it.

Behavioral tests under `test/python/host/benchmark/test_benchmark_*.py` are the executable enforcement of this contract. Generated performance reports describe individual runs and are not contract owners.

`benchmark_firmware.py` operates on one connected chip and writes its generated report under `docs/performance/`. The representative matrix is:

1. Native Lightweight
2. Native High Accuracy by runtime switching of the same Native firmware
3. ESPHome Lightweight
4. ESPHome High Accuracy by runtime switching of the same ESPHome firmware
5. Matter Lightweight
6. Matter High Accuracy by runtime switching of the same Matter firmware
7. Micro-ESPectre Lightweight

The numbered cases are a representative matrix for supported frontend and chip combinations. ESPHome, Native, and Matter support persisted runtime switching between Lightweight and High Accuracy; Micro-ESPectre deploys Lightweight only. ESP32-S2 is included for ESPHome, Native, and Micro-ESPectre, but not for Matter.

The benchmark reads laboratory settings from `tools/benchmark_firmware.local.env`, with exported `ESPECTRE_BENCHMARK_*` variables taking precedence.

Native, ESPHome, and Matter build their canonical configurations without benchmark-specific firmware overrides. Their flash command clears all device data; Native and ESPHome provision through Improv Serial, while Matter commissions through CHIP Tool. To exercise BSSID reassociation, set `ESPECTRE_BENCHMARK_WIFI_BSSID` to the target access point. The benchmark applies it once with `force=true`, uses the acknowledgement's `current_bssid` as pre-apply evidence, and verifies the resulting association. When `ESPECTRE_BENCHMARK_WIFI_CHANNEL` is set, it verifies the target access point on that channel; a channel without a BSSID is rejected before hardware access. A transport-level SSE loss while the staged reassociation is in progress is expected setup activity; malformed or incompatible SSE data and stream-close timeouts remain failures, as does any SSE failure during a scored window. A missing acknowledgement, failed association check, or observed uptime regression fails the benchmark. The detailed report retains the apply, reassociation, and association evidence, while the summary omits redundant setup columns.

Native, ESPHome, and Matter reuse one flashed canonical Lightweight image and select both scored detectors through Direct.

Matter reads the firmware-generated onboarding data after erase and commissions through CHIP Tool with a temporary controller store created for the run. The read-only Matter Improv RPC does not change this provisioning boundary or replace controller commissioning. The benchmark does not write the Wi-Fi password, onboarding payload, setup code, or fabric keys to its artifacts. CHIP Tool must come from the same `connectedhomeip` revision declared by the firmware's `esp-matter` component. Set `ESPECTRE_BENCHMARK_CHIP_TOOL` when it is not available as `chip-tool` on `PATH` or under `~/.local/bin/`. By default, Matter commissioning retries transient BLE failures twice; change the limit with `ESPECTRE_BENCHMARK_MATTER_COMMISSIONING_ATTEMPTS`. On macOS, install Apple's Bluetooth Central Matter Client Developer Mode profile and restart the host before commissioning.

Micro-ESPectre clears the flash, copies only the laboratory connectivity settings into an isolated temporary `config_local.py`, retains every production sensing default, and connects Direct through the Wi-Fi address reported by its serial launcher. Copy `tools/benchmark_firmware.local.env.example` to `tools/benchmark_firmware.local.env`, fill in the laboratory values required by the selected frontends, connect the target board, and run:

```bash
python tools/benchmark_firmware.py --chip c3 --port /dev/cu.usbmodem01
```

The benchmark requires `--port`, passes that value unchanged to every delegated command, and never performs serial discovery or reset recovery. It always passes `--chip` to `./espectre` and delegates canonical config selection, build artifact resolution, chip verification, full-data erasure during flash, provisioning, onboarding reads, and monitoring to the repository CLI. Delegated build, provisioning, Matter onboarding, and Micro Direct-ready events use final machine-readable JSON objects rather than human-output parsing. Flashing is one esptool operation owned by the CLI, including optional full erase, verified writes, and the post-write reset; a non-zero CLI exit status is final. In an interactive terminal, the benchmark pauses immediately before each ESP32-S2 USB CDC C++ flash so the operator can place the device in download mode; Micro-ESPectre requires the same manual preparation before its flash, and non-interactive runs require the device to be prepared in advance. Matter is omitted automatically for ESP32-S2 because the supported commissioning flow requires Bluetooth.

Use `--duration SECONDS` for a longer scored window, such as a five-minute Micro-ESPectre heap soak:

```bash
python tools/benchmark_firmware.py --chip c3 --port /dev/cu.usbmodem01 --frontend micro --duration 300 --update
```

Use `--resume` to keep passing results from the chip report and rerun only failed or missing cases. Optional frontend and detector filters limit which failed or missing cases are retried:

```bash
python tools/benchmark_firmware.py --chip c3 --port /dev/cu.usbmodem01 --resume
```

The command writes a partial report when a case fails, stops immediately after the first failed case, and returns success only when every case in the resulting report passes. Native, ESPHome, and Matter refresh that report and their structured artifacts after each detector completes, while retaining their shared build, flash, provisioning, and serial-monitor session. The benchmark does not retry a failed flash, reset the board independently, or power-cycle USB hardware. It stores normalized Direct samples and events, transport outcomes, firmware hashes, structured analysis, and a run manifest under `data/untracked/firmware_benchmarks/<run-id>/`. Runtime artifacts exclude raw serial output, raw Direct payloads, credentials, onboarding data, fabric keys, device identity, and local addresses.

Every sensing case waits for five consecutive ready, non-zero Direct diagnostics samples before scoring. Native, ESPHome, and Matter must first report the production defaults through Direct: Lightweight detection, the managed-traffic mode configured by that frontend and target, and a 100 pps target. Published product configurations currently start from `ping`. The benchmark changes only the detector needed by the scored case. Micro confirms its fixed production profile through Direct.

Heap-decline scoring begins 10 seconds into the scored window and requires two complete consecutive 10-second windows; a duration too short to provide both windows fails the case instead of falling back to endpoint samples. Device uptime must remain monotonic, Direct transport failure counters must not increase when available, detector timing must be present, and telemetry events are collected from Direct SSE. Micro diagnostics are sampled every 4.5 seconds and allow one second of gap tolerance for the device's one-second cached-snapshot cadence; the C++ frontends retain their 500 ms tolerance. Native, ESPHome, and Matter use the ordinary frontend build directories and incremental build behavior, so an existing valid firmware build is reused. Micro-ESPectre keeps generated `sdkconfig` when the board Kconfig inputs are unchanged. Serial logs are retained only to fail a case on fatal firmware errors, resets, or an unexpectedly terminated monitor; they are never a source of runtime metrics.

The manifest and generated report record the starting and ending Git revisions, worktree states, and firmware-source fingerprints. A revision change during the run invalidates every executed case; a source-fingerprint change on the same revision is reported as a warning without invalidating results. Each report therefore identifies its source state, hardware, environment, and run time; it does not certify later source revisions. Do not edit or reformat generated chip reports separately from a hardware benchmark run.

Expected sample counts tolerate one sample at the scored-window boundary. Direct cadence uses host-monotonic receive times and device uptime for every sensing frontend; any real gap over the cadence tolerance fails the case.

When `--update` or `--resume` preserves cases from an existing report, the report header identifies the updating run, not the provenance of every preserved case. Use the matching per-run artifact directory for the exact revision, duration, and structured evidence of each executed case.

## Research-Only Detector Experiments

The candidate tools answer different questions:

| Question | Tool |
|---|---|
| Does one feature or combination separate paired states before threshold tuning? | `benchmark_lightweight_candidate_pairs.py` |
| Does a candidate survive causal calibration, clean replay, and packet stress? | `replay_lightweight_candidates.py` |
| Does adjacent-bin aggregation change channel statistics or detector behavior? | `benchmark_subcarrier_aggregation.py` |
| How much does a metric move across training seeds? | `analyze_seed_dispersion.py` |
| Does a selected ML candidate survive the reserved selection roles? | `compare_reserved_selection.py` |

These tools are diagnostic by default and must not write production runtime artifacts. Their durable conclusions belong in [FEATURES.md](../docs/FEATURES.md); detector formulas belong in [ALGORITHMS.md](../docs/ALGORITHMS.md). Use an ADR only for a durable architectural or project-level decision.

Candidate replay uses the canonical dataset roles: an empty or missing role means `exclude`, including for empty-room recordings. Such recordings remain available as excluded diagnostics and are never admitted as training negatives by `--include-train-empty`.

Replay independent catalogs only after fitting and primary-corpus ranking. Repeat `--external-data-dir` for multiple sealed holdouts; add `--external-diagnostic-all-phy` only for a matching external catalog whose explicit non-production PHY rows are intentionally being evaluated as diagnostics:

```bash
python tools/replay_lightweight_candidates.py \
  --features turb_autocorr,turb_iqr_over_mean_aggr,chan_shape_excess_path \
  --stress-augment \
  --external-data-dir data/untracked/csi_sense_zero \
  --external-data-dir data/untracked/wisdom_lab \
  --external-diagnostic-all-phy data/untracked/wisdom_lab
```

`fit_lightweight_detector.py --apply` and ML export are deliberate promotion actions. Run the required real-data, long-recording, packet-rate, and C++/Python parity gates before applying their output.

## Visual Analysis

```bash
python tools/plot_constellation.py --chip S3 --packets 1000 --grid
python tools/plot_heatmap.py --chip S3 --environment bedroom --detrend
python tools/compare_chips.py --plot
```

Plots help diagnose signal structure; they do not establish detector quality by themselves. Use grouped replay metrics and the maintained gates for production conclusions.

## Cache Maintenance

Training and replay tools share a persistent NPZ cache. Normal runs validate cache provenance automatically. Runtime-supported features use complete replay matrices; host-only experiments use one row-spine artifact plus one column artifact per feature.

Adding a variant to an existing provider family leaves sibling columns valid, so later model comparisons compute only columns that are actually missing. Reordering or selecting a subset reads the same columns without rebuilding packet rows.

Cold producers serialize on a per-key process lock and recheck the cache after acquiring it. Long fills emit periodic `[npz-cache]` lines on stderr for hits, misses, in-progress builds, and writes when stderr is a TTY; `ESPECTRE_NPZ_CACHE_PROGRESS=0` disables that output, `=1` forces it, and `ESPECTRE_NPZ_CACHE_PROGRESS_INTERVAL_S` overrides the default `10` second heartbeat.

Pruning removes only artifacts that can no longer be used because their capture, implementation dependencies, layout, or artifact version changed. Historical but still reachable feature columns remain until an explicit age or size policy is requested:

```bash
python tools/prune_npz_cache.py
python tools/prune_npz_cache.py --artifact ml_replay_rows
```

## Related Documentation

- [ML_DATA_COLLECTION.md](../docs/ML_DATA_COLLECTION.md): collection labels, metadata, and dataset roles
- [ML_TRAINING.md](../docs/ML_TRAINING.md): training, model selection, promotion, and export
- [ALGORITHMS.md](../docs/ALGORITHMS.md): production detector behavior
- [FEATURES.md](../docs/FEATURES.md): feature evidence and verdicts
- [performance/README.md](../docs/performance/README.md): generated detector metrics
- [CLI.md](../docs/CLI.md): supported repository entry points for collection and device workflows
