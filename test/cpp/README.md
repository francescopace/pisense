# Test Suite

Host-side CMake and CTest suite for the ESPectre `core`, `runtime`, and `frontend` layers.

## Quick Start

```bash
# Activate virtualenv (from repo root)
source .venv/bin/activate

# Configure and run the full host-side suite
./test/cpp/run_all_tests.sh

# Run specific suite
./test/cpp/run_all_tests.sh -R test_motion_detection

# Override automatic logical-CPU detection when needed
CTEST_PARALLEL_LEVEL=2 ./test/cpp/run_all_tests.sh
```

## Test Suites

The registered targets are grouped by the layer they exercise:

- Core: `test_utils`, `test_core_helpers`, `test_hampel_filter`, `test_lightweight_detector`, and `test_high_accuracy_detector`
- Runtime: `test_traffic_generator`, `test_runtime_helpers`, `test_frontend_sysinfo_helpers`, `test_periodic_sensing_status_logger`, `test_device_identity`, `test_ota_service_https`, `test_runtime_frontend_controller`, `test_sdk_surface`, `test_runtime_detector_switch`, `test_wifi_lifecycle`, `test_wifi_lifecycle_dual_band`, `test_pending_event`, `test_wifi_provisioning_service`, `test_device_config_store`, `test_espectre_protocol`, `test_csi_pipeline`, `test_csi_frame_identity`, `test_csi_traffic_service`, and `test_udp_listener`
- Integration with real CSI: `test_motion_detection`, `test_long_recordings`, `test_low_rssi`, `test_empty_rooms`, and `test_packet_rate_adaptation`
- Frontend: `test_sensor_publisher`, `test_frontend_controls`, `test_native_frontend_lifecycle`, `test_native_direct_frontend`, `test_native_mqtt_frontend`, `test_home_assistant_mqtt_frontend`, `test_native_frontend_ota`, `test_recovery_button_service`, and `test_matter_frontend`

`test/cpp/suites/CMakeLists.txt` is the executable registration source of truth; this list is the human-readable catalog.

The normal test runner, direct single-config CMake builds, and the performance-report parity gate use `RelWithDebInfo` so replay-heavy suites run with compiler optimizations while retaining debug information and test assertions. Coverage builds remain `Debug` so their line and branch mapping stays reliable.

The test, coverage, and performance-report launchers use every detected logical CPU by default. Set the standard `CTEST_PARALLEL_LEVEL` environment variable to a positive integer to limit concurrency on constrained hosts.

`test_packet_rate_adaptation` replays 60-second prefixes at 120, 100, and 80 pps through the fixed temporal-admission grid. Those rates exercise denser capture, the default target, and a lower explicit target; they are not an automatic supported-cadence floor.

### Dataset Test Matrix

CTest registers `normal`, `reserved`, `weak`, `empty`, `long`, and `packet_rate` cases for ESP32, C3, S2, S3, C5, and C6. Names use the form `test_<suite>.<gate>.<chip>`, and labels expose `performance`, `chip:<chip>`, and `dataset:<gate>` filters.

A case exits with code 77, configured through CTest's `SKIP_RETURN_CODE`, only when the catalog has no eligible dataset for that chip and gate. A malformed catalog, broken pair, missing file, or failed replay is a test failure. The aggregate executables remain available without arguments for local summaries and the C++/Python performance-report parity gate.

### Source Ownership

Each contract has one primary unit-test owner. `coverage_ownership.json` is the machine-readable registry, and the Python contract suite rejects missing, duplicated, or stale source owners. Integration and parity suites consume those contracts without restating their constants or implementation layout.

| Production source | Primary test owner | Separate integration or parity gate |
|---|---|---|
| `src/cpp/core/utils.*`, feature helpers, CSI format helpers, and `temporal_csi_sampler.*` | `test_utils`, `test_core_helpers` | `test_motion_detection` only for replay metrics |
| `src/cpp/core/hampel_filter.*` | `test_hampel_filter` | Detector replay suites run it without duplicating filter expectations |
| `src/cpp/core/lightweight_detector.*` | `test_lightweight_detector` | `test_motion_detection`, `test_long_recordings`, `test_low_rssi`, and `test_empty_rooms` |
| `src/cpp/core/high_accuracy_detector.*` and generated weights | `test_high_accuracy_detector` | `test_motion_detection`, `test_long_recordings`, and `test_empty_rooms` |
| Shared runtime contracts, policies, configuration, CSI pipeline, and protocol | The matching `test_runtime_*`, `test_device_config_store`, `test_espectre_protocol`, `test_csi_*`, or service suite | `test_packet_rate_adaptation` for quantified cadence behavior |
| Published SDK facade | `test_sdk_surface` | Python `test_sdk_surface_invariants.py` checks facade and documentation registration |
| ESPHome, Native, and Matter adapters | The matching frontend suite; shared controls stay in `test_frontend_controls` | Full firmware builds validate SDK-specific integration |

Add a regression to the named owner. Create a new suite only for a genuinely new subsystem or an integration boundary with distinct setup and failure semantics.

### Target Metrics (Motion Detection)

- **Recall**: >95% for all chips (detect motion in motion datasets)
- **FP Rate**: <5% for all chips (avoid false alarms)

See [docs/performance](../../docs/performance/README.md) for detailed targets per chip and algorithm.

### Performance Report Parity Gate

- `tools/generate_performance_report.py` now depends on the host-side C++ integration suites staying aligned with the published Python replay metrics.
- The report command configures and builds `test/cpp/build` as `RelWithDebInfo`, runs `test_motion_detection` and `test_long_recordings`, and compares their structured `selection + holdout` aggregate outputs against the Python report data before writing `docs/performance/README.md`. The suites still execute training-role recordings for regression coverage, but omit them from the report parity payload.
- If the paired or long-recording aggregates drift, the report generation fails and prints the mismatched chip/algorithm/metric entries instead of publishing stale documentation.
- `test_motion_detection` and `test_long_recordings` are parity metric producers. Their local assertions protect replay accounting and output structure; the numerical promotion targets are owned by `test_validation_real_data.py::TestPerformanceMetrics` and the generated report gate.

## Real CSI Data

Tests load real CSI data from NPZ files in `data/` using the [cnpy](https://github.com/rogersce/cnpy) library.

### Datasets

| Chip | Static Presence | Motion |
|------|-----------------|--------|
| ESP32-C3 | `static_presence_c3_64sc_*.npz` | `motion_c3_64sc_*.npz` |
| ESP32-C5 | `static_presence_c5_64sc_*.npz` | `motion_c5_64sc_*.npz` |
| ESP32-C6 | `static_presence_c6_64sc_*.npz` | `motion_c6_64sc_*.npz` |
| ESP32-S2 | Pending | Pending |
| ESP32-S3 | `static_presence_s3_64sc_*.npz` | `motion_s3_64sc_*.npz` |
| ESP32 | `static_presence_esp32_64sc_*.npz` | `motion_esp32_64sc_*.npz` |

The Python and C++ suites register all six supported chips and read the same eligible 64-subcarrier HT20 NPZ files. S2 remains in every applicable test gate and is reported as skipped until a testable dataset is catalogued.

## Code Coverage

Run the host-side suite with coverage instrumentation:

```bash
./test/cpp/run_coverage.sh
```

The native coverage script uses the host compiler and prints line, function, and branch coverage for the aggregate suite and for `core`, `runtime`, and `frontend`. On macOS, that normally means Apple Clang and `llvm-cov`.

Use the Docker-backed GCC 13 runner to reproduce the Linux/amd64 coverage toolchain used by CI:

```bash
./test/cpp/run_gcc13_coverage.sh --ci
```

CI compares those canonical GCC results with the fixed runtime gates in `coverage-thresholds.json`: 85% line coverage, 85% function coverage, and 50% branch coverage. These thresholds are deliberate policy and are not generated from the latest run.

The Docker image is built locally and reused through Docker's build cache. The runner uses `linux/amd64` explicitly so Apple Silicon hosts match the GitHub-hosted runner architecture.

## Project Structure

```
test/cpp/
├── cmake/              # Shared CMake modules for the host-side suite
├── mocks/              # ESP-IDF / ESPHome host-side fakes
├── suites/             # Test suites grouped by layer
│   ├── core/
│   ├── runtime/
│   ├── integration/
│   └── frontend/
├── support/            # Harness, datasets, runtime shims, and in-memory traffic fakes
├── CMakeLists.txt      # Host-side test entrypoint
├── coverage-thresholds.json  # Fixed canonical runtime coverage gates
├── gcc13-coverage.Dockerfile  # Canonical Linux/GCC coverage environment
├── run_all_tests.sh    # Parallel build and test launcher
├── run_gcc13_coverage.sh      # Docker-backed canonical GCC 13 coverage runner
└── run_coverage.sh     # Coverage script
```

Production code under test lives outside `test/cpp/`:

- `src/cpp/core/` for reusable detection logic
- `src/cpp/runtime/` for the shared runtime contract and `src/cpp/runtime/esp_idf/` for the current runtime orchestration
- `src/cpp/frontend/esphome/components/espectre/` for the ESPHome component manifest and adapter layer
- `src/cpp/frontend/matter/espectre/` for the Matter adapter and surface mapping

Traffic policy and UDP-listener unit tests use the shared in-memory adapters in `support/csi_traffic_fakes.h`. Host tests must not open real UDP sockets; the lwIP implementation is validated by the ESP-IDF firmware builds.

## Adding New Tests

Create `test/cpp/suites/core/test_my_feature.cpp`:

```cpp
#include "test_harness.h"

void setUp(void) {}
void tearDown(void) {}

void test_example(void) {
    TEST_ASSERT_EQUAL(1, 1);
}

int process(void) {
    UNITY_BEGIN();
    RUN_TEST(test_example);
    return UNITY_END();
}

#if defined(ESP_PLATFORM)
extern "C" void app_main(void) { process(); }
#else
int main(int argc, char **argv) { return process(); }
#endif
```

Register the file in `test/cpp/suites/CMakeLists.txt` and run it with `ctest --test-dir test/cpp/build -R test_my_feature`.
