# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Python Test Fixtures

Shared pytest fixtures for ESPectre Python tests.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

import sys
import importlib
import math
import os
import hashlib
import pytest
import numpy as np
import json
from pathlib import Path

def _bootstrap_repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "tools" / "lib" / "repo_paths.py").is_file():
            return parent
    raise RuntimeError("Unable to locate the ESPectre repository root")


REPO_ROOT = _bootstrap_repo_root()
TESTS_PATH = Path(__file__).resolve().parent
PYTHON_ROOT_PATH = REPO_ROOT / "src" / "python"


def _prepend_sys_path(path: Path) -> None:
    """Add a repository-relative path once, keeping later entries stable."""
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)


_prepend_sys_path(REPO_ROOT)
_prepend_sys_path(TESTS_PATH)
_prepend_sys_path(PYTHON_ROOT_PATH)

from support.performance import (
    build_long_test_params,
    format_targets_summary_line,
    get_classic_fp_rate_target,
    get_classic_recall_target,
    get_ml_fp_rate_target,
    get_ml_recall_target,
    load_long_test_dataset,
)
from tools.lib.repo_paths import data_dir, tools_lib_dir, python_src_dir, repo_root

assert REPO_ROOT == repo_root()

# Add both the Python root and the Micro-ESPectre runtime source dir.
# The runtime dir is inserted last (position 0) so it takes precedence for
# direct imports like `import config`, while `espectre_cli` still resolves from
# `src/python/`.
SRC_PATH = python_src_dir()
_prepend_sys_path(SRC_PATH)
REFERENCE_PATH = tools_lib_dir()
_prepend_sys_path(REFERENCE_PATH)

# Historical tests import reference algorithms as flat modules. Bind those
# names once to the canonical package so test order cannot accidentally select
# the MicroPython facade with the same basename.
for _module_name in (
    "filters",
    "csi_features",
    "segmentation",
    "high_accuracy_detector",
    "lightweight_detector",
    "ml_feature_trackers",
    "ml_weights",
    "runtime_policy",
    "temporal_csi_sampler",
    "utils",
):
    sys.modules[_module_name] = importlib.import_module(
        f"tools.lib.{_module_name}"
    )

from config import (
    DEFAULT_SUBCARRIERS,
    HAMPEL_THRESHOLD,
    HAMPEL_WINDOW,
    SEGMENTATION_WINDOW_SIZE_MS,
)
from runtime_policy import derive_detector_timing, nominal_packet_interval_us

# Data directory (shared between tests and tools)
DATA_DIR = data_dir()
DATASET_INFO_PATH = DATA_DIR / 'dataset_info.json'
UNIT_TEST_SUBCARRIERS = DEFAULT_SUBCARRIERS
DEFAULT_XDIST_AUTO_WORKERS = 4


def pytest_xdist_auto_num_workers(config):
    """Cap replay-heavy auto parallelism while preserving an explicit override."""
    del config
    process_cpu_count = getattr(os, "process_cpu_count", os.cpu_count)
    available_workers = max(1, process_cpu_count() or 1)
    configured = os.environ.get("PYTEST_XDIST_AUTO_NUM_WORKERS")
    if configured is not None:
        try:
            workers = int(configured)
        except ValueError as exc:
            raise pytest.UsageError(
                "PYTEST_XDIST_AUTO_NUM_WORKERS must be a positive integer"
            ) from exc
        if workers < 1:
            raise pytest.UsageError(
                "PYTEST_XDIST_AUTO_NUM_WORKERS must be a positive integer"
            )
        return min(workers, available_workers)

    return min(DEFAULT_XDIST_AUTO_WORKERS, available_workers)


@pytest.fixture
def fp_rate_target(chip_type):
    """Lightweight FP-rate target fixture shared across test modules."""
    return get_classic_fp_rate_target(chip_type)


@pytest.fixture
def recall_target(chip_type):
    """Lightweight recall target fixture shared across test modules."""
    return get_classic_recall_target(chip_type)


@pytest.fixture
def ml_fp_rate_target(chip_type):
    """ML FP-rate target fixture shared across test modules."""
    return get_ml_fp_rate_target()


@pytest.fixture
def ml_recall_target(chip_type):
    """ML recall target fixture shared across test modules."""
    return get_ml_recall_target()


# ============================================================================
# Configuration Fixtures
# ============================================================================

@pytest.fixture(params=build_long_test_params())
def long_test_config(request):
    """Validated long-recording replay split from the current dataset layout."""
    return load_long_test_dataset(request.param)

@pytest.fixture
def default_subcarriers(request):
    """
    Default subcarrier band for testing (HT20: 64 SC only).
    
    Matches C++ test configuration exactly (test_motion_detection.cpp).
    """
    try:
        request.getfixturevalue('dataset_config')
    except pytest.FixtureLookupError:
        # Unit/integration tests that do not define dataset_config still need
        # a deterministic 12-SC band. Real-data performance tests define
        # dataset_config and stay strict metadata-driven.
        return UNIT_TEST_SUBCARRIERS

    return UNIT_TEST_SUBCARRIERS


@pytest.fixture
def optimal_threshold(request):
    """
    Legacy compatibility threshold fixture.

    Detector-specific startup thresholds are now calibrated per capture, so
    tests that still request this fixture receive a neutral placeholder.
    """
    return 1.0


@pytest.fixture
def segmentation_config():
    """Default segmentation configuration - matches C++ DETECTOR_DEFAULT_WINDOW_SIZE"""
    return {
        'window_size': derive_detector_timing(
            nominal_packet_interval_us(100), SEGMENTATION_WINDOW_SIZE_MS
        )["window_packets"],
        'threshold': 1.0,
        'enable_hampel': True,
        'hampel_window': HAMPEL_WINDOW,
        'hampel_threshold': HAMPEL_THRESHOLD,
    }


@pytest.fixture
def hampel_config():
    """Default Hampel filter configuration"""
    return {
        'window_size': HAMPEL_WINDOW,
        'threshold': HAMPEL_THRESHOLD,
    }


# ============================================================================
# Synthetic Data Fixtures
# ============================================================================

@pytest.fixture
def constant_values():
    """Constant value test data"""
    return [5.0] * 500


@pytest.fixture
def linear_ramp():
    """Linear ramp test data"""
    return [float(i) for i in range(500)]


@pytest.fixture
def sine_wave():
    """Sine wave test data"""
    return [math.sin(i * 0.1) * 10 + 50 for i in range(500)]


@pytest.fixture
def random_uniform():
    """Random uniform distribution test data"""
    np.random.seed(42)  # Reproducible
    return list(np.random.uniform(0, 100, 500))


@pytest.fixture
def random_normal():
    """Random normal distribution test data"""
    np.random.seed(42)  # Reproducible
    return list(np.random.normal(50, 15, 500))


@pytest.fixture
def step_function():
    """Step function test data"""
    return [10.0] * 250 + [90.0] * 250


@pytest.fixture
def impulse_data():
    """Impulse/spike test data"""
    return [50.0] * 200 + [200.0] + [50.0] * 299


@pytest.fixture
def synthetic_turbulence_baseline():
    """Simulated baseline turbulence (low variance)"""
    np.random.seed(42)
    return list(np.random.normal(5.0, 0.5, 500))


@pytest.fixture
def synthetic_turbulence_movement():
    """Simulated movement turbulence (high variance)"""
    np.random.seed(42)
    return list(np.random.normal(10.0, 3.0, 500))


# ============================================================================
# CSI Data Fixtures
# ============================================================================

@pytest.fixture
def synthetic_csi_packet():
    """Generate a synthetic CSI packet (64 subcarriers, I/Q pairs)"""
    np.random.seed(42)
    # Generate I/Q values as int8 (range -128 to 127)
    iq_data = np.random.randint(-50, 50, size=128, dtype=np.int8)
    return iq_data


@pytest.fixture
def synthetic_csi_static_presence_packets():
    """Generate synthetic baseline CSI packets (stable signal)"""
    np.random.seed(42)
    packets = []
    for _i in range(100):
        # Stable signal with small variations
        base_amplitude = 30
        iq_data = np.zeros(128, dtype=np.int8)
        for sc in range(64):
            I = int(base_amplitude + np.random.normal(0, 2))
            Q = int(base_amplitude * 0.3 + np.random.normal(0, 2))
            # Espressif CSI format: [Imaginary, Real, ...] per subcarrier
            iq_data[sc * 2] = np.clip(Q, -127, 127)      # Imaginary first
            iq_data[sc * 2 + 1] = np.clip(I, -127, 127)  # Real second
        packets.append({'csi_data': iq_data, 'label': 'static_presence'})
    return packets


@pytest.fixture
def synthetic_csi_baseline_packets(synthetic_csi_static_presence_packets):
    """Backward-compatible alias for static-presence synthetic packets."""
    return synthetic_csi_static_presence_packets


@pytest.fixture
def synthetic_csi_motion_packets():
    """Generate synthetic movement CSI packets (variable signal)"""
    np.random.seed(43)
    packets = []
    for _i in range(100):
        # Variable signal with larger variations
        base_amplitude = 25 + np.random.uniform(-10, 10)
        iq_data = np.zeros(128, dtype=np.int8)
        for sc in range(64):
            I = int(base_amplitude + np.random.normal(0, 8))
            Q = int(base_amplitude * 0.3 + np.random.normal(0, 8))
            # Espressif CSI format: [Imaginary, Real, ...] per subcarrier
            iq_data[sc * 2] = np.clip(Q, -127, 127)      # Imaginary first
            iq_data[sc * 2 + 1] = np.clip(I, -127, 127)  # Real second
        packets.append({'csi_data': iq_data, 'label': 'motion'})
    return packets


@pytest.fixture
def synthetic_csi_movement_packets(synthetic_csi_motion_packets):
    """Backward-compatible alias for motion synthetic packets."""
    return synthetic_csi_motion_packets


# ============================================================================
# Real CSI Data Fixtures (optional - skip if not available)
# ============================================================================

@pytest.fixture
def real_csi_data_available():
    """Check if real CSI data files are available"""
    from tools.lib.csi_io import find_static_presence_motion_dataset
    try:
        find_static_presence_motion_dataset(chip='C6')
        return True
    except FileNotFoundError:
        return False


@pytest.fixture
def real_static_presence_packets(real_csi_data_available):
    """Load real baseline CSI packets (skip if not available)"""
    if not real_csi_data_available:
        pytest.skip("Real CSI data not available")
    
    from tools.lib.csi_io import load_static_presence_and_motion
    baseline, _ = load_static_presence_and_motion()
    return baseline


@pytest.fixture
def real_baseline_packets(real_static_presence_packets):
    """Backward-compatible alias for static-presence real packets."""
    return real_static_presence_packets


@pytest.fixture
def real_motion_packets(real_csi_data_available):
    """Load real movement CSI packets (skip if not available)"""
    if not real_csi_data_available:
        pytest.skip("Real CSI data not available")
    
    from tools.lib.csi_io import load_static_presence_and_motion
    _, movement = load_static_presence_and_motion()
    return movement


@pytest.fixture
def real_movement_packets(real_motion_packets):
    """Backward-compatible alias for motion real packets."""
    return real_motion_packets


@pytest.fixture
def real_turbulence_values(real_csi_data_available, default_subcarriers):
    """Calculate turbulence values from real CSI data"""
    if not real_csi_data_available:
        pytest.skip("Real CSI data not available")
    
    from tools.lib.csi_analysis import calculate_spatial_turbulence
    from tools.lib.csi_io import load_static_presence_and_motion
    
    baseline, movement = load_static_presence_and_motion()
    turbulence_values = []
    
    for packet in baseline:
        turbulence = calculate_spatial_turbulence(
            packet['csi_data'],
            default_subcarriers,
        )
        turbulence_values.append(float(turbulence))
    
    for packet in movement:
        turbulence = calculate_spatial_turbulence(
            packet['csi_data'],
            default_subcarriers,
        )
        turbulence_values.append(float(turbulence))
    
    return turbulence_values


# ============================================================================
# Utility Fixtures
# ============================================================================

@pytest.fixture
def tolerance():
    """Standard tolerance for floating point comparisons"""
    return 1e-6


# ============================================================================
# Performance Results Collection (for summary table)
# ============================================================================

import tempfile
import fcntl

# Use a temp file to share results between test module and conftest hook
_PERF_RESULTS_FILE = os.path.join(
    tempfile.gettempdir(),
    f"espectre_perf_results_{hashlib.sha1(str(REPO_ROOT).encode('utf-8')).hexdigest()[:12]}.json",
)


def _is_worker_process(config) -> bool:
    """Return True when running inside an xdist worker process."""
    return hasattr(config, "workerinput")


def _load_perf_results_locked(file_obj):
    """Load performance results while holding an exclusive lock."""
    file_obj.seek(0)
    payload = file_obj.read().strip()
    if not payload:
        return {}
    try:
        return json.loads(payload)
    except json.JSONDecodeError:
        return {}


def record_performance(chip: str, algorithm: str, recall: float, fp_rate: float,
                       precision: float = 0.0, f1: float = 0.0,
                       dataset_id: str = ""):
    """
    Record performance metrics for the summary table.
    
    Args:
        chip: Chip type (C3, C5, C6, ESP32, S3)
        algorithm: Algorithm name (classic, ml, or a legacy comparison label)
        recall: Recall percentage
        fp_rate: False positive rate percentage
        precision: Precision percentage
        f1: F1-score percentage
        dataset_id: Stable static-presence/motion pair id
    """
    os.makedirs(os.path.dirname(_PERF_RESULTS_FILE), exist_ok=True)
    with open(_PERF_RESULTS_FILE, "a+", encoding="utf-8") as f:
        fcntl.flock(f.fileno(), fcntl.LOCK_EX)
        try:
            results = _load_perf_results_locked(f)

            if chip not in results:
                results[chip] = {}
            if algorithm not in results[chip]:
                results[chip][algorithm] = []
            elif isinstance(results[chip][algorithm], dict):
                results[chip][algorithm] = [results[chip][algorithm]]

            results[chip][algorithm].append({
                'dataset_id': dataset_id,
                'recall': recall,
                'fp_rate': fp_rate,
                'precision': precision,
                'f1': f1
            })

            f.seek(0)
            f.truncate()
            json.dump(results, f)
            f.flush()
            os.fsync(f.fileno())
        finally:
            fcntl.flock(f.fileno(), fcntl.LOCK_UN)


def pytest_configure(config):
    """Clear performance results at the start of test session."""
    if _is_worker_process(config):
        return
    if os.path.exists(_PERF_RESULTS_FILE):
        os.remove(_PERF_RESULTS_FILE)


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    """Print performance summary table at the end of test session."""
    if _is_worker_process(config):
        return
    results = {}
    if os.path.exists(_PERF_RESULTS_FILE):
        with open(_PERF_RESULTS_FILE, "a+", encoding="utf-8") as f:
            fcntl.flock(f.fileno(), fcntl.LOCK_EX)
            try:
                results = _load_perf_results_locked(f)
            finally:
                fcntl.flock(f.fileno(), fcntl.LOCK_UN)

    def average_metrics(entries):
        if isinstance(entries, dict):
            entries = [entries]
        if not entries:
            return None
        return {
            'count': len(entries),
            'recall': sum(r['recall'] for r in entries) / len(entries),
            'fp_rate': sum(r['fp_rate'] for r in entries) / len(entries),
            'precision': sum(r.get('precision', 0) for r in entries) / len(entries),
            'f1': sum(r.get('f1', 0) for r in entries) / len(entries),
        }

    if results:
        terminalreporter.write_line("")
        terminalreporter.write_line("=" * 105)
        terminalreporter.write_line("                              PERFORMANCE SUMMARY TABLE (Python)")
        terminalreporter.write_line("=" * 105)
        terminalreporter.write_line("")
        terminalreporter.write_line("| Chip   | Datasets | Lightweight                 | High Accuracy           |")
        terminalreporter.write_line("|--------|----------|-------------------------|-------------------------|")

        # Sort chips for consistent output
        for chip in ['C3', 'C5', 'C6', 'ESP32', 'S3']:
            if chip not in results:
                continue

            chip_results = results[chip]
            dataset_count = max(
                (len(v) if isinstance(v, list) else 1)
                for v in chip_results.values()
            )

            if 'classic' in chip_results:
                classic = average_metrics(chip_results['classic'])
                classic_str = f"{classic['recall']:.1f}% R, {classic['fp_rate']:.1f}% FP"
            else:
                classic_str = "N/A"

            if 'ml' in chip_results:
                ml = average_metrics(chip_results['ml'])
                ml_str = f"{ml['recall']:.1f}% R, {ml['fp_rate']:.1f}% FP"
            else:
                ml_str = "N/A"

            terminalreporter.write_line(
                f"| {chip:<6} | {dataset_count:>8} | {classic_str:<23} | {ml_str:<23} |"
            )

        terminalreporter.write_line("")
        terminalreporter.write_line("Legend: R = Recall, FP = False Positive Rate")
        terminalreporter.write_line(format_targets_summary_line())
        terminalreporter.write_line("=" * 105)

        # Detailed table for PERFORMANCE.md
        terminalreporter.write_line("")
        terminalreporter.write_line("                         DETAILED METRICS (for PERFORMANCE.md)")
        terminalreporter.write_line("-" * 105)
        terminalreporter.write_line("| Chip   | Algorithm   | Datasets | Recall  | Precision | FP Rate | F1-Score |")
        terminalreporter.write_line("|--------|-------------|----------|---------|-----------|---------|----------|")

        for chip in ['C3', 'C5', 'C6', 'ESP32', 'S3']:
            if chip not in results:
                continue

            chip_results = results[chip]

            if 'classic' in chip_results:
                classic = average_metrics(chip_results['classic'])
                terminalreporter.write_line(
                    f"| {chip:<6} | {'CLASSIC':<11} | {classic['count']:>8} | {classic['recall']:>6.1f}% | {classic.get('precision', 0):>8.1f}% | {classic['fp_rate']:>6.1f}% | {classic.get('f1', 0):>7.1f}% |"
                )

            if 'ml' in chip_results:
                r = average_metrics(chip_results['ml'])
                terminalreporter.write_line(
                    f"| {chip:<6} | {'ML':<11} | {r['count']:>8} | {r['recall']:>6.1f}% | {r.get('precision', 0):>8.1f}% | {r['fp_rate']:>6.1f}% | {r.get('f1', 0):>7.1f}% |"
                )

        terminalreporter.write_line("-" * 105)

    # Cleanup
    if os.path.exists(_PERF_RESULTS_FILE):
        os.remove(_PERF_RESULTS_FILE)
