# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Cached inputs that extend the generated detector performance report."""

from __future__ import annotations

import hashlib
import json
import os
import platform
import shutil
import subprocess
from pathlib import Path
from typing import Any, Callable, Optional


from . import npz_cache
from .dataset_metadata import dataset_info_revision, load_dataset_info
from .repo_paths import repo_root

REPO_ROOT = repo_root()
RESOURCE_BENCHMARK_SOURCE = (
    REPO_ROOT / "test" / "cpp" / "support" / "benchmark_detector_resources.cpp"
)
CORE_SOURCE_DIR = REPO_ROOT / "src" / "cpp" / "core"
RESOURCE_BENCHMARK_SOURCES = (
    RESOURCE_BENCHMARK_SOURCE,
    CORE_SOURCE_DIR / "base_detector.cpp",
    CORE_SOURCE_DIR / "lightweight_detector.cpp",
    CORE_SOURCE_DIR / "filtered_turbulence_ring.cpp",
    CORE_SOURCE_DIR / "filters.cpp",
    CORE_SOURCE_DIR / "high_accuracy_detector.cpp",
    CORE_SOURCE_DIR / "espectre_log.cpp",
)
RESOURCE_BENCHMARK_DEPENDENCIES = (
    *RESOURCE_BENCHMARK_SOURCES,
    *sorted(CORE_SOURCE_DIR.glob("*.h")),
)


def _compiler() -> str:
    compiler = os.environ.get("CXX") or shutil.which("c++")
    if not compiler:
        raise RuntimeError("C++ compiler not found; set CXX to generate resource metrics")
    return compiler


def _resource_binary() -> Path:
    compiler = _compiler()
    digest = hashlib.sha256()
    for source in RESOURCE_BENCHMARK_DEPENDENCIES:
        digest.update(str(source.relative_to(REPO_ROOT)).encode("utf-8"))
        digest.update(b"\0")
        digest.update(source.read_bytes())
    version = subprocess.run(
        [compiler, "--version"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()[0]
    digest.update(version.encode("utf-8"))
    digest.update(platform.machine().encode("utf-8"))
    binary_dir = npz_cache.artifact_dir("performance_resource_binaries")
    binary_path = binary_dir / digest.hexdigest()
    if binary_path.exists():
        return binary_path
    binary_dir.mkdir(parents=True, exist_ok=True)
    temporary = binary_dir / f"{binary_path.name}.{os.getpid()}.tmp"
    command = [
        compiler,
        "-O3",
        "-DNDEBUG",
        "-std=c++17",
        f"-I{REPO_ROOT / 'src' / 'cpp' / 'core'}",
        *(str(path) for path in RESOURCE_BENCHMARK_SOURCES),
        "-o",
        str(temporary),
    ]
    subprocess.run(command, check=True, capture_output=True, text=True)
    os.replace(temporary, binary_path)
    return binary_path


def run_current_resource_benchmark() -> dict[str, Any]:
    """Compile from the source-keyed cache, then always execute fresh timings."""
    result = subprocess.run(
        [str(_resource_binary())],
        check=True,
        capture_output=True,
        text=True,
    )
    json_start = result.stdout.find("{")
    if json_start < 0:
        raise RuntimeError("resource benchmark did not emit JSON")
    return json.loads(result.stdout[json_start:])


def _paired_records() -> list[dict[str, Any]]:
    info = load_dataset_info()
    files = info.get("files", {})
    motion = {
        str(entry.get("filename")): entry
        for entry in files.get("motion", [])
        if entry.get("filename")
    }
    records = []
    for static in files.get("static_presence", []):
        motion_entry = motion.get(str(static.get("optimal_pair_motion_file", "")))
        if motion_entry is None:
            continue
        role = str(static.get("dataset_role", "exclude")).strip().lower()
        if role not in {"train", "selection", "holdout"}:
            continue
        records.append(
            {
                "role": role,
                "chip": str(static.get("chip", "unknown")).upper(),
                "static_path": REPO_ROOT / "data" / "static_presence" / static["filename"],
                "motion_path": REPO_ROOT / "data" / "motion" / motion_entry["filename"],
            }
        )
    return records


def _metrics_from_counts(tp: int, fn: int, fp: int, tn: int) -> dict[str, float]:
    recall = tp / (tp + fn) * 100.0 if tp + fn else 0.0
    fp_rate = fp / (fp + tn) * 100.0 if fp + tn else 0.0
    precision = tp / (tp + fp) * 100.0 if tp + fp else 0.0
    f1 = (
        2.0 * precision * recall / (precision + recall)
        if precision + recall
        else 0.0
    )
    return {"recall": recall, "fp_rate": fp_rate, "f1": f1}


def compute_reserved_augmentation_diagnostic() -> Optional[dict[str, Any]]:
    """Evaluate Lightweight and High Accuracy on one shared two-seed reserved stress corpus."""
    from config import DEFAULT_SUBCARRIERS
    from tools.lib.high_accuracy_detector import FEATURE_NAMES
    from tools.lib.csi_io import load_npz_packet_view
    from tools.lib.performance_report import (
        _compute_ml_row_result,
        compute_classic_row_result,
        load_or_compute_classic_replay_rows,
        load_or_compute_ml_replay_rows,
    )
    from tools.lib.ml_training.augmentation import (
        _packet_augmentation_stream_provenance,
        _prepare_feature_packets_for_record,
        resolve_training_augmentation,
        training_packet_augmentation_seeds,
    )
    from tools.lib.ml_training.feature_cache import (
        _mix_packet_augmentation_replay_rows,
    )

    components = ("base", "drift", "burst-loss")
    _active, _feature_config, packet_config = resolve_training_augmentation(components)
    seeds = training_packet_augmentation_seeds(packet_config)
    counts = {"classic": [0, 0, 0, 0], "ml": [0, 0, 0, 0]}
    pair_count = 0
    for record in _paired_records():
        if record["role"] not in {"selection", "holdout"}:
            continue
        pair_complete = True
        pair_counts = {"classic": [0, 0, 0, 0], "ml": [0, 0, 0, 0]}
        ml_views = {"static": [], "motion": []}
        for view_index, seed in enumerate(seeds):
            provenance = _packet_augmentation_stream_provenance(packet_config, seed)
            augmented = {}
            for phase in ("static", "motion"):
                path = record[f"{phase}_path"]
                source = {"path": path, "packets": load_npz_packet_view(path)}
                augmented[phase] = _prepare_feature_packets_for_record(
                    source,
                    packet_augmentation=packet_config,
                    augmentation_seed=seed,
                )

            classic_rows = load_or_compute_classic_replay_rows(
                record["static_path"],
                record["motion_path"],
                static_presence_packets=augmented["static"],
                motion_packets=augmented["motion"],
                selected_subcarriers=DEFAULT_SUBCARRIERS,
                replay_kind="classic_reserved_augmentation",
                replay_provenance=provenance,
            )
            classic_result = compute_classic_row_result(
                classic_rows,
                row_stride=len(seeds),
                row_offset=view_index,
            )
            if classic_result is None:
                pair_complete = False
                break
            classic_metrics = classic_result[1]
            current_classic = [
                int(classic_metrics[key]) for key in ("tp", "fn", "fp", "tn")
            ]

            ml_rows = {}
            for phase in ("static", "motion"):
                path = record[f"{phase}_path"]
                ml_rows[phase] = load_or_compute_ml_replay_rows(
                    path,
                    packets=augmented[phase],
                    selected_subcarriers=DEFAULT_SUBCARRIERS,
                    window_size=None,
                    feature_names=FEATURE_NAMES,
                    sample_contract="replay_tick",
                    stream_provenance=provenance,
                )
                ml_views[phase].append(ml_rows[phase])
            pair_counts["classic"] = [
                total + value
                for total, value in zip(pair_counts["classic"], current_classic, strict=True)
            ]
        if not pair_complete:
            continue
        mixed_ml_rows = {
            phase: _mix_packet_augmentation_replay_rows(ml_views[phase])
            for phase in ("static", "motion")
        }
        ml_metrics, _feature_payload = _compute_ml_row_result(
            mixed_ml_rows["static"],
            mixed_ml_rows["motion"],
            0.5,
        )
        pair_counts["ml"] = [
            int(ml_metrics[key]) for key in ("tp", "fn", "fp", "tn")
        ]
        pair_count += 1
        for detector in counts:
            counts[detector] = [
                total + value
                for total, value in zip(counts[detector], pair_counts[detector], strict=True)
            ]

    if not pair_count:
        return None
    return {
        "recipe": "+".join(components),
        "roles": ["selection", "holdout"],
        "seeds": [int(seed) for seed in seeds],
        "pair_count": pair_count,
        "rows": [
            {"detector": detector, **_metrics_from_counts(*counts[detector])}
            for detector in ("classic", "ml")
        ],
    }


def collect_extended_report_inputs(
    progress: Optional[Callable[[str], None]] = None,
) -> tuple[dict[str, Any], Optional[dict[str, Any]]]:
    """Collect fresh cheap metrics and the cached augmentation diagnostic."""
    if progress:
        progress("running current C++ detector resource microbenchmark")
    resources = run_current_resource_benchmark()
    if progress:
        progress("loading the cached reserved augmentation diagnostic")
    from tools.lib.ml_training import implementation_paths

    diagnostic_implementations = (
        Path(__file__),
        REPO_ROOT / "tools" / "train_ml_model.py",
        *implementation_paths(),
        REPO_ROOT / "tools" / "lib" / "performance_report.py",
        REPO_ROOT / "tools" / "lib" / "lightweight_detector.py",
        REPO_ROOT / "tools" / "lib" / "csi_features.py",
        REPO_ROOT / "tools" / "lib" / "high_accuracy_detector.py",
    )
    augmentation_parameters = npz_cache.performance_report_result_parameters(
        kind="reserved_augmentation_diagnostic",
        inputs={
            "dataset_revision": dataset_info_revision(),
            "ml_weights": npz_cache.source_manifest(
                REPO_ROOT / "tools" / "lib" / "ml_weights.py"
            ),
            "implementations": {
                str(path.relative_to(REPO_ROOT)): npz_cache.source_manifest(path)
                for path in diagnostic_implementations
            },
        },
    )
    augmentation = npz_cache.load_performance_report_result(
        REPO_ROOT / "data" / "dataset_info.json",
        parameters=augmentation_parameters,
    )
    if augmentation is None:
        augmentation = compute_reserved_augmentation_diagnostic()
        if augmentation is not None:
            npz_cache.save_performance_report_result(
                REPO_ROOT / "data" / "dataset_info.json",
                parameters=augmentation_parameters,
                payload=augmentation,
            )
    return resources, augmentation
