# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Importable performance-test parameters, targets, and reporting support."""

from __future__ import annotations

import fcntl
import hashlib
import json
import os
import tempfile

import pytest

from support.chip_matrix import DETECTION_CHIPS
from tools.lib.performance_report import (
    extract_motion_start_from_description,  # noqa: F401 - re-exported test support API
    get_available_long_test_dataset_specs,
    get_available_long_test_datasets,  # noqa: F401 - re-exported test support API
    load_long_test_dataset,  # noqa: F401 - re-exported test support API
)
from tools.lib.repo_paths import repo_root


def get_classic_fp_rate_target(chip_type=None):
    """Bound the Lightweight motion share on static-presence baselines."""
    del chip_type
    return 12.0


def get_classic_recall_target(chip_type=None):
    """Return the shared Lightweight recall target."""
    del chip_type
    return 95.0


def get_ml_fp_rate_target():
    return 5.0


def get_ml_recall_target():
    return 95.0


def format_targets_summary_line():
    return (
        "Targets: "
        f"Lightweight >{get_classic_recall_target():.0f}% R, "
        f"<{get_classic_fp_rate_target():.1f}% FP | "
        f"ML >{get_ml_recall_target():.0f}% R, <{get_ml_fp_rate_target():.1f}% FP"
    )


def build_long_test_params(chips=None):
    """Build stable long-replay params with one visible case per chip."""
    requested_chips = tuple(chips) if chips is not None else DETECTION_CHIPS
    params = []
    for requested_chip in requested_chips:
        specs = get_available_long_test_dataset_specs(chips=(requested_chip,))
        if not specs:
            params.append(
                pytest.param(
                    None,
                    marks=pytest.mark.skip(
                        reason=f"No eligible long dataset for chip {requested_chip}"
                    ),
                    id=f"{requested_chip.lower()}_no_eligible_long_dataset",
                )
            )
            continue
        for spec in specs:
            _, motion_start_packet, num_packets, chip, _ = spec
            params.append(
                pytest.param(
                    spec,
                    id=(
                        f"{chip.lower()}_long_"
                        f"{motion_start_packet}b_{num_packets - motion_start_packet}m_"
                        f"start{motion_start_packet}"
                    ),
                )
            )
    return params


_PERF_RESULTS_FILE = os.path.join(
    tempfile.gettempdir(),
    f"espectre_perf_results_{hashlib.sha1(str(repo_root()).encode('utf-8')).hexdigest()[:12]}.json",
)


def _load_perf_results_locked(file_obj):
    file_obj.seek(0)
    payload = file_obj.read().strip()
    if not payload:
        return {}
    try:
        return json.loads(payload)
    except json.JSONDecodeError:
        return {}


def record_performance(
    chip: str,
    algorithm: str,
    recall: float,
    fp_rate: float,
    precision: float = 0.0,
    f1: float = 0.0,
    dataset_id: str = "",
):
    """Record one replay result for the terminal summary."""
    os.makedirs(os.path.dirname(_PERF_RESULTS_FILE), exist_ok=True)
    with open(_PERF_RESULTS_FILE, "a+", encoding="utf-8") as handle:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        try:
            results = _load_perf_results_locked(handle)
            entries = results.setdefault(chip, {}).setdefault(algorithm, [])
            if isinstance(entries, dict):
                entries = [entries]
                results[chip][algorithm] = entries
            entries.append(
                {
                    "dataset_id": dataset_id,
                    "recall": recall,
                    "fp_rate": fp_rate,
                    "precision": precision,
                    "f1": f1,
                }
            )
            handle.seek(0)
            handle.truncate()
            json.dump(results, handle)
            handle.flush()
            os.fsync(handle.fileno())
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def configure_performance_session(config) -> None:
    if hasattr(config, "workerinput"):
        return
    if os.path.exists(_PERF_RESULTS_FILE):
        os.remove(_PERF_RESULTS_FILE)


def write_performance_terminal_summary(terminalreporter, config) -> None:
    if hasattr(config, "workerinput"):
        return
    results = {}
    if os.path.exists(_PERF_RESULTS_FILE):
        with open(_PERF_RESULTS_FILE, "a+", encoding="utf-8") as handle:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
            try:
                results = _load_perf_results_locked(handle)
            finally:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)

    def average(entries):
        if isinstance(entries, dict):
            entries = [entries]
        if not entries:
            return None
        return {
            "count": len(entries),
            "recall": sum(row["recall"] for row in entries) / len(entries),
            "fp_rate": sum(row["fp_rate"] for row in entries) / len(entries),
            "precision": sum(row.get("precision", 0) for row in entries) / len(entries),
            "f1": sum(row.get("f1", 0) for row in entries) / len(entries),
        }

    if results:
        terminalreporter.write_line("")
        terminalreporter.write_line("=" * 105)
        terminalreporter.write_line("PERFORMANCE SUMMARY TABLE (Python)")
        terminalreporter.write_line("=" * 105)
        terminalreporter.write_line("| Chip   | Datasets | Lightweight             | High Accuracy           |")
        terminalreporter.write_line("|--------|----------|-------------------------|-------------------------|")
        for chip in DETECTION_CHIPS:
            if chip not in results:
                continue
            chip_results = results[chip]
            count = max(len(value) if isinstance(value, list) else 1 for value in chip_results.values())
            classic = average(chip_results.get("classic", []))
            ml = average(chip_results.get("ml", []))
            classic_text = "N/A" if classic is None else f"{classic['recall']:.1f}% R, {classic['fp_rate']:.1f}% FP"
            ml_text = "N/A" if ml is None else f"{ml['recall']:.1f}% R, {ml['fp_rate']:.1f}% FP"
            terminalreporter.write_line(
                f"| {chip:<6} | {count:>8} | {classic_text:<23} | {ml_text:<23} |"
            )
        terminalreporter.write_line(format_targets_summary_line())
        terminalreporter.write_line("=" * 105)

    if os.path.exists(_PERF_RESULTS_FILE):
        os.remove(_PERF_RESULTS_FILE)
