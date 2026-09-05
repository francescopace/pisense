#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Lightweight Candidate Replay

Research-only fitter and replay harness for one-, two-, and three-feature Lightweight detector
candidates. It mirrors the grouped logistic fit and startup-threshold workflow
of `fit_lightweight_detector.py`, but never writes runtime artifacts.

Usage:
    python tools/replay_lightweight_candidates.py --features turb_autocorr
    python tools/replay_lightweight_candidates.py --stress-augment \\
        --features turb_iqr_over_mean_aggr,l1_delta_lag_ratio
    python tools/replay_lightweight_candidates.py --calibration robust_logit \\
        --features turb_autocorr,turb_iqr_over_mean_aggr
    python tools/replay_lightweight_candidates.py \\
        --features turb_autocorr,turb_iqr_over_mean_aggr,chan_shape_excess_path \\
        --external-data-dir data/untracked/csi_sense_zero \\
        --external-data-dir data/untracked/wisdom_lab \\
        --external-diagnostic-all-phy data/untracked/wisdom_lab

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from itertools import product
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.lib.bootstrap import setup_paths  # noqa: E402

setup_paths()

import config  # noqa: E402
from tools.lib.ml_training import (
    augmentation,
    feature_cache,
)
from tools.lib.lightweight_detector import LightweightDetector  # noqa: E402
from tools.lib.ml_weights import FEATURE_NAMES  # noqa: E402
from tools.fit_lightweight_detector import (  # noqa: E402
    IDLE_LABEL,
    MOTION_LABEL,
    balanced_sample_weights,
    choose_base_threshold,
    fit_coefficients,
    logits,
)
from tools.lib.candidate_features import CANDIDATE_FEATURES  # noqa: E402
from tools.lib.csi_io import load_npz_as_packets  # noqa: E402
from tools.lib.dataset_metadata import (  # noqa: E402
    dataset_role,
    detector_window_packets,
    load_dataset_info,
    measure_packet_interval_us,
    paired_dataset_role,
    resolve_entry_path,
)
from tools.lib.performance_report import (  # noqa: E402
    load_or_compute_ml_replay_rows,
)
from tools.lib.temporal_replay import (  # noqa: E402
    iter_temporal_admissions,
    target_pps_for_packets,
)
from tools.lib.runtime_policy import RuntimeMotionPolicy  # noqa: E402

DISCOVERY_ROLES = ("train", "selection")
HOLDOUT_ROLE = "holdout"
PRIMARY_ROLES = DISCOVERY_ROLES + (HOLDOUT_ROLE,)
EXCLUDE_ROLE = "exclude"
REPLAY_ROLES = PRIMARY_ROLES + (EXCLUDE_ROLE,)
CURRENT_CLASSIC_COMBINATION = ("turb_autocorr", "turb_iqr_over_mean_aggr")
STRESS_SCENARIOS = {
    "base": ("base",),
    "drift": ("drift",),
    "burst-loss": ("burst-loss",),
    "combined": ("base", "drift", "burst-loss"),
}
CALIBRATION_Q95_SHIFT = "q95_shift"
CALIBRATION_ROBUST_LOGIT = "robust_logit"
CALIBRATION_FEATURE_SHIFT = "feature_shift"
CALIBRATION_GUARDED_UPWARD = "guarded_upward"
CALIBRATION_MODES = (
    CALIBRATION_Q95_SHIFT,
    CALIBRATION_ROBUST_LOGIT,
    CALIBRATION_FEATURE_SHIFT,
    CALIBRATION_GUARDED_UPWARD,
)
FUSION_LINEAR = "linear"
FUSION_INTERACTION = "interaction"
FUSION_QUADRATIC = "quadratic"
FUSION_MODES = (
    FUSION_LINEAR,
    FUSION_INTERACTION,
    FUSION_QUADRATIC,
)
RUNTIME_READY_FEATURES = tuple(FEATURE_NAMES)
HOST_ONLY_FEATURES = tuple(CANDIDATE_FEATURES)


class ReplayError(RuntimeError):
    """Raised when the requested replay cannot run."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--features",
        action="append",
        default=[],
        help="candidate feature set as feature_a[,feature_b[,feature_c]]",
    )
    parser.add_argument(
        "--fusion",
        action="append",
        choices=FUSION_MODES,
        default=[],
        help=(
            "research fusion surface; repeat for a comparison "
            f"(default: {FUSION_LINEAR}). Non-linear modes require two features"
        ),
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="emit the full replay payload as JSON",
    )
    parser.add_argument(
        "--no-baseline",
        action="store_true",
        help="skip the current Lightweight pair baseline and its replay rows",
    )
    parser.add_argument(
        "--external-data-dir",
        action="append",
        type=Path,
        default=[],
        help=(
            "independent dataset catalog to replay only after fitting and "
            "ranking; repeat for multiple holdouts"
        ),
    )
    parser.add_argument(
        "--external-diagnostic-all-phy",
        action="append",
        type=Path,
        default=[],
        help=(
            "external data directory whose explicit non-production PHY rows "
            "should be retained for diagnostic replay"
        ),
    )
    parser.add_argument(
        "--splits",
        type=int,
        default=5,
        help="grouped OOF folds for the operating point (default: 5)",
    )
    parser.add_argument(
        "--fp-target",
        type=float,
        default=3.0,
        help="false-positive ceiling for the operating point (default: 3.0)",
    )
    parser.add_argument(
        "--min-session-recall",
        type=float,
        default=0.0,
        help="reject operating points whose worst train session falls below this recall",
    )
    parser.add_argument(
        "--calibration",
        action="append",
        choices=CALIBRATION_MODES,
        default=[],
        help=(
            "calibration family to replay; repeat for a comparison "
            f"(default: {CALIBRATION_Q95_SHIFT})"
        ),
    )
    parser.add_argument(
        "--startup-strength",
        action="append",
        type=float,
        default=[],
        help=(
            "startup calibration strength to replay; repeat for a grid "
            f"(default: {LightweightDetector.STARTUP_STRENGTH})"
        ),
    )
    parser.add_argument(
        "--robust-scale-floor-ratio",
        action="append",
        type=float,
        default=[],
        help=(
            "minimum session IQR as a fraction of the training idle IQR for "
            "robust_logit; repeat for a grid (default: 0.25)"
        ),
    )
    parser.add_argument(
        "--feature-startup-quantile",
        action="append",
        type=float,
        default=[],
        help=(
            "per-feature startup location quantile for feature_shift; repeat "
            "for a grid (default: 0.5)"
        ),
    )
    parser.add_argument(
        "--upward-blocks",
        action="append",
        type=int,
        default=[],
        help=(
            "guarded quiet blocks required before upward recovery; repeat for "
            "a grid (default: 3)"
        ),
    )
    parser.add_argument(
        "--upward-quantile",
        action="append",
        type=float,
        default=[],
        help=(
            "within-block quantile used by guarded upward recovery; repeat "
            "for a grid (default: 0.95)"
        ),
    )
    parser.add_argument(
        "--upward-max-positive-fraction",
        action="append",
        type=float,
        default=[],
        help=(
            "maximum positive fraction for a block to count as quiet; repeat "
            "for a grid (default: 0.5)"
        ),
    )
    parser.add_argument(
        "--settle-margin-logits",
        action="append",
        type=float,
        default=[],
        help=(
            "settled-level margin in logits; repeat for a grid "
            f"(default: {LightweightDetector.SETTLE_MARGIN_LOGITS})"
        ),
    )
    parser.add_argument(
        "--include-train-empty",
        action="store_true",
        help=(
            "include train-role empty recordings as idle hard negatives in "
            "the coefficient and operating-point fit"
        ),
    )
    parser.add_argument(
        "--stress-augment",
        action="store_true",
        help=(
            "rank clean-fitted candidates by their worst discovery result over "
            "clean, base, drift, burst-loss, and the combined packet recipe"
        ),
    )
    parser.add_argument(
        "--stress-scenario",
        action="append",
        choices=tuple(STRESS_SCENARIOS),
        default=[],
        help=(
            "packet-stress scenario to replay; repeat to select more than one. "
            "Passing this option enables stress replay without --stress-augment"
        ),
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=20,
        help="limit printed rankings to the top K candidates (default: 20)",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="suppress per-file extraction progress",
    )
    return parser.parse_args()


def parse_feature_sets(raw_specs: Sequence[str]) -> List[Tuple[str, ...]]:
    if not raw_specs:
        raise ReplayError("Pass at least one --features combination to replay")
    parsed: List[Tuple[str, ...]] = []
    for raw in raw_specs:
        names = [part.strip() for part in raw.split(",") if part.strip()]
        if len(names) < 1 or len(names) > 3:
            raise ReplayError(
                f"Invalid --features {raw!r}; expected 1, 2, or 3 distinct features"
            )
        if len(set(names)) != len(names):
            raise ReplayError(f"Invalid --features {raw!r}; features must differ")
        parsed.append(tuple(names))
    return parsed


def transform_fusion_rows(rows: np.ndarray, mode: str) -> np.ndarray:
    """Map two extracted features to a small research-only fusion surface."""
    values = np.asarray(rows, dtype=np.float64)
    if values.ndim != 2:
        raise ReplayError("Fusion rows must be a two-dimensional matrix")
    if mode == FUSION_LINEAR:
        return values
    if mode not in FUSION_MODES:
        raise ReplayError(f"Unknown fusion mode: {mode}")
    if values.shape[1] != 2:
        raise ReplayError(f"Fusion mode {mode!r} requires exactly two features")
    first = values[:, 0]
    second = values[:, 1]
    interaction = first * second
    if mode == FUSION_INTERACTION:
        return np.column_stack((first, second, interaction))
    return np.column_stack(
        (first, second, interaction, first * first, second * second)
    )


def resolve_catalog_entry_path(
    label: str,
    entry: Mapping[str, Any],
    dataset_root: Optional[Path] = None,
) -> Path:
    """Resolve a primary or explicitly external catalog entry."""
    if dataset_root is None:
        return resolve_entry_path(label, dict(entry))
    relative_path = entry.get("relative_path")
    if relative_path:
        return dataset_root / str(relative_path)
    filename = entry.get("filename")
    if not filename:
        raise ReplayError(f"Catalog entry for {label!r} has no filename")
    return dataset_root / label / str(filename)


def iter_replay_pairs(
    dataset_info_path: Optional[Path] = None,
    dataset_root: Optional[Path] = None,
) -> List[Dict[str, Any]]:
    files = load_dataset_info(dataset_info_path)["files"]
    motion_by_name = {entry["filename"]: entry for entry in files["motion"]}
    pairs: List[Dict[str, Any]] = []
    for static_entry in files["static_presence"]:
        if bool(static_entry.get("synthetic")):
            continue
        motion_name = static_entry.get("optimal_pair_motion_file")
        motion_entry = motion_by_name.get(motion_name) if motion_name else None
        if motion_entry is None or bool(motion_entry.get("synthetic")):
            continue
        role = paired_dataset_role(
            static_entry,
            motion_entry,
            admitted_roles=REPLAY_ROLES,
        )
        if role is None:
            continue
        pairs.append(
            {
                "session": static_entry["filename"],
                "chip": str(static_entry.get("chip", "unknown")).upper(),
                "role": role,
                "low_rssi": bool(static_entry.get("low_rssi"))
                or bool(motion_entry.get("low_rssi")),
                "static_path": resolve_catalog_entry_path(
                    "static_presence", static_entry, dataset_root
                ),
                "motion_path": resolve_catalog_entry_path(
                    "motion", motion_entry, dataset_root
                ),
            }
        )
    if not pairs:
        raise ReplayError("No real paired datasets found for replay")
    return pairs


def iter_empty_replays(
    dataset_info_path: Optional[Path] = None,
    dataset_root: Optional[Path] = None,
) -> List[Dict[str, Any]]:
    files = load_dataset_info(dataset_info_path)["files"]
    empties: List[Dict[str, Any]] = []
    for entry in files["empty"]:
        if bool(entry.get("synthetic")):
            continue
        role = dataset_role(entry)
        if role not in REPLAY_ROLES:
            continue
        empties.append(
            {
                "session": entry["filename"],
                "chip": str(entry.get("chip", "unknown")).upper(),
                "role": role,
                "path": resolve_catalog_entry_path("empty", entry, dataset_root),
            }
        )
    return empties


def extract_window_features(
    packets: Sequence[Mapping[str, Any]],
    feature_names: Sequence[str],
) -> Tuple[np.ndarray, np.ndarray]:
    window_packets = detector_window_packets(packets)
    interval_us = measure_packet_interval_us(packets)
    extractor = feature_cache.StreamingFeatureExtractor(
        feature_names,
        window_packets=window_packets,
        packet_interval_us=interval_us,
    )
    target_pps = target_pps_for_packets(packets, interval_us)
    cadence = RuntimeMotionPolicy(
        evaluation_interval_ms=feature_cache.EVALUATION_INTERVAL_MS,
        segmentation_window_size_ms=feature_cache.SEGMENTATION_WINDOW_SIZE_MS,
    )
    rows: List[Sequence[float]] = []
    deoverlapped: List[bool] = []
    since_window = 0
    for admission in iter_temporal_admissions(
        packets,
        target_pps=target_pps,
        window_size_ms=feature_cache.SEGMENTATION_WINDOW_SIZE_MS,
        fallback_interval_us=interval_us,
    ):
        packet = admission.packet
        if admission.reset_required:
            extractor = feature_cache.StreamingFeatureExtractor(
                feature_names,
                window_packets=window_packets,
                packet_interval_us=interval_us,
            )
            cadence.reset()
            since_window = 0
        elif admission.missing_slots_before:
            extractor.advance_missing_slots(admission.missing_slots_before)
        cadence.note_packet(elapsed_us=admission.coverage_us)
        should_evaluate = cadence.should_evaluate()
        if should_evaluate:
            cadence.after_evaluation()
        values = extractor.process_packet(
            packet["csi_data"],
            packet=packet,
            timestamp_us=admission.timestamp_us,
        )
        since_window += max(
            1,
            int(round(admission.coverage_us / float(interval_us))),
        )
        if not should_evaluate or values is None:
            continue
        rows.append(values)
        deoverlapped.append(since_window >= window_packets)
        if since_window >= window_packets:
            since_window = 0
    return (
        np.asarray(rows, dtype=np.float64).reshape(-1, len(feature_names)),
        np.asarray(deoverlapped, dtype=bool),
    )


def build_replay_cache(
    paths: Iterable[Path],
    feature_names: Sequence[str],
    *,
    quiet: bool,
    packet_augmentation: Optional[Mapping[str, Any]] = None,
    augmentation_seed: Optional[int] = None,
    keep_all_phy: bool = False,
) -> Dict[str, Dict[str, np.ndarray]]:
    unique_paths = sorted({str(path) for path in paths})
    cache: Dict[str, Dict[str, np.ndarray]] = {}
    cache_hits = 0
    runtime_ready = all(name in RUNTIME_READY_FEATURES for name in feature_names)
    for index, path_text in enumerate(unique_paths, start=1):
        path = Path(path_text)
        if not quiet:
            print(f"  [{index}/{len(unique_paths)}] {path.name}", flush=True)
        packets_factory = None
        packet_stream_provenance = (
            {"packet_view": "all_explicit_phy"} if keep_all_phy else None
        )
        if packet_augmentation:
            prepared_packets = None

            def packets_factory(source_path: Path = path):
                nonlocal prepared_packets
                if prepared_packets is not None:
                    return prepared_packets
                record = {
                    "path": source_path,
                    "packets": (
                        load_npz_as_packets(source_path, keep_all_phy=True)
                        if keep_all_phy
                        else load_npz_as_packets(source_path)
                    ),
                }
                prepared_packets = augmentation._prepare_feature_packets_for_record(
                    record,
                    packet_augmentation=packet_augmentation,
                    augmentation_seed=augmentation_seed,
                )
                return prepared_packets

            packet_stream_provenance = augmentation._packet_augmentation_stream_provenance(
                packet_augmentation,
                augmentation_seed,
            )
            if keep_all_phy:
                packet_stream_provenance = dict(packet_stream_provenance)
                packet_stream_provenance["packet_view"] = "all_explicit_phy"
        elif keep_all_phy:
            prepared_packets = None

            def packets_factory(source_path: Path = path):
                nonlocal prepared_packets
                if prepared_packets is None:
                    prepared_packets = load_npz_as_packets(
                        source_path,
                        keep_all_phy=True,
                    )
                return prepared_packets
        if runtime_ready:
            replay_rows = load_or_compute_ml_replay_rows(
                path,
                packets_factory=packets_factory,
                selected_subcarriers=config.DEFAULT_SUBCARRIERS,
                window_size=None,
                feature_names=feature_names,
                sample_contract="replay_tick",
                stream_provenance=packet_stream_provenance,
            )
        else:
            host_stream_provenance = feature_cache._host_feature_stream_provenance(
                feature_names,
                packet_augmentation=packet_augmentation,
                augmentation_seed=augmentation_seed,
            )
            if keep_all_phy:
                host_stream_provenance = dict(host_stream_provenance)
                host_stream_provenance["packet_view"] = "all_explicit_phy"
            replay_rows = feature_cache.load_or_compute_host_feature_rows(
                path,
                packets_factory=packets_factory,
                feature_names=feature_names,
                sample_contract="replay_tick",
                stream_provenance=host_stream_provenance,
            )

        rows = np.asarray(replay_rows["X"], dtype=np.float64)
        packet_index = np.asarray(replay_rows["packet_index"], dtype=np.int64)
        reset_index = np.asarray(replay_rows["reset_index"], dtype=np.int64)
        packets_for_window = (
            packets_factory()
            if packets_factory is not None
            else (
                load_npz_as_packets(path, keep_all_phy=True)
                if keep_all_phy
                else load_npz_as_packets(path)
            )
        )
        window_packets = detector_window_packets(packets_for_window)
        deoverlapped = np.zeros(len(rows), dtype=bool)
        last_boundary_by_reset: Dict[int, int] = {}
        for row_index, (packet, reset) in enumerate(zip(packet_index, reset_index, strict=True)):
            last_boundary = last_boundary_by_reset.get(int(reset))
            if (
                last_boundary is None
                or int(packet) - last_boundary >= window_packets
            ):
                deoverlapped[row_index] = True
                last_boundary_by_reset[int(reset)] = int(packet)
        cache_hit = bool(replay_rows.get("cache_hit", False))
        cache_hits += int(cache_hit)
        cache[path_text] = {
            "rows": rows,
            "deoverlapped": deoverlapped,
            "cache_hit": cache_hit,
        }
    if not quiet:
        print(
            f"  Persistent feature cache: {cache_hits} hit(s), "
            f"{len(unique_paths) - cache_hits} miss(es)",
            flush=True,
        )
    return cache


def build_training_corpus(
    pairs: Sequence[Mapping[str, Any]],
    cache: Mapping[str, Mapping[str, np.ndarray]],
    feature_index: Mapping[str, int],
    combination: Sequence[str],
) -> Dict[str, np.ndarray]:
    cols = [feature_index[name] for name in combination]
    features: List[np.ndarray] = []
    labels: List[np.ndarray] = []
    sessions: List[np.ndarray] = []
    chips: List[np.ndarray] = []
    deoverlapped: List[np.ndarray] = []
    for pair in pairs:
        if pair["role"] != "train":
            continue
        for path_key, label in (
            (str(pair["static_path"]), IDLE_LABEL),
            (str(pair["motion_path"]), MOTION_LABEL),
        ):
            rows = np.asarray(cache[path_key]["rows"][:, cols], dtype=np.float64)
            if rows.size == 0:
                continue
            features.append(rows)
            labels.append(np.full(len(rows), label, dtype=np.int8))
            sessions.append(np.full(len(rows), pair["session"], dtype=object))
            chips.append(np.full(len(rows), pair["chip"], dtype=object))
            deoverlapped.append(np.asarray(cache[path_key]["deoverlapped"], dtype=bool))
    if not features:
        raise ReplayError(f"Replay produced no train rows for {tuple(combination)!r}")
    return {
        "x": np.vstack(features),
        "y": np.concatenate(labels),
        "session": np.concatenate(sessions),
        "chip": np.concatenate(chips),
        "deoverlapped": np.concatenate(deoverlapped),
    }


def append_training_empty_rows(
    corpus: Mapping[str, np.ndarray],
    empties: Sequence[Mapping[str, Any]],
    cache: Mapping[str, Mapping[str, np.ndarray]],
    feature_index: Mapping[str, int],
    combination: Sequence[str],
) -> Dict[str, np.ndarray]:
    """Append train-role empty streams as grouped idle hard negatives."""
    cols = [feature_index[name] for name in combination]
    features = [np.asarray(corpus["x"], dtype=np.float64)]
    labels = [np.asarray(corpus["y"], dtype=np.int8)]
    sessions = [np.asarray(corpus["session"], dtype=object)]
    chips = [np.asarray(corpus["chip"], dtype=object)]
    deoverlapped = [np.asarray(corpus["deoverlapped"], dtype=bool)]
    for empty in empties:
        if empty["role"] != "train":
            continue
        path_key = str(empty["path"])
        rows = np.asarray(cache[path_key]["rows"][:, cols], dtype=np.float64)
        if rows.size == 0:
            continue
        features.append(rows)
        labels.append(np.full(len(rows), IDLE_LABEL, dtype=np.int8))
        sessions.append(
            np.full(len(rows), empty["session"], dtype=object)
        )
        chips.append(np.full(len(rows), empty["chip"], dtype=object))
        deoverlapped.append(
            np.asarray(cache[path_key]["deoverlapped"], dtype=bool)
        )
    return {
        "x": np.vstack(features),
        "y": np.concatenate(labels),
        "session": np.concatenate(sessions),
        "chip": np.concatenate(chips),
        "deoverlapped": np.concatenate(deoverlapped),
    }


def probability(logit: float) -> float:
    if logit < -20.0:
        return 0.0
    if logit > 20.0:
        return 1.0
    return 1.0 / (1.0 + math.exp(-logit))


def probability_logit(value: float) -> float:
    clipped = min(1.0 - 1e-12, max(1e-12, float(value)))
    return math.log(clipped / (1.0 - clipped))


def robust_location_scale(
    values: np.ndarray,
    *,
    minimum_scale: float = 1e-6,
) -> Tuple[float, float]:
    """Return a median/IQR summary with a finite positive scale."""
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        raise ReplayError("Robust calibration requires at least one value")
    location = float(np.median(values))
    scale = float(np.quantile(values, 0.75) - np.quantile(values, 0.25))
    return location, max(float(minimum_scale), scale)


def calibration_policy(
    mode: str,
    *,
    startup_strength: float,
    robust_scale_floor_ratio: float = 0.25,
    feature_startup_quantile: float = 0.5,
    upward_blocks: int = 3,
    upward_quantile: float = 0.95,
    upward_max_positive_fraction: float = 0.5,
) -> Dict[str, Any]:
    """Build a serializable research calibration policy."""
    if mode not in CALIBRATION_MODES:
        raise ReplayError(f"Unknown calibration mode: {mode}")
    return {
        "mode": mode,
        "startup_strength": float(startup_strength),
        "robust_scale_floor_ratio": float(robust_scale_floor_ratio),
        "feature_startup_quantile": float(feature_startup_quantile),
        "upward_blocks": int(upward_blocks),
        "upward_quantile": float(upward_quantile),
        "upward_max_positive_fraction": float(upward_max_positive_fraction),
    }


def calibration_references(
    fit_x: np.ndarray,
    fit_y: np.ndarray,
    coefficients: Mapping[str, Any],
    oof_scores: np.ndarray,
    dense_y: np.ndarray,
    policy: Mapping[str, Any],
) -> Dict[str, Any]:
    """Fit training-only reference statistics for one calibration family."""
    idle_x = np.asarray(fit_x[fit_y == IDLE_LABEL], dtype=np.float64)
    idle_logits = logits(idle_x, coefficients)
    references: Dict[str, Any] = {
        "idle_q95_logit": float(
            np.quantile(idle_logits, LightweightDetector.STARTUP_QUANTILE)
        )
    }
    mode = str(policy["mode"])
    if mode == CALIBRATION_ROBUST_LOGIT:
        final_location, final_scale = robust_location_scale(idle_logits)
        oof_location, oof_scale = robust_location_scale(
            np.asarray(oof_scores[dense_y == IDLE_LABEL], dtype=np.float64)
        )
        references.update(
            {
                "final_location": final_location,
                "final_scale": final_scale,
                "oof_location": oof_location,
                "oof_scale": oof_scale,
            }
        )
    elif mode == CALIBRATION_FEATURE_SHIFT:
        quantile = float(policy["feature_startup_quantile"])
        references["feature_location"] = [
            float(value) for value in np.quantile(idle_x, quantile, axis=0)
        ]
    return references


def feature_location_contribution(
    rows: np.ndarray,
    coefficients: Mapping[str, Any],
    quantile: float,
    sample_limit: int,
) -> Optional[float]:
    prefix_count = min(len(rows), int(sample_limit))
    if prefix_count <= 0:
        return None
    location = np.quantile(
        np.asarray(rows[:prefix_count], dtype=np.float64),
        float(quantile),
        axis=0,
    )
    weight = np.asarray(coefficients["weight"], dtype=np.float64)
    scale = np.asarray(coefficients["scale"], dtype=np.float64)
    return float(np.dot(location, weight / scale))


def effective_robust_stats(
    series_logits: np.ndarray,
    references: Mapping[str, Any],
    policy: Mapping[str, Any],
    startup_sample_limit: int,
) -> Tuple[float, float]:
    prefix_count = min(len(series_logits), int(startup_sample_limit))
    reference_location = float(references["final_location"])
    reference_scale = float(references["final_scale"])
    if prefix_count <= 0:
        return reference_location, reference_scale
    floor = reference_scale * float(policy["robust_scale_floor_ratio"])
    session_location, session_scale = robust_location_scale(
        series_logits[:prefix_count],
        minimum_scale=floor,
    )
    strength = float(policy["startup_strength"])
    effective_location = reference_location + strength * (
        session_location - reference_location
    )
    scale_ratio = max(session_scale, floor) / reference_scale
    effective_scale = reference_scale * math.pow(scale_ratio, strength)
    return effective_location, effective_scale


def startup_evaluation_limit(
    calibration_duration_ms: int,
    window_size_ms: int,
    evaluation_interval_ms: int,
    sample_limit: int,
) -> int:
    """Return how many ready evaluations production startup can observe.

    The detector is evaluated throughout calibration, but it cannot emit a
    feature row before one full window is ready. With the production 10 s
    calibration and 1 s window this is 37 rows, not 10 and not the detector's
    storage cap (64).
    """
    calibration_duration_us = max(0, int(calibration_duration_ms)) * 1000
    window_duration_us = max(1, int(window_size_ms)) * 1000
    evaluation_interval_us = max(1, int(evaluation_interval_ms)) * 1000
    sample_limit = max(0, int(sample_limit))
    first_ready_tick_us = (
        (window_duration_us + evaluation_interval_us - 1) // evaluation_interval_us
    ) * evaluation_interval_us
    if first_ready_tick_us > calibration_duration_us:
        return 0
    available = 1 + (
        calibration_duration_us - first_ready_tick_us
    ) // evaluation_interval_us
    return min(sample_limit, available)


def session_centered_replay_scores(
    scores: np.ndarray,
    y: np.ndarray,
    sessions: np.ndarray,
    startup_strength: float,
    startup_sample_limit: int,
) -> np.ndarray:
    """Center dense OOF logits with the same quiet-prefix rule as replay."""
    centered = np.asarray(scores, dtype=np.float64).copy()
    sessions = np.asarray(sessions, dtype=object)
    for session_name in np.unique(sessions):
        session_mask = sessions == session_name
        idle_scores = scores[session_mask & (y == IDLE_LABEL)][
            :startup_sample_limit
        ]
        if idle_scores.size == 0:
            continue
        session_q95 = float(
            np.quantile(idle_scores, LightweightDetector.STARTUP_QUANTILE)
        )
        centered[session_mask] -= float(startup_strength) * session_q95
    return centered


def calibrated_replay_scores(
    scores: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    sessions: np.ndarray,
    coefficients: Mapping[str, Any],
    policy: Mapping[str, Any],
    references: Mapping[str, Any],
    startup_sample_limit: int,
) -> np.ndarray:
    """Map dense OOF scores into the selected causal calibration frame."""
    mode = str(policy["mode"])
    strength = float(policy["startup_strength"])
    if mode in (CALIBRATION_Q95_SHIFT, CALIBRATION_GUARDED_UPWARD):
        return session_centered_replay_scores(
            scores,
            y,
            sessions,
            strength,
            startup_sample_limit,
        )

    if mode == CALIBRATION_ROBUST_LOGIT:
        calibrated = (
            np.asarray(scores, dtype=np.float64)
            - float(references["oof_location"])
        ) / float(references["oof_scale"])
    else:
        calibrated = np.asarray(scores, dtype=np.float64).copy()
    sessions = np.asarray(sessions, dtype=object)
    for session_name in np.unique(sessions):
        session_mask = sessions == session_name
        idle_mask = session_mask & (y == IDLE_LABEL)
        idle_indices = np.flatnonzero(idle_mask)[:startup_sample_limit]
        if idle_indices.size == 0:
            continue
        if mode == CALIBRATION_FEATURE_SHIFT:
            contribution = feature_location_contribution(
                np.asarray(x[idle_indices], dtype=np.float64),
                coefficients,
                float(policy["feature_startup_quantile"]),
                startup_sample_limit,
            )
            if contribution is not None:
                calibrated[session_mask] -= strength * contribution
            continue

        if mode == CALIBRATION_ROBUST_LOGIT:
            reference_location = float(references["oof_location"])
            reference_scale = float(references["oof_scale"])
            floor = reference_scale * float(policy["robust_scale_floor_ratio"])
            session_location, session_scale = robust_location_scale(
                scores[idle_indices],
                minimum_scale=floor,
            )
            effective_location = reference_location + strength * (
                session_location - reference_location
            )
            effective_scale = reference_scale * math.pow(
                max(session_scale, floor) / reference_scale,
                strength,
            )
            calibrated[session_mask] = (
                np.asarray(scores[session_mask], dtype=np.float64)
                - effective_location
            ) / effective_scale
            continue

        raise ReplayError(f"Unsupported calibration mode: {mode}")
    return calibrated


def dense_out_of_fold_logits(
    fit_x: np.ndarray,
    fit_y: np.ndarray,
    fit_weights: np.ndarray,
    fit_sessions: np.ndarray,
    dense_x: np.ndarray,
    dense_sessions: np.ndarray,
    splits: int,
) -> Optional[np.ndarray]:
    """Fit folds on de-overlapped rows, then score every held-out runtime tick."""
    from sklearn.model_selection import StratifiedGroupKFold

    if len(np.unique(fit_sessions)) < splits:
        return None
    oof = np.full(len(dense_x), np.nan, dtype=np.float64)
    splitter = StratifiedGroupKFold(
        n_splits=splits,
        shuffle=True,
        random_state=0,
    )
    for train_index, test_index in splitter.split(
        fit_x,
        fit_y,
        groups=fit_sessions,
    ):
        fold = fit_coefficients(
            fit_x[train_index],
            fit_y[train_index],
            fit_weights[train_index],
        )
        held_out_sessions = np.unique(fit_sessions[test_index])
        dense_test = np.isin(dense_sessions, held_out_sessions)
        oof[dense_test] = logits(dense_x[dense_test], fold)
    return oof if not np.isnan(oof).any() else None


def startup_threshold(
    series_logits: np.ndarray,
    base_threshold: float,
    idle_q95: float,
    startup_strength: float,
    startup_sample_limit: int,
) -> float:
    prefix_count = min(len(series_logits), int(startup_sample_limit))
    if prefix_count <= 0:
        return float(base_threshold)
    startup_q95 = float(
        np.quantile(series_logits[:prefix_count], LightweightDetector.STARTUP_QUANTILE)
    )
    base_logit = float(np.log(base_threshold / (1.0 - base_threshold)))
    adapted_logit = base_logit + float(startup_strength) * (
        startup_q95 - idle_q95
    )
    return probability(adapted_logit)


def calibrated_startup_threshold(
    rows: np.ndarray,
    series_logits: np.ndarray,
    coefficients: Mapping[str, Any],
    base_threshold: float,
    policy: Mapping[str, Any],
    references: Mapping[str, Any],
    startup_sample_limit: int,
) -> float:
    """Apply one startup family using only the stream's causal prefix."""
    mode = str(policy["mode"])
    strength = float(policy["startup_strength"])
    if mode in (CALIBRATION_Q95_SHIFT, CALIBRATION_GUARDED_UPWARD):
        return startup_threshold(
            series_logits,
            base_threshold,
            float(references["idle_q95_logit"]),
            strength,
            startup_sample_limit,
        )
    if mode == CALIBRATION_FEATURE_SHIFT:
        session_contribution = feature_location_contribution(
            rows,
            coefficients,
            float(policy["feature_startup_quantile"]),
            startup_sample_limit,
        )
        if session_contribution is None:
            return float(base_threshold)
        reference_location = np.asarray(
            references["feature_location"], dtype=np.float64
        )
        weight = np.asarray(coefficients["weight"], dtype=np.float64)
        scale = np.asarray(coefficients["scale"], dtype=np.float64)
        reference_contribution = float(np.dot(reference_location, weight / scale))
        return probability(
            probability_logit(base_threshold)
            + strength * (session_contribution - reference_contribution)
        )
    if mode == CALIBRATION_ROBUST_LOGIT:
        effective_location, effective_scale = effective_robust_stats(
            series_logits,
            references,
            policy,
            startup_sample_limit,
        )
        reference_location = float(references["final_location"])
        reference_scale = float(references["final_scale"])
        standardized_threshold = (
            probability_logit(base_threshold) - reference_location
        ) / reference_scale
        return probability(
            effective_location + effective_scale * standardized_threshold
        )
    raise ReplayError(f"Unsupported calibration mode: {mode}")


def replay_one_stream(
    rows: np.ndarray,
    coefficients: Dict[str, Any],
    base_threshold: float,
    idle_q95: float,
    *,
    startup_strength: float,
    startup_sample_limit: int,
    settle_margin_logits: float,
    initial_threshold: Optional[float] = None,
    calibration: Optional[Mapping[str, Any]] = None,
    references: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    if rows.size == 0:
        return {
            "eval_count": 0,
            "positive_count": 0,
            "positive_rate": 0.0,
            "threshold": 0.0,
            "initial_threshold": 0.0,
        }
    series_logits = logits(rows, coefficients)
    policy = (
        dict(calibration)
        if calibration is not None
        else calibration_policy(
            CALIBRATION_Q95_SHIFT,
            startup_strength=startup_strength,
        )
    )
    calibration_refs = (
        dict(references)
        if references is not None
        else {"idle_q95_logit": float(idle_q95)}
    )
    threshold = (
        calibrated_startup_threshold(
            rows,
            series_logits,
            coefficients,
            base_threshold,
            policy,
            calibration_refs,
            startup_sample_limit,
        )
        if initial_threshold is None
        else float(initial_threshold)
    )
    initial_threshold_value = float(threshold)
    initial_threshold_logit = probability_logit(initial_threshold_value)
    settle_blocks: List[float] = []
    block_max = -1e9
    block_values: List[float] = []
    block_positive_count = 0
    upward_blocks: List[float] = []
    block_count = 0
    positive_count = 0
    for logit_value in series_logits:
        was_positive = probability(float(logit_value)) > threshold
        if logit_value > block_max:
            block_max = float(logit_value)
        block_values.append(float(logit_value))
        if was_positive:
            block_positive_count += 1
        block_count += 1
        if block_count >= LightweightDetector.SETTLE_BLOCK_EVALUATIONS:
            completed_block_count = block_count
            settle_blocks.append(block_max)
            if len(settle_blocks) > LightweightDetector.SETTLE_BLOCKS:
                settle_blocks.pop(0)
            block_max = -1e9
            block_count = 0
            if len(settle_blocks) >= LightweightDetector.SETTLE_BLOCKS:
                settled_logit = sorted(settle_blocks)[len(settle_blocks) // 2]
                settled_threshold = probability(
                    settled_logit + float(settle_margin_logits)
                )
                if settled_threshold < threshold:
                    threshold = settled_threshold
            if str(policy["mode"]) == CALIBRATION_GUARDED_UPWARD:
                positive_fraction = block_positive_count / completed_block_count
                if positive_fraction <= float(
                    policy["upward_max_positive_fraction"]
                ):
                    upward_blocks.append(
                        float(
                            np.quantile(
                                block_values,
                                float(policy["upward_quantile"]),
                            )
                        )
                    )
                    if len(upward_blocks) > int(policy["upward_blocks"]):
                        upward_blocks.pop(0)
                else:
                    upward_blocks = []
                if len(upward_blocks) >= int(policy["upward_blocks"]):
                    recovered_level = float(np.median(upward_blocks))
                    candidate_logit = min(
                        initial_threshold_logit,
                        recovered_level + float(settle_margin_logits),
                    )
                    if candidate_logit > probability_logit(threshold):
                        threshold = probability(candidate_logit)
            block_values = []
            block_positive_count = 0
        if probability(float(logit_value)) > threshold:
            positive_count += 1
    eval_count = int(len(series_logits))
    return {
        "eval_count": eval_count,
        "positive_count": positive_count,
        "positive_rate": 100.0 * positive_count / eval_count if eval_count else 0.0,
        "threshold": float(threshold),
        "initial_threshold": initial_threshold_value,
    }


def replay_one_pair(
    static_rows: np.ndarray,
    motion_rows: np.ndarray,
    coefficients: Dict[str, Any],
    base_threshold: float,
    idle_q95: float,
    *,
    startup_strength: float,
    startup_sample_limit: int,
    settle_margin_logits: float,
    calibration: Optional[Mapping[str, Any]] = None,
    references: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    policy = (
        dict(calibration)
        if calibration is not None
        else calibration_policy(
            CALIBRATION_Q95_SHIFT,
            startup_strength=startup_strength,
        )
    )
    calibration_refs = (
        dict(references)
        if references is not None
        else {"idle_q95_logit": float(idle_q95)}
    )
    static_logits = logits(static_rows, coefficients)
    initial_threshold = calibrated_startup_threshold(
        static_rows,
        static_logits,
        coefficients,
        base_threshold,
        policy,
        calibration_refs,
        startup_sample_limit,
    )
    static_metrics = replay_one_stream(
        static_rows,
        coefficients,
        base_threshold,
        idle_q95,
        startup_strength=startup_strength,
        startup_sample_limit=startup_sample_limit,
        settle_margin_logits=settle_margin_logits,
        initial_threshold=initial_threshold,
        calibration=policy,
        references=calibration_refs,
    )
    motion_metrics = replay_one_stream(
        motion_rows,
        coefficients,
        base_threshold,
        idle_q95,
        startup_strength=startup_strength,
        startup_sample_limit=startup_sample_limit,
        settle_margin_logits=settle_margin_logits,
        initial_threshold=initial_threshold,
        calibration=policy,
        references=calibration_refs,
    )
    return {
        "static": static_metrics,
        "motion": motion_metrics,
        "initial_threshold": float(initial_threshold),
    }


def aggregate_paired(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    if not rows:
        return {
            "count": 0,
            "weighted_recall": float("nan"),
            "weighted_fp_rate": float("nan"),
            "mean_recall": float("nan"),
            "mean_fp_rate": float("nan"),
            "worst_recall": float("nan"),
            "worst_low_rssi_recall": float("nan"),
            "max_fp_rate": float("nan"),
            "worst_recall_session": None,
            "max_fp_session": None,
        }
    recall_weight_total = sum(int(row["motion_eval_count"]) for row in rows)
    fp_weight_total = sum(int(row["static_eval_count"]) for row in rows)
    weighted_recall = (
        sum(float(row["recall"]) * int(row["motion_eval_count"]) for row in rows)
        / recall_weight_total
        if recall_weight_total
        else float("nan")
    )
    weighted_fp_rate = (
        sum(float(row["fp_rate"]) * int(row["static_eval_count"]) for row in rows)
        / fp_weight_total
        if fp_weight_total
        else float("nan")
    )
    worst_recall_row = min(rows, key=lambda row: float(row["recall"]))
    max_fp_row = max(rows, key=lambda row: float(row["fp_rate"]))
    low_rssi_rows = [row for row in rows if bool(row["low_rssi"])]
    worst_low_rssi = (
        min(float(row["recall"]) for row in low_rssi_rows)
        if low_rssi_rows
        else float("nan")
    )
    return {
        "count": len(rows),
        "weighted_recall": float(weighted_recall),
        "weighted_fp_rate": float(weighted_fp_rate),
        "mean_recall": float(np.mean([float(row["recall"]) for row in rows])),
        "mean_fp_rate": float(np.mean([float(row["fp_rate"]) for row in rows])),
        "worst_recall": float(worst_recall_row["recall"]),
        "worst_low_rssi_recall": float(worst_low_rssi),
        "max_fp_rate": float(max_fp_row["fp_rate"]),
        "worst_recall_session": str(worst_recall_row["session"]),
        "max_fp_session": str(max_fp_row["session"]),
    }


def aggregate_idle(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    if not rows:
        return {
            "count": 0,
            "mean_fp_rate": float("nan"),
            "max_fp_rate": float("nan"),
            "worst_session": None,
        }
    worst_row = max(rows, key=lambda row: float(row["fp_rate"]))
    return {
        "count": len(rows),
        "mean_fp_rate": float(np.mean([float(row["fp_rate"]) for row in rows])),
        "max_fp_rate": float(worst_row["fp_rate"]),
        "worst_session": str(worst_row["session"]),
    }


def replay_score(primary_pairs: Mapping[str, Any], primary_idle: Mapping[str, Any]) -> float:
    penalties = 0.0
    penalties += max(0.0, 95.0 - float(primary_pairs["worst_recall"])) * 3.0
    low_rssi = float(primary_pairs["worst_low_rssi_recall"])
    if not math.isnan(low_rssi):
        penalties += max(0.0, 85.0 - low_rssi) * 4.0
    penalties += max(0.0, float(primary_pairs["weighted_fp_rate"]) - 3.0) * 2.0
    penalties += max(0.0, float(primary_idle["max_fp_rate"]) - 6.0) * 4.0
    return penalties


def evaluate_candidate(
    combination: Sequence[str],
    fusion: str,
    pairs: Sequence[Mapping[str, Any]],
    empties: Sequence[Mapping[str, Any]],
    cache: Mapping[str, Mapping[str, np.ndarray]],
    feature_index: Mapping[str, int],
    args: argparse.Namespace,
    *,
    startup_strength: float,
    settle_margin_logits: float,
    calibration: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    combination = tuple(combination)
    runtime_ready = (
        fusion == FUSION_LINEAR
        and all(name in RUNTIME_READY_FEATURES for name in combination)
    )
    corpus = build_training_corpus(pairs, cache, feature_index, combination)
    include_train_empty = bool(
        getattr(args, "include_train_empty", False)
    )
    if include_train_empty:
        corpus = append_training_empty_rows(
            corpus,
            empties,
            cache,
            feature_index,
            combination,
        )
    x = transform_fusion_rows(corpus["x"], fusion)
    y = corpus["y"]
    deoverlapped = np.asarray(corpus["deoverlapped"], dtype=bool)
    fit_x, fit_y = x[deoverlapped], y[deoverlapped]
    fit_weights = balanced_sample_weights(
        fit_y,
        corpus["chip"][deoverlapped],
        corpus["session"][deoverlapped],
    )
    coefficients = fit_coefficients(fit_x, fit_y, fit_weights)
    all_weights = balanced_sample_weights(y, corpus["chip"], corpus["session"])
    oof = dense_out_of_fold_logits(
        fit_x,
        fit_y,
        fit_weights,
        corpus["session"][deoverlapped],
        x,
        corpus["session"],
        args.splits,
    )
    score_source = (
        f"grouped de-overlapped fit / dense OOF score ({args.splits} folds)"
    )
    if oof is None:
        score_source = "in-sample"
        oof = logits(x, coefficients)
    policy = (
        dict(calibration)
        if calibration is not None
        else calibration_policy(
            CALIBRATION_Q95_SHIFT,
            startup_strength=startup_strength,
        )
    )
    references = calibration_references(
        fit_x,
        fit_y,
        coefficients,
        oof,
        y,
        policy,
    )
    idle_q95 = float(references["idle_q95_logit"])
    startup_sample_limit = startup_evaluation_limit(
        config.CALIBRATION_DURATION_MS,
        config.SEGMENTATION_WINDOW_SIZE_MS,
        config.EVALUATION_INTERVAL_MS,
        LightweightDetector.STARTUP_SAMPLE_LIMIT,
    )
    centered = calibrated_replay_scores(
        oof,
        x,
        y,
        corpus["session"],
        coefficients,
        policy,
        references,
        startup_sample_limit,
    )
    centered_threshold, train_metrics = choose_base_threshold(
        centered,
        y,
        all_weights,
        session=corpus["session"],
        fp_target=args.fp_target,
        min_session_recall=args.min_session_recall,
    )
    centered_logit = float(np.log(centered_threshold / (1.0 - centered_threshold)))
    mode = str(policy["mode"])
    if mode in (CALIBRATION_Q95_SHIFT, CALIBRATION_GUARDED_UPWARD):
        base_logit = centered_logit + float(policy["startup_strength"]) * idle_q95
    elif mode == CALIBRATION_FEATURE_SHIFT:
        feature_location = np.asarray(
            references["feature_location"], dtype=np.float64
        )
        contribution = float(
            np.dot(
                feature_location,
                np.asarray(coefficients["weight"], dtype=np.float64)
                / np.asarray(coefficients["scale"], dtype=np.float64),
            )
        )
        base_logit = centered_logit + float(policy["startup_strength"]) * contribution
    elif mode == CALIBRATION_ROBUST_LOGIT:
        base_logit = float(references["final_location"]) + float(
            references["final_scale"]
        ) * centered_logit
    else:
        raise ReplayError(f"Unsupported calibration mode: {mode}")
    base_threshold = probability(base_logit)

    cols = [feature_index[name] for name in combination]
    paired_rows: List[Dict[str, Any]] = []
    for pair in pairs:
        static_rows = np.asarray(
            cache[str(pair["static_path"])]["rows"][:, cols],
            dtype=np.float64,
        )
        motion_rows = np.asarray(
            cache[str(pair["motion_path"])]["rows"][:, cols],
            dtype=np.float64,
        )
        static_rows = transform_fusion_rows(static_rows, fusion)
        motion_rows = transform_fusion_rows(motion_rows, fusion)
        replay_metrics = replay_one_pair(
            static_rows,
            motion_rows,
            coefficients,
            base_threshold,
            idle_q95,
            startup_strength=startup_strength,
            startup_sample_limit=startup_sample_limit,
            settle_margin_logits=settle_margin_logits,
            calibration=policy,
            references=references,
        )
        static_metrics = replay_metrics["static"]
        motion_metrics = replay_metrics["motion"]
        paired_rows.append(
            {
                "session": pair["session"],
                "role": pair["role"],
                "chip": pair["chip"],
                "low_rssi": bool(pair["low_rssi"]),
                "static_eval_count": int(static_metrics["eval_count"]),
                "motion_eval_count": int(motion_metrics["eval_count"]),
                "fp_rate": float(static_metrics["positive_rate"]),
                "recall": float(motion_metrics["positive_rate"]),
                "initial_threshold": float(replay_metrics["initial_threshold"]),
                "static_threshold": float(static_metrics["threshold"]),
                "motion_threshold": float(motion_metrics["threshold"]),
            }
        )
    idle_rows: List[Dict[str, Any]] = []
    for empty in empties:
        rows = np.asarray(cache[str(empty["path"])]["rows"][:, cols], dtype=np.float64)
        rows = transform_fusion_rows(rows, fusion)
        metrics = replay_one_stream(
            rows,
            coefficients,
            base_threshold,
            idle_q95,
            startup_strength=startup_strength,
            startup_sample_limit=startup_sample_limit,
            settle_margin_logits=settle_margin_logits,
            calibration=policy,
            references=references,
        )
        idle_rows.append(
            {
                "session": empty["session"],
                "role": empty["role"],
                "chip": empty["chip"],
                "fp_rate": float(metrics["positive_rate"]),
                "eval_count": int(metrics["eval_count"]),
                "threshold": float(metrics["threshold"]),
            }
        )

    discovery_pairs = [
        row for row in paired_rows if row["role"] in DISCOVERY_ROLES
    ]
    holdout_pairs = [
        row for row in paired_rows if row["role"] == HOLDOUT_ROLE
    ]
    exclude_pairs = [row for row in paired_rows if row["role"] == EXCLUDE_ROLE]
    discovery_idle = [
        row for row in idle_rows if row["role"] in DISCOVERY_ROLES
    ]
    holdout_idle = [
        row for row in idle_rows if row["role"] == HOLDOUT_ROLE
    ]
    exclude_idle = [row for row in idle_rows if row["role"] == EXCLUDE_ROLE]
    discovery_pair_summary = aggregate_paired(discovery_pairs)
    discovery_idle_summary = aggregate_idle(discovery_idle)
    holdout_pair_summary = aggregate_paired(holdout_pairs)
    holdout_idle_summary = aggregate_idle(holdout_idle)
    exclude_pair_summary = aggregate_paired(exclude_pairs)
    exclude_idle_summary = aggregate_idle(exclude_idle)
    return {
        "combination": list(combination),
        "combination_size": len(combination),
        "fusion": fusion,
        "runtime_ready": runtime_ready,
        "score": replay_score(
            discovery_pair_summary,
            discovery_idle_summary,
        ),
        "startup_strength": float(startup_strength),
        "calibration": dict(policy),
        "calibration_references": dict(references),
        "startup_sample_limit": int(startup_sample_limit),
        "settle_margin_logits": float(settle_margin_logits),
        "train_empty_hard_negatives": include_train_empty,
        "coefficients": {
            "center": [float(value) for value in coefficients["center"]],
            "scale": [float(value) for value in coefficients["scale"]],
            "weight": [float(value) for value in coefficients["weight"]],
            "intercept": float(coefficients["intercept"]),
        },
        "base_threshold": float(base_threshold),
        "train_idle_q95_logit": float(idle_q95),
        "train_operating_point": dict(train_metrics, source=score_source),
        "discovery": {
            "roles": list(DISCOVERY_ROLES),
            "paired": discovery_pair_summary,
            "idle": discovery_idle_summary,
        },
        "holdout": {
            "role": HOLDOUT_ROLE,
            "paired": holdout_pair_summary,
            "idle": holdout_idle_summary,
        },
        "exclude": {
            "paired": exclude_pair_summary,
            "idle": exclude_idle_summary,
        },
        "replay_rows": {
            "paired": paired_rows,
            "idle": idle_rows,
        },
    }


def evaluate_fixed_candidate(
    fitted: Mapping[str, Any],
    pairs: Sequence[Mapping[str, Any]],
    empties: Sequence[Mapping[str, Any]],
    cache: Mapping[str, Mapping[str, np.ndarray]],
    feature_index: Mapping[str, int],
) -> Dict[str, Any]:
    """Replay clean-fitted constants on another packet stream without refitting."""
    combination = tuple(str(name) for name in fitted["combination"])
    fusion = str(fitted.get("fusion", FUSION_LINEAR))
    cols = [feature_index[name] for name in combination]
    raw_coefficients = fitted["coefficients"]
    coefficients = {
        "center": np.asarray(raw_coefficients["center"], dtype=np.float64),
        "scale": np.asarray(raw_coefficients["scale"], dtype=np.float64),
        "weight": np.asarray(raw_coefficients["weight"], dtype=np.float64),
        "intercept": float(raw_coefficients["intercept"]),
    }
    base_threshold = float(fitted["base_threshold"])
    idle_q95 = float(fitted["train_idle_q95_logit"])
    startup_strength = float(fitted["startup_strength"])
    startup_sample_limit = int(fitted["startup_sample_limit"])
    settle_margin_logits = float(fitted["settle_margin_logits"])
    policy = dict(
        fitted.get(
            "calibration",
            calibration_policy(
                CALIBRATION_Q95_SHIFT,
                startup_strength=startup_strength,
            ),
        )
    )
    references = dict(
        fitted.get(
            "calibration_references",
            {"idle_q95_logit": idle_q95},
        )
    )

    paired_rows: List[Dict[str, Any]] = []
    for pair in pairs:
        static_rows = np.asarray(
            cache[str(pair["static_path"])]["rows"][:, cols], dtype=np.float64
        )
        motion_rows = np.asarray(
            cache[str(pair["motion_path"])]["rows"][:, cols], dtype=np.float64
        )
        static_rows = transform_fusion_rows(static_rows, fusion)
        motion_rows = transform_fusion_rows(motion_rows, fusion)
        replay_metrics = replay_one_pair(
            static_rows,
            motion_rows,
            coefficients,
            base_threshold,
            idle_q95,
            startup_strength=startup_strength,
            startup_sample_limit=startup_sample_limit,
            settle_margin_logits=settle_margin_logits,
            calibration=policy,
            references=references,
        )
        paired_rows.append(
            {
                "session": pair["session"],
                "role": pair["role"],
                "chip": pair["chip"],
                "low_rssi": bool(pair["low_rssi"]),
                "static_eval_count": int(replay_metrics["static"]["eval_count"]),
                "motion_eval_count": int(replay_metrics["motion"]["eval_count"]),
                "fp_rate": float(replay_metrics["static"]["positive_rate"]),
                "recall": float(replay_metrics["motion"]["positive_rate"]),
            }
        )

    idle_rows: List[Dict[str, Any]] = []
    for empty in empties:
        rows = np.asarray(
            cache[str(empty["path"])]["rows"][:, cols], dtype=np.float64
        )
        rows = transform_fusion_rows(rows, fusion)
        metrics = replay_one_stream(
            rows,
            coefficients,
            base_threshold,
            idle_q95,
            startup_strength=startup_strength,
            startup_sample_limit=startup_sample_limit,
            settle_margin_logits=settle_margin_logits,
            calibration=policy,
            references=references,
        )
        idle_rows.append(
            {
                "session": empty["session"],
                "role": empty["role"],
                "chip": empty["chip"],
                "fp_rate": float(metrics["positive_rate"]),
                "eval_count": int(metrics["eval_count"]),
            }
        )

    def summarize(role_names: Sequence[str]) -> Dict[str, Any]:
        role_set = set(role_names)
        paired = aggregate_paired(
            [row for row in paired_rows if row["role"] in role_set]
        )
        idle = aggregate_idle(
            [row for row in idle_rows if row["role"] in role_set]
        )
        return {"paired": paired, "idle": idle}

    discovery = summarize(DISCOVERY_ROLES)
    holdout = summarize((HOLDOUT_ROLE,))
    exclude = summarize((EXCLUDE_ROLE,))
    return {
        "score": replay_score(discovery["paired"], discovery["idle"]),
        "discovery": discovery,
        "holdout": holdout,
        "exclude": exclude,
        "replay_rows": {
            "paired": paired_rows,
            "idle": idle_rows,
        },
    }


def print_summary(result: Mapping[str, Any]) -> None:
    discovery_pairs = result["discovery"]["paired"]
    discovery_idle = result["discovery"]["idle"]
    holdout_pairs = result["holdout"]["paired"]
    holdout_idle = result["holdout"]["idle"]
    exclude_pairs = result["exclude"]["paired"]
    exclude_idle = result["exclude"]["idle"]
    bucket = "runtime-ready" if result["runtime_ready"] else "host-only"
    calibration = result.get("calibration", {})
    fusion = str(result.get("fusion", FUSION_LINEAR))
    mode = calibration.get("mode", CALIBRATION_Q95_SHIFT)
    calibration_detail = ""
    if mode == CALIBRATION_ROBUST_LOGIT:
        calibration_detail = (
            f" floor={calibration['robust_scale_floor_ratio']:.3f}"
        )
    elif mode == CALIBRATION_FEATURE_SHIFT:
        calibration_detail = (
            f" feature_q={calibration['feature_startup_quantile']:.3f}"
        )
    elif mode == CALIBRATION_GUARDED_UPWARD:
        calibration_detail = (
            f" up_blocks={calibration['upward_blocks']}"
            f" up_q={calibration['upward_quantile']:.3f}"
            f" up_guard={calibration['upward_max_positive_fraction']:.3f}"
        )
    print(
        f"#{result['rank']}  {' + '.join(result['combination'])}  "
        f"[{bucket}; fusion={fusion}]  score={result['score']:.3f}  "
        f"calibration={mode}{calibration_detail}  "
        f"startup={result['startup_strength']:.3f}  "
        f"settle_margin={result['settle_margin_logits']:.3f}"
    )
    print(
        "  "
        f"train OOF recall={result['train_operating_point']['recall']:.2f}%  "
        f"fp={result['train_operating_point']['fp_rate']:.2f}%  "
        f"base={result['base_threshold']:.4f}"
    )
    print(
        "  "
        f"discovery recall weighted={discovery_pairs['weighted_recall']:.2f}%  "
        f"worst={discovery_pairs['worst_recall']:.2f}%  "
        f"low_rssi_worst={discovery_pairs['worst_low_rssi_recall']:.2f}%"
    )
    print(
        "  "
        f"discovery fp weighted={discovery_pairs['weighted_fp_rate']:.2f}%  "
        f"paired max={discovery_pairs['max_fp_rate']:.2f}%  "
        f"idle max={discovery_idle['max_fp_rate']:.2f}%"
    )
    if holdout_pairs["count"] or holdout_idle["count"]:
        print(
            "  "
            f"holdout recall weighted={holdout_pairs['weighted_recall']:.2f}%  "
            f"worst={holdout_pairs['worst_recall']:.2f}%  "
            f"fp weighted={holdout_pairs['weighted_fp_rate']:.2f}%  "
            f"paired max={holdout_pairs['max_fp_rate']:.2f}%  "
            f"idle max={holdout_idle['max_fp_rate']:.2f}%"
        )
    if exclude_pairs["count"] or exclude_idle["count"]:
        print(
            "  "
            f"exclude recall weighted={exclude_pairs['weighted_recall']:.2f}%  "
            f"fp weighted={exclude_pairs['weighted_fp_rate']:.2f}%  "
            f"idle max={exclude_idle['max_fp_rate']:.2f}%"
        )
    for scenario, stress in result.get("stress", {}).items():
        stress_pairs = stress["discovery"]["paired"]
        stress_idle = stress["discovery"]["idle"]
        stress_holdout_pairs = stress["holdout"]["paired"]
        stress_holdout_idle = stress["holdout"]["idle"]
        stress_exclude_pairs = stress["exclude"]["paired"]
        stress_exclude_idle = stress["exclude"]["idle"]
        print(
            "  "
            f"stress {scenario}: score={stress['score']:.2f}  "
            f"recall={stress_pairs['weighted_recall']:.2f}%  "
            f"worst={stress_pairs['worst_recall']:.2f}%  "
            f"fp={stress_pairs['weighted_fp_rate']:.2f}%  "
            f"idle max={stress_idle['max_fp_rate']:.2f}%  "
            f"holdout worst/idle={stress_holdout_pairs['worst_recall']:.2f}%/"
            f"{stress_holdout_idle['max_fp_rate']:.2f}%  "
            f"exclude recall/fp/idle={stress_exclude_pairs['weighted_recall']:.2f}%/"
            f"{stress_exclude_pairs['weighted_fp_rate']:.2f}%/"
            f"{stress_exclude_idle['max_fp_rate']:.2f}%"
        )
    for data_dir, external in result.get("external_holdouts", {}).items():
        holdout_pairs = external["holdout"]["paired"]
        holdout_idle = external["holdout"]["idle"]
        exclude_pairs = external["exclude"]["paired"]
        exclude_idle = external["exclude"]["idle"]
        print(
            "  "
            f"external {Path(data_dir).name}: "
            f"holdout recall weighted/worst={holdout_pairs['weighted_recall']:.2f}%/"
            f"{holdout_pairs['worst_recall']:.2f}%  "
            f"paired/empty max fp={holdout_pairs['max_fp_rate']:.2f}%/"
            f"{holdout_idle['max_fp_rate']:.2f}%  "
            f"exclude recall/fp/empty={exclude_pairs['weighted_recall']:.2f}%/"
            f"{exclude_pairs['weighted_fp_rate']:.2f}%/"
            f"{exclude_idle['max_fp_rate']:.2f}%"
        )


def iter_calibration_policies(
    args: argparse.Namespace,
    startup_strengths: Sequence[float],
) -> List[Dict[str, Any]]:
    """Expand only parameters that belong to each requested family."""
    modes = list(args.calibration) if args.calibration else [CALIBRATION_Q95_SHIFT]
    if CALIBRATION_Q95_SHIFT not in modes:
        modes.insert(0, CALIBRATION_Q95_SHIFT)
    robust_floors = args.robust_scale_floor_ratio or [0.25]
    feature_quantiles = args.feature_startup_quantile or [0.5]
    upward_blocks = args.upward_blocks or [3]
    upward_quantiles = args.upward_quantile or [0.95]
    upward_guards = args.upward_max_positive_fraction or [0.5]
    policies: List[Dict[str, Any]] = []
    for mode in dict.fromkeys(modes):
        if mode == CALIBRATION_ROBUST_LOGIT:
            for strength, floor in product(startup_strengths, robust_floors):
                policies.append(
                    calibration_policy(
                        mode,
                        startup_strength=strength,
                        robust_scale_floor_ratio=floor,
                    )
                )
        elif mode == CALIBRATION_FEATURE_SHIFT:
            for strength, quantile in product(
                startup_strengths,
                feature_quantiles,
            ):
                policies.append(
                    calibration_policy(
                        mode,
                        startup_strength=strength,
                        feature_startup_quantile=quantile,
                    )
                )
        elif mode == CALIBRATION_GUARDED_UPWARD:
            for strength, blocks, quantile, guard in product(
                startup_strengths,
                upward_blocks,
                upward_quantiles,
                upward_guards,
            ):
                policies.append(
                    calibration_policy(
                        mode,
                        startup_strength=strength,
                        upward_blocks=blocks,
                        upward_quantile=quantile,
                        upward_max_positive_fraction=guard,
                    )
                )
        else:
            for strength in startup_strengths:
                policies.append(
                    calibration_policy(mode, startup_strength=strength)
                )
    return policies


def main() -> int:
    args = parse_args()
    external_data_dirs = {path.resolve() for path in args.external_data_dir}
    diagnostic_all_phy_dirs = {
        path.resolve() for path in args.external_diagnostic_all_phy
    }
    unknown_diagnostic_dirs = diagnostic_all_phy_dirs.difference(
        external_data_dirs
    )
    if unknown_diagnostic_dirs:
        raise ReplayError(
            "--external-diagnostic-all-phy requires a matching "
            "--external-data-dir: "
            + ", ".join(
                str(path)
                for path in sorted(unknown_diagnostic_dirs, key=str)
            )
        )
    if args.top_k < 1:
        raise ReplayError("--top-k must be at least 1")
    startup_strengths = (
        args.startup_strength
        if args.startup_strength
        else [LightweightDetector.STARTUP_STRENGTH]
    )
    settle_margins = (
        args.settle_margin_logits
        if args.settle_margin_logits
        else [LightweightDetector.SETTLE_MARGIN_LOGITS]
    )
    if any(value < 0.0 or value > 1.0 for value in startup_strengths):
        raise ReplayError("--startup-strength must be between 0 and 1")
    if any(value < 0.0 for value in settle_margins):
        raise ReplayError("--settle-margin-logits must be non-negative")
    robust_floors = args.robust_scale_floor_ratio or [0.25]
    feature_quantiles = args.feature_startup_quantile or [0.5]
    upward_blocks = args.upward_blocks or [3]
    upward_quantiles = args.upward_quantile or [0.95]
    upward_guards = args.upward_max_positive_fraction or [0.5]
    if any(value <= 0.0 for value in robust_floors):
        raise ReplayError("--robust-scale-floor-ratio must be positive")
    if any(value < 0.0 or value > 1.0 for value in feature_quantiles):
        raise ReplayError("--feature-startup-quantile must be between 0 and 1")
    if any(value < 1 for value in upward_blocks):
        raise ReplayError("--upward-blocks must be at least 1")
    if any(value < 0.0 or value > 1.0 for value in upward_quantiles):
        raise ReplayError("--upward-quantile must be between 0 and 1")
    if any(value < 0.0 or value > 1.0 for value in upward_guards):
        raise ReplayError(
            "--upward-max-positive-fraction must be between 0 and 1"
        )
    policies = iter_calibration_policies(args, startup_strengths)
    fusion_modes = list(dict.fromkeys(args.fusion or [FUSION_LINEAR]))
    candidates = parse_feature_sets(args.features)
    candidate_set = {tuple(candidate) for candidate in candidates}
    if not args.no_baseline:
        candidate_set.add(CURRENT_CLASSIC_COMBINATION)
    available = set(feature_cache.selectable_features())
    unknown = sorted(
        {name for candidate in candidate_set for name in candidate}.difference(available)
    )
    if unknown:
        raise ReplayError("Unknown feature(s) requested: " + ", ".join(unknown))
    if any(
        fusion != FUSION_LINEAR and len(candidate) != 2
        for fusion, candidate in product(fusion_modes, candidate_set)
    ):
        raise ReplayError("Non-linear fusion modes require exactly two features")

    feature_surface = sorted(
        {name for candidate in candidate_set for name in candidate}
    )
    pairs = iter_replay_pairs()
    empties = iter_empty_replays()
    paths = [pair["static_path"] for pair in pairs]
    paths.extend(pair["motion_path"] for pair in pairs)
    paths.extend(entry["path"] for entry in empties)
    runtime_candidates = [
        candidate
        for candidate in candidate_set
        if all(name in RUNTIME_READY_FEATURES for name in candidate)
    ]
    runtime_surface = sorted(
        {name for candidate in runtime_candidates for name in candidate}
    )
    host_candidates = [
        candidate
        for candidate in candidate_set
        if candidate not in runtime_candidates
    ]
    host_surface = sorted(
        {name for candidate in host_candidates for name in candidate}
    )
    runtime_cache: Dict[str, Dict[str, np.ndarray]] = {}
    status_file = sys.stderr if args.json else sys.stdout
    if runtime_surface:
        print(
            f"Extracting runtime replay rows for {len(paths)} files and "
            f"{len(runtime_surface)} features",
            file=status_file,
            flush=True,
        )
        runtime_cache = build_replay_cache(
            paths,
            runtime_surface,
            quiet=args.quiet,
        )
    runtime_index = {
        name: index for index, name in enumerate(runtime_surface)
    }
    host_cache: Dict[str, Dict[str, np.ndarray]] = {}
    if host_surface:
        print(
            f"Extracting shared host-only replay rows for {len(host_surface)} features",
            file=status_file,
            flush=True,
        )
        host_cache = build_replay_cache(
            paths,
            host_surface,
            quiet=args.quiet,
        )
    host_index = {name: index for index, name in enumerate(host_surface)}
    results = []
    for candidate in sorted(candidate_set):
        if candidate in runtime_candidates:
            candidate_cache = runtime_cache
            candidate_index = runtime_index
        else:
            candidate_cache = host_cache
            candidate_index = host_index
        for fusion, policy, settle_margin_logits in product(
            fusion_modes,
            policies,
            settle_margins,
        ):
            results.append(
                evaluate_candidate(
                    candidate,
                    fusion,
                    pairs,
                    empties,
                    candidate_cache,
                    candidate_index,
                    args,
                    startup_strength=float(policy["startup_strength"]),
                    settle_margin_logits=settle_margin_logits,
                    calibration=policy,
                )
            )
    stress_scenarios: Dict[str, Mapping[str, Any]] = {}
    if args.stress_augment or args.stress_scenario:
        scenario_names = (
            list(dict.fromkeys(args.stress_scenario))
            if args.stress_scenario else list(STRESS_SCENARIOS)
        )
        for scenario_name in scenario_names:
            components = STRESS_SCENARIOS[scenario_name]
            scenario = "+".join(components)
            _active, _feature_config, packet_config = (
                augmentation.resolve_training_augmentation(components)
            )
            stress_scenarios[scenario] = packet_config

        for scenario, packet_config in stress_scenarios.items():
            print(
                f"Extracting {scenario} packet-stress rows for "
                f"{len(paths)} files",
                file=status_file,
                flush=True,
            )
            stress_runtime_cache: Dict[str, Dict[str, np.ndarray]] = {}
            if runtime_surface:
                stress_runtime_cache = build_replay_cache(
                    paths,
                    runtime_surface,
                    quiet=args.quiet,
                    packet_augmentation=packet_config,
                    augmentation_seed=augmentation.training_packet_augmentation_seed(
                        packet_config
                    ),
                )
            stress_host_cache: Dict[str, Dict[str, np.ndarray]] = {}
            if host_surface:
                stress_host_cache = build_replay_cache(
                    paths,
                    host_surface,
                    quiet=args.quiet,
                    packet_augmentation=packet_config,
                    augmentation_seed=(
                        augmentation.training_packet_augmentation_seed(
                            packet_config
                        )
                    ),
                )
            for row in results:
                candidate = tuple(row["combination"])
                if candidate in runtime_candidates:
                    candidate_cache = stress_runtime_cache
                    candidate_index = runtime_index
                else:
                    candidate_cache = stress_host_cache
                    candidate_index = host_index
                row.setdefault("stress", {})[scenario] = evaluate_fixed_candidate(
                    row,
                    pairs,
                    empties,
                    candidate_cache,
                    candidate_index,
                )
        for row in results:
            row["robust_score"] = max(
                [float(row["score"])]
                + [float(stress["score"]) for stress in row["stress"].values()]
            )

    ranked = sorted(
        results,
        key=lambda row: (
            float(row.get("robust_score", row["score"])),
            float(row["score"]),
        ),
    )
    for rank, row in enumerate(ranked, start=1):
        row["rank"] = rank

    external_catalogs = []
    for raw_data_dir in args.external_data_dir:
        external_data_dir = raw_data_dir.resolve()
        diagnostic_all_phy = external_data_dir in diagnostic_all_phy_dirs
        dataset_info_path = external_data_dir / "dataset_info.json"
        if not dataset_info_path.is_file():
            raise ReplayError(
                f"External data directory has no dataset_info.json: "
                f"{external_data_dir}"
            )
        external_pairs = iter_replay_pairs(
            dataset_info_path=dataset_info_path,
            dataset_root=external_data_dir,
        )
        external_empties = iter_empty_replays(
            dataset_info_path=dataset_info_path,
            dataset_root=external_data_dir,
        )
        external_paths = [pair["static_path"] for pair in external_pairs]
        external_paths.extend(pair["motion_path"] for pair in external_pairs)
        external_paths.extend(entry["path"] for entry in external_empties)
        print(
            f"Extracting sealed external holdout {external_data_dir.name}: "
            f"{len(external_pairs)} pair(s), {len(external_empties)} empty stream(s)",
            file=status_file,
            flush=True,
        )
        external_runtime_cache: Dict[str, Dict[str, np.ndarray]] = {}
        if runtime_surface:
            external_runtime_cache = build_replay_cache(
                external_paths,
                runtime_surface,
                quiet=args.quiet,
                keep_all_phy=diagnostic_all_phy,
            )
        external_host_cache: Dict[str, Dict[str, np.ndarray]] = {}
        if host_surface:
            external_host_cache = build_replay_cache(
                external_paths,
                host_surface,
                quiet=args.quiet,
                keep_all_phy=diagnostic_all_phy,
            )
        for row in ranked:
            candidate = tuple(row["combination"])
            if candidate in runtime_candidates:
                candidate_cache = external_runtime_cache
                candidate_index = runtime_index
            else:
                candidate_cache = external_host_cache
                candidate_index = host_index
            row.setdefault("external_holdouts", {})[
                str(external_data_dir)
            ] = evaluate_fixed_candidate(
                row,
                external_pairs,
                external_empties,
                candidate_cache,
                candidate_index,
            )
        external_catalogs.append(
            {
                "data_dir": str(external_data_dir),
                "pair_count": len(external_pairs),
                "empty_count": len(external_empties),
                "diagnostic_all_phy": diagnostic_all_phy,
            }
        )

    baseline_rows = []
    for row in ranked:
        if tuple(row["combination"]) == CURRENT_CLASSIC_COMBINATION:
            baseline_rows.append(row)
    payload = {
        "feature_surface": feature_surface,
        "runtime_ready_feature_surface": list(RUNTIME_READY_FEATURES),
        "host_only_feature_surface": list(HOST_ONLY_FEATURES),
        "baseline_combination": list(CURRENT_CLASSIC_COMBINATION),
        "baseline": baseline_rows,
        "discovery_roles": list(DISCOVERY_ROLES),
        "holdout_role": HOLDOUT_ROLE,
        "external_holdouts": external_catalogs,
        "train_empty_hard_negatives": bool(args.include_train_empty),
        "stress_packet_augmentations": {
            name: dict(config) for name, config in stress_scenarios.items()
        },
        "candidates": ranked,
    }
    if args.json:
        print(json.dumps(payload, indent=2))
        return 0

    if baseline_rows:
        print("\nRefitted feature-pair surrogate (not the exported runtime baseline):")
        for row in baseline_rows:
            print_summary(row)
        print()
    ranking_basis = (
        "worst clean/packet-stress discovery score"
        if args.stress_augment or args.stress_scenario
        else "clean discovery score"
    )
    print(f"Candidate replay ranking ({ranking_basis}):")
    for row in ranked[: args.top_k]:
        print_summary(row)
        print()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ReplayError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
