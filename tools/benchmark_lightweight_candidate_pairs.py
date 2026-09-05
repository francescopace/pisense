#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Candidate Lightweight Combination Benchmark

Compare hand-picked Lightweight detector candidates from the current ML feature
surface without coupling the ranking to a threshold sweep. The projection fits
on `train`, the primary ranking uses `train` plus `selection`, and `holdout` and
`exclude` are reported separately without influencing candidate order.

Usage:
    python tools/benchmark_lightweight_candidate_pairs.py
    python tools/benchmark_lightweight_candidate_pairs.py --json
    python tools/benchmark_lightweight_candidate_pairs.py \
        --feature turb_autocorr
    python tools/benchmark_lightweight_candidate_pairs.py \
        --pair l1_delta_lag_ratio,chan_coh_gap \
        --pair l1_delta_lag_ratio,chan_shape_spread_subband
    python tools/benchmark_lightweight_candidate_pairs.py \
        --triple turb_autocorr,chan_freq_coh_curve_std,chan_coh_gap
    python tools/benchmark_lightweight_candidate_pairs.py --all-runtime-triplets
    python tools/benchmark_lightweight_candidate_pairs.py --all-host-triplets

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

import argparse
from importlib import import_module
import json
import math
import os
import sys
from itertools import combinations
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.lib.bootstrap import setup_paths  # noqa: E402

setup_paths()

from tools.lib.ml_training import (
    dataset,
    feature_cache,
)
from tools import replay_lightweight_candidates as candidate_replay  # noqa: E402
from tools.lib.ml_weights import FEATURE_NAMES  # noqa: E402
from tools.lib.dataset_metadata import load_dataset_info  # noqa: E402

FIT_ROLES = ("train",)
PRIMARY_ROLES = ("train", "selection")
HOLDOUT_ROLE = "holdout"
EXCLUDE_ROLE = "exclude"
DEFAULT_PAIRS = (
    ("l1_delta_lag_ratio", "chan_coh_gap"),
    ("l1_delta_lag_ratio", "chan_freq_coh_curve_std"),
    ("turb_autocorr", "chan_freq_coh_curve_std"),
)
DEFAULT_TRIPLETS = (
    ("turb_autocorr", "chan_freq_coh_curve_std", "chan_coh_gap"),
    ("turb_autocorr", "chan_freq_coh_curve_std", "l1_delta_lag_ratio"),
    ("turb_mad_over_mean", "turb_autocorr", "chan_freq_coh_curve_std"),
)
CURRENT_CLASSIC_COMBINATION = ("turb_autocorr", "turb_iqr_over_mean_aggr")
RUNTIME_READY_FEATURES = tuple(FEATURE_NAMES)
HOST_ONLY_FEATURES = tuple(
    name for name in feature_cache.selectable_features() if name not in FEATURE_NAMES
)


class BenchmarkError(RuntimeError):
    """Raised when the requested benchmark cannot run."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--feature",
        action="append",
        default=[],
        help="single candidate feature; repeat to add more features",
    )
    parser.add_argument(
        "--pair",
        action="append",
        default=[],
        help="candidate pair as feature_a,feature_b; repeat to add more pairs",
    )
    parser.add_argument(
        "--triple",
        action="append",
        default=[],
        help="candidate triplet as feature_a,feature_b,feature_c; repeat to add more triplets",
    )
    parser.add_argument(
        "--all-runtime-triplets",
        action="store_true",
        help="benchmark every runtime-ready triplet from the production feature surface",
    )
    parser.add_argument(
        "--all-host-triplets",
        action="store_true",
        help="benchmark every triplet that includes at least one host-only candidate feature",
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=20,
        help="limit printed rankings to the top K candidates (default: 20)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="emit the full benchmark payload as JSON",
    )
    parser.add_argument(
        "--no-baseline",
        action="store_true",
        help="skip the current Lightweight pair baseline section",
    )
    return parser.parse_args()


def parse_combination_specs(
    raw_specs: Sequence[str],
    expected_size: int,
    label: str,
) -> List[Tuple[str, ...]]:
    parsed: List[Tuple[str, ...]] = []
    for raw in raw_specs:
        names = [part.strip() for part in raw.split(",") if part.strip()]
        if len(names) != expected_size:
            raise BenchmarkError(
                f"Invalid --{label} {raw!r}; expected {expected_size} distinct features"
            )
        if len(set(names)) != expected_size:
            raise BenchmarkError(f"Invalid --{label} {raw!r}; features must differ")
        parsed.append(tuple(names))
    return parsed


def runtime_triplets() -> List[Tuple[str, str, str]]:
    return list(combinations(RUNTIME_READY_FEATURES, 3))


def host_triplets() -> List[Tuple[str, str, str]]:
    searchable = tuple(feature_cache.selectable_features())
    host_feature_set = set(HOST_ONLY_FEATURES)
    return [
        triplet
        for triplet in combinations(searchable, 3)
        if any(name in host_feature_set for name in triplet)
    ]


def resolve_candidate_combinations(args: argparse.Namespace) -> List[Tuple[str, ...]]:
    feature_specs = parse_combination_specs(args.feature, 1, "feature")
    pair_specs = parse_combination_specs(args.pair, 2, "pair")
    triple_specs = parse_combination_specs(args.triple, 3, "triple")
    requested_flags = int(bool(args.all_runtime_triplets)) + int(bool(args.all_host_triplets))
    if requested_flags > 1:
        raise BenchmarkError(
            "Choose at most one generated search space: --all-runtime-triplets or "
            "--all-host-triplets"
        )
    if requested_flags and (feature_specs or pair_specs or triple_specs):
        raise BenchmarkError(
            "Explicit --feature/--pair/--triple specs cannot be mixed with "
            "generated triplet searches"
        )
    if args.all_runtime_triplets:
        return runtime_triplets()
    if args.all_host_triplets:
        return host_triplets()
    if triple_specs:
        return triple_specs
    if pair_specs:
        return pair_specs
    if feature_specs:
        return feature_specs
    return list(DEFAULT_TRIPLETS)


def dataset_entry_by_filename() -> Dict[str, Dict[str, Any]]:
    files = load_dataset_info()["files"]
    lookup: Dict[str, Dict[str, Any]] = {}
    for label_name, entries in files.items():
        for entry in entries:
            enriched = dict(entry)
            enriched["label_name"] = label_name
            lookup[str(entry["filename"])] = enriched
    return lookup


def load_feature_matrix(
    requested_combinations: Sequence[Tuple[str, ...]],
) -> Dict[str, Any]:
    """Build one production-tick matrix for runtime and host-only candidates."""
    requested_names = sorted(
        {name for combination in requested_combinations for name in combination}
    )
    records, stats = dataset._load_training_file_records(
        dataset_roles=("train", "selection", "holdout", "exclude"),
    )
    replay_cache = candidate_replay.build_replay_cache(
        (record["path"] for record in records),
        requested_names,
        quiet=True,
    )
    X_parts: List[np.ndarray] = []
    y_parts: List[np.ndarray] = []
    context = {
        "source_file": [],
        "label_name": [],
        "pair_id": [],
        "synthetic": [],
    }
    for record in records:
        rows = np.asarray(
            replay_cache[str(record["path"])]["rows"],
            dtype=np.float32,
        )
        if len(rows) == 0:
            continue
        count = len(rows)
        X_parts.append(rows)
        y_parts.append(
            np.full(count, 1 if record["is_motion"] else 0, dtype=np.int8)
        )
        context["source_file"].append(
            np.full(count, record["path"].name, dtype=object)
        )
        context["label_name"].append(
            np.full(count, record["label_name"], dtype=object)
        )
        context["pair_id"].append(
            np.full(count, record["pair_id"], dtype=object)
        )
        context["synthetic"].append(
            np.full(count, bool(record["synthetic"]), dtype=bool)
        )
    if not X_parts:
        raise BenchmarkError("Time-aware replay produced no candidate rows")
    return {
        "X": np.vstack(X_parts),
        "y": np.concatenate(y_parts),
        "feature_names": requested_names,
        "sample_context": {
            key: np.concatenate(parts)
            for key, parts in context.items()
        },
        "stats": stats,
        "training_sample_contract": "time_aware_replay_tick",
    }


def build_sample_metadata(
    sample_context: Mapping[str, np.ndarray],
    entry_lookup: Mapping[str, Mapping[str, Any]],
) -> Dict[str, np.ndarray]:
    source_files = np.asarray(sample_context["source_file"], dtype=object)
    basenames = np.asarray(
        [os.path.basename(str(path)) for path in source_files],
        dtype=object,
    )
    roles = np.asarray(
        [str(entry_lookup[str(name)]["dataset_role"]) for name in basenames],
        dtype=object,
    )
    low_rssi = np.asarray(
        [bool(entry_lookup[str(name)].get("low_rssi", False)) for name in basenames],
        dtype=bool,
    )
    labels = np.asarray(sample_context["label_name"], dtype=object)
    pair_ids = np.asarray(sample_context["pair_id"], dtype=object)
    synthetic = np.asarray(sample_context["synthetic"], dtype=bool)
    return {
        "source_file": source_files,
        "source_basename": basenames,
        "dataset_role": roles,
        "low_rssi": low_rssi,
        "label_name": labels,
        "pair_id": pair_ids,
        "synthetic": synthetic,
    }


def safe_auc(y_true: np.ndarray, scores: np.ndarray) -> float:
    if len(y_true) == 0 or len(np.unique(y_true)) < 2:
        return float("nan")
    try:
        roc_auc_score = import_module("sklearn.metrics").roc_auc_score
    except ImportError as exc:
        raise BenchmarkError(
            "scikit-learn is required to run this benchmark; "
            "install requirements-ml.txt"
        ) from exc
    return float(roc_auc_score(y_true, scores))


def fit_lda_projection(x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    idle = x[y == 0]
    motion = x[y == 1]
    if len(idle) == 0 or len(motion) == 0:
        raise BenchmarkError("Need both idle and motion windows to fit a candidate")
    mean_idle = idle.mean(axis=0)
    mean_motion = motion.mean(axis=0)
    cov_idle = np.atleast_2d(np.cov(idle, rowvar=False))
    cov_motion = np.atleast_2d(np.cov(motion, rowvar=False))
    pooled = (
        ((len(idle) - 1) * cov_idle) + ((len(motion) - 1) * cov_motion)
    ) / max(len(idle) + len(motion) - 2, 1)
    pooled = np.asarray(pooled, dtype=np.float64) + np.eye(x.shape[1], dtype=np.float64) * 1e-9
    weights = np.linalg.solve(pooled, mean_motion - mean_idle)
    return weights, pooled


def correlation_summary(values: np.ndarray) -> Dict[str, Any]:
    width = values.shape[1]
    if len(values) < 2 or width < 2:
        return {
            "corr_abs": float("nan"),
            "corr_abs_mean": float("nan"),
            "corr_pairs": {},
        }
    strongest = 0.0
    absolute_values: List[float] = []
    pairs: Dict[str, float] = {}
    for left, right in combinations(range(width), 2):
        x_values = values[:, left]
        y_values = values[:, right]
        if np.std(x_values) < 1e-12 or np.std(y_values) < 1e-12:
            corr = 0.0
        else:
            corr = abs(float(np.corrcoef(x_values, y_values)[0, 1]))
        strongest = max(strongest, corr)
        absolute_values.append(corr)
        pairs[f"{left}-{right}"] = corr
    return {
        "corr_abs": strongest,
        "corr_abs_mean": float(sum(absolute_values) / len(absolute_values)),
        "corr_pairs": pairs,
    }


def pair_id_auc_rows(
    mask: np.ndarray,
    y: np.ndarray,
    scores: np.ndarray,
    pair_ids: np.ndarray,
    motion_mask: np.ndarray,
    static_mask: np.ndarray,
    low_rssi_mask: np.ndarray,
) -> Dict[str, Any]:
    normal_aucs: List[float] = []
    weak_aucs: List[float] = []
    flips = 0
    pair_count = 0
    for pair_id in sorted({str(value) for value in pair_ids[mask] if str(value)}):
        pair_mask = mask & (pair_ids == pair_id)
        if pair_mask.sum() == 0 or len(np.unique(y[pair_mask])) < 2:
            continue
        pair_count += 1
        auc = safe_auc(y[pair_mask], scores[pair_mask])
        idle_median = float(np.median(scores[pair_mask & static_mask]))
        motion_median = float(np.median(scores[pair_mask & motion_mask]))
        if motion_median <= idle_median:
            flips += 1
        if np.any(pair_mask & low_rssi_mask):
            weak_aucs.append(auc)
        else:
            normal_aucs.append(auc)
    combined = normal_aucs + weak_aucs
    return {
        "pair_count": pair_count,
        "flip_count": flips,
        "normal_pair_min_auc": min(normal_aucs) if normal_aucs else float("nan"),
        "weak_pair_min_auc": min(weak_aucs) if weak_aucs else float("nan"),
        "all_pair_min_auc": min(combined) if combined else float("nan"),
    }


def evaluate_scope(
    scope_mask: np.ndarray,
    idle_role_mask: np.ndarray,
    y: np.ndarray,
    scores: np.ndarray,
    pair_ids: np.ndarray,
    motion_mask: np.ndarray,
    static_mask: np.ndarray,
    empty_mask: np.ndarray,
    low_rssi_mask: np.ndarray,
    feature_values: np.ndarray,
) -> Dict[str, Any]:
    normal_mask = scope_mask & ~low_rssi_mask
    weak_mask = scope_mask & low_rssi_mask
    idle_mask = idle_role_mask & (static_mask | empty_mask) & ~low_rssi_mask
    empty_idle_mask = idle_role_mask & empty_mask & ~low_rssi_mask
    static_idle_mask = idle_role_mask & static_mask & ~low_rssi_mask
    corr = correlation_summary(feature_values[scope_mask])
    summary = {
        "window_count": int(scope_mask.sum()),
        "overall_auc": safe_auc(y[scope_mask], scores[scope_mask]),
        "normal_auc": safe_auc(y[normal_mask], scores[normal_mask]),
        "weak_auc": safe_auc(y[weak_mask], scores[weak_mask]),
        "corr_abs": corr["corr_abs"],
        "corr_abs_mean": corr["corr_abs_mean"],
        "corr_pairs": corr["corr_pairs"],
    }
    summary.update(
        pair_id_auc_rows(
            scope_mask,
            y,
            scores,
            pair_ids,
            motion_mask,
            static_mask,
            low_rssi_mask,
        )
    )
    if idle_mask.sum() > 1:
        idle_scores = scores[idle_mask]
        empty_scores = scores[empty_idle_mask]
        static_scores = scores[static_idle_mask]
        spread = float(np.std(idle_scores))
        shift = (
            abs(float(np.median(empty_scores) - np.median(static_scores)))
            if len(empty_scores) and len(static_scores)
            else float("nan")
        )
        summary["idle_spread"] = spread
        summary["idle_shift_sigma"] = (
            shift / (spread + 1e-9) if not math.isnan(shift) else float("nan")
        )
    else:
        summary["idle_spread"] = float("nan")
        summary["idle_shift_sigma"] = float("nan")
    return summary


def primary_ranking_key(result: Mapping[str, Any]) -> Tuple[float, ...]:
    primary = result["primary"]
    return (
        -float(primary["flip_count"]),
        float(primary["all_pair_min_auc"]),
        float(primary["weak_pair_min_auc"])
        if not math.isnan(primary["weak_pair_min_auc"]) else -1.0,
        float(primary["weak_auc"]) if not math.isnan(primary["weak_auc"]) else -1.0,
        float(primary["normal_pair_min_auc"]),
        float(primary["normal_auc"]),
        -float(primary["idle_shift_sigma"])
        if not math.isnan(primary["idle_shift_sigma"]) else -999.0,
    )


def evaluate_pair(
    combination: Tuple[str, ...],
    matrix: Mapping[str, Any],
    sample_meta: Mapping[str, np.ndarray],
) -> Dict[str, Any]:
    feature_names = list(matrix["feature_names"])
    indices = [feature_names.index(name) for name in combination]
    combination_values = np.asarray(matrix["X"][:, indices], dtype=np.float64)
    y = np.asarray(matrix["y"], dtype=np.int8)

    labels = np.asarray(sample_meta["label_name"], dtype=object)
    roles = np.asarray(sample_meta["dataset_role"], dtype=object)
    low_rssi = np.asarray(sample_meta["low_rssi"], dtype=bool)
    pair_ids = np.asarray(sample_meta["pair_id"], dtype=object)
    synthetic = np.asarray(sample_meta["synthetic"], dtype=bool)

    motion = labels == "motion"
    static = labels == "static_presence"
    empty = labels == "empty"
    real = ~synthetic
    paired = real & (motion | static)

    primary_fit_mask = paired & np.isin(roles, FIT_ROLES)
    if primary_fit_mask.sum() == 0:
        raise BenchmarkError(
            f"No admitted paired windows available for {combination!r}"
        )

    weights, pooled = fit_lda_projection(
        combination_values[primary_fit_mask], y[primary_fit_mask]
    )
    scores = combination_values @ weights

    idle_primary = combination_values[primary_fit_mask & (y == 0)]
    motion_primary = combination_values[primary_fit_mask & (y == 1)]
    delta = motion_primary.mean(axis=0) - idle_primary.mean(axis=0)
    mahalanobis_d2 = float(delta.T @ np.linalg.solve(pooled, delta))

    primary_scope = paired & np.isin(roles, PRIMARY_ROLES)
    primary_idle_scope = real & np.isin(roles, PRIMARY_ROLES)
    holdout_scope = paired & (roles == HOLDOUT_ROLE)
    holdout_idle_scope = real & (roles == HOLDOUT_ROLE)
    exclude_scope = paired & (roles == EXCLUDE_ROLE)
    exclude_idle_scope = real & (roles == EXCLUDE_ROLE)
    runtime_ready = all(name in RUNTIME_READY_FEATURES for name in combination)
    return {
        "combination": list(combination),
        "combination_size": len(combination),
        "runtime_ready": runtime_ready,
        "weights": [float(value) for value in weights],
        "mahalanobis_d2": mahalanobis_d2,
        "primary": evaluate_scope(
            primary_scope,
            primary_idle_scope,
            y,
            scores,
            pair_ids,
            motion,
            static,
            empty,
            low_rssi,
            combination_values,
        ),
        "holdout": evaluate_scope(
            holdout_scope,
            holdout_idle_scope,
            y,
            scores,
            pair_ids,
            motion,
            static,
            empty,
            low_rssi,
            combination_values,
        ),
        "exclude": evaluate_scope(
            exclude_scope,
            exclude_idle_scope,
            y,
            scores,
            pair_ids,
            motion,
            static,
            empty,
            low_rssi,
            combination_values,
        ),
    }


def print_summary(title: str, summary: Mapping[str, Any]) -> None:
    print(title)
    print(
        "  "
        f"AUC all={summary['overall_auc']:.4f}  "
        f"normal={summary['normal_auc']:.4f}  "
        f"weak={summary['weak_auc']:.4f}"
    )
    print(
        "  "
        f"worst pair all={summary['all_pair_min_auc']:.4f}  "
        f"normal={summary['normal_pair_min_auc']:.4f}  "
        f"weak={summary['weak_pair_min_auc']:.4f}"
    )
    print(
        "  "
        f"pairs={summary['pair_count']}  flips={summary['flip_count']}  "
        f"idle_shift={summary['idle_shift_sigma']:.3f} sigma  "
        f"max|corr|={summary['corr_abs']:.3f}  "
        f"mean|corr|={summary['corr_abs_mean']:.3f}"
    )


def main() -> int:
    args = parse_args()
    if args.top_k < 1:
        raise BenchmarkError("--top-k must be at least 1")
    candidate_combinations = resolve_candidate_combinations(args)
    available = set(feature_cache.selectable_features())
    requested = set(
        name for combination in candidate_combinations for name in combination
    )
    if not args.no_baseline:
        requested.update(CURRENT_CLASSIC_COMBINATION)
    unknown = sorted(requested - available)
    if unknown:
        raise BenchmarkError(
            "Unknown feature(s) requested: " + ", ".join(unknown)
        )

    benchmark_combinations = list(candidate_combinations)
    if not args.no_baseline:
        benchmark_combinations.append(CURRENT_CLASSIC_COMBINATION)
    matrix = load_feature_matrix(benchmark_combinations)
    sample_meta = build_sample_metadata(
        matrix["sample_context"],
        dataset_entry_by_filename(),
    )

    baseline = None
    if not args.no_baseline:
        baseline = evaluate_pair(CURRENT_CLASSIC_COMBINATION, matrix, sample_meta)
    results = [
        evaluate_pair(combination, matrix, sample_meta)
        for combination in candidate_combinations
    ]
    ranked = sorted(results, key=primary_ranking_key, reverse=True)
    for rank, row in enumerate(ranked, start=1):
        row["rank"] = rank

    payload = {
        "feature_surface": list(feature_cache.selectable_features()),
        "runtime_ready_feature_surface": list(RUNTIME_READY_FEATURES),
        "host_only_feature_surface": list(HOST_ONLY_FEATURES),
        "training_sample_contract": str(matrix.get("training_sample_contract")),
        "primary_roles": list(PRIMARY_ROLES),
        "fit_roles": list(FIT_ROLES),
        "holdout_role": HOLDOUT_ROLE,
        "secondary_role": EXCLUDE_ROLE,
        "baseline_combination": list(CURRENT_CLASSIC_COMBINATION),
        "baseline": baseline,
        "candidates": ranked,
    }
    if args.json:
        print(json.dumps(payload, indent=2))
        return 0

    print("Threshold-free benchmark for candidate Lightweight combinations")
    print("Primary ranking: admitted real paired datasets (train/selection)")
    print("Sealed diagnostic: real paired holdout datasets only")
    print("Secondary diagnostic: real paired exclude datasets only")
    print(f"Training sample contract: {matrix.get('training_sample_contract')}")
    print()
    if baseline is not None:
        print("Baseline: current Lightweight combination")
        print(f"  {' + '.join(baseline['combination'])}")
        print_summary("  Primary:", baseline["primary"])
        print_summary("  Holdout:", baseline["holdout"])
        print_summary("  Exclude:", baseline["exclude"])
        print()

    for row in ranked[: args.top_k]:
        bucket = "runtime-ready" if row["runtime_ready"] else "host-only"
        print(f"#{row['rank']}  {' + '.join(row['combination'])}  [{bucket}]")
        print(
            "  "
            f"primary Mahalanobis d^2={row['mahalanobis_d2']:.3f}  "
            + "weights=("
            + ", ".join(f"{value:+.3f}" for value in row["weights"])
            + ")"
        )
        print_summary("  Primary:", row["primary"])
        print_summary("  Holdout:", row["holdout"])
        print_summary("  Exclude:", row["exclude"])
        print()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BenchmarkError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
