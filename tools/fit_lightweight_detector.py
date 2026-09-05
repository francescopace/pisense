#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Lightweight Detector Coefficient Fit

Refits the Lightweight detector's weighted `turb_autocorr +
turb_iqr_over_mean_aggr` fusion and
exports the constants consumed by both runtimes.

The fit follows the recipe recorded alongside the previous coefficients: a
grouped, de-overlapped out-of-fold fit balanced by class, chip, and session.

- grouped: folds split on session, so windows from one recording never appear on
  both sides of a split
- de-overlapped: one feature vector per full window, so consecutive samples share
  no packets and the fit is not fed a smoothed random walk
- balanced: sample weights equalize class, chip, and session mass, so the long
  recordings and the better represented chips do not dominate
- fp-weight: optional idle-class multiplier after balancing, analogous to High
  Accuracy's `fp_weight`. `--fp-target` remains a real false-positive ceiling
  on the operating-point sweep and is not this multiplier. Sequential empty-room
  replay, not this weight, remains the export gate. Lightweight may raise at
  most one effective alarm per short empty recording; two alarms on one file
  still block export.

Only `train` datasets are fitted. `holdout` stays sealed, `selection` is left for
operating-point work, and `exclude` is dropped.

Usage:
    python tools/fit_lightweight_detector.py
    python tools/fit_lightweight_detector.py --apply
    python tools/fit_lightweight_detector.py --centered-threshold-logit 1.73 --apply

The optional centered-logit override makes a sequential-replay operating point
reproducible. The OOF sweep still reports its diagnostic point, but dense-window
FP does not encode the production empty-room alarm budget of at most one
effective alarm per short empty recording.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tools.lib.atomic_io import atomic_write_set  # noqa: E402
from tools.lib.bootstrap import setup_paths  # noqa: E402

setup_paths()

import config  # noqa: E402
from tools.lib.lightweight_detector import LightweightDetector  # noqa: E402
from tools.lib.temporal_csi_sampler import minimum_valid_slots, temporal_window_slots  # noqa: E402
from tools.lib.csi_io import load_npz_as_packets  # noqa: E402
from tools.lib.dataset_metadata import (  # noqa: E402
    derive_detector_timing,
    load_dataset_info,
    measure_packet_interval_us,
    paired_dataset_role,
    resolve_entry_path,
)
from tools.lib.performance_report import (  # noqa: E402
    temporal_detector_ticks,
)
from tools.lib.temporal_replay import target_pps_for_packets  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
PYTHON_SOURCE = REPO_ROOT / "tools" / "lib" / "lightweight_detector.py"
CPP_SOURCE = REPO_ROOT / "src" / "cpp" / "core" / "lightweight_detector.h"

FITTED_ROLES = ("train",)
IDLE_LABEL = 0
MOTION_LABEL = 1
DEFAULT_FP_WEIGHT = 1.0


class FitError(RuntimeError):
    """Raised when the corpus or the fit cannot produce usable coefficients."""


def iter_training_pairs() -> List[Dict[str, Any]]:
    """Collect paired static-presence and motion recordings used for the fit."""
    files = load_dataset_info()["files"]
    motion_by_name = {entry["filename"]: entry for entry in files["motion"]}

    pairs: List[Dict[str, Any]] = []
    for static_entry in files["static_presence"]:
        motion_name = static_entry.get("optimal_pair_motion_file")
        motion_entry = motion_by_name.get(motion_name) if motion_name else None
        if (
            motion_entry is None
            or paired_dataset_role(
                static_entry,
                motion_entry,
                admitted_roles=FITTED_ROLES,
            ) is None
        ):
            continue
        pairs.append(
            {
                "session": static_entry["filename"],
                "chip": str(static_entry.get("chip", "unknown")),
                "static_path": resolve_entry_path("static_presence", static_entry),
                "motion_path": resolve_entry_path("motion", motion_entry),
            }
        )
    if not pairs:
        raise FitError("No train-role paired datasets found")
    return pairs


def extract_window_features(
    packets: Sequence[Dict[str, Any]],
    selected_band: Sequence[int],
) -> np.ndarray:
    """Replay one recording at the runtime evaluation cadence.

    Returns one feature row per evaluation, plus a mask marking the rows that are
    de-overlapped. The two consumers need different sampling: the coefficient fit
    wants de-overlapped rows so consecutive samples share no packets and it is
    not fed a smoothed random walk, while the operating-point sweep wants every
    evaluation, because that is what the runtime thresholds. Sweeping on the
    de-overlapped subset understates false positives, since a brief excursion
    above the threshold spans several consecutive evaluations but only one
    de-overlapped window.

    Features come from the production detector rather than a reimplementation, so
    the fit can never drift from what the runtime computes. That includes the
    timing contract: the detector is sized from this recording's measured
    cadence exactly as the runtime sizes it, because the lags define what the
    two features mean and fitting them under a different cadence would produce
    coefficients for a feature the runtime never computes.
    """
    interval_us = measure_packet_interval_us(packets)
    target_pps = target_pps_for_packets(packets, interval_us)
    timing = derive_detector_timing(max(1, int(round(1_000_000.0 / target_pps))))
    timing["window_packets"] = temporal_window_slots(
        target_pps, config.SEGMENTATION_WINDOW_SIZE_MS
    )
    detector = LightweightDetector(
        window_size=timing["window_packets"],
        autocorr_lag=timing["autocorr_lag"],
    )
    detector.set_minimum_valid_samples(
        minimum_valid_slots(timing["window_packets"])
    )
    window = timing["window_packets"]
    rows: List[Tuple[float, float]] = []
    deoverlapped: List[bool] = []
    since_window = 0
    for admission, should_evaluate, slots_since_reset in temporal_detector_ticks(
        detector, packets, interval_us
    ):
        packet = admission.packet
        if admission.reset_required:
            since_window = 0
        detector.process_packet(packet["csi_data"], selected_band)
        since_window = max(since_window + 1, slots_since_reset)
        if not should_evaluate or not detector.is_ready():
            continue
        metrics = detector.update_state()
        rows.append(
            (
                metrics["turb_autocorr"],
                metrics["turb_iqr_over_mean_aggr"],
            )
        )
        deoverlapped.append(since_window >= window)
        if since_window >= window:
            since_window = 0
    return (
        np.asarray(rows, dtype=np.float64).reshape(-1, 2),
        np.asarray(deoverlapped, dtype=bool),
    )


def build_corpus(
    pairs: Sequence[Dict[str, Any]],
    selected_band: Sequence[int],
    progress: bool = True,
) -> Dict[str, np.ndarray]:
    """Replay every training pair into one labelled, grouped feature matrix."""
    features: List[np.ndarray] = []
    labels: List[np.ndarray] = []
    sessions: List[np.ndarray] = []
    chips: List[np.ndarray] = []
    deoverlapped: List[np.ndarray] = []

    for index, pair in enumerate(pairs, start=1):
        if progress:
            print(f"  [{index}/{len(pairs)}] {pair['session'][:64]}", flush=True)
        for path, label in ((pair["static_path"], IDLE_LABEL), (pair["motion_path"], MOTION_LABEL)):
            rows, row_deoverlapped = extract_window_features(
                load_npz_as_packets(path), selected_band
            )
            if rows.size == 0:
                continue
            features.append(rows)
            labels.append(np.full(len(rows), label, dtype=np.int8))
            sessions.append(np.full(len(rows), pair["session"], dtype=object))
            chips.append(np.full(len(rows), pair["chip"], dtype=object))
            deoverlapped.append(row_deoverlapped)

    if not features:
        raise FitError("Replay produced no windows; check the corpus and window size")

    return {
        "x": np.vstack(features),
        "y": np.concatenate(labels),
        "session": np.concatenate(sessions),
        "chip": np.concatenate(chips),
        "deoverlapped": np.concatenate(deoverlapped),
    }


def balanced_sample_weights(
    y: np.ndarray,
    chip: np.ndarray,
    session: np.ndarray,
) -> np.ndarray:
    """Equalize class, chip, and session mass so no stratum dominates the fit."""
    weights = np.ones(len(y), dtype=np.float64)
    for key in (y, chip, session):
        values, counts = np.unique(key, return_counts=True)
        lookup = {value: count for value, count in zip(values, counts, strict=True)}
        weights *= np.asarray([1.0 / lookup[value] for value in key], dtype=np.float64)
    return weights * (len(y) / weights.sum())


def apply_idle_fp_weight(
    weights: np.ndarray,
    y: np.ndarray,
    fp_weight: float,
) -> np.ndarray:
    """Scale idle mass after balancing so false positives cost more in the fit.

    This is the Lightweight counterpart of High Accuracy's class-0 `fp_weight`.
    The operating-point sweep does not use it: `--fp-target` is a real
    false-positive ceiling, not a loss multiplier.
    """
    if fp_weight == 1.0:
        return weights
    if fp_weight <= 0.0:
        raise FitError(f"fp_weight must be positive, got {fp_weight}")
    adjusted = np.array(weights, dtype=np.float64, copy=True)
    adjusted[y == IDLE_LABEL] *= float(fp_weight)
    return adjusted


def fit_coefficients(
    x: np.ndarray,
    y: np.ndarray,
    weights: np.ndarray,
) -> Dict[str, Any]:
    """Standardize the features and fit the two-feature logistic fusion."""
    from sklearn.linear_model import LogisticRegression

    center = np.average(x, axis=0, weights=weights)
    scale = np.sqrt(np.average((x - center) ** 2, axis=0, weights=weights))
    if not np.all(scale > 0.0):
        raise FitError(f"Degenerate feature scale {scale}; the corpus has no variation")

    standardized = (x - center) / scale
    model = LogisticRegression(max_iter=1000)
    model.fit(standardized, y, sample_weight=weights)

    # Plain Python floats: NumPy scalars repr as "np.float64(...)", which would be
    # written verbatim into both runtimes by the exporter.
    return {
        "center": tuple(float(value) for value in center),
        "scale": tuple(float(value) for value in scale),
        "weight": tuple(float(value) for value in model.coef_[0]),
        "intercept": float(model.intercept_[0]),
    }


def logits(x: np.ndarray, coefficients: Dict[str, Any]) -> np.ndarray:
    """Evaluate the fused logit exactly as both runtimes do."""
    standardized = (x - np.asarray(coefficients["center"])) / np.asarray(coefficients["scale"])
    return coefficients["intercept"] + standardized @ np.asarray(coefficients["weight"])


def out_of_fold_logits(
    fit_x: np.ndarray,
    fit_y: np.ndarray,
    fit_weights: np.ndarray,
    fit_session: np.ndarray,
    dense_x: np.ndarray,
    dense_session: np.ndarray,
    splits: int,
) -> Optional[np.ndarray]:
    """Fit de-overlapped folds and score every held-out runtime tick."""
    from sklearn.model_selection import StratifiedGroupKFold

    groups = np.unique(fit_session)
    if len(groups) < splits:
        return None

    oof = np.full(len(dense_x), np.nan, dtype=np.float64)
    splitter = StratifiedGroupKFold(n_splits=splits, shuffle=True, random_state=0)
    for train_index, test_index in splitter.split(
        fit_x, fit_y, groups=fit_session
    ):
        fold = fit_coefficients(
            fit_x[train_index], fit_y[train_index], fit_weights[train_index]
        )
        held_out = np.unique(fit_session[test_index])
        dense_test = np.isin(dense_session, held_out)
        oof[dense_test] = logits(dense_x[dense_test], fold)
    return oof if not np.isnan(oof).any() else None


def session_centered_scores(
    scores: np.ndarray,
    y: np.ndarray,
    sessions: Sequence[str],
) -> np.ndarray:
    """Re-express logits the way the runtime compares them.

    Startup calibration does not threshold the raw logit. It shifts the
    threshold by the session's own quiet level, so the effective decision is

        logit - STARTUP_STRENGTH * session_q95
            > base_logit - STARTUP_STRENGTH * TRAIN_IDLE_Q95_LOGIT

    Choosing an operating point on the raw logit therefore prices in a session
    shift of zero, which no session actually has. The left-hand side is the
    quantity the runtime compares, so it is the one the fit has to sweep.

    ``session_q95`` mirrors the runtime: the quantile is taken over the quiet
    prefix the calibration buffer covers, not over the whole quiet segment,
    because the runtime only ever sees that prefix and its noise.
    """
    prefix_windows = max(
        1,
        int(np.ceil(config.CALIBRATION_DURATION_MS / config.EVALUATION_INTERVAL_MS)),
    )
    sessions = np.asarray(sessions)
    centered = np.array(scores, dtype=np.float64)
    for session in np.unique(sessions):
        in_session = sessions == session
        idle_scores = scores[in_session & (y == IDLE_LABEL)][:prefix_windows]
        if idle_scores.size == 0:
            continue
        session_q95 = float(
            np.quantile(idle_scores, LightweightDetector.STARTUP_QUANTILE)
        )
        centered[in_session] -= LightweightDetector.STARTUP_STRENGTH * session_q95
    return centered


def base_threshold_from_centered(centered_threshold: float, idle_q95: float) -> float:
    """Convert a session-centered operating point back to BASE_THRESHOLD."""
    base_logit = centered_threshold + LightweightDetector.STARTUP_STRENGTH * idle_q95
    return float(1.0 / (1.0 + np.exp(-base_logit)))


def choose_base_threshold(
    scores: np.ndarray,
    y: np.ndarray,
    weights: np.ndarray,
    session: Optional[np.ndarray] = None,
    fp_target: float = 3.0,
    min_session_recall: float = 0.0,
) -> Tuple[float, Dict[str, float]]:
    """Pick the operating point the production promotion gate actually asks for.

    The gate is a false-positive ceiling with recall maximized underneath it, not
    best F1. Maximizing F1 lands well above the ceiling, because F1 trades recall
    against precision on equal terms while the runtime contract does not. Falls
    back to best F1 only when no candidate clears the ceiling.

    Every rate here is pooled over the whole corpus, and a pooled rate hides the
    one recording that fails. A single-feature experiment scored `98.0%` recall
    on this sweep while one capture sat at `62%`, because twenty-six good pairs
    outvoted it; the per-pair replay caught what the sweep could not. So the
    chosen point now also carries `worst_session_recall`, and `min_session_recall`
    can make it binding. Report it, and do not read the pooled recall alone.
    """
    probabilities = 1.0 / (1.0 + np.exp(-scores))
    candidates = np.unique(np.quantile(probabilities, np.linspace(0.001, 0.999, 999)))

    gated_threshold: Optional[float] = None
    gated_recall = -1.0
    gated: Dict[str, float] = {}

    best_threshold = 0.5
    best_f1 = -1.0
    best: Dict[str, float] = {}
    order = np.argsort(probabilities, kind="stable")
    ordered_probabilities = probabilities[order]
    ordered_weights = weights[order]
    ordered_motion = y[order] == MOTION_LABEL
    motion_weights = np.where(ordered_motion, ordered_weights, 0.0)
    idle_weights = np.where(~ordered_motion, ordered_weights, 0.0)
    motion_suffix = np.zeros(len(probabilities) + 1, dtype=np.float64)
    idle_suffix = np.zeros(len(probabilities) + 1, dtype=np.float64)
    motion_suffix[:-1] = np.cumsum(motion_weights[::-1], dtype=np.float64)[::-1]
    idle_suffix[:-1] = np.cumsum(idle_weights[::-1], dtype=np.float64)[::-1]
    motion_total = float(weights[y == MOTION_LABEL].sum())
    idle_total = float(weights[y == IDLE_LABEL].sum())

    session_motion = []
    if session is not None:
        motion = y == MOTION_LABEL
        for name in np.unique(session[motion]):
            rows = motion & (session == name)
            session_probabilities = probabilities[rows]
            session_weights = weights[rows]
            session_order = np.argsort(session_probabilities, kind="stable")
            session_probabilities = session_probabilities[session_order]
            session_weights = session_weights[session_order]
            suffix = np.zeros(len(session_weights) + 1, dtype=np.float64)
            suffix[:-1] = np.cumsum(
                session_weights[::-1], dtype=np.float64
            )[::-1]
            session_motion.append(
                (session_probabilities, suffix, float(session_weights.sum()))
            )

    for threshold in candidates:
        boundary = int(np.searchsorted(ordered_probabilities, threshold, side="right"))
        tp = float(motion_suffix[boundary])
        fp = float(idle_suffix[boundary])
        fn = motion_total - tp
        tn = idle_total - fp
        if tp <= 0.0:
            continue
        recall = tp / (tp + fn)
        precision = tp / (tp + fp) if (tp + fp) > 0.0 else 0.0
        if precision <= 0.0:
            continue
        f1 = 2.0 * precision * recall / (precision + recall)
        fp_rate = 100.0 * fp / (fp + tn) if (fp + tn) > 0.0 else 0.0
        worst_session_recall = 100.0 * recall
        if session_motion:
            per_session = []
            for session_probabilities, suffix, total in session_motion:
                session_boundary = int(
                    np.searchsorted(session_probabilities, threshold, side="right")
                )
                hit = float(suffix[session_boundary])
                if total > 0.0:
                    per_session.append(100.0 * hit / total)
            if per_session:
                worst_session_recall = float(min(per_session))
        point = {
            "f1": 100.0 * f1,
            "recall": 100.0 * recall,
            "fp_rate": fp_rate,
            "worst_session_recall": worst_session_recall,
        }

        if worst_session_recall < min_session_recall:
            continue

        if fp_rate <= fp_target and recall > gated_recall:
            gated_recall = recall
            gated_threshold = float(threshold)
            gated = point
        if f1 > best_f1:
            best_f1 = f1
            best_threshold = float(threshold)
            best = point

    if gated_threshold is not None:
        gated["gate"] = fp_target
        return gated_threshold, gated
    if best_f1 < 0.0:
        raise FitError("No usable operating point found")
    print(
        f"WARNING: no operating point holds false positives at or under {fp_target}%; "
        "falling back to best F1",
        file=sys.stderr,
    )
    return best_threshold, best


def replace_assignment(text: str, pattern: str, replacement: str, label: str) -> str:
    """Replace exactly one assignment, refusing to guess when the anchor moved."""
    updated, count = re.subn(pattern, replacement, text, count=1, flags=re.MULTILINE)
    if count != 1:
        raise FitError(f"Could not locate {label}; update the exporter alongside the source")
    return updated


def render_python(coefficients: Dict[str, Any], base_threshold: float, idle_q95: float) -> str:
    """Rewrite the Python detector constants in place."""
    center, scale = coefficients["center"], coefficients["scale"]
    weight, intercept = coefficients["weight"], coefficients["intercept"]
    text = PYTHON_SOURCE.read_text()
    for pattern, replacement, label in (
        (r"^    FEATURE_CENTER = .*$", f"    FEATURE_CENTER = ({center[0]!r}, {center[1]!r})", "FEATURE_CENTER"),
        (r"^    FEATURE_SCALE = .*$", f"    FEATURE_SCALE = ({scale[0]!r}, {scale[1]!r})", "FEATURE_SCALE"),
        (r"^    FEATURE_WEIGHT = .*$", f"    FEATURE_WEIGHT = ({weight[0]!r}, {weight[1]!r})", "FEATURE_WEIGHT"),
        (r"^    INTERCEPT = .*$", f"    INTERCEPT = {intercept!r}", "INTERCEPT"),
        (r"^    BASE_THRESHOLD = .*$", f"    BASE_THRESHOLD = {base_threshold!r}", "BASE_THRESHOLD"),
        (r"^    TRAIN_IDLE_Q95_LOGIT = .*$", f"    TRAIN_IDLE_Q95_LOGIT = {idle_q95!r}", "TRAIN_IDLE_Q95_LOGIT"),
    ):
        text = replace_assignment(text, pattern, replacement, label)
    return text


def render_cpp(coefficients: Dict[str, Any], base_threshold: float, idle_q95: float) -> str:
    """Rewrite the C++ detector constants in place."""
    center, scale = coefficients["center"], coefficients["scale"]
    weight, intercept = coefficients["weight"], coefficients["intercept"]
    text = CPP_SOURCE.read_text()
    for name, value in (
        ("LIGHTWEIGHT_AUTOCORR_CENTER", center[0]),
        ("LIGHTWEIGHT_AUTOCORR_SCALE", scale[0]),
        ("LIGHTWEIGHT_AUTOCORR_WEIGHT", weight[0]),
        ("LIGHTWEIGHT_TURB_IQR_OVER_MEAN_AGGR_CENTER", center[1]),
        ("LIGHTWEIGHT_TURB_IQR_OVER_MEAN_AGGR_SCALE", scale[1]),
        ("LIGHTWEIGHT_TURB_IQR_OVER_MEAN_AGGR_WEIGHT", weight[1]),
        ("LIGHTWEIGHT_INTERCEPT", intercept),
        ("LIGHTWEIGHT_DEFAULT_THRESHOLD", base_threshold),
        ("LIGHTWEIGHT_TRAIN_IDLE_Q95_LOGIT", idle_q95),
    ):
        text = replace_assignment(
            text,
            rf"^constexpr float {name} = .*$",
            f"constexpr float {name} = {float(value)!r}f;",
            name,
        )
    return text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--apply", action="store_true", help="write the constants into both runtimes")
    parser.add_argument("--splits", type=int, default=5, help="grouped OOF folds (default: 5)")
    parser.add_argument("--fp-target", type=float, default=3.0,
                        help="false-positive ceiling for the operating point (default: 3.0)")
    parser.add_argument(
        "--fp-weight",
        type=float,
        default=DEFAULT_FP_WEIGHT,
        help=(
            "idle-class multiplier for the coefficient fit after balancing "
            f"(default: {DEFAULT_FP_WEIGHT}; High Accuracy uses 1.75)"
        ),
    )
    parser.add_argument("--min-session-recall", type=float, default=0.0,
                        help="reject operating points whose worst single session falls below "
                             "this recall (default: 0.0, report only)")
    parser.add_argument(
        "--centered-threshold-logit",
        type=float,
        default=None,
        help=(
            "override the OOF operating point with a session-centered logit "
            "selected by sequential production replay"
        ),
    )
    parser.add_argument("--quiet", action="store_true", help="suppress per-dataset progress")
    args = parser.parse_args()

    selected_band = tuple(config.DEFAULT_SUBCARRIERS)
    pairs = iter_training_pairs()
    print(
        f"Fitting Lightweight on {len(pairs)} train pairs "
        f"(band={selected_band}, window={config.SEGMENTATION_WINDOW_SIZE_MS} ms, "
        f"fp_weight={args.fp_weight})"
    )
    corpus = build_corpus(pairs, selected_band, progress=not args.quiet)

    x, y = corpus["x"], corpus["y"]
    deoverlapped = corpus["deoverlapped"]
    weights = balanced_sample_weights(y, corpus["chip"], corpus["session"])
    print(
        f"\nEvaluations: {len(y)} ({int((y == MOTION_LABEL).sum())} motion, "
        f"{int((y == IDLE_LABEL).sum())} idle); "
        f"{int(deoverlapped.sum())} de-overlapped windows fit the coefficients"
    )

    # Coefficients come from the de-overlapped subset so consecutive samples
    # share no packets; the operating point is swept over every evaluation
    # below, because that is the cadence the runtime thresholds at.
    fit_x, fit_y = x[deoverlapped], y[deoverlapped]
    fit_weights = apply_idle_fp_weight(
        balanced_sample_weights(
            fit_y, corpus["chip"][deoverlapped], corpus["session"][deoverlapped]
        ),
        fit_y,
        args.fp_weight,
    )
    coefficients = fit_coefficients(fit_x, fit_y, fit_weights)

    oof = out_of_fold_logits(
        fit_x,
        fit_y,
        fit_weights,
        corpus["session"][deoverlapped],
        x,
        corpus["session"],
        args.splits,
    )
    if oof is None:
        print("WARNING: grouped OOF unavailable; operating point is in-sample", file=sys.stderr)
        oof = logits(x, coefficients)
        source = "in-sample"
    else:
        source = f"grouped OOF ({args.splits} folds)"
    idle_logits = logits(fit_x[fit_y == IDLE_LABEL], coefficients)
    idle_q95 = float(np.quantile(idle_logits, LightweightDetector.STARTUP_QUANTILE))

    # Sweep the quantity the runtime compares, then convert the chosen point
    # back into the constant the runtime stores.
    centered = session_centered_scores(oof, y, corpus["session"])
    centered_threshold, metrics = choose_base_threshold(
        centered,
        y,
        weights,
        session=corpus["session"],
        fp_target=args.fp_target,
        min_session_recall=args.min_session_recall,
    )
    centered_logit = float(np.log(centered_threshold / (1.0 - centered_threshold)))
    if args.centered_threshold_logit is not None:
        centered_logit = float(args.centered_threshold_logit)
    base_threshold = base_threshold_from_centered(centered_logit, idle_q95)

    center, scale = coefficients["center"], coefficients["scale"]
    weight, intercept = coefficients["weight"], coefficients["intercept"]
    print(f"\nOperating point from {source}:")
    print(f"  F1={metrics['f1']:.3f}%  recall={metrics['recall']:.3f}%  fp_rate={metrics['fp_rate']:.3f}%")
    worst_session = metrics.get("worst_session_recall")
    if worst_session is not None:
        print(f"  worst session recall={worst_session:.3f}%")
        if metrics["recall"] - worst_session > 10.0:
            print(
                f"WARNING: pooled recall {metrics['recall']:.1f}% hides a session at "
                f"{worst_session:.1f}%; check the per-pair replay before trusting this point",
                file=sys.stderr,
            )
    if args.centered_threshold_logit is not None:
        print(
            "  sequential-replay centered-logit override="
            f"{centered_logit!r}"
        )
    print("\nFitted constants:")
    print(f"  FEATURE_CENTER       = ({center[0]!r}, {center[1]!r})")
    print(f"  FEATURE_SCALE        = ({scale[0]!r}, {scale[1]!r})")
    print(f"  FEATURE_WEIGHT       = ({weight[0]!r}, {weight[1]!r})")
    print(f"  INTERCEPT            = {intercept!r}")
    print(f"  BASE_THRESHOLD       = {base_threshold!r}")
    print(f"  TRAIN_IDLE_Q95_LOGIT = {idle_q95!r}")

    if not args.apply:
        print("\nDry run; pass --apply to write both runtimes.")
        return 0

    python_text = render_python(coefficients, base_threshold, idle_q95)
    cpp_text = render_cpp(coefficients, base_threshold, idle_q95)
    atomic_write_set(
        {
            PYTHON_SOURCE: python_text.encode("utf-8"),
            CPP_SOURCE: cpp_text.encode("utf-8"),
        }
    )
    print(f"\nWrote {PYTHON_SOURCE.relative_to(REPO_ROOT)} and {CPP_SOURCE.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
