# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Model metrics, streaming evaluation, and promotion gates."""

from __future__ import annotations

from tools.lib.bootstrap import setup_paths

setup_paths()

import argparse
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from tools.lib.dataset_metadata import (
    admitted_dataset_role,
    paired_dataset_role,
    resolve_entry_path,
)
from time import perf_counter
from config import (
    DEFAULT_SUBCARRIERS,
    EVALUATION_INTERVAL_MS,
    MOTION_OFF_HITS,
    MOTION_ON_HITS,
)
from detector_interface import MotionState
from tools.lib.runtime_policy import (
    RuntimeMotionPolicy,
    derive_detector_timing,  # noqa: F401 - re-exported for training callers
)
from tools.lib.performance_report import (
    STRESS_TARGET_FP_RATE,
    STRESS_TARGET_RECALL,
    build_ml_replay_rows,
    load_or_compute_ml_replay_rows,
)
from tools.lib.occupancy_thinning import (
    OCCUPANCY_GATE_PERCENT,
    OCCUPANCY_GATE_TRANSFORM,
    TARGET_PPS as OCCUPANCY_GATE_TARGET_PPS,
    admit_packets,
    thin_packets,
    thin_to_occupancy,
)

from .augmentation import (
    _implementation_source_digest,
    _load_npz_packets_cached,
)

from .dataset import (
    format_duration,
    get_file_metadata,
    load_dataset_info,
    load_training_matrix,
    normalize_dataset_roles,
    parse_chip_filter,
    parse_environment_filter,
)

from .export import (
    exported_weight_matrices,
    extract_model_weights,
    load_exported_ml_weights,
    predict_exported_probabilities_from_weights,
    predict_probabilities_from_arrays,
)

from .feature_cache import (
    StreamingFeatureExtractor,
    TRAINING_FEATURES,
    _feature_rows_use_runtime_cache,
    _host_feature_stream_provenance,
    build_host_feature_rows,
    canonical_trajectory_bin,
    load_or_compute_host_feature_rows,
)

from .preprocessing import (
    get_preprocessor_arrays,
)

DEFAULT_PAIRED_GATE_CHIPS = ('C3', 'C5', 'C6', 'ESP32', 'S3')


DEFAULT_GAIN_STRESS_SCALES = (0.5, 0.75, 1.0, 1.25, 1.5, 2.0)


GAIN_SENSITIVE_FEATURES = ()


DEFAULT_GATE_TARGET_RECALL = 95.0


DEFAULT_GATE_TARGET_FP_RATE = 5.0


ROBUST_TAIL_GROUPS = 5


OOF_F1_EQUIVALENCE_MARGIN = 0.2


DEFAULT_REPORT_GROUP_KEYS = (
    'chip',
    'environment_group',
    'lineage_group',
    'session_group',
    'source_file',
)


def parse_gain_stress_scales(value):
    """Parse comma-separated positive gain stress multipliers."""
    if value is None:
        return tuple(DEFAULT_GAIN_STRESS_SCALES)
    if isinstance(value, (list, tuple)):
        parts = value
    else:
        parts = str(value).split(',')

    scales = []
    for item in parts:
        text = str(item).strip()
        if not text:
            continue
        try:
            scale = float(text)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"Invalid gain stress scale '{text}'"
            ) from exc
        if scale <= 0.0:
            raise argparse.ArgumentTypeError("gain stress scales must be > 0")
        scales.append(scale)
    if not scales:
        raise argparse.ArgumentTypeError("at least one gain stress scale is required")
    return tuple(scales)


def evaluate_probabilities(y_true, y_prob, threshold=0.5):
    """Evaluate predicted probabilities with the deployment-equivalent threshold."""
    y_true = np.asarray(y_true)
    y_prob = np.asarray(y_prob)
    y_pred = (y_prob > threshold).astype(int).flatten()
    tp = int(np.sum((y_true == 1) & (y_pred == 1)))
    fp = int(np.sum((y_true == 0) & (y_pred == 1)))
    tn = int(np.sum((y_true == 0) & (y_pred == 0)))
    fn = int(np.sum((y_true == 1) & (y_pred == 0)))

    recall = tp / (tp + fn) * 100 if (tp + fn) > 0 else 0.0
    precision = tp / (tp + fp) * 100 if (tp + fp) > 0 else 0.0
    fp_rate = fp / (fp + tn) * 100 if (fp + tn) > 0 else 0.0
    f1 = 2 * tp / (2 * tp + fp + fn) * 100 if (2 * tp + fp + fn) > 0 else 0.0

    return {
        'recall': recall,
        'precision': precision,
        'fp_rate': fp_rate,
        'f1': f1,
        'tp': tp,
        'fp': fp,
        'tn': tn,
        'fn': fn,
    }


def build_group_report(y_true, y_prob, group_values):
    """Compute per-group metrics and worst-group summaries."""
    if group_values is None:
        return None

    group_values = np.asarray(group_values)
    rows = []
    for group_name in sorted({str(v) for v in group_values}):
        if not group_name or group_name == 'unknown-environment':
            continue
        mask = (group_values == group_name)
        if not np.any(mask):
            continue
        metrics = evaluate_probabilities(y_true[mask], y_prob[mask])
        rows.append({
            'group': group_name,
            'samples': int(np.sum(mask)),
            'positives': int(np.sum(y_true[mask] == 1)),
            'negatives': int(np.sum(y_true[mask] == 0)),
            **metrics,
        })

    if not rows:
        return None

    recall_rows = [r for r in rows if r['positives'] > 0]
    fp_rows = [r for r in rows if r['negatives'] > 0]
    rows_by_recall = sorted(recall_rows or rows, key=lambda r: (r['recall'], r['fp_rate'], -r['samples']))
    rows_by_fp = sorted(fp_rows or rows, key=lambda r: (-r['fp_rate'], r['recall'], -r['samples']))
    recall_tail = rows_by_recall[:min(ROBUST_TAIL_GROUPS, len(rows_by_recall))]
    fp_tail = rows_by_fp[:min(ROBUST_TAIL_GROUPS, len(rows_by_fp))]

    def tail_summary(tail_rows, metric, denominator):
        return {
            'value': float(np.mean([row[metric] for row in tail_rows])),
            'groups': [row['group'] for row in tail_rows],
            # One changed evaluation per group is the natural resolution of
            # these blocked rates; use its mean as the equivalence margin.
            'resolution': float(np.mean([
                100.0 / max(int(row[denominator]), 1)
                for row in tail_rows
            ])),
        }

    return {
        'rows': rows,
        'worst_recall': rows_by_recall[0],
        'worst_fp_rate': rows_by_fp[0],
        'tail_recall': tail_summary(recall_tail, 'recall', 'positives'),
        'tail_fp_rate': tail_summary(fp_tail, 'fp_rate', 'negatives'),
        'count': len(rows),
    }


def apply_gain_stress_to_features(X, feature_names, scale):
    """Scale only feature dimensions that move linearly with amplitude gain."""
    X_scaled = np.asarray(X, dtype=np.float32).copy()
    sensitive_indices = [
        idx for idx, name in enumerate(feature_names)
        if name in GAIN_SENSITIVE_FEATURES
    ]
    if sensitive_indices:
        X_scaled[:, sensitive_indices] *= np.float32(scale)
    return X_scaled, sensitive_indices


def evaluate_gain_stress_gate(environment_filter=None, excluded_chips=None,
                              scales=DEFAULT_GAIN_STRESS_SCALES):
    """Evaluate current exported model under artificial gain scaling."""
    environment_filter = parse_environment_filter(environment_filter)
    excluded_chips = parse_chip_filter(excluded_chips)
    scales = parse_gain_stress_scales(scales)

    weights_module = load_exported_ml_weights()
    feature_names = list(getattr(weights_module, 'FEATURE_NAMES', TRAINING_FEATURES))
    matrix, _ = load_training_matrix(
        environment_filter=environment_filter,
        excluded_chips=excluded_chips,
        feature_names=feature_names,
    )
    X = np.asarray(matrix['X'], dtype=np.float32)
    y = np.asarray(matrix['y'], dtype=np.int8)
    actual_feature_names = list(matrix['feature_names'])
    sample_context = matrix['sample_context']
    stats = matrix['stats']
    if len(X) == 0:
        raise RuntimeError("No empty/static_presence/motion packets found for gain stress gate")

    if list(actual_feature_names) != feature_names:
        raise RuntimeError(
            "Extracted feature order does not match exported model: "
            f"{actual_feature_names} != {feature_names}"
        )

    scaled_features = [
        name for name in feature_names if name in GAIN_SENSITIVE_FEATURES
    ]
    results = {
        'feature_names': feature_names,
        'scaled_features': scaled_features,
        'invariant_features': [
            name for name in feature_names if name not in scaled_features
        ],
        'stats': stats,
        'samples': int(len(X)),
        'scales': {},
    }

    for gain_scale in scales:
        X_stressed, sensitive_indices = apply_gain_stress_to_features(
            X,
            feature_names,
            gain_scale,
        )
        y_prob = predict_exported_probabilities_from_weights(weights_module, X_stressed)
        scale_result = {
            'scale': float(gain_scale),
            'sensitive_indices': [int(idx) for idx in sensitive_indices],
            'overall': evaluate_probabilities(y, y_prob),
            'group_reports': {},
        }
        for group_key in DEFAULT_REPORT_GROUP_KEYS:
            report = build_group_report(y, y_prob, sample_context.get(group_key))
            if report is not None:
                scale_result['group_reports'][group_key] = report
        results['scales'][float(gain_scale)] = scale_result
    return results


def evaluate_candidate_gain_stress(model, scaler, feature_names, *,
                                   environment_filter=None, excluded_chips=None,
                                   dataset_roles=('train', 'selection', 'holdout'),
                                   scales=DEFAULT_GAIN_STRESS_SCALES):
    """Evaluate one in-memory candidate under artificial gain scaling."""
    environment_filter = parse_environment_filter(environment_filter)
    excluded_chips = parse_chip_filter(excluded_chips)
    scales = parse_gain_stress_scales(scales)
    center, scale = get_preprocessor_arrays(scaler)
    layers = _layer_arrays_from_model(model)
    matrix, _ = load_training_matrix(
        environment_filter=environment_filter,
        excluded_chips=excluded_chips,
        feature_names=feature_names,
        use_cache=True,
        dataset_roles=dataset_roles,
    )
    X = np.asarray(matrix['X'], dtype=np.float32)
    y = np.asarray(matrix['y'], dtype=np.int8)
    actual_feature_names = list(matrix['feature_names'])
    sample_context = matrix['sample_context']
    stats = matrix['stats']
    if len(X) == 0:
        raise RuntimeError("No empty/static_presence/motion packets found for gain stress gate")
    if list(actual_feature_names) != list(feature_names):
        raise RuntimeError(
            "Extracted feature order does not match trained model: "
            f"{actual_feature_names} != {feature_names}"
        )

    scaled_features = [
        name for name in feature_names if name in GAIN_SENSITIVE_FEATURES
    ]
    results = {
        'feature_names': list(feature_names),
        'scaled_features': scaled_features,
        'invariant_features': [
            name for name in feature_names if name not in scaled_features
        ],
        'stats': stats,
        'samples': int(len(X)),
        'scales': {},
    }
    for gain_scale in scales:
        X_stressed, sensitive_indices = apply_gain_stress_to_features(
            X,
            feature_names,
            gain_scale,
        )
        y_prob = _batch_predict_probabilities(X_stressed, center, scale, layers)
        scale_result = {
            'scale': float(gain_scale),
            'sensitive_indices': [int(idx) for idx in sensitive_indices],
            'overall': evaluate_probabilities(y, y_prob),
            'group_reports': {},
        }
        for group_key in DEFAULT_REPORT_GROUP_KEYS:
            report = build_group_report(y, y_prob, sample_context.get(group_key))
            if report is not None:
                scale_result['group_reports'][group_key] = report
        results['scales'][float(gain_scale)] = scale_result
    return results


def print_gain_stress_summary(results, title="EXPORTED ML GAIN-STRESS GATE"):
    """Print a compact gain stress report."""
    stats = results.get('stats', {})
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)
    print(f"Samples: {results['samples']}")
    print(f"Chips: {', '.join(stats.get('chips', []))}")
    if stats.get('environment_groups'):
        print(f"Environments: {', '.join(stats['environment_groups'])}")
    print(f"Scaled features: {', '.join(results['scaled_features']) or 'none'}")
    print(f"Invariant features: {', '.join(results['invariant_features']) or 'none'}")
    if not results['scaled_features']:
        print("Note: current export has no gain-sensitive feature dimensions; this")
        print("gate is informational and acts as a regression guard against")
        print("reintroducing them.")
    print()
    print(
        "  scale | recall  precision  FP rate      F1 | "
        "worst chip recall        worst chip FP"
    )
    print("  " + "-" * 86)
    for scale in sorted(results['scales']):
        row = results['scales'][scale]
        overall = row['overall']
        chip_report = row['group_reports'].get('chip', {})
        worst_recall = chip_report.get('worst_recall', {})
        worst_fp = chip_report.get('worst_fp_rate', {})
        recall_label = (
            f"{worst_recall.get('group', 'n/a')} {worst_recall.get('recall', 0.0):5.1f}%"
        )
        fp_label = (
            f"{worst_fp.get('group', 'n/a')} {worst_fp.get('fp_rate', 0.0):5.1f}%"
        )
        print(
            f"  {scale:5.2f} | "
            f"{overall['recall']:6.1f}% "
            f"{overall['precision']:9.1f}% "
            f"{overall['fp_rate']:7.1f}% "
            f"{overall['f1']:7.1f}% | "
            f"{recall_label:24} {fp_label}"
        )

    for group_key in ('environment_group', 'session_group', 'source_file'):
        print(f"\nWorst {group_key} by scale:")
        for scale in sorted(results['scales']):
            report = results['scales'][scale]['group_reports'].get(group_key)
            if not report:
                continue
            worst_recall = report['worst_recall']
            worst_fp = report['worst_fp_rate']
            print(
                f"  {scale:5.2f}: "
                f"R {worst_recall['group']}={worst_recall['recall']:.1f}% | "
                f"FP {worst_fp['group']}={worst_fp['fp_rate']:.1f}%"
            )


def build_candidate_key(cv_results):
    """Ranking key for seeds/architectures under the robust evaluation protocol."""
    group_reports = cv_results.get('group_reports', {})
    if not group_reports and cv_results.get('candidate_key') is not None:
        return tuple(float(value) for value in cv_results['candidate_key'])
    # Real sessions lead ranking whenever the provenance split exists; the
    # synthetic stress metrics act only as regression guards in the comparison.
    session_report = (
        group_reports.get('real_session_group')
        or group_reports.get('session_group')
        or {}
    )
    chip_report = group_reports.get('chip') or {}

    worst_session_recall = session_report.get('worst_recall', {}).get('recall', 0.0)
    worst_session_fp = session_report.get('worst_fp_rate', {}).get('fp_rate', 100.0)
    worst_chip_recall = chip_report.get('worst_recall', {}).get('recall', 0.0)
    worst_chip_fp = chip_report.get('worst_fp_rate', {}).get('fp_rate', 100.0)
    tail_session_recall = session_report.get('tail_recall', {}).get(
        'value', worst_session_recall)
    tail_session_fp = session_report.get('tail_fp_rate', {}).get(
        'value', worst_session_fp)

    return (
        tail_session_recall,
        -tail_session_fp,
        worst_session_recall,
        worst_chip_recall,
        -worst_session_fp,
        -worst_chip_fp,
        cv_results.get('oof_f1', 0.0),
        cv_results.get('f1_mean', 0.0),
    )


def _row_resolution(row, denominator):
    """Return the percentage-point step represented by one changed outcome."""
    if not row:
        return 0.0
    count = int(row.get(denominator, 0))
    return 100.0 / count if count > 0 else 0.0


def compare_robust_cv(candidate, baseline):
    """Compare grouped OOF results with one-event equivalence margins."""
    candidate_reports = candidate.get('group_reports', {})
    baseline_reports = baseline.get('group_reports', {})
    # Real sessions decide movement; combined reports remain the fallback for
    # datasets that never produced a provenance split.
    candidate_session = (
        candidate_reports.get('real_session_group')
        or candidate_reports.get('session_group')
        or {}
    )
    baseline_session = (
        baseline_reports.get('real_session_group')
        or baseline_reports.get('session_group')
        or {}
    )
    candidate_synthetic = candidate_reports.get('synthetic_session_group') or {}
    baseline_synthetic = baseline_reports.get('synthetic_session_group') or {}
    candidate_chip = candidate_reports.get('chip') or {}
    baseline_chip = baseline_reports.get('chip') or {}

    checks = []

    def add_higher(label, candidate_value, baseline_value, margin, guard_only=False):
        delta = float(candidate_value) - float(baseline_value)
        checks.append({
            'label': label,
            'delta': delta,
            'margin': float(margin),
            'regressed': delta < -float(margin) - 1e-9,
            'improved': not guard_only and delta > float(margin) + 1e-9,
        })

    def add_lower(label, candidate_value, baseline_value, margin, guard_only=False):
        delta = float(candidate_value) - float(baseline_value)
        checks.append({
            'label': label,
            'delta': delta,
            'margin': float(margin),
            'regressed': delta > float(margin) + 1e-9,
            'improved': not guard_only and delta < -float(margin) - 1e-9,
        })

    def add_session_checks(candidate_report, baseline_report, scope,
                           guard_only=False):
        candidate_worst_recall = candidate_report.get('worst_recall', {})
        baseline_worst_recall = baseline_report.get('worst_recall', {})
        add_higher(
            f'CV worst-{scope} recall',
            candidate_worst_recall.get('recall', 0.0),
            baseline_worst_recall.get('recall', 0.0),
            max(
                _row_resolution(candidate_worst_recall, 'positives'),
                _row_resolution(baseline_worst_recall, 'positives'),
            ),
            guard_only=guard_only,
        )

        candidate_worst_fp = candidate_report.get('worst_fp_rate', {})
        baseline_worst_fp = baseline_report.get('worst_fp_rate', {})
        add_lower(
            f'CV worst-{scope} FP',
            candidate_worst_fp.get('fp_rate', 100.0),
            baseline_worst_fp.get('fp_rate', 100.0),
            max(
                _row_resolution(candidate_worst_fp, 'negatives'),
                _row_resolution(baseline_worst_fp, 'negatives'),
            ),
            guard_only=guard_only,
        )

        for key, label, higher_is_better in (
            ('tail_recall', f'CV tail-{scope} recall', True),
            ('tail_fp_rate', f'CV tail-{scope} FP', False),
        ):
            candidate_tail = candidate_report.get(key, {})
            baseline_tail = baseline_report.get(key, {})
            candidate_value = candidate_tail.get('value')
            baseline_value = baseline_tail.get('value')
            if candidate_value is None or baseline_value is None:
                continue
            margin = max(
                float(candidate_tail.get('resolution', 0.0)),
                float(baseline_tail.get('resolution', 0.0)),
            )
            if higher_is_better:
                add_higher(label, candidate_value, baseline_value, margin,
                           guard_only=guard_only)
            else:
                add_lower(label, candidate_value, baseline_value, margin,
                          guard_only=guard_only)

    add_session_checks(candidate_session, baseline_session, 'session')
    # Synthetic stress sessions may only block: a candidate cannot win by
    # improving synthetic derivatives, nor materially regress on them.
    if candidate_synthetic and baseline_synthetic:
        add_session_checks(
            candidate_synthetic,
            baseline_synthetic,
            'synthetic-session',
            guard_only=True,
        )

    for report_key, label, higher_is_better, denominator in (
        ('worst_recall', 'CV worst-chip recall', True, 'positives'),
        ('worst_fp_rate', 'CV worst-chip FP', False, 'negatives'),
    ):
        candidate_row = candidate_chip.get(report_key, {})
        baseline_row = baseline_chip.get(report_key, {})
        metric = 'recall' if higher_is_better else 'fp_rate'
        margin = max(
            _row_resolution(candidate_row, denominator),
            _row_resolution(baseline_row, denominator),
        )
        if higher_is_better:
            add_higher(label, candidate_row.get(metric, 0.0), baseline_row.get(metric, 0.0), margin)
        else:
            add_lower(label, candidate_row.get(metric, 100.0), baseline_row.get(metric, 100.0), margin)

    add_higher(
        'Blocked OOF F1',
        candidate.get('oof_f1', 0.0),
        baseline.get('oof_f1', 0.0),
        OOF_F1_EQUIVALENCE_MARGIN,
    )

    regressions = [check for check in checks if check['regressed']]
    improvements = [check for check in checks if check['improved']]
    return {
        'passed': not regressions and bool(improvements),
        'non_regression': not regressions,
        'material_improvement': bool(improvements),
        'checks': checks,
        'regressions': regressions,
        'improvements': improvements,
    }


PAIRED_ALARM_BUDGET = 1


def _gate_row_passes(row):
    """Per-replay pass criterion under the link-class policy.

    Normal-link replays keep the strict production bar, allowing at most
    ``PAIRED_ALARM_BUDGET`` runtime-filtered alarms for real micro-motion of
    the present person. Real weak-link (`low_rssi`) replays are stress
    diagnostics: they use the relaxed stress targets, and their alarms are
    reported but bounded only by the per-recording non-regression checks.
    """
    if row.get('low_rssi'):
        return (
            row['recall'] > STRESS_TARGET_RECALL
            and row['fp_rate'] < STRESS_TARGET_FP_RATE
        )
    return (
        row['recall'] > DEFAULT_GATE_TARGET_RECALL
        and row['fp_rate'] < DEFAULT_GATE_TARGET_FP_RATE
        and row.get('effective_alarms', 0) <= PAIRED_ALARM_BUDGET
    )


def summarize_gate(by_chip):
    """Aggregate per-chip gate metrics."""
    rows = list(by_chip.values())
    if not rows:
        return None
    return {
        'by_chip': by_chip,
        'pass_count': int(sum(1 for row in rows if _gate_row_passes(row))),
        'mean_recall': float(np.mean([row['recall'] for row in rows])),
        'worst_chip_recall': float(np.min([row['recall'] for row in rows])),
        'mean_fp_rate': float(np.mean([row['fp_rate'] for row in rows])),
        'max_fp_rate': float(np.max([row['fp_rate'] for row in rows])),
        'mean_f1': float(np.mean([row['f1'] for row in rows])),
        'worst_chip_f1': float(np.min([row['f1'] for row in rows])),
        'total_fp': int(sum(row['fp'] for row in rows)),
        'total_fn': int(sum(row['fn'] for row in rows)),
        'total_effective_alarms': int(sum(row.get('effective_alarms', 0) for row in rows)),
        'max_effective_alarms': int(max(row.get('effective_alarms', 0) for row in rows)),
    }


def evaluate_runtime_policy_evaluations(raw_motion_states):
    """Apply production hit filtering to states already sampled at eval ticks."""
    policy = RuntimeMotionPolicy(
        evaluation_interval_ms=EVALUATION_INTERVAL_MS,
        motion_on_hits=MOTION_ON_HITS,
        motion_off_hits=MOTION_OFF_HITS,
    )
    effective_alarms = 0
    false_motion_evaluations = 0
    for raw_motion in raw_motion_states:
        raw_state = MotionState.MOTION if raw_motion else MotionState.IDLE
        effective_state, changed = policy.apply_state(raw_state)
        if changed and effective_state == MotionState.MOTION:
            effective_alarms += 1
        if effective_state == MotionState.MOTION:
            false_motion_evaluations += 1
    return {
        'effective_alarms': effective_alarms,
        'false_motion_evaluations': false_motion_evaluations,
    }


def _layer_arrays_from_model(model):
    """Return [(weights, biases, is_output), ...] arrays from a torch model."""
    raw_weights = extract_model_weights(model)
    layers = []
    for idx in range(0, len(raw_weights), 2):
        weights = raw_weights[idx]
        biases = raw_weights[idx + 1]
        is_output = idx == len(raw_weights) - 2
        layers.append((weights, biases, is_output))
    return layers


def _batch_predict_probabilities(features, center, scale, layers):
    """Compatibility wrapper around the shared runtime-array inference."""
    return predict_probabilities_from_arrays(features, center, scale, layers)


class StreamingEvaluator:
    """Evaluate a trained model with the runtime-equivalent feature path."""

    def __init__(self, model, scaler, feature_names):
        self.extractor = StreamingFeatureExtractor(feature_names)
        self.center, self.scale = get_preprocessor_arrays(scaler)
        self.layers = _layer_arrays_from_model(model)

    def process_packet(self, csi_data):
        features = self.extractor.process_packet(csi_data)
        if features is None:
            return None
        probabilities = _batch_predict_probabilities(
            np.asarray(features, dtype=np.float32).reshape(1, -1),
            self.center,
            self.scale,
            self.layers,
        )
        return float(probabilities[0])


class ArrayStreamingEvaluator:
    """Runtime-equivalent evaluator backed by exported weight arrays."""

    def __init__(self, center, scale, layers, feature_names):
        self.extractor = StreamingFeatureExtractor(feature_names)
        self.center = center
        self.scale = scale
        self.layers = layers

    def process_packet(self, csi_data):
        features = self.extractor.process_packet(csi_data)
        if features is None:
            return None
        probabilities = _batch_predict_probabilities(
            np.asarray(features, dtype=np.float32).reshape(1, -1),
            self.center,
            self.scale,
            self.layers,
        )
        return float(probabilities[0])


def _evaluate_replay_row_probabilities(center, scale, layers, rows):
    """Predict one already-evaluated replay-row stream."""
    X = np.asarray(rows.get('X', np.empty((0, 0), dtype=np.float32)), dtype=np.float32)
    if X.size == 0:
        return np.zeros(0, dtype=np.float64), 0
    probabilities = _batch_predict_probabilities(X, center, scale, layers)
    return probabilities, int(len(X))


def _evaluate_replay_row_split(center, scale, layers,
                               static_presence_rows, motion_rows, threshold=0.5):
    """Evaluate paired replay metrics from canonical time-aware replay rows."""
    static_probs, static_eval_count = _evaluate_replay_row_probabilities(
        center, scale, layers, static_presence_rows
    )
    motion_probs, motion_eval_count = _evaluate_replay_row_probabilities(
        center, scale, layers, motion_rows
    )
    static_presence_motion_states = static_probs > threshold
    motion_states = motion_probs > threshold
    static_presence_motion_packets = int(np.sum(static_presence_motion_states))
    motion_with_motion = int(np.sum(motion_states))
    motion_without_motion = int(motion_eval_count - motion_with_motion)
    tp = motion_with_motion
    fn = motion_without_motion
    fp = static_presence_motion_packets
    tn = max(static_eval_count - static_presence_motion_packets, 0)
    recall = tp / (tp + fn) * 100.0 if (tp + fn) else 0.0
    precision = tp / (tp + fp) * 100.0 if (tp + fp) else 0.0
    fp_rate = fp / static_eval_count * 100.0 if static_eval_count else 0.0
    f1 = (
        2 * (precision / 100.0) * (recall / 100.0) / ((precision + recall) / 100.0) * 100.0
        if (precision + recall)
        else 0.0
    )
    return {
        'recall': float(recall),
        'precision': float(precision),
        'fp_rate': float(fp_rate),
        'f1': float(f1),
        'tp': int(tp),
        'fp': int(fp),
        'tn': int(tn),
        'fn': int(fn),
        'static_presence_eval_count': int(static_eval_count),
        'motion_eval_count': int(motion_eval_count),
        **evaluate_runtime_policy_evaluations(static_presence_motion_states),
    }


def _evaluate_replay_row_idle(center, scale, layers, rows, threshold=0.5):
    """Evaluate quiet replay metrics from canonical time-aware replay rows."""
    probabilities, count = _evaluate_replay_row_probabilities(center, scale, layers, rows)
    raw_states = probabilities > threshold
    fp = int(np.sum(raw_states))
    return {
        'fp': fp,
        'evaluations': int(count),
        'fp_rate': fp / count * 100.0 if count else 0.0,
        **evaluate_runtime_policy_evaluations(raw_states),
    }


def evaluate_split(model, scaler, feature_names, static_presence_packets,
                   motion_packets, threshold=0.5):
    """Evaluate a split through reset-aware production-time replay ticks."""
    center, scale = get_preprocessor_arrays(scaler)
    layers = _layer_arrays_from_model(model)
    if _feature_rows_use_runtime_cache(feature_names):
        static_rows = build_ml_replay_rows(
            static_presence_packets,
            DEFAULT_SUBCARRIERS,
            None,
            feature_names,
            sample_contract="replay_tick",
        )
        motion_rows = build_ml_replay_rows(
            motion_packets,
            DEFAULT_SUBCARRIERS,
            None,
            feature_names,
            sample_contract="replay_tick",
        )
    else:
        static_rows = build_host_feature_rows(
            static_presence_packets,
            feature_names,
            sample_contract="replay_tick",
        )
        motion_rows = build_host_feature_rows(
            motion_packets,
            feature_names,
            sample_contract="replay_tick",
        )
    return _evaluate_replay_row_split(
        center,
        scale,
        layers,
        static_rows,
        motion_rows,
        threshold=threshold,
    )


def evaluate_array_split(center, scale, layers, feature_names,
                         static_presence_packets, motion_packets,
                         threshold=0.5):
    """Evaluate exported arrays through reset-aware production-time replay ticks."""
    if _feature_rows_use_runtime_cache(feature_names):
        static_rows = build_ml_replay_rows(
            static_presence_packets,
            DEFAULT_SUBCARRIERS,
            None,
            feature_names,
            sample_contract="replay_tick",
        )
        motion_rows = build_ml_replay_rows(
            motion_packets,
            DEFAULT_SUBCARRIERS,
            None,
            feature_names,
            sample_contract="replay_tick",
        )
    else:
        static_rows = build_host_feature_rows(
            static_presence_packets,
            feature_names,
            sample_contract="replay_tick",
        )
        motion_rows = build_host_feature_rows(
            motion_packets,
            feature_names,
            sample_contract="replay_tick",
        )
    return _evaluate_replay_row_split(
        center,
        scale,
        layers,
        static_rows,
        motion_rows,
        threshold=threshold,
    )


def _load_gate_feature_rows(path, label_name, feature_names, *,
                            dataset_info=None, file_metadata=None,
                            use_cache=True):
    """Load one replay file through the canonical time-aware row cache."""
    del label_name, dataset_info, file_metadata
    path = Path(path)
    if _feature_rows_use_runtime_cache(feature_names):
        return load_or_compute_ml_replay_rows(
            path,
            selected_subcarriers=DEFAULT_SUBCARRIERS,
            window_size=None,
            feature_names=feature_names,
            use_cache=use_cache,
            sample_contract="replay_tick",
        )
    rows = load_or_compute_host_feature_rows(
        path,
        feature_names=feature_names,
        sample_contract="replay_tick",
        use_cache=use_cache,
        stream_provenance=_host_feature_stream_provenance(
            feature_names,
        ),
    )
    return rows


def _evaluate_cached_feature_stream(center, scale, layers, rows):
    """Return probabilities for one canonical runtime-tick replay stream."""
    X = np.asarray(rows.get('X', np.empty((0, 0), dtype=np.float32)), dtype=np.float32)
    if X.size == 0:
        return np.zeros(0, dtype=np.float64), 0
    probabilities = _batch_predict_probabilities(X, center, scale, layers)
    return probabilities, int(len(X))


def evaluate_cached_feature_split(center, scale, layers,
                                  static_presence_rows, motion_rows, threshold=0.5):
    """Evaluate a paired split from canonical runtime-tick feature rows."""
    return _evaluate_replay_row_split(
        center,
        scale,
        layers,
        static_presence_rows,
        motion_rows,
        threshold=threshold,
    )


def evaluate_cached_idle_stream(center, scale, layers, rows, threshold=0.5):
    """Evaluate one quiet replay from canonical runtime-tick feature rows."""
    return _evaluate_replay_row_idle(
        center,
        scale,
        layers,
        rows,
        threshold=threshold,
    )


def evaluate_cached_array_split(center, scale, layers, feature_names,
                                static_presence_path, motion_path, threshold=0.5,
                                *, dataset_info=None, file_metadata=None,
                                use_cache=True):
    """Evaluate one paired replay directly from cached per-window features."""
    static_presence_rows = _load_gate_feature_rows(
        static_presence_path,
        'static_presence',
        feature_names,
        dataset_info=dataset_info,
        file_metadata=file_metadata,
        use_cache=use_cache,
    )
    motion_rows = _load_gate_feature_rows(
        motion_path,
        'motion',
        feature_names,
        dataset_info=dataset_info,
        file_metadata=file_metadata,
        use_cache=use_cache,
    )
    return evaluate_cached_feature_split(
        center,
        scale,
        layers,
        static_presence_rows,
        motion_rows,
        threshold=threshold,
    )


def evaluate_cached_idle_array(center, scale, layers, feature_names, path,
                               threshold=0.5, *, dataset_info=None,
                               file_metadata=None, use_cache=True):
    """Evaluate one quiet replay directly from cached per-window features."""
    rows = _load_gate_feature_rows(
        path,
        'empty',
        feature_names,
        dataset_info=dataset_info,
        file_metadata=file_metadata,
        use_cache=use_cache,
    )
    return evaluate_cached_idle_stream(
        center,
        scale,
        layers,
        rows,
        threshold=threshold,
    )


def _iter_paired_chip_replays(chips=None, roles=('selection',),
                              allow_legacy_fallback=True):
    """Yield role-isolated real pair replay paths, or one legacy train fallback."""
    dataset_info = load_dataset_info()
    files = dataset_info.get('files', {})
    roles = normalize_dataset_roles(roles, default=('selection',))
    motion_by_name = {
        str(entry.get('filename', '')): entry
        for entry in files.get('motion', [])
    }
    for chip in tuple(chips or DEFAULT_PAIRED_GATE_CHIPS):
        role_pairs = []
        for static_entry in files.get('static_presence', []):
            if str(static_entry.get('chip', '')).upper() != chip:
                continue
            if int(static_entry.get('subcarriers', 0) or 0) != 64:
                continue
            if bool(static_entry.get('synthetic')):
                continue
            motion_name = str(static_entry.get('optimal_pair_motion_file', ''))
            motion_entry = motion_by_name.get(motion_name)
            if motion_entry is None or bool(motion_entry.get('synthetic')):
                continue
            role = paired_dataset_role(
                static_entry,
                motion_entry,
                admitted_roles=roles,
            )
            if role is None:
                continue
            static_path = resolve_entry_path('static_presence', static_entry)
            motion_path = resolve_entry_path('motion', motion_entry)
            if static_path.exists() and motion_path.exists():
                low_rssi = bool(static_entry.get('low_rssi')) or bool(motion_entry.get('low_rssi'))
                role_pairs.append((role, static_path, motion_path, low_rssi))
        if role_pairs:
            for role, static_path, motion_path, low_rssi in sorted(role_pairs):
                key = f"{chip}:{role}:{static_path.name}"
                yield (key, static_path, motion_path, low_rssi)
            continue
        if not allow_legacy_fallback:
            continue

        # When no reserved pair exists, use only an explicitly admitted real
        # train pair. Missing roles default to exclude, and a newly generated
        # synthetic derivative can never become the deployment replay by being
        # the latest timestamped pair.
        legacy_pairs = []
        for static_entry in files.get('static_presence', []):
            if str(static_entry.get('chip', '')).upper() != chip:
                continue
            if int(static_entry.get('subcarriers', 0) or 0) != 64:
                continue
            if bool(static_entry.get('synthetic')):
                continue
            motion_name = str(static_entry.get('optimal_pair_motion_file', ''))
            motion_entry = motion_by_name.get(motion_name)
            if motion_entry is None or bool(motion_entry.get('synthetic')):
                continue
            if paired_dataset_role(
                static_entry,
                motion_entry,
                admitted_roles=('train',),
            ) != 'train':
                continue
            static_path = resolve_entry_path('static_presence', static_entry)
            motion_path = resolve_entry_path('motion', motion_entry)
            if static_path.exists() and motion_path.exists():
                sort_key = (
                    str(static_entry.get('collected_at', '')),
                    static_path.name,
                )
                low_rssi = bool(static_entry.get('low_rssi')) or bool(motion_entry.get('low_rssi'))
                legacy_pairs.append((sort_key, static_path, motion_path, low_rssi))
        if not legacy_pairs:
            continue
        _, static_path, motion_path, low_rssi = max(legacy_pairs, key=lambda row: row[0])
        yield (chip, static_path, motion_path, low_rssi)


def _iter_paired_chip_packets(chips=None, roles=('selection',),
                              allow_legacy_fallback=True):
    """Yield role-isolated real pairs, or the legacy latest pair as fallback."""
    for chip, static_path, motion_path, low_rssi in _iter_paired_chip_replays(
        chips,
        roles=roles,
        allow_legacy_fallback=allow_legacy_fallback,
    ):
        yield (
            chip,
            _load_npz_packets_cached(static_path),
            _load_npz_packets_cached(motion_path),
            low_rssi,
        )


def evaluate_paired_gate(model, scaler, feature_names, threshold=0.5, chips=None,
                         roles=('selection',), allow_legacy_fallback=True,
                         progress=None, use_cached_features=True,
                         use_cache=True):
    """Evaluate a candidate on the paired validation datasets."""
    pairs = list(_iter_paired_chip_replays(
        chips,
        roles=roles,
        allow_legacy_fallback=allow_legacy_fallback,
    ))
    center = scale = layers = dataset_info = file_metadata = None
    if use_cached_features:
        center, scale = get_preprocessor_arrays(scaler)
        layers = _layer_arrays_from_model(model)
        dataset_info = load_dataset_info()
        file_metadata = get_file_metadata(dataset_info)
    if progress is not None:
        progress(
            f"Paired gate: evaluating {len(pairs)} chip replay(s)"
        )
    by_chip = {}
    gate_start = perf_counter()
    for index, (chip, static_path, motion_path, low_rssi) in enumerate(
        pairs,
        start=1,
    ):
        step_start = perf_counter()
        if use_cached_features:
            row = evaluate_cached_array_split(
                center,
                scale,
                layers,
                feature_names,
                static_path,
                motion_path,
                threshold=threshold,
                dataset_info=dataset_info,
                file_metadata=file_metadata,
                use_cache=use_cache,
            )
        else:
            row = evaluate_split(
                model,
                scaler,
                feature_names,
                _load_npz_packets_cached(static_path),
                _load_npz_packets_cached(motion_path),
                threshold=threshold,
            )
        row['low_rssi'] = low_rssi
        by_chip[chip] = row
        if progress is not None:
            progress(
                f"Paired gate {index}/{len(pairs)} {chip}: "
                f"R={row['recall']:.2f}% FP={row['fp_rate']:.2f}% "
                f"alarms={row.get('effective_alarms', 0)} "
                f"in {format_duration(perf_counter() - step_start)}"
            )
    summary = summarize_gate(by_chip)
    if progress is not None and summary is not None:
        progress(
            f"Paired gate complete in {format_duration(perf_counter() - gate_start)}: "
            f"pass={summary['pass_count']} maxFP={summary['max_fp_rate']:.2f}% "
            f"worstRecall={summary['worst_chip_recall']:.2f}% "
            f"alarms={summary.get('total_effective_alarms', 0)}"
        )
    return summary


def _load_exported_model_arrays():
    """Load exported MicroPython weights as inference-ready arrays."""
    module = load_exported_ml_weights()
    center = np.asarray(module.FEATURE_MEAN, dtype=np.float32)
    scale = np.asarray(module.FEATURE_SCALE, dtype=np.float32)
    matrices = exported_weight_matrices(module)
    layers = []
    for idx, (weights, biases) in enumerate(zip(matrices, module.BIASES, strict=True)):
        layers.append((
            weights,
            np.asarray(biases, dtype=np.float32),
            idx == len(matrices) - 1,
        ))
    return list(module.FEATURE_NAMES), center, scale, layers


def _evaluate_exported_paired_gate_at_active_bin(
        threshold=0.5, chips=None, roles=('selection', 'holdout'),
        allow_legacy_fallback=True, use_cached_features=True, use_cache=True):
    """Evaluate exported runtime arrays on the paired validation datasets."""
    feature_names, center, scale, layers = _load_exported_model_arrays()
    by_chip = {}
    dataset_info = file_metadata = None
    if use_cached_features:
        dataset_info = load_dataset_info()
        file_metadata = get_file_metadata(dataset_info)
    for chip, static_path, motion_path, low_rssi in _iter_paired_chip_replays(
        chips,
        roles=roles,
        allow_legacy_fallback=allow_legacy_fallback,
    ):
        if use_cached_features:
            row = evaluate_cached_array_split(
                center,
                scale,
                layers,
                feature_names,
                static_path,
                motion_path,
                threshold=threshold,
                dataset_info=dataset_info,
                file_metadata=file_metadata,
                use_cache=use_cache,
            )
        else:
            row = evaluate_array_split(
                center,
                scale,
                layers,
                feature_names,
                _load_npz_packets_cached(static_path),
                _load_npz_packets_cached(motion_path),
                threshold=threshold,
            )
        row['low_rssi'] = low_rssi
        by_chip[chip] = row
    return summarize_gate(by_chip)


def evaluate_exported_paired_gate(threshold=0.5, chips=None,
                                  roles=('selection', 'holdout'),
                                  allow_legacy_fallback=True,
                                  use_cached_features=True,
                                  use_cache=True):
    """Evaluate exported arrays with their canonical production trajectory bin."""
    with canonical_trajectory_bin():
        return _evaluate_exported_paired_gate_at_active_bin(
            threshold=threshold,
            chips=chips,
            roles=roles,
            allow_legacy_fallback=allow_legacy_fallback,
            use_cached_features=use_cached_features,
            use_cache=use_cache,
        )


def _iter_quiet_gate_replays(roles=('selection', 'holdout')):
    """Yield real empty replay paths explicitly reserved for selection/holdout."""
    dataset_info = load_dataset_info()
    roles = normalize_dataset_roles(roles, default=('selection', 'holdout'))
    for entry in dataset_info.get('files', {}).get('empty', []):
        role = admitted_dataset_role(entry, admitted_roles=roles)
        if role is None or bool(entry.get('synthetic')):
            continue
        path = resolve_entry_path('empty', entry)
        if path.exists():
            chip = str(entry.get('chip', 'unknown')).upper()
            yield f"{chip}:{role}:{path.name}", path


def _iter_quiet_gate_packets(roles=('selection', 'holdout')):
    """Yield real empty recordings explicitly reserved for selection/holdout."""
    for key, path in _iter_quiet_gate_replays(roles=roles):
        yield key, _load_npz_packets_cached(path)


def evaluate_idle_streaming(evaluator, packets, threshold=0.5):
    """Evaluate one quiet stream at production cadence and hit filtering."""
    if not isinstance(evaluator, (ArrayStreamingEvaluator, StreamingEvaluator)):
        raise TypeError("Time-aware idle evaluation requires a streaming ML evaluator")
    feature_names = evaluator.extractor.feature_names
    if _feature_rows_use_runtime_cache(feature_names):
        rows = build_ml_replay_rows(
            packets,
            DEFAULT_SUBCARRIERS,
            None,
            feature_names,
            sample_contract="replay_tick",
        )
    else:
        rows = build_host_feature_rows(
            packets,
            feature_names,
            sample_contract="replay_tick",
        )
    return _evaluate_replay_row_idle(
        evaluator.center,
        evaluator.scale,
        evaluator.layers,
        rows,
        threshold=threshold,
    )


def summarize_quiet_gate(by_dataset):
    """Aggregate explicitly reserved empty-room safety replays."""
    if not by_dataset:
        return None
    rows = list(by_dataset.values())
    return {
        'by_dataset': by_dataset,
        'max_fp_rate': float(max(row['fp_rate'] for row in rows)),
        'total_effective_alarms': int(sum(row['effective_alarms'] for row in rows)),
        'max_effective_alarms': int(max(row['effective_alarms'] for row in rows)),
        'passed': all(
            row['fp_rate'] < DEFAULT_GATE_TARGET_FP_RATE
            and row['effective_alarms'] == 0
            for row in rows
        ),
    }


def evaluate_quiet_gate(model, scaler, feature_names, threshold=0.5,
                        roles=('selection', 'holdout'), progress=None,
                        use_cached_features=True, use_cache=True):
    """Evaluate an in-memory candidate on reserved real empty recordings."""
    datasets = list(_iter_quiet_gate_replays(roles=roles))
    center = scale = layers = dataset_info = file_metadata = None
    if use_cached_features:
        center, scale = get_preprocessor_arrays(scaler)
        layers = _layer_arrays_from_model(model)
        dataset_info = load_dataset_info()
        file_metadata = get_file_metadata(dataset_info)
    if progress is not None:
        progress(
            f"Quiet gate: evaluating {len(datasets)} reserved empty replay(s)"
        )
    by_dataset = {}
    gate_start = perf_counter()
    for index, (key, path) in enumerate(datasets, start=1):
        step_start = perf_counter()
        if use_cached_features:
            by_dataset[key] = evaluate_cached_idle_array(
                center,
                scale,
                layers,
                feature_names,
                path,
                threshold=threshold,
                dataset_info=dataset_info,
                file_metadata=file_metadata,
                use_cache=use_cache,
            )
        else:
            by_dataset[key] = evaluate_idle_streaming(
                StreamingEvaluator(model, scaler, feature_names),
                _load_npz_packets_cached(path),
                threshold=threshold,
            )
        row = by_dataset[key]
        if progress is not None:
            progress(
                f"Quiet gate {index}/{len(datasets)} {key}: "
                f"FP={row['fp_rate']:.2f}% alarms={row['effective_alarms']} "
                f"in {format_duration(perf_counter() - step_start)}"
            )
    summary = summarize_quiet_gate(by_dataset)
    if progress is not None and summary is not None:
        progress(
            f"Quiet gate complete in {format_duration(perf_counter() - gate_start)}: "
            f"{'pass' if summary['passed'] else 'fail'} "
            f"maxFP={summary['max_fp_rate']:.2f}% "
            f"alarms={summary['total_effective_alarms']}"
        )
    return summary


def _evaluate_exported_quiet_gate_at_active_bin(
        threshold=0.5, roles=('selection', 'holdout'),
        use_cached_features=True, use_cache=True):
    """Evaluate exported arrays on reserved real empty recordings."""
    feature_names, center, scale, layers = _load_exported_model_arrays()
    by_dataset = {}
    dataset_info = file_metadata = None
    if use_cached_features:
        dataset_info = load_dataset_info()
        file_metadata = get_file_metadata(dataset_info)
    for key, path in _iter_quiet_gate_replays(roles=roles):
        if use_cached_features:
            by_dataset[key] = evaluate_cached_idle_array(
                center,
                scale,
                layers,
                feature_names,
                path,
                threshold=threshold,
                dataset_info=dataset_info,
                file_metadata=file_metadata,
                use_cache=use_cache,
            )
        else:
            by_dataset[key] = evaluate_idle_streaming(
                ArrayStreamingEvaluator(center, scale, layers, feature_names),
                _load_npz_packets_cached(path),
                threshold=threshold,
            )
    return summarize_quiet_gate(by_dataset)


def evaluate_exported_quiet_gate(threshold=0.5, roles=('selection', 'holdout'),
                                 use_cached_features=True, use_cache=True):
    """Evaluate exported arrays with their canonical production trajectory bin."""
    with canonical_trajectory_bin():
        return _evaluate_exported_quiet_gate_at_active_bin(
            threshold=threshold,
            roles=roles,
            use_cached_features=use_cached_features,
            use_cache=use_cache,
        )


def _occupancy_gate_stream_provenance(dataset_id, *, keep_ratio, seed, phase):
    """Cache identity for one occupancy-thinned reserved replay."""
    return {
        'transform': OCCUPANCY_GATE_TRANSFORM,
        'occupancy_percent': OCCUPANCY_GATE_PERCENT,
        'target_pps': OCCUPANCY_GATE_TARGET_PPS,
        'keep_ratio': float(keep_ratio),
        'seed': int(seed),
        'dataset_id': str(dataset_id),
        'phase': str(phase),
        'implementation_sha256': _implementation_source_digest(
            thin_to_occupancy,
            admit_packets,
            thin_packets,
        ),
    }


def _load_occupancy_gate_feature_rows(
    path,
    feature_names,
    *,
    dataset_id,
    phase,
    offset=0,
    use_cache=True,
):
    """Load one reserved replay after deterministic 70% occupancy thinning."""
    path = Path(path)
    thinned, keep_ratio, seed = thin_to_occupancy(
        _load_npz_packets_cached(path),
        occupancy_percent=OCCUPANCY_GATE_PERCENT,
        dataset_id=str(dataset_id),
        offset=int(offset),
        target_pps=OCCUPANCY_GATE_TARGET_PPS,
    )
    provenance = _occupancy_gate_stream_provenance(
        dataset_id,
        keep_ratio=keep_ratio,
        seed=seed,
        phase=phase,
    )
    if _feature_rows_use_runtime_cache(feature_names):
        return load_or_compute_ml_replay_rows(
            path,
            packets=thinned,
            selected_subcarriers=DEFAULT_SUBCARRIERS,
            window_size=None,
            feature_names=feature_names,
            use_cache=use_cache,
            sample_contract="replay_tick",
            stream_provenance=provenance,
        )
    host_provenance = _host_feature_stream_provenance(feature_names)
    host_provenance['occupancy_gate'] = provenance
    return load_or_compute_host_feature_rows(
        path,
        packets=list(thinned),
        feature_names=feature_names,
        sample_contract="replay_tick",
        use_cache=use_cache,
        stream_provenance=host_provenance,
    )


def evaluate_occupancy_paired_gate(
    model,
    scaler,
    feature_names,
    threshold=0.5,
    chips=None,
    roles=('selection',),
    allow_legacy_fallback=True,
    progress=None,
    use_cache=True,
):
    """Evaluate a candidate on reserved pairs thinned to the 70% occupancy envelope."""
    pairs = list(_iter_paired_chip_replays(
        chips,
        roles=roles,
        allow_legacy_fallback=allow_legacy_fallback,
    ))
    center, scale = get_preprocessor_arrays(scaler)
    layers = _layer_arrays_from_model(model)
    if progress is not None:
        progress(
            f"Occupancy {OCCUPANCY_GATE_PERCENT}% paired gate: "
            f"evaluating {len(pairs)} chip replay(s)"
        )
    by_chip = {}
    gate_start = perf_counter()
    for index, (chip, static_path, motion_path, low_rssi) in enumerate(
        pairs,
        start=1,
    ):
        step_start = perf_counter()
        dataset_id = Path(static_path).name
        static_rows = _load_occupancy_gate_feature_rows(
            static_path,
            feature_names,
            dataset_id=dataset_id,
            phase="static_presence",
            offset=0,
            use_cache=use_cache,
        )
        motion_rows = _load_occupancy_gate_feature_rows(
            motion_path,
            feature_names,
            dataset_id=dataset_id,
            phase="motion",
            offset=1,
            use_cache=use_cache,
        )
        row = evaluate_cached_feature_split(
            center,
            scale,
            layers,
            static_rows,
            motion_rows,
            threshold=threshold,
        )
        row['low_rssi'] = low_rssi
        by_chip[chip] = row
        if progress is not None:
            progress(
                f"Occupancy paired gate {index}/{len(pairs)} {chip}: "
                f"R={row['recall']:.2f}% FP={row['fp_rate']:.2f}% "
                f"alarms={row.get('effective_alarms', 0)} "
                f"in {format_duration(perf_counter() - step_start)}"
            )
    summary = summarize_gate(by_chip)
    if progress is not None and summary is not None:
        progress(
            f"Occupancy paired gate complete in "
            f"{format_duration(perf_counter() - gate_start)}: "
            f"pass={summary['pass_count']} maxFP={summary['max_fp_rate']:.2f}% "
            f"worstRecall={summary['worst_chip_recall']:.2f}% "
            f"alarms={summary.get('total_effective_alarms', 0)}"
        )
    return summary


def evaluate_occupancy_quiet_gate(
    model,
    scaler,
    feature_names,
    threshold=0.5,
    roles=('selection', 'holdout'),
    progress=None,
    use_cache=True,
):
    """Evaluate reserved empty replays thinned to the 70% occupancy envelope."""
    datasets = list(_iter_quiet_gate_replays(roles=roles))
    center, scale = get_preprocessor_arrays(scaler)
    layers = _layer_arrays_from_model(model)
    if progress is not None:
        progress(
            f"Occupancy {OCCUPANCY_GATE_PERCENT}% quiet gate: "
            f"evaluating {len(datasets)} reserved empty replay(s)"
        )
    by_dataset = {}
    gate_start = perf_counter()
    for index, (key, path) in enumerate(datasets, start=1):
        step_start = perf_counter()
        rows = _load_occupancy_gate_feature_rows(
            path,
            feature_names,
            dataset_id=Path(path).name,
            phase="empty",
            offset=0,
            use_cache=use_cache,
        )
        by_dataset[key] = evaluate_cached_idle_stream(
            center,
            scale,
            layers,
            rows,
            threshold=threshold,
        )
        if progress is not None:
            row = by_dataset[key]
            progress(
                f"Occupancy quiet gate {index}/{len(datasets)} {key}: "
                f"FP={row['fp_rate']:.2f}% alarms={row['effective_alarms']} "
                f"in {format_duration(perf_counter() - step_start)}"
            )
    summary = summarize_quiet_gate(by_dataset)
    if progress is not None and summary is not None:
        progress(
            f"Occupancy quiet gate complete in "
            f"{format_duration(perf_counter() - gate_start)}: "
            f"{'pass' if summary['passed'] else 'fail'} "
            f"maxFP={summary['max_fp_rate']:.2f}% "
            f"alarms={summary['total_effective_alarms']}"
        )
    return summary


def evaluate_exported_occupancy_paired_gate(
    threshold=0.5,
    chips=None,
    roles=('selection', 'holdout'),
    allow_legacy_fallback=True,
    use_cache=True,
):
    """Evaluate exported arrays on occupancy-thinned reserved pairs."""
    with canonical_trajectory_bin():
        feature_names, center, scale, layers = _load_exported_model_arrays()
        pairs = list(_iter_paired_chip_replays(
            chips,
            roles=roles,
            allow_legacy_fallback=allow_legacy_fallback,
        ))
        by_chip = {}
        for chip, static_path, motion_path, low_rssi in pairs:
            dataset_id = Path(static_path).name
            static_rows = _load_occupancy_gate_feature_rows(
                static_path,
                feature_names,
                dataset_id=dataset_id,
                phase="static_presence",
                offset=0,
                use_cache=use_cache,
            )
            motion_rows = _load_occupancy_gate_feature_rows(
                motion_path,
                feature_names,
                dataset_id=dataset_id,
                phase="motion",
                offset=1,
                use_cache=use_cache,
            )
            row = evaluate_cached_feature_split(
                center,
                scale,
                layers,
                static_rows,
                motion_rows,
                threshold=threshold,
            )
            row['low_rssi'] = low_rssi
            by_chip[chip] = row
        return summarize_gate(by_chip)


def evaluate_exported_occupancy_quiet_gate(
    threshold=0.5,
    roles=('selection', 'holdout'),
    use_cache=True,
):
    """Evaluate exported arrays on occupancy-thinned reserved empty replays."""
    with canonical_trajectory_bin():
        feature_names, center, scale, layers = _load_exported_model_arrays()
        by_dataset = {}
        for key, path in _iter_quiet_gate_replays(roles=roles):
            rows = _load_occupancy_gate_feature_rows(
                path,
                feature_names,
                dataset_id=Path(path).name,
                phase="empty",
                offset=0,
                use_cache=use_cache,
            )
            by_dataset[key] = evaluate_cached_idle_stream(
                center,
                scale,
                layers,
                rows,
                threshold=threshold,
            )
        return summarize_quiet_gate(by_dataset)


@dataclass
class ExportedMLGateResult:
    """Verification result for exported ML artifacts (paired gate)."""

    paired_returncode: int
    paired_output: str
    paired_metrics: dict | None = None
    quiet_metrics: dict | None = None
    occupancy_paired_metrics: dict | None = None
    occupancy_quiet_metrics: dict | None = None

    @property
    def available(self):
        return self.paired_metrics is not None or self.quiet_metrics is not None

    @property
    def occupancy_passed(self):
        occupancy_paired = self.occupancy_paired_metrics
        occupancy_quiet = self.occupancy_quiet_metrics
        occupancy_total = len(occupancy_paired.get('by_chip', {})) if occupancy_paired else 0
        return bool(
            occupancy_total
            and occupancy_paired.get('pass_count', 0) == occupancy_total
            and occupancy_quiet is not None
            and occupancy_quiet.get('passed', False)
        )

    @property
    def passed(self):
        return (
            self.available
            and (self.paired_metrics is None or self.paired_returncode == 0)
            and (self.quiet_metrics is None or self.quiet_metrics.get('passed', False))
            and self.occupancy_passed
        )


def run_exported_ml_gates(roles=('selection', 'holdout'),
                          allow_legacy_fallback=True):
    """Run paired, quiet, and occupancy-70% gates for exported artifacts."""
    paired_metrics = evaluate_exported_paired_gate(
        roles=roles,
        allow_legacy_fallback=allow_legacy_fallback,
    )
    quiet_metrics = evaluate_exported_quiet_gate(roles=roles)
    occupancy_paired = evaluate_exported_occupancy_paired_gate(
        roles=roles,
        allow_legacy_fallback=allow_legacy_fallback,
    )
    occupancy_quiet = evaluate_exported_occupancy_quiet_gate(roles=roles)
    paired_total = len(paired_metrics.get('by_chip', {})) if paired_metrics else 0
    paired_rc = 0 if paired_total and paired_metrics['pass_count'] == paired_total else 1
    return ExportedMLGateResult(
        paired_returncode=paired_rc,
        paired_output="",
        paired_metrics=paired_metrics,
        quiet_metrics=quiet_metrics,
        occupancy_paired_metrics=occupancy_paired,
        occupancy_quiet_metrics=occupancy_quiet,
    )


def in_memory_gate_result(training_metrics):
    """Adapt one in-memory training result to the shared gate summary type."""
    paired_metrics = training_metrics.get('paired') if training_metrics else None
    quiet_metrics = training_metrics.get('quiet') if training_metrics else None
    occupancy_paired = (
        training_metrics.get('occupancy_paired') if training_metrics else None
    )
    occupancy_quiet = (
        training_metrics.get('occupancy_quiet') if training_metrics else None
    )
    paired_total = len(paired_metrics.get('by_chip', {})) if paired_metrics else 0
    paired_rc = (
        0
        if paired_total and paired_metrics.get('pass_count', 0) == paired_total
        else 1
    )
    return ExportedMLGateResult(
        paired_returncode=paired_rc,
        paired_output="",
        paired_metrics=paired_metrics,
        quiet_metrics=quiet_metrics,
        occupancy_paired_metrics=occupancy_paired,
        occupancy_quiet_metrics=occupancy_quiet,
    )
