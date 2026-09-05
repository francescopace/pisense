#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Detection Methods Comparison
Compares RSSI, Lightweight, and High Accuracy algorithms

Usage:
    python tools/compare_detection_methods.py              # Use C6 dataset
    python tools/compare_detection_methods.py --chip S3    # Use S3 dataset
    python tools/compare_detection_methods.py --plot       # Show visualization

Author: Francesco Pace <francesco.pace@gmail.com>
"""

import numpy as np
import matplotlib.pyplot as plt
import argparse
import time
import re
import sys
from collections.abc import Mapping
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.lib.bootstrap import setup_paths  # noqa: F401

from tools.lib.csi_io import load_static_presence_and_motion
from tools.lib.dataset_metadata import (
    estimate_runtime_threshold,
    detector_window_packets,
    load_dataset_info,
    measure_packet_interval_us,
    resolve_explicit_pair,
    select_dataset_interactively,
)
from tools.lib.performance_report import (
    get_available_long_test_dataset_specs,
    load_long_test_dataset,
    timing_cadence_for_window,
)
from tools.lib.temporal_replay import (
    TemporalReplayController,
    apply_temporal_admission,
    target_pps_for_packets,
)
from tools.lib.temporal_csi_sampler import minimum_valid_slots
from tools.lib.ui import show_plot_window
from config import (
    SEGMENTATION_WINDOW_SIZE_MS,
    ENABLE_HAMPEL_FILTER, HAMPEL_WINDOW, HAMPEL_THRESHOLD,
    ENABLE_LOWPASS_FILTER, LOWPASS_CUTOFF,
    DEFAULT_SUBCARRIERS,
)
from tools.lib.filters import HampelFilter, LowPassFilter
from threshold import DEFAULT_ADAPTIVE_FACTOR, calculate_startup_threshold_from_max
from tools.lib.lightweight_detector import LightweightDetector
from tools.lib.csi_features import L1_DELTA_STARTUP_THRESHOLD_FACTOR

# Check if ML model is available (production implementation).
ML_AVAILABLE = False
try:
    from tools.lib.high_accuracy_detector import HighAccuracyDetector as ProdHighAccuracyDetector, HIGH_ACCURACY_DEFAULT_THRESHOLD
    ML_AVAILABLE = True
except ImportError:
    ProdHighAccuracyDetector = None
    HIGH_ACCURACY_DEFAULT_THRESHOLD = None

# Configuration
WINDOW_SIZE_MS = SEGMENTATION_WINDOW_SIZE_MS
THRESHOLD = 1.0


def _packet_csi_data(packet):
    """Return CSI data from a packet mapping or a compact matrix row."""
    return packet["csi_data"] if isinstance(packet, Mapping) else packet


def _extract_motion_start_from_description(description):
    """Extract motion start packet index from free-text description."""
    if not description:
        return None
    match = re.search(
        r'motion\s+starts\s+at\s+packet(?:\s+index)?(?:\s+n\.)?\s+(\d+)',
        description,
        re.IGNORECASE
    )
    if match:
        return int(match.group(1))
    return None


def load_test_dataset(chip=None, motion_start_packet=None):
    """
    Load the latest test dataset for a chip and split it into static presence and motion.

    Split logic:
    - Use --test-motion-start-packet when provided
    - Else parse packet index from test description in dataset_info.json
    - Else use the full stream as quiet baseline
    """
    specs = get_available_long_test_dataset_specs(
        chips=[chip] if chip else None
    )
    if not specs:
        suffix = f" for chip {chip.upper()}" if chip else ""
        raise FileNotFoundError(
            f"No long-recording dataset found{suffix} in dataset_info.json"
        )

    selected = sorted(
        specs,
        key=lambda item: (
            str(item[4].get('collected_at', '')),
            str(item[4].get('filename', '')),
        ),
    )[-1]
    if motion_start_packet is not None:
        test_path, _old_start, num_packets, selected_chip, entry = selected
        selected = (test_path, motion_start_packet, num_packets, selected_chip, entry)
    return load_long_test_dataset(selected)


def resolve_context_aware_config_for_test(test_entry, static_presence_packets):
    """Resolve the threshold for a test split from its quiet prefix."""
    threshold = estimate_runtime_threshold(
        static_presence_packets,
        selected_subcarriers=tuple(DEFAULT_SUBCARRIERS),
    )
    if threshold is None:
        threshold = THRESHOLD
        context_source = 'test fallback threshold'
        confidence_factor = 0.5
    else:
        context_source = 'test runtime l1_delta calibration'
        confidence_factor = 1.0
    return {
        'threshold': threshold,
        'context_source': context_source,
        'confidence_factor': confidence_factor,
    }


def resolve_context_aware_config(pair, static_presence_packets):
    """Resolve the threshold from the selected pair static capture."""
    threshold = estimate_runtime_threshold(
        static_presence_packets,
        selected_subcarriers=tuple(DEFAULT_SUBCARRIERS),
    )
    if threshold is None:
        threshold = THRESHOLD
        context_source = 'explicit-pair fallback threshold'
        confidence_factor = 0.5
    else:
        context_source = 'explicit-pair runtime l1_delta calibration'
        confidence_factor = 1.0
    return {
        'threshold': threshold,
        'context_source': context_source,
        'confidence_factor': confidence_factor,
    }


def calculate_rssi(csi_packet):
    """Calculate RSSI (mean of all subcarrier amplitudes)"""
    amplitudes = []
    for sc_idx in range(64):
        Q = float(csi_packet[sc_idx * 2])
        I = float(csi_packet[sc_idx * 2 + 1])
        amplitudes.append(np.sqrt(I*I + Q*Q))
    return np.mean(amplitudes)


def calculate_adaptive_threshold(values, auto_factor=DEFAULT_ADAPTIVE_FACTOR):
    """Calculate threshold with the shared startup-threshold policy."""
    if len(values) == 0:
        return 1.0
    max_value = float(np.max(np.asarray(values, dtype=float)))
    threshold, _formula = calculate_startup_threshold_from_max(
        max_value, auto_factor=auto_factor
    )
    return float(threshold)


def apply_config_filters(series):
    """Apply Hampel -> low-pass filter chain from config to a 1D series."""
    filtered = []
    hampel = HampelFilter(window_size=HAMPEL_WINDOW, threshold=HAMPEL_THRESHOLD) if ENABLE_HAMPEL_FILTER else None
    lowpass = LowPassFilter(cutoff_hz=LOWPASS_CUTOFF, sample_rate_hz=100.0, enabled=True) if ENABLE_LOWPASS_FILTER else None
    for value in series:
        out = float(value)
        if hampel is not None:
            out = hampel.filter(out)
        if lowpass is not None:
            out = lowpass.filter(out)
        filtered.append(out)
    return np.array(filtered, dtype=float)


def compute_method_results(methods, method_thresholds):
    """Compute FP/TP/FN/Recall/Precision/F1 for every method."""
    results = []
    for method_name, method_data in methods.items():
        static_presence_data = method_data['static_presence']
        motion_data = method_data['motion']
        threshold = method_thresholds[method_name]
        fp = int(np.sum(static_presence_data > threshold))
        tp = int(np.sum(motion_data > threshold))
        fn = int(len(motion_data) - tp)
        recall = (tp / (tp + fn) * 100) if (tp + fn) > 0 else 0.0
        precision = (tp / (tp + fp) * 100) if (tp + fp) > 0 else 0.0
        f1 = (2 * precision * recall / (precision + recall)) if (precision + recall) > 0 else 0.0
        results.append({
            'name': method_name,
            'fp': fp,
            'tp': tp,
            'fn': fn,
            'recall': recall,
            'precision': precision,
            'f1': f1,
        })
    return results


class LightweightDetectorAdapter:
    """Compatibility wrapper around the production LightweightDetector."""

    def __init__(
        self,
        packets,
        window_size_ms=SEGMENTATION_WINDOW_SIZE_MS,
        threshold=1.0,
        track_data=False,
    ):
        window_size = detector_window_packets(packets, window_size_ms)
        self._detector = LightweightDetector(
            window_size=window_size,
            threshold=threshold,
            enable_lowpass=ENABLE_LOWPASS_FILTER,
            lowpass_cutoff=LOWPASS_CUTOFF,
            enable_hampel=ENABLE_HAMPEL_FILTER,
            hampel_window=HAMPEL_WINDOW,
            hampel_threshold=HAMPEL_THRESHOLD,
        )
        self._detector.set_minimum_valid_samples(minimum_valid_slots(window_size))
        self._track_data = bool(track_data)
        self._interval_us = measure_packet_interval_us(packets)
        _, self._cadence = timing_cadence_for_window(
            window_size,
            self._interval_us,
        )
        self._temporal = TemporalReplayController(
            target_pps_for_packets(packets, self._interval_us),
            window_size_ms,
            self._interval_us,
        )
        self.metric_history = []
        self.state_history = []

    def process_packet(self, packet):
        admission = self._temporal.admit(packet)
        if admission is None:
            return
        self._consume_admission(admission)

    def _consume_admission(self, admission):
        """Process the packet selected by the production temporal sampler."""
        if admission.reset_required:
            self._cadence.reset()
        apply_temporal_admission(self._detector, admission)
        self._cadence.note_packet(elapsed_us=admission.coverage_us)
        should_evaluate = self._cadence.should_evaluate()
        if should_evaluate:
            self._cadence.after_evaluation()
        csi_data = _packet_csi_data(admission.packet)
        self._detector.process_packet(csi_data, DEFAULT_SUBCARRIERS)
        if not should_evaluate:
            return
        state = self._detector.update_state()
        if self._track_data:
            self.metric_history.append(float(state.get('motion_metric', 0.0)))
            self.state_history.append(str(state.get('state', 'IDLE')).upper())

    def finish(self):
        """Flush the final selected temporal slot."""
        admission = self._temporal.finish()
        if admission is not None:
            self._consume_admission(admission)

    def reset(self):
        self._detector.reset()
        self._cadence.reset()
        self._temporal.reset()
        self.metric_history = []
        self.state_history = []


class HighAccuracyDetectorAdapter:
    """Compatibility wrapper around production HighAccuracyDetector."""

    def __init__(
        self,
        packets,
        window_size_ms=SEGMENTATION_WINDOW_SIZE_MS,
        track_data=False,
    ):
        window_size = detector_window_packets(packets, window_size_ms)
        self._detector = ProdHighAccuracyDetector(
            window_size=window_size,
            threshold=HIGH_ACCURACY_DEFAULT_THRESHOLD,
            enable_lowpass=ENABLE_LOWPASS_FILTER,
            lowpass_cutoff=LOWPASS_CUTOFF,
            enable_hampel=ENABLE_HAMPEL_FILTER,
            hampel_window=HAMPEL_WINDOW,
            hampel_threshold=HAMPEL_THRESHOLD,
        )
        self._detector.set_minimum_valid_samples(minimum_valid_slots(window_size))
        self._detector.track_data = track_data
        self._interval_us = measure_packet_interval_us(packets)
        _, self._cadence = timing_cadence_for_window(
            window_size,
            self._interval_us,
        )
        self._temporal = TemporalReplayController(
            target_pps_for_packets(packets, self._interval_us),
            window_size_ms,
            self._interval_us,
        )
        self.probability_history = self._detector.probability_history
        self.state_history = self._detector.state_history

    def process_packet(self, packet):
        admission = self._temporal.admit(packet)
        if admission is None:
            return
        self._consume_admission(admission)

    def _consume_admission(self, admission):
        """Process the packet selected by the production temporal sampler."""
        if admission.reset_required:
            self._cadence.reset()
        apply_temporal_admission(self._detector, admission)
        self._cadence.note_packet(elapsed_us=admission.coverage_us)
        should_evaluate = self._cadence.should_evaluate()
        if should_evaluate:
            self._cadence.after_evaluation()
        csi_data = _packet_csi_data(admission.packet)
        self._detector.process_packet(csi_data, DEFAULT_SUBCARRIERS)
        if not should_evaluate:
            return
        self._detector.update_state()
        self.probability_history = self._detector.probability_history
        self.state_history = self._detector.state_history

    def finish(self):
        """Flush the final selected temporal slot."""
        admission = self._temporal.finish()
        if admission is not None:
            self._consume_admission(admission)

    def get_motion_count(self):
        return self._detector.get_motion_count()

    def reset(self):
        self._detector.reset()
        self._cadence.reset()
        self._temporal.reset()
        self.probability_history = self._detector.probability_history
        self.state_history = self._detector.state_history


def compare_detection_methods(
    static_presence_packets,
    motion_packets,
    window_size,
    threshold,
):
    """
    Compare different detection methods on same data.

    ``threshold`` is the production-aligned Lightweight startup threshold
    calibrated from the selected static capture. RSSI calibrates from the same
    static capture using its own adaptive-threshold path.
    Returns metrics for each method.
    """
    methods = {
        'RSSI': {'static_presence': [], 'motion': []},
        'Lightweight': {'static_presence': [], 'motion': []},
    }
    
    if ML_AVAILABLE:
        methods['High Accuracy'] = {'static_presence': [], 'motion': []}
    
    timing = {}
    all_packets = list(static_presence_packets) + list(motion_packets)
    num_packets = len(all_packets)
    
    # Process static presence - simple metrics
    for pkt in static_presence_packets:
        methods['RSSI']['static_presence'].append(calculate_rssi(pkt['csi_data']))
    
    methods['RSSI']['static_presence'] = np.array(methods['RSSI']['static_presence'])
    
    # Process motion - simple metrics
    for pkt in motion_packets:
        methods['RSSI']['motion'].append(calculate_rssi(pkt['csi_data']))
    
    methods['RSSI']['motion'] = np.array(methods['RSSI']['motion'])

    # Lightweight static presence and motion (production detector)
    start = time.perf_counter()
    classic_baseline = LightweightDetectorAdapter(
        static_presence_packets, window_size, threshold, track_data=True
    )
    for pkt in static_presence_packets:
        classic_baseline.process_packet(pkt)
    classic_baseline.finish()
    methods['Lightweight']['static_presence'] = np.array(classic_baseline.metric_history)
    classic_movement = LightweightDetectorAdapter(
        motion_packets, window_size, threshold, track_data=True
    )
    for pkt in motion_packets:
        classic_movement.process_packet(pkt)
    classic_movement.finish()
    classic_time = time.perf_counter() - start
    timing['Lightweight'] = (classic_time / num_packets) * 1e6
    methods['Lightweight']['motion'] = np.array(classic_movement.metric_history)
    
    # Apply runtime filter chain to simple methods for fair comparison.
    for method_name in ('RSSI',):
        methods[method_name]['static_presence'] = apply_config_filters(methods[method_name]['static_presence'])
        methods[method_name]['motion'] = apply_config_filters(methods[method_name]['motion'])
    
    # Time simple methods
    start = time.perf_counter()
    for pkt in all_packets:
        calculate_rssi(pkt['csi_data'])
    timing['RSSI'] = ((time.perf_counter() - start) / num_packets) * 1e6
    
    # High-Accuracy detector (if available)
    ml_baseline = None
    ml_movement = None
    
    if ML_AVAILABLE:
        start = time.perf_counter()
        ml_baseline = HighAccuracyDetectorAdapter(
            static_presence_packets, window_size, track_data=True
        )
        for pkt in static_presence_packets:
            ml_baseline.process_packet(pkt)
        ml_baseline.finish()
        methods['High Accuracy']['static_presence'] = np.array(ml_baseline.probability_history)
        ml_movement = HighAccuracyDetectorAdapter(
            motion_packets, window_size, track_data=True
        )
        for pkt in motion_packets:
            ml_movement.process_packet(pkt)
        ml_movement.finish()
        methods['High Accuracy']['motion'] = np.array(ml_movement.probability_history)
        
        ml_time = time.perf_counter() - start
        timing['High Accuracy'] = (ml_time / num_packets) * 1e6

    # Method-specific thresholds.
    method_thresholds = {
        'RSSI': calculate_adaptive_threshold(methods['RSSI']['static_presence']),
        'Lightweight': float(threshold) if threshold > 0 else calculate_adaptive_threshold(
            methods['Lightweight']['static_presence'],
            auto_factor=L1_DELTA_STARTUP_THRESHOLD_FACTOR,
        ),
    }
    if ML_AVAILABLE and 'High Accuracy' in methods:
        method_thresholds['High Accuracy'] = HIGH_ACCURACY_DEFAULT_THRESHOLD

    results = compute_method_results(methods, method_thresholds)

    return methods, classic_baseline, classic_movement, timing, ml_baseline, ml_movement, method_thresholds, results


def plot_comparison(methods, classic_baseline, classic_movement,
                   threshold, timing,
                   ml_baseline=None, ml_movement=None,
                   method_thresholds=None, results=None):
    """Plot comparison of detection methods"""
    # Determine number of rows based on available methods
    method_names = ['RSSI', 'Lightweight']
    if ML_AVAILABLE and 'High Accuracy' in methods:
        method_names.append('High Accuracy')
    
    method_thresholds = method_thresholds or {}
    results = results or []
    result_by_name = {r['name']: r for r in results}
    best_method = max(results, key=lambda r: r['f1'])['name'] if results else method_names[0]
    
    n_rows = len(method_names)
    fig, axes = plt.subplots(n_rows, 2, figsize=(20, 2.5 * n_rows))
    fig.suptitle('Detection Methods Comparison', fontsize=14, fontweight='bold')
    
    # Maximize window
    try:
        mng = plt.get_current_fig_manager()
        if hasattr(mng, 'window'):
            if hasattr(mng.window, 'showMaximized'):
                mng.window.showMaximized()
            elif hasattr(mng.window, 'state'):
                mng.window.state('zoomed')
    except Exception:
        pass
    
    for row, method_name in enumerate(method_names):
        static_presence_data = methods[method_name]['static_presence']
        motion_data = methods[method_name]['motion']
        
        # For ML, pad warmup region with NaN so X-axis aligns with other methods.
        # Production ML emits probabilities only after the buffer is ready.
        static_presence_plot_data = static_presence_data
        motion_plot_data = motion_data
        ml_static_presence_offset = 0
        ml_motion_offset = 0
        if method_name == 'High Accuracy' and ml_baseline is not None and ml_movement is not None:
            full_static_presence_len = len(methods['Lightweight']['static_presence'])
            full_motion_len = len(methods['Lightweight']['motion'])
            ml_static_presence_offset = max(0, full_static_presence_len - len(static_presence_data))
            ml_motion_offset = max(0, full_motion_len - len(motion_data))
            static_presence_plot_data = np.concatenate([np.full(ml_static_presence_offset, np.nan), static_presence_data])
            motion_plot_data = np.concatenate([np.full(ml_motion_offset, np.nan), motion_data])
        
        method_threshold = method_thresholds.get(method_name, threshold)
        
        time_baseline = np.arange(len(static_presence_plot_data)) / 100.0
        time_movement = np.arange(len(motion_plot_data)) / 100.0
        
        # Colors
        if method_name == 'Lightweight':
            color, linewidth, linestyle = 'orange', 1.8, '-'
        elif method_name == 'High Accuracy':
            color, linewidth, linestyle = 'blue', 1.5, '--'
        else:
            color, linewidth, linestyle = 'green', 1.0, '-'
        
        # LEFT: Baseline
        ax_baseline = axes[row, 0]
        ax_baseline.plot(time_baseline, static_presence_plot_data, color=color, alpha=0.7, 
                        linewidth=linewidth, linestyle=linestyle, label=method_name)
        ax_baseline.axhline(y=method_threshold, color='r', linestyle='--',
                          linewidth=2, label=f'Threshold={method_threshold:.4f}')
        
        # Highlight false positives
        fp = result_by_name.get(method_name, {}).get('fp', 0)
        for i, val in enumerate(static_presence_data):
            if val > method_threshold:
                start_t = (i + ml_static_presence_offset) / 100.0 if method_name == 'High Accuracy' else i / 100.0
                ax_baseline.axvspan(start_t, start_t + 1/100.0, alpha=0.3, color='red')
        
        # Title
        title_prefix = '[BEST] ' if method_name == best_method else ''
        time_us = timing.get(method_name, 0)
        time_info = f"{time_us:.0f}us/pkt" if time_us > 0 else ""
        ax_baseline.set_title(f'{title_prefix}{method_name} - Static Presence (FP={fp}) [{time_info}]',
                            fontsize=11, fontweight='bold')
        ax_baseline.set_ylabel('Value', fontsize=10)
        ax_baseline.grid(True, alpha=0.3)
        ax_baseline.legend(fontsize=9)
        
        # Border for production detectors
        if method_name in ('Lightweight', 'High Accuracy'):
            for spine in ax_baseline.spines.values():
                spine.set_edgecolor('green')
                spine.set_linewidth(3)
        
        if row == n_rows - 1:
            ax_baseline.set_xlabel('Time (seconds)', fontsize=10)
        
        # RIGHT: Movement
        ax_movement = axes[row, 1]
        ax_movement.plot(time_movement, motion_plot_data, color=color, alpha=0.7, 
                        linewidth=linewidth, linestyle=linestyle, label=method_name)
        ax_movement.axhline(y=method_threshold, color='r', linestyle='--',
                          linewidth=2, label=f'Threshold={method_threshold:.4f}')
        
        # Highlight detections
        tp = result_by_name.get(method_name, {}).get('tp', 0)
        fn = result_by_name.get(method_name, {}).get('fn', len(motion_data))
        for i, val in enumerate(motion_data):
            start_t = (i + ml_motion_offset) / 100.0 if method_name == 'High Accuracy' else i / 100.0
            if val > method_threshold:
                ax_movement.axvspan(start_t, start_t + 1/100.0, alpha=0.3, color='green')
            else:
                ax_movement.axvspan(start_t, start_t + 1/100.0, alpha=0.2, color='red')

        recall = (tp / (tp + fn) * 100) if (tp + fn) > 0 else 0.0
        precision = (tp / (tp + fp) * 100) if (tp + fp) > 0 else 0.0
        
        ax_movement.set_title(f'{title_prefix}{method_name} - Motion (TP={tp}, R={recall:.0f}%, P={precision:.0f}%)',
                            fontsize=11, fontweight='bold')
        ax_movement.set_ylabel('Value', fontsize=10)
        ax_movement.grid(True, alpha=0.3)
        ax_movement.legend(fontsize=9)
        
        if method_name in ('Lightweight', 'High Accuracy'):
            for spine in ax_movement.spines.values():
                spine.set_edgecolor('green')
                spine.set_linewidth(3)
        
        if row == n_rows - 1:
            ax_movement.set_xlabel('Time (seconds)', fontsize=10)
    
    plt.tight_layout()
    show_plot_window(plt)


def print_comparison_summary(methods, classic_baseline, classic_movement,
                           threshold, timing,
                           ml_baseline=None, ml_movement=None,
                           method_thresholds=None, results=None):
    """Print comparison summary"""
    print("\n" + "="*80)
    print("  DETECTION METHODS COMPARISON SUMMARY")
    print("="*80 + "\n")
    
    print("Configuration:")
    print(f"  Fixed subcarriers: {list(DEFAULT_SUBCARRIERS)}")
    print(f"  Window: {WINDOW_SIZE_MS} ms")
    print(f"  Lightweight runtime threshold: {threshold}")
    if method_thresholds:
        print("  Method thresholds:")
        for method_name in ['RSSI', 'Lightweight']:
            if method_name in method_thresholds:
                print(f"    - {method_name}: {method_thresholds[method_name]:.4f}")
        if 'High Accuracy' in method_thresholds:
            print(f"    - High Accuracy: {method_thresholds['High Accuracy']:.4f} (fixed)")
    if ML_AVAILABLE:
        print("  ML Model: Neural Network (9→32→16→1)")
    print()
    
    results = results or compute_method_results(methods, method_thresholds or {})
    
    best_by_f1 = max(results, key=lambda r: r['f1'])
    
    print(f"{'Method':<15} {'FP':<8} {'TP':<8} {'FN':<8} {'Recall':<10} {'Precision':<12} {'F1':<10} {'Time':<10}")
    print("-" * 90)
    
    for r in results:
        marker = " *" if r['name'] == best_by_f1['name'] else "  "
        time_us = timing.get(r['name'], 0)
        time_str = f"{time_us:.0f}us" if time_us > 0 else "-"
        print(f"{marker} {r['name']:<13} {r['fp']:<8} {r['tp']:<8} {r['fn']:<8} "
              f"{r['recall']:<10.1f} {r['precision']:<12.1f} {r['f1']:<10.1f} {time_str:<10}")
    
    print("-" * 80)
    print(f"\n* Best method by F1 Score: {best_by_f1['name']}")
    print(f"   - F1: {best_by_f1['f1']:.1f}%")
    print(f"   - Recall: {best_by_f1['recall']:.1f}%")
    print(f"   - Precision: {best_by_f1['precision']:.1f}%")
    
    # Production detector comparison (Lightweight vs High Accuracy)
    prod_results = [r for r in results if r['name'] in ('Lightweight', 'High Accuracy')]

    print("\n" + "-"*80)
    if len(prod_results) > 1:
        print("  Production Detector Comparison")
        print("-"*80)
        print("  " + f"{'Metric':<15}"
              + "".join(f"{r['name']:<12}" for r in prod_results)
              + "Winner")
        print(f"  {'-'*60}")

        metric_rows = [
            ('Recall', lambda r: r['recall'], False),
            ('Precision', lambda r: r['precision'], False),
            ('F1 Score', lambda r: r['f1'], False),
            ('False Pos.', lambda r: float(r['fp']), True),
        ]

        wins = {r['name']: 0 for r in prod_results}
        for metric_name, getter, lower_is_better in metric_rows:
            values = [getter(r) for r in prod_results]
            best_value = min(values) if lower_is_better else max(values)
            winners = [r['name'] for r, v in zip(prod_results, values, strict=True) if v == best_value]
            winner = winners[0] if len(winners) == 1 else 'Tie'
            if winner != 'Tie':
                wins[winner] += 1
            print("  " + f"{metric_name:<15}"
                  + "".join(f"{value:<12.1f}" for value in values)
                  + winner)

        overall = ", ".join(f"{name} {count}/4" for name, count in wins.items())
        print(f"\n  Overall wins: {overall}\n")


def run_all_chips():
    """Run comparison on all available chips and print summary table."""
    dataset_info = load_dataset_info()
    chips = sorted({
        str(entry.get('chip', '')).upper()
        for entry in dataset_info.get('files', {}).get('static_presence', [])
        if entry.get('optimal_pair_motion_file')
    })
    if not chips:
        print("No datasets found!")
        return
    
    print("\n" + "="*80)
    print("           DETECTION METHODS COMPARISON - ALL CHIPS")
    print("="*80 + "\n")
    
    # Collect results for all chips
    all_results = []
    
    for chip in chips:
        try:
            pair = resolve_explicit_pair(chip=chip, num_sc=64)
            static_presence_packets, motion_packets = load_static_presence_and_motion(
                static_presence_file=pair.static_presence.path,
                motion_file=pair.motion.path,
                chip=chip,
            )
        except FileNotFoundError:
            continue

        context_cfg = resolve_context_aware_config(pair, static_presence_packets)
        chip_threshold = context_cfg['threshold']
        
        print(f"Processing {chip}...", end=" ", flush=True)
        
        result = compare_detection_methods(
            static_presence_packets,
            motion_packets,
            WINDOW_SIZE_MS,
            chip_threshold,
        )
        methods, classic_baseline, classic_movement, timing, ml_baseline, ml_movement, method_thresholds, results = result
        result_by_name = {r['name']: r for r in results}
        
        # Calculate metrics for Lightweight and High Accuracy.
        num_baseline = len(static_presence_packets)
        num_movement = len(motion_packets)

        classic_res = result_by_name.get('Lightweight', {'fp': 0, 'tp': 0})
        classic_fp = classic_res['fp']
        classic_tp = classic_res['tp']
        classic_recall = classic_tp / num_movement * 100 if num_movement > 0 else 0
        classic_precision = classic_tp / (classic_tp + classic_fp) * 100 if (classic_tp + classic_fp) > 0 else 0
        classic_f1 = 2 * classic_precision * classic_recall / (classic_precision + classic_recall) if (classic_precision + classic_recall) > 0 else 0
        
        # ML metrics from the trained-default threshold evaluation path
        if ml_baseline and ml_movement:
            ml_res = result_by_name.get('High Accuracy', {'fp': 0, 'tp': 0})
            ml_fp = ml_res['fp']
            ml_tp = ml_res['tp']
            ml_recall = ml_tp / num_movement * 100 if num_movement > 0 else 0
            ml_precision = ml_tp / (ml_tp + ml_fp) * 100 if (ml_tp + ml_fp) > 0 else 0
            ml_f1 = 2 * ml_precision * ml_recall / (ml_precision + ml_recall) if (ml_precision + ml_recall) > 0 else 0
        else:
            ml_recall = ml_precision = ml_f1 = ml_fp = 0
        
        all_results.append({
            'chip': chip,
            'context_source': context_cfg['context_source'],
            'num_baseline': num_baseline,
            'classic': {'recall': classic_recall, 'fp': classic_fp, 'precision': classic_precision, 'f1': classic_f1},
            'ml': {'recall': ml_recall, 'fp': ml_fp, 'precision': ml_precision, 'f1': ml_f1},
        })
        print("done")
    
    # Print summary table
    print("\n" + "="*80)
    print("                         SUMMARY TABLE")
    print("="*80 + "\n")
    
    print(f"{'Chip':<6} {'Detector':<10} {'Recall':>10} {'FP Rate':>10} {'Precision':>10} {'F1':>10}")
    print("-"*80)
    
    for r in all_results:
        chip = r['chip']
        num_baseline = r['num_baseline']
        print(f"Context source ({chip}): {r['context_source']}")
        
        for detector, data in [('Lightweight', r['classic']), ('High Accuracy', r['ml'])]:
            fp_rate = data['fp'] / num_baseline * 100 if num_baseline > 0 else 0
            # Highlight best detector per chip
            best_f1 = max(r['classic']['f1'], r['ml']['f1'])
            marker = "**" if data['f1'] == best_f1 and data['f1'] > 0 else ""
            print(f"{chip:<6} {marker}{detector:<8} {data['recall']:>9.1f}% {fp_rate:>9.1f}% {data['precision']:>9.1f}% {data['f1']:>9.1f}%")
        print()
    
    print("="*80)
    print("** = Best F1 score for chip")
    print("="*80 + "\n")


def main():
    raw_args = sys.argv[1:]
    chip_explicit = '--chip' in raw_args
    parser = argparse.ArgumentParser(description='Compare detection methods (RSSI, Lightweight, High Accuracy)')
    parser.add_argument('--chip', type=str, default='C6', help='Chip type: C6, S3, etc.')
    parser.add_argument('--dataset', type=str, default=None,
                        help='Dataset filename, stem, or dataset id; pair is resolved from metadata')
    parser.add_argument('--interactive', action='store_true',
                        help='Choose the dataset interactively from dataset_info.json')
    parser.add_argument('--all', action='store_true', help='Run on all available chips and show summary')
    parser.add_argument('--use-test-dataset', action='store_true',
                        help='Use the latest long-recording replay for the selected chip and split by motion start packet')
    parser.add_argument('--test-motion-start-packet', type=int, default=None,
                        help='Override motion start packet index when using --use-test-dataset')
    parser.add_argument('--plot', action='store_true', help='Show visualization plots')
    args = parser.parse_args()
    
    if args.all:
        run_all_chips()
        return
    
    print("\n" + "="*60)
    print("       Detection Methods Comparison (Lightweight vs High Accuracy)")
    print("="*60 + "\n")
    
    chip = args.chip.upper()
    if args.use_test_dataset:
        print("Loading test dataset...")
    else:
        print(f"Loading {chip} data...")

    try:
        if args.use_test_dataset:
            try:
                test_path, static_presence_packets, motion_packets, motion_start_packet, chip_name, test_entry = load_test_dataset(
                    chip=chip,
                    motion_start_packet=args.test_motion_start_packet
                )
            except FileNotFoundError:
                if chip_explicit:
                    raise
                print(f"   No test dataset for default chip {chip}, using latest available test dataset")
                test_path, static_presence_packets, motion_packets, motion_start_packet, chip_name, test_entry = load_test_dataset(
                    chip=None,
                    motion_start_packet=args.test_motion_start_packet
                )
            context_cfg = resolve_context_aware_config_for_test(test_entry, static_presence_packets)
            threshold = context_cfg['threshold']
            context_source = context_cfg['context_source']
            confidence_factor = context_cfg['confidence_factor']
        else:
            chip_filter = chip if chip_explicit and not args.dataset else (None if args.dataset else chip)
            if args.interactive:
                selected = select_dataset_interactively(
                    chip=chip if chip_explicit else None,
                    num_sc=64,
                    require_pair=True,
                    prompt='Select dataset for detection comparison',
                )
                pair = resolve_explicit_pair(dataset=selected.path.name, num_sc=64)
            else:
                pair = resolve_explicit_pair(dataset=args.dataset, chip=chip_filter, num_sc=64)
            static_presence_path = pair.static_presence.path
            motion_path = pair.motion.path
            chip_name = pair.chip
            static_presence_packets, motion_packets = load_static_presence_and_motion(
                static_presence_file=static_presence_path,
                motion_file=motion_path,
                chip=chip,
                dataset=args.dataset,
            )
            context_cfg = resolve_context_aware_config(pair, static_presence_packets)
            threshold = context_cfg['threshold']
            context_source = context_cfg['context_source']
            confidence_factor = context_cfg['confidence_factor']
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return
    except ValueError as e:
        print(f"Error: {e}")
        return

    print(f"   Chip: {chip_name}")
    if args.use_test_dataset:
        print(f"   Test dataset: {test_path.name}")
        print(f"   Motion starts at packet: {motion_start_packet}")
    else:
        print(f"   Context source: {context_source}")
    print(f"   Static presence: {len(static_presence_packets)} packets")
    print(f"   Motion:          {len(motion_packets)} packets\n")
    print(f"   Fixed subcarriers: {list(DEFAULT_SUBCARRIERS)}")
    print(f"   Lightweight runtime threshold: {threshold:.6f}")
    print(f"   Confidence factor: {confidence_factor:.1f}\n")
    
    result = compare_detection_methods(
        static_presence_packets,
        motion_packets,
        WINDOW_SIZE_MS,
        threshold,
    )
    methods, classic_baseline, classic_movement, timing, ml_baseline, ml_movement, method_thresholds, results = result
    
    print_comparison_summary(methods, classic_baseline, classic_movement,
                            threshold, timing,
                            ml_baseline, ml_movement,
                            method_thresholds, results)
    
    if args.plot:
        print("Generating comparison visualization...\n")
        plot_comparison(methods, classic_baseline, classic_movement,
                       threshold, timing,
                       ml_baseline, ml_movement, method_thresholds, results)


if __name__ == '__main__':
    main()
