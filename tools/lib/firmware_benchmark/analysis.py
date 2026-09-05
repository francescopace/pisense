# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Firmware benchmark analysis owner."""

from __future__ import annotations

import math
import re
import statistics
from typing import Sequence

from tools.lib.firmware_benchmark.models import RuntimeMetrics
from tools.lib.firmware_benchmark.settings import (
    DIRECT_SAMPLE_INTERVAL_SECONDS,
    HEAP_STABILITY_MAX_DECLINE_PERCENT,
    HEAP_STABILITY_WINDOW_SECONDS,
    MINIMUM_OCCUPANCY_PERCENT,
    MIN_MOTION_SAMPLES,
    RUNTIME_STATUS_BOUNDARY_TOLERANCE_SAMPLES,
    RUNTIME_STATUS_GAP_TOLERANCE_MS,
    STARTUP_GRACE_SECONDS,
)

ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


LOG_TIMESTAMP_RE = re.compile(r"\((?P<timestamp_ms>\d+)\)")


def strip_ansi(text: str) -> str:
    return ANSI_ESCAPE_RE.sub("", text)


def _append_occupancy_reasons(
    metrics: RuntimeMetrics,
    reasons: list[str],
    *,
    missing_reason: str = "CSI occupancy was not logged",
    low_reason_prefix: str = "mean CSI occupancy",
) -> None:
    if metrics.occupancy_samples == 0:
        reasons.append(missing_reason)
    elif metrics.occupancy_mean is not None and metrics.occupancy_mean < MINIMUM_OCCUPANCY_PERCENT:
        reasons.append(
            f"{low_reason_prefix} {metrics.occupancy_mean:.1f}% is below the "
            f"{MINIMUM_OCCUPANCY_PERCENT:.0f}% detector-ready floor"
        )


def _numeric(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)

def _integer(value: object) -> int | None:
    number = _numeric(value)
    return int(number) if number is not None else None

def _counter_rate(current: int | None, previous: int | None, elapsed_ms: int | None) -> float | None:
    if current is None or previous is None or elapsed_ms is None or elapsed_ms <= 0:
        return None
    delta = current - previous if current >= previous else (1 << 64) - previous + current
    return delta * 1000.0 / elapsed_ms

def analyze_direct_evidence(
    samples: Sequence[dict[str, object]],
    events: Sequence[dict[str, object]],
    *,
    duration_seconds: int,
    require_motion: bool,
    require_detection_timing: bool,
    sample_interval_seconds: float = DIRECT_SAMPLE_INTERVAL_SECONDS,
    status_gap_tolerance_ms: int = RUNTIME_STATUS_GAP_TOLERANCE_MS,
    attempts: Sequence[dict[str, object]] = (),
) -> tuple[RuntimeMetrics, list[str]]:
    metrics = RuntimeMetrics()
    reasons: list[str] = []
    metrics.status_samples = len(samples)
    metrics.direct_request_attempts = len(attempts)
    failed_attempts = [
        attempt for attempt in attempts if attempt.get("succeeded") is False
    ]
    metrics.direct_request_failures = len(failed_attempts)
    metrics.direct_request_censored = sum(
        attempt.get("censored") is True for attempt in failed_attempts
    )
    if failed_attempts:
        reasons.append(
            f"{len(failed_attempts)}/{len(attempts)} Direct control attempts failed "
            f"({metrics.direct_request_censored} censored)"
        )
    metrics.status_expected_samples = max(1, math.ceil(duration_seconds / sample_interval_seconds))
    metrics.motion_samples = sum(event.get("event") == "motion" for event in events)
    metrics.motion_expected_samples = MIN_MOTION_SAMPLES if require_motion else 0
    timestamps = [value for sample in samples if (value := _integer(sample.get("timestamp_ms"))) is not None]
    uptimes = [value for sample in samples if (value := _integer(sample.get("uptime"))) is not None]
    if timestamps:
        metrics.status_first_timestamp_ms = timestamps[0]
        metrics.status_last_timestamp_ms = timestamps[-1]
    host_times = [float(sample["host_elapsed_seconds"]) for sample in samples]
    if len(host_times) > 1:
        host_gaps = [(right - left) * 1000.0 for left, right in zip(host_times, host_times[1:], strict=False)]
        gaps = []
        stale_timestamps = 0
        for index, host_gap in enumerate(host_gaps):
            left = _integer(samples[index].get("timestamp_ms"))
            right = _integer(samples[index + 1].get("timestamp_ms"))
            if left is not None and right is not None and right > left:
                gaps.append(right - left)
            else:
                gaps.append(host_gap)
                if left is not None and right == left:
                    stale_timestamps += 1
        metrics.status_interval_mean_ms = statistics.fmean(gaps)
        metrics.status_interval_max_ms = int(max(gaps))
        max_gap_ms = int(sample_interval_seconds * 1000) + status_gap_tolerance_ms
        metrics.status_gap_count = sum(gap > max_gap_ms for gap in gaps)
        if metrics.status_gap_count:
            reasons.append(f"Direct diagnostics gap reached {max(gaps) / 1000.0:.2f}s")
        if stale_timestamps:
            reasons.append(
                f"Direct diagnostics timestamp did not advance in {stale_timestamps} sampled interval(s)"
            )
    if len(samples) < max(1, metrics.status_expected_samples - RUNTIME_STATUS_BOUNDARY_TOLERANCE_SAMPLES):
        reasons.append(
            f"only {len(samples)}/{metrics.status_expected_samples} expected Direct diagnostics samples were received"
        )
    if require_motion and metrics.motion_samples < MIN_MOTION_SAMPLES:
        reasons.append(f"only {metrics.motion_samples}/{MIN_MOTION_SAMPLES} Direct motion events were received")
    if any(right < left for left, right in zip(uptimes, uptimes[1:], strict=False)):
        metrics.device_reboots = 1
        reasons.append("Direct uptime regressed during the scored window")

    pps = [value for sample in samples if (value := _numeric(sample.get("csi_admitted_pps"))) is not None]
    metrics.packet_rate_samples = len(pps)
    if pps:
        metrics.pps_mean = statistics.fmean(pps)
        metrics.pps_min = int(min(pps))
        metrics.pps_max = int(max(pps))
        metrics.pps_stddev = statistics.pstdev(pps) if len(pps) > 1 else 0.0
        if metrics.pps_mean <= 0:
            reasons.append("Direct diagnostics reported no admitted CSI packets")
    elif require_motion:
        reasons.append("Direct diagnostics did not report CSI packet rate")

    occupancy = [
        value for sample in samples if (value := _numeric(sample.get("csi_occupancy_percent"))) is not None
    ]
    metrics.occupancy_samples = len(occupancy)
    if occupancy:
        metrics.occupancy_mean = statistics.fmean(occupancy)
        metrics.occupancy_min = int(min(occupancy))
        metrics.occupancy_max = int(max(occupancy))
        _append_occupancy_reasons(
            metrics,
            reasons,
            missing_reason="Direct CSI occupancy was not reported",
            low_reason_prefix="Direct mean CSI occupancy",
        )
    elif require_motion:
        reasons.append("Direct CSI occupancy was not reported")

    heap = [value for sample in samples if (value := _numeric(sample.get("free_memory_kb"))) is not None]
    settled_heap = [
        (float(sample["host_elapsed_seconds"]), value)
        for sample in samples
        if float(sample["host_elapsed_seconds"]) >= STARTUP_GRACE_SECONDS
        and (value := _numeric(sample.get("free_memory_kb"))) is not None
    ]
    if heap:
        metrics.heap_free_last = int(heap[-1] * 1024.0)
    if settled_heap:
        observed_end = settled_heap[-1][0]
        final_window_start = observed_end - HEAP_STABILITY_WINDOW_SECONDS
        previous_window_start = final_window_start - HEAP_STABILITY_WINDOW_SECONDS
        previous_window = [
            value
            for elapsed, value in settled_heap
            if previous_window_start < elapsed <= final_window_start
        ]
        final_window = [
            value for elapsed, value in settled_heap if final_window_start < elapsed <= observed_end
        ]
        windows_are_complete = (
            settled_heap[0][0] <= previous_window_start
            and bool(previous_window)
            and bool(final_window)
        )
        if windows_are_complete:
            plateau_first = statistics.median(previous_window)
            plateau_last = statistics.median(final_window)
            metrics.heap_free_settled_first = int(plateau_first * 1024.0)
            metrics.heap_free_settled_last = int(plateau_last * 1024.0)
            metrics.heap_free_settled_delta = (
                metrics.heap_free_settled_last - metrics.heap_free_settled_first
            )
            if metrics.heap_free_settled_first:
                metrics.heap_free_settled_delta_percent = (
                    100.0 * metrics.heap_free_settled_delta / metrics.heap_free_settled_first
                )
                if metrics.heap_free_settled_delta_percent < -HEAP_STABILITY_MAX_DECLINE_PERCENT:
                    reasons.append(
                        "free heap did not stabilize: the final window median declined by more than "
                        f"{HEAP_STABILITY_MAX_DECLINE_PERCENT:.0f}%"
                    )
        elif heap:
            reasons.append(
                "free heap stability requires two complete consecutive "
                f"{HEAP_STABILITY_WINDOW_SECONDS}-second windows after startup grace"
            )
    elif heap:
        reasons.append(
            "free heap stability requires two complete consecutive "
            f"{HEAP_STABILITY_WINDOW_SECONDS}-second windows after startup grace"
        )
    minimum_heap = [
        value for sample in samples if (value := _numeric(sample.get("minimum_free_memory_kb"))) is not None
    ]
    largest_heap = [
        value for sample in samples if (value := _numeric(sample.get("largest_free_memory_kb"))) is not None
    ]
    metrics.heap_min = int(minimum_heap[-1] * 1024.0) if minimum_heap else None
    metrics.heap_largest_last = int(largest_heap[-1] * 1024.0) if largest_heap else None

    performance_samples: list[dict[str, object]] = []
    previous_signature: tuple[object, ...] | None = None
    for sample in samples:
        if sample.get("performance_window_ready") is not True:
            continue
        signature = (
            sample.get("runtime_load_percent"),
            sample.get("loop_avg_us"),
            sample.get("loop_max_us"),
            sample.get("detection_samples"),
            sample.get("detection_sum_us"),
        )
        if signature != previous_signature:
            performance_samples.append(sample)
            previous_signature = signature
    loads = [value for sample in performance_samples if (value := _numeric(sample.get("runtime_load_percent"))) is not None]
    loop_averages = [value for sample in performance_samples if (value := _numeric(sample.get("loop_avg_us"))) is not None]
    loop_maxima = [value for sample in performance_samples if (value := _integer(sample.get("loop_max_us"))) is not None]
    metrics.runtime_load_mean = statistics.fmean(loads) if loads else None
    metrics.loop_avg_us_mean = statistics.fmean(loop_averages) if loop_averages else None
    metrics.loop_max_us_max = max(loop_maxima) if loop_maxima else None
    detection_windows = [sample for sample in performance_samples if sample.get("detection_timing_supported") is True]
    detection_counts = [value for sample in detection_windows if (value := _integer(sample.get("detection_samples"))) is not None]
    detection_averages = [value for sample in detection_windows if (value := _numeric(sample.get("detection_avg_us"))) is not None]
    detection_minima = [value for sample in detection_windows if (value := _integer(sample.get("detection_min_us"))) is not None]
    detection_maxima = [value for sample in detection_windows if (value := _integer(sample.get("detection_max_us"))) is not None]
    metrics.detection_samples = sum(detection_counts)
    metrics.detection_avg_us_mean = statistics.fmean(detection_averages) if detection_averages else None
    metrics.detection_min_us = min(detection_minima) if detection_minima else None
    metrics.detection_max_us = max(detection_maxima) if detection_maxima else None
    if require_detection_timing and metrics.detection_samples <= 0:
        reasons.append("Direct diagnostics did not report detector timing")

    stack_values = [
        value for sample in samples if (value := _integer(sample.get("task_stack_high_water_bytes"))) is not None
    ]
    if stack_values and min(stack_values) <= 0:
        reasons.append("Direct diagnostics reported an empty task stack high-water mark")
    for key, label in (
        ("direct_rejected_connections", "rejected connection"),
        ("direct_send_failures", "send failure"),
    ):
        values = [value for sample in samples if (value := _integer(sample.get(key))) is not None]
        if len(values) > 1 and values[-1] > values[0]:
            reasons.append(f"Direct transport recorded a {label} during the scored window")
    return metrics, reasons
