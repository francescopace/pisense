# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Tests for the production Lightweight detector."""

import math
from array import array

import pytest

from lightweight_detector import LightweightDetector
import micro_espectre.lightweight_detector as micro_lightweight_module
from micro_espectre.lightweight_detector import (
    LightweightDetector as MicroLightweightDetector,
)
from detector_interface import (
    MotionState,
    detector_needs_startup_calibration,
    get_detector_algorithm,
    load_detector_class,
)
from csi_features import TURB_IQR_AGGREGATION_WIDTH, calc_autocorrelation
from segmentation import SegmentationContext


def test_registry_exposes_lightweight_detector() -> None:
    detector = LightweightDetector()

    assert load_detector_class("lightweight") is LightweightDetector
    assert detector_needs_startup_calibration("lightweight")
    assert get_detector_algorithm(detector) == "lightweight"
    assert detector.get_name() == "Lightweight"


def test_linear_fusion_uses_exported_center_scale_and_weights() -> None:
    detector = LightweightDetector()

    assert detector._calculate_logit(
        detector.FEATURE_CENTER[0],
        detector.FEATURE_CENTER[1],
    ) == pytest.approx(detector.INTERCEPT)
    assert detector._calculate_logit(
        detector.FEATURE_CENTER[0] + detector.FEATURE_SCALE[0],
        detector.FEATURE_CENTER[1] + detector.FEATURE_SCALE[1],
    ) == pytest.approx(
        detector.INTERCEPT + sum(detector.FEATURE_WEIGHT)
    )


def test_hampel_master_switch_controls_turbulence_stream() -> None:
    enabled = LightweightDetector(enable_hampel=True)
    disabled = LightweightDetector(enable_hampel=False)

    assert enabled._context.hampel_filter is not None
    assert enabled._aggregated_context.hampel_filter is not None
    assert disabled._context.hampel_filter is None
    assert disabled._aggregated_context.hampel_filter is None


def test_native_detector_caches_subcarrier_plan(monkeypatch) -> None:
    calls = []

    class NativeFeatures:
        class Detector:
            def __init__(self, algorithm, **kwargs):
                calls.append(("init", algorithm, tuple(kwargs["subcarriers"])))

            def set_subcarriers(self, subcarriers):
                calls.append(("set", tuple(subcarriers)))

            def process(self, csi, timestamp_us):
                calls.append(("process", len(csi), timestamp_us))

            def deinit(self):
                calls.append(("deinit",))

    monkeypatch.setattr(micro_lightweight_module, "_native_features", NativeFeatures)
    monkeypatch.setattr(micro_lightweight_module, "array", array)
    detector = MicroLightweightDetector(enable_hampel=False)
    payload = bytearray(128)

    detector.process_packet(payload)
    changed_plan = tuple(detector._selected_subcarriers[:-1])
    detector.process_packet(payload, changed_plan)

    assert calls[0] == (
        "init",
        "lightweight",
        tuple(micro_lightweight_module.DEFAULT_SUBCARRIERS),
    )
    assert calls.count(("set", changed_plan)) == 1
    assert [call[0] for call in calls].count("process") == 2
    assert detector._context is None
    assert detector._aggregated_context is None


def test_full_native_detector_owns_device_hot_path(monkeypatch) -> None:
    calls = []

    class NativeDetector:
        class Detector:
            def __init__(self, algorithm, **kwargs):
                calls.append(("init", kwargs["window_size"], tuple(kwargs["subcarriers"])))

            def process(self, csi, timestamp_us):
                calls.append(("process", len(csi), timestamp_us))

            def update(self, output):
                output[:] = array("f", (1.0, 0.75, 0.6, 0.2, 0.3, 1.1))

            def set_minimum_valid(self, count):
                calls.append(("minimum", count))

            def is_ready(self):
                return True

            def get_total_packets(self):
                return 1

            def get_threshold(self):
                return 0.6

            def get_metric(self):
                return 0.75

            def deinit(self):
                calls.append(("deinit",))

    monkeypatch.setattr(micro_lightweight_module, "_native_features", NativeDetector)
    monkeypatch.setattr(micro_lightweight_module, "array", array)
    detector = MicroLightweightDetector(enable_hampel=False)

    detector.set_minimum_valid_samples(70)
    detector.process_packet(bytearray(128), timestamp_us=1234)
    metrics = detector.update_state()

    assert detector._context is None
    assert detector._aggregated_context is None
    assert calls[0][0] == "init"
    assert ("minimum", 70) in calls
    assert ("process", 128, 1234) in calls
    assert metrics["state"] is MotionState.MOTION
    assert metrics["motion_metric"] == pytest.approx(0.75)
    assert detector.total_packets == 1


def test_micropython_requires_compatible_core_module(monkeypatch) -> None:
    monkeypatch.setattr(micro_lightweight_module, "_native_features", None)

    with pytest.raises(RuntimeError, match="compatible espectre core module"):
        MicroLightweightDetector(enable_hampel=False)


def test_fused_packet_path_matches_shared_turbulence_helpers() -> None:
    detector = LightweightDetector(enable_hampel=False)
    payload = bytearray((index * 37 + 19) % 256 for index in range(128))
    selected = tuple(detector._selected_subcarriers)
    amplitudes = [0.0] * 64

    count = SegmentationContext.fill_subcarrier_energy_buffer(payload, amplitudes)
    SegmentationContext.energies_to_amplitudes_in_place(amplitudes, count)
    direct = SegmentationContext(enable_hampel=False)
    aggregated = SegmentationContext(
        enable_hampel=False,
        adjacent_aggregation_width=TURB_IQR_AGGREGATION_WIDTH,
    )
    expected_direct = direct.calculate_spatial_turbulence_from_subcarrier_amplitudes(
        amplitudes, count, selected
    )
    expected_aggregated = (
        aggregated.calculate_spatial_turbulence_from_subcarrier_amplitudes(
            amplitudes, count, selected
        )
    )

    detector.process_packet(payload, selected)

    assert detector._context.last_turbulence == pytest.approx(expected_direct)
    assert detector._aggregated_context.last_turbulence == pytest.approx(
        expected_aggregated
    )


def test_unsigned_lookup_matches_signed_csi_packet_path() -> None:
    payload = bytearray((index * 37 + 19) % 256 for index in range(128))
    unsigned_detector = LightweightDetector(enable_hampel=False)
    signed_detector = LightweightDetector(enable_hampel=False)

    unsigned_detector.process_packet(payload, unsigned_detector._selected_subcarriers)
    signed_detector.process_packet(
        memoryview(payload).cast("b"),
        signed_detector._selected_subcarriers,
    )

    assert unsigned_detector._context.last_turbulence == pytest.approx(
        signed_detector._context.last_turbulence
    )
    assert unsigned_detector._aggregated_context.last_turbulence == pytest.approx(
        signed_detector._aggregated_context.last_turbulence
    )


def test_direct_window_features_match_chronological_reference_with_missing_slots() -> None:
    detector = LightweightDetector(window_size=8, enable_hampel=False)
    direct_values = [0.12, 0.31, None, 0.27, 0.49, 0.18, None, 0.42, 0.36]
    aggregate_values = [0.21, 0.44, None, 0.19, 0.38, 0.29, None, 0.47, 0.33]
    for direct_value, aggregate_value in zip(direct_values, aggregate_values, strict=True):
        if direct_value is None:
            detector._context.add_missing_slot()
            detector._aggregated_context.add_missing_slot()
        else:
            detector._context.add_turbulence(direct_value)
            detector._aggregated_context.add_turbulence(aggregate_value)

    ordered = [0.0] * 8
    validity = [False] * 8
    count = detector._context.copy_chronological_into(ordered, validity)
    valid = [ordered[index] for index in range(count) if validity[index]]
    mean = sum(valid) / len(valid)
    variance = sum((value - mean) ** 2 for value in valid) / len(valid)
    expected_autocorr = calc_autocorrelation(
        ordered,
        count,
        mean=mean,
        variance=variance,
        lag=detector._autocorr_lag,
        validity=validity,
    )

    count = detector._aggregated_context.copy_chronological_into(ordered, validity)
    valid = sorted(ordered[index] for index in range(count) if validity[index])
    mean = sum(valid) / len(valid)
    q25 = detector._quantile_sorted(valid, 0.25)
    q75 = detector._quantile_sorted(valid, 0.75)
    expected_iqr = (q75 - q25) / max(abs(mean), 1e-6)

    assert detector._turb_autocorr() == pytest.approx(expected_autocorr)
    assert detector._turb_iqr_over_mean_aggr() == pytest.approx(expected_iqr)


def test_lightweight_allocates_aggregated_turbulence_state() -> None:
    detector = LightweightDetector()

    assert len(detector._aggregated_context.turbulence_buffer) == 100
    assert detector._aggregated_context.buffer_count == 0


def test_startup_q95_adapts_probability_threshold() -> None:
    detector = LightweightDetector()
    detector._startup_logits = [-1.0, -0.8, -0.6, -0.4]

    detector.set_adaptive_threshold(0.01)

    q95 = detector._quantile(detector._startup_logits, 0.95)
    base_logit = math.log(
        detector.BASE_THRESHOLD / (1.0 - detector.BASE_THRESHOLD)
    )
    expected = detector._sigmoid(
        base_logit
        + detector.STARTUP_STRENGTH * (q95 - detector.TRAIN_IDLE_Q95_LOGIT)
    )
    assert detector.get_threshold() == pytest.approx(expected)


def test_noisy_startup_still_uses_the_shifted_logit_threshold() -> None:
    detector = LightweightDetector()
    detector._startup_logits = [10.0] * 4

    detector.set_adaptive_threshold(0.01)

    q95 = detector._quantile(detector._startup_logits, 0.95)
    base_logit = math.log(
        detector.BASE_THRESHOLD / (1.0 - detector.BASE_THRESHOLD)
    )
    expected = detector._sigmoid(
        base_logit
        + detector.STARTUP_STRENGTH * (q95 - detector.TRAIN_IDLE_Q95_LOGIT)
    )
    assert detector.get_threshold() == pytest.approx(expected)
    assert detector.get_threshold() > detector.BASE_THRESHOLD


def test_manual_threshold_uses_probability_scale() -> None:
    detector = LightweightDetector()

    assert detector.set_threshold(0.75)
    assert detector.get_threshold() == pytest.approx(0.75)
    assert not detector.set_threshold(1.01)
    assert detector.get_threshold() == pytest.approx(0.75)


def test_update_state_uses_weighted_probability(monkeypatch) -> None:
    detector = LightweightDetector(window_size=20, threshold=0.5)
    monkeypatch.setattr(detector, "is_ready", lambda: True)
    monkeypatch.setattr(detector, "_turb_autocorr", lambda: detector.FEATURE_CENTER[0])
    monkeypatch.setattr(
        detector,
        "_turb_iqr_over_mean_aggr",
        lambda: detector.FEATURE_CENTER[1],
    )

    metrics = detector.update_state()

    expected = detector._sigmoid(detector.INTERCEPT)
    assert metrics["probability"] == pytest.approx(expected)
    assert metrics["turb_autocorr"] == pytest.approx(detector.FEATURE_CENTER[0])
    assert metrics["turb_iqr_over_mean_aggr"] == pytest.approx(
        detector.FEATURE_CENTER[1]
    )
    # Derived from the probability rather than pinned, so a refit that moves the
    # intercept across the threshold does not read as a state-machine fault.
    assert metrics["state"] == (
        MotionState.MOTION if expected > detector.get_threshold() else MotionState.IDLE
    )


def test_manual_threshold_suspends_settling_until_recalibration() -> None:
    detector = LightweightDetector()
    detector.set_adaptive_threshold(0.5)
    assert detector.set_threshold(0.9)
    detector.reset()
    detector._current_logit = -10.0
    evaluations = detector.SETTLE_BLOCKS * detector.SETTLE_BLOCK_EVALUATIONS

    for _ in range(evaluations):
        detector._observe_settled_level()
    assert detector.get_threshold() == pytest.approx(0.9)

    detector.on_startup_calibration_begin()
    detector.set_adaptive_threshold(0.5)
    calibrated = detector.get_threshold()
    assert not detector.set_threshold(1.01)
    for _ in range(evaluations):
        detector._observe_settled_level()
    assert detector.get_threshold() < calibrated


def test_reset_preserves_threshold_and_clears_feature_state() -> None:
    detector = LightweightDetector(threshold=0.7)
    detector._current_probability = 0.9
    detector._startup_logits = [1.0]

    detector.reset()

    assert detector.get_threshold() == pytest.approx(0.7)
    assert detector.get_motion_metric() == 0.0
    assert detector._startup_logits == []
    assert detector.get_state() == MotionState.IDLE
