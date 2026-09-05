# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
import numpy as np
import pytest

from tools.lib.host_feature_trackers import (
    CHANNEL_SHAPE_BIN_US,
    HT20_LIVE_BINS,
    ChannelShapeExcessPathTracker,
    guarded_kendall_distance,
    guarded_kendall_signature,
    rank_profile_distance,
)


def payload_from_subband_amplitudes(amplitudes, gain=1):
    payload = np.zeros(128, dtype=np.int8)
    for live_index, subcarrier in enumerate(HT20_LIVE_BINS):
        subband = live_index // 7
        payload[2 * subcarrier + 1] = int(amplitudes[subband] * gain)
    return payload


def evaluate_path(amplitude_path, gains=None, duplicate=False):
    tracker = ChannelShapeExcessPathTracker(
        track_subband_rank_gap=True,
        track_subband_kendall_lag_excess=True,
    )
    if gains is None:
        gains = [1] * len(amplitude_path)
    for index, (amplitudes, gain) in enumerate(zip(amplitude_path, gains, strict=True)):
        timestamp_us = index * CHANNEL_SHAPE_BIN_US
        payload = payload_from_subband_amplitudes(amplitudes, gain)
        tracker.process_packet(payload, timestamp_us)
        if duplicate:
            tracker.process_packet(payload.copy(), timestamp_us + 20_000)
    return tracker.excess_path(), tracker


def curved_path(count=12, base=12.0, amplitude=2.5):
    index = np.arange(8, dtype=np.float64)
    first_mode = np.cos(np.pi * (index + 0.5) / 8.0)
    second_mode = np.cos(2.0 * np.pi * (index + 0.5) / 8.0)
    path = []
    for step in range(count):
        angle = 1.5 * np.pi * step / (count - 1)
        values = base + amplitude * (
            np.cos(angle) * first_mode + np.sin(angle) * second_mode
        )
        path.append(np.rint(values).astype(int))
    return path


def test_independent_packet_gain_cancels() -> None:
    path = curved_path()
    baseline, baseline_tracker = evaluate_path(path)
    gained, gained_tracker = evaluate_path(path, gains=[1, 2, 3] * 4)

    assert gained == pytest.approx(baseline, abs=1e-12)
    assert gained_tracker.scale_curvature() == pytest.approx(
        baseline_tracker.scale_curvature(),
        abs=1e-12,
    )
    assert gained_tracker.coherent_innovation_energy() == pytest.approx(
        baseline_tracker.coherent_innovation_energy(),
        abs=1e-12,
    )
    assert gained_tracker.coherent_innovation_contrast() == pytest.approx(
        baseline_tracker.coherent_innovation_contrast(),
        abs=1e-12,
    )
    assert gained_tracker.subband_rank_gap() == pytest.approx(
        baseline_tracker.subband_rank_gap(),
        abs=1e-12,
    )
    assert gained_tracker.subband_kendall_lag_excess() == pytest.approx(
        baseline_tracker.subband_kendall_lag_excess(),
        abs=1e-12,
    )


def test_subband_rank_gap_detects_accumulated_ordinal_turnover() -> None:
    base = np.arange(10, 90, 10)
    path = [np.roll(base, step) for step in range(8)]
    _, tracker = evaluate_path(path)

    assert tracker.subband_rank_gap() > 0.0


def test_cached_subband_rank_gap_matches_direct_profile_formula() -> None:
    base = np.arange(10, 90, 10, dtype=np.float64)
    path = [np.roll(base, step) for step in range(8)]
    profiles = [values / np.linalg.norm(values) for values in path]
    adjacent = [
        rank_profile_distance(profiles[index], profiles[index - 1])
        for index in range(1, len(profiles))
    ]
    longer = [
        rank_profile_distance(profiles[index], profiles[index - 3])
        for index in range(3, len(profiles))
    ]
    expected = float(np.median(longer) - np.median(adjacent))
    _, tracker = evaluate_path(path)

    assert tracker.subband_rank_gap() == pytest.approx(expected, abs=1e-12)


def test_subband_kendall_lag_excess_matches_paired_formula() -> None:
    base = np.arange(10, 90, 10, dtype=np.float64)
    path = [np.roll(base, step) for step in range(8)]
    profiles = [values / np.linalg.norm(values) for values in path]
    signatures = [guarded_kendall_signature(profile) for profile in profiles]
    samples = []
    for index in range(3, len(signatures)):
        longer = guarded_kendall_distance(
            signatures[index], signatures[index - 3]
        )
        local = [
            guarded_kendall_distance(
                signatures[local_index], signatures[local_index - 1]
            )
            for local_index in range(index - 2, index + 1)
        ]
        assert longer is not None
        assert all(distance is not None for distance in local)
        samples.append(max(0.0, longer - float(np.mean(local))))
    _, tracker = evaluate_path(path)

    assert tracker.subband_kendall_lag_excess() == pytest.approx(
        np.median(samples),
        abs=1e-12,
    )
    assert tracker.subband_kendall_lag_excess() > 0.0


def test_guarded_kendall_signature_ignores_near_ties() -> None:
    profile = np.asarray([1.0, 0.99, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3])
    _order, valid = guarded_kendall_signature(profile)

    assert valid & 1 == 0


def test_coherent_innovation_contrast_uses_high_modes_as_noise_reference() -> None:
    coherent_path = [np.zeros(8), np.zeros(8), np.zeros(8)]
    coherent_path[1][1] = 1.0
    coherent_path[2][1] = 3.0
    noisy_path = [modes.copy() for modes in coherent_path]
    noisy_path[2][4] = 1.0

    coherent = ChannelShapeExcessPathTracker()
    coherent._binned_path = lambda: list(enumerate(coherent_path))
    noisy = ChannelShapeExcessPathTracker()
    noisy._binned_path = lambda: list(enumerate(noisy_path))

    assert coherent.coherent_innovation_contrast() == pytest.approx(1.0)
    assert noisy.coherent_innovation_contrast() == pytest.approx(0.0)


def test_exact_stutter_duplicates_do_not_change_the_path() -> None:
    path = curved_path()
    baseline, _ = evaluate_path(path)
    duplicated, _ = evaluate_path(path, duplicate=True)

    assert duplicated == pytest.approx(baseline, abs=1e-12)


def test_missing_bins_are_not_interpolated() -> None:
    path = curved_path(count=5)
    tracker = ChannelShapeExcessPathTracker()
    timestamps = [0, 80_000, 240_000, 320_000, 400_000]
    for amplitudes, timestamp_us in zip(path, timestamps, strict=True):
        tracker.process_packet(
            payload_from_subband_amplitudes(amplitudes),
            timestamp_us,
        )

    assert len(tracker._binned_path()) == len(path)


def test_curved_low_order_path_exceeds_a_slow_one_direction_path() -> None:
    base = np.arange(10, 18)
    direction = np.asarray([-1, -1, 0, 0, 0, 0, 1, 1])
    straight = [base + direction * (step // 4) for step in range(12)]

    straight_value, _ = evaluate_path(straight)
    curved_value, _ = evaluate_path(curved_path(base=70.0, amplitude=30.0))

    assert curved_value > straight_value
    assert curved_value > 0.0
