# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Host-only delay-compensated coherence checks against the plain formula.

The full-band and subband coherences of one reference now read a shared
cross-product array instead of each rebuilding it. The reference here spells the
definition out from scratch for both, so a regression in the shared buffer, in
the span arithmetic, or in the reuse across the lag and adjacent references
shows up as a value mismatch rather than a silent feature drift.
"""

import math
import random

import numpy as np
import pytest

from tools.lib import host_feature_trackers as host

HT20_LIVE_BINS = host.HT20_LIVE_BINS
HT20_LIVE_WIDTH = len(HT20_LIVE_BINS)
ChannelCoherenceTracker = host.ChannelCoherenceTracker
_SUBBAND_SPANS = host._SUBBAND_SPANS
complex_profile = host.complex_profile

# The reference keeps the absolute bins the definition is written in. The
# runtime no longer needs them, because the derotation factors them out as one
# shared phase per band, so deriving them here keeps the reference independent.
_SUBBAND_BIN_INDICES = tuple(
    tuple(float(HT20_LIVE_BINS[i]) for i in range(start, stop))
    for start, stop in _SUBBAND_SPANS
)

# NumPy reassociates its sums and the shared buffers change nothing else, so
# both paths land far below this bound. It is still many orders tighter than any
# real change to the cross products, the spans, or the ramp estimate.
RELATIVE_TOLERANCE = 1e-11

SUBBAND_COUNT = 4
SUBBAND_WIDTH = 14


def reference_full_band(current, reference):
    """The full live-band definition, written from scratch."""
    cross = [current[i] * reference[i].conjugate() for i in range(HT20_LIVE_WIDTH)]
    total = 0.0
    for value in cross:
        total += abs(value)
    if total <= 0.0:
        return 0.0
    ramp_sum = 0j
    for i in range(1, HT20_LIVE_WIDTH):
        # The DC gap splits the band, so that one pair is not adjacent.
        if HT20_LIVE_BINS[i] - HT20_LIVE_BINS[i - 1] != 1:
            continue
        ramp_sum += cross[i] * cross[i - 1].conjugate()
    ramp = math.atan2(ramp_sum.imag, ramp_sum.real)
    aligned = 0j
    for i in range(HT20_LIVE_WIDTH):
        angle = -ramp * float(HT20_LIVE_BINS[i])
        aligned += cross[i] * complex(math.cos(angle), math.sin(angle))
    return abs(aligned) / total


def reference_band(current, reference, start, stop, bin_index):
    """One contiguous subband, written from scratch."""
    cross = [current[i] * reference[i].conjugate() for i in range(start, stop)]
    total = 0.0
    for value in cross:
        total += abs(value)
    if total <= 0.0:
        return 0.0
    ramp_sum = 0j
    for pos in range(1, len(cross)):
        ramp_sum += cross[pos] * cross[pos - 1].conjugate()
    ramp = math.atan2(ramp_sum.imag, ramp_sum.real)
    aligned = 0j
    for pos in range(len(cross)):
        angle = -ramp * bin_index[pos]
        aligned += cross[pos] * complex(math.cos(angle), math.sin(angle))
    return abs(aligned) / total


def reference_subbands(current, reference):
    return [
        reference_band(current, reference, start, stop, _SUBBAND_BIN_INDICES[i])
        for i, (start, stop) in enumerate(_SUBBAND_SPANS)
    ]


def deterministic_pairs():
    """Named (current, reference) pairs covering flat, rotated, and null cases."""
    flat = [complex(9.0, -4.0)] * HT20_LIVE_WIDTH
    ramped = [
        complex(math.cos(0.15 * i), math.sin(0.15 * i)) * 20.0
        for i in range(HT20_LIVE_WIDTH)
    ]
    # A pure delay: every bin rotates by a constant per-bin phase, which the
    # ramp estimate is supposed to remove entirely.
    delayed = [
        ramped[i] * complex(math.cos(0.03 * HT20_LIVE_BINS[i]),
                            math.sin(0.03 * HT20_LIVE_BINS[i]))
        for i in range(HT20_LIVE_WIDTH)
    ]
    half_silent = [
        complex(0.0, 0.0) if i < HT20_LIVE_WIDTH // 2 else complex(6.0, 2.0)
        for i in range(HT20_LIVE_WIDTH)
    ]
    return {
        "flat_vs_flat": (flat, flat),
        "ramped_vs_delayed": (ramped, delayed),
        "delayed_vs_ramped": (delayed, ramped),
        "half_silent": (half_silent, ramped),
    }


def random_pairs(count=24, seed=20260805):
    rng = random.Random(seed)

    def profile():
        return [
            complex(rng.uniform(-128.0, 127.0), rng.uniform(-128.0, 127.0))
            for _ in range(HT20_LIVE_WIDTH)
        ]

    return [(profile(), profile()) for _ in range(count)]


def test_subbands_tile_the_live_band_exactly() -> None:
    """Sharing one cross array is only valid because the spans tile the band."""
    assert len(_SUBBAND_SPANS) == SUBBAND_COUNT

    covered = []
    for start, stop in _SUBBAND_SPANS:
        assert stop - start == SUBBAND_WIDTH
        covered.extend(range(start, stop))
    assert covered == list(range(HT20_LIVE_WIDTH))


@pytest.mark.parametrize("name", sorted(deterministic_pairs()))
def test_deterministic_pairs_match_the_reference(name) -> None:
    current, reference = deterministic_pairs()[name]
    np_current = np.asarray(current, dtype=np.complex128)
    np_reference = np.asarray(reference, dtype=np.complex128)

    expected_full = reference_full_band(current, reference)
    assert host.delay_compensated_coherence(np_current, np_reference) == pytest.approx(
        expected_full, rel=RELATIVE_TOLERANCE, abs=RELATIVE_TOLERANCE
    )

    expected_bands = reference_subbands(current, reference)
    host_bands = host.subband_coherences(np_current, np_reference)
    for i in range(SUBBAND_COUNT):
        assert host_bands[i] == pytest.approx(
            expected_bands[i], rel=RELATIVE_TOLERANCE, abs=RELATIVE_TOLERANCE
        )


def test_random_pairs_match_the_reference() -> None:
    for current, reference in random_pairs():
        np_current = np.asarray(current, dtype=np.complex128)
        np_reference = np.asarray(reference, dtype=np.complex128)

        expected_full = reference_full_band(current, reference)
        assert expected_full > 0.0
        assert host.delay_compensated_coherence(
            np_current, np_reference
        ) == pytest.approx(expected_full, rel=RELATIVE_TOLERANCE)

        expected_bands = reference_subbands(current, reference)
        host_bands = host.subband_coherences(np_current, np_reference)
        for i in range(SUBBAND_COUNT):
            assert expected_bands[i] > 0.0
            assert host_bands[i] == pytest.approx(
                expected_bands[i], rel=RELATIVE_TOLERANCE
            )


def test_zero_profiles_are_guarded_rather_than_divided() -> None:
    [0j] * HT20_LIVE_WIDTH
    np_zeros = np.zeros(HT20_LIVE_WIDTH, dtype=np.complex128)

    assert host.delay_compensated_coherence(np_zeros, np_zeros) == 0.0
    assert list(host.subband_coherences(np_zeros, np_zeros)) == [0.0] * SUBBAND_COUNT


def test_null_csi_gives_a_zero_profile_and_zero_coherence() -> None:
    for csi_data in (None, [], [0] * 16):
        profile = complex_profile(csi_data)
        assert host.delay_compensated_coherence(profile, profile) == 0.0
        assert list(host.subband_coherences(profile, profile)) == [0.0] * SUBBAND_COUNT


def test_shared_cross_buffer_is_not_reused_across_references() -> None:
    """The tracker fills one buffer per reference; lag must not leak into adjacent.

    A tracker at lag 1 sees the same reference twice, so this drives the buffer
    through both fills and compares against values computed independently.
    """
    rng = random.Random(31337)
    tracker = ChannelCoherenceTracker(window_size=32, lag=2)
    profiles = []

    for _ in range(6):
        csi_data = [rng.randrange(-128, 128) for _ in range(128)]
        profiles.append(complex_profile(csi_data))
        tracker.process_packet(csi_data)

    # Packet i is compared against packet i-2 (lag) and packet i-1 (adjacent).
    expected_lag = [
        reference_full_band(profiles[i], profiles[i - 2]) for i in range(2, 6)
    ]
    expected_adjacent = [
        reference_full_band(profiles[i], profiles[i - 1]) for i in range(1, 6)
    ]
    expected_gap = (
        sum(expected_adjacent) / len(expected_adjacent)
        - sum(expected_lag) / len(expected_lag)
    )

    assert tracker.count() == len(expected_lag)
    assert tracker.coherence_gap() == pytest.approx(expected_gap, rel=1e-9, abs=1e-12)


def test_tracker_subband_gap_follows_the_reference() -> None:
    rng = random.Random(777)
    tracker = ChannelCoherenceTracker(window_size=32, lag=3, track_subbands=True)
    profiles = []

    for _ in range(9):
        csi_data = [rng.randrange(-128, 128) for _ in range(128)]
        profiles.append(complex_profile(csi_data))
        tracker.process_packet(csi_data)

    lag_means = [0.0] * SUBBAND_COUNT
    adjacent_means = [0.0] * SUBBAND_COUNT
    lag_rows = [reference_subbands(profiles[i], profiles[i - 3]) for i in range(3, 9)]
    adjacent_rows = [
        reference_subbands(profiles[i], profiles[i - 1]) for i in range(1, 9)
    ]
    for band in range(SUBBAND_COUNT):
        lag_means[band] = sum(row[band] for row in lag_rows) / len(lag_rows)
        adjacent_means[band] = sum(row[band] for row in adjacent_rows) / len(
            adjacent_rows
        )
    gaps = sorted(adjacent_means[b] - lag_means[b] for b in range(SUBBAND_COUNT))
    expected_median = 0.5 * (gaps[1] + gaps[2])

    assert tracker.coherence_subband_gap_median() == pytest.approx(
        expected_median, rel=1e-9, abs=1e-12
    )
