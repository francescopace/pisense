# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Reference Production Feature Helpers

Pure-Python production feature extraction used by host replay and validation.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from micro_espectre.detector_interface import MotionState

try:
    from .filters import HampelFilter
    from .segmentation import SegmentationContext
except ImportError:
    from filters import HampelFilter
    from segmentation import SegmentationContext

# Match the detector path: compare normalized profiles 10 packets apart
# (~100 ms at 100 pps).
L1_DELTA_LAG = 10
L1_DELTA_STARTUP_THRESHOLD_FACTOR = 1.1
TURB_IQR_AGGREGATION_WIDTH = 5


def calc_autocorrelation(turbulence_buffer, buffer_count, mean=None, variance=None, lag=1,
                         validity=None):
    """Calculate lag-k autocorrelation coefficient."""
    if buffer_count < lag + 2:
        return 0.0

    valid_count = buffer_count
    if validity is not None:
        valid_count = sum(1 for i in range(buffer_count) if validity[i])
    if valid_count < lag + 1:
        return 0.0

    if mean is None:
        total = 0.0
        for i in range(buffer_count):
            if validity is None or validity[i]:
                total += turbulence_buffer[i]
        mean = total / valid_count

    if variance is None:
        variance = 0.0
        for i in range(buffer_count):
            if validity is not None and not validity[i]:
                continue
            diff = turbulence_buffer[i] - mean
            variance += diff * diff
        variance /= valid_count

    if variance < 1e-10:
        return 0.0

    autocovariance = 0.0
    pair_count = 0
    for i in range(buffer_count - lag):
        if validity is not None and not (validity[i] and validity[i + lag]):
            continue
        autocovariance += (turbulence_buffer[i] - mean) * (turbulence_buffer[i + lag] - mean)
        pair_count += 1
    if pair_count == 0:
        return 0.0
    autocovariance /= pair_count
    return autocovariance / variance


def calc_zero_crossing_rate(values, count, center, validity=None):
    """
    Calculate the crossing rate of ``values`` around ``center``.

    Shift and scale invariant when ``center`` tracks the window (median):
    white noise crosses its median almost every sample, while temporally
    coherent motion excursions stay on one side for long runs.
    """
    if count < 2:
        return 0.0
    crossings = 0
    pairs = 0
    for i in range(1, count):
        if validity is not None and not (validity[i - 1] and validity[i]):
            continue
        prev_above = values[i - 1] >= center
        curr_above = values[i] >= center
        if curr_above != prev_above:
            crossings += 1
        pairs += 1
    return crossings / pairs if pairs else 0.0


# Production feature set, and the only one. Every member is scale-invariant:
# a ratio, a correlation, or a crossing rate. That is the point, not a
# coincidence. The int8 CSI scaling factor varies per packet and is never
# recorded, so a feature carrying absolute magnitude carries the link's noise
# floor with it, and on weak links that floor can exceed the motion it is
# meant to measure.
#
# `l1_delta` and `l1_delta_std` were dropped on 2026-07-28 for exactly that.
# See docs/FEATURES.md and docs/adr/2026-08-11-promote-channel-shape-trajectory-ml-features.md:
# adding one
# strong-link capture to training took a weak-link pair from 0% to 100% false
# positives, because its idle displacement (0.2653) sat above its own motion
# (0.1830) and above the added capture's motion (0.0587). The promoted compact
# model keeps only scale-invariant members and the four physical-time
# channel-shape dynamics retained by the 2026-08-16 production decision.
PHASELESS8_FEATURES = [
    'turb_iqr_over_mean_aggr',
    'turb_autocorr',
    'turb_zcr',
    'l1_delta_lag_ratio',
    'chan_shape_spread_subband',
    'chan_shape_coherent_innovation_energy',
    'chan_shape_excess_path',
    'chan_shape_subband_kendall_lag_excess',
]

DEFAULT_FEATURES = PHASELESS8_FEATURES
ALL_FEATURES = tuple(DEFAULT_FEATURES)

AGGREGATED_TURBULENCE_FEATURES = frozenset({
    'turb_iqr_over_mean_aggr',
})

CHANNEL_SHAPE_TRAJECTORY_FEATURES = frozenset({
    'chan_shape_spread_subband',
    'chan_shape_coherent_innovation_energy',
    'chan_shape_excess_path',
    'chan_shape_subband_kendall_lag_excess',
})
# Features that need the L1 profile rings. Retired L1-series statistics live in
# the host-only candidate registry and never allocate a series in production.
L1_TRACKER_FEATURES = frozenset({'l1_delta_lag_ratio'})


class L1DeltaTracker:
    """Allocation-free L1-delta metric tracker without detector surface."""

    def __init__(self, window_size=100, threshold=1.0, lag=L1_DELTA_LAG,
                 allocate_amplitude_buffer=True, enable_hampel=False,
                 hampel_window=7, hampel_threshold=5.0):
        self.window_size = max(2, int(window_size))
        self.threshold = threshold
        self.lag = max(1, int(lag))

        profile_width = SegmentationContext.AMPLITUDE_BUFFER_SIZE
        self._profile_ring = [[0.0] * profile_width for _ in range(self.lag)]
        self._profile_len = [0] * self.lag
        self._profile_index = 0
        self._current_profile = [0.0] * profile_width
        self._amplitude_buffer = (
            [0.0] * profile_width if allocate_amplitude_buffer else None
        )
        self._amplitude_count = 0
        self._delta_ring = [0.0] * self.window_size
        self._delta_valid = [False] * self.window_size
        self._delta_index = 0
        self._delta_count = 0
        self._delta_slots = 0
        self._delta_sum = 0.0
        # Lag-1 displacement over the same window. Both references live in the
        # profile ring already, so the pair costs one extra running sum and no
        # extra normalization.
        self._delta1_ring = [0.0] * self.window_size
        self._delta1_valid = [False] * self.window_size
        self._delta1_index = 0
        self._delta1_count = 0
        self._delta1_slots = 0
        self._delta1_sum = 0.0

        self._packet_count = 0
        self._state = MotionState.IDLE
        self._current_metric = 0.0
        self.last_delta = 0.0
        self._hampel_filter = (
            HampelFilter(hampel_window, hampel_threshold)
            if enable_hampel else None
        )
        # The ratio divides one displacement by the other, so they must be
        # filtered alike: an outlier surviving only in the denominator would
        # depress the ratio and read as less motion.
        self._hampel_filter1 = (
            HampelFilter(hampel_window, hampel_threshold)
            if enable_hampel else None
        )

    def _push_delta(self, delta):
        if self._delta_slots >= self.window_size and self._delta_valid[self._delta_index]:
            self._delta_sum -= self._delta_ring[self._delta_index]
            self._delta_count -= 1
        valid = delta is not None
        self._delta_valid[self._delta_index] = valid
        self._delta_ring[self._delta_index] = 0.0 if delta is None else delta
        if valid:
            self._delta_sum += delta
            self._delta_count += 1
        self._delta_index += 1
        if self._delta_index >= self.window_size:
            self._delta_index = 0
        if self._delta_slots < self.window_size:
            self._delta_slots += 1

    def _push_delta1(self, delta):
        if self._delta1_slots >= self.window_size and self._delta1_valid[self._delta1_index]:
            self._delta1_sum -= self._delta1_ring[self._delta1_index]
            self._delta1_count -= 1
        valid = delta is not None
        self._delta1_valid[self._delta1_index] = valid
        self._delta1_ring[self._delta1_index] = 0.0 if delta is None else delta
        if valid:
            self._delta1_sum += delta
            self._delta1_count += 1
        self._delta1_index += 1
        if self._delta1_index >= self.window_size:
            self._delta1_index = 0
        if self._delta1_slots < self.window_size:
            self._delta1_slots += 1

    def process_packet(self, csi_data, selected_subcarriers=None):
        if self._amplitude_buffer is None:
            raise RuntimeError("amplitude buffer is disabled")
        self._amplitude_count = SegmentationContext._fill_amplitude_buffer(
            csi_data, selected_subcarriers, self._amplitude_buffer
        )
        self.process_amplitudes(self._amplitude_buffer, self._amplitude_count)

    def process_amplitudes(self, amplitudes, amplitude_count, amplitude_mean=None):
        """Update the L1 stream from an already extracted amplitude profile."""
        self._packet_count += 1
        lagged_delta = None
        adjacent_delta = None
        profile = self._current_profile
        ring_slot = self._profile_index
        reference = self._profile_ring[ring_slot]
        reference_len = self._profile_len[ring_slot]
        # The packet before this one is the slot behind the lagged reference,
        # so the lag-1 displacement needs no storage of its own.
        prev_slot = ring_slot - 1 if ring_slot > 0 else self.lag - 1
        previous = self._profile_ring[prev_slot]
        previous_len = self._profile_len[prev_slot]
        profile_len = 0
        if amplitudes is not None and 2 <= amplitude_count <= len(profile):
            mean = amplitude_mean
            if mean is None:
                total = 0.0
                for i in range(amplitude_count):
                    total += amplitudes[i]
                mean = total / amplitude_count
            if mean > 0.0:
                profile_len = amplitude_count
                lagged = reference_len == profile_len
                adjacent = previous_len == profile_len
                if lagged or adjacent:
                    delta_total = 0.0
                    delta1_total = 0.0
                    for i in range(profile_len):
                        value = amplitudes[i] / mean
                        profile[i] = value
                        if lagged:
                            diff = value - reference[i]
                            delta_total += diff if diff >= 0 else -diff
                        if adjacent:
                            diff1 = value - previous[i]
                            delta1_total += diff1 if diff1 >= 0 else -diff1
                    if lagged:
                        self.last_delta = delta_total / profile_len
                        if self._hampel_filter is not None:
                            self.last_delta = self._hampel_filter.filter(self.last_delta)
                        lagged_delta = self.last_delta
                    if adjacent:
                        delta1 = delta1_total / profile_len
                        if self._hampel_filter1 is not None:
                            delta1 = self._hampel_filter1.filter(delta1)
                        adjacent_delta = delta1
                else:
                    for i in range(profile_len):
                        profile[i] = amplitudes[i] / mean

        self._profile_ring[ring_slot] = profile
        self._profile_len[ring_slot] = profile_len
        self._current_profile = reference
        self._profile_index += 1
        if self._profile_index >= self.lag:
            self._profile_index = 0
        self._push_delta(lagged_delta)
        self._push_delta1(adjacent_delta)

    def advance_missing_slots(self, count):
        """Advance lag rings for absent temporal slots without fabricating data."""
        for _ in range(max(0, int(count))):
            self._profile_len[self._profile_index] = 0
            self._profile_index += 1
            if self._profile_index >= self.lag:
                self._profile_index = 0
            self._push_delta(None)
            self._push_delta1(None)

    def delta_lag_ratio(self):
        """Return mean(lag displacement) / mean(lag-1 displacement).

        Noise saturates the displacement immediately, so its ratio sits near
        `1.0`; real channel evolution keeps growing with the lag and lifts it.
        Both terms share the same units, so the ratio drops the noise floor that
        makes the raw mean unusable when the link is weak.
        """
        if self._delta_count == 0 or self._delta1_count == 0:
            return 1.0
        adjacent_mean = self._delta1_sum / self._delta1_count
        if adjacent_mean <= 0.0:
            return 1.0
        return (self._delta_sum / self._delta_count) / adjacent_mean

    def copy_deltas_into(self, out):
        """Copy the current delta window into ``out`` in chronological order."""
        count = min(self._delta_count, len(out))
        if count == 0:
            return 0
        if self._delta_count < self.window_size:
            start = 0
        else:
            start = self._delta_index
        source_index = start
        for i in range(count):
            out[i] = self._delta_ring[source_index]
            source_index += 1
            if source_index >= self.window_size:
                source_index = 0
        return count

    def update_metric(self):
        if self._delta_count >= self.window_size:
            self._current_metric = self._delta_sum / self._delta_count
        else:
            self._current_metric = 0.0
        return self._current_metric

    def mean(self):
        """Return the current delta-window mean without changing detector state."""
        return self._delta_sum / self._delta_count if self._delta_count else 0.0

    def update_state(self):
        metric = self.update_metric()
        if self._state == MotionState.IDLE:
            if metric > self.threshold:
                self._state = MotionState.MOTION
        elif metric < self.threshold:
            self._state = MotionState.IDLE
        return {
            "motion_metric": metric,
            "l1_delta": metric,
            "threshold": self.threshold,
            "state": self._state,
        }

    def set_threshold(self, threshold):
        if 0.0 <= threshold <= 10.0:
            self.threshold = threshold
            return True
        return False

    def set_adaptive_threshold(self, threshold):
        self.threshold = max(1e-6, min(10.0, threshold))

    def is_ready(self):
        return self._delta_count >= self.window_size

    def reset(self):
        for i in range(self.lag):
            self._profile_len[i] = 0
        self._delta_index = 0
        self._delta_count = 0
        self._delta_slots = 0
        self._delta_sum = 0.0
        for i in range(self.window_size):
            self._delta_valid[i] = False
        self._delta1_index = 0
        self._delta1_count = 0
        self._delta1_slots = 0
        self._delta1_sum = 0.0
        for i in range(self.window_size):
            self._delta1_valid[i] = False
        self._amplitude_count = 0
        self._profile_index = 0
        self._packet_count = 0
        self._state = MotionState.IDLE
        self._current_metric = 0.0
        self.last_delta = 0.0
        if self._hampel_filter is not None:
            self._hampel_filter.reset()
        if self._hampel_filter1 is not None:
            self._hampel_filter1.reset()

    def get_state(self):
        return self._state

    def get_motion_metric(self):
        return self._current_metric

    @property
    def count(self):
        return self._delta_count

    @property
    def total_packets(self):
        return self._packet_count


def extract_features_by_name(
    turbulence_buffer,
    buffer_count,
    feature_names=None,
    aggregated_turbulence_buffer=None,
    aggregated_turbulence_count=None,
    out=None,
    reuse_aggregated_turbulence_buffer=False,
    sort_scratch=None,
    l1_delta_lag_ratio=None,
    chan_shape_spread_subband=None,
    chan_shape_coherent_innovation_energy=None,
    chan_shape_excess_path=None,
    chan_shape_subband_kendall_lag_excess=None,
    turbulence_validity=None,
    aggregated_turbulence_validity=None,
):
    """Extract configured features from explicitly preprocessed streams."""
    if feature_names is None:
        feature_names = DEFAULT_FEATURES

    for name in feature_names:
        if name not in ALL_FEATURES:
            raise ValueError(f"Unknown feature: {name}")
    if (
        any(name in AGGREGATED_TURBULENCE_FEATURES for name in feature_names)
        and aggregated_turbulence_buffer is None
    ):
        raise ValueError(
            "aggregated_turbulence_buffer is required when an aggregated "
            "turbulence feature is selected"
        )
    if 'l1_delta_lag_ratio' in feature_names and l1_delta_lag_ratio is None:
        raise ValueError(
            "l1_delta_lag_ratio is required when that feature is selected; "
            "pass the explicitly preprocessed tracker metric"
        )
    required_scalars = {
        'chan_shape_spread_subband': chan_shape_spread_subband,
        'chan_shape_coherent_innovation_energy': (
            chan_shape_coherent_innovation_energy
        ),
        'chan_shape_excess_path': chan_shape_excess_path,
        'chan_shape_subband_kendall_lag_excess': (
            chan_shape_subband_kendall_lag_excess
        ),
    }
    for name in feature_names:
        if name in required_scalars and required_scalars[name] is None:
            raise ValueError(
                f"{name} is required when that feature is selected; "
                "pass the explicitly preprocessed tracker metric"
            )

    if out is not None and len(out) < len(feature_names):
        raise ValueError("Output feature buffer is too small")

    if buffer_count < 2:
        features = out if out is not None else [0.0] * len(feature_names)
        for i in range(len(feature_names)):
            features[i] = 0.0
        return features

    if isinstance(turbulence_buffer, list):
        turb_list = turbulence_buffer if len(turbulence_buffer) == buffer_count else turbulence_buffer[:buffer_count]
    else:
        turb_list = list(turbulence_buffer)[:buffer_count]

    n = len(turb_list)
    valid_values = [
        turb_list[i]
        for i in range(n)
        if turbulence_validity is None or turbulence_validity[i]
    ]
    valid_n = len(valid_values)
    if valid_n < 2:
        features = out if out is not None else [0.0] * len(feature_names)
        for i in range(len(feature_names)):
            features[i] = 0.0
        return features

    turb_mean = sum(valid_values) / valid_n

    var_sum = 0.0
    for value in valid_values:
        diff = value - turb_mean
        var_sum += diff * diff
    turb_var = var_sum / valid_n
    aggregated_iqr_over_mean = None
    if any(name in AGGREGATED_TURBULENCE_FEATURES for name in feature_names):
        aggregated_n = (
            len(aggregated_turbulence_buffer)
            if aggregated_turbulence_count is None
            else min(
                int(aggregated_turbulence_count),
                len(aggregated_turbulence_buffer),
            )
        )
        aggregated_values = [
            aggregated_turbulence_buffer[i]
            for i in range(aggregated_n)
            if (
                aggregated_turbulence_validity is None
                or aggregated_turbulence_validity[i]
            )
        ]
        aggregated_valid_n = len(aggregated_values)
        if aggregated_valid_n < 2:
            aggregated_iqr_over_mean = 0.0
        else:
            aggregated_mean = sum(aggregated_values) / aggregated_valid_n
            aggregated_denom = max(abs(aggregated_mean), 1e-6)
            aggregated_values.sort()
            q25_position = (aggregated_valid_n - 1) * 0.25
            q75_position = (aggregated_valid_n - 1) * 0.75
            q25_lower = int(q25_position)
            q75_lower = int(q75_position)
            q25_fraction = q25_position - q25_lower
            q75_fraction = q75_position - q75_lower
            q25 = (
                aggregated_values[q25_lower] * (1.0 - q25_fraction)
                + aggregated_values[min(q25_lower + 1, aggregated_valid_n - 1)]
                * q25_fraction
            )
            q75 = (
                aggregated_values[q75_lower] * (1.0 - q75_fraction)
                + aggregated_values[min(q75_lower + 1, aggregated_valid_n - 1)]
                * q75_fraction
            )
            aggregated_iqr_over_mean = (
                q75 - q25
            ) / aggregated_denom

    turb_autocorr = None
    turb_zcr = None
    for name in feature_names:
        if name == "turb_autocorr":
            turb_autocorr = calc_autocorrelation(
                turb_list, n, mean=turb_mean, variance=turb_var,
                validity=turbulence_validity,
            )
        elif name == "turb_zcr":
            # Crossing rate needs the time-ordered series; compute it before
            # any in-place sort of the reused turbulence buffer.
            sorted_copy = sorted(valid_values)
            turb_zcr = calc_zero_crossing_rate(
                turb_list, n, sorted_copy[valid_n // 2],
                validity=turbulence_validity,
            )
    features = out if out is not None else []
    for feature_index, name in enumerate(feature_names):
        if name == 'turb_iqr_over_mean_aggr':
            value = aggregated_iqr_over_mean
        elif name == 'turb_autocorr':
            value = turb_autocorr
        elif name == 'turb_zcr':
            value = turb_zcr
        elif name == 'l1_delta_lag_ratio':
            value = l1_delta_lag_ratio
        elif name == 'chan_shape_spread_subband':
            value = chan_shape_spread_subband
        elif name == 'chan_shape_coherent_innovation_energy':
            value = chan_shape_coherent_innovation_energy
        elif name == 'chan_shape_excess_path':
            value = chan_shape_excess_path
        elif name == 'chan_shape_subband_kendall_lag_excess':
            value = chan_shape_subband_kendall_lag_excess
        else:
            raise ValueError(f"Unknown feature: {name}")
        if out is None:
            features.append(value)
        else:
            features[feature_index] = value
    return features


# Alias for backward compatibility
FEATURE_NAMES = DEFAULT_FEATURES
