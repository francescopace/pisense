# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Host-Side Feature Trackers

Shared NumPy-based HT20 feature primitives and trackers used by the host-side
training and evaluation flows. Production and candidate feature registries build
on top of these helpers.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .bootstrap import setup_paths

setup_paths()

from tools.lib.csi_features import L1_DELTA_LAG  # pyright: ignore[reportMissingImports]

# HT20 data and pilot bins in the centered convention the loaders normalize to:
# DC sits at bin 32 and the guard bands at 0..3 and 61..63. Coherence reads the
# whole live band rather than the twelve classic tones, because the delay
# estimate below needs frequency span to be well conditioned.
HT20_LIVE_BINS: Tuple[int, ...] = tuple(range(4, 32)) + tuple(range(33, 61))
HT20_COHERENCE_SUBBANDS: Tuple[Tuple[int, ...], ...] = (
    tuple(range(4, 18)),
    tuple(range(18, 32)),
    tuple(range(33, 47)),
    tuple(range(47, 61)),
)

# Index pairs inside HT20_LIVE_BINS that are genuinely adjacent in frequency.
# The DC gap splits the band, so the pair that straddles it is excluded from the
# delay estimator while both halves still contribute to the coherent sum.
_ADJACENT_PAIRS: Tuple[int, ...] = tuple(
    i for i in range(len(HT20_LIVE_BINS) - 1)
    if HT20_LIVE_BINS[i + 1] - HT20_LIVE_BINS[i] == 1
)

_LIVE_LEFT = [i + 1 for i in _ADJACENT_PAIRS]
_LIVE_RIGHT = list(_ADJACENT_PAIRS)
_BIN_INDEX = np.asarray(HT20_LIVE_BINS, dtype=np.float64)
_SUBBAND_PROFILE_INDICES: Tuple[np.ndarray, ...] = tuple(
    np.asarray([HT20_LIVE_BINS.index(sc) for sc in subband], dtype=np.intp)
    for subband in HT20_COHERENCE_SUBBANDS
)
_SUBBAND_BIN_INDICES: Tuple[np.ndarray, ...] = tuple(
    np.asarray(subband, dtype=np.float64)
    for subband in HT20_COHERENCE_SUBBANDS
)
# Each subband is contiguous in profile index, and together they tile the live
# band, so a shared cross-product array can be sliced by span instead of gathered
# by index.
_SUBBAND_SPANS: Tuple[Tuple[int, int], ...] = tuple(
    (int(indices[0]), int(indices[-1]) + 1) for indices in _SUBBAND_PROFILE_INDICES
)

COHERENCE_GAP_LOW_THRESHOLD = 0.02

CHANNEL_COHERENCE_FEATURES = (
    'chan_coh_lag_ratio',
    'chan_coh_mean',
    'chan_coh_gap',
    'chan_coh_gap_low_frac',
    'chan_coh_gap_q20',
    'chan_coh_subband_gap_median',
    'chan_coh_subband_median_gap',
)
SUBBAND_COHERENCE_FEATURES = (
    'chan_coh_subband_gap_median',
    'chan_coh_subband_median_gap',
)
SPECTRAL_FEATURES = (
    'turb_band_power_ratio',
    'turb_cv',
    'turb_mad_over_mean',
    'turb_iqr_over_mean',
    'turb_p95_over_mean',
    'turb_p05_over_mean',
    'turb_max_over_mean',
    'turb_min_over_mean',
    'turb_range_over_mean',
    'turb_peak_over_mad',
    'waveform_length_over_mean',
    'turb_skewness',
)
AGGREGATED_SPECTRAL_FEATURES = (
    'turb_mad_over_mean_aggr',
    'turb_p95_over_mean_aggr',
    'turb_iqr_over_mean_aggr_tone_detrended',
)
AMPLITUDE_PROFILE_FEATURES = (
    'corr_amp_d1',
)
PHASE_FEATURES = (
    'phase_resid_lag_ratio',
    'phase_closure_var_std',
)
CHANNEL_SHAPE_FEATURES = (
    'chan_shape_lag_ratio',
    'chan_rank_gap',
    'chan_ratio_gap',
    'chan_shape_spread_ds2',
    'chan_shape_spread_ema_fast',
    'chan_shape_spread_ema_slow',
    'chan_freq_coh_cv',
    'chan_freq_coh_curve_iqr',
    'chan_freq_coh_curve_2_4_std',
    'chan_freq_coh_curve_4_12_std',
    'chan_freq_coh_decay_std',
    'chan_freq_coh_curvature_std',
)
L1_SERIES_FEATURES: Tuple[str, ...] = ()
CHANNEL_SHAPE_TRAJECTORY_FEATURES = (
    'chan_shape_scale_curvature',
    'chan_shape_coherent_innovation_contrast',
    'chan_shape_subband_kendall_lag_excess',
    'chan_shape_subband_rank_gap',
    'chan_shape_spread_subband',
    'chan_shape_coherent_innovation_energy',
    'chan_shape_excess_path',
)
PROMOTED_CHANNEL_SHAPE_TRAJECTORY_FEATURES = (
    'chan_shape_spread_subband',
    'chan_shape_coherent_innovation_energy',
    'chan_shape_excess_path',
    'chan_shape_subband_kendall_lag_excess',
)
PROMOTED_CHANNEL_SHAPE_FEATURES = (
    'chan_freq_coh_curve_std',
)
CLASSIC_ONLY_CHANNEL_SHAPE_FEATURES = (
    'chan_freq_coh_curve_std',
)
COMPOSITE_FEATURES = (
    'chan_coh_gap_spread',
)
CANDIDATE_FEATURES: Tuple[str, ...] = (
    CHANNEL_COHERENCE_FEATURES
    + SPECTRAL_FEATURES
    + AGGREGATED_SPECTRAL_FEATURES
    + PHASE_FEATURES
    + CHANNEL_SHAPE_FEATURES
    + CLASSIC_ONLY_CHANNEL_SHAPE_FEATURES
    + L1_SERIES_FEATURES
    + tuple(
        name for name in CHANNEL_SHAPE_TRAJECTORY_FEATURES
        if name not in PROMOTED_CHANNEL_SHAPE_TRAJECTORY_FEATURES
    )
    + AMPLITUDE_PROFILE_FEATURES
    + COMPOSITE_FEATURES
)

CHANNEL_SHAPE_SUBBAND_COUNT = 8
CHANNEL_SHAPE_BIN_US = 80_000
CHANNEL_SHAPE_KENDALL_RELATIVE_DEADBAND = 0.02
CHANNEL_SHAPE_KENDALL_MIN_COMPARABLE_PAIRS = 8
_CHANNEL_SHAPE_SUBBAND_WIDTH = (
    len(HT20_LIVE_BINS) // CHANNEL_SHAPE_SUBBAND_COUNT
)
_CHANNEL_SHAPE_DCT = np.sqrt(2.0 / CHANNEL_SHAPE_SUBBAND_COUNT) * np.cos(
    np.pi
    / CHANNEL_SHAPE_SUBBAND_COUNT
    * (
        np.arange(CHANNEL_SHAPE_SUBBAND_COUNT, dtype=np.float64)[:, None]
        + 0.5
    )
    * np.arange(CHANNEL_SHAPE_SUBBAND_COUNT, dtype=np.float64)[None, :]
)
_CHANNEL_SHAPE_DCT[:, 0] /= np.sqrt(2.0)


def complex_profile(csi_data, out=None) -> np.ndarray:
    """Return the complex channel over the live band.

    Payload layout matches `SegmentationContext._fill_amplitude_buffer`: each
    subcarrier is an ``(imag, real)`` int8 pair.
    """
    if csi_data is None:
        return np.zeros(len(HT20_LIVE_BINS), dtype=np.complex128)
    raw = np.asarray(csi_data, dtype=np.int8)
    if raw.ndim != 1 or raw.size < 2 * (HT20_LIVE_BINS[-1] + 1):
        return np.zeros(len(HT20_LIVE_BINS), dtype=np.complex128)
    imag = raw[0::2][list(HT20_LIVE_BINS)].astype(np.float64)
    real = raw[1::2][list(HT20_LIVE_BINS)].astype(np.float64)
    if out is None:
        return real + 1j * imag
    out[:] = real + 1j * imag
    return out


def delay_compensated_coherence(current: np.ndarray, reference: np.ndarray) -> float:
    """Coherence between two complex profiles, with the packet delay removed.

    ``|sum_k c_k e^{-j k d}| / sum_k |c_k|`` for ``c_k = H_k current * conj(H_k
    reference)``, with ``d`` estimated from the products of adjacent bins.

    The magnitude of the sum drops any common phase, so a per-packet carrier
    offset cancels; ``d`` absorbs the sampling-time offset that would otherwise
    read as decorrelation. Numerator and denominator are both quadratic in the
    channel, so the unrecorded int8 scaling factor cancels and the result stays
    in ``[0, 1]``: near 1 when the channel is unchanged, lower as movement
    decorrelates it.
    """
    cross, magnitude = coherence_cross(current, reference)
    return delay_compensated_coherence_from_cross(cross, magnitude)


def coherence_cross(
    current: np.ndarray,
    reference: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return the cross products of one reference and their magnitudes.

    The four subbands tile the live band exactly, so the full-band and subband
    coherences of a reference read the very same products. Building them once
    lets both consumers share the arrays instead of each recomputing all 56.
    """
    cross = current * np.conj(reference)
    return cross, np.abs(cross)


def delay_compensated_coherence_from_cross(
    cross: np.ndarray,
    magnitude: np.ndarray,
) -> float:
    """Full live-band coherence from cached cross products."""
    total = magnitude.sum()
    if total <= 0.0:
        return 0.0
    ramp = np.angle(np.sum(cross[_LIVE_LEFT] * np.conj(cross[_LIVE_RIGHT])))
    aligned = cross * np.exp(-1j * ramp * _BIN_INDEX)
    return float(abs(aligned.sum()) / total)


def _subband_coherence_from_cross(
    cross: np.ndarray,
    magnitude: np.ndarray,
    start: int,
    stop: int,
    bin_index: np.ndarray,
) -> float:
    """Delay-compensated coherence for one contiguous band of the shared array."""
    band = cross[start:stop]
    total = magnitude[start:stop].sum()
    if total <= 0.0:
        return 0.0
    ramp = np.angle(np.sum(band[1:] * np.conj(band[:-1])))
    aligned = band * np.exp(-1j * ramp * bin_index)
    return float(abs(aligned.sum()) / total)


def subband_coherences_from_cross(
    cross: np.ndarray,
    magnitude: np.ndarray,
    out: np.ndarray = None,
) -> np.ndarray:
    """Fill the four subband coherences from cached cross products."""
    values = np.empty(len(_SUBBAND_SPANS), dtype=np.float64) if out is None else out
    for i, (start, stop) in enumerate(_SUBBAND_SPANS):
        values[i] = _subband_coherence_from_cross(
            cross, magnitude, start, stop, _SUBBAND_BIN_INDICES[i]
        )
    return values


def subband_coherences(current: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Return delay-compensated coherence for four contiguous HT20 bands."""
    cross, magnitude = coherence_cross(current, reference)
    return subband_coherences_from_cross(cross, magnitude)


def turbulence_band_power_ratio(series: Sequence[float]) -> float:
    """Fraction of non-DC turbulence power below 0.1 cycles per sample.

    At the nominal 100 pps capture rate this is the historical 0.5-10 Hz
    movement band, rounded to the FFT bins available in the live window. The
    numerator and denominator scale quadratically together, so the statistic
    is invariant to the magnitude of the input series.
    """
    values = np.asarray(series, dtype=np.float64)
    if values.size < 4:
        return 0.0
    centered = values - float(np.mean(values))
    power = np.abs(np.fft.rfft(centered * np.hanning(values.size))) ** 2
    total = float(power[1:].sum())
    if total <= 0.0:
        return 0.0
    upper_bin = max(1, min(len(power) - 1, int(np.floor(values.size * 0.10))))
    return float(power[1:upper_bin + 1].sum() / total)


def sanitized_phase_profile(profile: np.ndarray) -> np.ndarray:
    """Return adjacent-bin phase residuals with CFO and STO removed.

    Adjacent-bin products cancel the common phase offset from CFO. A sampling
    delay becomes one common rotation across those products, which is removed
    by their circular mean. The remaining unit phasors describe only the
    frequency-selective phase shape, without using amplitude.
    """
    adjacent = profile[_LIVE_LEFT] * np.conj(profile[_LIVE_RIGHT])
    magnitude = np.abs(adjacent)
    residual = np.zeros(len(adjacent), dtype=np.complex128)
    valid = magnitude > 0.0
    residual[valid] = adjacent[valid] / magnitude[valid]
    common = residual[valid].sum()
    if abs(common) > 0.0:
        residual[valid] *= np.conj(common) / abs(common)
    return residual


def phase_profile_distance(current: np.ndarray, reference: np.ndarray) -> float:
    """Mean wrapped displacement between two sanitized phase profiles."""
    valid = (np.abs(current) > 0.0) & (np.abs(reference) > 0.0)
    if not np.any(valid):
        return 0.0
    delta = current[valid] * np.conj(reference[valid])
    return float(np.mean(np.abs(np.angle(delta))) / np.pi)


_PHASE_CLOSURE_TRIPLETS: Tuple[Tuple[int, int, int], ...] = tuple(
    (index - 1, index, index + 1)
    for index in range(1, len(HT20_LIVE_BINS) - 1)
    if (
        HT20_LIVE_BINS[index] - HT20_LIVE_BINS[index - 1] == 1
        and HT20_LIVE_BINS[index + 1] - HT20_LIVE_BINS[index] == 1
    )
)
_PHASE_CLOSURE_LEFT = np.asarray(
    [left for left, _, _ in _PHASE_CLOSURE_TRIPLETS],
    dtype=np.intp,
)
_PHASE_CLOSURE_CENTER = np.asarray(
    [center for _, center, _ in _PHASE_CLOSURE_TRIPLETS],
    dtype=np.intp,
)
_PHASE_CLOSURE_RIGHT = np.asarray(
    [right for _, _, right in _PHASE_CLOSURE_TRIPLETS],
    dtype=np.intp,
)


def local_phase_closure_variance(
    profile: np.ndarray,
    relative_floor: float = 0.02,
) -> float:
    """Circular variance of local phase curvature across frequency.

    ``angle(H[k-1] H[k+1] conj(H[k])^2)`` is the second phase difference, so
    common phase and a linear phase ramp cancel exactly. Triplets containing a
    bin below the packet-relative floor are excluded.
    """
    profile = np.asarray(profile, dtype=np.complex128)
    if profile.size != len(HT20_LIVE_BINS):
        return 0.0
    amplitude = np.abs(profile)
    maximum = float(np.max(amplitude))
    if maximum <= 0.0:
        return 0.0
    threshold = max(0.0, float(relative_floor)) * maximum
    valid = (
        (amplitude[_PHASE_CLOSURE_LEFT] > threshold)
        & (amplitude[_PHASE_CLOSURE_CENTER] > threshold)
        & (amplitude[_PHASE_CLOSURE_RIGHT] > threshold)
    )
    if np.count_nonzero(valid) < 4:
        return 0.0
    closure = (
        profile[_PHASE_CLOSURE_LEFT[valid]]
        * profile[_PHASE_CLOSURE_RIGHT[valid]]
        * np.conj(profile[_PHASE_CLOSURE_CENTER[valid]]) ** 2
    )
    magnitude = np.abs(closure)
    nonzero = magnitude > 0.0
    if np.count_nonzero(nonzero) < 4:
        return 0.0
    unit = closure[nonzero] / magnitude[nonzero]
    return float(1.0 - abs(np.mean(unit)))


def normalized_amplitude_profile(profile: np.ndarray) -> np.ndarray:
    """Return the channel magnitude divided by its packet L2 norm."""
    amplitude = np.abs(profile)
    norm = float(np.linalg.norm(amplitude))
    if norm <= 0.0:
        return np.zeros(len(amplitude), dtype=np.float64)
    return amplitude / norm


def hellinger_subband_profile(profile: np.ndarray) -> np.ndarray:
    """Return the gain-invariant Hellinger energy profile of eight subbands."""
    amplitude = np.abs(np.asarray(profile, dtype=np.complex128))
    if amplitude.shape != (len(HT20_LIVE_BINS),):
        return np.zeros(CHANNEL_SHAPE_SUBBAND_COUNT, dtype=np.float64)
    energy = amplitude * amplitude
    total = float(np.sum(energy))
    if total <= 0.0:
        return np.zeros(CHANNEL_SHAPE_SUBBAND_COUNT, dtype=np.float64)
    subband_energy = energy.reshape(
        CHANNEL_SHAPE_SUBBAND_COUNT,
        _CHANNEL_SHAPE_SUBBAND_WIDTH,
    ).sum(axis=1)
    return np.sqrt(subband_energy / total)


class ChannelShapeTrajectoryTracker:
    """Track physical-time geometry of the Hellinger channel profile.

    Packet profiles are reduced to eight contiguous live-band subbands, then
    grouped in fixed physical-time bins using component-wise medians. Exact
    consecutive duplicates are discarded and absent bins are left absent.
    """

    def __init__(
        self,
        window_duration_us: int = 1_000_000,
        bin_us: int = CHANNEL_SHAPE_BIN_US,
        track_subband_rank_gap: bool = False,
        track_subband_kendall_lag_excess: bool = False,
    ):
        self.window_duration_us = max(3, int(window_duration_us))
        self.bin_us = max(1, int(bin_us))
        self.track_subband_rank_gap = bool(track_subband_rank_gap)
        self.track_subband_kendall_lag_excess = bool(
            track_subband_kendall_lag_excess
        )
        self._window_bins = max(
            3,
            (self.window_duration_us + self.bin_us - 1) // self.bin_us,
        )
        self.reset()

    @staticmethod
    def _median_profile(profiles: Sequence[np.ndarray]) -> np.ndarray:
        median = np.median(np.asarray(profiles, dtype=np.float64), axis=0)
        norm = float(np.linalg.norm(median))
        return median / norm if norm > 0.0 else np.zeros_like(median)

    def _finalize_current_bin(self) -> None:
        if self._current_bin is None or not self._current_profiles:
            return
        profile = self._median_profile(self._current_profiles)
        modes = profile @ _CHANNEL_SHAPE_DCT
        self._bins.append(
            (
                self._current_bin,
                modes,
            )
        )
        if self.track_subband_kendall_lag_excess:
            self._subband_kendall_signatures[self._current_bin] = (
                guarded_kendall_signature(profile)
            )
        if self.track_subband_rank_gap:
            profiles = {
                index: stored_modes @ _CHANNEL_SHAPE_DCT.T
                for index, stored_modes in self._bins
            }
            for lag_bins in (1, 3):
                reference = profiles.get(self._current_bin - lag_bins)
                if reference is not None:
                    self._subband_rank_distances[
                        (self._current_bin, lag_bins)
                    ] = rank_profile_distance(profile, reference)

    def _trim(self, current_bin: int) -> None:
        first_bin = int(current_bin) - self._window_bins + 1
        while self._bins and self._bins[0][0] < first_bin:
            del self._bins[0]
        if self.track_subband_rank_gap:
            self._subband_rank_distances = {
                (index, lag_bins): distance
                for (index, lag_bins), distance
                in self._subband_rank_distances.items()
                if index - lag_bins >= first_bin
            }
        if self.track_subband_kendall_lag_excess:
            self._subband_kendall_signatures = {
                index: signature
                for index, signature in self._subband_kendall_signatures.items()
                if index >= first_bin
            }

    def process_packet(self, csi_data, timestamp_us: int) -> None:
        """Consume one CSI payload at its monotonic physical timestamp."""
        raw = np.asarray(csi_data, dtype=np.int8)
        bin_index = max(0, int(timestamp_us)) // self.bin_us
        if self._current_bin is None:
            self._current_bin = bin_index
        elif bin_index != self._current_bin:
            self._finalize_current_bin()
            self._current_bin = bin_index
            self._current_profiles = []
            self._trim(bin_index)
        # Duplicate payloads still advance the physical-time window.
        if self._previous_raw is not None and np.array_equal(
            raw,
            self._previous_raw,
        ):
            return
        self._previous_raw = raw.copy()
        profile = hellinger_subband_profile(complex_profile(raw))
        self._current_profiles.append(profile)

    def _binned_path(self) -> List[Tuple[int, np.ndarray]]:
        """Return the physical-time path in cached orthonormal DCT space."""
        path = list(self._bins)
        if self._current_profiles:
            profile = self._median_profile(self._current_profiles)
            path.append(
                (
                    self._current_bin,
                    profile @ _CHANNEL_SHAPE_DCT,
                )
            )
        return path

    def _distance_at_bin_lag(self, lag_bins: int) -> float:
        path = self._binned_path()
        profiles = {index: profile for index, profile in path}
        distances = [
            float(np.linalg.norm(profile - profiles[index - lag_bins]))
            for index, profile in path
            if index - lag_bins in profiles
        ]
        return float(np.median(distances)) if distances else 0.0

    def scale_curvature(self, epsilon: float = 1e-6) -> float:
        """Log-distance curvature at physical lags 80, 240, and 720 ms."""
        short = self._distance_at_bin_lag(1)
        middle = self._distance_at_bin_lag(3)
        long = self._distance_at_bin_lag(9)
        if min(short, middle, long) <= 0.0:
            return 0.0
        floor = max(float(epsilon), 1e-12)
        return float(
            2.0 * np.log(middle + floor)
            - np.log(short + floor)
            - np.log(long + floor)
        )

    def subband_rank_gap(self) -> float:
        """Ordinal profile turnover at 240 ms minus turnover at 80 ms.

        The eight Hellinger subbands already retained by the trajectory are
        ranked after removing values below two percent of either profile
        maximum. Exact physical-bin lags keep the statistic independent of
        packet cadence, while Spearman distance removes positive gain. Cached
        finalized-bin distances leave only the changing current bin to compare
        during extraction.
        """
        if not self.track_subband_rank_gap:
            raise ValueError(
                "subband rank gap tracking was not enabled at construction"
            )

        def distances(lag_bins: int) -> List[float]:
            values = [
                distance
                for (index, cached_lag), distance
                in self._subband_rank_distances.items()
                if cached_lag == lag_bins
            ]
            if self._current_bin is None or not self._current_profiles:
                return values
            references = dict(self._bins)
            reference_modes = references.get(self._current_bin - lag_bins)
            if reference_modes is None:
                return values
            profile = self._median_profile(self._current_profiles)
            reference = reference_modes @ _CHANNEL_SHAPE_DCT.T
            values.append(rank_profile_distance(profile, reference))
            return values

        adjacent = distances(1)
        longer = distances(3)
        if not adjacent or not longer:
            return 0.0
        return float(np.median(longer) - np.median(adjacent))

    def subband_kendall_lag_excess(self) -> float:
        """Guarded ordinal turnover beyond three local 80 ms steps.

        Each eight-subband profile becomes two 28-bit pairwise-order masks.
        Pairs separated by no more than two percent of the profile maximum are
        treated as ties. For every complete four-bin path, the 240 ms Kendall
        distance is compared with the mean of its three constituent 80 ms
        distances. The median positive excess rejects local ordinal jitter and
        retains only accumulated frequency-selective turnover.
        """
        if not self.track_subband_kendall_lag_excess:
            raise ValueError(
                "subband Kendall lag-excess tracking was not enabled at "
                "construction"
            )
        signatures = dict(self._subband_kendall_signatures)
        if self._current_bin is not None and self._current_profiles:
            profile = self._median_profile(self._current_profiles)
            signatures[self._current_bin] = guarded_kendall_signature(profile)
        samples = []
        for index in sorted(signatures):
            required = tuple(index - lag for lag in range(4))
            if not all(required_index in signatures for required_index in required):
                continue
            long_distance = guarded_kendall_distance(
                signatures[index], signatures[index - 3]
            )
            local_distances = [
                guarded_kendall_distance(
                    signatures[local_index],
                    signatures[local_index - 1],
                )
                for local_index in range(index - 2, index + 1)
            ]
            if long_distance is None or any(
                distance is None for distance in local_distances
            ):
                continue
            samples.append(
                max(0.0, long_distance - float(np.mean(local_distances)))
            )
        return float(np.median(samples)) if samples else 0.0

    def trajectory_features_with_spread(
        self,
        high_order_weight: float = 1.0,
    ) -> Tuple[float, float, float]:
        """Return innovation, excess path, and subband motion participation.

        Finalized bins use the same cached DCT representation as the device
        runtimes. Full-profile distances stay in mode space through Parseval;
        only the per-subband spread reconstructs adjacent profile deltas with
        the orthonormal inverse transform.
        """
        path = self._binned_path()
        if len(path) < 2:
            return 0.0, 0.0, 0.0

        spread_energy = np.zeros(CHANNEL_SHAPE_SUBBAND_COUNT, dtype=np.float64)
        for (previous_bin, previous), (current_bin, current) in zip(
            path,
            path[1:],
            strict=False,
        ):
            if current_bin - previous_bin != 1:
                continue
            delta_profile = (current - previous) @ _CHANNEL_SHAPE_DCT.T
            spread_energy += delta_profile * delta_profile

        innovation_samples = []
        excess_samples = []
        for (first_bin, first_modes), (middle_bin, middle_modes), (
            last_bin,
            last_modes,
        ) in zip(
            path,
            path[1:],
            path[2:],
            strict=False,
        ):
            previous_dt = middle_bin - first_bin
            current_dt = last_bin - middle_bin
            if previous_dt <= 0 or current_dt <= 0:
                continue
            prediction_modes = middle_modes + (
                float(current_dt) / float(previous_dt)
            ) * (middle_modes - first_modes)
            residual_modes = last_modes - prediction_modes
            low_energy = float(
                np.dot(residual_modes[1:4], residual_modes[1:4])
            )
            high_energy = float(
                np.dot(residual_modes[4:], residual_modes[4:])
            )
            innovation_samples.append(
                max(
                    0.0,
                    low_energy - max(0.0, float(high_order_weight)) * high_energy,
                )
            )
            first = middle_modes - first_modes
            second = last_modes - middle_modes
            chord = last_modes - first_modes
            raw_excess = (
                float(np.linalg.norm(first))
                + float(np.linalg.norm(second))
                - float(np.linalg.norm(chord))
            )
            high_excess = (
                float(np.linalg.norm(first[4:]))
                + float(np.linalg.norm(second[4:]))
                - float(np.linalg.norm(chord[4:]))
            )
            excess_samples.append(
                max(0.0, raw_excess - max(0.0, high_excess))
            )

        innovation = (
            float(np.median(innovation_samples))
            if innovation_samples else 0.0
        )
        excess = (
            float(np.median(excess_samples)) if excess_samples else 0.0
        )
        return innovation, excess, motion_participation(spread_energy)

    def coherent_innovation_energy(self) -> float:
        return self.trajectory_features_with_spread()[0]

    def coherent_innovation_contrast(self, epsilon: float = 1e-12) -> float:
        """Return coherent residual energy relative to high-order noise.

        Low DCT modes represent broad channel-shape motion, while high modes
        provide a causal within-window noise reference. The bounded contrast
        stays at zero when high-order residual energy matches or exceeds the
        coherent residual, and approaches one when the residual is dominated
        by broad channel-shape motion.
        """
        path = self._binned_path()
        samples = []
        floor = max(float(epsilon), 1e-18)
        for (first_bin, first_modes), (middle_bin, middle_modes), (
            last_bin,
            last_modes,
        ) in zip(path, path[1:], path[2:], strict=False):
            previous_dt = middle_bin - first_bin
            current_dt = last_bin - middle_bin
            if previous_dt <= 0 or current_dt <= 0:
                continue
            prediction_modes = middle_modes + (
                float(current_dt) / float(previous_dt)
            ) * (middle_modes - first_modes)
            residual_modes = last_modes - prediction_modes
            low_energy = float(
                np.dot(residual_modes[1:4], residual_modes[1:4])
            )
            high_energy = float(
                np.dot(residual_modes[4:], residual_modes[4:])
            )
            total_energy = low_energy + high_energy
            samples.append(
                max(0.0, low_energy - high_energy)
                / max(total_energy, floor)
            )
        return float(np.median(samples)) if samples else 0.0

    def excess_path(self) -> float:
        return self.trajectory_features_with_spread()[1]

    def shape_spread_subband(self) -> float:
        """Participation of adjacent-bin motion energy across eight subbands.

        This candidate reuses the physical-time profiles already retained for
        the promoted trajectory features. Missing bins are skipped rather than
        turning a longer interval into an apparent 80 ms displacement.
        """
        return self.trajectory_features_with_spread()[2]

    def reset(self) -> None:
        self._bins: List[Tuple[int, np.ndarray]] = []
        self._current_bin = None
        self._current_profiles: List[np.ndarray] = []
        self._previous_raw = None
        self._subband_rank_distances: Dict[Tuple[int, int], float] = {}
        self._subband_kendall_signatures: Dict[int, Tuple[int, int]] = {}


# Compatibility name retained for experiments created before the tracker grew
# the scale-curvature and coherent-innovation readouts.
ChannelShapeExcessPathTracker = ChannelShapeTrajectoryTracker


class AmplitudeProfileTracker:
    """Retain scale-free twelve-tone profiles for host-only candidates."""

    def __init__(self, window_size: int = 100):
        self.window_size = max(2, int(window_size))
        self.reset()

    @staticmethod
    def _normalized_profile(amplitudes) -> np.ndarray:
        profile = np.asarray(amplitudes, dtype=np.float64)
        if profile.ndim != 1 or profile.size < 2:
            return np.zeros(0, dtype=np.float64)
        mean = float(np.mean(profile))
        if mean <= 0.0:
            return np.zeros_like(profile)
        return profile / mean

    def process_amplitudes(
        self,
        amplitudes,
        aggregated_amplitudes=None,
    ) -> None:
        profile = self._normalized_profile(amplitudes)
        if profile.size:
            self._profiles.append(profile)
            if len(self._profiles) > self.window_size:
                del self._profiles[0]
        if aggregated_amplitudes is not None:
            aggregated = self._normalized_profile(aggregated_amplitudes)
            if aggregated.size:
                self._aggregated_profiles.append(aggregated)
                if len(self._aggregated_profiles) > self.window_size:
                    del self._aggregated_profiles[0]

    def adjacent_amplitude_correlation(self) -> float:
        """Mean Pearson correlation of consecutive amplitude profiles."""
        if len(self._profiles) < 2:
            return 0.0
        values = []
        for previous, current in zip(self._profiles, self._profiles[1:], strict=False):
            if previous.shape != current.shape:
                continue
            previous_centered = previous - float(np.mean(previous))
            current_centered = current - float(np.mean(current))
            denominator = float(
                np.linalg.norm(previous_centered)
                * np.linalg.norm(current_centered)
            )
            values.append(
                float(np.dot(previous_centered, current_centered) / denominator)
                if denominator > 0.0 else 0.0
            )
        return float(np.mean(values)) if values else 0.0

    def tone_detrended_aggregated_iqr(self) -> float:
        """Relative turbulence IQR after detrending each aggregated tone."""
        if len(self._aggregated_profiles) < 4:
            return 0.0
        matrix = np.asarray(self._aggregated_profiles, dtype=np.float64)
        time = np.arange(len(matrix), dtype=np.float64)
        time -= float(np.mean(time))
        denominator = float(np.dot(time, time))
        if denominator <= 0.0:
            return 0.0
        centered = matrix - np.mean(matrix, axis=0, keepdims=True)
        slopes = (time[:, None] * centered).sum(axis=0) / denominator
        detrended = matrix - time[:, None] * slopes[None, :]
        means = np.mean(detrended, axis=1)
        turbulence = np.divide(
            np.std(detrended, axis=1),
            means,
            out=np.zeros(len(detrended), dtype=np.float64),
            where=means > 0.0,
        )
        mean = float(np.mean(turbulence))
        if abs(mean) <= 1e-6:
            return 0.0
        q25, q75 = np.percentile(turbulence, [25, 75])
        return float((q75 - q25) / abs(mean))

    def reset(self) -> None:
        self._profiles: List[np.ndarray] = []
        self._aggregated_profiles: List[np.ndarray] = []


def _average_ranks(values: np.ndarray) -> np.ndarray:
    """Return zero-based ranks, averaging positions occupied by ties."""
    values = np.asarray(values, dtype=np.float64)
    order = np.argsort(values, kind='stable')
    sorted_values = values[order]
    ranks = np.empty(values.size, dtype=np.float64)
    start = 0
    while start < values.size:
        end = start + 1
        while end < values.size and sorted_values[end] == sorted_values[start]:
            end += 1
        ranks[order[start:end]] = 0.5 * (start + end - 1)
        start = end
    return ranks


def guarded_kendall_signature(
    profile: np.ndarray,
    relative_deadband: float = CHANNEL_SHAPE_KENDALL_RELATIVE_DEADBAND,
) -> Tuple[int, int]:
    """Encode material pairwise subband ordering as two bit masks."""
    values = np.asarray(profile, dtype=np.float64)
    if values.shape != (CHANNEL_SHAPE_SUBBAND_COUNT,):
        return 0, 0
    maximum = float(np.max(values))
    if maximum <= 0.0:
        return 0, 0
    threshold = max(0.0, float(relative_deadband)) * maximum
    order_mask = 0
    valid_mask = 0
    bit = 0
    for left in range(CHANNEL_SHAPE_SUBBAND_COUNT - 1):
        for right in range(left + 1, CHANNEL_SHAPE_SUBBAND_COUNT):
            difference = float(values[left] - values[right])
            flag = 1 << bit
            if abs(difference) > threshold:
                valid_mask |= flag
                if difference > 0.0:
                    order_mask |= flag
            bit += 1
    return order_mask, valid_mask


def guarded_kendall_distance(
    current: Tuple[int, int],
    reference: Tuple[int, int],
    min_comparable_pairs: int = CHANNEL_SHAPE_KENDALL_MIN_COMPARABLE_PAIRS,
) -> Optional[float]:
    """Return normalized discordance over materially ordered common pairs."""
    current_order, current_valid = current
    reference_order, reference_valid = reference
    common = int(current_valid) & int(reference_valid)
    comparable = common.bit_count()
    if comparable < max(1, int(min_comparable_pairs)):
        return None
    discordant = ((int(current_order) ^ int(reference_order)) & common).bit_count()
    return float(discordant) / float(comparable)


def rank_profile_distance(
    current: np.ndarray,
    reference: np.ndarray,
    relative_floor: float = 0.02,
) -> float:
    """Return bounded Spearman distance between two amplitude profiles.

    Bins below ``relative_floor`` times either packet maximum are excluded from
    that comparison. This prevents quantization around null tones from creating
    arbitrary rank turnover. The remaining ranks are invariant to any positive
    packet gain; ``(1 - rho) / 2`` maps correlation to ``[0, 1]``.
    """
    current = np.asarray(current, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    if current.shape != reference.shape or current.size == 0:
        return 0.0
    current_max = float(np.max(current))
    reference_max = float(np.max(reference))
    if current_max <= 0.0 or reference_max <= 0.0:
        return 0.0
    floor = max(0.0, float(relative_floor))
    valid = (
        (current > floor * current_max)
        & (reference > floor * reference_max)
    )
    if np.count_nonzero(valid) < 4:
        return 0.0
    current_ranks = _average_ranks(current[valid])
    reference_ranks = _average_ranks(reference[valid])
    current_ranks -= float(np.mean(current_ranks))
    reference_ranks -= float(np.mean(reference_ranks))
    denominator = float(
        np.linalg.norm(current_ranks) * np.linalg.norm(reference_ranks)
    )
    if denominator <= 0.0:
        return 0.0
    rho = float(np.dot(current_ranks, reference_ranks) / denominator)
    return 0.5 * (1.0 - float(np.clip(rho, -1.0, 1.0)))


def motion_participation(energy: np.ndarray) -> float:
    """Normalized participation ratio of motion energy across subcarriers.

    A value near ``1 / K`` means one live subcarrier dominates the change,
    while one means that the change is spread uniformly over the live band.
    """
    values = np.asarray(energy, dtype=np.float64)
    total = float(values.sum())
    squared = float(np.dot(values, values))
    if total <= 0.0 or squared <= 0.0:
        return 0.0
    return total * total / (values.size * squared)


def _frequency_coherence_pairs(offset: int) -> Tuple[Tuple[int, int], ...]:
    return tuple(
        (left, right)
        for left, left_bin in enumerate(HT20_LIVE_BINS)
        for right, right_bin in enumerate(HT20_LIVE_BINS)
        if right_bin - left_bin == offset
        and not (left_bin < 32 < right_bin)
    )


# `cross_subcarrier_ratio_distance` keeps a materialized pair table because it
# masks pair by pair. Frequency coherence reads the band through the halves
# below instead, so only the offset it uses is tabulated here.
_FREQUENCY_COHERENCE_LEFT = np.asarray(
    [left for left, _ in _frequency_coherence_pairs(4)],
    dtype=np.intp,
)
_FREQUENCY_COHERENCE_RIGHT = np.asarray(
    [right for _, right in _frequency_coherence_pairs(4)],
    dtype=np.intp,
)

FREQUENCY_COHERENCE_OFFSETS: Tuple[int, ...] = (2, 4, 12)
# The DC null splits the live band into two equal runs that are contiguous in
# both bin number and profile index, so a pair separated by `offset` bins is
# always `left + offset` inside one run. Coherence reshapes the band into those
# runs, which keeps it on views rather than the temporary arrays fancy indexing
# builds on every call.
_LIVE_BAND_SPLIT = next(
    (
        i
        for i in range(1, len(HT20_LIVE_BINS))
        if HT20_LIVE_BINS[i] - HT20_LIVE_BINS[i - 1] != 1
    ),
    len(HT20_LIVE_BINS),
)
_LIVE_BAND_HALVES = len(HT20_LIVE_BINS) // _LIVE_BAND_SPLIT


def cross_subcarrier_ratio_distance(
    current: np.ndarray,
    reference: np.ndarray,
    relative_floor: float = 0.02,
) -> float:
    """Return robust change in guarded cross-subcarrier log ratios.

    Fixed four-bin pairs avoid corpus-trained subcarrier selection. A pair is
    retained only when both of its bins clear the relative floor in both
    packets. The median of ``abs(delta log-ratio) / (1 + abs(delta
    log-ratio))`` is bounded to ``[0, 1]`` and exactly cancels positive
    per-packet gain.
    """
    current = np.asarray(current, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    if current.shape != reference.shape or current.size == 0:
        return 0.0
    current_max = float(np.max(current))
    reference_max = float(np.max(reference))
    if current_max <= 0.0 or reference_max <= 0.0:
        return 0.0
    floor = max(0.0, float(relative_floor))
    threshold_current = floor * current_max
    threshold_reference = floor * reference_max
    left = _FREQUENCY_COHERENCE_LEFT
    right = _FREQUENCY_COHERENCE_RIGHT
    valid = (
        (current[left] > threshold_current)
        & (current[right] > threshold_current)
        & (reference[left] > threshold_reference)
        & (reference[right] > threshold_reference)
    )
    if np.count_nonzero(valid) < 4:
        return 0.0
    current_ratio = np.log(current[left[valid]]) - np.log(current[right[valid]])
    reference_ratio = (
        np.log(reference[left[valid]]) - np.log(reference[right[valid]])
    )
    delta = np.abs(current_ratio - reference_ratio)
    return float(np.median(delta / (1.0 + delta)))


def frequency_coherence(profile: np.ndarray, offset: int = 4) -> float:
    """Normalized within-packet coherence at a fixed subcarrier separation.

    Common gain cancels in the normalized complex correlation. CFO contributes
    one common phase and STO contributes one common rotation for the fixed
    frequency separation; taking the magnitude removes both. The statistic is
    therefore a compact proxy for frequency coherence, and hence multipath
    delay structure, without requiring a stable absolute phase.
    """
    offset = int(offset)
    if offset not in FREQUENCY_COHERENCE_OFFSETS:
        return 0.0
    if profile.shape != (len(HT20_LIVE_BINS),):
        return 0.0
    # Row `i` is live-band half `i`, so a pair at `offset` is a plain column
    # shift and both operands stay views. Fancy indexing would materialize the
    # pair arrays on every call instead.
    halves = profile.reshape(_LIVE_BAND_HALVES, _LIVE_BAND_SPLIT)
    left = halves[:, :_LIVE_BAND_SPLIT - offset]
    right = halves[:, offset:]
    left_squared = float(np.vdot(left, left).real)
    right_squared = float(np.vdot(right, right).real)
    denominator = np.sqrt(left_squared) * np.sqrt(right_squared)
    if denominator <= 0.0:
        return 0.0
    return float(abs(np.vdot(left, right)) / denominator)


class ChannelShapeTracker:
    """Track only the requested gain-free channel-shape dynamics."""

    def __init__(self, window_size: int = 90, lag: int = L1_DELTA_LAG,
                 feature_names: Optional[Iterable[str]] = None):
        self.window_size = max(2, int(window_size))
        self.lag = max(1, int(lag))
        names = set(feature_names or ())
        track_all = feature_names is None
        self._track_lag_ratio = track_all or 'chan_shape_lag_ratio' in names
        self._track_spread = track_all or bool(
            {'chan_coh_gap_spread'} & names
        )
        self._track_spread_ds2 = 'chan_shape_spread_ds2' in names
        self._track_spread_ema_fast = 'chan_shape_spread_ema_fast' in names
        self._track_spread_ema_slow = 'chan_shape_spread_ema_slow' in names
        self._track_any_spread = any((
            self._track_spread,
            self._track_spread_ds2,
            self._track_spread_ema_fast,
            self._track_spread_ema_slow,
        ))
        self._track_rank = track_all or 'chan_rank_gap' in names
        self._track_ratio = track_all or 'chan_ratio_gap' in names
        self._track_frequency_cv = track_all or 'chan_freq_coh_cv' in names
        self._track_frequency_curve = (
            track_all or bool(
                {'chan_freq_coh_curve_std', 'chan_freq_coh_curve_iqr'} & names
            )
        )
        self._frequency_candidate_names = {
            name for name in (
                'chan_freq_coh_curve_2_4_std',
                'chan_freq_coh_curve_4_12_std',
                'chan_freq_coh_decay_std',
                'chan_freq_coh_curvature_std',
            ) if track_all or name in names
        }
        self._track_profile = any((
            self._track_lag_ratio,
            self._track_any_spread,
            self._track_rank,
            self._track_ratio,
        ))
        self._track_previous = any((
            self._track_lag_ratio,
            self._track_rank,
            self._track_ratio,
        ))
        width = len(HT20_LIVE_BINS)
        self._ring = (
            [np.zeros(width, dtype=np.float64) for _ in range(self.lag)]
            if self._track_profile else []
        )
        self._ring_filled = [False] * self.lag if self._track_profile else []
        self._index = 0
        self._previous = (
            np.zeros(width, dtype=np.float64)
            if self._track_previous else np.empty(0, dtype=np.float64)
        )
        self._has_previous = False
        self._lag_distance_ring = (
            [0.0] * self.window_size if self._track_lag_ratio else []
        )
        self._adjacent_distance_ring = (
            [0.0] * self.window_size if self._track_lag_ratio else []
        )
        self._lag_distance_slot = 0
        self._lag_distance_count = 0
        self._lag_distance_sum = 0.0
        self._adjacent_distance_slot = 0
        self._adjacent_distance_count = 0
        self._adjacent_distance_sum = 0.0
        self._motion_energy = np.zeros(
            width if self._track_spread else 0,
            dtype=np.float64,
        )
        self._motion_energy_ring = np.zeros(
            (self.window_size, width) if self._track_spread else (0, 0),
            dtype=np.float64,
        )
        self._motion_energy_slot = 0
        self._motion_energy_count = 0
        self._motion_energy_ds2 = np.zeros(
            width if self._track_spread_ds2 else 0,
            dtype=np.float64,
        )
        self._motion_energy_ds2_window = max(1, (self.window_size + 1) // 2)
        self._motion_energy_ds2_ring = np.zeros(
            (self._motion_energy_ds2_window, width)
            if self._track_spread_ds2 else (0, 0),
            dtype=np.float64,
        )
        self._motion_energy_ds2_slot = 0
        self._motion_energy_ds2_count = 0
        self._motion_energy_ds2_phase = 0
        self._motion_energy_ema_fast = np.zeros(
            width if self._track_spread_ema_fast else 0,
            dtype=np.float64,
        )
        self._motion_energy_ema_slow = np.zeros(
            width if self._track_spread_ema_slow else 0,
            dtype=np.float64,
        )
        self._motion_energy_ema_fast_count = 0
        self._motion_energy_ema_slow_count = 0
        self._motion_energy_ema_fast_alpha = 2.0 / (self.window_size + 1.0)
        self._motion_energy_ema_slow_alpha = 1.0 / self.window_size
        self._frequency_coherence_ring = (
            [0.0] * self.window_size if self._track_frequency_cv else []
        )
        self._frequency_coherence_slot = 0
        self._frequency_coherence_count = 0
        self._frequency_coherence_sum = 0.0
        self._frequency_coherence_square_sum = 0.0
        self._frequency_curve_ring = (
            [0.0] * self.window_size if self._track_frequency_curve else []
        )
        self._frequency_curve_slot = 0
        self._frequency_curve_count = 0
        self._frequency_curve_sum = 0.0
        self._frequency_curve_square_sum = 0.0
        self._frequency_candidate_rings = {
            name: [0.0] * self.window_size
            for name in self._frequency_candidate_names
        }
        self._frequency_candidate_slots = {
            name: 0 for name in self._frequency_candidate_names
        }
        self._frequency_candidate_counts = {
            name: 0 for name in self._frequency_candidate_names
        }
        self._frequency_candidate_sums = {
            name: 0.0 for name in self._frequency_candidate_names
        }
        self._frequency_candidate_square_sums = {
            name: 0.0 for name in self._frequency_candidate_names
        }
        self._rank_lag_ring = (
            [0.0] * self.window_size if self._track_rank else []
        )
        self._rank_lag_slot = 0
        self._rank_lag_count = 0
        self._rank_lag_sum = 0.0
        self._rank_adjacent_ring = (
            [0.0] * self.window_size if self._track_rank else []
        )
        self._rank_adjacent_slot = 0
        self._rank_adjacent_count = 0
        self._rank_adjacent_sum = 0.0
        self._ratio_lag_ring = (
            [0.0] * self.window_size if self._track_ratio else []
        )
        self._ratio_lag_slot = 0
        self._ratio_lag_count = 0
        self._ratio_lag_sum = 0.0
        self._ratio_adjacent_ring = (
            [0.0] * self.window_size if self._track_ratio else []
        )
        self._ratio_adjacent_slot = 0
        self._ratio_adjacent_count = 0
        self._ratio_adjacent_sum = 0.0

    def _push_scalar(self, value, ring, slot, count, total):
        if count < self.window_size:
            count += 1
        else:
            total -= ring[slot]
        ring[slot] = value
        total += value
        slot = (slot + 1) % self.window_size
        return slot, count, total

    def _push_motion_energy(self, values):
        if self._motion_energy_count < self.window_size:
            self._motion_energy_count += 1
        else:
            self._motion_energy -= self._motion_energy_ring[
                self._motion_energy_slot
            ]
        self._motion_energy_ring[self._motion_energy_slot] = values
        self._motion_energy += values
        self._motion_energy_slot = (
            self._motion_energy_slot + 1
        ) % self.window_size

    def _push_motion_energy_ds2(self, values):
        if self._motion_energy_ds2_phase == 0:
            if self._motion_energy_ds2_count < self._motion_energy_ds2_window:
                self._motion_energy_ds2_count += 1
            else:
                self._motion_energy_ds2 -= self._motion_energy_ds2_ring[
                    self._motion_energy_ds2_slot
                ]
            self._motion_energy_ds2_ring[self._motion_energy_ds2_slot] = values
            self._motion_energy_ds2 += values
            self._motion_energy_ds2_slot = (
                self._motion_energy_ds2_slot + 1
            ) % self._motion_energy_ds2_window
        self._motion_energy_ds2_phase = 1 - self._motion_energy_ds2_phase

    @staticmethod
    def _push_motion_energy_ema(energy, values, count, alpha):
        if count == 0:
            energy[:] = values
        else:
            energy += alpha * (values - energy)
        return count + 1

    def _push_frequency_coherence(self, value):
        if self._frequency_coherence_count < self.window_size:
            self._frequency_coherence_count += 1
        else:
            old = self._frequency_coherence_ring[
                self._frequency_coherence_slot
            ]
            self._frequency_coherence_sum -= old
            self._frequency_coherence_square_sum -= old * old
        self._frequency_coherence_ring[self._frequency_coherence_slot] = value
        self._frequency_coherence_sum += value
        self._frequency_coherence_square_sum += value * value
        self._frequency_coherence_slot = (
            self._frequency_coherence_slot + 1
        ) % self.window_size

    def _push_frequency_curve(self, value):
        if self._frequency_curve_count < self.window_size:
            self._frequency_curve_count += 1
        else:
            old = self._frequency_curve_ring[self._frequency_curve_slot]
            self._frequency_curve_sum -= old
            self._frequency_curve_square_sum -= old * old
        self._frequency_curve_ring[self._frequency_curve_slot] = value
        self._frequency_curve_sum += value
        self._frequency_curve_square_sum += value * value
        self._frequency_curve_slot = (
            self._frequency_curve_slot + 1
        ) % self.window_size

    def _push_frequency_candidate(self, name, value):
        ring = self._frequency_candidate_rings[name]
        slot = self._frequency_candidate_slots[name]
        count = self._frequency_candidate_counts[name]
        total = self._frequency_candidate_sums[name]
        square_total = self._frequency_candidate_square_sums[name]
        if count < self.window_size:
            count += 1
        else:
            old = ring[slot]
            total -= old
            square_total -= old * old
        ring[slot] = value
        total += value
        square_total += value * value
        self._frequency_candidate_slots[name] = (slot + 1) % self.window_size
        self._frequency_candidate_counts[name] = count
        self._frequency_candidate_sums[name] = total
        self._frequency_candidate_square_sums[name] = square_total

    def process_packet(self, csi_data) -> None:
        complex_values = complex_profile(csi_data)
        profile = (
            normalized_amplitude_profile(complex_values)
            if self._track_profile else None
        )
        slot = self._index if self._track_profile else 0
        if self._track_profile and self._ring_filled[slot]:
            delta = profile - self._ring[slot]
            if self._track_lag_ratio:
                (
                    self._lag_distance_slot,
                    self._lag_distance_count,
                    self._lag_distance_sum,
                ) = self._push_scalar(
                    float(np.linalg.norm(delta)),
                    self._lag_distance_ring,
                    self._lag_distance_slot,
                    self._lag_distance_count,
                    self._lag_distance_sum,
                )
            if self._track_any_spread:
                squared_delta = delta * delta
                if self._track_spread:
                    self._push_motion_energy(squared_delta)
                if self._track_spread_ds2:
                    self._push_motion_energy_ds2(squared_delta)
                if self._track_spread_ema_fast:
                    self._motion_energy_ema_fast_count = (
                        self._push_motion_energy_ema(
                            self._motion_energy_ema_fast,
                            squared_delta,
                            self._motion_energy_ema_fast_count,
                            self._motion_energy_ema_fast_alpha,
                        )
                    )
                if self._track_spread_ema_slow:
                    self._motion_energy_ema_slow_count = (
                        self._push_motion_energy_ema(
                            self._motion_energy_ema_slow,
                            squared_delta,
                            self._motion_energy_ema_slow_count,
                            self._motion_energy_ema_slow_alpha,
                        )
                    )
            if self._track_rank:
                (
                    self._rank_lag_slot,
                    self._rank_lag_count,
                    self._rank_lag_sum,
                ) = self._push_scalar(
                    rank_profile_distance(profile, self._ring[slot]),
                    self._rank_lag_ring,
                    self._rank_lag_slot,
                    self._rank_lag_count,
                    self._rank_lag_sum,
                )
            if self._track_ratio:
                (
                    self._ratio_lag_slot,
                    self._ratio_lag_count,
                    self._ratio_lag_sum,
                ) = self._push_scalar(
                    cross_subcarrier_ratio_distance(profile, self._ring[slot]),
                    self._ratio_lag_ring,
                    self._ratio_lag_slot,
                    self._ratio_lag_count,
                    self._ratio_lag_sum,
                )
        if self._track_previous and self._has_previous:
            if self._track_lag_ratio:
                distance = float(np.linalg.norm(profile - self._previous))
                (
                    self._adjacent_distance_slot,
                    self._adjacent_distance_count,
                    self._adjacent_distance_sum,
                ) = self._push_scalar(
                    distance,
                    self._adjacent_distance_ring,
                    self._adjacent_distance_slot,
                    self._adjacent_distance_count,
                    self._adjacent_distance_sum,
                )
            if self._track_rank:
                (
                    self._rank_adjacent_slot,
                    self._rank_adjacent_count,
                    self._rank_adjacent_sum,
                ) = self._push_scalar(
                    rank_profile_distance(profile, self._previous),
                    self._rank_adjacent_ring,
                    self._rank_adjacent_slot,
                    self._rank_adjacent_count,
                    self._rank_adjacent_sum,
                )
            if self._track_ratio:
                (
                    self._ratio_adjacent_slot,
                    self._ratio_adjacent_count,
                    self._ratio_adjacent_sum,
                ) = self._push_scalar(
                    cross_subcarrier_ratio_distance(profile, self._previous),
                    self._ratio_adjacent_ring,
                    self._ratio_adjacent_slot,
                    self._ratio_adjacent_count,
                    self._ratio_adjacent_sum,
                )
        if self._track_frequency_cv:
            self._push_frequency_coherence(frequency_coherence(complex_values))
        if self._track_frequency_curve or self._frequency_candidate_names:
            mid_coherence = frequency_coherence(complex_values, offset=4)
            long_coherence = frequency_coherence(complex_values, offset=12)
            if self._track_frequency_curve:
                coherence_sum = mid_coherence + long_coherence
                curve_contrast = (
                    (mid_coherence - long_coherence) / coherence_sum
                    if coherence_sum > 0.0
                    else 0.0
                )
                self._push_frequency_curve(curve_contrast)
            if self._frequency_candidate_names:
                short_coherence = frequency_coherence(complex_values, offset=2)
                coherence_sum = (
                    short_coherence + mid_coherence + long_coherence
                )
                if coherence_sum > 0.0:
                    decay = (
                        2.0 * short_coherence
                        + mid_coherence
                        - 3.0 * long_coherence
                    ) / (3.0 * coherence_sum)
                    curvature = (
                        mid_coherence
                        - 0.8 * short_coherence
                        - 0.2 * long_coherence
                    ) / coherence_sum
                else:
                    decay = 0.0
                    curvature = 0.0
                short_mid_sum = short_coherence + mid_coherence
                mid_long_sum = mid_coherence + long_coherence
                short_mid_curve = (
                    (short_coherence - mid_coherence) / short_mid_sum
                    if short_mid_sum > 0.0 else 0.0
                )
                mid_long_curve = (
                    (mid_coherence - long_coherence) / mid_long_sum
                    if mid_long_sum > 0.0 else 0.0
                )
                if 'chan_freq_coh_curve_2_4_std' in self._frequency_candidate_names:
                    self._push_frequency_candidate(
                        'chan_freq_coh_curve_2_4_std', short_mid_curve
                    )
                if 'chan_freq_coh_curve_4_12_std' in self._frequency_candidate_names:
                    self._push_frequency_candidate(
                        'chan_freq_coh_curve_4_12_std', mid_long_curve
                    )
                if 'chan_freq_coh_decay_std' in self._frequency_candidate_names:
                    self._push_frequency_candidate(
                        'chan_freq_coh_decay_std', decay
                    )
                if 'chan_freq_coh_curvature_std' in self._frequency_candidate_names:
                    self._push_frequency_candidate(
                        'chan_freq_coh_curvature_std', curvature
                    )
        if self._track_previous:
            self._previous = profile.copy()
            self._has_previous = True
        if self._track_profile:
            self._ring[slot] = profile
            self._ring_filled[slot] = True
            self._index = (self._index + 1) % self.lag

    def shape_lag_ratio(self) -> float:
        """Lagged normalized-shape displacement over adjacent displacement."""
        if (
            self._lag_distance_count == 0
            or self._adjacent_distance_count == 0
        ):
            return 1.0
        adjacent_mean = (
            self._adjacent_distance_sum / self._adjacent_distance_count
        )
        if adjacent_mean <= 0.0:
            return 1.0
        return (
            self._lag_distance_sum / self._lag_distance_count
        ) / adjacent_mean

    def shape_spread(self) -> float:
        """Participation ratio of lagged motion energy over the live band."""
        return motion_participation(self._motion_energy)

    def shape_spread_ds2(self) -> float:
        """Rectangular-window spread evaluated on every second lag delta."""
        return motion_participation(self._motion_energy_ds2)

    def shape_spread_ema_fast(self) -> float:
        """Spread from an EMA whose conventional span equals the window."""
        return motion_participation(self._motion_energy_ema_fast)

    def shape_spread_ema_slow(self) -> float:
        """Spread from a one-window exponential time constant."""
        return motion_participation(self._motion_energy_ema_slow)

    def frequency_coherence_cv(self) -> float:
        """Temporal CV of the gain- and offset-free frequency coherence."""
        if self._frequency_coherence_count == 0:
            return 0.0
        count = self._frequency_coherence_count
        mean = self._frequency_coherence_sum / count
        variance = max(
            0.0,
            self._frequency_coherence_square_sum / count - mean * mean,
        )
        if mean <= 0.0:
            return 0.0
        return float(np.sqrt(variance) / mean)

    def frequency_coherence_curve_std(self) -> float:
        """Temporal standard deviation of short-versus-long coherence."""
        if self._frequency_curve_count == 0:
            return 0.0
        count = self._frequency_curve_count
        mean = self._frequency_curve_sum / count
        variance = max(
            0.0,
            self._frequency_curve_square_sum / count - mean * mean,
        )
        return float(np.sqrt(variance))

    def frequency_coherence_curve_iqr(self) -> float:
        """Temporal IQR of the short-versus-long coherence contrast."""
        if self._frequency_curve_count == 0:
            return 0.0
        values = np.asarray(
            self._frequency_curve_ring[:self._frequency_curve_count],
            dtype=np.float64,
        )
        q25, q75 = np.quantile(values, [0.25, 0.75])
        return float(q75 - q25)

    def frequency_coherence_candidate_std(self, name: str) -> float:
        """Temporal standard deviation of one three-offset curve candidate."""
        count = self._frequency_candidate_counts.get(name, 0)
        if count == 0:
            return 0.0
        mean = self._frequency_candidate_sums[name] / count
        variance = max(
            0.0,
            self._frequency_candidate_square_sums[name] / count - mean * mean,
        )
        return float(np.sqrt(variance))

    def rank_gap(self) -> float:
        """Mean lagged rank distance minus adjacent-packet rank distance."""
        if self._rank_lag_count == 0 or self._rank_adjacent_count == 0:
            return 0.0
        return (
            self._rank_lag_sum / self._rank_lag_count
        ) - (
            self._rank_adjacent_sum / self._rank_adjacent_count
        )

    def ratio_gap(self) -> float:
        """Mean lagged ratio distance minus adjacent-packet ratio distance."""
        if self._ratio_lag_count == 0 or self._ratio_adjacent_count == 0:
            return 0.0
        return (
            self._ratio_lag_sum / self._ratio_lag_count
        ) - (
            self._ratio_adjacent_sum / self._ratio_adjacent_count
        )

    def reset(self) -> None:
        if self._track_profile:
            for i in range(self.lag):
                self._ring_filled[i] = False
        self._index = 0
        self._has_previous = False
        self._lag_distance_slot = 0
        self._lag_distance_count = 0
        self._lag_distance_sum = 0.0
        self._adjacent_distance_slot = 0
        self._adjacent_distance_count = 0
        self._adjacent_distance_sum = 0.0
        self._motion_energy.fill(0.0)
        self._motion_energy_slot = 0
        self._motion_energy_count = 0
        self._motion_energy_ds2.fill(0.0)
        self._motion_energy_ds2_slot = 0
        self._motion_energy_ds2_count = 0
        self._motion_energy_ds2_phase = 0
        self._motion_energy_ema_fast.fill(0.0)
        self._motion_energy_ema_slow.fill(0.0)
        self._motion_energy_ema_fast_count = 0
        self._motion_energy_ema_slow_count = 0
        self._frequency_coherence_slot = 0
        self._frequency_coherence_count = 0
        self._frequency_coherence_sum = 0.0
        self._frequency_coherence_square_sum = 0.0
        self._frequency_curve_slot = 0
        self._frequency_curve_count = 0
        self._frequency_curve_sum = 0.0
        self._frequency_curve_square_sum = 0.0
        for name in self._frequency_candidate_names:
            self._frequency_candidate_slots[name] = 0
            self._frequency_candidate_counts[name] = 0
            self._frequency_candidate_sums[name] = 0.0
            self._frequency_candidate_square_sums[name] = 0.0
        self._rank_lag_slot = 0
        self._rank_lag_count = 0
        self._rank_lag_sum = 0.0
        self._rank_adjacent_slot = 0
        self._rank_adjacent_count = 0
        self._rank_adjacent_sum = 0.0
        self._ratio_lag_slot = 0
        self._ratio_lag_count = 0
        self._ratio_lag_sum = 0.0
        self._ratio_adjacent_slot = 0
        self._ratio_adjacent_count = 0
        self._ratio_adjacent_sum = 0.0


class PhaseResidualTracker:
    """Running lag/adjacent displacement of sanitized phase profiles."""

    def __init__(self, window_size: int = 90, lag: int = L1_DELTA_LAG):
        self.window_size = max(2, int(window_size))
        self.lag = max(1, int(lag))
        width = len(_ADJACENT_PAIRS)
        self._ring = [np.zeros(width, dtype=np.complex128) for _ in range(self.lag)]
        self._ring_filled = [False] * self.lag
        self._index = 0
        self._previous = np.zeros(width, dtype=np.complex128)
        self._has_previous = False
        self._lag_ring = [0.0] * self.window_size
        self._adjacent_ring = [0.0] * self.window_size
        self._lag_slot = 0
        self._lag_count = 0
        self._lag_sum = 0.0
        self._adjacent_slot = 0
        self._adjacent_count = 0
        self._adjacent_sum = 0.0
        self._closure_ring = [0.0] * self.window_size
        self._closure_slot = 0
        self._closure_count = 0
        self._closure_sum = 0.0
        self._closure_square_sum = 0.0

    def _push(self, value, ring, slot, count, total):
        if count < self.window_size:
            count += 1
        else:
            total -= ring[slot]
        ring[slot] = value
        total += value
        slot += 1
        if slot >= self.window_size:
            slot = 0
        return slot, count, total

    def process_packet(self, csi_data) -> None:
        complex_values = complex_profile(csi_data)
        profile = sanitized_phase_profile(complex_values)
        slot = self._index
        if self._ring_filled[slot]:
            value = phase_profile_distance(profile, self._ring[slot])
            self._lag_slot, self._lag_count, self._lag_sum = self._push(
                value,
                self._lag_ring,
                self._lag_slot,
                self._lag_count,
                self._lag_sum,
            )
        if self._has_previous:
            value = phase_profile_distance(profile, self._previous)
            (
                self._adjacent_slot,
                self._adjacent_count,
                self._adjacent_sum,
            ) = self._push(
                value,
                self._adjacent_ring,
                self._adjacent_slot,
                self._adjacent_count,
                self._adjacent_sum,
            )
        self._previous = profile.copy()
        self._has_previous = True
        self._ring[slot] = profile
        self._ring_filled[slot] = True
        self._index = (self._index + 1) % self.lag
        closure = local_phase_closure_variance(complex_values)
        if self._closure_count < self.window_size:
            self._closure_count += 1
        else:
            old = self._closure_ring[self._closure_slot]
            self._closure_sum -= old
            self._closure_square_sum -= old * old
        self._closure_ring[self._closure_slot] = closure
        self._closure_sum += closure
        self._closure_square_sum += closure * closure
        self._closure_slot = (self._closure_slot + 1) % self.window_size

    def phase_residual_lag_ratio(self) -> float:
        if self._lag_count == 0 or self._adjacent_count == 0:
            return 1.0
        adjacent_mean = self._adjacent_sum / self._adjacent_count
        if adjacent_mean <= 0.0:
            return 1.0
        return (self._lag_sum / self._lag_count) / adjacent_mean

    def phase_closure_variance_std(self) -> float:
        """Temporal standard deviation of local closure circular variance."""
        if self._closure_count == 0:
            return 0.0
        mean = self._closure_sum / self._closure_count
        variance = max(
            0.0,
            self._closure_square_sum / self._closure_count - mean * mean,
        )
        return float(np.sqrt(variance))

    def reset(self) -> None:
        for i in range(self.lag):
            self._ring_filled[i] = False
        self._index = 0
        self._has_previous = False
        self._lag_slot = 0
        self._lag_count = 0
        self._lag_sum = 0.0
        self._adjacent_slot = 0
        self._adjacent_count = 0
        self._adjacent_sum = 0.0
        self._closure_slot = 0
        self._closure_count = 0
        self._closure_sum = 0.0
        self._closure_square_sum = 0.0


class ChannelCoherenceTracker:
    """Running delay-compensated coherence at the profile lag and at lag 1.

    Mirrors the shape of `L1DeltaTracker`: the same window of packets, the same
    pair of lags, and a ratio between them so no absolute magnitude survives.
    """

    def __init__(
        self,
        window_size: int = 90,
        lag: int = L1_DELTA_LAG,
        track_subbands: bool = False,
    ):
        self.window_size = max(2, int(window_size))
        self.lag = max(1, int(lag))
        self.track_subbands = bool(track_subbands)
        width = len(HT20_LIVE_BINS)
        self._ring = [np.zeros(width, dtype=np.complex128) for _ in range(self.lag)]
        self._ring_filled = [False] * self.lag
        self._index = 0
        self._previous = np.zeros(width, dtype=np.complex128)
        self._has_previous = False
        self._lag_sum = 0.0
        self._lag_count = 0
        self._adjacent_sum = 0.0
        self._adjacent_count = 0
        self._lag_ring = [0.0] * self.window_size
        self._adjacent_ring = [0.0] * self.window_size
        self._gap_ring = [0.0] * self.window_size
        self._lag_slot = 0
        self._adjacent_slot = 0
        self._gap_slot = 0
        self._gap_count = 0
        subband_count = len(HT20_COHERENCE_SUBBANDS)
        self._subband_lag_sum = np.zeros(subband_count, dtype=np.float64)
        self._subband_adjacent_sum = np.zeros(subband_count, dtype=np.float64)
        self._subband_lag_ring = np.zeros(
            (self.window_size, subband_count), dtype=np.float64
        )
        self._subband_adjacent_ring = np.zeros(
            (self.window_size, subband_count), dtype=np.float64
        )
        self._subband_lag_slot = 0
        self._subband_lag_count = 0
        self._subband_adjacent_slot = 0
        self._subband_adjacent_count = 0
        self._subband_lag_median_sum = 0.0
        self._subband_adjacent_median_sum = 0.0
        self._subband_lag_median_ring = [0.0] * self.window_size
        self._subband_adjacent_median_ring = [0.0] * self.window_size
        self._subband_lag_median_slot = 0
        self._subband_lag_median_count = 0
        self._subband_adjacent_median_slot = 0
        self._subband_adjacent_median_count = 0
    def _push(self, value, ring, slot, count, total):
        if count < self.window_size:
            count += 1
        else:
            total -= ring[slot]
        ring[slot] = value
        total += value
        slot += 1
        if slot >= self.window_size:
            slot = 0
        return slot, count, total

    def _push_subbands(self, values, ring, slot, count, total):
        if count < self.window_size:
            count += 1
        else:
            total -= ring[slot]
        ring[slot] = values
        total += values
        slot += 1
        if slot >= self.window_size:
            slot = 0
        return slot, count, total

    def process_packet(self, csi_data) -> None:
        """Consume one raw CSI payload."""
        profile = complex_profile(csi_data)
        slot = self._index
        lag_value = None
        adjacent_value = None
        if self._ring_filled[slot]:
            # One cross-product array per reference, shared by the full band and
            # the four subbands that tile it.
            cross, magnitude = coherence_cross(profile, self._ring[slot])
            lag_value = delay_compensated_coherence_from_cross(cross, magnitude)
            self._lag_slot, self._lag_count, self._lag_sum = self._push(
                lag_value, self._lag_ring, self._lag_slot, self._lag_count, self._lag_sum
            )
            if self.track_subbands:
                lag_subbands = subband_coherences_from_cross(cross, magnitude)
                (
                    self._subband_lag_slot,
                    self._subband_lag_count,
                    self._subband_lag_sum,
                ) = self._push_subbands(
                    lag_subbands,
                    self._subband_lag_ring,
                    self._subband_lag_slot,
                    self._subband_lag_count,
                    self._subband_lag_sum,
                )
                (
                    self._subband_lag_median_slot,
                    self._subband_lag_median_count,
                    self._subband_lag_median_sum,
                ) = self._push(
                    float(np.median(lag_subbands)),
                    self._subband_lag_median_ring,
                    self._subband_lag_median_slot,
                    self._subband_lag_median_count,
                    self._subband_lag_median_sum,
                )
        if self._has_previous:
            cross, magnitude = coherence_cross(profile, self._previous)
            adjacent_value = delay_compensated_coherence_from_cross(cross, magnitude)
            (self._adjacent_slot, self._adjacent_count,
             self._adjacent_sum) = self._push(
                adjacent_value, self._adjacent_ring, self._adjacent_slot,
                self._adjacent_count, self._adjacent_sum
            )
            if self.track_subbands:
                adjacent_subbands = subband_coherences_from_cross(cross, magnitude)
                (
                    self._subband_adjacent_slot,
                    self._subband_adjacent_count,
                    self._subband_adjacent_sum,
                ) = self._push_subbands(
                    adjacent_subbands,
                    self._subband_adjacent_ring,
                    self._subband_adjacent_slot,
                    self._subband_adjacent_count,
                    self._subband_adjacent_sum,
                )
                (
                    self._subband_adjacent_median_slot,
                    self._subband_adjacent_median_count,
                    self._subband_adjacent_median_sum,
                ) = self._push(
                    float(np.median(adjacent_subbands)),
                    self._subband_adjacent_median_ring,
                    self._subband_adjacent_median_slot,
                    self._subband_adjacent_median_count,
                    self._subband_adjacent_median_sum,
                )
        if lag_value is not None and adjacent_value is not None:
            self._gap_slot, self._gap_count, _ = self._push(
                adjacent_value - lag_value,
                self._gap_ring,
                self._gap_slot,
                self._gap_count,
                0.0,
            )
        self._previous = profile.copy()
        self._has_previous = True
        self._ring[slot] = profile
        self._ring_filled[slot] = True
        self._index += 1
        if self._index >= self.lag:
            self._index = 0

    def mean_coherence(self) -> float:
        """Mean lag-``lag`` coherence over the current window."""
        if self._lag_count == 0:
            return 1.0
        return self._lag_sum / self._lag_count

    def count(self) -> int:
        """Return the number of lagged coherence samples in the window."""
        return self._lag_count

    def coherence_lag_ratio(self) -> float:
        """Lag-``lag`` coherence divided by lag-1 coherence.

        Both terms carry the same link and hardware conditions, so the ratio
        reports how much faster the channel decorrelates over the profile lag
        than between neighbouring packets. It sits near 1 on a still channel and
        falls as motion decorrelates the longer lag first.
        """
        if self._lag_count == 0 or self._adjacent_count == 0:
            return 1.0
        adjacent_mean = self._adjacent_sum / self._adjacent_count
        if adjacent_mean <= 0.0:
            return 1.0
        return (self._lag_sum / self._lag_count) / adjacent_mean

    def coherence_gap(self) -> float:
        """Adjacent-packet coherence minus lag-``lag`` coherence.

        Still channels keep both terms near one, so the gap stays near zero.
        Motion lowers the longer-lag coherence first, pushing the gap positive.
        """
        if self._lag_count == 0 or self._adjacent_count == 0:
            return 0.0
        return (self._adjacent_sum / self._adjacent_count) - (
            self._lag_sum / self._lag_count
        )

    def _gap_values(self) -> np.ndarray:
        if self._gap_count == 0:
            return np.zeros(0, dtype=np.float64)
        return np.asarray(self._gap_ring[:self._gap_count], dtype=np.float64)

    def coherence_gap_low_frac(self, threshold: float = COHERENCE_GAP_LOW_THRESHOLD) -> float:
        """Fraction of window entries whose coherence gap exceeds ``threshold``.

        Isolated noisy packets leave this close to zero, while sustained motion
        keeps the long-lag coherence consistently below the adjacent coherence.
        """
        values = self._gap_values()
        if values.size == 0:
            return 0.0
        return float(np.mean(values > float(threshold)))

    def coherence_gap_q20(self) -> float:
        """20th percentile of the coherence-gap window.

        This stays near zero unless a large share of the window shows a positive
        adjacent-minus-lag margin, making it more robust to short noisy bursts.
        """
        values = self._gap_values()
        if values.size == 0:
            return 0.0
        return float(np.quantile(values, 0.20))

    def coherence_subband_median_gap(self) -> float:
        """Gap after taking the median subband coherence for every pair."""
        if (
            self._subband_lag_median_count == 0
            or self._subband_adjacent_median_count == 0
        ):
            return 0.0
        return (
            self._subband_adjacent_median_sum
            / self._subband_adjacent_median_count
        ) - (
            self._subband_lag_median_sum
            / self._subband_lag_median_count
        )

    def coherence_subband_gap_median(self) -> float:
        """Median across subbands of their adjacent-minus-lag mean gaps."""
        if self._subband_lag_count == 0 or self._subband_adjacent_count == 0:
            return 0.0
        gaps = (
            self._subband_adjacent_sum / self._subband_adjacent_count
        ) - (
            self._subband_lag_sum / self._subband_lag_count
        )
        return float(np.median(gaps))

    def reset(self) -> None:
        for i in range(self.lag):
            self._ring_filled[i] = False
        self._index = 0
        self._has_previous = False
        self._lag_sum = 0.0
        self._lag_count = 0
        self._adjacent_sum = 0.0
        self._adjacent_count = 0
        self._lag_slot = 0
        self._adjacent_slot = 0
        self._gap_slot = 0
        self._gap_count = 0
        self._subband_lag_sum.fill(0.0)
        self._subband_adjacent_sum.fill(0.0)
        self._subband_lag_slot = 0
        self._subband_lag_count = 0
        self._subband_adjacent_slot = 0
        self._subband_adjacent_count = 0
        self._subband_lag_median_sum = 0.0
        self._subband_adjacent_median_sum = 0.0
        self._subband_lag_median_slot = 0
        self._subband_lag_median_count = 0
        self._subband_adjacent_median_slot = 0
        self._subband_adjacent_median_count = 0
