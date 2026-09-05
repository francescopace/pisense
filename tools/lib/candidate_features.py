# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Host-Side Candidate Features

Registry and evaluator for the still-active host-only candidate features.
Shared tracker math and HT20 feature primitives live in
`host_feature_trackers.py`. Promoted features use the MicroPython production
extractor; this module owns only candidates that have not been promoted.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

import ast
import hashlib
import inspect
import textwrap
from functools import lru_cache
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np

from .host_feature_trackers import (
    AGGREGATED_SPECTRAL_FEATURES,
    AMPLITUDE_PROFILE_FEATURES,
    CHANNEL_COHERENCE_FEATURES,
    CHANNEL_SHAPE_FEATURES,
    CLASSIC_ONLY_CHANNEL_SHAPE_FEATURES,
    CHANNEL_SHAPE_TRAJECTORY_FEATURES,
    COMPOSITE_FEATURES,
    L1_SERIES_FEATURES,
    PHASE_FEATURES,
    PROMOTED_CHANNEL_SHAPE_FEATURES,
    PROMOTED_CHANNEL_SHAPE_TRAJECTORY_FEATURES,
    SPECTRAL_FEATURES,
    SUBBAND_COHERENCE_FEATURES,
    AmplitudeProfileTracker,
    ChannelCoherenceTracker,
    ChannelShapeTrajectoryTracker,
    ChannelShapeTracker,
    PhaseResidualTracker,
    turbulence_band_power_ratio,
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

# Cache identities deliberately live below the feature registry. Adding a new
# sibling does not alter an existing feature identity. Increment a provider
# version only when its shared tracker or preprocessing contract changes;
# feature-local formula edits are detected from the matching ``name == ...``
# branch in ``candidate_values``.
FEATURE_PROVIDER_VERSIONS = {
    'channel_coherence': 1,
    'turbulence_series': 1,
    'aggregated_turbulence_series': 1,
    'phase_residual': 1,
    'channel_shape': 1,
    'channel_shape_trajectory': 1,
    'amplitude_profile': 1,
    'l1_series': 1,
    'composite': 1,
}


def candidate_feature_provider(feature_name: str) -> str:
    """Return the shared intermediate provider for one host-only feature."""
    name = str(feature_name)
    if name in CHANNEL_COHERENCE_FEATURES:
        return 'channel_coherence'
    if name in SPECTRAL_FEATURES:
        return 'turbulence_series'
    if name in AGGREGATED_SPECTRAL_FEATURES:
        return 'aggregated_turbulence_series'
    if name in PHASE_FEATURES:
        return 'phase_residual'
    shape_features = (
        CHANNEL_SHAPE_FEATURES
        + CLASSIC_ONLY_CHANNEL_SHAPE_FEATURES
    )
    if name in shape_features:
        return 'channel_shape'
    if name in CHANNEL_SHAPE_TRAJECTORY_FEATURES:
        return 'channel_shape_trajectory'
    if name in AMPLITUDE_PROFILE_FEATURES:
        return 'amplitude_profile'
    if name in L1_SERIES_FEATURES:
        return 'l1_series'
    if name in COMPOSITE_FEATURES:
        return 'composite'
    raise ValueError(f"Unknown candidate feature: {name}")


@lru_cache(maxsize=None)
def _feature_formula_digest(feature_name: str) -> str:
    """Hash the feature-local branch without hashing sibling branches."""
    source = textwrap.dedent(inspect.getsource(candidate_values))
    tree = ast.parse(source)
    matching = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.If):
            continue
        test = node.test
        if not isinstance(test, ast.Compare) or len(test.ops) != 1:
            continue
        if not isinstance(test.ops[0], ast.Eq) or len(test.comparators) != 1:
            continue
        operands = (test.left, test.comparators[0])
        names = [item for item in operands if isinstance(item, ast.Name)]
        constants = [item for item in operands if isinstance(item, ast.Constant)]
        if (
            any(item.id == 'name' for item in names)
            and any(item.value == feature_name for item in constants)
        ):
            matching.append(
                ast.dump(
                    ast.Module(body=node.body, type_ignores=[]),
                    annotate_fields=True,
                    include_attributes=False,
                )
            )
    payload = '\n'.join(matching) or f"registry-only:{feature_name}"
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()


@lru_cache(maxsize=None)
def _candidate_feature_cache_identity_items(
    feature_name: str,
) -> Tuple[Tuple[str, object], ...]:
    """Return one immutable memoized candidate-feature identity."""
    name = str(feature_name)
    provider = candidate_feature_provider(name)
    return tuple({
        'feature_name': name,
        'provider': provider,
        'provider_version': FEATURE_PROVIDER_VERSIONS[provider],
        'formula_sha256': _feature_formula_digest(name),
    }.items())


def candidate_feature_cache_identity(feature_name: str) -> Dict[str, object]:
    """Return an isolated copy of one memoized candidate-feature identity."""
    return dict(_candidate_feature_cache_identity_items(str(feature_name)))


def needs_channel_coherence(feature_names: Iterable[str]) -> bool:
    """Return whether any requested feature needs the coherence tracker."""
    return any(
        name in CHANNEL_COHERENCE_FEATURES
        or name in COMPOSITE_FEATURES
        for name in feature_names
    )


def needs_subband_coherence(feature_names: Iterable[str]) -> bool:
    """Return whether any requested feature needs per-subband coherence."""
    return any(name in SUBBAND_COHERENCE_FEATURES for name in feature_names)


def needs_turbulence_series(feature_names: Iterable[str]) -> bool:
    """Return whether any requested candidate reads the turbulence window."""
    return any(name in SPECTRAL_FEATURES for name in feature_names)


def needs_l1_series(feature_names: Iterable[str]) -> bool:
    """Return whether a host-only candidate reads the L1-delta series."""
    return any(name in L1_SERIES_FEATURES for name in feature_names)


def needs_aggregated_turbulence(feature_names: Iterable[str]) -> bool:
    """Return whether any requested candidate reads the aggregated window."""
    return any(name in AGGREGATED_SPECTRAL_FEATURES for name in feature_names)


def needs_amplitude_profiles(feature_names: Iterable[str]) -> bool:
    """Return whether a candidate reads base or aggregated tone profiles."""
    return any(
        name in AMPLITUDE_PROFILE_FEATURES
        or name == 'turb_iqr_over_mean_aggr_tone_detrended'
        for name in feature_names
    )


def needs_phase_residual(feature_names: Iterable[str]) -> bool:
    """Return whether any requested feature needs sanitized phase tracking."""
    return any(name in PHASE_FEATURES for name in feature_names)


def needs_channel_shape(feature_names: Iterable[str]) -> bool:
    """Return whether any requested feature needs normalized channel shape."""
    return any(
        name in CHANNEL_SHAPE_FEATURES
        or name in PROMOTED_CHANNEL_SHAPE_FEATURES
        or name in COMPOSITE_FEATURES
        for name in feature_names
    )


def needs_channel_shape_trajectory(feature_names: Iterable[str]) -> bool:
    """Return whether a requested feature needs time-binned channel shape."""
    return any(
        name in CHANNEL_SHAPE_TRAJECTORY_FEATURES
        for name in feature_names
    )


def split_feature_names(
    feature_names: Iterable[str],
) -> Tuple[List[str], List[str]]:
    """Split a requested set into production names and candidate names."""
    names = list(feature_names)
    candidates = [name for name in names if name in CANDIDATE_FEATURES]
    production = [name for name in names if name not in CANDIDATE_FEATURES]
    return production, candidates


def assemble_feature_vector(
    feature_names: Sequence[str],
    production_names: Sequence[str],
    production_values: Sequence[float],
    candidate_feature_values: Mapping[str, float],
) -> List[float]:
    """Rebuild the full feature vector in the caller's requested order."""
    production_lookup = dict(zip(production_names, production_values, strict=True))
    return [
        candidate_feature_values[name]
        if name in candidate_feature_values
        else production_lookup[name]
        for name in feature_names
    ]


def candidate_values(
    feature_names: Iterable[str],
    coherence_tracker: ChannelCoherenceTracker = None,
    turbulence_series: Sequence[float] = None,
    aggregated_turbulence_series: Sequence[float] = None,
    phase_tracker: PhaseResidualTracker = None,
    shape_tracker: ChannelShapeTracker = None,
    shape_trajectory_tracker: ChannelShapeTrajectoryTracker = None,
    amplitude_profile_tracker: AmplitudeProfileTracker = None,
    l1_series: Sequence[float] = None,
) -> Dict[str, float]:
    """Evaluate the requested candidates from their preprocessed trackers."""
    values: Dict[str, float] = {}
    turbulence = None
    if turbulence_series is not None:
        turbulence = np.asarray(turbulence_series, dtype=np.float64)
    aggregated_turbulence = None
    if aggregated_turbulence_series is not None:
        aggregated_turbulence = np.asarray(
            aggregated_turbulence_series,
            dtype=np.float64,
        )
    mean_denom = None
    iqr = None
    q95 = None
    q05 = None
    turbulence_median = None
    turbulence_mad = None
    turbulence_std = None
    aggregated_mean_denom = None
    aggregated_mad = None
    aggregated_q95 = None

    for name in feature_names:
        if name not in CANDIDATE_FEATURES:
            continue
        if name in SPECTRAL_FEATURES:
            if turbulence is None:
                raise ValueError(
                    f"{name} needs the turbulence window; pass the explicitly "
                    f"preprocessed stream"
                )
            if len(turbulence) < 4:
                values[name] = 0.0
                continue
            mean = float(np.mean(turbulence))
            if mean_denom is None:
                mean_denom = abs(mean) if abs(mean) > 1e-6 else 1e-6
            if name == 'turb_iqr_over_mean':
                if iqr is None:
                    q25, q75 = np.percentile(turbulence, [25, 75])
                    iqr = float(q75 - q25)
                values[name] = iqr / mean_denom
                continue
            if name == 'turb_band_power_ratio':
                values[name] = turbulence_band_power_ratio(turbulence)
                continue
            if name == 'turb_cv':
                if turbulence_std is None:
                    turbulence_std = float(np.std(turbulence))
                values[name] = turbulence_std / mean_denom
                continue
            if name == 'turb_mad_over_mean':
                if turbulence_median is None:
                    turbulence_median = float(np.median(turbulence))
                if turbulence_mad is None:
                    turbulence_mad = float(
                        np.median(np.abs(turbulence - turbulence_median))
                    )
                values[name] = turbulence_mad / mean_denom
                continue
            if name == 'turb_p95_over_mean':
                if q95 is None:
                    q95 = float(np.percentile(turbulence, 95))
                values[name] = q95 / mean_denom
                continue
            if name == 'turb_p05_over_mean':
                if q05 is None:
                    q05 = float(np.percentile(turbulence, 5))
                values[name] = q05 / mean_denom
                continue
            if name == 'turb_max_over_mean':
                values[name] = float(np.max(turbulence)) / mean_denom
                continue
            if name == 'turb_min_over_mean':
                values[name] = float(np.min(turbulence)) / mean_denom
                continue
            if name == 'turb_range_over_mean':
                values[name] = (
                    float(np.max(turbulence)) - float(np.min(turbulence))
                ) / mean_denom
                continue
            if name == 'turb_peak_over_mad':
                if turbulence_median is None:
                    turbulence_median = float(np.median(turbulence))
                if turbulence_mad is None:
                    turbulence_mad = float(
                        np.median(np.abs(turbulence - turbulence_median))
                    )
                values[name] = (
                    (float(np.max(turbulence)) - mean) / turbulence_mad
                    if turbulence_mad > 0.0 else 0.0
                )
                continue
            if name == 'waveform_length_over_mean':
                values[name] = float(
                    np.mean(np.abs(np.diff(turbulence)))
                ) / mean_denom
                continue
            if name == 'turb_skewness':
                if turbulence_std is None:
                    turbulence_std = float(np.std(turbulence))
                values[name] = (
                    float(np.mean((turbulence - mean) ** 3))
                    / (turbulence_std ** 3)
                    if turbulence_std > 0.0 else 0.0
                )
                continue
        if name in AGGREGATED_SPECTRAL_FEATURES:
            if aggregated_turbulence is None:
                raise ValueError(
                    f"{name} needs the aggregated turbulence window; pass the "
                    f"explicitly preprocessed aggregated stream"
                )
            if len(aggregated_turbulence) < 4:
                values[name] = 0.0
                continue
            aggregated_mean = float(np.mean(aggregated_turbulence))
            if aggregated_mean_denom is None:
                aggregated_mean_denom = (
                    abs(aggregated_mean)
                    if abs(aggregated_mean) > 1e-6
                    else 1e-6
                )
            if name == 'turb_mad_over_mean_aggr':
                if aggregated_mad is None:
                    median = float(np.median(aggregated_turbulence))
                    aggregated_mad = float(
                        np.median(np.abs(aggregated_turbulence - median))
                    )
                values[name] = aggregated_mad / aggregated_mean_denom
                continue
            if name == 'turb_p95_over_mean_aggr':
                if aggregated_q95 is None:
                    aggregated_q95 = float(np.percentile(aggregated_turbulence, 95))
                values[name] = aggregated_q95 / aggregated_mean_denom
                continue
            if name == 'turb_iqr_over_mean_aggr_tone_detrended':
                if amplitude_profile_tracker is None:
                    raise ValueError(
                        f"{name} needs the amplitude-profile tracker"
                    )
                values[name] = (
                    amplitude_profile_tracker.tone_detrended_aggregated_iqr()
                )
                continue
        if name in CHANNEL_COHERENCE_FEATURES and coherence_tracker is None:
            raise ValueError(
                f"{name} needs the channel coherence tracker; pass the "
                f"explicitly preprocessed stream"
            )
        if (
            name in SUBBAND_COHERENCE_FEATURES
            and not coherence_tracker.track_subbands
        ):
            raise ValueError(
                f"{name} needs subband coherence tracking; construct the "
                f"tracker with track_subbands=True"
            )
        if name == 'chan_coh_lag_ratio':
            values[name] = coherence_tracker.coherence_lag_ratio()
        elif name == 'chan_coh_mean':
            values[name] = coherence_tracker.mean_coherence()
        elif name == 'chan_coh_gap_low_frac':
            values[name] = coherence_tracker.coherence_gap_low_frac()
        elif name == 'chan_coh_gap_q20':
            values[name] = coherence_tracker.coherence_gap_q20()
        elif name == 'chan_coh_subband_median_gap':
            values[name] = coherence_tracker.coherence_subband_median_gap()
        elif name == 'chan_coh_gap':
            values[name] = coherence_tracker.coherence_gap()
        elif name == 'chan_coh_subband_gap_median':
            values[name] = coherence_tracker.coherence_subband_gap_median()
        elif name == 'phase_resid_lag_ratio':
            if phase_tracker is None:
                raise ValueError(
                    f"{name} needs the sanitized phase tracker"
                )
            values[name] = phase_tracker.phase_residual_lag_ratio()
        elif name == 'phase_closure_var_std':
            if phase_tracker is None:
                raise ValueError(
                    f"{name} needs the sanitized phase tracker"
                )
            values[name] = phase_tracker.phase_closure_variance_std()
        elif name == 'chan_shape_excess_path':
            if shape_trajectory_tracker is None:
                raise ValueError(
                    f"{name} needs the time-binned channel-shape tracker"
                )
            values[name] = shape_trajectory_tracker.excess_path()
        elif name == 'chan_shape_scale_curvature':
            if shape_trajectory_tracker is None:
                raise ValueError(
                    f"{name} needs the time-binned channel-shape tracker"
                )
            values[name] = shape_trajectory_tracker.scale_curvature()
        elif name == 'chan_shape_subband_rank_gap':
            if shape_trajectory_tracker is None:
                raise ValueError(
                    f"{name} needs the time-binned channel-shape tracker"
                )
            values[name] = shape_trajectory_tracker.subband_rank_gap()
        elif name == 'chan_shape_subband_kendall_lag_excess':
            if shape_trajectory_tracker is None:
                raise ValueError(
                    f"{name} needs the time-binned channel-shape tracker"
                )
            values[name] = (
                shape_trajectory_tracker.subband_kendall_lag_excess()
            )
        elif name == 'chan_shape_coherent_innovation_energy':
            if shape_trajectory_tracker is None:
                raise ValueError(
                    f"{name} needs the time-binned channel-shape tracker"
                )
            values[name] = (
                shape_trajectory_tracker.coherent_innovation_energy()
            )
        elif name == 'chan_shape_coherent_innovation_contrast':
            if shape_trajectory_tracker is None:
                raise ValueError(
                    f"{name} needs the time-binned channel-shape tracker"
                )
            values[name] = (
                shape_trajectory_tracker.coherent_innovation_contrast()
            )
        elif name == 'chan_shape_lag_ratio':
            if shape_tracker is None:
                raise ValueError(f"{name} needs the channel-shape tracker")
            values[name] = shape_tracker.shape_lag_ratio()
        elif name == 'chan_shape_spread_ds2':
            if shape_tracker is None:
                raise ValueError(f"{name} needs the channel-shape tracker")
            values[name] = shape_tracker.shape_spread_ds2()
        elif name == 'chan_shape_spread_ema_fast':
            if shape_tracker is None:
                raise ValueError(f"{name} needs the channel-shape tracker")
            values[name] = shape_tracker.shape_spread_ema_fast()
        elif name == 'chan_shape_spread_ema_slow':
            if shape_tracker is None:
                raise ValueError(f"{name} needs the channel-shape tracker")
            values[name] = shape_tracker.shape_spread_ema_slow()
        elif name == 'chan_rank_gap':
            if shape_tracker is None:
                raise ValueError(f"{name} needs the channel-shape tracker")
            values[name] = shape_tracker.rank_gap()
        elif name == 'chan_ratio_gap':
            if shape_tracker is None:
                raise ValueError(f"{name} needs the channel-shape tracker")
            values[name] = shape_tracker.ratio_gap()
        elif name == 'chan_freq_coh_cv':
            if shape_tracker is None:
                raise ValueError(f"{name} needs the channel-shape tracker")
            values[name] = shape_tracker.frequency_coherence_cv()
        elif name == 'chan_freq_coh_curve_iqr':
            if shape_tracker is None:
                raise ValueError(f"{name} needs the channel-shape tracker")
            values[name] = shape_tracker.frequency_coherence_curve_iqr()
        elif name == 'chan_freq_coh_curve_std':
            if shape_tracker is None:
                raise ValueError(f"{name} needs the channel-shape tracker")
            values[name] = shape_tracker.frequency_coherence_curve_std()
        elif name in (
            'chan_freq_coh_curve_2_4_std',
            'chan_freq_coh_curve_4_12_std',
            'chan_freq_coh_decay_std',
            'chan_freq_coh_curvature_std',
        ):
            if shape_tracker is None:
                raise ValueError(f"{name} needs the channel-shape tracker")
            values[name] = shape_tracker.frequency_coherence_candidate_std(name)
        elif name == 'chan_coh_gap_spread':
            if coherence_tracker is None or shape_tracker is None:
                raise ValueError(
                    f"{name} needs coherence and channel-shape trackers"
                )
            values[name] = max(
                0.0,
                coherence_tracker.coherence_gap(),
            ) * shape_tracker.shape_spread()
        elif name == 'corr_amp_d1':
            if amplitude_profile_tracker is None:
                raise ValueError(f"{name} needs the amplitude-profile tracker")
            values[name] = (
                amplitude_profile_tracker.adjacent_amplitude_correlation()
            )
    return values
