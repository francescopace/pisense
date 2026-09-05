# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Host feature extraction, stream identity, and feature caches."""

from __future__ import annotations

from tools.lib.bootstrap import setup_paths

setup_paths()

import ast
import copy
import hashlib
import inspect
import numpy as np
import textwrap
from collections.abc import Mapping
from functools import lru_cache
from pathlib import Path
from tools.lib.dataset_metadata import (
    measure_packet_interval_us,
)
from tools.lib.repo_paths import (
    tools_lib_dir,
    python_src_dir,
)
from tools.lib import npz_cache
from contextlib import contextmanager, nullcontext
from tools.lib.csi_io import load_npz_packet_view
from config import (
    DEFAULT_SUBCARRIERS,
    ENABLE_HAMPEL_FILTER,
    ENABLE_LOWPASS_FILTER,
    HAMPEL_THRESHOLD,
    HAMPEL_WINDOW,
    LOWPASS_CUTOFF,
    SEGMENTATION_WINDOW_SIZE_MS,
)
from tools.lib.runtime_policy import (
    nominal_packet_interval_us,
)
from tools.lib.temporal_csi_sampler import (
    TemporalCsiSampler,
    minimum_valid_slots,
    temporal_window_slots,
)
from tools.lib.ml_feature_trackers import (
    ChannelShapeTrajectoryTracker as ProductionChannelShapeTrajectoryTracker,
)
from tools.lib.performance_report import (
    load_or_compute_ml_replay_rows,
    timing_cadence_for_window,
)
from tools.lib.temporal_replay import (
    iter_temporal_admissions,
    packet_timestamp_us,
    target_pps_for_packets,
)
from tools.lib.csi_features import (
    AGGREGATED_TURBULENCE_FEATURES,
    ALL_FEATURES,
    DEFAULT_FEATURES,
    L1_DELTA_LAG,
    L1_TRACKER_FEATURES,
    L1DeltaTracker,
    TURB_IQR_AGGREGATION_WIDTH,
    calc_autocorrelation,
    calc_zero_crossing_rate,
    extract_features_by_name,
)
from tools.lib.candidate_features import (
    CANDIDATE_FEATURES,
    assemble_feature_vector,
    candidate_feature_cache_identity,
    candidate_values,
    needs_aggregated_turbulence,
    needs_amplitude_profiles,
    needs_channel_coherence,
    needs_channel_shape,
    needs_channel_shape_trajectory,
    needs_l1_series as needs_candidate_l1_series,
    needs_phase_residual,
    needs_subband_coherence,
    split_feature_names,
)
from tools.lib.host_feature_trackers import (
    AmplitudeProfileTracker,
    CHANNEL_SHAPE_BIN_US,
    ChannelCoherenceTracker,
    ChannelShapeTrajectoryTracker,
    ChannelShapeTracker,
    PhaseResidualTracker,
)
from tools.lib.high_accuracy_detector import (  # noqa: F401 (re-exported for tests)
    FEATURE_NAMES as EXPORTED_FEATURE_NAMES,
    HighAccuracyDetector,
    ProductionFeatureExtractor,
)

from .augmentation import (
    _implementation_source_digest,
    _packet_augmentation_stream_provenance,
    _prepare_feature_packets_for_record,
)

DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE = temporal_window_slots(
    100,
    SEGMENTATION_WINDOW_SIZE_MS,
)


def _needs_l1_tracker(feature_names):
    """Return whether any requested feature needs the L1-delta tracker."""
    return (
        any(name in L1_TRACKER_FEATURES for name in feature_names)
        or needs_candidate_l1_series(feature_names)
    )


def _needs_l1_series(feature_names):
    """Return whether any requested feature reads the rebuilt L1-delta series.

    The lag ratio needs the tracker but not the series, so the two questions
    are asked separately; mirrors MLFeatureSource in csi_features.h.
    """
    return needs_candidate_l1_series(feature_names)


TRAINING_FEATURES = DEFAULT_FEATURES


ACTIVE_TRAJECTORY_BIN_US = CHANNEL_SHAPE_BIN_US


DEFAULT_AGGREGATED_CANDIDATE_WIDTH = TURB_IQR_AGGREGATION_WIDTH


def selectable_features():
    """Names `--features` accepts.

    Host-side candidates widen the selectable set without touching the
    production surface the two runtimes share; the export guard still rejects
    them because they have no C++ extractor id. Resolved per call so tests can
    substitute the production surface.
    """
    return tuple(ALL_FEATURES) + tuple(CANDIDATE_FEATURES)


def set_active_trajectory_bin_ms(value):
    """Select the host-side trajectory bin used by read-only experiments."""
    global ACTIVE_TRAJECTORY_BIN_US
    milliseconds = int(value)
    if milliseconds < 1:
        raise ValueError("trajectory bin must be at least 1 ms")
    ACTIVE_TRAJECTORY_BIN_US = milliseconds * 1000


@contextmanager
def canonical_trajectory_bin():
    """Temporarily restore the production bin for exported-model baselines."""
    global ACTIVE_TRAJECTORY_BIN_US
    previous = ACTIVE_TRAJECTORY_BIN_US
    ACTIVE_TRAJECTORY_BIN_US = CHANNEL_SHAPE_BIN_US
    try:
        yield
    finally:
        ACTIVE_TRAJECTORY_BIN_US = previous


TRAINING_SAMPLE_CONTRACT = "stream_dense"


@lru_cache(maxsize=64)
def _host_feature_base_stream_provenance(feature_names, trajectory_bin_us):
    """Build shared host provenance once per ordered feature schema."""
    feature_identities = {
        name: _host_feature_cache_identity(name)
        for name in feature_names
    }
    for name, identity in feature_identities.items():
        if (
            name in CANDIDATE_FEATURES
            and identity.get('provider') == 'channel_shape_trajectory'
        ):
            identity['trajectory_bin_us'] = int(trajectory_bin_us)
    return {
        'transform': 'host_feature_rows_v4',
        'feature_names': list(feature_names),
        'row_stream': {
            'contract': 'host_feature_row_spine_v1',
            'timing_sources': {
                'runtime_policy': npz_cache.source_manifest(
                    tools_lib_dir() / 'runtime_policy.py'
                ),
                'temporal_csi_sampler': npz_cache.source_manifest(
                    tools_lib_dir() / 'temporal_csi_sampler.py'
                ),
                'config': npz_cache.source_manifest(
                    python_src_dir() / 'config.py'
                ),
                'row_builder_sha256': _implementation_source_digest(
                    build_host_feature_rows,
                    timing_cadence_for_window,
                    iter_temporal_admissions,
                    TemporalCsiSampler.admit,
                    ProductionFeatureExtractor.process_packet,
                    ProductionFeatureExtractor.advance_missing_slots,
                    ProductionFeatureExtractor.ordered_series,
                    ProductionFeatureExtractor.extract_features,
                    StreamingFeatureExtractor.process_packet,
                    StreamingFeatureExtractor.advance_missing_slots,
                ),
            },
        },
        'feature_identities': feature_identities,
    }


def _host_feature_stream_provenance(feature_names, *,
                                    packet_augmentation=None,
                                    augmentation_seed=None):
    """Return an isolated copy of memoized granular host provenance."""
    names = tuple(str(name) for name in feature_names)
    provenance = copy.deepcopy(_host_feature_base_stream_provenance(
        names,
        int(ACTIVE_TRAJECTORY_BIN_US),
    ))
    packet_provenance = _packet_augmentation_stream_provenance(
        packet_augmentation,
        augmentation_seed,
    )
    if packet_provenance is not None:
        provenance['packet_augmentation'] = packet_provenance
    return provenance


PRODUCTION_FEATURE_PROVIDER_VERSIONS = {
    'turbulence_window': 2,
    'aggregated_turbulence_window': 2,
    'l1_delta_tracker': 1,
    'channel_shape_trajectory': 2,
}


def _production_feature_provider(feature_name):
    """Return the independently versioned provider of one runtime feature."""
    if feature_name == 'turb_iqr_over_mean_aggr':
        return 'aggregated_turbulence_window'
    if feature_name in {'turb_autocorr', 'turb_zcr'}:
        return 'turbulence_window'
    if feature_name == 'l1_delta_lag_ratio':
        return 'l1_delta_tracker'
    if feature_name in {
        'chan_shape_spread_subband',
        'chan_shape_coherent_innovation_energy',
        'chan_shape_excess_path',
        'chan_shape_subband_kendall_lag_excess',
    }:
        return 'channel_shape_trajectory'
    raise ValueError(f'Unknown production feature: {feature_name}')


@lru_cache(maxsize=None)
def _named_feature_branch_digest(function, feature_name):
    """Hash only branches that explicitly implement one named feature."""
    tree = ast.parse(textwrap.dedent(inspect.getsource(function)))
    matching = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.If) or not isinstance(node.test, ast.Compare):
            continue
        operands = (node.test.left, *node.test.comparators)
        if not any(
            isinstance(item, ast.Name) and item.id == 'name'
            for item in operands
        ):
            continue
        if not any(
            isinstance(item, ast.Constant) and item.value == feature_name
            for item in operands
        ):
            continue
        matching.append(
            ast.dump(
                ast.Module(body=node.body, type_ignores=[]),
                annotate_fields=True,
                include_attributes=False,
            )
        )
    payload = '\n'.join(matching)
    if not payload:
        raise ValueError(f'No extraction branch found for {feature_name}')
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()


@lru_cache(maxsize=None)
def _production_provider_digest(provider):
    """Hash shared provider behavior without hashing unrelated siblings."""
    if provider == 'turbulence_window':
        return _implementation_source_digest(
            calc_autocorrelation,
            calc_zero_crossing_rate,
        )
    if provider == 'aggregated_turbulence_window':
        return hashlib.sha256(
            f'aggregation-width:{TURB_IQR_AGGREGATION_WIDTH}'.encode('utf-8')
        ).hexdigest()
    if provider == 'l1_delta_tracker':
        return _implementation_source_digest(
            L1DeltaTracker.__init__,
            L1DeltaTracker.process_amplitudes,
            L1DeltaTracker.delta_lag_ratio,
            L1DeltaTracker.reset,
        )
    if provider == 'channel_shape_trajectory':
        return _implementation_source_digest(
            ProductionChannelShapeTrajectoryTracker.__init__,
            ProductionChannelShapeTrajectoryTracker.reset,
            ProductionChannelShapeTrajectoryTracker.process_packet,
            ProductionChannelShapeTrajectoryTracker._bin_at,
            ProductionChannelShapeTrajectoryTracker._kendall_at,
            ProductionChannelShapeTrajectoryTracker._modes,
            ProductionChannelShapeTrajectoryTracker.trajectory_features_with_spread,
            ProductionChannelShapeTrajectoryTracker.subband_kendall_lag_excess,
        )
    raise ValueError(f'Unknown production provider: {provider}')


@lru_cache(maxsize=None)
def _host_feature_cache_identity_cached(name):
    """Build one memoized host-feature identity."""
    if name in CANDIDATE_FEATURES:
        return candidate_feature_cache_identity(name)
    provider = _production_feature_provider(name)
    return {
        'feature_name': name,
        'provider': provider,
        'provider_version': PRODUCTION_FEATURE_PROVIDER_VERSIONS[provider],
        'provider_sha256': _production_provider_digest(provider),
        'formula_sha256': _named_feature_branch_digest(
            extract_features_by_name,
            name,
        ),
    }


def _host_feature_cache_identity(feature_name):
    """Return an isolated copy of one memoized host-feature identity."""
    return dict(_host_feature_cache_identity_cached(str(feature_name)))


def _host_row_stream_identity(stream_provenance):
    """Strip the requested schema while retaining stream-transform identity."""
    provenance = dict(stream_provenance or {})
    return {
        'transform': provenance.get('transform', 'host_feature_rows_v3'),
        'row_stream': provenance.get('row_stream', {}),
        'packet_augmentation': provenance.get('packet_augmentation'),
    }


def _mix_packet_augmentation_replay_rows(view_rows):
    """Return one constant-size deterministic mix of augmented replay views.

    View ``i`` contributes row positions congruent to ``i`` modulo the number
    of views. The rule is local to each source capture, so adding or removing a
    different capture cannot change an existing file's assignments.
    """
    rows_by_view = list(view_rows)
    if not rows_by_view:
        raise ValueError("at least one packet-augmentation view is required")
    feature_names = list(rows_by_view[0]['feature_names'])
    row_keys = ('X', 'packet_index', 'evaluation_index', 'reset_index', 'evaluation_due')
    selected = {key: [] for key in row_keys}
    view_count = len(rows_by_view)
    for view_index, rows in enumerate(rows_by_view):
        if list(rows['feature_names']) != feature_names:
            raise ValueError("packet-augmentation views have different feature schemas")
        row_count = int(len(rows['X']))
        mask = np.arange(row_count, dtype=np.int64) % view_count == view_index
        for key in row_keys:
            default_dtype = bool if key == 'evaluation_due' else np.int32
            if key == 'X':
                values = np.asarray(rows[key], dtype=np.float32)
            else:
                values = np.asarray(rows.get(key, np.empty(0)), dtype=default_dtype)
            if len(values) != row_count:
                raise ValueError(f"packet-augmentation row field {key} has inconsistent length")
            selected[key].append(values[mask])
    return {
        'X': np.concatenate(selected['X'], axis=0).astype(np.float32, copy=False),
        'feature_names': feature_names,
        'packet_index': np.concatenate(selected['packet_index']).astype(np.int32, copy=False),
        'evaluation_index': np.concatenate(selected['evaluation_index']).astype(np.int32, copy=False),
        'reset_index': np.concatenate(selected['reset_index']).astype(np.int32, copy=False),
        'evaluation_due': np.concatenate(selected['evaluation_due']).astype(bool, copy=False),
    }


def _concatenate_packet_augmentation_replay_rows(view_rows):
    """Concatenate packet-augmentation views already selected by row position."""
    rows_by_view = list(view_rows)
    if not rows_by_view:
        raise ValueError("at least one packet-augmentation view is required")
    feature_names = list(rows_by_view[0]['feature_names'])
    row_keys = ('X', 'packet_index', 'evaluation_index', 'reset_index', 'evaluation_due')
    combined = {key: [] for key in row_keys}
    for rows in rows_by_view:
        if list(rows['feature_names']) != feature_names:
            raise ValueError("packet-augmentation views have different feature schemas")
        row_count = int(len(rows['X']))
        for key in row_keys:
            default_dtype = bool if key == 'evaluation_due' else np.int32
            values = np.asarray(
                rows[key] if key == 'X' else rows.get(key, np.empty(0)),
                dtype=np.float32 if key == 'X' else default_dtype,
            )
            if len(values) != row_count:
                raise ValueError(
                    f"packet-augmentation row field {key} has inconsistent length"
                )
            combined[key].append(values)
    return {
        'X': np.concatenate(combined['X'], axis=0).astype(np.float32, copy=False),
        'feature_names': feature_names,
        'packet_index': np.concatenate(combined['packet_index']).astype(np.int32, copy=False),
        'evaluation_index': np.concatenate(combined['evaluation_index']).astype(np.int32, copy=False),
        'reset_index': np.concatenate(combined['reset_index']).astype(np.int32, copy=False),
        'evaluation_due': np.concatenate(combined['evaluation_due']).astype(bool, copy=False),
    }


def _packet_augmentation_mix_stream_provenance(packet_augmentation,
                                                augmentation_seeds,
                                                feature_names,
                                                use_runtime_cache):
    """Return the complete identity for the promoted mixed-view row cache."""
    seeds = tuple(int(seed) for seed in augmentation_seeds)
    if not packet_augmentation or not seeds:
        return None
    if use_runtime_cache:
        views = [
            _packet_augmentation_stream_provenance(packet_augmentation, seed)
            for seed in seeds
        ]
    else:
        views = [
            _host_feature_stream_provenance(
                feature_names,
                packet_augmentation=packet_augmentation,
                augmentation_seed=seed,
            )
            for seed in seeds
        ]
    return {
        'transform': 'training_packet_augmentation_mix_v1',
        'views': views,
        'selection': {
            'scope': 'source_file',
            'rule': 'row_position_modulo_view_count',
            'offsets': list(range(len(seeds))),
        },
        'implementation_sha256': _implementation_source_digest(
            _mix_packet_augmentation_replay_rows,
            _concatenate_packet_augmentation_replay_rows,
        ),
    }


def _load_or_compute_packet_augmentation_mix_rows(record, *,
                                                   packet_augmentation,
                                                   augmentation_seeds,
                                                   feature_names,
                                                   use_cache,
                                                   use_runtime_cache):
    """Load or build one cached deterministic mix for a source capture."""
    seeds = tuple(int(seed) for seed in augmentation_seeds)
    if len(seeds) < 2:
        raise ValueError("mixed packet augmentation requires at least two seeds")
    mix_provenance = _packet_augmentation_mix_stream_provenance(
        packet_augmentation,
        seeds,
        feature_names,
        use_runtime_cache,
    )
    parameters = npz_cache.ml_training_augmentation_row_parameters(
        selected_subcarriers=DEFAULT_SUBCARRIERS,
        feature_names=feature_names,
        stream_provenance=mix_provenance,
    )
    if use_cache:
        cached = npz_cache.load_ml_training_augmentation_row_artifact(
            record['path'],
            parameters=parameters,
        )
        if cached is not None:
            cached['cache_hit'] = True
            return cached
    lock_context = (
        npz_cache.artifact_build_lock(
            record['path'],
            artifact_name='ml_training_augmentation_rows',
            artifact_version=npz_cache.ML_TRAINING_AUGMENTATION_ROW_ARTIFACT_VERSION,
            parameters=parameters,
        )
        if use_cache
        else nullcontext()
    )
    with lock_context:
        if use_cache:
            cached = npz_cache.load_ml_training_augmentation_row_artifact(
                record['path'],
                parameters=parameters,
            )
            if cached is not None:
                cached['cache_hit'] = True
                return cached
        views = []
        for view_index, seed in enumerate(seeds):
            def packets_factory(current_record=record, current_seed=seed):
                return _prepare_feature_packets_for_record(
                    current_record,
                    packet_augmentation=packet_augmentation,
                    augmentation_seed=current_seed,
                )
            if use_runtime_cache:
                rows = load_or_compute_ml_replay_rows(
                    record['path'],
                    packets_factory=packets_factory,
                    selected_subcarriers=DEFAULT_SUBCARRIERS,
                    window_size=None,
                    feature_names=feature_names,
                    sample_contract=TRAINING_SAMPLE_CONTRACT,
                    use_cache=use_cache,
                    cache_write=False,
                    stream_provenance=_packet_augmentation_stream_provenance(
                        packet_augmentation,
                        seed,
                    ),
                    row_stride=len(seeds),
                    row_offset=view_index,
                )
            else:
                rows = load_or_compute_host_feature_rows(
                    record['path'],
                    packets_factory=packets_factory,
                    feature_names=feature_names,
                    sample_contract=TRAINING_SAMPLE_CONTRACT,
                    use_cache=use_cache,
                    cache_write=True,
                    stream_provenance=_host_feature_stream_provenance(
                        feature_names,
                        packet_augmentation=packet_augmentation,
                        augmentation_seed=seed,
                    ),
                )
                rows = _select_host_feature_rows(rows, len(seeds), view_index)
            views.append(rows)

        mixed = _concatenate_packet_augmentation_replay_rows(views)
        if use_cache:
            npz_cache.save_ml_training_augmentation_row_artifact(
                record['path'],
                parameters=parameters,
                rows=mixed,
            )
    mixed['cache_hit'] = False
    return mixed


class StreamingFeatureExtractor:
    """Compute runtime-equivalent feature vectors from a CSI packet stream."""

    def __init__(
        self,
        feature_names,
        window_packets=DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        packet_interval_us=None,
    ):
        self.feature_names = list(feature_names)
        self.window_packets = max(1, int(window_packets))
        self.packet_interval_us = max(
            1,
            int(
                packet_interval_us
                if packet_interval_us is not None
                else nominal_packet_interval_us(self.window_packets)
            ),
        )
        self.trajectory_elapsed_us = 0
        self.trajectory_packet_count = 0
        self.production_names, self.candidate_names = split_feature_names(
            self.feature_names
        )
        self.needs_l1_tracker = _needs_l1_tracker(self.feature_names)
        self.needs_l1_series = _needs_l1_series(self.feature_names)
        l1_capacity = max(2, self.window_packets - L1_DELTA_LAG)
        needs_aggregated = (
            any(
                name in AGGREGATED_TURBULENCE_FEATURES
                for name in self.feature_names
            )
            or needs_aggregated_turbulence(self.feature_names)
        )
        self.production_extractor = ProductionFeatureExtractor(
            self.production_names,
            window_size=self.window_packets,
            enable_lowpass=ENABLE_LOWPASS_FILTER,
            lowpass_cutoff=LOWPASS_CUTOFF,
            enable_hampel=ENABLE_HAMPEL_FILTER,
            hampel_window=HAMPEL_WINDOW,
            hampel_threshold=HAMPEL_THRESHOLD,
            force_aggregated=needs_aggregated,
            force_l1_tracker=self.needs_l1_tracker,
        )
        self.context = self.production_extractor.context
        self.aggregated_context = self.production_extractor.aggregated_context
        self.l1_tracker = self.production_extractor.l1_tracker
        self.l1_series = [0.0] * l1_capacity if self.needs_l1_series else None
        self.coherence_tracker = (
            ChannelCoherenceTracker(
                window_size=l1_capacity,
                lag=L1_DELTA_LAG,
                track_subbands=needs_subband_coherence(self.candidate_names),
            )
            if needs_channel_coherence(self.candidate_names) else None
        )
        self.phase_tracker = (
            PhaseResidualTracker(window_size=l1_capacity, lag=L1_DELTA_LAG)
            if needs_phase_residual(self.candidate_names) else None
        )
        self.shape_tracker = (
            ChannelShapeTracker(
                window_size=l1_capacity,
                lag=L1_DELTA_LAG,
                feature_names=self.candidate_names,
            )
            if needs_channel_shape(self.candidate_names) else None
        )
        self.shape_trajectory_tracker = (
            ChannelShapeTrajectoryTracker(
                window_duration_us=SEGMENTATION_WINDOW_SIZE_MS * 1000,
                bin_us=ACTIVE_TRAJECTORY_BIN_US,
                track_subband_rank_gap=(
                    'chan_shape_subband_rank_gap' in self.candidate_names
                ),
                track_subband_kendall_lag_excess=(
                    'chan_shape_subband_kendall_lag_excess'
                    in self.candidate_names
                ),
            )
            if needs_channel_shape_trajectory(self.candidate_names) else None
        )
        self.amplitude_profile_tracker = (
            AmplitudeProfileTracker(window_size=self.window_packets)
            if needs_amplitude_profiles(self.candidate_names) else None
        )

    def _trajectory_timestamp_us(self, packet, timestamp_us=None):
        if timestamp_us is not None:
            return int(timestamp_us)
        resolved = packet_timestamp_us(
            packet,
            fallback_index=self.trajectory_packet_count,
            fallback_interval_us=self.packet_interval_us,
        ) if packet is not None else None
        if resolved is not None:
            self.trajectory_packet_count += 1
            return int(resolved)
        if self.trajectory_packet_count == 0:
            self.trajectory_packet_count = 1
            return 0
        self.trajectory_elapsed_us += self.packet_interval_us
        self.trajectory_packet_count += 1
        return self.trajectory_elapsed_us

    def advance_missing_slots(self, count):
        """Preserve temporal holes in every tracker that owns slot state."""
        missing = max(0, int(count))
        self.production_extractor.advance_missing_slots(missing)
        for tracker in (
            self.coherence_tracker,
            self.phase_tracker,
            self.shape_tracker,
            self.amplitude_profile_tracker,
        ):
            advance = getattr(tracker, 'advance_missing_slots', None)
            if callable(advance):
                advance(missing)

    def is_ready(self, minimum_valid_samples=None):
        """Match the production detector readiness contract for host rows."""
        return self.production_extractor.is_ready(minimum_valid_samples)

    def process_packet(self, csi_data, packet=None, timestamp_us=None):
        needs_timestamp = (
            self.production_extractor.shape_trajectory_tracker is not None
            or self.shape_trajectory_tracker is not None
        )
        resolved_timestamp = (
            self._trajectory_timestamp_us(packet, timestamp_us)
            if needs_timestamp else timestamp_us
        )
        self.production_extractor.process_packet(
            csi_data,
            DEFAULT_SUBCARRIERS,
            resolved_timestamp,
        )
        if self.amplitude_profile_tracker is not None:
            amplitudes, amplitude_count = self.production_extractor.packet_amplitudes
            aggregated_amplitudes, aggregated_count = (
                self.production_extractor.aggregated_amplitudes
            )
            self.amplitude_profile_tracker.process_amplitudes(
                amplitudes[:amplitude_count],
                (
                    aggregated_amplitudes[:aggregated_count]
                    if aggregated_amplitudes is not None else None
                ),
            )
        if self.coherence_tracker is not None:
            self.coherence_tracker.process_packet(csi_data)
        if self.phase_tracker is not None:
            self.phase_tracker.process_packet(csi_data)
        if self.shape_tracker is not None:
            self.shape_tracker.process_packet(csi_data)
        if self.shape_trajectory_tracker is not None:
            self.shape_trajectory_tracker.process_packet(
                csi_data,
                resolved_timestamp,
            )
        if self.context.buffer_count < self.context.window_size:
            return None

        turb_values, _, turb_count = self.production_extractor.ordered_series()
        turb_list = list(turb_values[:turb_count])
        aggregated_turb_list = None
        if self.aggregated_context is not None:
            aggregated_values, _, aggregated_count = (
                self.production_extractor.ordered_series(aggregated=True)
            )
            aggregated_turb_list = list(aggregated_values[:aggregated_count])
        l1_count = (
            self.l1_tracker.copy_deltas_into(self.l1_series)
            if self.l1_series is not None else 0
        )
        features = self.production_extractor.extract_features()
        if not self.candidate_names:
            return features
        return assemble_feature_vector(
            self.feature_names,
            self.production_names,
            features,
            candidate_values(
                self.candidate_names,
                self.coherence_tracker,
                turbulence_series=turb_list,
                aggregated_turbulence_series=aggregated_turb_list,
                phase_tracker=self.phase_tracker,
                shape_tracker=self.shape_tracker,
                shape_trajectory_tracker=self.shape_trajectory_tracker,
                amplitude_profile_tracker=self.amplitude_profile_tracker,
                l1_series=(
                    self.l1_series[:l1_count]
                    if self.l1_series is not None else None
                ),
            ),
        )


def packet_csi_data(packet):
    """Return CSI data from a packet mapping or a compact matrix row."""
    return packet['csi_data'] if isinstance(packet, Mapping) else packet


def _normalize_feature_row_contract(sample_contract):
    """Normalize one local feature-row sampling contract."""
    contract = str(sample_contract).strip().lower()
    if contract not in {'replay_tick', 'stream_dense'}:
        raise ValueError(f"Unsupported ML sample contract: {sample_contract!r}")
    return contract


def _feature_rows_use_runtime_cache(feature_names):
    """Return True when canonical runtime replay rows can serve this request."""
    return (
        ACTIVE_TRAJECTORY_BIN_US == CHANNEL_SHAPE_BIN_US
        and _feature_names_support_replay_rows(feature_names)
    )


def _empty_feature_rows(feature_names):
    """Return one empty feature-row payload for the requested schema."""
    resolved = [str(name) for name in feature_names]
    return {
        'X': np.empty((0, len(resolved)), dtype=np.float32),
        'feature_names': resolved,
        'packet_index': np.empty(0, dtype=np.int32),
        'evaluation_index': np.empty(0, dtype=np.int32),
        'reset_index': np.empty(0, dtype=np.int32),
        'evaluation_due': np.empty(0, dtype=bool),
    }


def _normalize_host_row_selection(row_stride, row_offset):
    """Validate an optional dense host-row modulo selection."""
    if row_stride is None:
        if int(row_offset) != 0:
            raise ValueError("row_offset requires row_stride")
        return None, 0
    stride = int(row_stride)
    offset = int(row_offset)
    if stride < 1 or offset < 0 or offset >= stride:
        raise ValueError("row selection requires 0 <= row_offset < row_stride")
    return stride, offset


def _select_host_feature_rows(rows, row_stride, row_offset):
    """Select cached dense host rows by position."""
    stride, offset = _normalize_host_row_selection(row_stride, row_offset)
    if stride is None:
        return rows
    row_count = len(np.asarray(rows.get('packet_index', ())))
    mask = np.arange(row_count, dtype=np.int64) % stride == offset
    selected = {
        'X': np.asarray(rows['X'], dtype=np.float32)[mask],
        'feature_names': list(rows['feature_names']),
        'packet_index': np.asarray(rows['packet_index'], dtype=np.int32)[mask],
        'evaluation_index': np.asarray(rows['evaluation_index'], dtype=np.int32)[mask],
        'reset_index': np.asarray(rows['reset_index'], dtype=np.int32)[mask],
        'evaluation_due': np.asarray(rows['evaluation_due'], dtype=bool)[mask],
    }
    if 'cache_hit' in rows:
        selected['cache_hit'] = bool(rows['cache_hit'])
    return selected


def build_host_feature_rows(packets, feature_names, *,
                            sample_contract='replay_tick',
                            row_stride=None, row_offset=0):
    """Build reset-aware feature rows through the host streaming path."""
    requested_feature_names = [str(name) for name in feature_names]
    normalized_contract = _normalize_feature_row_contract(sample_contract)
    stride, offset = _normalize_host_row_selection(row_stride, row_offset)
    if stride is not None and normalized_contract != 'stream_dense':
        raise ValueError("dense row selection requires sample_contract='stream_dense'")
    if not packets:
        return _empty_feature_rows(requested_feature_names)

    interval_us = measure_packet_interval_us(packets)
    target_pps = target_pps_for_packets(packets, interval_us)
    window_packets = temporal_window_slots(
        target_pps,
        SEGMENTATION_WINDOW_SIZE_MS,
    )
    _, cadence = timing_cadence_for_window(window_packets, interval_us)
    extractor = StreamingFeatureExtractor(
        requested_feature_names,
        window_packets,
        packet_interval_us=interval_us,
    )
    minimum_samples = minimum_valid_slots(window_packets)
    packets_since_reset = 0
    reset_index = 0
    evaluation_index = 0
    row_features = []
    packet_index_values = []
    evaluation_index_values = []
    reset_index_values = []
    evaluation_due_values = []

    for admission in iter_temporal_admissions(
        packets,
        target_pps=target_pps,
        window_size_ms=SEGMENTATION_WINDOW_SIZE_MS,
        fallback_interval_us=interval_us,
    ):
        packet_index = admission.packet_index
        packet = admission.packet
        if admission.reset_required:
            extractor = StreamingFeatureExtractor(
                requested_feature_names,
                window_packets,
                packet_interval_us=interval_us,
            )
            cadence.reset()
            reset_index += 1
            packets_since_reset = 0
        elif admission.missing_slots_before:
            extractor.advance_missing_slots(admission.missing_slots_before)
        cadence.note_packet(elapsed_us=admission.coverage_us)
        should_evaluate = cadence.should_evaluate()
        if should_evaluate:
            cadence.after_evaluation()
        values = extractor.process_packet(
            packet_csi_data(packet),
            packet=packet,
            timestamp_us=admission.timestamp_us,
        )
        packets_since_reset = admission.slot_index + 1
        if (
            values is None
            or packets_since_reset < window_packets
            or not extractor.is_ready(minimum_samples)
        ):
            continue
        dense_row_index = evaluation_index
        evaluation_index += 1
        if stride is not None and dense_row_index % stride != offset:
            continue
        row_features.append(np.asarray(values, dtype=np.float32))
        packet_index_values.append(int(packet_index))
        evaluation_index_values.append(int(dense_row_index))
        reset_index_values.append(int(reset_index))
        evaluation_due_values.append(bool(should_evaluate))

    if not row_features:
        return _empty_feature_rows(requested_feature_names)

    dense_rows = {
        'X': np.vstack(row_features).astype(np.float32, copy=False),
        'feature_names': requested_feature_names,
        'packet_index': np.asarray(packet_index_values, dtype=np.int32),
        'evaluation_index': np.asarray(evaluation_index_values, dtype=np.int32),
        'reset_index': np.asarray(reset_index_values, dtype=np.int32),
        'evaluation_due': np.asarray(evaluation_due_values, dtype=bool),
    }
    row_mask = (
        dense_rows['evaluation_due']
        if normalized_contract == 'replay_tick'
        else np.ones(len(dense_rows['packet_index']), dtype=bool)
    )
    projected = {
        'X': dense_rows['X'][row_mask],
        'feature_names': requested_feature_names,
        'packet_index': dense_rows['packet_index'][row_mask],
        'evaluation_index': dense_rows['evaluation_index'][row_mask],
        'reset_index': dense_rows['reset_index'][row_mask],
        'evaluation_due': dense_rows['evaluation_due'][row_mask],
    }
    if normalized_contract == 'replay_tick':
        projected['evaluation_index'] = np.arange(
            len(projected['packet_index']),
            dtype=np.int32,
        )
    return projected


def load_or_compute_host_feature_rows(source_path, *,
                                      packets=None,
                                      packets_factory=None,
                                      feature_names=(),
                                      sample_contract='replay_tick',
                                      use_cache=True,
                                      cache_write=True,
                                      stream_provenance=None,
                                      row_stride=None,
                                      row_offset=0):
    """Load or assemble host rows from independently cached feature columns."""
    if packets is not None and packets_factory is not None:
        raise ValueError("pass packets or packets_factory, not both")
    requested_feature_names = [str(name) for name in feature_names]
    normalized_contract = _normalize_feature_row_contract(sample_contract)
    stride, offset = _normalize_host_row_selection(row_stride, row_offset)
    if stride is not None and normalized_contract != 'stream_dense':
        raise ValueError("dense row selection requires sample_contract='stream_dense'")
    if stride is not None and use_cache and cache_write:
        raise ValueError("selected rows cannot be written under a full-row cache key")
    resolved_provenance = stream_provenance or _host_feature_stream_provenance(
        requested_feature_names
    )
    stream_identity = _host_row_stream_identity(resolved_provenance)
    spine_parameters = {
        'contract': 'host_feature_row_spine_v1',
        'selected_subcarriers': [int(value) for value in DEFAULT_SUBCARRIERS],
        'stream': stream_identity,
    }

    def feature_parameters(name):
        identities = resolved_provenance.get('feature_identities', {})
        identity = identities.get(name) or _host_feature_cache_identity(name)
        return {
            'contract': 'host_feature_column_v1',
            'spine': spine_parameters,
            'feature': identity,
        }

    def load_cached_parts():
        spine = npz_cache.load_host_feature_row_spine_artifact(
            source_path,
            parameters=spine_parameters,
        )
        if spine is None:
            return None, {}
        row_count = len(spine['packet_index'])
        columns = {}
        for name in requested_feature_names:
            column = npz_cache.load_host_feature_column_artifact(
                source_path,
                parameters=feature_parameters(name),
            )
            if column is not None and len(column) == row_count:
                columns[name] = column
        return spine, columns

    def assemble(spine, columns, *, cache_hit):
        row_count = len(spine['packet_index'])
        matrix = (
            np.column_stack([columns[name] for name in requested_feature_names])
            .astype(np.float32, copy=False)
            if requested_feature_names
            else np.empty((row_count, 0), dtype=np.float32)
        )
        rows = {
            'X': matrix,
            'feature_names': list(requested_feature_names),
            **spine,
            'cache_hit': bool(cache_hit),
        }
        if normalized_contract == 'stream_dense':
            return _select_host_feature_rows(rows, stride, offset)
        row_mask = np.asarray(rows['evaluation_due'], dtype=bool)
        return {
            'X': matrix[row_mask],
            'feature_names': list(requested_feature_names),
            'packet_index': np.asarray(rows['packet_index'], dtype=np.int32)[row_mask],
            'evaluation_index': np.arange(int(np.sum(row_mask)), dtype=np.int32),
            'reset_index': np.asarray(rows['reset_index'], dtype=np.int32)[row_mask],
            'evaluation_due': np.asarray(rows['evaluation_due'], dtype=bool)[row_mask],
            'cache_hit': bool(cache_hit),
        }

    spine, columns = load_cached_parts() if use_cache else (None, {})
    missing = [name for name in requested_feature_names if name not in columns]
    if spine is not None and not missing:
        return assemble(spine, columns, cache_hit=True)
    if stride is not None:
        if packets is not None:
            packet_stream = packets
        elif packets_factory is not None:
            packet_stream = packets_factory()
        else:
            packet_stream = load_npz_packet_view(Path(source_path))
        rows = build_host_feature_rows(
            packet_stream,
            requested_feature_names,
            sample_contract='stream_dense',
            row_stride=stride,
            row_offset=offset,
        )
        rows['cache_hit'] = False
        return rows

    build_lock_parameters = {
        'contract': 'host_feature_column_build_v1',
        'spine': spine_parameters,
    }
    lock_context = (
        npz_cache.artifact_build_lock(
            source_path,
            artifact_name='host_feature_columns',
            artifact_version=npz_cache.HOST_FEATURE_COLUMN_ARTIFACT_VERSION,
            parameters=build_lock_parameters,
        )
        if use_cache and cache_write and missing
        else nullcontext()
    )
    with lock_context:
        if use_cache:
            spine, columns = load_cached_parts()
            missing = [name for name in requested_feature_names if name not in columns]
            if spine is not None and not missing:
                return assemble(spine, columns, cache_hit=True)

        if packets is not None:
            packet_stream = packets
        elif packets_factory is not None:
            packet_stream = packets_factory()
        else:
            packet_stream = load_npz_packet_view(Path(source_path))
        computed_names = missing or requested_feature_names
        computed = build_host_feature_rows(
            packet_stream,
            computed_names,
            sample_contract='stream_dense',
            row_stride=stride,
            row_offset=offset,
        )
        computed_spine = {
            key: np.asarray(computed[key])
            for key in (
                'packet_index',
                'evaluation_index',
                'reset_index',
                'evaluation_due',
            )
        }
        if spine is not None:
            for key, values in computed_spine.items():
                if not np.array_equal(values, np.asarray(spine[key])):
                    raise RuntimeError(
                        f"cached host feature row spine diverged for {key}"
                    )
        else:
            spine = computed_spine
            if use_cache and cache_write:
                npz_cache.save_host_feature_row_spine_artifact(
                    source_path,
                    parameters=spine_parameters,
                    rows=spine,
                )
        computed_matrix = np.asarray(computed['X'], dtype=np.float32)
        for index, name in enumerate(computed_names):
            column = computed_matrix[:, index]
            columns[name] = column
            if use_cache and cache_write:
                npz_cache.save_host_feature_column_artifact(
                    source_path,
                    parameters=feature_parameters(name),
                    values=column,
                )
        return assemble(spine, columns, cache_hit=False)


def _feature_names_support_replay_rows(feature_names):
    """Return True when a feature set can use canonical runtime replay rows."""
    return all(str(name) in EXPORTED_FEATURE_NAMES for name in feature_names)
