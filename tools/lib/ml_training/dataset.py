# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Dataset loading, filtering, provenance, and training matrices."""

from __future__ import annotations

from tools.lib.bootstrap import setup_paths

setup_paths()

import argparse
import json
import numpy as np
from functools import lru_cache
from pathlib import Path
from tools.lib.dataset_metadata import (
    dataset_role,
    measure_packet_interval_us,  # noqa: F401 - re-exported for training callers
)
from tools.lib.repo_paths import (
    cpp_core_dir,
    generated_data_dir,
    tools_lib_dir,
    python_src_dir,
)
from tools.lib import npz_cache
from tools.lib.timing_quality import summarize_capture_timing
from datetime import datetime
from time import perf_counter
from tools.lib.csi_io import load_npz_packet_view
from tools.lib.dataset_metadata import DATA_DIR
from config import (
    DEFAULT_SUBCARRIERS,
)
from tools.lib.performance_report import (
    load_or_compute_ml_replay_rows,
)
from tools.lib.csi_features import (
    DEFAULT_FEATURES,
)

from .augmentation import (
    _build_sample_context_for_replay_rows,
    _ensure_record_packets,
    _first_non_empty,
    _implementation_source_digest,
    _packet_augmentation_stream_provenance,
    _prepare_feature_packets_for_record,
)

from .feature_cache import (
    TRAINING_SAMPLE_CONTRACT,
    _feature_names_support_replay_rows,
    _feature_rows_use_runtime_cache,
    _host_feature_stream_provenance,
    _load_or_compute_packet_augmentation_mix_rows,
    load_or_compute_host_feature_rows,
)

def format_duration(seconds):
    """Render elapsed time in a compact human-readable form."""
    seconds = float(seconds)
    if seconds < 1.0:
        return f"{seconds * 1000:.0f} ms"
    if seconds < 60.0:
        return f"{seconds:.2f} s"
    minutes, rem = divmod(seconds, 60.0)
    return f"{int(minutes)}m {rem:.1f}s"


BINARY_TRAINING_LABELS = ('empty', 'static_presence', 'motion')


GENERATED_DATA_DIR = generated_data_dir()


SRC_DIR = python_src_dir()


REFERENCE_SRC_DIR = tools_lib_dir()


CPP_DIR = cpp_core_dir()


DEFAULT_EXCLUDED_CHIPS = ()


DATASET_ROLES = ('train', 'selection', 'holdout', 'exclude')


DEFAULT_TRAINING_ROLES = ('train',)


DEFAULT_TIMING_QUALITY_POLICY = "keep"


DEFAULT_TIMING_WARN_WEIGHT = 0.5


TIMING_QUALITY_POLICIES = (
    "keep",
    "exclude-fail",
    "downweight-warn",
    "exclude-fail-downweight-warn",
)


def load_dataset_info():
    """Load dataset_info.json with label mappings."""
    info_path = DATA_DIR / 'dataset_info.json'
    if info_path.exists():
        with open(info_path, 'r') as f:
            return json.load(f)
    return {'labels': {}}


def parse_environment_filter(value):
    """Normalize a comma-separated environment filter into a set."""
    if value is None:
        return None
    if isinstance(value, (list, tuple, set)):
        items = value
    else:
        items = str(value).split(',')
    normalized = {str(item).strip() for item in items if str(item).strip()}
    return normalized or None


def parse_chip_filter(value):
    """Normalize a comma-separated chip filter into an uppercase set."""
    if value is None:
        return None
    if isinstance(value, (list, tuple, set)):
        items = value
    else:
        items = str(value).split(',')
    normalized = {str(item).strip().upper() for item in items if str(item).strip()}
    return normalized or None


def parse_timing_quality_policy(value):
    """Normalize one timing-quality policy name."""
    if value is None:
        return DEFAULT_TIMING_QUALITY_POLICY
    policy = str(value).strip().lower()
    if policy not in TIMING_QUALITY_POLICIES:
        raise argparse.ArgumentTypeError(
            "timing-quality policy must be one of: "
            + ", ".join(TIMING_QUALITY_POLICIES)
        )
    return policy


def normalize_allowed_labels(labels):
    """Normalize an iterable of labels to a lowercase set."""
    if labels is None:
        return None
    return {str(label).strip().lower() for label in labels if str(label).strip()} or None


def normalize_dataset_roles(roles, *, default=DEFAULT_TRAINING_ROLES):
    """Normalize dataset roles and reject unknown role names."""
    if roles is None:
        roles = default
    if isinstance(roles, str):
        roles = roles.split(',')
    normalized = {str(role).strip().lower() for role in roles if str(role).strip()}
    unknown = normalized.difference(DATASET_ROLES)
    if unknown:
        raise ValueError(f"Unsupported dataset role(s): {', '.join(sorted(unknown))}")
    return normalized


def timing_policy_excludes_status(status, policy):
    """Return True when one timing-quality status is filtered out."""
    return str(policy) in ("exclude-fail", "exclude-fail-downweight-warn") and str(status) == "FAIL"


def timing_policy_weight(status, policy, warn_weight=DEFAULT_TIMING_WARN_WEIGHT):
    """Return one per-window multiplier derived from timing provenance."""
    if (
        str(policy) in ("downweight-warn", "exclude-fail-downweight-warn")
        and str(status) == "WARN"
    ):
        return float(warn_weight)
    return 1.0


def apply_positive_chip_boost(sample_weights, sample_context, y, chip_boosts):
    """
    Boost motion samples for specific chips, then renormalize overall mean to 1.0.
    """
    if chip_boosts is None:
        return sample_weights, {}
    if sample_context is None or 'chip' not in sample_context:
        return sample_weights, {}

    weights = np.asarray(sample_weights, dtype=np.float32).copy()
    chips = np.asarray(sample_context['chip']).astype(str)
    labels = np.asarray(y)
    summary = {}

    for chip, factor in sorted(chip_boosts.items()):
        mask = (chips == chip) & (labels == 1)
        affected = int(np.sum(mask))
        if affected == 0:
            summary[chip] = {'factor': factor, 'affected': 0}
            continue
        weights[mask] *= np.float32(factor)
        summary[chip] = {'factor': factor, 'affected': affected}

    mean_weight = float(np.mean(weights))
    if mean_weight > 1e-6:
        weights /= np.float32(mean_weight)
    return weights, summary


def _parse_iso_timestamp(value):
    """Parse ISO timestamps from dataset metadata."""
    if not value:
        return None
    try:
        return datetime.fromisoformat(str(value))
    except ValueError:
        return None


def _resolve_counterpart_name(label, entry, dataset_info=None):
    """Resolve the paired baseline/movement file from explicit metadata."""
    if label not in ('static_presence', 'motion'):
        return None

    counterpart_field = (
        'optimal_pair_motion_file'
        if label == 'static_presence'
        else 'optimal_pair_static_presence_file'
    )
    explicit = entry.get(counterpart_field)
    if explicit:
        return str(explicit)
    return None


def _build_pair_id(label, entry, dataset_info=None):
    """Build a stable pair/session id shared by baseline and movement files."""
    if label not in ('static_presence', 'motion'):
        return None

    filename = entry.get('filename')
    if not filename:
        return None

    counterpart = None
    if dataset_info is not None:
        counterpart = _resolve_counterpart_name(label, entry, dataset_info)
    if counterpart is None:
        counterpart_field = (
            'optimal_pair_motion_file'
            if label == 'static_presence'
            else 'optimal_pair_static_presence_file'
        )
        counterpart = entry.get(counterpart_field)
    if not counterpart:
        return None

    names = sorted([str(filename), str(counterpart)])
    return f"pair:{names[0]}::{names[1]}"


def _build_file_context(label, file_info, dataset_info=None):
    """Derive grouping metadata used for honest evaluation and reporting."""
    filename = str(file_info.get('filename', ''))
    chip = str(file_info.get('chip', 'unknown')).upper()
    collected_at = _parse_iso_timestamp(file_info.get('collected_at'))
    explicit_environment = _first_non_empty(
        file_info,
        (
            'environment',
            'environment_id',
            'environment_name',
        ),
    )
    explicit_session = _first_non_empty(
        file_info,
        (
            'session',
            'session_id',
            'session_name',
        ),
    )
    pair_id = _build_pair_id(label, file_info, dataset_info=dataset_info)
    day_group = collected_at.date().isoformat() if collected_at else 'unknown-day'
    session_group = explicit_session or pair_id or f"file:{filename or 'unknown'}"
    lineage_group = str(file_info.get('_lineage_group') or session_group)
    role = dataset_role(file_info)

    return {
        'chip': chip,
        'collected_at': collected_at.isoformat() if collected_at else '',
        'day_group': day_group,
        'pair_id': pair_id or '',
        # Session grouping is the primary evaluation key. Use explicit session
        # metadata when available, otherwise fall back to the paired capture or file.
        'session_group': session_group,
        # Synthetic derivatives and their real source pair share one lineage so
        # grouped CV cannot train on one representation and validate on the other.
        'lineage_group': lineage_group,
        'dataset_role': role,
        'synthetic': bool(file_info.get('synthetic', False)),
        'long_recording': bool(file_info.get('long_recording', False)),
        # Keep a dedicated environment field so future datasets can report
        # room/environment worst-groups without changing the training code again.
        'environment_group': explicit_environment or 'unknown-environment',
    }


def _fallback_file_context(filename, label, packet):
    """Create grouping metadata for files missing from dataset_info.json."""
    fallback = {
        'filename': filename,
        'chip': packet.get('chip', 'unknown'),
        'collected_at': packet.get('collected_at', ''),
        'dataset_role': dataset_role(packet),
        'synthetic': packet.get('synthetic', False),
        'long_recording': packet.get('long_recording', False),
    }
    return _build_file_context(label, fallback)


def is_motion_label(label_name, dataset_info):
    """
    Determine if a label represents motion or idle.

    Uses dataset_info.json labels when available (name-based schema).

    Args:
        label_name: Label name from npz file
        dataset_info: Loaded dataset_info.json

    Returns:
        bool: True if motion, False if idle
    """
    labels = dataset_info.get('labels', {})
    if label_name in labels:
        return label_name == 'motion'
    # Default: only 'motion' is motion
    return label_name == 'motion'


def _npz_provenance_fields(label, file_info):
    """Read scalar lineage metadata missing from older dataset-info entries."""
    fields = {}
    filename = str(file_info.get('filename', ''))
    if not filename:
        return fields
    path = DATA_DIR / str(label) / filename
    if not path.exists():
        return fields
    try:
        with np.load(path, allow_pickle=False) as data:
            for key in ('source_dataset', 'generation_group', 'generation_mode', 'synthetic'):
                if key not in data.files:
                    continue
                value = np.asarray(data[key])
                if value.ndim == 0:
                    fields[key] = value.item()
    except (OSError, ValueError):
        return fields
    return fields


def get_file_metadata(dataset_info):
    """
    Get metadata for all files in dataset_info.json.

    Returns a dict mapping filename to metadata including normalization flags and
    grouping context used by training/evaluation.

    Args:
        dataset_info: Loaded dataset_info.json

    Returns:
        dict: {filename: {...}}
    """
    file_metadata = {}
    files_by_label = dataset_info.get('files', {})
    entries_by_filename = {}
    for label, file_list in files_by_label.items():
        for file_info in file_list:
            filename = file_info.get('filename', '')
            if filename:
                enriched = dict(file_info)
                for key, value in _npz_provenance_fields(label, enriched).items():
                    enriched.setdefault(key, value)
                entries_by_filename[str(filename)] = (str(label), enriched)

    for filename, (label, file_info) in entries_by_filename.items():
        base_context = _build_file_context(label, file_info, dataset_info=dataset_info)
        lineage_group = base_context['session_group']
        source_dataset = str(file_info.get('source_dataset', '')).strip()
        if source_dataset and source_dataset in entries_by_filename:
            source_label, source_info = entries_by_filename[source_dataset]
            source_context = _build_file_context(
                source_label,
                source_info,
                dataset_info=dataset_info,
            )
            lineage_group = source_context['session_group']
        elif file_info.get('synthetic') and file_info.get('generation_group'):
            lineage_group = f"synthetic:{file_info['generation_group']}"
        enriched = dict(file_info)
        enriched['_lineage_group'] = lineage_group
        file_metadata[filename] = _build_file_context(
            label,
            enriched,
            dataset_info=dataset_info,
        )
    return file_metadata


@lru_cache(maxsize=1)
def _training_source_metadata_parameters():
    """Return the cache identity for lightweight training-source admission."""
    return {
        'contract': 'ml_training_source_metadata_v1',
        'implementation_sha256': _implementation_source_digest(
            _build_training_source_metadata,
        ),
        'sources': {
            'csi_io': npz_cache.source_manifest(tools_lib_dir() / 'csi_io.py'),
            'dataset_metadata': npz_cache.source_manifest(
                tools_lib_dir() / 'dataset_metadata.py'
            ),
            'timing_quality': npz_cache.source_manifest(
                tools_lib_dir() / 'timing_quality.py'
            ),
            'python_config': npz_cache.source_manifest(
                python_src_dir() / 'config.py'
            ),
            'python_runtime_policy': npz_cache.source_manifest(
                tools_lib_dir() / 'runtime_policy.py'
            ),
        },
    }


def _build_training_source_metadata(npz_file):
    """Materialize only the admission metadata reused across training runs."""
    packets = load_npz_packet_view(npz_file)
    if not packets:
        raise RuntimeError(
            f"{Path(npz_file).name} has no HT20/HT-LTF/64-SC sensing packets "
            "after format filtering"
        )
    first_packet = packets[0]
    return {
        'label': str(first_packet.get('label', Path(npz_file).parent.name)),
        'chip': str(first_packet.get('chip', 'unknown')).upper(),
        'packet_count': len(packets),
        'has_sync_metadata': any(
            packet.get('wifi_rx_start_ts_ns') is not None
            or packet.get('device_ticks_us') is not None
            or packet.get('wifi_rx_ts_us') is not None
            for packet in packets
        ),
        'timing_summary': summarize_capture_timing(packets),
        'fallback_context': {
            'chip': first_packet.get('chip', 'unknown'),
            'collected_at': first_packet.get('collected_at', ''),
            'dataset_role': dataset_role(first_packet),
            'synthetic': bool(first_packet.get('synthetic', False)),
            'long_recording': bool(first_packet.get('long_recording', False)),
        },
    }


def _load_or_compute_training_source_metadata(npz_file):
    """Load cached source admission metadata without retaining packet rows."""
    parameters = _training_source_metadata_parameters()
    cached = npz_cache.load_ml_training_source_metadata_artifact(
        npz_file,
        parameters=parameters,
    )
    if cached is not None:
        return cached
    with npz_cache.artifact_build_lock(
        npz_file,
        artifact_name='ml_training_source_metadata',
        artifact_version=npz_cache.ML_TRAINING_SOURCE_METADATA_ARTIFACT_VERSION,
        parameters=parameters,
    ):
        cached = npz_cache.load_ml_training_source_metadata_artifact(
            npz_file,
            parameters=parameters,
        )
        if cached is not None:
            return cached
        metadata = _build_training_source_metadata(npz_file)
        npz_cache.save_ml_training_source_metadata_artifact(
            npz_file,
            parameters=parameters,
            metadata=metadata,
        )
        persisted = npz_cache.load_ml_training_source_metadata_artifact(
            npz_file,
            parameters=parameters,
        )
        return persisted if persisted is not None else metadata


def _load_training_file_records(environment_filter=None, excluded_chips=None,
                                allowed_labels=BINARY_TRAINING_LABELS,
                                require_sync_metadata=False,
                                dataset_roles=DEFAULT_TRAINING_ROLES,
                                timing_quality_policy=DEFAULT_TIMING_QUALITY_POLICY,
                                timing_warn_weight=DEFAULT_TIMING_WARN_WEIGHT):
    """Return filtered per-file training records plus aggregate stats."""
    environment_filter = parse_environment_filter(environment_filter)
    excluded_chips = parse_chip_filter(excluded_chips)
    allowed_labels = normalize_allowed_labels(allowed_labels)
    dataset_roles = normalize_dataset_roles(dataset_roles)
    timing_quality_policy = parse_timing_quality_policy(timing_quality_policy)
    timing_warn_weight = float(timing_warn_weight)
    stats = {
        'chips': set(),
        'labels': {},
        'total': 0,
        'files': [],
        'excluded_labels': set(),
        'excluded_chips': set(),
        'excluded_environments': set(),
        'excluded_missing_sync_metadata': set(),
        'excluded_dataset_roles': set(),
        'excluded_long_recordings': set(),
        'excluded_timing_quality': set(),
        'session_groups': set(),
        'lineage_groups': set(),
        'environment_groups': set(),
        'sync_metadata_files': set(),
        'timing_quality_counts': {
            'clean': 0,
            'degraded': 0,
            'poor': 0,
            'unknown': 0,
        },
    }
    records = []

    # Load dataset info for label mapping and file metadata
    dataset_info = load_dataset_info()
    file_metadata = get_file_metadata(dataset_info)
    # Scan raw dataset subdirectories only. Generated artifacts such as
    # data/auto_generated/*.npz are not packet captures and should not be
    # parsed as CSI inputs.
    excluded_dirs = {'.', GENERATED_DATA_DIR.name}
    for subdir in sorted(DATA_DIR.iterdir()):
        if not subdir.is_dir() or subdir.name in excluded_dirs:
            continue

        # Load all npz files in this directory
        for npz_file in sorted(subdir.glob('*.npz')):
            try:
                source_metadata = _load_or_compute_training_source_metadata(
                    npz_file
                )

                # Get label from the shared packet view metadata.
                label = source_metadata.get('label', subdir.name)

                label_lc = str(label).lower()
                if allowed_labels is not None and label_lc not in allowed_labels:
                    stats['excluded_labels'].add(label_lc)
                    continue

                # Get chip
                chip = str(source_metadata.get('chip', 'unknown')).upper()
                if excluded_chips is not None and chip in excluded_chips:
                    stats['excluded_chips'].add(chip)
                    continue

                # Get file-specific metadata
                meta = file_metadata.get(npz_file.name)
                if meta is None:
                    meta = _fallback_file_context(
                        npz_file.name,
                        label_lc,
                        source_metadata.get('fallback_context', {}),
                    )

                if label_lc == 'empty' and bool(meta.get('long_recording', False)):
                    stats['excluded_long_recordings'].add(npz_file.name)
                    continue

                role = dataset_role(meta)
                if role not in dataset_roles:
                    stats['excluded_dataset_roles'].add(role)
                    continue

                environment_group = meta.get('environment_group', 'unknown-environment')
                if environment_filter is not None and environment_group not in environment_filter:
                    stats['excluded_environments'].add(environment_group)
                    continue

                has_sync_metadata = bool(
                    source_metadata.get('has_sync_metadata', False)
                )
                if require_sync_metadata and not has_sync_metadata:
                    stats['excluded_missing_sync_metadata'].add(npz_file.name)
                    continue
                if has_sync_metadata:
                    stats['sync_metadata_files'].add(npz_file.name)

                timing_summary = source_metadata['timing_summary']
                timing_status = str(timing_summary['quality_status'])
                timing_bucket = str(timing_summary['quality_bucket'])
                if timing_policy_excludes_status(timing_status, timing_quality_policy):
                    stats['excluded_timing_quality'].add(npz_file.name)
                    continue
                stats['timing_quality_counts'][timing_bucket] += 1

                # Track stats after all active filters
                if label not in stats['labels']:
                    stats['labels'][label] = 0
                packet_count = int(source_metadata['packet_count'])
                stats['labels'][label] += packet_count
                stats['total'] += packet_count
                stats['chips'].add(chip)

                stats['session_groups'].add(meta.get('session_group', f"file:{npz_file.name}"))
                stats['lineage_groups'].add(meta.get('lineage_group', f"file:{npz_file.name}"))
                if environment_group != 'unknown-environment':
                    stats['environment_groups'].add(environment_group)

                is_motion = is_motion_label(label, dataset_info)
                stats['files'].append(npz_file.name)
                records.append({
                    'path': npz_file,
                    'packets_loader': (
                        lambda current_path=npz_file: load_npz_packet_view(
                            current_path
                        )
                    ),
                    'label_name': label_lc,
                    'is_motion': is_motion,
                    'chip': meta.get('chip', chip),
                    'collected_at': meta.get('collected_at', ''),
                    'day_group': meta.get('day_group', 'unknown-day'),
                    'pair_id': meta.get('pair_id', ''),
                    'session_group': meta.get('session_group', f"file:{npz_file.name}"),
                    'lineage_group': meta.get('lineage_group', meta.get('session_group', f"file:{npz_file.name}")),
                    'dataset_role': role,
                    'synthetic': bool(meta.get('synthetic', False)),
                    'long_recording': bool(meta.get('long_recording', False)),
                    'environment_group': environment_group,
                    'timing_quality_status': timing_status,
                    'timing_quality_bucket': timing_bucket,
                    'timing_summary': timing_summary,
                    'timing_weight': timing_policy_weight(
                        timing_status,
                        timing_quality_policy,
                        warn_weight=timing_warn_weight,
                    ),
                })

            except RuntimeError:
                # Sensing-contract violations must stop training explicitly;
                # a silently skipped file would hide a contaminated dataset.
                raise
            except Exception as e:
                print(f"  Warning: Could not load {npz_file.name}: {e}")

    stats['chips'] = sorted(stats['chips'])
    stats['excluded_labels'] = sorted(stats['excluded_labels'])
    stats['excluded_chips'] = sorted(stats['excluded_chips'])
    stats['excluded_environments'] = sorted(stats['excluded_environments'])
    stats['excluded_missing_sync_metadata'] = sorted(stats['excluded_missing_sync_metadata'])
    stats['excluded_dataset_roles'] = sorted(stats['excluded_dataset_roles'])
    stats['excluded_long_recordings'] = sorted(stats['excluded_long_recordings'])
    stats['excluded_timing_quality'] = sorted(stats['excluded_timing_quality'])
    stats['session_groups'] = sorted(stats['session_groups'])
    stats['lineage_groups'] = sorted(stats['lineage_groups'])
    stats['environment_groups'] = sorted(stats['environment_groups'])
    stats['sync_metadata_files'] = sorted(stats['sync_metadata_files'])
    return records, stats


def load_all_data(environment_filter=None, excluded_chips=None,
                  allowed_labels=BINARY_TRAINING_LABELS,
                  require_sync_metadata=False,
                  dataset_roles=DEFAULT_TRAINING_ROLES,
                  timing_quality_policy=DEFAULT_TIMING_QUALITY_POLICY,
                  timing_warn_weight=DEFAULT_TIMING_WARN_WEIGHT):
    """
    Load all available CSI data from the data/ directory.

    Reads label from npz file metadata (not folder structure).
    Uses dataset_info.json only to determine if label is motion or idle.
    Uses the normalized turbulence pipeline and attaches grouping metadata
    (file, session/pair, environment when available).

    Args:
        environment_filter: Optional set/string of environment names to keep.
        excluded_chips: Optional set/string of chip names to exclude.
        allowed_labels: Iterable of lowercase labels to include.

    Returns:
        tuple: (all_packets, stats) where stats is a dict with dataset info
    """
    records, stats = _load_training_file_records(
        environment_filter=environment_filter,
        excluded_chips=excluded_chips,
        allowed_labels=allowed_labels,
        require_sync_metadata=require_sync_metadata,
        dataset_roles=dataset_roles,
        timing_quality_policy=timing_quality_policy,
        timing_warn_weight=timing_warn_weight,
    )
    all_packets = []
    for record in records:
        for idx, packet in enumerate(_ensure_record_packets(record)):
            enriched = dict(packet)
            enriched['is_motion'] = record['is_motion']
            enriched['label_name'] = record['label_name']
            enriched['source_file'] = record['path'].name
            enriched['packet_index'] = idx
            enriched['chip'] = record['chip']
            enriched['collected_at'] = record['collected_at']
            enriched['day_group'] = record['day_group']
            enriched['pair_id'] = record['pair_id']
            enriched['session_group'] = record['session_group']
            enriched['lineage_group'] = record['lineage_group']
            enriched['dataset_role'] = record['dataset_role']
            enriched['synthetic'] = record['synthetic']
            enriched['long_recording'] = record['long_recording']
            enriched['environment_group'] = record['environment_group']
            enriched['timing_quality_status'] = record['timing_quality_status']
            enriched['timing_quality_bucket'] = record['timing_quality_bucket']
            all_packets.append(enriched)
    return all_packets, stats


CONTEXT_INT_KEYS = ('packet_index', 'window_index', 'reset_index')


CONTEXT_BOOL_KEYS = ('synthetic',)


def load_training_matrix(environment_filter=None, excluded_chips=None,
                         feature_names=None, use_cache=True,
                         packet_augmentation=None, augmentation_seed=None,
                         augmentation_seeds=None,
                         dataset_roles=DEFAULT_TRAINING_ROLES,
                         timing_quality_policy=DEFAULT_TIMING_QUALITY_POLICY,
                         timing_warn_weight=DEFAULT_TIMING_WARN_WEIGHT):
    """Load the canonical reset-aware streaming feature matrix used by training."""
    if augmentation_seed is not None and augmentation_seeds is not None:
        raise ValueError("pass augmentation_seed or augmentation_seeds, not both")
    resolved_augmentation_seeds = (
        tuple(int(seed) for seed in augmentation_seeds)
        if augmentation_seeds is not None
        else tuple()
    )
    if len(set(resolved_augmentation_seeds)) != len(resolved_augmentation_seeds):
        raise ValueError("augmentation_seeds must not contain duplicates")
    if (
        packet_augmentation
        and augmentation_seed is None
        and not resolved_augmentation_seeds
    ):
        raise ValueError("packet augmentation requires a deterministic seed")
    if feature_names is None:
        feature_names = DEFAULT_FEATURES.copy()
    feature_names = list(feature_names)
    use_runtime_cache = _feature_rows_use_runtime_cache(feature_names)
    if use_cache and use_runtime_cache:
        print("  Training feature cache: enabled (canonical time-aware replay rows)")
    elif use_cache:
        print("  Training feature cache: enabled (host-side replay rows)")
    else:
        reason = []
        if not _feature_names_support_replay_rows(feature_names):
            reason.append("host-only feature extraction")
        detail = f" ({', '.join(reason)})" if reason else ""
        print(f"  Training feature cache: disabled{detail}")

    load_start = perf_counter()
    records, stats = _load_training_file_records(
        environment_filter=environment_filter,
        excluded_chips=excluded_chips,
        dataset_roles=dataset_roles,
        timing_quality_policy=timing_quality_policy,
        timing_warn_weight=timing_warn_weight,
    )
    print(f"  Load time: {format_duration(perf_counter() - load_start)}")

    if not stats['chips']:
        return {
            'X': np.empty((0, len(feature_names)), dtype=np.float32),
            'y': np.asarray([], dtype=np.int8),
            'feature_names': feature_names,
            'sample_context': {},
            'sample_weights': np.asarray([], dtype=np.float32),
            'stats': stats,
        }, None

    print("\nExtracting features...")
    features_start = perf_counter()
    X_parts = []
    y_parts = []
    weight_parts = []
    context_parts = {
        'chip': [],
        'source_file': [],
        'lineage_group': [],
        'session_group': [],
        'environment_group': [],
        'pair_id': [],
        'day_group': [],
        'dataset_role': [],
        'timing_quality_status': [],
        'timing_quality_bucket': [],
        'synthetic': [],
        'label_name': [],
        'packet_index': [],
        'window_index': [],
        'reset_index': [],
    }
    cache_hits = 0
    cache_misses = 0
    actual_feature_names = feature_names

    for record in records:
        if packet_augmentation and len(resolved_augmentation_seeds) > 1:
            replay_rows = _load_or_compute_packet_augmentation_mix_rows(
                record,
                packet_augmentation=packet_augmentation,
                augmentation_seeds=resolved_augmentation_seeds,
                feature_names=feature_names,
                use_cache=use_cache,
                use_runtime_cache=use_runtime_cache,
            )
            cache_hit = bool(replay_rows.get('cache_hit', False))
        elif use_runtime_cache and packet_augmentation:
            resolved_seed = (
                resolved_augmentation_seeds[0]
                if resolved_augmentation_seeds
                else augmentation_seed
            )
            stream_provenance = _packet_augmentation_stream_provenance(
                packet_augmentation,
                resolved_seed,
            )
            replay_rows = load_or_compute_ml_replay_rows(
                record['path'],
                packets_factory=lambda current_record=record, current_seed=resolved_seed: (
                    _prepare_feature_packets_for_record(
                        current_record,
                        packet_augmentation=packet_augmentation,
                        augmentation_seed=current_seed,
                    )
                ),
                selected_subcarriers=DEFAULT_SUBCARRIERS,
                window_size=None,
                feature_names=feature_names,
                sample_contract=TRAINING_SAMPLE_CONTRACT,
                use_cache=use_cache and stream_provenance is not None,
                stream_provenance=stream_provenance,
            )
            cache_hit = bool(replay_rows.get('cache_hit', False))
        elif use_runtime_cache:
            replay_rows = load_or_compute_ml_replay_rows(
                record['path'],
                selected_subcarriers=DEFAULT_SUBCARRIERS,
                window_size=None,
                feature_names=feature_names,
                use_cache=use_cache,
                sample_contract=TRAINING_SAMPLE_CONTRACT,
            )
            cache_hit = bool(replay_rows.get('cache_hit', False))
        else:
            resolved_seed = (
                resolved_augmentation_seeds[0]
                if resolved_augmentation_seeds
                else augmentation_seed
            )
            stream_provenance = _host_feature_stream_provenance(
                feature_names,
                packet_augmentation=packet_augmentation,
                augmentation_seed=resolved_seed,
            )
            replay_rows = load_or_compute_host_feature_rows(
                record['path'],
                packets_factory=lambda current_record=record, current_seed=resolved_seed: (
                    _prepare_feature_packets_for_record(
                        current_record,
                        packet_augmentation=packet_augmentation,
                        augmentation_seed=current_seed,
                    )
                ),
                feature_names=feature_names,
                sample_contract=TRAINING_SAMPLE_CONTRACT,
                use_cache=use_cache,
                stream_provenance=stream_provenance,
            )
            cache_hit = bool(replay_rows.get('cache_hit', False))
        file_matrix = {
            'X': np.asarray(replay_rows['X'], dtype=np.float32),
            'feature_names': list(replay_rows['feature_names']),
        }
        if cache_hit:
            cache_hits += 1
        else:
            cache_misses += 1
        X_file = np.asarray(file_matrix['X'], dtype=np.float32)
        actual_feature_names = list(file_matrix['feature_names'])
        if len(X_file) == 0:
            continue
        X_parts.append(X_file)
        y_parts.append(
            np.full(len(X_file), 1 if record['is_motion'] else 0, dtype=np.int8)
        )
        weight_parts.append(
            np.full(
                len(X_file),
                float(record.get('timing_weight', 1.0)),
                dtype=np.float32,
            )
        )
        sample_context = _build_sample_context_for_replay_rows(record, replay_rows)
        for key, values in sample_context.items():
            context_parts[key].append(values)

    print(
        f"  Feature cache files: {cache_hits} hit(s), {cache_misses} miss(es)"
    )
    print(f"  Feature extraction time: {format_duration(perf_counter() - features_start)}")

    if X_parts:
        X = np.concatenate(X_parts, axis=0)
        y = np.concatenate(y_parts, axis=0)
        sample_context = {
            key: np.concatenate(parts, axis=0)
            for key, parts in context_parts.items()
            if parts
        }
        sample_weights = np.concatenate(weight_parts, axis=0).astype(np.float32, copy=False)
        mean_weight = float(np.mean(sample_weights))
        if mean_weight > 1e-6:
            sample_weights /= np.float32(mean_weight)
    else:
        X = np.empty((0, len(actual_feature_names)), dtype=np.float32)
        y = np.asarray([], dtype=np.int8)
        sample_context = {
            'chip': np.empty(0, dtype=object),
            'source_file': np.empty(0, dtype=object),
            'lineage_group': np.empty(0, dtype=object),
            'session_group': np.empty(0, dtype=object),
            'environment_group': np.empty(0, dtype=object),
            'pair_id': np.empty(0, dtype=object),
            'day_group': np.empty(0, dtype=object),
            'dataset_role': np.empty(0, dtype=object),
            'timing_quality_status': np.empty(0, dtype=object),
            'timing_quality_bucket': np.empty(0, dtype=object),
            'synthetic': np.empty(0, dtype=bool),
            'label_name': np.empty(0, dtype=object),
            'packet_index': np.empty(0, dtype=np.int32),
            'window_index': np.empty(0, dtype=np.int32),
            'reset_index': np.empty(0, dtype=np.int32),
        }
        sample_weights = np.asarray([], dtype=np.float32)

    matrix = {
        'X': X,
        'y': y,
        'feature_names': actual_feature_names,
        'sample_context': sample_context,
        'sample_weights': sample_weights,
        'stats': stats,
    }
    return matrix, None
