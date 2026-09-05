# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Performance Report Helpers

Shared performance-report helpers for tests and tooling.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

import time
from collections.abc import Mapping as MappingABC
from collections import defaultdict
from functools import lru_cache
from contextlib import nullcontext
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Mapping, Optional, Sequence

import numpy as np

from .bootstrap import setup_paths
from .atomic_io import atomic_write_text
from .dataset_metadata import (
    admitted_dataset_role,
    build_calibrated_lightweight_detector,
    build_lightweight_detector,
    dataset_info_revision,
    derive_detector_timing,
    load_dataset_info,
    measure_packet_interval_us,
    paired_dataset_role,
    resolve_entry_path,
)
from . import dataset_metadata, npz_cache
from .repo_paths import data_dir, repo_root

setup_paths()

import config
from config import (
    CALIBRATION_DURATION_MS,
    DEFAULT_SUBCARRIERS,
    EVALUATION_INTERVAL_MS,
    MOTION_OFF_HITS,
    MOTION_ON_HITS,
    SEGMENTATION_WINDOW_SIZE_MS,
)

from detector_interface import MotionState
from tools.lib.csi_features import FEATURE_NAMES as RUNTIME_FEATURE_NAMES
from tools.lib.runtime_policy import (
    PacketTimingTracker,
    RuntimeMotionPolicy,
    make_evaluation_cadence as _make_evaluation_cadence,
    nominal_packet_interval_us,
)
from tools.lib.temporal_csi_sampler import minimum_valid_slots, temporal_window_slots
from tools.lib.csi_io import load_npz_arrays, load_npz_packet_view, load_npz_sensing_arrays
from tools.lib.temporal_replay import (
    apply_temporal_admission,
    iter_temporal_admissions,
    packet_timestamp_us,
    target_pps_for_packets,
)

DATA_DIR = data_dir()
PERFORMANCE_DOC_PATH = repo_root() / "docs" / "performance" / "README.md"
PERFORMANCE_REPLAY_IMPLEMENTATION_VERSION = 5
REPORT_DATASET_ROLES = frozenset(("selection", "holdout"))
DIAGNOSTIC_ALL_PHY = False
# Published parity metrics use the C++ production default. Micro-ESPectre may
# disable Hampel preprocessing to preserve heap and CPU headroom.
REPORT_HAMPEL_ENABLED = True

# Link-class policy: real weak-link (`low_rssi: true`) recordings are stress
# diagnostics, not standard promotion material. Normal-link sessions keep the
# strict production targets; stress replays use these relaxed ML targets and
# stay report-only for the Lightweight detector.
STRESS_TARGET_RECALL = 90.0
STRESS_TARGET_FP_RATE = 10.0


def timing_cadence_for_window(
    window_packets: Optional[int] = None,
    interval_us: Optional[int] = None,
) -> tuple[PacketTimingTracker, RuntimeMotionPolicy]:
    """Return the shared packet-timing helpers used by replay workflows.

    ``interval_us`` is the measured cadence when the caller knows it. Without
    it the configured nominal interval is used as legacy replay fallback.
    """
    nominal_interval_us = (
        nominal_packet_interval_us(config.CSI_TARGET_PPS)
        if interval_us is None
        else max(1, int(interval_us))
    )
    if window_packets is None:
        window_packets = derive_detector_timing(
            nominal_interval_us,
            SEGMENTATION_WINDOW_SIZE_MS,
        )["window_packets"]
    cadence = _make_evaluation_cadence(
        evaluation_interval_ms=config.EVALUATION_INTERVAL_MS,
        segmentation_window_size_ms=SEGMENTATION_WINDOW_SIZE_MS,
    )
    return PacketTimingTracker(nominal_interval_us), cadence


def note_evaluation_tick(
    cadence: RuntimeMotionPolicy,
    *,
    packet: Any = None,
    timing_tracker: Optional[PacketTimingTracker] = None,
) -> tuple[bool, bool]:
    """Record one packet and return True when a deploy-time evaluation is due."""
    timing = (
        timing_tracker.observe_packet(packet)
        if timing_tracker is not None
        else {"coverage_us": 0, "contaminated": False}
    )
    cadence.note_packet(elapsed_us=timing["coverage_us"])
    should_evaluate = cadence.should_evaluate()
    if should_evaluate:
        cadence.after_evaluation()
    return should_evaluate, bool(timing["contaminated"])


def temporal_detector_ticks(
    detector: Any,
    packets: Sequence[Any],
    interval_us: int,
):
    """Yield admitted replay packets after applying production slot semantics."""
    target_pps = target_pps_for_packets(packets, interval_us)
    _, cadence = timing_cadence_for_window(interval_us=interval_us)
    slots_since_reset = 0
    for admission in iter_temporal_admissions(
        packets,
        target_pps=target_pps,
        window_size_ms=SEGMENTATION_WINDOW_SIZE_MS,
        fallback_interval_us=interval_us,
    ):
        if admission.reset_required:
            cadence.reset()
            slots_since_reset = 0
        apply_temporal_admission(detector, admission)
        cadence.note_packet(elapsed_us=admission.coverage_us)
        should_evaluate = cadence.should_evaluate()
        if should_evaluate:
            cadence.after_evaluation()
        slots_since_reset = admission.slot_index + 1
        yield admission, should_evaluate, slots_since_reset


def _is_scored_replay_evaluation(detector: Any, packets_since_reset: int, warmup: int) -> bool:
    """Return True when a cadence tick is a scored detector evaluation.

    Occupancy holes leave the detector not ready. Firmware still runs
    ``update_state()``, which reports IDLE, but C++/Python report metrics skip
    those ticks so occupancy loss is not counted as a false negative or a
    true negative.
    """
    if int(packets_since_reset) < int(warmup):
        return False
    is_ready = getattr(detector, "is_ready", None)
    return not callable(is_ready) or bool(is_ready())


def _evaluate_idle_runtime_policy_evaluations(raw_motion_states: Sequence[bool]) -> Dict[str, int]:
    """Apply production hit filtering to states sampled at evaluation ticks."""
    policy = RuntimeMotionPolicy(
        evaluation_interval_ms=EVALUATION_INTERVAL_MS,
        motion_on_hits=MOTION_ON_HITS,
        motion_off_hits=MOTION_OFF_HITS,
    )
    effective_alarms = 0
    false_motion_evaluations = 0

    for raw_motion in raw_motion_states:
        raw_state = MotionState.MOTION if raw_motion else MotionState.IDLE
        effective_state, changed = policy.apply_state(raw_state)
        if changed and effective_state == MotionState.MOTION:
            effective_alarms += 1
        if effective_state == MotionState.MOTION:
            false_motion_evaluations += 1

    return {
        "effective_alarms": effective_alarms,
        "false_motion_evaluations": false_motion_evaluations,
    }


CHIP_ORDER = ("C3", "C5", "C6", "ESP32", "S3")
PAIRED_CHIP_LABELS = {
    "C3": "ESP32-C3",
    "C5": "ESP32-C5",
    "C6": "ESP32-C6",
    "ESP32": "ESP32",
    "S3": "ESP32-S3",
}

ProgressCallback = Callable[[str], None]
ExecutionInfo = Dict[str, Any]


def configure_dataset_root(
    data_root: str | Path,
    *,
    diagnostic_all_phy: bool = False,
) -> None:
    """Point report discovery and replay at one ESPectre dataset root."""
    global DATA_DIR, DIAGNOSTIC_ALL_PHY

    DATA_DIR = Path(data_root).resolve()
    DIAGNOSTIC_ALL_PHY = bool(diagnostic_all_phy)
    dataset_metadata.DATA_DIR = DATA_DIR
    dataset_metadata.DATASET_INFO_FILE = DATA_DIR / "dataset_info.json"
    _get_available_paired_dataset_specs_cached.cache_clear()
    _get_available_empty_datasets_cached.cache_clear()
    _get_available_long_test_dataset_specs_cached.cache_clear()
    _load_long_test_packets_cached.cache_clear()
    compute_classic_dataset_result.cache_clear()
    compute_classic_empty_fp_result.cache_clear()


def report_evaluation_view() -> str:
    """Return the stable label for the packet view used by detector replay."""
    return (
        "all explicit PHY rows (diagnostic)"
        if DIAGNOSTIC_ALL_PHY
        else "HT20/HT-LTF"
    )


def _load_report_packet_view(path: str | Path) -> tuple[dict[str, Any], ...]:
    """Load packets under the report's selected PHY policy."""
    return load_npz_packet_view(Path(path), keep_all_phy=DIAGNOSTIC_ALL_PHY)


def _report_replay_provenance(
    provenance: Optional[Mapping[str, Any]],
) -> Optional[dict[str, Any]]:
    """Separate diagnostic all-PHY rows from production sensing-row caches."""
    if not DIAGNOSTIC_ALL_PHY:
        return None if provenance is None else dict(provenance)
    resolved = dict(provenance or {})
    resolved["packet_view"] = "all_explicit_phy"
    return resolved


class _CsiRowView(Sequence[Any]):
    """Zero-copy unsigned-byte rows over one contiguous CSI matrix."""

    def __init__(self, matrix: np.ndarray, start: int = 0, stop: Optional[int] = None):
        if matrix.ndim != 2 or matrix.dtype != np.int8 or not matrix.flags.c_contiguous:
            raise ValueError("CSI matrix must be a contiguous two-dimensional int8 array")
        matrix_stop = len(matrix) if stop is None else int(stop)
        if start < 0 or matrix_stop < start or matrix_stop > len(matrix):
            raise ValueError("Invalid CSI row view bounds")
        self._matrix = matrix
        self._bytes = memoryview(matrix).cast("B")
        self._row_size = matrix.shape[1]
        self._start = int(start)
        self._stop = matrix_stop

    def __len__(self) -> int:
        return self._stop - self._start

    def __getitem__(self, index: int | slice) -> Any:
        if isinstance(index, slice):
            start, stop, step = index.indices(len(self))
            if step != 1:
                return [self[i] for i in range(start, stop, step)]
            return _CsiRowView(
                self._matrix,
                self._start + start,
                self._start + stop,
            )

        row_index = int(index)
        if row_index < 0:
            row_index += len(self)
        if row_index < 0 or row_index >= len(self):
            raise IndexError("CSI row index out of range")
        byte_start = (self._start + row_index) * self._row_size
        return self._bytes[byte_start:byte_start + self._row_size]


def _coerce_long_test_rssi_series(
    rssi_dbm: Any,
    expected_length: int,
) -> Optional[np.ndarray]:
    """Normalize optional long-recording RSSI metadata to one aligned array."""
    if rssi_dbm is None:
        return None
    series = np.asarray(rssi_dbm)
    if series.ndim == 0:
        return np.full(int(expected_length), int(series.item()), dtype=np.int16)
    if len(series) != int(expected_length):
        return None
    return series


def _coerce_long_test_int_series(
    values: Any,
    expected_length: int,
    *,
    dtype: Any,
) -> Optional[np.ndarray]:
    """Normalize optional long-recording integer metadata to one aligned array."""
    if values is None:
        return None
    series = np.asarray(values, dtype=dtype)
    if series.ndim == 0:
        return np.full(int(expected_length), int(series.item()), dtype=dtype)
    if len(series) != int(expected_length):
        return None
    return series


class _LongTestPacketView(Sequence[Any]):
    """Zero-copy packet dictionaries over CSI rows plus optional RSSI metadata."""

    def __init__(
        self,
        csi_rows: _CsiRowView,
        rssi_dbm: Optional[np.ndarray] = None,
        seq_num: Optional[np.ndarray] = None,
        device_ticks_us: Optional[np.ndarray] = None,
        wifi_rx_ts_us: Optional[np.ndarray] = None,
    ):
        self._csi_rows = csi_rows
        self._rssi_dbm = rssi_dbm
        self._seq_num = seq_num
        self._device_ticks_us = device_ticks_us
        self._wifi_rx_ts_us = wifi_rx_ts_us

    def __len__(self) -> int:
        return len(self._csi_rows)

    def __getitem__(self, index: int | slice) -> Any:
        if isinstance(index, slice):
            start, stop, step = index.indices(len(self))
            if step != 1:
                return [self[i] for i in range(start, stop, step)]
            sliced_rssi = None if self._rssi_dbm is None else self._rssi_dbm[start:stop]
            sliced_seq = None if self._seq_num is None else self._seq_num[start:stop]
            sliced_ticks = (
                None if self._device_ticks_us is None else self._device_ticks_us[start:stop]
            )
            sliced_wifi = (
                None if self._wifi_rx_ts_us is None else self._wifi_rx_ts_us[start:stop]
            )
            return _LongTestPacketView(
                self._csi_rows[index],
                sliced_rssi,
                sliced_seq,
                sliced_ticks,
                sliced_wifi,
            )

        row_index = int(index)
        if row_index < 0:
            row_index += len(self)
        if row_index < 0 or row_index >= len(self):
            raise IndexError("packet index out of range")

        packet = {"csi_data": self._csi_rows[row_index]}
        if self._rssi_dbm is not None:
            packet["rssi_dbm"] = int(self._rssi_dbm[row_index])
        if self._seq_num is not None:
            packet["seq_num"] = int(self._seq_num[row_index])
        if self._device_ticks_us is not None:
            packet["device_ticks_us"] = int(self._device_ticks_us[row_index])
        if self._wifi_rx_ts_us is not None:
            packet["wifi_rx_ts_us"] = int(self._wifi_rx_ts_us[row_index])
        return packet


def _emit_progress(progress: Optional[ProgressCallback], message: str) -> None:
    if progress is not None:
        progress(message)


def _format_progress_duration(seconds: float) -> str:
    if seconds < 60.0:
        return f"{seconds:.2f}s"
    minutes, remaining_seconds = divmod(seconds, 60.0)
    if minutes < 60.0:
        return f"{int(minutes)}m {remaining_seconds:.2f}s"
    hours, remaining_minutes = divmod(minutes, 60.0)
    return f"{int(hours)}h {int(remaining_minutes)}m {remaining_seconds:.2f}s"


def _load_dataset_info() -> Dict[str, Any]:
    return load_dataset_info(DATA_DIR / "dataset_info.json")


@lru_cache(maxsize=1)
def _get_available_paired_dataset_specs_cached(
    dataset_revision: str,
) -> tuple[tuple[Path, Path, int, str, str, bool, str, bool, str], ...]:
    """Return explicit static-presence/motion pairs (HT20: 64 SC only)."""
    del dataset_revision
    dataset_info = _load_dataset_info()
    files = dataset_info.get("files", {})
    motion_by_filename = {
        entry.get("filename"): entry
        for entry in files.get("motion", [])
        if entry.get("filename")
    }

    pair_entries = []
    for static_entry in files.get("static_presence", []):
        if static_entry.get("subcarriers") != 64:
            continue
        motion_filename = static_entry.get("optimal_pair_motion_file")
        motion_entry = motion_by_filename.get(motion_filename)
        if not motion_entry or motion_entry.get("subcarriers") != 64:
            continue

        chip = static_entry.get("chip")
        static_path = resolve_entry_path("static_presence", static_entry)
        motion_path = resolve_entry_path("motion", motion_entry)
        if not chip or not static_path.exists() or not motion_path.exists():
            continue

        environment = static_entry.get("environment") or "unknown"
        dataset_id = (
            f"{str(chip).lower()}_{environment}_"
            f"{Path(str(static_entry['filename'])).stem}"
        )
        synthetic = bool(static_entry.get("synthetic"))
        if synthetic != bool(motion_entry.get("synthetic")):
            continue
        # Both sides must carry the same explicit admitted role. Missing,
        # invalid, mismatched, and excluded roles stay outside replay gates.
        dataset_role = paired_dataset_role(static_entry, motion_entry)
        if dataset_role is None:
            continue
        low_rssi = bool(static_entry.get("low_rssi")) or bool(motion_entry.get("low_rssi"))
        pair_entries.append(
            (static_path, motion_path, 64, str(chip).upper(), dataset_id, synthetic,
             dataset_role, low_rssi, str(environment))
        )

    pair_entries.sort(key=lambda item: (item[3], item[4]))
    return tuple(pair_entries)


def is_low_rssi_paired_dataset(static_presence_path: str | Path) -> bool:
    """Return True when the pair is a real weak-link (`low_rssi`) capture."""
    name = Path(static_presence_path).name
    for spec in _get_available_paired_dataset_specs_cached(dataset_info_revision()):
        if spec[0].name == name:
            return bool(spec[7])
    return False


def get_paired_dataset_role(static_presence_path: str | Path) -> Optional[str]:
    """Return the normalized dataset role for one paired replay."""
    name = Path(static_presence_path).name
    for spec in _get_available_paired_dataset_specs_cached(dataset_info_revision()):
        if spec[0].name == name:
            return str(spec[6])
    return None


def get_available_paired_datasets(
    *,
    synthetic: Optional[bool] = None,
    roles: Optional[Iterable[str]] = None,
) -> list[tuple[Path, Path, int, str, str]]:
    """Return paired datasets, optionally restricted by provenance and role."""
    requested_roles = (
        None if roles is None else {str(role).strip().lower() for role in roles}
    )
    return [
        (static_path, motion_path, num_sc, chip, dataset_id)
        for static_path, motion_path, num_sc, chip, dataset_id, is_synthetic, role, _low_rssi, _environment
        in _get_available_paired_dataset_specs_cached(dataset_info_revision())
        if synthetic is None or is_synthetic == synthetic
        if requested_roles is None or role in requested_roles
    ]


@lru_cache(maxsize=1)
def _get_available_empty_datasets_cached(dataset_revision: str) -> tuple[Path, ...]:
    del dataset_revision
    dataset_info = _load_dataset_info()
    empty_paths = []
    for entry in dataset_info.get("files", {}).get("empty", []):
        if (
            admitted_dataset_role(entry) is None
            or bool(entry.get("long_recording"))
        ):
            continue
        filename = entry.get("filename")
        if not filename:
            continue
        path = resolve_entry_path("empty", entry)
        if path.exists():
            empty_paths.append(path)
    return tuple(sorted(empty_paths))


def get_available_empty_datasets() -> list[Path]:
    """Return the empty-room recordings used by the ML FP gate."""
    return list(_get_available_empty_datasets_cached(dataset_info_revision()))


def _long_recording_entry_records(dataset_info: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    """Return (label_group, entry) records for explicit long-recording replays.

    Preferred layout stores quiet long-runs under `empty` with `long_recording:
    true`. Older datasets may still keep them under `test`.
    """
    files = dataset_info.get("files", {})
    explicit = [
        ("empty", entry)
        for entry in files.get("empty", [])
        if bool(entry.get("long_recording"))
        and admitted_dataset_role(entry) is not None
    ]
    if explicit:
        return explicit
    return [
        ("test", entry)
        for entry in files.get("test", [])
        if admitted_dataset_role(entry) is not None
    ]


def get_available_chip_types() -> list[str]:
    """Return the stable set of chips covered by the paired real-data datasets."""
    chips = []
    for _static_path, _motion_path, _num_sc, chip, _dataset_id in get_available_paired_datasets():
        chips.append(chip)
    return sorted(dict.fromkeys(chips))


def load_real_data_cached(static_presence_path: str | Path, motion_path: str | Path) -> tuple[tuple[dict[str, Any], ...], tuple[dict[str, Any], ...]]:
    """Load paired packet streams through the shared in-process NPZ runtime cache."""
    return (
        _load_report_packet_view(static_presence_path),
        _load_report_packet_view(motion_path),
    )


def _resolve_ml_replay_feature_names(feature_names: Sequence[str] = ()) -> tuple[str, ...]:
    """Validate and normalize one requested ML replay feature subset."""
    requested = tuple(str(name) for name in (feature_names or tuple(RUNTIME_FEATURE_NAMES)))
    supported = set(RUNTIME_FEATURE_NAMES)
    try:
        from tools.lib.high_accuracy_detector import FEATURE_NAMES as EXPORTED_FEATURE_NAMES
        supported.update(str(name) for name in EXPORTED_FEATURE_NAMES)
    except (ImportError, AttributeError):
        pass
    missing = [name for name in requested if name not in supported]
    if missing:
        raise ValueError(
            "Time-aware replay rows only support runtime ML features: "
            + ", ".join(missing)
        )
    return requested


def _normalize_ml_sample_contract(sample_contract: str = "replay_tick") -> str:
    """Normalize one ML row-production contract."""
    contract = str(sample_contract).strip().lower()
    if contract not in {"replay_tick", "stream_dense"}:
        raise ValueError(f"Unsupported ML sample contract: {sample_contract!r}")
    return contract


def _empty_ml_replay_rows(
    feature_names: Sequence[str], target_pps: int = 0
) -> Dict[str, Any]:
    """Return one empty replay-row payload for the requested feature subset."""
    resolved = [str(name) for name in feature_names]
    return {
        "X": np.empty((0, len(resolved)), dtype=np.float32),
        "feature_names": resolved,
        "packet_index": np.empty(0, dtype=np.int32),
        "evaluation_index": np.empty(0, dtype=np.int32),
        "reset_index": np.empty(0, dtype=np.int32),
        "slot_index": np.empty(0, dtype=np.int64),
        "target_pps": int(target_pps),
        "evaluation_due": np.empty(0, dtype=bool),
    }


def _normalize_replay_row_selection(
    row_stride: Optional[int],
    row_offset: int,
) -> tuple[Optional[int], int]:
    """Validate an optional dense-row modulo selection."""
    if row_stride is None:
        if int(row_offset) != 0:
            raise ValueError("row_offset requires row_stride")
        return None, 0
    stride = int(row_stride)
    offset = int(row_offset)
    if stride < 1 or offset < 0 or offset >= stride:
        raise ValueError("row selection requires 0 <= row_offset < row_stride")
    return stride, offset


def _select_ml_replay_rows(
    rows: Mapping[str, Any],
    *,
    row_stride: Optional[int],
    row_offset: int,
) -> Dict[str, Any]:
    """Select dense replay rows by position without changing their metadata."""
    stride, offset = _normalize_replay_row_selection(row_stride, row_offset)
    if stride is None:
        return dict(rows)
    row_count = len(np.asarray(rows.get("packet_index", ())))
    mask = np.arange(row_count, dtype=np.int64) % stride == offset
    selected = {
        "X": np.asarray(rows["X"], dtype=np.float32)[mask],
        "feature_names": list(rows.get("feature_names", ())),
        "packet_index": np.asarray(rows["packet_index"], dtype=np.int32)[mask],
        "evaluation_index": np.asarray(
            rows["evaluation_index"], dtype=np.int32
        )[mask],
        "reset_index": np.asarray(rows["reset_index"], dtype=np.int32)[mask],
        "slot_index": np.asarray(rows["slot_index"], dtype=np.int64)[mask],
        "target_pps": int(rows.get("target_pps", 0)),
        "evaluation_due": np.asarray(rows["evaluation_due"], dtype=bool)[mask],
    }
    if "cache_hit" in rows:
        selected["cache_hit"] = bool(rows["cache_hit"])
    return selected


def _project_ml_replay_rows(
    rows: Mapping[str, Any],
    feature_names: Sequence[str],
    sample_contract: str,
) -> Dict[str, Any]:
    """Project the canonical dense stream onto one schema and row contract."""
    requested = [str(name) for name in feature_names]
    available = [str(name) for name in rows.get("feature_names", ())]
    indices = [available.index(name) for name in requested]
    evaluation_due = np.asarray(rows.get("evaluation_due", ()), dtype=bool)
    row_count = len(np.asarray(rows.get("packet_index", ())))
    if len(evaluation_due) != row_count:
        raise ValueError("Cached ML replay rows do not contain cadence metadata")
    normalized_contract = _normalize_ml_sample_contract(sample_contract)
    row_mask = (
        evaluation_due
        if normalized_contract == "replay_tick"
        else np.ones(row_count, dtype=bool)
    )
    projected = {
        "X": np.asarray(rows["X"], dtype=np.float32)[row_mask][:, indices],
        "feature_names": requested,
        "packet_index": np.asarray(rows["packet_index"], dtype=np.int32)[row_mask],
        "evaluation_index": np.asarray(rows["evaluation_index"], dtype=np.int32)[row_mask],
        "reset_index": np.asarray(rows["reset_index"], dtype=np.int32)[row_mask],
        "slot_index": np.asarray(rows["slot_index"], dtype=np.int64)[row_mask],
        "target_pps": int(rows.get("target_pps", 0)),
        "evaluation_due": evaluation_due[row_mask],
    }
    if normalized_contract == "replay_tick":
        projected["evaluation_index"] = np.arange(
            len(projected["packet_index"]), dtype=np.int32
        )
    if "cache_hit" in rows:
        projected["cache_hit"] = bool(rows["cache_hit"])
    return projected


def build_ml_replay_rows(
    packets: Sequence[dict[str, Any]],
    selected_subcarriers: Sequence[int],
    window_size: Optional[int] = None,
    feature_names: Sequence[str] = (),
    *,
    sample_contract: str = "replay_tick",
    row_stride: Optional[int] = None,
    row_offset: int = 0,
    target_pps: Optional[int] = None,
) -> Dict[str, Any]:
    """Build reset-aware ML rows and project them onto one sampling contract."""
    from tools.lib.high_accuracy_detector import HighAccuracyDetector, FEATURE_NAMES as EXPORTED_FEATURE_NAMES

    requested_feature_names = _resolve_ml_replay_feature_names(feature_names)
    missing = [
        name for name in requested_feature_names
        if name not in EXPORTED_FEATURE_NAMES
    ]
    if missing:
        raise ValueError(
            "The currently exported High-Accuracy detector cannot produce replay rows for: "
            + ", ".join(missing)
        )
    normalized_contract = _normalize_ml_sample_contract(sample_contract)
    stride, offset = _normalize_replay_row_selection(row_stride, row_offset)
    if stride is not None and normalized_contract != "stream_dense":
        raise ValueError("dense row selection requires sample_contract='stream_dense'")
    if not packets:
        return _empty_ml_replay_rows(requested_feature_names)

    interval_us = measure_packet_interval_us(packets)
    inferred_pps = target_pps_for_packets(packets, interval_us)
    # Paired C++ replay sizes both phases from the baseline capture. A caller
    # that already resolved that window must keep this file on the same grid
    # rather than re-inferring a different rate from its own timestamps.
    if target_pps is not None:
        target_pps = max(1, int(target_pps))
    elif window_size is not None:
        target_pps = max(
            1,
            int(round(int(window_size) * 1000.0 / SEGMENTATION_WINDOW_SIZE_MS)),
        )
    else:
        target_pps = inferred_pps
    window_size = temporal_window_slots(target_pps, SEGMENTATION_WINDOW_SIZE_MS)
    nominal_interval_us = max(1, int(round(1_000_000.0 / target_pps)))
    detector = HighAccuracyDetector(window_size=window_size, threshold=0.5)
    if hasattr(detector, "set_minimum_valid_samples"):
        detector.set_minimum_valid_samples(minimum_valid_slots(window_size))
    feature_indices = [EXPORTED_FEATURE_NAMES.index(name) for name in requested_feature_names]
    _, cadence = timing_cadence_for_window(window_size, nominal_interval_us)
    packets_since_reset = 0
    reset_index = 0
    evaluation_index = 0
    row_features: list[np.ndarray] = []
    packet_index_values: list[int] = []
    evaluation_index_values: list[int] = []
    reset_index_values: list[int] = []
    slot_index_values: list[int] = []
    evaluation_due_values: list[bool] = []

    for admission in iter_temporal_admissions(
        packets,
        target_pps=target_pps,
        window_size_ms=SEGMENTATION_WINDOW_SIZE_MS,
        fallback_interval_us=nominal_interval_us,
    ):
        packet_index = admission.packet_index
        packet = admission.packet
        if admission.reset_required:
            cadence.reset()
            reset_index += 1
            packets_since_reset = 0
        apply_temporal_admission(detector, admission)
        cadence.note_packet(elapsed_us=admission.coverage_us)
        should_evaluate = cadence.should_evaluate()
        if should_evaluate:
            cadence.after_evaluation()
        detector.process_packet(
            _packet_csi_data(packet),
            selected_subcarriers,
            timestamp_us=_packet_timestamp_us(
                packet, packet_index, nominal_interval_us
            ),
        )
        packets_since_reset = admission.slot_index + 1
        if packets_since_reset < window_size or not detector.is_ready():
            continue
        dense_row_index = evaluation_index
        evaluation_index += 1
        if stride is not None and dense_row_index % stride != offset:
            continue
        values = np.asarray(detector._extract_features(), dtype=np.float32)
        row_features.append(values[feature_indices].astype(np.float32, copy=False))
        packet_index_values.append(int(packet_index))
        evaluation_index_values.append(int(dense_row_index))
        reset_index_values.append(int(reset_index))
        slot_index_values.append(int(admission.slot_index))
        evaluation_due_values.append(bool(should_evaluate))

    if not row_features:
        return _empty_ml_replay_rows(requested_feature_names, target_pps)
    dense_rows = {
        "X": np.vstack(row_features).astype(np.float32, copy=False),
        "feature_names": list(requested_feature_names),
        "packet_index": np.asarray(packet_index_values, dtype=np.int32),
        "evaluation_index": np.asarray(evaluation_index_values, dtype=np.int32),
        "reset_index": np.asarray(reset_index_values, dtype=np.int32),
        "slot_index": np.asarray(slot_index_values, dtype=np.int64),
        "target_pps": int(target_pps),
        "evaluation_due": np.asarray(evaluation_due_values, dtype=bool),
    }
    return _project_ml_replay_rows(
        dense_rows,
        requested_feature_names,
        normalized_contract,
    )


def load_or_compute_ml_replay_rows(
    source_path: str | Path,
    *,
    packets: Optional[Sequence[dict[str, Any]]] = None,
    packets_factory: Optional[Callable[[], Sequence[dict[str, Any]]]] = None,
    selected_subcarriers: Sequence[int],
    window_size: Optional[int],
    feature_names: Sequence[str] = (),
    sample_contract: str = "replay_tick",
    use_cache: bool = True,
    cache_write: bool = True,
    stream_provenance: Optional[Mapping[str, Any]] = None,
    row_stride: Optional[int] = None,
    row_offset: int = 0,
) -> Dict[str, Any]:
    """Load or build one canonical ML replay-row artifact for a source capture."""
    if packets is not None and packets_factory is not None:
        raise ValueError("pass packets or packets_factory, not both")
    requested_feature_names = _resolve_ml_replay_feature_names(feature_names)
    resolved_stream_provenance = _report_replay_provenance(stream_provenance)
    normalized_contract = _normalize_ml_sample_contract(sample_contract)
    stride, offset = _normalize_replay_row_selection(row_stride, row_offset)
    if stride is not None and normalized_contract != "stream_dense":
        raise ValueError("dense row selection requires sample_contract='stream_dense'")
    if stride is not None and use_cache and cache_write:
        raise ValueError("selected rows cannot be written under a full-row cache key")
    from tools.lib.high_accuracy_detector import FEATURE_NAMES as EXPORTED_FEATURE_NAMES
    cached_feature_names = tuple(EXPORTED_FEATURE_NAMES)
    packet_stream: Optional[Sequence[dict[str, Any]]] = None
    resolved_window_size = window_size
    if resolved_window_size is None:
        if packets is not None:
            packet_stream = packets
        elif packets_factory is not None:
            packet_stream = packets_factory()
        else:
            packet_stream = _load_report_packet_view(source_path)
        interval_us = measure_packet_interval_us(packet_stream)
        resolved_window_size = temporal_window_slots(
            target_pps_for_packets(packet_stream, interval_us),
            SEGMENTATION_WINDOW_SIZE_MS,
        )
    parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=selected_subcarriers,
        window_size=resolved_window_size,
        feature_names=cached_feature_names,
        stream_provenance=resolved_stream_provenance,
    )
    if use_cache:
        cached = npz_cache.load_ml_replay_row_artifact(
            source_path,
            parameters=parameters,
        )
        if cached is not None:
            cached["cache_hit"] = True
            return _select_ml_replay_rows(
                _project_ml_replay_rows(
                    cached,
                    requested_feature_names,
                    normalized_contract,
                ),
                row_stride=stride,
                row_offset=offset,
            )
    lock_context = (
        npz_cache.artifact_build_lock(
            source_path,
            artifact_name="ml_replay_rows",
            artifact_version=npz_cache.ML_REPLAY_ROW_ARTIFACT_VERSION,
            parameters=parameters,
        )
        if use_cache and cache_write
        else nullcontext()
    )
    with lock_context:
        if use_cache:
            cached = npz_cache.load_ml_replay_row_artifact(
                source_path,
                parameters=parameters,
            )
            if cached is not None:
                cached["cache_hit"] = True
                return _select_ml_replay_rows(
                    _project_ml_replay_rows(
                        cached,
                        requested_feature_names,
                        normalized_contract,
                    ),
                    row_stride=stride,
                    row_offset=offset,
                )
        if packet_stream is None:
            if packets is not None:
                packet_stream = packets
            elif packets_factory is not None:
                packet_stream = packets_factory()
            else:
                packet_stream = _load_report_packet_view(source_path)
        rows = build_ml_replay_rows(
            packet_stream,
            selected_subcarriers,
            resolved_window_size,
            cached_feature_names,
            sample_contract="stream_dense",
            row_stride=stride,
            row_offset=offset,
        )
        if use_cache and cache_write:
            npz_cache.save_ml_replay_row_artifact(
                source_path,
                parameters=parameters,
                X=rows["X"],
                feature_names=rows["feature_names"],
                packet_index=rows["packet_index"],
                evaluation_index=rows["evaluation_index"],
                reset_index=rows["reset_index"],
                evaluation_due=rows["evaluation_due"],
                slot_index=rows["slot_index"],
                target_pps=rows["target_pps"],
            )
    rows["cache_hit"] = False
    return _project_ml_replay_rows(
        rows,
        requested_feature_names,
        normalized_contract,
    )


def _empty_classic_replay_phase_rows() -> Dict[str, np.ndarray]:
    """Return one empty phase in the canonical Lightweight replay-row schema."""
    return {
        "X": np.empty((0, 2), dtype=np.float64),
        "ready": np.empty(0, dtype=bool),
        "eligible": np.empty(0, dtype=bool),
        "packet_index": np.empty(0, dtype=np.int32),
        "packet_weight": np.empty(0, dtype=np.int32),
        "reset_index": np.empty(0, dtype=np.int32),
    }


def _collect_classic_replay_phase_rows(
    packets: Sequence[dict[str, Any]],
    selected_subcarriers: Sequence[int],
    timing: Mapping[str, int],
    warmup_packets: int,
    detector: Any,
) -> Dict[str, np.ndarray]:
    """Collect every production evaluation tick for one Lightweight replay phase."""
    if not packets:
        return _empty_classic_replay_phase_rows()

    interval_us = int(timing["interval_us"])
    target_pps = target_pps_for_packets(packets, interval_us)
    window_packets = temporal_window_slots(target_pps, SEGMENTATION_WINDOW_SIZE_MS)
    _, cadence = timing_cadence_for_window(window_packets, interval_us)
    packets_since_reset = 0
    reset_index = 0
    features: list[tuple[float, float]] = []
    ready_values: list[bool] = []
    eligible_values: list[bool] = []
    packet_indices: list[int] = []
    packet_weights: list[int] = []
    reset_indices: list[int] = []

    for admission in iter_temporal_admissions(
        packets,
        target_pps=target_pps,
        window_size_ms=SEGMENTATION_WINDOW_SIZE_MS,
        fallback_interval_us=interval_us,
    ):
        packet_index = admission.packet_index
        packet = admission.packet
        if admission.reset_required:
            cadence.reset()
            packets_since_reset = 0
            reset_index += 1
        apply_temporal_admission(detector, admission)
        detector.process_packet(
            _packet_csi_data(packet),
            selected_subcarriers,
            rssi_dbm=packet.get("rssi_dbm"),
            timestamp_us=_packet_timestamp_us(packet, packet_index, interval_us),
        )
        packets_since_reset = admission.slot_index + 1
        cadence.note_packet(elapsed_us=admission.coverage_us)
        if not cadence.should_evaluate():
            continue
        packet_weight = cadence.equivalent_packets_since_evaluation(interval_us)
        values = detector.update_state()
        cadence.after_evaluation()
        ready = bool(detector.is_ready())
        features.append(
            (
                float(values.get("turb_autocorr", 0.0)),
                float(values.get("turb_iqr_over_mean_aggr", 0.0)),
            )
        )
        ready_values.append(ready)
        eligible_values.append(packets_since_reset >= int(warmup_packets))
        packet_indices.append(int(packet_index))
        packet_weights.append(int(packet_weight))
        reset_indices.append(int(reset_index))

    return {
        "X": np.asarray(features, dtype=np.float64).reshape(-1, 2),
        "ready": np.asarray(ready_values, dtype=bool),
        "eligible": np.asarray(eligible_values, dtype=bool),
        "packet_index": np.asarray(packet_indices, dtype=np.int32),
        "packet_weight": np.asarray(packet_weights, dtype=np.int32),
        "reset_index": np.asarray(reset_indices, dtype=np.int32),
    }


def build_classic_replay_rows(
    static_presence_packets: Sequence[dict[str, Any]],
    motion_packets: Sequence[dict[str, Any]],
    selected_subcarriers: Sequence[int],
    *,
    timing: Optional[Mapping[str, int]] = None,
    replay_interval_us: Optional[int] = None,
    warmup_packets: Optional[int] = None,
) -> Dict[str, Any]:
    """Build time-aware Lightweight feature rows for one paired replay."""
    measured_interval_us = (
        measure_packet_interval_us(static_presence_packets)
        if replay_interval_us is None
        else max(1, int(replay_interval_us))
    )
    target_pps = target_pps_for_packets(
        static_presence_packets, measured_interval_us
    )
    resolved_timing = dict(
        timing
        or derive_detector_timing(
            max(1, int(round(1_000_000.0 / target_pps)))
        )
    )
    resolved_timing["window_packets"] = temporal_window_slots(
        target_pps, SEGMENTATION_WINDOW_SIZE_MS
    )
    resolved_warmup_packets = (
        int(resolved_timing["window_packets"])
        if warmup_packets is None
        else max(0, int(warmup_packets))
    )
    calibration_detector = build_lightweight_detector(
        threshold=1.0,
        enable_hampel=REPORT_HAMPEL_ENABLED,
        timing=resolved_timing,
    )
    calibration_rows = _collect_classic_replay_phase_rows(
        static_presence_packets,
        selected_subcarriers,
        resolved_timing,
        resolved_warmup_packets,
        calibration_detector,
    )
    calibrated = build_calibrated_lightweight_detector(
        static_presence_packets,
        selected_subcarriers=selected_subcarriers,
        enable_hampel=REPORT_HAMPEL_ENABLED,
    )
    detector = (
        build_lightweight_detector(
            threshold=1.0,
            enable_hampel=REPORT_HAMPEL_ENABLED,
            timing=resolved_timing,
        )
        if calibrated is None
        else calibrated[0]
    )
    replay_timing = dict(resolved_timing)
    replay_timing["interval_us"] = measured_interval_us
    static_rows = _collect_classic_replay_phase_rows(
        static_presence_packets,
        selected_subcarriers,
        replay_timing,
        resolved_warmup_packets,
        detector,
    )
    # Production paired replay keeps the detector feature history across the
    # phase boundary, while restarting cadence and the phase-local warmup count.
    motion_rows = _collect_classic_replay_phase_rows(
        motion_packets,
        selected_subcarriers,
        replay_timing,
        resolved_warmup_packets,
        detector,
    )
    return {
        "calibration": calibration_rows,
        "static": static_rows,
        "motion": motion_rows,
        "timing": resolved_timing,
        "replay_interval_us": measured_interval_us,
    }


def load_or_compute_classic_replay_rows(
    static_presence_path: str | Path,
    motion_path: Optional[str | Path] = None,
    *,
    static_presence_packets: Optional[Sequence[dict[str, Any]]] = None,
    motion_packets: Optional[Sequence[dict[str, Any]]] = None,
    selected_subcarriers: Sequence[int],
    replay_kind: str,
    warmup_packets: Optional[int] = None,
    replay_provenance: Optional[Mapping[str, Any]] = None,
    use_cache: bool = True,
) -> Dict[str, Any]:
    """Load or build one persisted time-aware Lightweight replay-row artifact."""
    resolved_replay_provenance = _report_replay_provenance(replay_provenance)
    static_packets = (
        static_presence_packets
        if static_presence_packets is not None
        else _load_report_packet_view(static_presence_path)
    )
    replay_interval_us = measure_packet_interval_us(static_packets)
    replay_target_pps = target_pps_for_packets(static_packets, replay_interval_us)
    timing = derive_detector_timing(
        max(1, int(round(1_000_000.0 / replay_target_pps)))
    )
    timing["window_packets"] = temporal_window_slots(
        replay_target_pps, SEGMENTATION_WINDOW_SIZE_MS
    )
    resolved_warmup = (
        int(timing["window_packets"])
        if warmup_packets is None
        else max(0, int(warmup_packets))
    )
    parameters = npz_cache.classic_replay_row_parameters(
        replay_kind=replay_kind,
        selected_subcarriers=selected_subcarriers,
        timing=timing,
        replay_interval_us=replay_interval_us,
        warmup_packets=resolved_warmup,
        secondary_source=motion_path,
        replay_provenance=resolved_replay_provenance,
    )
    if use_cache:
        cached = npz_cache.load_classic_replay_row_artifact(
            static_presence_path,
            parameters=parameters,
        )
        if cached is not None:
            cached["timing"] = timing
            cached["replay_interval_us"] = replay_interval_us
            cached["cache_hit"] = True
            return cached
    lock_context = (
        npz_cache.artifact_build_lock(
            static_presence_path,
            artifact_name="classic_replay_rows",
            artifact_version=npz_cache.CLASSIC_REPLAY_ROW_ARTIFACT_VERSION,
            parameters=parameters,
        )
        if use_cache
        else nullcontext()
    )
    with lock_context:
        if use_cache:
            cached = npz_cache.load_classic_replay_row_artifact(
                static_presence_path,
                parameters=parameters,
            )
            if cached is not None:
                cached["timing"] = timing
                cached["replay_interval_us"] = replay_interval_us
                cached["cache_hit"] = True
                return cached
        resolved_motion_packets = motion_packets
        if resolved_motion_packets is None:
            resolved_motion_packets = (
                () if motion_path is None else _load_report_packet_view(motion_path)
            )
        rows = build_classic_replay_rows(
            static_packets,
            resolved_motion_packets,
            selected_subcarriers,
            timing=timing,
            replay_interval_us=replay_interval_us,
            warmup_packets=resolved_warmup,
        )
        if use_cache:
            npz_cache.save_classic_replay_row_artifact(
                static_presence_path,
                parameters=parameters,
                rows=rows,
            )
    rows["cache_hit"] = False
    return rows


class _ClassicCalibrationRowDetector:
    """Minimal detector view consumed by StartupThresholdCalibrator."""

    def __init__(self) -> None:
        self.ready = False
        self.motion_metric = 0.0

    def is_ready(self) -> bool:
        return self.ready

    def get_motion_metric(self) -> float:
        return self.motion_metric


def _calibrate_classic_replay_rows(
    rows: Mapping[str, Any],
    timing: Mapping[str, int],
) -> Optional[float]:
    """Reproduce Lightweight startup calibration from cached evaluation rows."""
    from tools.lib.lightweight_detector import LightweightDetector
    from threshold import (
        StartupThresholdCalibrator,
        get_detector_auto_factor,
        get_detector_startup_gate,
    )

    detector = build_lightweight_detector(
        threshold=1.0,
        enable_hampel=REPORT_HAMPEL_ENABLED,
        timing=dict(timing),
    )
    calibration_target_packets = max(
        1,
        int(round(CALIBRATION_DURATION_MS * 1000.0 / timing["interval_us"])),
    )
    calibrator = StartupThresholdCalibrator(
        calibration_target_packets,
        auto_factor=get_detector_auto_factor(detector),
        gate_enabled=get_detector_startup_gate(detector),
    )
    adapter = _ClassicCalibrationRowDetector()
    startup_logits: list[float] = []
    last_reset: Optional[int] = None
    X = np.asarray(rows.get("X", np.empty((0, 2))), dtype=np.float64)
    ready = np.asarray(rows.get("ready", np.empty(0)), dtype=bool)
    weights = np.asarray(rows.get("packet_weight", np.empty(0)), dtype=np.int32)
    resets = np.asarray(rows.get("reset_index", np.empty(0)), dtype=np.int32)

    for values, row_ready, packet_weight, reset_index in zip(
        X, ready, weights, resets, strict=True
    ):
        current_reset = int(reset_index)
        if last_reset is not None and current_reset != last_reset:
            calibrator = StartupThresholdCalibrator(
                calibration_target_packets,
                auto_factor=get_detector_auto_factor(detector),
                gate_enabled=get_detector_startup_gate(detector),
            )
            startup_logits = []
        last_reset = current_reset
        adapter.ready = bool(row_ready)
        if adapter.ready:
            logit = detector._calculate_logit(float(values[0]), float(values[1]))
            adapter.motion_metric = detector._sigmoid(logit)
            if len(startup_logits) < LightweightDetector.STARTUP_SAMPLE_LIMIT:
                startup_logits.append(float(logit))
        else:
            adapter.motion_metric = 0.0
        if adapter.ready:
            calibrator.observe_detector(adapter, packet_weight=int(packet_weight))
        if calibrator.is_complete():
            break

    if not calibrator.is_successful():
        return None
    calibrator.calculate_threshold()
    session_q95 = detector._quantile(startup_logits, detector.STARTUP_QUANTILE)
    if session_q95 is None:
        return float(detector.BASE_THRESHOLD)
    base_logit = np.log(detector.BASE_THRESHOLD / (1.0 - detector.BASE_THRESHOLD))
    adapted_logit = base_logit + detector.STARTUP_STRENGTH * (
        session_q95 - detector.TRAIN_IDLE_Q95_LOGIT
    )
    return float(detector._sigmoid(float(adapted_logit)))


def _score_classic_replay_phase_rows(
    rows: Mapping[str, Any],
    detector: Any,
    *,
    row_stride: Optional[int] = None,
    row_offset: int = 0,
) -> list[bool]:
    """Advance Lightweight decision state over one cached replay phase."""
    stride, offset = _normalize_replay_row_selection(row_stride, row_offset)
    states: list[bool] = []
    eligible_position = 0
    X = np.asarray(rows.get("X", np.empty((0, 2))), dtype=np.float64)
    ready = np.asarray(rows.get("ready", np.empty(0)), dtype=bool)
    eligible = np.asarray(rows.get("eligible", np.empty(0)), dtype=bool)
    resets = np.asarray(rows.get("reset_index", np.empty(0)), dtype=np.int32)
    last_reset: Optional[int] = None
    for values, row_ready, row_eligible, reset_index in zip(
        X, ready, eligible, resets, strict=True
    ):
        current_reset = int(reset_index)
        if last_reset is not None and current_reset != last_reset:
            detector.reset()
        last_reset = current_reset
        if not row_ready:
            detector._current_probability = 0.0
            detector._state = MotionState.IDLE
            continue
        detector._current_turb_autocorr = float(values[0])
        detector._current_turb_iqr_over_mean_aggr = float(values[1])
        detector._current_logit = detector._calculate_logit(
            detector._current_turb_autocorr,
            detector._current_turb_iqr_over_mean_aggr,
        )
        detector._current_probability = detector._sigmoid(detector._current_logit)
        detector._observe_settled_level()
        detector._state = (
            MotionState.MOTION
            if detector._current_probability > detector._threshold
            else MotionState.IDLE
        )
        if row_eligible:
            if stride is None or eligible_position % stride == offset:
                states.append(detector._state == MotionState.MOTION)
            eligible_position += 1
    return states


def compute_classic_row_result(
    rows: Mapping[str, Any],
    *,
    row_stride: Optional[int] = None,
    row_offset: int = 0,
) -> Optional[tuple[float, Dict[str, float]]]:
    """Evaluate one paired Lightweight replay from canonical time-aware rows."""
    timing = dict(rows["timing"])
    adaptive_threshold = _calibrate_classic_replay_rows(
        rows["calibration"], timing
    )
    if adaptive_threshold is None:
        return None
    detector = build_lightweight_detector(
        threshold=adaptive_threshold,
        enable_hampel=REPORT_HAMPEL_ENABLED,
        timing=timing,
    )
    detector._adapted_threshold_ready = True
    baseline_states = _score_classic_replay_phase_rows(
        rows["static"],
        detector,
        row_stride=row_stride,
        row_offset=row_offset,
    )
    motion_states = _score_classic_replay_phase_rows(
        rows["motion"],
        detector,
        row_stride=row_stride,
        row_offset=row_offset,
    )
    tp = sum(1 for state in motion_states if state)
    fn = len(motion_states) - tp
    fp = sum(1 for state in baseline_states if state)
    tn = len(baseline_states) - fp
    recall = tp / (tp + fn) * 100.0 if (tp + fn) > 0 else 0.0
    precision = tp / (tp + fp) * 100.0 if (tp + fp) > 0 else 0.0
    fp_rate = fp / len(baseline_states) * 100.0 if baseline_states else 0.0
    f1 = (
        2 * (precision / 100.0) * (recall / 100.0)
        / ((precision + recall) / 100.0)
        * 100.0
        if (precision + recall) > 0
        else 0.0
    )
    return adaptive_threshold, {
        "tp": tp,
        "fn": fn,
        "tn": tn,
        "fp": fp,
        "recall": recall,
        "precision": precision,
        "fp_rate": fp_rate,
        "f1": f1,
        "num_baseline": len(baseline_states),
        "num_movement": len(motion_states),
        **_evaluate_idle_runtime_policy_evaluations(baseline_states),
    }


def evaluate_detector_packets(
    detector: Any,
    static_presence_packets: Sequence[dict[str, Any]],
    motion_packets: Sequence[dict[str, Any]],
    selected_band: Sequence[int],
    warmup: Optional[int] = None,
) -> Dict[str, float]:
    """Replay one baseline/motion pair at the production evaluation cadence."""
    warmup = 0 if warmup is None else max(0, int(warmup))
    baseline_eval_count = 0
    movement_eval_count = 0
    static_presence_motion_packets = 0
    baseline_motion_states = []
    # The detector was built for the cadence this stream actually runs at, so
    # the replay has to advance it on the same cadence rather than the nominal
    # one, and warm up over the window the detector really holds.
    interval_us = measure_packet_interval_us(static_presence_packets)
    detector_window = getattr(detector, "get_window_size", None)
    if callable(detector_window):
        warmup = max(1, int(detector_window()))
    if warmup and hasattr(detector, "set_minimum_valid_samples"):
        detector.set_minimum_valid_samples(minimum_valid_slots(warmup))
    target_pps = target_pps_for_packets(static_presence_packets, interval_us)
    _, cadence = timing_cadence_for_window(warmup, interval_us)
    packets_since_reset = 0
    for admission in iter_temporal_admissions(
        static_presence_packets,
        target_pps=target_pps,
        window_size_ms=SEGMENTATION_WINDOW_SIZE_MS,
        fallback_interval_us=interval_us,
    ):
        packet_index, pkt = admission.packet_index, admission.packet
        if admission.reset_required:
            cadence.reset()
            packets_since_reset = 0
        apply_temporal_admission(detector, admission)
        cadence.note_packet(elapsed_us=admission.coverage_us)
        should_evaluate = cadence.should_evaluate()
        if should_evaluate:
            cadence.after_evaluation()
        detector.process_packet(
            _packet_csi_data(pkt),
            selected_band,
            rssi_dbm=pkt.get("rssi_dbm"),
            timestamp_us=_packet_timestamp_us(pkt, packet_index, interval_us),
        )
        packets_since_reset = admission.slot_index + 1
        if not should_evaluate:
            continue
        detector.update_state()
        if not _is_scored_replay_evaluation(detector, packets_since_reset, warmup):
            continue
        baseline_eval_count += 1
        is_motion = detector.get_state() == MotionState.MOTION
        baseline_motion_states.append(is_motion)
        if is_motion:
            static_presence_motion_packets += 1

    motion_with_motion = 0
    motion_without_motion = 0
    _, cadence = timing_cadence_for_window(warmup, interval_us)
    packets_since_reset = 0
    for admission in iter_temporal_admissions(
        motion_packets,
        target_pps=target_pps,
        window_size_ms=SEGMENTATION_WINDOW_SIZE_MS,
        fallback_interval_us=interval_us,
    ):
        packet_index, pkt = admission.packet_index, admission.packet
        if admission.reset_required:
            cadence.reset()
            packets_since_reset = 0
        apply_temporal_admission(detector, admission)
        cadence.note_packet(elapsed_us=admission.coverage_us)
        should_evaluate = cadence.should_evaluate()
        if should_evaluate:
            cadence.after_evaluation()
        detector.process_packet(
            _packet_csi_data(pkt),
            selected_band,
            rssi_dbm=pkt.get("rssi_dbm"),
            timestamp_us=_packet_timestamp_us(pkt, packet_index, interval_us),
        )
        packets_since_reset = admission.slot_index + 1
        if not should_evaluate:
            continue
        detector.update_state()
        if not _is_scored_replay_evaluation(detector, packets_since_reset, warmup):
            continue
        movement_eval_count += 1
        if detector.get_state() == MotionState.MOTION:
            motion_with_motion += 1
        else:
            motion_without_motion += 1

    pkt_tp = motion_with_motion
    pkt_fn = motion_without_motion
    pkt_tn = max(baseline_eval_count - static_presence_motion_packets, 0)
    pkt_fp = static_presence_motion_packets
    pkt_recall = pkt_tp / (pkt_tp + pkt_fn) * 100.0 if (pkt_tp + pkt_fn) > 0 else 0.0
    pkt_precision = pkt_tp / (pkt_tp + pkt_fp) * 100.0 if (pkt_tp + pkt_fp) > 0 else 0.0
    pkt_fp_rate = (
        pkt_fp / baseline_eval_count * 100.0 if baseline_eval_count > 0 else 0.0
    )
    pkt_f1 = (
        2 * (pkt_precision / 100.0) * (pkt_recall / 100.0) / ((pkt_precision + pkt_recall) / 100.0) * 100.0
        if (pkt_precision + pkt_recall) > 0
        else 0.0
    )
    policy_metrics = _evaluate_idle_runtime_policy_evaluations(baseline_motion_states)
    return {
        "tp": pkt_tp,
        "fn": pkt_fn,
        "tn": pkt_tn,
        "fp": pkt_fp,
        "recall": pkt_recall,
        "precision": pkt_precision,
        "fp_rate": pkt_fp_rate,
        "f1": pkt_f1,
        "num_baseline": baseline_eval_count,
        "num_movement": movement_eval_count,
        **policy_metrics,
    }


def compute_classic_packet_result(
    static_presence_packets: Sequence[dict[str, Any]],
    motion_packets: Sequence[dict[str, Any]],
    selected_band: Sequence[int],
    window_size: Optional[int],
) -> Optional[tuple[float, Dict[str, float]]]:
    """Replay the Lightweight detector on explicit packet streams."""
    calibrated = build_calibrated_lightweight_detector(
        static_presence_packets,
        selected_subcarriers=selected_band,
        enable_hampel=REPORT_HAMPEL_ENABLED,
    )
    if calibrated is None:
        return None

    detector, adaptive_threshold = calibrated
    if window_size is None:
        window_size = detector.get_window_size()
    metrics = evaluate_detector_packets(
        detector,
        static_presence_packets,
        motion_packets,
        selected_band,
        warmup=window_size,
    )
    return adaptive_threshold, metrics


@lru_cache(maxsize=None)
def compute_classic_dataset_result(
    static_presence_path: str | Path,
    motion_path: str | Path,
    selected_band: tuple[int, ...],
    window_size: Optional[int],
) -> Optional[tuple[float, Dict[str, float]]]:
    """Run Lightweight inference over canonical time-aware replay rows."""
    rows = load_or_compute_classic_replay_rows(
        replay_kind="classic_dataset",
        static_presence_path=static_presence_path,
        motion_path=motion_path,
        selected_subcarriers=selected_band,
        warmup_packets=window_size,
    )
    return compute_classic_row_result(rows)


def compute_ml_packet_result(
    static_presence_packets: Sequence[dict[str, Any]],
    motion_packets: Sequence[dict[str, Any]],
    selected_subcarriers: Sequence[int],
    window_size: Optional[int],
    threshold: float,
    feature_names: Sequence[str] = (),
) -> tuple[Dict[str, float], Dict[str, Dict[str, tuple[float, ...]]]]:
    """Replay the High-Accuracy detector on explicit packet streams."""
    runtime_feature_names = tuple(RUNTIME_FEATURE_NAMES)
    interval_us = measure_packet_interval_us(static_presence_packets)
    replay_target_pps = target_pps_for_packets(static_presence_packets, interval_us)
    resolved_window_size = (
        int(window_size)
        if window_size is not None
        else temporal_window_slots(replay_target_pps, SEGMENTATION_WINDOW_SIZE_MS)
    )
    static_rows = build_ml_replay_rows(
        static_presence_packets,
        selected_subcarriers,
        resolved_window_size,
        runtime_feature_names,
        target_pps=replay_target_pps,
    )
    motion_rows = build_ml_replay_rows(
        motion_packets,
        selected_subcarriers,
        resolved_window_size,
        runtime_feature_names,
        target_pps=replay_target_pps,
    )
    return _compute_ml_row_result(
        static_rows,
        motion_rows,
        threshold,
        feature_names,
    )


def _compute_ml_row_result(
    static_rows: Mapping[str, Any],
    motion_rows: Mapping[str, Any],
    threshold: float,
    feature_names: Sequence[str] = (),
) -> tuple[Dict[str, float], Dict[str, Dict[str, tuple[float, ...]]]]:
    """Evaluate canonical runtime-tick rows with the exported inference path."""
    from tools.lib.high_accuracy_detector import predict as predict_runtime_probability

    runtime_feature_names = tuple(RUNTIME_FEATURE_NAMES)
    requested_feature_names = _resolve_ml_replay_feature_names(feature_names)
    # Report the deployed scalar inference contract. NumPy matrix
    # multiplication can accumulate the same float32 weights in a different
    # order and move probabilities that sit close to 0.5 across the decision
    # boundary, even though the feature rows themselves are identical.
    static_probabilities = np.fromiter(
        (
            predict_runtime_probability(features)
            for features in np.asarray(static_rows["X"], dtype=np.float32)
        ),
        dtype=np.float64,
        count=len(static_rows["X"]),
    )
    motion_probabilities = np.fromiter(
        (
            predict_runtime_probability(features)
            for features in np.asarray(motion_rows["X"], dtype=np.float32)
        ),
        dtype=np.float64,
        count=len(motion_rows["X"]),
    )
    static_presence_motion_states = static_probabilities > float(threshold)
    motion_states = motion_probabilities > float(threshold)
    static_presence_motion_packets = int(np.sum(static_presence_motion_states))
    static_presence_eval_count = int(len(static_probabilities))
    motion_eval_count = int(len(motion_probabilities))
    motion_with_motion = int(np.sum(motion_states))
    motion_without_motion = int(motion_eval_count - motion_with_motion)

    pkt_tp = motion_with_motion
    pkt_fn = motion_without_motion
    pkt_tn = max(static_presence_eval_count - static_presence_motion_packets, 0)
    pkt_fp = static_presence_motion_packets
    pkt_recall = pkt_tp / (pkt_tp + pkt_fn) * 100.0 if (pkt_tp + pkt_fn) > 0 else 0.0
    pkt_precision = pkt_tp / (pkt_tp + pkt_fp) * 100.0 if (pkt_tp + pkt_fp) > 0 else 0.0
    pkt_fp_rate = pkt_fp / static_presence_eval_count * 100.0 if static_presence_eval_count > 0 else 0.0
    pkt_f1 = (
        2 * (pkt_precision / 100.0) * (pkt_recall / 100.0) / ((pkt_precision + pkt_recall) / 100.0) * 100.0
        if (pkt_precision + pkt_recall) > 0
        else 0.0
    )

    policy_metrics = _evaluate_idle_runtime_policy_evaluations(static_presence_motion_states)
    metrics = {
        "tp": pkt_tp,
        "fn": pkt_fn,
        "tn": pkt_tn,
        "fp": pkt_fp,
        "recall": pkt_recall,
        "precision": pkt_precision,
        "fp_rate": pkt_fp_rate,
        "f1": pkt_f1,
        "num_baseline": static_presence_eval_count,
        "num_movement": motion_eval_count,
        **policy_metrics,
    }
    payload_feature_indices = {
        name: runtime_feature_names.index(name) for name in requested_feature_names
    }
    feature_payload = {
        "baseline": {
            name: tuple(float(value) for value in static_rows["X"][:, payload_feature_indices[name]])
            for name in requested_feature_names
        },
        "motion": {
            name: tuple(float(value) for value in motion_rows["X"][:, payload_feature_indices[name]])
            for name in requested_feature_names
        },
    }
    return metrics, feature_payload


def compute_ml_dataset_result(
    static_presence_path: str | Path,
    motion_path: str | Path,
    selected_subcarriers: tuple[int, ...],
    window_size: Optional[int],
    threshold: float,
    feature_names: tuple[str, ...] = (),
) -> tuple[Dict[str, float], Dict[str, Dict[str, tuple[float, ...]]]]:
    """Run ML inference over the shared canonical time-aware row cache."""
    runtime_feature_names = tuple(RUNTIME_FEATURE_NAMES)
    resolved_window_size = window_size
    if resolved_window_size is None:
        static_packets = _load_report_packet_view(static_presence_path)
        interval_us = measure_packet_interval_us(static_packets)
        resolved_window_size = temporal_window_slots(
            target_pps_for_packets(static_packets, interval_us),
            SEGMENTATION_WINDOW_SIZE_MS,
        )
    static_rows = load_or_compute_ml_replay_rows(
        static_presence_path,
        selected_subcarriers=selected_subcarriers,
        window_size=resolved_window_size,
        feature_names=runtime_feature_names,
        sample_contract="replay_tick",
    )
    motion_rows = load_or_compute_ml_replay_rows(
        motion_path,
        selected_subcarriers=selected_subcarriers,
        window_size=resolved_window_size,
        feature_names=runtime_feature_names,
        sample_contract="replay_tick",
    )
    return _compute_ml_row_result(
        static_rows,
        motion_rows,
        threshold,
        feature_names,
    )


def replay_idle_stream(
    detector: Any,
    packets: Sequence[dict[str, Any]],
    selected_subcarriers: Sequence[int],
    window_size: int,
) -> list[bool]:
    """Replay one motion-free stream, returning the raw state per evaluation.

    Empty-room recordings are the only ground truth in the corpus that contains
    no person at all, which makes them the reference both detectors are gated
    on. See the empty-room false-positive ADR.
    """
    interval_us = measure_packet_interval_us(packets)
    target_pps = target_pps_for_packets(packets, interval_us)
    _, cadence = timing_cadence_for_window(window_size, interval_us)
    packets_since_reset = 0
    raw_motion_states: list[bool] = []
    for admission in iter_temporal_admissions(
        packets,
        target_pps=target_pps,
        window_size_ms=SEGMENTATION_WINDOW_SIZE_MS,
        fallback_interval_us=interval_us,
    ):
        packet_index, pkt = admission.packet_index, admission.packet
        if admission.reset_required:
            cadence.reset()
            packets_since_reset = 0
        apply_temporal_admission(detector, admission)
        cadence.note_packet(elapsed_us=admission.coverage_us)
        should_evaluate = cadence.should_evaluate()
        if should_evaluate:
            cadence.after_evaluation()
        detector.process_packet(
            _packet_csi_data(pkt),
            selected_subcarriers,
            rssi_dbm=pkt.get("rssi_dbm"),
            timestamp_us=_packet_timestamp_us(pkt, packet_index, interval_us),
        )
        packets_since_reset = admission.slot_index + 1
        if not should_evaluate:
            continue
        detector.update_state()
        if not _is_scored_replay_evaluation(
            detector, packets_since_reset, window_size
        ):
            continue
        raw_motion_states.append(detector.get_state() == MotionState.MOTION)
    return raw_motion_states


def _idle_stream_metrics(raw_motion_states: Sequence[bool]) -> Dict[str, float]:
    """Summarise an idle replay as raw rate plus debounced alarm count."""
    eval_count = len(raw_motion_states)
    motion_packets = sum(1 for state in raw_motion_states if state)
    policy = _evaluate_idle_runtime_policy_evaluations(raw_motion_states)
    return {
        "motion_packets": motion_packets,
        "eval_count": eval_count,
        "fp_rate": motion_packets / eval_count * 100.0 if eval_count > 0 else 0.0,
        "effective_alarms": policy["effective_alarms"],
    }


@lru_cache(maxsize=None)
def compute_classic_empty_fp_result(
    empty_dataset_path: str | Path,
    selected_subcarriers: tuple[int, ...],
) -> Dict[str, float]:
    """Run empty-room Lightweight inference over canonical time-aware rows."""
    rows = load_or_compute_classic_replay_rows(
        empty_dataset_path,
        replay_kind="classic_empty_fp",
        selected_subcarriers=selected_subcarriers,
    )
    result = compute_classic_row_result(rows)
    if result is None:
        return {}
    _adaptive_threshold, metrics = result
    return {
        "motion_packets": int(metrics["fp"]),
        "eval_count": int(metrics["num_baseline"]),
        "fp_rate": float(metrics["fp_rate"]),
        "effective_alarms": int(metrics["effective_alarms"]),
    }


def compute_ml_empty_fp_result(
    empty_dataset_path: str | Path,
    selected_subcarriers: tuple[int, ...],
    window_size: Optional[int],
    threshold: float,
) -> Dict[str, float]:
    """Run empty-room ML inference over canonical runtime-tick rows."""
    from tools.lib.ml_training.export import (
        load_exported_ml_weights,
        predict_exported_probabilities_from_weights,
    )

    rows = load_or_compute_ml_replay_rows(
        empty_dataset_path,
        selected_subcarriers=selected_subcarriers,
        window_size=window_size,
        feature_names=tuple(RUNTIME_FEATURE_NAMES),
        sample_contract="replay_tick",
    )
    probabilities = np.asarray(
        predict_exported_probabilities_from_weights(
            load_exported_ml_weights(),
            np.asarray(rows["X"], dtype=np.float32),
        ),
        dtype=np.float64,
    )
    return _idle_stream_metrics(
        probabilities > float(threshold)
    )


def extract_motion_start_from_description(description: Optional[str]) -> Optional[int]:
    """Extract motion start packet index from free-text test metadata."""
    if not description:
        return None

    import re

    match = re.search(
        r"motion\s+starts\s+at\s+packet(?:\s+index)?(?:\s+n\.)?\s+(\d+)",
        str(description),
        re.IGNORECASE,
    )
    if match:
        return int(match.group(1))
    return None


def _normalize_long_test_chip_filter(chips: Optional[Iterable[str]]) -> Optional[tuple[str, ...]]:
    if not chips:
        return None
    return tuple(sorted({str(chip).upper() for chip in chips}))


@lru_cache(maxsize=None)
def _get_available_long_test_dataset_specs_cached(
    dataset_revision: str,
    chips_key: Optional[tuple[str, ...]],
) -> tuple[tuple[Any, ...], ...]:
    """Return long-recording metadata without loading packet payloads."""
    del dataset_revision
    dataset_info = _load_dataset_info()
    long_recording_records = _long_recording_entry_records(dataset_info)
    if not long_recording_records:
        return tuple()

    requested = set(chips_key) if chips_key else None
    datasets = []

    for label_group, entry in long_recording_records:
        chip = str(entry.get("chip", "")).upper()
        if requested and chip not in requested:
            continue

        filename = entry.get("filename")
        if not filename:
            continue

        test_path = resolve_entry_path(label_group, entry)
        if not test_path.exists():
            continue

        num_packets = int(entry.get("num_packets", 0) or 0)
        if num_packets < 2:
            continue

        motion_start_packet = extract_motion_start_from_description(entry.get("description"))
        if motion_start_packet is None:
            motion_start_packet = num_packets

        if motion_start_packet <= 0 or motion_start_packet > num_packets:
            continue

        datasets.append(
            (
                test_path,
                motion_start_packet,
                num_packets,
                chip,
                entry,
            )
        )

    datasets.sort(key=lambda item: item[3])
    return tuple(datasets)


def get_available_long_test_dataset_specs(
    chips: Optional[Iterable[str]] = None,
) -> list[tuple[Any, ...]]:
    """Return lightweight long-recording specs suitable for parametrization."""
    return list(
        _get_available_long_test_dataset_specs_cached(
            dataset_info_revision(),
            _normalize_long_test_chip_filter(chips)
        )
    )


@lru_cache(maxsize=None)
def _load_long_test_packets_cached(path_value: str) -> _LongTestPacketView:
    """Load one long-recording packet stream as a zero-copy packet+metadata view."""
    arrays = (
        load_npz_arrays(Path(path_value))
        if DIAGNOSTIC_ALL_PHY
        else load_npz_sensing_arrays(Path(path_value))
    )
    if "csi_data" in arrays:
        csi_key = "csi_data"
    elif "csi" in arrays:
        csi_key = "csi"
    else:
        raise ValueError(f"No CSI data found in {path_value}")

    csi_matrix = np.asarray(arrays[csi_key], dtype=np.int8)
    if csi_matrix.ndim != 2 or not csi_matrix.flags.c_contiguous:
        csi_matrix = np.ascontiguousarray(csi_matrix, dtype=np.int8)
    rssi_dbm = _coerce_long_test_rssi_series(arrays.get("rssi_dbm"), len(csi_matrix))
    seq_num = _coerce_long_test_int_series(
        arrays.get("stream_seq_num"),
        len(csi_matrix),
        dtype=np.uint32,
    )
    device_ticks_us = _coerce_long_test_int_series(
        arrays.get("device_ticks_us"),
        len(csi_matrix),
        dtype=np.uint64,
    )
    wifi_rx_ts_us = _coerce_long_test_int_series(
        arrays.get("wifi_rx_ts_us"),
        len(csi_matrix),
        dtype=np.uint32,
    )
    return _LongTestPacketView(
        _CsiRowView(csi_matrix),
        rssi_dbm,
        seq_num,
        device_ticks_us,
        wifi_rx_ts_us,
    )


def load_long_test_dataset(spec: tuple[Any, ...]) -> tuple[Any, ...]:
    """Materialize one long-recording spec as baseline and movement views."""
    test_path, motion_start_packet, num_packets, chip, entry = spec
    packets = _load_long_test_packets_cached(str(test_path))
    if len(packets) != num_packets:
        raise ValueError(
            f"Packet count mismatch for {test_path}: metadata={num_packets}, npz={len(packets)}"
        )
    return (
        test_path,
        packets[:motion_start_packet],
        packets[motion_start_packet:],
        motion_start_packet,
        chip,
        entry,
    )


def get_available_long_test_datasets(chips: Optional[Iterable[str]] = None) -> list[tuple[Any, ...]]:
    """Load long-recording replays with validated split metadata."""
    return [
        load_long_test_dataset(spec)
        for spec in get_available_long_test_dataset_specs(chips=chips)
    ]


def _packet_csi_data(packet: Any) -> Any:
    """Return CSI bytes from a packet dictionary or a compact CSI row.

    The payload is handed to the MicroPython detectors, which index it element
    by element dozens of times per packet. The packet view stores int8 NumPy
    arrays, and every element read from one builds a NumPy scalar, so plain
    Python ints are worth roughly a factor of two on replay. `int8` is already
    signed, so `tolist()` preserves every value exactly.
    """
    csi_data = packet["csi_data"] if isinstance(packet, MappingABC) else packet
    return csi_data.tolist() if isinstance(csi_data, np.ndarray) else csi_data


def _packet_timestamp_us(
    packet: Any,
    packet_index: int,
    nominal_interval_us: int,
) -> int:
    """Return the replay timestamp using the same precedence as the C++ gate."""
    return int(
        packet_timestamp_us(
            packet,
            fallback_index=packet_index,
            fallback_interval_us=nominal_interval_us,
        )
    )


def _packet_rssi_dbm(packet: Any) -> Any:
    """Return optional RSSI metadata from a packet dictionary or packet object."""
    if isinstance(packet, MappingABC):
        return packet.get("rssi_dbm")
    return getattr(packet, "rssi_dbm", None)


def _evaluate_ml_long_cached_rows(
    source_path: str | Path,
    motion_start_packet: int,
    threshold: float = 0.5,
) -> Dict[str, float]:
    """Evaluate one long capture from canonical runtime-tick rows."""
    from tools.lib.ml_training.export import (
        load_exported_ml_weights,
        predict_exported_probabilities_from_weights,
    )

    rows = load_or_compute_ml_replay_rows(
        source_path,
        selected_subcarriers=DEFAULT_SUBCARRIERS,
        window_size=None,
        feature_names=tuple(RUNTIME_FEATURE_NAMES),
        sample_contract="replay_tick",
    )
    probabilities = np.asarray(
        predict_exported_probabilities_from_weights(
            load_exported_ml_weights(),
            np.asarray(rows["X"], dtype=np.float32),
        ),
        dtype=np.float64,
    )
    packet_index = np.asarray(rows["packet_index"], dtype=np.int64)
    baseline_states = probabilities[packet_index < int(motion_start_packet)] > threshold
    movement_states = probabilities[packet_index >= int(motion_start_packet)] > threshold
    baseline_eval_count = int(len(baseline_states))
    movement_eval_count = int(len(movement_states))
    fp = int(np.sum(baseline_states))
    tp = int(np.sum(movement_states))
    fn = int(movement_eval_count - tp)
    tn = int(baseline_eval_count - fp)
    recall = tp / (tp + fn) * 100.0 if (tp + fn) > 0 else 0.0
    precision = tp / (tp + fp) * 100.0 if (tp + fp) > 0 else 0.0
    fp_rate = fp / baseline_eval_count * 100.0 if baseline_eval_count > 0 else 0.0
    f1 = (
        2 * (precision / 100.0) * (recall / 100.0)
        / ((precision + recall) / 100.0)
        * 100.0
        if (precision + recall) > 0
        else 0.0
    )
    return {
        "baseline_eval_count": baseline_eval_count,
        "movement_eval_count": movement_eval_count,
        "tp": tp,
        "fn": fn,
        "fp": fp,
        "tn": tn,
        "recall": recall,
        "precision": precision,
        "fp_rate": fp_rate,
        "f1": f1,
        **_evaluate_idle_runtime_policy_evaluations(baseline_states),
    }


def compute_classic_long_recording_result(
    source_path: str | Path,
    motion_start_packet: int,
    *,
    selected_subcarriers: Sequence[int] = DEFAULT_SUBCARRIERS,
) -> Optional[Dict[str, float]]:
    """Replay one long recording through cached time-aware Lightweight rows."""
    packets = _load_long_test_packets_cached(str(source_path))
    baseline_packets = packets[:motion_start_packet]
    movement_packets = packets[motion_start_packet:]
    rows = load_or_compute_classic_replay_rows(
        source_path,
        replay_kind="classic_long_recording",
        static_presence_packets=baseline_packets,
        motion_packets=movement_packets,
        selected_subcarriers=selected_subcarriers,
        warmup_packets=None,
        replay_provenance={"motion_start_packet": int(motion_start_packet)},
    )
    result = compute_classic_row_result(rows)
    if result is None:
        return None
    adaptive_threshold, metrics = result
    return {
        "baseline_eval_count": int(metrics["num_baseline"]),
        "movement_eval_count": int(metrics["num_movement"]),
        "adaptive_threshold": float(adaptive_threshold),
        "tp": int(metrics["tp"]),
        "fn": int(metrics["fn"]),
        "fp": int(metrics["fp"]),
        "tn": int(metrics["tn"]),
        "recall": float(metrics["recall"]),
        "precision": float(metrics["precision"]),
        "fp_rate": float(metrics["fp_rate"]),
        "f1": float(metrics["f1"]),
        "effective_alarms": int(metrics["effective_alarms"]),
        "false_motion_evaluations": int(metrics["false_motion_evaluations"]),
    }


def evaluate_ml_long_recording(
    baseline_packets: Sequence[Any],
    movement_packets: Sequence[Any],
    *,
    source_path: Optional[str | Path] = None,
    motion_start_packet: Optional[int] = None,
) -> Dict[str, float]:
    """Run HighAccuracyDetector at the production evaluation cadence."""
    if source_path is not None and motion_start_packet is not None:
        return _evaluate_ml_long_cached_rows(source_path, motion_start_packet)

    from tools.lib.high_accuracy_detector import HighAccuracyDetector

    interval_us = measure_packet_interval_us(baseline_packets)
    target_pps = target_pps_for_packets(baseline_packets, interval_us)
    warmup = temporal_window_slots(target_pps, SEGMENTATION_WINDOW_SIZE_MS)
    detector = HighAccuracyDetector(threshold=0.5, window_size=warmup)
    if hasattr(detector, "set_minimum_valid_samples"):
        detector.set_minimum_valid_samples(minimum_valid_slots(warmup))

    baseline_eval_count = 0
    movement_eval_count = 0
    baseline_motion_packets = 0
    baseline_motion_states = []
    movement_with_motion = 0
    movement_without_motion = 0

    for admission, should_evaluate, packets_since_reset in temporal_detector_ticks(
        detector, baseline_packets, interval_us
    ):
        packet_index, pkt = admission.packet_index, admission.packet
        detector.process_packet(
            _packet_csi_data(pkt),
            DEFAULT_SUBCARRIERS,
            rssi_dbm=_packet_rssi_dbm(pkt),
            timestamp_us=_packet_timestamp_us(pkt, packet_index, interval_us),
        )
        if not should_evaluate:
            continue
        detector.update_state()
        if _is_scored_replay_evaluation(detector, packets_since_reset, warmup):
            baseline_eval_count += 1
            is_motion = detector.get_state() == MotionState.MOTION
            baseline_motion_states.append(is_motion)
            if is_motion:
                baseline_motion_packets += 1

    for admission, should_evaluate, packets_since_reset in temporal_detector_ticks(
        detector, movement_packets, interval_us
    ):
        packet_index, pkt = admission.packet_index, admission.packet
        detector.process_packet(
            _packet_csi_data(pkt),
            DEFAULT_SUBCARRIERS,
            rssi_dbm=_packet_rssi_dbm(pkt),
            timestamp_us=_packet_timestamp_us(pkt, packet_index, interval_us),
        )
        if not should_evaluate:
            continue
        detector.update_state()
        if _is_scored_replay_evaluation(detector, packets_since_reset, warmup):
            movement_eval_count += 1
            if detector.get_state() == MotionState.MOTION:
                movement_with_motion += 1
            else:
                movement_without_motion += 1

    tp = movement_with_motion
    fn = movement_without_motion
    fp = baseline_motion_packets
    tn = max(baseline_eval_count - baseline_motion_packets, 0)
    recall = tp / (tp + fn) * 100.0 if (tp + fn) > 0 else 0.0
    precision = tp / (tp + fp) * 100.0 if (tp + fp) > 0 else 0.0
    fp_rate = fp / baseline_eval_count * 100.0 if baseline_eval_count > 0 else 0.0
    f1 = (
        2 * (precision / 100.0) * (recall / 100.0) / ((precision + recall) / 100.0) * 100.0
        if (precision + recall) > 0
        else 0.0
    )

    policy_metrics = _evaluate_idle_runtime_policy_evaluations(baseline_motion_states)
    return {
        "baseline_eval_count": baseline_eval_count,
        "movement_eval_count": movement_eval_count,
        "tp": tp,
        "fn": fn,
        "fp": fp,
        "tn": tn,
        "recall": recall,
        "precision": precision,
        "fp_rate": fp_rate,
        "f1": f1,
        **policy_metrics,
    }


def evaluate_classic_long_recording(
    baseline_packets: Sequence[Any],
    movement_packets: Sequence[Any],
    *,
    source_path: Optional[str | Path] = None,
    motion_start_packet: Optional[int] = None,
) -> Optional[Dict[str, float]]:
    """Run startup-calibrated LightweightDetector at the production cadence."""
    if source_path is not None and motion_start_packet is not None:
        return compute_classic_long_recording_result(
            source_path,
            motion_start_packet,
            selected_subcarriers=DEFAULT_SUBCARRIERS,
        )

    calibrated = build_calibrated_lightweight_detector(
        baseline_packets,
        selected_subcarriers=DEFAULT_SUBCARRIERS,
        enable_hampel=REPORT_HAMPEL_ENABLED,
    )
    if calibrated is None:
        return None
    detector, adaptive_threshold = calibrated
    warmup = detector.get_window_size()
    baseline_eval_count = 0
    movement_eval_count = 0
    baseline_motion_packets = 0
    movement_with_motion = 0
    movement_without_motion = 0

    baseline_motion_states = []

    interval_us = measure_packet_interval_us(baseline_packets)
    for admission, should_evaluate, packets_since_reset in temporal_detector_ticks(
        detector, baseline_packets, interval_us
    ):
        packet_index, pkt = admission.packet_index, admission.packet
        detector.process_packet(
            _packet_csi_data(pkt),
            DEFAULT_SUBCARRIERS,
            rssi_dbm=_packet_rssi_dbm(pkt),
            timestamp_us=_packet_timestamp_us(pkt, packet_index, interval_us),
        )
        if not should_evaluate:
            continue
        detector.update_state()
        if _is_scored_replay_evaluation(detector, packets_since_reset, warmup):
            baseline_eval_count += 1
            is_motion = detector.get_state() == MotionState.MOTION
            baseline_motion_states.append(is_motion)
            if is_motion:
                baseline_motion_packets += 1

    for admission, should_evaluate, packets_since_reset in temporal_detector_ticks(
        detector, movement_packets, interval_us
    ):
        packet_index, pkt = admission.packet_index, admission.packet
        detector.process_packet(
            _packet_csi_data(pkt),
            DEFAULT_SUBCARRIERS,
            rssi_dbm=_packet_rssi_dbm(pkt),
            timestamp_us=_packet_timestamp_us(pkt, packet_index, interval_us),
        )
        if not should_evaluate:
            continue
        detector.update_state()
        if _is_scored_replay_evaluation(detector, packets_since_reset, warmup):
            movement_eval_count += 1
            if detector.get_state() == MotionState.MOTION:
                movement_with_motion += 1
            else:
                movement_without_motion += 1

    tp = movement_with_motion
    fn = movement_without_motion
    fp = baseline_motion_packets
    tn = max(baseline_eval_count - baseline_motion_packets, 0)
    recall = tp / (tp + fn) * 100.0 if (tp + fn) > 0 else 0.0
    precision = tp / (tp + fp) * 100.0 if (tp + fp) > 0 else 0.0
    fp_rate = fp / baseline_eval_count * 100.0 if baseline_eval_count > 0 else 0.0
    f1 = (
        2 * (precision / 100.0) * (recall / 100.0) / ((precision + recall) / 100.0) * 100.0
        if (precision + recall) > 0
        else 0.0
    )
    policy_metrics = _evaluate_idle_runtime_policy_evaluations(baseline_motion_states)
    return {
        "adaptive_threshold": adaptive_threshold,
        "warmup": warmup,
        "baseline_eval_count": baseline_eval_count,
        "movement_eval_count": movement_eval_count,
        "tp": tp,
        "fn": fn,
        "fp": fp,
        "tn": tn,
        "recall": recall,
        "precision": precision,
        "fp_rate": fp_rate,
        "f1": f1,
        **policy_metrics,
    }


def _average_detector_metrics(entries: Sequence[Dict[str, float]]) -> Optional[Dict[str, float]]:
    if not entries:
        return None
    return {
        "count": len(entries),
        "recall": sum(entry["recall"] for entry in entries) / len(entries),
        "min_recall": min(entry["recall"] for entry in entries),
        "precision": sum(entry["precision"] for entry in entries) / len(entries),
        "fp_rate": sum(entry["fp_rate"] for entry in entries) / len(entries),
        "max_fp_rate": max(entry["fp_rate"] for entry in entries),
        "f1": sum(entry["f1"] for entry in entries) / len(entries),
        "effective_alarms": sum(entry["effective_alarms"] for entry in entries),
    }


def compute_performance_report_data(
    progress: Optional[ProgressCallback] = None,
) -> Dict[str, Dict[str, Dict[str, Dict[str, float]]]]:
    """Compute all metrics published in docs/performance/README.md."""
    paired_results: dict[
        str, dict[str, dict[str, list[Dict[str, float]]]]
    ] = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    # Published detector metrics use only reserved selection and holdout pairs.
    # Training pairs remain covered by the validation suites but are not replayed
    # or summarized by this report generator.
    normal_results: dict[str, dict[str, list[Dict[str, float]]]] = {
        "classic": defaultdict(list),
        "ml": defaultdict(list),
    }
    stress_results: dict[str, dict[str, list[Dict[str, float]]]] = {
        "classic": defaultdict(list),
        "ml": defaultdict(list),
    }
    long_results: dict[str, dict[str, list[Dict[str, float]]]] = defaultdict(lambda: defaultdict(list))

    paired_datasets = tuple(
        spec
        for spec in _get_available_paired_dataset_specs_cached(dataset_info_revision())
        if spec[6] in REPORT_DATASET_ROLES
    )
    real_count = sum(not item[5] for item in paired_datasets)
    synthetic_count = sum(item[5] for item in paired_datasets)
    _emit_progress(
        progress,
        f"discovered {real_count} real and {synthetic_count} synthetic paired validation datasets",
    )
    paired_phase_started = time.perf_counter()
    for index, (static_path, motion_path, _num_sc, chip, dataset_id, synthetic,
                _dataset_role, low_rssi, _environment) in enumerate(paired_datasets, start=1):
        dataset_started = time.perf_counter()
        section = "paired_synthetic" if synthetic else "paired"
        static_presence_packets, motion_packets = load_real_data_cached(
            static_path,
            motion_path,
        )
        classic_result = compute_classic_dataset_result(
            static_path,
            motion_path,
            tuple(DEFAULT_SUBCARRIERS),
            None,
        )
        classic_metrics = None
        if classic_result is not None:
            _adaptive_threshold, classic_metrics = classic_result
            paired_results[section]["classic"][chip].append(classic_metrics)

        ml_metrics, _feature_payload = compute_ml_dataset_result(
            static_path,
            motion_path,
            tuple(DEFAULT_SUBCARRIERS),
            None,
            0.5,
        )
        paired_results[section]["ml"][chip].append(ml_metrics)
        if not synthetic:
            if low_rssi:
                if classic_metrics is not None:
                    stress_results["classic"][chip].append(classic_metrics)
                stress_results["ml"][chip].append(ml_metrics)
            else:
                if classic_metrics is not None:
                    normal_results["classic"][chip].append(classic_metrics)
                normal_results["ml"][chip].append(ml_metrics)
        _emit_progress(
            progress,
            (
                f"paired dataset {index}/{len(paired_datasets)} complete: "
                f"{chip} ({'synthetic' if synthetic else 'real'}, {dataset_id}) in "
                f"{_format_progress_duration(time.perf_counter() - dataset_started)}"
            ),
        )

    _emit_progress(
        progress,
        (
            f"paired validation complete: {len(paired_datasets)} datasets in "
            f"{_format_progress_duration(time.perf_counter() - paired_phase_started)}"
        ),
    )

    long_test_datasets = get_available_long_test_datasets()
    _emit_progress(progress, f"discovered {len(long_test_datasets)} long quiet validation datasets")
    long_phase_started = time.perf_counter()
    for index, (test_path, baseline_packets, movement_packets, _motion_start, chip, _entry) in enumerate(
        long_test_datasets,
        start=1,
    ):
        dataset_started = time.perf_counter()
        classic_metrics = evaluate_classic_long_recording(baseline_packets, movement_packets)
        if classic_metrics is not None:
            long_results["classic"][chip].append(classic_metrics)

        ml_metrics = evaluate_ml_long_recording(
            baseline_packets,
            movement_packets,
            source_path=test_path,
            motion_start_packet=_motion_start,
        )
        long_results["ml"][chip].append(ml_metrics)
        _emit_progress(
            progress,
            (
                f"long quiet dataset {index}/{len(long_test_datasets)} complete: "
                f"{chip} ({test_path.stem}) in {_format_progress_duration(time.perf_counter() - dataset_started)}"
            ),
        )

    _emit_progress(
        progress,
        (
            f"long quiet validation complete: {len(long_test_datasets)} datasets in "
            f"{_format_progress_duration(time.perf_counter() - long_phase_started)}"
        ),
    )

    summary_started = time.perf_counter()
    paired_summaries: Dict[str, Dict[str, Dict[str, Dict[str, float]]]] = {}
    for section in ("paired", "paired_synthetic"):
        section_summary: Dict[str, Dict[str, Dict[str, float]]] = {
            "classic": {},
            "ml": {},
        }
        for algorithm, by_chip in paired_results[section].items():
            for chip, entries in by_chip.items():
                averaged = _average_detector_metrics(entries)
                if averaged is not None:
                    section_summary[algorithm][chip] = averaged
        paired_summaries[section] = section_summary

    normal_summary: Dict[str, Dict[str, Dict[str, float]]] = {
        "classic": {},
        "ml": {},
    }
    for algorithm, by_chip in normal_results.items():
        for chip, entries in by_chip.items():
            averaged = _average_detector_metrics(entries)
            if averaged is not None:
                normal_summary[algorithm][chip] = averaged

    stress_summary: Dict[str, Dict[str, Dict[str, float]]] = {"classic": {}, "ml": {}}
    for algorithm, by_chip in stress_results.items():
        for chip, entries in by_chip.items():
            averaged = _average_detector_metrics(entries)
            if averaged is not None:
                stress_summary[algorithm][chip] = averaged

    long_summary: Dict[str, Dict[str, Dict[str, float]]] = {"classic": {}, "ml": {}}
    for algorithm, by_chip in long_results.items():
        for chip, entries in by_chip.items():
            if not entries:
                continue
            recalls = [entry["recall"] for entry in entries]
            fp_rates = [entry["fp_rate"] for entry in entries]
            long_summary[algorithm][chip] = {
                "count": len(entries),
                "min_recall": min(recalls),
                "avg_fp_rate": sum(fp_rates) / len(fp_rates),
                "max_fp_rate": max(fp_rates),
                "effective_alarms": sum(entry["effective_alarms"] for entry in entries),
            }

    _emit_progress(
        progress,
        f"summary aggregation complete in {_format_progress_duration(time.perf_counter() - summary_started)}",
    )
    _emit_progress(progress, "render data ready")
    return {
        "paired": paired_summaries["paired"],
        "paired_normal": normal_summary,
        "paired_stress_real": stress_summary,
        "paired_synthetic": paired_summaries["paired_synthetic"],
        "long_quiet": long_summary,
    }


def render_performance_report_markdown(
    report_data: Dict[str, Dict[str, Dict[str, Dict[str, float]]]],
    execution_info: Optional[ExecutionInfo] = None,
) -> str:
    """Render the published performance markdown from computed metrics."""
    lines = [
        "<!-- Generated file. Do not edit manually. -->",
        "",
        "# Motion Detection Performance",
        "",
    ]

    if execution_info is not None:
        lines.extend([
            f"Last update: {execution_info['last_update']}",
            f"Source: `{execution_info['source']}`",
            f"Evaluation view: `{execution_info.get('evaluation_view', 'HT20/HT-LTF')}`",
            f"Dataset revision: `sha256:{execution_info['dataset_revision']}`",
        ])
        if execution_info.get('input_revision'):
            lines.append(
                f"Input revision: `sha256:{execution_info['input_revision']}`"
            )
        lines.extend([
            f"Generated by: `{execution_info['generated_by']}`",
            f"Run started: `{execution_info['run_started']}`",
            f"Run duration: `{execution_info['run_duration']}`",
            (
                "Inputs: "
                f"`{execution_info['real_paired_dataset_count']}` real paired datasets, "
                f"`{execution_info['synthetic_paired_dataset_count']}` synthetic paired datasets, "
                f"`{execution_info['long_quiet_dataset_count']}` long quiet datasets"
            ),
            "",
        ])

    parity_checked = (
        True
        if execution_info is None
        else bool(execution_info.get("cpp_parity_checked", True))
    )
    algorithms_link = (
        "../ALGORITHMS.md"
        if execution_info is None
        else str(execution_info.get("algorithms_link", "../ALGORITHMS.md"))
    )
    parity_note = (
        "`tools/generate_performance_report.py` also verifies that the host-side "
        "C++ and Python implementations produce matching results when they replay "
        "the same recordings."
        if parity_checked
        else "C++/Python replay parity was not run for this external diagnostic report."
    )
    lines.extend([
        "This report shows how ESPectre's motion detectors perform on recorded data.",
        "",
        parity_note,
        "",
        (
            "- **Lightweight Detection**: Uses feature-fusion coefficients fitted offline on "
            "training recordings and calibrates its threshold for each recording without "
            "refitting those coefficients."
        ),
        (
            "- **High-Accuracy Detection**: Uses the ML neural implementation trained on "
            "recordings marked as motion or no motion. Published replay results use only "
            "recordings outside its training corpus."
        ),
        "",
        f"See [ALGORITHMS.md]({algorithms_link}) for the full detector design.",
        "",
        "---",
        "",
        "## Current Host Resource Benchmark",
        "",
        (
            "The generator compiles the current production C++ sources once per source "
            "revision and executes the benchmark on every report run. Timings and CPU "
            "estimates therefore describe the report host, not an ESP device. Persistent "
            "memory is the detector object's `sizeof` plus live heap allocations after "
            "construction; allocator metadata is not included."
        ),
        "",
    ])

    resources = report_data.get("resources", {})
    resource_detectors = resources.get("detectors", {})
    if resource_detectors:
        lines.extend([
            (
                "Nominal load: "
                f"`{resources.get('packet_rate_hz', 100):g}` packets/s and "
                f"`{resources.get('inference_rate_hz', 4):g}` inferences/s; "
                f"window `{int(resources.get('window_packets', 100))}` packets."
            ),
            "",
            "| Detection profile | Persistent memory | Packet median / p90 | Inference median / p90 | Modeled detector CPU | Transient heap |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ])
        for detector_name, label in (("classic", "Lightweight"), ("ml", "High Accuracy")):
            metrics = resource_detectors.get(detector_name)
            if not metrics:
                continue
            lines.append(
                f"| {label} | {int(metrics['persistent_bytes']):,} B | "
                f"{metrics['packet_median_ns'] / 1000.0:.2f} / "
                f"{metrics['packet_p90_ns'] / 1000.0:.2f} us | "
                f"{metrics['inference_median_ns'] / 1000.0:.2f} / "
                f"{metrics['inference_p90_ns'] / 1000.0:.2f} us | "
                f"{metrics['cpu_us_per_second']:.2f} us/s | "
                f"{int(metrics['transient_heap_bytes']):,} B |"
            )
    else:
        lines.append("Resource benchmark unavailable.")

    lines.extend([
        "",
        "---",
        "",
        "## Required Performance",
        "",
        "| Metric | Target | ",
        "|-------|--------|",
        "| Recall | >95% |",
        "| FP Rate | <5% |",
        "",
        "Recall is the percentage of motion correctly detected. FP Rate is the percentage "
        "of no-motion readings incorrectly reported as motion.",
        "",
        "---",
        "",
        "## Tests Included",
        "",
        "- C++ `test_motion_detection`",
        "- C++ `test_long_recordings`",
        "- Python `TestPerformanceMetrics`",
        "- Python `test_validation_long_recordings.py`",
        "",
        "---",
        "",
        "## Normal Wi-Fi Signal",
        "",
        "Effective Alarms counts how many times the detector incorrectly switches to "
        "MOTION when no motion is present. It uses the same filtering as the firmware.",
        "",
        (
            "These recordings contain periods without motion followed by periods with "
            "motion, captured with a normal Wi-Fi signal. Both detectors are reported "
            "only on the combined `selection + holdout` corpus."
        ),
        "",
        "### Lightweight Detection",
        "",
    ])

    paired = report_data["paired"]
    paired_normal = report_data.get(
        "paired_normal", paired
    )
    paired_stress_real = report_data.get(
        "paired_stress_real", {"classic": {}, "ml": {}}
    )
    paired_header = "| Metric | " + " | ".join(PAIRED_CHIP_LABELS[chip] for chip in CHIP_ORDER) + " |"
    paired_divider = "|--------|" + "|".join("----------" for _ in CHIP_ORDER) + "|"
    paired_row_specs = (
        ("recall", "Recall", lambda value: f"{value:.1f}%"),
        ("min_recall", "Min Recall", lambda value: f"{value:.1f}%"),
        ("precision", "Precision", lambda value: f"{value:.1f}%"),
        ("fp_rate", "FP Rate", lambda value: f"{value:.1f}%"),
        ("max_fp_rate", "Max FP Rate", lambda value: f"{value:.1f}%"),
        ("f1", "F1-Score", lambda value: f"{value:.1f}%"),
        ("effective_alarms", "Effective Alarms", lambda value: f"{int(value)}"),
    )

    def _append_paired_table(section, algorithm):
        lines.append(paired_header)
        lines.append(paired_divider)
        for key, label, formatter in paired_row_specs:
            values = []
            for chip in CHIP_ORDER:
                metrics = section[algorithm].get(chip)
                value = metrics.get(key) if metrics is not None else None
                values.append(formatter(value) if value is not None else "N/A")
            lines.append(f"| {label} | " + " | ".join(values) + " |")

    _append_paired_table(paired_normal, "classic")

    lines.extend([
        "",
        "### High-Accuracy Detection",
        "",
    ])
    _append_paired_table(paired_normal, "ml")

    paired_synthetic = report_data.get(
        "paired_synthetic", {"classic": {}, "ml": {}}
    )
    if any(paired_synthetic.get(algorithm) for algorithm in ("classic", "ml")):
        lines.extend([
            "",
            "---",
            "",
            "## Reconstructed or Synthetic Holdout",
            "",
            (
                "These results are diagnostic because at least one signal view was "
                "reconstructed or otherwise marked synthetic. They are not production "
                "training or PHY-validation evidence."
            ),
            "",
            "### Lightweight Detection",
            "",
        ])
        _append_paired_table(paired_synthetic, "classic")
        lines.extend([
            "",
            "### High-Accuracy Detection",
            "",
        ])
        _append_paired_table(paired_synthetic, "ml")

    lines.extend([
        "",
        "---",
        "",
        "## Reserved Augmentation Diagnostic",
        "",
    ])
    augmentation = report_data.get("augmentation")
    if augmentation:
        lines.extend([
            (
                "Lightweight and High Accuracy are evaluated on the same reserved `selection + holdout` "
                "pairs after deterministic "
                f"`{augmentation.get('recipe', 'unknown')}` packet augmentation. "
                "As in production ML training, the fixed views "
                f"`{', '.join(str(seed) for seed in augmentation.get('seeds', []))}` "
                "contribute alternating row positions. These combined reserved results are "
                "diagnostic, not an uncontaminated promotion gate."
            ),
            "",
            "| Detection profile | Corpus | Recall | FP Rate | F1 |",
            "| --- | --- | ---: | ---: | ---: |",
        ])
        for row in augmentation.get("rows", []):
            detector_name = str(row["detector"]).lower()
            detector = "High Accuracy" if detector_name == "ml" else "Lightweight"
            lines.append(
                f"| {detector} | Reserved, mixed two-seed augmentation "
                f"({int(augmentation.get('pair_count', 0))} pairs) | "
                f"{float(row['recall']):.1f}% | "
                f"{float(row['fp_rate']):.1f}% | {float(row['f1']):.1f}% |"
            )
    else:
        lines.append(
            "Cached augmentation diagnostic unavailable; report generation will not "
            "start a training run to create it."
        )
    lines.extend([
        "",
        "---",
        "",
        "## Weak Wi-Fi Signal",
        "",
        (
            "These reserved `selection + holdout` recordings show detector behavior "
            "when the Wi-Fi signal is weak. The High Accuracy requirements for this test are recall "
            f">{STRESS_TARGET_RECALL:.0f}% and FP <{STRESS_TARGET_FP_RATE:.0f}%; "
            "Lightweight results are included for information only."
        ),
        "",
        "### Lightweight Detection",
        "",
    ])
    _append_paired_table({"classic": paired_stress_real.get("classic", {})}, "classic")

    lines.extend([
        "",
        "### High-Accuracy Detection",
        "",
    ])
    _append_paired_table(paired_stress_real, "ml")

    lines.extend([
        "",
        "---",
        "",
        "## Long Quiet Reserved Recordings",
        "",
        (
            "These `selection + holdout` recordings contain no motion. They show how "
            "often each detector reports motion during quiet periods."
        ),
        "",
        "### Lightweight Detection",
        "",
    ])

    long_quiet = report_data["long_quiet"]
    long_row_specs = (
        ("avg_fp_rate", "Avg FP Rate", lambda value: f"{value:.2f}%"),
        ("max_fp_rate", "Max FP Rate", lambda value: f"{value:.2f}%"),
        ("effective_alarms", "Effective Alarms", lambda value: f"{int(value)}"),
    )

    def _append_long_quiet_table(algorithm):
        long_header = "| Metric | " + " | ".join(CHIP_ORDER) + " |"
        long_divider = "|--------|" + "|".join("----" for _ in CHIP_ORDER) + "|"
        lines.append(long_header)
        lines.append(long_divider)
        for key, label, formatter in long_row_specs:
            values = []
            for chip in CHIP_ORDER:
                metrics = long_quiet[algorithm].get(chip)
                value = metrics.get(key) if metrics is not None else None
                values.append(formatter(value) if value is not None else "N/A")
            lines.append(f"| {label} | " + " | ".join(values) + " |")

    _append_long_quiet_table("classic")

    lines.extend([
        "",
        "### High-Accuracy Detection",
        "",
    ])
    _append_long_quiet_table("ml")

    return "\n".join(lines) + "\n"


def write_performance_report(
    output_path: Optional[Path] = None,
    report_data: Optional[Dict[str, Dict[str, Dict[str, Dict[str, float]]]]] = None,
    progress: Optional[ProgressCallback] = None,
    execution_info: Optional[ExecutionInfo] = None,
) -> Path:
    """Compute and write docs/performance/README.md."""
    destination = PERFORMANCE_DOC_PATH if output_path is None else Path(output_path)
    _emit_progress(progress, f"ensuring output directory exists: {destination.parent}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    _emit_progress(progress, f"writing markdown to {destination}")
    computed_report_data = report_data
    if computed_report_data is None:
        computed_report_data = (
            compute_performance_report_data()
            if progress is None
            else compute_performance_report_data(progress=progress)
        )
    atomic_write_text(
        destination,
        render_performance_report_markdown(
            computed_report_data,
            execution_info=execution_info,
        ),
    )
    _emit_progress(progress, f"report written to {destination}")
    return destination
