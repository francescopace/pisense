# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Shared NPZ Cache

Shared cache helpers for host-side tooling that derives repeated artifacts from
the same dataset NPZ files.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import sys
import threading
import time
import uuid
from collections import OrderedDict
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable, Iterator, Mapping, MutableMapping, Optional

import numpy as np

from .repo_paths import cpp_core_dir, tools_lib_dir, python_src_dir, repo_root

CACHE_LAYOUT_VERSION = 2
CLASSIC_REPLAY_ROW_ARTIFACT_VERSION = 2
ML_REPLAY_ROW_ARTIFACT_VERSION = 5
ML_TRAINING_AUGMENTATION_ROW_ARTIFACT_VERSION = 1
ML_TRAINING_SOURCE_METADATA_ARTIFACT_VERSION = 1
HOST_FEATURE_ROW_SPINE_ARTIFACT_VERSION = 1
HOST_FEATURE_COLUMN_ARTIFACT_VERSION = 1
PERFORMANCE_REPORT_RESULT_ARTIFACT_VERSION = 1

CURRENT_ARTIFACT_VERSIONS = {
    "classic_replay_rows": CLASSIC_REPLAY_ROW_ARTIFACT_VERSION,
    "ml_replay_rows": ML_REPLAY_ROW_ARTIFACT_VERSION,
    "ml_training_augmentation_rows": ML_TRAINING_AUGMENTATION_ROW_ARTIFACT_VERSION,
    "ml_training_source_metadata": ML_TRAINING_SOURCE_METADATA_ARTIFACT_VERSION,
    "host_feature_row_spines": HOST_FEATURE_ROW_SPINE_ARTIFACT_VERSION,
    "host_feature_columns": HOST_FEATURE_COLUMN_ARTIFACT_VERSION,
    "performance_report_results": PERFORMANCE_REPORT_RESULT_ARTIFACT_VERSION,
}
OBSOLETE_ARTIFACT_NAMES = {
    "feature_matrix",
    "feature_column",
    "idle_baseline",
}

FEATURE_INDEXED_ARTIFACT_NAMES = {
    "ml_replay_rows",
    "ml_training_augmentation_rows",
}


def performance_report_result_parameters(
    *, kind: str, inputs: Mapping[str, Any]
) -> dict[str, Any]:
    """Return the stable identity for one report-level cached result."""
    return {
        "artifact_version": PERFORMANCE_REPORT_RESULT_ARTIFACT_VERSION,
        "kind": str(kind),
        "inputs": _json_safe(dict(inputs)),
    }


def load_performance_report_result(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
) -> Optional[dict[str, Any]]:
    """Load one JSON report result from the shared NPZ cache."""
    payload = load_npz_artifact(
        source_path,
        artifact_name="performance_report_results",
        artifact_version=PERFORMANCE_REPORT_RESULT_ARTIFACT_VERSION,
        parameters=parameters,
    )
    if payload is None:
        return None
    try:
        return json.loads(str(np.asarray(payload["payload_json"]).item()))
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


def save_performance_report_result(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
    payload: Mapping[str, Any],
) -> Path:
    """Persist one JSON report result in the shared NPZ cache."""
    return save_npz_artifact(
        source_path,
        artifact_name="performance_report_results",
        artifact_version=PERFORMANCE_REPORT_RESULT_ARTIFACT_VERSION,
        parameters=parameters,
        payload={
            "payload_json": np.asarray(
                json.dumps(_json_safe(dict(payload)), sort_keys=True)
            )
        },
    )

RUNTIME_CACHE_MAX_ENTRIES = 64

_RUNTIME_CACHE: "OrderedDict[tuple[str, str], Any]" = OrderedDict()
_RUNTIME_CACHE_LOCK = threading.RLock()

_SOURCE_DIGEST_CACHE: dict[tuple[str, int, int, int], str] = {}
_SOURCE_DIGEST_LOCK = threading.RLock()


NPZ_CACHE_DIR_ENV = "ESPECTRE_NPZ_CACHE_DIR"
NPZ_CACHE_PROGRESS_ENV = "ESPECTRE_NPZ_CACHE_PROGRESS"
NPZ_CACHE_PROGRESS_INTERVAL_ENV = "ESPECTRE_NPZ_CACHE_PROGRESS_INTERVAL_S"
_DEFAULT_PROGRESS_INTERVAL_S = 10.0


def npz_cache_dir() -> Path:
    """Return the NPZ cache directory.

    Defaults to the workspace-local `.cache/npz`. `ESPECTRE_NPZ_CACHE_DIR`
    redirects it, which keeps tests off the working cache and allows placing
    artifacts on another volume.
    """
    override = os.environ.get(NPZ_CACHE_DIR_ENV)
    if override:
        return Path(override).expanduser()
    return repo_root() / ".cache" / "npz"


def npz_cache_progress_enabled() -> bool:
    """Return whether cache progress should be written to stderr.

    Unset follows stderr's TTY state so pytest stays quiet. ``1`` forces
    progress on; ``0``, ``false``, ``no``, and ``off`` force it off.
    """
    raw = os.environ.get(NPZ_CACHE_PROGRESS_ENV)
    if raw is not None:
        return raw.strip().lower() not in {"", "0", "false", "no", "off"}
    try:
        return sys.stderr.isatty()
    except Exception:
        return False


def npz_cache_progress_interval_s() -> float:
    """Return the heartbeat interval for cache progress, in seconds."""
    raw = os.environ.get(NPZ_CACHE_PROGRESS_INTERVAL_ENV)
    if raw is None or not str(raw).strip():
        return _DEFAULT_PROGRESS_INTERVAL_S
    try:
        return max(0.05, float(raw))
    except ValueError:
        return _DEFAULT_PROGRESS_INTERVAL_S


def _progress_source_label(source_path: str | Path) -> str:
    """Return a short source identity for cache progress lines."""
    return Path(source_path).name


def _format_progress_elapsed(seconds: float) -> str:
    """Render a compact elapsed duration for cache progress lines."""
    total = max(0.0, float(seconds))
    if total < 60.0:
        return f"{total:.1f}s"
    minutes, secs = divmod(total, 60.0)
    if minutes < 60.0:
        return f"{int(minutes)}m {secs:04.1f}s"
    hours, minutes = divmod(int(minutes), 60)
    return f"{hours}h {minutes:02d}m {secs:04.1f}s"


def _payload_row_count(payload: Mapping[str, Any]) -> Optional[int]:
    """Return a cheap row count from a persisted cache payload, if present."""
    for key in ("X", "values", "packet_index"):
        if key not in payload:
            continue
        array = np.asarray(payload[key])
        if array.ndim == 0:
            continue
        return int(array.shape[0])
    return None


def _format_byte_size(size_bytes: int) -> str:
    """Render a compact byte size for cache write progress."""
    mib = float(size_bytes) / (1024.0 * 1024.0)
    if mib >= 0.1:
        return f"{mib:.1f} MiB"
    kib = float(size_bytes) / 1024.0
    if kib >= 1.0:
        return f"{kib:.1f} KiB"
    return f"{int(size_bytes)} B"


class _CacheProgress:
    """Accumulate cache hits/misses/writes and emit throttled stderr heartbeats."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._hits = 0
        self._misses = 0
        self._writes = 0
        self._session_start: Optional[float] = None
        self._last_emit: Optional[float] = None
        self._last_read: Optional[tuple[str, str]] = None
        self._active_builds: list[dict[str, Any]] = []
        self._stop_heartbeat = threading.Event()
        self._heartbeat_thread: Optional[threading.Thread] = None

    def reset(self) -> None:
        """Stop heartbeats and clear counters. Tests use this between cases."""
        thread: Optional[threading.Thread] = None
        with self._lock:
            self._stop_heartbeat.set()
            thread = self._heartbeat_thread
            self._heartbeat_thread = None
            self._active_builds.clear()
            self._hits = 0
            self._misses = 0
            self._writes = 0
            self._session_start = None
            self._last_emit = None
            self._last_read = None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._stop_heartbeat = threading.Event()

    def note_load(
        self,
        source_path: str | Path,
        artifact_name: str,
        *,
        hit: bool,
    ) -> None:
        if not npz_cache_progress_enabled():
            return
        label = _progress_source_label(source_path)
        with self._lock:
            self._ensure_session_locked()
            if hit:
                self._hits += 1
            else:
                self._misses += 1
            self._last_read = (str(artifact_name), label)
            first_event = self._last_emit is None
            if not self._should_emit_locked(force=first_event):
                return
            if first_event:
                kind = "hit" if hit else "miss"
                self._emit_locked(f"{kind} {artifact_name} {label}")
                return
            self._emit_summary_locked()

    def note_write(
        self,
        source_path: str | Path,
        artifact_name: str,
        *,
        artifact_path: Path,
        payload: Mapping[str, Any],
    ) -> None:
        if not npz_cache_progress_enabled():
            return
        label = _progress_source_label(source_path)
        details: list[str] = []
        now = time.monotonic()
        with self._lock:
            self._ensure_session_locked()
            self._writes += 1
            for build in reversed(self._active_builds):
                if (
                    build["artifact_name"] == str(artifact_name)
                    and build["source_label"] == label
                ):
                    details.append(
                        _format_progress_elapsed(now - float(build["started_at"]))
                    )
                    break
            try:
                details.append(_format_byte_size(artifact_path.stat().st_size))
            except OSError:
                pass
            rows = _payload_row_count(payload)
            if rows is not None:
                details.append(f"{rows} row(s)")
            suffix = f" ({', '.join(details)})" if details else ""
            self._emit_locked(f"wrote {artifact_name} {label}{suffix}")

    def begin_build(self, source_path: str | Path, artifact_name: str) -> None:
        if not npz_cache_progress_enabled():
            return
        label = _progress_source_label(source_path)
        with self._lock:
            self._ensure_session_locked()
            self._active_builds.append(
                {
                    "artifact_name": str(artifact_name),
                    "source_label": label,
                    "started_at": time.monotonic(),
                }
            )
            if self._heartbeat_thread is None or not self._heartbeat_thread.is_alive():
                self._stop_heartbeat.clear()
                self._heartbeat_thread = threading.Thread(
                    target=self._run_heartbeat,
                    name="npz-cache-progress",
                    daemon=True,
                )
                self._heartbeat_thread.start()

    def end_build(self, source_path: str | Path, artifact_name: str) -> None:
        label = _progress_source_label(source_path)
        thread: Optional[threading.Thread] = None
        with self._lock:
            for index in range(len(self._active_builds) - 1, -1, -1):
                build = self._active_builds[index]
                if (
                    build["artifact_name"] == str(artifact_name)
                    and build["source_label"] == label
                ):
                    self._active_builds.pop(index)
                    break
            if not self._active_builds:
                self._stop_heartbeat.set()
                thread = self._heartbeat_thread
                self._heartbeat_thread = None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def _ensure_session_locked(self) -> None:
        if self._session_start is None:
            self._session_start = time.monotonic()

    def _should_emit_locked(self, *, force: bool) -> bool:
        if force or self._last_emit is None:
            return True
        return (time.monotonic() - self._last_emit) >= npz_cache_progress_interval_s()

    def _emit_summary_locked(self) -> None:
        parts = [f"{self._hits} hit(s)", f"{self._misses} miss(es)"]
        if self._writes:
            parts.append(f"{self._writes} write(s)")
        if self._last_read is not None:
            parts.append(f"last {self._last_read[0]} {self._last_read[1]}")
        self._emit_locked(", ".join(parts))

    def _emit_locked(self, message: str) -> None:
        started = self._session_start
        elapsed = 0.0 if started is None else time.monotonic() - started
        print(
            f"[npz-cache] {_format_progress_elapsed(elapsed)} {message}",
            file=sys.stderr,
            flush=True,
        )
        self._last_emit = time.monotonic()

    def _run_heartbeat(self) -> None:
        interval = npz_cache_progress_interval_s()
        while not self._stop_heartbeat.wait(interval):
            self._emit_heartbeat()

    def _emit_heartbeat(self) -> None:
        with self._lock:
            if not self._active_builds:
                return
            now = time.monotonic()
            build = self._active_builds[-1]
            held = _format_progress_elapsed(now - float(build["started_at"]))
            self._emit_locked(
                "still building "
                f"{build['artifact_name']} {build['source_label']} ({held})"
            )


_CACHE_PROGRESS = _CacheProgress()


def reset_npz_cache_progress() -> None:
    """Reset cache progress counters and stop any heartbeat thread."""
    _CACHE_PROGRESS.reset()


def _json_safe(value: Any) -> Any:
    """Convert NumPy-heavy nested structures into JSON-safe Python values."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def artifact_dir(artifact_name: str) -> Path:
    """Return the cache directory path for one artifact type without creating it."""
    return npz_cache_dir() / str(artifact_name)


def _safe_relative_path(path: Path) -> str:
    """Return a stable relative path inside the repo when possible."""
    resolved = Path(path).resolve()
    try:
        return str(resolved.relative_to(repo_root()))
    except ValueError:
        return str(resolved)


def _source_digest_memo_enabled() -> bool:
    """Return whether stat metadata can safely key the digest memo."""
    return os.name != "nt"


def source_content_digest(source_path: str | Path) -> str:
    """Return the SHA-256 of one source NPZ, memoized per stat signature.

    Size, modification time, and change time are only a fast path for skipping
    a rehash of an unchanged file; they never reach the manifest. POSIX change
    time catches a same-size rewrite even when modification time is deliberately
    restored. Windows exposes creation time through ``st_ctime_ns``, so it
    always rehashes instead of trusting an unsafe memo key. Full hashing of the
    whole capture corpus costs well under a second, which is the price of an
    identity that survives a checkout.
    """
    resolved = Path(source_path).resolve()
    stat = resolved.stat()
    key = (
        str(resolved),
        int(stat.st_size),
        int(stat.st_mtime_ns),
        int(stat.st_ctime_ns),
    )
    memo_enabled = _source_digest_memo_enabled()
    if memo_enabled:
        with _SOURCE_DIGEST_LOCK:
            cached = _SOURCE_DIGEST_CACHE.get(key)
        if cached is not None:
            return cached
    digest = hashlib.sha256()
    with resolved.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    value = digest.hexdigest()
    if memo_enabled:
        with _SOURCE_DIGEST_LOCK:
            _SOURCE_DIGEST_CACHE[key] = value
    return value


def source_manifest(source_path: str | Path) -> dict[str, Any]:
    """Return the stable cache identity for one source NPZ.

    The identity is content-based and machine-independent. Modification time is
    excluded because `git checkout` rewrites it, and the absolute path is
    excluded because it differs between a workstation and a CI runner; either
    one would make a transported or restored cache miss on every entry.
    """
    resolved = Path(source_path).resolve()
    return {
        "path": _safe_relative_path(resolved),
        "size": int(resolved.stat().st_size),
        "content_sha256": source_content_digest(resolved),
    }


def _ml_feature_source_manifests() -> dict[str, Any]:
    """Return stable identities for the time-aware ML feature extractor."""
    manifests: dict[str, Any] = {}
    sources = {
        "python_config": python_src_dir() / "config.py",
        "python_csi_features": tools_lib_dir() / "csi_features.py",
        "python_device_utils": python_src_dir() / "device_utils.py",
        "python_filters": tools_lib_dir() / "filters.py",
        "python_high_accuracy_detector": tools_lib_dir() / "high_accuracy_detector.py",
        "python_ml_feature_trackers": tools_lib_dir() / "ml_feature_trackers.py",
        "python_runtime_policy": tools_lib_dir() / "runtime_policy.py",
        "python_segmentation": tools_lib_dir() / "segmentation.py",
        "host_csi_io": repo_root() / "tools" / "lib" / "csi_io.py",
        "host_dataset_metadata": repo_root() / "tools" / "lib" / "dataset_metadata.py",
        "host_ml_replay": repo_root() / "tools" / "lib" / "performance_report.py",
    }
    for name, path in sources.items():
        if path.exists():
            manifests[name] = source_manifest(path)
    return manifests


def _replay_policy_source_manifests() -> dict[str, Any]:
    """Return identities for shared replay timing and calibration policy."""
    manifests: dict[str, Any] = {}
    sources = {
        "python_runtime_policy": tools_lib_dir() / "runtime_policy.py",
        "python_temporal_csi_sampler": tools_lib_dir() / "temporal_csi_sampler.py",
        "host_dataset_metadata": repo_root() / "tools" / "lib" / "dataset_metadata.py",
        "host_classic_replay": repo_root() / "tools" / "lib" / "performance_report.py",
        "host_temporal_replay": repo_root() / "tools" / "lib" / "temporal_replay.py",
    }
    for name, path in sources.items():
        if path.exists():
            manifests[name] = source_manifest(path)
    return manifests


def _lightweight_detector_source_manifests() -> dict[str, Any]:
    """Return stable identities for the current Lightweight detector sources."""
    manifests: dict[str, Any] = {}
    sources = {
        "python_lightweight_detector": tools_lib_dir() / "lightweight_detector.py",
        "cpp_lightweight_detector_header": cpp_core_dir() / "lightweight_detector.h",
        "cpp_lightweight_detector_impl": cpp_core_dir() / "lightweight_detector.cpp",
    }
    for name, path in sources.items():
        if path.exists():
            manifests[name] = source_manifest(path)
    return manifests


def resolve_manifest_source(manifest: Mapping[str, Any]) -> Path:
    """Return the current location of one manifest's source capture."""
    stored = Path(str(manifest.get("path", "") or ""))
    return stored if stored.is_absolute() else repo_root() / stored


def artifact_manifest(
    source_path: str | Path,
    *,
    artifact_name: str,
    artifact_version: int,
    parameters: Optional[Mapping[str, Any]] = None,
) -> dict[str, Any]:
    """Build a stable manifest for one derived artifact.

    Parameters are normalized to JSON-safe values so the in-memory manifest and
    its serialized form stay equal. Without this, NumPy scalars fail to digest
    and tuples digest but never match their own round-trip, which turns every
    lookup into a silent miss.
    """
    return {
        "cache_layout_version": CACHE_LAYOUT_VERSION,
        "artifact_name": str(artifact_name),
        "artifact_version": int(artifact_version),
        "source": source_manifest(source_path),
        "parameters": _json_safe(dict(parameters or {})),
    }


def manifest_digest(manifest: Mapping[str, Any]) -> str:
    """Return a stable hex digest for one manifest."""
    payload = json.dumps(
        manifest,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def artifact_cache_path(
    source_path: str | Path,
    *,
    artifact_name: str,
    artifact_version: int,
    parameters: Optional[Mapping[str, Any]] = None,
    extension: str = ".npz",
) -> tuple[dict[str, Any], Path]:
    """Return ``(manifest, cache_path)`` for one derived artifact."""
    manifest = artifact_manifest(
        source_path,
        artifact_name=artifact_name,
        artifact_version=artifact_version,
        parameters=parameters,
    )
    digest = manifest_digest(manifest)
    cache_dir = artifact_dir(str(artifact_name))
    return manifest, cache_dir / f"{digest}{extension}"


def runtime_cache_key(
    source_path: str | Path,
    *,
    artifact_name: str,
    artifact_version: int,
    parameters: Optional[Mapping[str, Any]] = None,
) -> tuple[str, str]:
    """Return the stable in-process memo key for one source artifact."""
    manifest = artifact_manifest(
        source_path,
        artifact_name=artifact_name,
        artifact_version=artifact_version,
        parameters=parameters,
    )
    return str(artifact_name), manifest_digest(manifest)


@contextmanager
def artifact_build_lock(
    source_path: str | Path,
    *,
    artifact_name: str,
    artifact_version: int,
    parameters: Optional[Mapping[str, Any]] = None,
) -> Iterator[None]:
    """Serialize producers of one persisted artifact across host processes.

    Lock files intentionally remain in ``.locks``. Removing a lock pathname
    while another process is waiting on its inode permits a third process to
    create a different lock for the same key and defeats mutual exclusion.
    """
    manifest = artifact_manifest(
        source_path,
        artifact_name=artifact_name,
        artifact_version=artifact_version,
        parameters=parameters,
    )
    lock_directory = artifact_dir(artifact_name) / ".locks"
    lock_directory.mkdir(parents=True, exist_ok=True)
    lock_path = lock_directory / f"{manifest_digest(manifest)}.lock"
    handle = lock_path.open("a+b")
    try:
        try:
            import fcntl
        except ImportError:  # pragma: no cover - Windows has no fcntl
            fcntl = None
        if fcntl is not None:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        _CACHE_PROGRESS.begin_build(source_path, artifact_name)
        try:
            yield
        finally:
            _CACHE_PROGRESS.end_build(source_path, artifact_name)
    finally:
        if fcntl is not None:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def get_runtime_artifact(
    source_path: str | Path,
    *,
    artifact_name: str,
    artifact_version: int,
    builder: Callable[[], Any],
    parameters: Optional[Mapping[str, Any]] = None,
) -> Any:
    """Return one in-process memoized artifact for one NPZ source."""
    key = runtime_cache_key(
        source_path,
        artifact_name=artifact_name,
        artifact_version=artifact_version,
        parameters=parameters,
    )
    with _RUNTIME_CACHE_LOCK:
        cached = _RUNTIME_CACHE.pop(key, None)
        if cached is not None:
            _RUNTIME_CACHE[key] = cached
            return cached
    built = builder()
    with _RUNTIME_CACHE_LOCK:
        _RUNTIME_CACHE[key] = built
        while len(_RUNTIME_CACHE) > RUNTIME_CACHE_MAX_ENTRIES:
            _RUNTIME_CACHE.popitem(last=False)
    return built


def clear_runtime_artifacts(*artifact_names: str) -> None:
    """Drop in-process memoized artifacts, optionally filtered by type."""
    selected = {str(name) for name in artifact_names if name}
    with _RUNTIME_CACHE_LOCK:
        if not selected:
            _RUNTIME_CACHE.clear()
            return
        for key in list(_RUNTIME_CACHE):
            if key[0] in selected:
                _RUNTIME_CACHE.pop(key, None)


def clear_persisted_artifacts(*artifact_names: str) -> None:
    """Delete persisted cache files for the selected artifact types."""
    cache_root = npz_cache_dir()
    if not cache_root.exists():
        return
    selected = [str(name) for name in artifact_names if name]
    if not selected:
        shutil.rmtree(cache_root)
        return
    for artifact_name in selected:
        shutil.rmtree(cache_root / artifact_name, ignore_errors=True)


def read_artifact_manifest(artifact_path: str | Path) -> Optional[dict[str, Any]]:
    """Return one persisted artifact's manifest, or None when unreadable."""
    try:
        with np.load(Path(artifact_path), allow_pickle=False) as data:
            return json.loads(str(np.asarray(data["manifest_json"]).item()))
    except Exception:
        return None


def _nested_source_manifests(value: Any) -> list[Mapping[str, Any]]:
    """Return every source manifest nested inside artifact parameters."""
    found: list[Mapping[str, Any]] = []
    if isinstance(value, Mapping):
        if {"path", "size", "content_sha256"}.issubset(value):
            found.append(value)
        for nested in value.values():
            found.extend(_nested_source_manifests(nested))
    elif isinstance(value, (list, tuple)):
        for nested in value:
            found.extend(_nested_source_manifests(nested))
    return found


def _source_manifest_state(manifest: Mapping[str, Any]) -> str:
    """Return ``current``, ``missing``, or ``stale`` for one source manifest."""
    try:
        source = resolve_manifest_source(manifest)
        if not source.is_file():
            return "missing"
        return "current" if source_manifest(source) == manifest else "stale"
    except (OSError, TypeError, ValueError):
        return "stale"


def _prune_feature_index(artifact_directory: Path) -> int:
    """Remove malformed or orphaned feature-index entries."""
    index_root = artifact_directory / ".feature_index"
    if not index_root.is_dir():
        return 0
    removed = 0
    for entry_path in index_root.glob("*/*.json"):
        try:
            entry = json.loads(entry_path.read_text(encoding="utf-8"))
            artifact_path = artifact_directory / str(
                entry.get("artifact_filename", "")
            )
            indexed_manifest = entry.get("manifest", {})
            artifact_manifest_value = read_artifact_manifest(artifact_path)
            valid = (
                artifact_path.is_file()
                and isinstance(indexed_manifest, Mapping)
                and artifact_manifest_value == indexed_manifest
            )
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            valid = False
        if not valid:
            entry_path.unlink(missing_ok=True)
            removed += 1
    for directory in sorted(
        (path for path in index_root.rglob("*") if path.is_dir()),
        key=lambda path: len(path.parts),
        reverse=True,
    ):
        try:
            directory.rmdir()
        except OSError:
            pass
    try:
        index_root.rmdir()
    except OSError:
        pass
    return removed


def prune_persisted_artifacts(*artifact_names: str) -> dict[str, int]:
    """Delete unreachable persisted artifacts and return what was removed.

    An artifact is unreachable when it is unreadable, when its source capture is
    gone or changed, or when any source dependency recorded in its parameters is
    gone or changed. Feature-index entries are usable only while their complete
    target artifact remains available and unchanged.
    """
    cache_root = npz_cache_dir()
    removed = {
        "unreadable": 0,
        "missing_source": 0,
        "stale_source": 0,
        "obsolete_artifact": 0,
        "obsolete_version": 0,
        "missing_dependency": 0,
        "stale_dependency": 0,
        "orphaned_index": 0,
    }
    if not cache_root.exists():
        return removed
    selected = [str(name) for name in artifact_names if name]
    directories = (
        [cache_root / name for name in selected]
        if selected
        else [entry for entry in cache_root.iterdir() if entry.is_dir()]
    )
    for directory in directories:
        if not directory.is_dir():
            continue
        for artifact_path in sorted(directory.glob("*.npz")):
            if artifact_path.name.endswith(".tmp.npz"):
                continue
            manifest = read_artifact_manifest(artifact_path)
            if manifest is None:
                reason = "unreadable"
            elif str(manifest.get("artifact_name", "")) in OBSOLETE_ARTIFACT_NAMES:
                reason = "obsolete_artifact"
            elif (
                str(manifest.get("artifact_name", "")) in CURRENT_ARTIFACT_VERSIONS
                and (
                    int(manifest.get("cache_layout_version", -1)) != CACHE_LAYOUT_VERSION
                    or int(manifest.get("artifact_version", -1))
                    != CURRENT_ARTIFACT_VERSIONS[str(manifest["artifact_name"])]
                )
            ):
                reason = "obsolete_version"
            else:
                source_state = _source_manifest_state(manifest.get("source", {}))
                if source_state == "missing":
                    reason = "missing_source"
                elif source_state == "stale":
                    reason = "stale_source"
                else:
                    dependency_states = {
                        _source_manifest_state(dependency)
                        for dependency in _nested_source_manifests(
                            manifest.get("parameters", {})
                        )
                    }
                    if "missing" in dependency_states:
                        reason = "missing_dependency"
                    elif "stale" in dependency_states:
                        reason = "stale_dependency"
                    else:
                        continue
            artifact_path.unlink(missing_ok=True)
            removed[reason] += 1
        removed["orphaned_index"] += _prune_feature_index(directory)

    for directory in directories:
        try:
            directory.rmdir()
        except OSError:
            pass
    return removed


def load_npz_artifact(
    source_path: str | Path,
    *,
    artifact_name: str,
    artifact_version: int,
    parameters: Optional[Mapping[str, Any]] = None,
) -> Optional[dict[str, np.ndarray]]:
    """Load one persisted NPZ artifact when the manifest still matches."""
    manifest, artifact_path = artifact_cache_path(
        source_path,
        artifact_name=artifact_name,
        artifact_version=artifact_version,
        parameters=parameters,
    )
    if not artifact_path.exists():
        _CACHE_PROGRESS.note_load(source_path, artifact_name, hit=False)
        return None
    try:
        with np.load(artifact_path, allow_pickle=False) as data:
            cached_manifest = json.loads(str(np.asarray(data["manifest_json"]).item()))
            if cached_manifest != manifest:
                _CACHE_PROGRESS.note_load(source_path, artifact_name, hit=False)
                return None
            payload = {
                key: np.asarray(data[key]) for key in data.files if key != "manifest_json"
            }
    except Exception:
        _CACHE_PROGRESS.note_load(source_path, artifact_name, hit=False)
        return None
    _CACHE_PROGRESS.note_load(source_path, artifact_name, hit=True)
    return payload


def _feature_index_parameters(parameters: Mapping[str, Any]) -> dict[str, Any]:
    """Return the explicitly schema-independent identity of feature rows.

    Only the top-level derived schema and the duplicated host transform schema
    are omitted. Recursively dropping every ``window_size`` used to alias
    unrelated transform parameters that happened to use the same field name.
    """
    normalized = _json_safe(dict(parameters))
    normalized.pop("feature_names", None)
    normalized.pop("window_size", None)
    provenance = normalized.get("stream_provenance")
    if isinstance(provenance, dict) and provenance.get("transform") in {
        "host_feature_rows_v2",
        "training_packet_augmentation_mix_v1",
    }:
        provenance.pop("feature_names", None)
    return normalized


def _feature_index_identity(manifest: Mapping[str, Any]) -> dict[str, Any]:
    """Return the stable lookup identity shared by feature supersets."""
    return {
        "cache_layout_version": manifest.get("cache_layout_version"),
        "artifact_name": manifest.get("artifact_name"),
        "artifact_version": manifest.get("artifact_version"),
        "source": manifest.get("source"),
        "parameters": _feature_index_parameters(
            manifest.get("parameters", {})
        ),
    }


def _feature_index_directory(manifest: Mapping[str, Any]) -> Path:
    """Return the lookup directory shared by compatible feature schemas."""
    artifact_name = str(manifest.get("artifact_name", ""))
    identity = _feature_index_identity(manifest)
    return artifact_dir(artifact_name) / ".feature_index" / manifest_digest(identity)


def _register_feature_artifact(
    manifest: Mapping[str, Any],
    artifact_path: Path,
) -> None:
    """Publish a lightweight pointer that enables feature-superset reuse."""
    artifact_name = str(manifest.get("artifact_name", ""))
    parameters = manifest.get("parameters", {})
    feature_names = [str(name) for name in parameters.get("feature_names", ())]
    provenance = parameters.get("stream_provenance", {})
    transform = str(provenance.get("transform", ""))
    if (
        artifact_name not in FEATURE_INDEXED_ARTIFACT_NAMES
        or not feature_names
        or transform not in {
            "host_feature_rows_v2",
            "training_packet_augmentation_mix_v1",
        }
    ):
        return
    index_directory = _feature_index_directory(manifest)
    index_directory.mkdir(parents=True, exist_ok=True)
    entry_path = index_directory / f"{manifest_digest(manifest)}.json"
    entry = {
        "artifact_filename": artifact_path.name,
        "feature_names": feature_names,
        "manifest": manifest,
    }
    if entry_path.exists():
        return
    tmp_path = entry_path.parent / (
        f"{entry_path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp"
    )
    try:
        tmp_path.write_text(json.dumps(entry, sort_keys=True), encoding="utf-8")
        os.replace(tmp_path, entry_path)
    except BaseException:
        tmp_path.unlink(missing_ok=True)
        raise


def load_feature_superset_artifact(
    source_path: str | Path,
    *,
    artifact_name: str,
    artifact_version: int,
    parameters: Mapping[str, Any],
) -> Optional[dict[str, np.ndarray]]:
    """Load the smallest indexed artifact containing the requested features."""
    requested_manifest = artifact_manifest(
        source_path,
        artifact_name=artifact_name,
        artifact_version=artifact_version,
        parameters=parameters,
    )
    requested_features = [
        str(name) for name in parameters.get("feature_names", ())
    ]
    requested_set = set(requested_features)
    index_directory = _feature_index_directory(requested_manifest)
    if not requested_features or not index_directory.is_dir():
        return None
    candidates: list[tuple[int, Path, Mapping[str, Any]]] = []
    for entry_path in index_directory.glob("*.json"):
        try:
            entry = json.loads(entry_path.read_text(encoding="utf-8"))
            available = [str(name) for name in entry.get("feature_names", ())]
            if requested_set.issubset(available):
                candidates.append((len(available), entry_path, entry))
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            continue
    for _, _, entry in sorted(candidates, key=lambda item: (item[0], str(item[1]))):
        cached_manifest = entry.get("manifest", {})
        if _feature_index_identity(cached_manifest) != _feature_index_identity(
            requested_manifest
        ):
            continue
        artifact_path = artifact_dir(artifact_name) / str(
            entry.get("artifact_filename", "")
        )
        if not artifact_path.is_file():
            continue
        try:
            with np.load(artifact_path, allow_pickle=False) as data:
                stored_manifest = json.loads(
                    str(np.asarray(data["manifest_json"]).item())
                )
                if stored_manifest != cached_manifest:
                    continue
                payload = {
                    key: np.asarray(data[key])
                    for key in data.files
                    if key != "manifest_json"
                }
        except Exception:
            continue
        _CACHE_PROGRESS.note_load(source_path, artifact_name, hit=True)
        return payload
    return None


def save_npz_artifact(
    source_path: str | Path,
    *,
    artifact_name: str,
    artifact_version: int,
    payload: Mapping[str, Any],
    parameters: Optional[Mapping[str, Any]] = None,
) -> Path:
    """Persist one NPZ artifact atomically and return its path."""
    manifest, artifact_path = artifact_cache_path(
        source_path,
        artifact_name=artifact_name,
        artifact_version=artifact_version,
        parameters=parameters,
    )
    artifact_path.parent.mkdir(parents=True, exist_ok=True)
    serializable: MutableMapping[str, np.ndarray] = {
        "manifest_json": np.asarray(json.dumps(manifest, sort_keys=True)),
    }
    for key, value in payload.items():
        serializable[str(key)] = np.asarray(value)
    # Stage under a writer-private name: concurrent producers of the same
    # artifact (pytest workers, or training alongside validation) would
    # otherwise interleave writes into one file and publish a truncated archive.
    tmp_path = artifact_path.parent / f"{artifact_path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp.npz"
    try:
        np.savez(tmp_path, **serializable)
        os.replace(tmp_path, artifact_path)
        _register_feature_artifact(manifest, artifact_path)
    except BaseException:
        tmp_path.unlink(missing_ok=True)
        raise
    _CACHE_PROGRESS.note_write(
        source_path,
        artifact_name,
        artifact_path=artifact_path,
        payload=payload,
    )
    return artifact_path


def load_ml_training_source_metadata_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
) -> Optional[dict[str, Any]]:
    """Load cached metadata needed to admit one ML training source."""
    payload = load_npz_artifact(
        source_path,
        artifact_name="ml_training_source_metadata",
        artifact_version=ML_TRAINING_SOURCE_METADATA_ARTIFACT_VERSION,
        parameters=parameters,
    )
    if payload is None:
        return None
    try:
        return json.loads(str(np.asarray(payload["metadata_json"]).item()))
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


def save_ml_training_source_metadata_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
    metadata: Mapping[str, Any],
) -> Path:
    """Persist metadata needed to admit one ML training source."""
    return save_npz_artifact(
        source_path,
        artifact_name="ml_training_source_metadata",
        artifact_version=ML_TRAINING_SOURCE_METADATA_ARTIFACT_VERSION,
        parameters=parameters,
        payload={
            "metadata_json": np.asarray(
                json.dumps(_json_safe(dict(metadata)), sort_keys=True)
            ),
        },
    )


def load_host_feature_row_spine_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
) -> Optional[dict[str, np.ndarray]]:
    """Load row coordinates shared by independently cached host features."""
    payload = load_npz_artifact(
        source_path,
        artifact_name="host_feature_row_spines",
        artifact_version=HOST_FEATURE_ROW_SPINE_ARTIFACT_VERSION,
        parameters=parameters,
    )
    if payload is None:
        return None
    return {
        "packet_index": np.asarray(payload.get("packet_index", ()), dtype=np.int32),
        "evaluation_index": np.asarray(
            payload.get("evaluation_index", ()), dtype=np.int32
        ),
        "reset_index": np.asarray(payload.get("reset_index", ()), dtype=np.int32),
        "evaluation_due": np.asarray(payload.get("evaluation_due", ()), dtype=bool),
    }


def save_host_feature_row_spine_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
    rows: Mapping[str, Any],
) -> Path:
    """Persist row coordinates independently from feature values."""
    return save_npz_artifact(
        source_path,
        artifact_name="host_feature_row_spines",
        artifact_version=HOST_FEATURE_ROW_SPINE_ARTIFACT_VERSION,
        parameters=parameters,
        payload={
            "packet_index": np.asarray(rows.get("packet_index", ()), dtype=np.int32),
            "evaluation_index": np.asarray(
                rows.get("evaluation_index", ()), dtype=np.int32
            ),
            "reset_index": np.asarray(rows.get("reset_index", ()), dtype=np.int32),
            "evaluation_due": np.asarray(
                rows.get("evaluation_due", ()), dtype=bool
            ),
        },
    )


def load_host_feature_column_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
) -> Optional[np.ndarray]:
    """Load one independently versioned host feature column."""
    payload = load_npz_artifact(
        source_path,
        artifact_name="host_feature_columns",
        artifact_version=HOST_FEATURE_COLUMN_ARTIFACT_VERSION,
        parameters=parameters,
    )
    if payload is None or "values" not in payload:
        return None
    values = np.asarray(payload["values"], dtype=np.float32)
    return values if values.ndim == 1 else None


def save_host_feature_column_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
    values: Any,
) -> Path:
    """Persist one feature column independently from sibling features."""
    column = np.asarray(values, dtype=np.float32)
    if column.ndim != 1:
        raise ValueError("host feature columns must be one-dimensional")
    return save_npz_artifact(
        source_path,
        artifact_name="host_feature_columns",
        artifact_version=HOST_FEATURE_COLUMN_ARTIFACT_VERSION,
        parameters=parameters,
        payload={"values": column},
    )


def classic_replay_row_parameters(
    *,
    replay_kind: str,
    selected_subcarriers: Any,
    timing: Mapping[str, Any],
    replay_interval_us: int,
    warmup_packets: int,
    secondary_source: Optional[str | Path] = None,
    replay_provenance: Optional[Mapping[str, Any]] = None,
) -> dict[str, Any]:
    """Return the identity for one time-aware Lightweight feature-row replay."""
    parameters: dict[str, Any] = {
        "artifact_version": CLASSIC_REPLAY_ROW_ARTIFACT_VERSION,
        "replay_kind": str(replay_kind),
        "selected_subcarriers": [int(sc) for sc in selected_subcarriers],
        "feature_names": ["turb_autocorr", "turb_iqr_over_mean_aggr"],
        "timing": {str(key): int(value) for key, value in timing.items()},
        "replay_interval_us": int(replay_interval_us),
        "warmup_packets": int(warmup_packets),
        "replay_policy_sources": _replay_policy_source_manifests(),
        "classic_sources": _lightweight_detector_source_manifests(),
    }
    if secondary_source is not None:
        parameters["secondary_source"] = source_manifest(secondary_source)
    if replay_provenance is not None:
        parameters["replay_provenance"] = _json_safe(dict(replay_provenance))
    return parameters


def ml_replay_row_parameters(
    *,
    selected_subcarriers: Any,
    window_size: int,
    feature_names: Any,
    stream_provenance: Optional[Mapping[str, Any]] = None,
) -> dict[str, Any]:
    """Return the identity for the canonical time-aware ML row stream.

    The persisted artifact is independent of the consumer's sampling contract:
    replay-tick rows are a projection of the ready-packet stream. Numeric model
    weights are intentionally excluded because feature extraction does not use
    them. Deterministic input transforms, such as packet augmentation, must
    provide their own provenance so they cannot alias the raw capture rows.
    """
    parameters = {
        "artifact_version": ML_REPLAY_ROW_ARTIFACT_VERSION,
        "sample_contract": "time_aware_stream_rows_v1",
        "selected_subcarriers": [int(sc) for sc in selected_subcarriers],
        "window_size": int(window_size),
        "feature_names": [str(name) for name in feature_names],
        "feature_sources": _ml_feature_source_manifests(),
    }
    if stream_provenance is not None:
        parameters["stream_provenance"] = _json_safe(dict(stream_provenance))
    return parameters


def ml_training_augmentation_row_parameters(
    *,
    selected_subcarriers: Any,
    feature_names: Any,
    stream_provenance: Mapping[str, Any],
) -> dict[str, Any]:
    """Return the identity for one deterministic mixed augmentation row set.

    Unlike a single replay stream, a cached mix can contain views with distinct
    effective window sizes. Its identity therefore follows the source capture,
    requested feature schema, feature implementation, and complete mixing
    provenance instead of pretending that the result has one window size.
    """
    return {
        "artifact_version": ML_TRAINING_AUGMENTATION_ROW_ARTIFACT_VERSION,
        "sample_contract": "training_augmentation_row_mix_v1",
        "selected_subcarriers": [int(sc) for sc in selected_subcarriers],
        "feature_names": [str(name) for name in feature_names],
        "feature_sources": _ml_feature_source_manifests(),
        "stream_provenance": _json_safe(dict(stream_provenance)),
    }


def load_ml_training_augmentation_row_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
) -> Optional[dict[str, Any]]:
    """Load one persisted deterministic mixed augmentation row set."""
    manifest, artifact_path = artifact_cache_path(
        source_path,
        artifact_name="ml_training_augmentation_rows",
        artifact_version=ML_TRAINING_AUGMENTATION_ROW_ARTIFACT_VERSION,
        parameters=parameters,
    )
    payload = load_npz_artifact(
        source_path,
        artifact_name="ml_training_augmentation_rows",
        artifact_version=ML_TRAINING_AUGMENTATION_ROW_ARTIFACT_VERSION,
        parameters=parameters,
    )
    if payload is not None:
        _register_feature_artifact(manifest, artifact_path)
    else:
        payload = load_feature_superset_artifact(
            source_path,
            artifact_name="ml_training_augmentation_rows",
            artifact_version=ML_TRAINING_AUGMENTATION_ROW_ARTIFACT_VERSION,
            parameters=parameters,
        )
    if payload is None:
        return None
    payload = _project_feature_payload(payload, parameters.get("feature_names", ()))
    return {
        "X": np.asarray(payload.get("X", np.empty((0, 0))), dtype=np.float32),
        "feature_names": np.asarray(
            payload.get("feature_names", np.empty(0))
        ).astype(str).tolist(),
        "packet_index": np.asarray(
            payload.get("packet_index", np.empty(0)), dtype=np.int32
        ),
        "evaluation_index": np.asarray(
            payload.get("evaluation_index", np.empty(0)), dtype=np.int32
        ),
        "reset_index": np.asarray(
            payload.get("reset_index", np.empty(0)), dtype=np.int32
        ),
        "evaluation_due": np.asarray(
            payload.get("evaluation_due", np.empty(0)), dtype=bool
        ),
    }


def save_ml_training_augmentation_row_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
    rows: Mapping[str, Any],
) -> Path:
    """Persist one deterministic mixed augmentation row set."""
    return save_npz_artifact(
        source_path,
        artifact_name="ml_training_augmentation_rows",
        artifact_version=ML_TRAINING_AUGMENTATION_ROW_ARTIFACT_VERSION,
        parameters=parameters,
        payload={
            "X": np.asarray(rows.get("X", np.empty((0, 0))), dtype=np.float32),
            "feature_names": np.asarray(
                [str(name) for name in rows.get("feature_names", ())]
            ),
            "packet_index": np.asarray(
                rows.get("packet_index", np.empty(0)), dtype=np.int32
            ),
            "evaluation_index": np.asarray(
                rows.get("evaluation_index", np.empty(0)), dtype=np.int32
            ),
            "reset_index": np.asarray(
                rows.get("reset_index", np.empty(0)), dtype=np.int32
            ),
            "evaluation_due": np.asarray(
                rows.get("evaluation_due", np.empty(0)), dtype=bool
            ),
        },
    )


def load_ml_replay_row_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
) -> Optional[dict[str, Any]]:
    """Load one persisted ML replay-row artifact."""
    manifest, artifact_path = artifact_cache_path(
        source_path,
        artifact_name="ml_replay_rows",
        artifact_version=ML_REPLAY_ROW_ARTIFACT_VERSION,
        parameters=parameters,
    )
    payload = load_npz_artifact(
        source_path,
        artifact_name="ml_replay_rows",
        artifact_version=ML_REPLAY_ROW_ARTIFACT_VERSION,
        parameters=parameters,
    )
    if payload is not None:
        _register_feature_artifact(manifest, artifact_path)
    else:
        payload = load_feature_superset_artifact(
            source_path,
            artifact_name="ml_replay_rows",
            artifact_version=ML_REPLAY_ROW_ARTIFACT_VERSION,
            parameters=parameters,
        )
    if payload is None:
        return None
    payload = _project_feature_payload(payload, parameters.get("feature_names", ()))
    packet_index = np.asarray(
        payload.get("packet_index", np.empty(0)), dtype=np.int32
    )
    slot_index = np.asarray(
        payload.get("slot_index", np.empty(0)), dtype=np.int64
    )
    if len(slot_index) != len(packet_index):
        slot_index = packet_index.astype(np.int64, copy=True)
    return {
        "X": np.asarray(payload.get("X", np.empty((0, 0))), dtype=np.float32),
        "feature_names": np.asarray(
            payload.get("feature_names", np.empty(0))
        ).astype(str).tolist(),
        "packet_index": packet_index,
        "evaluation_index": np.asarray(
            payload.get("evaluation_index", np.empty(0)), dtype=np.int32
        ),
        "reset_index": np.asarray(
            payload.get("reset_index", np.empty(0)), dtype=np.int32
        ),
        "slot_index": slot_index,
        "target_pps": int(np.asarray(payload.get("target_pps", 0)).item()),
        "evaluation_due": np.asarray(
            payload.get("evaluation_due", np.empty(0)), dtype=bool
        ),
    }


def _project_feature_payload(
    payload: Mapping[str, Any],
    feature_names: Any,
) -> dict[str, Any]:
    """Project a cached feature superset while preserving row metadata."""
    requested = [str(name) for name in feature_names]
    available = np.asarray(payload.get("feature_names", np.empty(0))).astype(str).tolist()
    if not requested or requested == available:
        return dict(payload)
    if not set(requested).issubset(available):
        return dict(payload)
    indices = [available.index(name) for name in requested]
    projected = dict(payload)
    projected["X"] = np.asarray(payload.get("X", np.empty((0, 0))))[:, indices]
    projected["feature_names"] = np.asarray(requested)
    return projected


def save_ml_replay_row_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
    X: np.ndarray,
    feature_names: Any,
    packet_index: np.ndarray,
    evaluation_index: np.ndarray,
    reset_index: np.ndarray,
    evaluation_due: np.ndarray,
    slot_index: Any = None,
    target_pps: Any = 0,
) -> Path:
    """Persist one canonical ML replay-row artifact."""
    return save_npz_artifact(
        source_path,
        artifact_name="ml_replay_rows",
        artifact_version=ML_REPLAY_ROW_ARTIFACT_VERSION,
        parameters=parameters,
        payload={
            "X": np.asarray(X, dtype=np.float32),
            "feature_names": np.asarray([str(name) for name in feature_names]),
            "packet_index": np.asarray(packet_index, dtype=np.int32),
            "evaluation_index": np.asarray(evaluation_index, dtype=np.int32),
            "reset_index": np.asarray(reset_index, dtype=np.int32),
            "slot_index": np.asarray(
                np.empty(0) if slot_index is None else slot_index,
                dtype=np.int64,
            ),
            "target_pps": np.asarray(int(target_pps)),
            "evaluation_due": np.asarray(evaluation_due, dtype=bool),
        },
    )


def load_classic_replay_row_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
) -> Optional[dict[str, Any]]:
    """Load one persisted time-aware Lightweight replay-row artifact."""
    payload = load_npz_artifact(
        source_path,
        artifact_name="classic_replay_rows",
        artifact_version=CLASSIC_REPLAY_ROW_ARTIFACT_VERSION,
        parameters=parameters,
    )
    if payload is None:
        return None
    rows: dict[str, Any] = {}
    for phase in ("calibration", "static", "motion"):
        prefix = f"{phase}_"
        rows[phase] = {
            "X": np.asarray(
                payload.get(prefix + "X", np.empty((0, 2))), dtype=np.float64
            ),
            "ready": np.asarray(
                payload.get(prefix + "ready", np.empty(0)), dtype=bool
            ),
            "eligible": np.asarray(
                payload.get(prefix + "eligible", np.empty(0)), dtype=bool
            ),
            "packet_index": np.asarray(
                payload.get(prefix + "packet_index", np.empty(0)), dtype=np.int32
            ),
            "packet_weight": np.asarray(
                payload.get(prefix + "packet_weight", np.empty(0)), dtype=np.int32
            ),
            "reset_index": np.asarray(
                payload.get(prefix + "reset_index", np.empty(0)), dtype=np.int32
            ),
        }
    return rows


def save_classic_replay_row_artifact(
    source_path: str | Path,
    *,
    parameters: Mapping[str, Any],
    rows: Mapping[str, Mapping[str, Any]],
) -> Path:
    """Persist one canonical time-aware Lightweight replay-row artifact."""
    payload: dict[str, np.ndarray] = {}
    for phase in ("calibration", "static", "motion"):
        phase_rows = rows.get(phase, {})
        prefix = f"{phase}_"
        payload[prefix + "X"] = np.asarray(
            phase_rows.get("X", np.empty((0, 2))), dtype=np.float64
        )
        payload[prefix + "ready"] = np.asarray(
            phase_rows.get("ready", np.empty(0)), dtype=bool
        )
        payload[prefix + "eligible"] = np.asarray(
            phase_rows.get("eligible", np.empty(0)), dtype=bool
        )
        payload[prefix + "packet_index"] = np.asarray(
            phase_rows.get("packet_index", np.empty(0)), dtype=np.int32
        )
        payload[prefix + "packet_weight"] = np.asarray(
            phase_rows.get("packet_weight", np.empty(0)), dtype=np.int32
        )
        payload[prefix + "reset_index"] = np.asarray(
            phase_rows.get("reset_index", np.empty(0)), dtype=np.int32
        )
    return save_npz_artifact(
        source_path,
        artifact_name="classic_replay_rows",
        artifact_version=CLASSIC_REPLAY_ROW_ARTIFACT_VERSION,
        parameters=parameters,
        payload=payload,
    )
