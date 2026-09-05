# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
import os
import time

import numpy as np
import pytest

from tools.lib import npz_cache
from tools.lib import csi_io


@pytest.fixture(autouse=True)
def isolated_cache_root(tmp_path, monkeypatch):
    """Keep every cache test off the shared workspace cache.

    These tests clear whole artifact trees, so without redirection they delete
    the developer's working cache and force an unrelated cold rebuild.
    """
    root = tmp_path / "npz_cache_root"
    monkeypatch.setenv(npz_cache.NPZ_CACHE_DIR_ENV, str(root))
    monkeypatch.setenv(npz_cache.NPZ_CACHE_PROGRESS_ENV, "0")
    npz_cache.clear_runtime_artifacts()
    npz_cache._SOURCE_DIGEST_CACHE.clear()
    npz_cache.reset_npz_cache_progress()
    yield root
    npz_cache.reset_npz_cache_progress()
    npz_cache.clear_runtime_artifacts()
    npz_cache._SOURCE_DIGEST_CACHE.clear()


def _write_source_npz(path, *, values):
    np.savez(path, csi_data=np.asarray([values], dtype=np.int8))


def test_default_cache_root_is_inside_workspace_cache(tmp_path, monkeypatch):
    monkeypatch.delenv(npz_cache.NPZ_CACHE_DIR_ENV, raising=False)
    monkeypatch.setattr(npz_cache, "repo_root", lambda: tmp_path)
    assert npz_cache.npz_cache_dir() == tmp_path / ".cache" / "npz"


def test_cache_root_follows_the_environment_override(isolated_cache_root):
    assert npz_cache.npz_cache_dir() == isolated_cache_root


def test_persisted_cache_miss_does_not_create_directories(
    isolated_cache_root, tmp_path
):
    source_path = tmp_path / "read_only_miss.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])

    cached = npz_cache.load_npz_artifact(
        source_path,
        artifact_name="unit_read_only",
        artifact_version=1,
    )

    assert cached is None
    assert not isolated_cache_root.exists()


def test_ml_replay_cache_projects_an_indexed_feature_superset(tmp_path):
    source_path = tmp_path / "feature_superset.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    superset_parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=[1, 2],
        window_size=100,
        feature_names=["f0", "f1", "f2"],
        stream_provenance={
            "transform": "host_feature_rows_v2",
            "feature_names": ["f0", "f1", "f2"],
            "implementation": {"digest": "stable"},
        },
    )
    npz_cache.save_ml_replay_row_artifact(
        source_path,
        parameters=superset_parameters,
        X=np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32),
        feature_names=["f0", "f1", "f2"],
        packet_index=np.asarray([10, 20], dtype=np.int32),
        evaluation_index=np.asarray([0, 1], dtype=np.int32),
        reset_index=np.asarray([0, 0], dtype=np.int32),
        slot_index=np.asarray([100, 110], dtype=np.int64),
        target_pps=100,
        evaluation_due=np.asarray([True, False]),
    )
    subset_parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=[1, 2],
        window_size=0,
        feature_names=["f2", "f0"],
        stream_provenance={
            "transform": "host_feature_rows_v2",
            "feature_names": ["f2", "f0"],
            "implementation": {"digest": "stable"},
        },
    )

    cached = npz_cache.load_ml_replay_row_artifact(
        source_path,
        parameters=subset_parameters,
    )

    assert cached is not None
    assert cached["feature_names"] == ["f2", "f0"]
    np.testing.assert_array_equal(
        cached["X"],
        np.asarray([[3.0, 1.0], [6.0, 4.0]], dtype=np.float32),
    )
    np.testing.assert_array_equal(cached["packet_index"], [10, 20])
    np.testing.assert_array_equal(cached["slot_index"], [100, 110])
    assert cached["target_pps"] == 100


def test_feature_superset_index_keeps_stream_provenance_isolated(tmp_path):
    source_path = tmp_path / "isolated_streams.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=[1, 2],
        window_size=100,
        feature_names=["f0", "f1"],
        stream_provenance={
            "transform": "host_feature_rows_v2",
            "feature_names": ["f0", "f1"],
            "packet_augmentation": {"seed": 123},
        },
    )
    npz_cache.save_ml_replay_row_artifact(
        source_path,
        parameters=parameters,
        X=np.asarray([[1.0, 2.0]], dtype=np.float32),
        feature_names=["f0", "f1"],
        packet_index=np.asarray([10], dtype=np.int32),
        evaluation_index=np.asarray([0], dtype=np.int32),
        reset_index=np.asarray([0], dtype=np.int32),
        evaluation_due=np.asarray([True]),
    )
    incompatible = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=[1, 2],
        window_size=0,
        feature_names=["f0"],
        stream_provenance={
            "transform": "host_feature_rows_v2",
            "feature_names": ["f0"],
            "packet_augmentation": {"seed": 124},
        },
    )

    assert npz_cache.load_ml_replay_row_artifact(
        source_path,
        parameters=incompatible,
    ) is None


def test_feature_index_keeps_nested_window_parameters_distinct():
    first = {
        "feature_names": ["f0"],
        "window_size": 100,
        "stream_provenance": {
            "transform": "host_feature_rows_v2",
            "feature_names": ["f0"],
            "provider": {"window_size": 32},
        },
    }
    second = {
        **first,
        "stream_provenance": {
            **first["stream_provenance"],
            "provider": {"window_size": 64},
        },
    }

    assert npz_cache._feature_index_parameters(first) != (
        npz_cache._feature_index_parameters(second)
    )


def test_runtime_artifact_reuses_value_until_source_changes(tmp_path):
    source_path = tmp_path / "sample.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])

    calls = {"count": 0}

    def build():
        calls["count"] += 1
        return {"call": calls["count"]}

    first = npz_cache.get_runtime_artifact(
        source_path,
        artifact_name="unit_runtime",
        artifact_version=1,
        builder=build,
    )
    second = npz_cache.get_runtime_artifact(
        source_path,
        artifact_name="unit_runtime",
        artifact_version=1,
        builder=build,
    )

    assert first is second
    assert calls["count"] == 1

    _write_source_npz(source_path, values=[1, 2, 3, 4, 5, 6])

    third = npz_cache.get_runtime_artifact(
        source_path,
        artifact_name="unit_runtime",
        artifact_version=1,
        builder=build,
    )

    assert third is not first
    assert calls["count"] == 2


def test_runtime_cache_evicts_oldest_entries_when_capacity_is_exceeded(tmp_path):
    calls = {"count": 0}
    source_paths = []
    for index in range(npz_cache.RUNTIME_CACHE_MAX_ENTRIES + 1):
        source_path = tmp_path / f"runtime_{index}.npz"
        _write_source_npz(source_path, values=[index, index + 1])
        source_paths.append(source_path)
        npz_cache.get_runtime_artifact(
            source_path,
            artifact_name="unit_runtime",
            artifact_version=1,
            builder=lambda index=index: {"call": index},
        )

    first_source = source_paths[0]

    def rebuild():
        calls["count"] += 1
        return {"call": "rebuilt"}

    rebuilt = npz_cache.get_runtime_artifact(
        first_source,
        artifact_name="unit_runtime",
        artifact_version=1,
        builder=rebuild,
    )

    assert rebuilt == {"call": "rebuilt"}
    assert calls["count"] == 1


def test_packet_view_cache_does_not_pin_raw_arrays(tmp_path):
    source_path = tmp_path / "packet_only.npz"
    np.savez(
        source_path,
        csi_data=np.zeros((1, 128), dtype=np.int8),
        num_subcarriers=np.array(64),
        label=np.array("motion"),
        chip=np.array("c3"),
    )

    packets = csi_io.load_npz_packet_view(source_path)

    assert len(packets) == 1
    assert all(key[0] != "raw_arrays" for key in npz_cache._RUNTIME_CACHE)


def test_packet_csi_data_accepts_read_only_packet_views(tmp_path):
    from tools.lib.ml_training import feature_cache

    source_path = tmp_path / "packet_view_mapping.npz"
    np.savez(
        source_path,
        csi_data=np.arange(128, dtype=np.int8).reshape(1, 128),
        num_subcarriers=np.array(64),
        label=np.array("motion"),
        chip=np.array("c3"),
    )

    packet_view = csi_io.load_npz_packet_view(source_path)

    csi_data = feature_cache.packet_csi_data(packet_view[0])

    assert isinstance(csi_data, np.ndarray)
    np.testing.assert_array_equal(csi_data, packet_view[0]["csi_data"])


def test_compare_detection_methods_accepts_read_only_packet_views(tmp_path):
    import tools.compare_detection_methods as compare_methods

    source_path = tmp_path / "compare_packet_view_mapping.npz"
    np.savez(
        source_path,
        csi_data=np.arange(128, dtype=np.int8).reshape(1, 128),
        num_subcarriers=np.array(64),
        label=np.array("motion"),
        chip=np.array("c3"),
    )

    packet_view = csi_io.load_npz_packet_view(source_path)

    csi_data = compare_methods._packet_csi_data(packet_view[0])

    assert isinstance(csi_data, np.ndarray)
    np.testing.assert_array_equal(csi_data, packet_view[0]["csi_data"])


def test_compare_detection_methods_consumes_selected_packet_and_flushes_final_slot():
    import tools.compare_detection_methods as compare_methods

    packets = [
        {
            "csi_data": np.full(128, value, dtype=np.int8),
            "wifi_rx_ts_us": timestamp_us,
            "csi_target_pps": 10,
        }
        for value, timestamp_us in ((1, 0), (2, 40_000), (3, 100_000))
    ]
    adapter = compare_methods.LightweightDetectorAdapter(packets)
    consumed = []
    adapter._detector.process_packet = (
        lambda csi_data, *_args, **_kwargs: consumed.append(int(csi_data[0]))
    )

    for packet in packets:
        adapter.process_packet(packet)
    assert consumed == [1]

    adapter.finish()
    assert consumed == [1, 3]


def test_source_identity_survives_a_modification_time_rewrite(tmp_path):
    """A checkout rewrites mtime without changing content; the cache must hit."""
    source_path = tmp_path / "checkout.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])

    params = {"contract": "time-aware"}
    npz_cache.save_npz_artifact(
        source_path,
        artifact_name="unit_mtime",
        artifact_version=1,
        parameters=params,
        payload={"value": np.asarray([1.0], dtype=np.float32)},
    )

    stat = source_path.stat()
    os.utime(source_path, ns=(stat.st_atime_ns, stat.st_mtime_ns - 5_000_000_000))

    assert npz_cache.load_npz_artifact(
        source_path,
        artifact_name="unit_mtime",
        artifact_version=1,
        parameters=params,
    ) is not None


def test_source_digest_detects_same_size_rewrite_with_restored_mtime(tmp_path):
    source_path = tmp_path / "same_stat_rewrite.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    original_stat = source_path.stat()
    original_digest = npz_cache.source_content_digest(source_path)

    _write_source_npz(source_path, values=[4, 3, 2, 1])
    os.utime(
        source_path,
        ns=(original_stat.st_atime_ns, original_stat.st_mtime_ns),
    )
    rewritten_stat = source_path.stat()

    assert rewritten_stat.st_size == original_stat.st_size
    assert rewritten_stat.st_mtime_ns == original_stat.st_mtime_ns
    assert rewritten_stat.st_ctime_ns != original_stat.st_ctime_ns
    assert npz_cache.source_content_digest(source_path) != original_digest


def test_source_digest_bypasses_unsafe_memo_metadata_on_windows(
    monkeypatch, tmp_path
):
    source_path = tmp_path / "windows_rehash.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    original_digest = npz_cache.source_content_digest(source_path)

    _write_source_npz(source_path, values=[4, 3, 2, 1])
    stat = source_path.stat()
    current_key = (
        str(source_path.resolve()),
        int(stat.st_size),
        int(stat.st_mtime_ns),
        int(stat.st_ctime_ns),
    )
    npz_cache._SOURCE_DIGEST_CACHE[current_key] = original_digest
    monkeypatch.setattr(npz_cache, "_source_digest_memo_enabled", lambda: False)

    assert npz_cache.source_content_digest(source_path) != original_digest


def test_artifact_parameters_accept_numpy_and_tuple_values(tmp_path):
    """Parameters must round-trip, or every lookup becomes a silent miss."""
    source_path = tmp_path / "params.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])

    params = {
        "threshold": np.float32(0.5),
        "band": (1, 2, 3),
        "count": np.int64(7),
        "flag": np.bool_(True),
    }
    npz_cache.save_npz_artifact(
        source_path,
        artifact_name="unit_params",
        artifact_version=1,
        parameters=params,
        payload={"v": np.arange(3)},
    )

    assert npz_cache.load_npz_artifact(
        source_path,
        artifact_name="unit_params",
        artifact_version=1,
        parameters=params,
    ) is not None


def test_trainer_and_validator_address_the_same_time_aware_artifact(
    monkeypatch, tmp_path
):
    """Training and quality validation must share the canonical row artifact."""
    from tools.lib.dataset_quality import metrics, severity

    feature_names = tuple(severity.VALIDATION_FEATURE_NAMES)
    calls = []

    def fake_load_rows(_source_path, **kwargs):
        calls.append(kwargs)
        return {
            "X": np.empty((0, len(feature_names)), dtype=np.float32),
            "feature_names": list(feature_names),
        }

    monkeypatch.setattr(
        metrics,
        "load_or_compute_ml_replay_rows",
        fake_load_rows,
    )
    metrics._load_or_compute_validation_feature_matrix(
        tmp_path / "shared.npz",
        feature_names=feature_names,
    )

    assert len(calls) == 1
    assert calls[0]["window_size"] is None
    resolved_window_size = 100
    validator_parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=calls[0]["selected_subcarriers"],
        window_size=resolved_window_size,
        feature_names=calls[0]["feature_names"],
    )
    expected_parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=metrics.DEFAULT_SUBCARRIERS,
        window_size=resolved_window_size,
        feature_names=feature_names,
    )
    assert validator_parameters == expected_parameters

    source_path = tmp_path / "shared.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    _, validator_path = npz_cache.artifact_cache_path(
        source_path,
        artifact_name="ml_replay_rows",
        artifact_version=npz_cache.ML_REPLAY_ROW_ARTIFACT_VERSION,
        parameters=validator_parameters,
    )
    _, trainer_path = npz_cache.artifact_cache_path(
        source_path,
        artifact_name="ml_replay_rows",
        artifact_version=npz_cache.ML_REPLAY_ROW_ARTIFACT_VERSION,
        parameters=expected_parameters,
    )

    assert validator_path == trainer_path


def test_clear_persisted_artifacts_removes_selected_artifact_tree(tmp_path):
    source_path = tmp_path / "baseline.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])

    npz_cache.save_npz_artifact(
        source_path,
        artifact_name="unit_clear",
        artifact_version=1,
        parameters={"kind": "baseline"},
        payload={"score": np.asarray(95.0)},
    )

    artifact_root = npz_cache.artifact_dir("unit_clear")
    assert artifact_root.exists()

    npz_cache.clear_persisted_artifacts("unit_clear")

    assert not artifact_root.exists()


def test_prune_removes_only_unreachable_artifacts(tmp_path):
    live_source = tmp_path / "live.npz"
    dead_source = tmp_path / "dead.npz"
    _write_source_npz(live_source, values=[1, 2, 3, 4])
    _write_source_npz(dead_source, values=[5, 6, 7, 8])

    params = {"feature_names": ["f0"]}
    live_path = npz_cache.save_npz_artifact(
        live_source,
        artifact_name="unit_prune",
        artifact_version=1,
        parameters=params,
        payload={"score": np.asarray(1.0)},
    )
    dead_path = npz_cache.save_npz_artifact(
        dead_source,
        artifact_name="unit_prune",
        artifact_version=1,
        parameters=params,
        payload={"score": np.asarray(2.0)},
    )
    dead_source.unlink()

    removed = npz_cache.prune_persisted_artifacts()

    assert removed["missing_source"] == 1
    assert not dead_path.exists()
    assert live_path.exists()
    assert npz_cache.load_npz_artifact(
        live_source,
        artifact_name="unit_prune",
        artifact_version=1,
        parameters=params,
    ) is not None


def test_prune_removes_obsolete_known_artifact_versions(tmp_path):
    source_path = tmp_path / "obsolete.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    artifact_path = npz_cache.save_npz_artifact(
        source_path,
        artifact_name="ml_replay_rows",
        artifact_version=npz_cache.ML_REPLAY_ROW_ARTIFACT_VERSION - 1,
        parameters={"sample_contract": "old"},
        payload={"X": np.empty((0, 0), dtype=np.float32)},
    )

    removed = npz_cache.prune_persisted_artifacts("ml_replay_rows")

    assert removed["obsolete_version"] == 1
    assert not artifact_path.exists()


@pytest.mark.parametrize(
    ("mutation", "expected_reason"),
    (("rewrite", "stale_dependency"), ("delete", "missing_dependency")),
)
def test_prune_removes_artifacts_with_unusable_nested_dependencies(
    tmp_path, mutation, expected_reason
):
    source_path = tmp_path / "capture.npz"
    dependency_path = tmp_path / "extractor.py"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    dependency_path.write_text("VERSION = 1\n", encoding="utf-8")
    artifact_path = npz_cache.save_npz_artifact(
        source_path,
        artifact_name="unit_dependency",
        artifact_version=1,
        parameters={
            "implementation": {
                "sources": [npz_cache.source_manifest(dependency_path)]
            }
        },
        payload={"value": np.arange(4)},
    )
    if mutation == "rewrite":
        dependency_path.write_text("VERSION = 2\n", encoding="utf-8")
    else:
        dependency_path.unlink()

    removed = npz_cache.prune_persisted_artifacts("unit_dependency")

    assert removed[expected_reason] == 1
    assert not artifact_path.exists()


def test_prune_keeps_artifacts_with_current_nested_dependencies(tmp_path):
    source_path = tmp_path / "capture.npz"
    dependency_path = tmp_path / "extractor.py"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    dependency_path.write_text("VERSION = 1\n", encoding="utf-8")
    artifact_path = npz_cache.save_npz_artifact(
        source_path,
        artifact_name="unit_dependency",
        artifact_version=1,
        parameters={"source": npz_cache.source_manifest(dependency_path)},
        payload={"value": np.arange(4)},
    )

    removed = npz_cache.prune_persisted_artifacts("unit_dependency")

    assert sum(removed.values()) == 0
    assert artifact_path.exists()


def test_prune_removes_orphaned_feature_index_entries(tmp_path):
    source_path = tmp_path / "indexed.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=[1, 2],
        window_size=100,
        feature_names=["f0", "f1"],
        stream_provenance={
            "transform": "host_feature_rows_v2",
            "feature_names": ["f0", "f1"],
        },
    )
    artifact_path = npz_cache.save_ml_replay_row_artifact(
        source_path,
        parameters=parameters,
        X=np.asarray([[1.0, 2.0]], dtype=np.float32),
        feature_names=["f0", "f1"],
        packet_index=np.asarray([10], dtype=np.int32),
        evaluation_index=np.asarray([0], dtype=np.int32),
        reset_index=np.asarray([0], dtype=np.int32),
        evaluation_due=np.asarray([True]),
    )
    index_entries = list(
        (npz_cache.artifact_dir("ml_replay_rows") / ".feature_index").glob(
            "*/*.json"
        )
    )
    assert len(index_entries) == 1
    artifact_path.unlink()

    removed = npz_cache.prune_persisted_artifacts("ml_replay_rows")

    assert removed["orphaned_index"] == 1
    assert not index_entries[0].exists()


@pytest.mark.parametrize("artifact_name", ("feature_column", "idle_baseline"))
def test_prune_removes_retired_artifacts(tmp_path, artifact_name):
    source_path = tmp_path / "legacy_dense.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    artifact_path = npz_cache.save_npz_artifact(
        source_path,
        artifact_name=artifact_name,
        artifact_version=1,
        parameters={"feature_name": "f0"},
        payload={"column": np.asarray([1.0], dtype=np.float32)},
    )

    removed = npz_cache.prune_persisted_artifacts()

    assert removed["obsolete_artifact"] == 1
    assert not artifact_path.exists()


def test_prune_tool_removes_selected_obsolete_artifacts(tmp_path, capsys):
    from tools import prune_npz_cache

    source_path = tmp_path / "retired_summary.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    artifact_path = npz_cache.save_npz_artifact(
        source_path,
        artifact_name="idle_baseline",
        artifact_version=1,
        payload={"score": np.asarray(95.0)},
    )

    assert prune_npz_cache.main(["--artifact", "idle_baseline"]) == 0

    output = capsys.readouterr()
    assert "obsolete_artifact=1" in output.out
    assert "Total: 1 artifact(s)" in output.out
    assert not artifact_path.exists()
    assert not npz_cache.artifact_dir("idle_baseline").exists()


def test_ml_replay_parameters_expose_version_to_derived_caches():
    parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=(1, 2, 3),
        window_size=64,
        feature_names=("f0",),
    )

    assert parameters["artifact_version"] == npz_cache.ML_REPLAY_ROW_ARTIFACT_VERSION


def test_classic_replay_row_artifact_roundtrip_preserves_secondary_source_identity(
    tmp_path,
):
    static_source = tmp_path / "static.npz"
    motion_source = tmp_path / "motion.npz"
    _write_source_npz(static_source, values=[1, 2, 3, 4])
    _write_source_npz(motion_source, values=[5, 6, 7, 8])

    parameters = npz_cache.classic_replay_row_parameters(
        replay_kind="classic_dataset",
        selected_subcarriers=(1, 2, 3),
        timing={
            "interval_us": 10_000,
            "window_packets": 4,
            "lag": 1,
            "autocorr_lag": 1,
        },
        replay_interval_us=10_000,
        warmup_packets=4,
        secondary_source=motion_source,
    )
    rows = {
        "calibration": {
            "X": np.asarray([[0.05, 0.1]], dtype=np.float64),
            "ready": np.asarray([True]),
            "eligible": np.asarray([True]),
            "packet_index": np.asarray([3]),
            "packet_weight": np.asarray([2]),
            "reset_index": np.asarray([0]),
        },
        "static": {
            "X": np.asarray([[0.1, 0.2]], dtype=np.float64),
            "ready": np.asarray([True]),
            "eligible": np.asarray([True]),
            "packet_index": np.asarray([3]),
            "packet_weight": np.asarray([2]),
            "reset_index": np.asarray([0]),
        },
        "motion": {
            "X": np.asarray([[0.3, 0.4]], dtype=np.float64),
            "ready": np.asarray([True]),
            "eligible": np.asarray([True]),
            "packet_index": np.asarray([3]),
            "packet_weight": np.asarray([2]),
            "reset_index": np.asarray([0]),
        },
    }
    npz_cache.save_classic_replay_row_artifact(
        static_source,
        parameters=parameters,
        rows=rows,
    )

    cached = npz_cache.load_classic_replay_row_artifact(
        static_source,
        parameters=parameters,
    )

    assert cached is not None
    for phase in ("calibration", "static", "motion"):
        for key in rows[phase]:
            np.testing.assert_array_equal(cached[phase][key], rows[phase][key])


def test_ml_replay_row_artifact_roundtrip_tracks_source_identity(tmp_path):
    source_path = tmp_path / "replay_rows.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])

    parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=(1, 2),
        window_size=100,
        feature_names=("turbulence", "l1_delta"),
    )
    expected_X = np.asarray([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
    npz_cache.save_ml_replay_row_artifact(
        source_path,
        parameters=parameters,
        X=expected_X,
        feature_names=("turbulence", "l1_delta"),
        packet_index=np.asarray([99, 199], dtype=np.int32),
        evaluation_index=np.asarray([0, 1], dtype=np.int32),
        reset_index=np.asarray([0, 1], dtype=np.int32),
        evaluation_due=np.asarray([False, True]),
    )

    cached = npz_cache.load_ml_replay_row_artifact(
        source_path,
        parameters=parameters,
    )

    assert cached is not None
    np.testing.assert_allclose(cached["X"], expected_X)
    assert cached["feature_names"] == ["turbulence", "l1_delta"]
    assert cached["packet_index"].tolist() == [99, 199]
    assert cached["evaluation_index"].tolist() == [0, 1]
    assert cached["reset_index"].tolist() == [0, 1]
    assert cached["evaluation_due"].tolist() == [False, True]

    _write_source_npz(source_path, values=[9, 8, 7, 6, 5, 4])

    assert npz_cache.load_ml_replay_row_artifact(
        source_path,
        parameters=parameters,
    ) is None


def test_ml_replay_row_key_tracks_features_but_not_numeric_weights(
    monkeypatch, tmp_path
):
    python_dir = tmp_path / "src" / "python" / "micro_espectre"
    python_dir.mkdir(parents=True)
    feature_source = python_dir / "csi_features.py"
    weights_source = python_dir / "ml_weights.py"
    feature_source.write_text("FEATURE_NAMES = ['f0']\n")
    weights_source.write_text("W1 = [1.0]\n")
    monkeypatch.setattr(npz_cache, "python_src_dir", lambda: python_dir)
    monkeypatch.setattr(npz_cache, "tools_lib_dir", lambda: python_dir)

    first = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=(1, 2),
        window_size=100,
        feature_names=("f0",),
    )

    weights_source.write_text("W1 = [2.0]\n")
    after_weight_change = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=(1, 2),
        window_size=100,
        feature_names=("f0",),
    )
    assert after_weight_change == first

    feature_source.write_text("FEATURE_NAMES = ['f0']\nLAG = 11\n")
    after_feature_change = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=(1, 2),
        window_size=100,
        feature_names=("f0",),
    )
    assert after_feature_change != first


def test_classic_replay_row_parameters_change_when_detector_changes(
    monkeypatch, tmp_path
):
    python_dir = tmp_path / "src" / "python" / "micro_espectre"
    cpp_dir = tmp_path / "src" / "cpp" / "core"
    python_dir.mkdir(parents=True)
    cpp_dir.mkdir(parents=True)
    python_detector = python_dir / "lightweight_detector.py"
    cpp_header = cpp_dir / "lightweight_detector.h"
    cpp_impl = cpp_dir / "lightweight_detector.cpp"
    python_detector.write_text("BASE_THRESHOLD = 0.8\n")
    cpp_header.write_text("// classic v1\n")
    cpp_impl.write_text("// classic impl v1\n")

    monkeypatch.setattr(npz_cache, "python_src_dir", lambda: python_dir)
    monkeypatch.setattr(npz_cache, "tools_lib_dir", lambda: python_dir)
    monkeypatch.setattr(npz_cache, "cpp_core_dir", lambda: cpp_dir)

    timing = {
        "interval_us": 10_000,
        "window_packets": 4,
        "lag": 1,
        "autocorr_lag": 1,
    }
    first = npz_cache.classic_replay_row_parameters(
        replay_kind="classic_dataset",
        selected_subcarriers=(1, 2, 3),
        timing=timing,
        replay_interval_us=10_000,
        warmup_packets=4,
    )

    python_detector.write_text("BASE_THRESHOLD = 0.7\n")

    second = npz_cache.classic_replay_row_parameters(
        replay_kind="classic_dataset",
        selected_subcarriers=(1, 2, 3),
        timing=timing,
        replay_interval_us=10_000,
        warmup_packets=4,
    )

    assert first["classic_sources"] != second["classic_sources"]
    assert first != second


def test_classic_replay_row_parameters_change_when_temporal_policy_changes(
    monkeypatch, tmp_path
):
    python_dir = tmp_path / "src" / "python" / "micro_espectre"
    tools_dir = tmp_path / "tools" / "lib"
    python_dir.mkdir(parents=True)
    tools_dir.mkdir(parents=True)
    temporal_sampler = python_dir / "temporal_csi_sampler.py"
    temporal_sampler.write_text("SLOT_POLICY = 'first'\n")
    (python_dir / "runtime_policy.py").write_text("CADENCE = 1\n")
    (tools_dir / "dataset_metadata.py").write_text("# metadata\n")
    (tools_dir / "performance_report.py").write_text("# replay\n")
    (tools_dir / "temporal_replay.py").write_text("# temporal adapter\n")

    monkeypatch.setattr(npz_cache, "python_src_dir", lambda: python_dir)
    monkeypatch.setattr(npz_cache, "tools_lib_dir", lambda: python_dir)
    monkeypatch.setattr(npz_cache, "repo_root", lambda: tmp_path)

    timing = {
        "interval_us": 10_000,
        "window_packets": 4,
        "lag": 1,
        "autocorr_lag": 1,
    }
    first = npz_cache.classic_replay_row_parameters(
        replay_kind="classic_dataset",
        selected_subcarriers=(1, 2, 3),
        timing=timing,
        replay_interval_us=10_000,
        warmup_packets=4,
    )

    temporal_sampler.write_text("SLOT_POLICY = 'closest_center'\n")
    second = npz_cache.classic_replay_row_parameters(
        replay_kind="classic_dataset",
        selected_subcarriers=(1, 2, 3),
        timing=timing,
        replay_interval_us=10_000,
        warmup_packets=4,
    )

    assert first["replay_policy_sources"] != second["replay_policy_sources"]
    assert first != second


def test_classic_dataset_result_reuses_persisted_rows(monkeypatch, tmp_path):
    import tools.lib.performance_report as performance_report

    static_source = tmp_path / "static_pair.npz"
    motion_source = tmp_path / "motion_pair.npz"
    _write_source_npz(static_source, values=[1, 2, 3, 4])
    _write_source_npz(motion_source, values=[5, 6, 7, 8])
    calls = {"load": 0, "build": 0}
    packets = (
        {"csi_data": [0] * 128, "device_ticks_us": 0, "seq_num": 0},
        {"csi_data": [0] * 128, "device_ticks_us": 10_000, "seq_num": 1},
    )

    def fake_load_packet_view(*_args, **_kwargs):
        calls["load"] += 1
        return packets

    phase = {
        "X": np.empty((0, 2), dtype=np.float64),
        "ready": np.empty(0, dtype=bool),
        "eligible": np.empty(0, dtype=bool),
        "packet_index": np.empty(0, dtype=np.int32),
        "packet_weight": np.empty(0, dtype=np.int32),
        "reset_index": np.empty(0, dtype=np.int32),
    }

    def fake_build(*_args, timing, **_kwargs):
        calls["build"] += 1
        return {"static": phase, "motion": phase, "timing": dict(timing)}

    monkeypatch.setattr(
        performance_report,
        "load_npz_packet_view",
        fake_load_packet_view,
    )
    monkeypatch.setattr(performance_report, "build_classic_replay_rows", fake_build)

    first = performance_report.load_or_compute_classic_replay_rows(
        static_source,
        motion_source,
        selected_subcarriers=(1, 2, 3),
        replay_kind="classic_dataset",
        warmup_packets=4,
    )
    second = performance_report.load_or_compute_classic_replay_rows(
        static_source,
        motion_source,
        selected_subcarriers=(1, 2, 3),
        replay_kind="classic_dataset",
        warmup_packets=4,
    )

    assert first["cache_hit"] is False
    assert second["cache_hit"] is True
    assert calls == {"load": 3, "build": 1}


def test_cache_progress_stays_quiet_when_disabled(tmp_path, capsys):
    source_path = tmp_path / "quiet.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    npz_cache.save_npz_artifact(
        source_path,
        artifact_name="unit_progress",
        artifact_version=1,
        payload={"X": np.asarray([[1.0, 2.0]], dtype=np.float32)},
    )
    loaded = npz_cache.load_npz_artifact(
        source_path,
        artifact_name="unit_progress",
        artifact_version=1,
    )

    assert loaded is not None
    assert "[npz-cache]" not in capsys.readouterr().err


def test_cache_progress_reports_miss_write_and_hit(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv(npz_cache.NPZ_CACHE_PROGRESS_ENV, "1")
    monkeypatch.setenv(npz_cache.NPZ_CACHE_PROGRESS_INTERVAL_ENV, "0.05")
    npz_cache.reset_npz_cache_progress()
    source_path = tmp_path / "progress_source.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])

    assert (
        npz_cache.load_npz_artifact(
            source_path,
            artifact_name="unit_progress",
            artifact_version=1,
        )
        is None
    )
    npz_cache.save_npz_artifact(
        source_path,
        artifact_name="unit_progress",
        artifact_version=1,
        payload={"X": np.asarray([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)},
    )
    assert (
        npz_cache.load_npz_artifact(
            source_path,
            artifact_name="unit_progress",
            artifact_version=1,
        )
        is not None
    )
    time.sleep(0.08)
    assert (
        npz_cache.load_npz_artifact(
            source_path,
            artifact_name="unit_progress",
            artifact_version=1,
        )
        is not None
    )

    err = capsys.readouterr().err
    assert "miss unit_progress progress_source.npz" in err
    assert "wrote unit_progress progress_source.npz" in err
    assert "2 row(s)" in err
    assert "hit(s)" in err


def test_feature_superset_load_reports_cache_progress_hit(
    tmp_path, monkeypatch, capsys
):
    monkeypatch.setenv(npz_cache.NPZ_CACHE_PROGRESS_ENV, "1")
    monkeypatch.setenv(npz_cache.NPZ_CACHE_PROGRESS_INTERVAL_ENV, "30")
    npz_cache.reset_npz_cache_progress()
    source_path = tmp_path / "feature_superset.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])
    superset_parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=[1, 2],
        window_size=100,
        feature_names=["f0", "f1", "f2"],
        stream_provenance={
            "transform": "host_feature_rows_v2",
            "feature_names": ["f0", "f1", "f2"],
            "implementation": {"digest": "stable"},
        },
    )
    npz_cache.save_ml_replay_row_artifact(
        source_path,
        parameters=superset_parameters,
        X=np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32),
        feature_names=["f0", "f1", "f2"],
        packet_index=np.asarray([10, 20], dtype=np.int32),
        evaluation_index=np.asarray([0, 1], dtype=np.int32),
        reset_index=np.asarray([0, 0], dtype=np.int32),
        slot_index=np.asarray([100, 110], dtype=np.int64),
        target_pps=100,
        evaluation_due=np.asarray([True, False]),
    )
    capsys.readouterr()
    npz_cache.reset_npz_cache_progress()
    subset_parameters = npz_cache.ml_replay_row_parameters(
        selected_subcarriers=[1, 2],
        window_size=0,
        feature_names=["f2", "f0"],
        stream_provenance={
            "transform": "host_feature_rows_v2",
            "feature_names": ["f2", "f0"],
            "implementation": {"digest": "stable"},
        },
    )

    cached = npz_cache.load_feature_superset_artifact(
        source_path,
        artifact_name="ml_replay_rows",
        artifact_version=npz_cache.ML_REPLAY_ROW_ARTIFACT_VERSION,
        parameters=subset_parameters,
    )

    assert cached is not None
    err = capsys.readouterr().err
    assert "hit ml_replay_rows feature_superset.npz" in err


def test_artifact_build_lock_emits_progress_heartbeat(
    tmp_path, monkeypatch, capsys
):
    monkeypatch.setenv(npz_cache.NPZ_CACHE_PROGRESS_ENV, "1")
    monkeypatch.setenv(npz_cache.NPZ_CACHE_PROGRESS_INTERVAL_ENV, "0.05")
    npz_cache.reset_npz_cache_progress()
    source_path = tmp_path / "slow_build.npz"
    _write_source_npz(source_path, values=[1, 2, 3, 4])

    with npz_cache.artifact_build_lock(
        source_path,
        artifact_name="unit_progress",
        artifact_version=1,
    ):
        time.sleep(0.18)

    err = capsys.readouterr().err
    assert "still building unit_progress slow_build.npz" in err
