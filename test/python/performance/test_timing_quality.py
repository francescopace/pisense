# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
import json
from pathlib import Path

import numpy as np
import pytest
from tools.lib.ml_training import (
    dataset,
    feature_cache,
)
from tools import replay_lightweight_candidates
from tools.lib import performance_report
from tools.lib.timing_quality import merge_timing_summaries, summarize_capture_timing


def _timed_packets(
    count=32,
    *,
    gap_index=None,
    gap_us=400_000,
    missing_seq_step=None,
):
    packets = []
    timestamp_us = 0
    seq_num = 100
    base_row = np.tile(np.asarray([20, -12], dtype=np.int8), 64)
    for index in range(count):
        step = 1
        delta_us = 10_000
        if gap_index is not None and index == gap_index:
            delta_us = gap_us
            if missing_seq_step is not None:
                step = missing_seq_step
        timestamp_us += delta_us
        seq_num += step
        packets.append(
            {
                "csi_data": base_row.copy(),
                "device_ticks_us": timestamp_us,
                "stream_seq_num": seq_num,
            }
        )
    return packets


def test_summarize_capture_timing_classifies_clean_stream():
    summary = summarize_capture_timing(_timed_packets())

    assert summary["quality_status"] == "PASS"
    assert summary["quality_bucket"] == "clean"
    assert summary["contaminated_packets"] == 0
    assert summary["packet_rate_pps"] == pytest.approx(100.0, rel=1e-3)
    assert summary["max_gap_ms"] == pytest.approx(10.0)


def test_summarize_capture_timing_classifies_gap_contamination():
    summary = summarize_capture_timing(
        _timed_packets(gap_index=20, gap_us=400_000, missing_seq_step=40)
    )

    assert summary["quality_status"] == "FAIL"
    assert summary["quality_bucket"] == "poor"
    assert summary["contaminated_packets"] >= 1
    assert summary["max_sequence_gap_packets"] >= 20
    assert summary["max_gap_ms"] == pytest.approx(400.0)


def test_merge_timing_summaries_keeps_the_worst_bucket():
    merged = merge_timing_summaries(
        summarize_capture_timing(_timed_packets()),
        summarize_capture_timing(
            _timed_packets(count=64, gap_index=18, gap_us=180_000, missing_seq_step=2)
        ),
    )

    assert merged["quality_status"] == "WARN"
    assert merged["quality_bucket"] == "degraded"
    assert merged["packet_rate_pps"] == pytest.approx(89.38, rel=1e-3)
    assert merged["max_gap_ms"] > 150.0


def test_build_ml_replay_rows_preserves_subwindow_missing_slots_without_reset():
    rows = performance_report.build_ml_replay_rows(
        _timed_packets(count=256, gap_index=128, gap_us=400_000, missing_seq_step=40),
        feature_cache.DEFAULT_SUBCARRIERS,
        feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_cache.EXPORTED_FEATURE_NAMES,
    )

    assert rows["X"].shape[1] == len(feature_cache.EXPORTED_FEATURE_NAMES)
    assert len(rows["packet_index"]) == len(rows["evaluation_index"]) == len(rows["reset_index"])
    assert np.all(rows["reset_index"] == 0)
    assert rows["evaluation_index"][0] == 0
    assert np.all(np.diff(rows["packet_index"]) > 0)


def test_ml_replay_rows_keep_caller_window_rate_on_mismatched_capture():
    """Paired replay must keep the motion phase on the baseline temporal grid."""
    packets = _timed_packets(count=256)
    native = performance_report.build_ml_replay_rows(
        packets,
        feature_cache.DEFAULT_SUBCARRIERS,
        feature_names=feature_cache.EXPORTED_FEATURE_NAMES,
    )
    forced = performance_report.build_ml_replay_rows(
        packets,
        feature_cache.DEFAULT_SUBCARRIERS,
        90,
        feature_cache.EXPORTED_FEATURE_NAMES,
    )

    assert native["target_pps"] == 100
    assert forced["target_pps"] == 90


def test_replay_packet_timestamp_prefers_wifi_rx_like_cpp_gate():
    packet = {"wifi_rx_ts_us": 111, "device_ticks_us": 222}

    assert performance_report._packet_timestamp_us(packet, 3, 10_000) == 111
    assert performance_report._packet_timestamp_us(
        {"device_ticks_us": 222}, 3, 10_000
    ) == 222
    assert performance_report._packet_timestamp_us({}, 3, 10_000) == 30_000


def test_host_candidate_rows_match_runtime_temporal_readiness():
    packets = _timed_packets(count=220)
    timestamp_us = 0
    for index, packet in enumerate(packets):
        timestamp_us += 20_000 if index and index % 10 == 0 else 10_000
        packet["device_ticks_us"] = timestamp_us
        packet["csi_target_pps"] = 100

    runtime_rows = performance_report.build_ml_replay_rows(
        packets,
        feature_cache.DEFAULT_SUBCARRIERS,
        feature_names=feature_cache.EXPORTED_FEATURE_NAMES,
        sample_contract="stream_dense",
    )
    host_rows = feature_cache.build_host_feature_rows(
        packets,
        [
            *feature_cache.EXPORTED_FEATURE_NAMES[:-1],
            "chan_shape_subband_rank_gap",
        ],
        sample_contract="stream_dense",
    )

    assert len(runtime_rows["X"]) > 0
    np.testing.assert_array_equal(
        host_rows["packet_index"], runtime_rows["packet_index"]
    )
    np.testing.assert_array_equal(
        host_rows["reset_index"], runtime_rows["reset_index"]
    )
    shared = len(feature_cache.EXPORTED_FEATURE_NAMES) - 1
    np.testing.assert_allclose(
        host_rows["X"][:, :shared],
        runtime_rows["X"][:, :shared],
        rtol=1e-5,
        atol=1e-5,
    )

    sparse_packets = _timed_packets(count=160)
    timestamp_us = 0
    for packet in sparse_packets:
        timestamp_us += 20_000
        packet["device_ticks_us"] = timestamp_us
        packet["csi_target_pps"] = 100

    sparse_runtime = performance_report.build_ml_replay_rows(
        sparse_packets,
        feature_cache.DEFAULT_SUBCARRIERS,
        feature_names=feature_cache.EXPORTED_FEATURE_NAMES,
        sample_contract="stream_dense",
    )
    sparse_host = feature_cache.build_host_feature_rows(
        sparse_packets,
        [
            *feature_cache.EXPORTED_FEATURE_NAMES[:-1],
            "chan_shape_subband_rank_gap",
        ],
        sample_contract="stream_dense",
    )

    assert len(sparse_runtime["X"]) == 0
    assert len(sparse_host["X"]) == 0


def test_stream_dense_emits_every_packet_after_warmup_on_clean_stream():
    packets = _timed_packets(count=256)
    replay_rows = performance_report.build_ml_replay_rows(
        packets,
        feature_cache.DEFAULT_SUBCARRIERS,
        feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_cache.EXPORTED_FEATURE_NAMES,
        sample_contract="replay_tick",
    )
    stream_dense_rows = performance_report.build_ml_replay_rows(
        packets,
        feature_cache.DEFAULT_SUBCARRIERS,
        feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_cache.EXPORTED_FEATURE_NAMES,
        sample_contract="stream_dense",
    )

    assert (
        len(stream_dense_rows["X"])
        == len(packets) - feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE + 1
    )
    assert len(stream_dense_rows["X"]) > len(replay_rows["X"])
    assert (
        stream_dense_rows["packet_index"][0]
        == feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE - 1
    )
    assert np.all(stream_dense_rows["reset_index"] == 0)


def test_load_training_matrix_preserves_timing_context_and_weights(monkeypatch):
    records = [
        {
            "path": Path("clean.npz"),
            "packets": (),
            "label_name": "empty",
            "is_motion": False,
            "chip": "C6",
            "collected_at": "",
            "day_group": "2026-07-30",
            "pair_id": "pair-a",
            "session_group": "session-a",
            "lineage_group": "lineage-a",
            "dataset_role": "train",
            "synthetic": False,
            "long_recording": False,
            "environment_group": "lab",
            "timing_quality_status": "PASS",
            "timing_quality_bucket": "clean",
            "timing_summary": {},
            "timing_weight": 1.0,
        },
        {
            "path": Path("warn.npz"),
            "packets": (),
            "label_name": "motion",
            "is_motion": True,
            "chip": "C6",
            "collected_at": "",
            "day_group": "2026-07-30",
            "pair_id": "pair-b",
            "session_group": "session-b",
            "lineage_group": "lineage-b",
            "dataset_role": "train",
            "synthetic": False,
            "long_recording": False,
            "environment_group": "lab",
            "timing_quality_status": "WARN",
            "timing_quality_bucket": "degraded",
            "timing_summary": {},
            "timing_weight": 0.25,
        },
    ]
    stats = {
        "chips": ["C6"],
        "labels": {"empty": 1, "motion": 1},
        "total": 2,
        "files": ["clean.npz", "warn.npz"],
        "excluded_labels": [],
        "excluded_chips": [],
        "excluded_environments": [],
        "excluded_missing_sync_metadata": [],
        "excluded_dataset_roles": [],
        "excluded_long_recordings": [],
        "excluded_timing_quality": [],
        "session_groups": ["session-a", "session-b"],
        "lineage_groups": ["lineage-a", "lineage-b"],
        "environment_groups": ["lab"],
        "sync_metadata_files": [],
        "timing_quality_counts": {
            "clean": 1,
            "degraded": 1,
            "poor": 0,
            "unknown": 0,
        },
    }

    def fake_load_training_file_records(**_kwargs):
        return records, stats

    def fake_load_or_compute_ml_replay_rows(path, **_kwargs):
        record = next(item for item in records if item["path"] == path)
        value = 1.0 if record["label_name"] == "empty" else 2.0
        return {
            "X": np.asarray([[value]], dtype=np.float32),
            "feature_names": [feature_cache.EXPORTED_FEATURE_NAMES[0]],
            "packet_index": np.asarray(
                [feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE - 1], dtype=np.int32
            ),
            "evaluation_index": np.asarray([0], dtype=np.int32),
            "reset_index": np.asarray([0], dtype=np.int32),
            "cache_hit": True,
        }

    monkeypatch.setattr(dataset, "_load_training_file_records", fake_load_training_file_records)
    monkeypatch.setattr(
        dataset,
        "load_or_compute_ml_replay_rows",
        fake_load_or_compute_ml_replay_rows,
    )

    matrix, _ = dataset.load_training_matrix(
        feature_names=[feature_cache.EXPORTED_FEATURE_NAMES[0]],
        timing_quality_policy="downweight-warn",
        timing_warn_weight=0.25,
    )

    assert matrix["sample_context"]["timing_quality_bucket"].tolist() == [
        "clean",
        "degraded",
    ]
    np.testing.assert_allclose(
        matrix["sample_weights"],
        np.asarray([1.6, 0.4], dtype=np.float32),
        rtol=1e-6,
    )


def test_load_or_compute_ml_replay_rows_reuses_full_runtime_cache(monkeypatch, tmp_path):
    source_path = tmp_path / "capture.npz"
    packets = _timed_packets(count=256)
    np.savez(
        source_path,
        csi_data=np.asarray([packet["csi_data"] for packet in packets], dtype=np.int8),
        device_ticks_us=np.asarray([packet["device_ticks_us"] for packet in packets], dtype=np.int64),
        stream_seq_num=np.asarray([packet["stream_seq_num"] for packet in packets], dtype=np.int64),
        num_subcarriers=np.asarray(64),
        label=np.asarray("motion"),
        chip=np.asarray("c6"),
    )
    cached_rows = performance_report.build_ml_replay_rows(
        packets,
        feature_cache.DEFAULT_SUBCARRIERS,
        feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_cache.EXPORTED_FEATURE_NAMES,
        sample_contract="stream_dense",
    )
    load_calls = []

    def fake_load(source, *, parameters):
        load_calls.append(parameters["feature_names"])
        return cached_rows

    monkeypatch.setattr(
        feature_cache.npz_cache,
        "load_ml_replay_row_artifact",
        fake_load,
    )
    monkeypatch.setattr(
        performance_report,
        "load_npz_packet_view",
        lambda *_args, **_kwargs: pytest.fail("cache hit must not load packet rows"),
    )

    rows = performance_report.load_or_compute_ml_replay_rows(
        source_path,
        selected_subcarriers=feature_cache.DEFAULT_SUBCARRIERS,
        window_size=feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_names=feature_cache.EXPORTED_FEATURE_NAMES[:2],
        sample_contract="stream_dense",
        use_cache=True,
    )
    replay_rows = performance_report.load_or_compute_ml_replay_rows(
        source_path,
        selected_subcarriers=feature_cache.DEFAULT_SUBCARRIERS,
        window_size=feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_names=feature_cache.EXPORTED_FEATURE_NAMES[:2],
        sample_contract="replay_tick",
        use_cache=True,
    )
    selected_rows = performance_report.load_or_compute_ml_replay_rows(
        source_path,
        selected_subcarriers=feature_cache.DEFAULT_SUBCARRIERS,
        window_size=feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_names=feature_cache.EXPORTED_FEATURE_NAMES[:2],
        sample_contract="stream_dense",
        use_cache=True,
        cache_write=False,
        row_stride=2,
        row_offset=1,
    )

    assert len(load_calls) == 3
    assert tuple(load_calls[0]) == tuple(feature_cache.EXPORTED_FEATURE_NAMES)
    assert load_calls[0] == load_calls[1]
    assert rows["feature_names"] == list(feature_cache.EXPORTED_FEATURE_NAMES[:2])
    np.testing.assert_allclose(rows["X"], cached_rows["X"][:, :2])
    evaluation_due = np.asarray(cached_rows["evaluation_due"], dtype=bool)
    np.testing.assert_allclose(
        replay_rows["X"],
        cached_rows["X"][evaluation_due, :2],
    )
    np.testing.assert_allclose(selected_rows["X"], cached_rows["X"][1::2, :2])
    np.testing.assert_array_equal(
        selected_rows["evaluation_index"],
        cached_rows["evaluation_index"][1::2],
    )


def test_host_feature_cache_hit_does_not_materialize_packets(monkeypatch, tmp_path):
    source_path = tmp_path / "host_feature_cache_hit.npz"
    packets = _timed_packets(count=128)
    np.savez(
        source_path,
        csi_data=np.asarray([packet["csi_data"] for packet in packets], dtype=np.int8),
    )
    cached_rows = {
        "X": np.asarray([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32),
        "feature_names": ["turb_autocorr", "turb_zcr"],
        "packet_index": np.asarray([100, 101], dtype=np.int32),
        "evaluation_index": np.asarray([0, 1], dtype=np.int32),
        "reset_index": np.asarray([0, 0], dtype=np.int32),
        "evaluation_due": np.asarray([True, False]),
    }
    seen_features = []

    def fake_load_spine(_source_path, *, parameters):
        assert parameters["contract"] == "host_feature_row_spine_v1"
        return {
            key: cached_rows[key]
            for key in (
                "packet_index",
                "evaluation_index",
                "reset_index",
                "evaluation_due",
            )
        }

    def fake_load_column(_source_path, *, parameters):
        name = parameters["feature"]["feature_name"]
        seen_features.append(name)
        return cached_rows["X"][:, cached_rows["feature_names"].index(name)]

    monkeypatch.setattr(
        feature_cache.npz_cache,
        "load_host_feature_row_spine_artifact",
        fake_load_spine,
    )
    monkeypatch.setattr(
        feature_cache.npz_cache,
        "load_host_feature_column_artifact",
        fake_load_column,
    )

    rows = feature_cache.load_or_compute_host_feature_rows(
        source_path,
        packets_factory=lambda: pytest.fail("cache hit must not build packets"),
        feature_names=["turb_autocorr", "turb_zcr"],
        sample_contract="replay_tick",
        stream_provenance={"transform": "host_feature_rows_v3"},
    )

    assert seen_features == ["turb_autocorr", "turb_zcr"]
    assert rows["cache_hit"] is True
    assert rows["feature_names"] == ["turb_autocorr", "turb_zcr"]
    np.testing.assert_array_equal(rows["X"], [[1.0, 2.0]])


def test_host_feature_cache_computes_only_new_columns(monkeypatch, tmp_path):
    cache_root = tmp_path / "cache"
    monkeypatch.setenv(feature_cache.npz_cache.NPZ_CACHE_DIR_ENV, str(cache_root))
    source_path = tmp_path / "host_feature_columns.npz"
    np.savez(source_path, csi_data=np.zeros((4, 128), dtype=np.int8))
    calls = []

    def fake_build(_packets, feature_names, **_kwargs):
        calls.append(list(feature_names))
        values = {
            "turb_cv": np.asarray([1.0, 2.0], dtype=np.float32),
            "turb_mad_over_mean": np.asarray([3.0, 4.0], dtype=np.float32),
        }
        return {
            "X": np.column_stack([values[name] for name in feature_names]),
            "feature_names": list(feature_names),
            "packet_index": np.asarray([100, 101], dtype=np.int32),
            "evaluation_index": np.asarray([0, 1], dtype=np.int32),
            "reset_index": np.asarray([0, 0], dtype=np.int32),
            "evaluation_due": np.asarray([True, False]),
        }

    monkeypatch.setattr(feature_cache, "build_host_feature_rows", fake_build)

    first = feature_cache.load_or_compute_host_feature_rows(
        source_path,
        packets=[object()],
        feature_names=["turb_cv"],
        sample_contract="stream_dense",
    )
    extended = feature_cache.load_or_compute_host_feature_rows(
        source_path,
        packets=[object()],
        feature_names=["turb_cv", "turb_mad_over_mean"],
        sample_contract="stream_dense",
    )
    warm = feature_cache.load_or_compute_host_feature_rows(
        source_path,
        packets_factory=lambda: pytest.fail("warm columns must not build packets"),
        feature_names=["turb_mad_over_mean", "turb_cv"],
        sample_contract="stream_dense",
    )

    assert calls == [["turb_cv"], ["turb_mad_over_mean"]]
    assert first["cache_hit"] is False
    assert extended["cache_hit"] is False
    assert warm["cache_hit"] is True
    np.testing.assert_array_equal(extended["X"], [[1.0, 3.0], [2.0, 4.0]])
    np.testing.assert_array_equal(warm["X"], [[3.0, 1.0], [4.0, 2.0]])


def test_build_ml_replay_rows_selects_before_materializing_dense_features():
    packets = _timed_packets(count=256)
    full_rows = performance_report.build_ml_replay_rows(
        packets,
        feature_cache.DEFAULT_SUBCARRIERS,
        feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_cache.EXPORTED_FEATURE_NAMES,
        sample_contract="stream_dense",
    )

    selected_rows = performance_report.build_ml_replay_rows(
        packets,
        feature_cache.DEFAULT_SUBCARRIERS,
        feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_cache.EXPORTED_FEATURE_NAMES,
        sample_contract="stream_dense",
        row_stride=2,
        row_offset=1,
    )

    for key in (
        "X",
        "packet_index",
        "evaluation_index",
        "reset_index",
        "evaluation_due",
    ):
        np.testing.assert_array_equal(selected_rows[key], full_rows[key][1::2])


def test_augmented_replay_rows_persist_only_for_matching_provenance(
    monkeypatch,
    tmp_path,
):
    cache_root = tmp_path / "cache"
    monkeypatch.setenv(feature_cache.npz_cache.NPZ_CACHE_DIR_ENV, str(cache_root))
    source_path = tmp_path / "capture.npz"
    packets = _timed_packets(count=128)
    np.savez(
        source_path,
        csi_data=np.asarray(
            [packet["csi_data"] for packet in packets],
            dtype=np.int8,
        ),
        device_ticks_us=np.asarray(
            [packet["device_ticks_us"] for packet in packets],
            dtype=np.int64,
        ),
        stream_seq_num=np.asarray(
            [packet["stream_seq_num"] for packet in packets],
            dtype=np.int64,
        ),
        num_subcarriers=np.asarray(64),
        label=np.asarray("motion"),
        chip=np.asarray("c6"),
    )
    builds = []

    def packet_factory():
        builds.append("built")
        return packets

    provenance = {
        "transform": "training_packet_augmentation_v1",
        "config": {"packet_loss": 0.05},
        "seed": 123,
    }
    first = performance_report.load_or_compute_ml_replay_rows(
        source_path,
        packets_factory=packet_factory,
        selected_subcarriers=feature_cache.DEFAULT_SUBCARRIERS,
        window_size=feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_names=feature_cache.EXPORTED_FEATURE_NAMES[:2],
        sample_contract="stream_dense",
        stream_provenance=provenance,
    )
    second = performance_report.load_or_compute_ml_replay_rows(
        source_path,
        packets_factory=lambda: pytest.fail(
            "matching persisted provenance must not rebuild packets"
        ),
        selected_subcarriers=feature_cache.DEFAULT_SUBCARRIERS,
        window_size=feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_names=feature_cache.EXPORTED_FEATURE_NAMES[:2],
        sample_contract="stream_dense",
        stream_provenance=provenance,
    )
    different_seed = performance_report.load_or_compute_ml_replay_rows(
        source_path,
        packets_factory=packet_factory,
        selected_subcarriers=feature_cache.DEFAULT_SUBCARRIERS,
        window_size=feature_cache.DEFAULT_WINDOW_PACKETS_AT_NOMINAL_RATE,
        feature_names=feature_cache.EXPORTED_FEATURE_NAMES[:2],
        sample_contract="stream_dense",
        stream_provenance={**provenance, "seed": 124},
    )

    assert first["cache_hit"] is False
    assert second["cache_hit"] is True
    assert different_seed["cache_hit"] is False
    assert builds == ["built", "built"]
    assert len(list((cache_root / "ml_replay_rows").glob("*.npz"))) == 2


def test_load_training_matrix_stream_dense_uses_stream_dense_rows(monkeypatch):
    records = [
        {
            "path": Path("clean.npz"),
            "packets": _timed_packets(count=96),
            "label_name": "empty",
            "is_motion": False,
            "chip": "C6",
            "collected_at": "",
            "day_group": "2026-07-30",
            "pair_id": "pair-a",
            "session_group": "session-a",
            "lineage_group": "lineage-a",
            "dataset_role": "train",
            "synthetic": False,
            "long_recording": False,
            "environment_group": "lab",
            "timing_quality_status": "PASS",
            "timing_quality_bucket": "clean",
            "timing_summary": {},
            "timing_weight": 1.0,
        }
    ]
    stats = {
        "chips": ["C6"],
        "labels": {"empty": 1},
        "total": 1,
        "files": ["clean.npz"],
        "excluded_labels": [],
        "excluded_chips": [],
        "excluded_environments": [],
        "excluded_missing_sync_metadata": [],
        "excluded_dataset_roles": [],
        "excluded_long_recordings": [],
        "excluded_timing_quality": [],
        "session_groups": ["session-a"],
        "lineage_groups": ["lineage-a"],
        "environment_groups": ["lab"],
        "sync_metadata_files": [],
        "timing_quality_counts": {
            "clean": 1,
            "degraded": 0,
            "poor": 0,
            "unknown": 0,
        },
    }
    replay_rows = {
        "X": np.asarray([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32),
        "feature_names": list(feature_cache.EXPORTED_FEATURE_NAMES[:2]),
        "packet_index": np.asarray([99, 100], dtype=np.int32),
        "evaluation_index": np.asarray([0, 1], dtype=np.int32),
        "reset_index": np.asarray([0, 0], dtype=np.int32),
    }
    seen_calls = []

    def fake_load_training_file_records(**_kwargs):
        return records, stats

    def fake_load_or_compute_ml_replay_rows(*_args, **_kwargs):
        seen_calls.append(_kwargs)
        return replay_rows

    monkeypatch.setattr(dataset, "_load_training_file_records", fake_load_training_file_records)
    monkeypatch.setattr(
        dataset,
        "load_or_compute_ml_replay_rows",
        fake_load_or_compute_ml_replay_rows,
    )
    monkeypatch.setattr(
        feature_cache.npz_cache,
        "load_ml_replay_row_artifact",
        lambda *_args, **_kwargs: None,
    )

    matrix, _ = dataset.load_training_matrix(
        feature_names=list(feature_cache.EXPORTED_FEATURE_NAMES[:2]),
    )

    assert [call["sample_contract"] for call in seen_calls] == ["stream_dense"]
    np.testing.assert_allclose(matrix["X"], replay_rows["X"])
    assert matrix["sample_context"]["packet_index"].tolist() == [99, 100]

    dataset.load_training_matrix(
        feature_names=list(feature_cache.EXPORTED_FEATURE_NAMES[:2]),
        packet_augmentation={"packet_loss": 0.05},
        augmentation_seed=123,
    )

    augmented_call = seen_calls[1]
    assert callable(augmented_call["packets_factory"])
    assert augmented_call["stream_provenance"]["seed"] == 123
    assert augmented_call["stream_provenance"]["config"] == {
        "packet_loss": 0.05,
    }
    assert augmented_call["use_cache"] is True


def test_classic_candidate_replay_reuses_time_aware_runtime_rows(monkeypatch):
    seen = []

    def fake_load_rows(path, **kwargs):
        seen.append((path, kwargs))
        return {
            "X": np.asarray([[1.0], [2.0], [3.0], [4.0]], dtype=np.float32),
            "packet_index": np.asarray([99, 124, 199, 299], dtype=np.int32),
            "reset_index": np.asarray([0, 0, 0, 1], dtype=np.int32),
        }

    monkeypatch.setattr(
        replay_lightweight_candidates,
        "load_or_compute_ml_replay_rows",
        fake_load_rows,
    )
    monkeypatch.setattr(
        replay_lightweight_candidates,
        "load_npz_as_packets",
        lambda _path: [],
    )
    monkeypatch.setattr(
        replay_lightweight_candidates,
        "detector_window_packets",
        lambda _packets: 100,
    )

    cache = replay_lightweight_candidates.build_replay_cache(
        [Path("runtime.npz")],
        [feature_cache.EXPORTED_FEATURE_NAMES[0]],
        quiet=True,
    )

    assert seen[0][1]["sample_contract"] == "replay_tick"
    assert cache["runtime.npz"]["deoverlapped"].tolist() == [
        True,
        False,
        True,
        True,
    ]


def test_classic_candidate_replay_can_retain_external_diagnostic_phy(monkeypatch):
    seen = {}
    packets = [{"csi_data": np.asarray([1, 2], dtype=np.int8)}]

    def fake_load_packets(path, *, keep_all_phy=False):
        seen["packet_path"] = path
        seen["keep_all_phy"] = keep_all_phy
        return packets

    def fake_load_rows(path, **kwargs):
        seen["row_path"] = path
        seen["stream_provenance"] = kwargs["stream_provenance"]
        assert kwargs["packets_factory"]() is packets
        return {
            "X": np.asarray([[1.0]], dtype=np.float32),
            "packet_index": np.asarray([99], dtype=np.int32),
            "reset_index": np.asarray([0], dtype=np.int32),
        }

    monkeypatch.setattr(
        replay_lightweight_candidates,
        "load_npz_as_packets",
        fake_load_packets,
    )
    monkeypatch.setattr(
        replay_lightweight_candidates,
        "load_or_compute_ml_replay_rows",
        fake_load_rows,
    )
    monkeypatch.setattr(
        replay_lightweight_candidates,
        "detector_window_packets",
        lambda _packets: 100,
    )

    replay_lightweight_candidates.build_replay_cache(
        [Path("diagnostic.npz")],
        [feature_cache.EXPORTED_FEATURE_NAMES[0]],
        quiet=True,
        keep_all_phy=True,
    )

    assert seen["packet_path"] == Path("diagnostic.npz")
    assert seen["keep_all_phy"] is True
    assert seen["stream_provenance"] == {"packet_view": "all_explicit_phy"}


def test_classic_candidate_replay_resolves_external_holdout_catalog(tmp_path):
    catalog = {
        "files": {
            "static_presence": [
                {
                    "filename": "static-logical.npz",
                    "relative_path": "idle/static.npz",
                    "dataset_role": "holdout",
                    "chip": "ESP32",
                    "optimal_pair_motion_file": "motion-logical.npz",
                }
            ],
            "motion": [
                {
                    "filename": "motion-logical.npz",
                    "relative_path": "walk/motion.npz",
                    "dataset_role": "holdout",
                    "chip": "ESP32",
                }
            ],
            "empty": [
                {
                    "filename": "empty-logical.npz",
                    "relative_path": "empty/quiet.npz",
                    "dataset_role": "holdout",
                    "chip": "ESP32",
                }
            ],
        }
    }
    dataset_info_path = tmp_path / "dataset_info.json"
    dataset_info_path.write_text(json.dumps(catalog), encoding="utf-8")

    pairs = replay_lightweight_candidates.iter_replay_pairs(
        dataset_info_path=dataset_info_path,
        dataset_root=tmp_path,
    )
    empties = replay_lightweight_candidates.iter_empty_replays(
        dataset_info_path=dataset_info_path,
        dataset_root=tmp_path,
    )

    assert len(pairs) == 1
    assert pairs[0]["role"] == "holdout"
    assert pairs[0]["static_path"] == tmp_path / "idle/static.npz"
    assert pairs[0]["motion_path"] == tmp_path / "walk/motion.npz"
    assert empties == [
        {
            "session": "empty-logical.npz",
            "chip": "ESP32",
            "role": "holdout",
            "path": tmp_path / "empty/quiet.npz",
        }
    ]


def test_classic_candidate_replay_persists_host_feature_rows(monkeypatch):
    seen = {}

    def fake_load_rows(path, **kwargs):
        seen["path"] = path
        seen["kwargs"] = kwargs
        return {
            "X": np.asarray([[1.0], [2.0]], dtype=np.float32),
            "packet_index": np.asarray([99, 199], dtype=np.int32),
            "reset_index": np.asarray([0, 0], dtype=np.int32),
            "cache_hit": True,
        }

    monkeypatch.setattr(
        replay_lightweight_candidates.feature_cache,
        "load_or_compute_host_feature_rows",
        fake_load_rows,
    )
    monkeypatch.setattr(
        replay_lightweight_candidates.feature_cache,
        "_host_feature_stream_provenance",
        lambda names, **_kwargs: {"features": list(names)},
    )
    monkeypatch.setattr(
        replay_lightweight_candidates,
        "load_npz_as_packets",
        lambda _path: [],
    )
    monkeypatch.setattr(
        replay_lightweight_candidates,
        "detector_window_packets",
        lambda _packets: 100,
    )

    cache = replay_lightweight_candidates.build_replay_cache(
        [Path("host.npz")],
        ["chan_freq_coh_curve_std"],
        quiet=True,
    )

    assert seen["kwargs"]["sample_contract"] == "replay_tick"
    assert seen["kwargs"]["stream_provenance"] == {
        "features": ["chan_freq_coh_curve_std"]
    }
    assert cache["host.npz"]["cache_hit"] is True
    assert cache["host.npz"]["deoverlapped"].tolist() == [True, True]


def test_classic_candidate_packet_stress_has_distinct_cache_provenance(monkeypatch):
    seen = {}
    packets = [{"csi_data": np.asarray([1, 2], dtype=np.int8)}]

    monkeypatch.setattr(
        replay_lightweight_candidates,
        "load_npz_as_packets",
        lambda _path: packets,
    )

    def fake_prepare_packets(record, **kwargs):
        seen["prepared"] = (record, kwargs)
        return packets

    monkeypatch.setattr(
        replay_lightweight_candidates.augmentation,
        "_prepare_feature_packets_for_record",
        fake_prepare_packets,
    )
    monkeypatch.setattr(
        replay_lightweight_candidates.augmentation,
        "_packet_augmentation_stream_provenance",
        lambda config, seed: {"config": dict(config), "seed": seed},
    )

    def fake_load_rows(_path, **kwargs):
        seen["kwargs"] = kwargs
        kwargs["packets_factory"]()
        return {
            "X": np.asarray([[1.0]], dtype=np.float32),
            "packet_index": np.asarray([99], dtype=np.int32),
            "reset_index": np.asarray([0], dtype=np.int32),
        }

    monkeypatch.setattr(
        replay_lightweight_candidates,
        "load_or_compute_ml_replay_rows",
        fake_load_rows,
    )

    replay_lightweight_candidates.build_replay_cache(
        [Path("runtime.npz")],
        [feature_cache.EXPORTED_FEATURE_NAMES[0]],
        quiet=True,
        packet_augmentation={"packet_loss": 0.05},
        augmentation_seed=123,
    )

    assert callable(seen["kwargs"]["packets_factory"])
    assert seen["kwargs"]["stream_provenance"] == {
        "config": {"packet_loss": 0.05},
        "seed": 123,
    }
    assert seen["prepared"][1] == {
        "packet_augmentation": {"packet_loss": 0.05},
        "augmentation_seed": 123,
    }
