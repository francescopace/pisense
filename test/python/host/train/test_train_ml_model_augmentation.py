# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
import argparse
import sys
import types

import numpy as np
import pytest

from tools.lib.ml_training import (
    augmentation,
    dataset,
    evaluation,
    export,
    feature_cache,
    preprocessing,
    training,
)
import tools.train_ml_model as training_cli
from tools.lib.adjacent_aggregation import aggregation_groups, make_aggregating_fill
from tools.lib.candidate_features import candidate_values


def _synthetic_packets(count=200, *, source="sample.npz", interval_us=10_000):
    packets = []
    base_row = np.tile(np.asarray([20, -12], dtype=np.int8), 64)
    for index in range(count):
        packets.append(
            {
                "csi_data": base_row.copy(),
                "source_file": source,
                "device_ticks_us": index * interval_us,
            }
        )
    return packets


@pytest.mark.parametrize("evaluate_deployment", [False, True])
def test_clipped_scaler_rejects_runtime_training_before_loading_data(
    monkeypatch, evaluate_deployment,
):
    def unexpected_load(*args, **kwargs):
        pytest.fail("runtime-incompatible preprocessing must fail before loading data")

    monkeypatch.setattr(training, "ensure_torch_available", unexpected_load)
    result = training.train_all(
        scaler_mode="clipped_standard",
        export_artifacts=not evaluate_deployment,
        evaluate_deployment=evaluate_deployment,
        seed=17,
    )
    assert result == (1, 17, None)


@pytest.mark.parametrize("exporter", [export.export_micropython, export.export_cpp_weights])
def test_clipped_scaler_cannot_silently_export_affine_normalization(tmp_path, exporter):
    scaler = preprocessing.ClippedStandardScaler().fit(
        np.arange(100, dtype=np.float32).reshape(-1, 1)
    )
    model = types.SimpleNamespace(
        get_weights=lambda: [np.ones((1, 1)), np.zeros(1)],
    )
    destination = tmp_path / "weights.py"
    destination.write_text("existing weights\n", encoding="utf-8")
    with pytest.raises(ValueError, match="affine scaler"):
        exporter(model, scaler, destination)
    assert destination.read_text(encoding="utf-8") == "existing weights\n"
    with pytest.raises(ValueError, match="affine scaler"):
        evaluation.StreamingEvaluator(model, scaler, ["turb_autocorr"])

    # Host CV still clips outliers and can derive augmentation bounds.
    assert scaler.transform([[1000]]) == pytest.approx(
        scaler.transform(scaler.upper_bounds_.reshape(1, -1))
    )
    lower, upper = preprocessing.normalized_feature_bounds(scaler, ["turb_autocorr"])
    assert np.all(np.isfinite(lower)) and np.all(np.isfinite(upper))


def test_parse_augmentation_components_normalizes_order_and_deduplicates():
    assert augmentation.parse_augmentation_components("burst-loss,base,drift,base") == (
        "base",
        "drift",
        "burst-loss",
    )
    assert augmentation.parse_augmentation_components(True) == (
        "base",
        "drift",
        "burst-loss",
    )
    assert augmentation.parse_augmentation_components(None) == tuple()


def test_parse_augmentation_components_rejects_unknown_names():
    with pytest.raises(argparse.ArgumentTypeError):
        augmentation.parse_augmentation_components("base,unknown")


def test_resolve_training_augmentation_merges_selected_components():
    components, feature_augmentation, packet_augmentation = augmentation.resolve_training_augmentation(
        "base,drift,burst-loss"
    )

    assert components == ("base", "drift", "burst-loss")
    assert feature_augmentation["jitter_sigma"] == pytest.approx(0.10)
    assert packet_augmentation["noise_sigma"] == pytest.approx(0.01)
    assert packet_augmentation["packet_loss"] == pytest.approx(0.05)
    assert packet_augmentation["stutter_probability"] == pytest.approx(0.08)
    assert packet_augmentation["packet_rate_scale"] == pytest.approx((0.7, 1.0))
    assert packet_augmentation["min_target_rate_pps"] == pytest.approx(70.0)
    assert packet_augmentation["drift_sigma"] > 0.0
    assert packet_augmentation["burst_loss_starts_per_minute"] > 0.0


def test_cache_provenance_fingerprints_only_stream_implementations():
    packet_provenance = augmentation._packet_augmentation_stream_provenance(
        {"packet_loss": 0.05},
        20260807,
    )
    host_provenance = feature_cache._host_feature_stream_provenance(
        ["turb_mad_over_mean_aggr"],
    )

    assert packet_provenance["transform"] == "training_packet_augmentation_v2"
    assert len(packet_provenance["implementation_sha256"]) == 64
    assert packet_provenance["timing_quality"]["content_sha256"]
    assert host_provenance["transform"] == "host_feature_rows_v4"
    feature_identity = host_provenance["feature_identities"][
        "turb_mad_over_mean_aggr"
    ]
    assert feature_identity["provider"] == "aggregated_turbulence_series"
    assert len(feature_identity["formula_sha256"]) == 64
    assert "train_ml_model" not in host_provenance["row_stream"]


def test_production_missing_slot_contract_has_distinct_cache_identity():
    provenance = feature_cache._host_feature_stream_provenance(
        ["turb_autocorr", "turb_iqr_over_mean_aggr", "chan_shape_excess_path"],
    )
    identities = provenance["feature_identities"]

    assert identities["turb_autocorr"]["provider_version"] == 2
    assert identities["turb_iqr_over_mean_aggr"]["provider_version"] == 2
    assert identities["chan_shape_excess_path"]["provider_version"] == 2


def test_cache_provenance_memoization_returns_isolated_values():
    feature_cache._host_feature_base_stream_provenance.cache_clear()
    augmentation._packet_augmentation_stream_provenance_cached.cache_clear()
    feature_names = ["turb_mad_over_mean_aggr"]
    packet_config = {"packet_loss": 0.05, "noise_sigma": 0.01}

    first_host = feature_cache._host_feature_stream_provenance(feature_names)
    first_packet = augmentation._packet_augmentation_stream_provenance(
        packet_config,
        20260807,
    )
    first_host["feature_names"].append("mutated")
    first_packet["config"]["packet_loss"] = 1.0

    second_host = feature_cache._host_feature_stream_provenance(feature_names)
    second_packet = augmentation._packet_augmentation_stream_provenance(
        {"noise_sigma": 0.01, "packet_loss": 0.05},
        20260807,
    )

    assert second_host["feature_names"] == feature_names
    assert second_packet["config"]["packet_loss"] == pytest.approx(0.05)
    assert feature_cache._host_feature_base_stream_provenance.cache_info().hits == 1
    assert augmentation._packet_augmentation_stream_provenance_cached.cache_info().hits == 1


def test_trajectory_bin_experiment_has_distinct_host_cache_identity(monkeypatch):
    monkeypatch.setattr(
        feature_cache,
        "ACTIVE_TRAJECTORY_BIN_US",
        feature_cache.CHANNEL_SHAPE_BIN_US,
    )
    feature_cache.set_active_trajectory_bin_ms(40)
    forty_ms = feature_cache._host_feature_stream_provenance(
        [
            "turb_autocorr",
            "chan_shape_excess_path",
            "chan_shape_scale_curvature",
        ],
    )
    extractor = feature_cache.StreamingFeatureExtractor(
        ["chan_shape_excess_path", "chan_shape_scale_curvature"],
    )

    feature_cache.set_active_trajectory_bin_ms(80)
    eighty_ms = feature_cache._host_feature_stream_provenance(
        [
            "turb_autocorr",
            "chan_shape_excess_path",
            "chan_shape_scale_curvature",
        ],
    )

    assert extractor.shape_trajectory_tracker.bin_us == 40_000
    assert (
        forty_ms["feature_identities"]["turb_autocorr"]
        == eighty_ms["feature_identities"]["turb_autocorr"]
    )
    assert (
        forty_ms["feature_identities"]["chan_shape_excess_path"]
        == eighty_ms["feature_identities"]["chan_shape_excess_path"]
    )
    assert (
        forty_ms["feature_identities"]["chan_shape_scale_curvature"]
        != eighty_ms["feature_identities"]["chan_shape_scale_curvature"]
    )


def test_promoted_trajectory_feature_uses_only_production_tracker():
    extractor = feature_cache.StreamingFeatureExtractor(["chan_shape_excess_path"])

    assert extractor.shape_trajectory_tracker is None
    assert extractor.production_extractor.shape_trajectory_tracker is not None


def test_non_default_trajectory_bin_cannot_export(monkeypatch, capsys):
    monkeypatch.setattr(
        sys,
        "argv",
        ["train_ml_model.py", "--trajectory-bin-ms", "50", "--seed", "7"],
    )
    monkeypatch.setattr(
        training_cli,
        "train_all",
        lambda **kwargs: pytest.fail("experimental bin must not reach export"),
    )

    assert training_cli.main() == 1
    assert "requires a read-only flow" in capsys.readouterr().out


def test_promoted_packet_augmentation_uses_two_fixed_views():
    assert augmentation.training_packet_augmentation_seeds({"packet_loss": 0.05}) == (
        20260807,
        20260808,
    )
    assert augmentation.training_packet_augmentation_seeds(None) == tuple()
    assert (
        augmentation.training_packet_augmentation_seed({"packet_loss": 0.05})
        == 20260807
    )


def test_seed_search_restore_removes_artifacts_created_after_backup(
    monkeypatch,
    tmp_path,
):
    existing = tmp_path / "ml_weights.py"
    initially_missing = tmp_path / "ml_weights.h"
    existing.write_text("before\n", encoding="utf-8")
    monkeypatch.setattr(
        export,
        "_model_artifact_paths",
        lambda: [existing, initially_missing],
    )

    backup_dir, saved_files = export._backup_artifacts()
    try:
        existing.write_text("after\n", encoding="utf-8")
        initially_missing.write_text("created\n", encoding="utf-8")
        export._restore_artifacts(saved_files)
    finally:
        export.shutil.rmtree(backup_dir, ignore_errors=True)

    assert existing.read_text(encoding="utf-8") == "before\n"
    assert not initially_missing.exists()


def test_packet_augmentation_view_mix_is_constant_size_and_deterministic():
    def rows(offset, count):
        return {
            "X": np.arange(offset, offset + count, dtype=np.float32).reshape(-1, 1),
            "feature_names": ["feature"],
            "packet_index": np.arange(count, dtype=np.int32) + offset,
            "evaluation_index": np.arange(count, dtype=np.int32),
            "reset_index": np.zeros(count, dtype=np.int32),
            "evaluation_due": np.ones(count, dtype=bool),
        }

    first = feature_cache._mix_packet_augmentation_replay_rows((rows(0, 5), rows(10, 5)))
    second = feature_cache._mix_packet_augmentation_replay_rows((rows(0, 5), rows(10, 5)))

    assert first["X"].ravel().tolist() == [0.0, 2.0, 4.0, 11.0, 13.0]
    assert len(first["X"]) == 5
    for key in (
        "X",
        "packet_index",
        "evaluation_index",
        "reset_index",
        "evaluation_due",
    ):
        np.testing.assert_array_equal(first[key], second[key])


def test_mixed_packet_augmentation_cache_skips_both_views_when_warm(
    monkeypatch,
    tmp_path,
):
    cache_root = tmp_path / "cache"
    monkeypatch.setenv(feature_cache.npz_cache.NPZ_CACHE_DIR_ENV, str(cache_root))
    source_path = tmp_path / "capture.npz"
    np.savez(source_path, csi_data=np.zeros((4, 2), dtype=np.int8))
    record = {"path": source_path}
    calls = []

    def fake_load_rows(_path, **kwargs):
        seed = int(kwargs["stream_provenance"]["seed"])
        calls.append(seed)
        offset = 0 if seed == 20260807 else 10
        row_stride = int(kwargs["row_stride"])
        row_offset = int(kwargs["row_offset"])
        mask = np.arange(4) % row_stride == row_offset
        return {
            "X": np.arange(offset, offset + 4, dtype=np.float32).reshape(-1, 1)[mask],
            "feature_names": [feature_cache.EXPORTED_FEATURE_NAMES[0]],
            "packet_index": np.arange(4, dtype=np.int32)[mask],
            "evaluation_index": np.arange(4, dtype=np.int32)[mask],
            "reset_index": np.zeros(4, dtype=np.int32)[mask],
            "evaluation_due": np.ones(4, dtype=bool)[mask],
            "cache_hit": False,
        }

    monkeypatch.setattr(feature_cache, "load_or_compute_ml_replay_rows", fake_load_rows)
    kwargs = {
        "packet_augmentation": {"packet_loss": 0.05},
        "augmentation_seeds": augmentation.FIXED_PACKET_AUGMENTATION_SEEDS,
        "feature_names": [feature_cache.EXPORTED_FEATURE_NAMES[0]],
        "use_cache": True,
        "use_runtime_cache": True,
    }

    first = feature_cache._load_or_compute_packet_augmentation_mix_rows(record, **kwargs)
    second = feature_cache._load_or_compute_packet_augmentation_mix_rows(record, **kwargs)

    assert first["cache_hit"] is False
    assert second["cache_hit"] is True
    assert calls == [20260807, 20260808]
    assert first["X"].ravel().tolist() == [0.0, 2.0, 11.0, 13.0]
    np.testing.assert_array_equal(first["X"], second["X"])
    assert (
        len(list((cache_root / "ml_training_augmentation_rows").glob("*.npz")))
        == 1
    )
    assert not (cache_root / "ml_replay_rows").exists()


def test_vectorized_scaled_iq_noise_matches_scalar_reference():
    raw = np.arange(128, dtype=np.float64) - 64.0
    expected = raw.copy()
    expected_rng = np.random.default_rng(1234)
    for subcarrier in range(64):
        pair = slice(2 * subcarrier, 2 * subcarrier + 2)
        magnitude = max(1.0, float(np.linalg.norm(expected[pair])))
        expected[pair] += expected_rng.normal(
            0.0,
            0.01 * magnitude / np.sqrt(2.0),
            size=2,
        )

    actual = raw.copy()
    augmentation._add_scaled_iq_noise(
        actual,
        64,
        0.01,
        np.random.default_rng(1234),
    )

    np.testing.assert_array_equal(actual, expected)


def test_training_source_metadata_cache_avoids_reloading_packet_rows(
    monkeypatch,
    tmp_path,
):
    cache_root = tmp_path / "cache"
    monkeypatch.setenv(feature_cache.npz_cache.NPZ_CACHE_DIR_ENV, str(cache_root))
    source_path = tmp_path / "motion.npz"
    np.savez(
        source_path,
        csi_data=np.zeros((8, 128), dtype=np.int8),
        device_ticks_us=np.arange(8, dtype=np.int64) * 10_000,
        stream_seq_num=np.arange(8, dtype=np.int64),
        num_subcarriers=np.asarray(64),
        label=np.asarray("motion"),
        chip=np.asarray("c6"),
    )

    first = dataset._load_or_compute_training_source_metadata(source_path)
    feature_cache.npz_cache.clear_runtime_artifacts()
    monkeypatch.setattr(
        dataset,
        "load_npz_packet_view",
        lambda *_args, **_kwargs: pytest.fail(
            "metadata cache hit must not reload packet rows"
        ),
    )
    second = dataset._load_or_compute_training_source_metadata(source_path)

    assert first == second
    assert first["packet_count"] == 8
    assert first["label"] == "motion"
    assert len(
        list((cache_root / "ml_training_source_metadata").glob("*.npz"))
    ) == 1


def test_normalized_feature_bounds_clamp_nonnegative_candidates():
    feature_names = [
        "turb_mad_over_mean",
        "turb_iqr_over_mean",
        "turb_p95_over_mean",
        "turb_mad_over_mean_aggr",
        "turb_iqr_over_mean_aggr",
        "turb_p95_over_mean_aggr",
    ]
    preprocessor = type(
        "Preprocessor",
        (),
        {
            "mean_": np.asarray([0.2] * len(feature_names), dtype=np.float32),
            "scale_": np.asarray([0.1] * len(feature_names), dtype=np.float32),
        },
    )()

    lower, upper = preprocessing.normalized_feature_bounds(preprocessor, feature_names)

    np.testing.assert_allclose(lower, np.full(len(feature_names), -2.0))
    assert np.all(np.isposinf(upper))


def test_w5_aggregation_groups_stay_inside_live_ht20_bins():
    groups = aggregation_groups((4, 28, 36, 60), 5)

    assert groups == (
        (4, 5, 6, 7, 8),
        (26, 27, 28, 29, 30),
        (34, 35, 36, 37, 38),
        (56, 57, 58, 59, 60),
    )
    assert all(32 not in group for group in groups)


def test_w5_aggregation_averages_magnitudes():
    csi_data = np.zeros(128, dtype=np.int8)
    for subcarrier in range(64):
        csi_data[subcarrier * 2 + 1] = subcarrier
    output = [0.0] * 4

    written = make_aggregating_fill(5)(csi_data, (4, 28, 36, 60), output)

    assert written == 4
    assert output == pytest.approx([6.0, 28.0, 36.0, 58.0])


def test_aggregated_dispersion_candidates_match_their_definitions():
    turbulence = np.asarray([1.0, 2.0, 4.0, 8.0])

    values = candidate_values(
        [
            "turb_mad_over_mean_aggr",
            "turb_p95_over_mean_aggr",
        ],
        aggregated_turbulence_series=turbulence,
    )

    mean = float(np.mean(turbulence))
    median = float(np.median(turbulence))
    expected_mad = float(np.median(np.abs(turbulence - median))) / mean
    q95 = np.percentile(turbulence, 95)
    assert values == pytest.approx(
        {
            "turb_mad_over_mean_aggr": expected_mad,
            "turb_p95_over_mean_aggr": float(q95) / mean,
        }
    )


def test_cpp_feature_ids_accept_promoted_iqr_and_reject_host_only_candidates():
    assert export.resolve_cpp_feature_ids(["turb_iqr_over_mean_aggr"]) == [45]
    assert export.resolve_cpp_feature_ids(["chan_shape_spread_subband"]) == [48]
    assert export.resolve_cpp_feature_ids(
        ["chan_shape_subband_kendall_lag_excess"]
    ) == [49]
    with pytest.raises(ValueError, match="no C\\+\\+ extractor id"):
        export.resolve_cpp_feature_ids(["turb_mad_over_mean_aggr"])
    with pytest.raises(ValueError, match="no C\\+\\+ extractor id"):
        export.resolve_cpp_feature_ids(["chan_shape_scale_curvature"])


def test_training_default_is_the_promoted_subband_production_set():
    from csi_features import DEFAULT_FEATURES

    assert feature_cache.TRAINING_FEATURES == DEFAULT_FEATURES


def test_in_memory_gate_result_uses_training_metrics():
    paired = {"by_chip": {"C3": {}}, "pass_count": 1}
    quiet = {"passed": True}
    occupancy_paired = {"by_chip": {"C3": {}}, "pass_count": 1}
    occupancy_quiet = {"passed": True}

    result = evaluation.in_memory_gate_result(
        {
            "paired": paired,
            "quiet": quiet,
            "occupancy_paired": occupancy_paired,
            "occupancy_quiet": occupancy_quiet,
        }
    )

    assert result.paired_returncode == 0
    assert result.paired_metrics is paired
    assert result.quiet_metrics is quiet
    assert result.occupancy_paired_metrics is occupancy_paired
    assert result.occupancy_quiet_metrics is occupancy_quiet
    assert result.occupancy_passed
    assert result.passed


def test_occupancy_thinning_reduces_admitted_count_deterministically():
    import tools.lib.occupancy_thinning as occupancy_thinning
    from temporal_csi_sampler import (
        MINIMUM_COVERAGE_DENOMINATOR,
        MINIMUM_COVERAGE_NUMERATOR,
    )

    packets = _synthetic_packets(count=400)
    expected_gate_percent = (
        100 * MINIMUM_COVERAGE_NUMERATOR // MINIMUM_COVERAGE_DENOMINATOR
    )
    assert occupancy_thinning.OCCUPANCY_GATE_PERCENT == expected_gate_percent
    first, keep_ratio, seed = occupancy_thinning.thin_to_occupancy(
        packets,
        occupancy_percent=occupancy_thinning.OCCUPANCY_GATE_PERCENT,
        dataset_id="sample.npz",
    )
    second, second_ratio, second_seed = occupancy_thinning.thin_to_occupancy(
        packets,
        occupancy_percent=occupancy_thinning.OCCUPANCY_GATE_PERCENT,
        dataset_id="sample.npz",
    )

    assert 0.0 < keep_ratio <= 1.0
    assert keep_ratio == pytest.approx(second_ratio)
    assert seed == second_seed
    assert 0 < len(first) < len(packets)
    assert len(first) == len(second)
    assert [packet["device_ticks_us"] for packet in first] == [
        packet["device_ticks_us"] for packet in second
    ]
    assert occupancy_thinning.mean_window_occupancy(first) == pytest.approx(
        expected_gate_percent / 100.0
    )


def test_in_memory_gate_result_requires_occupancy_pass():
    result = evaluation.in_memory_gate_result(
        {
            "paired": {"by_chip": {"C3": {}}, "pass_count": 1},
            "quiet": {"passed": True},
        }
    )

    assert result.paired_returncode == 0
    assert not result.occupancy_passed
    assert not result.passed


def test_candidate_gain_stress_respects_deployment_roles(monkeypatch):
    captured_roles = []

    def fake_load_training_matrix(**kwargs):
        captured_roles.append(kwargs["dataset_roles"])
        return (
            {
                "X": np.asarray([[0.25]], dtype=np.float32),
                "y": np.asarray([0], dtype=np.int8),
                "feature_names": ["turb_mad_over_mean_aggr"],
                "sample_context": {},
                "stats": {"chips": ["C3"]},
            },
            None,
        )

    monkeypatch.setattr(evaluation, "load_training_matrix", fake_load_training_matrix)
    monkeypatch.setattr(
        evaluation,
        "get_preprocessor_arrays",
        lambda scaler: (np.asarray([0.0]), np.asarray([1.0])),
    )
    monkeypatch.setattr(evaluation, "_layer_arrays_from_model", lambda model: [])
    monkeypatch.setattr(
        evaluation,
        "_batch_predict_probabilities",
        lambda features, center, scale, layers: np.asarray([0.0]),
    )

    evaluation.evaluate_candidate_gain_stress(
        object(),
        object(),
        ["turb_mad_over_mean_aggr"],
        dataset_roles=("selection",),
        scales=(1.0,),
    )

    assert captured_roles == [("selection",)]


def test_main_rejects_plain_host_only_export(monkeypatch, capsys):
    monkeypatch.setattr(
        "sys.argv",
        ["train_ml_model.py", "--features", "turb_mad_over_mean_aggr"],
    )

    assert training_cli.main() == 1
    assert "cannot be exported" in capsys.readouterr().out


def test_train_all_rejects_host_only_export_before_training(capsys):
    returncode, used_seed, metrics = training.train_all(
        seed=7,
        feature_names=["turb_mad_over_mean_aggr"],
        export_artifacts=True,
    )

    assert returncode == 1
    assert used_seed == 7
    assert metrics is None
    assert "requires a C++ extractor id" in capsys.readouterr().out


def test_main_allows_production_augmentation_for_shap(monkeypatch):
    captured = {}

    def fake_train_all(**kwargs):
        captured.update(kwargs)
        return 0, kwargs["seed"], {}

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "train_ml_model.py",
            "--shap",
            "6",
            "--seed",
            "1049082371",
            "--augment",
            "--no-export",
        ],
    )
    monkeypatch.setattr(training_cli, "train_all", fake_train_all)

    assert training_cli.main() == 0
    assert captured["feature_importance"] is True
    assert captured["shap_samples"] == 6
    assert captured["augment"] == ("base", "drift", "burst-loss")
    assert captured["export_artifacts"] is False


def test_main_evaluate_selection_keeps_holdout_sealed(monkeypatch):
    captured = {}

    def fake_train_all(**kwargs):
        captured.update(kwargs)
        return 0, kwargs["seed"], {}

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "train_ml_model.py",
            "--augment",
            "--seed",
            "1049082371",
            "--evaluate-selection",
        ],
    )
    monkeypatch.setattr(training_cli, "train_all", fake_train_all)

    assert training_cli.main() == 0
    assert captured["export_artifacts"] is False
    assert captured["evaluate_deployment"] is True
    assert captured["deployment_roles"] == ("selection",)


def test_main_rejects_both_gate_evaluation_modes(monkeypatch, capsys):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "train_ml_model.py",
            "--evaluate-selection",
            "--evaluate-gates",
        ],
    )

    assert training_cli.main() == 1
    assert "mutually exclusive" in capsys.readouterr().out


def test_cross_validate_shap_uses_clean_background_with_packet_augmentation(
    monkeypatch,
):
    class FixedStratifiedGroupKFold:
        def __init__(self, n_splits, **kwargs):
            self.n_splits = n_splits

        def split(self, values, labels, groups):
            unique_groups = np.unique(groups)
            for fold_groups in np.array_split(unique_groups, self.n_splits):
                validation = np.flatnonzero(np.isin(groups, fold_groups))
                training = np.flatnonzero(~np.isin(groups, fold_groups))
                yield training, validation

    class IdentityScaler:
        def transform(self, values):
            return np.asarray(values, dtype=np.float32)

    train_sizes = []
    background_sizes = []

    sklearn_module = types.ModuleType("sklearn")
    model_selection_module = types.ModuleType("sklearn.model_selection")
    model_selection_module.StratifiedGroupKFold = FixedStratifiedGroupKFold
    sklearn_module.model_selection = model_selection_module
    monkeypatch.setitem(sys.modules, "sklearn", sklearn_module)
    monkeypatch.setitem(
        sys.modules, "sklearn.model_selection", model_selection_module
    )
    monkeypatch.setitem(sys.modules, "shap", types.ModuleType("shap"))
    monkeypatch.setattr(training, "build_preprocessor", lambda mode: IdentityScaler())
    monkeypatch.setattr(training, "fit_preprocessor", lambda *args, **kwargs: None)

    def fake_train_model(values, labels, **kwargs):
        train_sizes.append(len(values))
        assert len(values) == len(labels)
        return object()

    monkeypatch.setattr(training, "train_model", fake_train_model)
    monkeypatch.setattr(
        training,
        "predict_probabilities",
        lambda model, values: np.full(len(values), 0.4, dtype=np.float32),
    )

    def fake_shap(model, background, explained, **kwargs):
        background_sizes.append(len(background))
        return np.zeros((len(explained), explained.shape[1]), dtype=np.float32)

    monkeypatch.setattr(training, "calculate_shap_values", fake_shap)

    X = np.arange(24, dtype=np.float32).reshape(12, 2)
    y = np.tile(np.asarray([0, 1], dtype=np.int8), 6)
    groups = np.repeat(np.asarray([f"g{i}" for i in range(6)]), 2)
    context = {
        "chip": np.repeat("C3", 12),
        "environment_group": np.repeat("bedroom", 12),
        "lineage_group": groups,
        "session_group": groups,
        "source_file": groups,
    }

    result = training.cross_validate(
        X,
        y,
        n_folds=3,
        groups=groups,
        sample_context=context,
        block_stride=1,
        seed=7,
        shap_samples=3,
        shap_feature_names=["a", "b"],
        shap_seed=11,
        X_aug=X + 0.5,
        y_aug=y,
        groups_aug=groups,
    )

    assert train_sizes == [16, 16, 16]
    assert background_sizes == [8, 8, 8]
    assert result["shap_samples"] == 3


def test_feature_ablation_removes_column_from_clean_and_augmented_rows():
    dataset = {
        "X": np.arange(12, dtype=np.float32).reshape(4, 3),
        "X_aug": np.arange(15, dtype=np.float32).reshape(5, 3),
        "feature_names": ["first", "removed", "last"],
    }

    candidate = training.build_feature_ablation_dataset(dataset, "removed")

    assert candidate["feature_names"] == ["first", "last"]
    np.testing.assert_array_equal(candidate["X"], dataset["X"][:, [0, 2]])
    np.testing.assert_array_equal(
        candidate["X_aug"], dataset["X_aug"][:, [0, 2]]
    )

    joint = training.build_feature_ablation_dataset(dataset, "first+last")
    assert joint["feature_names"] == ["removed"]
    np.testing.assert_array_equal(joint["X"], dataset["X"][:, [1]])
    np.testing.assert_array_equal(joint["X_aug"], dataset["X_aug"][:, [1]])


def test_main_passes_production_augmentation_to_multi_feature_ablation(
    monkeypatch,
):
    captured = {}

    def fake_ablation(feature_name, **kwargs):
        captured["feature_name"] = feature_name
        captured.update(kwargs)
        return 0

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "train_ml_model.py",
            "--ablation-feature",
            "chan_coh_subband_gap_median,chan_freq_coh_cv",
            "--seed",
            "1049082371",
            "--augment",
        ],
    )
    monkeypatch.setattr(training_cli, "experiment_feature_ablation", fake_ablation)

    assert training_cli.main() == 0
    assert captured["feature_name"] == (
        "chan_coh_subband_gap_median,chan_freq_coh_cv"
    )
    assert captured["augment"] == ("base", "drift", "burst-loss")


def test_host_only_seed_search_keeps_candidates_in_memory(monkeypatch):
    calls = []
    base_metrics = {
        "oof_f1": 97.0,
        "group_reports": {
            "session_group": {
                "worst_recall": {"recall": 90.0},
                "worst_fp_rate": {"fp_rate": 5.0},
            },
            "chip": {"worst_recall": {"recall": 95.0}},
        },
    }
    failed_paired = {
        "by_chip": {"C3": {}},
        "pass_count": 0,
        "max_fp_rate": 10.0,
        "worst_chip_recall": 90.0,
    }

    def fake_train_all(*, seed, export_artifacts, evaluate_deployment=False, **kwargs):
        calls.append(
            (
                seed,
                export_artifacts,
                evaluate_deployment,
                tuple(kwargs["feature_names"]),
                tuple(kwargs["augment"]),
            )
        )
        metrics = dict(base_metrics)
        if evaluate_deployment:
            metrics["paired"] = failed_paired
            metrics["quiet"] = {"passed": True}
        return 0, seed, metrics

    passing_baseline = evaluation.ExportedMLGateResult(
        paired_returncode=0,
        paired_output="",
        paired_metrics={"by_chip": {"C3": {}}, "pass_count": 1},
        quiet_metrics={"passed": True},
        occupancy_paired_metrics={"by_chip": {"C3": {}}, "pass_count": 1},
        occupancy_quiet_metrics={"passed": True},
    )
    unavailable_holdout = evaluation.ExportedMLGateResult(1, "")
    gate_results = iter((passing_baseline, unavailable_holdout))

    monkeypatch.setattr(training, "ensure_torch_available", lambda: None)
    monkeypatch.setattr(training, "describe_torch_device", lambda: "cpu")
    monkeypatch.setattr(training, "read_exported_seed", lambda: 7)
    monkeypatch.setattr(training, "generate_random_training_seed", lambda: 9)
    monkeypatch.setattr(training, "train_all", fake_train_all)
    monkeypatch.setattr(training, "run_exported_ml_gates", lambda **kwargs: next(gate_results))
    monkeypatch.setattr(
        training,
        "_format_candidate_comparison",
        lambda candidate, baseline: ({"regressions": []}, "tie"),
    )
    monkeypatch.setattr(training, "_candidate_beats_baseline", lambda *args: False)
    monkeypatch.setattr(
        training,
        "_backup_artifacts",
        lambda: pytest.fail("host-only seed search must not back up artifacts"),
    )

    result = training.train_until_improvement(
        1,
        feature_names=["turb_mad_over_mean_aggr"],
        augment="base,drift,burst-loss",
        search_output_path=None,
    )

    assert result == 1
    assert calls == [
        (
            7,
            False,
            False,
            tuple(feature_cache.DEFAULT_FEATURES),
            ("base", "drift", "burst-loss"),
        ),
        (
            9,
            False,
            True,
            ("turb_mad_over_mean_aggr",),
            ("base", "drift", "burst-loss"),
        ),
    ]


def test_main_passes_no_export_to_seed_search(monkeypatch):
    captured = {}

    def fake_seed_search(max_trials, **kwargs):
        captured["max_trials"] = max_trials
        captured.update(kwargs)
        return 0

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "train_ml_model.py",
            "--seed-search-until-improvement",
            "3",
            "--augment",
            "--no-export",
        ],
    )
    monkeypatch.setattr(training_cli, "train_until_improvement", fake_seed_search)

    assert training_cli.main() == 0
    assert captured["max_trials"] == 3
    assert captured["augment"] == ("base", "drift", "burst-loss")
    assert captured["export_artifacts"] is False


def test_packet_rate_estimate_uses_effective_throughput_for_bursty_capture():
    packets = _synthetic_packets(count=101)
    timestamp_us = 0
    for index, packet in enumerate(packets):
        if index:
            timestamp_us += 91_000 if index % 10 == 0 else 1_000
        packet["device_ticks_us"] = timestamp_us

    assert augmentation._estimate_packet_rate_pps(packets) == pytest.approx(100.0)


def test_stable_rate_augmentation_reduces_the_temporal_window_sample_count():
    packets = _synthetic_packets(count=400)

    augmented = augmentation.augment_csi_packets(
        packets,
        {"packet_rate_scale": (0.8, 0.8)},
        seed=17,
    )

    interval_us = dataset.measure_packet_interval_us(augmented)
    timing = evaluation.derive_detector_timing(
        interval_us,
        feature_cache.SEGMENTATION_WINDOW_SIZE_MS,
    )
    assert interval_us == 12_500
    assert timing["window_packets"] == 80


def test_stable_rate_augmentation_reaches_the_seventy_pps_floor():
    packets = _synthetic_packets(count=400)

    augmented = augmentation.augment_csi_packets(
        packets,
        {"packet_rate_scale": (0.7, 0.7)},
        seed=17,
    )

    interval_us = dataset.measure_packet_interval_us(augmented)
    timing = evaluation.derive_detector_timing(
        interval_us,
        feature_cache.SEGMENTATION_WINDOW_SIZE_MS,
    )
    assert interval_us == 14_286
    assert timing["window_packets"] == 70


def test_drift_augmentation_is_deterministic_and_count_preserving():
    packets = _synthetic_packets()
    config = {
        "drift_sigma": 0.25,
        "drift_episode_count": 1,
        "drift_duration_seconds": (1.0, 1.0),
    }

    first = augmentation.augment_csi_packets(packets, config, seed=7)
    second = augmentation.augment_csi_packets(packets, config, seed=7)

    assert len(first) == len(packets)
    assert len(second) == len(packets)
    assert any(
        np.any(first_packet["csi_data"] != original_packet["csi_data"])
        for first_packet, original_packet in zip(first, packets, strict=True)
    )
    for first_packet, second_packet in zip(first, second, strict=True):
        np.testing.assert_array_equal(first_packet["csi_data"], second_packet["csi_data"])


def test_burst_loss_augmentation_is_deterministic_and_drops_packets():
    packets = _synthetic_packets()
    config = {
        "burst_loss_starts_per_minute": 120.0,
        "burst_length_packets": (2, 2),
    }

    first = augmentation.augment_csi_packets(packets, config, seed=11)
    second = augmentation.augment_csi_packets(packets, config, seed=11)

    assert 0 < len(first) < len(packets)
    assert len(second) == len(first)
    for first_packet, second_packet in zip(first, second, strict=True):
        np.testing.assert_array_equal(first_packet["csi_data"], second_packet["csi_data"])
