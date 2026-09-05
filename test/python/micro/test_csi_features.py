# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Feature Extraction Tests

Unit tests for shared feature extraction helpers.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

import pytest
import math
import numpy as np
from csi_features import (
    calc_autocorrelation,
    calc_zero_crossing_rate,
    extract_features_by_name,
    ALL_FEATURES,
    DEFAULT_FEATURES,
    FEATURE_NAMES,
)


def _stats(values, count=None):
    """Helper: compute (count, mean, std) for a list of values."""
    if count is None:
        count = len(values)
    if count == 0:
        return count, 0.0, 0.0
    mean = sum(values[:count]) / count
    var = sum((values[i] - mean) ** 2 for i in range(count)) / count
    std = math.sqrt(var) if var > 0 else 0.0
    return count, mean, std


def _promoted_tracker_kwargs():
    return {
        "chan_shape_spread_subband": 0.4,
        "chan_shape_coherent_innovation_energy": 0.03,
        "chan_shape_excess_path": 0.02,
        "chan_shape_subband_kendall_lag_excess": 0.05,
    }


class TestCalcAutocorrelation:
    """Test lag-1 autocorrelation calculation"""
    
    def test_empty_buffer(self):
        """Test autocorrelation of empty buffer"""
        assert calc_autocorrelation([], 0) == 0.0
    
    def test_two_values(self):
        """Test autocorrelation of two values (needs 3+)"""
        assert calc_autocorrelation([1.0, 2.0], 2) == 0.0
    
    def test_constant_values(self):
        """Test autocorrelation of constant values"""
        buffer = [5.0] * 10
        ac = calc_autocorrelation(buffer, 10)
        assert ac == 0.0  # Variance is 0
    
    def test_highly_correlated_signal(self):
        """Test that smooth signal has high autocorrelation"""
        # Slow sinusoid -> high autocorrelation
        buffer = [math.sin(i * 0.1) for i in range(50)]
        ac = calc_autocorrelation(buffer, 50)
        assert ac > 0.9  # Very high correlation
    
    def test_random_signal_low_autocorrelation(self):
        """Test that random signal has low autocorrelation"""
        np.random.seed(42)
        buffer = list(np.random.normal(0, 1, 100))
        ac = calc_autocorrelation(buffer, 100)
        # Random noise should have low autocorrelation
        assert abs(ac) < 0.3
    
    def test_output_range(self):
        """Test that autocorrelation is in [-1, 1]"""
        np.random.seed(42)
        buffer = list(np.random.normal(5, 2, 50))
        ac = calc_autocorrelation(buffer, 50)
        assert -1.0 <= ac <= 1.0


class TestExtractAllFeatures:
    """Test full feature extraction"""
    
    def test_returns_default_feature_count(self):
        """Test that the default feature count is returned"""
        buffer = [float(i) for i in range(50)]
        features = extract_features_by_name(
            buffer, 50, feature_names=DEFAULT_FEATURES,
            aggregated_turbulence_buffer=list(buffer),
            l1_delta_lag_ratio=1.0,
            **_promoted_tracker_kwargs(),
        )
        assert len(features) == len(DEFAULT_FEATURES)
    
    def test_empty_buffer_returns_zeros(self):
        """Test that empty buffer returns zeros"""
        features = extract_features_by_name(
            [], 0, feature_names=DEFAULT_FEATURES, l1_delta_lag_ratio=0.0,
            aggregated_turbulence_buffer=[],
            **_promoted_tracker_kwargs(),
        )
        assert features == [0.0] * len(DEFAULT_FEATURES)
    
    def test_single_value_returns_zeros(self):
        """Test that single-value buffer returns zeros"""
        features = extract_features_by_name(
            [5.0], 1, feature_names=DEFAULT_FEATURES, l1_delta_lag_ratio=0.0,
            aggregated_turbulence_buffer=[5.0],
            **_promoted_tracker_kwargs(),
        )
        assert features == [0.0] * len(DEFAULT_FEATURES)
    
    def test_feature_names_match(self):
        """Test that FEATURE_NAMES matches DEFAULT_FEATURES."""
        assert len(FEATURE_NAMES) == len(DEFAULT_FEATURES)
        assert FEATURE_NAMES == DEFAULT_FEATURES

    def test_runtime_surface_contains_only_current_inputs(self):
        """The runtime exposes only the promoted detector inputs."""
        assert set(ALL_FEATURES) == set(DEFAULT_FEATURES)
        assert DEFAULT_FEATURES == [
            'turb_iqr_over_mean_aggr',
            'turb_autocorr',
            'turb_zcr',
            'l1_delta_lag_ratio',
            'chan_shape_spread_subband',
            'chan_shape_coherent_innovation_energy',
            'chan_shape_excess_path',
            'chan_shape_subband_kendall_lag_excess',
        ]

    def test_unknown_feature_raises(self):
        """Unknown feature names are rejected."""
        buffer = [float(i) for i in range(50)]
        with pytest.raises(ValueError, match="Unknown feature"):
            extract_features_by_name(buffer, 50, feature_names=['not_a_feature'])
    
    def test_all_features_are_float(self):
        """Test that all features are floats"""
        np.random.seed(42)
        buffer = list(np.random.normal(5, 2, 50))
        features = extract_features_by_name(
            buffer, 50, feature_names=DEFAULT_FEATURES,
            aggregated_turbulence_buffer=list(buffer),
            l1_delta_lag_ratio=1.0,
            **_promoted_tracker_kwargs(),
        )
        for i, f in enumerate(features):
            assert isinstance(f, (int, float)), f"Feature {i} ({FEATURE_NAMES[i]}) is {type(f)}"
    
    def test_motion_vs_idle_features_differ(self):
        """Test that motion-like and idle-like buffers produce different features"""
        # Idle-like: low variance, stable signal
        idle_buffer = [5.0 + 0.01 * (i % 3) for i in range(50)]
        # Motion-like: high variance, turbulent signal
        np.random.seed(42)
        motion_buffer = list(np.random.normal(5, 3, 50))
        
        idle_features = extract_features_by_name(
            idle_buffer, 50, feature_names=DEFAULT_FEATURES,
            aggregated_turbulence_buffer=list(idle_buffer),
            l1_delta_lag_ratio=1.0,
            **_promoted_tracker_kwargs(),
        )
        motion_features = extract_features_by_name(
            motion_buffer, 50, feature_names=DEFAULT_FEATURES,
            aggregated_turbulence_buffer=list(motion_buffer),
            l1_delta_lag_ratio=2.0,
            **_promoted_tracker_kwargs(),
        )

        # Aggregated IQR is the production robust-dispersion feature and rises
        # with turbulence, so motion must exceed idle.
        iqr_idx = FEATURE_NAMES.index('turb_iqr_over_mean_aggr')
        assert motion_features[iqr_idx] > idle_features[iqr_idx]
        assert motion_features != idle_features

class TestCalcZeroCrossingRate:
    """Test the median-crossing rate helper"""

    def test_short_buffer_returns_zero(self):
        assert calc_zero_crossing_rate([1.0], 1, 0.0) == 0.0

    def test_alternating_signal_crosses_every_sample(self):
        values = [1.0, -1.0] * 25
        assert calc_zero_crossing_rate(values, len(values), 0.0) == 1.0

    def test_single_excursion_crosses_twice(self):
        values = [0.0] * 20 + [5.0] * 10 + [0.0] * 20
        rate = calc_zero_crossing_rate(values, len(values), 2.5)
        assert rate == pytest.approx(2 / 49)

    def test_shift_and_scale_invariance_with_median_center(self):
        np.random.seed(7)
        base = list(np.random.normal(0.0, 1.0, 60))
        transformed = [0.02 * v + 10.0 for v in base]
        base_median = sorted(base)[30]
        transformed_median = sorted(transformed)[30]
        assert calc_zero_crossing_rate(base, 60, base_median) == pytest.approx(
            calc_zero_crossing_rate(transformed, 60, transformed_median)
        )


class TestFeatureSemantics:
    """Test feature semantics and invariance contracts."""

    def test_turb_zcr_separates_noise_from_coherent_excursions(self):
        np.random.seed(11)
        noise = list(np.random.normal(5.0, 1.0, 50))
        excursion = [5.0] * 20 + [9.0 + 0.01 * i for i in range(15)] + [5.0] * 15
        noise_zcr = extract_features_by_name(noise, 50, feature_names=['turb_zcr'])[0]
        excursion_zcr = extract_features_by_name(excursion, 50, feature_names=['turb_zcr'])[0]
        assert noise_zcr > excursion_zcr

    def test_aggregated_iqr_matches_linear_percentiles(self):
        turbulence = [5.0] * 4
        aggregated = [1.0, 2.0, 4.0, 8.0]
        value = extract_features_by_name(
            turbulence,
            len(turbulence),
            feature_names=['turb_iqr_over_mean_aggr'],
            aggregated_turbulence_buffer=aggregated,
        )[0]
        q25, q75 = np.percentile(aggregated, [25, 75])
        assert value == pytest.approx((q75 - q25) / np.mean(aggregated))

    def test_every_production_feature_is_scale_invariant(self):
        """The reason the production set is what it is.

        The per-packet CSI scaling factor is never recorded, so a feature that
        carries absolute magnitude carries the link's noise floor with it. On
        weak links that floor can exceed the motion it is meant to measure,
        which is how l1_delta and l1_delta_std took a weak pair from 0% to
        100% false positives on 2026-07-27.
        """
        np.random.seed(13)
        turb = [5.0 + 0.1 * (i % 5) for i in range(50)]

        base = extract_features_by_name(
            turb, 50, feature_names=DEFAULT_FEATURES,
            aggregated_turbulence_buffer=list(turb),
            l1_delta_lag_ratio=1.4,
            **_promoted_tracker_kwargs())
        boosted = extract_features_by_name(
            [v * 10.0 for v in turb], 50, feature_names=DEFAULT_FEATURES,
            aggregated_turbulence_buffer=[v * 10.0 for v in turb],
            l1_delta_lag_ratio=1.4,
            **_promoted_tracker_kwargs())

        for name, before, after in zip(DEFAULT_FEATURES, base, boosted, strict=True):
            assert after == pytest.approx(before, abs=1e-9), (
                f"{name} moved when both streams were scaled by 10")

    def test_l1_delta_lag_ratio_uses_preprocessed_tracker_metric(self):
        turb = [5.0 + 0.1 * (i % 5) for i in range(50)]
        value = extract_features_by_name(
            turb,
            50,
            feature_names=['l1_delta_lag_ratio'],
            l1_delta_lag_ratio=1.75,
        )[0]

        assert value == pytest.approx(1.75)

    def test_l1_delta_lag_ratio_requires_preprocessed_tracker_metric(self):
        turb = [5.0 + 0.1 * (i % 5) for i in range(50)]

        with pytest.raises(ValueError, match="l1_delta_lag_ratio is required"):
            extract_features_by_name(
                turb,
                50,
                feature_names=['l1_delta_lag_ratio'],
            )


    def test_missing_slots_are_excluded_from_turbulence_statistics(self):
        turbulence = [1.0] * 5
        aggregated = [1.0, 0.0, 3.0, 0.0, 5.0]
        validity = [True, False, True, False, True]
        skipped = extract_features_by_name(
            turbulence,
            len(turbulence),
            feature_names=['turb_iqr_over_mean_aggr'],
            aggregated_turbulence_buffer=aggregated,
            aggregated_turbulence_validity=validity,
        )[0]
        compact = extract_features_by_name(
            turbulence[:3],
            3,
            feature_names=['turb_iqr_over_mean_aggr'],
            aggregated_turbulence_buffer=[1.0, 3.0, 5.0],
        )[0]
        treated_as_samples = extract_features_by_name(
            turbulence,
            len(turbulence),
            feature_names=['turb_iqr_over_mean_aggr'],
            aggregated_turbulence_buffer=aggregated,
        )[0]

        assert skipped == pytest.approx(compact)
        assert treated_as_samples != pytest.approx(skipped)
