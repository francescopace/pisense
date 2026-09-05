# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - High-Accuracy Detector Tests

Tests for the ML motion detector module.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

import pytest
import numpy as np

from tools.lib.repo_paths import generated_data_dir
from tools.lib.host_feature_trackers import (
    ChannelShapeTrajectoryTracker as HostChannelShapeTrajectoryTracker,
)

from config import DEFAULT_SUBCARRIERS
import high_accuracy_detector as high_accuracy_detector_module
from high_accuracy_detector import (
    relu, sigmoid, predict, is_motion,
    HighAccuracyDetector, HIGH_ACCURACY_DEFAULT_THRESHOLD, HIGH_ACCURACY_METRIC_SCALE
)
from detector_interface import MotionState
from ml_feature_trackers import (
    CHANNEL_SHAPE_BIN_US,
    ChannelShapeTrajectoryTracker,
)
from segmentation import SegmentationContext
from csi_features import L1DeltaTracker

from ml_weights import FEATURE_MEAN, FEATURE_NAMES


MODEL_INPUT_SIZE = len(FEATURE_MEAN)


def _make_band_profile(offset=0):
    """Return a simple frequency-selective amplitude profile over model subcarriers."""
    return {
        sc: (20 + 3 * ((idx + offset) % 5))
        for idx, sc in enumerate(DEFAULT_SUBCARRIERS)
    }


def _make_csi_payload(profile):
    """Build a 64-subcarrier I/Q payload using the provided real amplitudes."""
    csi = [0] * 128
    for sc_idx, amplitude in profile.items():
        csi[sc_idx * 2] = 0
        csi[sc_idx * 2 + 1] = int(amplitude)
    return csi


def _make_trajectory_payload(step):
    profile = {
        sc: 18 + ((sc * 5 + step * step + step * (sc % 7)) % 70)
        for sc in range(64)
    }
    return _make_csi_payload(profile)


def test_shared_packet_frame_matches_direct_trajectory_tracker():
    direct_trajectory = ChannelShapeTrajectoryTracker()
    shared_trajectory = ChannelShapeTrajectoryTracker()
    frame = [0.0] * 64

    for step in range(12):
        payload = _make_trajectory_payload(step)
        timestamp_us = step * CHANNEL_SHAPE_BIN_US
        count = SegmentationContext.fill_subcarrier_energy_buffer(payload, frame)
        direct_trajectory.process_packet(payload, timestamp_us)
        shared_trajectory.process_packet(payload, timestamp_us, frame, count)

    direct_innovation, direct_excess, direct_spread = (
        direct_trajectory.trajectory_features_with_spread()
    )
    shared_innovation, shared_excess, shared_spread = (
        shared_trajectory.trajectory_features_with_spread()
    )
    assert shared_innovation == pytest.approx(direct_innovation, abs=1e-12)
    assert shared_excess == pytest.approx(direct_excess, abs=1e-12)
    assert shared_spread == pytest.approx(direct_spread, abs=1e-12)
    assert shared_trajectory.subband_kendall_lag_excess() == pytest.approx(
        direct_trajectory.subband_kendall_lag_excess(),
        abs=1e-12,
    )
    assert shared_trajectory.coherent_innovation_energy() == pytest.approx(
        shared_innovation,
        abs=1e-12,
    )
    assert shared_trajectory.excess_path() == pytest.approx(
        shared_excess,
        abs=1e-12,
    )
    assert shared_trajectory.shape_spread_subband() == pytest.approx(
        shared_spread,
        abs=1e-12,
    )


def test_cached_dct_trajectory_matches_host_dct_reference():
    runtime = ChannelShapeTrajectoryTracker()
    reference = HostChannelShapeTrajectoryTracker(
        track_subband_kendall_lag_excess=True,
    )
    for step in range(12):
        payload = _make_trajectory_payload(step)
        timestamp_us = step * CHANNEL_SHAPE_BIN_US
        runtime.process_packet(payload, timestamp_us)
        reference.process_packet(np.asarray(payload, dtype=np.int8), timestamp_us)

    runtime_innovation, runtime_excess, runtime_spread = (
        runtime.trajectory_features_with_spread()
    )
    reference_innovation, reference_excess, reference_spread = (
        reference.trajectory_features_with_spread()
    )
    assert runtime_innovation == pytest.approx(
        reference_innovation,
        abs=1e-10,
    )
    assert runtime_excess == pytest.approx(
        reference_excess,
        abs=1e-10,
    )
    assert runtime_spread == pytest.approx(reference_spread, abs=1e-10)
    assert runtime.subband_kendall_lag_excess() == pytest.approx(
        reference.subband_kendall_lag_excess(),
        abs=1e-10,
    )


def test_trajectory_duplicate_detection_accepts_signed_payloads():
    tracker = ChannelShapeTrajectoryTracker()
    payload = [(-128 + index * 17) % 256 - 128 for index in range(128)]

    tracker.process_packet(payload, 0)
    profile_count = tracker._current_profile_count
    tracker.process_packet(list(payload), CHANNEL_SHAPE_BIN_US // 2)

    assert tracker._current_profile_count == profile_count


@pytest.mark.parametrize("tracker_class", [
    ChannelShapeTrajectoryTracker, HostChannelShapeTrajectoryTracker,
])
def test_trajectory_duplicate_packets_expire_old_motion(tracker_class):
    tracker = (
        tracker_class(track_subband_kendall_lag_excess=True)
        if tracker_class is HostChannelShapeTrajectoryTracker
        else tracker_class()
    )
    for step in range(12):
        payload = _make_trajectory_payload(step)
        tracker.process_packet(payload, step * CHANNEL_SHAPE_BIN_US)
    assert tracker.shape_spread_subband() > 0.0

    for step in range(12, 40):
        tracker.process_packet(payload, step * CHANNEL_SHAPE_BIN_US)
    assert tracker.trajectory_features_with_spread() == pytest.approx((0.0, 0.0, 0.0))
    assert tracker.subband_kendall_lag_excess() == 0.0

    for step in range(40, 52):
        tracker.process_packet(_make_trajectory_payload(step), step * CHANNEL_SHAPE_BIN_US)
    assert tracker.shape_spread_subband() > 0.0


def test_l1_precomputed_mean_matches_default_normalization():
    direct = L1DeltaTracker(window_size=32, enable_hampel=False)
    shared_mean = L1DeltaTracker(window_size=32, enable_hampel=False)

    for packet in range(24):
        amplitudes = [
            1.0 + ((packet + 3 * tone) % 11)
            for tone in range(SegmentationContext.AMPLITUDE_BUFFER_SIZE)
        ]
        direct.process_amplitudes(amplitudes, len(amplitudes))
        shared_mean.process_amplitudes(
            amplitudes,
            len(amplitudes),
            sum(amplitudes) / len(amplitudes),
        )

    assert shared_mean.delta_lag_ratio() == direct.delta_lag_ratio()
    assert shared_mean._delta_ring == direct._delta_ring


class TestRelu:
    """Test ReLU activation function."""
    
    def test_positive_input(self):
        """Positive values pass through unchanged."""
        assert relu(5.0) == 5.0
        assert relu(0.1) == 0.1
        assert relu(100.0) == 100.0
    
    def test_negative_input(self):
        """Negative values return 0."""
        assert relu(-5.0) == 0.0
        assert relu(-0.1) == 0.0
        assert relu(-100.0) == 0.0
    
    def test_zero_input(self):
        """Zero returns zero."""
        assert relu(0.0) == 0.0


class TestSigmoid:
    """Test Sigmoid activation function."""
    
    def test_zero_input(self):
        """Sigmoid(0) = 0.5."""
        assert sigmoid(0.0) == 0.5
    
    def test_positive_input(self):
        """Positive values return > 0.5."""
        assert sigmoid(1.0) > 0.5
        assert sigmoid(5.0) > 0.9
    
    def test_negative_input(self):
        """Negative values return < 0.5."""
        assert sigmoid(-1.0) < 0.5
        assert sigmoid(-5.0) < 0.1
    
    def test_large_positive_overflow_protection(self):
        """Large positive values return 1.0 (overflow protection)."""
        assert sigmoid(100.0) == 1.0
        assert sigmoid(21.0) == 1.0
    
    def test_large_negative_overflow_protection(self):
        """Large negative values return 0.0 (overflow protection)."""
        assert sigmoid(-100.0) == 0.0
        assert sigmoid(-21.0) == 0.0
    
    def test_output_range(self):
        """Output is always in (0, 1)."""
        for x in [-10, -5, -1, 0, 1, 5, 10]:
            result = sigmoid(x)
            assert 0.0 <= result <= 1.0


class TestPredict:
    """Test neural network prediction."""
    
    def test_predict_returns_float(self):
        """Predict returns a float."""
        features = ([14.0, 2.0, 17.0, 9.0, 0.30,
                     -1.5, 8.0, 2.0, 0.15, 0.7, 0.001, 0.0][:MODEL_INPUT_SIZE]
                    + [0.0] * max(0, MODEL_INPUT_SIZE - 12))
        result = predict(features)
        assert isinstance(result, float)
    
    def test_predict_output_range(self):
        """Prediction is always in [0, 1]."""
        # Test with various feature combinations
        test_cases = [
            [0.0] * MODEL_INPUT_SIZE,  # All zeros
            [10.0] * MODEL_INPUT_SIZE,  # All same value
            [float(i) for i in range(1, MODEL_INPUT_SIZE + 1)],  # Increasing
        ]
        for features in test_cases:
            result = predict(features)
            assert 0.0 <= result <= HIGH_ACCURACY_METRIC_SCALE
    
    def test_predict_different_inputs_different_outputs(self):
        """Different inputs produce different outputs."""
        # Use real reference samples from the current exported model data.
        # Pick the pair with the minimum and maximum exported outputs rather
        # than assuming the first two samples should differ after retraining.
        test_data_path = generated_data_dir() / "ml_test_data.npz"
        if not test_data_path.exists():
            pytest.skip(f"Test data not found: {test_data_path}")

        test_data = np.load(test_data_path)
        features = test_data["features"]
        expected_outputs = test_data["expected_outputs"]

        min_idx = int(np.argmin(expected_outputs))
        max_idx = int(np.argmax(expected_outputs))
        if min_idx == max_idx:
            pytest.skip("Reference outputs are constant across exported test data")

        features1 = features[min_idx].tolist()
        features2 = features[max_idx].tolist()

        result1 = predict(features1)
        result2 = predict(features2)

        assert features1 != features2
        assert result1 != result2


class TestIsMotion:
    """Test motion detection function."""
    
    def test_is_motion_returns_bool(self):
        """is_motion returns a boolean."""
        features = [5.0] * MODEL_INPUT_SIZE
        result = is_motion(features)
        assert isinstance(result, bool)
    
    def test_is_motion_default_threshold(self):
        """Default threshold is 0.5."""
        features = [5.0] * MODEL_INPUT_SIZE
        prob = predict(features)
        expected = prob > HIGH_ACCURACY_DEFAULT_THRESHOLD
        assert is_motion(features) == expected
    
    def test_is_motion_custom_threshold(self):
        """Custom threshold works correctly."""
        features = [5.0] * MODEL_INPUT_SIZE
        prob = predict(features)
        
        # With threshold above probability, should be False
        assert not is_motion(features, threshold=prob + 0.1)
        # With threshold below probability, should be True
        if prob > 0.01:
            assert is_motion(features, threshold=prob - 0.01)


class TestHighAccuracyDetector:
    """Test HighAccuracyDetector class."""
    
    def test_initialization_defaults(self):
        """Test default initialization."""
        detector = HighAccuracyDetector()
        assert detector.ALGORITHM == "high_accuracy"
        assert detector._threshold == HIGH_ACCURACY_DEFAULT_THRESHOLD
        assert detector._state == MotionState.IDLE
        assert detector._packet_count == 0
        assert not detector.track_data
    
    def test_hampel_enabled_by_default(self):
        """Hampel filter is enabled by default (matches training pipeline)."""
        detector = HighAccuracyDetector()
        assert detector._context.hampel_filter is not None

    def test_lowpass_disabled_by_default(self):
        """Low-pass filter is disabled by default."""
        detector = HighAccuracyDetector()
        assert detector._context.lowpass_filter is None
    
    def test_hampel_disabled_explicitly(self):
        """Hampel filter can be disabled explicitly."""
        detector = HighAccuracyDetector(enable_hampel=False)
        assert detector._context.hampel_filter is None
    
    def test_initialization_custom_params(self):
        """Test initialization with custom parameters."""
        detector = HighAccuracyDetector(window_size=100, threshold=0.7)
        assert detector._threshold == 0.7
        assert detector._context.window_size == 100

    def test_l1_history_uses_streaming_delta_window(self):
        detector = HighAccuracyDetector(window_size=100)

        assert detector._l1_tracker is not None
        assert len(detector._l1_tracker._profile_ring) == 10
        assert len(detector._l1_tracker._delta_ring) == 90
    
    def test_get_name(self):
        """Test get_name returns the product-facing detector name."""
        detector = HighAccuracyDetector()
        assert detector.get_name() == "High Accuracy"
    
    def test_get_state_initial(self):
        """Initial state is IDLE."""
        detector = HighAccuracyDetector()
        assert detector.get_state() == MotionState.IDLE
    
    def test_get_threshold(self):
        """Test get_threshold."""
        detector = HighAccuracyDetector(threshold=0.6)
        assert detector.get_threshold() == 0.6
    
    def test_set_threshold_valid(self):
        """Test setting valid threshold."""
        detector = HighAccuracyDetector()
        assert detector.set_threshold(0.7)
        assert detector._threshold == 0.7
    
    def test_set_threshold_invalid(self):
        """Test setting invalid threshold."""
        detector = HighAccuracyDetector()
        original = detector._threshold
        assert not detector.set_threshold(1.1)
        assert not detector.set_threshold(-0.1)
        assert detector._threshold == original

    def test_detector_initializes_segmentation_context(self):
        """High-Accuracy detector initializes its segmentation context."""
        detector = HighAccuracyDetector()
        assert detector._context is not None
    
    def test_is_ready_empty(self):
        """Detector is not ready before filling buffer."""
        detector = HighAccuracyDetector(window_size=50)
        assert not detector.is_ready()
    
    def test_get_motion_metric_initial(self):
        """Initial motion metric is 0."""
        detector = HighAccuracyDetector()
        assert detector.get_motion_metric() == 0.0

    def test_update_state_exposes_common_motion_metric(self):
        """Update state returns the common motion_metric alias."""
        detector = HighAccuracyDetector(window_size=5)
        csi_data = [10, 10] * 64

        for _ in range(6):
            detector.process_packet(csi_data, DEFAULT_SUBCARRIERS)

        metrics = detector.update_state()
        assert 'motion_metric' in metrics
        assert metrics['motion_metric'] == metrics['probability']
    
    def test_total_packets_initial(self):
        """Initial packet count is 0."""
        detector = HighAccuracyDetector()
        assert detector.total_packets == 0
    
    def test_reset(self):
        """Test reset clears state."""
        detector = HighAccuracyDetector()
        detector._packet_count = 100
        detector._state = MotionState.MOTION
        detector._current_probability = 0.8
        detector._motion_count = 10
        detector.probability_history = [0.5, 0.6]
        detector.state_history = ['IDLE', 'MOTION']
        
        detector.reset()
        
        assert detector._state == MotionState.IDLE
        assert detector._current_probability == 0.0
        assert detector._motion_count == 0
        assert detector.probability_history == []
        assert detector.state_history == []
        assert detector.total_packets == 0


class TestHighAccuracyDetectorProcessing:
    """Test HighAccuracyDetector packet processing with synthetic data."""
    
    @pytest.fixture
    def detector(self):
        """Create a detector with small window for testing."""
        return HighAccuracyDetector(window_size=10, threshold=HIGH_ACCURACY_DEFAULT_THRESHOLD)
    
    @pytest.fixture
    def sample_csi_data(self):
        """Create sample CSI data (64 subcarriers * 2 = 128 bytes)."""
        # Espressif CSI format: [Imaginary, Real, ...] per subcarrier
        data = []
        for i in range(64):  # 64 subcarriers
            data.append(10 + i)  # Q (imaginary first)
            data.append(20 + i)  # I (real second)
        return data
    
    def test_process_packet_increments_count(self, detector, sample_csi_data):
        """Processing packet increments packet count."""
        initial = detector._packet_count
        detector.process_packet(sample_csi_data, DEFAULT_SUBCARRIERS)
        assert detector._packet_count == initial + 1
    
    def test_process_multiple_packets(self, detector, sample_csi_data):
        """Processing multiple packets fills buffer."""
        subcarriers = DEFAULT_SUBCARRIERS
        for _ in range(10):
            detector.process_packet(sample_csi_data, subcarriers)
        
        assert detector._packet_count == 10
        assert detector.is_ready()
    
    def test_update_state_before_ready(self, detector, sample_csi_data):
        """Update state before buffer is full returns default values."""
        detector.process_packet(sample_csi_data, DEFAULT_SUBCARRIERS)
        
        metrics = detector.update_state()
        
        assert metrics['state'] == MotionState.IDLE
        assert metrics['probability'] == 0.0
        assert metrics['threshold'] == HIGH_ACCURACY_DEFAULT_THRESHOLD
    
    def test_update_state_after_ready(self, detector, sample_csi_data):
        """Update state after buffer is full runs inference."""
        subcarriers = DEFAULT_SUBCARRIERS
        for _ in range(10):
            detector.process_packet(sample_csi_data, subcarriers)
        
        metrics = detector.update_state()
        
        assert 'state' in metrics
        assert 'probability' in metrics
        assert 'threshold' in metrics
        assert 0.0 <= metrics['probability'] <= HIGH_ACCURACY_METRIC_SCALE

    def test_tracking_enabled(self, detector, sample_csi_data):
        """Test that tracking records data when enabled."""
        detector.track_data = True
        subcarriers = DEFAULT_SUBCARRIERS
        
        for _ in range(10):
            detector.process_packet(sample_csi_data, subcarriers)
        
        detector.update_state()
        
        assert len(detector.probability_history) == 1
        assert len(detector.state_history) == 1
    
    def test_tracking_disabled(self, detector, sample_csi_data):
        """Test that tracking does not record when disabled."""
        detector.track_data = False
        subcarriers = DEFAULT_SUBCARRIERS
        
        for _ in range(10):
            detector.process_packet(sample_csi_data, subcarriers)
        
        detector.update_state()
        
        assert len(detector.probability_history) == 0
        assert len(detector.state_history) == 0


class TestExtractFeaturesIntegration:
    """Test that extract_features_by_name is correctly integrated."""
    
    def test_extract_features_returns_model_feature_count(self):
        """_extract_features matches the exported feature count."""
        detector = HighAccuracyDetector(window_size=10)
        
        # Fill buffer with synthetic data
        csi_data = [20] * 128  # 64 subcarriers * 2
        subcarriers = DEFAULT_SUBCARRIERS
        
        for _ in range(10):
            detector.process_packet(csi_data, subcarriers)
        
        features = detector._extract_features()
        
        assert len(features) == len(FEATURE_NAMES)
        assert all(isinstance(f, (int, float)) for f in features)

    def test_extract_features_supports_l1_delta_lag_ratio(self, monkeypatch):
        """HighAccuracyDetector forwards the tracker's floor-invariant lag ratio."""
        monkeypatch.setattr(
            high_accuracy_detector_module,
            "FEATURE_NAMES",
            ["l1_delta_lag_ratio"],
        )
        detector = HighAccuracyDetector(window_size=20)
        monkeypatch.setattr(detector._l1_tracker, "delta_lag_ratio", lambda: 1.75)

        quiet = _make_csi_payload(_make_band_profile(offset=0))
        for _ in range(30):
            detector.process_packet(quiet, DEFAULT_SUBCARRIERS)

        assert detector._extract_features() == pytest.approx([1.75])

    def test_hampel_configuration_covers_l1_feature_stream(self, monkeypatch):
        """The ML Hampel flag configures both turbulence and L1 streams."""
        monkeypatch.setattr(high_accuracy_detector_module, "FEATURE_NAMES", ["l1_delta_lag_ratio"])

        enabled = HighAccuracyDetector(window_size=20, enable_hampel=True)
        disabled = HighAccuracyDetector(window_size=20, enable_hampel=False)

        assert enabled._context.hampel_filter is not None
        assert enabled._l1_tracker._hampel_filter is not None
        assert disabled._context.hampel_filter is None
        assert disabled._l1_tracker._hampel_filter is None


class TestHighAccuracyDetectorMotionTracking:
    """Test motion tracking with data that triggers MOTION state."""
    
    def test_motion_count_increments_on_motion(self):
        """Motion count increments when MOTION is detected."""
        detector = HighAccuracyDetector(window_size=10, threshold=0.0)  # Very low threshold
        detector.track_data = True
        
        # Create varying CSI data to trigger motion
        subcarriers = DEFAULT_SUBCARRIERS
        for i in range(10):
            # Vary data to create turbulence
            csi_data = [(20 + i * 5) % 127] * 128
            detector.process_packet(csi_data, subcarriers)
        
        # Update state - with threshold=0, should detect motion
        detector.update_state()
        
        # Should have recorded in history
        assert len(detector.probability_history) == 1
        assert len(detector.state_history) == 1
    
    def test_get_motion_count(self):
        """Test get_motion_count method."""
        detector = HighAccuracyDetector(window_size=10, threshold=0.0)
        detector.track_data = True
        
        subcarriers = DEFAULT_SUBCARRIERS
        for i in range(10):
            csi_data = [(20 + i * 5) % 127] * 128
            detector.process_packet(csi_data, subcarriers)
        
        # Update multiple times
        detector.update_state()
        count = detector.get_motion_count()
        
        assert isinstance(count, int)
        assert count >= 0
    
    def test_state_changes_to_motion(self):
        """Test that state changes to MOTION with low threshold."""
        detector = HighAccuracyDetector(window_size=10, threshold=0.0)
        
        subcarriers = DEFAULT_SUBCARRIERS
        for _i in range(10):
            csi_data = [50] * 128
            detector.process_packet(csi_data, subcarriers)
        
        metrics = detector.update_state()
        
        # With threshold=0, any probability > 0 triggers motion
        if metrics['probability'] > 0:
            assert metrics['state'] == MotionState.MOTION
