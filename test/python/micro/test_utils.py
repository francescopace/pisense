# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Utility Function Tests

Unit tests for shared utility functions.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

import pytest
import math
import numpy as np
from micro_espectre import device_utils as device_utils_module
from micro_espectre.device_utils import (
    CsiPayloadNormalizationState,
    HT20_CENTERED_ONLY_NULL_BINS,
    HT20_CLASSIC_ONLY_NULL_BINS,
    assess_ht20_sensing_phy,
    impute_ht20_lltf_detector_bins,
    select_csi_capture_profile,
)
from utils import (
    CsiFrameTimestampFilter,
    DISPOSITION_DROP,
    DISPOSITION_SENSE,
    LAYOUT_BINS_CENTERED,
    LAYOUT_BINS_CLASSIC,
    LAYOUT_BINS_UNKNOWN,
    detect_ht20_bin_layout,
    rotate_ht20_classic_to_centered,
    NORMALIZATION_HT57_TO_64,
    REASON_NONE,
    REASON_UNKNOWN_LAYOUT,
    REASON_UNSUPPORTED_WIDTH,
    assess_ht20_payload_layout,
    assess_ht20_sensing_frame,
    to_signed_int8,
    normalize_ht20_csi_payload,
    is_ht20_sensing_frame,
    calculate_median,
    insertion_sort,
    calculate_percentile,
    calculate_variance,
    calculate_std,
    calculate_magnitude,
    calculate_spatial_turbulence,
    extract_amplitude,
    extract_amplitudes,
    extract_all_magnitudes,
    extract_phase,
    extract_phases,
    extract_amplitudes_and_phases,
)

class TestNormalizeHt20CsiPayload:
    """Test HT20 CSI payload normalization/remap scenarios."""

    def test_c5_ack_capture_keeps_dc_centered(self):
        # Pre-normalization C5 ACK, ESP-IDF 5.5.5, channel 10, 2026-09-07.
        # The documentation's wraparound order would move DC away from bin 32.
        payload = bytes.fromhex(
            "f8f0fdf503fb09000f051409190c1d0e200f2410250e250d250c230a1f081a0613030b02"
            "0201f900ee00e400db00d2fecdffc9000000c900cbffd2ffd900e4feeefefafd06fd10fb"
            "1af922f828f62cf52ef32ff02dee2bee25ed1fee17f00ff206f7fdfbf401eb06e30b"
        )
        assert [i for i in range(53) if payload[i * 2:i * 2 + 2] == b"\x00\x00"] == [26]
        normalized, _, _ = normalize_ht20_csi_payload(payload)
        assert [i for i in range(6, 59) if normalized[i * 2:i * 2 + 2] == b"\x00\x00"] == [32]
        assert normalized[64:66] == b"\x00\x00"

    def test_compact_lltf_preserves_physical_tones_and_zero_fills_missing_bins(self):
        tones = list(range(-26, 27))
        payload = bytes(value & 255 for tone in tones for value in (tone, -tone))
        buffer = bytearray([127] * 128)
        state = CsiPayloadNormalizationState()
        state.bin_layout = LAYOUT_BINS_CLASSIC
        normalized, raw_len, tag = normalize_ht20_csi_payload(
            payload, remap_buffer=buffer, state=state
        )
        assert normalized is buffer
        assert raw_len == 106
        assert tag == "lltf53_to_64"
        for tone in range(-32, 32):
            index = (tone + 32) * 2
            expected = bytes((tone & 255, (-tone) & 255)) if -26 <= tone <= 26 else b"\x00\x00"
            assert normalized[index:index + 2] == expected
        detector_view = impute_ht20_lltf_detector_bins(normalized)
        assert normalized[8:12] == b"\x00" * 4
        assert normalized[118:122] == b"\x00" * 4
        assert detector_view[8:10] == normalized[12:14]
        assert detector_view[120:122] == normalized[116:118]

    @pytest.mark.parametrize("mode,width,allow,accepted", [
        (0, 0, True, True), (0, 0, False, False),
        (1, 0, True, False), (3, 0, True, False), (0, 1, True, False),
    ])
    def test_compact_lltf_requires_legacy_phy(self, mode, width, allow, accepted):
        frame = [0] * 10
        frame[7], frame[9] = mode, width
        assessment = assess_ht20_sensing_frame(
            frame, bytes(106), allow_legacy_lltf=allow
        )
        assert (assessment["disposition"] == DISPOSITION_SENSE) == accepted

    def test_passthrough_128_bytes(self):
        payload = bytes(range(128))
        normalized, raw_len, tag = normalize_ht20_csi_payload(payload, expected_len=128)
        assert normalized == payload
        assert raw_len == 128
        assert tag is None

    def test_collapse_256_to_128(self):
        payload = bytes([x % 256 for x in range(256)])
        normalized, raw_len, tag = normalize_ht20_csi_payload(payload, expected_len=128)
        assert normalized == payload[:128]
        assert raw_len == 256
        assert tag == "double_ht20"

    def test_remap_114_to_128(self):
        payload = bytes([x % 256 for x in range(114)])
        remap_buffer = bytearray(128)
        normalized, raw_len, tag = normalize_ht20_csi_payload(
            payload, expected_len=128, remap_buffer=remap_buffer
        )
        assert len(normalized) == 128
        assert normalized[:8] == b"\x00" * 8
        assert normalized[8:122] == payload
        assert normalized[122:] == b"\x00" * 6
        assert raw_len == 114
        assert tag == "ht57_to_64"

    def test_collapse_228_then_remap_to_128(self):
        payload = bytes([x % 256 for x in range(228)])
        remap_buffer = bytearray(128)
        normalized, raw_len, tag = normalize_ht20_csi_payload(
            payload, expected_len=128, remap_buffer=remap_buffer
        )
        assert len(normalized) == 128
        assert normalized[:8] == b"\x00" * 8
        assert normalized[8:122] == payload[:114]
        assert normalized[122:] == b"\x00" * 6
        assert raw_len == 228
        assert tag == "double_ht57_to_64"

    def test_unsupported_length_returns_none(self):
        payload = bytes([0] * 64)
        normalized, raw_len, tag = normalize_ht20_csi_payload(payload, expected_len=128)
        assert normalized is None
        assert raw_len == 64
        assert tag is None

    def test_lltf_detector_fill_copies_nearest_live_edge_tones(self):
        payload = bytearray(range(128))
        detector_buffer = bytearray(128)
        payload[12:14] = b"\x21\xa2"
        payload[116:118] = b"\x43\xc4"

        detector_view = impute_ht20_lltf_detector_bins(
            payload, detector_buffer
        )

        assert detector_view is detector_buffer
        assert payload[8:12] != b"\x21\xa2\x21\xa2"
        assert detector_view[8:12] == b"\x21\xa2\x21\xa2"
        assert detector_view[118:122] == b"\x43\xc4\x43\xc4"
        assert detector_view[16:18] == payload[16:18]

    def test_lltf_detector_fill_rejects_non_ht20_payloads(self):
        assert impute_ht20_lltf_detector_bins(bytearray(126)) is None


def _layout_packet(null_bins):
    """Build a payload populated everywhere except one layout's guard bins."""
    payload = bytearray(128)
    for bin_index in range(64):
        payload[bin_index * 2] = 7 + (bin_index % 5)
        payload[bin_index * 2 + 1] = 200 + (bin_index % 4)
    for bin_index in null_bins:
        payload[bin_index * 2] = 0
        payload[bin_index * 2 + 1] = 0
    return payload


class TestHt20BinLayout:
    """Test HT20 bin-ordering detection and rotation."""

    def test_detects_both_conventions(self):
        classic = _layout_packet(HT20_CLASSIC_ONLY_NULL_BINS)
        centered = _layout_packet(HT20_CENTERED_ONLY_NULL_BINS)
        assert detect_ht20_bin_layout(classic) == LAYOUT_BINS_CLASSIC
        assert detect_ht20_bin_layout(centered) == LAYOUT_BINS_CENTERED

    def test_all_zero_payload_is_inconclusive(self):
        assert detect_ht20_bin_layout(bytearray(128)) == LAYOUT_BINS_UNKNOWN

    def test_single_faded_guard_tone_withdraws_evidence(self):
        faded = _layout_packet(HT20_CLASSIC_ONLY_NULL_BINS)
        first = HT20_CENTERED_ONLY_NULL_BINS[0]
        faded[first * 2] = 0
        faded[first * 2 + 1] = 0
        assert detect_ht20_bin_layout(faded) == LAYOUT_BINS_UNKNOWN

    def test_wrong_length_is_inconclusive(self):
        assert detect_ht20_bin_layout(bytearray(114)) == LAYOUT_BINS_UNKNOWN

    def test_rotation_maps_classic_onto_centered(self):
        classic = _layout_packet(HT20_CLASSIC_ONLY_NULL_BINS)
        rotated = rotate_ht20_classic_to_centered(classic)
        assert detect_ht20_bin_layout(rotated) == LAYOUT_BINS_CENTERED
        for bin_index in range(64):
            source = (bin_index + 32) % 64
            assert rotated[bin_index * 2] == classic[source * 2]
            assert rotated[bin_index * 2 + 1] == classic[source * 2 + 1]

    def test_rotation_is_its_own_inverse(self):
        classic = _layout_packet(HT20_CLASSIC_ONLY_NULL_BINS)
        assert rotate_ht20_classic_to_centered(rotate_ht20_classic_to_centered(classic)) == classic

    def test_normalize_rotates_classic_payload(self):
        classic = _layout_packet(HT20_CLASSIC_ONLY_NULL_BINS)
        normalized, raw_len, tag = normalize_ht20_csi_payload(classic, expected_len=128)
        assert raw_len == 128
        assert tag is None
        assert detect_ht20_bin_layout(normalized) == LAYOUT_BINS_CENTERED

    def test_normalize_leaves_centered_payload_alone(self):
        centered = _layout_packet(HT20_CENTERED_ONLY_NULL_BINS)
        normalized, _raw_len, _tag = normalize_ht20_csi_payload(centered, expected_len=128)
        assert normalized == centered

    def test_latched_layout_covers_inconclusive_packet(self):
        faded = _layout_packet(HT20_CLASSIC_ONLY_NULL_BINS)
        first = HT20_CENTERED_ONLY_NULL_BINS[0]
        faded[first * 2] = 0
        faded[first * 2 + 1] = 0
        assert detect_ht20_bin_layout(faded) == LAYOUT_BINS_UNKNOWN

        without_latch, _raw_len, _tag = normalize_ht20_csi_payload(faded, expected_len=128)
        assert without_latch == faded

        with_latch, _raw_len, _tag = normalize_ht20_csi_payload(
            faded, expected_len=128, bin_layout=LAYOUT_BINS_CLASSIC
        )
        assert with_latch == rotate_ht20_classic_to_centered(faded)

    def test_normalization_state_classifies_layout_only_once(self, monkeypatch):
        centered = _layout_packet(HT20_CENTERED_ONLY_NULL_BINS)
        state = CsiPayloadNormalizationState()
        calls = 0
        original = device_utils_module.detect_ht20_bin_layout

        def counting_detect(payload, expected_len=128):
            nonlocal calls
            calls += 1
            return original(payload, expected_len)

        monkeypatch.setattr(
            device_utils_module,
            "detect_ht20_bin_layout",
            counting_detect,
        )
        first, _raw_len, _tag = normalize_ht20_csi_payload(
            centered,
            expected_len=128,
            state=state,
        )
        second, _raw_len, _tag = normalize_ht20_csi_payload(
            centered,
            expected_len=128,
            state=state,
        )

        assert first == centered
        assert second == centered
        assert calls == 1
        assert state.bin_layout == LAYOUT_BINS_CENTERED


class TestIsHt20SensingFrame:
    """Test MicroPython CSI frame HT20 PHY gating."""

    def test_ht20_accepted(self):
        frame = [0] * 10
        frame[7] = 1  # HT
        frame[9] = 0  # 20 MHz
        assert is_ht20_sensing_frame(frame) is True

    def test_legacy_rejected(self):
        frame = [0] * 10
        frame[7] = 0
        frame[9] = 0
        assert is_ht20_sensing_frame(frame) is False

    def test_ht40_rejected(self):
        frame = [0] * 10
        frame[7] = 1
        frame[9] = 1
        assert is_ht20_sensing_frame(frame) is False

    def test_short_frame_defaults_to_ht20(self):
        assert is_ht20_sensing_frame([0] * 6) is True


class TestHt20Assessment:
    """Classifier-first HT20 assessment tests."""

    def test_payload_layout_marks_114_as_normalized_ht20(self):
        assessment = assess_ht20_payload_layout(114)
        assert assessment["disposition"] == DISPOSITION_SENSE
        assert assessment["reason_code"] == REASON_NONE
        assert assessment["normalization_id"] == NORMALIZATION_HT57_TO_64

    def test_payload_layout_rejects_unknown_even_length(self):
        assessment = assess_ht20_payload_layout(64)
        assert assessment["disposition"] == DISPOSITION_DROP
        assert assessment["reason_code"] == REASON_UNKNOWN_LAYOUT

    def test_sensing_frame_rejects_ht40_before_normalization(self):
        frame = [0] * 10
        frame[5] = bytes([0] * 256)
        frame[7] = 1
        frame[9] = 1
        assessment = assess_ht20_sensing_frame(frame, frame[5])
        assert assessment["disposition"] == DISPOSITION_DROP
        assert assessment["reason_code"] == REASON_UNSUPPORTED_WIDTH

    def test_legacy_lltf_requires_explicit_opt_in(self):
        rejected = assess_ht20_sensing_phy(0, 0)
        accepted = assess_ht20_sensing_phy(0, 0, allow_legacy_lltf=True)

        assert rejected["disposition"] == DISPOSITION_DROP
        assert accepted["disposition"] == DISPOSITION_SENSE

    def test_sensing_frame_accepts_double_ht20_under_ht20_phy(self):
        frame = [0] * 10
        frame[5] = bytes([0] * 256)
        frame[7] = 1
        frame[9] = 0
        assessment = assess_ht20_sensing_frame(frame, frame[5])
        assert assessment["disposition"] == DISPOSITION_SENSE
        assert assessment["normalization_id"] == "double_ht20"

    def test_sensing_frame_short_metadata_uses_historical_ht20_compatibility(self):
        frame = [0] * 6
        assessment = assess_ht20_sensing_frame(frame, bytes([0] * 128))
        assert assessment["disposition"] == DISPOSITION_SENSE
        assert assessment["reason_code"] == REASON_NONE
        assert assessment["normalization_id"] is None

    def test_sensing_frame_accepts_ht20_when_phy_metadata_is_unavailable(self):
        frame = [0] * 22
        frame[5] = bytes([0] * 128)
        assessment = assess_ht20_sensing_frame(
            frame,
            frame[5],
            metadata_missing=True,
        )
        assert assessment["disposition"] == DISPOSITION_SENSE
        assert assessment["reason_code"] == REASON_NONE
        assert assessment["normalization_id"] is None


class TestCsiCaptureProfile:
    def test_esp32_and_s2_always_use_lltf20(self):
        assert select_csi_capture_profile("ESP32", 6) == "lltf20"
        assert select_csi_capture_profile("esp32", 36) == "lltf20"
        assert select_csi_capture_profile("S2", 6) == "lltf20"
        assert select_csi_capture_profile("esp32-s2", 36) == "lltf20"
        assert select_csi_capture_profile("esp32s2", 6) == "lltf20"

    def test_c5_uses_vht20_only_on_5ghz(self):
        assert select_csi_capture_profile("C5", 6) == "ht20"
        assert select_csi_capture_profile("esp32-c5", 36) == "vht20"
        assert select_csi_capture_profile("esp32c5", 36) == "vht20"

    def test_other_targets_use_ht20(self):
        assert select_csi_capture_profile("C6", 6) == "ht20"
        assert select_csi_capture_profile("S3", 36) == "ht20"

class TestCsiFrameTimestampFilter:
    """Test wrap-aware filtering before MicroPython detector callbacks."""

    @staticmethod
    def _frame(timestamp):
        frame = [0] * 10
        frame[4] = timestamp
        return frame

    def test_rejects_duplicate_and_stale_frames_without_poisoning_state(self):
        timestamp_filter = CsiFrameTimestampFilter()

        accepted = [
            timestamp_filter.accept(self._frame(timestamp))
            for timestamp in (100, 101, 3, 101, 102)
        ]

        assert accepted == [True, True, False, False, True]

    def test_accepts_wrap_missing_metadata_and_reset(self):
        timestamp_filter = CsiFrameTimestampFilter()

        assert all(
            timestamp_filter.accept(self._frame(timestamp))
            for timestamp in (0xFFFFFFFE, 0xFFFFFFFF, 0, 1)
        )
        assert timestamp_filter.accept([0] * 4)

        timestamp_filter.reset()
        assert timestamp_filter.accept(self._frame(1))


# ============================================================================
# Signed Integer Conversion Tests
# ============================================================================

class TestToSignedInt8:
    """Test unsigned to signed int8 conversion"""
    
    def test_positive_values(self):
        """Test values < 128 remain positive"""
        assert to_signed_int8(0) == 0
        assert to_signed_int8(1) == 1
        assert to_signed_int8(127) == 127
    
    def test_negative_values(self):
        """Test values >= 128 become negative"""
        assert to_signed_int8(128) == -128
        assert to_signed_int8(255) == -1
        assert to_signed_int8(200) == 200 - 256  # -56


# ============================================================================
# Median Calculation Tests
# ============================================================================

class TestCalculateMedian:
    """Test median calculation"""
    
    def test_empty_list(self):
        """Test median of empty list"""
        assert calculate_median([]) == 0
    
    def test_single_value(self):
        """Test median of single value"""
        assert calculate_median([5]) == 5
    
    def test_odd_count(self):
        """Test median of odd-length list"""
        values = [3, 1, 2]
        assert calculate_median(values) == 2
    
    def test_even_count(self):
        """Test median of even-length list (integer average)"""
        values = [1, 2, 3, 4]
        assert calculate_median(values) == 2  # (2 + 3) // 2 = 2
    
    def test_with_floats(self):
        """Test median with float values"""
        values = [1.0, 5.0, 3.0, 2.0, 4.0]
        assert calculate_median(values) == 3.0


# ============================================================================
# Insertion Sort Tests
# ============================================================================

class TestInsertionSort:
    """Test insertion sort implementation"""
    
    def test_empty_array(self):
        """Test sorting empty array"""
        arr = []
        insertion_sort(arr, 0)
        assert arr == []
    
    def test_single_element(self):
        """Test sorting single element"""
        arr = [5]
        insertion_sort(arr, 1)
        assert arr == [5]
    
    def test_sorted_array(self):
        """Test already sorted array"""
        arr = [1, 2, 3, 4, 5]
        insertion_sort(arr, 5)
        assert arr == [1, 2, 3, 4, 5]
    
    def test_reverse_sorted(self):
        """Test reverse sorted array"""
        arr = [5, 4, 3, 2, 1]
        insertion_sort(arr, 5)
        assert arr == [1, 2, 3, 4, 5]
    
    def test_random_array(self):
        """Test random array"""
        arr = [3, 1, 4, 1, 5, 9, 2, 6]
        insertion_sort(arr, 8)
        assert arr == [1, 1, 2, 3, 4, 5, 6, 9]
    
    def test_partial_sort(self):
        """Test sorting only first n elements"""
        arr = [5, 3, 1, 9, 7]
        insertion_sort(arr, 3)  # Only sort first 3
        assert arr[:3] == [1, 3, 5]
        assert arr[3:] == [9, 7]  # Unchanged


# ============================================================================
# Percentile Calculation Tests
# ============================================================================

class TestCalculatePercentile:
    """Test percentile calculation"""
    
    def test_empty_list(self):
        """Test percentile of empty list"""
        assert calculate_percentile([], 50) == 0.0
    
    def test_single_value(self):
        """Test percentile of single value"""
        assert calculate_percentile([5.0], 50) == 5.0
    
    def test_p0(self):
        """Test 0th percentile (minimum)"""
        values = [1.0, 2.0, 3.0, 4.0, 5.0]
        assert calculate_percentile(values, 0) == 1.0
    
    def test_p100(self):
        """Test 100th percentile (maximum)"""
        values = [1.0, 2.0, 3.0, 4.0, 5.0]
        assert calculate_percentile(values, 100) == 5.0
    
    def test_p50(self):
        """Test 50th percentile (median)"""
        values = [1.0, 2.0, 3.0, 4.0, 5.0]
        result = calculate_percentile(values, 50)
        assert result == pytest.approx(3.0, rel=1e-6)
    
    def test_interpolation(self):
        """Test linear interpolation between values"""
        values = [0.0, 10.0]
        assert calculate_percentile(values, 50) == pytest.approx(5.0, rel=1e-6)


# ============================================================================
# Variance and Std Tests
# ============================================================================

class TestCalculateVariance:
    """Test variance calculation"""
    
    def test_empty_list(self):
        """Test variance of empty list"""
        assert calculate_variance([]) == 0.0
    
    def test_single_value(self):
        """Test variance of single value (should be 0)"""
        assert calculate_variance([5.0]) == 0.0
    
    def test_constant_values(self):
        """Test variance of constant values"""
        values = [5.0] * 10
        assert calculate_variance(values) == 0.0
    
    def test_known_variance(self):
        """Test with known variance"""
        # Values: [1, 3, 5] -> mean = 3
        # Var = ((1-3)^2 + (3-3)^2 + (5-3)^2) / 3 = (4 + 0 + 4) / 3 = 8/3
        values = [1.0, 3.0, 5.0]
        result = calculate_variance(values)
        assert result == pytest.approx(8.0 / 3.0, rel=1e-6)


class TestCalculateStd:
    """Test standard deviation calculation"""
    
    def test_empty_list(self):
        """Test std of empty list"""
        assert calculate_std([]) == 0.0
    
    def test_constant_values(self):
        """Test std of constant values"""
        values = [5.0] * 10
        assert calculate_std(values) == 0.0
    
    def test_known_std(self):
        """Test with known std"""
        values = [1.0, 3.0, 5.0]
        result = calculate_std(values)
        expected = math.sqrt(8.0 / 3.0)
        assert result == pytest.approx(expected, rel=1e-6)


# ============================================================================
# Magnitude Calculation Tests
# ============================================================================

class TestCalculateMagnitude:
    """Test I/Q magnitude calculation"""
    
    def test_zero_components(self):
        """Test magnitude of zero"""
        assert calculate_magnitude(0, 0) == 0.0
    
    def test_real_only(self):
        """Test magnitude with only real component"""
        assert calculate_magnitude(3, 0) == 3.0
    
    def test_imag_only(self):
        """Test magnitude with only imaginary component"""
        assert calculate_magnitude(0, 4) == 4.0
    
    def test_known_magnitude(self):
        """Test 3-4-5 triangle"""
        assert calculate_magnitude(3, 4) == 5.0
    
    def test_negative_components(self):
        """Test with negative components"""
        assert calculate_magnitude(-3, -4) == 5.0


# ============================================================================
# Spatial Turbulence Tests
# ============================================================================

class TestCalculateSpatialTurbulence:
    """Test spatial turbulence calculation"""
    
    def test_empty_band(self):
        """Test with empty band"""
        magnitudes = [10.0] * 64
        assert calculate_spatial_turbulence(magnitudes, []) == 0.0
    
    def test_constant_magnitudes_cv(self):
        """Test with constant magnitudes."""
        magnitudes = [10.0] * 64
        band = [0, 1, 2, 3, 4]
        result = calculate_spatial_turbulence(magnitudes, band)
        assert result == 0.0  # std = 0, so std/mean = 0
    
    def test_constant_magnitudes_zero_turbulence(self):
        """Constant magnitudes always yield zero turbulence."""
        magnitudes = [10.0] * 64
        band = [0, 1, 2, 3, 4]
        result = calculate_spatial_turbulence(magnitudes, band)
        assert result == 0.0  # std = 0, therefore std/mean = 0
    
    def test_varied_magnitudes(self):
        """Test with varied magnitudes"""
        magnitudes = [float(i) for i in range(64)]
        band = [10, 20, 30, 40, 50]
        
        result = calculate_spatial_turbulence(magnitudes, band)
        assert result > 0  # std/mean > 0 for non-constant values


# ============================================================================
# CSI I/Q Parsing Tests
# ============================================================================

class TestExtractAmplitude:
    """Test single subcarrier amplitude extraction"""
    
    def test_invalid_index(self):
        """Test with out-of-bounds index"""
        csi_data = [0] * 10
        assert extract_amplitude(csi_data, 100) == 0.0
    
    def test_zero_iq(self):
        """Test with zero I/Q values"""
        # Each subcarrier is 2 bytes: [Q, I]
        csi_data = [0] * 128
        assert extract_amplitude(csi_data, 0) == 0.0
    
    def test_known_amplitude(self):
        """Test with known I/Q values (3, 4 -> magnitude 5)"""
        csi_data = [0] * 128
        # Subcarrier 5: bytes 10, 11
        # Format: [Q, I] = [4, 3]
        csi_data[10] = 4  # Q
        csi_data[11] = 3  # I
        
        result = extract_amplitude(csi_data, 5)
        assert result == 5.0


class TestExtractAmplitudes:
    """Test multiple subcarrier amplitude extraction"""
    
    def test_default_all_subcarriers(self):
        """Test extracting all available subcarriers"""
        csi_data = [1] * 128  # 64 subcarriers
        result = extract_amplitudes(csi_data)
        
        assert len(result) == 64
    
    def test_specific_subcarriers(self):
        """Test extracting specific subcarriers"""
        csi_data = [0] * 128
        result = extract_amplitudes(csi_data, subcarriers=[0, 10, 20])
        
        assert len(result) == 3


class TestExtractAllMagnitudes:
    """Test extracting all magnitudes"""
    
    def test_returns_indexed_list(self):
        """Test that result is indexed by subcarrier"""
        csi_data = [0] * 128
        result = extract_all_magnitudes(csi_data)
        
        assert len(result) == 64
        assert all(isinstance(x, float) for x in result)


class TestExtractPhase:
    """Test single subcarrier phase extraction"""
    
    def test_invalid_index(self):
        """Test with out-of-bounds index"""
        csi_data = [0] * 10
        assert extract_phase(csi_data, 100) == 0.0
    
    def test_zero_iq(self):
        """Test with zero I/Q values"""
        csi_data = [0] * 128
        assert extract_phase(csi_data, 0) == 0.0
    
    def test_known_phase(self):
        """Test with known I/Q values"""
        csi_data = [0] * 128
        # I = 1, Q = 0 -> phase = atan2(0, 1) = 0
        csi_data[0] = 0  # Q
        csi_data[1] = 1  # I
        
        result = extract_phase(csi_data, 0)
        assert result == pytest.approx(0.0, abs=1e-6)
    
    def test_phase_range(self):
        """Test that phase is in [-pi, pi]"""
        csi_data = [0] * 128
        csi_data[0] = 127  # Q = 127
        csi_data[1] = 128  # I = -128 (signed)
        
        result = extract_phase(csi_data, 0)
        assert -math.pi <= result <= math.pi


class TestExtractPhases:
    """Test multiple subcarrier phase extraction"""
    
    def test_default_all_subcarriers(self):
        """Test extracting all phases"""
        csi_data = [1] * 128
        result = extract_phases(csi_data)
        
        assert len(result) == 64
    
    def test_specific_subcarriers(self):
        """Test extracting specific subcarrier phases"""
        csi_data = [1] * 128
        result = extract_phases(csi_data, subcarriers=[0, 10, 20])
        
        assert len(result) == 3


class TestExtractAmplitudesAndPhases:
    """Test combined amplitude and phase extraction"""
    
    def test_returns_two_lists(self):
        """Test that two lists are returned"""
        csi_data = [1] * 128
        amplitudes, phases = extract_amplitudes_and_phases(csi_data)
        
        assert len(amplitudes) == 64
        assert len(phases) == 64
    
    def test_specific_subcarriers(self):
        """Test with specific subcarriers"""
        csi_data = [1] * 128
        amplitudes, phases = extract_amplitudes_and_phases(csi_data, subcarriers=[0, 10, 20])
        
        assert len(amplitudes) == 3
        assert len(phases) == 3
    
    def test_consistency_with_separate_calls(self):
        """Test that results match separate extraction calls"""
        np.random.seed(42)
        csi_data = list(np.random.randint(0, 256, 128))
        
        amps_combined, phases_combined = extract_amplitudes_and_phases(csi_data)
        amps_separate = extract_amplitudes(csi_data)
        phases_separate = extract_phases(csi_data)
        
        for i in range(len(amps_combined)):
            assert amps_combined[i] == pytest.approx(amps_separate[i], rel=1e-6)
            assert phases_combined[i] == pytest.approx(phases_separate[i], rel=1e-6)


class TestInvalidFirstCsiWord:
    """Hardware-invalid pairs must never become live detector input."""

    @pytest.mark.parametrize('length', [128, 256])
    def test_centered_guard_bytes_are_cleaned_without_changing_live_pairs(self, length):
        payload = _layout_packet(HT20_CENTERED_ONLY_NULL_BINS) * (length // 128)
        payload[:4] = b'\x7f\x80\x7f\x80'
        original = bytes(payload)
        normalized, raw_len, _ = normalize_ht20_csi_payload(payload, first_word_invalid=True)
        assert raw_len == length
        assert normalized[:4] == b'\x00' * 4
        assert normalized[4:] == payload[4:128]
        assert bytes(payload) == original

    @pytest.mark.parametrize('length', [106, 114, 228])
    def test_compact_invalid_live_pairs_are_rejected(self, length):
        assert normalize_ht20_csi_payload(bytearray([7] * length), first_word_invalid=True)[0] is None

    def test_classic_and_ambiguous_invalid_pairs_are_rejected(self):
        for payload in (_layout_packet(HT20_CLASSIC_ONLY_NULL_BINS), bytearray(128)):
            assert normalize_ht20_csi_payload(payload, first_word_invalid=True)[0] is None


def test_hardware_rx_error_is_rejected_before_fast_path():
    frame = [0] * 22
    frame[7] = 1
    frame[21] = 1
    result = assess_ht20_sensing_frame(frame, bytearray(128), static_fast_path=True)
    assert result['disposition'] == 'drop'
    assert result['reason_code'] == 'rx_error'
