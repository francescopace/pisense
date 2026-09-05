# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Reference Utility Functions

Shared utility functions used across multiple modules.
Mirrors utils.h from ESPectre C++ implementation.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

# This module intentionally mirrors and re-exports the device utility surface.
# ruff: noqa: F401

import math

from micro_espectre.device_utils import (
        CsiFrameTimestampFilter as CsiFrameTimestampFilter,
        DETECTOR_RESET_DROP_STREAK,
        DISPOSITION_DROP,
        DISPOSITION_SENSE,
        FORMAT_ID_HT20,
        FORMAT_ID_UNKNOWN,
        LAYOUT_ID_HT20_57,
        LAYOUT_ID_HT20_57_DOUBLE,
        LAYOUT_ID_HT20_64,
        LAYOUT_ID_HT20_64_DOUBLE,
        LAYOUT_ID_UNKNOWN,
        LAYOUT_BINS_CENTERED,
        LAYOUT_BINS_CLASSIC,
        LAYOUT_BINS_UNKNOWN,
        NORMALIZATION_DOUBLE_HT20,
        NORMALIZATION_DOUBLE_HT57_TO_64,
        NORMALIZATION_HT57_TO_64,
        REASON_BAD_LENGTH,
        REASON_MISSING_METADATA,
        REASON_NONE,
        REASON_NULL_OR_EMPTY,
        REASON_UNEXPECTED_LTF,
        REASON_UNKNOWN_LAYOUT,
        REASON_UNSUPPORTED_PHY,
        REASON_UNSUPPORTED_WIDTH,
        assess_ht20_payload_layout as assess_ht20_payload_layout,
        assess_ht20_sensing_frame as assess_ht20_sensing_frame,
        calculate_variance,
        csi_read_frame as csi_read_frame,
        detect_ht20_bin_layout as detect_ht20_bin_layout,
        insertion_sort as insertion_sort,
        is_ht20_sensing_frame as is_ht20_sensing_frame,
        normalize_ht20_csi_payload as normalize_ht20_csi_payload,
        rotate_ht20_classic_to_centered as rotate_ht20_classic_to_centered,
        to_signed_int8,
)


def calculate_median(values):
    """
    Calculate median of a list.
    
    Note: This function sorts the input list in-place for efficiency
    (avoids memory allocation on MicroPython).
    
    Args:
        values: List of numeric values (will be sorted in-place)
    
    Returns:
        Median value (0 if empty, integer for int lists)
    """
    if not values:
        return 0
    values.sort()
    n = len(values)
    if n % 2 == 0:
        return (values[n // 2 - 1] + values[n // 2]) // 2
    return values[n // 2]


def calculate_percentile(values, percentile):
    """
    Calculate percentile value from a list.
    
    Uses linear interpolation between adjacent values.
    
    Args:
        values: List of numeric values
        percentile: Percentile to calculate (0-100)
    
    Returns:
        float: Percentile value (0.0 if list is empty, matching the C++ helper)
    """
    if not values:
        return 0.0
    
    sorted_values = sorted(values)
    n = len(sorted_values)
    p = percentile / 100.0
    k = int((n - 1) * p)
    
    if k >= n - 1:
        return sorted_values[-1]
    
    # Linear interpolation
    frac = (n - 1) * p - k
    return sorted_values[k] * (1 - frac) + sorted_values[k + 1] * frac


def calculate_std(values):
    """
    Calculate standard deviation.
    
    Args:
        values: List of numeric values
    
    Returns:
        float: Standard deviation (0.0 if empty)
    """
    var = calculate_variance(values)
    return math.sqrt(var) if var > 0 else 0.0


def calculate_magnitude(i, q):
    """
    Calculate magnitude (amplitude) from I/Q components.
    
    Args:
        i: In-phase component (int8)
        q: Quadrature component (int8)
    
    Returns:
        float: Magnitude = sqrt(I² + Q²)
    """
    fi = float(i)
    fq = float(q)
    return math.sqrt(fi * fi + fq * fq)


def calculate_spatial_turbulence(magnitudes, band):
    """
    Calculate spatial turbulence from magnitudes.

    The returned turbulence is always gain-invariant `std/mean`.
    
    Args:
        magnitudes: List of magnitude values (one per subcarrier)
        band: List of subcarrier indices to use
    
    Returns:
        float: Turbulence value (0.0 if no valid subcarriers)
    """
    band_mags = [magnitudes[sc] for sc in band if sc < len(magnitudes)]
    
    if not band_mags:
        return 0.0
    
    std = calculate_std(band_mags)
    mean = sum(band_mags) / len(band_mags)
    return std / mean if mean > 0 else 0.0


# =============================================================================
# CSI I/Q Parsing Functions
# =============================================================================

def extract_amplitude(csi_data, sc_idx):
    """
    Extract amplitude for a single subcarrier from CSI data.
    
    Uses Espressif CSI format: [Imaginary, Real, ...] per subcarrier.
    CSI values are signed int8 stored as uint8.
    
    Args:
        csi_data: Raw CSI data (bytes or list of uint8)
        sc_idx: Subcarrier index (0-63 for HT20)
    
    Returns:
        float: Amplitude (magnitude) value, or 0.0 if invalid index
    """
    i_idx = sc_idx * 2 + 1  # Real (In-phase) is second
    q_idx = sc_idx * 2      # Imaginary (Quadrature) is first
    
    if q_idx + 1 >= len(csi_data):
        return 0.0
    
    # Convert to signed int8
    real = to_signed_int8(csi_data[i_idx])
    imag = to_signed_int8(csi_data[q_idx])
    
    return math.sqrt(float(real * real + imag * imag))


def extract_amplitudes(csi_data, subcarriers=None):
    """
    Extract amplitudes for multiple subcarriers from CSI data.
    
    Uses Espressif CSI format: [Imaginary, Real, ...] per subcarrier.
    
    Args:
        csi_data: Raw CSI data (bytes or list of uint8)
        subcarriers: List of subcarrier indices (default: all 64)
    
    Returns:
        list: Amplitude values for each subcarrier
    """
    if subcarriers is None:
        # Use all available subcarriers (HT20: 64 max)
        max_sc = min(64, len(csi_data) // 2)
        subcarriers = range(max_sc)
    
    amplitudes = []
    for sc_idx in subcarriers:
        amp = extract_amplitude(csi_data, sc_idx)
        if amp > 0.0 or sc_idx * 2 + 1 < len(csi_data):
            amplitudes.append(amp)
    
    return amplitudes


def extract_all_magnitudes(csi_data):
    """
    Extract magnitudes for ALL subcarriers from CSI data.
    
    Returns a list indexed by subcarrier number (0-63 for HT20).
    This is useful when you need to access magnitudes by subcarrier index.
    
    Args:
        csi_data: Raw CSI data (bytes or list of uint8)
    
    Returns:
        list: Magnitudes indexed by subcarrier (length = num_subcarriers)
    """
    num_sc = min(64, len(csi_data) // 2)
    magnitudes = [0.0] * num_sc
    
    for sc_idx in range(num_sc):
        magnitudes[sc_idx] = extract_amplitude(csi_data, sc_idx)
    
    return magnitudes


def extract_phase(csi_data, sc_idx):
    """
    Extract phase for a single subcarrier from CSI data.
    
    Uses Espressif CSI format: [Imaginary, Real, ...] per subcarrier.
    CSI values are signed int8 stored as uint8.
    
    Args:
        csi_data: Raw CSI data (bytes or list of uint8)
        sc_idx: Subcarrier index (0-63 for HT20)
    
    Returns:
        float: Phase value in radians (-pi to pi), or 0.0 if invalid index
    """
    i_idx = sc_idx * 2 + 1  # Real (In-phase) is second
    q_idx = sc_idx * 2      # Imaginary (Quadrature) is first
    
    if q_idx + 1 >= len(csi_data):
        return 0.0
    
    real = to_signed_int8(csi_data[i_idx])
    imag = to_signed_int8(csi_data[q_idx])
    
    return math.atan2(float(imag), float(real))


def extract_phases(csi_data, subcarriers=None):
    """
    Extract phases for multiple subcarriers from CSI data.
    
    Uses Espressif CSI format: [Imaginary, Real, ...] per subcarrier.
    
    Args:
        csi_data: Raw CSI data (bytes or list of uint8)
        subcarriers: List of subcarrier indices (default: all 64)
    
    Returns:
        list: Phase values in radians for each subcarrier
    """
    if subcarriers is None:
        # Use all available subcarriers (HT20: 64 max)
        max_sc = min(64, len(csi_data) // 2)
        subcarriers = range(max_sc)
    
    phases = []
    for sc_idx in subcarriers:
        if sc_idx * 2 + 1 < len(csi_data):
            phases.append(extract_phase(csi_data, sc_idx))
    
    return phases


def extract_amplitudes_and_phases(csi_data, subcarriers=None):
    """
    Extract both amplitudes and phases for subcarriers from CSI data.
    
    More efficient than calling extract_amplitudes and extract_phases separately
    since it only parses I/Q once per subcarrier.
    
    Args:
        csi_data: Raw CSI data (bytes or list of uint8)
        subcarriers: List of subcarrier indices (default: all 64)
    
    Returns:
        tuple: (amplitudes, phases) lists
    """
    if subcarriers is None:
        max_sc = min(64, len(csi_data) // 2)
        subcarriers = range(max_sc)
    
    amplitudes = []
    phases = []
    
    for sc_idx in subcarriers:
        i_idx = sc_idx * 2 + 1  # Real (In-phase) is second
        q_idx = sc_idx * 2      # Imaginary (Quadrature) is first
        
        if q_idx + 1 >= len(csi_data):
            continue
        
        real = float(to_signed_int8(csi_data[i_idx]))
        imag = float(to_signed_int8(csi_data[q_idx]))
        
        amplitudes.append(math.sqrt(real * real + imag * imag))
        phases.append(math.atan2(imag, real))
    
    return amplitudes, phases
