/*
 * ESPectre - CSI Format
 *
 * HT20 CSI layout constants, subcarrier band selection, and helpers that
 * extract amplitudes and turbulence directly from raw CSI payloads.
 * Keep aligned with src/python/micro_espectre/config.py and utils.py.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "csi_types.h"
#include "utils.h"

namespace espectre {

// =============================================================================
// HT20 Constants (64 subcarriers - do not change)
// =============================================================================
// Subcarriers +/-4, +/-9, +/-14, +/-19, +/-24, +/-28. Spans the full usable range
// because the motion perturbation stays coherent over ~10 subcarriers (3.1 MHz)
// while quiet noise is nearly per-tone independent, so span is what buys
// independent looks. Stops short of |sc| <= 3, where relative jitter rises ~10%.
// See docs/adr/2026-07-25-select-the-classic-band-from-channel-coherence.md.
// =============================================================================
// HT20 Bin Layout
// =============================================================================
// Wi-Fi 6 parts deliver HT20 CSI centered on DC (bin = subcarrier + 32), while
// classic-MAC parts deliver Espressif's native "0~31, -32~-1" order with DC in
// bin 0. `DEFAULT_SUBCARRIERS` and `HT20_DC_SUBCARRIER` assume the centered
// convention, so classic payloads must be rotated before the band means the same
// physical subcarriers on every chip.
//
// The two layouts are told apart by their guard nulls, which the radio reports as
// exactly zero. Bins 0 and 32 are null under both conventions and carry no
// information; these are the bins that are null under exactly one of them.
constexpr uint8_t HT20_CLASSIC_ONLY_NULL_BINS[] = {29, 30, 31, 33, 34, 35};
constexpr uint8_t HT20_CENTERED_ONLY_NULL_BINS[] = {1, 2, 3, 61, 62, 63};
// LLTF carries physical subcarriers -26..-1 and +1..+26, so the two
// outer data tones on either side are missing in the centered convention.
constexpr uint8_t HT20_LLTF_MISSING_BINS[] = {4, 5, 59, 60};

enum class Ht20BinLayout : uint8_t {
    UNKNOWN = 0,
    CENTERED,  // bin = subcarrier + 32, DC at bin 32
    CLASSIC,   // bin = subcarrier mod 64, DC at bin 0
};

inline uint8_t ht20_bins_with_energy(const int8_t* csi_data, const uint8_t* bins, uint8_t count,
                                         bool first_word_invalid = false) {
    uint8_t populated = 0;
    for (uint8_t i = 0; i < count; ++i) {
        if (first_word_invalid && bins[i] < 2U) continue;
        const uint16_t byte_index = static_cast<uint16_t>(bins[i]) * 2U;
        if (csi_data[byte_index] != 0 || csi_data[byte_index + 1] != 0) {
            populated++;
        }
    }
    return populated;
}

/**
 * Identify which HT20 bin ordering a 64-subcarrier payload uses.
 *
 * Requires positive evidence in both directions: one guard set must be entirely
 * null and the other entirely populated. Absence of energy alone is not enough,
 * because a sparse or degenerate payload is null under both conventions.
 *
 * @param csi_data Raw CSI payload (interleaved I/Q pairs)
 * @param csi_len Payload length in bytes (must be HT20_CSI_LEN)
 * @param first_word_invalid Ignore the two hardware-invalid source pairs
 * @return The detected layout, or UNKNOWN when the evidence is inconclusive
 */
inline Ht20BinLayout detect_ht20_bin_layout(const int8_t* csi_data, size_t csi_len,
                                            bool first_word_invalid = false) {
    if (csi_data == nullptr || csi_len != HT20_CSI_LEN) {
        return Ht20BinLayout::UNKNOWN;
    }

    constexpr uint8_t kNullBinCount =
        static_cast<uint8_t>(sizeof(HT20_CLASSIC_ONLY_NULL_BINS));
    const uint8_t classic_energy =
        ht20_bins_with_energy(csi_data, HT20_CLASSIC_ONLY_NULL_BINS, kNullBinCount);
    const uint8_t centered_energy =
        ht20_bins_with_energy(csi_data, HT20_CENTERED_ONLY_NULL_BINS, kNullBinCount, first_word_invalid);

    if (classic_energy == 0U && centered_energy == kNullBinCount - (first_word_invalid ? 1U : 0U)) {
        return Ht20BinLayout::CLASSIC;
    }
    if (centered_energy == 0U && classic_energy == kNullBinCount) {
        return Ht20BinLayout::CENTERED;
    }
    return Ht20BinLayout::UNKNOWN;
}

/**
 * Mark LLTF's unavailable edge tones as missing in a normalized raw view.
 *
 * @param csi_data Centered, interleaved I/Q HT20 payload to update in place.
 * @param csi_len Payload length in bytes; must equal HT20_CSI_LEN.
 * @return true when the payload was updated, or false for invalid input.
 */
inline bool zero_ht20_lltf_missing_bins(int8_t* csi_data, size_t csi_len) {
    if (csi_data == nullptr || csi_len != HT20_CSI_LEN) {
        return false;
    }
    for (uint8_t bin : HT20_LLTF_MISSING_BINS) {
        const uint16_t byte_index = static_cast<uint16_t>(bin) * 2U;
        csi_data[byte_index] = 0;
        csi_data[byte_index + 1U] = 0;
    }
    return true;
}

/**
 * Fill LLTF's missing edge tones in a private detector input buffer.
 *
 * Each missing I/Q pair is copied from the nearest live physical tone, -26 or
 * +26. Keep the normalized raw view zero-filled and call this helper only on a
 * separate detector buffer.
 *
 * @param csi_data Centered, interleaved I/Q HT20 payload to update in place.
 * @param csi_len Payload length in bytes; must equal HT20_CSI_LEN.
 * @return true when the payload was updated, or false for invalid input.
 */
inline bool impute_ht20_lltf_detector_bins(int8_t* csi_data, size_t csi_len) {
    if (csi_data == nullptr || csi_len != HT20_CSI_LEN) {
        return false;
    }
    for (uint8_t target_bin : HT20_LLTF_MISSING_BINS) {
        const uint16_t target = static_cast<uint16_t>(target_bin) * 2U;
        const uint16_t source = target_bin < HT20_DC_SUBCARRIER ? 12U : 116U;
        csi_data[target] = csi_data[source];
        csi_data[target + 1U] = csi_data[source + 1U];
    }
    return true;
}

/**
 * Rotate a classic-order HT20 payload into the centered convention.
 *
 * Rotating by half the FFT size is its own inverse, so one swap of the payload
 * halves maps `0~31, -32~-1` onto `-32~+31`.
 *
 * @param csi_data Source payload of HT20_CSI_LEN bytes
 * @param out Destination buffer of HT20_CSI_LEN bytes (must not alias the source)
 */
inline void rotate_ht20_classic_to_centered(const int8_t* csi_data, int8_t* out) {
    if (csi_data == nullptr || out == nullptr) {
        return;
    }
    constexpr uint16_t kHalf = HT20_CSI_LEN / 2U;
    std::memcpy(out, csi_data + kHalf, kHalf);
    std::memcpy(out + kHalf, csi_data, kHalf);
}

/**
 * Calculate spatial turbulence from pre-calculated magnitudes
 *
 * Spatial turbulence is the standard deviation of magnitudes across
 * selected subcarriers. It measures the spatial variability of the
 * Wi-Fi channel - higher values indicate motion/disturbance.
 *
 * @param magnitudes Array of magnitude values (one per subcarrier)
 * @param subcarriers Array of selected subcarrier indices
 * @param num_subcarriers Number of selected subcarriers (max 12)
 * @param max_subcarrier Maximum valid subcarrier index (default: 64 for HT20)
 * @return Turbulence value
 */
inline float calculate_spatial_turbulence(const float* magnitudes,
                                          const uint8_t* subcarriers,
                                          uint8_t num_subcarriers,
                                          uint16_t max_subcarrier = 64) {
    if (num_subcarriers == 0 || !magnitudes || !subcarriers) {
        return 0.0f;
    }

    // Collect valid magnitudes (max 12 subcarriers for band selection)
    float valid_mags[12];
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < num_subcarriers && valid_count < 12; i++) {
        if (subcarriers[i] < max_subcarrier) {
            valid_mags[valid_count++] = magnitudes[subcarriers[i]];
        }
    }

    if (valid_count == 0) {
        return 0.0f;
    }

    const MeanVariance stats = calculate_mean_variance_two_pass(valid_mags, valid_count);
    return apply_cv_normalization(std::sqrt(stats.variance), stats.mean);
}

/**
 * Extract subcarrier amplitudes from raw CSI data (I/Q pairs)
 *
 * Mirrors the Python `SegmentationContext._fill_amplitude_buffer` helper.
 *
 * @param csi_data Raw CSI data (interleaved I/Q pairs, Espressif format)
 * @param csi_len Length of CSI data in bytes
 * @param subcarriers Array of selected subcarrier indices
 * @param num_subcarriers Number of selected subcarriers
 * @param out Output amplitude buffer
 * @param out_capacity Capacity of the output buffer
 * @return Number of amplitudes written
 */
inline uint8_t extract_subcarrier_amplitudes(const int8_t* csi_data,
                                             size_t csi_len,
                                             const uint8_t* subcarriers,
                                             uint8_t num_subcarriers,
                                             float* out,
                                             uint8_t out_capacity) {
    if (!csi_data || csi_len < 2 || num_subcarriers == 0 || !subcarriers || !out) {
        return 0;
    }

    int total_subcarriers = static_cast<int>(csi_len / 2);
    uint8_t valid_count = 0;

    for (int i = 0; i < num_subcarriers && valid_count < out_capacity; i++) {
        int sc_idx = subcarriers[i];
        if (sc_idx >= total_subcarriers) {
            continue;
        }

        // Espressif CSI format: [Imaginary, Real, ...] per subcarrier
        out[valid_count++] = calculate_magnitude(csi_data[sc_idx * 2 + 1],
                                                 csi_data[sc_idx * 2]);
    }
    return valid_count;
}

/** Extract one packet-wide amplitude frame for reuse by multiple feature paths. */
inline uint8_t extract_packet_subcarrier_amplitudes(const int8_t* csi_data,
                                                    size_t csi_len,
                                                    float* out,
                                                    uint8_t out_capacity) {
    if (csi_data == nullptr || out == nullptr) return 0U;
    const uint8_t count = static_cast<uint8_t>(std::min<size_t>(
        HT20_NUM_SUBCARRIERS, std::min<size_t>(out_capacity, csi_len / 2U)));
    for (uint8_t i = 0U; i < count; ++i) {
        out[i] = calculate_magnitude(csi_data[i * 2U + 1U], csi_data[i * 2U]);
    }
    return count;
}

/** Fill one packet-wide squared-magnitude frame for energy-domain consumers. */
inline uint8_t fill_packet_subcarrier_energies(const int8_t* csi_data,
                                               size_t csi_len,
                                               float* out,
                                               uint8_t out_capacity) {
    if (csi_data == nullptr || out == nullptr) return 0U;
    const uint8_t count = static_cast<uint8_t>(std::min<size_t>(
        HT20_NUM_SUBCARRIERS, std::min<size_t>(out_capacity, csi_len / 2U)));
    for (uint8_t i = 0U; i < count; ++i) {
        const float imag = static_cast<float>(csi_data[i * 2U]);
        const float real = static_cast<float>(csi_data[i * 2U + 1U]);
        out[i] = real * real + imag * imag;
    }
    return count;
}

inline void energies_to_amplitudes_in_place(float* values, uint8_t count) {
    if (values == nullptr) return;
    for (uint8_t i = 0U; i < count; ++i) values[i] = std::sqrt(values[i]);
}

namespace detail {

constexpr std::array<bool, HT20_NUM_SUBCARRIERS> required_amplitude_bins(
        const uint8_t* subcarriers, uint8_t count, uint8_t width) {
    std::array<bool, HT20_NUM_SUBCARRIERS> required{};
    if (subcarriers == nullptr) return required;
    for (uint8_t i = 0U; i < count; ++i) {
        if (subcarriers[i] < HT20_NUM_SUBCARRIERS) required[subcarriers[i]] = true;
        if (width == 0U) continue;
        const int half = (width - 1U) / 2U;
        int low = static_cast<int>(subcarriers[i]) - half;
        int high = static_cast<int>(subcarriers[i]) + width - 1U - half;
        if (low < HT20_GUARD_BAND_LOW) {
            low = HT20_GUARD_BAND_LOW;
            high = HT20_GUARD_BAND_LOW + width - 1U;
        }
        if (high > HT20_GUARD_BAND_HIGH) {
            low = HT20_GUARD_BAND_HIGH - width + 1U;
            high = HT20_GUARD_BAND_HIGH;
        }
        for (int bin = low; bin <= high; ++bin) {
            if (bin >= 0 && bin < HT20_NUM_SUBCARRIERS && bin != HT20_DC_SUBCARRIER) {
                required[bin] = true;
            }
        }
    }
    return required;
}

// Production's fixed band is resolved at compile time. Custom SDK bands retain
// the same bounds and aggregation behavior without changing the packet layout.
template <uint8_t Width>
inline void required_energies_to_amplitudes(float* values, uint8_t count,
        const uint8_t* subcarriers, uint8_t subcarrier_count, bool aggregated) {
    static constexpr auto selected = required_amplitude_bins(
        DEFAULT_SUBCARRIERS, HT20_SELECTED_BAND_SIZE, 0U);
    static constexpr auto adjacent = required_amplitude_bins(
        DEFAULT_SUBCARRIERS, HT20_SELECTED_BAND_SIZE, Width);
    const auto convert = [values, count](const auto& required) {
        for (uint8_t i = 0U; i < count; ++i) {
            if (required[i]) values[i] = std::sqrt(values[i]);
        }
    };
    if (subcarriers == DEFAULT_SUBCARRIERS && subcarrier_count == HT20_SELECTED_BAND_SIZE) {
        convert(aggregated ? adjacent : selected);
    } else {
        convert(required_amplitude_bins(subcarriers, subcarrier_count, aggregated ? Width : 0U));
    }
}

}  // namespace detail

/** Select the configured tones from a packet-wide amplitude frame. */
inline uint8_t select_subcarrier_amplitudes(const float* packet_amplitudes,
                                            uint8_t packet_count,
                                            const uint8_t* subcarriers,
                                            uint8_t num_subcarriers,
                                            float* out,
                                            uint8_t out_capacity) {
    if (packet_amplitudes == nullptr || subcarriers == nullptr || out == nullptr) return 0U;
    uint8_t written = 0U;
    for (uint8_t i = 0U; i < num_subcarriers && written < out_capacity; ++i) {
        if (subcarriers[i] < packet_count) out[written++] = packet_amplitudes[subcarriers[i]];
    }
    return written;
}

/** Select adjacent-bin mean amplitudes from a packet-wide amplitude frame. */
inline uint8_t select_adjacent_aggregated_subcarrier_amplitudes(
        const float* packet_amplitudes, uint8_t packet_count,
        const uint8_t* subcarriers, uint8_t num_subcarriers, uint8_t width,
        float* out, uint8_t out_capacity) {
    if (packet_amplitudes == nullptr || subcarriers == nullptr || width == 0U || out == nullptr) return 0U;
    const int half = static_cast<int>((width - 1U) / 2U);
    uint8_t written = 0U;
    for (uint8_t i = 0U; i < num_subcarriers && written < out_capacity; ++i) {
        int low = static_cast<int>(subcarriers[i]) - half;
        int high = static_cast<int>(subcarriers[i]) + static_cast<int>(width - 1U) - half;
        if (low < HT20_GUARD_BAND_LOW) {
            low = HT20_GUARD_BAND_LOW;
            high = HT20_GUARD_BAND_LOW + width - 1U;
        }
        if (high > HT20_GUARD_BAND_HIGH) {
            low = HT20_GUARD_BAND_HIGH - width + 1U;
            high = HT20_GUARD_BAND_HIGH;
        }
        float total = 0.0f;
        uint8_t count = 0U;
        for (int bin = low; bin <= high; ++bin) {
            if (bin == HT20_DC_SUBCARRIER || bin < 0 || bin >= packet_count) continue;
            total += packet_amplitudes[bin];
            ++count;
        }
        if (count > 0U) out[written++] = total / static_cast<float>(count);
    }
    return written;
}

/**
 * Extract one mean magnitude per selected tone from adjacent live HT20 bins.
 *
 * Windows are clamped to bins 4..60 and skip the DC null at bin 32. This is
 * the production counterpart of
 * `SegmentationContext._fill_adjacent_aggregated_amplitude_buffer`.
 */
inline uint8_t extract_adjacent_aggregated_subcarrier_amplitudes(
        const int8_t* csi_data,
        size_t csi_len,
        const uint8_t* subcarriers,
        uint8_t num_subcarriers,
        uint8_t width,
        float* out,
        uint8_t out_capacity) {
    if (csi_data == nullptr || csi_len < 2U || subcarriers == nullptr ||
        num_subcarriers == 0U || width == 0U || out == nullptr) {
        return 0U;
    }

    const int total_subcarriers = static_cast<int>(csi_len / 2U);
    const int half = static_cast<int>((width - 1U) / 2U);
    uint8_t written = 0U;
    for (uint8_t i = 0U; i < num_subcarriers && written < out_capacity; ++i) {
        int low = static_cast<int>(subcarriers[i]) - half;
        int high = static_cast<int>(subcarriers[i]) +
                   static_cast<int>(width - 1U) - half;
        if (low < HT20_GUARD_BAND_LOW) {
            low = HT20_GUARD_BAND_LOW;
            high = HT20_GUARD_BAND_LOW + width - 1U;
        }
        if (high > HT20_GUARD_BAND_HIGH) {
            low = HT20_GUARD_BAND_HIGH - width + 1U;
            high = HT20_GUARD_BAND_HIGH;
        }

        float magnitude_sum = 0.0f;
        uint8_t count = 0U;
        for (int bin = low; bin <= high; ++bin) {
            if (bin == HT20_DC_SUBCARRIER || bin < 0 ||
                bin >= total_subcarriers) {
                continue;
            }
            magnitude_sum += calculate_magnitude(
                csi_data[bin * 2 + 1], csi_data[bin * 2]);
            ++count;
        }
        if (count > 0U) {
            out[written++] = magnitude_sum / static_cast<float>(count);
        }
    }
    return written;
}

inline float calculate_spatial_turbulence_from_amplitudes(const float* amplitudes,
                                                          uint8_t count) {
    if (amplitudes == nullptr || count == 0) {
        return 0.0f;
    }
    const MeanVariance stats = calculate_mean_variance_two_pass(amplitudes, count);
    return apply_cv_normalization(std::sqrt(stats.variance), stats.mean);
}

/**
 * Calculate spatial turbulence directly from raw CSI data (I/Q pairs)
 *
 * This is a convenience wrapper that calculates magnitudes internally
 * before computing spatial turbulence.
 *
 * HT20 only: 64 subcarriers, 128 bytes CSI data.
 *
 * @param csi_data Raw CSI data (interleaved I/Q pairs)
 * @param csi_len Length of CSI data in bytes (expected: 128 for HT20)
 * @param subcarriers Array of selected subcarrier indices
 * @param num_subcarriers Number of selected subcarriers (max 12)
 * @return Turbulence value
 */
inline float calculate_spatial_turbulence_from_csi(const int8_t* csi_data,
                                                   size_t csi_len,
                                                   const uint8_t* subcarriers,
                                                   uint8_t num_subcarriers) {
    float amplitudes[HT20_SELECTED_BAND_SIZE];
    uint8_t valid_count = extract_subcarrier_amplitudes(
        csi_data, csi_len, subcarriers, num_subcarriers,
        amplitudes, HT20_SELECTED_BAND_SIZE);
    return calculate_spatial_turbulence_from_amplitudes(amplitudes, valid_count);
}

}  // namespace espectre
