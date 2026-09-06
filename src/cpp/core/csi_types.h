/*
 * ESPectre - CSI Public Types
 *
 * Stable HT20 dimensions and detector subcarrier selection shared by the
 * core-only and full-runtime SDK surfaces.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <array>
#include <cstdint>

namespace espectre {

constexpr uint16_t HT20_NUM_SUBCARRIERS = 64U;
constexpr uint16_t HT20_CSI_LEN = 128U;
constexpr uint16_t HT20_CSI_LEN_DOUBLE = 256U;
constexpr uint16_t HT20_CSI_LEN_SHORT = 114U;
constexpr uint16_t LLTF20_CSI_LEN_SHORT = 106U;
constexpr uint16_t HT20_CSI_LEN_SHORT_DOUBLE = 228U;
constexpr uint8_t HT20_CSI_LEN_SHORT_LEFT_PAD = 8U;
constexpr uint8_t HT20_GUARD_BAND_LOW = 4U;
constexpr uint8_t HT20_GUARD_BAND_HIGH = 60U;
constexpr uint8_t HT20_DC_SUBCARRIER = 32U;
constexpr uint8_t HT20_SELECTED_BAND_SIZE = 12U;

inline constexpr uint8_t DEFAULT_SUBCARRIERS[HT20_SELECTED_BAND_SIZE] = {
    4U, 8U, 13U, 18U, 23U, 28U, 36U, 41U, 46U, 51U, 56U, 60U,
};
using SelectedSubcarriers = std::array<uint8_t, HT20_SELECTED_BAND_SIZE>;

constexpr SelectedSubcarriers make_default_subcarriers() {
  SelectedSubcarriers subcarriers{};
  for (uint8_t i = 0U; i < HT20_SELECTED_BAND_SIZE; ++i) {
    subcarriers[i] = DEFAULT_SUBCARRIERS[i];
  }
  return subcarriers;
}

}  // namespace espectre
