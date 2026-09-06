/*
 * ESPectre - CSI Payload Normalizer
 *
 * Normalizes ESP-IDF CSI payloads into the internal HT20 layout used by
 * ESPectre.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace espectre {

enum class NormalizedCSIPayloadTag : uint8_t {
  NONE = 0,
  DOUBLE_HT20,
  HT57_TO_64,
  DOUBLE_HT57_TO_64,
  LLTF53_TO_64,
};

struct NormalizedCSIPayload {
  const int8_t *data{nullptr};
  size_t len{0};
  // Describes how the payload length was normalized. Bin ordering is tracked
  // separately for full-width payloads. Named compact layouts have a fixed
  // input ordering and are normalized directly into centered bins.
  NormalizedCSIPayloadTag tag{NormalizedCSIPayloadTag::NONE};
  // True when the payload arrived in Espressif's classic bin order and was
  // rotated into the centered convention that `DEFAULT_SUBCARRIERS` assumes.
  bool rotated_to_centered{false};
  // True when a format transition or a long invalid streak requires detector
  // history to be cleared before this accepted packet is consumed.
  bool reset_detector_before_consume{false};

  bool valid() const { return data != nullptr; }
};

NormalizedCSIPayload normalize_ht20_csi_payload(const int8_t *csi_data,
                                                size_t csi_len,
                                                int8_t *remap_buffer,
                                                size_t remap_buffer_len);

const char *normalized_csi_payload_tag_to_string(NormalizedCSIPayloadTag tag);

}  // namespace espectre
