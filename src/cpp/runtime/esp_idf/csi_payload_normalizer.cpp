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
#include "csi_payload_normalizer.h"

#include <cstring>

#include "csi_format.h"

namespace espectre {

NormalizedCSIPayload normalize_ht20_csi_payload(const int8_t *csi_data,
                                                size_t csi_len,
                                                int8_t *remap_buffer,
                                                size_t remap_buffer_len) {
  if (csi_data == nullptr) {
    return {};
  }

  NormalizedCSIPayloadTag tag = NormalizedCSIPayloadTag::NONE;
  if (csi_len == HT20_CSI_LEN_DOUBLE) {
    csi_len = HT20_CSI_LEN;
    tag = NormalizedCSIPayloadTag::DOUBLE_HT20;
  } else if (csi_len == HT20_CSI_LEN_SHORT_DOUBLE) {
    csi_len = HT20_CSI_LEN_SHORT;
    tag = NormalizedCSIPayloadTag::DOUBLE_HT57_TO_64;
  }

  if (csi_len == HT20_CSI_LEN) {
    return {csi_data, HT20_CSI_LEN, tag, false};
  }

  if (remap_buffer == nullptr || remap_buffer_len < HT20_CSI_LEN) {
    return {};
  }

  if (csi_len == LLTF20_CSI_LEN_SHORT) {
    // C5 compact LLTF is already centered: -26..+26, with DC at pair 26.
    // Pad six bins on the left and five on the right to keep DC at bin 32.
    std::memset(remap_buffer, 0, HT20_CSI_LEN);
    std::memcpy(remap_buffer + (HT20_DC_SUBCARRIER - 26U) * 2U,
                csi_data, LLTF20_CSI_LEN_SHORT);
    return {remap_buffer, HT20_CSI_LEN, NormalizedCSIPayloadTag::LLTF53_TO_64, false};
  }

  if (csi_len != HT20_CSI_LEN_SHORT) {
    return {};
  }

  // The short layout is already centered: 57 subcarriers spanning -28..+28 land
  // at bins 4..60, which puts DC at bin 32.
  std::memset(remap_buffer, 0, HT20_CSI_LEN);
  std::memcpy(&remap_buffer[HT20_CSI_LEN_SHORT_LEFT_PAD], csi_data, HT20_CSI_LEN_SHORT);
  if (tag == NormalizedCSIPayloadTag::NONE) {
    tag = NormalizedCSIPayloadTag::HT57_TO_64;
  }

  return {remap_buffer, HT20_CSI_LEN, tag, false};
}

const char *normalized_csi_payload_tag_to_string(NormalizedCSIPayloadTag tag) {
  switch (tag) {
    case NormalizedCSIPayloadTag::NONE:
      return "none";
    case NormalizedCSIPayloadTag::DOUBLE_HT20:
      return "double_ht20";
    case NormalizedCSIPayloadTag::HT57_TO_64:
      return "ht57_to_64";
    case NormalizedCSIPayloadTag::DOUBLE_HT57_TO_64:
      return "double_ht57_to_64";
    case NormalizedCSIPayloadTag::LLTF53_TO_64:
      return "lltf53_to_64";
    default:
      return "unknown";
  }
}

}  // namespace espectre
