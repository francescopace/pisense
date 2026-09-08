/*
 * ESPectre - CSI Format Classifier
 *
 * Classifies CSI packets before any HT20 normalization so sensing can enforce
 * one explicit production contract.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "csi_format.h"
#include "csi_capture_profile.h"
#include "csi_payload_normalizer.h"
#include "csi_phy_filter.h"
#include "esp_wifi.h"

namespace espectre {

enum class CsiFormatId : uint8_t {
  UNKNOWN = 0,
  HT20 = 1,
  VHT20 = 2,
};

enum class CsiLayoutId : uint8_t {
  UNKNOWN = 0,
  HT20_64 = 1,
  HT20_57 = 2,
  HT20_64_DOUBLE = 3,
  HT20_57_DOUBLE = 4,
  LLTF20_53 = 5,
};

enum class CsiPayloadView : uint8_t {
  RAW = 0,
  NORMALIZED = 1,
};

enum class CsiMetadataSource : uint8_t {
  UNKNOWN = 0,
  WIFI_RX_CTRL = 1,
};

enum class CsiFormatDisposition : uint8_t {
  DROP = 0,
  SENSE = 1,
};

enum class CsiFormatReasonCode : uint8_t {
  NONE = 0,
  NULL_OR_EMPTY = 1,
  BAD_LENGTH = 2,
  UNSUPPORTED_PHY = 3,
  UNSUPPORTED_WIDTH = 4,
  // Host-only in practice: ESP-IDF rx_ctrl metadata cannot separate the LTF
  // from the PHY decision, so the firmware classifier reports UNSUPPORTED_PHY
  // instead. Kept for parity with the host reason vocabulary.
  UNEXPECTED_LTF = 5,
  UNKNOWN_LAYOUT = 6,
  MISSING_METADATA = 7,
  RX_ERROR = 8,
  RX_END_ERROR = 9,
  INVALID_ESTIMATE = 10,
  INVALID_FIRST_WORD = 11,
};

struct CsiFormatAssessment {
  CsiFormatId format_id{CsiFormatId::UNKNOWN};
  CsiLayoutId layout_id{CsiLayoutId::UNKNOWN};
  CsiMetadataSource metadata_source{CsiMetadataSource::UNKNOWN};
  CsiPayloadView payload_view{CsiPayloadView::RAW};
  CsiFormatDisposition disposition{CsiFormatDisposition::DROP};
  CsiFormatReasonCode reason_code{CsiFormatReasonCode::BAD_LENGTH};
  NormalizedCSIPayloadTag normalization_tag{NormalizedCSIPayloadTag::NONE};
  uint16_t raw_len{0U};
  uint16_t raw_num_subcarriers{0U};
  uint16_t normalized_len{0U};
  uint16_t normalized_num_subcarriers{0U};
  bool reset_detector_before_consume{false};

  bool is_sensing_accepted() const { return disposition == CsiFormatDisposition::SENSE; }
  bool requires_normalization() const { return payload_view == CsiPayloadView::NORMALIZED; }
};

inline const char *csi_format_reason_code_to_string(CsiFormatReasonCode code) {
  switch (code) {
    case CsiFormatReasonCode::NONE:
      return "none";
    case CsiFormatReasonCode::NULL_OR_EMPTY:
      return "null_or_empty";
    case CsiFormatReasonCode::BAD_LENGTH:
      return "bad_length";
    case CsiFormatReasonCode::UNSUPPORTED_PHY:
      return "unsupported_phy";
    case CsiFormatReasonCode::UNSUPPORTED_WIDTH:
      return "unsupported_width";
    case CsiFormatReasonCode::UNEXPECTED_LTF:
      return "unexpected_ltf";
    case CsiFormatReasonCode::UNKNOWN_LAYOUT:
      return "unknown_layout";
    case CsiFormatReasonCode::MISSING_METADATA:
      return "missing_metadata";
    case CsiFormatReasonCode::RX_ERROR:
      return "rx_error";
    case CsiFormatReasonCode::RX_END_ERROR:
      return "rx_end_error";
    case CsiFormatReasonCode::INVALID_ESTIMATE:
      return "invalid_estimate";
    case CsiFormatReasonCode::INVALID_FIRST_WORD:
      return "invalid_first_word";
    default:
      return "unknown";
  }
}

inline bool csi_info_is_legacy_lltf(const wifi_csi_info_t *info,
                                    CsiCaptureProfile profile) {
#if CONFIG_SOC_WIFI_HE_SUPPORT
  return info != nullptr && csi_capture_profile_uses_lltf(profile) &&
         info->rx_ctrl.cur_bb_format == RX_BB_FORMAT_11G && info->rx_ctrl.second == 0U;
#else
  return info != nullptr && csi_capture_profile_uses_lltf(profile) &&
         info->rx_ctrl.sig_mode == 0U && info->rx_ctrl.cwb == 0U;
#endif
}

inline CsiFormatAssessment assess_ht20_sensing_format(
    const wifi_csi_info_t *info,
    CsiCaptureProfile profile = CsiCaptureProfile::HT20) {
  CsiFormatAssessment assessment{};
  if (info == nullptr) {
    assessment.reason_code = CsiFormatReasonCode::NULL_OR_EMPTY;
    return assessment;
  }

  assessment.metadata_source = CsiMetadataSource::WIFI_RX_CTRL;
  assessment.raw_len = info->len;
  assessment.raw_num_subcarriers = (assessment.raw_len % 2U) == 0U
      ? static_cast<uint16_t>(assessment.raw_len / 2U) : 0U;

  // Hardware faults take priority over malformed payloads: unrelated bad
  // reception must not contribute to the format-change reset streak.
  if (info->rx_ctrl.rx_state != 0U) {
    assessment.reason_code = CsiFormatReasonCode::RX_ERROR;
    return assessment;
  }
#if CONFIG_SOC_WIFI_HE_SUPPORT
  if (info->rx_ctrl.rxend_state != 0U) {
    assessment.reason_code = CsiFormatReasonCode::RX_END_ERROR;
    return assessment;
  }
  if (info->rx_ctrl.rx_channel_estimate_info_vld == 0U) {
    assessment.reason_code = CsiFormatReasonCode::INVALID_ESTIMATE;
    return assessment;
  }
#endif

  if (info->first_word_invalid) {
    const bool full_width = info->len == HT20_CSI_LEN || info->len == HT20_CSI_LEN * 2U;
    // Compact layouts and classic ordering map invalid source pairs onto live
    // tones. Only independently identified centered guard pairs are safe.
    if (info->buf == nullptr || !full_width ||
        detect_ht20_bin_layout(info->buf, HT20_CSI_LEN, true) != Ht20BinLayout::CENTERED) {
      assessment.reason_code = CsiFormatReasonCode::INVALID_FIRST_WORD;
      return assessment;
    }
  }

  if (info->buf == nullptr || info->len == 0U) {
    assessment.reason_code = CsiFormatReasonCode::NULL_OR_EMPTY;
    return assessment;
  }

  if ((assessment.raw_len % 2U) != 0U) {
    assessment.reason_code = CsiFormatReasonCode::BAD_LENGTH;
    return assessment;
  }

  const bool is_ht20 = profile != CsiCaptureProfile::VHT20 &&
                       csi_info_is_ht20_sensing(info);
  const bool is_vht20 = profile == CsiCaptureProfile::VHT20 &&
                        csi_info_is_vht20_sensing(info);
  const bool is_legacy_lltf = csi_info_is_legacy_lltf(info, profile);
  if (!is_ht20 && !is_vht20 && !is_legacy_lltf) {
    const bool is_selected_phy =
#if CONFIG_SOC_WIFI_HE_SUPPORT
        (profile == CsiCaptureProfile::VHT20 &&
         info->rx_ctrl.cur_bb_format == RX_BB_FORMAT_VHT) ||
        (profile != CsiCaptureProfile::VHT20 &&
         info->rx_ctrl.cur_bb_format == RX_BB_FORMAT_HT) ||
        (profile == CsiCaptureProfile::LLTF20 &&
         info->rx_ctrl.cur_bb_format == RX_BB_FORMAT_11G);
#else
        (profile == CsiCaptureProfile::HT20 && info->rx_ctrl.sig_mode == 1U) ||
        (profile == CsiCaptureProfile::LLTF20 &&
         (info->rx_ctrl.sig_mode == 0U || info->rx_ctrl.sig_mode == 1U));
#endif
    assessment.reason_code =
        is_selected_phy ? CsiFormatReasonCode::UNSUPPORTED_WIDTH
                        : CsiFormatReasonCode::UNSUPPORTED_PHY;
    return assessment;
  }

  assessment.format_id = is_vht20 ? CsiFormatId::VHT20 : CsiFormatId::HT20;
  assessment.disposition = CsiFormatDisposition::SENSE;
  assessment.reason_code = CsiFormatReasonCode::NONE;
  assessment.normalized_len = HT20_CSI_LEN;
  assessment.normalized_num_subcarriers = HT20_NUM_SUBCARRIERS;

  if (is_legacy_lltf && assessment.raw_len == LLTF20_CSI_LEN_SHORT) {
    assessment.layout_id = CsiLayoutId::LLTF20_53;
    assessment.payload_view = CsiPayloadView::NORMALIZED;
    assessment.normalization_tag = NormalizedCSIPayloadTag::LLTF53_TO_64;
    return assessment;
  }

  if (is_legacy_lltf && assessment.raw_len != HT20_CSI_LEN) {
    assessment.disposition = CsiFormatDisposition::DROP;
    assessment.reason_code = CsiFormatReasonCode::UNKNOWN_LAYOUT;
    assessment.normalized_len = 0U;
    assessment.normalized_num_subcarriers = 0U;
    return assessment;
  }

  switch (assessment.raw_len) {
    case HT20_CSI_LEN:
      assessment.layout_id = CsiLayoutId::HT20_64;
      assessment.payload_view = CsiPayloadView::RAW;
      assessment.normalization_tag = NormalizedCSIPayloadTag::NONE;
      return assessment;
    case HT20_CSI_LEN_SHORT:
      assessment.layout_id = CsiLayoutId::HT20_57;
      assessment.payload_view = CsiPayloadView::NORMALIZED;
      assessment.normalization_tag = NormalizedCSIPayloadTag::HT57_TO_64;
      return assessment;
    case HT20_CSI_LEN_DOUBLE:
      assessment.layout_id = CsiLayoutId::HT20_64_DOUBLE;
      assessment.payload_view = CsiPayloadView::NORMALIZED;
      assessment.normalization_tag = NormalizedCSIPayloadTag::DOUBLE_HT20;
      return assessment;
    case HT20_CSI_LEN_SHORT_DOUBLE:
      assessment.layout_id = CsiLayoutId::HT20_57_DOUBLE;
      assessment.payload_view = CsiPayloadView::NORMALIZED;
      assessment.normalization_tag = NormalizedCSIPayloadTag::DOUBLE_HT57_TO_64;
      return assessment;
    default:
      assessment.disposition = CsiFormatDisposition::DROP;
      assessment.reason_code = CsiFormatReasonCode::UNKNOWN_LAYOUT;
      assessment.normalized_len = 0U;
      assessment.normalized_num_subcarriers = 0U;
      return assessment;
  }
}

}  // namespace espectre
