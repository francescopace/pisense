/*
 * ESPectre - CSI Capture Service
 *
 * Enables ESP-IDF CSI capture, classifies/normalizes supported HT20 payloads,
 * and forwards valid packets.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "csi_capture_service.h"

#include <algorithm>
#include <cinttypes>

#include "csi_format.h"
#include "csi_platform_config.h"
#include "espectre_log.h"
#include "sdkconfig.h"

namespace espectre {

namespace {

static const char *const TAG = "CsiCapture";
static constexpr uint32_t DETECTOR_RESET_DROP_STREAK = 8U;

}  // namespace

void CsiCaptureService::init(IWiFiCSI *wifi_csi) {
  wifi_csi_ = wifi_csi ? wifi_csi : &default_wifi_csi_;
  reset_session();
}

void CsiCaptureService::reset_session() {
  filtered_packets_.store(0U, std::memory_order_relaxed);
  callback_invocations_.store(0U, std::memory_order_relaxed);
  null_or_empty_packets_.store(0U, std::memory_order_relaxed);
  normalized_invalid_packets_.store(0U, std::memory_order_relaxed);
  valid_packets_.store(0U, std::memory_order_relaxed);
  rejected_out_of_order_packets_.store(0U, std::memory_order_relaxed);
  unsupported_phy_packets_.store(0U, std::memory_order_relaxed);
  unsupported_width_packets_.store(0U, std::memory_order_relaxed);
  unexpected_ltf_packets_.store(0U, std::memory_order_relaxed);
  unknown_layout_packets_.store(0U, std::memory_order_relaxed);
  bad_length_packets_.store(0U, std::memory_order_relaxed);
  missing_metadata_packets_.store(0U, std::memory_order_relaxed);
  normalization_collapse_packets_.store(0U, std::memory_order_relaxed);
  normalization_remap_packets_.store(0U, std::memory_order_relaxed);
  enable_attempts_.store(0U, std::memory_order_relaxed);
  disable_attempts_.store(0U, std::memory_order_relaxed);
  last_configure_err_.store(ESP_OK, std::memory_order_relaxed);
  last_set_callback_err_.store(ESP_OK, std::memory_order_relaxed);
  last_set_enabled_err_.store(ESP_OK, std::memory_order_relaxed);
  last_disable_err_.store(ESP_OK, std::memory_order_relaxed);
  collapse_seen_.store(false, std::memory_order_relaxed);
  remap_seen_.store(false, std::memory_order_relaxed);
  rx_timestamp_tracker_.reset();
  collapse_log_event_.clear();
  remap_log_event_.clear();
  reset_channel_tracking_();
  last_assessment_ = {};
  consecutive_format_drops_ = 0U;
  last_accepted_normalization_tag_ = NormalizedCSIPayloadTag::NONE;
  bin_layout_ = Ht20BinLayout::UNKNOWN;
  has_accepted_packet_ = false;
}

void CsiCaptureService::reset_channel_tracking_() {
  current_channel_.store(0U, std::memory_order_relaxed);
  channel_change_pending_.store(false, std::memory_order_relaxed);
  channel_change_event_.clear();
}

void CsiCaptureService::loop() {
  if (collapse_log_event_.take()) {
    ESPECTRE_LOGI(TAG, "CSI double-length collapse active: 256->128 and/or 228->114");
  }
  if (remap_log_event_.take()) {
    ESPECTRE_LOGI(TAG, "CSI compact payload remap active: 53/57->64 SC");
  }
  uint8_t previous_channel = 0U;
  uint8_t current_channel = 0U;
  if (channel_change_event_.take(previous_channel, current_channel)) {
    ESPECTRE_LOGW(TAG, "Wi-Fi channel changed: %u -> %u, invalidating CSI capture session",
             static_cast<unsigned>(previous_channel), static_cast<unsigned>(current_channel));
    if (channel_change_callback_ != nullptr) {
      channel_change_callback_(channel_change_callback_context_, previous_channel, current_channel);
    } else {
      channel_change_pending_.store(false, std::memory_order_relaxed);
    }
  }
}

esp_err_t CsiCaptureService::enable(CsiCaptureProfile profile) {
  if (enabled_) {
    ESPECTRE_LOGW(TAG, "CSI already enabled");
    return ESP_OK;
  }

  capture_profile_ = profile;
  bin_layout_ = Ht20BinLayout::UNKNOWN;
  consecutive_format_drops_ = 0U;
  has_accepted_packet_ = false;
  last_accepted_normalization_tag_ = NormalizedCSIPayloadTag::NONE;
  const uint32_t attempt = enable_attempts_.fetch_add(1U, std::memory_order_relaxed) + 1U;
  ESPECTRE_LOGI(TAG, "Arming CSI attempt=%" PRIu32 " profile=%s", attempt,
               csi_capture_profile_name(capture_profile_));
  reset_channel_tracking_();
  rx_timestamp_tracker_.reset();

  // Follow Espressif's CSI initialization order: configure the measurement
  // engine, register the receive callback, and then enable the hardware path.
  esp_err_t err = configure_platform_specific_();
  last_configure_err_.store(err, std::memory_order_relaxed);
  if (err != ESP_OK) {
    ESPECTRE_LOGE(TAG, "Failed to configure CSI: %s", esp_err_to_name(err));
    return err;
  }

  err = wifi_csi_->set_csi_rx_cb(&CsiCaptureService::csi_rx_callback_wrapper_, this);
  last_set_callback_err_.store(err, std::memory_order_relaxed);
  if (err != ESP_OK) {
    ESPECTRE_LOGE(TAG, "Failed to set CSI callback: %s", esp_err_to_name(err));
    return err;
  }

  err = wifi_csi_->set_csi(true);
  last_set_enabled_err_.store(err, std::memory_order_relaxed);
  if (err != ESP_OK) {
    ESPECTRE_LOGE(TAG, "Failed to enable CSI: %s", esp_err_to_name(err));
    const esp_err_t rollback_err = wifi_csi_->set_csi_rx_cb(nullptr, nullptr);
    if (rollback_err != ESP_OK) {
      ESPECTRE_LOGE(TAG, "Failed to roll back CSI callback registration: %s", esp_err_to_name(rollback_err));
    }
    return err;
  }

  enabled_ = true;
  ESPECTRE_LOGI(TAG,
           "CSI armed attempt=%" PRIu32 " configure=%s set_cb=%s set_enabled=%s",
           attempt,
           esp_err_to_name(last_configure_err()),
           esp_err_to_name(last_set_callback_err()),
           esp_err_to_name(last_set_enabled_err()));
  return ESP_OK;
}

esp_err_t CsiCaptureService::disable() {
  if (!enabled_) {
    return ESP_OK;
  }

  const uint32_t attempt = disable_attempts_.fetch_add(1U, std::memory_order_relaxed) + 1U;
  // Stop capture before detaching the callback. In particular, the classic
  // ESP32 driver may accept a later re-enable without rebuilding its RX path
  // when callback removal made the preceding disable return INVALID_ARG.
  // The object remains alive throughout this method, so a final in-flight
  // callback is safe while the driver reaches its quiescent state.
  const esp_err_t disable_err = wifi_csi_->set_csi(false);
  esp_err_t callback_err = wifi_csi_->set_csi_rx_cb(nullptr, nullptr);
  if (callback_err != ESP_OK) {
    callback_err = wifi_csi_->set_csi_rx_cb(nullptr, nullptr);
    if (callback_err != ESP_OK) {
      callback_err = wifi_csi_->set_csi_rx_cb(
          &CsiCaptureService::disabled_csi_rx_callback_, nullptr);
      if (callback_err == ESP_OK) {
        ESPECTRE_LOGW(TAG, "Replaced retained CSI callback with a disabled trampoline");
      }
    }
  }
  if (callback_err != ESP_OK) {
    last_disable_err_.store(callback_err, std::memory_order_relaxed);
    ESPECTRE_LOGE(TAG, "Failed to unregister CSI callback after stopping capture: %s",
             esp_err_to_name(callback_err));
    return callback_err;
  }

  last_disable_err_.store(disable_err, std::memory_order_relaxed);
  if (disable_err != ESP_OK) {
    ESPECTRE_LOGE(TAG, "Failed to disable CSI before unregistering its callback: %s",
             esp_err_to_name(disable_err));
    return disable_err;
  }

  enabled_ = false;
  rx_timestamp_tracker_.reset();
  reset_channel_tracking_();
  ESPECTRE_LOGI(TAG, "CSI disabled attempt=%" PRIu32 " disable=%s", attempt, esp_err_to_name(last_disable_err()));
  return ESP_OK;
}

bool CsiCaptureService::accept_rx_timestamp_(const wifi_csi_info_t *data) {
  const uint32_t timestamp = data->rx_ctrl.timestamp;
  if (timestamp == 0U || rx_timestamp_tracker_.accept(timestamp)) {
    return true;
  }
  filtered_packets_.fetch_add(1U, std::memory_order_relaxed);
  rejected_out_of_order_packets_.fetch_add(1U, std::memory_order_relaxed);
  return false;
}

void CsiCaptureService::record_format_drop_(CsiFormatReasonCode reason_code) {
  filtered_packets_.fetch_add(1U, std::memory_order_relaxed);
  consecutive_format_drops_++;
  switch (reason_code) {
    case CsiFormatReasonCode::NULL_OR_EMPTY:
      // Already counted in null_or_empty_packets_ at the call site; do not
      // inflate the bad-length or normalization counters as well.
      break;
    case CsiFormatReasonCode::BAD_LENGTH:
      bad_length_packets_.fetch_add(1U, std::memory_order_relaxed);
      normalized_invalid_packets_.fetch_add(1U, std::memory_order_relaxed);
      break;
    case CsiFormatReasonCode::UNSUPPORTED_PHY:
      unsupported_phy_packets_.fetch_add(1U, std::memory_order_relaxed);
      break;
    case CsiFormatReasonCode::UNSUPPORTED_WIDTH:
      unsupported_width_packets_.fetch_add(1U, std::memory_order_relaxed);
      break;
    case CsiFormatReasonCode::UNEXPECTED_LTF:
      unexpected_ltf_packets_.fetch_add(1U, std::memory_order_relaxed);
      break;
    case CsiFormatReasonCode::UNKNOWN_LAYOUT:
      unknown_layout_packets_.fetch_add(1U, std::memory_order_relaxed);
      normalized_invalid_packets_.fetch_add(1U, std::memory_order_relaxed);
      break;
    case CsiFormatReasonCode::MISSING_METADATA:
      missing_metadata_packets_.fetch_add(1U, std::memory_order_relaxed);
      break;
    case CsiFormatReasonCode::NONE:
    default:
      break;
  }
}

void CsiCaptureService::process_packet(wifi_csi_info_t *data) {
  if (data == nullptr || data->buf == nullptr || data->len == 0U) {
    null_or_empty_packets_.fetch_add(1U, std::memory_order_relaxed);
    last_assessment_ = assess_ht20_sensing_format(data, capture_profile_);
    record_format_drop_(last_assessment_.reason_code);
    return;
  }

  CsiFormatAssessment assessment = assess_ht20_sensing_format(data, capture_profile_);
  if (!assessment.is_sensing_accepted()) {
    last_assessment_ = assessment;
    record_format_drop_(assessment.reason_code);
    return;
  }

  // Match the MicroPython runtime: the tag-change reset applies only once a
  // packet has been accepted, so the very first packet never forces a reset.
  const bool should_reset_detector =
      consecutive_format_drops_ >= DETECTOR_RESET_DROP_STREAK ||
      (has_accepted_packet_ && assessment.normalization_tag != last_accepted_normalization_tag_);
  assessment.reset_detector_before_consume = should_reset_detector;
  consecutive_format_drops_ = 0U;

  NormalizedCSIPayload normalized{data->buf, HT20_CSI_LEN, NormalizedCSIPayloadTag::NONE};
  if (assessment.requires_normalization()) {
    normalized = normalize_ht20_csi_payload(data->buf, data->len, remap_scratch_.data(), remap_scratch_.size());
  } else {
    normalized = {data->buf, HT20_CSI_LEN, NormalizedCSIPayloadTag::NONE};
  }

  // Bin ordering is independent of payload length: classic MACs deliver
  // "0~31, -32~-1" while Wi-Fi 6 parts deliver the centered convention that
  // DEFAULT_SUBCARRIERS assumes. Latch the first confident detection so a single
  // packet with a fully faded guard-adjacent tone cannot leave one frame
  // unrotated in an otherwise rotated stream.
  if (normalized.valid() && normalized.len == HT20_CSI_LEN) {
    const bool compact_lltf = normalized.tag == NormalizedCSIPayloadTag::LLTF53_TO_64;
    if (!compact_lltf && bin_layout_ == Ht20BinLayout::UNKNOWN) {
      const Ht20BinLayout detected =
          detect_ht20_bin_layout(normalized.data, normalized.len);
      if (detected != Ht20BinLayout::UNKNOWN) {
        bin_layout_ = detected;
      }
    }
    if (!compact_lltf && bin_layout_ == Ht20BinLayout::CLASSIC) {
      rotate_ht20_classic_to_centered(normalized.data, rotation_scratch_.data());
      normalized.data = rotation_scratch_.data();
      normalized.rotated_to_centered = true;
    }

    // Preserve the 64-bin raw contract after layout detection.
    if (csi_capture_profile_uses_lltf(capture_profile_)) {
      // Both normalization buffers are private to this callback. Reuse an
      // existing writable view; copy only when the view still belongs to Wi-Fi.
      int8_t *writable = normalized.data == rotation_scratch_.data()
                             ? rotation_scratch_.data() : remap_scratch_.data();
      if (normalized.data != writable) {
        std::copy_n(normalized.data, normalized.len, writable);
      }
      (void) zero_ht20_lltf_missing_bins(writable, normalized.len);
      normalized.data = writable;
    }
  }

  if (normalized.tag == NormalizedCSIPayloadTag::DOUBLE_HT20 ||
      normalized.tag == NormalizedCSIPayloadTag::DOUBLE_HT57_TO_64) {
    normalization_collapse_packets_.fetch_add(1U, std::memory_order_relaxed);
    if (!collapse_seen_.exchange(true, std::memory_order_relaxed)) {
      collapse_log_event_.post();
    }
  }

  if (normalized.tag == NormalizedCSIPayloadTag::LLTF53_TO_64 ||
      normalized.tag == NormalizedCSIPayloadTag::HT57_TO_64 ||
      normalized.tag == NormalizedCSIPayloadTag::DOUBLE_HT57_TO_64) {
    normalization_remap_packets_.fetch_add(1U, std::memory_order_relaxed);
    if (!remap_seen_.exchange(true, std::memory_order_relaxed)) {
      remap_log_event_.post();
    }
  }

  if (!normalized.valid() || normalized.len != HT20_CSI_LEN) {
    assessment.disposition = CsiFormatDisposition::DROP;
    assessment.reason_code = CsiFormatReasonCode::UNKNOWN_LAYOUT;
    last_assessment_ = assessment;
    record_format_drop_(assessment.reason_code);
    return;
  }

  if (!accept_rx_timestamp_(data)) {
    return;
  }

  normalized.reset_detector_before_consume = assessment.reset_detector_before_consume;
  last_assessment_ = assessment;
  const uint8_t packet_channel = data->rx_ctrl.channel;
  const uint8_t previous_channel = current_channel_.load(std::memory_order_relaxed);
  if (channel_change_pending_.load(std::memory_order_relaxed)) {
    filtered_packets_.fetch_add(1U, std::memory_order_relaxed);
    return;
  }
  if (previous_channel != 0U && packet_channel != previous_channel) {
    bin_layout_ = Ht20BinLayout::UNKNOWN;
    current_channel_.store(packet_channel, std::memory_order_relaxed);
    channel_change_pending_.store(true, std::memory_order_relaxed);
    channel_change_event_.post(previous_channel, packet_channel);
    filtered_packets_.fetch_add(1U, std::memory_order_relaxed);
    return;
  }
  current_channel_.store(packet_channel, std::memory_order_relaxed);
  last_accepted_normalization_tag_ = normalized.tag;
  has_accepted_packet_ = true;
  if (packet_callback_) {
    packet_callback_(packet_callback_context_, data, normalized);
  }
  valid_packets_.fetch_add(1U, std::memory_order_relaxed);
}

void IRAM_ATTR CsiCaptureService::csi_rx_callback_wrapper_(void *ctx, wifi_csi_info_t *data) {
  auto *service = static_cast<CsiCaptureService *>(ctx);
  if (service != nullptr) {
    service->callback_invocations_.fetch_add(1U, std::memory_order_relaxed);
    service->process_packet(data);
  }
}

void IRAM_ATTR CsiCaptureService::disabled_csi_rx_callback_(void *ctx,
                                                            wifi_csi_info_t *data) {
  (void) ctx;
  (void) data;
}

esp_err_t CsiCaptureService::configure_platform_specific_() {
#ifdef CONFIG_IDF_TARGET
  ESPECTRE_LOGI(TAG, "Using %s CSI configuration", CONFIG_IDF_TARGET);
#else
  ESPECTRE_LOGI(TAG, "Using host CSI configuration");
#endif
  return configure_csi(wifi_csi_, capture_profile_);
}

}  // namespace espectre
