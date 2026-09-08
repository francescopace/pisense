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
#pragma once

#include <array>
#include <atomic>

#include "csi_format_classifier.h"
#include "csi_payload_normalizer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "pending_event.h"
#include "serial_sequence.h"
#include "utils.h"
#include "wifi_csi_interface.h"

namespace espectre {

using csi_capture_packet_callback_t = void (*)(void *, const wifi_csi_info_t *, const NormalizedCSIPayload &);
using csi_capture_channel_change_callback_t = void (*)(void *, uint8_t, uint8_t);

class CsiCaptureService {
 public:
  void init(IWiFiCSI *wifi_csi = nullptr);
  void reset_session();

  esp_err_t enable(CsiCaptureProfile profile = CsiCaptureProfile::HT20);
  esp_err_t disable();
  void loop();
  void process_packet(wifi_csi_info_t *data);

  bool is_enabled() const { return enabled_; }
  CsiCaptureProfile capture_profile() const { return capture_profile_; }
  uint32_t filtered_packets() const { return filtered_packets_.load(std::memory_order_relaxed); }
  uint32_t callback_invocations() const { return callback_invocations_.load(std::memory_order_relaxed); }
  uint32_t null_or_empty_packets() const { return null_or_empty_packets_.load(std::memory_order_relaxed); }
  uint32_t normalized_invalid_packets() const {
    return normalized_invalid_packets_.load(std::memory_order_relaxed);
  }
  uint32_t valid_packets() const { return valid_packets_.load(std::memory_order_relaxed); }
  uint32_t rejected_out_of_order_packets() const {
    return rejected_out_of_order_packets_.load(std::memory_order_relaxed);
  }
  uint32_t unsupported_phy_packets() const { return unsupported_phy_packets_.load(std::memory_order_relaxed); }
  uint32_t unsupported_width_packets() const {
    return unsupported_width_packets_.load(std::memory_order_relaxed);
  }
  uint32_t unexpected_ltf_packets() const { return unexpected_ltf_packets_.load(std::memory_order_relaxed); }
  uint32_t unknown_layout_packets() const { return unknown_layout_packets_.load(std::memory_order_relaxed); }
  uint32_t bad_length_packets() const { return bad_length_packets_.load(std::memory_order_relaxed); }
  uint32_t missing_metadata_packets() const {
    return missing_metadata_packets_.load(std::memory_order_relaxed);
  }
  uint32_t rx_error_packets() const {
    return rx_error_packets_.load(std::memory_order_relaxed);
  }
  uint32_t rx_end_error_packets() const {
    return rx_end_error_packets_.load(std::memory_order_relaxed);
  }
  uint32_t invalid_estimate_packets() const {
    return invalid_estimate_packets_.load(std::memory_order_relaxed);
  }
  uint32_t invalid_first_word_packets() const {
    return invalid_first_word_packets_.load(std::memory_order_relaxed);
  }
  uint32_t sanitized_first_word_packets() const {
    return sanitized_first_word_packets_.load(std::memory_order_relaxed);
  }
  uint32_t estimate_length_mismatch_packets() const {
    return estimate_length_mismatch_packets_.load(std::memory_order_relaxed);
  }
  uint32_t normalization_collapse_packets() const {
    return normalization_collapse_packets_.load(std::memory_order_relaxed);
  }
  uint32_t normalization_remap_packets() const {
    return normalization_remap_packets_.load(std::memory_order_relaxed);
  }
  const CsiFormatAssessment &last_assessment() const { return last_assessment_; }
  uint32_t enable_attempts() const { return enable_attempts_.load(std::memory_order_relaxed); }
  uint32_t disable_attempts() const { return disable_attempts_.load(std::memory_order_relaxed); }
  esp_err_t last_configure_err() const { return static_cast<esp_err_t>(last_configure_err_.load(std::memory_order_relaxed)); }
  esp_err_t last_set_callback_err() const {
    return static_cast<esp_err_t>(last_set_callback_err_.load(std::memory_order_relaxed));
  }
  esp_err_t last_set_enabled_err() const {
    return static_cast<esp_err_t>(last_set_enabled_err_.load(std::memory_order_relaxed));
  }
  esp_err_t last_disable_err() const { return static_cast<esp_err_t>(last_disable_err_.load(std::memory_order_relaxed)); }

  void set_packet_callback(csi_capture_packet_callback_t callback, void *context = nullptr) {
    packet_callback_ = callback;
    packet_callback_context_ = context;
  }

  void set_channel_change_callback(csi_capture_channel_change_callback_t callback,
                                   void *context = nullptr) {
    channel_change_callback_ = callback;
    channel_change_callback_context_ = context;
  }

 private:
  static void IRAM_ATTR csi_rx_callback_wrapper_(void *ctx, wifi_csi_info_t *data);
  static void IRAM_ATTR disabled_csi_rx_callback_(void *ctx, wifi_csi_info_t *data);
  esp_err_t configure_platform_specific_();
  bool accept_rx_timestamp_(const wifi_csi_info_t *data);
  void record_format_drop_(CsiFormatReasonCode reason_code);
  void reset_channel_tracking_();

  bool enabled_{false};
  CsiCaptureProfile capture_profile_{CsiCaptureProfile::HT20};
  IWiFiCSI *wifi_csi_{nullptr};
  WiFiCSIReal default_wifi_csi_;
  csi_capture_packet_callback_t packet_callback_{nullptr};
  void *packet_callback_context_{nullptr};
  csi_capture_channel_change_callback_t channel_change_callback_{nullptr};
  void *channel_change_callback_context_{nullptr};
  std::atomic<uint32_t> filtered_packets_{0U};
  std::atomic<uint32_t> callback_invocations_{0U};
  std::atomic<uint32_t> null_or_empty_packets_{0U};
  std::atomic<uint32_t> normalized_invalid_packets_{0U};
  std::atomic<uint32_t> valid_packets_{0U};
  std::atomic<uint32_t> rejected_out_of_order_packets_{0U};
  std::atomic<uint32_t> unsupported_phy_packets_{0U};
  std::atomic<uint32_t> unsupported_width_packets_{0U};
  std::atomic<uint32_t> unexpected_ltf_packets_{0U};
  std::atomic<uint32_t> unknown_layout_packets_{0U};
  std::atomic<uint32_t> bad_length_packets_{0U};
  std::atomic<uint32_t> missing_metadata_packets_{0U};
  std::atomic<uint32_t> rx_error_packets_{0U};
  std::atomic<uint32_t> rx_end_error_packets_{0U};
  std::atomic<uint32_t> invalid_estimate_packets_{0U};
  std::atomic<uint32_t> invalid_first_word_packets_{0U};
  std::atomic<uint32_t> sanitized_first_word_packets_{0U};
  std::atomic<uint32_t> estimate_length_mismatch_packets_{0U};

  std::atomic<uint32_t> normalization_collapse_packets_{0U};
  std::atomic<uint32_t> normalization_remap_packets_{0U};
  std::atomic<uint32_t> enable_attempts_{0U};
  std::atomic<uint32_t> disable_attempts_{0U};
  std::atomic<int32_t> last_configure_err_{ESP_OK};
  std::atomic<int32_t> last_set_callback_err_{ESP_OK};
  std::atomic<int32_t> last_set_enabled_err_{ESP_OK};
  std::atomic<int32_t> last_disable_err_{ESP_OK};
  std::atomic<bool> collapse_seen_{false};
  std::atomic<bool> remap_seen_{false};
  SerialSequenceTracker rx_timestamp_tracker_;
  PendingEvent<> collapse_log_event_;
  PendingEvent<> remap_log_event_;
  PendingEvent<uint8_t, uint8_t> channel_change_event_;
  // Capture is single-producer; keeping normalization scratch on the service
  // avoids reserving two HT20 buffers on every Wi-Fi callback stack frame.
  std::array<int8_t, HT20_CSI_LEN> remap_scratch_{};
  std::array<int8_t, HT20_CSI_LEN> rotation_scratch_{};
  CsiFormatAssessment last_assessment_{};
  uint32_t consecutive_format_drops_{0U};
  NormalizedCSIPayloadTag last_accepted_normalization_tag_{NormalizedCSIPayloadTag::NONE};
  // Latched HT20 bin ordering for this radio. Detection needs a fully populated
  // guard set, so it can come back UNKNOWN on an individual packet; reusing the
  // last confident answer keeps the stream internally consistent.
  Ht20BinLayout bin_layout_{Ht20BinLayout::UNKNOWN};
  bool has_accepted_packet_{false};
  std::atomic<uint8_t> current_channel_{0U};
  std::atomic<bool> channel_change_pending_{false};
};

}  // namespace espectre
