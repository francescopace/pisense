/*
 * ESPectre - CSI Pipeline
 *
 * Runs CSI capture, detector evaluation, and motion-state publishing for
 * sensing runtimes.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>

#include "base_detector.h"
#include "csi_capture_service.h"
#include "csi_frame_identity.h"
#include "evaluation_cadence.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "csi_format.h"
#include "pending_event.h"
#include "pending_queue.h"
#include "raw_csi.h"
#include "runtime_sensing_schema.h"
#include "temporal_csi_sampler.h"
#include "wifi_csi_interface.h"

namespace espectre {

struct DetectionTimingStats {
  uint64_t duration_sum_us{0U};
  uint32_t samples{0U};
  uint32_t minimum_us{0U};
  uint32_t maximum_us{0U};
};

class PendingDetectionTiming {
 public:
  void record(uint32_t duration_us) {
    std::lock_guard<detail::PendingEventLock> lock(lock_);
    stats_.duration_sum_us += duration_us;
    stats_.minimum_us = stats_.samples == 0U ? duration_us : std::min(stats_.minimum_us, duration_us);
    stats_.maximum_us = std::max(stats_.maximum_us, duration_us);
    stats_.samples++;
  }

  bool take(DetectionTimingStats *stats) {
    if (stats == nullptr) {
      return false;
    }
    std::lock_guard<detail::PendingEventLock> lock(lock_);
    if (stats_.samples == 0U) {
      return false;
    }
    *stats = stats_;
    stats_ = {};
    return true;
  }

  void clear() {
    std::lock_guard<detail::PendingEventLock> lock(lock_);
    stats_ = {};
  }

 private:
  detail::PendingEventLock lock_{};
  DetectionTimingStats stats_{};
};

// Callback type for processed CSI data
using csi_processed_callback_t = std::function<void(MotionState, uint32_t)>;

// Callback type for immediate motion-state changes
using motion_state_callback_t = std::function<void(MotionState)>;

// Callback type for live telemetry updates emitted on evaluation ticks.
using live_telemetry_callback_t = std::function<void(float movement, float threshold)>;

// Callback type for Wi-Fi channel changes observed by the CSI stream. The
// callback is deferred to loop() so runtimes can safely rearm CSI hardware.
using channel_change_callback_t = std::function<void(uint8_t previous_channel, uint8_t current_channel)>;

// Callback type for intercepting normalized CSI packets before detector
// processing. The interceptor is told whether this packet closes an evaluation
// window and how many packets that window covers, so it evaluates on the same
// cadence the detection path does instead of counting packets itself. A true
// temporal_reset marks a contaminating gap that cleared detector history.
using csi_packet_interceptor_t = bool (*)(void *, const int8_t *, size_t, int8_t,
                                          bool, uint32_t, bool);

/**
 * CSI Pipeline
 * 
 * Manages complete CSI pipeline: hardware configuration, data processing, and motion detection.
 * Handles platform-specific differences between ESP32-C6 and ESP32-S3.
 * Orchestrates CSI packet processing and band calibration.
 */
class CsiPipeline {
 public:
  /**
   * Initialize CSI Pipeline
   * 
   * @param detector Motion detector instance (BaseDetector*)
   * @param wifi_csi WiFi CSI interface (nullptr for real implementation)
   */
  void init(BaseDetector* detector, IWiFiCSI* wifi_csi = nullptr);
  
  /**
   * Update segmentation threshold
   * 
   * @param threshold New threshold value
   */
  bool set_threshold(float threshold);
  void set_detector(BaseDetector *detector);
  void set_evaluation_interval_ms(uint32_t interval_ms) { cadence_.set_interval_ms(interval_ms); }
  bool set_segmentation_window_size_ms(uint32_t window_size_ms) {
    if (!sampler_.configure(sampler_.target_pps(), window_size_ms)) {
      return false;
    }
    cadence_.set_window_size_ms(window_size_ms);
    pending_candidate_valid_ = false;
    if (detector_ != nullptr) {
      detector_->set_minimum_valid_samples(
          static_cast<uint16_t>(sampler_.minimum_valid_slots()));
    }
    return true;
  }
  bool set_csi_target_pps(uint32_t target_pps) {
    if (!sampler_.configure(target_pps, sampler_.window_size_ms())) {
      return false;
    }
    pending_candidate_valid_ = false;
    if (detector_ != nullptr) {
      detector_->set_minimum_valid_samples(
          static_cast<uint16_t>(sampler_.minimum_valid_slots()));
    }
    return true;
  }
  void set_motion_on_hits(uint8_t hits) { motion_on_hits_ = hits > 0 ? hits : 1; }
  void set_motion_off_hits(uint8_t hits) { motion_off_hits_ = hits > 0 ? hits : 1; }
  void set_motion_hit_thresholds(uint8_t motion_on_hits, uint8_t motion_off_hits, bool reset_filter = false);
  
  /**
   * Enable CSI hardware and start processing
   * 
   * @param packet_callback Callback to invoke at the configured time interval
   * @return ESP_OK on success
   */
  esp_err_t enable(csi_processed_callback_t packet_callback = nullptr,
                   CsiCaptureProfile profile = CsiCaptureProfile::HT20);
  
  /**
   * Disable CSI hardware
   * 
   * @return ESP_OK on success
   */
  esp_err_t disable();

  /** Change the active capture profile, clearing samples and retaining callbacks. */
  esp_err_t reconfigure_capture(CsiCaptureProfile profile);

  CsiCaptureProfile capture_profile() const {
    return capture_service_.capture_profile();
  }

  /** Drain diagnostics and frontend notifications from the runtime loop. */
  void loop();

  /** Emit the periodic heartbeat when its monotonic deadline is due. */
  void heartbeat_if_due(uint32_t now_ms);
  
  /**
   * Process incoming CSI packet
   * 
   * Orchestrates: calibration check → processing → callbacks
   * 
   * @param data CSI packet data
   */
  void process_packet(wifi_csi_info_t* data);
  /** Emit the final buffered slot when a finite stream has ended. */
  bool flush_pending_candidate();
  
  /**
   * Set an optional packet interceptor.
   *
   * When present, normalized CSI packets are offered to the interceptor before
   * the detector sees them. Returning true consumes the packet.
   */
  void set_packet_interceptor(csi_packet_interceptor_t interceptor, void *context = nullptr) {
    packet_interceptor_ = interceptor;
    packet_interceptor_context_ = context;
  }

  /** Route normalized capture directly to a bounded raw consumer. */
  bool start_raw_capture(raw_csi_packet_callback_t callback, void *context = nullptr);
  /** Stop the pre-sampler raw branch and clear all transient capture state. */
  void stop_raw_capture();
  bool raw_capture_active() const {
    return raw_packet_callback_.load(std::memory_order_acquire) != nullptr;
  }
  
  /**
   * Check if CSI is currently enabled
   */
  bool is_enabled() const { return enabled_; }
  /** Whether an admitted detector input was processed within the current window. */
  bool has_current_detector_input(int64_t now_us) const {
    return enabled_ && has_detector_input_ && now_us >= last_detector_input_us_ &&
           static_cast<uint64_t>(now_us - last_detector_input_us_) <
               static_cast<uint64_t>(sampler_.window_size_ms()) * 1000U;
  }
  uint64_t accepted_packets_total() const {
    return accepted_packets_total_.load(std::memory_order_relaxed);
  }
  uint64_t detector_admitted_packets_total() const {
    return sampler_.accepted_packets();
  }
  uint64_t detector_excess_packets_total() const {
    return sampler_.excess_packets();
  }
  uint64_t detector_missing_slots_total() const {
    return sampler_.missing_slots();
  }
  uint64_t detector_stale_packets_total() const {
    return sampler_.stale_packets();
  }
  uint64_t detector_out_of_order_packets_total() const {
    return sampler_.out_of_order_packets();
  }
  uint32_t detector_window_occupancy_slots() const {
    return sampler_.occupancy_slots();
  }
  uint32_t detector_window_slots() const { return sampler_.window_slots(); }
  uint64_t rejected_out_of_order_packets_total() const {
    return capture_service_.rejected_out_of_order_packets();
  }
  uint64_t capture_callback_invocations_total() const {
    return capture_service_.callback_invocations();
  }
  uint64_t capture_filtered_packets_total() const {
    return capture_service_.filtered_packets();
  }
  uint64_t capture_rx_error_total() const {
    return capture_service_.rx_error_packets();
  }
  uint64_t capture_rx_end_error_total() const {
    return capture_service_.rx_end_error_packets();
  }
  uint64_t capture_invalid_estimate_total() const {
    return capture_service_.invalid_estimate_packets();
  }
  uint64_t capture_invalid_first_word_total() const {
    return capture_service_.invalid_first_word_packets();
  }
  uint64_t capture_sanitized_first_word_total() const {
    return capture_service_.sanitized_first_word_packets();
  }
  uint64_t capture_estimate_length_mismatch_total() const {
    return capture_service_.estimate_length_mismatch_packets();
  }
  uint64_t pending_frame_drops_total() const {
    return pending_frame_drops_.load(std::memory_order_relaxed);
  }
  size_t pending_frame_count() const { return pending_frames_.size(); }
  static constexpr size_t pending_frame_capacity() { return kPendingCsiFrameCapacity; }
  uint64_t traffic_classified_packets_total() const {
    return traffic_classified_packets_total_.load(std::memory_order_relaxed);
  }
  uint64_t traffic_rejected_packets_total() const {
    return traffic_rejected_packets_total_.load(std::memory_order_relaxed);
  }
  /** RSSI of the most recently accepted CSI packet. */
  int8_t last_rssi_dbm() const { return last_rssi_dbm_; }
  /** Channel the most recently accepted CSI packet arrived on. */
  uint8_t last_channel() const { return last_channel_; }
  /**
   * Set callback for live telemetry updates.
   */
  void set_live_telemetry_callback(live_telemetry_callback_t callback) {
    live_telemetry_callback_ = callback;
  }
  
  /**
   * Set callback for immediate motion-state changes.
   */
  void set_motion_state_callback(motion_state_callback_t callback) {
    motion_state_callback_ = callback;
  }

  void set_channel_change_callback(channel_change_callback_t callback) {
    channel_change_callback_ = callback;
  }

  /**
   * Get the detector instance
   */
  BaseDetector* get_detector() { return detector_; }
  
  /**
   * Clear detector buffer (for calibration reset)
   */
  void clear_detector_buffer();
  void set_traffic_filter(const CsiFrameFilterConfig &config);
  bool take_detection_timing(DetectionTimingStats *stats);
  
 private:
  struct PendingCsiFrame {
    wifi_pkt_rx_ctrl_t rx_ctrl{};
    std::array<int8_t, HT20_CSI_LEN> csi{};
    uint32_t callback_time_us{0U};
    uint16_t len{0U};
    bool reset_detector_before_consume{false};
  };

  struct PendingCsiCandidate {
    std::array<int8_t, HT20_CSI_LEN> csi{};
    size_t len{0U};
    uint32_t timestamp_us{0U};
    int8_t rssi_dbm{INT8_MIN};
  };

  void process_pending_frame_(const PendingCsiFrame &frame);
  void drain_pending_frames_();
  void process_admitted_candidate_();
  void apply_gap_history_reset_();
  void store_candidate_(const wifi_csi_info_t *data,
                        const NormalizedCSIPayload &normalized);
  static void capture_packet_callback_(void *context,
                                       const wifi_csi_info_t *data,
                                       const NormalizedCSIPayload &normalized);
  static void capture_channel_change_callback_(void *context,
                                               uint8_t previous_channel,
                                               uint8_t current_channel);
  void clear_detector_state_();
  void request_motion_state_callback_(MotionState previous_state, MotionState current_state);
  MotionState update_effective_motion_state_(MotionState detector_state);
  void reset_motion_state_filter_(MotionState state = MotionState::IDLE);
  
  bool enabled_{false};
  BaseDetector* detector_{nullptr};
  csi_packet_interceptor_t packet_interceptor_{nullptr};
  void *packet_interceptor_context_{nullptr};
  csi_processed_callback_t packet_callback_;
  motion_state_callback_t motion_state_callback_;
  live_telemetry_callback_t live_telemetry_callback_;
  channel_change_callback_t channel_change_callback_;
  uint32_t last_heartbeat_ms_{0U};
  int64_t last_detector_input_us_{0};
  bool has_detector_input_{false};
  // Evaluation advances on elapsed packet time, not on packet count, so a
  // window keeps its deploy-time meaning when the stream runs off-nominal.
  EvaluationCadence cadence_{};
  TemporalCsiSampler sampler_{};
  PendingCsiCandidate pending_candidate_{};
  bool pending_candidate_valid_{false};
  std::atomic<uint32_t> packets_processed_{0U};
  std::atomic<MotionState> heartbeat_motion_state_{MotionState::IDLE};
  std::atomic<uint64_t> accepted_packets_total_{0U};
  std::atomic<uint64_t> traffic_classified_packets_total_{0U};
  std::atomic<uint64_t> traffic_rejected_packets_total_{0U};
  int8_t last_rssi_dbm_{INT8_MIN};
  uint8_t last_channel_{0};
  uint8_t motion_on_hits_{RUNTIME_MOTION_ON_HITS_DEFAULT};
  uint8_t motion_off_hits_{RUNTIME_MOTION_OFF_HITS_DEFAULT};
  uint8_t pending_state_hits_{0};
  MotionState effective_motion_state_{MotionState::IDLE};
  MotionState pending_motion_state_{MotionState::IDLE};
  detail::PendingEventLock traffic_filter_lock_{};
  CsiFrameFilterConfig traffic_filter_{};
  bool traffic_filter_configured_{false};

  static constexpr size_t kPendingCsiFrameCapacity = 8U;
  PendingQueue<PendingCsiFrame, kPendingCsiFrameCapacity> pending_frames_;
  std::atomic<uint64_t> pending_frame_drops_{0U};
  // A task mutex keeps caller-owned raw context alive through the complete
  // bounded callback without running application code in a critical section.
  std::mutex raw_packet_lock_;
  std::atomic<raw_csi_packet_callback_t> raw_packet_callback_{nullptr};
  void *raw_packet_context_{nullptr};
  // Notifications are produced while the owning loop evaluates detector state
  // and are consumed after the pending CSI batch has been drained.
  PendingEvent<MotionState> motion_state_event_;
  PendingEvent<float, float> live_telemetry_event_;
  PendingDetectionTiming detection_timing_;
  CsiCaptureService capture_service_;

  static constexpr uint8_t NUM_SUBCARRIERS = HT20_SELECTED_BAND_SIZE;
};

}  // namespace espectre
