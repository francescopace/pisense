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
#include "csi_pipeline.h"
#include <algorithm>
#include <sdkconfig.h>
#include "espectre_log.h"
#include "esp_timer.h"
#include "csi_frame_identity.h"
#include "csi_phy_filter.h"

namespace espectre {

static const char *TAG = "CsiPipeline";

void CsiPipeline::init(BaseDetector* detector, IWiFiCSI* wifi_csi) {
  detector_ = detector;
  last_heartbeat_ms_ = 0U;
  packets_processed_.store(0U, std::memory_order_relaxed);
  capture_service_.init(wifi_csi);
  capture_service_.set_packet_callback(&CsiPipeline::capture_packet_callback_, this);
  capture_service_.set_channel_change_callback(&CsiPipeline::capture_channel_change_callback_, this);
  accepted_packets_total_.store(0U, std::memory_order_relaxed);
  traffic_classified_packets_total_.store(0U, std::memory_order_relaxed);
  traffic_rejected_packets_total_.store(0U, std::memory_order_relaxed);
  pending_frames_.clear();
  pending_frame_drops_.store(0U, std::memory_order_relaxed);
  sampler_.reset();
  pending_candidate_valid_ = false;
  if (detector_ != nullptr) {
    detector_->set_minimum_valid_samples(
        static_cast<uint16_t>(sampler_.minimum_valid_slots()));
  }
  reset_motion_state_filter_();
  
  ESPECTRE_LOGD(TAG, "CSI Pipeline initialized with %s detector",
           detector_ ? detector_->get_name() : "NULL");
}

bool CsiPipeline::set_threshold(float threshold) {
  if (detector_ == nullptr) {
    return false;
  }
  if (!detector_->set_threshold(threshold)) {
    ESPECTRE_LOGW(TAG, "Rejected invalid threshold: %.3f", threshold);
    return false;
  }
  ESPECTRE_LOGD(TAG, "Threshold updated: %.2f", threshold);
  return true;
}

void CsiPipeline::set_motion_hit_thresholds(uint8_t motion_on_hits, uint8_t motion_off_hits, bool reset_filter) {
  set_motion_on_hits(motion_on_hits);
  set_motion_off_hits(motion_off_hits);
  if (reset_filter) {
    reset_motion_state_filter_(effective_motion_state_);
  }
}

void CsiPipeline::set_detector(BaseDetector *detector) {
  detector_ = detector;
  if (detector_ != nullptr) {
    detector_->set_minimum_valid_samples(
        static_cast<uint16_t>(sampler_.minimum_valid_slots()));
  }
  clear_detector_state_();
  ESPECTRE_LOGD(TAG, "Detector updated to %s", detector_ != nullptr ? detector_->get_name() : "NULL");
}

void CsiPipeline::clear_detector_buffer() {
  // Frames accepted before this reset belong to the previous calibration or
  // channel epoch and must not repopulate the freshly cleared detector.
  pending_frames_.clear();
  live_telemetry_event_.clear();
  clear_detector_state_();
  MotionState motion_state = MotionState::IDLE;
  if (motion_state_event_.take(motion_state) && motion_state_callback_) {
    motion_state_callback_(motion_state);
  }
}

bool CsiPipeline::start_raw_capture(raw_csi_packet_callback_t callback, void *context) {
  if (callback == nullptr || raw_capture_active()) {
    return false;
  }
  pending_frames_.clear();
  pending_candidate_valid_ = false;
  live_telemetry_event_.clear();
  motion_state_event_.clear();
  detection_timing_.clear();
  clear_detector_state_();
  {
    std::lock_guard<std::mutex> lock(raw_packet_lock_);
    raw_packet_context_ = context;
    raw_packet_callback_.store(callback, std::memory_order_release);
  }
  return true;
}

void CsiPipeline::stop_raw_capture() {
  {
    std::lock_guard<std::mutex> lock(raw_packet_lock_);
    raw_packet_callback_.store(nullptr, std::memory_order_release);
    raw_packet_context_ = nullptr;
  }
  pending_frames_.clear();
  pending_candidate_valid_ = false;
  live_telemetry_event_.clear();
  motion_state_event_.clear();
  detection_timing_.clear();
  clear_detector_state_();
}

void CsiPipeline::clear_detector_state_() {
  if (detector_) {
    MotionState previous_state = effective_motion_state_;
    // Cold reset: clear turbulence history and state.
    // Required after channel switch and post-calibration to avoid stale samples.
    detector_->clear_buffer();
    cadence_.reset_window();
    // Detector changes and calibration boundaries invalidate feature data,
    // but they are not discontinuities in the Wi-Fi RX clock. Preserve the
    // temporal grid phase so the same packet stream keeps the same slot map.
    sampler_.clear_window_preserving_phase();
    pending_candidate_valid_ = false;
    reset_motion_state_filter_();
    request_motion_state_callback_(previous_state, effective_motion_state_);
  }
}

void CsiPipeline::request_motion_state_callback_(MotionState previous_state, MotionState current_state) {
  if (previous_state == current_state) {
    return;
  }
  motion_state_event_.post(current_state);
}

void CsiPipeline::loop() {
  drain_pending_frames_();
  capture_service_.loop();
  MotionState motion_state = MotionState::IDLE;
  if (motion_state_event_.take(motion_state) && motion_state_callback_) {
    motion_state_callback_(motion_state);
  }
  float movement = 0.0f;
  float threshold = 0.0f;
  if (live_telemetry_event_.take(movement, threshold) && live_telemetry_callback_) {
    live_telemetry_callback_(movement, threshold);
  }
}

void CsiPipeline::heartbeat_if_due(uint32_t now_ms) {
  if (!enabled_ || !packet_callback_) {
    return;
  }
  if (last_heartbeat_ms_ == 0U) {
    last_heartbeat_ms_ = now_ms;
    return;
  }
  if (now_ms - last_heartbeat_ms_ < RUNTIME_HEARTBEAT_INTERVAL_MS) {
    return;
  }
  last_heartbeat_ms_ = now_ms;
  const uint32_t packets_received =
      packets_processed_.exchange(0U, std::memory_order_relaxed);
  packet_callback_(heartbeat_motion_state_.load(std::memory_order_relaxed), packets_received);
}

void CsiPipeline::set_traffic_filter(const CsiFrameFilterConfig &config) {
  std::lock_guard<detail::PendingEventLock> lock(traffic_filter_lock_);
  traffic_filter_ = config;
  traffic_filter_configured_ = true;
}

MotionState CsiPipeline::update_effective_motion_state_(MotionState detector_state) {
  if (detector_state == effective_motion_state_) {
    pending_motion_state_ = effective_motion_state_;
    pending_state_hits_ = 0;
    return effective_motion_state_;
  }

  if (detector_state != pending_motion_state_) {
    pending_motion_state_ = detector_state;
    pending_state_hits_ = 1;
  } else if (pending_state_hits_ < UINT8_MAX) {
    pending_state_hits_++;
  }

  uint8_t required_hits = (pending_motion_state_ == MotionState::MOTION) ? motion_on_hits_ : motion_off_hits_;
  if (pending_state_hits_ >= required_hits) {
    effective_motion_state_ = pending_motion_state_;
    pending_state_hits_ = 0;
  }

  return effective_motion_state_;
}

void CsiPipeline::reset_motion_state_filter_(MotionState state) {
  effective_motion_state_ = state;
  heartbeat_motion_state_.store(state, std::memory_order_relaxed);
  pending_motion_state_ = state;
  pending_state_hits_ = 0;
}

void CsiPipeline::process_packet(wifi_csi_info_t* data) {
  if (!data || !detector_) {
    return;
  }
  capture_service_.process_packet(data);
  // Direct callers are synchronous test/host paths. The hardware callback
  // enters through CsiCaptureService and is drained by the runtime loop.
  loop();
}

bool CsiPipeline::flush_pending_candidate() {
  if (!pending_candidate_valid_ || !sampler_.flush()) {
    return false;
  }
  process_admitted_candidate_();
  loop();
  return true;
}

void CsiPipeline::process_pending_frame_(const PendingCsiFrame &frame) {
  if (detector_ == nullptr || frame.len == 0U || frame.len > frame.csi.size()) {
    return;
  }
  if (frame.reset_detector_before_consume) {
    clear_detector_state_();
  }

  accepted_packets_total_.fetch_add(1U, std::memory_order_relaxed);

  const int8_t rssi_dbm = frame.rx_ctrl.rssi;
  last_rssi_dbm_ = rssi_dbm;
  last_channel_ = frame.rx_ctrl.channel;

  // The Wi-Fi RX timestamp belongs to the MAC clock, not the esp_timer clock.
  // Measure only callback-to-loop queue age with esp_timer, then translate that
  // duration into the RX timestamp domain for the sampler's unsigned age check.
  const uint32_t processing_time_us =
      static_cast<uint32_t>(static_cast<uint64_t>(esp_timer_get_time()));
  const uint32_t queue_age_us = processing_time_us - frame.callback_time_us;
  const uint32_t sampler_now_us = frame.rx_ctrl.timestamp + queue_age_us;
  const bool emitted = sampler_.admit(
      frame.rx_ctrl.timestamp, true, sampler_now_us, true);
  if (emitted && pending_candidate_valid_) {
    process_admitted_candidate_();
  }
  if (sampler_.gap_reset_required()) {
    apply_gap_history_reset_();
  }
  if (sampler_.selected_current()) {
    wifi_csi_info_t info{};
    info.rx_ctrl = frame.rx_ctrl;
    const NormalizedCSIPayload normalized{frame.csi.data(), frame.len};
    store_candidate_(&info, normalized);
  }
}

void CsiPipeline::drain_pending_frames_() {
  PendingCsiFrame frame;
  // Bound one runtime-loop iteration to the backlog observed on entry. The
  // Wi-Fi callback may refill the queue while detector work is in progress;
  // consuming that refill here could keep ESPHome's watched loop task inside
  // the pipeline indefinitely.
  const size_t pending_at_loop_start = pending_frames_.size();
  for (size_t processed = 0U; processed < pending_at_loop_start; ++processed) {
    if (!pending_frames_.take(frame)) {
      break;
    }
    process_pending_frame_(frame);
  }
}

void CsiPipeline::apply_gap_history_reset_() {
  if (detector_ == nullptr) {
    return;
  }
  const MotionState previous_state = effective_motion_state_;
  detector_->clear_buffer();
  cadence_.reset_window();
  reset_motion_state_filter_();
  request_motion_state_callback_(previous_state, effective_motion_state_);
}

void CsiPipeline::store_candidate_(const wifi_csi_info_t *data,
                                   const NormalizedCSIPayload &normalized) {
  if (data == nullptr || normalized.data == nullptr ||
      normalized.len == 0U || normalized.len > pending_candidate_.csi.size()) {
    pending_candidate_valid_ = false;
    return;
  }
  std::copy_n(normalized.data, normalized.len, pending_candidate_.csi.begin());
  pending_candidate_.len = normalized.len;
  pending_candidate_.timestamp_us = data->rx_ctrl.timestamp;
  pending_candidate_.rssi_dbm = data->rx_ctrl.rssi;
  pending_candidate_valid_ = true;
}

void CsiPipeline::process_admitted_candidate_() {
  if (!pending_candidate_valid_ || detector_ == nullptr) {
    return;
  }
  pending_candidate_valid_ = false;
  const int8_t *csi_data = pending_candidate_.csi.data();
  const size_t csi_len = pending_candidate_.len;
  const int8_t rssi_dbm = pending_candidate_.rssi_dbm;
  const uint32_t timestamp_us = pending_candidate_.timestamp_us;
  detector_->set_packet_timestamp_us(timestamp_us);
  if (sampler_.reset_required()) {
    const MotionState previous_state = effective_motion_state_;
    detector_->clear_buffer();
    cadence_.reset_window();
    reset_motion_state_filter_();
    request_motion_state_callback_(previous_state, effective_motion_state_);
  }
  if (sampler_.missing_slots_before() > 0U) {
    detector_->advance_missing_slots(
        static_cast<uint32_t>(std::min<uint64_t>(
            sampler_.missing_slots_before(), sampler_.window_slots())));
  }

  // Evaluation cadence consumes the same admitted stream as calibration and
  // feature processing. Elapsed packet time still drives the deadline.
  const bool cadence_due = cadence_.observe(timestamp_us);

  if (packet_interceptor_ &&
      packet_interceptor_(packet_interceptor_context_, csi_data, csi_len, rssi_dbm,
                          cadence_due, cadence_.packets_since_evaluation(),
                          sampler_.reset_required())) {
    if (cadence_due) {
      cadence_.after_evaluation();
    }
    return;
  }

  detector_->process_packet(csi_data, csi_len, DEFAULT_SUBCARRIERS, NUM_SUBCARRIERS, rssi_dbm);

  packets_processed_.fetch_add(1U, std::memory_order_relaxed);

  if (cadence_due) {
    const int64_t start_us = esp_timer_get_time();
    // Update detector state on the internal cadence.
    MotionState previous_state = effective_motion_state_;
    detector_->update_state();
    MotionState current_state = update_effective_motion_state_(detector_->get_state());
    heartbeat_motion_state_.store(current_state, std::memory_order_relaxed);
    request_motion_state_callback_(previous_state, current_state);
    cadence_.after_evaluation();

    const int64_t elapsed_us = esp_timer_get_time() - start_us;
    detection_timing_.record(static_cast<uint32_t>(elapsed_us));

    // Emit live telemetry on each detector evaluation tick.
    if (live_telemetry_callback_) {
      live_telemetry_event_.post(detector_->get_motion_metric(), detector_->get_threshold());
    }
  }
}

bool CsiPipeline::take_detection_timing(DetectionTimingStats *stats) {
  return detection_timing_.take(stats);
}

void CsiPipeline::capture_packet_callback_(void *context,
                                           const wifi_csi_info_t *data,
                                           const NormalizedCSIPayload &normalized) {
  auto *pipeline = static_cast<CsiPipeline *>(context);
  if (pipeline == nullptr || data == nullptr || !normalized.valid() ||
      normalized.len == 0U || normalized.len > HT20_CSI_LEN) {
    return;
  }

  CsiFrameFilterConfig traffic_filter;
  bool traffic_filter_configured = false;
  {
    std::lock_guard<detail::PendingEventLock> lock(pipeline->traffic_filter_lock_);
    traffic_filter = pipeline->traffic_filter_;
    traffic_filter_configured = pipeline->traffic_filter_configured_;
  }
  if (traffic_filter_configured) {
    if (!csi_frame_matches_traffic(data, traffic_filter, pipeline->capture_profile())) {
      pipeline->traffic_rejected_packets_total_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    pipeline->traffic_classified_packets_total_.fetch_add(1U, std::memory_order_relaxed);
  }

  if (pipeline->raw_packet_callback_.load(std::memory_order_acquire) != nullptr) {
    // Never wait in Wi-Fi capture context. Contention means the owner is
    // changing raw sessions; this frame belongs to that transition.
    std::unique_lock<std::mutex> lock(pipeline->raw_packet_lock_, std::try_to_lock);
    if (!lock.owns_lock()) return;
    raw_csi_packet_callback_t raw_callback =
        pipeline->raw_packet_callback_.load(std::memory_order_acquire);
    if (raw_callback == nullptr) return;
    RawCsiPacketView packet;
    packet.csi = normalized.data;
    packet.csi_len = static_cast<uint16_t>(normalized.len);
    packet.captured_at_us = static_cast<uint64_t>(esp_timer_get_time());
    packet.wifi_rx_ts_us = data->rx_ctrl.timestamp;
    packet.record_flags = RAW_CSI_FLAG_FRESH;
    if (data->first_word_invalid) {
      packet.record_flags |= RAW_CSI_FLAG_FIRST_WORD_INVALID;
    }
    if (data->rx_ctrl.timestamp != 0U) {
      packet.record_flags |= RAW_CSI_FLAG_WIFI_RX_TS_VALID;
    }
    packet.channel = data->rx_ctrl.channel;
    packet.rssi_dbm = data->rx_ctrl.rssi;
    packet.noise_floor_dbm = static_cast<int8_t>(data->rx_ctrl.noise_floor);
    const CsiCaptureProfile profile = pipeline->capture_profile();
    packet.phy_mode = profile == CsiCaptureProfile::VHT20
                          ? RawCsiPhyMode::VHT
                          : (csi_info_is_legacy_lltf(data, profile)
                                 ? RawCsiPhyMode::LEGACY
                                 : RawCsiPhyMode::HT);
    packet.ltf_type = profile == CsiCaptureProfile::VHT20
                          ? RawCsiLtfType::VHT_LTF
                          : (csi_capture_profile_uses_lltf(profile)
                                 ? RawCsiLtfType::LLTF
                                 : RawCsiLtfType::HT_LTF);
    packet.channel_width = RawCsiChannelWidth::MHZ_20;
    (void) raw_callback(pipeline->raw_packet_context_, packet);
    return;
  }

  PendingCsiFrame frame;
  frame.rx_ctrl = data->rx_ctrl;
  frame.callback_time_us =
      static_cast<uint32_t>(static_cast<uint64_t>(esp_timer_get_time()));
  frame.len = static_cast<uint16_t>(normalized.len);
  frame.reset_detector_before_consume = normalized.reset_detector_before_consume;
  std::copy_n(normalized.data, normalized.len, frame.csi.begin());
  if (csi_capture_profile_uses_lltf(pipeline->capture_profile())) {
    (void) impute_ht20_lltf_detector_bins(frame.csi.data(), frame.len);
  }
  if (!pipeline->pending_frames_.post(frame)) {
    pipeline->pending_frame_drops_.fetch_add(1U, std::memory_order_relaxed);
  }
}

void CsiPipeline::capture_channel_change_callback_(void *context,
                                                   uint8_t previous_channel,
                                                   uint8_t current_channel) {
  auto *pipeline = static_cast<CsiPipeline *>(context);
  if (pipeline != nullptr && pipeline->channel_change_callback_) {
    pipeline->channel_change_callback_(previous_channel, current_channel);
  }
}

esp_err_t CsiPipeline::enable(csi_processed_callback_t packet_callback,
                              CsiCaptureProfile profile) {
  if (enabled_) {
    return ESP_OK;
  }
  
  packet_callback_ = packet_callback;
  capture_service_.set_packet_callback(&CsiPipeline::capture_packet_callback_, this);

  esp_err_t err = capture_service_.enable(profile);
  if (err == ESP_OK) {
    enabled_ = true;
    last_heartbeat_ms_ = 0U;
    packets_processed_.store(0U, std::memory_order_relaxed);
  }
  return err;
}

esp_err_t CsiPipeline::reconfigure_capture(CsiCaptureProfile profile) {
  if (!enabled_) return ESP_ERR_INVALID_STATE;
  if (capture_profile() == profile) return ESP_OK;
  const CsiCaptureProfile previous_profile = capture_profile();
  const auto callback = packet_callback_;
  const esp_err_t disable_err = disable();
  if (disable_err != ESP_OK) return disable_err;
  const esp_err_t err = enable(callback, profile);
  if (err != ESP_OK) {
    const esp_err_t restore_err = enable(callback, previous_profile);
    if (restore_err != ESP_OK) {
      ESPECTRE_LOGE(TAG, "Failed to restore CSI profile: %s", esp_err_to_name(restore_err));
    }
  }
  return err;
}

esp_err_t CsiPipeline::disable() {
  if (!enabled_) {
    return ESP_OK;
  }
  
  esp_err_t err = capture_service_.disable();
  if (err != ESP_OK && capture_service_.is_enabled()) {
    return err;
  }
  
  enabled_ = false;
  stop_raw_capture();
  packet_callback_ = nullptr;
  capture_service_.set_packet_callback(nullptr, nullptr);
  motion_state_event_.clear();
  packets_processed_.store(0U, std::memory_order_relaxed);
  last_heartbeat_ms_ = 0U;
  last_rssi_dbm_ = INT8_MIN;
  last_channel_ = 0U;
  cadence_.reset();
  sampler_.reset();
  pending_candidate_valid_ = false;
  reset_motion_state_filter_();
  return err;
}

}  // namespace espectre
