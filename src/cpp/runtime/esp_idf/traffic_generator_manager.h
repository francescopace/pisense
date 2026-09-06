/*
 * ESPectre - Traffic Generator Manager
 *
 * Generates paced traffic to the gateway or associated AP at the configured
 * CSI target. Scheduling, local send backoff, and stall logging are shared by
 * all backends. Occupancy never changes the send rate.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <sys/types.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "csi_traffic_service.h"

namespace espectre {

struct SendErrorState {
  uint32_t error_count{0};
  int64_t last_log_time{0};
  static constexpr int64_t LOG_INTERVAL_US = 1000000;
};

inline bool handle_send_error(SendErrorState &state, ssize_t sent, int err_no, int64_t current_time) {
  (void) sent;
  state.error_count++;
  if (current_time - state.last_log_time > SendErrorState::LOG_INTERVAL_US) {
    state.error_count = 0;
    state.last_log_time = current_time;
  }
  return err_no == 12;
}

inline int64_t next_traffic_send_deadline_us(int64_t previous_deadline_us,
                                             int64_t send_started_us,
                                             int64_t interval_us) {
  if (interval_us <= 0) {
    return send_started_us;
  }
  if (previous_deadline_us <= 0) {
    return send_started_us + interval_us;
  }

  const int64_t phase_deadline_us = previous_deadline_us + interval_us;
  const int64_t remaining_us = phase_deadline_us - send_started_us;
  if (remaining_us < interval_us / 2) {
    return send_started_us + interval_us;
  }
  return phase_deadline_us;
}

constexpr size_t TRAFFIC_DNS_QUERY_PAYLOAD_SIZE = 17U;
constexpr size_t TRAFFIC_DNS_TCP_FRAME_SIZE = TRAFFIC_DNS_QUERY_PAYLOAD_SIZE + 2U;
constexpr size_t TRAFFIC_NULL_DATA_FRAME_SIZE = 24U;

size_t build_null_data_frame(const uint8_t *bssid, const uint8_t *station_mac,
                             uint8_t *buffer, size_t buffer_len);

size_t build_dns_query_payload(uint16_t transaction_id,
                               uint8_t *buffer,
                               size_t buffer_len);

size_t build_dns_tcp_query_frame(uint16_t transaction_id,
                                 uint8_t *buffer,
                                 size_t buffer_len);

class TrafficGeneratorManager : public ICsiTrafficGenerator {
 public:
  void init(uint32_t target_pps,
            RuntimeTrafficMode mode = RuntimeTrafficMode::PING) override;

  bool start(uint32_t gateway_addr) override;
  void loop() override;
  void stop() override;

  bool is_running() const override { return running_.load(std::memory_order_relaxed); }
  void pause();
  void resume();
  bool is_paused() const { return paused_.load(std::memory_order_relaxed); }

  uint32_t target_rate_pps() const { return target_pps_; }
  uint32_t current_rate_pps() const { return current_rate_pps_.load(std::memory_order_relaxed); }
  uint32_t send_success_count() const override {
    return send_success_count_.load(std::memory_order_relaxed);
  }
  uint32_t send_error_count() const { return send_error_count_.load(std::memory_order_relaxed); }
  uint16_t icmp_identifier() const override { return icmp_identifier_; }

 private:
  static void traffic_task_(void *arg);
  void reset_runtime_state_();

  TaskHandle_t task_handle_{nullptr};
  int sock_{-1};
  uint32_t gateway_addr_{0U};
  uint32_t target_pps_{0U};
  RuntimeTrafficMode mode_{RuntimeTrafficMode::PING};
  uint16_t icmp_identifier_{0U};
  uint8_t null_data_frame_[TRAFFIC_NULL_DATA_FRAME_SIZE]{};
  std::atomic<uint32_t> current_rate_pps_{0U};
  std::atomic<bool> running_{false};
  std::atomic<bool> paused_{false};
  std::atomic<bool> task_exited_{true};
  std::atomic<uint32_t> send_success_count_{0U};
  std::atomic<uint32_t> send_error_count_{0U};
  uint32_t previous_send_success_count_{0U};
  int64_t last_send_progress_us_{0};
  int64_t last_health_check_us_{0};

  static constexpr int64_t HEALTH_CHECK_INTERVAL_US = 1000000;
  static constexpr int64_t SEND_STALL_TIMEOUT_US = 5000000;
  static constexpr uint32_t CONSECUTIVE_ERROR_REOPEN_THRESHOLD = 32U;
};

}  // namespace espectre
