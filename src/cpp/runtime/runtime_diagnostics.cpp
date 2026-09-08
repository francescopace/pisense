/*
 * ESPectre - Runtime Diagnostics
 *
 * Runtime diagnostics snapshot helpers.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "runtime_diagnostics.h"

#include "counter_helpers.h"

#include <cstdio>
#include <string>

#include "runtime_config_utils.h"

namespace espectre {

namespace {

float packets_per_second(uint64_t delta, uint32_t elapsed_ms) {
  return elapsed_ms > 0U
             ? static_cast<float>(delta) * 1000.0f / static_cast<float>(elapsed_ms)
             : 0.0f;
}

void append_json_key(std::string *out, const char *key) {
  *out += ",\"";
  *out += key;
  *out += "\":";
}

void append_json_uint(std::string *out, const char *key, uint64_t value) {
  append_json_key(out, key);
  *out += std::to_string(value);
}

void append_json_float(std::string *out, const char *key, float value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.6g", static_cast<double>(value));
  append_json_key(out, key);
  *out += text;
}

void append_json_bool(std::string *out, const char *key, bool value) {
  append_json_key(out, key);
  *out += value ? "true" : "false";
}

void append_json_null(std::string *out, const char *key) {
  append_json_key(out, key);
  *out += "null";
}

}  // namespace

void append_runtime_csi_quality_diagnostics_json(std::string *out,
                                                const RuntimeDiagnosticsSnapshot &diagnostics) {
  if (out == nullptr) return;
  append_json_uint(out, "csi_rx_error_total", diagnostics.csi_rx_error_total);
  append_json_uint(out, "csi_rx_end_error_total", diagnostics.csi_rx_end_error_total);
  append_json_uint(out, "csi_invalid_estimate_total", diagnostics.csi_invalid_estimate_total);
  append_json_uint(out, "csi_invalid_first_word_total", diagnostics.csi_invalid_first_word_total);
  append_json_uint(out, "csi_sanitized_first_word_total", diagnostics.csi_sanitized_first_word_total);
  append_json_uint(out, "csi_estimate_length_mismatch_total", diagnostics.csi_estimate_length_mismatch_total);
}

void append_runtime_performance_diagnostics_json(std::string *out,
                                                 const RuntimeDiagnosticsSnapshot &diagnostics,
                                                 bool include_current_memory) {
  if (out == nullptr) {
    return;
  }
  if (include_current_memory) {
    append_json_float(out, "free_memory_kb", static_cast<float>(diagnostics.free_memory_bytes) / 1024.0f);
  }
  append_json_float(out,
                    "minimum_free_memory_kb",
                    static_cast<float>(diagnostics.minimum_free_memory_bytes) / 1024.0f);
  append_json_float(out,
                    "largest_free_memory_kb",
                    static_cast<float>(diagnostics.largest_free_memory_block_bytes) / 1024.0f);
  append_json_uint(out, "cpu_frequency_mhz", diagnostics.cpu_frequency_mhz);
  append_json_bool(out, "performance_window_ready", diagnostics.performance_window_ready);
  if (diagnostics.performance_window_ready) {
    append_json_float(out,
                      "performance_window_ms",
                      static_cast<float>(diagnostics.performance_window_duration_us) / 1000.0f);
    append_json_float(out, "runtime_load_percent", diagnostics.runtime_load_percent);
    append_json_uint(out, "loop_samples", diagnostics.loop_samples);
    append_json_uint(out, "loop_avg_us", diagnostics.loop_average_us);
    append_json_uint(out, "loop_max_us", diagnostics.loop_maximum_us);
  } else {
    append_json_null(out, "performance_window_ms");
    append_json_null(out, "runtime_load_percent");
    append_json_null(out, "loop_samples");
    append_json_null(out, "loop_avg_us");
    append_json_null(out, "loop_max_us");
  }
  append_json_bool(out, "detection_timing_supported", diagnostics.detection_timing_supported);
  if (diagnostics.performance_window_ready && diagnostics.detection_timing_supported) {
    append_json_uint(out, "detection_samples", diagnostics.detection_samples);
    append_json_uint(out, "detection_sum_us", diagnostics.detection_sum_us);
    append_json_uint(out, "detection_avg_us", diagnostics.detection_average_us);
    append_json_uint(out, "detection_min_us", diagnostics.detection_minimum_us);
    append_json_uint(out, "detection_max_us", diagnostics.detection_maximum_us);
  } else {
    append_json_null(out, "detection_samples");
    append_json_null(out, "detection_sum_us");
    append_json_null(out, "detection_avg_us");
    append_json_null(out, "detection_min_us");
    append_json_null(out, "detection_max_us");
  }
}

void RuntimeDiagnosticsSampler::reset(const RuntimeDiagnosticsSnapshot &snapshot, uint32_t now_ms) {
  previous_ = snapshot;
  previous_ms_ = now_ms;
  baseline_ready_ = true;
}

RuntimeDiagnosticsSample RuntimeDiagnosticsSampler::sample(const RuntimeDiagnosticsSnapshot &snapshot,
                                                            uint32_t now_ms) {
  RuntimeDiagnosticsSample result;
  result.wifi_rssi_dbm = snapshot.wifi_rssi_dbm;
  result.wifi_channel = snapshot.wifi_channel;
  if (!baseline_ready_) {
    reset(snapshot, now_ms);
    return result;
  }

  const uint32_t elapsed_ms = now_ms - previous_ms_;
  if (elapsed_ms == 0U) {
    return result;
  }
  result.traffic_tx_pps = packets_per_second(
      counter_delta(snapshot.traffic_packets_total, previous_.traffic_packets_total), elapsed_ms);
  result.csi_callback_pps = packets_per_second(
      counter_delta(snapshot.csi_callbacks_total, previous_.csi_callbacks_total), elapsed_ms);
  result.csi_classified_pps = packets_per_second(
      counter_delta(snapshot.csi_classified_total, previous_.csi_classified_total), elapsed_ms);
  result.csi_provenance_rejected_pps = packets_per_second(
      counter_delta(snapshot.csi_provenance_rejected_total,
                    previous_.csi_provenance_rejected_total),
      elapsed_ms);
  result.csi_accepted_pps = packets_per_second(
      counter_delta(snapshot.csi_accepted_total, previous_.csi_accepted_total), elapsed_ms);
  result.csi_admitted_pps = packets_per_second(
      counter_delta(snapshot.csi_admitted_total, previous_.csi_admitted_total), elapsed_ms);
  result.csi_filtered_pps = packets_per_second(
      counter_delta(snapshot.csi_filtered_total, previous_.csi_filtered_total), elapsed_ms);
  result.csi_pending_frame_drop_pps = packets_per_second(
      counter_delta(snapshot.csi_pending_frame_drops_total,
                    previous_.csi_pending_frame_drops_total),
      elapsed_ms);
  result.csi_missing_slots_pps = packets_per_second(
      counter_delta(snapshot.csi_missing_slots_total, previous_.csi_missing_slots_total), elapsed_ms);
  result.csi_excess_pps = packets_per_second(
      counter_delta(snapshot.csi_excess_total, previous_.csi_excess_total), elapsed_ms);
  result.csi_stale_pps = packets_per_second(
      counter_delta(snapshot.csi_stale_total, previous_.csi_stale_total), elapsed_ms);
  result.csi_out_of_order_pps = packets_per_second(
      counter_delta(snapshot.csi_out_of_order_total, previous_.csi_out_of_order_total), elapsed_ms);
  result.csi_occupancy_ratio = snapshot.csi_window_slots > 0U
      ? static_cast<float>(snapshot.csi_occupancy_slots) /
            static_cast<float>(snapshot.csi_window_slots)
      : 0.0f;
  reset(snapshot, now_ms);
  return result;
}

void visit_runtime_diagnostics(const RuntimeConfig &config,
                               const RuntimeSnapshot &snapshot,
                               runtime_diagnostic_visitor_t visitor) {
  if (!visitor) {
    return;
  }

  char value[64];

  std::snprintf(value, sizeof(value), "%.6f", snapshot.threshold);
  visitor("threshold", value);
  std::snprintf(value, sizeof(value), "%u", static_cast<unsigned>(config.segmentation_window_size_ms));
  visitor("window_ms", value);
  visitor("detector", snapshot.detector_name);
  visitor("lowpass", config.lowpass_enabled ? "on" : "off");
  std::snprintf(value, sizeof(value), "%.1f", config.lowpass_cutoff);
  visitor("lowpass_cutoff", value);
  visitor("hampel", config.hampel_enabled ? "on" : "off");
  if (config.hampel_enabled) {
    std::snprintf(value, sizeof(value), "%u", static_cast<unsigned>(config.hampel_window));
    visitor("hampel_window", value);
    std::snprintf(value, sizeof(value), "%.1f", config.hampel_threshold);
    visitor("hampel_threshold", value);
  }
  visitor("traffic_mode", traffic_mode_name(config.traffic_generator_mode));
  visitor("csi_traffic_mode", csi_traffic_mode_name(config.csi_traffic_mode));
  std::snprintf(value, sizeof(value), "%u", static_cast<unsigned>(config.csi_target_pps));
  visitor("csi_target_pps", value);
  std::snprintf(value, sizeof(value), "%u", static_cast<unsigned>(config.evaluation_interval_ms));
  visitor("evaluation_interval_ms", value);
  std::snprintf(value,
                sizeof(value),
                "%u/%u",
                static_cast<unsigned>(config.motion_on_hits),
                static_cast<unsigned>(config.motion_off_hits));
  visitor("motion_hits", value);
}

}  // namespace espectre
