/*
 * ESPectre - Runtime Diagnostics
 *
 * Runtime diagnostics snapshot helpers.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "runtime_interface.h"
#include "runtime_snapshot.h"

namespace espectre {

/**
 * Rate and link diagnostics derived from cumulative runtime counters.
 *
 * Produced by the runtime-owned `RuntimeDiagnosticsSampler`. The rates are
 * what the runtime's monotonic totals moved by between two periodic sensing
 * updates, and every frontend reads the same latest sample.
 *
 * A zero rate means the counter did not move over the interval, and the first
 * sample after `RuntimeDiagnosticsSampler::reset()` reports zero rates because
 * it establishes the baseline. The link fields are carried through either way.
 */
struct RuntimeDiagnosticsSample {
  /** Traffic packets per second sent or observed by the active traffic source. */
  float traffic_tx_pps{0.0f};
  /** Raw CSI callbacks per second, before any capture-level validation. */
  float csi_callback_pps{0.0f};
  /** CSI packets per second classified as configured generator traffic. */
  float csi_classified_pps{0.0f};
  /** CSI packets per second rejected by the generator provenance filter. */
  float csi_provenance_rejected_pps{0.0f};
  /** CSI packets per second accepted by capture validation. */
  float csi_accepted_pps{0.0f};
  /** CSI packets per second admitted to the detector's temporal grid. */
  float csi_admitted_pps{0.0f};
  /** CSI packets per second rejected by capture-level validation. */
  float csi_filtered_pps{0.0f};
  /** Valid CSI callbacks per second dropped because the pending queue was full. */
  float csi_pending_frame_drop_pps{0.0f};
  /** Missing detector slots per second. */
  float csi_missing_slots_pps{0.0f};
  /** Same-slot excess drops per second. */
  float csi_excess_pps{0.0f};
  /** Stale temporal drops per second. */
  float csi_stale_pps{0.0f};
  /** Out-of-order temporal drops per second. */
  float csi_out_of_order_pps{0.0f};
  /** Valid-slot occupancy of the active temporal detector window. */
  float csi_occupancy_ratio{0.0f};
  /** RSSI of the current association. `INT8_MIN` when unavailable. */
  int8_t wifi_rssi_dbm{INT8_MIN};
  /** Primary channel of the current association. Zero when unavailable. */
  uint8_t wifi_channel{0U};
};

/**
 * Converts cumulative diagnostics into rates over the interval between reads.
 *
 * Call `reset()` when the owning runtime starts. Counter resets are treated as
 * a new epoch, so rearming a traffic source cannot underflow a rate.
 *
 * @code
 * // once, when the runtime starts sensing:
 * sampler.reset(runtime.get_diagnostics(), now_ms);
 * // on the runtime's existing sensing heartbeat:
 * latest = sampler.sample(runtime.get_diagnostics(), now_ms);
 * @endcode
 *
 * @par Threading
 * Not synchronized, and it holds the previous read. Sample it from the task
 * that owns the runtime.
 */
class RuntimeDiagnosticsSampler {
 public:
  /**
   * Establish the baseline the next `sample()` measures against.
   *
   * @param snapshot Current cumulative counters.
   * @param now_ms Monotonic frontend clock, in milliseconds.
   */
  void reset(const RuntimeDiagnosticsSnapshot &snapshot, uint32_t now_ms);
  /**
   * Derive rates since the previous read and adopt this one as the baseline.
   *
   * The caller owns the window. Shipped frontends invoke this from their
   * existing periodic sensing update, so diagnostics do not add a timer.
   *
   * @param snapshot Current cumulative counters.
   * @param now_ms Monotonic frontend clock, in milliseconds.
   * @return Rates over the elapsed interval. The link fields are always
   *         carried through; the rates are zero when there is no baseline yet
   *         or no time has elapsed.
   */
  RuntimeDiagnosticsSample sample(const RuntimeDiagnosticsSnapshot &snapshot, uint32_t now_ms);

 private:
  RuntimeDiagnosticsSnapshot previous_{};
  uint32_t previous_ms_{0U};
  bool baseline_ready_{false};
};

/** Append capture-quality counters to an already opened JSON object. */
void append_runtime_csi_quality_diagnostics_json(std::string *out,
                                                const RuntimeDiagnosticsSnapshot &diagnostics);

/**
 * Append shared platform and performance fields to an existing JSON object.
 *
 * The object must already contain at least one field. Metrics from an
 * incomplete aggregation window are emitted as `null`; unsupported detector
 * timing is identified separately by `detection_timing_supported`.
 */
void append_runtime_performance_diagnostics_json(std::string *out,
                                                 const RuntimeDiagnosticsSnapshot &diagnostics,
                                                 bool include_current_memory = true);

using runtime_diagnostic_visitor_t = std::function<void(const char *key, const char *value)>;

void visit_runtime_diagnostics(const RuntimeConfig &config,
                               const RuntimeSnapshot &snapshot,
                               runtime_diagnostic_visitor_t visitor);

}  // namespace espectre
