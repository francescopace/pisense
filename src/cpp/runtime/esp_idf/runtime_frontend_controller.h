/*
 * ESPectre - Runtime Frontend Controller
 *
 * Owns runtime lifecycle and exposes a frontend-friendly control surface.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <memory>

#include "runtime_capabilities.h"
#include "runtime_events.h"
#include "runtime_interface.h"
#include "runtime_snapshot.h"

namespace espectre {

/**
 * The recommended entry point for firmware embedding ESPectre.
 *
 * It owns the sensing runtime, caches the latest snapshot and discovered
 * capabilities, and validates control calls before they reach the backend.
 *
 * @code
 * class ProductFrontend : public espectre::IRuntimeListener {
 *  public:
 *   bool setup() {
 *     espectre::RuntimeConfig config;
 *     config.detection_algorithm = espectre::DetectionAlgorithm::LIGHTWEIGHT;
 *     runtime_.set_config(config);
 *     return runtime_.setup(this);
 *   }
 *
 *   void loop() { runtime_.loop(); }
 *
 *   void on_motion_state_changed(const espectre::RuntimeSnapshot &snapshot) override {
 *     if (snapshot.ready_to_publish) publish(snapshot.motion_state);
 *   }
 *
 *  private:
 *   espectre::RuntimeFrontendController runtime_;
 * };
 * @endcode
 *
 * @par Lifecycle
 * `set_config()` -> `setup(listener)` -> `loop()` repeatedly -> `shutdown()`.
 * The controller is reusable after `shutdown()`: configuration survives, and
 * `set_config()` becomes effective again.
 *
 * @par Threading
 * Carries no internal locking. Run `setup()`, `loop()`, and `shutdown()` on
 * one task. See `espectre_sdk.h` for the full contract, including where
 * listener callbacks land and how to handle controls driven from a transport
 * callback.
 *
 * @par Control calls before setup
 * The setters work before `setup()` and simply update the pending
 * configuration, so a frontend can accept provisioning commands during boot
 * without special-casing the ordering.
 */
class RuntimeFrontendController : private IRuntimeListener {
 public:
  /** Shut the runtime down on scope exit. Explicit `shutdown()` remains recommended. */
  ~RuntimeFrontendController() override;
  /**
   * Stage the configuration used by the next `setup()`.
   *
   * Ignored once setup has started, so reconfiguring a running runtime means
   * `shutdown()` first, or the `set_*_runtime()` methods for the fields that
   * support live changes.
   */
  void set_config(const RuntimeConfig &config);
  /**
   * Mutable access to the staged configuration.
   *
   * Provided so a frontend can adjust individual fields before `setup()`
   * without rebuilding the whole struct. After a successful setup it reflects
   * the backend's effective configuration, including persisted overrides.
   * Writing to it after setup stages the next setup only; live controls
   * continue to validate against the active configuration.
   */
  RuntimeConfig &config() { return config_; }
  /** Read-only view of the staged or last effective configuration. */
  const RuntimeConfig &config() const { return config_; }
  /**
   * Latest known snapshot, without querying the backend.
   *
   * Refreshed automatically at `setup()`, by control calls, and before every
   * listener callback is forwarded to your frontend.
   * Use the cached snapshot for on-demand reads such as answering a status
   * query; use the listener callbacks to react to change.
   */
  const RuntimeSnapshot &snapshot() const { return snapshot_; }
  /**
   * Read backend counters without touching the cached sensing snapshot.
   *
   * Unlike `snapshot()`, this queries the backend on every call. Invoke it from
   * an existing periodic sensing callback, not from the hot loop. Returns a
   * zeroed snapshot before `setup()`.
   */
  RuntimeDiagnosticsSnapshot diagnostics() const;
  /** Latest runtime-owned rate sample, or `nullptr` before setup. */
  const RuntimeDiagnosticsSample *diagnostics_sample() const;
  /**
   * What the active backend supports. Meaningful only after `setup()`.
   *
   * Gate your product surface on it rather than hardcoding: the controller
   * already refuses capability-gated calls, and this is how you avoid exposing
   * a control the runtime will reject.
   */
  const RuntimeCapabilities &capabilities() const { return capabilities_; }
  /** True between a successful `setup()` and the next `shutdown()`. */
  bool is_setup_complete() const { return setup_complete_; }

  /**
   * Create the backend, apply the configuration, and start sensing.
   *
   * Calling it twice is a no-op that returns true.
   *
   * @param listener Event sink, or `nullptr` for none. Not owned; it must
   *        outlive the controller.
   * @return false when the backend cannot start or its bounded working storage
   *         cannot be allocated. On failure the backend is dropped and the
   *         controller stays un-setup, so it is safe to fix the config and
   *         retry.
   */
  bool setup(IRuntimeListener *listener);
  /**
   * Advance runtime work and deliver pending listener callbacks.
   *
   * Call it continuously from your loop task. Safe, and a no-op, before setup.
   */
  void loop();
  /** Stop sensing and release the backend. Safe before setup and to repeat. */
  void shutdown();

  /**
   * Gate runtime-owned services without tearing the runtime down.
   *
   * Sticky: the value is remembered and reapplied to the backend created by a
   * later `setup()`. Matter uses it to stay silent until commissioning.
   * Frontends use it to pause CSI without dropping Wi-Fi. During raw
   * collection the requested value is staged and applied when collection
   * stops, because changing sensing services cannot interrupt the capture
   * callback in place.
   */
  void set_services_armed(bool armed);
  /** Enable or suppress `IRuntimeListener::on_live_telemetry()`. Also sticky. */
  void set_live_telemetry_enabled(bool enabled);
  /** Current armed state, including before setup. */
  bool services_armed() const { return services_armed_; }
  /**
   * Temporarily quiet the runtime without releasing its backend or configuration.
   *
   * Disables live telemetry and sensing services, and stops active raw collection.
   * Restore the desired service and telemetry gates explicitly when resuming.
   */
  void quiesce();

  /**
   * Set the motion threshold, validating it against the active detector.
   *
   * @param threshold Value on the 0..1 metric scale.
   * @return false when out of range, or when the backend refuses it. Before
   *         setup the value is staged and returns true.
   */
  bool set_threshold_runtime(float threshold);
  /**
   * Set the hit filter.
   *
   * @param motion_on_hits Consecutive above-threshold evaluations to report
   *        motion (1..20). Higher trades latency for fewer false positives.
   * @param motion_off_hits Consecutive below-threshold evaluations to clear it
   *        (1..20).
   * @return false when either value is out of range, or when the runtime is up
   *         and does not advertise
   *         `RuntimeCapabilities::supports_runtime_motion_hits_updates`.
   */
  bool set_motion_hits_runtime(uint8_t motion_on_hits, uint8_t motion_off_hits);
  /**
   * Change the live CSI traffic ownership mode.
   *
   * @return false when the mode is invalid, or when the runtime is up and does
   *         not advertise `RuntimeCapabilities::supports_traffic_control`.
   */
  bool set_csi_traffic_mode_runtime(CsiTrafficMode mode);
  /**
   * Change the live internal traffic generator packet type.
   *
   * @return false when the mode is invalid, or when the runtime is up and does
   *         not advertise `RuntimeCapabilities::supports_traffic_control`.
   */
  bool set_traffic_generator_mode_runtime(RuntimeTrafficMode mode);
  /**
   * Switch detector while running.
   *
   * The threshold follows the detector: the controller adopts the new
   * detector's threshold rather than carrying the old value across scales.
   *
   * @return false for an unknown algorithm, or when the runtime is up and does
   *         not advertise
   *         `RuntimeCapabilities::supports_runtime_detector_selection`.
   */
  bool set_detection_algorithm_runtime(DetectionAlgorithm algorithm);
  /**
   * Restart startup calibration.
   *
   * @return false before setup, or when the backend does not advertise
   *         `RuntimeCapabilities::supports_manual_recalibration`. Success only
   *         means calibration started; the outcome arrives through
   *         `IRuntimeListener::on_calibration_finished()`.
   */
  bool trigger_recalibration();
  /** True while the backend is calibrating. False before setup. */
  bool is_calibrating() const;
  /**
   * Enter transient raw collection through the active sensing backend.
   *
   * The callback runs synchronously in the Wi-Fi CSI capture context, not from
   * `loop()`. It must remain bounded, non-blocking, and allocation-free. Copy
   * any bytes needed after the callback returns; the packet view and its CSI
   * buffer expire with the call. See `raw_csi_packet_callback_t` for the return
   * convention.
   *
   * @param callback Capture-context packet consumer. Must not be `nullptr`.
   * @param context Opaque caller-owned value passed to every callback. The
   *        caller must keep it valid until collection stops.
   * @return false before setup, without raw-CSI capability, with no Wi-Fi link,
   *         or when another transient operation is active.
   */
  bool start_raw_collection(raw_csi_packet_callback_t callback, void *context);
  /** Leave transient raw collection and restore the prior armed state. */
  bool stop_raw_collection(RawCsiStopReason reason = RawCsiStopReason::REQUESTED);
  /** Current transient backend operation. */
  RuntimeOperationState operation_state() const;

 private:
  void on_motion_state_changed(const RuntimeSnapshot &snapshot) override;
  void on_periodic_update(const RuntimeSnapshot &snapshot, uint32_t packets_received) override;
  void on_threshold_changed(const RuntimeSnapshot &snapshot) override;
  void on_detector_changed(const RuntimeSnapshot &snapshot) override;
  void on_calibration_started(const RuntimeSnapshot &snapshot) override;
  void on_calibration_finished(const RuntimeSnapshot &snapshot, bool success) override;
  void on_live_telemetry(float movement, float threshold) override;
  void on_runtime_fault(const char *message) override;

  void cache_snapshot_(const RuntimeSnapshot &snapshot);
  void adopt_effective_threshold_(float threshold);
  void adopt_effective_detector_(DetectionAlgorithm algorithm);
  void begin_callback_();
  void end_callback_();
  void apply_deferred_shutdown_();

  RuntimeConfig config_{};
  RuntimeConfig active_config_{};
  RuntimeSnapshot snapshot_{};
  RuntimeCapabilities capabilities_{};
  std::unique_ptr<IEspectreRuntime> runtime_;
  IRuntimeListener *listener_{nullptr};
  bool setup_complete_{false};
  bool services_armed_{true};
  bool live_telemetry_enabled_{true};
  uint8_t callback_depth_{0U};
  bool shutdown_requested_{false};
};

}  // namespace espectre
