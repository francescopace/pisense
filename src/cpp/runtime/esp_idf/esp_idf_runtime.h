/*
 * ESPectre - ESP-IDF Runtime
 *
 * ESP-IDF runtime that wires Wi-Fi, CSI capture, calibration, and
 * detection together.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include "base_detector.h"
#include "csi_pipeline.h"
#include "esp_idf_runtime_base.h"
#include "pending_event.h"
#include "periodic_sensing_status_logger.h"
#include "runtime_diagnostics.h"
#include "runtime_interface.h"
#include "csi_traffic_service.h"
#include "threshold.h"
#include "traffic_generator_manager.h"
#include "udp_listener.h"
#include "wifi_lifecycle.h"

namespace espectre {

class EspIdfRuntime : public EspIdfRuntimeBase {
 public:
  explicit EspIdfRuntime(const RuntimeConfig &config);
  EspIdfRuntime(const RuntimeConfig &config,
                ICsiTrafficGenerator &traffic_generator,
                ICsiTrafficIngress &traffic_ingress);

  /** Configuration actually applied after persisted overrides are loaded. */
  const RuntimeConfig &effective_config() const { return config_; }
  RuntimeSnapshot get_snapshot() const override;
  RuntimeDiagnosticsSnapshot get_diagnostics() const override;
  const RuntimeDiagnosticsSample *get_diagnostics_sample() const override;

  bool setup() override;
  void shutdown() override;
  void loop() override;
  void set_services_armed(bool armed) override;
  void set_live_telemetry_enabled(bool enabled) override;

  bool set_threshold_runtime(float threshold) override;
  bool set_motion_hits_runtime(uint8_t motion_on_hits, uint8_t motion_off_hits) override;
  bool set_csi_traffic_mode_runtime(CsiTrafficMode mode) override;
  bool set_traffic_generator_mode_runtime(RuntimeTrafficMode mode) override;
  bool set_detection_algorithm_runtime(DetectionAlgorithm algorithm) override;
  bool trigger_recalibration() override;
  bool is_calibrating() const override;
  bool start_raw_collection(raw_csi_packet_callback_t callback, void *context) override;
  bool stop_raw_collection(RawCsiStopReason reason) override;
  RuntimeOperationState operation_state() const override;

 private:
  void update_live_telemetry_callback_();
  void notify_threshold_if_changed_(float threshold);
  bool configure_detector_();
  std::unique_ptr<BaseDetector> make_detector_(DetectionAlgorithm algorithm, float threshold,
                                               uint16_t window_packets);
  void cancel_calibration_(bool notify_listener);
  void on_wifi_connected_(const esp_netif_ip_info_t &ip_info);
  void on_wifi_disconnected_();
  void invalidate_csi_receive_path_refresh_();
  void maybe_resume_sensing_after_wifi_reconfigure_();
  void finish_csi_receive_path_refresh_(esp_err_t result);
  void refresh_wifi_association_from_csi_();
  void start_sensing_services_(const esp_netif_ip_info_t &ip_info);
  CsiCaptureProfile sensing_capture_profile_() const;
  void stop_sensing_services_();
  void on_csi_channel_changed_(uint8_t previous_channel, uint8_t current_channel);
  bool apply_traffic_runtime_config_(bool restart_service, bool recalibrate_if_active);
  void restore_traffic_runtime_config_(const RuntimeConfig &previous_config);
  bool start_calibration_(bool reset_high_accuracy_threshold = true);
  bool handle_threshold_calibration_packet_(const int8_t *csi_data, size_t csi_len,
                                            int8_t rssi_dbm, bool evaluation_due,
                                            uint32_t packets_in_window,
                                            bool temporal_reset);
  static bool threshold_calibration_packet_callback_(void *context,
                                                     const int8_t *csi_data,
                                                     size_t csi_len,
                                                     int8_t rssi_dbm,
                                                     bool evaluation_due,
                                                     uint32_t packets_in_window,
                                                     bool temporal_reset);
  void finish_threshold_calibration_(bool success);
  void refresh_csi_local_identity_(uint32_t local_ip_addr);
  void log_periodic_status_(uint32_t packets_received);
  void reset_periodic_status_logger_();
  void initialize_runtime_state_();

  std::unique_ptr<BaseDetector> detector_;
  uint16_t resolved_window_packets_{DETECTOR_DEFAULT_WINDOW_SIZE};

  CsiPipeline csi_pipeline_;
  WiFiLifecycleManager wifi_lifecycle_;
  TrafficGeneratorManager traffic_generator_;
  UDPListener traffic_ingress_;
  CsiTrafficService csi_traffic_service_;

  PeriodicSensingStatusLogger status_logger_{};
  RuntimeDiagnosticsSampler diagnostics_sampler_{};
  RuntimeDiagnosticsSample latest_diagnostics_{};
  std::unique_ptr<StartupThresholdCalibrator> threshold_calibrator_;
  std::atomic<bool> threshold_calibration_active_{false};
  // Posted from the CSI callback with the outcome, completed from the loop.
  PendingEvent<bool> calibration_finished_event_;
  bool wifi_ready_{false};
  esp_netif_ip_info_t wifi_ip_info_{};
  int8_t wifi_rssi_dbm_{INT8_MIN};
  uint8_t wifi_channel_{0U};
  bool csi_receive_path_refresh_required_{false};
  bool csi_receive_path_refresh_in_progress_{false};
  std::atomic<RuntimeOperationState> operation_state_{RuntimeOperationState::SENSING};
};

}  // namespace espectre
