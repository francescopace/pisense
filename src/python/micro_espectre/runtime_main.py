# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
Micro-ESPectre - Main Application

Main entry point for the Micro-ESPectre Wi-Fi CSI runtime.

Author: Francesco Pace <francesco.pace@gmail.com>
"""
import time
import gc
import os
import src.config as config
from src.config import NUM_SUBCARRIERS, EXPECTED_CSI_LEN
from src.device_utils import (
    CsiPayloadNormalizationState,
    CsiFrameTimestampFilter,
    DETECTOR_RESET_DROP_STREAK,
    DISPOSITION_SENSE,
    NORMALIZATION_DOUBLE_HT20,
    NORMALIZATION_DOUBLE_HT57_TO_64,
    NORMALIZATION_HT57_TO_64,
    NORMALIZATION_LLTF53_TO_64,
    assess_ht20_sensing_frame,
    impute_ht20_lltf_detector_bins,
    normalize_ht20_csi_payload,
    csi_read_frame,
    select_csi_capture_profile,
)
from src.detector_interface import (
    get_detector_label,
    load_detector_class,
)
from src.runtime_motion_policy import RuntimeMotionPolicy
from src.wifi_bootstrap import cleanup_wifi, connect_wifi, print_wifi_status, recover_wifi

try:
    from src.console_output import format_detection_publish_line, print_log
except ImportError:
    from console_output import format_detection_publish_line, print_log

HEARTBEAT_INTERVAL_MS = 1000
HEARTBEAT_MAX_DRAIN_DEFERRAL_MS = 100
DIAGNOSTIC_INTERVAL_MS = 1000
DIAGNOSTIC_MAX_DRAIN_DEFERRAL_MS = 500

# Global state for calibration mode and performance metrics
class GlobalState:
    def __init__(self):
        self.calibration_mode = False  # Authoritative calibration status
        self.chip_type = None  # Detected chip type (S3, C6, etc.)
        self.csi_phy_metadata_missing = False  # C5/C6 RX metadata omits legacy PHY fields
        self.csi_capture_profile = "ht20"  # Runtime-owned, read-only CSI profile
        self.current_channel = 0  # Track WiFi channel for change detection


g_state = GlobalState()


def refresh_csi_capture_context(wlan):
    """Refresh the runtime-owned channel and CSI profile after association."""
    try:
        active_channel = wlan.config('channel')
    except Exception:
        active_channel = 0
    previous_profile = g_state.csi_capture_profile
    g_state.current_channel = active_channel
    g_state.csi_capture_profile = select_csi_capture_profile(
        g_state.chip_type, active_channel
    )
    return previous_profile != g_state.csi_capture_profile


def print_heap(label):
    """Print a compact heap snapshot for boot/runtime profiling."""
    idf_suffix = ""
    try:
        import esp32

        regions = esp32.idf_heap_info(esp32.HEAP_DATA)
        idf_free = sum(region[1] for region in regions)
        idf_largest = max((region[2] for region in regions), default=0)
        idf_minimum = sum(region[3] for region in regions)
        idf_suffix = " idf_free={} idf_largest={} idf_min={}".format(
            idf_free,
            idf_largest,
            idf_minimum,
        )
    except (ImportError, AttributeError, OSError, TypeError, ValueError):
        pass
    print(
        "[MEM] {}: free={} alloc={}{}".format(
            label,
            gc.mem_free(),
            gc.mem_alloc(),
            idf_suffix,
        )
    )


def collect_and_print_heap(label):
    """Collect garbage, then print the resulting compact heap snapshot."""
    gc.collect()
    print_heap(label)


def create_detector(detection_algorithm, window_packets):
    """
    Create the configured detector instance from the shared registry.

    The runtime keeps one common detector contract:
    - canonical algorithm key via `ALGORITHM`
    - shared `motion_metric` field in update_state()
    """
    try:
        detector_class = load_detector_class(detection_algorithm)
    except ValueError:
        raise ValueError(f"Unsupported Micro detector: {detection_algorithm}") from None

    print_log("INFO", "Detection algorithm: {}".format(get_detector_label(detection_algorithm)))
    detector = detector_class(
        window_size=window_packets,
        threshold=1.0,
        enable_lowpass=config.ENABLE_LOWPASS_FILTER,
        lowpass_cutoff=config.LOWPASS_CUTOFF,
        enable_hampel=config.ENABLE_HAMPEL_FILTER,
        hampel_window=config.HAMPEL_WINDOW,
        hampel_threshold=config.HAMPEL_THRESHOLD,
    )
    get_backend = getattr(detector, "get_backend", None)
    backend = get_backend() if callable(get_backend) else None
    if backend != "espectre_core":
        raise RuntimeError(
            "Micro-ESPectre requires the espectre_core detector backend"
        )
    print_log("INFO", "Detector backend: {}".format(backend))
    return detector


def run_startup_calibration(wlan, detector, traffic_gen):
    """
    Run startup calibration with fixed subcarriers.

    Args:
        wlan: WLAN instance
        detector: IDetector instance
        traffic_gen: TrafficGenerator instance
    Returns:
        bool: True if startup calibration completed
    """
    detector_name = detector.get_name()
    g_state.calibration_mode = True

    gc.collect()
    detector.reset()

    print('')
    print('='*60)
    print('Startup Threshold Calibration')
    print('='*60)
    print(f'Free memory: {gc.mem_free()} bytes')
    print('Calibration: stay quiet first, then one short motion is OK.')

    from src.threshold import (
        StartupThresholdCalibrator,
        get_detector_auto_factor,
        get_detector_startup_gate,
    )
    from src.temporal_csi_sampler import (
        TemporalCsiSampler,
        minimum_valid_slots,
        temporal_window_slots,
    )

    target_pps = max(1, int(getattr(config, 'CSI_TARGET_PPS', 100)))
    calibration_target_packets = temporal_window_slots(
        target_pps,
        getattr(config, 'CALIBRATION_DURATION_MS', 10_000),
    )
    calibration_tracker = StartupThresholdCalibrator(
        calibration_target_packets,
        auto_factor=get_detector_auto_factor(detector),
        gate_enabled=get_detector_startup_gate(detector),
    )
    begin_calibration = getattr(detector, "on_startup_calibration_begin", None)
    if callable(begin_calibration):
        begin_calibration()
    evaluation_interval_ms = max(1, int(getattr(config, 'EVALUATION_INTERVAL_MS', 250)))
    # Calibration evaluates on the same cadence steady-state detection does.
    # Mirrors the C++ EvaluationCadence shared by CsiPipeline and its
    # calibration interceptor.
    calibration_cadence = RuntimeMotionPolicy(
        evaluation_interval_ms=evaluation_interval_ms,
        segmentation_window_size_ms=getattr(config, 'SEGMENTATION_WINDOW_SIZE_MS', 1000),
    )
    temporal_sampler = TemporalCsiSampler(
        target_pps,
        getattr(config, 'SEGMENTATION_WINDOW_SIZE_MS', 1000),
    )
    set_minimum_valid = getattr(detector, "set_minimum_valid_samples", None)
    if callable(set_minimum_valid):
        set_minimum_valid(minimum_valid_slots(temporal_sampler.window_slots))

    print('')
    print('-'*60)
    print(f'{detector_name} Threshold Bootstrap ({calibration_target_packets} packets, evaluate every {evaluation_interval_ms} ms) [HT20: {NUM_SUBCARRIERS} SC]')
    print('-'*60)

    max_timeout_ms = 15000
    # Allow warmup and reduced occupancy, but never let arrivals or repeated
    # sampler resets extend calibration indefinitely.
    calibration_timeout_ms = max(
        max_timeout_ms,
        2 * int(getattr(config, 'CALIBRATION_DURATION_MS', 10_000))
        + int(getattr(config, 'SEGMENTATION_WINDOW_SIZE_MS', 1000)),
    )
    filtered_count = 0
    accepted_packet_count = 0
    calibration_progress = 0
    packets_since_evaluation = 0
    next_progress_report = 100
    last_packet_time = time.ticks_ms()
    calibration_started_ms = last_packet_time
    rate_report_time = last_packet_time
    rate_previous_accepted = 0
    rate_previous_admitted = 0
    rate_previous_tx = traffic_gen.get_packet_count()
    get_dropped = getattr(wlan, "csi_dropped", None)
    rate_previous_dropped = int(get_dropped()) if callable(get_dropped) else 0
    collapse_logged = False
    remap_logged = False
    ht57_remap_buffer = bytearray(EXPECTED_CSI_LEN)
    use_lltf = g_state.csi_capture_profile == 'lltf20'
    lltf_detector_buffer = bytearray(EXPECTED_CSI_LEN) if use_lltf else None
    normalization_state = CsiPayloadNormalizationState()
    frame_timestamp_filter = CsiFrameTimestampFilter()
    frame_result = None
    pending_csi_data = bytearray(EXPECTED_CSI_LEN)
    emitted_csi_data = bytearray(EXPECTED_CSI_LEN)
    pending_timestamp_us = 0
    # Reused per-frame assessment mapping: keeps this loop allocation-free.
    assessment_result = {}
    advance_missing = getattr(detector, "advance_missing_slots", None)
    while not calibration_tracker.is_complete():
        now_ms = time.ticks_ms()
        if (time.ticks_diff(now_ms, calibration_started_ms) >= calibration_timeout_ms
                or time.ticks_diff(now_ms, last_packet_time) >= max_timeout_ms):
            print_log(
                "WARN",
                "Startup calibration aborted: timed out waiting for sufficient CSI coverage "
                "(collected {}/{})".format(
                    calibration_progress, calibration_target_packets,
                ),
            )
            detector.reset()
            g_state.calibration_mode = False
            return False
        frame = csi_read_frame(wlan, frame_result)
        if frame:
            frame_result = frame
            assessment = assess_ht20_sensing_frame(
                frame,
                frame[5],
                expected_len=EXPECTED_CSI_LEN,
                metadata_missing=g_state.csi_phy_metadata_missing,
                allow_legacy_lltf=use_lltf,
                out=assessment_result,
                static_fast_path=True,
            )
            if assessment["disposition"] != DISPOSITION_SENSE:
                filtered_count += 1
                if filtered_count % 100 == 1:
                    print_log(
                        "WARN",
                        "Filtered {} packets before calibration "
                        "(reason={}, len={})".format(
                            filtered_count,
                            assessment["reason_code"],
                            assessment["raw_len"],
                        ),
                    )
                del frame
                continue

            csi_data, _, remap_tag = normalize_ht20_csi_payload(
                frame[5], EXPECTED_CSI_LEN,
                remap_buffer=ht57_remap_buffer,
                assessment=assessment,
                state=normalization_state,
            )
            if csi_data is None:
                filtered_count += 1
                del frame
                continue

            if use_lltf:
                csi_data = impute_ht20_lltf_detector_bins(
                    csi_data, lltf_detector_buffer
                )

            if not frame_timestamp_filter.accept(frame):
                del frame
                continue

            if remap_tag in (NORMALIZATION_DOUBLE_HT20, NORMALIZATION_DOUBLE_HT57_TO_64) and not collapse_logged:
                print_log("INFO", "CSI double-length collapse active: 256->128 and/or 228->114")
                collapse_logged = True
            if remap_tag in (NORMALIZATION_LLTF53_TO_64, NORMALIZATION_HT57_TO_64, NORMALIZATION_DOUBLE_HT57_TO_64) and not remap_logged:
                print_log("INFO", "CSI compact payload remap active: 53/57->64 SC")
                remap_logged = True
            del frame
            current_timestamp_us = frame_result[4]
            accepted_packet_count += 1
            emitted = temporal_sampler.admit(current_timestamp_us)
            if emitted:
                emitted_csi_data[:] = pending_csi_data
                emitted_timestamp_us = pending_timestamp_us
            if temporal_sampler.selected_current:
                pending_csi_data[:] = csi_data
                pending_timestamp_us = current_timestamp_us
            if emitted:
                if temporal_sampler.reset_required:
                    detector.reset()
                    calibration_cadence.reset()
                    calibration_tracker = StartupThresholdCalibrator(
                        calibration_target_packets,
                        auto_factor=get_detector_auto_factor(detector),
                        gate_enabled=get_detector_startup_gate(detector),
                    )
                    begin_calibration = getattr(detector, "on_startup_calibration_begin", None)
                    if callable(begin_calibration):
                        begin_calibration()
                    packets_since_evaluation = 0
                if temporal_sampler.missing_slots_before and callable(advance_missing):
                    advance_missing(temporal_sampler.missing_slots_before)
                detector.process_packet(
                    emitted_csi_data,
                    config.DEFAULT_SUBCARRIERS,
                    timestamp_us=emitted_timestamp_us,
                )
                packets_since_evaluation += 1
                last_packet_time = time.ticks_ms()
                calibration_cadence.note_arrival(emitted_timestamp_us)
            if temporal_sampler.gap_reset_required:
                detector.reset()
                calibration_cadence.reset()
                calibration_tracker = StartupThresholdCalibrator(
                    calibration_target_packets,
                    auto_factor=get_detector_auto_factor(detector),
                    gate_enabled=get_detector_startup_gate(detector),
                )
                begin_calibration = getattr(detector, "on_startup_calibration_begin", None)
                if callable(begin_calibration):
                    begin_calibration()
                packets_since_evaluation = 0
            if not emitted:
                continue

            if not calibration_cadence.should_evaluate():
                continue
            calibration_cadence.after_evaluation()

            detector.update_state()
            if detector.is_ready():
                calibration_tracker.observe_detector(
                    detector,
                    packet_weight=packets_since_evaluation,
                )
            packets_since_evaluation = 0
            calibration_progress = calibration_tracker.packet_count
            if calibration_progress >= next_progress_report:
                current_time = time.ticks_ms()
                elapsed_ms = max(1, time.ticks_diff(current_time, rate_report_time))
                rate_scale = 1000.0 / elapsed_ms
                admitted_total = temporal_sampler.accepted_packets
                tx_total = traffic_gen.get_packet_count()
                dropped_total = int(get_dropped()) if callable(get_dropped) else 0
                admitted_pps = (admitted_total - rate_previous_admitted) * rate_scale
                accepted_pps = (accepted_packet_count - rate_previous_accepted) * rate_scale
                tx_pps = (tx_total - rate_previous_tx) * rate_scale
                dropped_pps = (dropped_total - rate_previous_dropped) * rate_scale
                current_mv = detector.get_motion_metric() if detector.is_ready() else None
                movement_text = current_mv if current_mv is not None else "--"
                print(
                    "Calibration {}/{} | mvmt:{} thr:{:.6f} | "
                    "csi:{:.1f}/{:.1f}pps tx:{:.1f}pps drop:{:.1f}pps".format(
                        calibration_progress,
                        calibration_target_packets,
                        movement_text,
                        detector.get_threshold(),
                        admitted_pps,
                        accepted_pps,
                        tx_pps,
                        dropped_pps,
                    )
                )
                rate_report_time = current_time
                rate_previous_accepted = accepted_packet_count
                rate_previous_admitted = admitted_total
                rate_previous_tx = tx_total
                rate_previous_dropped = dropped_total
                while next_progress_report <= calibration_progress:
                    next_progress_report += 100
        else:
            time.sleep_us(100)

    gc.collect()
    success = calibration_tracker.is_successful()
    if success:
        startup_threshold, threshold_formula = calibration_tracker.calculate_threshold()
        detector.set_adaptive_threshold(startup_threshold)
        startup_threshold = detector.get_threshold()
        threshold_source = f"automatic ({threshold_formula})"
        print_log(
            "INFO",
            "Startup threshold: {:.4f} ({})".format(startup_threshold, threshold_source),
        )

        detector.reset()

        print('')
        print('='*60)
        print(f'{detector_name} Startup Calibration Complete!')
        print(f'   Subcarriers: {list(config.DEFAULT_SUBCARRIERS)}')
        print(f'   Threshold: {detector.get_threshold():.4f} ({threshold_source})')
        print('='*60)
        print('')
    else:
        print('')
        print('='*60)
        print(f'{detector_name} Startup Calibration Failed')
        print(f'   Keeping threshold: {detector.get_threshold():.4f}')
        print(f'   Subcarriers: {list(config.DEFAULT_SUBCARRIERS)}')
        print('='*60)
        print('')

    g_state.calibration_mode = False
    return success


def get_chip_type():
    """Extract short chip type from os.uname().machine."""
    machine = os.uname().machine.upper()
    # Check for specific variants first
    for variant in ['S2', 'S3', 'C3', 'C5', 'C6']:
        if variant in machine:
            return variant
    # Fallback to ESP32 base
    if 'ESP32' in machine:
        return 'ESP32'
    return machine


def restart_traffic_generator(traffic_gen):
    """Restart the traffic generator after calibration-sensitive work completes."""
    if not traffic_gen or not getattr(config, 'TRAFFIC_GENERATOR_ENABLED', True):
        return

    time.sleep(1)  # Give the Wi-Fi stack time to settle before reopening the raw socket.
    gc.collect()
    target_pps = max(1, int(getattr(config, 'CSI_TARGET_PPS', 100)))
    if not traffic_gen.start(target_pps):
        print_log("WARN", "Failed to restart traffic generator, retrying...")
        time.sleep(2)
        gc.collect()
        traffic_gen.start(target_pps)


def main(wlan=None):
    """Main application loop"""
    print_log("INFO", "Micro-ESPectre starting...")
    collect_and_print_heap('boot')

    # Detect chip type
    g_state.chip_type = get_chip_type()
    # ESP-IDF's CSI RX-control v2, used by C5 and C6, does not expose the
    # sig_mode/cwb fields carried by the classic RX-control structure.
    g_state.csi_phy_metadata_missing = g_state.chip_type in ("C5", "C6")
    print_log("INFO", "Detected chip: {}".format(g_state.chip_type))

    # Connect to WiFi
    if wlan is None:
        wlan = connect_wifi()
    refresh_csi_capture_context(wlan)
    print_log("INFO", "CSI capture profile: {}".format(g_state.csi_capture_profile))
    collect_and_print_heap('after_connect_wifi')

    # Detector capacity is fixed by the configured temporal grid. Measured
    # delivery rate is diagnostic only and never reconstructs the detector.
    detection_algorithm = 'lightweight'
    from src.temporal_csi_sampler import (
        TemporalCsiSampler,
        minimum_valid_slots,
        temporal_window_slots,
    )
    target_pps = max(1, int(getattr(config, 'CSI_TARGET_PPS', 100)))
    detector_window_packets = temporal_window_slots(
        target_pps,
        getattr(config, 'SEGMENTATION_WINDOW_SIZE_MS', 1000),
    )

    # Initialize and start traffic generator (target CSI rate from config.py)
    gc.collect()  # Free memory before creating socket
    from src.traffic_generator import TrafficGenerator
    traffic_gen = TrafficGenerator(
        getattr(config, 'TRAFFIC_GENERATOR_MODE', 'ping')
    )
    collect_and_print_heap('after_traffic_gen_init')
    if getattr(config, 'TRAFFIC_GENERATOR_ENABLED', True):
        if not traffic_gen.start(target_pps):
            print_log(
                "ERROR",
                "Traffic generator failed to start - CSI will not work; "
                "check WiFi connection and gateway availability",
            )
            import machine
            time.sleep(5)
            machine.reset()  # Reboot and retry

        print_log(
            "INFO",
            "Traffic generator started ({}, target={} CSI pps)".format(
                traffic_gen.get_mode(),
                target_pps,
            ),
        )
        collect_and_print_heap('after_traffic_gen_start')

        # Verify CSI packets are flowing with retry logic
        max_tg_retries = 3
        for tg_attempt in range(max_tg_retries):
            time.sleep(2)  # Wait for traffic to start generating CSI packets

            print_log("INFO", "Waiting for CSI packets...")
            csi_received = 0
            frame_result = None
            for _ in range(100):  # Max 100 attempts (~5 seconds)
                frame = csi_read_frame(wlan, frame_result)
                if frame:
                    frame_result = frame
                    csi_received += 1
                    if csi_received >= 17:
                        break
                time.sleep(0.05)

            if csi_received >= 17:
                break  # Success

            if tg_attempt < max_tg_retries - 1:
                print_log(
                    "WARN",
                    "Only {} CSI packets - restarting TG (attempt {}/{})".format(
                        csi_received,
                        tg_attempt + 2,
                        max_tg_retries,
                    ),
                )
                traffic_gen.stop()
                time.sleep(1)
                traffic_gen.start(target_pps)
            else:
                print_log(
                    "ERROR",
                    "No CSI packets after {} attempts - cannot operate without traffic; "
                    "check WiFi connection and retry".format(max_tg_retries),
                )
                import sys
                sys.exit(1)
        collect_and_print_heap('after_csi_flow_check')

    else:
        print_log("INFO", "Waiting for external CSI packets...")
        csi_timestamps = []
        frame_result = None
        for _ in range(100):
            frame = csi_read_frame(wlan, frame_result)
            if frame:
                frame_result = frame
                csi_timestamps.append(int(frame[4]))
                if len(csi_timestamps) >= 17:
                    break
            time.sleep(0.05)
        deltas = []
        for previous, current in zip(csi_timestamps, csi_timestamps[1:], strict=False):
            delta = (current - previous) % (1 << 32)
            if 0 < delta < (1 << 31):
                deltas.append(delta)
        if not deltas:
            raise RuntimeError('External CSI traffic did not provide advancing timestamps')

    detector = create_detector(detection_algorithm, detector_window_packets)
    set_minimum_valid = getattr(detector, "set_minimum_valid_samples", None)
    if callable(set_minimum_valid):
        set_minimum_valid(minimum_valid_slots(detector_window_packets))
    print_log(
        "INFO",
        "Detector window: {} samples for {} ms".format(
            detector_window_packets,
            getattr(config, "SEGMENTATION_WINDOW_SIZE_MS", 1000),
        ),
    )
    collect_and_print_heap('after_detector_init')

    # Detector allocation can fragment the original ESP32 heap enough that the
    # socket opened during the flow probe starts returning ENOMEM. Reopen it
    # against the post-allocation heap before calibration begins.
    if getattr(config, 'TRAFFIC_GENERATOR_ENABLED', True):
        traffic_gen.stop()
        time.sleep(1)
        gc.collect()
        if not traffic_gen.start(target_pps):
            cleanup_wifi(wlan)
            raise RuntimeError("Traffic generator restart failed after detector initialization")
        time.sleep(1)

    runtime_policy = RuntimeMotionPolicy(
        evaluation_interval_ms=getattr(config, 'EVALUATION_INTERVAL_MS', 250),
        motion_on_hits=getattr(config, 'MOTION_ON_HITS', 4),
        motion_off_hits=getattr(config, 'MOTION_OFF_HITS', 3),
        segmentation_window_size_ms=getattr(config, 'SEGMENTATION_WINDOW_SIZE_MS', 1000),
    )
    calibration_ok = run_startup_calibration(
        wlan,
        detector,
        traffic_gen,
    )
    if not calibration_ok:
        if traffic_gen.is_running():
            traffic_gen.stop()
        cleanup_wifi(wlan)
        raise RuntimeError("Startup calibration failed")
    collect_and_print_heap('after_calibration')

    # The calibration helper is large and is not needed by the steady-state loop.
    import sys
    sys.modules.pop("src.threshold", None)
    # Retain the already allocated traffic socket.
    gc.collect()

    from src.branding import ASCII_BANNER
    if getattr(config, 'TRAFFIC_GENERATOR_ENABLED', True) and not traffic_gen.is_running():
        restart_traffic_generator(traffic_gen)

    print('')
    print(ASCII_BANNER)
    print('')

    # Force garbage collection before main loop
    gc.collect()
    print_log(
        "INFO",
        "Free memory before main loop: {} bytes".format(gc.mem_free()),
    )

    # Main CSI processing loop with bounded Direct HTTP publishing.
    processed_packet_count = 0
    callback_packet_count = 0
    filtered_count = 0  # Packets with wrong SC count
    last_heartbeat_time = time.ticks_ms()
    last_diagnostic_time = last_heartbeat_time
    last_csi_frame_time = last_heartbeat_time
    csi_recovery_attempts = 0
    collapse_logged = False
    remap_logged = False
    ht57_remap_buffer = bytearray(EXPECTED_CSI_LEN)
    use_lltf = g_state.csi_capture_profile == 'lltf20'
    lltf_detector_buffer = bytearray(EXPECTED_CSI_LEN) if use_lltf else None
    normalization_state = CsiPayloadNormalizationState()
    frame_timestamp_filter = CsiFrameTimestampFilter()
    out_of_order_count = 0
    frame_result = None
    format_drop_streak = 0
    last_normalization_id = None
    # Reused per-frame assessment mapping: keeps the hot loop allocation-free.
    assessment_result = {}
    temporal_sampler = TemporalCsiSampler(
        target_pps,
        getattr(config, 'SEGMENTATION_WINDOW_SIZE_MS', 1000),
    )
    from src.runtime_diagnostics import (
        RuntimeDiagnosticsSampler,
        RuntimePerformanceDiagnostics,
        advance_periodic_anchor,
        collect_runtime_diagnostics_snapshot,
        periodic_maintenance_due,
        wifi_csi_available,
        wifi_csi_callbacks,
        wifi_csi_dropped,
        wifi_rssi_dbm,
    )
    diagnostics_sampler = RuntimeDiagnosticsSampler()
    performance_diagnostics = RuntimePerformanceDiagnostics()
    cumulative_diagnostics = {}
    diagnostics = {}
    native_callback_total = wifi_csi_callbacks(wlan)
    diagnostics_sampler.reset(
        collect_runtime_diagnostics_snapshot(
            traffic_generator=traffic_gen,
            callback_total=(
                native_callback_total
                if native_callback_total is not None
                else wifi_csi_dropped(wlan)
            ),
            accepted_total=0,
            admitted_total=0,
            filtered_total=0,
            missing_slots_total=0,
            excess_total=0,
            stale_total=0,
            out_of_order_total=0,
            occupancy_slots=0,
            window_slots=temporal_sampler.window_slots,
            wifi_channel=g_state.current_channel,
            rssi_dbm=wifi_rssi_dbm(wlan),
            out=cumulative_diagnostics,
        ),
        time.ticks_ms(),
    )
    pending_csi_data = bytearray(EXPECTED_CSI_LEN)
    emitted_csi_data = bytearray(EXPECTED_CSI_LEN)
    pending_timestamp_us = 0
    latest_motion_metric = 0.0
    latest_threshold = detector.get_threshold()
    latest_effective_state = runtime_policy.effective_state
    latest_loop_duration_us = 0
    loop_measurement_counter = 0
    from src.direct_api import DirectApi
    direct_api = DirectApi(
        config,
        wlan,
        detector,
        g_state,
        runtime_policy,
        traffic_gen,
    )
    advance_missing = getattr(detector, "advance_missing_slots", None)
    try:
        direct_api.start()
        collect_and_print_heap('after_direct_http_start')
        while True:
            loop_measurement_counter = (loop_measurement_counter + 1) & 7
            measure_loop = loop_measurement_counter == 0
            loop_start = time.ticks_us() if measure_loop else 0
            current_time = time.ticks_ms()
            time_delta = time.ticks_diff(current_time, last_heartbeat_time)
            status_due = time_delta >= HEARTBEAT_INTERVAL_MS
            if status_due and direct_api.take_recalibration_request():
                recalibration_ok = False
                try:
                    g_state.calibration_mode = True
                    direct_api.refresh_status(current_time)
                    recalibration_ok = run_startup_calibration(
                        wlan,
                        detector,
                        traffic_gen,
                    )
                    # The calibration helper is imported only for calibration;
                    # release its module mapping again before steady state.
                    import sys
                    sys.modules.pop("src.threshold", None)
                    gc.collect()
                    detector.reset()
                    runtime_policy.reset()
                    temporal_sampler.clear_history()
                    normalization_state.reset()
                    frame_timestamp_filter.reset()
                    pending_timestamp_us = 0
                    frame_result = None
                    format_drop_streak = 0
                    last_normalization_id = None
                    latest_motion_metric = 0.0
                    latest_threshold = detector.get_threshold()
                    latest_effective_state = runtime_policy.effective_state
                    csi_recovery_attempts = 0
                    performance_diagnostics.reset()
                    completed_time = time.ticks_ms()
                    native_callback_total = wifi_csi_callbacks(wlan)
                    diagnostics_sampler.reset(
                        collect_runtime_diagnostics_snapshot(
                            traffic_generator=traffic_gen,
                            callback_total=(
                                native_callback_total
                                if native_callback_total is not None
                                else callback_packet_count + wifi_csi_dropped(wlan)
                            ),
                            accepted_total=processed_packet_count,
                            admitted_total=temporal_sampler.accepted_packets,
                            filtered_total=filtered_count + out_of_order_count,
                            missing_slots_total=temporal_sampler.missing_slots,
                            excess_total=temporal_sampler.excess_packets,
                            stale_total=temporal_sampler.stale_packets,
                            out_of_order_total=temporal_sampler.out_of_order_packets,
                            occupancy_slots=temporal_sampler.occupancy_slots,
                            window_slots=temporal_sampler.window_slots,
                            wifi_channel=g_state.current_channel,
                            rssi_dbm=wifi_rssi_dbm(wlan),
                            out=cumulative_diagnostics,
                        ),
                        completed_time,
                    )
                    if not recalibration_ok:
                        print_log("WARN", "Direct recalibration did not find a stable threshold")
                    last_heartbeat_time = completed_time
                    last_diagnostic_time = completed_time
                    last_csi_frame_time = completed_time
                finally:
                    g_state.calibration_mode = False
                    try:
                        direct_api.refresh_config()
                        direct_api.refresh_status(time.ticks_ms())
                    finally:
                        direct_api.complete_recalibration()
                continue

            frame = csi_read_frame(wlan, frame_result)
            csi_backlog = wifi_csi_available(wlan)
            # Prefer an empty ring for allocation-heavy maintenance, but bound
            # that deferral so diagnostics and garbage collection cannot starve.
            drain_deferral_expired = (
                time_delta >= HEARTBEAT_INTERVAL_MS + HEARTBEAT_MAX_DRAIN_DEFERRAL_MS
            )
            ring_drained = csi_backlog == 0
            diagnostics_due = periodic_maintenance_due(
                current_time,
                last_diagnostic_time,
                DIAGNOSTIC_INTERVAL_MS,
                ring_drained,
                DIAGNOSTIC_MAX_DRAIN_DEFERRAL_MS,
            )
            if diagnostics_due:
                gc.collect()
                native_callback_total = wifi_csi_callbacks(wlan)
                diagnostics = diagnostics_sampler.sample(
                    collect_runtime_diagnostics_snapshot(
                        traffic_generator=traffic_gen,
                        callback_total=(
                            native_callback_total
                            if native_callback_total is not None
                            else callback_packet_count + wifi_csi_dropped(wlan)
                        ),
                        accepted_total=processed_packet_count,
                        admitted_total=temporal_sampler.accepted_packets,
                        filtered_total=filtered_count + out_of_order_count,
                        missing_slots_total=temporal_sampler.missing_slots,
                        excess_total=temporal_sampler.excess_packets,
                        stale_total=temporal_sampler.stale_packets,
                        out_of_order_total=temporal_sampler.out_of_order_packets,
                        occupancy_slots=temporal_sampler.occupancy_slots,
                        window_slots=temporal_sampler.window_slots,
                        wifi_channel=g_state.current_channel,
                        rssi_dbm=wifi_rssi_dbm(wlan),
                        out=cumulative_diagnostics,
                    ),
                    current_time,
                    out=diagnostics,
                )
                performance_diagnostics.update_if_due(
                    current_time,
                    gc.mem_free(),
                    out=diagnostics,
                )
                diagnostics["loop_time_ms"] = latest_loop_duration_us / 1000.0
                print(format_detection_publish_line(
                    diagnostics=diagnostics,
                    motion_metric=latest_motion_metric,
                    threshold=latest_threshold,
                    effective_state=latest_effective_state,
                ))
                direct_api.refresh_diagnostics(current_time, diagnostics)
                last_diagnostic_time = advance_periodic_anchor(
                    last_diagnostic_time,
                    time.ticks_ms(),
                    DIAGNOSTIC_INTERVAL_MS,
                )

            if status_due and (ring_drained or drain_deferral_expired):
                direct_api.refresh_status(current_time)
                last_heartbeat_time = advance_periodic_anchor(
                    last_heartbeat_time,
                    time.ticks_ms(),
                    HEARTBEAT_INTERVAL_MS,
                )

            if frame:
                last_csi_frame_time = current_time
                csi_recovery_attempts = 0
                frame_result = frame
                callback_packet_count += 1
                assessment = assess_ht20_sensing_frame(
                    frame,
                    frame[5],
                    expected_len=EXPECTED_CSI_LEN,
                    metadata_missing=g_state.csi_phy_metadata_missing,
                    allow_legacy_lltf=use_lltf,
                    out=assessment_result,
                    static_fast_path=True,
                )
                if assessment["disposition"] != DISPOSITION_SENSE:
                    filtered_count += 1
                    format_drop_streak += 1
                    if filtered_count % 100 == 1:
                        print_log(
                            "WARN",
                            "Filtered {} packets before detection "
                            "(reason={}, len={})".format(
                                filtered_count,
                                assessment["reason_code"],
                                assessment["raw_len"],
                            ),
                        )
                    del frame
                    if measure_loop:
                        latest_loop_duration_us = time.ticks_diff(time.ticks_us(), loop_start)
                        performance_diagnostics.record_loop_duration(
                            latest_loop_duration_us,
                            weight=8,
                        )
                    continue

                csi_data, _, remap_tag = normalize_ht20_csi_payload(
                    frame[5], EXPECTED_CSI_LEN,
                    remap_buffer=ht57_remap_buffer,
                    assessment=assessment,
                    state=normalization_state,
                )
                if csi_data is None:
                    filtered_count += 1
                    format_drop_streak += 1
                    del frame
                    if measure_loop:
                        latest_loop_duration_us = time.ticks_diff(time.ticks_us(), loop_start)
                        performance_diagnostics.record_loop_duration(
                            latest_loop_duration_us,
                            weight=8,
                        )
                    continue

                if use_lltf:
                    csi_data = impute_ht20_lltf_detector_bins(
                        csi_data, lltf_detector_buffer
                    )

                if not frame_timestamp_filter.accept(frame):
                    out_of_order_count += 1
                    if out_of_order_count % 100 == 1:
                        print_log(
                            "WARN",
                            "Filtered {} duplicate or out-of-order CSI frames".format(
                                out_of_order_count
                            ),
                        )
                    del frame
                    if measure_loop:
                        latest_loop_duration_us = time.ticks_diff(time.ticks_us(), loop_start)
                        performance_diagnostics.record_loop_duration(
                            latest_loop_duration_us,
                            weight=8,
                        )
                    continue

                should_reset_detector = (
                    format_drop_streak >= DETECTOR_RESET_DROP_STREAK
                    or (
                        last_normalization_id is not None
                        and assessment["normalization_id"] != last_normalization_id
                    )
                )
                format_drop_streak = 0
                if should_reset_detector:
                    print_log(
                        "WARN",
                        "CSI format stream changed after incompatible packets, resetting detection buffer",
                    )
                    detector.reset()
                    runtime_policy.reset()
                    frame_timestamp_filter.reset()
                last_normalization_id = assessment["normalization_id"]

                if remap_tag in (NORMALIZATION_DOUBLE_HT20, NORMALIZATION_DOUBLE_HT57_TO_64) and not collapse_logged:
                    print_log("INFO", "CSI double-length collapse active: 256->128 and/or 228->114")
                    collapse_logged = True
                if remap_tag in (NORMALIZATION_LLTF53_TO_64, NORMALIZATION_HT57_TO_64, NORMALIZATION_DOUBLE_HT57_TO_64) and not remap_logged:
                    print_log("INFO", "CSI compact payload remap active: 53/57->64 SC")
                    remap_logged = True
                packet_channel = frame[1]

                del frame

                processed_packet_count += 1

                if g_state.current_channel != 0 and packet_channel != g_state.current_channel:
                    print_log(
                        "WARN",
                        "WiFi channel changed: {} -> {}, resetting detection buffer".format(
                            g_state.current_channel,
                            packet_channel,
                        ),
                    )
                    detector.reset()
                    runtime_policy.reset()
                    temporal_sampler.clear_history()
                    normalization_state.reset()
                g_state.current_channel = packet_channel

                current_timestamp_us = frame_result[4]
                emitted = temporal_sampler.admit(current_timestamp_us)
                if emitted:
                    emitted_csi_data[:] = pending_csi_data
                    emitted_timestamp_us = pending_timestamp_us
                if temporal_sampler.selected_current:
                    pending_csi_data[:] = csi_data
                    pending_timestamp_us = current_timestamp_us
                if emitted:
                    if temporal_sampler.reset_required:
                        detector.reset()
                        runtime_policy.reset()
                        latest_motion_metric = 0.0
                        latest_effective_state = runtime_policy.effective_state
                    if temporal_sampler.missing_slots_before and callable(advance_missing):
                        advance_missing(temporal_sampler.missing_slots_before)

                    detector.process_packet(
                        emitted_csi_data,
                        config.DEFAULT_SUBCARRIERS,
                        timestamp_us=emitted_timestamp_us,
                    )
                    runtime_policy.note_arrival(emitted_timestamp_us)
                if temporal_sampler.gap_reset_required:
                    detector.reset()
                    runtime_policy.reset()
                    latest_motion_metric = 0.0
                    latest_effective_state = runtime_policy.effective_state
                if not emitted:
                    if measure_loop:
                        latest_loop_duration_us = time.ticks_diff(time.ticks_us(), loop_start)
                        performance_diagnostics.record_loop_duration(
                            latest_loop_duration_us,
                            weight=8,
                        )
                    continue

                if runtime_policy.should_evaluate():
                    detection_start = time.ticks_us()
                    metrics = detector.update_state()
                    performance_diagnostics.record_detection_duration(
                        time.ticks_diff(time.ticks_us(), detection_start),
                    )
                    effective_state, _ = runtime_policy.apply_state(metrics['state'])
                    runtime_policy.after_evaluation()
                    latest_motion_metric = metrics.get('motion_metric', 0.0)
                    latest_threshold = metrics['threshold']
                    latest_effective_state = effective_state
                    direct_api.publish_motion(
                        latest_motion_metric,
                        latest_effective_state,
                        latest_threshold,
                        current_time,
                    )

                if measure_loop:
                    latest_loop_duration_us = time.ticks_diff(time.ticks_us(), loop_start)
                    performance_diagnostics.record_loop_duration(
                        latest_loop_duration_us,
                        weight=8,
                    )

            else:
                recovery_recorded = False
                recovery_timeout_ms = max(
                    1000,
                    int(getattr(config, 'CSI_LINK_RECOVERY_TIMEOUT_MS', 5000)),
                )
                if time.ticks_diff(current_time, last_csi_frame_time) >= recovery_timeout_ms:
                    recovery_start_us = time.ticks_us()
                    print_heap('before_csi_recovery')
                    traffic_enabled = getattr(config, 'TRAFFIC_GENERATOR_ENABLED', True)
                    force_reconnect = csi_recovery_attempts > 0
                    station_reconnect = force_reconnect or not wlan.isconnected()
                    action = 'reconnecting WiFi' if station_reconnect else 'rearming CSI'
                    print_log("WARN", "CSI link stalled; {}".format(action))
                    if station_reconnect:
                        if traffic_enabled:
                            traffic_gen.stop()
                        # Tear down sockets and mDNS before cycling the station.
                        # Keeping Direct alive while its netif disappears can
                        # retain scarce lwIP and heap resources on small chips.
                        direct_api.stop()
                        gc.collect()
                    if not recover_wifi(wlan, force_reconnect=force_reconnect):
                        raise RuntimeError('CSI link recovery failed')
                    if station_reconnect:
                        profile_changed = refresh_csi_capture_context(wlan)
                        if profile_changed:
                            print_log(
                                "INFO",
                                "CSI capture profile changed to {}".format(
                                    g_state.csi_capture_profile
                                ),
                            )
                        use_lltf = g_state.csi_capture_profile == 'lltf20'
                        lltf_detector_buffer = (
                            bytearray(EXPECTED_CSI_LEN) if use_lltf else None
                        )
                        print_wifi_status(wlan)
                        if traffic_enabled and not traffic_gen.start(target_pps):
                            raise RuntimeError('CSI traffic generator recovery failed')
                        if not run_startup_calibration(wlan, detector, traffic_gen):
                            raise RuntimeError('CSI link recovery calibration failed')
                        import sys
                        # Release the calibration-only module mapping again.
                        sys.modules.pop("src.threshold", None)
                        gc.collect()
                    detector.reset()
                    runtime_policy.reset()
                    temporal_sampler.clear_history()
                    normalization_state.reset()
                    frame_timestamp_filter.reset()
                    pending_timestamp_us = 0
                    latest_motion_metric = 0.0
                    latest_effective_state = runtime_policy.effective_state
                    last_normalization_id = None
                    csi_recovery_attempts += 1
                    if station_reconnect:
                        direct_api.start()
                        print_log("INFO", "WiFi, CSI, and Direct link recovered")
                    # Arm the next stall deadline only after every blocking
                    # recovery step has completed.
                    last_csi_frame_time = time.ticks_ms()
                    latest_loop_duration_us = time.ticks_diff(
                        time.ticks_us(),
                        recovery_start_us,
                    )
                    performance_diagnostics.record_loop_duration(
                        latest_loop_duration_us,
                        weight=1,
                    )
                    recovery_recorded = True
                if measure_loop and not recovery_recorded:
                    latest_loop_duration_us = time.ticks_diff(time.ticks_us(), loop_start)
                    performance_diagnostics.record_loop_duration(
                        latest_loop_duration_us,
                        weight=8,
                    )

                time.sleep_us(100)

    except KeyboardInterrupt:
        print_log("INFO", "Stopping...")

    finally:
        print_log("INFO", "Cleaning up...")
        direct_api.stop()
        if traffic_gen.is_running():
            traffic_gen.stop()
        close_detector = getattr(detector, 'close', None)
        if callable(close_detector):
            close_detector()
        cleanup_wifi(wlan)

if __name__ == '__main__':
    main()
