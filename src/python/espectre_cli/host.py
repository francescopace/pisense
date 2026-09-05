# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - CLI Host

Host-side ESPectre tools.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

import inspect
import signal
import socket
import sys
import time
from urllib.parse import urlsplit, urlunsplit

from .common import Fore, Style, cli_command, print_box_banner
from .device_discovery import (
    COLLECT_DISCOVERY_QUIET_WINDOW_S,
    DeviceDiscoveryError,
    DiscoveredDevice,
    ESPECTRE_DIRECT_PORT,
    choose_device_interactively,
    discover_devices,
)


def _format_expected_device_id(device_id: int | None) -> str:
    if device_id is None:
        return "unknown"
    return f"{int(device_id):016x}"


def _discover_collect_devices_or_exit(frontend: str | None = None) -> list[DiscoveredDevice]:
    try:
        records = discover_devices(
            frontend=frontend,
            quiet_window_s=COLLECT_DISCOVERY_QUIET_WINDOW_S,
        )
    except DeviceDiscoveryError as exc:
        print(f"{Fore.RED}❌ {exc}{Style.RESET_ALL}")
        raise SystemExit(1)
    return [record for record in records if "csi" in record.capabilities]


def _resolve_collect_target_via_discovery(args) -> None:
    target = str(getattr(args, "target", "") or "").strip()
    frontend = getattr(args, "frontend", None)
    normalized_id = target.lower().removeprefix("0x")
    target_is_device_id = len(normalized_id) == 16 and all(char in "0123456789abcdef" for char in normalized_id)
    if target and not target_is_device_id:
        endpoint = target if "://" in target else f"http://{target}"
        parsed = urlsplit(endpoint)
        if parsed.scheme != "http" or not parsed.hostname:
            print(f"{Fore.RED}❌ Invalid Direct target: {target}{Style.RESET_ALL}")
            raise SystemExit(1)
        try:
            resolved_ip = socket.gethostbyname(parsed.hostname)
        except OSError as exc:
            print(f"{Fore.RED}❌ Cannot resolve Direct target {parsed.hostname}: {exc}{Style.RESET_ALL}")
            raise SystemExit(1)
        discovered_match = None
        if parsed.port is None:
            try:
                candidates = discover_devices(
                    frontend=frontend,
                    quiet_window_s=COLLECT_DISCOVERY_QUIET_WINDOW_S,
                )
            except DeviceDiscoveryError:
                candidates = []
            raw_candidates = [
                record for record in candidates
                if "csi" in record.capabilities and record.ip_address == resolved_ip
            ]
            if len(raw_candidates) == 1:
                discovered_match = raw_candidates[0]
        if discovered_match is not None:
            args.direct_endpoint = discovered_match.endpoint
            args.traffic_target = discovered_match.ip_address
            args.expected_discovery_device_id = discovered_match.device_id
            args.expected_discovery_device_id_text = discovered_match.device_id_text
            args.target_frontend = discovered_match.frontend
            return
        port = parsed.port or ESPECTRE_DIRECT_PORT
        authority = parsed.hostname if port == 80 else f"{parsed.hostname}:{port}"
        args.direct_endpoint = urlunsplit(("http", authority, "/espectre/v1", "", ""))
        args.traffic_target = resolved_ip
        args.expected_discovery_device_id = None
        args.target_frontend = frontend or "unknown"
        return

    records = _discover_collect_devices_or_exit(frontend)
    if target_is_device_id:
        records = [record for record in records if record.device_id_text == normalized_id]
    frontend_label = frontend.capitalize() if frontend else "raw-capable Direct"
    if not records:
        print(f"{Fore.RED}❌ No {frontend_label} devices discovered via mDNS.{Style.RESET_ALL}")
        print(
            f"{Fore.YELLOW}Use --target <ip|hostname|device-id|endpoint> for deterministic collection, "
            f"or verify that the firmware is online on the same LAN.{Style.RESET_ALL}"
        )
        raise SystemExit(1)

    if len(records) == 1:
        selected = records[0]
        print(
            f"{Fore.CYAN}Auto-selected {frontend_label}:{Style.RESET_ALL} "
            f"{selected.device_id_text} {selected.ip_address}:{selected.target_port}"
        )
    else:
        try:
            selected = choose_device_interactively(records, frontend_label=frontend_label)
        except KeyboardInterrupt:
            print(f"\n{Fore.YELLOW}Discovery selection cancelled{Style.RESET_ALL}")
            raise SystemExit(1)
        print(
            f"{Fore.CYAN}Selected {frontend_label}:{Style.RESET_ALL} "
            f"{selected.device_id_text} {selected.ip_address}:{selected.target_port}"
        )

    args.target = selected.ip_address
    args.direct_endpoint = selected.endpoint
    args.traffic_target = selected.ip_address
    args.target_frontend = selected.frontend
    args.expected_discovery_device_id = selected.device_id
    args.expected_discovery_device_id_text = selected.device_id_text


def _wait_before_collection(delay_seconds: float) -> None:
    """Wait before starting collection so the operator can leave the room."""
    if delay_seconds <= 0:
        return

    print(f"  {Fore.YELLOW}Starting collection in {delay_seconds:.1f}s...{Style.RESET_ALL}")
    remaining = delay_seconds
    while remaining > 0:
        sleep_for = min(1.0, remaining)
        print(f"  {Fore.YELLOW}  {remaining:.1f}s remaining{Style.RESET_ALL}")
        time.sleep(sleep_for)
        remaining = max(0.0, remaining - sleep_for)
    print(f"  {Fore.GREEN}  Starting now.{Style.RESET_ALL}")
    print()


def _prepare_raw_http_collection(args, direct_client_cls, receiver_cls, generator_cls):
    """Persist external traffic mode and construct the unpaced HTTP data plane."""
    direct_endpoint = str(args.direct_endpoint)
    traffic_target = str(args.traffic_target)
    requested_pps = float(args.pps)
    if requested_pps <= 0:
        raise ValueError(f"external traffic rate must be > 0 pps, got {requested_pps:g}")
    with direct_client_cls(direct_endpoint) as control:
        capabilities = control.request("get", "capabilities")
        raw_capability = capabilities.get("csi", {})
        if not isinstance(raw_capability, dict) or raw_capability.get("protocol_version") != 1:
            raise RuntimeError("target does not advertise raw HTTP v1")
        if raw_capability.get("marker") != generator_cls.TRAFFIC_MARKER:
            raise RuntimeError("target does not advertise the canonical external traffic marker")
        control.request("patch", "sensing", {"csi_traffic_mode": "external"})
        runtime_config = control.request("get", "sensing")
    if not isinstance(runtime_config, dict) or runtime_config.get("csi_traffic_mode") != "external":
        raise RuntimeError("device did not persist external CSI traffic mode")
    traffic_port = int(raw_capability.get("traffic_udp_port", runtime_config.get("csi_traffic_udp_port", 5555)))
    receiver = receiver_cls(direct_endpoint, buffer_size=4000, derive_complex=False)
    receiver.requested_pps = requested_pps
    generator = generator_cls(
        [traffic_target],
        port=traffic_port,
        rate_pps=requested_pps,
        source_ip=getattr(args, "source_ip", None),
    )
    return receiver, generator, traffic_port


def _start_raw_http_collection(receiver, traffic_generator) -> None:
    """Start external traffic before opening the automatic CSI stream."""
    try:
        receiver.start_session()
        traffic_generator.start()
        receiver.bind_stream()
    except Exception:
        traffic_generator.stop()
        receiver.stop()
        raise


def _post_collect_quality_issue_sort_key(result) -> tuple[int, str]:
    """Prioritize occupancy, then continuity gaps, in the post-collect summary."""
    priorities = {
        "temporal_occupancy": 0,
        "stream_seq_max_gap": 1,
        "inter_packet_gap": 2,
        "stream_seq_gaps": 3,
    }
    return priorities.get(getattr(result, "name", ""), 99), str(getattr(result, "name", ""))


def _run_post_collect_quality_checks(saved_paths) -> bool:
    """Run canonical quality checks and report whether every file passed."""
    try:
        from tools.validate_dataset_quality import validate_capture_file
    except ImportError as exc:
        print(
            f"  {Fore.YELLOW}⚠️ Post-collect quality checks unavailable: {exc}{Style.RESET_ALL}"
        )
        return True

    print(f"  {Fore.CYAN}Post-collect quality:{Style.RESET_ALL}")
    all_passed = True
    for filepath in saved_paths:
        try:
            file_results = validate_capture_file(
                filepath,
                include_packet_rate=False,
            )

            counts = {
                "PASS": sum(1 for result in file_results if result.status == "PASS"),
                "WARN": sum(1 for result in file_results if result.status == "WARN"),
                "FAIL": sum(1 for result in file_results if result.status == "FAIL"),
            }
            issues = sorted(
                (result for result in file_results if result.status in ("WARN", "FAIL")),
                key=_post_collect_quality_issue_sort_key,
            )
        except Exception as exc:
            print(
                f"    {Fore.YELLOW}⚠️ {filepath.name}: quality checks skipped ({exc}){Style.RESET_ALL}"
            )
            continue

        if counts["FAIL"]:
            all_passed = False

        if not issues:
            print(
                f"    {Fore.GREEN}✅ {filepath.name}: quality checks all pass "
                f"({counts['PASS']} checks){Style.RESET_ALL}"
            )
            continue

        print(
            f"    {Fore.YELLOW}⚠️ {filepath.name}: "
            f"{counts['WARN']} warn, {counts['FAIL']} fail{Style.RESET_ALL}"
        )
        for result in issues:
            color = Fore.RED if result.status == "FAIL" else Fore.YELLOW
            print(
                f"      {color}{result.status}{Style.RESET_ALL} "
                f"{result.name}: {result.message}"
            )
    return all_passed


def _print_dataset_catalog_stats(stats) -> None:
    environments = list(stats.get("environments", []))
    chips = list(stats.get("chips", []))
    print()
    print_box_banner("Dataset Statistics")
    print()
    if not environments:
        print(f"  {Fore.YELLOW}No samples collected yet.{Style.RESET_ALL}")
        print()
        print(f"  {Fore.CYAN}To collect data:{Style.RESET_ALL}")
        print("    1. Run a raw-capable ESPectre firmware on the device")
        print(f"    2. Collect samples: {cli_command('collect', '--label', 'wave', '--duration', '45', '--target', '192.168.1.50')}")
        print()
        return

    label_width = max(
        len("Label"),
        max(
            (len(str(row.get("label", ""))) for environment in environments for row in environment.get("rows", [])),
            default=0,
        ),
    )
    chip_widths = {
        chip: max(len(str(chip)), len("0"))
        for chip in chips
    }
    total_width = max(len("Total"), len(str(stats.get("total_samples", 0))))

    for environment in environments:
        rows = list(environment.get("rows", []))
        print(f"  {Fore.CYAN}Environment:{Style.RESET_ALL} {environment.get('environment', 'unknown')}")
        header = (
            f"  {'Label':<{label_width}} "
            + " ".join(f"{chip:>{chip_widths[chip]}}" for chip in chips)
            + f" {'Total':>{total_width}}"
        )
        print(header)
        print(f"  {'-' * (len(header) - 2)}")
        for row in rows:
            counts = row.get("counts", {})
            count_cells = " ".join(
                f"{int(counts.get(chip, 0)):>{chip_widths[chip]}}"
                for chip in chips
            )
            print(
                f"  {str(row.get('label', '')):<{label_width}} "
                f"{count_cells} {int(row.get('total', 0)):>{total_width}}"
            )
        total_counts = {
            chip: sum(int(row.get("counts", {}).get(chip, 0)) for row in rows)
            for chip in chips
        }
        total_cells = " ".join(
            f"{int(total_counts.get(chip, 0)):>{chip_widths[chip]}}"
            for chip in chips
        )
        print(f"  {'-' * (len(header) - 2)}")
        print(
            f"  {Fore.GREEN}{'Total':<{label_width}} "
            f"{total_cells} {int(environment.get('total_samples', 0)):>{total_width}}{Style.RESET_ALL}"
        )
        print()

    print(f"  {Fore.GREEN}Grand total:{Style.RESET_ALL} {int(stats.get('total_samples', 0))}")
    print()


def _show_dataset_info() -> None:
    """Print dataset statistics from dataset_info.json."""
    try:
        from tools.lib.dataset_metadata import get_dataset_catalog_stats
    except ImportError as e:
        print(f"{Fore.RED}❌ Failed to import dataset metadata helpers: {e}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Make sure the tools library package is available{Style.RESET_ALL}")
        raise SystemExit(1)
    _print_dataset_catalog_stats(get_dataset_catalog_stats())


def collect_csi_data(args) -> None:
    """Run the unified host-side collect command."""
    ready_stable_seconds = float(getattr(args, "ready_stable_seconds", 3.0))
    if ready_stable_seconds < 0:
        print(f"{Fore.RED}❌ Ready gate seconds must be >= 0{Style.RESET_ALL}")
        raise SystemExit(1)
    args.ready_stable_seconds = ready_stable_seconds
    if getattr(args, "info", False):
        _show_dataset_info()
        return
    if getattr(args, "label", None) is not None:
        try:
            from tools.lib.dataset_metadata import validate_dataset_label

            args.label = validate_dataset_label(args.label)
        except (ImportError, ValueError) as exc:
            print(f"{Fore.RED}❌ Invalid dataset label: {exc}{Style.RESET_ALL}")
            raise SystemExit(1)
    _resolve_collect_target_via_discovery(args)
    _run_live_collect(args)


def _run_live_collect(args) -> None:
    """Run the host-side live collect pipeline."""
    from tools.lib.detector_loader import load_detector_class
    from tools.lib.high_accuracy_detector import HIGH_ACCURACY_DEFAULT_THRESHOLD
    from tools.lib.runtime_policy import (
        PacketTimingTracker,
        RuntimeMotionPolicy,
        duration_packet_count,
        nominal_packet_interval_us,
    )
    from tools.lib.temporal_csi_sampler import (
        minimum_valid_slots,
        temporal_window_slots,
    )

    try:
        from tools.lib.csi_io import (
            CSICollector,
            DirectRawCSIReceiver,
        )
        from tools.espectre_traffic_generator import ExternalTrafficGenerator
        from .device_transport import DirectClient
        from tools.lib.temporal_replay import TemporalReplayController
        import config
        from console_output import format_calibration_status_line, format_detection_publish_line
        from detector_interface import (
            detector_needs_startup_calibration,
            get_detector_label,
            normalize_detector_algorithm,
            supported_detector_algorithms,
        )
        from runtime_diagnostics import RuntimeDiagnosticsSampler, empty_diagnostics_sample
        from threshold import (
            StartupThresholdCalibrator,
            get_detector_auto_factor,
            get_detector_startup_gate,
        )
    except ImportError:
        try:
            from tools.lib.csi_io import (
                CSICollector,
                DirectRawCSIReceiver,
            )
            from tools.espectre_traffic_generator import ExternalTrafficGenerator
            from .device_transport import DirectClient
            from tools.lib.temporal_replay import TemporalReplayController
            import src.config as config
            from src.console_output import format_calibration_status_line, format_detection_publish_line
            from src.detector_interface import (
                detector_needs_startup_calibration,
                get_detector_label,
                normalize_detector_algorithm,
                supported_detector_algorithms,
            )
            from src.runtime_diagnostics import RuntimeDiagnosticsSampler, empty_diagnostics_sample
            from src.threshold import (
                StartupThresholdCalibrator,
                get_detector_auto_factor,
                get_detector_startup_gate,
            )
        except ImportError as e:
            print(f"{Fore.RED}❌ Failed to import live collect modules: {e}{Style.RESET_ALL}")
            raise SystemExit(1)

    supported_detectors = supported_detector_algorithms()
    detector_kinds = list(dict.fromkeys(
        normalize_detector_algorithm(kind.strip().lower())
        for kind in str(getattr(args, "detector", "lightweight")).split(",")
        if kind.strip()
    ))
    if not detector_kinds:
        detector_kinds = ["lightweight"]
    unsupported = [kind for kind in detector_kinds if kind not in supported_detectors]
    if unsupported:
        print(f"{Fore.RED}❌ Unsupported detector(s): {', '.join(unsupported)}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Supported detectors: {', '.join(supported_detectors)}{Style.RESET_ALL}")
        raise SystemExit(1)

    calibrated_kinds = [kind for kind in detector_kinds if detector_needs_startup_calibration(kind)]
    detector_tag_width = max(len(kind) for kind in detector_kinds)

    label = getattr(args, "label", None)
    live_duration = getattr(args, "duration", None)
    save_enabled = bool(label)
    ready_stable_seconds = float(getattr(args, "ready_stable_seconds", 3.0))
    expected_discovery_device_id = getattr(args, "expected_discovery_device_id", None)
    start_delay = float(getattr(args, "start_delay", 0.0) or 0.0)

    if live_duration is not None and live_duration <= 0:
        print(f"{Fore.RED}❌ Duration must be > 0 seconds{Style.RESET_ALL}")
        raise SystemExit(1)
    if start_delay < 0:
        print(f"{Fore.RED}❌ Start delay must be >= 0 seconds{Style.RESET_ALL}")
        raise SystemExit(1)
    if start_delay > 0 and live_duration is None:
        print(f"{Fore.RED}❌ Start delay requires --duration{Style.RESET_ALL}")
        raise SystemExit(1)
    if not getattr(args, "target", None):
        print(f"{Fore.RED}❌ Target required. Use --target <ip|hostname|device-id> or discovery.{Style.RESET_ALL}")
        raise SystemExit(1)

    requested_pps = float(args.pps)
    configured_window_ms = max(
        1,
        int(getattr(config, "SEGMENTATION_WINDOW_SIZE_MS", 1000)),
    )
    target_pps = max(1, int(round(requested_pps)))
    initial_nominal_interval_us = nominal_packet_interval_us(target_pps)
    initial_window_packets = temporal_window_slots(target_pps, configured_window_ms)
    initial_minimum_valid_slots = minimum_valid_slots(initial_window_packets)
    effective_evaluation_interval_ms = max(
        1,
        int(getattr(config, "EVALUATION_INTERVAL_MS", 250)),
    )
    calibration_duration_ms = max(
        configured_window_ms,
        int(getattr(config, "CALIBRATION_DURATION_MS", configured_window_ms * 10)),
    )
    status_render_interval_seconds = 1.0

    def get_initial_threshold(kind):
        if kind == "high_accuracy":
            return HIGH_ACCURACY_DEFAULT_THRESHOLD
        return load_detector_class(kind).BASE_THRESHOLD

    def get_detector_threshold(detector, fallback=1.0):
        if hasattr(detector, "get_threshold"):
            return detector.get_threshold()
        return fallback

    def extract_motion_metric(metrics):
        return metrics.get("motion_metric", metrics.get("probability", 0.0))

    def supports_inline_terminal(stream=None):
        target_stream = sys.stdout if stream is None else stream
        isatty = getattr(target_stream, "isatty", None)
        return bool(callable(isatty) and isatty())

    def emit_status_block(summary_line, detail_lines, *, previous_line_count=0, inline=None):
        target_stream = sys.stdout
        use_inline = supports_inline_terminal(target_stream) if inline is None else inline
        lines = [summary_line, *detail_lines]

        if not use_inline:
            for line in lines:
                target_stream.write(f"{line}\n")
            target_stream.flush()
            return len(lines)

        if previous_line_count > 0:
            target_stream.write(f"\x1b[{previous_line_count}F")

        total_lines = max(previous_line_count, len(lines))
        for idx in range(total_lines):
            target_stream.write("\x1b[2K")
            if idx < len(lines):
                target_stream.write(lines[idx])
            target_stream.write("\n")

        target_stream.flush()
        return len(lines)

    def clear_status_block():
        line_count = state["summary_line_count"]
        if line_count <= 0:
            return

        target_stream = sys.stdout
        if not state["summary_use_inline"]:
            state["summary_line_count"] = 0
            return

        target_stream.write(f"\x1b[{line_count}F")
        for _ in range(line_count):
            target_stream.write("\x1b[2K\n")
        target_stream.write(f"\x1b[{line_count}F")
        target_stream.flush()
        state["summary_line_count"] = 0

    def format_device_label(device_state):
        source_ip = str(device_state.get("source_ip") or "?")
        chip_label = str(device_state.get("chip") or "unknown").upper()
        return f"ip={source_ip} chip={chip_label}"

    def format_backpressure_text(device_state):
        total = device_state.get("transport_backpressure_total")
        if total is None:
            return " | bp:--"
        recent_delta = int(device_state.get("transport_backpressure_last_delta", 0) or 0)
        if recent_delta > 0:
            return f" | bp:active(+{recent_delta})"
        return " | bp:no"

    def format_transport_text(device_state):
        packet_count = int(device_state.get("packet_count", 0) or 0)
        dropped_count = int(device_state.get("dropped_count", 0) or 0)
        total_expected = max(packet_count + dropped_count, 1)
        drop_rate = (float(dropped_count) / float(total_expected)) * 100.0
        return f" | http:{int(device_state.get('pps', 0) or 0)} drop:{drop_rate:.1f}%"

    def build_device_diagnostics_snapshot(device_state):
        sampler = device_state["temporal_controller"].sampler
        return {
            "traffic_packets_total": int(device_state.get("fresh_record_total", 0) or 0),
            "csi_callbacks_total": int(device_state.get("packet_count", 0) or 0),
            "csi_accepted_total": int(device_state.get("packet_count", 0) or 0),
            "csi_admitted_total": int(getattr(sampler, "accepted_packets", 0) or 0),
            "csi_filtered_total": int(device_state.get("filtered_count", 0) or 0),
            "csi_missing_slots_total": int(getattr(sampler, "missing_slots", 0) or 0),
            "csi_excess_total": int(getattr(sampler, "excess_packets", 0) or 0),
            "csi_stale_total": int(getattr(sampler, "stale_packets", 0) or 0),
            "csi_out_of_order_total": int(getattr(sampler, "out_of_order_packets", 0) or 0),
            "csi_occupancy_slots": int(getattr(sampler, "occupancy_slots", 0) or 0),
            "csi_window_slots": int(getattr(sampler, "window_slots", 0) or 0),
            "wifi_channel": int(device_state.get("channel") or 0),
            "wifi_rssi_dbm": device_state.get("rssi_dbm"),
        }

    def sample_device_diagnostics(device_state, now):
        diagnostics = device_state["diagnostics_sampler"].sample(
            build_device_diagnostics_snapshot(device_state),
            int(now * 1000.0),
        )
        device_state["latest_diagnostics"] = diagnostics
        return diagnostics

    def get_packet_device_id(pkt):
        device_id = getattr(pkt, "device_id", None)
        if device_id is None:
            return None
        return int(device_id)

    def create_detector(kind, threshold, window_packets):
        return load_detector_class(kind)(
            window_size=window_packets,
            threshold=threshold,
            enable_lowpass=config.ENABLE_LOWPASS_FILTER,
            lowpass_cutoff=config.LOWPASS_CUTOFF,
            enable_hampel=config.ENABLE_HAMPEL_FILTER,
            hampel_window=config.HAMPEL_WINDOW,
            hampel_threshold=config.HAMPEL_THRESHOLD,
        )

    def start_startup_session(detector):
        if hasattr(detector, "on_startup_calibration_begin"):
            detector.on_startup_calibration_begin()

    def build_calibration_tracker(detector, target_packets):
        return StartupThresholdCalibrator(
            target_packets,
            auto_factor=get_detector_auto_factor(detector),
            gate_enabled=get_detector_startup_gate(detector),
        )

    def build_timing_tracker(nominal_interval_us):
        return PacketTimingTracker(nominal_interval_us)

    def policy_note_packet(policy, elapsed_us):
        policy.note_packet(elapsed_us=elapsed_us)

    def policy_should_evaluate(policy):
        return bool(policy.should_evaluate())

    def detector_process_packet(detector, csi_data, rssi_dbm, timestamp_us=None):
        detector.process_packet(
            csi_data,
            subcarriers,
            rssi_dbm=rssi_dbm,
            timestamp_us=timestamp_us,
        )

    def policy_equivalent_packets(policy, fallback_packets, nominal_interval_us):
        getter = getattr(policy, "equivalent_packets_since_evaluation", None)
        if callable(getter):
            return int(getter(nominal_interval_us))
        return max(1, int(fallback_packets))

    def restart_calibration_slot(slot, device_state):
        calibration_detector = slot.get("calibration_detector")
        calibration_tracker = slot.get("calibration_tracker")
        if calibration_detector is None or calibration_tracker is None:
            return
        if hasattr(calibration_detector, "reset"):
            calibration_detector.reset()
        start_startup_session(calibration_detector)
        slot["calibration_tracker"] = build_calibration_tracker(
            calibration_detector,
            device_state["calibration_target_packets"],
        )
        slot["calibration_policy"] = RuntimeMotionPolicy(
            evaluation_interval_ms=effective_evaluation_interval_ms,
            motion_on_hits=1,
            motion_off_hits=1,
        )
        slot["calibration_packets_since_evaluation"] = 0
        slot["motion_metric"] = 0.0
        slot["metric_threshold"] = get_detector_threshold(calibration_detector, slot["metric_threshold"])
        slot["status"] = "WAITING"

    def reset_runtime_slot(slot):
        detector = slot["detector"]
        if hasattr(detector, "reset"):
            detector.reset()
        runtime_policy = slot["runtime_policy"]
        if hasattr(runtime_policy, "reset"):
            runtime_policy.reset()
        slot["motion_metric"] = 0.0
        slot["effective_state"] = 0
        slot["status"] = "WARMUP"
        slot["ready_below_since"] = None
        slot["ready_stable_for"] = 0.0

    def packet_timestamp_us(pkt):
        for field in ("wifi_rx_ts_us", "device_ticks_us"):
            value = getattr(pkt, field, None)
            if value is not None:
                return int(value)
        return None

    def reset_temporal_device(device_state):
        if state["calibration_active"]:
            for slot in device_state["slots"]:
                if not slot.get("calibration_done"):
                    restart_calibration_slot(slot, device_state)
        else:
            for slot in device_state["slots"]:
                reset_runtime_slot(slot)

    def build_detector_slot(kind, window_packets, calibration_target_packets):
        needs_calibration = kind in calibrated_kinds
        slot_initial_threshold = get_initial_threshold(kind)
        detector = create_detector(kind, slot_initial_threshold, window_packets)
        if hasattr(detector, "set_minimum_valid_samples"):
            detector.set_minimum_valid_samples(initial_minimum_valid_slots)
        start_startup_session(detector)
        runtime_policy = RuntimeMotionPolicy(
            evaluation_interval_ms=effective_evaluation_interval_ms,
            motion_on_hits=config.MOTION_ON_HITS,
            motion_off_hits=config.MOTION_OFF_HITS,
        )
        calibration_detector = (
            create_detector(kind, 1.0, window_packets)
            if needs_calibration
            else None
        )
        if calibration_detector is not None:
            if hasattr(calibration_detector, "set_minimum_valid_samples"):
                calibration_detector.set_minimum_valid_samples(
                    initial_minimum_valid_slots
                )
            start_startup_session(calibration_detector)
        return {
            "kind": kind,
            "detector": detector,
            "runtime_policy": runtime_policy,
            "motion_metric": 0.0,
            "metric_threshold": get_detector_threshold(detector, slot_initial_threshold),
            "effective_state": 0,
            "status": "WARMUP" if not needs_calibration else "WAITING",
            "calibration_detector": calibration_detector,
            "calibration_tracker": (
                build_calibration_tracker(calibration_detector, calibration_target_packets)
                if calibration_detector is not None
                else None
            ),
            "calibration_policy": (
                RuntimeMotionPolicy(
                    evaluation_interval_ms=effective_evaluation_interval_ms,
                    motion_on_hits=1,
                    motion_off_hits=1,
                )
                if calibration_detector is not None
                else None
            ),
            "calibration_packets_since_evaluation": 0,
            "calibration_done": not needs_calibration,
            "calibration_success": not needs_calibration,
            "calibration_threshold_source": None if needs_calibration else "fixed",
            "ready_below_since": None,
            "ready_stable_for": 0.0,
        }

    def build_device_state(device_id, now):
        calibration_target_packets = duration_packet_count(
            calibration_duration_ms,
            initial_nominal_interval_us
        )
        temporal_controller = TemporalReplayController(
            target_pps,
            configured_window_ms,
        )
        device_state = {
            "device_id": device_id,
            "source_ip": "?",
            "chip": "unknown",
            "channel": None,
            "rssi_dbm": None,
            "filtered_count": 0,
            "label": "",
            "packet_count": 0,
            "dropped_count": 0,
            "last_seq_num": None,
            "pps": 0,
            "pps_window_started_at": None,
            "pps_window_packets": 0,
            "last_status_render_at": None,
            "transport_backpressure_total": None,
            "transport_backpressure_last_delta": 0,
            "fresh_record_total": None,
            "nominal_interval_us": initial_nominal_interval_us,
            "window_packets": initial_window_packets,
            "calibration_target_packets": calibration_target_packets,
            "timing_tracker": build_timing_tracker(initial_nominal_interval_us),
            "temporal_controller": temporal_controller,
            "diagnostics_sampler": RuntimeDiagnosticsSampler(),
            "latest_diagnostics": empty_diagnostics_sample(),
            "slots": [
                build_detector_slot(
                    kind,
                    initial_window_packets,
                    calibration_target_packets,
                )
                for kind in detector_kinds
            ],
        }
        device_state["label"] = format_device_label(device_state)
        device_state["diagnostics_sampler"].reset(
            build_device_diagnostics_snapshot(device_state),
            int(now * 1000.0),
        )
        device_state["latest_diagnostics"] = empty_diagnostics_sample()
        return device_state

    def get_device_state(pkt, now):
        device_id = get_packet_device_id(pkt)
        device_state = state["devices"].get(device_id)
        if device_state is None:
            device_state = build_device_state(device_id, now)
            state["devices"][device_id] = device_state
        source_ip = getattr(pkt, "source_ip", None)
        if source_ip:
            device_state["source_ip"] = str(source_ip)
        chip = getattr(pkt, "chip", None)
        if chip not in (None, "", "unknown"):
            device_state["chip"] = str(chip).upper()
        channel = getattr(pkt, "channel", None)
        if channel is not None:
            device_state["channel"] = int(channel)
        rssi_dbm = getattr(pkt, "rssi_dbm", None)
        if rssi_dbm is not None:
            device_state["rssi_dbm"] = int(rssi_dbm)
        transport_backpressure = getattr(pkt, "transport_backpressure_total", None)
        if transport_backpressure is not None:
            previous = device_state.get("transport_backpressure_total")
            device_state["transport_backpressure_total"] = int(transport_backpressure)
            if previous is not None:
                device_state["transport_backpressure_last_delta"] = max(0, int(transport_backpressure) - int(previous))
        fresh_record_total = getattr(pkt, "fresh_record_total", None)
        if fresh_record_total is not None:
            device_state["fresh_record_total"] = int(fresh_record_total)
        device_state["label"] = format_device_label(device_state)
        return device_state

    def update_device_pps(device_state, now):
        if device_state["pps_window_started_at"] is None:
            device_state["pps_window_started_at"] = now
        device_state["pps_window_packets"] += 1
        elapsed = now - device_state["pps_window_started_at"]
        if elapsed >= 1.0:
            device_state["pps"] = int(device_state["pps_window_packets"] / elapsed) if elapsed > 0 else 0
            device_state["pps_window_started_at"] = now
            device_state["pps_window_packets"] = 0

    def update_ready_gate_state(device_state, now):
        for slot in device_state["slots"]:
            if not save_enabled or state["calibration_active"]:
                slot["ready_below_since"] = None
                slot["ready_stable_for"] = 0.0
                continue
            detector = slot["detector"]
            threshold = float(slot.get("metric_threshold", 0.0) or 0.0)
            if threshold <= 0 or not detector.is_ready():
                slot["ready_below_since"] = None
                slot["ready_stable_for"] = 0.0
                continue
            if float(slot["motion_metric"]) <= threshold:
                if slot["ready_below_since"] is None:
                    slot["ready_below_since"] = now
                slot["ready_stable_for"] = max(0.0, now - slot["ready_below_since"])
            else:
                slot["ready_below_since"] = None
                slot["ready_stable_for"] = 0.0

    def get_slot_gate_label(slot):
        if not save_enabled:
            return None
        if state["calibration_active"] and slot["calibration_tracker"] is not None:
            if slot["calibration_done"]:
                return "READY"
            if slot["calibration_tracker"].packet_count > 0:
                return "CALIBRATING"
            return "WAITING"
        detector = slot["detector"]
        if not detector.is_ready():
            return "WARMUP"
        if float(slot["motion_metric"]) > float(slot["metric_threshold"]):
            return "UNSTABLE"
        if float(slot["ready_stable_for"]) >= ready_stable_seconds:
            return "READY"
        return "STABLE"

    def summarize_ready_gate():
        observed_count = len(state["devices"])
        required_count = max(1, len(targets))
        if observed_count < required_count:
            return {
                "ready": False,
                "status": f"DEVICES {observed_count}/{required_count}",
                "stable_elapsed": 0.0,
            }
        relevant_states = list(state["devices"].values())
        warm_count = sum(
            1
            for device_state in relevant_states
            if all(slot["detector"].is_ready() for slot in device_state["slots"])
        )
        if warm_count < observed_count:
            return {
                "ready": False,
                "status": f"WARMUP {warm_count}/{required_count}",
                "stable_elapsed": 0.0,
            }
        stable_count = sum(
            1
            for device_state in relevant_states
            if all(
                float(slot["motion_metric"]) <= float(slot["metric_threshold"])
                for slot in device_state["slots"]
            )
        )
        if stable_count < observed_count:
            return {
                "ready": False,
                "status": f"UNSTABLE {stable_count}/{required_count}",
                "stable_elapsed": 0.0,
            }
        stable_elapsed = min(
            min(float(slot["ready_stable_for"]) for slot in device_state["slots"])
            for device_state in relevant_states
        )
        if stable_elapsed >= ready_stable_seconds:
            return {
                "ready": True,
                "status": f"READY {observed_count}/{required_count}",
                "stable_elapsed": ready_stable_seconds,
            }
        return {
            "ready": False,
            "status": f"STABLE {observed_count}/{required_count}",
            "stable_elapsed": stable_elapsed,
        }

    def get_slot_status(slot):
        if state["calibration_active"] and slot["calibration_tracker"] is not None:
            if slot["calibration_done"]:
                return "READY"
            if slot["calibration_tracker"].packet_count > 0:
                return "CALIBRATING"
            return "WAITING"
        detector = slot["detector"]
        if not detector.is_ready():
            return "WARMUP"
        return "MOTION" if int(slot["effective_state"]) == 1 else "IDLE"

    def finalize_slot_calibration(slot):
        detector = slot["detector"]
        runtime_policy = slot["runtime_policy"]
        calibration_tracker = slot["calibration_tracker"]
        slot["calibration_done"] = True
        if hasattr(runtime_policy, "reset"):
            runtime_policy.reset()
        if hasattr(detector, "reset"):
            detector.reset()

        if calibration_tracker is not None and calibration_tracker.is_successful():
            startup_threshold, threshold_formula = calibration_tracker.calculate_threshold()
            if hasattr(detector, "set_adaptive_threshold"):
                detector.set_adaptive_threshold(startup_threshold)
            elif hasattr(detector, "set_threshold"):
                detector.set_threshold(startup_threshold)
            slot["calibration_threshold_source"] = f"automatic ({threshold_formula})"
            slot["calibration_success"] = True
        else:
            slot["calibration_success"] = False
            slot["calibration_threshold_source"] = "failed"
        slot["metric_threshold"] = get_detector_threshold(detector, slot["metric_threshold"])

        slot["motion_metric"] = 0.0
        slot["effective_state"] = 0
        slot["status"] = "IDLE"
        slot["ready_below_since"] = None
        slot["ready_stable_for"] = 0.0

    def process_calibration_packet(device_state, pkt, timing):
        finalized_any = False
        evaluated_any = False
        for slot in device_state["slots"]:
            calibration_detector = slot["calibration_detector"]
            calibration_tracker = slot["calibration_tracker"]
            calibration_policy = slot.get("calibration_policy")
            if calibration_detector is None or calibration_tracker is None or slot["calibration_done"]:
                continue

            detector_process_packet(
                calibration_detector,
                pkt.iq_raw,
                getattr(pkt, "rssi_dbm", None),
                packet_timestamp_us(pkt),
            )
            slot["calibration_packets_since_evaluation"] += 1
            if calibration_policy is None:
                continue
            policy_note_packet(calibration_policy, timing["coverage_us"])
            if not policy_should_evaluate(calibration_policy):
                continue
            calibration_metrics = calibration_detector.update_state()
            evaluated_any = True
            if calibration_detector.is_ready():
                calibration_tracker.observe_detector(
                    calibration_detector,
                    packet_weight=policy_equivalent_packets(
                        calibration_policy,
                        slot["calibration_packets_since_evaluation"],
                        device_state["nominal_interval_us"],
                    ),
                )
            calibration_policy.after_evaluation()
            slot["calibration_packets_since_evaluation"] = 0
            slot["motion_metric"] = extract_motion_metric(calibration_metrics)
            slot["metric_threshold"] = calibration_metrics.get("threshold", calibration_detector.get_threshold())
            slot["status"] = getattr(calibration_tracker, "get_phase_label", lambda: "CALIBRATING")()

            if calibration_tracker.is_complete():
                finalize_slot_calibration(slot)
                finalized_any = True
        return evaluated_any or finalized_any

    def process_temporal_admission(device_state, admission):
        if admission.reset_required:
            reset_temporal_device(device_state)
        if state["calibration_active"]:
            if admission.missing_slots_before:
                for slot in device_state["slots"]:
                    calibration_detector = slot.get("calibration_detector")
                    if (
                        calibration_detector is not None
                        and hasattr(calibration_detector, "advance_missing_slots")
                    ):
                        calibration_detector.advance_missing_slots(
                            admission.missing_slots_before
                        )
            return process_calibration_packet(
                device_state,
                admission.packet,
                {**admission.context, "coverage_us": admission.coverage_us},
            )

        should_render = False
        for slot in device_state["slots"]:
            detector = slot["detector"]
            runtime_policy = slot["runtime_policy"]
            if (
                admission.missing_slots_before
                and hasattr(detector, "advance_missing_slots")
            ):
                detector.advance_missing_slots(admission.missing_slots_before)
            detector_process_packet(
                detector,
                admission.packet.iq_raw,
                getattr(admission.packet, "rssi_dbm", None),
                admission.timestamp_us,
            )
            policy_note_packet(runtime_policy, admission.coverage_us)
            if not policy_should_evaluate(runtime_policy):
                continue
            metrics = detector.update_state()
            slot["motion_metric"] = extract_motion_metric(metrics)
            slot["metric_threshold"] = metrics["threshold"]

            effective_state, _ = runtime_policy.apply_state(metrics["state"])
            runtime_policy.after_evaluation()
            slot["effective_state"] = effective_state
            slot["status"] = get_slot_status(slot)
            should_render = True
        return should_render

    def is_calibration_complete():
        required_count = max(1, len(targets))
        if len(state["devices"]) < required_count:
            return False
        return all(
            slot["calibration_done"]
            for device_state in state["devices"].values()
            for slot in device_state["slots"]
        )

    def maybe_stop_live_session(now):
        start_time = state["capture_started_at"] if save_enabled else state["session_started_at"]
        if live_duration is None or start_time is None:
            return False
        if (now - start_time) < live_duration:
            return False
        if save_enabled:
            state["capture_completed"] = True
        state["running"] = False
        receiver.stop()
        return True

    def format_slot_label(device_state, slot):
        if len(detector_kinds) == 1:
            return device_state["label"]
        return f"{device_state['label']} [{slot['kind']:<{detector_tag_width}s}]"

    def render_multi_device_summary(now):
        observed_count = len(state["devices"])
        required_count = max(1, len(targets))
        detail_lines = []
        for device_id in sorted(state["devices"], key=lambda value: (value is None, value if value is not None else 0)):
            device_state = state["devices"][device_id]
            diagnostics = sample_device_diagnostics(device_state, now)
            for slot in device_state["slots"]:
                status = get_slot_status(slot)
                slot_label = format_slot_label(device_state, slot)
                if state["calibration_active"] and slot["calibration_tracker"] is not None:
                    calibration_tracker = slot["calibration_tracker"]
                    calibration_packets = calibration_tracker.packet_count
                    if slot["calibration_done"]:
                        detail_line = (
                            "    "
                            + format_detection_publish_line(
                                diagnostics=diagnostics,
                                motion_metric=slot["motion_metric"],
                                threshold=slot["metric_threshold"],
                                effective_state=slot["effective_state"],
                                device_label=slot_label,
                                filled_char="█",
                                empty_char="░",
                            )
                        )
                        detail_line += format_backpressure_text(device_state) + format_transport_text(device_state)
                        detail_lines.append(detail_line)
                    else:
                        detail_line = (
                            "    "
                            + format_calibration_status_line(
                                progress=(calibration_packets / device_state["calibration_target_packets"]),
                                motion_metric=slot["motion_metric"],
                                threshold=slot["metric_threshold"],
                                diagnostics=diagnostics,
                                effective_state_label=status,
                                device_label=slot_label,
                                filled_char="█",
                                empty_char="░",
                            )
                        )
                        detail_line += format_backpressure_text(device_state) + format_transport_text(device_state)
                        detail_lines.append(detail_line)
                else:
                    detail_line = (
                        "    "
                        + format_detection_publish_line(
                            diagnostics=diagnostics,
                            motion_metric=slot["motion_metric"],
                            threshold=slot["metric_threshold"],
                            effective_state=slot["effective_state"],
                            device_label=slot_label,
                            filled_char="█",
                            empty_char="░",
                        )
                    )
                    detail_line += format_backpressure_text(device_state) + format_transport_text(device_state)
                    if save_enabled and not state["capture_ready"]:
                        detail_line += f" | {get_slot_gate_label(slot)}"
                    detail_lines.append(detail_line)

        if state["calibration_active"]:
            summary_line = (
                f"  STATUS: CALIBRATING {observed_count}/{required_count} | "
                f"target {calibration_duration_ms} ms/device | capture {len(state['capture_packets'])}"
            )
        elif save_enabled and not state["capture_ready"]:
            ready_summary = summarize_ready_gate()
            summary_line = (
                f"  STATUS: STABILIZING {observed_count}/{required_count} | "
                f"{ready_summary['status'].lower()} | ready {ready_summary['stable_elapsed']:.1f}/{ready_stable_seconds:.1f}s "
                f"| packets {state['packet_count']} | capture {len(state['capture_packets'])}"
            )
        elif save_enabled:
            elapsed = 0.0 if state["capture_started_at"] is None else max(0.0, now - state["capture_started_at"])
            if live_duration is None:
                duration_text = "recording until Ctrl+C"
            else:
                duration_text = f"{elapsed:.1f}/{live_duration:.1f}s"
            summary_line = (
                f"  STATUS: RECORDING {observed_count}/{required_count} | "
                f"{duration_text if live_duration is None else f'elapsed {duration_text}'} | capture {len(state['capture_packets'])}"
            )
        else:
            elapsed = 0.0 if state["session_started_at"] is None else max(0.0, now - state["session_started_at"])
            if live_duration is None:
                duration_text = "collecting until Ctrl+C"
            else:
                duration_text = f"{elapsed:.1f}/{live_duration:.1f}s"
            summary_line = (
                f"  STATUS: COLLECTING {observed_count}/{required_count} | "
                f"{duration_text if live_duration is None else f'elapsed {duration_text}'} | packets {state['packet_count']}"
            )

        state["summary_line_count"] = emit_status_block(
            summary_line,
            detail_lines,
            previous_line_count=state["summary_line_count"],
            inline=state["summary_use_inline"],
        )

    direct_endpoint = str(args.direct_endpoint)
    traffic_target = str(args.traffic_target)
    targets = [traffic_target]
    subcarriers = list(config.DEFAULT_SUBCARRIERS)
    requested_pps = float(args.pps)
    try:
        receiver, traffic_generator, traffic_port = _prepare_raw_http_collection(
            args, DirectClient, DirectRawCSIReceiver, ExternalTrafficGenerator)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"{Fore.RED}❌ Cannot prepare Direct raw collection: {exc}{Style.RESET_ALL}")
        raise SystemExit(1)
    capture_writer = None
    if save_enabled:
        capture_writer = CSICollector(
            label=label,
            port=0,
            contributor=getattr(args, "contributor", None),
            description=getattr(args, "description", None),
            bind_host="127.0.0.1",
            expected_device_count=1,
            expected_source_hosts=targets,
            expected_device_id=expected_discovery_device_id,
            target_pps=int(round(requested_pps)),
        )

    state = {
        "running": True,
        "packet_count": 0,
        "capture_packets": [],
        "session_started_at": None,
        "capture_started_at": None,
        "capture_ready": (not save_enabled) or ready_stable_seconds <= 0.0,
        "capture_completed": False,
        "interrupted": False,
        "devices": {},
        "summary_line_count": 0,
        "summary_use_inline": supports_inline_terminal(),
        "calibration_active": bool(calibrated_kinds),
        "device_id_mismatch": None,
    }
    strict_source_filter = True
    allowed_source_hosts = set(targets)

    def handle_sigint(_signum, _frame):
        state["interrupted"] = True
        state["running"] = False

    def on_packet(pkt):
        if not state["running"]:
            return

        now = time.monotonic()
        if strict_source_filter:
            source_ip = getattr(pkt, "source_ip", None)
            if source_ip is None or str(source_ip) not in allowed_source_hosts:
                maybe_stop_live_session(now)
                return
        if expected_discovery_device_id is not None:
            packet_device_id = get_packet_device_id(pkt)
            if packet_device_id is None:
                state["device_id_mismatch"] = (
                    "Discovered target expected "
                    f"{_format_expected_device_id(expected_discovery_device_id)}, "
                    "but the stream packet had no device_id metadata"
                )
                state["running"] = False
                return
            if packet_device_id != int(expected_discovery_device_id):
                state["device_id_mismatch"] = (
                    "Discovered target expected "
                    f"{_format_expected_device_id(expected_discovery_device_id)}, "
                    f"but received {_format_expected_device_id(packet_device_id)} from {getattr(pkt, 'source_ip', '?')}"
                )
                state["running"] = False
                return

        state["packet_count"] += 1
        if state["session_started_at"] is None:
            state["session_started_at"] = now

        device_state = get_device_state(pkt, now)
        device_state["packet_count"] += 1
        timing = device_state["timing_tracker"].observe_packet(pkt)
        temporal_controller = device_state["temporal_controller"]
        admission = temporal_controller.admit(pkt, context=timing)
        admitted = admission is not None
        seq_num = getattr(pkt, "seq_num", None)
        if seq_num is not None:
            device_state["last_seq_num"] = int(seq_num)
        device_state["dropped_count"] += timing["missing_seq"]
        update_device_pps(device_state, now)
        if state["calibration_active"]:
            calibration_render_due = False
            if admitted:
                calibration_render_due = process_temporal_admission(
                    device_state, admission
                )
            if temporal_controller.sampler.gap_reset_required:
                reset_temporal_device(device_state)
                calibration_render_due = True
            [
                slot["calibration_tracker"]
                for slot in device_state["slots"]
                if slot["calibration_tracker"] is not None
            ]
            if is_calibration_complete():
                state["calibration_active"] = False
                for observed_device in state["devices"].values():
                    observed_device["temporal_controller"].clear_window_preserving_phase()
                render_multi_device_summary(now)
            elif calibration_render_due:
                render_multi_device_summary(now)
            if not save_enabled and maybe_stop_live_session(now):
                return
            return

        last_status_render_at = device_state["last_status_render_at"]
        if last_status_render_at is None:
            device_state["last_status_render_at"] = now
            status_render_due = False
        else:
            status_render_due = now - last_status_render_at >= status_render_interval_seconds
        should_render_summary = False
        if admitted:
            should_render_summary = process_temporal_admission(
                device_state, admission
            )
        if temporal_controller.sampler.gap_reset_required:
            reset_temporal_device(device_state)
            should_render_summary = True

        update_ready_gate_state(device_state, now)
        if status_render_due:
            device_state["last_status_render_at"] = now
            should_render_summary = True

        if save_enabled and not state["capture_ready"]:
            ready_summary = summarize_ready_gate()
            if ready_summary["ready"]:
                state["capture_ready"] = True
                state["capture_started_at"] = now
                should_render_summary = True

        if save_enabled and state["capture_ready"]:
            if state["capture_started_at"] is None:
                state["capture_started_at"] = now
            if maybe_stop_live_session(now):
                return
            state["capture_packets"].append(pkt)
        elif not save_enabled and maybe_stop_live_session(now):
            return

        if should_render_summary:
            render_multi_device_summary(now)

    def flush_temporal_devices(now):
        should_render_summary = False
        for device_state in state["devices"].values():
            admission = device_state["temporal_controller"].finish()
            if admission is None:
                continue
            should_render_summary = (
                process_temporal_admission(device_state, admission)
                or should_render_summary
            )

        if state["calibration_active"] and is_calibration_complete():
            state["calibration_active"] = False
            for device_state in state["devices"].values():
                device_state["temporal_controller"].clear_window_preserving_phase()
            should_render_summary = True
        if not state["calibration_active"]:
            for device_state in state["devices"].values():
                update_ready_gate_state(device_state, now)
        if should_render_summary:
            render_multi_device_summary(now)

    receiver.add_callback(on_packet)
    signal.signal(signal.SIGINT, handle_sigint)

    print()
    print_box_banner("Live CSI Collection")
    print()
    print(f"  {Fore.CYAN}Target:{Style.RESET_ALL}    {direct_endpoint} (HTTP Direct)")
    if expected_discovery_device_id is not None:
        print(
            f"  {Fore.CYAN}Device ID:{Style.RESET_ALL} "
            f"{_format_expected_device_id(expected_discovery_device_id)} (from mDNS)"
        )
    print(f"  {Fore.CYAN}Traffic:{Style.RESET_ALL}   {traffic_target}:{traffic_port}")
    if start_delay > 0:
        print(f"  {Fore.CYAN}Start delay:{Style.RESET_ALL} {start_delay:.1f}s")
    print(
        f"  {Fore.CYAN}Detector:{Style.RESET_ALL}  "
        f"{', '.join(get_detector_label(kind) for kind in detector_kinds)}"
    )
    if "high_accuracy" in detector_kinds:
        high_accuracy_suffix = " (high accuracy, fixed)" if len(detector_kinds) > 1 else ""
        print(
            f"  {Fore.CYAN}Threshold:{Style.RESET_ALL} "
            f"{get_initial_threshold('high_accuracy'):.2f}{high_accuracy_suffix}"
        )
    if calibrated_kinds:
        print(f"  {Fore.CYAN}Threshold:{Style.RESET_ALL} automatic (after startup calibration)")
    print(
        f"  {Fore.CYAN}Pps:{Style.RESET_ALL}       {requested_pps:g}pps external"
    )
    print(
        f"  {Fore.CYAN}Window:{Style.RESET_ALL}    {configured_window_ms} ms "
        f"({initial_window_packets} temporal slots at {target_pps:g} pps)"
    )
    print(
        f"  {Fore.CYAN}Evaluation:{Style.RESET_ALL} "
        f"{effective_evaluation_interval_ms} ms"
    )
    print(f"  {Fore.CYAN}Low-pass:{Style.RESET_ALL}  {'ON' if config.ENABLE_LOWPASS_FILTER else 'OFF'}")
    print(f"  {Fore.CYAN}Hampel:{Style.RESET_ALL}    {'ON' if config.ENABLE_HAMPEL_FILTER else 'OFF'}")
    if save_enabled:
        duration_text = "until Ctrl+C" if live_duration is None else f"{live_duration:g}s"
        print(f"  {Fore.CYAN}Save:{Style.RESET_ALL}      label={label} duration={duration_text}")
        if ready_stable_seconds <= 0.0:
            print(f"  {Fore.CYAN}Ready gate:{Style.RESET_ALL} disabled")
        else:
            print(f"  {Fore.CYAN}Ready gate:{Style.RESET_ALL} {ready_stable_seconds:.1f}s below threshold before saving")
        if getattr(args, "description", None):
            print(f"  {Fore.CYAN}Description:{Style.RESET_ALL} {args.description}")
    else:
        print(f"  {Fore.CYAN}Save:{Style.RESET_ALL}      disabled")
    print()
    print(f"  {Fore.YELLOW}The external traffic mode is persisted on the device after collection{Style.RESET_ALL}")
    if calibrated_kinds:
        print(f"  {Fore.YELLOW}Please remain still during the startup calibration phase{Style.RESET_ALL}")
    print(f"  {Fore.YELLOW}Press Ctrl+C to stop{Style.RESET_ALL}")
    print()

    try:
        _wait_before_collection(start_delay)
        _start_raw_http_collection(receiver, traffic_generator)
        supports_socket_rcvbuf_announcement = (
            "announce_socket_rcvbuf" in inspect.signature(receiver.run).parameters
        )
        while state["running"]:
            announce_socket_rcvbuf = state.get("socket_rcvbuf_reported") is not True
            run_kwargs = {"timeout": 1.0, "quiet": True}
            if supports_socket_rcvbuf_announcement:
                run_kwargs["announce_socket_rcvbuf"] = announce_socket_rcvbuf
            receiver.run(**run_kwargs)
            maybe_stop_live_session(time.monotonic())
            if announce_socket_rcvbuf and receiver.effective_socket_rcvbuf_bytes is not None:
                state["socket_rcvbuf_reported"] = True
        flush_temporal_devices(time.monotonic())
    except KeyboardInterrupt:
        state["interrupted"] = True
        now = time.monotonic()
        flush_temporal_devices(now)
        render_multi_device_summary(now)
    except Exception as e:
        print(f"\n{Fore.RED}❌ Error during live collect: {e}{Style.RESET_ALL}")
        raise SystemExit(1)
    finally:
        traffic_generator.stop()
        receiver.stop()
        if state["capture_packets"]:
            apply_final_raw_diagnostics = getattr(
                receiver, "apply_final_raw_diagnostics", None
            )
            if callable(apply_final_raw_diagnostics):
                apply_final_raw_diagnostics(state["capture_packets"][-1])
        clear_status_block()
        if state["device_id_mismatch"] is not None:
            print(f"{Fore.RED}❌ {state['device_id_mismatch']}{Style.RESET_ALL}")
            raise SystemExit(1)
        if capture_writer is not None:
            captured_packets = state["capture_packets"]
            if live_duration is not None and state["interrupted"] and not state["capture_completed"]:
                print(f"{Fore.YELLOW}Live capture interrupted before duration elapsed; nothing saved{Style.RESET_ALL}")
            elif captured_packets:
                try:
                    saved_paths = capture_writer.save_samples_by_device(captured_packets)
                except ValueError as e:
                    print(f"{Fore.RED}❌ Failed to save live capture: {e}{Style.RESET_ALL}")
                    raise SystemExit(1)
                if saved_paths:
                    quality_passed = _run_post_collect_quality_checks(saved_paths)
                    print(
                        f"{Fore.GREEN}✅ Saved {len(saved_paths)} live capture file(s) "
                        f"from {len(captured_packets)} packets{Style.RESET_ALL}"
                    )
                    for saved_path in saved_paths:
                        print(f"  - {saved_path.name}")
                    if not quality_passed:
                        print(
                            f"{Fore.RED}❌ Saved capture failed post-collect quality admission"
                            f"{Style.RESET_ALL}"
                        )
                        raise SystemExit(1)
                else:
                    print(f"{Fore.RED}❌ Live capture had no packets to save{Style.RESET_ALL}")
            else:
                print(f"{Fore.YELLOW}No live capture packets received; nothing saved{Style.RESET_ALL}")
        print(f"\n{Fore.GREEN}Done.{Style.RESET_ALL}\n")
