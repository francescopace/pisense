# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
Micro-ESPectre - Runtime Diagnostics

Rate and link diagnostics derived from cumulative runtime counters.
Mirrors the canonical C++ runtime diagnostic window for Direct queries.
"""
import time

STATS_DIAGNOSTIC_KEYS = (
    "traffic_tx_pps",
    "csi_callback_pps",
    "csi_accepted_pps",
    "csi_admitted_pps",
    "csi_filtered_pps",
    "csi_missing_slots_pps",
    "csi_excess_pps",
    "csi_stale_pps",
    "csi_out_of_order_pps",
    "csi_occupancy",
    "wifi_channel",
    "wifi_rssi_dbm",
)

def _ticks_diff(new, old):
    diff_fn = getattr(time, "ticks_diff", None)
    return diff_fn(new, old) if diff_fn is not None else new - old


def advance_periodic_anchor(previous_ms, completed_ms, interval_ms):
    """Advance a periodic anchor without accumulating routine work time."""
    add_fn = getattr(time, "ticks_add", None)
    scheduled_ms = (
        add_fn(previous_ms, interval_ms)
        if add_fn is not None
        else previous_ms + interval_ms
    )
    if _ticks_diff(completed_ms, scheduled_ms) >= interval_ms:
        return completed_ms
    return scheduled_ms


def periodic_maintenance_due(
    now_ms,
    previous_ms,
    interval_ms,
    resource_ready,
    max_deferral_ms,
):
    """Return whether periodic work may run now, with bounded deferral."""
    elapsed_ms = _ticks_diff(now_ms, previous_ms)
    if elapsed_ms < interval_ms:
        return False
    return bool(resource_ready) or elapsed_ms >= interval_ms + max_deferral_ms


def _counter_delta(current, previous):
    current = int(current)
    previous = int(previous)
    return current - previous if current >= previous else current


def _packets_per_second(delta, elapsed_ms):
    if elapsed_ms <= 0:
        return 0.0
    return (float(delta) * 1000.0) / float(elapsed_ms)


def empty_diagnostics_sample(wifi_channel=0, wifi_rssi_dbm=None, out=None):
    """Return the CSI/Wi-Fi fields with zero rates."""
    if out is None:
        out = {}
    out["traffic_tx_pps"] = 0.0
    out["csi_callback_pps"] = 0.0
    out["csi_accepted_pps"] = 0.0
    out["csi_admitted_pps"] = 0.0
    out["csi_filtered_pps"] = 0.0
    out["csi_missing_slots_pps"] = 0.0
    out["csi_excess_pps"] = 0.0
    out["csi_stale_pps"] = 0.0
    out["csi_out_of_order_pps"] = 0.0
    out["csi_occupancy"] = 0.0
    out["wifi_channel"] = int(wifi_channel or 0)
    out["wifi_rssi_dbm"] = wifi_rssi_dbm
    return out


def wifi_rssi_dbm(wlan):
    """Return association RSSI in dBm, or None when unavailable."""
    if wlan is None:
        return None
    try:
        isconnected = getattr(wlan, "isconnected", None)
        if callable(isconnected) and not isconnected():
            return None
        status = getattr(wlan, "status", None)
        if not callable(status):
            return None
        rssi = status("rssi")
        if rssi is None:
            return None
        return int(rssi)
    except (AttributeError, OSError, TypeError, ValueError):
        return None


def _wifi_csi_counter(wlan, name, default=0):
    try:
        getter = getattr(wlan, name, None)
        return max(0, int(getter())) if callable(getter) else default
    except (AttributeError, OSError, TypeError, ValueError):
        return default


def wifi_csi_dropped(wlan):
    """Return the cumulative native CSI ring-buffer drop count."""
    return _wifi_csi_counter(wlan, "csi_dropped")


def wifi_csi_callbacks(wlan):
    """Return native CSI callback invocations, or None on older firmware."""
    return _wifi_csi_counter(wlan, "csi_callbacks", None)


def wifi_csi_available(wlan):
    """Return the number of complete records waiting in the native CSI ring."""
    return _wifi_csi_counter(wlan, "csi_available")


def collect_runtime_diagnostics_snapshot(
    traffic_generator=None,
    callback_total=0,
    accepted_total=0,
    admitted_total=0,
    filtered_total=0,
    missing_slots_total=0,
    excess_total=0,
    stale_total=0,
    out_of_order_total=0,
    occupancy_slots=0,
    window_slots=0,
    wifi_channel=0,
    rssi_dbm=None,
    out=None,
    wlan=None,
):
    """Build the cumulative counter snapshot consumed by the rate sampler."""
    traffic_packets_total = 0
    if traffic_generator is not None:
        get_count = getattr(traffic_generator, "get_packet_count", None)
        if callable(get_count):
            try:
                traffic_packets_total = int(get_count())
            except (TypeError, ValueError):
                traffic_packets_total = 0
    if out is None:
        out = {}
    out["traffic_packets_total"] = int(traffic_packets_total)
    out["csi_callbacks_total"] = int(callback_total)
    out["csi_accepted_total"] = int(accepted_total)
    out["csi_admitted_total"] = int(admitted_total)
    out["csi_filtered_total"] = int(filtered_total) + _wifi_csi_counter(wlan, "csi_filtered")
    out["csi_missing_slots_total"] = int(missing_slots_total)
    out["csi_excess_total"] = int(excess_total)
    out["csi_stale_total"] = int(stale_total)
    out["csi_out_of_order_total"] = int(out_of_order_total)
    out["csi_occupancy_slots"] = int(occupancy_slots)
    out["csi_window_slots"] = int(window_slots)
    out["wifi_channel"] = int(wifi_channel or 0)
    out["wifi_rssi_dbm"] = rssi_dbm
    return out


def apply_diagnostics_sample(payload, sample, wifi_channel=0, rssi_dbm=None):
    """Copy diagnostic keys into `payload`, filling zeros for missing ones."""
    defaults = empty_diagnostics_sample(wifi_channel=wifi_channel, wifi_rssi_dbm=rssi_dbm)
    source = sample if isinstance(sample, dict) else defaults
    for key in STATS_DIAGNOSTIC_KEYS:
        if key in source:
            payload[key] = source[key]
        else:
            payload[key] = defaults[key]
    return payload


class RuntimeDiagnosticsSampler:
    """Convert cumulative counters into rates over the interval between reads."""

    def __init__(self):
        self._previous = {}
        self._previous_ms = 0
        self._baseline_ready = False

    def reset(self, snapshot, now_ms):
        self._previous.update(snapshot)
        self._previous_ms = int(now_ms)
        self._baseline_ready = True

    def sample(self, snapshot, now_ms, out=None):
        now_ms = int(now_ms)
        result = empty_diagnostics_sample(
            wifi_channel=snapshot.get("wifi_channel", 0),
            wifi_rssi_dbm=snapshot.get("wifi_rssi_dbm"),
            out=out,
        )
        if not self._baseline_ready:
            self.reset(snapshot, now_ms)
            return result

        elapsed_ms = _ticks_diff(now_ms, self._previous_ms)
        if elapsed_ms <= 0:
            return result

        previous = self._previous
        result["traffic_tx_pps"] = _packets_per_second(
            _counter_delta(snapshot["traffic_packets_total"], previous["traffic_packets_total"]),
            elapsed_ms,
        )
        result["csi_callback_pps"] = _packets_per_second(
            _counter_delta(snapshot["csi_callbacks_total"], previous["csi_callbacks_total"]),
            elapsed_ms,
        )
        result["csi_accepted_pps"] = _packets_per_second(
            _counter_delta(snapshot["csi_accepted_total"], previous["csi_accepted_total"]),
            elapsed_ms,
        )
        result["csi_admitted_pps"] = _packets_per_second(
            _counter_delta(snapshot["csi_admitted_total"], previous["csi_admitted_total"]),
            elapsed_ms,
        )
        result["csi_filtered_pps"] = _packets_per_second(
            _counter_delta(snapshot["csi_filtered_total"], previous["csi_filtered_total"]),
            elapsed_ms,
        )
        result["csi_missing_slots_pps"] = _packets_per_second(
            _counter_delta(snapshot["csi_missing_slots_total"], previous["csi_missing_slots_total"]),
            elapsed_ms,
        )
        result["csi_excess_pps"] = _packets_per_second(
            _counter_delta(snapshot["csi_excess_total"], previous["csi_excess_total"]),
            elapsed_ms,
        )
        result["csi_stale_pps"] = _packets_per_second(
            _counter_delta(snapshot["csi_stale_total"], previous["csi_stale_total"]),
            elapsed_ms,
        )
        result["csi_out_of_order_pps"] = _packets_per_second(
            _counter_delta(snapshot["csi_out_of_order_total"], previous["csi_out_of_order_total"]),
            elapsed_ms,
        )
        window_slots = int(snapshot.get("csi_window_slots", 0) or 0)
        occupancy_slots = int(snapshot.get("csi_occupancy_slots", 0) or 0)
        result["csi_occupancy"] = (
            occupancy_slots / float(window_slots) if window_slots > 0 else 0.0
        )
        self.reset(snapshot, now_ms)
        return result


class RuntimePerformanceDiagnostics:
    """Aggregate loop, detector, and heap measurements into canonical windows."""

    WINDOW_INTERVAL_MS = 10_000
    def __init__(self):
        self._minimum_heap_free = None
        self._latest = self._empty_snapshot()
        self.reset()

    @staticmethod
    def _empty_snapshot():
        return {
            "performance_window_ready": False,
            "performance_window_ms": None,
            "runtime_load_percent": None,
            "loop_samples": None,
            "loop_avg_us": None,
            "loop_max_us": None,
            "detection_timing_supported": True,
            "detection_samples": None,
            "detection_sum_us": None,
            "detection_avg_us": None,
            "detection_min_us": None,
            "detection_max_us": None,
        }

    def reset(self):
        """Clear the current timing window while retaining the heap low-water mark."""
        self._window_start_ms = None
        self._loop_busy_us = 0
        self._loop_duration_sum_us = 0
        self._loop_duration_max_us = 0
        self._loop_samples = 0
        self._detection_duration_sum_us = 0
        self._detection_duration_min_us = 0
        self._detection_duration_max_us = 0
        self._detection_samples = 0

    def record_loop_duration(self, duration_us, weight=1):
        """Record one measured main-loop body duration."""
        duration_us = max(0, int(duration_us))
        weight = max(1, int(weight))
        self._loop_busy_us += duration_us * weight
        self._loop_duration_sum_us += duration_us * weight
        self._loop_duration_max_us = max(self._loop_duration_max_us, duration_us)
        self._loop_samples += weight

    def record_detection_duration(self, duration_us):
        """Record one detector evaluation duration."""
        duration_us = max(0, int(duration_us))
        self._detection_duration_sum_us += duration_us
        if self._detection_samples == 0:
            self._detection_duration_min_us = duration_us
        else:
            self._detection_duration_min_us = min(
                self._detection_duration_min_us,
                duration_us,
            )
        self._detection_duration_max_us = max(
            self._detection_duration_max_us,
            duration_us,
        )
        self._detection_samples += 1

    def update_if_due(self, now_ms, heap_free, out=None):
        """Complete a window when due and return the current canonical snapshot."""
        now_ms = int(now_ms)
        heap_free = max(0, int(heap_free))
        if self._minimum_heap_free is None:
            self._minimum_heap_free = heap_free
        else:
            self._minimum_heap_free = min(self._minimum_heap_free, heap_free)
        if self._window_start_ms is None:
            self._window_start_ms = now_ms
            return self.snapshot(heap_free, out=out)
        elapsed_ms = _ticks_diff(now_ms, self._window_start_ms)
        if elapsed_ms < self.WINDOW_INTERVAL_MS:
            return self.snapshot(heap_free, out=out)

        elapsed_us = max(1, int(elapsed_ms) * 1000)
        runtime_load = min(100.0, self._loop_busy_us * 100.0 / elapsed_us)
        loop_average = (
            self._loop_duration_sum_us // self._loop_samples
            if self._loop_samples
            else 0
        )
        detection_average = (
            self._detection_duration_sum_us // self._detection_samples
            if self._detection_samples
            else 0
        )
        self._latest["performance_window_ready"] = True
        self._latest["performance_window_ms"] = elapsed_ms
        self._latest["runtime_load_percent"] = runtime_load
        self._latest["loop_samples"] = self._loop_samples
        self._latest["loop_avg_us"] = loop_average
        self._latest["loop_max_us"] = self._loop_duration_max_us
        self._latest["detection_timing_supported"] = True
        self._latest["detection_samples"] = self._detection_samples
        self._latest["detection_sum_us"] = self._detection_duration_sum_us
        self._latest["detection_avg_us"] = detection_average
        self._latest["detection_min_us"] = self._detection_duration_min_us
        self._latest["detection_max_us"] = self._detection_duration_max_us
        self.reset()
        self._window_start_ms = now_ms
        return self.snapshot(heap_free, out=out)

    def snapshot(self, heap_free, out=None):
        """Return the latest window with current and low-water heap values."""
        heap_free = max(0, int(heap_free))
        if self._minimum_heap_free is None:
            self._minimum_heap_free = heap_free
        else:
            self._minimum_heap_free = min(self._minimum_heap_free, heap_free)
        result = {} if out is None else out
        result.update(self._latest)
        result["free_memory_kb"] = heap_free / 1024.0
        result["minimum_free_memory_kb"] = self._minimum_heap_free / 1024.0
        return result
