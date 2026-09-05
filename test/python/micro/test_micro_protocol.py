# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Micro-ESPectre protocol and read-only Direct facade contracts."""

import errno
import json
import subprocess
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import protocol
from console_output import format_detection_publish_line, print_log
from src.python.micro_espectre.runtime_diagnostics import (
    advance_periodic_anchor,
    periodic_maintenance_due,
    wifi_csi_available,
)
from tools.lib.repo_paths import repo_root


def test_micro_heartbeat_uses_the_shared_runtime_status_format():
    line = format_detection_publish_line(
        diagnostics={
            "csi_admitted_pps": 99.0,
            "csi_accepted_pps": 100.0,
            "csi_callback_pps": 102.0,
            "traffic_tx_pps": 101.0,
            "csi_occupancy": 0.8,
            "csi_missing_slots_pps": 1.0,
            "csi_excess_pps": 2.0,
            "csi_stale_pps": 3.0,
            "csi_out_of_order_pps": 4.0,
            "wifi_channel": 6,
            "wifi_rssi_dbm": -50,
        },
        motion_metric=0.75,
        threshold=0.25,
        effective_state=1,
    )

    assert line == (
        "[#####|#########-----] | mvmt:0.750000 thr:0.250000 | MOTION | "
        "csi:99/100 cb:102 tx:101 occ:80% miss:1 excess:2 stale:3 ooo:4 | ch:6 rssi:-50"
    )


def test_print_log_uses_a_stable_level_prefix(capsys):
    print_log("WARN", "CSI link stalled; rearming CSI")

    assert capsys.readouterr().out == "[WARN] CSI link stalled; rearming CSI\n"


def test_micro_heartbeat_schedule_does_not_accumulate_maintenance_time():
    assert advance_periodic_anchor(10_000, 11_040, 1_000) == 11_000
    assert advance_periodic_anchor(11_000, 12_075, 1_000) == 12_000


def test_micro_heartbeat_schedule_reanchors_after_a_missed_interval():
    assert advance_periodic_anchor(10_000, 12_100, 1_000) == 12_100


def test_micro_heartbeat_schedule_handles_tick_wrap(monkeypatch):
    tick_period = 1 << 30
    half_period = tick_period // 2
    monkeypatch.setattr(
        time,
        "ticks_add",
        lambda value, delta: (value + delta) % tick_period,
        raising=False,
    )
    monkeypatch.setattr(
        time,
        "ticks_diff",
        lambda current, previous: (
            (current - previous + half_period) % tick_period
        ) - half_period,
        raising=False,
    )

    assert advance_periodic_anchor(tick_period - 500, 540, 1_000) == 500


def test_micro_maintenance_waits_for_ring_then_honors_maximum_deferral():
    assert not periodic_maintenance_due(14_999, 10_000, 5_000, False, 500)
    assert periodic_maintenance_due(15_000, 10_000, 5_000, True, 500)
    assert not periodic_maintenance_due(15_499, 10_000, 5_000, False, 500)
    assert periodic_maintenance_due(15_500, 10_000, 5_000, False, 500)


def test_micro_csi_backlog_uses_native_ring_occupancy():
    wlan = SimpleNamespace(csi_available=lambda: 7)

    assert wifi_csi_available(wlan) == 7
    assert wifi_csi_available(SimpleNamespace()) == 0


def test_device_id_matches_native_sha256_pseudonym():
    assert protocol._derive_device_id_from_mac(bytes.fromhex("7c2c6742bbac")) == "3cf79180d3a0aca4"


def test_chip_labels_match_the_shared_short_names():
    assert protocol._normalize_chip_label("esp32s2") == "S2"
    assert protocol._normalize_chip_label("ESP32-C6") == "C6"


def test_cpp_and_python_protocol_catalogs_match():
    root = repo_root()
    build_dir = root / "test" / "cpp" / "build"
    probe = build_dir / "suites" / "espectre_capabilities_probe"
    subprocess.run(["cmake", "-S", str(root / "test" / "cpp"), "-B", str(build_dir)], check=True)
    subprocess.run(["cmake", "--build", str(build_dir), "--target", "espectre_capabilities_probe"], check=True)

    cpp_catalog = json.loads(subprocess.run(
        [str(probe), "micro"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout)
    python_catalog = {
        "capabilities": protocol.build_capabilities_payload("0000000000000000"),
        "message_model": protocol.build_protocol_catalog(),
    }
    assert cpp_catalog == python_catalog


def test_micro_capabilities_are_bounded_and_include_recalibration():
    payload = protocol.build_capabilities_payload("0123456789abcdef")
    commands = {command["name"] for command in payload["operations"]}

    assert commands == {"read_diagnostics", "recalibrate"}
    recalibrate = next(command for command in payload["operations"] if command["name"] == "recalibrate")
    assert recalibrate == {
        "name": "recalibrate",
        "method": "POST",
        "path": "/espectre/v1/sensing/calibrations",
    }
    assert payload["events"] == ["motion"]
    assert payload["resources"] == ["health", "device", "capabilities", "sensing", "wifi", "diagnostics"]
    assert payload["features"] == {"csi": False}


def test_micro_diagnostics_payload_keeps_only_canonical_fields():
    payload = protocol.build_diagnostics_payload(
        "0123456789abcdef",
        12_000,
        12,
        {
            "free_memory_kb": 118.5,
            "csi_admitted_pps": 99.0,
            "performance_window_ready": True,
            "packet_processing_us": 400,
            "gc_pause_us": 500,
        },
    )

    assert payload["free_memory_kb"] == 118.5
    assert payload["csi_admitted_pps"] == 99.0
    assert payload["performance_window_ready"] is True
    assert "packet_processing_us" not in payload
    assert "gc_pause_us" not in payload


def test_native_direct_diagnostics_use_pinned_double_buffer_without_heap_copy():
    source = (
        repo_root()
        / "src/python/micro_espectre/firmware/native_components/native_direct.c"
    ).read_text(encoding="utf-8")

    assert (
        "char diagnostics[DIRECT_DIAGNOSTICS_BUFFER_COUNT]"
        "[DIRECT_MAX_DIAGNOSTICS_BYTES + 1];"
    ) in source
    assert "uint8_t diagnostics_readers[DIRECT_DIAGNOSTICS_BUFFER_COUNT];" in source
    assert "char status_staging[DIRECT_MAX_STATUS_BYTES + 1];" in source
    assert "json_staging" not in source
    assert "snapshot = direct_state.diagnostics" not in source
    assert "result.data = direct_state.diagnostics[index];" in source
    assert "result.diagnostics_pinned = true;" in source
    assert "direct_state.diagnostics_readers[index] -= 1U;" in source
    assert "free(snapshot);" not in source


def test_native_direct_enforces_bounded_secure_sse_profile():
    source = (
        repo_root()
        / "src/python/micro_espectre/firmware/native_components/native_direct.c"
    ).read_text(encoding="utf-8")

    assert "#define DIRECT_MAX_REQUEST_BYTES (512)" in source
    assert "#define DIRECT_MAX_EVENT_BYTES (4096)" in source
    assert "#define DIRECT_HEARTBEAT_INTERVAL_US (10000000ULL)" in source
    assert "CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED" in source
    assert "return origin != NULL && origin[0] != '\\0' &&" in source
    assert 'strcmp(origin, "https://espectre.dev") == 0' in source
    assert 'strcmp(origin, "https://www.espectre.dev") == 0' in source
    assert 'strcmp(origin, "https://test.espectre.dev") == 0' in source
    assert 'DIRECT_HEARTBEAT_FRAME = ": heartbeat\\n\\n"' in source
    assert 'httpd_resp_set_hdr(request, "Cache-Control", "no-store")' in source
    assert "!direct_state.heartbeat_work_pending" in source
    assert "direct_state.event_work_pending || direct_state.heartbeat_work_pending" in source
    assert "esp_timer_start_periodic(" in source
    assert "direct_stop_heartbeat();" in source
    assert "esp_timer_delete(direct_state.heartbeat_timer)" in source
    assert "if (active && result != ESP_OK)" in source
    assert "direct_peer_disconnect_reason" in source
    assert "esp_app_get_description()" in source
    assert "CONFIG_IDF_TARGET" in source
    assert '{"firmware", firmware_version}' in source


def test_direct_facade_starts_and_publishes_canonical_motion(monkeypatch):
    native = MagicMock()
    native.start.side_effect = [OSError(errno.EBUSY), None]
    native.firmware_version.return_value = "2.8.0-356-gfa155f8"
    native.chip_target.return_value = "esp32c3"
    native.diagnostics.side_effect = lambda target: target.update(
        accepted_connections=3,
        rejected_connections=2,
        dropped_motion_events=4,
        send_failures=1,
    )
    monkeypatch.setitem(sys.modules, "espectre_native_direct", native)
    if not hasattr(time, "ticks_ms"):
        monkeypatch.setattr(time, "ticks_ms", lambda: 1000, raising=False)
    if not hasattr(time, "ticks_diff"):
        monkeypatch.setattr(time, "ticks_diff", lambda current, previous: current - previous, raising=False)
    if not hasattr(time, "sleep_ms"):
        monkeypatch.setattr(time, "sleep_ms", lambda _milliseconds: None, raising=False)
    sys.modules.pop("direct_api", None)
    from direct_api import DirectApi, _wifi_band

    assert _wifi_band(None) == ""
    assert _wifi_band(0) == ""
    assert _wifi_band(6) == "2g"
    assert _wifi_band(36) == "5g"

    wlan = MagicMock()
    wlan_values = {
        "mac": bytes.fromhex("7c2c6742bbac"),
        "channel": 6,
        "ssid": "lab",
        "bssid": bytes.fromhex("001122334455"),
    }
    wlan.config.side_effect = lambda key: wlan_values[key]
    wlan.status.return_value = -50
    wlan.active.return_value = True
    wlan.isconnected.return_value = True
    detector = MagicMock()
    detector.get_threshold.return_value = 0.25
    detector.is_ready.return_value = True
    policy = SimpleNamespace(motion_on_hits=4, motion_off_hits=3)
    traffic = MagicMock()
    traffic.is_running.return_value = True
    traffic.get_mode.return_value = "dns"
    state = SimpleNamespace(chip_type="C3", current_channel=6, calibration_mode=False)
    config = SimpleNamespace(
        WIFI_SSID="lab",
        CSI_TARGET_PPS=100,
        EVALUATION_INTERVAL_MS=250,
    )

    facade = DirectApi(config, wlan, detector, state, policy, traffic)
    facade.start()
    assert native.start.call_count == 2
    facade.refresh_snapshots(facade.started_ms + 1000, {"csi_admitted_pps": 99.0})
    initial_diagnostics = native.update_diagnostics.call_args.args[0].copy()
    native.update_status.reset_mock()
    native.update_diagnostics.reset_mock()
    facade.refresh_status(facade.started_ms + 1250)
    native.update_status.assert_called_once()
    native.update_diagnostics.assert_not_called()
    native.update_config.assert_not_called()
    detector.is_ready.return_value = False
    facade.refresh_status(facade.started_ms + 1500)
    native.update_config.assert_called_once()
    facade.refresh_diagnostics(
        facade.started_ms + 5000,
        {"csi_admitted_pps": 98.0},
    )
    native.update_diagnostics.assert_called_once()
    facade.refresh_config()
    assert native.update_config.call_count == 2
    native.take_recalibration_request.return_value = True
    assert facade.take_recalibration_request() is True
    facade.complete_recalibration()
    native.complete_recalibration.assert_called_once_with()
    native.publish.assert_not_called()
    native.has_event_client.return_value = False
    facade.publish_motion(0.75, 1, 0.25, facade.started_ms + 1000)
    native.publish.assert_not_called()
    native.has_event_client.return_value = True
    facade.publish_motion(0.75, 1, 0.25, facade.started_ms + 1000)

    start_args = native.start.call_args.kwargs
    capabilities = start_args["capabilities"]
    info = start_args["info"]
    assert {entry["name"] for entry in capabilities["operations"]} == {
        "read_diagnostics", "recalibrate"
    }
    assert start_args["hostname"] == "espectre-3cf79180d3a0aca4"
    assert start_args["instance"] == info["name"]
    assert start_args["protocol_version"] == protocol.PROTOCOL_VERSION
    assert start_args["dns_sd_schema_version"] == protocol.DNS_SD_TXT_SCHEMA_VERSION
    assert start_args["firmware_version"] == "2.8.0-356-gfa155f8"
    assert info["name"].startswith("ESPectre C3 ")
    assert info["label"] == ""
    assert info["firmware"] == "2.8.0-356-gfa155f8"
    assert info["chip"] == "esp32c3"
    assert info["csi_profile"] == "ht20"
    assert facade._config()["csi_traffic_mode"] == "internal"
    assert facade._config()["traffic_generator_mode"] == "dns"
    assert facade._wifi()["band"] == "2g"
    wlan_values["channel"] = 36
    assert facade._wifi()["band"] == "5g"
    wlan_values["channel"] = 0
    assert facade._wifi()["band"] == ""
    traffic.is_running.return_value = False
    assert facade._config()["csi_traffic_mode"] == "external"
    event_name, motion = native.publish.call_args.args
    assert event_name == "motion"
    assert motion == {"timestamp_ms": facade.started_ms + 1000, "state": "motion", "score": 0.75}
    diagnostics = initial_diagnostics
    assert diagnostics["uptime"] == 1
    assert diagnostics["direct_http"]["accepted_connections"] == 3
    assert diagnostics["direct_http"]["rejected_connections"] == 2
    assert diagnostics["direct_http"]["dropped_motion_events"] == 4
    assert diagnostics["direct_http"]["send_failures"] == 1
    native.diagnostics.assert_called()

    facade.stop()
    native.stop.assert_called_once_with()


def test_direct_facade_uptime_accumulates_across_tick_wrap(monkeypatch):
    native = MagicMock()
    monkeypatch.setitem(sys.modules, "espectre_native_direct", native)
    tick_period = 1 << 30
    half_period = tick_period // 2

    def ticks_diff(current, previous):
        return ((current - previous + half_period) % tick_period) - half_period

    monkeypatch.setattr(time, "ticks_diff", ticks_diff, raising=False)
    sys.modules.pop("direct_api", None)
    from direct_api import DirectApi

    facade = object.__new__(DirectApi)
    facade._uptime_last_ms = tick_period - 500
    facade._uptime_ms = 0

    assert facade._uptime_seconds(500) == 1
    assert facade._uptime_seconds(1500) == 2


def test_native_direct_rejects_oversized_bodies_before_recalibration(tmp_path):
    """Execute the device handler with HTTPD stubs, including boundary sizes."""
    source = (
        repo_root() / "src/python/micro_espectre/firmware/native_components/native_direct.c"
    ).read_text(encoding="utf-8")
    limit = next(line for line in source.splitlines()
                 if line.startswith("#define DIRECT_MAX_REQUEST_BYTES "))
    handler = source[
        source.index("static bool direct_request_size_allowed("):
        source.index("static esp_err_t direct_options_handler(")
    ]
    stubs = r'''

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
typedef int esp_err_t;
typedef struct { int method; const char *uri; size_t content_len; } httpd_req_t;
typedef struct { const char *data; } direct_command_snapshot_t;
enum { HTTP_POST=1, HTTP_GET=2, ESP_FAIL=-1 };
static struct { unsigned rate_limited_requests; unsigned oversized_requests; } direct_state;
static bool queued;
static bool close_connection;
static const char *response_status;
static bool direct_set_cors(httpd_req_t *r) { return true; }
static bool direct_request_allowed(void) { return true; }
static void direct_increment(unsigned *p) { ++*p; }
static void httpd_resp_set_status(httpd_req_t *r, const char *s) { response_status=s; }
static int httpd_resp_sendstr(httpd_req_t *r, const char *s) { return 0; }
static void httpd_resp_set_type(httpd_req_t *r, const char *s) {}
static int httpd_resp_set_hdr(httpd_req_t *r, const char *k, const char *v) {
    if (strcmp(k, "Connection") == 0 && strcmp(v, "close") == 0) close_connection = true;
    return 0;
}
static direct_command_snapshot_t direct_acquire_command_snapshot(const char *s) { return (direct_command_snapshot_t){"{}"}; }
static void direct_release_command_snapshot(direct_command_snapshot_t *s) {}
static bool direct_queue_recalibration(void) { queued=true; return true; }
static int direct_send_result(httpd_req_t *r, const char *id, const char *cmd, bool ok, const char *code, const char *msg, void *data, const char *status) { response_status=status; return 0; }
'''
    main = r'''

int main(void) {
    const size_t sizes[] = {0, DIRECT_MAX_REQUEST_BYTES, DIRECT_MAX_REQUEST_BYTES + 1, 1048576};
    for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
        queued = false;
        close_connection = false;
        response_status = NULL;
        direct_state.oversized_requests = 0;
        httpd_req_t request = {HTTP_POST, "/espectre/v1/sensing/calibrations", sizes[i]};
        int result = direct_request_handler(&request);
        bool oversized = sizes[i] > DIRECT_MAX_REQUEST_BYTES;
        if (queued == oversized || close_connection != oversized ||
            direct_state.oversized_requests != oversized ||
            result != (oversized ? ESP_FAIL : 0) ||
            strcmp(response_status, oversized ? "413 Payload Too Large" : "202 Accepted") != 0) {
            return 1;
        }
    }
    return 0;
}
'''
    probe_source = tmp_path / "direct_request_probe.c"
    probe_source.write_text(limit + "\n" + stubs + handler + main, encoding="utf-8")
    executable = tmp_path / "direct_request_probe"
    subprocess.run(["cc", str(probe_source), "-o", str(executable)], check=True,
                   capture_output=True, text=True)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)
