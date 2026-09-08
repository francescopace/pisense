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

import pytest
import protocol
from src.python.micro_espectre import runtime_diagnostics as diagnostics
from console_output import format_detection_publish_line, print_log
from src.python.micro_espectre.runtime_diagnostics import (
    advance_periodic_anchor,
    periodic_maintenance_due,
    wifi_csi_available,
)
from tools.lib.repo_paths import repo_root


def test_runtime_diagnostic_rates_reuse_output_and_handle_counter_reset():
    sampler = diagnostics.RuntimeDiagnosticsSampler()
    output = {}

    def snapshot(total, window_slots=100):
        return diagnostics.collect_runtime_diagnostics_snapshot(
            traffic_generator=SimpleNamespace(get_packet_count=lambda: total),
            callback_total=total, accepted_total=total, admitted_total=total,
            filtered_total=total, missing_slots_total=total, excess_total=total,
            stale_total=total, out_of_order_total=total, occupancy_slots=75,
            window_slots=window_slots, wifi_channel=6, rssi_dbm=-57,
        )

    assert sampler.sample(snapshot(10), 1000, out=output) is output
    rates = [key for key in diagnostics.STATS_DIAGNOSTIC_KEYS if key.endswith("_pps")]
    assert all(output[key] == 0 for key in rates)
    assert output["wifi_channel"] == 6
    assert output["wifi_rssi_dbm"] == -57
    for now in [1000, 999]:
        sampler.sample(snapshot(20), now, out=output)
        assert all(output[key] == 0 for key in rates)
    sampler.sample(snapshot(30), 1500, out=output)
    assert all(output[key] == 40 for key in rates)
    assert output["csi_occupancy"] == 0.75
    sampler.sample(snapshot(5, window_slots=0), 2000, out=output)
    assert all(output[key] == 10 for key in rates)
    assert output["csi_occupancy"] == 0


@pytest.mark.parametrize("generator", [
    None, SimpleNamespace(), SimpleNamespace(get_packet_count=lambda: "invalid"),
])
def test_runtime_diagnostic_snapshot_handles_missing_traffic_counter(generator):
    output = {}
    result = diagnostics.collect_runtime_diagnostics_snapshot(traffic_generator=generator, out=output)
    assert result is output
    assert result["traffic_packets_total"] == 0


@pytest.mark.parametrize("reader,method,unavailable", [
    (diagnostics.wifi_rssi_dbm, "status", None),
    (diagnostics.wifi_csi_dropped, "csi_dropped", 0),
    (diagnostics.wifi_csi_callbacks, "csi_callbacks", None),
    (diagnostics.wifi_csi_available, "csi_available", 0),
])
def test_runtime_wifi_diagnostics_tolerate_older_or_unavailable_drivers(reader, method, unavailable):
    assert reader(None) == unavailable
    assert reader(SimpleNamespace()) == unavailable
    for value in [None, "invalid"]:
        assert reader(SimpleNamespace(**{method: lambda *_args, value=value: value})) == unavailable

    def failed(*_args):
        raise OSError("driver unavailable")

    assert reader(SimpleNamespace(**{method: failed})) == unavailable
    value = -60 if method == "status" else 12
    assert reader(SimpleNamespace(**{method: lambda *_args: value})) == value
    if method != "status":
        assert reader(SimpleNamespace(**{method: lambda: -1})) == 0
    else:
        assert reader(SimpleNamespace(isconnected=lambda: False, status=lambda *_args: -60)) is None


def test_runtime_diagnostics_include_native_quality_rejections_without_ring_drops():
    wlan = SimpleNamespace(csi_filtered=lambda: 100, csi_dropped=lambda: 7)
    snapshot = diagnostics.collect_runtime_diagnostics_snapshot(
        wlan=wlan, callback_total=200, filtered_total=3,
    )
    assert snapshot["csi_callbacks_total"] == 200
    assert snapshot["csi_filtered_total"] == 103
    assert diagnostics.wifi_csi_dropped(wlan) == 7
    for older_driver in (None, SimpleNamespace()):
        snapshot = diagnostics.collect_runtime_diagnostics_snapshot(
            wlan=older_driver, filtered_total=3,
        )
        assert snapshot["csi_filtered_total"] == 3


def test_runtime_performance_windows_preserve_heap_minimum_and_clear_samples():
    performance = diagnostics.RuntimePerformanceDiagnostics()
    window = performance.WINDOW_INTERVAL_MS
    output = {}
    assert performance.update_if_due(0, 4096, out=output) is output
    assert output["performance_window_ready"] is False
    assert output["loop_samples"] is None
    performance.record_loop_duration(100, weight=2)
    performance.record_loop_duration(400)
    performance.record_detection_duration(30)
    performance.record_detection_duration(10)
    assert performance.update_if_due(window - 1, 2048)["performance_window_ready"] is False
    sample = performance.update_if_due(window, 3072)
    assert sample["performance_window_ready"] is True
    assert sample["performance_window_ms"] == window
    assert sample["loop_samples"] == 3
    assert sample["loop_avg_us"] == 200
    assert sample["loop_max_us"] == 400
    assert sample["runtime_load_percent"] == pytest.approx(600 * 100 / (window * 1000))
    assert sample["detection_samples"] == 2
    assert sample["detection_sum_us"] == 40
    assert sample["detection_avg_us"] == 20
    assert sample["detection_min_us"] == 10
    assert sample["detection_max_us"] == 30
    assert sample["free_memory_kb"] == 3
    assert sample["minimum_free_memory_kb"] == 2
    empty = performance.update_if_due(window * 2, 4096)
    assert empty["loop_samples"] == empty["detection_samples"] == 0
    assert empty["loop_avg_us"] == empty["detection_avg_us"] == 0
    assert empty["runtime_load_percent"] == 0
    assert empty["minimum_free_memory_kb"] == 2
    # Returned snapshots remain independent of subsequent window updates.
    assert sample["loop_samples"] == 3


def test_runtime_performance_clamps_invalid_measurements_and_overload():
    performance = diagnostics.RuntimePerformanceDiagnostics()
    assert performance.snapshot(-1)["free_memory_kb"] == 0
    window = performance.WINDOW_INTERVAL_MS
    performance.update_if_due(0, 1024)
    performance.record_loop_duration(-10, weight=0)
    performance.record_loop_duration(window * 2000)
    performance.record_detection_duration(-10)
    sample = performance.update_if_due(window, 2048)
    assert sample["loop_samples"] == 2
    assert sample["runtime_load_percent"] == 100
    assert sample["detection_min_us"] == sample["detection_max_us"] == 0
    assert sample["minimum_free_memory_kb"] == 0


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


def test_native_direct_validates_requests_before_dispatch(tmp_path):
    """Execute the device handler with HTTPD and JSON parser boundary stubs."""
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
typedef int httpd_method_t;
typedef struct { int method; const char *uri; size_t content_len; } httpd_req_t;
typedef struct { const char *data; } direct_command_snapshot_t;
enum { HTTP_POST=1, HTTP_GET=2, HTTP_PATCH=3, ESP_OK=0, ESP_FAIL=-1 };
static struct { unsigned rate_limited_requests; unsigned oversized_requests; unsigned malformed_requests; } direct_state;
static bool queued, close_connection, busy;
static unsigned snapshot_reads, receive_calls, parser_calls;
static const char *response_status, *response_code, *incoming, *content_type;
static size_t incoming_length, received;
static int json_kind;
#ifdef TEST_REAL_CJSON
#include "cJSON.h"
static cJSON *test_parse(const char *body, size_t length, const char **end, bool strict) {
    ++parser_calls;
    return cJSON_ParseWithLengthOpts(body, length, end, strict);
}
#define cJSON_ParseWithLengthOpts test_parse
#else
/* The JSON dependency supplies a decoded object or a parse failure. The
 * contract exercised here is validation and dispatch around that boundary. */
typedef struct cJSON { struct cJSON *child; bool object; } cJSON;
static cJSON *cJSON_ParseWithLengthOpts(const char *body, size_t length, const char **end, bool strict) {
    static cJSON value;
    ++parser_calls;
    if (!strict || length != strlen(body) + 1U || json_kind == 0) return NULL;
    value.child = json_kind == 2 ? &value : NULL;
    value.object = json_kind != 3;
    return &value;
}
static bool cJSON_IsObject(const cJSON *value) { return value != NULL && value->object; }
static void cJSON_Delete(cJSON *value) {}
#endif
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
static int httpd_req_get_hdr_value_str(httpd_req_t *r, const char *key, char *out, size_t size) {
    if (content_type == NULL || strlen(content_type) >= size) return ESP_FAIL;
    strcpy(out, content_type);
    return ESP_OK;
}
static int httpd_req_recv(httpd_req_t *r, char *out, size_t size) {
    ++receive_calls;
    if (received >= incoming_length) return ESP_FAIL;
    /* Fragment the body to exercise repeated receive calls. */
    size_t count = incoming_length - received;
    if (count > size) count = size;
    if (count > 3U) count = 3U;
    memcpy(out, incoming + received, count);
    received += count;
    return (int) count;
}
static direct_command_snapshot_t direct_acquire_command_snapshot(const char *s) {
    ++snapshot_reads;
    return (direct_command_snapshot_t){"{}"};
}
static void direct_release_command_snapshot(direct_command_snapshot_t *s) {}
static bool direct_queue_recalibration(void) { queued = !busy; return queued; }
static int direct_send_result(httpd_req_t *r, const char *id, const char *cmd, bool ok, const char *code, const char *msg, void *data, const char *status) {
    response_status=status;
    response_code=code;
    return 0;
}
static void reset(const char *body, size_t size, int kind) {
    memset(&direct_state, 0, sizeof(direct_state));
    queued = close_connection = busy = false;
    snapshot_reads = receive_calls = parser_calls = 0U;
    response_status = "200 OK";
    response_code = "";
    incoming = body;
    incoming_length = size;
    received = 0U;
    content_type = "application/json";
    json_kind = kind;
}
'''
    main = r'''
#define CHECK(condition) do { if (!(condition)) { fprintf(stderr, "line %d: %s\n", __LINE__, #condition); return 1; } } while (0)
int main(void) {
    char padded[DIRECT_MAX_REQUEST_BYTES + 1];
    memset(padded, ' ', sizeof(padded));
    padded[0] = '{'; padded[1] = '}'; padded[DIRECT_MAX_REQUEST_BYTES] = '\0';
    const size_t sizes[] = {0, DIRECT_MAX_REQUEST_BYTES, DIRECT_MAX_REQUEST_BYTES + 1, 1048576};
    for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
        reset(padded, DIRECT_MAX_REQUEST_BYTES, 1);
        httpd_req_t request = {HTTP_POST, "/espectre/v1/sensing/calibrations", sizes[i]};
        int result = direct_request_handler(&request);
        bool oversized = sizes[i] > DIRECT_MAX_REQUEST_BYTES;
        CHECK(queued != oversized && close_connection == oversized);
        CHECK(direct_state.oversized_requests == oversized);
        CHECK(result == (oversized ? ESP_FAIL : ESP_OK));
        CHECK(strcmp(response_status, oversized ? "413 Payload Too Large" : "202 Accepted") == 0);
        CHECK(!oversized || (receive_calls == 0U && parser_calls == 0U));
    }
    const struct { const char *body; int kind; } cases[] = {
        {"{", 0}, {"[]", 3}, {"null", 3}, {"{} garbage", 0},
        {"{\"force\":true}", 2}, {"{\"force\":true,\"force\":false}", 2},
    };
    for (unsigned route = 0; route < sizeof(direct_routes) / sizeof(direct_routes[0]); ++route) {
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            reset(cases[i].body, strlen(cases[i].body), cases[i].kind);
            httpd_req_t request = {direct_routes[route].method, direct_routes[route].path, incoming_length};
            CHECK(direct_request_handler(&request) == ESP_FAIL);
            CHECK(!queued && snapshot_reads == 0U && parser_calls == 1U);
            CHECK(direct_state.malformed_requests == 1U);
            CHECK(strcmp(response_status, "400 Bad Request") == 0);
            CHECK(strcmp(response_code, "invalid_params") == 0);
        }
        reset("{}", 2U, 1);
        httpd_req_t request = {direct_routes[route].method, direct_routes[route].path, 2U};
        CHECK(direct_request_handler(&request) == ESP_OK);
        CHECK(parser_calls == 1U);
        CHECK(queued == (request.method == HTTP_POST));
        CHECK(snapshot_reads == (request.method == HTTP_GET));
    }
    const char *nested = "[[[[[[[[[0]]]]]]]]]";
    reset(nested, strlen(nested), 3);
    httpd_req_t deep_request = {HTTP_POST, "/espectre/v1/sensing/calibrations", incoming_length};
    CHECK(direct_request_handler(&deep_request) == ESP_FAIL);
    CHECK(!queued && parser_calls == 0U && direct_state.malformed_requests == 1U);
    reset("{}\0junk", 7U, 1);
    httpd_req_t request = {HTTP_POST, "/espectre/v1/sensing/calibrations", 7U};
    CHECK(direct_request_handler(&request) == ESP_FAIL);
    CHECK(!queued && parser_calls == 0U && direct_state.malformed_requests == 1U);
    reset("{", 1U, 0);
    request.content_len = 2U;
    CHECK(direct_request_handler(&request) == ESP_FAIL);
    CHECK(!queued && close_connection && parser_calls == 0U);
    for (int missing = 0; missing < 2; ++missing) {
        reset("{}", 2U, 1);
        content_type = missing ? NULL : "text/plain";
        CHECK(direct_request_handler(&request) == ESP_FAIL);
        CHECK(!queued && parser_calls == 0U);
        CHECK(strcmp(response_status, "415 Unsupported Media Type") == 0);
    }
    reset("{}", 2U, 1);
    busy = true;
    CHECK(direct_request_handler(&request) == ESP_OK);
    CHECK(!queued && strcmp(response_status, "409 Conflict") == 0);
    CHECK(strcmp(response_code, "busy") == 0);
    return 0;
}
'''
    probe_source = tmp_path / "direct_request_probe.c"
    probe_source.write_text(limit + "\n" + stubs + handler + main, encoding="utf-8")
    executable = tmp_path / "direct_request_probe"
    subprocess.run(["cc", str(probe_source), "-o", str(executable)], check=True,
                   capture_output=True, text=True)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)
