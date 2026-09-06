# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Current HTTP-only collector and Direct discovery contracts."""

from __future__ import annotations

import argparse
from types import SimpleNamespace

import pytest
import numpy as np

from espectre_cli.app import build_parser
from espectre_cli import device_discovery, host
from espectre_cli.device_discovery import DiscoveredDevice, ESPECTRE_DIRECT_PORT, ESPECTRE_SERVICE_TYPE
from tools.espectre_traffic_generator import ExternalTrafficGenerator
from tools.lib import csi_io


def discovered_device(
    *,
    frontend: str = "native",
    port: int = ESPECTRE_DIRECT_PORT,
    device_id: int = 0x1234,
    capabilities: tuple[str, ...] = ("sensing", "motion", "csi"),
) -> DiscoveredDevice:
    authority = "192.168.1.23" if port == 80 else f"192.168.1.23:{port}"
    return DiscoveredDevice(
        service_name=f"ESPectre {frontend}._espectre._tcp.local.",
        service_type=ESPECTRE_SERVICE_TYPE,
        frontend=frontend,
        device_id=device_id,
        device_id_text=f"{device_id:016x}",
        name=f"ESPectre {frontend}",
        chip="esp32c3",
        ip_address="192.168.1.23",
        port=port,
        transport="http",
        endpoint=f"http://{authority}/espectre/v1",
        protocol="1",
        events_endpoint=f"http://{authority}/espectre/v1/events",
        capabilities=capabilities,
    )


def collect_args(**overrides) -> argparse.Namespace:
    values = {
        "target": "192.168.1.23",
        "frontend": None,
        "source_ip": None,
        "pps": 100,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_collect_parser_exposes_only_http_collection_options() -> None:
    args = build_parser().parse_args(
        [
            "collect",
            "--target",
            f"espectre.local:{ESPECTRE_DIRECT_PORT}",
            "--frontend",
            "esphome",
            "--source-ip",
            "192.168.1.8",
            "--pps",
            "325",
            "--label",
            "benchmark",
        ]
    )

    assert args.target == f"espectre.local:{ESPECTRE_DIRECT_PORT}"
    assert args.frontend == "esphome"
    assert args.source_ip == "192.168.1.8"
    assert args.pps == 325
    assert args.label == "benchmark"


def test_collect_rejects_unsafe_label_before_resolving_device(monkeypatch) -> None:
    target_resolution_attempted = False

    def resolve_target(_args):
        nonlocal target_resolution_attempted
        target_resolution_attempted = True

    monkeypatch.setattr(host, "_resolve_collect_target_via_discovery", resolve_target)
    args = collect_args(
        label="../outside",
        info=False,
        ready_stable_seconds=3.0,
    )

    with pytest.raises(SystemExit):
        host.collect_csi_data(args)

    assert target_resolution_attempted is False


def test_collect_allows_live_inspection_without_label(monkeypatch) -> None:
    calls = []
    monkeypatch.setattr(
        host,
        "_resolve_collect_target_via_discovery",
        lambda _args: calls.append("resolve"),
    )
    monkeypatch.setattr(host, "_run_live_collect", lambda _args: calls.append("run"))
    args = collect_args(
        label=None,
        info=False,
        ready_stable_seconds=3.0,
    )

    host.collect_csi_data(args)

    assert calls == ["resolve", "run"]


@pytest.mark.parametrize("label", [None, "capture"])
def test_collect_duration_expires_after_packets_stop(monkeypatch, tmp_path, label):
    clock = [0.0]
    saved_packets = []
    monkeypatch.setattr(host.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(host.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(host, "_run_post_collect_quality_checks", lambda _paths: True)
    # Exercise the recording deadline independently of startup calibration.
    monkeypatch.setattr("detector_interface.detector_needs_startup_calibration", lambda _kind: False)
    packet = csi_io.CSIPacket(
        timestamp=20.0,
        seq_num=0,
        num_subcarriers=64,
        iq_raw=np.ones(128, dtype=np.int8),
        device_id=1,
        device_ticks_us=10000,
        source_ip="192.0.2.1",
    )

    class Receiver:
        effective_socket_rcvbuf_bytes = None
        calls = 0
        stopped = False

        def add_callback(self, callback):
            self.callback = callback

        def run(self, **_kwargs):
            self.calls += 1
            assert self.calls <= 2, "collection exceeded its deadline without new packets"
            if self.calls == 1:
                # Waiting for the first packet does not consume recording time.
                clock[0] = 20.0
                self.callback(packet)
            else:
                clock[0] += 1.0

        def stop(self):
            self.stopped = True

    class Writer:
        def __init__(self, **_kwargs):
            pass

        def save_samples_by_device(self, packets):
            saved_packets.extend(packets)
            return [tmp_path / "capture.npz"]

    receiver = Receiver()
    generator_stops = []
    generator = SimpleNamespace(stop=lambda: generator_stops.append(True))
    monkeypatch.setattr(host, "_prepare_raw_http_collection", lambda *_args: (receiver, generator, 5555))
    monkeypatch.setattr(host, "_start_raw_http_collection", lambda *_args: None)
    monkeypatch.setattr(csi_io, "CSICollector", Writer)
    args = build_parser().parse_args([
        "collect", "--target", "192.0.2.1", "--duration", "1",
        "--ready-stable-seconds", "0",
    ])
    args.label = label
    args.direct_endpoint = "http://192.0.2.1:8080/espectre/v1"
    args.traffic_target = "192.0.2.1"

    host._run_live_collect(args)

    assert receiver.calls == 2
    assert receiver.stopped
    assert generator_stops
    assert saved_packets == ([packet] if label else [])


def test_discovery_frontends_exclude_streamer() -> None:
    assert device_discovery.SUPPORTED_DISCOVERY_FRONTENDS == ("native", "esphome", "matter", "micro")


def discovery_info(**overrides):
    properties = {
        b"device_id": b"0x1234",
        b"frontend": b"native",
        b"transport": b"http",
        b"path": b"/espectre/v1",
        b"txtvers": device_discovery.DNS_SD_TXT_SCHEMA_VERSION.encode(),
        b"protovers": device_discovery.PROTOCOL_VERSION.encode(),
        b"capabilities": b"sensing, motion, ,csi",
    }
    properties.update(overrides.pop("properties", {}))
    return SimpleNamespace(
        properties=properties,
        port=overrides.pop("port", ESPECTRE_DIRECT_PORT),
        parsed_addresses=lambda _version: overrides.get("addresses", ["192.168.1.23"]),
    )


@pytest.mark.parametrize("frontend", device_discovery.SUPPORTED_DISCOVERY_FRONTENDS)
@pytest.mark.parametrize("port", [80, ESPECTRE_DIRECT_PORT])
def test_discovery_parses_canonical_records(frontend, port):
    info = discovery_info(port=port, properties={b"frontend": frontend})
    record = device_discovery._parse_record(ESPECTRE_SERVICE_TYPE, "sensor._espectre._tcp.local.", info)

    assert record.frontend == frontend
    assert record.device_id == 0x1234
    assert record.display_id == "0000000000001234"
    assert record.target_port == port
    assert record.name == "sensor"
    assert record.chip == "unknown"
    authority = "192.168.1.23" if port == 80 else f"192.168.1.23:{port}"
    assert record.endpoint == f"http://{authority}/espectre/v1"
    assert record.events_endpoint == f"http://{authority}/espectre/v1/events"
    serialized = record.as_serializable_dict()
    assert serialized["device_id"] == record.display_id
    assert serialized["capabilities"] == ["sensing", "motion", "csi"]
    assert serialized["metadata"] == {}


@pytest.mark.parametrize("properties", [
    {b"device_id": None}, {b"device_id": b"invalid"}, {b"device_id": b"0"},
    {b"device_id": b"10000000000000000"}, {b"frontend": b"unknown"},
    {b"transport": b"mqtt"}, {b"path": b"/other"},
    {b"txtvers": b"unsupported"}, {b"protovers": b"unsupported"},
])
def test_discovery_rejects_incompatible_record_metadata(properties):
    assert device_discovery._parse_record(
        ESPECTRE_SERVICE_TYPE, "sensor._espectre._tcp.local.", discovery_info(properties=properties)
    ) is None


@pytest.mark.parametrize("service_type,overrides", [
    ("_http._tcp.local.", {}), (ESPECTRE_SERVICE_TYPE, {"addresses": []}),
    (ESPECTRE_SERVICE_TYPE, {"port": 0}), (ESPECTRE_SERVICE_TYPE, {"port": 65536}),
])
def test_discovery_rejects_unusable_service_endpoints(service_type, overrides):
    assert device_discovery._parse_record(service_type, "sensor", discovery_info(**overrides)) is None


def test_discovery_listener_updates_filters_and_removes_records():
    records = {"native": discovery_info(), "micro": discovery_info(properties={b"frontend": b"micro"})}
    zeroconf = SimpleNamespace(get_service_info=lambda _type, name, **_kwargs: records.get(name))
    listener = device_discovery._DeviceListener(zeroconf)
    for name in ["native", "micro", "missing", "native"]:
        listener.add_service(zeroconf, ESPECTRE_SERVICE_TYPE, name)
    assert [record.frontend for record in listener.snapshot()] == ["micro", "native"]
    assert len(listener.snapshot("native")) == 1
    records["native"] = discovery_info(properties={b"name": b"Renamed sensor", b"chip": b"c3"})
    listener.update_service(zeroconf, ESPECTRE_SERVICE_TYPE, "native")
    assert listener.snapshot("native")[0].name == "Renamed sensor"
    for name in ["native", "native"]:
        listener.remove_service(zeroconf, ESPECTRE_SERVICE_TYPE, name)
    assert listener.snapshot("native") == []
    assert len(listener.snapshot()) == 1


@pytest.mark.parametrize("has_record", [False, True])
def test_discovery_wait_uses_deadline_or_quiet_window(monkeypatch, has_record):
    clock = [0.0]
    waits = []
    monkeypatch.setattr(device_discovery.time, "monotonic", lambda: clock[0])
    zeroconf = SimpleNamespace(get_service_info=lambda *_args, **_kwargs: discovery_info())
    listener = device_discovery._DeviceListener(zeroconf)
    if has_record:
        listener.add_service(zeroconf, ESPECTRE_SERVICE_TYPE, "sensor")

    def wait(duration):
        waits.append(duration)
        clock[0] += duration

    monkeypatch.setattr(listener._records_changed, "wait", wait)
    listener.wait_for_quiet(2.0, 0.25)
    assert waits == [0.25 if has_record else 2.0]


@pytest.mark.parametrize("browser_fails", [False, True])
def test_discovery_closes_resources_after_browse(monkeypatch, browser_fails):
    events = []
    zeroconf = SimpleNamespace(close=lambda: events.append("close"),
        get_service_info=lambda *_args, **_kwargs: discovery_info())
    monkeypatch.setattr(device_discovery, "Zeroconf", lambda **_kwargs: zeroconf)
    monkeypatch.setattr(device_discovery._DeviceListener, "wait_for_quiet", lambda *_args: None)

    def browser(_zeroconf, service_type, listener):
        assert service_type == ESPECTRE_SERVICE_TYPE
        if browser_fails:
            raise OSError("browse failed")
        listener.add_service(zeroconf, service_type, "sensor")
        return SimpleNamespace(cancel=lambda: events.append("cancel"))

    monkeypatch.setattr(device_discovery, "ServiceBrowser", browser)
    if browser_fails:
        with pytest.raises(OSError):
            device_discovery.discover_devices()
        assert events == ["close"]
    else:
        result = device_discovery.discover_devices(frontend="native")
        assert len(result) == 1
        assert result[0].device_id == 0x1234
        assert events == ["cancel", "close"]


@pytest.mark.parametrize("options", [
    {"frontend": "unknown"}, {"timeout_s": 0}, {"timeout_s": float("nan")},
    {"quiet_window_s": -1}, {"quiet_window_s": float("inf")},
])
def test_discovery_validates_options_before_opening_network(monkeypatch, options):
    opened = []
    monkeypatch.setattr(device_discovery, "Zeroconf", lambda **_kwargs: opened.append(True))
    with pytest.raises(ValueError):
        device_discovery.discover_devices(**options)
    assert opened == []


def test_discovery_reports_unavailable_dependency_or_interfaces(monkeypatch):
    def unavailable(**_kwargs):
        raise OSError("no multicast interface")

    monkeypatch.setattr(device_discovery, "Zeroconf", unavailable)
    with pytest.raises(device_discovery.DeviceDiscoveryError) as error:
        device_discovery.discover_devices()
    assert isinstance(error.value.__cause__, OSError)
    monkeypatch.setattr(device_discovery, "Zeroconf", None)
    with pytest.raises(device_discovery.DeviceDiscoveryError):
        device_discovery.discover_devices()


def test_discovery_requires_explicit_selection_for_ambiguous_devices(monkeypatch):
    records = [discovered_device(device_id=1), discovered_device(device_id=2)]
    with pytest.raises(device_discovery.DeviceDiscoveryError):
        device_discovery.select_discovered_device(records, interactive=False)
    with pytest.raises(device_discovery.DeviceDiscoveryError):
        device_discovery.select_discovered_device(records, chip="s3")
    choices = iter(["", "invalid", "0", "3", "2"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))
    assert device_discovery.select_discovered_device(records, frontend_label="native") == records[1]


def test_collect_explicit_esphome_target_uses_shared_direct_port(monkeypatch) -> None:
    monkeypatch.setattr(host.socket, "gethostbyname", lambda _host: "192.168.1.23")
    monkeypatch.setattr(host, "discover_devices", lambda **_kwargs: [])
    args = collect_args(target="espectre.local", frontend="esphome")

    host._resolve_collect_target_via_discovery(args)

    assert args.direct_endpoint == f"http://espectre.local:{ESPECTRE_DIRECT_PORT}/espectre/v1"
    assert args.traffic_target == "192.168.1.23"
    assert args.expected_discovery_device_id is None


def test_collect_explicit_endpoint_preserves_nondefault_direct_port(monkeypatch) -> None:
    monkeypatch.setattr(host.socket, "gethostbyname", lambda _host: "192.168.1.23")
    args = collect_args(target="http://espectre.local:61443/espectre/v1")

    host._resolve_collect_target_via_discovery(args)

    assert args.direct_endpoint == "http://espectre.local:61443/espectre/v1"
    assert args.traffic_target == "192.168.1.23"


def test_collect_bare_hostname_uses_discovered_esphome_port(monkeypatch) -> None:
    selected = discovered_device(frontend="esphome", port=ESPECTRE_DIRECT_PORT)
    monkeypatch.setattr(host.socket, "gethostbyname", lambda _host: selected.ip_address)
    monkeypatch.setattr(host, "discover_devices", lambda **_kwargs: [selected])
    args = collect_args(target="espectre.local")

    host._resolve_collect_target_via_discovery(args)

    assert args.direct_endpoint == selected.endpoint
    assert args.target_frontend == "esphome"
    assert args.expected_discovery_device_id == selected.device_id


def test_collect_discovers_only_raw_capable_direct_devices(monkeypatch) -> None:
    raw = discovered_device(frontend="matter", device_id=0x1234)
    no_raw = discovered_device(frontend="native", device_id=0x5678, capabilities=("sensing", "motion"))
    monkeypatch.setattr(host, "discover_devices", lambda **_kwargs: [no_raw, raw])
    args = collect_args(target=None)

    host._resolve_collect_target_via_discovery(args)

    assert args.direct_endpoint == raw.endpoint
    assert args.traffic_target == raw.ip_address
    assert args.target_frontend == "matter"
    assert args.expected_discovery_device_id == raw.device_id


def test_collect_resolves_full_device_id_through_direct_discovery(monkeypatch) -> None:
    selected = discovered_device(device_id=0x1122334455667788)
    other = discovered_device(device_id=0x8877665544332211)
    monkeypatch.setattr(host, "discover_devices", lambda **_kwargs: [other, selected])
    args = collect_args(target="1122334455667788")

    host._resolve_collect_target_via_discovery(args)

    assert args.direct_endpoint == selected.endpoint
    assert args.expected_discovery_device_id == selected.device_id


def test_prepare_raw_collection_persists_external_before_constructing_data_plane() -> None:
    calls: list[tuple[str, object]] = []

    class FakeControl:
        def __init__(self, endpoint):
            calls.append(("open", endpoint))

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            calls.append(("close", None))

        def request(self, verb, resource, data=None):
            calls.append((verb, resource, data))
            if resource == "capabilities":
                return {
                    "csi": {
                        "protocol_version": 1,
                        "traffic_udp_port": 6123,
                        "marker": ExternalTrafficGenerator.TRAFFIC_MARKER,
                    }
                }
            if resource == "sensing":
                return {
                    "csi_traffic_mode": "external",
                    "csi_traffic_udp_port": 6123,
                }
            return {}

    class FakeReceiver:
        def __init__(self, endpoint, **kwargs):
            self.endpoint = endpoint
            self.kwargs = kwargs

    class FakeGenerator:
        TRAFFIC_MARKER = ExternalTrafficGenerator.TRAFFIC_MARKER

        def __init__(self, targets, **kwargs):
            self.targets = targets
            self.kwargs = kwargs

    args = SimpleNamespace(
        direct_endpoint="http://192.168.1.23/espectre/v1",
        traffic_target="192.168.1.23",
        source_ip="192.168.1.8",
        pps=400,
    )

    receiver, generator, port = host._prepare_raw_http_collection(
        args, FakeControl, FakeReceiver, FakeGenerator)

    assert calls == [
        ("open", args.direct_endpoint),
        ("get", "capabilities", None),
        ("patch", "sensing", {"csi_traffic_mode": "external"}),
        ("get", "sensing", None),
        ("close", None),
    ]
    assert receiver.endpoint == args.direct_endpoint
    assert generator.targets == [args.traffic_target]
    assert generator.kwargs == {"port": 6123, "rate_pps": 400.0, "source_ip": "192.168.1.8"}
    assert port == 6123


def test_prepare_raw_collection_rejects_unconfirmed_persistent_mode() -> None:
    class FakeControl:
        def __init__(self, _endpoint):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def request(self, verb, resource, _data=None):
            assert verb in {"get", "patch"}
            if resource == "capabilities":
                return {
                    "csi": {
                        "protocol_version": 1,
                        "marker": ExternalTrafficGenerator.TRAFFIC_MARKER,
                    }
                }
            if resource == "sensing" and verb == "get":
                return {"csi_traffic_mode": "internal"}
            return {}

    args = SimpleNamespace(
        direct_endpoint="http://192.168.1.23/espectre/v1",
        traffic_target="192.168.1.23",
        source_ip=None,
        pps=100,
    )
    with pytest.raises(RuntimeError, match="did not persist"):
        host._prepare_raw_http_collection(args, FakeControl, object, ExternalTrafficGenerator)


def test_prepare_raw_collection_rejects_incompatible_protocol_version() -> None:
    class FakeControl:
        def __init__(self, _endpoint):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def request(self, verb, resource, _data=None):
            assert (verb, resource) == ("get", "capabilities")
            return {"csi": {"protocol_version": 2}}

    args = SimpleNamespace(
        direct_endpoint="http://192.168.1.23/espectre/v1",
        traffic_target="192.168.1.23",
        source_ip=None,
        pps=100,
    )
    with pytest.raises(RuntimeError, match="raw HTTP v1"):
        host._prepare_raw_http_collection(args, FakeControl, object, object)


@pytest.mark.parametrize("raw_capability", [
    {"protocol_version": 1, "marker": "."},
    {"protocol_version": 1, "traffic_marker": "👻"},
])
def test_prepare_raw_collection_rejects_noncanonical_marker(raw_capability) -> None:
    class FakeControl:
        def __init__(self, _endpoint):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def request(self, verb, resource, _data=None):
            assert (verb, resource) == ("get", "capabilities")
            return {"csi": raw_capability}

    args = SimpleNamespace(
        direct_endpoint="http://192.168.1.23/espectre/v1",
        traffic_target="192.168.1.23",
        source_ip=None,
        pps=100,
    )
    with pytest.raises(RuntimeError, match="canonical external traffic marker"):
        host._prepare_raw_http_collection(args, FakeControl, object, ExternalTrafficGenerator)


def test_start_raw_collection_opens_session_then_starts_traffic_before_http_bind() -> None:
    calls: list[str] = []

    class FakeReceiver:
        def start_session(self):
            calls.append("session")

        def bind_stream(self):
            calls.append("bind")

        def stop(self):
            calls.append("receiver_stop")

    class FakeGenerator:
        def start(self):
            calls.append("generator")

        def stop(self):
            calls.append("generator_stop")

    host._start_raw_http_collection(FakeReceiver(), FakeGenerator())

    assert calls == ["session", "generator", "bind"]


def test_start_raw_collection_stops_traffic_when_http_bind_fails() -> None:
    calls: list[str] = []

    class FailingReceiver:
        def start_session(self):
            calls.append("session")

        def bind_stream(self):
            calls.append("bind")
            raise TimeoutError("bind failed")

        def stop(self):
            calls.append("receiver_stop")

    class FakeGenerator:
        def start(self):
            calls.append("generator")

        def stop(self):
            calls.append("generator_stop")

    with pytest.raises(TimeoutError, match="bind failed"):
        host._start_raw_http_collection(FailingReceiver(), FakeGenerator())

    assert calls == ["session", "generator", "bind", "generator_stop", "receiver_stop"]
