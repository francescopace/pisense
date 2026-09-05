# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - CLI Wrapper Tests

Tests for host-side ESPectre CLI wrapper modules.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import threading
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from espectre_cli import (
    app,
    build_artifacts,
    common,
    device_control,
    device_discovery,
    esphome,
    esptool_runner,
    idf,
    idf_container,
    micro,
    mqtt_shell,
    serial_monitor,
    targets,
)


def _mqtt_args() -> argparse.Namespace:
    return argparse.Namespace(
        broker="broker.local",
        port_mqtt=1884,
        topic_prefix="espectre/v1/devices",
        device_id="0x0000111122223333",
        username="user",
        password="pass",
    )


def test_build_mqtt_namespace_maps_cli_fields() -> None:
    namespace = common.build_mqtt_namespace(_mqtt_args())

    assert namespace.broker == "broker.local"
    assert namespace.port == 1884
    assert namespace.topic_prefix == "espectre/v1/devices"
    assert namespace.device_id == "0x0000111122223333"
    assert namespace.username == "user"
    assert namespace.password == "pass"


def test_cli_command_uses_platform_launcher(monkeypatch) -> None:
    monkeypatch.setattr(common.os, "name", "posix", raising=False)
    assert common.cli_command("micro", "deploy") == "./espectre micro deploy"
    assert common.copy_config_command() == "cp src/python/micro_espectre/config_local.py.example src/python/micro_espectre/config_local.py"
    assert common.serial_port_example() == "/dev/cu.usbmodemXXXX"

    monkeypatch.setattr(common.os, "name", "nt", raising=False)
    assert common.cli_command("micro", "deploy") == r".\espectre.cmd micro deploy"
    assert common.copy_config_command() == r"copy src\python\micro_espectre\config_local.py.example src\python\micro_espectre\config_local.py"
    assert common.serial_port_example() == "COM5"


def test_add_mqtt_connection_args_uses_environment_defaults(monkeypatch) -> None:
    monkeypatch.setenv("MQTT_BROKER", "mqtt.local")
    monkeypatch.setenv("MQTT_PORT", "2883")
    monkeypatch.setenv("MQTT_TOPIC_PREFIX", "custom/topic")
    monkeypatch.setenv("MQTT_USERNAME", "env-user")
    monkeypatch.setenv("MQTT_PASSWORD", "env-pass")

    parser = argparse.ArgumentParser()
    common.add_mqtt_connection_args(parser)
    args = parser.parse_args([])

    assert args.broker == "mqtt.local"
    assert args.port_mqtt == 2883
    assert args.topic_prefix == "custom/topic"
    assert args.device_id is None
    assert args.username == "env-user"
    assert args.password == "env-pass"

def test_detect_serial_ports_filters_usb_like_devices(monkeypatch) -> None:
    fake_serial = ModuleType("serial")
    fake_tools = ModuleType("serial.tools")
    fake_list_ports = ModuleType("serial.tools.list_ports")
    fake_list_ports.comports = lambda: [
        SimpleNamespace(device="/dev/cu.usbmodem1", description="USB Serial Device", vid=None),
        SimpleNamespace(device="/dev/cu.Bluetooth-Incoming-Port", description="Bluetooth", vid=None),
        SimpleNamespace(device="/dev/cu.usbserial2", description="FTDI UART", vid=None),
        SimpleNamespace(
            device="/dev/cu.espressif",
            description="Espressif Device",
            vid=common.ESPRESSIF_USB_VENDOR_ID,
        ),
    ]
    fake_tools.list_ports = fake_list_ports
    fake_serial.tools = fake_tools

    monkeypatch.setitem(sys.modules, "serial", fake_serial)
    monkeypatch.setitem(sys.modules, "serial.tools", fake_tools)
    monkeypatch.setitem(sys.modules, "serial.tools.list_ports", fake_list_ports)

    assert common.detect_serial_ports() == [
        "/dev/cu.usbmodem1",
        "/dev/cu.usbserial2",
        "/dev/cu.espressif",
    ]


def test_build_artifact_metadata_reports_exact_file(tmp_path: Path) -> None:
    artifact = tmp_path / "firmware.bin"
    artifact.write_bytes(b"firmware")

    metadata = build_artifacts.build_artifact_metadata(
        frontend="native",
        chip="s3",
        artifact=artifact,
    )

    assert metadata["artifact"] == str(artifact.resolve())
    assert metadata["command"] == "build"
    assert metadata["firmware_size_bytes"] == 8
    assert len(metadata["firmware_sha256"]) == 64


def test_build_artifact_metadata_rejects_missing_file(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="firmware build artifact not found"):
        build_artifacts.build_artifact_metadata(
            frontend="native",
            chip="s3",
            artifact=tmp_path / "missing.bin",
        )


def test_discovered_device_selection_filters_chip_before_ambiguity() -> None:
    def record(chip: str, address: str) -> device_discovery.DiscoveredDevice:
        return device_discovery.DiscoveredDevice(
            service_name=f"{chip}._espectre._tcp.local.",
            service_type=device_discovery.ESPECTRE_SERVICE_TYPE,
            frontend="native",
            device_id=1,
            device_id_text="0000000000000001",
            name=chip,
            chip=chip,
            ip_address=address,
            port=device_discovery.ESPECTRE_DIRECT_PORT,
            transport="direct-http",
            endpoint=f"http://{address}:62587/espectre/v1",
            protocol="1.0",
        )

    c3 = record("esp32c3", "192.0.2.10")
    s3 = record("esp32-s3", "192.0.2.11")

    assert device_discovery.select_discovered_device(
        [c3, s3],
        chip="s3",
        interactive=False,
    ) is s3


def test_get_serial_port_returns_compatible_explicit_argument(monkeypatch) -> None:
    monkeypatch.setattr(
        common,
        "compatible_serial_ports",
        lambda **_kwargs: ["/dev/cu.explicit"],
    )

    assert common.get_serial_port("/dev/cu.explicit") == "/dev/cu.explicit"


def test_matter_onboarding_json_is_machine_readable(monkeypatch, capsys) -> None:
    fake_serial = ModuleType("serial")
    reset_calls: list[tuple[str, str, str]] = []

    class FakeConnection:
        def __init__(self, *_args, **_kwargs):
            self.lines = iter(
                [
                    b"I app: MATTER_QR=MT:TESTPAYLOAD\n",
                    b"I app: MATTER_MANUAL_CODE=12704227053\n",
                ]
            )

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def readline(self):
            return next(self.lines, b"")

        def close(self):
            return None

    fake_serial.Serial = FakeConnection
    fake_serial.SerialException = OSError
    monkeypatch.setitem(sys.modules, "serial", fake_serial)
    monkeypatch.setattr(idf, "resolve_serial_port", lambda port, **_kwargs: port)
    monkeypatch.setattr(
        idf,
        "run_firmware",
        lambda *, chip, idf_target, port: reset_calls.append(
            (chip, idf_target, port)
        ),
    )

    assert idf.read_matter_onboarding(
        "/dev/cu.test",
        chip="s3",
        json_output=True,
    )
    event = json.loads(capsys.readouterr().out.splitlines()[-1])
    assert event == {
        "chip": "s3",
        "event": "matter_onboarding",
        "frontend": "matter",
        "manual_code": "12704227053",
        "port": "/dev/cu.test",
        "qr_payload": "MT:TESTPAYLOAD",
    }
    assert reset_calls == [("s3", "esp32s3", "/dev/cu.test")]


def test_matter_onboarding_can_read_current_boot_without_reset(monkeypatch) -> None:
    fake_serial = ModuleType("serial")
    reset_calls: list[tuple[str, str, str]] = []

    class FakeConnection:
        def __init__(self, *_args, **_kwargs):
            self.lines = iter(
                [
                    b"MATTER_QR=MT:TESTPAYLOAD\n",
                    b"MATTER_MANUAL_CODE=12704227053\n",
                ]
            )

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def readline(self):
            return next(self.lines, b"")

        def close(self):
            return None

    fake_serial.Serial = FakeConnection
    fake_serial.SerialException = OSError
    monkeypatch.setitem(sys.modules, "serial", fake_serial)
    monkeypatch.setattr(idf, "resolve_serial_port", lambda port, **_kwargs: port)
    monkeypatch.setattr(
        idf,
        "run_firmware",
        lambda *, chip, idf_target, port: reset_calls.append(
            (chip, idf_target, port)
        ),
    )

    assert idf.read_matter_onboarding("/dev/cu.test", reset=False)
    assert reset_calls == []


def test_matter_onboarding_does_not_retry_serial_failure(monkeypatch) -> None:
    fake_serial = ModuleType("serial")
    opened: list[str] = []

    class FakeConnection:
        def __init__(self, port, **_kwargs):
            opened.append(port)

        def readline(self):
            raise OSError("device disconnected")

        def close(self):
            return None

    fake_serial.Serial = FakeConnection
    fake_serial.SerialException = OSError
    monkeypatch.setitem(sys.modules, "serial", fake_serial)
    resolutions: list[str] = []
    monkeypatch.setattr(
        idf,
        "resolve_serial_port",
        lambda port, **_kwargs: resolutions.append(port) or port,
    )

    assert not idf.read_matter_onboarding("/dev/cu.loader", reset=False)
    assert opened == ["/dev/cu.loader"]
    assert resolutions == ["/dev/cu.loader"]


def test_micro_run_json_emits_direct_ready_event(monkeypatch, capsys) -> None:
    class FakeProcess:
        stdout = iter(
            [
                "[INFO] WiFi connected - IP: 192.0.2.10, Protocol: 802.11n, "
                "Bandwidth: 20MHz\n"
            ]
        )

        @staticmethod
        def wait():
            return 0

    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda *_args, **_kwargs: "/dev/cu.test")
    monkeypatch.setattr(micro.subprocess, "Popen", lambda *_args, **_kwargs: FakeProcess())

    micro.run_application(
        argparse.Namespace(port=None, chip="s3", json=True)
    )

    events = []
    for line in capsys.readouterr().out.splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            events.append(value)
    assert events == [
        {
            "chip": "s3",
            "endpoint": "http://192.0.2.10:62587/espectre/v1",
            "event": "direct_ready",
            "frontend": "micro",
            "port": "/dev/cu.test",
        }
    ]


def test_get_serial_port_auto_detects_single_port(monkeypatch) -> None:
    monkeypatch.setattr(common, "detect_serial_ports", lambda: ["/dev/cu.single"])

    assert common.get_serial_port(None) == "/dev/cu.single"


def test_get_serial_port_prompts_for_multiple_ports(monkeypatch) -> None:
    monkeypatch.setattr(common, "detect_serial_ports", lambda: ["/dev/cu.a", "/dev/cu.b"])
    monkeypatch.setattr("builtins.input", lambda _prompt: "2")

    assert common.get_serial_port(None) == "/dev/cu.b"


def test_get_serial_port_rejects_invalid_selection(monkeypatch) -> None:
    monkeypatch.setattr(common, "detect_serial_ports", lambda: ["/dev/cu.a", "/dev/cu.b"])
    monkeypatch.setattr("builtins.input", lambda _prompt: "9")

    with pytest.raises(SystemExit):
        common.get_serial_port(None)


@pytest.mark.parametrize(
    ("chip", "console"),
    [
        ("esp32", "uart"),
        ("s2", "usb_cdc"),
        ("c3", "usb_serial_jtag"),
        ("c5", "usb_serial_jtag"),
        ("c6", "usb_serial_jtag"),
        ("s3", "usb_serial_jtag"),
    ],
)
def test_native_console_matches_chip_transport(chip: str, console: str) -> None:
    assert common.NATIVE_CONSOLE_BY_CHIP[chip] == console
    assert common.serial_console_mode(chip) == console


def test_format_serial_candidate_includes_chip_and_console() -> None:
    candidate = common.SerialCandidate("/dev/cu.usbmodem1", "c6", "usb_serial_jtag")

    assert common.format_serial_candidate(candidate) == "/dev/cu.usbmodem1  ESP32-C6  usb_serial_jtag"


def test_resolve_serial_port_rejects_explicit_incompatible_port(monkeypatch) -> None:
    monkeypatch.setattr(
        common,
        "compatible_serial_ports",
        lambda **_kwargs: ["/dev/cu.usb-jtag"],
    )
    with pytest.raises(SystemExit):
        common.resolve_serial_port(
            "/dev/cu.bridge",
            chip="s3",
            frontend="native",
            purpose="improv",
        )


@pytest.mark.parametrize("port", [None, "/dev/cu.usbmodem01"])
def test_resolve_serial_port_waits_for_port_reenumeration(monkeypatch, port) -> None:
    attempts = iter([[], ["/dev/cu.usbmodem01"]])
    clock = [0.0]
    monkeypatch.setattr(
        common,
        "compatible_serial_ports",
        lambda **_kwargs: next(attempts),
    )
    monkeypatch.setattr(common.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(
        common.time,
        "sleep",
        lambda duration: clock.__setitem__(0, clock[0] + duration),
    )

    assert common.resolve_serial_port(
        port,
        chip="s2",
        frontend="native",
        purpose="improv",
        wait_timeout_s=10.0,
    ) == "/dev/cu.usbmodem01"
    assert clock[0] == pytest.approx(0.1)


@pytest.mark.parametrize("port", [None, "/dev/cu.usbmodem01"])
def test_resolve_serial_port_stops_waiting_at_deadline(monkeypatch, port) -> None:
    clock = [0.0]
    monkeypatch.setattr(common, "compatible_serial_ports", lambda **_kwargs: [])
    monkeypatch.setattr(common.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(common.time, "sleep", lambda seconds: clock.__setitem__(0, clock[0] + seconds))

    with pytest.raises(SystemExit) as exc:
        common.resolve_serial_port(
            port, chip="s2", frontend="micro", purpose="deploy", wait_timeout_s=0.25,
        )

    assert exc.value.code == 1
    assert clock[0] == pytest.approx(0.25)


def test_resolve_serial_port_uses_metadata_without_opening_devices(monkeypatch) -> None:
    fake_serial = ModuleType("serial")
    fake_tools = ModuleType("serial.tools")
    fake_list_ports = ModuleType("serial.tools.list_ports")
    fake_list_ports.comports = lambda: [
        SimpleNamespace(
            device="/dev/cu.native",
            description="USB JTAG/serial debug unit",
            product="USB JTAG/serial debug unit",
            interface=None,
            manufacturer="Espressif",
            vid=common.ESPRESSIF_USB_VENDOR_ID,
        )
    ]
    fake_tools.list_ports = fake_list_ports
    fake_serial.tools = fake_tools
    fake_serial.Serial = lambda *_args, **_kwargs: pytest.fail("serial port was opened")
    monkeypatch.setitem(sys.modules, "serial", fake_serial)
    monkeypatch.setitem(sys.modules, "serial.tools", fake_tools)
    monkeypatch.setitem(sys.modules, "serial.tools.list_ports", fake_list_ports)

    assert common.resolve_serial_port(
        None,
        chip="s3",
        frontend="native",
        purpose="flash",
    ) == "/dev/cu.native"


def test_compatible_serial_ports_keeps_uart_bridge_for_flash(monkeypatch) -> None:
    monkeypatch.setattr(
        common,
        "detect_serial_ports",
        lambda: ["/dev/cu.SLAB_USBtoUART"],
    )

    assert common.compatible_serial_ports(
        chip="c3",
        frontend="native",
        purpose="flash",
    ) == ["/dev/cu.SLAB_USBtoUART"]


def test_get_serial_port_forwards_operation_to_shared_resolver(monkeypatch) -> None:
    observed = []
    monkeypatch.setattr(
        common,
        "resolve_serial_port",
        lambda port_arg, **kwargs: observed.append((port_arg, kwargs))
        or "/dev/cu.c6",
    )

    assert common.get_serial_port(
        None,
        chip="c6",
        frontend="micro",
        purpose="deploy",
    ) == "/dev/cu.c6"
    assert observed == [
        (
            None,
            {
                "chip": "c6",
                "frontend": "micro",
                "purpose": "deploy",
                "wait_timeout_s": 0.0,
            },
        ),
    ]


def test_improv_provision_json_reports_selected_port(monkeypatch, capsys) -> None:
    observed = []

    class FakeImprovClient:
        def __init__(self, port):
            observed.append(("port", port))

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def provision(self, ssid, password, *, timeout):
            observed.append(("provision", ssid, password, timeout))
            return SimpleNamespace(
                endpoint="http://192.0.2.5",
                device_info=("ESPectre", "1.0", "s3", "Native"),
                states=("ready", "provisioned"),
            )

    monkeypatch.setenv("TEST_ESPECTRE_WIFI_PASSWORD", "secret")
    monkeypatch.setattr(
        device_control,
        "resolve_serial_port",
        lambda port, **_kwargs: port or "/dev/cu.valid",
    )
    monkeypatch.setattr(device_control, "ImprovSerialClient", FakeImprovClient)

    result = device_control.run_improv_provision_command(
        argparse.Namespace(
            port=None,
            chip="s3",
            frontend="native",
            ssid="lab",
            password_env="TEST_ESPECTRE_WIFI_PASSWORD",
            timeout=60.0,
            json=True,
        )
    )

    assert result == 0
    assert json.loads(capsys.readouterr().out) == {
        "chip": "s3",
        "device_info": ["ESPectre", "1.0", "s3", "Native"],
        "endpoint": "http://192.0.2.5/espectre/v1",
        "frontend": "native",
        "port": "/dev/cu.valid",
        "states": ["ready", "provisioned"],
    }
    assert observed == [
        ("port", "/dev/cu.valid"),
        ("provision", "lab", "secret", 60.0),
    ]


def test_s2_improv_provision_waits_for_usb_reenumeration(monkeypatch) -> None:
    observed = []

    def capture_port(port, **kwargs):
        observed.append((port, kwargs))
        raise SystemExit(1)

    monkeypatch.setattr(device_control, "resolve_serial_port", capture_port)

    with pytest.raises(SystemExit):
        device_control.run_improv_provision_command(
            argparse.Namespace(
                port="/dev/cu.usbmodem01",
                chip="s2",
                frontend="native",
                ssid="lab",
                password_env="TEST_ESPECTRE_WIFI_PASSWORD",
                timeout=60.0,
                json=False,
            )
        )

    assert observed == [
        (
            "/dev/cu.usbmodem01",
            {
                "chip": "s2",
                "frontend": "native",
                "purpose": "improv",
                "wait_timeout_s": 10.0,
            },
        )
    ]


def test_improv_provision_validates_port_before_requesting_password(monkeypatch) -> None:
    def reject_port(*_args, **_kwargs):
        raise SystemExit(1)

    monkeypatch.delenv("TEST_ESPECTRE_WIFI_PASSWORD", raising=False)
    monkeypatch.setattr(
        device_control,
        "resolve_serial_port",
        reject_port,
    )
    monkeypatch.setattr(
        device_control.getpass,
        "getpass",
        lambda _prompt: pytest.fail(
            "password must not be requested before port validation"
        ),
    )

    with pytest.raises(SystemExit):
        device_control.run_improv_provision_command(
            argparse.Namespace(
                port=None,
                chip=None,
                frontend="native",
                ssid="lab",
                password_env="TEST_ESPECTRE_WIFI_PASSWORD",
                timeout=60.0,
                json=False,
            )
        )


def test_improv_provision_prompts_and_prints_text_result(monkeypatch, capsys) -> None:
    class FakeImprovClient:
        def __init__(self, _port):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def provision(self, _ssid, password, *, timeout):
            assert password == "prompted-secret"
            assert timeout == 30.0
            return SimpleNamespace(
                endpoint="http://192.0.2.10",
                device_info=(),
                states=(),
            )

    monkeypatch.delenv("TEST_ESPECTRE_WIFI_PASSWORD", raising=False)
    monkeypatch.setattr(
        device_control,
        "resolve_serial_port",
        lambda *_args, **_kwargs: "/dev/cu.valid",
    )
    monkeypatch.setattr(device_control.getpass, "getpass", lambda _prompt: "prompted-secret")
    monkeypatch.setattr(device_control, "ImprovSerialClient", FakeImprovClient)

    result = device_control.run_improv_provision_command(
        argparse.Namespace(
            port=None,
            chip=None,
            frontend="native",
            ssid="lab",
            password_env="TEST_ESPECTRE_WIFI_PASSWORD",
            timeout=30.0,
            json=False,
        )
    )

    assert result == 0
    assert capsys.readouterr().out.splitlines() == [
        "Improv provisioning completed.",
        "Device endpoint: http://192.0.2.10/espectre/v1",
    ]


def test_resolve_direct_endpoint_supports_explicit_and_discovered_devices(
    monkeypatch,
) -> None:
    observed = []
    records = [SimpleNamespace(endpoint="http://192.0.2.20/espectre/v1")]
    monkeypatch.setattr(
        device_control,
        "discover_devices",
        lambda **kwargs: observed.append(("discover", kwargs)) or records,
    )
    monkeypatch.setattr(
        device_control,
        "select_discovered_device",
        lambda found, **kwargs: observed.append(("select", found, kwargs)) or records[0],
    )

    explicit = argparse.Namespace(endpoint="http://192.0.2.19", chip=None)
    assert device_control._resolve_direct_endpoint(explicit) == (
        "http://192.0.2.19/espectre/v1"
    )
    with pytest.raises(ValueError, match="--chip requires --frontend discovery"):
        device_control._resolve_direct_endpoint(
            argparse.Namespace(endpoint="http://192.0.2.19", chip="c6")
        )

    discovered = argparse.Namespace(
        endpoint=None,
        chip="c6",
        frontend="native",
        discovery_timeout=2.5,
    )
    assert device_control._resolve_direct_endpoint(discovered) == records[0].endpoint
    assert observed == [
        ("discover", {"frontend": "native", "timeout_s": 2.5}),
        (
            "select",
            records,
            {"frontend_label": "native", "chip": "c6"},
        ),
    ]


@pytest.mark.parametrize(
    ("http_method", "resource", "expected_negotiate"),
    [("get", "capabilities", False), ("post", "config", True)],
)
def test_direct_request_negotiates_only_when_required(
    monkeypatch, capsys, http_method, resource, expected_negotiate
) -> None:
    events = []

    class FakeDirectClient:
        def __init__(self, endpoint, *, origin, timeout):
            events.append(("open", endpoint, origin, timeout))

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def negotiate(self):
            events.append("negotiate")

        def request(self, method, requested_resource, params):
            events.append(("request", method, requested_resource, params))
            return {"ok": True}

    monkeypatch.setattr(device_control, "DirectClient", FakeDirectClient)
    result = device_control.run_direct_request_command(
        argparse.Namespace(
            data='{"sample": 1}',
            endpoint="http://192.0.2.30",
            chip=None,
            frontend="native",
            discovery_timeout=2.0,
            origin="http://localhost",
            timeout=5.0,
            http_method=http_method,
            resource=resource,
        )
    )

    assert result == 0
    assert json.loads(capsys.readouterr().out) == {"ok": True}
    assert ("negotiate" in events) is expected_negotiate
    assert events[0] == (
        "open",
        "http://192.0.2.30/espectre/v1",
        "http://localhost",
        5.0,
    )
    assert events[-1] == ("request", http_method, resource, {"sample": 1})


@pytest.mark.parametrize("data", ["not-json", "[]"])
def test_direct_request_rejects_invalid_data(data, capsys) -> None:
    result = device_control.run_direct_request_command(
        argparse.Namespace(data=data)
    )

    assert result == 1
    assert capsys.readouterr().out.startswith("Direct request failed:")


def test_resolve_esphome_config_supports_chip_and_explicit_path(tmp_path: Path) -> None:
    relative = Path("src/cpp/frontend/esphome/examples/espectre-c3.yaml")

    assert targets.resolve_esphome_config("c3", None).name == "espectre-c3.yaml"
    assert targets.resolve_esphome_config(None, str(relative)) == common.REPO_ROOT / relative
    assert targets.resolve_esphome_config(None, str(tmp_path)) == tmp_path


def test_esphome_build_roots_are_config_specific() -> None:
    roots = {
        esphome.esphome_build_root(config_path)
        for config_path in targets.ESPHOME_CONFIGS.values()
    }

    assert len(roots) == len(targets.ESPHOME_CONFIGS)
    for config_path in targets.ESPHOME_CONFIGS.values():
        assert esphome.esphome_build_root(config_path).name == config_path.stem


def test_resolve_target_helpers_reject_invalid_inputs() -> None:
    with pytest.raises(ValueError):
        targets.resolve_esphome_config(None, None)

    with pytest.raises(ValueError):
        targets.resolve_esphome_config("bad-chip", None)

    with pytest.raises(ValueError):
        targets.resolve_idf_target("native", "bad-chip")


def test_resolve_idf_target_returns_app_dir_and_target() -> None:
    app_dir, chip = targets.resolve_idf_target("matter", "c3")

    assert app_dir.name == "app"
    assert chip == "esp32c3"


def test_esp32_s2_is_supported_without_claiming_matter() -> None:
    assert targets.resolve_esphome_config("s2", None).name == "espectre-s2.yaml"
    assert targets.resolve_idf_target("native", "s2")[1] == "esp32s2"
    with pytest.raises(ValueError, match="Unsupported matter target: s2"):
        targets.resolve_idf_target("matter", "s2")


def test_esphome_command_environment_exports_shared_git_version(monkeypatch, tmp_path: Path) -> None:
    config_path = tmp_path / "firmware.yaml"
    monkeypatch.delenv("ESPECTRE_GIT_VERSION", raising=False)
    monkeypatch.setattr(
        esphome,
        "detect_espectre_project_version",
        lambda: "2.8.0-417-g2b49a9c",
    )

    environment = esphome.esphome_command_environment(config_path)

    assert environment["ESPECTRE_GIT_VERSION"] == "2.8.0-417-g2b49a9c"
    assert environment["ESPHOME_BUILD_PATH"] == str(esphome.esphome_build_root(config_path))


def test_esphome_command_environment_preserves_explicit_git_version(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("ESPECTRE_GIT_VERSION", "3.0.0-release")
    monkeypatch.setattr(
        esphome,
        "detect_espectre_project_version",
        lambda *args, **kwargs: pytest.fail("explicit version must not be replaced"),
    )

    environment = esphome.esphome_command_environment(tmp_path / "firmware.yaml")

    assert environment["ESPECTRE_GIT_VERSION"] == "3.0.0-release"


def test_run_esphome_command_uses_resolved_config_and_device(monkeypatch, tmp_path: Path) -> None:
    config_path = tmp_path / "firmware.yaml"
    config_path.write_text("esphome:", encoding="utf-8")
    build_dir = tmp_path / "build"
    calls: list[tuple[object, ...]] = []
    resolutions: list[tuple[str, dict[str, object]]] = []

    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: config_path)
    monkeypatch.setattr(
        esphome,
        "resolve_serial_port",
        lambda port, **kwargs: resolutions.append((port, kwargs)) or port,
    )
    monkeypatch.setattr(esphome, "resolve_esphome_build_artifact", lambda _config: build_dir / "espectre.bin")
    monkeypatch.setattr(
        esphome,
        "flash_prebuilt_idf_build",
        lambda *args, **kwargs: calls.append((*args, kwargs)),
    )
    monkeypatch.setattr(
        esphome.subprocess,
        "run",
        lambda *_args, **_kwargs: pytest.fail("serial flash must use the shared IDF lifecycle"),
    )

    esphome.run_esphome_command(
        argparse.Namespace(chip="c3", config=None, esphome_command="flash", device="/dev/cu.usb")
    )

    assert calls == [
        (
            build_dir,
            "/dev/cu.usb",
            "esp32c3",
            {"chip": "c3", "erase": False},
        )
    ]
    assert resolutions == [
        (
            "/dev/cu.usb",
            {"chip": "c3", "frontend": "esphome", "purpose": "flash"},
        )
    ]


def test_run_esphome_serial_firmware_uses_factory_image(monkeypatch, tmp_path: Path) -> None:
    config_path = tmp_path / "firmware.yaml"
    config_path.write_text("esphome:", encoding="utf-8")
    factory_image = tmp_path / "firmware.factory.bin"
    factory_image.write_bytes(b"factory")
    calls: list[tuple[object, ...]] = []
    resolutions: list[tuple[str, dict[str, object]]] = []

    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: config_path)
    monkeypatch.setattr(
        esphome,
        "resolve_serial_port",
        lambda port, **kwargs: resolutions.append((port, kwargs)) or port,
    )
    monkeypatch.setattr(
        esphome,
        "resolve_esphome_build_artifact",
        lambda _config: pytest.fail("a factory image must not require a local build artifact"),
    )
    monkeypatch.setattr(
        esphome,
        "flash_factory_image",
        lambda *args, **kwargs: calls.append((*args, kwargs)),
    )

    esphome.run_esphome_command(
        argparse.Namespace(
            chip="s2",
            config=None,
            esphome_command="flash",
            device="/dev/cu.loader",
            firmware=str(factory_image),
            erase=False,
        )
    )

    assert calls == [
        (
            factory_image,
            "/dev/cu.loader",
            "esp32s2",
            {
                "chip": "s2",
                "erase": False,
            },
        )
    ]
    assert resolutions == [
        (
            "/dev/cu.loader",
            {"chip": "s2", "frontend": "esphome", "purpose": "flash"},
        ),
    ]


def test_esphome_build_json_reports_the_cli_owned_artifact(
    monkeypatch,
    tmp_path: Path,
    capsys,
) -> None:
    config_path = tmp_path / "firmware.yaml"
    config_path.write_text("esphome:", encoding="utf-8")
    artifact = tmp_path / ".esphome" / "build" / "espectre" / "build" / "espectre.bin"
    artifact.parent.mkdir(parents=True)
    artifact.write_bytes(b"firmware")
    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: config_path)
    monkeypatch.setattr(esphome.subprocess, "run", lambda *_args, **_kwargs: None)

    esphome.run_esphome_command(
        argparse.Namespace(
            chip="s3",
            config=None,
            esphome_command="build",
            device=None,
            clean=False,
            clean_all=False,
            json=True,
        )
    )

    metadata = json.loads(capsys.readouterr().out.splitlines()[-1])
    assert metadata["artifact"] == str(artifact.resolve())
    assert metadata["frontend"] == "esphome"


def test_idf_build_json_reports_the_selected_build_directory(tmp_path: Path, capsys) -> None:
    artifact = tmp_path / "build-esp32s3" / "espectre-native.bin"
    artifact.parent.mkdir()
    artifact.write_bytes(b"firmware")

    idf.print_idf_build_metadata("native", "s3", tmp_path, "build-esp32s3")

    metadata = json.loads(capsys.readouterr().out)
    assert metadata["artifact"] == str(artifact.resolve())
    assert metadata["frontend"] == "native"


def test_run_esphome_flash_uploads_prebuilt_firmware(monkeypatch, tmp_path: Path) -> None:
    config_path = tmp_path / "firmware.yaml"
    config_path.write_text("esphome:", encoding="utf-8")
    firmware_path = tmp_path / "firmware.ota.bin"
    firmware_path.write_bytes(b"firmware")
    calls: list[list[str]] = []

    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: config_path)
    monkeypatch.setattr(
        esphome,
        "resolve_serial_port",
        lambda *_args, **_kwargs: pytest.fail("OTA uploads must not resolve a serial port"),
    )
    monkeypatch.setattr(esphome.subprocess, "run", lambda cmd, check, **_kwargs: calls.append(cmd))

    esphome.run_esphome_command(
        argparse.Namespace(
            chip="c6",
            config=None,
            esphome_command="flash",
            device="espectre.local",
            firmware=str(firmware_path),
        )
    )

    assert calls == [
        [
            *esphome.ESPHOME_COMMAND_PREFIX,
            "upload",
            str(config_path),
            "--device",
            "espectre.local",
            "--file",
            str(firmware_path),
        ]
    ]


def test_run_esphome_flash_can_erase_all_data_before_upload(monkeypatch, tmp_path: Path) -> None:
    config_path = tmp_path / "firmware.yaml"
    config_path.write_text("esphome:", encoding="utf-8")
    build_dir = tmp_path / "build"
    calls: list[tuple[object, ...]] = []

    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: config_path)
    monkeypatch.setattr(esphome, "resolve_serial_port", lambda *_args, **_kwargs: "/dev/cu.resolved")
    monkeypatch.setattr(esphome, "resolve_esphome_build_artifact", lambda _config: build_dir / "espectre.bin")
    monkeypatch.setattr(
        esphome,
        "flash_prebuilt_idf_build",
        lambda *args, **kwargs: calls.append((*args, kwargs)),
    )
    monkeypatch.setattr(
        esphome.subprocess,
        "run",
        lambda *_args, **_kwargs: pytest.fail("serial flash must use the shared IDF lifecycle"),
    )

    esphome.run_esphome_command(
        argparse.Namespace(
            chip="s2",
            config=None,
            esphome_command="flash",
            device=None,
            firmware=None,
            erase=True,
        )
    )

    assert calls == [
        (
            build_dir,
            "/dev/cu.resolved",
            "esp32s2",
            {"chip": "s2", "erase": True},
        )
    ]


def test_run_esphome_monitor_uses_logs_action(monkeypatch, tmp_path: Path) -> None:
    config_path = tmp_path / "firmware.yaml"
    config_path.write_text("esphome:", encoding="utf-8")
    calls: list[list[str]] = []

    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: config_path)
    monkeypatch.setattr(
        esphome,
        "resolve_serial_port",
        lambda port, **_kwargs: port,
    )
    monkeypatch.setattr(esphome.subprocess, "run", lambda cmd, check, **_kwargs: calls.append(cmd))

    esphome.run_esphome_command(
        argparse.Namespace(chip="c3", config=None, esphome_command="monitor", device="/dev/cu.usb")
    )

    assert calls == [
        [*esphome.ESPHOME_COMMAND_PREFIX, "logs", str(config_path), "--device", "/dev/cu.usb"]
    ]


def test_run_esphome_command_build_runs_esphome_clean_when_requested(monkeypatch, tmp_path: Path) -> None:
    config_path = tmp_path / "firmware.yaml"
    config_path.write_text("esphome:", encoding="utf-8")
    calls: list[tuple[list[str], str]] = []

    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: config_path)
    monkeypatch.setattr(
        esphome.subprocess,
        "run",
        lambda cmd, check, env, **_kwargs: calls.append((cmd, env["ESPHOME_BUILD_PATH"])),
    )

    esphome.run_esphome_command(
        argparse.Namespace(chip="c3", config=None, esphome_command="build", device=None, clean=True, clean_all=False)
    )

    build_root = str(tmp_path / ".esphome" / "build" / "firmware")
    assert calls == [
        ([*esphome.ESPHOME_COMMAND_PREFIX, "clean", str(config_path)], build_root),
        ([*esphome.ESPHOME_COMMAND_PREFIX, "compile", str(config_path)], build_root),
    ]


def test_run_esphome_command_build_runs_esphome_clean_all_when_requested(monkeypatch, tmp_path: Path) -> None:
    config_path = tmp_path / "firmware.yaml"
    config_path.write_text("esphome:", encoding="utf-8")
    calls: list[list[str]] = []

    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: config_path)
    monkeypatch.setattr(esphome.subprocess, "run", lambda cmd, check, **_kwargs: calls.append(cmd))

    esphome.run_esphome_command(
        argparse.Namespace(chip="c3", config=None, esphome_command="build", device=None, clean=False, clean_all=True)
    )

    assert calls == [
        [*esphome.ESPHOME_COMMAND_PREFIX, "clean-all", str(config_path)],
        [*esphome.ESPHOME_COMMAND_PREFIX, "compile", str(config_path)],
    ]


def test_run_esphome_command_handles_missing_config(monkeypatch, tmp_path: Path) -> None:
    missing = tmp_path / "missing.yaml"
    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: missing)

    with pytest.raises(SystemExit):
        esphome.run_esphome_command(
            argparse.Namespace(chip="c3", config=None, esphome_command="build", device=None, clean=False, clean_all=False)
        )


def test_run_esphome_command_surfaces_subprocess_failures(monkeypatch, tmp_path: Path) -> None:
    config_path = tmp_path / "firmware.yaml"
    config_path.write_text("esphome:", encoding="utf-8")
    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: config_path)

    def _raise_not_found(_cmd, check, **_kwargs):
        raise FileNotFoundError()

    monkeypatch.setattr(esphome.subprocess, "run", _raise_not_found)
    with pytest.raises(SystemExit):
        esphome.run_esphome_command(
            argparse.Namespace(chip="c3", config=None, esphome_command="build", device=None, clean=False, clean_all=False)
        )

    def _raise_called(_cmd, check, **_kwargs):
        raise subprocess.CalledProcessError(7, ["esphome"])

    monkeypatch.setattr(esphome.subprocess, "run", _raise_called)
    with pytest.raises(SystemExit) as exc:
        esphome.run_esphome_command(
            argparse.Namespace(chip="c3", config=None, esphome_command="build", device=None, clean=False, clean_all=False)
        )

    assert exc.value.code == 7


@pytest.fixture
def idf_publication_stub(monkeypatch):
    """Isolate launcher and cleanup tests whose fake compilers produce no files."""
    monkeypatch.setattr(idf, "publish_idf_build", lambda *_args: None)


def test_run_idf_command_build_uses_wifi_defaults_when_present(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    (app_dir / "sdkconfig.wifi").write_text("", encoding="utf-8")
    calls: list[tuple[list[str], Path]] = []
    env = idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py")

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf.shutil, "which", lambda binary: "/usr/bin/idf.py" if binary == "idf.py" else None)
    monkeypatch.setattr(idf, "resolve_idf_environment", lambda: env)
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, cwd, check, **_kwargs: calls.append((cmd, Path(cwd))))

    idf.run_idf_command("native", argparse.Namespace(chip="c3", idf_command="build", port=None, clean=False))

    assert calls == [
        (
            [
                "idf.py",
                "-B",
                "build-esp32c3",
                "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi",
                "-DIDF_TARGET=esp32c3",
                "-DSDKCONFIG=build-esp32c3/sdkconfig",
                "build",
            ],
            app_dir,
        ),
    ]


def test_run_idf_command_build_ignores_shared_sdkconfig_target(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    (app_dir / "sdkconfig").write_text('CONFIG_IDF_TARGET="esp32c6"\n', encoding="utf-8")
    calls: list[tuple[list[str], Path]] = []
    env = idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py")

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf, "resolve_idf_environment", lambda: env)
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, cwd, check, **_kwargs: calls.append((cmd, Path(cwd))))

    idf.run_idf_command("native", argparse.Namespace(chip="c3", idf_command="build", port=None, clean=False))

    assert calls == [
        (
            [
                "idf.py",
                "-B",
                "build-esp32c3",
                "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults",
                "-DIDF_TARGET=esp32c3",
                "-DSDKCONFIG=build-esp32c3/sdkconfig",
                "build",
            ],
            app_dir,
        ),
    ]


def test_run_idf_command_build_uses_explicit_sdkconfig_when_missing(
    monkeypatch,
    tmp_path: Path,
    idf_publication_stub,
) -> None:
    app_dir = tmp_path / "app"
    build_dir = app_dir / "build-esp32c3"
    build_dir.mkdir(parents=True)
    generated_sdkconfig = build_dir / "sdkconfig.lightweight"
    calls: list[tuple[list[str], Path]] = []
    env = idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py")

    monkeypatch.setenv("ESPECTRE_IDF_SDKCONFIG", str(generated_sdkconfig))
    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf, "resolve_idf_environment", lambda: env)
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, cwd, check, **_kwargs: calls.append((cmd, Path(cwd))))

    idf.run_idf_command("native", argparse.Namespace(chip="c3", idf_command="build", port=None, clean=False))

    assert calls == [
        (
            [
                "idf.py",
                "-B",
                "build-esp32c3",
                "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults",
                "-DIDF_TARGET=esp32c3",
                f"-DSDKCONFIG={generated_sdkconfig}",
                "build",
            ],
            app_dir,
        ),
    ]


def test_run_idf_command_build_falls_back_to_cached_docker_backend(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    calls: list[dict[str, object]] = []

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf, "resolve_idf_environment", lambda: (_ for _ in ()).throw(FileNotFoundError()))
    monkeypatch.setattr(idf, "ensure_docker_backend", lambda _policy: "/usr/bin/docker")
    monkeypatch.setattr(idf, "run_idf_container", lambda **kwargs: calls.append(kwargs))

    idf.run_idf_command(
        "native",
        argparse.Namespace(
            chip="c3",
            idf_command="build",
            port=None,
            clean=False,
            clean_all=False,
            backend="auto",
            pull="ask",
            ota_channel="preview",
        ),
    )

    assert calls == [
        {
            "frontend": "native",
            "app_path": app_dir,
            "commands": [
                [
                    "idf.py",
                    "-B",
                    "build-esp32c3-docker",
                    "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults",
                    "-DIDF_TARGET=esp32c3",
                    "-DSDKCONFIG=build-esp32c3-docker/sdkconfig",
                    "-DNATIVE_OTA_CHANNEL=preview",
                    "build",
                ],
            ],
            "repo_root": common.REPO_ROOT,
            "sdkconfig_defaults": "sdkconfig.defaults",
            "pull_policy": "ask",
            "docker": "/usr/bin/docker",
        }
    ]


def test_run_idf_command_forced_local_backend_does_not_try_docker(monkeypatch, tmp_path: Path) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    docker_calls: list[str] = []

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf, "resolve_idf_environment", lambda: (_ for _ in ()).throw(FileNotFoundError()))
    monkeypatch.setattr(idf, "ensure_docker_backend", lambda policy: docker_calls.append(policy))

    with pytest.raises(SystemExit):
        idf.run_idf_command(
            "native",
            argparse.Namespace(
                chip="c3",
                idf_command="build",
                port=None,
                clean=False,
                clean_all=False,
                backend="local",
                pull="ask",
            ),
        )

    assert docker_calls == []


def test_docker_backend_failure_does_not_clean_existing_build(monkeypatch, tmp_path: Path) -> None:
    app_dir = tmp_path / "app"
    build_dir = app_dir / "build-esp32c3-docker"
    build_dir.mkdir(parents=True)
    (build_dir / "firmware.bin").write_text("keep", encoding="utf-8")

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(
        idf,
        "ensure_docker_backend",
        lambda _policy: (_ for _ in ()).throw(idf.DockerBackendError("download declined")),
    )

    with pytest.raises(SystemExit):
        idf.run_idf_command(
            "native",
            argparse.Namespace(
                chip="c3",
                idf_command="build",
                port=None,
                clean=True,
                clean_all=False,
                backend="docker",
                pull="ask",
            ),
        )

    assert (build_dir / "firmware.bin").read_text(encoding="utf-8") == "keep"


def test_docker_backend_uses_cached_image_without_prompt(monkeypatch) -> None:
    prompts: list[str] = []
    monkeypatch.setattr(idf_container, "docker_executable", lambda: "/usr/bin/docker")
    monkeypatch.setattr(idf_container, "docker_daemon_is_running", lambda _docker: True)
    monkeypatch.setattr(idf_container, "docker_image_is_present", lambda _docker, _image: True)

    docker = idf_container.ensure_docker_backend("ask", input_fn=lambda prompt: prompts.append(prompt) or "n")

    assert docker == "/usr/bin/docker"
    assert prompts == []


def test_docker_backend_asks_before_downloading_missing_image(monkeypatch) -> None:
    calls: list[list[str]] = []
    monkeypatch.setattr(idf_container, "docker_executable", lambda: "/usr/bin/docker")
    monkeypatch.setattr(idf_container, "docker_daemon_is_running", lambda _docker: True)
    monkeypatch.setattr(idf_container, "docker_image_is_present", lambda _docker, _image: False)
    monkeypatch.setattr(idf_container, "_interactive_terminal", lambda: True)
    monkeypatch.setattr(
        idf_container.subprocess,
        "run",
        lambda command, check: calls.append(command) or SimpleNamespace(returncode=0),
    )

    idf_container.ensure_docker_backend("ask", input_fn=lambda _prompt: "yes")

    assert calls == [["/usr/bin/docker", "pull", idf_container.IDF_DOCKER_IMAGE]]


def test_docker_backend_requires_explicit_pull_in_noninteractive_session(monkeypatch) -> None:
    monkeypatch.setattr(idf_container, "docker_executable", lambda: "/usr/bin/docker")
    monkeypatch.setattr(idf_container, "docker_daemon_is_running", lambda _docker: True)
    monkeypatch.setattr(idf_container, "docker_image_is_present", lambda _docker, _image: False)
    monkeypatch.setattr(idf_container, "_interactive_terminal", lambda: False)

    with pytest.raises(idf_container.DockerBackendError, match="--pull missing"):
        idf_container.ensure_docker_backend("ask")


def test_docker_backend_waits_for_user_to_start_engine(monkeypatch) -> None:
    engine_states = iter((False, True))
    prompts: list[str] = []
    monkeypatch.setattr(idf_container, "docker_executable", lambda: "/usr/bin/docker")
    monkeypatch.setattr(idf_container, "docker_daemon_is_running", lambda _docker: next(engine_states))
    monkeypatch.setattr(idf_container, "docker_image_is_present", lambda _docker, _image: True)
    monkeypatch.setattr(idf_container, "_interactive_terminal", lambda: True)

    docker = idf_container.ensure_docker_backend(
        "ask", input_fn=lambda prompt: prompts.append(prompt) or ""
    )

    assert docker == "/usr/bin/docker"
    assert len(prompts) == 1


def test_build_docker_command_mounts_repository_and_uses_separate_build_dir(tmp_path: Path) -> None:
    repo_root = tmp_path / "repo"
    app_dir = repo_root / "src" / "cpp" / "frontend" / "native" / "app"
    app_dir.mkdir(parents=True)
    commands = [["idf.py", "-B", "build-esp32c3-docker", "build"]]

    command = idf_container.build_docker_command(
        "/usr/bin/docker",
        frontend="native",
        app_path=app_dir,
        commands=commands,
        repo_root=repo_root,
        sdkconfig_defaults="sdkconfig.defaults;sdkconfig.wifi",
    )

    assert command[:3] == ["/usr/bin/docker", "run", "--rm"]
    assert f"{repo_root.resolve()}:/work" in command
    assert "/work/src/cpp/frontend/native/app" in command
    assert "IDF_CCACHE_ENABLE=1" in command
    assert "CCACHE_DIR=/work/.cache/build/native-home/ccache" in command
    assert "CCACHE_MAXSIZE=2G" in command
    assert "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi" in command
    assert command[-1] == "idf.py -B build-esp32c3-docker build"
    assert (repo_root / ".cache" / "build" / "native-home" / "ccache").is_dir()


def test_apply_local_ccache_enables_when_binary_exists(monkeypatch) -> None:
    monkeypatch.setattr(idf, "ccache_binary", lambda path=None: "/usr/bin/ccache")
    env = {"PATH": "/usr/bin"}

    assert idf.apply_local_ccache(env) is True
    assert env["IDF_CCACHE_ENABLE"] == "1"


def test_apply_local_ccache_keeps_explicit_disable(monkeypatch) -> None:
    monkeypatch.setattr(idf, "ccache_binary", lambda path=None: "/usr/bin/ccache")
    env = {"IDF_CCACHE_ENABLE": "0", "PATH": "/usr/bin"}

    assert idf.apply_local_ccache(env) is False
    assert env["IDF_CCACHE_ENABLE"] == "0"


def test_idf_subprocess_env_injects_ccache_for_unconfigured_local_backend(monkeypatch) -> None:
    monkeypatch.delenv("IDF_CCACHE_ENABLE", raising=False)
    monkeypatch.setattr(idf, "ccache_binary", lambda path=None: "/opt/homebrew/bin/ccache")
    env = idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py")

    process_env = idf.idf_subprocess_env(env)

    assert process_env is not None
    assert process_env["IDF_CCACHE_ENABLE"] == "1"


def test_idf_subprocess_env_inherits_shell_when_already_configured(monkeypatch) -> None:
    monkeypatch.setenv("IDF_CCACHE_ENABLE", "1")
    monkeypatch.setattr(idf, "ccache_binary", lambda path=None: "/opt/homebrew/bin/ccache")
    env = idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py")

    assert idf.idf_subprocess_env(env) is None


def test_run_idf_command_build_uses_target_specific_defaults_when_present(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    (app_dir / "sdkconfig.defaults.esp32").write_text("CONFIG_TEST=y\n", encoding="utf-8")
    (app_dir / "sdkconfig.wifi").write_text("", encoding="utf-8")
    calls: list[tuple[list[str], Path]] = []
    env = idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py")

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32"))
    monkeypatch.setattr(idf.shutil, "which", lambda binary: "/usr/bin/idf.py" if binary == "idf.py" else None)
    monkeypatch.setattr(idf, "resolve_idf_environment", lambda: env)
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, cwd, check, **_kwargs: calls.append((cmd, Path(cwd))))

    idf.run_idf_command("native", argparse.Namespace(chip="esp32", idf_command="build", port=None, clean=False))

    assert calls == [
        (
            [
                "idf.py",
                "-B",
                "build-esp32",
                "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.defaults.esp32;sdkconfig.wifi",
                "-DIDF_TARGET=esp32",
                "-DSDKCONFIG=build-esp32/sdkconfig",
                "build",
            ],
            app_dir,
        ),
    ]


def test_run_idf_command_build_cleans_generated_artifacts_when_requested(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    build_dir = app_dir / "build-esp32c3"
    build_dir.mkdir()
    (build_dir / "firmware.bin").write_text("bin", encoding="utf-8")
    legacy_build_dir = app_dir / "build"
    legacy_build_dir.mkdir()
    (legacy_build_dir / "firmware.bin").write_text("legacy", encoding="utf-8")
    (app_dir / "sdkconfig").write_text("CONFIG_TEST=y\n", encoding="utf-8")
    (app_dir / "sdkconfig.old").write_text("CONFIG_TEST_OLD=y\n", encoding="utf-8")
    (app_dir / "dependencies.lock").write_text("lock", encoding="utf-8")
    (app_dir / "sdkconfig.wifi").write_text("", encoding="utf-8")
    calls: list[tuple[list[str], Path]] = []
    env = idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py")

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf.shutil, "which", lambda binary: "/usr/bin/idf.py" if binary == "idf.py" else None)
    monkeypatch.setattr(idf, "resolve_idf_environment", lambda: env)
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, cwd, check, **_kwargs: calls.append((cmd, Path(cwd))))

    idf.run_idf_command("native", argparse.Namespace(chip="c3", idf_command="build", port=None, clean=True))

    assert not build_dir.exists()
    assert legacy_build_dir.exists()
    assert (app_dir / "sdkconfig").exists()
    assert (app_dir / "sdkconfig.old").exists()
    assert (app_dir / "dependencies.lock").exists()
    assert (app_dir / "sdkconfig.wifi").exists()
    assert calls == [
        (
            [
                "idf.py",
                "-B",
                "build-esp32c3",
                "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi",
                "-DIDF_TARGET=esp32c3",
                "-DSDKCONFIG=build-esp32c3/sdkconfig",
                "build",
            ],
            app_dir,
        ),
    ]


def test_run_idf_command_build_uses_env_defaults_and_custom_build_dir(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    build_dir = app_dir / "build-esp32c3"
    build_dir.mkdir()
    (build_dir / "firmware.bin").write_text("bin", encoding="utf-8")
    calls: list[tuple[list[str], Path]] = []

    monkeypatch.setenv("SDKCONFIG_DEFAULTS", "sdkconfig.defaults;sdkconfig.extra.defaults")
    monkeypatch.setenv("ESPECTRE_IDF_BUILD_DIR", "build-esp32c3")
    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf.shutil, "which", lambda binary: "/usr/bin/idf.py" if binary == "idf.py" else None)
    monkeypatch.setattr(
        idf,
        "resolve_idf_environment",
        lambda: idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py"),
    )
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, cwd, check, **_kwargs: calls.append((cmd, Path(cwd))))

    idf.run_idf_command("native", argparse.Namespace(chip="c3", idf_command="build", port=None, clean=True))

    assert not build_dir.exists()
    assert calls == [
        (
            [
                "idf.py",
                "-B",
                "build-esp32c3",
                "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.extra.defaults",
                "-DIDF_TARGET=esp32c3",
                "-DSDKCONFIG=build-esp32c3/sdkconfig",
                "build",
            ],
            app_dir,
        ),
    ]


def test_run_idf_command_build_uses_isolated_sdkconfig(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    isolated_sdkconfig = app_dir / ".benchmark.sdkconfig"
    calls: list[tuple[list[str], Path]] = []

    monkeypatch.setenv("ESPECTRE_IDF_SDKCONFIG", str(isolated_sdkconfig))
    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf.shutil, "which", lambda binary: "/usr/bin/idf.py" if binary == "idf.py" else None)
    monkeypatch.setattr(
        idf,
        "resolve_idf_environment",
        lambda: idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py"),
    )
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, cwd, check, **_kwargs: calls.append((cmd, Path(cwd))))

    idf.run_idf_command("native", argparse.Namespace(chip="c3", idf_command="build", port=None, clean=True))

    sdkconfig_arg = f"-DSDKCONFIG={isolated_sdkconfig.resolve()}"
    assert calls == [
        (
            [
                "idf.py",
                "-B",
                "build-esp32c3",
                "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults",
                "-DIDF_TARGET=esp32c3",
                sdkconfig_arg,
                "build",
            ],
            app_dir,
        ),
    ]


def test_run_idf_command_build_clean_all_removes_all_builds_and_shared_artifacts(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    for build_dir_name in ("build", "build-esp32", "build-esp32c3", "build-flash-esp32c3"):
        build_dir = app_dir / build_dir_name
        build_dir.mkdir()
        (build_dir / "artifact.bin").write_text("bin", encoding="utf-8")
    (app_dir / "sdkconfig").write_text("CONFIG_TEST=y\n", encoding="utf-8")
    (app_dir / "sdkconfig.old").write_text("CONFIG_TEST_OLD=y\n", encoding="utf-8")
    (app_dir / "dependencies.lock").write_text("lock", encoding="utf-8")
    (app_dir / "sdkconfig.wifi").write_text("", encoding="utf-8")
    calls: list[tuple[list[str], Path]] = []

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf.shutil, "which", lambda binary: "/usr/bin/idf.py" if binary == "idf.py" else None)
    monkeypatch.setattr(
        idf,
        "resolve_idf_environment",
        lambda: idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py"),
    )
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, cwd, check, **_kwargs: calls.append((cmd, Path(cwd))))

    idf.run_idf_command(
        "native",
        argparse.Namespace(chip="c3", idf_command="build", port=None, clean=False, clean_all=True),
    )

    assert not (app_dir / "build").exists()
    assert not (app_dir / "build-esp32").exists()
    assert not (app_dir / "build-esp32c3").exists()
    assert not (app_dir / "build-flash-esp32c3").exists()
    assert not (app_dir / "sdkconfig").exists()
    assert not (app_dir / "sdkconfig.old").exists()
    assert not (app_dir / "dependencies.lock").exists()
    assert (app_dir / "sdkconfig.wifi").exists()
    assert calls == [
        (
            [
                "idf.py",
                "-B",
                "build-esp32c3",
                "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi",
                "-DIDF_TARGET=esp32c3",
                "-DSDKCONFIG=build-esp32c3/sdkconfig",
                "build",
            ],
            app_dir,
        ),
    ]


def _write_flasher_args(build_dir: Path) -> None:
    build_dir.mkdir(parents=True, exist_ok=True)
    (build_dir / "app.bin").write_bytes(b"application")
    (build_dir / "bootloader.bin").write_bytes(b"bootloader")
    (build_dir / "flasher_args.json").write_text(
        json.dumps(
            {
                "write_flash_args": [
                    "--flash_mode",
                    "dio",
                    "--flash_freq",
                    "40m",
                    "--flash_size",
                    "4MB",
                ],
                "flash_files": {
                    "0x10000": "app.bin",
                    "0x0": "bootloader.bin",
                },
            }
        ),
        encoding="utf-8",
    )


@pytest.mark.parametrize("frontend, chip", [("native", "c3"), ("matter", "s3")])
def test_idf_flash_uses_last_successful_build_across_backends(monkeypatch, tmp_path, frontend, chip):
    app_dir = tmp_path / frontend
    app_dir.mkdir()
    idf_target = targets.IDF_TARGET_BY_CHIP[chip]
    monkeypatch.delenv("ESPECTRE_IDF_BUILD_DIR", raising=False)
    monkeypatch.setitem(idf.IDF_FRONTENDS, frontend, {"app_dir": app_dir, "targets": {chip: idf_target}})
    monkeypatch.setattr(idf, "resolve_serial_port", lambda port, **_kwargs: port)
    monkeypatch.setattr(idf, "read_matter_onboarding_for_command", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(
        idf, "resolve_idf_build_backend",
        lambda backend, _pull: idf.ResolvedIdfBuildBackend(
            mode=backend,
            idf_environment=idf.ResolvedIdfEnvironment(mode="path", source="test"),
            docker="docker",
        ),
    )
    payload = b""
    fail_build = False

    def compile_firmware(command, *, cwd):
        build_dir = Path(cwd) / command[command.index("-B") + 1]
        _write_flasher_args(build_dir)
        (build_dir / "app.bin").write_bytes(payload)
        if fail_build:
            raise subprocess.CalledProcessError(7, command)

    def compile_container(**kwargs):
        try:
            compile_firmware(kwargs["commands"][0], cwd=kwargs["app_path"])
        except subprocess.CalledProcessError as exc:
            raise idf_container.DockerBackendError("build failed") from exc

    monkeypatch.setattr(idf, "run_idf_subprocess", lambda command, _env, *, cwd: compile_firmware(command, cwd=cwd))
    monkeypatch.setattr(idf, "run_idf_container", compile_container)
    flashed = []

    def flash(command, *, cwd):
        offset = command.index("0x10000")
        flashed.append((Path(cwd), (Path(cwd) / command[offset + 1]).read_bytes()))

    monkeypatch.setattr(esptool_runner, "run_esptool", flash)
    previous_payload = None
    for index, backend in enumerate(("local", "docker", "local")):
        payload = f"firmware-{index}-{backend}".encode()
        build_args = app.build_parser().parse_args([frontend, "build", "--chip", chip, "--backend", backend])
        idf.run_idf_command(frontend, build_args)
        flash_args = app.build_parser().parse_args([frontend, "flash", "--chip", chip, "--port", "/dev/test"])
        idf.run_idf_command(frontend, flash_args)
        assert flashed[-1] == (app_dir / f"build-flash-{idf_target}", payload)
        assert flashed[-1][1] != previous_payload
        previous_payload = payload

        fail_build = True
        payload = b"incomplete build"
        with pytest.raises(SystemExit):
            idf.run_idf_command(frontend, build_args)
        idf.run_idf_command(frontend, flash_args)
        assert flashed[-1][1] == previous_payload
        fail_build = False


def test_idf_flash_preserves_explicit_build_directory_override(monkeypatch, tmp_path):
    monkeypatch.setenv("ESPECTRE_IDF_BUILD_DIR", "custom-build")
    assert idf.resolve_flash_idf_selection("native", tmp_path, "c3") == ("esp32c3", "custom-build")


@pytest.mark.parametrize("failure", ["missing_binary", "manifest_replace"])
def test_idf_publication_keeps_previous_image_on_failure(monkeypatch, tmp_path, failure):
    build_dir = tmp_path / "build"
    published = tmp_path / "published"
    _write_flasher_args(build_dir)
    build_artifacts.publish_idf_flash_artifacts(build_dir, published)
    original = (published / "flasher_args.json").read_bytes()
    (build_dir / "app.bin").write_bytes(b"new firmware")
    if failure == "missing_binary":
        (build_dir / "bootloader.bin").unlink()
    else:
        replace = os.replace

        def fail_manifest(source, destination):
            if Path(destination).name == "flasher_args.json":
                raise OSError("publication interrupted")
            replace(source, destination)

        monkeypatch.setattr(build_artifacts.os, "replace", fail_manifest)
    with pytest.raises(OSError):
        build_artifacts.publish_idf_flash_artifacts(build_dir, published)
    assert (published / "flasher_args.json").read_bytes() == original
    for name in json.loads(original)["flash_files"].values():
        assert (published / name).is_file()


def test_idf_publication_is_self_contained_and_preserves_active_flash_files(monkeypatch, tmp_path):
    build_dir = tmp_path / "build"
    published = tmp_path / "published"
    _write_flasher_args(build_dir)
    metadata = esptool_runner.read_idf_flash_metadata(build_dir)
    (build_dir / "nested").mkdir()
    (build_dir / "app.bin").rename(build_dir / "nested" / "app.bin")
    metadata["flash_files"]["0x10000"] = "nested/app.bin"
    metadata["flash_files"]["0x0"] = str(build_dir / "bootloader.bin")
    # A second offset may legitimately contain the same bytes.
    metadata["flash_files"]["0x20000"] = "nested/app.bin"
    (build_dir / "flasher_args.json").write_text(json.dumps(metadata))
    build_artifacts.publish_idf_flash_artifacts(build_dir, published)
    previous = esptool_runner.read_idf_flash_metadata(published)
    (build_dir / "nested" / "app.bin").write_bytes(b"new firmware")
    replace = os.replace

    def check_commit(source, destination):
        if Path(destination).name == "flasher_args.json":
            assert esptool_runner.read_idf_flash_metadata(published) == previous
            for name in json.loads(Path(source).read_text())["flash_files"].values():
                assert (published / name).is_file()
        replace(source, destination)

    monkeypatch.setattr(build_artifacts.os, "replace", check_commit)
    build_artifacts.publish_idf_flash_artifacts(build_dir, published)
    current = esptool_runner.read_idf_flash_metadata(published)
    assert current["write_flash_args"] == metadata["write_flash_args"]
    assert (published / current["flash_files"]["0x10000"]).read_bytes() == b"new firmware"
    assert (published / previous["flash_files"]["0x10000"]).read_bytes() == b"application"
    assert all(Path(name).name == name for name in current["flash_files"].values())


@pytest.mark.parametrize(
    ("chip", "idf_target", "console", "before", "after", "baud"),
    [
        ("esp32", "esp32", "uart", "default-reset", "hard-reset", "115200"),
        ("s2", "esp32s2", "usb_cdc", "no-reset", "watchdog-reset", "115200"),
        ("s2", "esp32s2", "uart", "default-reset", "watchdog-reset", "460800"),
        ("s3", "esp32s3", "usb_serial_jtag", "default-reset", "watchdog-reset", "460800"),
        ("c6", "esp32c6", "usb_serial_jtag", "default-reset", "watchdog-reset", "460800"),
    ],
)
def test_flash_build_uses_one_esptool_session(
    monkeypatch,
    tmp_path: Path,
    chip: str,
    idf_target: str,
    console: str,
    before: str,
    after: str,
    baud: str,
) -> None:
    build_dir = tmp_path / "build"
    _write_flasher_args(build_dir)
    calls: list[tuple[list[str], Path, bool]] = []
    monkeypatch.setattr(
        esptool_runner,
        "serial_console_mode",
        lambda _chip, _port: console,
    )
    monkeypatch.setattr(
        esptool_runner.subprocess,
        "run",
        lambda command, *, cwd, check: calls.append((command, Path(cwd), check)),
    )

    esptool_runner.flash_build(
        build_dir,
        chip=chip,
        idf_target=idf_target,
        port="/dev/cu.test",
        erase=True,
    )

    assert calls == [
        (
            [
                sys.executable,
                "-m",
                "esptool",
                "--chip",
                idf_target,
                "--port",
                "/dev/cu.test",
                "--baud",
                baud,
                "--before",
                before,
                "--after",
                after,
                "write-flash",
                "--erase-all",
                "--flash-mode",
                "dio",
                "--flash-freq",
                "40m",
                "--flash-size",
                "4MB",
                "0x0",
                "bootloader.bin",
                "0x10000",
                "app.bin",
            ],
            build_dir,
            True,
        )
    ]


def test_factory_flash_and_application_start_use_public_esptool_commands(
    monkeypatch,
    tmp_path: Path,
) -> None:
    image = tmp_path / "factory.bin"
    image.write_bytes(b"factory")
    calls: list[tuple[list[str], Path | None]] = []
    monkeypatch.setattr(
        esptool_runner,
        "serial_console_mode",
        lambda selected_chip, _port: (
            "usb_cdc" if selected_chip == "s2" else "usb_serial_jtag"
        ),
    )
    monkeypatch.setattr(
        esptool_runner,
        "run_esptool",
        lambda args, *, cwd=None: calls.append((args, cwd)),
    )

    esptool_runner.flash_factory_image(
        image,
        chip="s2",
        idf_target="esp32s2",
        port="/dev/cu.loader",
        erase=False,
    )
    esptool_runner.run_firmware(
        chip="c6",
        idf_target="esp32c6",
        port="/dev/cu.test",
    )

    assert calls == [
        (
            [
                "--chip",
                "esp32s2",
                "--port",
                "/dev/cu.loader",
                "--baud",
                "115200",
                "--before",
                "no-reset",
                "--after",
                "watchdog-reset",
                "write-flash",
                "0x0",
                str(image),
            ],
            common.REPO_ROOT,
        ),
        (
            [
                "--chip",
                "esp32c6",
                "--port",
                "/dev/cu.test",
                "--before",
                "default-reset",
                "--after",
                "no-reset",
                "run",
            ],
            common.REPO_ROOT,
        ),
    ]


def test_run_idf_flash_delegates_existing_build(monkeypatch, tmp_path: Path) -> None:
    app_dir = tmp_path / "app"
    build_dir = app_dir / "build-flash-esp32s3"
    _write_flasher_args(build_dir)
    calls: list[tuple[object, ...]] = []
    monkeypatch.setitem(
        idf.IDF_FRONTENDS,
        "native",
        {"app_dir": app_dir, "targets": {"s3": "esp32s3"}},
    )
    monkeypatch.setattr(idf, "resolve_serial_port", lambda port, **_kwargs: port)
    monkeypatch.setattr(
        idf,
        "flash_prebuilt_idf_build",
        lambda *args, **kwargs: calls.append((*args, kwargs)),
    )

    idf.run_idf_command(
        "native",
        argparse.Namespace(
            chip="s3",
            idf_command="flash",
            port="/dev/cu.test",
            erase=True,
        ),
    )

    assert calls == [
        (
            build_dir,
            "/dev/cu.test",
            "esp32s3",
            {"chip": "s3", "erase": True},
        )
    ]


def test_matter_flash_fails_when_startup_codes_are_not_captured(
    monkeypatch,
    tmp_path: Path,
) -> None:
    app_dir = tmp_path / "app"
    _write_flasher_args(app_dir / "build-flash-esp32c3")
    monkeypatch.setitem(
        idf.IDF_FRONTENDS,
        "matter",
        {"app_dir": app_dir, "targets": {"c3": "esp32c3"}},
    )
    monkeypatch.setattr(idf, "resolve_serial_port", lambda port, **_kwargs: port)
    monkeypatch.setattr(idf, "flash_prebuilt_idf_build", lambda *_args, **_kwargs: None)
    onboarding: list[tuple[str, bool]] = []
    monkeypatch.setattr(
        idf,
        "read_matter_onboarding_for_command",
        lambda port, _args, *, reset: onboarding.append((port, reset)) or False,
    )

    with pytest.raises(SystemExit) as exc_info:
        idf.run_idf_command(
            "matter",
            argparse.Namespace(
                chip="c3",
                idf_command="flash",
                port="/dev/cu.test",
                erase=False,
            ),
        )

    assert exc_info.value.code == 1
    assert onboarding == [("/dev/cu.test", False)]


def test_run_matter_qr_reads_without_idf_environment(monkeypatch, tmp_path: Path) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    ports: list[str] = []

    monkeypatch.setitem(idf.IDF_FRONTENDS, "matter", {"app_dir": app_dir, "targets": {"c3": "esp32c3"}})
    monkeypatch.setattr(idf, "get_serial_port", lambda port, **_kwargs: port or "/dev/cu.auto")
    monkeypatch.setattr(
        idf,
        "read_matter_onboarding_for_command",
        lambda port, _args: ports.append(port) or True,
    )

    idf.run_idf_command(
        "matter",
        argparse.Namespace(
            chip="c3",
            idf_command="qr",
            port=None,
            no_reset=False,
            timeout=20.0,
            json=False,
        ),
    )

    assert ports == ["/dev/cu.auto"]


def test_serial_monitor_reset_uses_esptool_run_before_open(monkeypatch) -> None:
    events: list[object] = []

    class FakeSerialConnection:
        def __init__(self, port: str, *, baudrate: int, timeout: float) -> None:
            events.append(("open", port, baudrate, timeout))

        @property
        def in_waiting(self) -> int:
            return 0

        def read(self, _size: int) -> bytes:
            raise KeyboardInterrupt

        def close(self) -> None:
            events.append("close")

    fake_serial = SimpleNamespace(
        Serial=FakeSerialConnection,
        SerialException=OSError,
    )
    monkeypatch.setattr(serial_monitor, "serial", fake_serial)
    monkeypatch.setattr(
        serial_monitor,
        "resolve_serial_port",
        lambda *_args, **_kwargs: "/dev/cu.test",
    )
    monkeypatch.setattr(serial_monitor, "serial_console_mode", lambda *_args: "uart")
    monkeypatch.setattr(
        serial_monitor,
        "run_firmware",
        lambda **kwargs: events.append(("run", kwargs)),
    )

    serial_monitor.run_serial_monitor(
        argparse.Namespace(
            port="/dev/cu.test",
            chip="c6",
            frontend="native",
            baud=115200,
            raw=False,
            reset=True,
        )
    )

    assert events == [
        (
            "run",
            {
                "chip": "c6",
                "idf_target": "esp32c6",
                "port": "/dev/cu.test",
            },
        ),
        ("open", "/dev/cu.test", 115200, 1.0),
        "close",
    ]


def test_serial_monitor_does_not_retry_a_disconnect(monkeypatch) -> None:
    class DisconnectedSerial:
        def __init__(self, *_args, **_kwargs) -> None:
            pass

        @property
        def in_waiting(self) -> int:
            raise OSError("disconnected")

        def close(self) -> None:
            pass

    monkeypatch.setattr(
        serial_monitor,
        "serial",
        SimpleNamespace(Serial=DisconnectedSerial, SerialException=OSError),
    )
    resolutions: list[object] = []
    monkeypatch.setattr(
        serial_monitor,
        "resolve_serial_port",
        lambda port, **_kwargs: resolutions.append(port) or "/dev/cu.test",
    )

    with pytest.raises(SystemExit):
        serial_monitor.run_serial_monitor(
            argparse.Namespace(
                port="/dev/cu.test",
                chip="s3",
                frontend="native",
                baud=115200,
                raw=False,
                reset=False,
            )
        )

    assert resolutions == ["/dev/cu.test"]


def test_serial_monitor_requires_pyserial(monkeypatch, capsys) -> None:
    monkeypatch.setattr(serial_monitor, "serial", None)

    with pytest.raises(SystemExit) as exc_info:
        serial_monitor._require_pyserial()

    assert exc_info.value.code == 1
    assert "pyserial not found" in capsys.readouterr().out


def test_serial_output_supports_raw_and_text_modes(monkeypatch) -> None:
    text_writes = []
    raw_writes = []
    flushes = []
    fake_stdout = SimpleNamespace(
        buffer=SimpleNamespace(
            write=lambda data: raw_writes.append(data),
            flush=lambda: flushes.append("raw"),
        ),
        write=lambda data: text_writes.append(data),
        flush=lambda: flushes.append("text"),
    )
    monkeypatch.setattr(serial_monitor.sys, "stdout", fake_stdout)

    serial_monitor._write_serial_output(b"\xff", raw=True)
    serial_monitor._write_serial_output(b"hello\xff", raw=False)

    assert raw_writes == [b"\xff"]
    assert text_writes == ["hello�"]
    assert flushes == ["raw", "text"]


@pytest.mark.parametrize(
    ("chip", "console_mode", "message"),
    [
        ("s3", "usb_cdc", "Automatic reset is unavailable"),
        (None, "uart", "--chip is required with --reset"),
    ],
)
def test_serial_monitor_rejects_unsupported_reset(
    monkeypatch, capsys, chip, console_mode, message
) -> None:
    monkeypatch.setattr(serial_monitor, "serial", SimpleNamespace())
    monkeypatch.setattr(
        serial_monitor,
        "resolve_serial_port",
        lambda *_args, **_kwargs: "/dev/cu.test",
    )
    monkeypatch.setattr(serial_monitor, "serial_console_mode", lambda *_args: console_mode)

    with pytest.raises(SystemExit) as exc_info:
        serial_monitor.run_serial_monitor(
            argparse.Namespace(
                port="/dev/cu.test",
                chip=chip,
                frontend="native",
                baud=115200,
                raw=False,
                reset=True,
            )
        )

    assert exc_info.value.code == 1
    assert message in capsys.readouterr().out


def test_serial_monitor_streams_nonempty_data_and_skips_empty_reads(monkeypatch) -> None:
    writes = []

    class FakeSerialConnection:
        def __init__(self, *_args, **_kwargs):
            self.reads = iter((b"hello", b""))

        @property
        def in_waiting(self):
            return 0

        def read(self, _size):
            try:
                return next(self.reads)
            except StopIteration as exc:
                raise KeyboardInterrupt from exc

        def close(self):
            pass

    monkeypatch.setattr(
        serial_monitor,
        "serial",
        SimpleNamespace(Serial=FakeSerialConnection, SerialException=OSError),
    )
    monkeypatch.setattr(
        serial_monitor,
        "resolve_serial_port",
        lambda *_args, **_kwargs: "/dev/cu.test",
    )
    monkeypatch.setattr(
        serial_monitor,
        "_write_serial_output",
        lambda data, *, raw: writes.append((data, raw)),
    )

    serial_monitor.run_serial_monitor(
        argparse.Namespace(
            port="/dev/cu.test",
            chip="c6",
            frontend="native",
            baud=115200,
            raw=True,
            reset=False,
        )
    )

    assert writes == [(b"hello", True)]


def test_build_parser_accepts_top_level_monitor() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["monitor", "--port", "/dev/cu.test", "--baud", "74880", "--raw"])

    assert args.namespace == "monitor"
    assert args.port == "/dev/cu.test"
    assert args.baud == 74880
    assert args.raw is True
    assert args.reset is False

    reset_args = parser.parse_args(["monitor", "--port", "/dev/cu.test", "--reset"])
    assert reset_args.reset is True


def test_build_parser_accepts_doctor() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["doctor"])

    assert args.namespace == "doctor"


def test_idf_build_parser_accepts_clean_flag() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["native", "build", "--chip", "c6", "--clean"])

    assert args.namespace == "native"
    assert args.idf_command == "build"
    assert args.chip == "c6"
    assert args.clean is True


@pytest.mark.parametrize(
    "arguments",
    [
        ["native", "build", "--chip", "s3", "--json"],
        ["matter", "build", "--chip", "s3", "--json"],
        ["esphome", "build", "--chip", "s3", "--json"],
        ["micro", "build", "--chip", "s3", "--json"],
    ],
)
def test_build_parsers_accept_json(arguments) -> None:
    args = app.build_parser().parse_args(arguments)

    assert args.json is True


def test_discovery_parsers_accept_chip_filters() -> None:
    parser = app.build_parser()

    devices = parser.parse_args(["devices", "--frontend", "matter", "--chip", "s3"])
    direct = parser.parse_args(["direct", "get", "health", "--frontend", "matter", "--chip", "s3"])

    assert devices.chip == "s3"
    assert direct.chip == "s3"


def test_idf_flash_parser_accepts_chip() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["native", "flash", "--chip", "c5"])

    assert args.namespace == "native"
    assert args.idf_command == "flash"
    assert args.chip == "c5"
    assert args.port is None


@pytest.mark.parametrize("frontend", ["native", "matter"])
def test_idf_flash_parser_accepts_full_erase(frontend: str) -> None:
    parser = app.build_parser()

    args = parser.parse_args([frontend, "flash", "--chip", "s3", "--erase"])

    assert args.erase is True


def test_esphome_flash_parser_accepts_full_erase() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["esphome", "flash", "--chip", "c3", "--erase"])

    assert args.erase is True


def test_provision_parser_accepts_optional_chip_and_json() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["provision", "--chip", "s3", "--ssid", "lab", "--json"])

    assert args.chip == "s3"
    assert args.port is None
    assert args.json is True


def test_micro_device_parsers_accept_optional_chip() -> None:
    parser = app.build_parser()

    deploy_args = parser.parse_args(["micro", "deploy", "--chip", "c6"])
    run_args = parser.parse_args(["micro", "run", "--chip", "c3"])
    verify_args = parser.parse_args(["micro", "verify", "--chip", "s3"])

    assert deploy_args.chip == "c6"
    assert run_args.chip == "c3"
    assert verify_args.chip == "s3"


@pytest.mark.parametrize("command", ["build", "flash", "deploy", "run", "verify"])
def test_micro_parsers_accept_s2(command: str) -> None:
    args = app.build_parser().parse_args(["micro", command, "--chip", "s2"])

    assert args.chip == "s2"


def test_generic_parsers_continue_to_accept_s2() -> None:
    parser = app.build_parser()

    devices = parser.parse_args(["devices", "--chip", "s2"])
    monitor = parser.parse_args(["monitor", "--chip", "s2"])

    assert devices.chip == "s2"
    assert monitor.chip == "s2"


def test_matter_qr_parser_requires_chip() -> None:
    parser = app.build_parser()

    args = parser.parse_args(
        ["matter", "qr", "--chip", "c6", "--no-reset", "--timeout", "45"]
    )

    assert args.chip == "c6"
    assert args.port is None
    assert args.no_reset is True
    assert args.timeout == 45.0

    with pytest.raises(SystemExit):
        parser.parse_args(["matter", "qr", "--no-reset"])


@pytest.mark.parametrize("frontend", ["native", "matter", "micro"])
def test_serial_flash_parsers_require_chip(frontend: str) -> None:
    with pytest.raises(SystemExit):
        app.build_parser().parse_args([frontend, "flash"])


def test_esphome_serial_flash_requires_chip(monkeypatch, tmp_path: Path) -> None:
    config = tmp_path / "firmware.yaml"
    config.write_text("esphome:\n", encoding="utf-8")
    monkeypatch.setattr(esphome, "resolve_esphome_config", lambda *_args: config)
    monkeypatch.setattr(
        esphome,
        "resolve_serial_port",
        lambda *_args, **_kwargs: pytest.fail("port resolution requires a chip"),
    )

    with pytest.raises(SystemExit):
        esphome.run_esphome_command(
            argparse.Namespace(
                chip=None,
                config=str(config),
                esphome_command="flash",
                device="/dev/cu.test",
            )
        )


def test_micro_flash_rejects_custom_firmware_option() -> None:
    with pytest.raises(SystemExit):
        app.build_parser().parse_args(
            ["micro", "flash", "--chip", "s3", "--firmware", "custom.bin"]
        )


def test_idf_build_parser_accepts_backend_and_pull_policy(monkeypatch) -> None:
    monkeypatch.delenv("NATIVE_OTA_CHANNEL", raising=False)
    parser = app.build_parser()

    args = parser.parse_args(
        ["native", "build", "--chip", "c3", "--backend", "docker", "--pull", "missing"]
    )

    assert args.backend == "docker"
    assert args.pull == "missing"
    assert args.ota_channel == "release"


def test_micro_build_and_flash_accept_shared_backend_policy() -> None:
    parser = app.build_parser()

    build_args = parser.parse_args(
        ["micro", "build", "--chip", "c3", "--backend", "docker", "--pull", "missing"]
    )
    flash_args = parser.parse_args(
        ["micro", "flash", "--chip", "c3", "--backend", "local"]
    )

    assert build_args.backend == "docker"
    assert build_args.pull == "missing"
    assert flash_args.backend == "local"
    assert flash_args.pull == "ask"


def test_native_build_parser_accepts_ota_channel() -> None:
    parser = app.build_parser()

    args = parser.parse_args(
        ["native", "build", "--chip", "c3", "--ota-channel", "develop"]
    )

    assert args.ota_channel == "develop"


def test_run_native_build_passes_ota_channel_to_cmake(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    calls: list[tuple[list[str], Path]] = []
    env = idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py")

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf, "resolve_idf_environment", lambda: env)
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, cwd, check, **_kwargs: calls.append((cmd, Path(cwd))))

    idf.run_idf_command(
        "native",
        argparse.Namespace(
            chip="c3",
            idf_command="build",
            port=None,
            clean=False,
            ota_channel="develop",
        ),
    )

    assert calls == [
        (
            [
                "idf.py",
                "-B",
                "build-esp32c3",
                "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults",
                "-DIDF_TARGET=esp32c3",
                "-DSDKCONFIG=build-esp32c3/sdkconfig",
                "-DNATIVE_OTA_CHANNEL=develop",
                "build",
            ],
            app_dir,
        ),
    ]


def test_idf_build_parser_defaults_to_automatic_backend() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["matter", "build", "--chip", "c6"])

    assert args.backend == "auto"
    assert args.pull == "ask"


def test_idf_build_parser_accepts_clean_all_flag() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["native", "build", "--chip", "c6", "--clean-all"])

    assert args.namespace == "native"
    assert args.idf_command == "build"
    assert args.chip == "c6"
    assert args.clean_all is True


def test_esphome_build_parser_accepts_clean_flag() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["esphome", "build", "--chip", "c6", "--clean"])

    assert args.namespace == "esphome"
    assert args.esphome_command == "build"
    assert args.chip == "c6"
    assert args.clean is True


def test_esphome_build_parser_accepts_clean_all_flag() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["esphome", "build", "--chip", "c6", "--clean-all"])

    assert args.namespace == "esphome"
    assert args.esphome_command == "build"
    assert args.chip == "c6"
    assert args.clean_all is True


def test_esphome_monitor_parser_accepts_device() -> None:
    parser = app.build_parser()

    args = parser.parse_args(["esphome", "monitor", "--chip", "c6", "--device", "/dev/cu.test"])

    assert args.namespace == "esphome"
    assert args.esphome_command == "monitor"
    assert args.chip == "c6"
    assert args.device == "/dev/cu.test"


def test_esphome_flash_parser_accepts_prebuilt_firmware() -> None:
    parser = app.build_parser()

    args = parser.parse_args(
        ["esphome", "flash", "--chip", "c6", "--device", "espectre.local", "--firmware", "firmware.ota.bin"]
    )

    assert args.namespace == "esphome"
    assert args.esphome_command == "flash"
    assert args.firmware == "firmware.ota.bin"


def test_run_idf_command_handles_resolution_and_subprocess_errors(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (_ for _ in ()).throw(ValueError("bad target")))

    with pytest.raises(SystemExit):
        idf.run_idf_command("native", argparse.Namespace(chip="bad", idf_command="build", port=None, clean=False))

    app_dir = tmp_path / "app"
    app_dir.mkdir()
    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(
        idf,
        "resolve_idf_environment",
        lambda: idf.ResolvedIdfEnvironment(mode="path", source="PATH", idf_path_entry="/usr/bin/idf.py"),
    )

    def _raise_not_found(_cmd, cwd, check, **_kwargs):
        raise FileNotFoundError()

    monkeypatch.setattr(idf.subprocess, "run", _raise_not_found)
    with pytest.raises(SystemExit):
        idf.run_idf_command("native", argparse.Namespace(chip="c3", idf_command="build", port=None, clean=False))

    def _raise_called(_cmd, cwd, check, **_kwargs):
        raise subprocess.CalledProcessError(9, ["idf.py"])

    monkeypatch.setattr(idf.subprocess, "run", _raise_called)
    with pytest.raises(SystemExit) as exc:
        idf.run_idf_command("native", argparse.Namespace(chip="c3", idf_command="build", port=None, clean=False))

    assert exc.value.code == 9


def test_resolve_idf_environment_prefers_standard_export(monkeypatch, tmp_path: Path) -> None:
    export_script = tmp_path / "esp" / "esp-idf" / "export.sh"
    export_script.parent.mkdir(parents=True)
    export_script.write_text("#!/bin/sh\n", encoding="utf-8")

    monkeypatch.delenv("IDF_PATH", raising=False)
    monkeypatch.setattr(idf.Path, "home", lambda: tmp_path)
    monkeypatch.setattr(idf.shutil, "which", lambda _binary: None)

    env = idf.resolve_idf_environment()

    assert env.mode == "export"
    assert env.source == "standard ESP-IDF install"
    assert env.export_script == export_script
    assert env.export_kind == "sh"


def test_resolve_idf_environment_reuses_esphome_native_toolchain(monkeypatch, tmp_path: Path) -> None:
    tools_path = tmp_path / "idf"
    framework_path = tools_path / "frameworks" / idf_container.IDF_VERSION
    python_env_path = tools_path / "penvs" / idf_container.IDF_VERSION
    idf_py = framework_path / "tools" / "idf.py"
    python_executable = python_env_path / "bin" / "python"
    idf_py.parent.mkdir(parents=True)
    python_executable.parent.mkdir(parents=True)
    idf_py.write_text("", encoding="utf-8")
    python_executable.write_text("", encoding="utf-8")
    (python_env_path / idf.ESPHOME_IDF_STAMP_FILE).write_text("{}", encoding="utf-8")
    process_env = {"IDF_PATH": str(framework_path)}

    monkeypatch.delenv("IDF_PATH", raising=False)
    monkeypatch.setattr(idf.Path, "home", lambda: tmp_path)
    monkeypatch.setattr(idf.shutil, "which", lambda _binary: None)
    monkeypatch.setattr(idf, "get_esphome_idf_tools_path", lambda: tools_path)
    monkeypatch.setattr(
        idf,
        "build_esphome_idf_process_environment",
        lambda framework, python_env: process_env,
    )

    env = idf.resolve_idf_environment()

    assert env.mode == "esphome"
    assert env.install_dir == framework_path
    assert env.idf_path_entry == str(idf_py)
    assert env.python_executable == python_executable
    assert env.process_env == process_env


def test_resolve_idf_environment_repairs_incomplete_esphome_python_env(
    monkeypatch, tmp_path: Path
) -> None:
    tools_path = tmp_path / "idf"
    framework_path = tools_path / "frameworks" / idf_container.IDF_VERSION
    python_env_path = tools_path / "penvs" / idf_container.IDF_VERSION
    idf_py = framework_path / "tools" / "idf.py"
    python_executable = python_env_path / "bin" / "python"
    idf_py.parent.mkdir(parents=True)
    idf_py.write_text("", encoding="utf-8")
    repaired: list[bool] = []
    process_env = {"IDF_PATH": str(framework_path)}

    def repair_install() -> tuple[Path, Path]:
        repaired.append(True)
        python_executable.parent.mkdir(parents=True)
        python_executable.write_text("", encoding="utf-8")
        return framework_path, python_env_path

    monkeypatch.delenv("IDF_PATH", raising=False)
    monkeypatch.setattr(idf.Path, "home", lambda: tmp_path)
    monkeypatch.setattr(idf.shutil, "which", lambda _binary: None)
    monkeypatch.setattr(idf, "get_esphome_idf_tools_path", lambda: tools_path)
    monkeypatch.setattr(idf, "repair_esphome_managed_idf_install", repair_install)
    monkeypatch.setattr(
        idf,
        "build_esphome_idf_process_environment",
        lambda framework, python_env: process_env,
    )

    env = idf.resolve_idf_environment()

    assert repaired == [True]
    assert env.mode == "esphome"
    assert env.python_executable == python_executable
    assert env.process_env == process_env


def test_run_idf_command_build_uses_esphome_managed_environment(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    (app_dir / "sdkconfig").write_text('CONFIG_IDF_TARGET="esp32c3"\n', encoding="utf-8")
    framework_path = tmp_path / "idf" / "frameworks" / idf_container.IDF_VERSION
    python_executable = tmp_path / "idf" / "penvs" / idf_container.IDF_VERSION / "bin" / "python"
    idf_py = framework_path / "tools" / "idf.py"
    process_env = {"IDF_PATH": str(framework_path)}
    calls: list[tuple[list[str], Path, dict[str, str]]] = []
    env = idf.ResolvedIdfEnvironment(
        mode="esphome",
        source="ESPHome-managed native toolchain",
        install_dir=framework_path,
        idf_path_entry=str(idf_py),
        python_executable=python_executable,
        process_env=process_env,
    )

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(idf, "resolve_idf_environment", lambda: env)
    monkeypatch.setattr(idf, "ccache_binary", lambda path=None: None)
    monkeypatch.setattr(
        idf.subprocess,
        "run",
        lambda cmd, cwd, check, env: calls.append((cmd, Path(cwd), env)),
    )

    idf.run_idf_command("native", argparse.Namespace(chip="c3", idf_command="build", port=None, clean=False))

    assert calls == [
        (
            [
                str(python_executable),
                str(idf_py),
                "-B",
                "build-esp32c3",
                "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults",
                "-DIDF_TARGET=esp32c3",
                "-DSDKCONFIG=build-esp32c3/sdkconfig",
                "build",
            ],
            app_dir,
            process_env,
        )
    ]


def test_prepare_idf_subprocess_command_uses_standard_export(monkeypatch, tmp_path: Path) -> None:
    export_script = tmp_path / "esp" / "esp-idf" / "export.sh"
    export_script.parent.mkdir(parents=True)
    export_script.write_text("#!/bin/sh\n", encoding="utf-8")

    monkeypatch.setattr(
        idf.shutil,
        "which",
        lambda binary: {"bash": "/bin/bash", "zsh": None}.get(binary),
    )

    env = idf.ResolvedIdfEnvironment(
        mode="export",
        source="standard ESP-IDF install",
        install_dir=export_script.parent,
        export_script=export_script,
        export_kind="sh",
    )
    command, used_export = idf.prepare_idf_subprocess_command(["idf.py", "build"], env)

    assert command == ["/bin/bash", "-lc", f". {shlex.quote(str(export_script))} >/dev/null && idf.py build"]
    assert used_export == export_script


def test_prepare_idf_subprocess_command_sequence_combines_exported_build_steps(
    monkeypatch, tmp_path: Path
) -> None:
    export_script = tmp_path / "esp" / "esp-idf" / "export.sh"
    export_script.parent.mkdir(parents=True)
    export_script.write_text("#!/bin/sh\n", encoding="utf-8")

    monkeypatch.setattr(
        idf.shutil,
        "which",
        lambda binary: {"bash": "/bin/bash", "zsh": None}.get(binary),
    )

    env = idf.ResolvedIdfEnvironment(
        mode="export",
        source="standard ESP-IDF install",
        install_dir=export_script.parent,
        export_script=export_script,
        export_kind="sh",
    )
    command, used_export = idf.prepare_idf_subprocess_command_sequence(
        [
            ["idf.py", "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi", "reconfigure"],
            ["idf.py", "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi", "build"],
        ],
        env,
    )

    assert command == [
        "/bin/bash",
        "-lc",
        (
            f". {shlex.quote(str(export_script))} >/dev/null"
            " && idf.py '-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi' reconfigure"
            " && idf.py '-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi' build"
        ),
    ]
    assert used_export == export_script


def test_run_idf_command_build_uses_single_exported_subprocess(monkeypatch, tmp_path: Path, idf_publication_stub) -> None:
    app_dir = tmp_path / "app"
    app_dir.mkdir()
    (app_dir / "sdkconfig.wifi").write_text("", encoding="utf-8")
    calls: list[tuple[list[str], Path]] = []
    export_script = tmp_path / "esp" / "esp-idf" / "export.sh"
    export_script.parent.mkdir(parents=True)
    export_script.write_text("#!/bin/sh\n", encoding="utf-8")

    monkeypatch.setattr(idf, "resolve_idf_target", lambda *_args: (app_dir, "esp32c3"))
    monkeypatch.setattr(
        idf.shutil,
        "which",
        lambda binary: {"bash": "/bin/bash", "zsh": None}.get(binary),
    )
    monkeypatch.setattr(
        idf,
        "resolve_idf_environment",
        lambda: idf.ResolvedIdfEnvironment(
            mode="export",
            source="standard ESP-IDF install",
            install_dir=export_script.parent,
            export_script=export_script,
            export_kind="sh",
        ),
    )
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, cwd, check, **_kwargs: calls.append((cmd, Path(cwd))))

    idf.run_idf_command("native", argparse.Namespace(chip="c3", idf_command="build", port=None, clean=False))

    assert calls == [
        (
            [
                "/bin/bash",
                "-lc",
                (
                    f". {shlex.quote(str(export_script))} >/dev/null"
                    " && idf.py -B build-esp32c3 '-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.wifi'"
                    " -DIDF_TARGET=esp32c3 -DSDKCONFIG=build-esp32c3/sdkconfig build"
                ),
            ],
            app_dir,
        )
    ]


def test_resolve_idf_environment_supports_windows_export_bat(monkeypatch, tmp_path: Path) -> None:
    export_script = tmp_path / "esp" / "esp-idf" / "export.bat"
    export_script.parent.mkdir(parents=True)
    export_script.write_text("@echo off\r\n", encoding="utf-8")

    monkeypatch.setattr(idf, "is_windows_host", lambda: True)
    monkeypatch.setenv("USERPROFILE", str(tmp_path))
    monkeypatch.delenv("IDF_PATH", raising=False)
    monkeypatch.setattr(idf.shutil, "which", lambda _binary: None)

    env = idf.resolve_idf_environment()

    assert env.mode == "export"
    assert env.source == "standard ESP-IDF install"
    assert env.export_script == export_script
    assert env.export_kind == "bat"


def test_run_idf_doctor_uses_export_fallback_on_windows(monkeypatch, tmp_path: Path) -> None:
    export_script = tmp_path / "esp" / "esp-idf" / "export.bat"
    export_script.parent.mkdir(parents=True)
    export_script.write_text("@echo off\r\n", encoding="utf-8")
    calls: list[list[str]] = []

    monkeypatch.setattr(idf, "is_windows_host", lambda: True)
    monkeypatch.setenv("USERPROFILE", str(tmp_path))
    monkeypatch.delenv("IDF_PATH", raising=False)
    monkeypatch.setattr(
        idf.shutil,
        "which",
        lambda binary: {"idf.py": None, "cmd": "cmd.exe"}.get(binary),
    )
    monkeypatch.setattr(idf.subprocess, "run", lambda cmd, check, **_kwargs: calls.append(cmd))

    assert idf.run_idf_doctor(argparse.Namespace()) == 0
    assert calls == [["cmd.exe", "/d", "/c", f'call "{export_script}" >NUL && idf.py --version']]


class _FakeMQTTClient:
    def __init__(self):
        self.username = None
        self.password = None
        self.subscriptions: list[str] = []
        self.unsubscriptions: list[str] = []
        self.published: list[tuple[str, str]] = []
        self.connected: list[tuple[str, int, int]] = []
        self.loop_started = 0
        self.loop_stopped = 0
        self.disconnected = 0
        self.raise_publish = False
        self.raise_connect: Exception | None = None
        self.on_connect = None
        self.on_message = None
        self.auto_ack = True

    def username_pw_set(self, username: str, password: str) -> None:
        self.username = username
        self.password = password

    def subscribe(self, topic: str) -> None:
        self.subscriptions.append(topic)

    def unsubscribe(self, topic: str) -> None:
        self.unsubscriptions.append(topic)

    def publish(self, topic: str, payload: str) -> None:
        if self.raise_publish:
            raise RuntimeError("publish failed")
        self.published.append((topic, payload))
        if not self.auto_ack or self.on_message is None or not topic.endswith("/commands/request"):
            return
        data = json.loads(payload)
        command = str(data.get("command") or "")
        base = topic[: -len("/commands/request")]
        self.on_message(
            self,
            None,
            SimpleNamespace(
                topic=f"{base}/commands/result",
                payload=json.dumps(
                    {
                        "command_id": data.get("command_id", ""),
                        "command": command,
                        "accepted": True,
                        "code": "ok",
                        "message": f"{command} returned" if command else "ok",
                        "data": {
                            "device_id": "0x0000000000000001",
                            "operations": [
                                {"name": "update_device"},
                                {"name": "update_sensing"},
                                {"name": "read_diagnostics"},
                                {"name": "check_ota"},
                            ],
                        } if command == "capabilities" else {"device_id": "0x0000000000000001"},
                    }
                ).encode(),
            ),
        )

    def connect(self, host: str, port: int, keepalive: int) -> None:
        if self.raise_connect is not None:
            raise self.raise_connect
        self.connected.append((host, port, keepalive))

    def loop_start(self) -> None:
        self.loop_started += 1

    def loop_stop(self) -> None:
        self.loop_stopped += 1

    def disconnect(self) -> None:
        self.disconnected += 1


class _FakePromptSession:
    def __init__(self, responses: list[object]):
        self._responses = list(responses)

    def prompt(self, _prompt):
        if not self._responses:
            raise EOFError()
        response = self._responses.pop(0)
        if isinstance(response, BaseException):
            raise response
        return response


def _build_shell(
    monkeypatch,
    responses: list[object] | None = None,
    device_id: str | None = "0x0000000000000001",
):
    client = _FakeMQTTClient()
    prompt_session = _FakePromptSession(responses or [])
    rendered: list[object] = []

    monkeypatch.setattr(mqtt_shell.mqtt, "Client", lambda *args, **kwargs: client)
    monkeypatch.setattr(mqtt_shell, "PromptSession", lambda **_kwargs: prompt_session)
    monkeypatch.setattr(mqtt_shell, "FileHistory", lambda _path: None)
    monkeypatch.setattr(mqtt_shell.NestedCompleter, "from_nested_dict", lambda _data: None)
    monkeypatch.setattr(mqtt_shell.PromptStyle, "from_dict", lambda data: data)
    monkeypatch.setattr(mqtt_shell, "print_formatted_text", lambda *args, **kwargs: rendered.append((args, kwargs)))
    monkeypatch.setattr(mqtt_shell.time, "sleep", lambda _seconds: None)
    shell = mqtt_shell.EspectreMQTTShell(
        argparse.Namespace(
            broker="broker.local",
            port=1883,
            topic_prefix="espectre/v1/devices",
            device_id=device_id,
            username="user",
            password="pass",
        )
    )
    return shell, client, rendered


def test_mqtt_shell_initialization_and_connect_callbacks(monkeypatch, capsys) -> None:
    shell, client, _rendered = _build_shell(monkeypatch)

    assert shell.topic_cmd == "espectre/v1/devices/0x0000000000000001/commands/request"
    assert shell.topic_responses == [
        "espectre/v1/devices/0x0000000000000001/commands/result",
        "espectre/v1/devices/0x0000000000000001/capabilities",
        "espectre/v1/devices/0x0000000000000001/device",
        "espectre/v1/devices/0x0000000000000001/health",
        "espectre/v1/devices/0x0000000000000001/sensing",
        "espectre/v1/devices/0x0000000000000001/wifi",
        "espectre/v1/devices/0x0000000000000001/ota",
    ]
    assert client.username == "user"
    assert client.password == "pass"

    shell.on_connect(client, None, None, 0)
    shell.on_connect(client, None, None, 5)
    captured = capsys.readouterr().out

    assert client.subscriptions == [
        "espectre/v1/devices/0x0000000000000001/commands/result",
        "espectre/v1/devices/0x0000000000000001/capabilities",
        "espectre/v1/devices/0x0000000000000001/device",
        "espectre/v1/devices/0x0000000000000001/health",
        "espectre/v1/devices/0x0000000000000001/sensing",
        "espectre/v1/devices/0x0000000000000001/wifi",
        "espectre/v1/devices/0x0000000000000001/ota",
    ]
    assert "Connected to: broker.local:1883" in captured
    assert "Failed to connect, return code 5" in captured


def test_mqtt_shell_discovers_and_selects_device(monkeypatch, capsys) -> None:
    shell, client, _rendered = _build_shell(monkeypatch, device_id=None)
    monkeypatch.setattr("builtins.input", lambda _prompt: "1")

    shell.on_connect(client, None, None, 0)
    shell.on_message(
        None,
        None,
        SimpleNamespace(
            topic="espectre/v1/devices/0x00000000000000aa/device",
            payload=(
                b'{"device_id":"0x00000000000000aa","name":"ESPectre C6 00aa",'
                b'"label":"Lab","frontend":"micro"}'
            ),
        ),
    )
    shell.on_message(
        None,
        None,
        SimpleNamespace(
            topic="espectre/v1/devices/0x00000000000000aa/health",
            payload=b'{"device_id":"0x00000000000000aa","online":true}',
        ),
    )

    assert shell.select_device() is True
    captured = capsys.readouterr().out

    assert shell.device_id == "0x00000000000000aa"
    assert shell.topic_cmd == "espectre/v1/devices/0x00000000000000aa/commands/request"
    assert client.subscriptions == [
        "espectre/v1/devices/+/device",
        "espectre/v1/devices/+/health",
        "espectre/v1/devices/0x00000000000000aa/commands/result",
        "espectre/v1/devices/0x00000000000000aa/capabilities",
        "espectre/v1/devices/0x00000000000000aa/device",
        "espectre/v1/devices/0x00000000000000aa/health",
        "espectre/v1/devices/0x00000000000000aa/sensing",
        "espectre/v1/devices/0x00000000000000aa/wifi",
        "espectre/v1/devices/0x00000000000000aa/ota",
    ]
    assert client.unsubscriptions == [
        "espectre/v1/devices/+/device",
        "espectre/v1/devices/+/health",
    ]
    assert "Discovered MQTT devices:" in captured
    assert "Selected device: 0x00000000000000aa" in captured


def test_mqtt_shell_guards_discovery_updates_and_snapshots(monkeypatch) -> None:
    shell, _client, _rendered = _build_shell(monkeypatch, device_id=None)

    class GuardedDevices(dict):
        def setdefault(self, key, default=None):
            assert shell._discovery_lock.locked()
            return super().setdefault(key, default)

        def values(self):
            assert shell._discovery_lock.locked()
            return super().values()

    shell.discovered_devices = GuardedDevices()
    shell._record_discovered_device(
        "espectre/v1/devices/device-a/device",
        b'{"device_id":"device-a","label":"Lab"}',
    )

    devices = shell._print_discovered_devices()

    assert devices == [{"device_id": "device-a", "label": "Lab"}]


def test_mqtt_shell_message_send_and_command_routing(monkeypatch, capsys) -> None:
    shell, client, rendered = _build_shell(monkeypatch)
    cleared: list[str] = []

    monkeypatch.setattr(mqtt_shell.os, "system", lambda cmd: cleared.append(cmd))

    shell.on_message(None, None, SimpleNamespace(payload=b'{"ok": true}'))
    shell.on_message(
        None,
        None,
        SimpleNamespace(
            topic="espectre/v1/devices/0x0000000000000001/device",
            payload=b'{"device_id":"0x0000000000000001","frontend":"native"}',
        ),
    )
    shell.on_message(
        None,
        None,
        SimpleNamespace(
            topic="espectre/v1/devices/0x0000000000000001/commands/result",
            payload=b'{"command":"update_device","accepted":true,"message":"device updated"}',
        ),
    )
    shell.on_message(
        None,
        None,
        SimpleNamespace(
            topic="espectre/v1/devices/0x0000000000000001/commands/result",
            payload=b'{"command":"update_sensing","accepted":false,"message":"invalid threshold"}',
        ),
    )
    shell.on_message(None, None, SimpleNamespace(payload=b"not-json"))
    shell.send_command({"command": "update_device", "label": "Lab"})
    client.raise_publish = True
    shell.send_command({"command": "read_diagnostics"})
    client.raise_publish = False

    shell.process_input("")
    shell.process_input("update_device label=Lab")
    shell.process_input("read_diagnostics")
    shell.process_input("update_sensing 0.35")
    shell.process_input("check_ota")
    shell.process_input("start_ota")
    shell.process_input("check_ota unexpected")
    shell.process_input("start_ota unexpected")
    shell.process_input("clear")
    shell.process_input("help")
    shell.process_input("about")
    shell.process_input("unknown")
    shell.process_input("exit")

    captured = capsys.readouterr().out
    published = [json.loads(payload) for _, payload in client.published]
    assert client.published[0][0] == shell.topic_cmd
    assert [item["command"] for item in published] == [
        "update_device",
        "update_device",
        "read_diagnostics",
        "update_sensing",
        "check_ota",
        "start_ota",
        "unknown",
    ]
    assert published[3]["threshold"] == 0.35
    assert cleared == ["clear"]
    assert rendered
    assert "Received:" in captured
    assert "Received on device:" in captured
    assert "✓ update_device" in captured
    assert "Received on commands/result:" not in captured
    assert "device updated" not in captured
    assert "Error parsing message" in captured
    assert "Error sending command" in captured
    assert "Unknown command: unknown" not in captured
    assert "invalid ota channel (accepted: release, preview, and develop)" in captured
    assert shell.running is False


def test_mqtt_shell_updates_pending_events_under_the_state_lock(monkeypatch) -> None:
    shell, _client, _rendered = _build_shell(monkeypatch)

    class LockAwareEvent:
        def __init__(self):
            self.event = threading.Event()
            self.clear_count = 0

        def clear(self):
            assert shell._pending_lock.locked()
            self.clear_count += 1
            self.event.clear()

        def set(self):
            assert shell._pending_lock.locked()
            self.event.set()

        def wait(self, timeout=None):
            return self.event.wait(timeout)

    result_event = LockAwareEvent()
    payload_event = LockAwareEvent()
    shell._pending_result_event = result_event
    shell._pending_payload_event = payload_event

    shell.send_command({"command": "update_sensing", "threshold": 0.5})

    assert result_event.clear_count == 2
    assert payload_event.clear_count == 2


def test_mqtt_command_payload_parses_resource_updates_and_key_value_tokens() -> None:
    payload, error = mqtt_shell._mqtt_command_payload("update_sensing", ["0.35"])
    assert error is None
    assert payload == {"command": "update_sensing", "threshold": 0.35}

    payload, error = mqtt_shell._mqtt_command_payload("update_sensing", ["detector=lightweight"])
    assert error is None
    assert payload == {"command": "update_sensing", "detector": "lightweight"}

    payload, error = mqtt_shell._mqtt_command_payload(
        "update_sensing",
        ["motion_on_hits=4", "motion_off_hits=3"],
    )
    assert error is None
    assert payload == {"command": "update_sensing", "motion_on_hits": 4, "motion_off_hits": 3}

    payload, error = mqtt_shell._mqtt_command_payload("f", [])
    assert error is None
    assert payload == {"command": "f"}

    payload, error = mqtt_shell._mqtt_command_payload("check_ota", ["unexpected"])
    assert payload is None
    assert error == "invalid ota channel (accepted: release, preview, and develop)"

    payload, error = mqtt_shell._mqtt_command_payload("check_ota", ["preview"])
    assert error is None
    assert payload == {"command": "check_ota", "channel": "preview"}

    payload, error = mqtt_shell._mqtt_command_payload("start_ota", ["channel=develop"])
    assert error is None
    assert payload == {"command": "start_ota", "channel": "develop"}

    payload, error = mqtt_shell._mqtt_command_payload("start_ota", ["channel=latest"])
    assert payload is None
    assert error == "invalid ota channel (accepted: release, preview, and develop)"


def test_mqtt_shell_builds_command_catalog_from_device_payloads(monkeypatch) -> None:
    catalog = {"operations": [{"name": "update_device"}, {"name": "update_sensing"}]}
    assert mqtt_shell._mqtt_commands_from_catalog(catalog) == ["update_device", "update_sensing"]

    shell, _client, rendered = _build_shell(monkeypatch)
    catalogs: list[dict[str, object]] = []
    monkeypatch.setattr(
        mqtt_shell.NestedCompleter,
        "from_nested_dict",
        lambda data: catalogs.append(dict(data)) or object(),
    )
    shell._apply_catalog_payload({"operations": [{"name": "update_device"}, {"name": "update_sensing"}, {"name": "scan_wifi"}]})
    assert shell._device_commands == ["update_device", "update_sensing"]
    assert catalogs[-1]["update_device"] is None
    assert catalogs[-1]["update_sensing"] is None
    assert catalogs[-1]["st"] is None
    assert catalogs[-1]["help"] is None
    assert "scan_wifi" not in catalogs[-1]

    shell.show_help()
    help_arg = rendered[-1][0][0]
    help_html = getattr(help_arg, "value", str(help_arg))
    assert "update_sensing" in help_html
    assert "Device commands" in help_html
    assert "st 0.35" in help_html
    assert "key=value" not in help_html


def test_mqtt_shell_annotates_typed_command_on_tty(monkeypatch) -> None:
    shell, _client, _rendered = _build_shell(monkeypatch)
    writes: list[str] = []
    monkeypatch.setattr(shell, "_can_annotate_typed_command", lambda _typed: True)
    monkeypatch.setattr(mqtt_shell.sys.stdout, "write", lambda text: writes.append(text) or len(text))
    monkeypatch.setattr(mqtt_shell.sys.stdout, "flush", lambda: None)

    shell.process_input("update_device label=Lab")
    output = "".join(writes)
    assert "\033[A" in output or "\x1b[A" in output
    assert "✓" in output
    assert "✗" not in output


def test_mqtt_shell_completes_when_reject_omits_command_id(monkeypatch, capsys) -> None:
    shell, client, _rendered = _build_shell(monkeypatch)
    client.auto_ack = False

    def publish(topic: str, payload: str) -> None:
        client.published.append((topic, payload))
        client.on_message(
            client,
            None,
            SimpleNamespace(
                    topic=topic.replace("/commands/request", "/commands/result"),
                payload=b'{"command":"unknown","accepted":false,"message":"invalid command"}',
            ),
        )

    client.publish = publish
    shell.process_input("unknown")
    captured = capsys.readouterr().out
    assert "invalid command" in captured
    assert "timed out waiting for device" not in captured


def test_mqtt_shell_start_handles_prompt_loop_and_shutdown(monkeypatch, capsys) -> None:
    shell, client, _rendered = _build_shell(monkeypatch, responses=[KeyboardInterrupt(), "update_device label=Lab", EOFError()])

    shell.start()
    captured = capsys.readouterr().out

    assert client.connected == [("broker.local", 1883, 60)]
    assert client.loop_started == 1
    assert client.loop_stopped == 1
    assert client.disconnected == 1
    assert "Type 'help' for commands" in captured
    assert "Exiting..." in captured
    assert len(client.published) == 1
    assert [json.loads(payload)["command"] for _, payload in client.published] == ["update_device"]


def test_send_mqtt_command_and_wait_waits_for_suback(monkeypatch) -> None:
    class FakeClient:
        def __init__(self) -> None:
            self.on_connect = None
            self.on_message = None
            self.on_subscribe = None
            self.suback_received = False
            self.next_mid = 1

        def connect(self, host: str, port: int, keepalive: int) -> None:
            assert (host, port, keepalive) == ("broker.local", 1883, 60)

        def loop_start(self) -> None:
            assert self.on_connect is not None
            self.on_connect(self, None, None, 0)

        def loop_stop(self) -> None:
            return None

        def disconnect(self) -> None:
            return None

        def subscribe(self, topic: str):
            mid = self.next_mid
            self.next_mid += 1
            threading.Timer(0.01, lambda: self._ack_subscribe(mid)).start()
            return (mqtt_shell.mqtt.MQTT_ERR_SUCCESS, mid)

        def _ack_subscribe(self, mid: int) -> None:
            self.suback_received = True
            assert self.on_subscribe is not None
            self.on_subscribe(self, None, mid, [0])

        def publish(self, topic: str, payload: str):
            assert self.suback_received is True
            data = json.loads(payload)
            assert topic.endswith("/commands/request")
            assert self.on_message is not None
            self.on_message(
                self,
                None,
                SimpleNamespace(
                    topic=topic.replace("/request", "/accepted"),
                    payload=json.dumps({"command_id": data["command_id"], "accepted": True}).encode(),
                ),
            )
            return SimpleNamespace(rc=mqtt_shell.mqtt.MQTT_ERR_SUCCESS)

    monkeypatch.setattr(mqtt_shell, "_make_mqtt_client", lambda *_args, **_kwargs: FakeClient())

    response = mqtt_shell.send_mqtt_command_and_wait(
        argparse.Namespace(
            broker="broker.local",
            port=1883,
            topic_prefix="espectre/v1/devices",
            device_id="0x1234",
            username="",
            password="",
        ),
        {"command": "set_detector", "detector": "high_accuracy"},
        timeout_s=0.5,
    )

    assert response["accepted"] is True


def test_request_mqtt_diagnostics_and_wait_waits_for_all_subacks(monkeypatch) -> None:
    class FakeClient:
        def __init__(self) -> None:
            self.on_connect = None
            self.on_message = None
            self.on_subscribe = None
            self.suback_count = 0
            self.next_mid = 1

        def connect(self, host: str, port: int, keepalive: int) -> None:
            assert (host, port, keepalive) == ("broker.local", 1883, 60)

        def loop_start(self) -> None:
            assert self.on_connect is not None
            self.on_connect(self, None, None, 0)

        def loop_stop(self) -> None:
            return None

        def disconnect(self) -> None:
            return None

        def subscribe(self, topic: str):
            mid = self.next_mid
            self.next_mid += 1
            threading.Timer(0.01, lambda: self._ack_subscribe(mid)).start()
            return (mqtt_shell.mqtt.MQTT_ERR_SUCCESS, mid)

        def _ack_subscribe(self, mid: int) -> None:
            self.suback_count += 1
            assert self.on_subscribe is not None
            self.on_subscribe(self, None, mid, [0])

        def publish(self, topic: str, payload: str):
            assert self.suback_count == 1
            data = json.loads(payload)
            base = topic.removesuffix("/commands/request")
            assert self.on_message is not None
            self.on_message(
                self,
                None,
                SimpleNamespace(
                    topic=f"{base}/commands/result",
                    payload=json.dumps({
                        "command_id": data["command_id"],
                        "command": "read_diagnostics",
                        "accepted": True,
                        "code": "ok",
                        "message": "diagnostics returned",
                        "data": {"uptime": 42},
                    }).encode(),
                ),
            )
            return SimpleNamespace(rc=mqtt_shell.mqtt.MQTT_ERR_SUCCESS)

    monkeypatch.setattr(mqtt_shell, "_make_mqtt_client", lambda *_args, **_kwargs: FakeClient())

    command_response, diagnostics_response = mqtt_shell.request_mqtt_diagnostics_and_wait(
        argparse.Namespace(
            broker="broker.local",
            port=1883,
            topic_prefix="espectre/v1/devices",
            device_id="0x1234",
            username="",
            password="",
        ),
        timeout_s=0.5,
    )

    assert command_response["accepted"] is True
    assert diagnostics_response["uptime"] == 42


def test_send_mqtt_command_and_wait_reports_request_echo_on_timeout(monkeypatch) -> None:
    class FakeClient:
        def __init__(self) -> None:
            self.on_connect = None
            self.on_message = None
            self.on_subscribe = None
            self.next_mid = 1

        def connect(self, host: str, port: int, keepalive: int) -> None:
            assert (host, port, keepalive) == ("broker.local", 1883, 60)

        def loop_start(self) -> None:
            assert self.on_connect is not None
            self.on_connect(self, None, None, 0)

        def loop_stop(self) -> None:
            return None

        def disconnect(self) -> None:
            return None

        def subscribe(self, topic: str):
            mid = self.next_mid
            self.next_mid += 1
            threading.Timer(0.01, lambda: self._ack_subscribe(mid)).start()
            return (mqtt_shell.mqtt.MQTT_ERR_SUCCESS, mid)

        def _ack_subscribe(self, mid: int) -> None:
            assert self.on_subscribe is not None
            self.on_subscribe(self, None, mid, [0])

        def publish(self, topic: str, payload: str):
            data = json.loads(payload)
            assert self.on_message is not None
            self.on_message(
                self,
                None,
                SimpleNamespace(
                    topic=topic,
                    payload=json.dumps(data).encode(),
                ),
            )
            return SimpleNamespace(rc=mqtt_shell.mqtt.MQTT_ERR_SUCCESS)

    monkeypatch.setattr(mqtt_shell, "_make_mqtt_client", lambda *_args, **_kwargs: FakeClient())

    with pytest.raises(RuntimeError, match=r"request_echo=yes"):
        mqtt_shell.send_mqtt_command_and_wait(
            argparse.Namespace(
                broker="broker.local",
                port=1883,
                topic_prefix="espectre/v1/devices",
                device_id="0x1234",
                username="",
                password="",
            ),
            {"command": "set_detector", "detector": "high_accuracy"},
            timeout_s=0.1,
            observe_request_echo=True,
        )


def test_run_mqtt_shell_and_main_dispatch(monkeypatch) -> None:
    calls: list[object] = []

    class FakeShell:
        def __init__(self, args):
            calls.append(("shell", args.device_id, args.port))

        def start(self):
            calls.append("start")

    monkeypatch.setattr(app, "EspectreMQTTShell", FakeShell)

    assert app.run_mqtt_shell(_mqtt_args()) == 0
    assert calls == [("shell", "0x0000111122223333", 1884), "start"]

    monkeypatch.setattr(app, "run_mqtt_shell", lambda args: calls.append(("mqtt", args.namespace)) or 0)
    assert app.main([]) == 0
    assert app.main(["mqtt"]) == 0
    assert ("mqtt", "mqtt") in calls

    with pytest.raises(SystemExit):
        app.main(["micro"])
