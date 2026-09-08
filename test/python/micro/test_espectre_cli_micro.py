# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - CLI Micro Tests

Tests for espectre_cli.micro host-side helpers.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

import argparse
import ast
import subprocess
import tokenize
from io import BytesIO
from pathlib import Path
from types import SimpleNamespace

import pytest

from espectre_cli import micro
from espectre_cli import micro_firmware
from espectre_cli.common import (
    FIRMWARE_CACHE_DIR,
    MICRO_CHIP_CHOICES,
    MICRO_ESPECTRE_SRC_DIR,
)


def test_firmware_cache_is_scoped_to_micro_espectre_project() -> None:
    assert FIRMWARE_CACHE_DIR == MICRO_ESPECTRE_SRC_DIR / ".firmware"


def test_project_firmware_preserves_and_refreshes_stale_pinned_sources(
    tmp_path: Path,
) -> None:
    micropython_dir = tmp_path / "micro-esp32" / "micropython"
    source_path = micropython_dir / "ports" / "esp32" / "network_wlan_csi.c"
    source_path.parent.mkdir(parents=True)
    source_path.write_text("pinned\n", encoding="utf-8")
    subprocess.run(["git", "init", "-q", str(micropython_dir)], check=True)
    subprocess.run(
        ["git", "-C", str(micropython_dir), "add", str(source_path)], check=True
    )
    subprocess.run(
        [
            "git",
            "-C",
            str(micropython_dir),
            "-c",
            "user.name=ESPectre Tests",
            "-c",
            "user.email=tests@espectre.local",
            "commit",
            "-qm",
            "fixture",
        ],
        check=True,
    )
    source_path.write_text("variable prototype\n", encoding="utf-8")

    stamp_path = micro_firmware._prepare_micropython_patch_revision(micropython_dir)

    assert source_path.read_text(encoding="utf-8") == "pinned\n"
    assert not stamp_path.exists()
    backup_path = (
        micropython_dir.parent
        / f"micropython-before-{micro_firmware.MICROPYTHON_PATCH_REVISION}.patch"
    )
    assert "variable prototype" in backup_path.read_text(encoding="utf-8")


def _make_args(**overrides) -> argparse.Namespace:
    args = {
        "port": None,
        "chip": "c3",
        "erase": False,
        "firmware": None,
        "clean": False,
    }
    args.update(overrides)
    return argparse.Namespace(**args)


def _make_verify_args(**overrides) -> argparse.Namespace:
    args = {"port": None}
    args.update(overrides)
    return argparse.Namespace(**args)


def _create_micro_src_tree(base_dir: Path) -> None:
    for rel_path in micro.MICRO_DEVICE_RELATIVE_FILES:
        target = base_dir / rel_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("# test\n", encoding="utf-8")


def _string_token_is_fstring(token_string: str) -> bool:
    prefix = []
    for char in token_string:
        if char in "\"'":
            break
        prefix.append(char.lower())
    return "f" in prefix


def test_device_sources_avoid_unsupported_future_annotations() -> None:
    for rel_path in micro.MICRO_DEVICE_RELATIVE_FILES:
        source = micro.PYTHON_SRC_DIR / rel_path
        if not source.exists():
            # config_local.py only exists after local setup; check the
            # shipped template instead.
            source = source.with_name(source.name + ".example")
            assert source.exists(), rel_path
        assert "from __future__ import annotations" not in source.read_text(
            encoding="utf-8"
        ), rel_path


def test_device_sources_avoid_implicit_fstring_concatenation() -> None:
    skip_types = {
        tokenize.ENCODING,
        tokenize.ENDMARKER,
        tokenize.NL,
        tokenize.NEWLINE,
        tokenize.INDENT,
        tokenize.DEDENT,
        tokenize.COMMENT,
        tokenize.FSTRING_MIDDLE,
    }
    violations: list[str] = []
    for rel_path in micro.MICRO_DEVICE_RELATIVE_FILES:
        source_path = micro.PYTHON_SRC_DIR / rel_path
        if not source_path.exists():
            source_path = source_path.with_name(source_path.name + ".example")
            assert source_path.exists(), rel_path
        previous_ended_string = False
        previous_was_fstring = False
        for tok in tokenize.tokenize(
            BytesIO(source_path.read_bytes()).readline
        ):
            if tok.type in skip_types:
                continue
            starts_string = tok.type in (tokenize.STRING, tokenize.FSTRING_START)
            ends_string = tok.type in (tokenize.STRING, tokenize.FSTRING_END)
            is_fstring = tok.type in (
                tokenize.FSTRING_START,
                tokenize.FSTRING_END,
            ) or (
                tok.type == tokenize.STRING
                and _string_token_is_fstring(tok.string)
            )
            if (
                starts_string
                and previous_ended_string
                and (previous_was_fstring or is_fstring)
            ):
                violations.append(f"{rel_path}:{tok.start[0]}")
            previous_ended_string = ends_string
            previous_was_fstring = bool(ends_string and is_fstring)
    assert violations == []


def test_deploy_manifest_contains_local_runtime_imports() -> None:
    deployed = set(micro.MICRO_DEVICE_RELATIVE_FILES)
    missing: set[str] = set()
    for rel_path in deployed:
        if rel_path == "config_local.py":
            continue
        source_path = micro.PYTHON_SRC_DIR / rel_path
        tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=rel_path)
        parent = Path(rel_path).parent
        for node in ast.walk(tree):
            if not isinstance(node, ast.ImportFrom):
                continue
            if node.module == "src":
                required = (f"{alias.name.replace('.', '/')}.py" for alias in node.names)
            elif node.module and node.module.startswith("src."):
                required = (f"{node.module[4:].replace('.', '/')}.py",)
            elif node.level == 1 and node.module:
                required = (str(parent / f"{node.module.replace('.', '/')}.py"),)
            else:
                continue
            missing.update(required_path for required_path in required if required_path not in deployed)

    assert missing == set()


def test_require_mpremote_accepts_installed_binary(monkeypatch) -> None:
    calls: list[list[str]] = []

    def fake_run(cmd, capture_output, check):
        calls.append(cmd)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    micro._require_mpremote()

    assert calls == [["mpremote", "--version"]]


def test_require_mpremote_exits_when_binary_missing(monkeypatch) -> None:
    def fake_run(cmd, capture_output, check):
        raise FileNotFoundError()

    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    with pytest.raises(SystemExit):
        micro._require_mpremote()


def test_require_mpy_cross_accepts_installed_binary(monkeypatch) -> None:
    calls: list[list[str]] = []

    def fake_run(cmd, capture_output, check):
        calls.append(cmd)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    micro._require_mpy_cross()

    assert calls == [["mpy-cross-v6.3", "--version"]]


def test_require_mpy_cross_exits_when_binary_missing(monkeypatch) -> None:
    def fake_run(cmd, capture_output, check):
        raise FileNotFoundError()

    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    with pytest.raises(SystemExit):
        micro._require_mpy_cross()


def test_reset_device_reports_command_result(monkeypatch) -> None:
    calls: list[list[str]] = []
    monkeypatch.setattr(micro.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        micro,
        "_wait_for_micropython",
        lambda _port: (False, "device unavailable"),
    )

    def fake_run(cmd, timeout, capture_output, text, check):
        calls.append(cmd)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(micro.subprocess, "run", fake_run)
    assert micro._reset_device("/dev/cu.usbmodem1") is True

    def fake_run_fail(cmd, timeout, capture_output, text, check):
        raise subprocess.CalledProcessError(1, cmd, stderr="busy")

    monkeypatch.setattr(micro.subprocess, "run", fake_run_fail)
    assert micro._reset_device("/dev/cu.usbmodem1") is False

    assert calls == [["mpremote", "connect", "/dev/cu.usbmodem1", "exec", "import machine; machine.reset()"]]


def test_reset_device_accepts_timeout_when_repl_is_ready(monkeypatch, capsys) -> None:
    monkeypatch.setattr(micro.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        micro,
        "_wait_for_micropython",
        lambda _port: (True, ""),
    )

    def fake_run(cmd, timeout, capture_output, text, check):
        raise subprocess.TimeoutExpired(
            cmd,
            timeout,
            output=b"\x00ESP-ROM:esp32s3\r\nMicroPython\r\n>>> ",
        )

    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    assert micro._reset_device("/dev/cu.usbmodem1") is True
    output = capsys.readouterr().out
    assert "ESP32 reset completed" in output
    assert "ESP-ROM" not in output
    assert "b'" not in output


def test_reset_device_decodes_bytes_when_repl_is_unavailable(monkeypatch, capsys) -> None:
    monkeypatch.setattr(micro.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        micro,
        "_wait_for_micropython",
        lambda _port: (False, ""),
    )

    def fake_run(cmd, timeout, capture_output, text, check):
        raise subprocess.TimeoutExpired(
            cmd,
            timeout,
            output=b"\x00boot output\r\nreset did not complete\r\n",
        )

    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    assert micro._reset_device("/dev/cu.usbmodem1") is False
    output = capsys.readouterr().out
    assert "boot output\nreset did not complete" in output
    assert "b'" not in output
    assert "\\r\\n" not in output


def test_flash_firmware_delegates_canonical_build_to_shared_runner(
    monkeypatch,
    tmp_path: Path,
) -> None:
    build_dir = tmp_path / "build"
    image = build_dir / "firmware.bin"
    image.parent.mkdir()
    image.write_bytes(b"firmware")
    calls: list[tuple[object, ...]] = []
    monkeypatch.setattr(micro, "resolve_serial_port", lambda port, **_kwargs: port)
    monkeypatch.setattr(
        micro,
        "build_project_firmware_artifact",
        lambda **_kwargs: micro_firmware.BuiltProjectFirmware(
            image=image,
            build_dir=build_dir,
        ),
    )
    monkeypatch.setattr(
        micro,
        "flash_build",
        lambda *args, **kwargs: calls.append((*args, kwargs)),
    )

    micro.flash_firmware(
        _make_args(chip="c6", port="/dev/cu.test", erase=True)
    )

    assert calls == [
        (
            build_dir,
            {
                "chip": "c6",
                "idf_target": "esp32c6",
                "port": "/dev/cu.test",
                "erase": True,
            },
        )
    ]


def test_flash_firmware_propagates_first_runner_failure(
    monkeypatch,
    tmp_path: Path,
) -> None:
    artifact = micro_firmware.BuiltProjectFirmware(
        image=tmp_path / "firmware.bin",
        build_dir=tmp_path,
    )
    artifact.image.write_bytes(b"firmware")
    attempts: list[Path] = []
    monkeypatch.setattr(micro, "resolve_serial_port", lambda port, **_kwargs: port)
    monkeypatch.setattr(
        micro,
        "build_project_firmware_artifact",
        lambda **_kwargs: artifact,
    )

    def fail_once(build_dir: Path, **_kwargs) -> None:
        attempts.append(build_dir)
        raise subprocess.CalledProcessError(7, ["esptool"])

    monkeypatch.setattr(micro, "flash_build", fail_once)

    with pytest.raises(SystemExit) as exc_info:
        micro.flash_firmware(
            _make_args(chip="s3", port="/dev/cu.test")
        )

    assert exc_info.value.code == 7
    assert attempts == [tmp_path]


def test_flash_firmware_accepts_s2_profile(monkeypatch, tmp_path: Path) -> None:
    build_dir = tmp_path / "build"
    image = build_dir / "firmware.bin"
    image.parent.mkdir()
    image.write_bytes(b"firmware")
    calls: list[tuple[object, ...]] = []
    monkeypatch.setattr(micro, "resolve_serial_port", lambda port, **_kwargs: port)
    monkeypatch.setattr(
        micro,
        "build_project_firmware_artifact",
        lambda **_kwargs: micro_firmware.BuiltProjectFirmware(
            image=image,
            build_dir=build_dir,
        ),
    )
    monkeypatch.setattr(
        micro,
        "flash_build",
        lambda *args, **kwargs: calls.append((*args, kwargs)),
    )

    micro.flash_firmware(_make_args(chip="s2", port="/dev/cu.test", erase=True))

    assert calls == [
        (
            build_dir,
            {
                "chip": "s2",
                "idf_target": "esp32s2",
                "port": "/dev/cu.test",
                "erase": True,
            },
        )
    ]


def test_s2_deploy_waits_for_usb_reenumeration(monkeypatch) -> None:
    observed = []

    def capture_port(port, **kwargs):
        observed.append((port, kwargs))
        raise SystemExit(1)

    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", capture_port)

    with pytest.raises(SystemExit):
        micro.deploy_code(
            _make_args(chip="s2", port="/dev/cu.usbmodem01")
        )

    assert observed == [
        (
            "/dev/cu.usbmodem01",
            {
                "chip": "s2",
                "frontend": "micro",
                "purpose": "deploy",
                "wait_timeout_s": 10.0,
            },
        )
    ]


def test_deploy_code_requires_config_local(monkeypatch, tmp_path: Path) -> None:
    src_dir = tmp_path / "src"
    src_dir.mkdir()
    monkeypatch.setattr(micro, "PYTHON_SRC_DIR", src_dir)
    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")

    with pytest.raises(SystemExit):
        micro.deploy_code(_make_args())


def test_deploy_code_uploads_files_to_device(monkeypatch, tmp_path: Path) -> None:
    src_dir = tmp_path / "src"
    _create_micro_src_tree(src_dir)
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(cmd)
        if cmd[:4] == ["mpremote", "connect", "/dev/cu.usbmodem1", "exec"]:
            return SimpleNamespace(returncode=0, stdout="MP_OK", stderr="")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(micro, "PYTHON_SRC_DIR", src_dir)
    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    micro.deploy_code(_make_args())

    mkdir_calls = [cmd for cmd in calls if "mkdir" in cmd]
    cp_calls = [cmd for cmd in calls if "cp" in cmd]
    exec_scripts = [cmd[-1] for cmd in calls if cmd[:4] == ["mpremote", "connect", "/dev/cu.usbmodem1", "exec"]]
    assert len(mkdir_calls) == 1
    assert mkdir_calls[0][-1] == ":src.stage"
    assert len(cp_calls) == len(micro.MICRO_DEVICE_RELATIVE_FILES)
    assert any(cmd[-1] == ":src.stage/main.mpy" for cmd in cp_calls)
    assert not any(cmd[-1].startswith(":src.stage/mqtt/") for cmd in cp_calls)
    assert any(cmd[-2].endswith("console_output.mpy") for cmd in cp_calls)
    assert any(cmd[-2].endswith("branding.mpy") for cmd in cp_calls)
    assert any(cmd[-2].endswith("lightweight_detector.mpy") for cmd in cp_calls)
    assert any(cmd[-2].endswith("runtime_diagnostics.mpy") for cmd in cp_calls)
    assert any(cmd[-2].endswith("protocol.mpy") for cmd in cp_calls)
    assert all(cmd[-2].endswith(".mpy") for cmd in cp_calls)
    assert all(cmd[-1].startswith(":src.stage/") for cmd in cp_calls)
    assert any("remove_tree('/src.stage')" in script for script in exec_scripts)
    assert any("os.rename('/src.stage', '/src')" in script for script in exec_scripts)
    assert any("os.rename('/src.previous', '/src')" in script for script in exec_scripts)

    compile_calls = [
        cmd
        for cmd in calls
        if cmd and cmd[0] == micro.MPY_CROSS_COMMAND and micro.MPY_OPTIMIZATION_LEVEL in cmd
    ]
    assert len(compile_calls) == len(micro.MICRO_DEVICE_RELATIVE_FILES)
    assert all(micro.MPY_OPTIMIZATION_LEVEL in cmd for cmd in compile_calls)


def test_project_manifest_does_not_freeze_application(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.py"

    micro_firmware._write_manifest(manifest)

    assert manifest.read_text(encoding="utf-8") == (
        'freeze("$(PORT_DIR)/modules", ("_boot.py", "flashbdev.py", "inisetup.py"))\n'
    )


def test_project_firmware_stages_only_micropython_support(tmp_path: Path) -> None:
    destination = tmp_path / "firmware-support"

    micro_firmware._stage_firmware_support(micro.PYTHON_SRC_DIR, destination)

    assert not (destination / "shared_core").exists()
    assert (
        destination / "components" / "espectre_core" / "CMakeLists.txt"
    ).is_file()
    assert (destination / "native_components" / "micropython.cmake").is_file()


def test_project_firmware_rejects_unsupported_chip(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="Unsupported project firmware chip: h2"):
        micro_firmware.build_project_firmware(tmp_path, chip="h2", cache_dir=tmp_path)


def test_project_firmware_uses_shared_cpp_identity(tmp_path: Path) -> None:
    cmake_path = tmp_path / "ports" / "esp32" / "CMakeLists.txt"
    cmake_path.parent.mkdir(parents=True)
    cmake_path.write_text("project(micropython)\n", encoding="utf-8")
    micro_firmware._configure_project_identity(tmp_path)
    micro_firmware._configure_project_identity(tmp_path)
    identity = tmp_path / "identity.txt"
    script = tmp_path / "verify.cmake"
    script.write_text(
        "cmake_minimum_required(VERSION 3.16)\n"
        f'set(ESPECTRE_CORE_SDK_ROOT "{micro_firmware.REPO_ROOT / "src" / "cpp"}")\n'
        "macro(project name)\n"
        f'  file(WRITE "{identity}" "${{name}}\\n${{PROJECT_VER}}\\n")\n'
        "endmacro()\n"
        f'include("{cmake_path}")\n',
        encoding="utf-8",
    )
    result = subprocess.run(
        ["cmake", "-DESPECTRE_GIT_VERSION=2.8.0-417-g2b49a9c", "-P", str(script)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert identity.read_text(encoding="utf-8").splitlines() == [
        micro_firmware.PROJECT_FIRMWARE_PROJECT_NAME, "2.8.0-417-g2b49a9c"
    ]
    defaults = (
        micro.PYTHON_SRC_DIR / "firmware" / "boards" / "sdkconfig.micro_espectre"
    ).read_text(encoding="utf-8").splitlines()
    for key in ("APP_EXCLUDE_PROJECT_NAME_VAR", "APP_EXCLUDE_PROJECT_VER_VAR", "APP_PROJECT_VER_FROM_CONFIG"):
        assert f"CONFIG_{key}=n" in defaults


def test_project_firmware_rejects_unexpected_upstream_project(tmp_path: Path) -> None:
    cmake_path = tmp_path / "ports" / "esp32" / "CMakeLists.txt"
    cmake_path.parent.mkdir(parents=True)
    cmake_path.write_text("project(unrelated)\n", encoding="utf-8")
    with pytest.raises(RuntimeError):
        micro_firmware._configure_project_identity(tmp_path)
    assert cmake_path.read_text(encoding="utf-8") == "project(unrelated)\n"


def test_project_firmware_aligns_idf_55_lockfile(tmp_path: Path) -> None:
    micropython_dir = tmp_path / "micropython"
    lockfile = (
        micropython_dir
        / "ports"
        / "esp32"
        / "lockfiles"
        / "dependencies.lock.esp32s3"
    )
    lockfile.parent.mkdir(parents=True)
    lockfile.write_text(
        "dependencies:\n  idf:\n    source:\n      type: idf\n    version: 5.5.2\n",
        encoding="utf-8",
    )
    micro_firmware._align_idf_lockfile(micropython_dir, "s3", "5.5.5")

    assert "    version: 5.5.5\n" in lockfile.read_text(encoding="utf-8")


def test_project_firmware_rejects_bootloader_cache_from_another_idf(tmp_path: Path) -> None:
    build_dir = tmp_path / "build-s3"
    bootloader_cache = build_dir / "bootloader" / "CMakeCache.txt"
    bootloader_cache.parent.mkdir(parents=True)
    bootloader_cache.write_text(
        "IDF_PATH:UNINITIALIZED=/opt/old-idf\n"
        "CMAKE_HOME_DIRECTORY:INTERNAL=/opt/old-idf/components/bootloader/subproject\n",
        encoding="utf-8",
    )

    assert not micro_firmware._local_build_cache_matches_toolchain(
        build_dir,
        tmp_path / "micropython",
        tmp_path / "current-idf",
    )


def test_project_firmware_accepts_matching_local_cmake_cache(tmp_path: Path) -> None:
    build_dir = tmp_path / "build-s3"
    micropython_dir = tmp_path / "micropython"
    idf_path = tmp_path / "idf"
    build_dir.mkdir()
    (build_dir / "CMakeCache.txt").write_text(
        f"CMAKE_HOME_DIRECTORY:INTERNAL={micropython_dir / 'ports' / 'esp32'}\n",
        encoding="utf-8",
    )
    bootloader_cache = build_dir / "bootloader" / "CMakeCache.txt"
    bootloader_cache.parent.mkdir()
    bootloader_cache.write_text(
        f"IDF_PATH:UNINITIALIZED={idf_path}\n"
        f"CMAKE_HOME_DIRECTORY:INTERNAL={idf_path / 'components' / 'bootloader' / 'subproject'}\n",
        encoding="utf-8",
    )

    assert micro_firmware._local_build_cache_matches_toolchain(
        build_dir,
        micropython_dir,
        idf_path,
    )


def test_generated_sdkconfig_is_reused_when_kconfig_profile_matches(tmp_path: Path) -> None:
    source_dir = tmp_path / "micro"
    boards = source_dir / "firmware" / "boards"
    boards.mkdir(parents=True)
    (boards / "sdkconfig.micro_espectre").write_text("CONFIG_A=y\n", encoding="utf-8")
    component = source_dir / "firmware" / "components" / "espectre_core"
    component.mkdir(parents=True)
    scheduling_kconfig = component / "Kconfig.projbuild"
    scheduling_kconfig.write_text("config ESPECTRE_PRIORITY\n", encoding="utf-8")
    build_dir = tmp_path / "build-esp32c3"
    build_dir.mkdir()
    (build_dir / "sdkconfig").write_text('CONFIG_IDF_TARGET="esp32c3"\n', encoding="utf-8")
    profile = micro_firmware._firmware_kconfig_profile(source_dir, "c3")
    micro_firmware._write_kconfig_profile(build_dir, profile)

    assert micro_firmware._generated_sdkconfig_is_current(build_dir, profile)

    (boards / "sdkconfig.micro_espectre").write_text("CONFIG_A=n\n", encoding="utf-8")
    changed = micro_firmware._firmware_kconfig_profile(source_dir, "c3")
    assert not micro_firmware._generated_sdkconfig_is_current(build_dir, changed)

    (boards / "sdkconfig.micro_espectre").write_text("CONFIG_A=y\n", encoding="utf-8")
    restored = micro_firmware._firmware_kconfig_profile(source_dir, "c3")
    micro_firmware._write_kconfig_profile(build_dir, restored)
    scheduling_kconfig.write_text("config ESPECTRE_OTHER_PRIORITY\n", encoding="utf-8")
    changed_kconfig = micro_firmware._firmware_kconfig_profile(source_dir, "c3")
    assert not micro_firmware._generated_sdkconfig_is_current(build_dir, changed_kconfig)


@pytest.mark.parametrize(
    ("chip", "expected_lltf", "expected_htltf"),
    tuple(
        (chip, 1 if chip == "esp32" else 0, 0 if chip == "esp32" else 1)
        for chip in MICRO_CHIP_CHOICES
    ),
)
def test_project_firmware_configures_csi_phy_without_rebuilding_payload_bound(
    tmp_path: Path,
    chip: str,
    expected_lltf: int,
    expected_htltf: int,
) -> None:
    source_path = tmp_path / "ports" / "esp32" / "network_wlan_csi.c"
    source_path.parent.mkdir(parents=True)
    source_path.write_text(
        "#define CSI_MAX_DATA_LEN (512)\n"
        "wifi_csi_config_t wifi_6 = {\n"
        "    .acquire_csi_legacy = 1,\n"
        "    .acquire_csi_ht20 = 1,\n"
        "};\n"
        "    #if CONFIG_IDF_TARGET_ESP32C6\n"
        "    config->acquire_csi_he_stbc = 0;\n"
        "    #endif\n"
        "wifi_csi_config_t legacy = {\n"
        "    .lltf_en = 1,\n"
        "    .htltf_en = 1,\n"
        "};\n",
        encoding="utf-8",
    )

    micro_firmware._configure_project_csi_capture(tmp_path, chip)
    micro_firmware._configure_project_csi_capture(tmp_path, chip)

    source = source_path.read_text(encoding="utf-8")
    assert ".acquire_csi_legacy = 0," in source
    assert ".acquire_csi_legacy = 1," not in source
    if chip == "c5":
        assert "config->acquire_csi_ht20 = !use_vht20;" in source
        assert "config->acquire_csi_vht = use_vht20;" in source
        assert "esp_wifi_sta_get_ap_info(&ap_info)" in source
    else:
        assert "config->acquire_csi_vht = use_vht20;" not in source
    assert f".lltf_en = {expected_lltf}," in source
    assert f".htltf_en = {expected_htltf}," in source
    assert "#define CSI_MAX_DATA_LEN (512)" in source
    assert "#define CSI_MAX_DATA_LEN (256)" not in source


def test_project_firmware_uses_configurable_fixed_csi_records(tmp_path: Path) -> None:
    source_path = tmp_path / "ports" / "esp32" / "network_wlan_csi.c"
    source_path.parent.mkdir(parents=True)
    source_path.write_text(
        """#include "modnetwork.h"
#include <stdint.h>

#define CSI_MAX_DATA_LEN (512)

typedef struct {
    uint16_t len;
    int8_t data[CSI_MAX_DATA_LEN];
} csi_frame_t;

// ringbuf_t uses uint16_t for the byte size, so keep the Python-visible limit
// within the maximum addressable ringbuffer capacity.
#define CSI_MAX_BUFFER_SIZE ((UINT16_MAX - 1) / sizeof(csi_frame_t))

typedef struct {
    ringbuf_t ringbuffer;
    uint16_t buffer_size;
    volatile uint32_t dropped;
} csi_state_t;

static csi_state_t *wifi_csi_get_state(void) {
    csi_state_t *state;
    if (state == NULL) {
        state->buffer_size = MICROPY_PY_NETWORK_WLAN_CSI_DEFAULT_BUFFER_SIZE;
    }
    return state;
}

static void wifi_csi_rx_cb(wifi_csi_info_t *info, csi_state_t *state) {
    static csi_frame_t frame;
    frame.len = info->len > CSI_MAX_DATA_LEN ? CSI_MAX_DATA_LEN : info->len;
    ringbuf_put_bytes(&state->ringbuffer, (uint8_t *)&frame, sizeof(frame));
}

static void wifi_csi_enable(csi_state_t *state) {
    ringbuf_alloc(&state->ringbuffer, sizeof(csi_frame_t) * state->buffer_size);
}

static bool wifi_csi_read_frame(csi_frame_t *frame, csi_state_t *state) {
    return ringbuf_get_bytes(&state->ringbuffer, (uint8_t *)frame, sizeof(*frame));
}

static mp_obj_t network_wlan_csi_enable(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_buffer_size, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = MICROPY_PY_NETWORK_WLAN_CSI_DEFAULT_BUFFER_SIZE} },
    };
    mp_arg_val_t parsed_args[MP_ARRAY_SIZE(allowed_args)];
    mp_int_t buffer_size = parsed_args[0].u_int;
    if (buffer_size < 1 || buffer_size > CSI_MAX_BUFFER_SIZE) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer_size out of range"));
    }
    csi_state_t *state;
    state->buffer_size = buffer_size;
    esp_exceptions(wifi_csi_enable(state));
}

static mp_obj_t network_wlan_csi_available(csi_state_t *state) {
    size_t available = ringbuf_avail(&state->ringbuffer);
    return MP_OBJ_NEW_SMALL_INT(available / sizeof(csi_frame_t));
}
""",
        encoding="utf-8",
    )

    micro_firmware._configure_project_csi_fixed_records(tmp_path)
    micro_firmware._configure_project_csi_fixed_records(tmp_path)

    source = source_path.read_text(encoding="utf-8")
    assert "offsetof(csi_frame_t, data) + state->max_data_len" in source
    assert "frame.len = info->len > state->max_data_len" in source
    assert "MP_QSTR_max_data_len" in source
    assert "max_data_len < 1 || max_data_len > CSI_MAX_DATA_LEN" in source
    assert "wifi_csi_record_size(state) * state->buffer_size + 1" in source
    assert "available / wifi_csi_record_size(state)" in source
    assert source.count("static size_t wifi_csi_record_size") == 1


def test_project_firmware_hardens_and_observes_csi_ring(tmp_path: Path) -> None:
    esp32_dir = tmp_path / "ports" / "esp32"
    esp32_dir.mkdir(parents=True)
    implementation_path = esp32_dir / "network_wlan_csi.c"
    implementation_path.write_text(
        """#include "esp_timer.h"

typedef struct {
    ringbuf_t ringbuffer;
    uint16_t buffer_size;
    volatile uint32_t dropped;
} csi_state_t;

static csi_state_t *wifi_csi_get_state(void) {
    csi_state_t *state = MP_STATE_PORT(csi_state);
    if (state == NULL) {
        state = m_new_obj(csi_state_t);
        memset(state, 0, sizeof(*state));
        state->buffer_size = MICROPY_PY_NETWORK_WLAN_CSI_DEFAULT_BUFFER_SIZE;
    }
    return state;
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    csi_state_t *state = MP_STATE_PORT(csi_state);
    if (state == NULL || state->ringbuffer.buf == NULL) {
        return;
    }
    csi_frame_t frame;
    if (ringbuf_put_bytes(&state->ringbuffer, (uint8_t *)&frame, sizeof(frame)) != 0) {
        state->dropped++;
    }
}

static esp_err_t wifi_csi_enable(csi_state_t *state) {
    ringbuf_alloc(&state->ringbuffer, sizeof(csi_frame_t) * state->buffer_size);
    state->dropped = 0;
}

static esp_err_t wifi_csi_disable(csi_state_t *state) {
    esp_err_t err = esp_wifi_set_csi(false);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_csi_rx_cb(NULL, NULL);
    m_del(uint8_t, state->ringbuffer.buf, state->ringbuffer.size);
    state->dropped = 0;
    return ESP_OK;
}

void wifi_csi_deinit(void) {
    csi_state_t *state = (csi_state_t *)MP_STATE_PORT(csi_state);
    if (state == NULL) {
        return;
    }

    if (state->ringbuffer.buf != NULL) {
        esp_wifi_set_csi(false);
        esp_wifi_set_csi_rx_cb(NULL, NULL);
        m_del(uint8_t, state->ringbuffer.buf, state->ringbuffer.size);
    }

    m_del_obj(csi_state_t, state);
    MP_STATE_PORT(csi_state) = NULL;
}

static bool wifi_csi_read_frame(csi_frame_t *frame) {
    csi_state_t *state = (csi_state_t *)MP_STATE_PORT(csi_state);
    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    int result = ringbuf_get_bytes(&state->ringbuffer, (uint8_t *)frame, sizeof(*frame));
    MICROPY_END_ATOMIC_SECTION(atomic_state);
    return result == 0;
}

static mp_obj_t network_wlan_csi_read(size_t n_args, const mp_obj_t *args) {
    mp_obj_list_t *result;
    csi_frame_t frame;
    result->items[2] = mp_obj_new_bytes(frame.mac, sizeof(frame.mac));
}

static mp_obj_t network_wlan_csi_disable(mp_obj_t self_in) {
}

static mp_obj_t network_wlan_csi_dropped(mp_obj_t self_in) {
    (void)self_in;
    csi_state_t *state = (csi_state_t *)MP_STATE_PORT(csi_state);
    return mp_obj_new_int(state == NULL ? 0 : state->dropped);
}
MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_csi_dropped_obj, network_wlan_csi_dropped);

static mp_obj_t network_wlan_csi_available(mp_obj_t self_in) {
    csi_state_t *state = (csi_state_t *)MP_STATE_PORT(csi_state);
    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    size_t available = ringbuf_avail(&state->ringbuffer);
    MICROPY_END_ATOMIC_SECTION(atomic_state);
    return MP_OBJ_NEW_SMALL_INT(available / sizeof(csi_frame_t));
}
""",
        encoding="utf-8",
    )
    header_path = esp32_dir / "network_wlan_csi.h"
    header_path.write_text(
        "MP_DECLARE_CONST_FUN_OBJ_1(network_wlan_csi_disable_obj);\n"
        "MP_DECLARE_CONST_FUN_OBJ_1(network_wlan_csi_dropped_obj);\n",
        encoding="utf-8",
    )
    wlan_path = esp32_dir / "network_wlan.c"
    wlan_path.write_text(
        "    { MP_ROM_QSTR(MP_QSTR_csi_disable), MP_ROM_PTR(&network_wlan_csi_disable_obj) },\n"
        "    { MP_ROM_QSTR(MP_QSTR_csi_dropped), MP_ROM_PTR(&network_wlan_csi_dropped_obj) },\n",
        encoding="utf-8",
    )

    micro_firmware._configure_project_csi_rearm(tmp_path)
    micro_firmware._configure_project_csi_rearm(tmp_path)

    implementation = implementation_path.read_text(encoding="utf-8")
    assert implementation.count("network_wlan_csi_rearm_obj") == 1
    assert "volatile uint32_t callbacks;" in implementation
    assert "portMUX_TYPE lock;" in implementation
    assert "portMUX_INITIALIZE(&state->lock);" in implementation
    assert "__atomic_fetch_add(&state->callbacks, 1" in implementation
    assert "portENTER_CRITICAL(&state->lock);" in implementation
    assert "MICROPY_BEGIN_ATOMIC_SECTION" not in implementation
    assert '#include "esp_heap_caps.h"' in implementation
    assert "MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT" in implementation
    assert "IRAM_ATTR wifi_csi_rx_cb" not in implementation
    assert "serialized Wi-Fi task, not an ISR" in implementation
    assert "network_wlan_csi_update_mac(&result->items[2], frame.mac);" in implementation
    assert implementation.count("mp_obj_new_bytes(mac, 6)") == 1
    assert "mp_obj_new_bytes(frame.mac, sizeof(frame.mac))" not in implementation
    assert "heap_caps_malloc" in implementation
    assert "heap_caps_free" in implementation
    assert "m_del(uint8_t, state->ringbuffer.buf, state->ringbuffer.size);" not in implementation
    assert "wifi_csi_disable(state) != ESP_OK" in implementation
    assert "intentionally retain the" in implementation
    assert "sizeof(csi_frame_t) * state->buffer_size + 1" in implementation
    assert implementation.index(
        "esp_wifi_set_csi_rx_cb(NULL, NULL)"
    ) < implementation.index("esp_wifi_set_csi(false)")
    assert "state->ringbuffer.iget = 0;" in implementation
    assert "state->callbacks = 0;" in implementation
    assert implementation.count("network_wlan_csi_callbacks_obj") == 1
    assert header_path.read_text(encoding="utf-8").count("network_wlan_csi_rearm_obj") == 1
    assert header_path.read_text(encoding="utf-8").count("network_wlan_csi_callbacks_obj") == 1
    assert wlan_path.read_text(encoding="utf-8").count("MP_QSTR_csi_rearm") == 1
    assert wlan_path.read_text(encoding="utf-8").count("MP_QSTR_csi_callbacks") == 1


def test_project_firmware_rejects_stale_variable_csi_records(tmp_path: Path) -> None:
    esp32_dir = tmp_path / "ports" / "esp32"
    esp32_dir.mkdir(parents=True)
    implementation_path = esp32_dir / "network_wlan_csi.c"
    implementation = """typedef struct {} csi_frame_header_t;
typedef struct {
    ringbuf_t ringbuffer;
} csi_state_t;
ringbuf_alloc(&state->ringbuffer, ring_size);
MICROPY_BEGIN_ATOMIC_SECTION();
wifi_csi_release_ring(state);
network_wlan_csi_callbacks_obj;
MP_REGISTER_ROOT_POINTER(void *csi_state);
    if (disable_err == ESP_ERR_WIFI_NOT_STARTED || disable_err == ESP_ERR_WIFI_NOT_INIT) {
}
"""
    implementation_path.write_text(implementation, encoding="utf-8")
    (esp32_dir / "network_wlan_csi.h").write_text(
        "network_wlan_csi_callbacks_obj;\n",
        encoding="utf-8",
    )
    (esp32_dir / "network_wlan.c").write_text(
        "MP_QSTR_csi_callbacks;\n",
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="stale variable-record CSI source"):
        micro_firmware._configure_project_csi_rearm(tmp_path)


def test_project_firmware_reserves_native_heap_from_gc_growth(tmp_path: Path) -> None:
    source_path = tmp_path / "ports" / "esp32" / "gccollect.c"
    source_path.parent.mkdir(parents=True)
    source_path.write_text(
        """// The largest new region that is available to become Python heap is the largest
// free block in the ESP-IDF system heap.
size_t gc_get_max_new_split(void) {
    return heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
}
""",
        encoding="utf-8",
    )

    micro_firmware._configure_project_gc_heap_reserve(tmp_path)
    micro_firmware._configure_project_gc_heap_reserve(tmp_path)

    source = source_path.read_text(encoding="utf-8")
    assert source.count("ESPECTRE_GC_MAX_NEW_SPLIT_SIZE") == 4
    assert source.count("ESPECTRE_GC_NATIVE_HEAP_RESERVE") == 4
    assert "#define ESPECTRE_GC_MAX_NEW_SPLIT_SIZE (56 * 1024)" in source
    assert "#define ESPECTRE_GC_NATIVE_HEAP_RESERVE (32 * 1024)" in source
    assert "internal_largest - ESPECTRE_GC_NATIVE_HEAP_RESERVE" in source
    assert "MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT" in source
    assert "#if CONFIG_SPIRAM" in source
    assert "MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT" in source
    assert "external_available > available" in source
    assert "heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)" not in source


def test_project_firmware_exposes_dual_band_mode_configuration(tmp_path: Path) -> None:
    source_path = tmp_path / "ports" / "esp32" / "network_wlan.c"
    source_path.parent.mkdir(parents=True)
    source_path.write_text(
        """                    case MP_QSTR_bandwidth: {
                        esp_exceptions(esp_wifi_set_bandwidth(self->if_id, mp_obj_get_int(kwargs->table[i].value)));
                        break;
                    }
    { MP_ROM_QSTR(MP_QSTR_BANDWIDTH_20), MP_ROM_INT(WIFI_BW20) },
""",
        encoding="utf-8",
    )

    micro_firmware._configure_project_wifi_band_mode(tmp_path)
    micro_firmware._configure_project_wifi_band_mode(tmp_path)

    source = source_path.read_text(encoding="utf-8")
    assert source.count("case MP_QSTR_band_mode:") == 1
    assert source.count("esp_wifi_set_band_mode") == 1
    assert source.count("esp_wifi_set_protocols") == 1
    assert source.count("esp_wifi_set_bandwidths") == 1
    assert "WIFI_PROTOCOL_11AC" in source
    assert source.count("MP_QSTR_BAND_MODE_2G_ONLY") == 1
    assert source.count("MP_QSTR_BAND_MODE_AUTO") == 1


def test_project_firmware_matches_s2_rom_usb_serial(tmp_path: Path) -> None:
    source_path = tmp_path / "ports" / "esp32" / "usb.c"
    source_path.parent.mkdir(parents=True)
    source_path.write_text(
        """void mp_usbd_port_get_serial_number(char *serial_buf) {
    // use factory default MAC as serial ID
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    MP_STATIC_ASSERT(sizeof(mac) * 2 <= MICROPY_HW_USB_DESC_STR_MAX);
    mp_usbd_hex_str(serial_buf, mac, sizeof(mac));
}
""",
        encoding="utf-8",
    )

    micro_firmware._configure_project_s2_usb_serial(tmp_path)
    micro_firmware._configure_project_s2_usb_serial(tmp_path)

    source = source_path.read_text(encoding="utf-8")
    assert source.count("#if CONFIG_IDF_TARGET_ESP32S2") == 1
    assert source.count("serial_buf[0] = '0';") == 1
    assert source.count("serial_buf[1] = '\\0';") == 1
    assert source.count("esp_efuse_mac_get_default(mac);") == 1


def test_project_firmware_exposes_bssid_channel_pin(tmp_path: Path) -> None:
    source_path = tmp_path / "ports" / "esp32" / "network_wlan.c"
    source_path.parent.mkdir(parents=True)
    source_path.write_text(
        """    enum { ARG_ssid, ARG_key, ARG_bssid };
        { MP_QSTR_bssid, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
            memcpy(wifi_sta_config.sta.bssid, p, sizeof(wifi_sta_config.sta.bssid));
        }
""",
        encoding="utf-8",
    )

    micro_firmware._configure_project_wifi_channel_pin(tmp_path)
    micro_firmware._configure_project_wifi_channel_pin(tmp_path)

    source = source_path.read_text(encoding="utf-8")
    assert source.count("ARG_channel") == 4
    assert source.count("MP_QSTR_channel") == 1
    assert source.count("wifi_sta_config.sta.channel") == 1
    assert "args[ARG_channel].u_int > 255" in source
    assert "args[ARG_channel].u_int > 14" not in source


def test_project_boards_cover_micro_chip_registry_and_only_esp32_override() -> None:
    boards_dir = micro.PYTHON_SRC_DIR / "firmware" / "boards"

    assert set(micro_firmware.PROJECT_FIRMWARE_BOARDS) == set(MICRO_CHIP_CHOICES)
    assert set(micro_firmware.PROJECT_FIRMWARE_NAMES) == set(MICRO_CHIP_CHOICES)

    for board in micro_firmware.PROJECT_FIRMWARE_BOARDS.values():
        board_cmake = (boards_dir / board / "mpconfigboard.cmake").read_text(
            encoding="utf-8"
        )
        assert "../micro_espectre.cmake" in board_cmake

    overrides = sorted(
        path.relative_to(boards_dir).as_posix()
        for path in boards_dir.rglob("sdkconfig.override")
    )
    assert overrides == ["ESP32_MICRO_ESPECTRE/sdkconfig.override"]

    common_header = (boards_dir / "mpconfigboard_common.h").read_text(encoding="utf-8")
    assert "MICROPY_HW_ENABLE_MDNS_RESPONDER (1)" in common_header
    assert "MICROPY_PY_ARRAY (1)" in common_header

    native_cmake = (
        micro.PYTHON_SRC_DIR / "firmware" / "native_components" / "micropython.cmake"
    ).read_text(encoding="utf-8")
    assert "native_direct.c" in native_cmake
    assert "native_features.cpp" in native_cmake
    assert "native_features_module.c" in native_cmake
    assert "native_log_sink.cpp" in native_cmake
    assert "native_traffic.cpp" in native_cmake
    assert "shared_core" not in native_cmake
    assert "idf::espectre_core" in native_cmake
    assert "idf::espectre_runtime_traffic" in native_cmake
    assert "idf::log" in native_cmake
    assert "native_traffic.c" in native_cmake
    assert "native_mqtt.c" not in native_cmake
    assert "idf::mqtt" not in native_cmake

    core_component = (
        micro.PYTHON_SRC_DIR / "firmware" / "components" / "espectre_core"
    )
    component_cmake = (core_component / "CMakeLists.txt").read_text(encoding="utf-8")
    component_kconfig = (core_component / "Kconfig.projbuild").read_text(encoding="utf-8")
    assert "ESPECTRE_CORE_SOURCES" in component_cmake
    assert "idf_component_register" in component_cmake
    assert "$ENV{ESPECTRE_CORE_SDK_ROOT}" in component_cmake
    assert "        log\n" not in component_cmake
    assert "ESPECTRE_DIRECT_HTTPD_TASK_PRIORITY" in component_kconfig
    assert "ESPECTRE_TRAFFIC_TASK_PRIORITY" in component_kconfig

    traffic_component = (
        micro.PYTHON_SRC_DIR
        / "firmware"
        / "components"
        / "espectre_runtime_traffic"
        / "CMakeLists.txt"
    ).read_text(encoding="utf-8")
    assert "traffic_generator_manager.cpp" in traffic_component
    assert "sta_socket_helpers.cpp" in traffic_component
    assert "ESPECTRE_CORE_SOURCES" not in traffic_component
    assert "        espectre_core\n" in traffic_component
    assert "        log\n" not in traffic_component

    native_log_sink = (
        micro.PYTHON_SRC_DIR
        / "firmware"
        / "native_components"
        / "native_log_sink.cpp"
    ).read_text(encoding="utf-8")
    assert "espectre::set_log_sink" in native_log_sink
    assert "esp_log_va" in native_log_sink
    assert "ESP_LOG_CONFIGS_DEFAULT" in native_log_sink
    assert "std::vsnprintf" not in native_log_sink
    assert "ESP_LOG_LEVEL" not in native_log_sink
    assert "esp_log_writev" not in native_log_sink
    assert "#ifndef NO_QSTR" in native_log_sink

    shared_sdkconfig = (boards_dir / "sdkconfig.micro_espectre").read_text(
        encoding="utf-8"
    )
    assert "CONFIG_LOG_VERSION_2=y" in shared_sdkconfig

    native_cpp = (
        micro.PYTHON_SRC_DIR
        / "firmware"
        / "native_components"
        / "native_features.cpp"
    ).read_text(encoding="utf-8")
    assert '#include "espectre_core_sdk.h"' in native_cpp
    assert '#include "core/' not in native_cpp
    assert "espectre_native_ensure_log_sink();" in native_cpp

    native_traffic_cpp = (
        micro.PYTHON_SRC_DIR
        / "firmware"
        / "native_components"
        / "native_traffic.cpp"
    ).read_text(encoding="utf-8")
    assert "espectre_native_ensure_log_sink();" in native_traffic_cpp

    native_module = (
        micro.PYTHON_SRC_DIR
        / "firmware"
        / "native_components"
        / "native_features_module.c"
    ).read_text(encoding="utf-8")
    assert native_module.count("mp_obj_malloc_with_finaliser") == 2
    assert native_module.count("MP_QSTR___del__") == 2
    assert "MP_QSTR_Detector" in native_module
    assert "MP_QSTR_TemporalCsiSampler" in native_module
    assert "high_accuracy" not in native_module
    assert "HighAccuracyDetector" not in native_cpp


def test_device_manifest_is_lightweight_direct_only() -> None:
    deployed = set(micro.MICRO_DEVICE_RELATIVE_FILES)

    assert {"lightweight_detector.py", "direct_api.py", "protocol.py"} <= deployed
    assert "high_accuracy_detector.py" not in deployed
    assert "ml_feature_trackers.py" not in deployed
    assert "ml_weights.py" not in deployed
    assert "utils.py" not in deployed
    assert "filters.py" not in deployed
    assert "csi_features.py" not in deployed
    assert "segmentation.py" not in deployed
    assert not any(path.startswith("mqtt/") for path in deployed)


def test_deploy_code_uses_selected_config_as_device_override(monkeypatch, tmp_path: Path) -> None:
    src_dir = tmp_path / "src"
    _create_micro_src_tree(src_dir)
    benchmark_config = tmp_path / "benchmark_config.py"
    benchmark_config.write_text("CSI_TARGET_PPS = 80\n", encoding="utf-8")
    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(cmd)
        if cmd[:4] == ["mpremote", "connect", "/dev/cu.usbmodem1", "exec"]:
            return SimpleNamespace(returncode=0, stdout="MP_OK", stderr="")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(micro, "PYTHON_SRC_DIR", src_dir)
    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    micro.deploy_code(_make_args(config=benchmark_config))

    config_compile = next(
        cmd
        for cmd in calls
        if cmd and cmd[0] == micro.MPY_CROSS_COMMAND and cmd[-1] == str(benchmark_config)
    )
    assert config_compile[3] == "src/config_local.py"
    assert any(cmd[-1] == ":src.stage/config_local.mpy" for cmd in calls if "cp" in cmd)


def test_deploy_code_retries_healthcheck_while_micropython_starts(monkeypatch, tmp_path: Path) -> None:
    src_dir = tmp_path / "src"
    _create_micro_src_tree(src_dir)
    health_attempts = 0

    def fake_run(cmd, **kwargs):
        nonlocal health_attempts
        if cmd[:4] == ["mpremote", "connect", "/dev/cu.usbmodem1", "exec"]:
            if "MP_OK" in cmd[-1]:
                health_attempts += 1
                if health_attempts == 1:
                    return SimpleNamespace(returncode=1, stdout="", stderr="port is not ready")
                return SimpleNamespace(returncode=0, stdout="MP_OK", stderr="")
            return SimpleNamespace(returncode=0, stdout="NONE", stderr="")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(micro, "PYTHON_SRC_DIR", src_dir)
    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "run", fake_run)
    monkeypatch.setattr(micro.time, "sleep", lambda _seconds: None)

    micro.deploy_code(_make_args())

    assert health_attempts == 2


def test_deploy_code_rejects_invalid_healthcheck(monkeypatch, tmp_path: Path) -> None:
    src_dir = tmp_path / "src"
    _create_micro_src_tree(src_dir)

    def fake_run(cmd, **kwargs):
        if cmd[:4] == ["mpremote", "connect", "/dev/cu.usbmodem1", "exec"]:
            return SimpleNamespace(returncode=1, stdout="", stderr="bad boot")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(micro, "PYTHON_SRC_DIR", src_dir)
    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "run", fake_run)
    monkeypatch.setattr(micro, "MICROPYTHON_READY_TIMEOUT_SECONDS", 0.0)

    with pytest.raises(SystemExit):
        micro.deploy_code(_make_args())


def test_deploy_code_rejects_incomplete_source_tree(monkeypatch, tmp_path: Path) -> None:
    src_dir = tmp_path / "src"
    _create_micro_src_tree(src_dir)
    (src_dir / "device_utils.py").unlink()
    calls = []

    monkeypatch.setattr(micro, "PYTHON_SRC_DIR", src_dir)
    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "run", lambda *args, **kwargs: calls.append(args))

    with pytest.raises(SystemExit):
        micro.deploy_code(_make_args())

    assert calls == []


def test_deploy_code_exits_on_copy_failure(monkeypatch, tmp_path: Path) -> None:
    src_dir = tmp_path / "src"
    _create_micro_src_tree(src_dir)

    calls: list[list[str]] = []

    def fake_run(cmd, **kwargs):
        calls.append(cmd)
        if cmd[:4] == ["mpremote", "connect", "/dev/cu.usbmodem1", "exec"]:
            return SimpleNamespace(returncode=0, stdout="MP_OK", stderr="")
        if "cp" in cmd:
            raise subprocess.CalledProcessError(2, cmd)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(micro, "PYTHON_SRC_DIR", src_dir)
    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    with pytest.raises(SystemExit):
        micro.deploy_code(_make_args())

    exec_scripts = [cmd[-1] for cmd in calls if cmd[:4] == ["mpremote", "connect", "/dev/cu.usbmodem1", "exec"]]
    assert not any("os.rename('/src.stage', '/src')" in script for script in exec_scripts)


def test_run_application_starts_mpremote_process(monkeypatch) -> None:
    started: list[list[str]] = []

    class FakeProcess:
        def wait(self):
            return 0

    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "Popen", lambda cmd: started.append(cmd) or FakeProcess())

    micro.run_application(_make_args())

    assert started == [[
        "mpremote",
        "connect",
        "/dev/cu.usbmodem1",
        "exec",
        "from src.main import main; main()",
    ]]


def test_run_application_propagates_mpremote_failure(monkeypatch) -> None:
    class FakeProcess:
        def wait(self):
            return 2

    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "Popen", lambda _cmd: FakeProcess())

    with pytest.raises(SystemExit, match="2"):
        micro.run_application(_make_args())


def test_run_application_handles_keyboard_interrupt_and_resets_device(monkeypatch) -> None:
    events: list[str] = []

    class FakeProcess:
        def wait(self, timeout=None):
            if timeout is None:
                raise KeyboardInterrupt
            raise subprocess.TimeoutExpired(cmd="mpremote", timeout=timeout)

        def terminate(self):
            events.append("terminate")

        def kill(self):
            events.append("kill")

    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "Popen", lambda _cmd: FakeProcess())
    monkeypatch.setattr(micro, "_reset_device", lambda port: events.append(f"reset:{port}") or True)

    micro.run_application(_make_args())

    assert events == ["terminate", "kill", "reset:/dev/cu.usbmodem1"]


def test_run_application_exits_when_interrupt_reset_fails(monkeypatch) -> None:
    class FakeProcess:
        def wait(self, timeout=None):
            if timeout is None:
                raise KeyboardInterrupt
            return 0

        def terminate(self):
            return None

    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "Popen", lambda _cmd: FakeProcess())
    monkeypatch.setattr(micro, "_reset_device", lambda _port: False)

    with pytest.raises(SystemExit):
        micro.run_application(_make_args())


def test_run_application_exits_on_subprocess_error(monkeypatch) -> None:
    monkeypatch.setattr(micro, "_require_mpremote", lambda: None)
    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(
        micro.subprocess,
        "Popen",
        lambda _cmd: (_ for _ in ()).throw(subprocess.CalledProcessError(1, ["mpremote"])),
    )

    with pytest.raises(SystemExit):
        micro.run_application(_make_args())


def test_verify_installation_passes_when_all_checks_succeed(monkeypatch) -> None:
    src_listing = [
        Path(rel_path).with_suffix(".mpy").name
        for rel_path in micro.MICRO_DEVICE_RELATIVE_FILES
        if "/" not in rel_path
    ]
    results = [
        SimpleNamespace(stdout="csi_start,csi_stop\n", stderr=""),
        SimpleNamespace(stdout="espectre_core True True\n", stderr=""),
        SimpleNamespace(stdout="(1, 24, 0)\n", stderr=""),
        SimpleNamespace(stdout=f"{src_listing!r}\n", stderr=""),
        SimpleNamespace(stdout="True\n", stderr=""),
    ]

    def fake_run(cmd, capture_output, text, check):
        return results.pop(0)

    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    micro.verify_installation(_make_verify_args())


def test_verify_installation_raises_when_required_checks_fail(monkeypatch) -> None:
    calls = [
        SimpleNamespace(stdout="NONE\n", stderr=""),
        subprocess.CalledProcessError(1, ["mpremote"], stderr="missing core"),
        subprocess.CalledProcessError(1, ["mpremote"], stderr="version error"),
        subprocess.CalledProcessError(1, ["mpremote"], stderr="missing src"),
        subprocess.CalledProcessError(1, ["mpremote"], stderr="config missing"),
    ]

    def fake_run(cmd, capture_output, text, check):
        result = calls.pop(0)
        if isinstance(result, Exception):
            raise result
        return result

    monkeypatch.setattr(micro, "get_serial_port", lambda _port, **_kwargs: "/dev/cu.usbmodem1")
    monkeypatch.setattr(micro.subprocess, "run", fake_run)

    with pytest.raises(SystemExit):
        micro.verify_installation(_make_verify_args())


@pytest.mark.parametrize('he', [False, True])
def test_csi_quality_patch_filters_hardware_errors_and_cleans_only_guards(tmp_path, he):
    """Compile the patched callback to test admission, not generated source text."""
    path = tmp_path / 'ports' / 'esp32' / 'network_wlan_csi.c'
    path.parent.mkdir(parents=True)
    (path.parent / 'network_wlan_csi.h').write_text(
        'MP_DECLARE_CONST_FUN_OBJ_1(network_wlan_csi_callbacks_obj);\n')
    (path.parent / 'network_wlan.c').write_text(
        '    { MP_ROM_QSTR(MP_QSTR_csi_callbacks), MP_ROM_PTR(&network_wlan_csi_callbacks_obj) },\n')
    path.write_text(r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
typedef struct {
    uint8_t rx_state;
#if CONFIG_SOC_WIFI_HE_SUPPORT
    uint8_t rxend_state, rx_channel_estimate_info_vld;
#endif
} rx_ctrl_t;
typedef struct {
    rx_ctrl_t rx_ctrl;
    uint16_t len;
    int8_t *buf;
    bool first_word_invalid;
} wifi_csi_info_t;
typedef struct {
    volatile uint32_t callbacks;
} csi_state_t;
static csi_state_t native_state;
typedef uint32_t mp_obj_t;
#define MP_DECLARE_CONST_FUN_OBJ_1(name) extern mp_obj_t (*name)(mp_obj_t)
#define MP_DEFINE_CONST_FUN_OBJ_1(name, function) mp_obj_t (*name)(mp_obj_t) = function
#include "network_wlan_csi.h"
static mp_obj_t network_wlan_csi_callbacks(mp_obj_t self_in) {
    (void)self_in;
    return __atomic_load_n(&native_state.callbacks, __ATOMIC_RELAXED);
}
MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_csi_callbacks_obj, network_wlan_csi_callbacks);
#define MP_ROM_QSTR(value) #value
#define MP_ROM_PTR(value) value
static const struct { const char *name; mp_obj_t (**getter)(mp_obj_t); } methods[] = {
#include "network_wlan.c"
};
static void reset_counters(void) {
    csi_state_t *state = &native_state;
    state->callbacks = 0;
}
static struct { uint16_t len; int8_t data[256]; } frame;
static unsigned accepted;
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    (void)ctx;
    if (info == NULL) return;
    csi_state_t *state = &native_state;
    __atomic_fetch_add(&state->callbacks, 1, __ATOMIC_RELAXED);
    frame.len = info->len;
    memcpy(frame.data, info->buf, frame.len);
    ++accepted;
}
int main(void) {
    int8_t payload[256]; memset(payload, 7, sizeof(payload));
    wifi_csi_info_t info = {.buf=payload, .len=128};
#if CONFIG_SOC_WIFI_HE_SUPPORT
    info.rx_ctrl.rx_channel_estimate_info_vld = 1;
#endif
    wifi_csi_rx_cb(NULL, &info); assert(accepted == 1);
    info.rx_ctrl.rx_state=1;
    wifi_csi_rx_cb(NULL, &info); assert(accepted == 1);
    info.rx_ctrl.rx_state=0;
#if CONFIG_SOC_WIFI_HE_SUPPORT
    info.rx_ctrl.rxend_state=1;
    wifi_csi_rx_cb(NULL, &info); assert(accepted == 1);
    info.rx_ctrl.rxend_state=0; info.rx_ctrl.rx_channel_estimate_info_vld=0;
    wifi_csi_rx_cb(NULL, &info); assert(accepted == 1);
    info.rx_ctrl.rx_channel_estimate_info_vld=1;
#endif
    info.first_word_invalid=true;
    wifi_csi_rx_cb(NULL, &info); assert(accepted == 1);
    const uint8_t guards[] = {2,3,61,62,63};
    for (unsigned i=0;i<sizeof(guards);++i) payload[guards[i]*2]=payload[guards[i]*2+1]=0;
    wifi_csi_rx_cb(NULL, &info); assert(accepted == 2);
    for (unsigned i=0;i<4;++i) assert(frame.data[i] == 0 && payload[i] == 7);
    assert(memcmp(frame.data+4,payload+4,124) == 0);
    info.len=256;
    wifi_csi_rx_cb(NULL, &info); assert(accepted == 3);
    info.len=114;
    wifi_csi_rx_cb(NULL, &info); assert(accepted == 3);
    wifi_csi_rx_cb(NULL, NULL); assert(accepted == 3);
    assert(sizeof(methods) / sizeof(methods[0]) == 2);
    assert(strcmp(methods[1].name, "MP_QSTR_csi_filtered") == 0);
    assert((*methods[0].getter)(0) == (CONFIG_SOC_WIFI_HE_SUPPORT ? 8 : 6));
    assert((*methods[1].getter)(0) == (CONFIG_SOC_WIFI_HE_SUPPORT ? 5 : 3));
    reset_counters();
    assert((*methods[0].getter)(0) == 0);
    assert((*methods[1].getter)(0) == 0);
    return 0;
}
''')
    micro_firmware._configure_project_csi_quality(tmp_path)
    paths = [path, path.parent / 'network_wlan_csi.h', path.parent / 'network_wlan.c']
    patched = [source.read_bytes() for source in paths]
    micro_firmware._configure_project_csi_quality(tmp_path)
    assert [source.read_bytes() for source in paths] == patched
    binary = tmp_path / 'quality-test'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Werror',
                    f'-DCONFIG_SOC_WIFI_HE_SUPPORT={int(he)}', str(path), '-o', str(binary)],
                   check=True, capture_output=True, text=True)
    subprocess.run([str(binary)], check=True, capture_output=True, text=True)
