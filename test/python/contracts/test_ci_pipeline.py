# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Regression tests for release, rolling, and GitHub Pages automation."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
from urllib.parse import urlparse

import pytest

from espectre_cli.idf_container import IDF_DOCKER_IMAGE
from tools.lib.repo_paths import repo_root


REPO_ROOT = repo_root()
SCRIPTS_DIR = REPO_ROOT / ".github" / "scripts"
WORKFLOWS_DIR = REPO_ROOT / ".github" / "workflows"
PROTOCOL_HEADER = REPO_ROOT / "src" / "cpp" / "runtime" / "espectre_protocol.h"
PYTHON_COVERAGE_THRESHOLDS = REPO_ROOT / "test" / "python" / "coverage-thresholds.json"
BUILD_PAGES_ACTION = REPO_ROOT / ".github" / "actions" / "build-pages" / "action.yml"
WEB_COVERAGE_RUNNER = REPO_ROOT / "test" / "web" / "run_coverage.sh"
WEB_COVERAGE_THRESHOLDS = REPO_ROOT / "test" / "web" / "coverage-thresholds.json"


def _workflow_job(source: str, job_name: str) -> str:
    match = re.search(
        rf"(?ms)^  {re.escape(job_name)}:\n(?P<body>.*?)(?=^  [a-zA-Z0-9_-]+:\n|\Z)",
        source,
    )
    assert match is not None, f"missing workflow job {job_name}"
    return match.group("body")


def _workflow_chip_matrix(source: str, job_name: str) -> set[str]:
    return set(
        re.findall(
            r"(?m)^\s+- chip: (ESP32(?:-[A-Z0-9]+)?)$",
            _workflow_job(source, job_name),
        )
    )


def test_ci_chip_matrices_follow_production_registries() -> None:
    from support.chip_matrix import ESPHOME_CHIPS, MATTER_CHIPS, NATIVE_CHIPS

    def workflow_label(chip: str) -> str:
        return "ESP32" if chip == "ESP32" else f"ESP32-{chip}"

    source = (WORKFLOWS_DIR / "ci.yml").read_text(encoding="utf-8")
    assert _workflow_chip_matrix(source, "build-esphome") == {
        workflow_label(chip) for chip in ESPHOME_CHIPS
    }
    assert _workflow_chip_matrix(source, "build-native") == {
        workflow_label(chip) for chip in NATIVE_CHIPS
    }
    assert _workflow_chip_matrix(source, "build-matter") == {
        workflow_label(chip) for chip in MATTER_CHIPS
    }
    cpp_job = _workflow_job(source, "test-cpp")
    assert "./test/cpp/run_coverage.sh --ci" in cpp_job
    assert "--kind cpp-runtime" in cpp_job
    assert "--report .cache/reports/coverage/cpp/coverage-summary.json" in cpp_job
    assert "--output .cache/reports/coverage/coverage-cpp-runtime.json" in cpp_job
    assert "name: cpp-coverage-badge" in cpp_job
    assert (
        "if: always() && hashFiles('.cache/reports/coverage/cpp/coverage-summary.json') != ''"
        in cpp_job
    )
    assert "if: always() && hashFiles('.cache/reports/coverage/coverage-cpp-runtime.json') != ''" in cpp_job

    python_job = _workflow_job(source, "test-python")
    assert "--cov-branch" in python_job
    assert "--cov-report=json:.cache/reports/coverage/python-coverage.json" in python_job
    assert "--cov-report=xml" not in python_job
    assert "python test/python/check_coverage.py .cache/reports/coverage/python-coverage.json" in python_job
    assert "--kind python" in python_job
    assert "--report .cache/reports/coverage/python-coverage.json" in python_job
    assert "--output .cache/reports/coverage/coverage-python.json" in python_job
    assert "name: python-coverage-badge" in python_job
    assert "if: always() && hashFiles('.cache/reports/coverage/python-coverage.json') != ''" in python_job
    assert "if: always() && hashFiles('.cache/reports/coverage/coverage-python.json') != ''" in python_job


def test_python_coverage_gate_has_fixed_thresholds() -> None:
    thresholds = json.loads(PYTHON_COVERAGE_THRESHOLDS.read_text(encoding="utf-8"))

    assert thresholds == {
        "minimums": {"branches": 50.0, "lines": 60.0},
        "version": 1,
    }


@pytest.mark.parametrize(
    ("covered_lines", "covered_branches", "failure"),
    [
        (60, 50, None),
        (59, 50, "lines: 59.00% < 60.00%"),
        (60, 49, "branches: 49.00% < 50.00%"),
    ],
)
def test_python_coverage_gate_enforces_each_metric(
    tmp_path: Path,
    covered_lines: int,
    covered_branches: int,
    failure: str | None,
) -> None:
    report = tmp_path / "coverage.json"
    report.write_text(
        json.dumps(
            {
                "totals": {
                    "covered_branches": covered_branches,
                    "covered_lines": covered_lines,
                    "num_branches": 100,
                    "num_statements": 100,
                }
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [sys.executable, "test/python/check_coverage.py", str(report)],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == (1 if failure else 0)
    if failure:
        assert failure in result.stdout
    else:
        assert "Python coverage thresholds satisfied." in result.stdout


def test_web_coverage_gate_uses_canonical_thresholds() -> None:
    workflow = (WORKFLOWS_DIR / "ci.yml").read_text(encoding="utf-8")
    build_site = _workflow_job(workflow, "build-site")
    action = BUILD_PAGES_ACTION.read_text(encoding="utf-8")
    runner = WEB_COVERAGE_RUNNER.read_text(encoding="utf-8")
    thresholds = json.loads(WEB_COVERAGE_THRESHOLDS.read_text(encoding="utf-8"))

    assert "./test/web/run_coverage.sh" in action
    assert "test/web/coverage-thresholds.json" in runner
    variables = {
        "branches": "branch_threshold",
        "functions": "function_threshold",
        "lines": "line_threshold",
    }
    for metric, variable in variables.items():
        assert metric in thresholds["minimums"]
        assert f'--test-coverage-{metric}=${{{variable}}}' in runner
    assert "docs/web/assets/js/espectre-direct.js" in runner
    assert "--kind web" in build_site
    assert "--report .cache/reports/coverage/web-coverage.log" in build_site
    assert "--output .cache/reports/coverage/coverage-web.json" in build_site
    assert "name: web-coverage-badge" in build_site
    assert "if: always() && hashFiles('.cache/reports/coverage/web-coverage.log') != ''" in build_site
    assert "if: always() && hashFiles('.cache/reports/coverage/coverage-web.json') != ''" in build_site


def test_snapshot_publishes_stable_coverage_badge_endpoints() -> None:
    workflow = (WORKFLOWS_DIR / "snapshot.yml").read_text(encoding="utf-8")
    validator = _workflow_job(workflow, "validate-run")
    release_job = _workflow_job(workflow, "release")
    publisher = _workflow_job(workflow, "publish-coverage")

    assert "github.event.workflow_run.conclusion == 'success'" not in validator
    assert (
        "context.eventName === 'workflow_dispatch' && conclusion !== 'success'"
        in validator
    )
    assert "core.setOutput('conclusion', conclusion)" in validator
    assert "coverage-badges/*.json" not in release_job
    assert "needs.validate-run.outputs.conclusion == 'success'" in release_job
    assert "always()" in publisher
    assert "needs.validate-run.outputs.conclusion == 'failure'" in publisher
    assert "pattern: '*-coverage-badge'" in publisher
    assert "gh release upload" in publisher
    assert "--clobber" in publisher
    assert "Recheck source commit before publishing coverage" in publisher


def test_coverage_badge_builder_reads_each_report_format(tmp_path: Path) -> None:
    builder = load_script("build_coverage_badges")
    python_report = tmp_path / "python.json"
    cpp_report = tmp_path / "cpp.json"
    web_report = tmp_path / "web.log"
    python_report.write_text(
        json.dumps(
            {
                "totals": {
                    "covered_branches": 51,
                    "covered_lines": 61,
                    "num_branches": 100,
                    "num_statements": 100,
                }
            }
        ),
        encoding="utf-8",
    )
    cpp_report.write_text(
        json.dumps(
            {
                "segments": {
                    "runtime": {
                        "branches": 51.0,
                        "functions": 86.0,
                        "lines": 81.0,
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    web_report.write_text(
        "# all files | 88.98 | 76.25 | 74.24 |\n",
        encoding="utf-8",
    )

    expected_python_badge = {
        "subject": "python coverage",
        "status": "61.00%",
        "color": "4c1",
    }
    assert builder.build_badge(
        "python", python_report, PYTHON_COVERAGE_THRESHOLDS
    ) == expected_python_badge
    expected_cpp_badge = {
        "subject": "c++ coverage",
        "status": "81.00%",
        "color": "4c1",
    }
    assert builder.build_badge(
        "cpp-runtime",
        cpp_report,
        REPO_ROOT / "test" / "cpp" / "coverage-thresholds.json",
    ) == expected_cpp_badge
    expected_web_badge = {
        "subject": "web coverage",
        "status": "88.98%",
        "color": "4c1",
    }
    assert builder.build_badge(
        "web", web_report, WEB_COVERAGE_THRESHOLDS
    ) == expected_web_badge

    output = tmp_path / "coverage-python.json"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPTS_DIR / "build_coverage_badges.py"),
            "--kind",
            "python",
            "--report",
            str(python_report),
            "--thresholds",
            str(PYTHON_COVERAGE_THRESHOLDS),
            "--output",
            str(output),
        ],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert json.loads(output.read_text(encoding="utf-8")) == expected_python_badge

    python_report.write_text(
        python_report.read_text(encoding="utf-8").replace(
            '"covered_lines": 61', '"covered_lines": 59'
        ),
        encoding="utf-8",
    )
    assert builder.build_badge(
        "python", python_report, PYTHON_COVERAGE_THRESHOLDS
    )["color"] == "e05d44"


def _ota_release_tags() -> tuple[str, str]:
    header = PROTOCOL_HEADER.read_text(encoding="utf-8")
    preview = re.search(r'ESPECTRE_OTA_RELEASE_TAG_PREVIEW\s*=\s*"([^"]+)"', header)
    develop = re.search(r'ESPECTRE_OTA_RELEASE_TAG_DEVELOP\s*=\s*"([^"]+)"', header)
    assert preview is not None and develop is not None
    return preview.group(1), develop.group(1)


def load_script(name: str):
    path = SCRIPTS_DIR / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"test_{name}", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_sdk_archives_and_manifest_are_reproducible(tmp_path: Path) -> None:
    builder = load_script("build_sdk_package")
    component_cmake = (REPO_ROOT / "src" / "cpp" / "CMakeLists.txt").read_text(encoding="utf-8")
    for dependency in (
        "mqtt",
        "app_update",
        "esp_http_client",
        "esp_http_server",
        "esp_https_ota",
        "esp-tls",
        "improv",
        "mdns",
    ):
        assert re.search(rf"(?m)^    {re.escape(dependency)}$", component_cmake)
    outputs = [tmp_path / "first", tmp_path / "second"]
    for output in outputs:
        args = argparse.Namespace(
            channel="release",
            version="3.0.0",
            release_tag="3.0.0",
            output_dir=str(output),
            commit="0123456789abcdef",
            source_date_epoch=1_800_000_000,
            url_prefix=None,
        )
        builder.build_sdk_package(args)

    first_files = sorted(path.name for path in outputs[0].iterdir())
    second_files = sorted(path.name for path in outputs[1].iterdir())
    assert first_files == second_files
    for filename in first_files:
        assert (outputs[0] / filename).read_bytes() == (outputs[1] / filename).read_bytes()

    manifest_path = next(outputs[0].glob("sdk-manifest-*.json"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["generated_at"] == "2027-01-15T08:00:00+00:00"
    assert manifest["install_surfaces"]["cmake"]["optional_source_groups"] == [
        "ESPECTRE_RUNTIME_FRONTEND_SUPPORT_SOURCES",
        "ESPECTRE_RUNTIME_ESP_IDF_MQTT_SOURCES",
        "ESPECTRE_RUNTIME_ESP_IDF_PROVISIONING_SOURCES",
        "ESPECTRE_RUNTIME_ESP_IDF_OTA_SOURCES",
        "ESPECTRE_RUNTIME_ESP_IDF_DIRECT_SOURCES",
    ]
    zip_path = next(outputs[0].glob("*.zip"))
    with zipfile.ZipFile(zip_path) as archive:
        archived = set(archive.namelist())
        component_cmake_name = next(
            name for name in archived if name.endswith("/src/cpp/CMakeLists.txt")
        )
        doxy_name = next(name for name in archived if name.endswith("/src/cpp/Doxyfile"))
        bundled_doxyfile = archive.read(doxy_name).decode("utf-8")
        guide_name = next(name for name in archived if name.endswith("/docs/SDK.md"))
        bundled_guide = archive.read(guide_name).decode("utf-8")
        facade_name = next(name for name in archived if name.endswith("/src/cpp/espectre_sdk.h"))
        bundled_facade = archive.read(facade_name).decode("utf-8")
    bundle_root = component_cmake_name.removesuffix("/src/cpp/CMakeLists.txt")
    assert f"{bundle_root}/CMakeLists.txt" not in archived
    assert manifest["install_surfaces"]["esp_idf_component"]["component_root"] == "src/cpp"
    assert re.search(r"(?m)^OUTPUT_DIRECTORY\s*=\s*output\s*$", bundled_doxyfile)
    assert re.search(r"(?m)^PROJECT_NUMBER\s*=\s*3\.0\.0\s*$", bundled_doxyfile)
    assert re.search(r"(?m)^GENERATE_HTML\s*=\s*NO\s*$", bundled_doxyfile)
    assert re.search(r"(?m)^GENERATE_XML\s*=\s*YES\s*$", bundled_doxyfile)
    assert not any("/src/cpp/doxygen/" in path for path in archived)
    assert "docs/web/artifacts/sdk" not in bundled_doxyfile
    assert "https://github.com/francescopace/espectre/blob/0123456789abcdef/docs/ARCHITECTURE.md" in bundled_guide
    assert "https://github.com/francescopace/espectre/blob/0123456789abcdef/LICENSING.md" in bundled_guide
    assert "https://github.com/francescopace/espectre/blob/0123456789abcdef/docs/SDK.md" in bundled_facade
    assert "https://github.com/francescopace/espectre/blob/main/docs/SDK.md" not in bundled_facade
    assert re.search(r"\]\((?!https?://|mailto:|#)[^)]+\.md(?:#[^)]+)?\)", bundled_guide) is None
    repo_doxyfile = (REPO_ROOT / "src" / "cpp" / "Doxyfile").read_text(encoding="utf-8")
    assert re.search(r"(?m)^OUTPUT_DIRECTORY\s*=\s*docs/web/artifacts/sdk\s*$", repo_doxyfile)
    assert re.search(r"(?m)^PROJECT_NUMBER\s*=\s*UNSTAMPED\s*$", repo_doxyfile)
    assert any(path.endswith("/THIRD_PARTY_NOTICES.md") for path in archived)
    for artifact in manifest["artifacts"]:
        assert artifact["sha256"] == file_sha256(outputs[0] / artifact["filename"])


def test_web_sdk_rejects_a_channel_mismatch_before_cleaning(tmp_path: Path) -> None:
    stage = load_script("stage_web_sdk")
    sdk_dir = tmp_path / "sdk"
    output_dir = tmp_path / "output"
    sdk_dir.mkdir()
    output_dir.mkdir()
    (sdk_dir / "sdk-manifest-release.json").write_text(
        json.dumps({"schema_version": 2, "channel": "release"}), encoding="utf-8"
    )
    sentinel = output_dir / "index.html"
    sentinel.write_text("keep", encoding="utf-8")

    with pytest.raises(ValueError, match="channel mismatch"):
        stage.stage_web_sdk(
            argparse.Namespace(
                sdk_dir=str(sdk_dir),
                output_dir=str(output_dir),
                channel="preview",
            )
        )
    assert sentinel.read_text(encoding="utf-8") == "keep"


def test_web_sdk_normalizes_matching_legacy_version_aliases() -> None:
    stage = load_script("stage_web_sdk")
    manifest = {
        "schema_version": 1,
        "version": "3.0.0-rc1",
        "package_version": "3.0.0-rc1",
        "sdk_version": "3.0.0-rc1",
    }

    normalized = stage.normalize_sdk_manifest(manifest)

    assert normalized["schema_version"] == 2
    assert normalized["version"] == "3.0.0-rc1"
    assert "package_version" not in normalized
    assert "sdk_version" not in normalized
    assert manifest["schema_version"] == 1


def test_web_sdk_rejects_conflicting_legacy_version_aliases() -> None:
    stage = load_script("stage_web_sdk")
    manifest = {
        "schema_version": 1,
        "version": "3.0.0-rc1",
        "package_version": "2.8.0",
        "sdk_version": "3.0.0-rc1",
    }

    with pytest.raises(ValueError, match="Legacy SDK version aliases disagree"):
        stage.normalize_sdk_manifest(manifest)


@pytest.mark.parametrize(
    ("version", "expected_stability", "production_ready"),
    [
        ("3.0.0", "final", True),
        ("3.0.0-rc1", "prerelease", False),
    ],
)
def test_release_sdk_page_exposes_version_stability(
    version: str, expected_stability: str, production_ready: bool
) -> None:
    stage = load_script("stage_web_sdk")
    manifest = {
        "channel": "release",
        "version": version,
        "release_tag": version,
        "protocol_version": 1,
        "supported_esp_idf": ">=5.5.0",
        "commit": "0123456789abcdef",
        "artifacts": [
            {
                "url": f"https://example.invalid/espectre-sdk-{version}.zip",
                "format": "zip",
                "filename": f"espectre-sdk-{version}.zip",
            }
        ],
        "install_surfaces": {
            "cmake": {
                "entrypoint": "src/cpp/espectre_sources.cmake",
                "optional_source_groups": [],
            },
            "esp_idf_component": {
                "component_root": "src/cpp",
                "cmake": "src/cpp/CMakeLists.txt",
                "kconfig": "src/cpp/Kconfig.projbuild",
            },
        },
    }

    page = stage.render_page(manifest, "release")

    assert f'data-sdk-stability="{expected_stability}"' in page
    assert ('data-sdk-production-ready="false"' in page) is not production_ready


def test_web_sdk_page_escapes_manifest_metadata() -> None:
    stage = load_script("stage_web_sdk")
    payload = '"><script>alert(1)</script>'
    manifest = {
        "channel": "release",
        "version": payload,
        "release_tag": payload,
        "protocol_version": payload,
        "supported_esp_idf": payload,
        "commit": payload,
        "artifacts": [{"url": f"https://example.invalid/{payload}", "format": "zip", "filename": payload}],
        "install_surfaces": {
            "cmake": {"entrypoint": payload, "optional_source_groups": [payload]},
            "esp_idf_component": {
                "component_root": payload,
                "cmake": payload,
                "kconfig": payload,
            },
        },
    }

    page = stage.render_page(manifest, "release")

    assert "<script>alert(1)</script>" not in page
    assert "&lt;script&gt;alert(1)&lt;/script&gt;" in page
    assert "&quot;&gt;&lt;script&gt;" in page


@pytest.mark.parametrize("artifact_url", ["javascript:alert(1)", "https:artifact.zip"])
def test_web_sdk_manifest_rejects_unsafe_artifact_urls(artifact_url: str) -> None:
    stage = load_script("stage_web_sdk")
    manifest = {
        "schema_version": 2,
        "channel": "preview",
        "version": "3.0.0-1-gabcdef1",
        "release_tag": "snapshot",
        "protocol_version": "1",
        "supported_esp_idf": ">=5.5.0",
        "commit": "abcdef1",
        "artifacts": [{"url": artifact_url, "format": "zip", "filename": "sdk.zip"}],
        "install_surfaces": {
            "cmake": {"entrypoint": "src/cpp/espectre_sources.cmake", "optional_source_groups": []},
            "esp_idf_component": {
                "component_root": "src/cpp",
                "cmake": "src/cpp/CMakeLists.txt",
                "kconfig": "src/cpp/Kconfig.projbuild",
            },
        },
    }

    with pytest.raises(ValueError, match="root-relative or HTTPS"):
        stage.validate_sdk_manifest(manifest, "preview")


@pytest.mark.parametrize("tag", ["v3.0.0", "03.0.0", "3.0.0-01", "3.0", "release"])
def test_release_validator_rejects_non_semver_tags(tag: str) -> None:
    validator = load_script("validate_release")
    with pytest.raises(ValueError, match="semantic versioning"):
        validator.validate(tag)


def test_release_validator_requires_a_finalized_matching_changelog(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    validator = load_script("validate_release")
    changelog = tmp_path / "CHANGELOG.md"
    monkeypatch.setattr(validator, "CHANGELOG", changelog)
    monkeypatch.setattr(validator, "detect_git_version", lambda **_kwargs: "3.0.0-rc1")

    changelog.write_text("## [3.0.0-rc1] - Unreleased\n", encoding="utf-8")
    with pytest.raises(ValueError, match="not finalized"):
        validator.validate("3.0.0-rc1")

    changelog.write_text("## [3.0.0-rc1] - 2026-08-12\n", encoding="utf-8")
    validator.validate("3.0.0-rc1")


def test_unstamped_sdk_header_has_no_numeric_fallback() -> None:
    builder = load_script("build_sdk_package")
    header = (REPO_ROOT / "src" / "cpp" / "runtime" / "espectre_sdk_version.h").read_text(
        encoding="utf-8"
    )
    assert "#define ESPECTRE_SDK_VERSION_STRING" not in header
    assert "ESPectre SDK version is unresolved" in header
    with pytest.raises(ValueError, match="Unable to detect ESPECTRE_SDK_VERSION_STRING"):
        builder.detect_sdk_version()


def test_git_version_cmake_reads_environment_before_git_describe() -> None:
    cmake = (REPO_ROOT / "src" / "cpp" / "espectre_git_version.cmake").read_text(encoding="utf-8")
    env_index = cmake.index("ENV{ESPECTRE_GIT_VERSION}")
    describe_index = cmake.index('git describe --tags --match "[0-9]*" --abbrev=7')
    workspace_index = cmake.index("ENV{GITHUB_WORKSPACE}")
    assert env_index < describe_index
    assert describe_index < cmake.index("header is not stamped")
    assert workspace_index < cmake.index("header is not stamped")


def test_native_loop_processes_wifi_events_before_frontend_updates() -> None:
    source = (
        REPO_ROOT / "src" / "cpp" / "frontend" / "native" / "app" / "main" / "app_main.cpp"
    ).read_text(encoding="utf-8")
    loop = source[source.index("void espectre_loop_task") : source.index("bool init_wifi_station")]

    assert loop.index("g_wifi_manager.loop();") < loop.index("g_frontend->loop();")


def test_esphome_forwards_numeric_project_version_to_sdk_cmake() -> None:
    pytest.importorskip("esphome")
    path = (
        REPO_ROOT
        / "src"
        / "cpp"
        / "frontend"
        / "esphome"
        / "components"
        / "espectre"
        / "__init__.py"
    )
    spec = importlib.util.spec_from_file_location("espectre_esphome_component", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    module._git_describe_version = lambda _root: "2.8.0-1-gabcdef0"
    assert module.resolve_espectre_git_version("9.9.9-ci-gdeadbee") == "9.9.9-ci-gdeadbee"
    assert module.resolve_espectre_git_version("main") == "2.8.0-1-gabcdef0"

    module._git_describe_version = lambda _root: None
    assert module.resolve_espectre_git_version("main") is None


def test_git_version_cmake_honors_environment(tmp_path: Path) -> None:
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("cmake is not installed")
    script = tmp_path / "probe.cmake"
    cmake_file = (REPO_ROOT / "src" / "cpp" / "espectre_git_version.cmake").as_posix()
    script.write_text(
        f'include("{cmake_file}")\n'
        "if(NOT ESPECTRE_GIT_VERSION STREQUAL \"2.8.0-99-gdeadbee\")\n"
        "  message(FATAL_ERROR \"got ${ESPECTRE_GIT_VERSION}\")\n"
        "endif()\n",
        encoding="utf-8",
    )
    env = os.environ.copy()
    env["ESPECTRE_GIT_VERSION"] = "2.8.0-99-gdeadbee"
    result = subprocess.run(
        [cmake, "-P", str(script)],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_detect_git_version_ignores_rolling_tags(monkeypatch: pytest.MonkeyPatch) -> None:
    detector = load_script("detect_git_version")

    def fake_run(command, **_kwargs):
        assert command == list(detector.GIT_DESCRIBE_CMD)
        class Result:
            returncode = 0
            stdout = "2.8.0-237-g7439944\n"
            stderr = ""

        return Result()

    monkeypatch.setattr(detector.subprocess, "run", fake_run)
    assert detector.detect_git_version() == "2.8.0-237-g7439944"
    assert detector.parse_version_core("2.8.0-237-g7439944") == (2, 8, 0)
    assert detector.parse_version_core("3.0.0-rc1") == (3, 0, 0)
    with pytest.raises(ValueError, match="numeric MAJOR.MINOR.PATCH"):
        detector.parse_version_core("preview")


def test_sdk_snapshot_stamps_git_describe_identity(tmp_path: Path) -> None:
    builder = load_script("build_sdk_package")
    _, develop_tag = _ota_release_tags()
    args = argparse.Namespace(
        channel="develop",
        version="2.8.0-237-g7439944",
        release_tag=develop_tag,
        output_dir=str(tmp_path),
        commit="7439944d441e9a8e485a1d610d99265d743e93f8",
        source_date_epoch=1_800_000_000,
        url_prefix=None,
    )
    manifest = builder.build_sdk_package(args)
    assert manifest["schema_version"] == 2
    assert manifest["version"] == "2.8.0-237-g7439944"
    assert "package_version" not in manifest
    assert "sdk_version" not in manifest
    assert manifest["release_tag"] == develop_tag
    assert manifest["supported_esp_idf"] == builder.SDK_SUPPORTED_ESP_IDF
    assert manifest["artifacts"][0]["filename"] == "espectre-sdk-develop.tar.gz"
    assert f"/releases/download/{develop_tag}/" in manifest["artifacts"][0]["url"]

    zip_path = tmp_path / "espectre-sdk-develop.zip"
    with zipfile.ZipFile(zip_path) as archive:
        header_name = next(name for name in archive.namelist() if name.endswith("/src/cpp/runtime/espectre_sdk_version.h"))
        header = archive.read(header_name).decode("utf-8")
        yml_name = next(name for name in archive.namelist() if name.endswith("/src/cpp/idf_component.yml"))
        yml = archive.read(yml_name).decode("utf-8")
        doxy_name = next(name for name in archive.namelist() if name.endswith("/src/cpp/Doxyfile"))
        bundled_doxyfile = archive.read(doxy_name).decode("utf-8")
    assert '#define ESPECTRE_SDK_VERSION_STRING "2.8.0-237-g7439944"' in header
    assert "#define ESPECTRE_SDK_VERSION_MAJOR 2" in header
    assert "ESPectre SDK version is unresolved" not in header
    assert 'version: "2.8.0-237-g7439944"' in yml
    source_manifest = builder.IDF_COMPONENT_MANIFEST.read_text(encoding="utf-8")
    assert yml.split("\ndependencies:\n", 1)[1] == source_manifest.split("\ndependencies:\n", 1)[1]
    assert f'version: "{builder.SDK_SUPPORTED_ESP_IDF}"' in yml
    assert "https://github.com/improv-wifi/sdk-cpp.git" in yml
    assert "espressif/mdns:" in yml
    assert "espressif/esp_tinyusb:" in yml
    assert '- if: "target in [esp32s2, esp32s3]"' in yml
    assert re.search(r"(?m)^PROJECT_NUMBER\s*=\s*2\.8\.0-237-g7439944\s*$", bundled_doxyfile)
    for relative_path in (
        "src/cpp/frontend/native/espectre/idf_component.yml",
        "src/cpp/frontend/matter/espectre/idf_component.yml",
        "src/cpp/frontend/matter/app/main/idf_component.yml",
    ):
        frontend_manifest = (REPO_ROOT / relative_path).read_text(encoding="utf-8")
        assert f'version: "{builder.SDK_SUPPORTED_ESP_IDF}"' in frontend_manifest

    native_manifest = (
        REPO_ROOT / "src" / "cpp" / "frontend" / "native" / "espectre" / "idf_component.yml"
    ).read_text(encoding="utf-8")
    assert "https://github.com/improv-wifi/sdk-cpp.git" in native_manifest
    assert "espressif/mdns:" in native_manifest

    for relative_path in (
        "src/cpp/frontend/native/espectre/idf_component.yml",
        "src/cpp/frontend/esphome/components/espectre/idf_component.yml",
    ):
        frontend_manifest = (REPO_ROOT / relative_path).read_text(encoding="utf-8")
        assert "espressif/esp_tinyusb:" in frontend_manifest
        assert '- if: "target in [esp32s2, esp32s3]"' in frontend_manifest

    matter_manifest = (
        REPO_ROOT / "src" / "cpp" / "frontend" / "matter" / "espectre" / "idf_component.yml"
    ).read_text(encoding="utf-8")
    assert "espressif/mdns:" in matter_manifest
    assert "espressif/esp_tinyusb:" in matter_manifest
    assert '- if: "target == esp32s3"' in matter_manifest
    assert "esp32s2" not in matter_manifest
    assert not (
        REPO_ROOT / "src" / "cpp" / "frontend" / "matter" / "app" / "sdkconfig.defaults.esp32s2"
    ).exists()


def test_release_sdk_rejects_a_version_tag_mismatch(tmp_path: Path) -> None:
    builder = load_script("build_sdk_package")
    args = argparse.Namespace(
        channel="release",
        version="3.0.0-rc1",
        release_tag="3.0.0",
        output_dir=str(tmp_path),
        commit="0123456789abcdef",
        source_date_epoch=1_800_000_000,
        url_prefix=None,
    )

    with pytest.raises(ValueError, match="Release SDK version and release tag must match"):
        builder.build_sdk_package(args)


def test_release_firmware_rejects_a_version_tag_mismatch(tmp_path: Path) -> None:
    builder = load_script("build_firmware_manifest")
    firmware_dir = tmp_path / "firmware"
    output = tmp_path / "firmware-manifest-release.json"
    firmware_dir.mkdir()
    args = argparse.Namespace(
        firmware_dir=str(firmware_dir),
        output=str(output),
        channel="release",
        version="3.0.0-rc1",
        release_tag="3.0.0",
        commit="0123456789abcdef",
        url_prefix=None,
    )

    with pytest.raises(ValueError, match="Release firmware version and release tag must match"):
        builder.build_manifest(args)
    assert not output.exists()


def test_local_release_staging_uses_the_build_version(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.syspath_prepend(str(SCRIPTS_DIR))
    stager = importlib.import_module("stage_web_firmware")
    existing = {"release_tag": "3.0.0-rc1"}

    assert (
        stager.resolve_local_release_tag(
            "release",
            "2.8.0-408-ga7d5d0a",
            None,
            existing,
        )
        == "2.8.0-408-ga7d5d0a"
    )
    assert stager.resolve_local_release_tag("preview", "snapshot", None, existing) == "3.0.0-rc1"
    assert stager.resolve_local_release_tag("release", "snapshot", "override", existing) == "override"


def test_local_staging_discovers_nested_esphome_builds(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.syspath_prepend(str(SCRIPTS_DIR))
    stager = importlib.import_module("stage_web_firmware")
    build_root = tmp_path / "build"
    for chip in ("esp32", "esp32c3"):
        artifact_dir = build_root / f"espectre-{chip}" / "espectre" / "build"
        artifact_dir.mkdir(parents=True)
        (artifact_dir / "firmware.factory.bin").write_bytes(chip.encode("utf-8"))
        (artifact_dir / "project_description.json").write_text(
            json.dumps({"target": chip}),
            encoding="utf-8",
        )
    monkeypatch.setattr(stager, "ESPHOME_BUILD_ROOT", build_root)

    images = stager.discover_esphome_images(None)

    assert [(image.frontend, image.chip) for image in images] == [
        ("esphome", "esp32"),
        ("esphome", "esp32c3"),
    ]


def test_web_firmware_staging_validates_before_cleaning_output(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.syspath_prepend(str(SCRIPTS_DIR))
    stager = importlib.import_module("stage_web_firmware")
    firmware_dir = tmp_path / "firmware"
    output_dir = tmp_path / "output"
    firmware_dir.mkdir()
    output_dir.mkdir()
    existing = output_dir / "espectre-native-3.0.0-esp32s3.bin"
    existing.write_bytes(b"existing firmware")
    args = argparse.Namespace(
        firmware_dir=str(firmware_dir),
        output_dir=str(output_dir),
        channel="release",
        version="3.0.0-rc1",
        release_tag="3.0.0",
        commit="0123456789abcdef",
        url_prefix="/artifacts/firmware/release",
    )

    with pytest.raises(ValueError, match="Release firmware version and release tag must match"):
        stager.stage_web_firmware(args)
    assert existing.read_bytes() == b"existing firmware"


def test_generate_sdk_api_stamps_a_working_copy_without_mutating_the_repo(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    generator = load_script("generate_sdk_api")
    repo_doxyfile = (REPO_ROOT / "src" / "cpp" / "Doxyfile").read_text(encoding="utf-8")
    stamped_versions: list[str] = []
    api_output = tmp_path / "sdk" / "api"
    api_output.mkdir(parents=True)
    stale_page = api_output / "stale-internal-type.html"
    stale_page.write_text("stale", encoding="utf-8")

    def stamped_output(path: Path) -> Path:
        source = path.read_text(encoding="utf-8")
        match = re.search(r"(?m)^OUTPUT_DIRECTORY\s*=\s*(\S+)\s*$", source)
        assert match is not None
        return Path(match.group(1))

    def fake_doxygen(path: Path) -> None:
        stamped = path.read_text(encoding="utf-8")
        match = re.search(r"(?m)^PROJECT_NUMBER\s*=\s*(\S+)\s*$", stamped)
        assert match is not None
        stamped_versions.append(match.group(1))
        xml = stamped_output(path) / "xml"
        xml.mkdir(parents=True)
        (xml / "index.xml").write_text(
            '<doxygenindex version="1.17">'
            '<compound refid="classespectre_1_1_runtime_frontend_controller" kind="class"><name>espectre::RuntimeFrontendController</name></compound>'
            '<compound refid="espectre__sdk__version_8h" kind="file"><name>espectre_sdk_version.h</name></compound>'
            "</doxygenindex>",
            encoding="utf-8",
        )
        (xml / "classespectre_1_1_runtime_frontend_controller.xml").write_text(
            '<doxygen><compounddef><sectiondef kind="private-func"><memberdef prot="private" kind="function"/></sectiondef></compounddef></doxygen>',
            encoding="utf-8",
        )

    def fake_mcss(path: Path, _root: Path | None) -> None:
        output = stamped_output(path)
        assert 'prot="private"' not in (
            output / "xml" / "classespectre_1_1_runtime_frontend_controller.xml"
        ).read_text(encoding="utf-8")
        rendered = output / "rendered"
        rendered.mkdir()
        (rendered / "index.html").write_text(
            '<article data-api-reference-fragment="index">'
            '<a href="classespectre_1_1_runtime_frontend_controller.html" '
            'onclick="return toggle(this)">Controller</a>'
            '<script>function toggle() { return false; }</script>'
            '</article>',
            encoding="utf-8",
        )
        (rendered / "classespectre_1_1_runtime_frontend_controller.html").write_text(
            '<article data-api-reference-fragment="classespectre_1_1_runtime_frontend_controller"><h1>Controller</h1><nav class="m-block m-default"><h3>Local navigation</h3></nav><section id="members"><h2>Members</h2></section></article>',
            encoding="utf-8",
        )
        (rendered / "files.html").write_text(
            '<article data-api-reference-fragment="files"><h1>Files</h1></article>',
            encoding="utf-8",
        )
        (rendered / "espectre__sdk__version_8h.html").write_text(
            '<article data-api-reference-fragment="espectre__sdk__version_8h"><section class="m-doc-details">Version defines</section></article>',
            encoding="utf-8",
        )

    monkeypatch.setattr(generator, "run_doxygen", fake_doxygen)
    monkeypatch.setattr(generator, "run_mcss", fake_mcss)
    monkeypatch.setattr(generator, "API_OUTPUT_DIR", api_output)
    version = generator.generate_sdk_api("3.0.0-12-gabcdef1")
    assert version == "3.0.0-12-gabcdef1"
    assert stamped_versions == ["3.0.0-12-gabcdef1"]
    assert not stale_page.exists()
    manifest = json.loads((api_output / "api-index.json").read_text(encoding="utf-8"))
    assert manifest["sdk_version"] == "3.0.0-12-gabcdef1"
    assert manifest["renderer"] == "m.css"
    entries = {entry["refid"]: entry for entry in manifest["entries"]}
    assert entries["index"]["discoverable"] is True
    assert entries["classespectre_1_1_runtime_frontend_controller"]["discoverable"] is True
    assert entries["files"]["discoverable"] is False
    assert entries["espectre__sdk__version_8h"]["discoverable"] is True
    controller_fragment = (
        api_output / "fragments" / "classespectre_1_1_runtime_frontend_controller.html"
    ).read_text(encoding="utf-8")
    assert '<nav class="m-block' not in controller_fragment
    assert '<section id="members">' in controller_fragment
    index_fragment = (api_output / "fragments" / "index.html").read_text(encoding="utf-8")
    assert "onclick=" not in index_fragment
    assert "<script" not in index_fragment
    assert (REPO_ROOT / "src" / "cpp" / "Doxyfile").read_text(encoding="utf-8") == repo_doxyfile
    assert re.search(r"(?m)^PROJECT_NUMBER\s*=\s*UNSTAMPED\s*$", repo_doxyfile)


def test_sdk_bundle_rewrites_the_repo_doxyfile_preamble(tmp_path: Path) -> None:
    packager = load_script("build_sdk_package")
    bundled = tmp_path / "Doxyfile"
    bundled.write_text((REPO_ROOT / "src" / "cpp" / "Doxyfile").read_text(encoding="utf-8"))
    packager.rewrite_bundle_doxyfile(bundled, "3.0.0")
    rewritten = bundled.read_text(encoding="utf-8")
    assert "# Usage, from the unpacked SDK bundle root:" in rewritten
    assert re.search(r"(?m)^OUTPUT_DIRECTORY\s*=\s*output\s*$", rewritten)
    assert re.search(r"(?m)^PROJECT_NUMBER\s*=\s*3\.0\.0\s*$", rewritten)


def test_generate_sdk_api_requires_the_pinned_doxygen_version(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    generator = load_script("generate_sdk_api")
    required_version = generator.REQUIRED_DOXYGEN_VERSION
    major, minor, patch = map(int, required_version.split("."))
    other_version = f"{major}.{minor}.{patch + 1}"
    monkeypatch.setattr(generator, "detect_doxygen_version", lambda: other_version)
    with pytest.raises(ValueError, match=re.escape(required_version)):
        generator.require_doxygen_version()
    monkeypatch.setattr(generator, "detect_doxygen_version", lambda: required_version)
    generator.require_doxygen_version()


def test_sdk_api_fragment_security_rejects_active_markup() -> None:
    security = load_script("web_html_security")
    for fragment in (
        '<article><iframe src="https://example.com"></iframe></article>',
        '<article><a href="javascript:alert(1)">unsafe</a></article>',
        '<article><a href="//example.com/unsafe">unsafe</a></article>',
        '<article style="background: url(https://example.com)">unsafe</article>',
    ):
        with pytest.raises(ValueError, match="unsafe"):
            security.passivize_api_fragment(fragment)


def test_indexnow_retries_transient_failures_and_sends_the_sitemap(tmp_path: Path) -> None:
    indexnow = load_script("notify_indexnow")
    sitemap = tmp_path / "sitemap.xml"
    sitemap.write_text(
        '<?xml version="1.0"?><urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">'
        "<url><loc>https://espectre.dev/</loc></url>"
        "<url><loc>https://espectre.dev/sdk/</loc></url></urlset>",
        encoding="utf-8",
    )
    urls = indexnow.sitemap_urls(sitemap)
    calls: list[tuple[object, float]] = []
    sleeps: list[float] = []

    class Response:
        status = 202

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return None

    def request(request, *, timeout):
        calls.append((request, timeout))
        if len(calls) < 3:
            raise OSError("temporary failure")
        return Response()

    indexnow.notify(urls, request_fn=request, sleep_fn=sleeps.append, timeout=7.5)

    assert len(calls) == 3
    assert sleeps == [1.0, 2.0]
    assert all(timeout == 7.5 for _, timeout in calls)
    payload = json.loads(calls[-1][0].data)
    assert payload["host"] == "espectre.dev"
    assert payload["urlList"] == urls


def test_sitemap_lastmod_dates_use_utc() -> None:
    sitemap_builder = load_script("build_sitemap")
    assert sitemap_builder.normalized_date("2026-08-19T00:41:27+02:00") == "2026-08-18"
    assert sitemap_builder.normalized_date("2026-08-18T23:30:00Z") == "2026-08-18"
    assert sitemap_builder.normalized_date("2026-08-19T00:00:00+00:00") == "2026-08-19"
    assert sitemap_builder.normalized_date("2026-08-19") == "2026-08-19"


def test_sitemap_builder_uses_git_and_sdk_manifest_dates(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    sitemap_builder = load_script("build_sitemap")
    web_root = tmp_path / "web"
    release_dir = web_root / "artifacts" / "sdk" / "release"
    preview_dir = web_root / "artifacts" / "sdk" / "preview"
    develop_dir = web_root / "artifacts" / "sdk" / "develop"
    release_dir.mkdir(parents=True)
    preview_dir.mkdir(parents=True)
    develop_dir.mkdir(parents=True)
    (release_dir / "sdk-manifest-release.json").write_text(
        json.dumps({"channel": "release", "generated_at": "2026-08-01T09:30:00Z"}),
        encoding="utf-8",
    )
    (preview_dir / "sdk-manifest-preview.json").write_text(
        json.dumps({"channel": "preview", "generated_at": "2026-08-12T10:45:00+00:00"}),
        encoding="utf-8",
    )
    (develop_dir / "sdk-manifest-develop.json").write_text(
        json.dumps({"channel": "develop", "generated_at": "2026-08-14T11:15:00+00:00"}),
        encoding="utf-8",
    )
    for channel_dir in (release_dir, preview_dir, develop_dir):
        (channel_dir / "index.html").write_text("<main></main>", encoding="utf-8")
    monkeypatch.setattr(sitemap_builder, "WEB_ROOT", web_root)

    def fake_git_date(paths):
        if paths == sitemap_builder.SDK_CHANNEL_PAGE_INPUTS:
            return "2026-08-10"
        if sitemap_builder.DOXYFILE in paths:
            return "2026-08-08"
        return "2026-08-09"

    monkeypatch.setattr(sitemap_builder, "latest_git_date", fake_git_date)
    output = tmp_path / "generated.xml"
    sitemap_builder.build_sitemap(output)

    root = ET.parse(output).getroot()
    namespace = {"s": sitemap_builder.SITEMAP_NAMESPACE}
    entries = {
        entry.findtext("s:loc", namespaces=namespace): entry.findtext("s:lastmod", namespaces=namespace)
        for entry in root.findall("s:url", namespace)
    }
    assert {url: entries[url] for url in (
        "https://espectre.dev/",
        "https://espectre.dev/sdk/api/",
        "https://espectre.dev/artifacts/sdk/release/",
        "https://espectre.dev/artifacts/sdk/preview/",
        "https://espectre.dev/artifacts/sdk/develop/",
    )} == {
        "https://espectre.dev/": "2026-08-09",
        "https://espectre.dev/sdk/api/": "2026-08-08",
        "https://espectre.dev/artifacts/sdk/release/": "2026-08-10",
        "https://espectre.dev/artifacts/sdk/preview/": "2026-08-12",
        "https://espectre.dev/artifacts/sdk/develop/": "2026-08-14",
    }
    assert root.findall("s:url/s:changefreq", namespace) == []

    expected_urls = {
        f"{sitemap_builder.SITE_ORIGIN}{route['staticPath']}"
        for route in sitemap_builder.ROUTE_MANIFEST["routes"]
    } | {
        f"{sitemap_builder.SITE_ORIGIN}{channel['path']}"
        for channel in sitemap_builder.ROUTE_MANIFEST["sdkChannels"]
    }
    assert entries.keys() == expected_urls


def test_sitemap_omits_unstaged_sdk_channels_and_rejects_partial_staging(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    sitemap_builder = load_script("build_sitemap")
    web_root = tmp_path / "web"
    monkeypatch.setattr(sitemap_builder, "WEB_ROOT", web_root)
    output = tmp_path / "generated.xml"
    sitemap_builder.build_sitemap(output)
    urls = {
        element.text
        for element in ET.parse(output).getroot().findall(
            f"{{{sitemap_builder.SITEMAP_NAMESPACE}}}url/"
            f"{{{sitemap_builder.SITEMAP_NAMESPACE}}}loc"
        )
    }
    assert not any("/artifacts/sdk/" in str(url) for url in urls)

    release_dir = web_root / "artifacts" / "sdk" / "release"
    release_dir.mkdir(parents=True)
    (release_dir / "sdk-manifest-release.json").write_text("{}", encoding="utf-8")
    with pytest.raises(ValueError, match="Incomplete staged SDK channel release"):
        sitemap_builder.build_sitemap(output)


def test_pages_build_outputs_do_not_overlap_committed_sources() -> None:
    static_pages = load_script("build_static_pages")
    sitemap_builder = load_script("build_sitemap")
    source_paths = set(
        subprocess.run(
            [
                "git",
                "ls-files",
                "--cached",
                "--others",
                "--exclude-standard",
                ".github/scripts",
                "docs/web",
            ],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.splitlines()
    )
    source_paths.difference_update(
        subprocess.run(
            ["git", "ls-files", "--deleted", "docs/web"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.splitlines()
    )
    generated_paths = {
        "docs/web/artifacts",
        "docs/web/node_modules",
        "docs/web/sitemap.xml",
        "docs/web/vendor",
        *(
            f"docs/web/{page['output'].strip('/')}"
            for page in static_pages.PAGES
        ),
    }

    assert sitemap_builder.DEFAULT_SITEMAP_OUTPUT == (
        REPO_ROOT / "docs" / "web" / "sitemap.xml"
    )
    assert "docs/web/routes.json" in source_paths
    for generated_path in generated_paths:
        assert not any(
            path == generated_path or path.startswith(f"{generated_path}/")
            for path in source_paths
        ), f"Pages build output overlaps committed source: {generated_path}"


def test_pages_verifier_spa_routes_match_the_route_registry() -> None:
    verifier = load_script("verify_web_build")
    verifier.verify_spa_routes()


def test_pages_verifier_requires_api_reference_to_show_sdk_version(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    verifier = load_script("verify_web_build")
    monkeypatch.setattr(verifier, "WEB_ROOT", tmp_path)
    version = "2.8.0-237-g7439944"
    monkeypatch.setattr(verifier, "detect_git_version", lambda: version)
    api = tmp_path / "artifacts" / "sdk" / "api"
    fragments = api / "fragments"
    fragments.mkdir(parents=True)
    refids = (
        "classespectre_1_1_runtime_frontend_controller",
        "structespectre_1_1_runtime_config",
        "classespectre_1_1_i_runtime_listener",
    )
    entries = []
    for refid in refids:
        fragment = f"fragments/{refid}.html"
        (api / fragment).write_text("<article>API reference</article>", encoding="utf-8")
        entries.append({"refid": refid, "fragment": fragment, "discoverable": True})
    manifest = {
        "sdk_version": version,
        "renderer": "m.css",
        "renderer_revision": "0123456789abcdef",
        "entries": entries,
    }
    manifest_path = api / "api-index.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    verifier.verify_sdk_api_version()
    manifest["sdk_version"] = "UNSTAMPED"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="does not show version"):
        verifier.verify_sdk_api_version()


def test_pages_verifier_rejects_missing_spa_routes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    verifier = load_script("verify_web_build")
    monkeypatch.setattr(verifier, "WEB_ROOT", tmp_path)
    monkeypatch.setattr(verifier, "ROUTE_MANIFEST", {
        "routes": [
            {"name": "home", "staticPath": "/"},
            {"name": "device", "staticPath": "/device/"},
        ]
    })
    (tmp_path / "index.html").write_text('<main data-page="home"></main>', encoding="utf-8")
    with pytest.raises(ValueError, match=r"missing=\['device'\]"):
        verifier.verify_spa_routes()


def test_pages_verifier_requires_every_registered_static_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    verifier = load_script("verify_web_build")
    monkeypatch.setattr(verifier, "WEB_ROOT", tmp_path)
    monkeypatch.setattr(verifier, "ROUTE_MANIFEST", {
        "siteOrigin": "https://espectre.dev",
        "routes": [
            {
                "name": "guides", "staticPath": "/guides/",
                "title": "Guides | ESPectre", "description": "Guides",
            },
            {
                "name": "guide-home-assistant", "staticPath": "/guides/home-assistant/",
                "title": "Home Assistant | ESPectre", "description": "Home Assistant",
            },
        ]
    })
    guides_dir = tmp_path / "guides"
    guides_dir.mkdir()
    (guides_dir / "index.html").write_text("<main></main>", encoding="utf-8")

    with pytest.raises(
        FileNotFoundError,
        match="guides/home-assistant/index.html",
    ):
        verifier.verify_generated_pages()


def test_static_page_builder_escapes_and_verifies_route_metadata(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    static_pages = load_script("build_static_pages")
    verifier = load_script("verify_web_build")
    title = 'A "quoted" <route> | ESPectre'
    description = 'Use "Direct" safely & keep <metadata> intact.'
    spec = {
        **static_pages.PAGES[0],
        "name": "quoted",
        "source": "content/quoted.html",
        "output": "quoted",
        "title": title,
        "description": description,
    }
    content = tmp_path / "content"
    content.mkdir()
    (content / "quoted.html").write_text("<article><h1>Quoted</h1></article>", encoding="utf-8")
    monkeypatch.setattr(static_pages, "WEB_ROOT", tmp_path)
    monkeypatch.setattr(static_pages, "PAGES", (spec,))
    monkeypatch.setattr(static_pages, "asset_version", lambda _path: "0123456789ab")
    static_pages.build()

    route = {
        "name": "quoted",
        "staticPath": "/quoted/",
        "title": title,
        "description": description,
    }
    monkeypatch.setattr(verifier, "WEB_ROOT", tmp_path)
    monkeypatch.setattr(verifier, "ROUTE_MANIFEST", {
        "siteOrigin": "https://espectre.dev",
        "routes": [route],
    })
    verifier.verify_generated_pages()
    generated = (tmp_path / "quoted" / "index.html").read_text(encoding="utf-8")
    assert '&quot;Direct&quot;' in generated
    assert "&lt;metadata&gt;" in generated


def test_generated_pages_have_sitemap_lastmod_ownership() -> None:
    static_pages = load_script("build_static_pages")
    sitemap_builder = load_script("build_sitemap")
    verifier = load_script("verify_web_build")

    sitemap_paths = {urlparse(url).path for url in sitemap_builder.public_urls()}
    generated_pages = {
        f"/{page['output'].strip('/')}/": Path("docs/web") / page["source"]
        for page in static_pages.PAGES
    }

    assert len(generated_pages) == len(static_pages.PAGES), "Generated page routes must be unique"
    assert not generated_pages.keys() - sitemap_paths, (
        "Generated pages missing from the sitemap: "
        f"{sorted(generated_pages.keys() - sitemap_paths)}"
    )
    assert sitemap_paths == verifier.expected_sitemap_paths()

    for route, source in generated_pages.items():
        assert route in sitemap_builder.ROUTE_SOURCES, (
            f"Generated page {route} has no sitemap lastmod ownership mapping"
        )
        ownership = sitemap_builder.ROUTE_SOURCES[route]
        assert source in ownership, f"Sitemap lastmod for {route} does not track {source}"
        assert sitemap_builder.STATIC_PAGE_BUILDER in ownership, (
            f"Sitemap lastmod for {route} does not track the static page builder"
        )
        assert sitemap_builder.WEB_PAGE_SHELL in ownership, (
            f"Sitemap lastmod for {route} does not track the shared page shell"
        )
        assert sitemap_builder.ROUTE_BOOTSTRAP in ownership, (
            f"Sitemap lastmod for {route} does not track the route bootstrap"
        )

    assert sitemap_builder.SDK_API_BUILDER in sitemap_builder.SDK_API_INPUTS
    assert sitemap_builder.MCSS_TEMPLATES in sitemap_builder.SDK_API_INPUTS


def test_sitemap_verifier_requires_accurate_dates(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    verifier = load_script("verify_web_build")
    monkeypatch.setattr(verifier, "WEB_ROOT", tmp_path)
    namespace = verifier.SITEMAP_NAMESPACE
    expected_paths = verifier.expected_sitemap_paths()
    entries = "".join(
        f"<url><loc>https://espectre.dev{path}</loc>"
        + "<lastmod>2026-08-12</lastmod>"
        + "</url>"
        for path in sorted(expected_paths)
    )
    sitemap = tmp_path / "sitemap.xml"
    sitemap.write_text(
        f'<?xml version="1.0"?><urlset xmlns="{namespace}">{entries}</urlset>',
        encoding="utf-8",
    )
    valid_source = sitemap.read_text(encoding="utf-8")
    verifier.verify_sitemap(require_preview=False, require_release=False, require_develop=False)

    future = valid_source.replace(
        "<lastmod>2026-08-12</lastmod>",
        "<lastmod>2099-01-01</lastmod>",
        1,
    )
    sitemap.write_text(future, encoding="utf-8")
    with pytest.raises(ValueError, match="lastmod is in the future"):
        verifier.verify_sitemap(require_preview=False, require_release=False, require_develop=False)
    sitemap.write_text(
        future.replace("<lastmod>2099-01-01</lastmod>", "<lastmod>2026-08-12</lastmod>", 1),
        encoding="utf-8",
    )

    source = sitemap.read_text(encoding="utf-8").replace(
        "<lastmod>2026-08-12</lastmod>",
        "<lastmod>2026-08-12</lastmod><changefreq>daily</changefreq>",
        1,
    )
    sitemap.write_text(source, encoding="utf-8")
    with pytest.raises(ValueError, match="must not contain changefreq"):
        verifier.verify_sitemap(require_preview=False, require_release=False, require_develop=False)

    sitemap.write_text(
        valid_source.replace("<lastmod>2026-08-12</lastmod>", "", 1),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="missing lastmod"):
        verifier.verify_sitemap(require_preview=False, require_release=False, require_develop=False)


def test_pages_verifier_enforces_exact_artifact_contracts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    verifier = load_script("verify_web_build")
    monkeypatch.setattr(verifier, "WEB_ROOT", tmp_path)
    firmware_dir = tmp_path / "artifacts" / "firmware" / "preview"
    sdk_dir = tmp_path / "artifacts" / "sdk" / "preview"
    firmware_dir.mkdir(parents=True)
    sdk_dir.mkdir(parents=True)

    frontends = {}
    for frontend in sorted(verifier.EXPECTED_FRONTENDS):
        artifacts = []
        for chip in sorted(verifier.EXPECTED_CHIPS_BY_FRONTEND[frontend]):
            filename = f"espectre-{frontend}-{chip}.bin"
            (firmware_dir / filename).write_bytes(b"firmware")
            artifacts.append({"build_type": "factory", "chip": chip, "filename": filename})
        frontends[frontend] = {"artifacts": artifacts}
    firmware_manifest = {
        "channel": "preview",
        "version": "3.0.0-rc1-12-gabcdef1",
        "frontends": frontends,
    }
    firmware_manifest_path = firmware_dir / "firmware-manifest-preview.json"
    firmware_manifest_path.write_text(json.dumps(firmware_manifest), encoding="utf-8")
    verifier.verify_firmware_channel("preview")

    firmware_manifest["version"] = "2.8.0-434-g2a4cfc8"
    firmware_manifest_path.write_text(json.dumps(firmware_manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="version 3 or newer"):
        verifier.verify_firmware_channel("preview")
    firmware_manifest["version"] = "3.0.0-rc1-12-gabcdef1"

    frontends["native"]["artifacts"].append(frontends["native"]["artifacts"][0])
    firmware_manifest_path.write_text(json.dumps(firmware_manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate firmware"):
        verifier.verify_firmware_channel("preview")

    (sdk_dir / "index.html").write_text("SDK", encoding="utf-8")
    sdk_manifest_path = sdk_dir / "sdk-manifest-preview.json"
    sdk_manifest = {
        "schema_version": 2,
        "channel": "preview",
        "version": "3.0.0-rc1-12-gabcdef1",
        "release_tag": "snapshot",
        "artifacts": [
            {"format": "tar.gz", "sha256": "a" * 64},
            {"format": "zip", "sha256": "b" * 64},
        ],
    }
    sdk_manifest_path.write_text(json.dumps(sdk_manifest), encoding="utf-8")
    verifier.verify_sdk_channel("preview")
    sdk_manifest["version"] = "2.8.0"
    sdk_manifest_path.write_text(json.dumps(sdk_manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="version 3 or newer"):
        verifier.verify_sdk_channel("preview")
    sdk_manifest["version"] = "3.0.0-rc1-12-gabcdef1"
    sdk_manifest["artifacts"][0]["sha256"] = "invalid"
    sdk_manifest_path.write_text(json.dumps(sdk_manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="SHA-256"):
        verifier.verify_sdk_channel("preview")

    with pytest.raises(ValueError, match="escapes"):
        verifier.require_file("../outside")


def test_workflows_keep_publication_and_supply_chain_guardrails() -> None:
    workflow_sources = {
        path.name: path.read_text(encoding="utf-8")
        for path in sorted(WORKFLOWS_DIR.glob("*.yml"))
    }
    combined = "\n".join(workflow_sources.values())
    assert "ubuntu-latest" not in combined
    assert combined.count("runs-on:") == combined.count("timeout-minutes:")

    external_action = re.compile(r"^\s*uses:\s*([^./\s][^@\s]*)@([^\s#]+)", re.MULTILINE)
    refs = external_action.findall(combined)
    assert refs
    assert all(re.fullmatch(r"[0-9a-f]{40}", ref) for _, ref in refs)

    ci = workflow_sources["ci.yml"]
    snapshot = workflow_sources["snapshot.yml"]
    release = workflow_sources["release.yml"]
    assert "HEAD~1" not in snapshot
    assert "gh release delete" not in snapshot
    assert "git.getRef" in snapshot
    assert "git.updateRef" in snapshot and "git.createRef" in snapshot
    assert "workflow_dispatch:" in snapshot
    assert "ci_run_id:" in snapshot
    assert "needs.validate-run.outputs.run_id" in snapshot
    assert "github.event.workflow_run.id" not in snapshot
    assert "validate-release:" in release
    assert "No successful main CI push run" in release
    assert "git merge-base --is-ancestor" in release
    preview_tag, develop_tag = _ota_release_tags()
    assert re.search(rf'(?m)^              echo "tag={re.escape(develop_tag)}"$', snapshot)
    assert re.search(rf'(?m)^              echo "tag={re.escape(preview_tag)}"$', snapshot)
    assert re.search(rf'(?m)^              echo "release_tag={re.escape(develop_tag)}"$', ci)
    assert re.search(rf'(?m)^              echo "release_tag={re.escape(preview_tag)}"$', ci)
    assert "detect_git_version.py" in ci
    assert "detect_git_version.py" in snapshot
    assert "ESPECTRE_GIT_VERSION: ${{ steps.git-version.outputs.version }}" in ci
    assert "ESPECTRE_GIT_VERSION: ${{ github.ref_name }}" in release
    published_channel_action = (
        REPO_ROOT / ".github" / "actions" / "stage-published-web-channel" / "action.yml"
    ).read_text(encoding="utf-8")
    assert "detect_git_version.py" in published_channel_action
    assert snapshot.count("uses: ./.github/actions/stage-published-web-channel") == 2
    assert release.count("uses: ./.github/actions/stage-published-web-channel") == 2
    for tag in (preview_tag, develop_tag):
        assert f"release-tag: {tag}" in release
    assert f"release-tag: {develop_tag}" in snapshot
    assert "release-tag: latest" in snapshot
    for expected in (
        "gh release view",
        "gh release download",
        ".github/scripts/stage_web_firmware.py",
        ".github/scripts/stage_web_sdk.py",
        "firmware-compliance-*.zip",
    ):
        assert expected in published_channel_action
    for source in (ci, snapshot, release):
        assert "uses: ./.github/actions/build-pages" in source
        assert "fetch-depth: 0" in source
    for source in (snapshot, release):
        assert "docs/web/artifacts/firmware/release" in source
        assert "name: website-sitemap" in source
        assert "path: docs/web/sitemap.xml" in source
        assert "path: deployed-website" in source
        assert "notify_indexnow.py --sitemap deployed-website/sitemap.xml" in source
    assert 'require-preview: "true"' in snapshot
    assert "require-release: ${{ steps.release-assets.outputs.staged }}" in snapshot
    assert 'require-release: "true"' in release
    assert "require-preview: ${{ steps.preview-assets.outputs.staged }}" in release
    pages_action = (REPO_ROOT / ".github" / "actions" / "build-pages" / "action.yml").read_text(
        encoding="utf-8"
    )
    assert ".github/scripts/build_sitemap.py" in pages_action
    assert ".github/scripts/generate_sdk_api.py" in pages_action
    assert "doxygen src/cpp/Doxyfile" not in pages_action
    generator = load_script("generate_sdk_api")
    assert f'version="{generator.REQUIRED_DOXYGEN_VERSION}"' in pages_action
    assert "doxygen-${version}.linux.bin.tar.gz" in pages_action
    assert "apt-get install -y --no-install-recommends doxygen" not in pages_action

    for script_name in (
        "build_matter_firmware.sh",
        "build_native_firmware.sh",
    ):
        source = (SCRIPTS_DIR / script_name).read_text(encoding="utf-8")
        assert IDF_DOCKER_IMAGE in source
        assert 'BUILD_DIR="build-container-${' in source
        assert 'detect_git_version.py' in source
        assert '-e ESPECTRE_GIT_VERSION="${ESPECTRE_GIT_VERSION}"' in source
        assert ".espectre-requirements-\\${REQUIREMENTS_HASH}" in source
        assert "--backend local" in source


def test_website_sources_integrate_sdk_api_fragments_in_portal_page() -> None:
    sdk_landing = (REPO_ROOT / "docs" / "web" / "content" / "sdk.html").read_text(
        encoding="utf-8"
    )
    api_orientation = (
        REPO_ROOT / "docs" / "web" / "content" / "sdk" / "api.html"
    ).read_text(encoding="utf-8")

    assert 'href="/sdk/api/" class="doc-link"' in sdk_landing
    assert 'data-api-reference-browser' in api_orientation
    assert 'data-api-index="/artifacts/sdk/api/api-index.json"' in api_orientation
    assert 'data-api-reference-content' in api_orientation
    assert 'data-api-reference-picker' in api_orientation
    assert 'data-api-reference-filter' in api_orientation
    assert 'data-api-reference-results' in api_orientation
    assert 'data-api-reference-toggle' not in api_orientation
    assert 'data-page-toc' in api_orientation
    assert 'data-page-path="sdk"' in api_orientation
    assert 'api-reference-index' not in api_orientation
    assert '<iframe' not in api_orientation
