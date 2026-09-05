# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Enforce the C++ dependency direction at source level."""

from __future__ import annotations

import os
import json
import re
from pathlib import Path

from tools.lib.repo_paths import repo_root


REPO_ROOT = repo_root()
CPP_ROOT = REPO_ROOT / "src" / "cpp"
INCLUDE_PATTERN = re.compile(r'^\s*#include\s+"([^"]+)"', re.MULTILINE)
SOURCE_SUFFIXES = {".cpp", ".h"}
IGNORED_PARTS = {".esphome", "managed_components"}
OWNERSHIP_MANIFEST = REPO_ROOT / "test" / "cpp" / "coverage_ownership.json"
CPP_SOURCES_CMAKE = CPP_ROOT / "espectre_sources.cmake"
CPP_TEST_CMAKE = REPO_ROOT / "test" / "cpp" / "suites" / "CMakeLists.txt"
CI_WORKFLOW = REPO_ROOT / ".github" / "workflows" / "ci.yml"
CPP_COVERAGE_THRESHOLDS = REPO_ROOT / "test" / "cpp" / "coverage-thresholds.json"
CPP_COVERAGE_RUNNER = REPO_ROOT / "test" / "cpp" / "run_coverage.sh"
FIRMWARE_WIRING_SOURCES = {
    "src/cpp/frontend/native/app/main/app_main.cpp",
    "src/cpp/frontend/matter/app/main/app_main.cpp",
    "src/cpp/frontend/matter/app/main/matter_bindings_esp_matter.cpp",
}


def test_every_cpp_production_source_has_an_explicit_test_owner() -> None:
    cmake_source = CPP_SOURCES_CMAKE.read_text(encoding="utf-8")
    expected = {
        f"src/cpp/{path}"
        for path in re.findall(r'\$\{ESPECTRE_CPP_ROOT\}/([^"\s]+\.cpp)', cmake_source)
    }
    expected.update(FIRMWARE_WIRING_SOURCES)
    manifest = json.loads(OWNERSHIP_MANIFEST.read_text(encoding="utf-8"))
    test_cmake = CPP_TEST_CMAKE.read_text(encoding="utf-8")
    ci_workflow = CI_WORKFLOW.read_text(encoding="utf-8")
    declared: list[str] = []

    assert manifest["version"] == 1
    for group in manifest["owners"]:
        assert group["kind"] in {"host_test", "firmware_build"}
        assert group["sources"] and group["owners"]
        declared.extend(group["sources"])
        if group["kind"] == "host_test":
            for owner in group["owners"]:
                assert re.search(
                    rf"add_espectre_test\({re.escape(owner)}(?:\s|\n)", test_cmake
                ), f"unknown C++ test owner {owner}"
        else:
            assert set(group["sources"]) <= FIRMWARE_WIRING_SOURCES
            for owner in group["owners"]:
                assert re.search(rf"(?m)^  {re.escape(owner)}:$", ci_workflow)

    assert len(declared) == len(set(declared)), "C++ sources must have exactly one owner group"
    assert set(declared) == expected, (
        f"missing owners={sorted(expected - set(declared))}; "
        f"stale owners={sorted(set(declared) - expected)}"
    )


def test_cpp_dataset_and_coverage_contracts_are_complete() -> None:
    from support.chip_matrix import DETECTION_CHIPS

    test_cmake = CPP_TEST_CMAKE.read_text(encoding="utf-8")
    chip_list = re.search(r"set\(shared_chips (?P<chips>[^)]+)\)", test_cmake)
    assert chip_list is not None
    assert set(chip_list.group("chips").split()) == set(DETECTION_CHIPS)
    assert "SKIP_RETURN_CODE 77" in test_cmake
    for gate in ("normal", "reserved", "long", "weak", "empty", "packet_rate"):
        assert "add_espectre_dataset_cases(" in test_cmake
        assert re.search(rf"add_espectre_dataset_cases\([^\n]+ {gate}\)", test_cmake)

    thresholds = json.loads(CPP_COVERAGE_THRESHOLDS.read_text(encoding="utf-8"))
    assert thresholds == {
        "segments": {
            "runtime": {
                "branches": 50.0,
                "functions": 85.0,
                "lines": 80.0,
            }
        },
        "version": 1,
    }

    coverage_runner = CPP_COVERAGE_RUNNER.read_text(encoding="utf-8")
    assert "coverage-thresholds.json" in coverage_runner
    assert "C++ coverage threshold not met" in coverage_runner
    assert "--update-baseline" not in coverage_runner


def is_first_party_source(path: Path) -> bool:
    """Exclude generated build trees and managed dependencies."""
    relative = path.relative_to(CPP_ROOT)
    return not any(
        part in IGNORED_PARTS or part == "build" or part.startswith("build-")
        for part in relative.parts
    )


def first_party_sources() -> list[Path]:
    """Return maintained sources from the three architectural layers."""
    sources: list[Path] = []
    for layer_root in (CPP_ROOT / "core", CPP_ROOT / "runtime", CPP_ROOT / "frontend"):
        for current_root, directories, filenames in os.walk(layer_root):
            directories[:] = [
                directory
                for directory in directories
                if directory not in IGNORED_PARTS
                and directory != "build"
                and not directory.startswith("build-")
            ]
            sources.extend(
                Path(current_root) / filename
                for filename in filenames
                if Path(filename).suffix in SOURCE_SUFFIXES
            )
    return sorted(sources)


def layer(path: Path) -> int:
    """Order layers from lowest-level domain code to concrete frontends."""
    parts = path.relative_to(CPP_ROOT).parts
    if parts[0] == "core":
        return 0
    if parts[0] == "runtime":
        return 1
    if parts[0] == "frontend":
        return 2
    raise AssertionError(f"unclassified C++ source: {path}")


def frontend_name(path: Path) -> str | None:
    """Return the concrete frontend name."""
    parts = path.relative_to(CPP_ROOT).parts
    if len(parts) >= 2 and parts[0] == "frontend":
        return parts[1]
    return None


def test_cpp_dependencies_only_point_to_same_or_lower_layers() -> None:
    """Reject Core -> Runtime/Frontend and Runtime -> Frontend dependencies."""
    sources = first_party_sources()
    headers_by_name: dict[str, list[Path]] = {}
    for header in (path for path in sources if path.suffix == ".h"):
        headers_by_name.setdefault(header.name, []).append(header)

    violations: list[str] = []
    for source in sources:
        for include in INCLUDE_PATTERN.findall(source.read_text(encoding="utf-8")):
            candidates = [source.parent / include, CPP_ROOT / include]
            if "/" not in include:
                candidates.extend(headers_by_name.get(include, []))
            target = next(
                (candidate.resolve() for candidate in candidates if candidate.is_file()),
                None,
            )
            if (
                target is None
                or not target.is_relative_to(CPP_ROOT)
                or not is_first_party_source(target)
            ):
                continue

            source_frontend = frontend_name(source)
            target_frontend = frontend_name(target)
            crosses_frontends = (
                source_frontend is not None
                and target_frontend is not None
                and source_frontend != target_frontend
            )
            if layer(target) > layer(source) or crosses_frontends:
                violations.append(
                    f"{source.relative_to(CPP_ROOT)} includes {target.relative_to(CPP_ROOT)}"
                )

    assert not violations, (
        "C++ dependencies point upward or across frontends:\n" + "\n".join(violations)
    )
