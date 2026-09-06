#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - SDK Package Builder

Build source-first SDK bundles and release metadata for stable and snapshot
channels.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import zipfile
from datetime import datetime, timezone
from pathlib import Path

_SCRIPTS_DIR = str(Path(__file__).resolve().parent)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from detect_git_version import parse_version_core

REPO_ROOT = Path(__file__).resolve().parents[2]
CPP_ROOT = REPO_ROOT / "src" / "cpp"
RUNTIME_PROTOCOL_HEADER = CPP_ROOT / "runtime" / "espectre_protocol.h"
SDK_VERSION_HEADER = CPP_ROOT / "runtime" / "espectre_sdk_version.h"
IDF_COMPONENT_MANIFEST = CPP_ROOT / "idf_component.yml"
SDK_SUPPORTED_ESP_IDF = ">=5.5.0"
OPTIONAL_SOURCE_GROUPS = (
    "ESPECTRE_RUNTIME_FRONTEND_SUPPORT_SOURCES",
    "ESPECTRE_RUNTIME_ESP_IDF_MQTT_SOURCES",
    "ESPECTRE_RUNTIME_ESP_IDF_PROVISIONING_SOURCES",
    "ESPECTRE_RUNTIME_ESP_IDF_OTA_SOURCES",
    "ESPECTRE_RUNTIME_ESP_IDF_DIRECT_SOURCES",
)
SDK_REQUIRED_PATHS = (
    Path("src/cpp/CMakeLists.txt"),
    Path("src/cpp/Kconfig.projbuild"),
    Path("src/cpp/idf_component.yml"),
    Path("src/cpp/espectre_core_sdk.h"),
    Path("src/cpp/espectre_sdk.h"),
    Path("src/cpp/espectre_sources.cmake"),
    Path("src/cpp/espectre_git_version.cmake"),
    Path("src/cpp/core/ml_weights.h"),
    Path("src/cpp/runtime/espectre_sdk_version.h"),
    Path("docs/SDK.md"),
    Path("src/cpp/Doxyfile"),
    Path("src/cpp/runtime/espectre_protocol.h"),
    Path("src/cpp/runtime/esp_idf/runtime_sensing_kconfig.cpp"),
    Path("src/cpp/runtime/esp_idf/espectre_config/CMakeLists.txt"),
    Path("src/cpp/runtime/esp_idf/espectre_config/Kconfig.projbuild"),
    Path("src/cpp/runtime/esp_idf/espectre_config/espectre_config_stub.c"),
)
SDK_ROOTS = (
    Path("src/cpp/core"),
    Path("src/cpp/runtime"),
)
SDK_TOP_LEVEL_FILES = (
    Path("src/cpp/CMakeLists.txt"),
    Path("src/cpp/Kconfig.projbuild"),
    Path("src/cpp/idf_component.yml"),
    Path("src/cpp/espectre_core_sdk.h"),
    Path("src/cpp/espectre_sdk.h"),
    Path("src/cpp/espectre_sources.cmake"),
    Path("src/cpp/espectre_git_version.cmake"),
    Path("src/cpp/Doxyfile"),
    # The integration guide travels with the sources so a bundle is
    # self-contained: `doxygen src/cpp/Doxyfile` from the bundle root rebuilds
    # the API XML offline. Packaging rewrites OUTPUT_DIRECTORY to output because
    # the repo Doxyfile targets docs/web/artifacts/sdk.
    Path("docs/SDK.md"),
    Path("LICENSE"),
    Path("LICENSING.md"),
    Path("THIRD_PARTY_NOTICES.md"),
)

# Repo Doxyfile writes under docs/web/; bundles rewrite to a single-segment path
# Doxygen can create without the website tree.
BUNDLE_DOXYFILE_OUTPUT_DIRECTORY = "output"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build ESPectre SDK bundles and manifests.")
    parser.add_argument(
        "--channel",
        choices=("release", "preview", "develop"),
        required=True,
        help="Release channel for this SDK bundle.",
    )
    parser.add_argument("--version", required=True, help="Human-readable SDK version label.")
    parser.add_argument("--release-tag", required=True, help="GitHub release tag for the published assets.")
    parser.add_argument("--output-dir", required=True, help="Directory where bundle assets are written.")
    parser.add_argument("--commit", help="Optional source commit SHA for preview and develop builds.")
    parser.add_argument(
        "--source-date-epoch",
        type=int,
        help="Reproducible archive timestamp; defaults to SOURCE_DATE_EPOCH or the checkout commit time.",
    )
    parser.add_argument(
        "--url-prefix",
        help="Optional URL prefix used instead of GitHub Releases for artifact URLs.",
    )
    return parser.parse_args()


def detect_protocol_version() -> str:
    match = re.search(
        r'ESPECTRE_PROTOCOL_VERSION\s*=\s*"([^"]+)"',
        RUNTIME_PROTOCOL_HEADER.read_text(encoding="utf-8"),
    )
    if not match:
        raise ValueError("Unable to detect ESPECTRE_PROTOCOL_VERSION")
    return match.group(1)


def detect_sdk_version(header: Path | None = None) -> str:
    """Read the compile-time SDK version from a stamped header."""
    source = (header or SDK_VERSION_HEADER).read_text(encoding="utf-8")
    match = re.search(r'#define\s+ESPECTRE_SDK_VERSION_STRING\s+"([^"]+)"', source)
    if not match:
        raise ValueError("Unable to detect ESPECTRE_SDK_VERSION_STRING")
    version_string = match.group(1)

    components = {}
    for name in ("MAJOR", "MINOR", "PATCH"):
        component = re.search(rf"#define\s+ESPECTRE_SDK_VERSION_{name}\s+(\d+)", source)
        if not component:
            raise ValueError(f"Unable to detect ESPECTRE_SDK_VERSION_{name}")
        components[name] = component.group(1)

    major, minor, patch = parse_version_core(version_string)
    expected = (int(components["MAJOR"]), int(components["MINOR"]), int(components["PATCH"]))
    if expected != (major, minor, patch):
        raise ValueError(
            f"ESPECTRE_SDK_VERSION_STRING is {version_string!r} but the numeric macros say "
            f"{components['MAJOR']}.{components['MINOR']}.{components['PATCH']}"
        )
    return version_string


def idf_component_manifest_version(manifest: Path | None = None) -> str:
    match = re.search(
        r'^version:\s*"?([^"\s]+)"?\s*$',
        (manifest or IDF_COMPONENT_MANIFEST).read_text(encoding="utf-8"),
        re.MULTILINE,
    )
    if not match:
        raise ValueError("Unable to detect the ESP-IDF component manifest version")
    return match.group(1)


def release_asset_stem(channel: str, version: str) -> str:
    if channel == "release":
        return f"espectre-sdk-{version}"
    if channel == "preview":
        return "espectre-sdk-preview"
    return "espectre-sdk-develop"


def collect_bundle_files() -> list[Path]:
    files: list[Path] = []
    for root in SDK_ROOTS:
        for path in sorted((REPO_ROOT / root).rglob("*")):
            if path.is_file():
                files.append(path.relative_to(REPO_ROOT))
    files.extend(SDK_TOP_LEVEL_FILES)
    deduped = sorted(dict.fromkeys(files))
    return deduped


def validate_layout(bundle_files: list[Path]) -> None:
    bundle_file_set = set(bundle_files)
    missing = [str(path) for path in SDK_REQUIRED_PATHS if path not in bundle_file_set]
    if missing:
        raise ValueError(f"SDK bundle is missing required paths: {missing}")


def detect_doxyfile_project_number(path: Path) -> str:
    match = re.search(
        r'(?m)^PROJECT_NUMBER\s*=\s*"?([^"\s]+)"?\s*$',
        path.read_text(encoding="utf-8"),
    )
    if not match:
        raise ValueError(f"Unable to detect PROJECT_NUMBER in {path}")
    return match.group(1)


def stamp_doxyfile_project_number(path: Path, version: str) -> None:
    parse_version_core(version)
    text, count = re.subn(
        r"(?m)^PROJECT_NUMBER\s*=\s*.*$",
        f"PROJECT_NUMBER         = {version}",
        path.read_text(encoding="utf-8"),
        count=1,
    )
    if count != 1:
        raise ValueError(f"Unable to stamp PROJECT_NUMBER in {path}")
    path.write_text(text, encoding="utf-8")


def validate_stamped_sdk_identity(destination_root: Path, version: str) -> None:
    """Require stamped header macros, idf_component.yml, and Doxygen to match the SDK version."""
    header = destination_root / "src" / "cpp" / "runtime" / "espectre_sdk_version.h"
    manifest = destination_root / "src" / "cpp" / "idf_component.yml"
    doxyfile = destination_root / "src" / "cpp" / "Doxyfile"
    stamped = detect_sdk_version(header)
    yml_version = idf_component_manifest_version(manifest)
    project_number = detect_doxyfile_project_number(doxyfile)
    mismatched = {
        path: value
        for path, value in (
            (str(header.relative_to(destination_root)), stamped),
            (str(manifest.relative_to(destination_root)), yml_version),
            (str(doxyfile.relative_to(destination_root)), project_number),
        )
        if value != version
    }
    if mismatched:
        raise ValueError(
            f"Stamped SDK identity is {version!r} but packaging metadata disagrees: {mismatched}"
        )


def stamp_sdk_version_header(path: Path, version: str) -> None:
    major, minor, patch = parse_version_core(version)
    source = path.read_text(encoding="utf-8")
    stamped = (
        "/* ESPECTRE_SDK_VERSION_VALUES_BEGIN */\n"
        f"#define ESPECTRE_SDK_VERSION_MAJOR {major}\n"
        f"#define ESPECTRE_SDK_VERSION_MINOR {minor}\n"
        f"#define ESPECTRE_SDK_VERSION_PATCH {patch}\n"
        f'#define ESPECTRE_SDK_VERSION_STRING "{version}"\n'
        "/* ESPECTRE_SDK_VERSION_VALUES_END */"
    )
    source, count = re.subn(
        r"/\* ESPECTRE_SDK_VERSION_VALUES_BEGIN \*/.*?/\* ESPECTRE_SDK_VERSION_VALUES_END \*/",
        stamped,
        source,
        count=1,
        flags=re.DOTALL,
    )
    if count != 1:
        raise ValueError(f"Unable to stamp SDK version values in {path}")
    path.write_text(source, encoding="utf-8")


def stamp_idf_component_manifest(path: Path, version: str) -> None:
    lines = path.read_text(encoding="utf-8").splitlines()
    replaced = False
    output_lines: list[str] = []
    for line in lines:
        if line.startswith("version: "):
            output_lines.append(f'version: "{version}"')
            replaced = True
        else:
            output_lines.append(line)
    if not replaced:
        output_lines.insert(0, f'version: "{version}"')
    path.write_text("\n".join(output_lines) + "\n", encoding="utf-8")


def rewrite_bundle_doxyfile(path: Path, version: str) -> None:
    """Point the bundled Doxyfile at output and stamp the bundle identity."""
    text = path.read_text(encoding="utf-8")
    if not re.search(r"(?m)^OUTPUT_DIRECTORY\s*=", text):
        raise ValueError(f"Unable to rewrite OUTPUT_DIRECTORY in {path}")

    # Replace the repository usage/output preamble with bundle-oriented guidance.
    text, preamble_count = re.subn(
        r"# Usage, from the repository root.*?(?=\nPROJECT_NAME)",
        (
            "# Usage, from the unpacked SDK bundle root:\n"
            "#   doxygen src/cpp/Doxyfile\n"
            "#\n"
            "# Doxygen XML is written to output/xml/. Packaging rewrote OUTPUT_DIRECTORY\n"
            "# away from the repository website path so this works without docs/web/.\n"
        ),
        text,
        count=1,
        flags=re.DOTALL,
    )
    if preamble_count != 1:
        raise ValueError(f"Unable to rewrite Doxyfile usage preamble in {path}")

    text, output_count = re.subn(
        r"(?m)^OUTPUT_DIRECTORY\s*=\s*.*$",
        f"OUTPUT_DIRECTORY       = {BUNDLE_DOXYFILE_OUTPUT_DIRECTORY}",
        text,
        count=1,
    )
    if output_count != 1:
        raise ValueError(f"Unable to rewrite OUTPUT_DIRECTORY in {path}")

    # Replace repository-only output guidance with the bundle location.
    text, mkdir_count = re.subn(
        r"(?m)^# The repository generator replaces this with an isolated temporary directory\.\n"
        r"# Direct Doxygen runs write ignored XML under the website artifact tree\.\n",
        (
            "# The bundled configuration writes tool-neutral XML below output/.\n"
        ),
        text,
        count=1,
    )
    if mkdir_count != 1:
        raise ValueError(f"Unable to rewrite Doxyfile OUTPUT_DIRECTORY comments in {path}")

    path.write_text(text, encoding="utf-8")
    stamp_doxyfile_project_number(path, version)


def rewrite_bundle_sdk_guide(path: Path, source_ref: str) -> None:
    """Point repository-relative Markdown links at the exact packaged revision."""
    source = path.read_text(encoding="utf-8")

    def replace_link(match: re.Match[str]) -> str:
        target = match.group(1)
        anchor = match.group(2) or ""
        normalized = os.path.normpath(os.path.join("docs", target)).replace(os.sep, "/")
        if normalized == ".." or normalized.startswith("../"):
            raise ValueError(f"Bundled SDK guide link escapes the repository: {target}")
        return (
            "](https://github.com/francescopace/espectre/blob/"
            f"{source_ref}/{normalized}{anchor})"
        )

    rewritten, count = re.subn(
        r"\]\((?!https?://)(?!mailto:)([^)#]+\.md)(#[^)]+)?\)",
        replace_link,
        source,
    )
    if count == 0:
        raise ValueError(f"Bundled SDK guide has no relative Markdown links to rewrite: {path}")
    path.write_text(rewritten, encoding="utf-8")


def rewrite_bundle_sdk_facade(path: Path, source_ref: str) -> None:
    """Pin the generated reference's SDK guide link to the packaged revision."""
    source = path.read_text(encoding="utf-8")
    current = "https://github.com/francescopace/espectre/blob/main/docs/SDK.md"
    replacement = f"https://github.com/francescopace/espectre/blob/{source_ref}/docs/SDK.md"
    if source.count(current) != 1:
        raise ValueError(f"Unable to rewrite SDK guide link in {path}")
    path.write_text(source.replace(current, replacement), encoding="utf-8")


def stage_bundle_tree(destination_root: Path, version: str, source_ref: str,
                      bundle_files: list[Path]) -> int:
    for relative_path in bundle_files:
        source = REPO_ROOT / relative_path
        target = destination_root / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    stamp_idf_component_manifest(destination_root / "src" / "cpp" / "idf_component.yml", version)
    stamp_sdk_version_header(
        destination_root / "src" / "cpp" / "runtime" / "espectre_sdk_version.h",
        version,
    )
    rewrite_bundle_doxyfile(destination_root / "src" / "cpp" / "Doxyfile", version)
    rewrite_bundle_sdk_guide(destination_root / "docs" / "SDK.md", source_ref)
    rewrite_bundle_sdk_facade(destination_root / "src" / "cpp" / "espectre_sdk.h", source_ref)
    validate_stamped_sdk_identity(destination_root, version)
    return len(bundle_files)


def resolve_source_date_epoch(explicit_epoch: int | None = None) -> int:
    if explicit_epoch is not None:
        epoch = explicit_epoch
    elif os.environ.get("SOURCE_DATE_EPOCH"):
        epoch = int(os.environ["SOURCE_DATE_EPOCH"])
    else:
        result = subprocess.run(
            ["git", "show", "-s", "--format=%ct", "HEAD"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        epoch = int(result.stdout.strip())
    if epoch < 0:
        raise ValueError("SOURCE_DATE_EPOCH must not be negative")
    return epoch


def normalized_mode(path: Path) -> int:
    if path.is_dir() or path.stat().st_mode & stat.S_IXUSR:
        return 0o755
    return 0o644


def normalize_tar_info(info: tarfile.TarInfo, path: Path, epoch: int) -> tarfile.TarInfo:
    info.uid = 0
    info.gid = 0
    info.uname = ""
    info.gname = ""
    info.mtime = epoch
    info.mode = normalized_mode(path)
    return info


def write_tarball(source_dir: Path, output_path: Path, root_dir_name: str, epoch: int) -> None:
    paths = [source_dir, *sorted(source_dir.rglob("*"))]
    with output_path.open("wb") as output_file:
        with gzip.GzipFile(filename="", mode="wb", fileobj=output_file, mtime=epoch) as compressed:
            with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as archive:
                for path in paths:
                    relative = path.relative_to(source_dir) if path != source_dir else Path()
                    arcname = Path(root_dir_name) / relative
                    info = normalize_tar_info(archive.gettarinfo(str(path), str(arcname)), path, epoch)
                    if info.isfile():
                        with path.open("rb") as source_file:
                            archive.addfile(info, source_file)
                    else:
                        archive.addfile(info)


def write_zipfile(source_dir: Path, output_path: Path, root_dir_name: str, epoch: int) -> None:
    zip_epoch = max(epoch, 315532800)
    timestamp = datetime.fromtimestamp(zip_epoch, timezone.utc)
    date_time = (timestamp.year, timestamp.month, timestamp.day, timestamp.hour, timestamp.minute, timestamp.second)
    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(source_dir.rglob("*")):
            if not path.is_file():
                continue
            relative = path.relative_to(source_dir)
            info = zipfile.ZipInfo(str(Path(root_dir_name) / relative), date_time=date_time)
            info.create_system = 3
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = normalized_mode(path) << 16
            archive.writestr(info, path.read_bytes(), compress_type=zipfile.ZIP_DEFLATED, compresslevel=9)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_artifact_url(filename: str, release_tag: str, url_prefix: str | None) -> str:
    if url_prefix:
        return f"{url_prefix.rstrip('/')}/{filename}"
    return f"https://github.com/francescopace/espectre/releases/download/{release_tag}/{filename}"


def build_manifest(
    *,
    channel: str,
    version: str,
    release_tag: str,
    commit: str | None,
    tarball_name: str,
    zip_name: str,
    bundle_file_count: int,
    bundle_root: str,
    url_prefix: str | None,
    generated_at: str,
    tarball_sha256: str,
    zip_sha256: str,
) -> dict:
    return {
        "schema_version": 2,
        "artifact_kind": "sdk",
        "channel": channel,
        "version": version,
        "release_tag": release_tag,
        "generated_at": generated_at,
        "commit": commit,
        "protocol_version": detect_protocol_version(),
        "supported_esp_idf": SDK_SUPPORTED_ESP_IDF,
        "bundle": {
            "root_dir": bundle_root,
            "file_count": bundle_file_count,
            "required_paths": [str(path) for path in SDK_REQUIRED_PATHS],
            "source_roots": [str(path) for path in SDK_ROOTS],
            "top_level_files": [str(path) for path in SDK_TOP_LEVEL_FILES],
        },
        "artifacts": [
            {
                "format": "tar.gz",
                "filename": tarball_name,
                "url": build_artifact_url(tarball_name, release_tag, url_prefix),
                "sha256": tarball_sha256,
            },
            {
                "format": "zip",
                "filename": zip_name,
                "url": build_artifact_url(zip_name, release_tag, url_prefix),
                "sha256": zip_sha256,
            },
        ],
        "install_surfaces": {
            "cmake": {
                "entrypoint": "src/cpp/espectre_sources.cmake",
                "optional_source_groups": list(OPTIONAL_SOURCE_GROUPS),
            },
            "esp_idf_component": {
                "component_root": "src/cpp",
                "cmake": "src/cpp/CMakeLists.txt",
                "manifest": "src/cpp/idf_component.yml",
                "kconfig": "src/cpp/Kconfig.projbuild",
            },
        },
    }


def build_sdk_package(args: argparse.Namespace) -> dict:
    if args.channel == "release" and args.version != args.release_tag:
        raise ValueError(
            "Release SDK version and release tag must match: "
            f"{args.version!r} != {args.release_tag!r}"
        )

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    bundle_files = collect_bundle_files()
    validate_layout(bundle_files)

    parse_version_core(args.version)
    asset_stem = release_asset_stem(args.channel, args.version)
    bundle_root = asset_stem
    tarball_name = f"{asset_stem}.tar.gz"
    zip_name = f"{asset_stem}.zip"
    manifest_suffix = args.release_tag if args.channel == "release" else args.channel
    manifest_name = f"sdk-manifest-{manifest_suffix}.json"
    source_date_epoch = resolve_source_date_epoch(getattr(args, "source_date_epoch", None))
    generated_at = datetime.fromtimestamp(source_date_epoch, timezone.utc).isoformat()
    tarball_path = output_dir / tarball_name
    zip_path = output_dir / zip_name

    with tempfile.TemporaryDirectory(prefix="espectre-sdk-") as tmp_dir:
        staged_root = Path(tmp_dir) / bundle_root
        source_ref = args.commit or args.release_tag
        file_count = stage_bundle_tree(staged_root, args.version, source_ref, bundle_files)
        write_tarball(staged_root, tarball_path, bundle_root, source_date_epoch)
        write_zipfile(staged_root, zip_path, bundle_root, source_date_epoch)

    manifest = build_manifest(
        channel=args.channel,
        version=args.version,
        release_tag=args.release_tag,
        commit=args.commit,
        tarball_name=tarball_name,
        zip_name=zip_name,
        bundle_file_count=file_count,
        bundle_root=bundle_root,
        url_prefix=args.url_prefix,
        generated_at=generated_at,
        tarball_sha256=sha256_file(tarball_path),
        zip_sha256=sha256_file(zip_path),
    )
    (output_dir / manifest_name).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    build_sdk_package(parse_args())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
