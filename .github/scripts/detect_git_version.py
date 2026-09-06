#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Git Describe Version

Resolve the shared firmware/SDK identity from numeric git tags. Rolling GitHub
tags such as `snapshot` and `snapshot-dev` are ignored so they cannot become the
version string.

Keep the git arguments in sync with `src/cpp/espectre_git_version.cmake`. CMake also
accepts `-DESPECTRE_GIT_VERSION` or the `ESPECTRE_GIT_VERSION` environment variable
for first-party firmware builds when the checkout has no numeric tags.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
GIT_DESCRIBE_CMD = ("git", "describe", "--tags", "--match", "[0-9]*", "--abbrev=7")
CORE_PATTERN = re.compile(
    r"^(?P<major>0|[1-9]\d*)\.(?P<minor>0|[1-9]\d*)\.(?P<patch>0|[1-9]\d*)"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print the ESPectre git-describe version.")
    parser.add_argument(
        "--from-sdk-dir",
        help="Read `version` from the unique sdk-manifest-*.json in this directory.",
    )
    parser.add_argument(
        "--exact-match",
        action="store_true",
        help="Require HEAD to be an exact numeric tag.",
    )
    return parser.parse_args()


def parse_version_core(version: str) -> tuple[int, int, int]:
    match = CORE_PATTERN.match(version)
    if not match:
        raise ValueError(f"Version {version!r} does not start with numeric MAJOR.MINOR.PATCH")
    return int(match.group("major")), int(match.group("minor")), int(match.group("patch"))


def detect_git_version(
    repo_root: Path | None = None,
    *,
    exact_match: bool = False,
) -> str:
    command = list(GIT_DESCRIBE_CMD)
    if exact_match:
        command.append("--exact-match")
    result = subprocess.run(
        command,
        cwd=repo_root or REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or f"exit {result.returncode}"
        raise ValueError(f"git describe failed: {detail}")
    version = result.stdout.strip()
    parse_version_core(version)
    return version


def version_from_sdk_dir(sdk_dir: Path) -> str:
    matches = sorted(sdk_dir.glob("sdk-manifest-*.json"))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one SDK manifest in {sdk_dir}, found {len(matches)}")
    payload = json.loads(matches[0].read_text(encoding="utf-8"))
    version = payload.get("version")
    if not isinstance(version, str) or not version:
        raise ValueError(f"SDK manifest {matches[0].name} is missing version")
    return version


def main() -> int:
    args = parse_args()
    if args.from_sdk_dir:
        print(version_from_sdk_dir(Path(args.from_sdk_dir)))
    else:
        print(detect_git_version(exact_match=args.exact_match))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
