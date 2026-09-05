#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Stage pinned browser dependencies into the generated website tree."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


WEB_ROOT = Path(__file__).resolve().parents[2] / "docs" / "web"
NODE_MODULES = WEB_ROOT / "node_modules"
VENDOR_ROOT = WEB_ROOT / "vendor"


def require(path: Path) -> Path:
    if not path.exists():
        raise FileNotFoundError(f"Missing npm dependency asset: {path}")
    return path


def copy_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(require(source), destination)


def stage_vendor() -> None:
    if VENDOR_ROOT.exists():
        shutil.rmtree(VENDOR_ROOT)

    subprocess.run(
        ["npm", "run", "build:headless"],
        cwd=WEB_ROOT,
        check=True,
    )
    serial_destination = VENDOR_ROOT / "espectre-web-serial-0.6.1-2.8.1"
    copy_file(WEB_ROOT / "build" / "headless-web-serial.js", serial_destination / "headless.js")
    copy_file(NODE_MODULES / "esptool-js" / "LICENSE", serial_destination / "LICENSE.esptool-js")
    copy_file(
        NODE_MODULES / "atob-lite" / "LICENSE.md",
        serial_destination / "LICENSE.atob-lite",
    )
    copy_file(
        NODE_MODULES / "pako" / "LICENSE",
        serial_destination / "LICENSE.pako",
    )
    copy_file(
        NODE_MODULES / "improv-wifi-serial-sdk" / "LICENSE",
        serial_destination / "LICENSE.improv-wifi-serial-sdk",
    )

    qrcode_destination = VENDOR_ROOT / "qrcodejs-1.0.0"
    copy_file(NODE_MODULES / "qrcodejs" / "qrcode.min.js", qrcode_destination / "qrcode.min.js")
    copy_file(NODE_MODULES / "qrcodejs" / "LICENSE", qrcode_destination / "LICENSE")

    ansi_destination = VENDOR_ROOT / "ansi_up-6.0.6"
    copy_file(NODE_MODULES / "ansi_up" / "ansi_up.js", ansi_destination / "ansi_up.js")
    copy_file(NODE_MODULES / "ansi_up" / "LICENSE", ansi_destination / "LICENSE")


def main() -> int:
    stage_vendor()
    print(f"staged browser dependencies under {VENDOR_ROOT.relative_to(WEB_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
