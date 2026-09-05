#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Verify the complete generated GitHub Pages tree before upload."""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import urlparse

_SCRIPTS_DIR = str(Path(__file__).resolve().parent)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from detect_git_version import detect_git_version, parse_version_core
from web_html_security import validate_passive_api_fragment
from web_routes import SITEMAP_NAMESPACE, load_manifest, staged_sdk_channels


REPO_ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = REPO_ROOT / "docs" / "web"
EXPECTED_FRONTENDS = {"esphome", "matter", "native"}
EXPECTED_CHIPS_BY_FRONTEND = {
    "esphome": {"esp32", "esp32s2", "esp32s3", "esp32c3", "esp32c5", "esp32c6"},
    "matter": {"esp32", "esp32s3", "esp32c3", "esp32c5", "esp32c6"},
    "native": {"esp32", "esp32s2", "esp32s3", "esp32c3", "esp32c5", "esp32c6"},
}
ROUTE_MANIFEST = load_manifest()
SITE_HOST = urlparse(ROUTE_MANIFEST["siteOrigin"]).hostname or ""
SDK_CHANNEL_PATHS = {
    sdk_channel["sdkChannel"]: sdk_channel["path"]
    for sdk_channel in ROUTE_MANIFEST["sdkChannels"]
}
SPA_PAGE_ROUTE_RE = re.compile(r'<main\b[^>]*\bdata-page="([^"]+)"')


def expected_sitemap_paths() -> set[str]:
    return {
        *(route["staticPath"] for route in ROUTE_MANIFEST["routes"]),
        *(sdk_channel["path"] for sdk_channel in staged_sdk_channels(WEB_ROOT, ROUTE_MANIFEST)),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Verify the generated ESPectre Pages tree.")
    parser.add_argument("--require-preview", action="store_true")
    parser.add_argument("--require-release", action="store_true")
    parser.add_argument("--require-develop", action="store_true")
    return parser.parse_args()


def require_file(relative_path: str) -> Path:
    path = (WEB_ROOT / relative_path).resolve()
    try:
        path.relative_to(WEB_ROOT.resolve())
    except ValueError as error:
        raise ValueError(f"Website path escapes the generated tree: {relative_path}") from error
    if not path.is_file():
        raise FileNotFoundError(f"Missing generated website file: {relative_path}")
    return path


def registered_spa_routes() -> list[str]:
    return [route["name"] for route in ROUTE_MANIFEST["routes"]]


class StaticPageMetadataParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.in_title = False
        self.title_parts: list[str] = []
        self.canonical = ""
        self.meta: dict[str, str] = {}
        self.static_page = False
        self.main_content = False

    @property
    def title(self) -> str:
        return "".join(self.title_parts)

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        values = dict(attrs)
        if tag == "html":
            self.static_page = "data-static-page" in values
        elif tag == "title":
            self.in_title = True
        elif tag == "link" and values.get("rel") == "canonical":
            self.canonical = values.get("href") or ""
        elif tag == "meta":
            key = values.get("name") or values.get("property")
            if key:
                self.meta[key] = values.get("content") or ""
        elif tag == "main" and values.get("id") == "main-content":
            self.main_content = True

    def handle_endtag(self, tag: str) -> None:
        if tag == "title":
            self.in_title = False

    def handle_data(self, data: str) -> None:
        if self.in_title:
            self.title_parts.append(data)


def verify_spa_routes() -> None:
    index = require_file("index.html").read_text(encoding="utf-8")
    expected = registered_spa_routes()
    found = SPA_PAGE_ROUTE_RE.findall(index)
    missing = sorted(set(expected) - set(found))
    unexpected = sorted(set(found) - set(expected))
    if missing or unexpected or sorted(expected) != sorted(found):
        raise ValueError(
            "Invalid SPA route inventory: "
            f"missing={missing}, unexpected={unexpected}"
        )


def verify_generated_pages() -> None:
    routes = [route for route in ROUTE_MANIFEST["routes"] if route["staticPath"] != "/"]
    pages = []
    for route in routes:
        static_path = route["staticPath"]
        relative_path = static_path.strip("/")
        pages.append((route, require_file(f"{relative_path}/index.html")))

    for route, page in pages:
        static_path = route["staticPath"]
        parser = StaticPageMetadataParser()
        parser.feed(page.read_text(encoding="utf-8"))
        canonical = f'{ROUTE_MANIFEST["siteOrigin"]}{static_path}'
        expected_meta = {
            "description": route["description"],
            "og:url": canonical,
            "og:title": route["title"],
            "og:description": route["description"],
            "twitter:title": route["title"],
            "twitter:description": route["description"],
        }
        if not parser.static_page or not parser.main_content:
            raise ValueError(f"Generated page has no static shell: {static_path}")
        if parser.title != route["title"] or parser.canonical != canonical:
            raise ValueError(
                f"Generated page metadata mismatch for {static_path}: "
                f"title={parser.title!r}, canonical={parser.canonical!r}"
            )
        mismatched_meta = {
            key: parser.meta.get(key)
            for key, expected in expected_meta.items()
            if parser.meta.get(key) != expected
        }
        if mismatched_meta:
            raise ValueError(
                f"Generated page metadata mismatch for {static_path}: {mismatched_meta}"
            )


def verify_sitemap(*, require_preview: bool, require_release: bool, require_develop: bool) -> None:
    sitemap_path = require_file("sitemap.xml")
    root = ET.parse(sitemap_path).getroot()
    expected_root = f"{{{SITEMAP_NAMESPACE}}}urlset"
    if root.tag != expected_root:
        raise ValueError(f"Unexpected sitemap root: {root.tag}")

    paths: set[str] = set()
    today = datetime.now(timezone.utc).date()
    for entry in root.findall(f"{{{SITEMAP_NAMESPACE}}}url"):
        location = entry.find(f"{{{SITEMAP_NAMESPACE}}}loc")
        if location is None or not (location.text or "").strip():
            raise ValueError("Sitemap entry has no loc")
        url = (location.text or "").strip()
        parsed = urlparse(url)
        if parsed.scheme != "https" or parsed.hostname != SITE_HOST:
            raise ValueError(f"Sitemap URL must use https://{SITE_HOST}: {url}")
        if parsed.path in paths:
            raise ValueError(f"Duplicate sitemap path: {parsed.path}")
        paths.add(parsed.path)

        if entry.find(f"{{{SITEMAP_NAMESPACE}}}changefreq") is not None:
            raise ValueError(f"Sitemap must not contain changefreq: {url}")
        lastmod = entry.find(f"{{{SITEMAP_NAMESPACE}}}lastmod")
        if lastmod is None or not (lastmod.text or "").strip():
            raise ValueError(f"Sitemap entry is missing lastmod: {parsed.path}")
        value = (lastmod.text or "").strip()
        try:
            parsed_date = datetime.strptime(value, "%Y-%m-%d").date()
        except ValueError as error:
            raise ValueError(f"Invalid sitemap lastmod for {url}: {value!r}") from error
        if parsed_date > today:
            raise ValueError(f"Sitemap lastmod is in the future for {url}: {value}")

    expected_paths = expected_sitemap_paths()
    if paths != expected_paths:
        raise ValueError(
            "Invalid sitemap URL inventory: "
            f"missing={sorted(expected_paths - paths)}, "
            f"unexpected={sorted(paths - expected_paths)}"
        )
    required_channels = {
        channel
        for channel, required in (
            ("preview", require_preview),
            ("release", require_release),
            ("develop", require_develop),
        )
        if required
    }
    missing_required = sorted(
        SDK_CHANNEL_PATHS[channel]
        for channel in required_channels
        if SDK_CHANNEL_PATHS[channel] not in paths
    )
    if missing_required:
        raise ValueError(f"Sitemap is missing required SDK channels: {missing_required}")


def verify_channel_version(manifest: dict) -> None:
    if parse_version_core(manifest.get("version", ""))[0] < 3:
        raise ValueError("Website channels require ESPectre version 3 or newer")


def verify_firmware_channel(channel: str) -> None:
    channel_dir = WEB_ROOT / "artifacts" / "firmware" / channel
    manifest_path = require_file(f"artifacts/firmware/{channel}/firmware-manifest-{channel}.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    verify_channel_version(manifest)
    if manifest.get("channel") != channel:
        raise ValueError(
            f"Firmware manifest channel mismatch: expected {channel!r}, "
            f"found {manifest.get('channel')!r}"
        )
    if channel == "release" and manifest.get("version") != manifest.get("release_tag"):
        raise ValueError("Release firmware version and release tag do not match")

    frontends = manifest.get("frontends", {})
    if set(frontends) != EXPECTED_FRONTENDS:
        raise ValueError(
            f"Invalid {channel} website frontends: "
            f"expected={sorted(EXPECTED_FRONTENDS)}, found={sorted(frontends)}"
        )

    seen = set()
    artifact_count = 0
    for frontend, metadata in frontends.items():
        for artifact in metadata.get("artifacts", []):
            artifact_count += 1
            if artifact.get("build_type") != "factory":
                raise ValueError(f"Website manifest contains non-factory firmware: {artifact}")
            key = (frontend, artifact.get("chip"))
            if key in seen:
                raise ValueError(f"Website manifest contains duplicate firmware: {key}")
            seen.add(key)
            filename = artifact.get("filename", "")
            if not filename or Path(filename).name != filename:
                raise ValueError(f"Invalid firmware artifact filename: {filename!r}")
            require_file(f"artifacts/firmware/{channel}/{filename}")

    expected = {
        (frontend, chip)
        for frontend, chips in EXPECTED_CHIPS_BY_FRONTEND.items()
        for chip in chips
    }
    if seen != expected:
        raise ValueError(
            f"Invalid {channel} website firmware matrix: "
            f"missing={sorted(expected - seen)}, unexpected={sorted(seen - expected)}"
        )
    binaries = sorted(channel_dir.glob("*.bin"))
    if artifact_count != len(expected) or len(binaries) != len(expected):
        raise ValueError(
            f"Expected {len(expected)} {channel} firmware artifacts and images, "
            f"found {artifact_count} manifest entries and {len(binaries)} images"
        )


def verify_sdk_api_version() -> None:
    version = detect_git_version()
    manifest = json.loads(require_file("artifacts/sdk/api/api-index.json").read_text(encoding="utf-8"))
    if manifest.get("sdk_version") != version:
        raise ValueError(f"Generated SDK API reference does not show version {version!r}")
    if manifest.get("renderer") != "m.css" or not manifest.get("renderer_revision"):
        raise ValueError("Generated SDK API reference has no pinned m.css renderer identity")
    entries = manifest.get("entries", [])
    if not entries or any(not isinstance(entry.get("discoverable"), bool) for entry in entries):
        raise ValueError("Generated SDK API reference has no picker discoverability metadata")
    required = {
        "classespectre_1_1_runtime_frontend_controller",
        "structespectre_1_1_runtime_config",
        "classespectre_1_1_i_runtime_listener",
    }
    available = {entry.get("refid") for entry in entries}
    if not required <= available:
        raise ValueError(f"Generated SDK API reference is missing public types: {sorted(required - available)}")
    discoverable = {entry.get("refid") for entry in entries if entry["discoverable"]}
    if not required <= discoverable:
        raise ValueError(f"Generated SDK API picker is missing public types: {sorted(required - discoverable)}")
    for entry in entries:
        fragment = require_file(f"artifacts/sdk/api/{entry.get('fragment', '')}")
        source = fragment.read_text(encoding="utf-8")
        validate_passive_api_fragment(source)
        if "<html" in source.lower() or "<iframe" in source.lower():
            raise ValueError(f"Generated SDK API fragment is not portal-native: {fragment}")
        if re.search(r'<nav\b[^>]*class="[^"]*\bm-block\b', source):
            raise ValueError(f"Generated SDK API fragment still contains duplicate local navigation: {fragment}")


def verify_sdk_channel(channel: str) -> None:
    require_file(f"artifacts/sdk/{channel}/index.html")
    manifest_path = require_file(f"artifacts/sdk/{channel}/sdk-manifest-{channel}.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    verify_channel_version(manifest)
    if manifest.get("channel") != channel:
        raise ValueError(
            f"SDK manifest channel mismatch: expected {channel!r}, found {manifest.get('channel')!r}"
        )
    if manifest.get("schema_version") != 2:
        raise ValueError(f"Invalid {channel} SDK manifest schema")
    redundant = {"package_version", "sdk_version"}.intersection(manifest)
    if redundant:
        raise ValueError(f"Redundant {channel} SDK version fields: {sorted(redundant)}")
    if channel == "release" and manifest.get("version") != manifest.get("release_tag"):
        raise ValueError("Release SDK version and release tag do not match")
    artifacts = manifest.get("artifacts", [])
    if {artifact.get("format") for artifact in artifacts} != {"tar.gz", "zip"}:
        raise ValueError(f"Invalid {channel} SDK artifact formats")
    for artifact in artifacts:
        if not re.fullmatch(r"[0-9a-f]{64}", artifact.get("sha256", "")):
            raise ValueError(f"Invalid SDK SHA-256 metadata: {artifact}")


def verify(args: argparse.Namespace) -> None:
    for path in (
        "sitemap.xml",
        "index.html",
        "404.html",
        "routes.json",
        "assets/js/app.js",
        "assets/js/flash-tool.js",
        "assets/js/route-registry.js",
        "assets/js/espectre-direct.js",
        "assets/css/styles.css",
        "vendor/qrcodejs-1.0.0/qrcode.min.js",
        "vendor/qrcodejs-1.0.0/LICENSE",
        "vendor/ansi_up-6.0.6/ansi_up.js",
        "vendor/ansi_up-6.0.6/LICENSE",
        "vendor/espectre-web-serial-0.6.1-2.8.1/headless.js",
        "vendor/espectre-web-serial-0.6.1-2.8.1/LICENSE.esptool-js",
        "vendor/espectre-web-serial-0.6.1-2.8.1/LICENSE.atob-lite",
        "vendor/espectre-web-serial-0.6.1-2.8.1/LICENSE.improv-wifi-serial-sdk",
        "vendor/espectre-web-serial-0.6.1-2.8.1/LICENSE.pako",
        "artifacts/sdk/api/api-index.json",
    ):
        require_file(path)
    verify_spa_routes()
    verify_generated_pages()
    verify_sdk_api_version()
    verify_sitemap(
        require_preview=args.require_preview,
        require_release=args.require_release,
        require_develop=args.require_develop,
    )
    if args.require_preview:
        verify_firmware_channel("preview")
        verify_sdk_channel("preview")
    if args.require_release:
        verify_firmware_channel("release")
        verify_sdk_channel("release")
    if args.require_develop:
        verify_firmware_channel("develop")
        verify_sdk_channel("develop")


def main() -> int:
    verify(parse_args())
    print("Website build verified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
