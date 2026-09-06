# Third-Party Notices

This file records third-party material distributed directly by the ESPectre repository. It does not replace the upstream license text. Published GitHub firmware releases group the build-specific SPDX SBOMs, notices, and license archives generated for each image into `firmware-compliance-<channel-or-version>.zip`. The ESPectre website stages the same files next to their corresponding firmware images.

## Firmware frameworks

ESPectre firmware can be built with the following frameworks and components. They are downloaded by their owning build systems rather than copied into the ESPectre SDK source bundle.

| Component | Role | License used by ESPectre | Commercial firmware |
| --- | --- | --- | --- |
| ESP-IDF | ESP32 framework, Wi-Fi, networking, TLS, MQTT, OTA, RTOS integration, and the BLE/NimBLE stack used by Matter | Apache-2.0 with bundled permissive third-party components | Compatible; preserve the build-specific notices |
| ESP-IDF mDNS component | Direct DNS-SD advertisement and peer-assisted discovery in the C++ frontends and SDK Direct group | Apache-2.0 | Compatible; preserve the build-specific notices |
| Improv Wi-Fi SDK for C++ | Native Improv Serial framing and the SDK provisioning group | Apache-2.0 | Compatible; the Native and SDK component manifests pin the exact source revision |
| esp-matter and Matter SDK | Matter frontend | Apache-2.0 | Compatible; preserve `NOTICE`, and do not imply Matter certification or trademark rights |
| ESPHome C++ runtime | ESPHome frontend | GPL-3.0-only | Not included in the commercial track; ESPHome firmware is GPLv3-only |
| ESPHome Python tooling | ESPHome code generation and build tooling | MIT | Build-time only |

The upstream Matter [NOTICE](src/cpp/frontend/matter/third_party/esp_matter/NOTICE) is preserved with the Matter frontend and included in every Matter firmware license archive.

ESP-IDF and esp-matter contain additional permissively licensed components. Each per-build `*-third-party-licenses.zip` contains the license files collected from the linked component set, while the adjacent `*-sbom.spdx.json` records the firmware checksum and package and component inventory.

Firmware audits fail CI and release checks for incompatible license expressions or incomplete scans. Unidentified licenses remain review warnings. CVE findings are uploaded to GitHub Security through SARIF and do not block firmware builds, snapshots, or releases; triage and dismissals are managed in GitHub Code scanning. A failed SARIF upload fails its CI or release audit job. The CI summary contains license findings only. Snapshot publication still requires a successful source CI run.

## Repository-vendored source

| Material | Location | License |
| --- | --- | --- |
| cnpy test support by Carl Rogers | `test/cpp/support/cnpy.cpp` and `test/cpp/support/cnpy.h` | MIT; full text in `test/cpp/support/LICENSE.cnpy` |

`cnpy` is used only by host-side C++ tests and is not linked into production firmware.

## Documentation build tooling

The SDK API generator fetches m.css at the revision pinned in `.github/scripts/generate_sdk_api.py`. m.css is MIT-licensed build-time tooling; its source is not included in SDK bundles or the deployed website. Jinja2 and Pygments are installed from `requirements.txt` to run its templates and syntax highlighting.

## Browser dependencies

The production site stages pinned browser assets and the runtime dependencies included in its bundles. `.github/scripts/stage_web_vendor.py` copies each asset together with its upstream license file.

| Package | Version | License |
| --- | --- | --- |
| esptool-js | 0.6.1 | Apache-2.0 |
| atob-lite | 2.0.0 | MIT |
| pako | 2.2.0 | MIT and Zlib |
| improv-wifi-serial-sdk | 2.8.0 | Apache-2.0 |
| qrcodejs | 1.0.0 | MIT |
| ansi_up | 6.0.6 | MIT |

Rollup and its plugins are build-time tooling and are not deployed. The npm lockfile also records transitive build and browser packages under Apache-2.0, BSD-3-Clause, ISC, MIT, 0BSD, and zlib terms. Production deployment uses the staged bundles and does not publish `node_modules`.

## Host-side Python environment

`requirements.txt` and `requirements-ml.txt` describe a development and operator environment, not libraries linked into the commercial firmware SDK. Notable non-permissive or weak-copyleft packages include:

- `esptool`, GPL-2.0-or-later, used as a separate flashing and image-merging tool;
- `zeroconf`, LGPL-2.1-or-later, used by the host CLI for discovery;
- transitive MPL and LGPL packages installed by ESPHome and analysis tooling; and
- `paho-mqtt`, available under EPL-2.0 or the permissive Eclipse Distribution License 1.0. ESPectre relies on the EDL option where GPL compatibility is required.

Redistributors that package the Python environment must reproduce its installed distributions' license files and satisfy their source, relinking, and notice obligations. ESPectre firmware and SDK releases do not redistribute that Python environment.

## Trademarks, media, and external links

Third-party product names, logos, article thumbnails, video thumbnails, screenshots, and linked media remain the property of their respective owners. ESPectre's GPL or commercial software license does not grant trademark, certification, publicity, or media-reuse rights in that material. In particular, use of Matter trademarks and claims of Matter certification require authorization from the Connectivity Standards Alliance.
