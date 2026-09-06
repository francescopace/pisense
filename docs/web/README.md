# Website

The website is published at `espectre.dev` through GitHub Pages. Generated canonical pages provide indexable content, while a persistent SPA shell handles browser tools and in-app navigation.

## Local preview

From the repository root, run:

```bash
python -m http.server 8090 --directory docs/web
```

Open `http://localhost:8090`. Native development firmware accepts loopback Origins only when `CONFIG_ESPECTRE_DIRECT_DEV_ORIGINS_ENABLED=y`; published firmware keeps this exception disabled. Flash, Improv Serial, and the Matter QR reader require a Chromium-based browser.

The hosted Direct workflow is validated with Chrome 151 or later on macOS. Physical coverage on Windows and native Linux is still pending, and local discovery depends on the operating system's mDNS support. Other browsers are not guaranteed to work. A local HTTP preview does not prove hosted compatibility.

## Sources and generated pages

Edit shared page fragments under `content/`, styles under `assets/css/`, images under `assets/images/`, and first-party scripts under `assets/js/`. Do not edit generated route `index.html` files. In `routes.json`, `routes` owns public pages, metadata, canonical paths, navigation groups, and Analytics names, `contentGroups` owns their Analytics grouping, and `sdkChannels` owns the generated Release, Preview, and Develop SDK artifact pages. Public routes always contribute to the sitemap; SDK channels contribute only when both their manifest and generated page are staged. `assets/js/route-registry.js` loads the manifest directly in hosted and local previews.

Generate the standalone pages before testing direct route URLs:

```bash
python3 .github/scripts/build_static_pages.py
```

The generator adds route-specific titles, descriptions, canonical URLs, Open Graph metadata, and Twitter metadata from `routes.json`. The SPA loads the same manifest and fragments from `/content/`, and updates its runtime metadata when the active route changes.

First-party CSS, JavaScript, and brand assets referenced by committed entry pages use a 12-character SHA-256 prefix in `?v=`. The route manifest and SPA content fragments use HTTP revalidation so they remain directly loadable in local previews without generating assets. Restamp committed entry pages after changing hashed assets:

```bash
python3 .github/scripts/web_asset_versions.py
```

Website tests reject stale hashes. Generated static and SDK pages compute their asset hashes at build time.

## Browser dependencies

The browser installer uses a same-origin ESM bundle built from pinned `esptool-js` 0.6.1 and `improv-wifi-serial-sdk` 2.8.1 dependencies. The bundle adds ESPectre's `GET_MATTER_ONBOARDING` (`0x80`) RPC, QRCode.js 1.0.0 renders the returned Matter setup code, and ansi_up 6.0.6 renders ANSI styling in serial logs. Install and stage the dependencies locally with:

```bash
npm --prefix docs/web ci --ignore-scripts
npm --prefix docs/web run stage:vendor
```

`package-lock.json` owns the versions. `stage:vendor` builds the headless Web Serial bundle and copies it with the QR and ANSI renderers and upstream licenses. CI stages the same files, while `build/`, `vendor/`, and `node_modules/` remain ignored. There is no remote fallback: a local preview must run both commands before the installer can connect to a board.

The local Improv wrapper propagates initialization failures and cancels pending state requests before releasing the serial reader. A device that does not answer within the probe timeout proceeds through the USB detection fallbacks without leaving a background RPC timeout. State requests are retried once per second during the probe to allow firmware to finish booting.

During USB selection, identification, and restart, the Connect button replaces its normal label with a spinner, the current phase, and elapsed time. Progress distinguishes port opening and closing, Improv requests, startup logs, bootloader attempts, chip information, flash-reader startup, partition-table reads, and each firmware partition. Its normal label returns when the transition finishes or the USB flow is reset.

The installer's Device settings link appears only when the firmware supplies a device address in the Improv URL's `target` parameter. The link forwards that parameter to `/tools/device-settings/`.

USB identification keeps progress in the Connect button without emitting per-step, per-block, periodic, or descriptor debug logs. Transfer failures retain their phase and I/O stage on the error reported by the installer. The reader consumes the stub's trailing digest frame before the next command, without verifying its checksum.

The bootloader connection requests a 64 KiB Web Serial buffer. Metadata reads use 1024-byte blocks with one block in flight, and validate each frame's exact expected length, including the final shorter block, before sending an ACK. A truncated frame fails immediately instead of acknowledging an incomplete transfer and waiting for bytes the stub has already sent.

Each partition-table or app-descriptor read has a ten-second overall deadline, including command transmission, replies, ACKs, and the trailing digest. Failed transfers stop firmware identification and display an error instead of continuing with an uncertain serial session. Pending reads and writes are cancelled on failure, and late completions cannot send another ACK. Error feedback appears before USB cleanup; cleanup waits for at most three seconds in the error handler. If the browser keeps the port locked, the error requests a physical reconnect, and another connection attempt cannot reuse the session while cleanup is pending. These limits do not change erase or firmware-write timeouts.

## Firmware and artifacts

USB detection tries Improv Serial first, then resets the board to inspect boot logs, and finally reads app descriptors through the bootloader when firmware identity or version is still unavailable. The fallbacks recognize Native, Matter, and ESPectre ESPHome firmware. ESPHome logs report the `francescopace.espectre` project version, including on renamed devices. Descriptor-only ESPHome detection recognizes the standard `espectre` app name and leaves the firmware version unavailable because that descriptor contains the ESPHome framework version. Custom ESPHome app names require Improv or project logs for identification. Initial detection does not request Matter pairing codes; the Matter QR action prefers the Improv onboarding RPC and retains serial markers as a compatibility fallback.

Micro-ESPectre is identified from `micro-espectre` app metadata or its startup marker and chip log. New project builds embed the ESPectre Git version, as Native and Matter do; that descriptor identifies the firmware build, not later filesystem deployments. Older startup logs can identify Micro-ESPectre without supplying its version, and detection then avoids a flash read solely to search for a missing version. An ESPectre-branded MicroPython banner is labeled as inferred, with any MicroPython version kept separate from the application version. A generic MicroPython banner or named descriptor identifies only MicroPython. When both descriptor fields are empty, the exact three-partition MicroPython 4 MiB+ layout permits only an inferred MicroPython label. Other layouts remain unknown. Detected MicroPython firmware is not treated as Native, does not offer USB Wi-Fi provisioning, and requires a firmware change rather than an in-place Native update. These fallbacks require neither Improv nor Direct HTTP.

Use locally built firmware in the browser preview by restaging the available Native, Matter, and ESPHome factory images:

```bash
./test/web/generate_firmware_manifest.sh
./test/web/generate_firmware_manifest.sh --dry-run
./test/web/generate_firmware_manifest.sh --replace
```

The helper writes the release catalog under `artifacts/firmware/release/`. It preserves previously staged factory images unless `--replace` is present. Official deployments serve ESPectre version 3 and newer, including 3.x prereleases. Release contains the most recently published numeric GitHub release from version 3, including release candidates; it identifies a tested tagged release, not necessarily a stable version. Selection uses publication time, includes prereleases, and excludes drafts and rolling tags. Older published channels are omitted; until a supported tagged release exists, the Release channel remains unavailable. Rolling versions come from the SDK manifest, with the numeric Git tag ancestry used only to identify older releases without a manifest. New builds also need a numeric 3.x or newer Git tag in their ancestry; builds still identified as `2.8.0-<commits>-g<sha>` cannot be deployed.

All downloads live under the ignored `artifacts/` tree. Firmware uses `artifacts/firmware/<channel>/`; SDK archives use `artifacts/sdk/<channel>/`; and the generated API reference uses `artifacts/sdk/api/`. Generate the API reference with `python3 .github/scripts/generate_sdk_api.py`. It requires Doxygen 1.17.0 and a pinned m.css revision; `--mcss-root` reuses an existing checkout.

The home badge compares the Release and Preview firmware manifest versions and links to the browser installer. It compares numeric version components, prerelease identifiers in natural numeric order, and then the Git snapshot commit count after the same tag. Final tags sort after prereleases, and Release wins ties. Manifest generation timestamps are not used because staging rewrites them. If only one channel has a valid version, the badge uses it; if neither does, it remains hidden. The SDK API reference displays its own build version independently.

Commit CI runs website tests, builds pages and the API reference, and verifies the site without downloading published channels. Before deployment, the Snapshot and Release workflows stage firmware and SDK artifacts from the source CI run or current tag for the channel being updated, recover the other supported published channels, and require verification of every staged channel.

The shared `build-pages` action stages dependencies, runs the web tests, builds static routes and the API reference, and verifies the output before upload. `build_sitemap.py` generates the ignored `sitemap.xml` from `routes.json` and the SDK channels present in the staged Pages tree. Its `lastmod` dates come from the owning Git commits and staged SDK manifests, so Pages builds require full Git history. After deployment, IndexNow receives this exact generated sitemap inventory.

## Routing and Analytics

`404-suggestions.json` maps retired paths to a related destination and its link title. The 404 page loads this map with HTTP revalidation and shows one suggestion for an exact path match, accepting a missing trailing slash or an `index.html` suffix. Unknown paths and failed map requests retain the generic navigation. Suggestions do not redirect or change the requested URL or HTTP 404 status. Internal destinations must be registered public routes; external destinations are limited to the project's GitHub repository.

With Analytics consent, the existing static `page_view` records the requested path without query parameters. Clicking a suggestion also sends `select_404_suggestion` with no custom parameters; its page location identifies the old URL, and the JSON map identifies the destination. Normal destination-link events can still fire independently. No Cloudflare or deployment configuration is required.

The SPA uses canonical paths with the History API. Legacy root hash links remain valid entry points and are replaced with their registered canonical path. Static tool calls to action may use this legacy handoff so the browser opens the persistent shell without losing the selected tool.

Device settings and Monitor load with the shared device session. CSI visualizer, Game, and Theremin load their scripts on first use through `data-script-src`. Keep `app.js` last among the core `defer` scripts because it binds their initializers.

In Demo mode, mouse movement simulates motion. Touch and pen users can drag on the Monitor chart or Theremin pitch display; scrolling remains available outside those areas. The Game retains its press-and-hold flight control.

`assets/js/analytics.js` enables GA4 on production and allowlisted debug hosts only after explicit consent. The router sends manual `page_view` events with canonical `page_location`, `page_path`, `page_title`, and `content_group` values. GA4 page changes based on browser history events must remain disabled to avoid duplicate page views.

All website custom events pass through `trackEvent()`. It rejects unregistered events, strips parameters outside the event contract, validates categorical values and numeric bounds, and normalizes error types and public firmware versions before calling `gtag`. Rolling Git versions are reported as `<major>.<minor>.<patch>-dev`; other unrecognized values become `unknown` or are omitted.

Keep Analytics parameters low-cardinality. They must not include device IDs, network names or addresses, credentials, pairing codes, payloads, raw CSI, or exception messages. Enhanced Measurement is configured in GA4 and does not pass through this custom-event gate. The Analytics tests verify every custom event emitted by the browser tools. The public policy is in [privacy.html](content/privacy.html).

## Direct HTTP

`assets/js/espectre-direct.js` owns resource-oriented Direct HTTP, incremental SSE parsing, abort, and reconnect behavior. Device settings and the live tools share one connection picker with Local connection, Demo, and the planned Remote connection. Relay support is not implemented. The wire contract and capability boundaries are in [API.md](../API.md).

The shared Local connection panel states the minimum supported device firmware, ESPectre 3.0.0-rc1, below its USB setup and connection-help links. This requirement applies to device connections; USB installation and Demo mode remain available without compatible firmware.

Starting Demo or leaving a route cancels pending device discovery; late results cannot replace the active session. The raw CSI parser accepts split or aggregated HTTP chunks while keeping its working buffer bounded.

The shared SSE connection stays open across routes and while the browser is hidden so the device indicator retains live motion. Diagnostics are requested once per second only while the Monitor diagnostics panel is open and the browser document is visible. Wi-Fi scan results are polled after one second, then two seconds, and then every three seconds; leaving Device settings or hiding the document stops polling. Configuration verification reads only the changed resource.

Connection refreshes share in-flight reads and reuse session snapshots maintained by SSE. Device identity, health, sensing, and supported OTA status feed the shared interface; Wi-Fi, MQTT, and settings diagnostics are loaded when Device settings opens. Resources without SSE updates are invalidated when leaving settings. Reconnection discards the previous session's snapshots, aborts its pending HTTP requests, and prevents obsolete refreshes from continuing.

`assets/js/browser-support.js` owns the browser matrix and Local Network Access permission checks. The active connection picker reports recovery guidance for permission, Origin, discovery, timeout, protocol, and SSE capacity failures. Direct support does not scan the LAN or relax a global security header.

## Tests

Run the hardware-independent Direct HTTP, Analytics, and structural tests from the repository root:

```bash
node --test 'test/web/*.mjs'
```

`test/web/generate_firmware_manifest.sh` stages local firmware and is not part of the Node test runner.
