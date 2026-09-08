# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Reproducible Micro-ESPectre firmware build support."""

from __future__ import annotations

import hashlib
import os
import re
import shutil
import subprocess
from dataclasses import dataclass, replace
from pathlib import Path

try:
    import fcntl
except ImportError:  # pragma: no cover - exercised on Windows
    fcntl = None

try:
    import msvcrt
except ImportError:  # pragma: no cover - exercised on POSIX
    msvcrt = None

from .common import FIRMWARE_CACHE_DIR, MICROPYTHON_FIRMWARE_BUILD, REPO_ROOT
from .idf import (
    resolve_idf_build_backend,
    resolve_idf_build_dir_name,
    run_in_idf_environment,
)
from .idf_container import IDF_VERSION, run_toolchain_container


MICROPYTHON_REPOSITORY = "https://github.com/micropython/micropython.git"
MICROPYTHON_COMMIT = "1c3c201149f37fe8d81246191b3127bb198d6306"
MICROPYTHON_LIB_REPOSITORY = "https://github.com/micropython/micropython-lib.git"
MICROPYTHON_LIB_COMMIT = "ee4bb8ff139e24c42b739935fbd8ec7c4d061e02"
MICROPYTHON_PATCH_REVISION = "csi-quality-v2"
PROJECT_FIRMWARE_PROJECT_NAME = "micro-espectre"
PROJECT_FIRMWARE_BOARDS = {
    "esp32": "ESP32_MICRO_ESPECTRE",
    "c3": "ESP32C3_MICRO_ESPECTRE",
    "c5": "ESP32C5_MICRO_ESPECTRE",
    "c6": "ESP32C6_MICRO_ESPECTRE",
    "s2": "ESP32S2_MICRO_ESPECTRE",
    "s3": "ESP32S3_MICRO_ESPECTRE",
}


@dataclass(frozen=True)
class BuiltProjectFirmware:
    """Canonical MicroPython image and the build arguments that produced it."""

    image: Path
    build_dir: Path


PROJECT_FIRMWARE_NAMES = {
    "esp32": f"ESP32_GENERIC-{MICROPYTHON_FIRMWARE_BUILD}-espectre.bin",
    "c3": f"ESP32_GENERIC_C3-{MICROPYTHON_FIRMWARE_BUILD}-espectre.bin",
    "c5": f"ESP32_GENERIC_C5-{MICROPYTHON_FIRMWARE_BUILD}-espectre.bin",
    "c6": f"ESP32_GENERIC_C6-{MICROPYTHON_FIRMWARE_BUILD}-espectre.bin",
    "s2": f"ESP32_GENERIC_S2-{MICROPYTHON_FIRMWARE_BUILD}-espectre.bin",
    "s3": f"ESP32_GENERIC_S3-{MICROPYTHON_FIRMWARE_BUILD}-espectre.bin",
}


def _checkout_pinned_repository(url: str, commit: str, destination: Path) -> None:
    """Create or validate a cached checkout at one exact revision."""
    if destination.is_dir():
        current = subprocess.run(
            ["git", "-C", str(destination), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        if current == commit:
            return
        raise RuntimeError(
            f"Cached checkout has unexpected revision: {destination} ({current})"
        )

    destination.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["git", "clone", "--filter=blob:none", "--no-checkout", url, str(destination)],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(destination), "fetch", "--depth", "1", "origin", commit],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(destination), "checkout", "--detach", commit],
        check=True,
    )


def _prepare_micropython_patch_revision(micropython_dir: Path) -> Path:
    """Restore the pinned sources once when the project patch set changes."""
    stamp_path = micropython_dir / ".espectre-patch-revision"
    if (
        stamp_path.is_file()
        and stamp_path.read_text(encoding="utf-8").strip()
        == MICROPYTHON_PATCH_REVISION
    ):
        return stamp_path

    source_diff = subprocess.run(
        ["git", "-C", str(micropython_dir), "diff", "--binary", "--", "."],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    if source_diff:
        backup_path = (
            micropython_dir.parent
            / f"micropython-before-{MICROPYTHON_PATCH_REVISION}.patch"
        )
        if not backup_path.exists():
            backup_path.write_text(source_diff, encoding="utf-8")

    subprocess.run(
        [
            "git",
            "-C",
            str(micropython_dir),
            "restore",
            "--worktree",
            "--source=HEAD",
            "--",
            ".",
        ],
        check=True,
    )
    stamp_path.unlink(missing_ok=True)
    return stamp_path


def _read_idf_version(idf_path: Path) -> str:
    """Read and normalize the ESP-IDF release used by a local backend."""
    version_file = idf_path / "version.txt"
    if not version_file.is_file():
        raise RuntimeError(f"ESP-IDF version file is missing: {version_file}")
    return version_file.read_text(encoding="utf-8").strip().removeprefix("v")


def _configure_project_identity(micropython_dir: Path) -> None:
    """Set the Micro-ESPectre name and use the shared Native and Matter Git version resolver."""
    cmake_path = micropython_dir / "ports" / "esp32" / "CMakeLists.txt"
    source = cmake_path.read_text(encoding="utf-8")
    original = "project(micropython)"
    replacement = (
        'include("${ESPECTRE_CORE_SDK_ROOT}/espectre_git_version.cmake")\n'
        'set(PROJECT_VER "${ESPECTRE_GIT_VERSION}")\n'
        f"project({PROJECT_FIRMWARE_PROJECT_NAME})"
    )
    if replacement in source:
        return
    if source.count(original) != 1:
        raise RuntimeError("Unable to configure the Micro-ESPectre project identity")
    cmake_path.write_text(source.replace(original, replacement), encoding="utf-8")


def _align_idf_lockfile(micropython_dir: Path, chip: str, idf_version: str) -> None:
    """Align the cached MicroPython component lock with the ESP-IDF 5.5 patch release."""
    if re.fullmatch(r"5\.5\.\d+", idf_version) is None:
        raise RuntimeError(f"ESP-IDF 5.5 is required; found {idf_version or 'unknown'}")

    lockfile = (
        micropython_dir
        / "ports"
        / "esp32"
        / "lockfiles"
        / f"dependencies.lock.esp32{'' if chip == 'esp32' else chip}"
    )
    lines = lockfile.read_text(encoding="utf-8").splitlines(keepends=True)
    in_idf_dependency = False
    for index, line in enumerate(lines):
        if line.rstrip("\r\n") == "  idf:":
            in_idf_dependency = True
            continue
        if in_idf_dependency and line.startswith("    version: "):
            newline = "\r\n" if line.endswith("\r\n") else "\n"
            lines[index] = f"    version: {idf_version}{newline}"
            lockfile.write_text("".join(lines), encoding="utf-8")
            return
        if in_idf_dependency and not line.startswith("    "):
            break
    raise RuntimeError(f"ESP-IDF dependency entry is missing: {lockfile}")


def _configure_project_csi_capture(micropython_dir: Path, chip: str) -> None:
    """Match the target's production CSI PHY profile."""
    source_path = micropython_dir / "ports" / "esp32" / "network_wlan_csi.c"
    source = source_path.read_text(encoding="utf-8")
    source = source.replace(".acquire_csi_legacy = 1,", ".acquire_csi_legacy = 0,", 1)
    c5_profile_block = """    #if CONFIG_IDF_TARGET_ESP32C5
    wifi_ap_record_t ap_info = {0};
    const bool use_vht20 =
        esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK && ap_info.primary > 14;
    config->acquire_csi_ht20 = !use_vht20;
    config->acquire_csi_vht = use_vht20;
    #endif"""
    if chip == "c5" and c5_profile_block not in source:
        c6_profile_block = """    #if CONFIG_IDF_TARGET_ESP32C6
    config->acquire_csi_he_stbc = 0;
    #endif"""
        if c6_profile_block not in source:
            raise RuntimeError(
                f"MicroPython CSI profile builder anchor is missing: {source_path}"
            )
        source = source.replace(
            c6_profile_block, c6_profile_block + "\n\n" + c5_profile_block, 1
        )
    classic_esp32 = chip == "esp32"
    desired_lltf = 1 if classic_esp32 else 0
    desired_htltf = 0 if classic_esp32 else 1
    for field, desired in (("lltf", desired_lltf), ("htltf", desired_htltf)):
        replacement = f".{field}_en = {desired},"
        source = source.replace(f".{field}_en = 0,", replacement)
        source = source.replace(f".{field}_en = 1,", replacement)
    source_path.write_text(source, encoding="utf-8")

    has_wifi_6_profile = ".acquire_csi_legacy" in source
    has_legacy_profile = ".lltf_en" in source
    if has_wifi_6_profile and ".acquire_csi_legacy = 0," not in source:
        raise RuntimeError(
            f"MicroPython CSI legacy-capture setting is missing: {source_path}"
        )
    if (
        chip == "c5"
        and c5_profile_block not in source
    ):
        raise RuntimeError(
            f"MicroPython CSI VHT20 capture setting is missing: {source_path}"
        )
    if has_legacy_profile and (
        f".lltf_en = {desired_lltf}," not in source
        or f".htltf_en = {desired_htltf}," not in source
    ):
        raise RuntimeError(
            f"MicroPython CSI HT20 LTF profile is missing: {source_path}"
        )
    if not has_wifi_6_profile and not has_legacy_profile:
        raise RuntimeError(f"MicroPython CSI PHY profile is missing: {source_path}")


def _configure_project_csi_rearm(micropython_dir: Path) -> None:
    """Harden the CSI ring and expose native callback observability."""
    esp32_dir = micropython_dir / "ports" / "esp32"
    implementation_path = esp32_dir / "network_wlan_csi.c"
    header_path = esp32_dir / "network_wlan_csi.h"
    wlan_path = esp32_dir / "network_wlan.c"

    implementation = implementation_path.read_text(encoding="utf-8")
    documented_disable_errors = (
        "    if (disable_err == ESP_ERR_WIFI_NOT_STARTED || "
        "disable_err == ESP_ERR_WIFI_NOT_INIT) {"
    )
    normalized_disable_errors = (
        "    if (disable_err == ESP_ERR_INVALID_ARG ||\n"
        "        disable_err == ESP_ERR_WIFI_NOT_STARTED || "
        "disable_err == ESP_ERR_WIFI_NOT_INIT) {"
    )
    if documented_disable_errors in implementation:
        implementation = implementation.replace(
            documented_disable_errors,
            normalized_disable_errors,
            1,
        )
        implementation_path.write_text(implementation, encoding="utf-8")
    # The variable-record prototype was benchmarked and rejected. A patch
    # revision refresh must restore the pinned fixed-record source before this
    # hardening runs, otherwise an old cache could silently select that layout.
    if "csi_frame_header_t" in implementation:
        raise RuntimeError(
            f"stale variable-record CSI source survived the patch refresh: {implementation_path}"
        )

    # Newer CSI sources own their state and receive ring in native memory and
    # already provide the hardened lifecycle. Do not rewrite that upstreamable
    # implementation with the compatibility transforms below.
    if "static csi_state_t *wifi_csi_state;" in implementation:
        header = header_path.read_text(encoding="utf-8")
        wlan = wlan_path.read_text(encoding="utf-8")
        required_implementation = (
            "heap_caps_calloc",
            "heap_caps_malloc",
            "wifi_csi_release_ring",
            "network_wlan_csi_rearm_obj",
            "network_wlan_csi_callbacks_obj",
        )
        required_header = (
            "network_wlan_csi_rearm_obj",
            "network_wlan_csi_callbacks_obj",
        )
        required_wlan = (
            "MP_QSTR_csi_rearm",
            "MP_QSTR_csi_callbacks",
        )
        if (
            any(token not in implementation for token in required_implementation)
            or any(token not in header for token in required_header)
            or any(token not in wlan for token in required_wlan)
        ):
            raise RuntimeError(
                f"MicroPython native CSI lifecycle is incomplete: {implementation_path}"
            )
        return

    implementation = implementation.replace(
        "static void IRAM_ATTR wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {",
        "static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {",
        1,
    ).replace(
        "        const byte *current = mp_obj_str_get_data(*mac_obj, &current_length);",
        "        const char *current = mp_obj_str_get_data(*mac_obj, &current_length);",
        1,
    ).replace(
        "    // Keep this static to avoid putting a large frame on the ISR stack.\n",
        "    // The ESP-IDF Wi-Fi task serializes callback invocations. Keep the\n"
        "    // frame static to avoid consuming that task's limited stack.\n",
        1,
    )
    callback_signature = (
        "static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {\n"
    )
    if "ESP-IDF Wi-Fi task serializes callback invocations" not in implementation:
        implementation = implementation.replace(
            callback_signature,
            callback_signature
            + "    // ESP-IDF invokes CSI from its serialized Wi-Fi task, not an ISR.\n",
            1,
        )
    mac_allocation = (
        "    result->items[2] = mp_obj_new_bytes(frame.mac, sizeof(frame.mac));\n"
    )
    if mac_allocation in implementation:
        read_anchor = (
            "static mp_obj_t network_wlan_csi_read(size_t n_args, "
            "const mp_obj_t *args) {\n"
        )
        if read_anchor not in implementation:
            raise RuntimeError(
                f"MicroPython CSI reusable-MAC read anchor is missing: {implementation_path}"
            )
        mac_helper = """static void network_wlan_csi_update_mac(mp_obj_t *mac_obj, const uint8_t *mac) {
    bool unchanged = false;
    if (*mac_obj != MP_OBJ_NULL && mp_obj_is_type(*mac_obj, &mp_type_bytes)) {
        size_t current_length = 0;
        const char *current = mp_obj_str_get_data(*mac_obj, &current_length);
        unchanged = current_length == 6 && memcmp(current, mac, 6) == 0;
    }
    if (!unchanged) {
        *mac_obj = mp_obj_new_bytes(mac, 6);
    }
}

"""
        implementation = implementation.replace(
            read_anchor,
            mac_helper + read_anchor,
            1,
        ).replace(
            mac_allocation,
            "    network_wlan_csi_update_mac(&result->items[2], frame.mac);\n",
            1,
        )
    # Persist the independent callback/MAC hot-path updates before the
    # idempotent hardening blocks reread the generated source below.  Without
    # this write, an already-hardened checkout silently lost these two updates.
    implementation_path.write_text(implementation, encoding="utf-8")
    if "volatile uint32_t callbacks;" not in implementation:
        replacements = (
            (
                "    if (state == NULL || state->ringbuffer.buf == NULL) {\n"
                "        return;\n"
                "    }",
                "    if (info == NULL || state == NULL || state->ringbuffer.buf == NULL) {\n"
                "        return;\n"
                "    }",
            ),
            (
                "    volatile uint32_t dropped;\n} csi_state_t;",
                "    volatile uint32_t dropped;\n"
                "    volatile uint32_t callbacks;\n"
                "} csi_state_t;",
            ),
            (
                "    if (ringbuf_put_bytes(&state->ringbuffer, (uint8_t *)&frame, sizeof(frame)) != 0) {\n"
                "        state->dropped++;\n"
                "    }",
                "    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();\n"
                "    state->callbacks++;\n"
                "    if (ringbuf_put_bytes(&state->ringbuffer, (uint8_t *)&frame, sizeof(frame)) != 0) {\n"
                "        state->dropped++;\n"
                "    }\n"
                "    MICROPY_END_ATOMIC_SECTION(atomic_state);",
            ),
            (
                "    ringbuf_alloc(&state->ringbuffer, sizeof(csi_frame_t) * state->buffer_size);\n"
                "    state->dropped = 0;",
                "    // ringbuf_t reserves one byte to distinguish full from empty.\n"
                "    ringbuf_alloc(&state->ringbuffer, sizeof(csi_frame_t) * state->buffer_size + 1);\n"
                "    state->dropped = 0;\n"
                "    state->callbacks = 0;",
            ),
            (
                "    state->dropped = 0;\n"
                "    return ESP_OK;",
                "    state->dropped = 0;\n"
                "    state->callbacks = 0;\n"
                "    return ESP_OK;",
            ),
            (
                "    esp_err_t err = esp_wifi_set_csi(false);\n"
                "    if (err != ESP_OK) {\n"
                "        return err;\n"
                "    }\n"
                "\n"
                "    err = esp_wifi_set_csi_rx_cb(NULL, NULL);",
                "    // Detach the callback before disabling capture so no producer can\n"
                "    // retain a reference to the ring while it is being released.\n"
                "    esp_err_t err = esp_wifi_set_csi_rx_cb(NULL, NULL);\n"
                "    if (err != ESP_OK) {\n"
                "        return err;\n"
                "    }\n"
                "\n"
                "    err = esp_wifi_set_csi(false);",
            ),
            (
                "        esp_wifi_set_csi(false);\n"
                "        esp_wifi_set_csi_rx_cb(NULL, NULL);",
                "        esp_wifi_set_csi_rx_cb(NULL, NULL);\n"
                "        esp_wifi_set_csi(false);",
            ),
            (
                "static mp_obj_t network_wlan_csi_dropped(mp_obj_t self_in) {\n"
                "    (void)self_in;\n"
                "    csi_state_t *state = (csi_state_t *)MP_STATE_PORT(csi_state);\n"
                "    return mp_obj_new_int(state == NULL ? 0 : state->dropped);\n"
                "}\n"
                "MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_csi_dropped_obj, network_wlan_csi_dropped);",
                "static mp_obj_t network_wlan_csi_dropped(mp_obj_t self_in) {\n"
                "    (void)self_in;\n"
                "    csi_state_t *state = (csi_state_t *)MP_STATE_PORT(csi_state);\n"
                "    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();\n"
                "    uint32_t dropped = state == NULL ? 0 : state->dropped;\n"
                "    MICROPY_END_ATOMIC_SECTION(atomic_state);\n"
                "    return mp_obj_new_int_from_uint(dropped);\n"
                "}\n"
                "MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_csi_dropped_obj, network_wlan_csi_dropped);\n"
                "\n"
                "static mp_obj_t network_wlan_csi_callbacks(mp_obj_t self_in) {\n"
                "    (void)self_in;\n"
                "    csi_state_t *state = (csi_state_t *)MP_STATE_PORT(csi_state);\n"
                "    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();\n"
                "    uint32_t callbacks = state == NULL ? 0 : state->callbacks;\n"
                "    MICROPY_END_ATOMIC_SECTION(atomic_state);\n"
                "    return mp_obj_new_int_from_uint(callbacks);\n"
                "}\n"
                "MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_csi_callbacks_obj, network_wlan_csi_callbacks);",
            ),
        )
        for original, replacement in replacements:
            if original not in implementation:
                raise RuntimeError(
                    f"MicroPython CSI hardening anchor is missing: {implementation_path}"
                )
            implementation = implementation.replace(original, replacement, 1)
        implementation_path.write_text(implementation, encoding="utf-8")

    implementation = implementation_path.read_text(encoding="utf-8")
    if "portMUX_TYPE lock;" not in implementation:
        lock_replacements = (
            (
                "    volatile uint32_t callbacks;\n} csi_state_t;",
                "    volatile uint32_t callbacks;\n"
                "    portMUX_TYPE lock;\n"
                "} csi_state_t;",
            ),
            (
                "        memset(state, 0, sizeof(*state));\n"
                "        state->buffer_size = MICROPY_PY_NETWORK_WLAN_CSI_DEFAULT_BUFFER_SIZE;",
                "        memset(state, 0, sizeof(*state));\n"
                "        portMUX_INITIALIZE(&state->lock);\n"
                "        state->buffer_size = MICROPY_PY_NETWORK_WLAN_CSI_DEFAULT_BUFFER_SIZE;",
            ),
            (
                "    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();\n"
                "    state->callbacks++;\n"
                "    if (ringbuf_put_bytes(&state->ringbuffer, (uint8_t *)&frame, sizeof(frame)) != 0) {\n"
                "        state->dropped++;\n"
                "    }\n"
                "    MICROPY_END_ATOMIC_SECTION(atomic_state);",
                "    portENTER_CRITICAL(&state->lock);\n"
                "    if (ringbuf_put_bytes(&state->ringbuffer, (uint8_t *)&frame, sizeof(frame)) != 0) {\n"
                "        __atomic_fetch_add(&state->dropped, 1, __ATOMIC_RELAXED);\n"
                "    }\n"
                "    portEXIT_CRITICAL(&state->lock);",
            ),
            (
                "    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();\n"
                "    int result = ringbuf_get_bytes(&state->ringbuffer, (uint8_t *)frame, sizeof(*frame));\n"
                "    MICROPY_END_ATOMIC_SECTION(atomic_state);",
                "    portENTER_CRITICAL(&state->lock);\n"
                "    int result = ringbuf_get_bytes(&state->ringbuffer, (uint8_t *)frame, sizeof(*frame));\n"
                "    portEXIT_CRITICAL(&state->lock);",
            ),
            (
                "    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();\n"
                "    uint32_t dropped = state == NULL ? 0 : state->dropped;\n"
                "    MICROPY_END_ATOMIC_SECTION(atomic_state);",
                "    uint32_t dropped = state == NULL\n"
                "        ? 0\n"
                "        : __atomic_load_n(&state->dropped, __ATOMIC_RELAXED);",
            ),
            (
                "    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();\n"
                "    uint32_t callbacks = state == NULL ? 0 : state->callbacks;\n"
                "    MICROPY_END_ATOMIC_SECTION(atomic_state);",
                "    uint32_t callbacks = state == NULL\n"
                "        ? 0\n"
                "        : __atomic_load_n(&state->callbacks, __ATOMIC_RELAXED);",
            ),
            (
                "    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();\n"
                "    size_t available = ringbuf_avail(&state->ringbuffer);\n"
                "    MICROPY_END_ATOMIC_SECTION(atomic_state);",
                "    portENTER_CRITICAL(&state->lock);\n"
                "    size_t available = ringbuf_avail(&state->ringbuffer);\n"
                "    portEXIT_CRITICAL(&state->lock);",
            ),
        )
        for original, replacement in lock_replacements:
            if original not in implementation:
                raise RuntimeError(
                    f"MicroPython CSI dedicated-lock anchor is missing: {implementation_path}"
                )
            implementation = implementation.replace(original, replacement, 1)
        callback_anchor = (
            "    if (info == NULL || state == NULL || state->ringbuffer.buf == NULL) {\n"
            "        return;\n"
            "    }\n"
        )
        if callback_anchor not in implementation:
            raise RuntimeError(
                f"MicroPython CSI callback-counter anchor is missing: {implementation_path}"
            )
        implementation = implementation.replace(
            callback_anchor,
            callback_anchor
            + "\n    __atomic_fetch_add(&state->callbacks, 1, __ATOMIC_RELAXED);\n",
            1,
        )
        implementation_path.write_text(implementation, encoding="utf-8")

    implementation = implementation_path.read_text(encoding="utf-8")
    if "heap_caps_malloc" not in implementation:
        # The Wi-Fi task owns the producer side of this ring. Keep its opaque CSI
        # bytes out of the conservative MicroPython collector so they cannot be
        # scanned as false object roots or mutate while a dual-core GC scans them.
        native_allocation = (
            "    // The Wi-Fi task writes this ring concurrently with MicroPython GC.\n"
            "    // Keep it outside the GC heap so collection cannot scan or reclaim it.\n"
            "    size_t ring_size = sizeof(csi_frame_t) * state->buffer_size + 1;\n"
            "    state->ringbuffer.buf = heap_caps_malloc(\n"
            "        ring_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);\n"
            "    if (state->ringbuffer.buf == NULL) {\n"
            "        return ESP_ERR_NO_MEM;\n"
            "    }\n"
            "    state->ringbuffer.size = ring_size;\n"
            "    state->ringbuffer.iget = 0;\n"
            "    state->ringbuffer.iput = 0;"
        )
        gc_allocation = (
            "    // ringbuf_t reserves one byte to distinguish full from empty.\n"
            "    ringbuf_alloc(&state->ringbuffer, sizeof(csi_frame_t) * state->buffer_size + 1);"
        )
        if gc_allocation not in implementation:
            raise RuntimeError(
                f"MicroPython CSI native-ring allocation anchor is missing: {implementation_path}"
            )
        implementation = implementation.replace(gc_allocation, native_allocation, 1)
        implementation = implementation.replace(
            "m_del(uint8_t, state->ringbuffer.buf, state->ringbuffer.size);",
            "heap_caps_free(state->ringbuffer.buf);",
        )
        implementation = implementation.replace(
            '#include "esp_timer.h"\n',
            '#include "esp_heap_caps.h"\n#include "esp_timer.h"\n',
            1,
        )
        implementation_path.write_text(implementation, encoding="utf-8")

    implementation = implementation_path.read_text(encoding="utf-8")
    unsafe_deinit = """    if (state->ringbuffer.buf != NULL) {
        esp_wifi_set_csi_rx_cb(NULL, NULL);
        esp_wifi_set_csi(false);
        heap_caps_free(state->ringbuffer.buf);
    }

    m_del_obj(csi_state_t, state);
    MP_STATE_PORT(csi_state) = NULL;"""
    safe_deinit = """    if (state->ringbuffer.buf != NULL && wifi_csi_disable(state) != ESP_OK) {
        // A callback that raced with this teardown resolves the root on entry.
        // Clear it before GC can reclaim the state, and intentionally retain the
        // native ring rather than exposing the Wi-Fi task to freed memory.
        MP_STATE_PORT(csi_state) = NULL;
        return;
    }

    m_del_obj(csi_state_t, state);
    MP_STATE_PORT(csi_state) = NULL;"""
    if unsafe_deinit in implementation:
        implementation = implementation.replace(unsafe_deinit, safe_deinit, 1)
        implementation_path.write_text(implementation, encoding="utf-8")
    elif safe_deinit not in implementation:
        raise RuntimeError(
            f"MicroPython CSI safe-deinit anchor is missing: {implementation_path}"
        )

    if "network_wlan_csi_rearm_obj" not in implementation:
        anchor = "static mp_obj_t network_wlan_csi_disable(mp_obj_t self_in) {\n"
        addition = """static mp_obj_t network_wlan_csi_rearm(mp_obj_t self_in) {
    (void)self_in;
    csi_state_t *state = (csi_state_t *)MP_STATE_PORT(csi_state);
    if (state == NULL || state->ringbuffer.buf == NULL) {
        esp_exceptions(ESP_ERR_INVALID_STATE);
    }

    esp_exceptions(esp_wifi_set_csi_rx_cb(NULL, NULL));
    esp_exceptions(esp_wifi_set_csi(false));

    wifi_csi_config_t config;
    wifi_csi_build_config(&config);
    esp_exceptions(esp_wifi_set_csi_config(&config));

    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    state->ringbuffer.iget = 0;
    state->ringbuffer.iput = 0;
    state->dropped = 0;
    state->callbacks = 0;
    MICROPY_END_ATOMIC_SECTION(atomic_state);

    esp_exceptions(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    esp_exceptions(esp_wifi_set_csi(true));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_csi_rearm_obj, network_wlan_csi_rearm);

"""
        if anchor not in implementation:
            raise RuntimeError(
                f"MicroPython CSI rearm implementation anchor is missing: {implementation_path}"
            )
        implementation_path.write_text(
            implementation.replace(anchor, addition + anchor, 1),
            encoding="utf-8",
        )

    implementation = implementation_path.read_text(encoding="utf-8")
    old_rearm_lock = (
        "    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();\n"
        "    state->ringbuffer.iget = 0;\n"
        "    state->ringbuffer.iput = 0;\n"
        "    state->dropped = 0;\n"
        "    state->callbacks = 0;\n"
        "    MICROPY_END_ATOMIC_SECTION(atomic_state);"
    )
    if old_rearm_lock in implementation:
        implementation = implementation.replace(
            old_rearm_lock,
            "    portENTER_CRITICAL(&state->lock);\n"
            "    state->ringbuffer.iget = 0;\n"
            "    state->ringbuffer.iput = 0;\n"
            "    state->dropped = 0;\n"
            "    state->callbacks = 0;\n"
            "    portEXIT_CRITICAL(&state->lock);",
            1,
        )
    old_rearm_order = "    esp_exceptions(esp_wifi_set_csi(false));\n    esp_exceptions(esp_wifi_set_csi_rx_cb(NULL, NULL));"
    if old_rearm_order in implementation:
        implementation = implementation.replace(
            old_rearm_order,
            "    esp_exceptions(esp_wifi_set_csi_rx_cb(NULL, NULL));\n    esp_exceptions(esp_wifi_set_csi(false));",
            1,
        )
    old_rearm_reset = "    state->dropped = 0;\n    MICROPY_END_ATOMIC_SECTION(atomic_state);"
    rearm_start = implementation.find("static mp_obj_t network_wlan_csi_rearm(")
    rearm_end = implementation.find("MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_csi_rearm_obj", rearm_start)
    if rearm_start < 0 or rearm_end < 0:
        raise RuntimeError(
            f"MicroPython CSI rearm implementation is missing: {implementation_path}"
        )
    rearm_implementation = implementation[rearm_start:rearm_end]
    if "state->callbacks = 0;" not in rearm_implementation:
        if old_rearm_reset not in rearm_implementation:
            raise RuntimeError(
                f"MicroPython CSI rearm reset anchor is missing: {implementation_path}"
            )
        rearm_implementation = rearm_implementation.replace(
            old_rearm_reset,
            "    state->dropped = 0;\n    state->callbacks = 0;\n    MICROPY_END_ATOMIC_SECTION(atomic_state);",
            1,
        )
        implementation = (
            implementation[:rearm_start]
            + rearm_implementation
            + implementation[rearm_end:]
        )
    implementation_path.write_text(implementation, encoding="utf-8")

    header = header_path.read_text(encoding="utf-8")
    if "network_wlan_csi_rearm_obj" not in header:
        anchor = "MP_DECLARE_CONST_FUN_OBJ_1(network_wlan_csi_disable_obj);\n"
        if anchor not in header:
            raise RuntimeError(
                f"MicroPython CSI rearm header anchor is missing: {header_path}"
            )
        header_path.write_text(
            header.replace(
                anchor,
                anchor + "MP_DECLARE_CONST_FUN_OBJ_1(network_wlan_csi_rearm_obj);\n",
                1,
            ),
            encoding="utf-8",
        )
        header = header_path.read_text(encoding="utf-8")
    if "network_wlan_csi_callbacks_obj" not in header:
        anchor = "MP_DECLARE_CONST_FUN_OBJ_1(network_wlan_csi_dropped_obj);\n"
        if anchor not in header:
            raise RuntimeError(
                f"MicroPython CSI callback-counter header anchor is missing: {header_path}"
            )
        header_path.write_text(
            header.replace(
                anchor,
                anchor + "MP_DECLARE_CONST_FUN_OBJ_1(network_wlan_csi_callbacks_obj);\n",
                1,
            ),
            encoding="utf-8",
        )

    wlan = wlan_path.read_text(encoding="utf-8")
    if "MP_QSTR_csi_rearm" not in wlan:
        anchor = "    { MP_ROM_QSTR(MP_QSTR_csi_disable), MP_ROM_PTR(&network_wlan_csi_disable_obj) },\n"
        if anchor not in wlan:
            raise RuntimeError(
                f"MicroPython CSI rearm WLAN anchor is missing: {wlan_path}"
            )
        wlan_path.write_text(
            wlan.replace(
                anchor,
                anchor + "    { MP_ROM_QSTR(MP_QSTR_csi_rearm), MP_ROM_PTR(&network_wlan_csi_rearm_obj) },\n",
                1,
            ),
            encoding="utf-8",
        )
        wlan = wlan_path.read_text(encoding="utf-8")
    if "MP_QSTR_csi_callbacks" not in wlan:
        anchor = "    { MP_ROM_QSTR(MP_QSTR_csi_dropped), MP_ROM_PTR(&network_wlan_csi_dropped_obj) },\n"
        if anchor not in wlan:
            raise RuntimeError(
                f"MicroPython CSI callback-counter WLAN anchor is missing: {wlan_path}"
            )
        wlan_path.write_text(
            wlan.replace(
                anchor,
                anchor + "    { MP_ROM_QSTR(MP_QSTR_csi_callbacks), MP_ROM_PTR(&network_wlan_csi_callbacks_obj) },\n",
                1,
            ),
            encoding="utf-8",
        )


def _configure_project_csi_quality(micropython_dir: Path) -> None:
    """Reject invalid hardware estimates before publishing a MicroPython frame."""
    esp32_dir = micropython_dir / "ports" / "esp32"
    path = esp32_dir / "network_wlan_csi.c"
    source = path.read_text(encoding="utf-8")
    marker = "// ESPectre CSI quality admission."
    if marker in source:
        return
    signature = "static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {\n"
    copy = "memcpy(frame.data, info->buf, frame.len);"
    counter = "    __atomic_fetch_add(&state->callbacks, 1, __ATOMIC_RELAXED);\n"
    field = "    volatile uint32_t callbacks;\n"
    reset = "    state->callbacks = 0;\n"
    getter_start = "static mp_obj_t network_wlan_csi_callbacks("
    getter_end = "MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_csi_callbacks_obj, network_wlan_csi_callbacks);"
    if any(anchor not in source for anchor in
           (signature, copy, counter, field, reset, getter_start, getter_end)):
        raise RuntimeError(f"MicroPython CSI quality admission anchors are missing: {path}")
    admission = """// ESPectre CSI quality admission.
static bool espectre_csi_quality_valid(const wifi_csi_info_t *info) {
    if (info == NULL || info->buf == NULL || info->len == 0 || (info->len & 1) ||
        info->rx_ctrl.rx_state != 0) {
        return false;
    }
    #if CONFIG_SOC_WIFI_HE_SUPPORT
    if (info->rx_ctrl.rxend_state != 0 || !info->rx_ctrl.rx_channel_estimate_info_vld) {
        return false;
    }
    #endif
    if (info->first_word_invalid) {
        // Only independently identified, full-width centered guards can be
        // cleaned. Compact and classic-order first pairs contain live tones.
        if (info->len != 128 && info->len != 256) {
            return false;
        }
        static const uint8_t guard_bins[] = {2, 3, 61, 62, 63};
        static const uint8_t live_bins[] = {29, 30, 31, 33, 34, 35};
        for (unsigned i = 0; i < sizeof(guard_bins); ++i) {
            const unsigned offset = guard_bins[i] * 2;
            if (info->buf[offset] != 0 || info->buf[offset + 1] != 0) {
                return false;
            }
        }
        for (unsigned i = 0; i < sizeof(live_bins); ++i) {
            const unsigned offset = live_bins[i] * 2;
            if (info->buf[offset] == 0 && info->buf[offset + 1] == 0) {
                return false;
            }
        }
    }
    return true;
}

"""
    source = source.replace(signature, admission + signature, 1)
    source = source.replace(counter, counter + "    if (!espectre_csi_quality_valid(info)) {\n"
                            "        __atomic_fetch_add(&state->filtered, 1, __ATOMIC_RELAXED);\n"
                            "        return;\n"
                            "    }\n", 1)
    source = source.replace(copy, copy + "\n        if (info->first_word_invalid) {\n"
                            "            memset(frame.data, 0, frame.len < 4 ? frame.len : 4);\n"
                            "        }", 1)
    source = source.replace(field, field + "    volatile uint32_t filtered;\n", 1)
    source = source.replace(reset, reset + "    state->filtered = 0;\n")
    # Keep native counter reads and MicroPython registration identical.
    getter = source[source.index(getter_start):source.index(getter_end) + len(getter_end)]
    source = source.replace(getter, getter + "\n\n" + getter.replace("callbacks", "filtered"), 1)
    registrations = (
        (esp32_dir / "network_wlan_csi.h",
         "MP_DECLARE_CONST_FUN_OBJ_1(network_wlan_csi_callbacks_obj);"),
        (esp32_dir / "network_wlan.c",
         "    { MP_ROM_QSTR(MP_QSTR_csi_callbacks), MP_ROM_PTR(&network_wlan_csi_callbacks_obj) },"),
    )
    updates = []
    for registration_path, anchor in registrations:
        content = registration_path.read_text(encoding="utf-8")
        if anchor not in content:
            raise RuntimeError(f"MicroPython CSI counter registration anchor is missing: {registration_path}")
        updates.append((registration_path, content.replace(
            anchor, anchor + "\n" + anchor.replace("callbacks", "filtered"), 1)))
    for registration_path, content in updates:
        registration_path.write_text(content, encoding="utf-8")
    path.write_text(source, encoding="utf-8")


def _configure_project_csi_fixed_records(micropython_dir: Path) -> None:
    """Use one fixed ring stride selected by the runtime payload bound."""
    source_path = micropython_dir / "ports" / "esp32" / "network_wlan_csi.c"
    source = source_path.read_text(encoding="utf-8")
    required = (
        "static size_t wifi_csi_record_size(const csi_state_t *state)",
        "offsetof(csi_frame_t, data) + state->max_data_len",
        "MP_QSTR_max_data_len",
        "wifi_csi_record_size(state) * state->buffer_size + 1",
        "ringbuf_put_bytes(&state->ringbuffer, (uint8_t *)&frame, wifi_csi_record_size(state))",
        "ringbuf_get_bytes(&state->ringbuffer, (uint8_t *)frame, wifi_csi_record_size(state))",
        "available / wifi_csi_record_size(state)",
    )
    if all(token in source for token in required):
        return
    if "csi_frame_header_t" in source:
        raise RuntimeError(
            f"variable-record CSI source cannot be configured as fixed records: {source_path}"
        )

    native_allocation = (
        "size_t ring_size = sizeof(csi_frame_t) * state->buffer_size + 1;"
    )
    managed_allocation = (
        "ringbuf_alloc(&state->ringbuffer, sizeof(csi_frame_t) * state->buffer_size);"
    )
    if native_allocation in source:
        source = source.replace(
            native_allocation,
            "size_t ring_size = wifi_csi_record_size(state) * state->buffer_size + 1;",
            1,
        )
    elif managed_allocation in source:
        source = source.replace(
            managed_allocation,
            "ringbuf_alloc(&state->ringbuffer, "
            "wifi_csi_record_size(state) * state->buffer_size + 1);",
            1,
        )
    else:
        raise RuntimeError(
            f"MicroPython fixed-record CSI allocation anchor is missing: {source_path}"
        )

    replacements = (
        (
            '#include "modnetwork.h"\n#include <stdint.h>',
            '#include "modnetwork.h"\n#include <stddef.h>\n#include <stdint.h>',
        ),
        (
            "// ringbuf_t uses uint16_t for the byte size, so keep the Python-visible limit\n"
            "// within the maximum addressable ringbuffer capacity.\n"
            "#define CSI_MAX_BUFFER_SIZE ((UINT16_MAX - 1) / sizeof(csi_frame_t))\n\n",
            "",
        ),
        (
            "    uint16_t buffer_size;\n"
            "    volatile uint32_t dropped;",
            "    uint16_t buffer_size;\n"
            "    uint16_t max_data_len;\n"
            "    volatile uint32_t dropped;",
        ),
        (
            "} csi_state_t;\n\n"
            "static csi_state_t *wifi_csi_get_state(void) {",
            "} csi_state_t;\n\n"
            "static size_t wifi_csi_record_size(const csi_state_t *state) {\n"
            "    return offsetof(csi_frame_t, data) + state->max_data_len;\n"
            "}\n\n"
            "static csi_state_t *wifi_csi_get_state(void) {",
        ),
        (
            "        state->buffer_size = MICROPY_PY_NETWORK_WLAN_CSI_DEFAULT_BUFFER_SIZE;",
            "        state->buffer_size = MICROPY_PY_NETWORK_WLAN_CSI_DEFAULT_BUFFER_SIZE;\n"
            "        state->max_data_len = CSI_MAX_DATA_LEN;",
        ),
        (
            "frame.len = info->len > CSI_MAX_DATA_LEN ? CSI_MAX_DATA_LEN : info->len;",
            "frame.len = info->len > state->max_data_len ? state->max_data_len : info->len;",
        ),
        (
            "ringbuf_put_bytes(&state->ringbuffer, (uint8_t *)&frame, sizeof(frame))",
            "ringbuf_put_bytes(&state->ringbuffer, (uint8_t *)&frame, wifi_csi_record_size(state))",
        ),
        (
            "ringbuf_get_bytes(&state->ringbuffer, (uint8_t *)frame, sizeof(*frame))",
            "ringbuf_get_bytes(&state->ringbuffer, (uint8_t *)frame, wifi_csi_record_size(state))",
        ),
        (
            "    static const mp_arg_t allowed_args[] = {\n"
            "        { MP_QSTR_buffer_size, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = MICROPY_PY_NETWORK_WLAN_CSI_DEFAULT_BUFFER_SIZE} },\n"
            "    };",
            "    enum { ARG_buffer_size, ARG_max_data_len };\n"
            "    static const mp_arg_t allowed_args[] = {\n"
            "        { MP_QSTR_buffer_size, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = MICROPY_PY_NETWORK_WLAN_CSI_DEFAULT_BUFFER_SIZE} },\n"
            "        { MP_QSTR_max_data_len, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = CSI_MAX_DATA_LEN} },\n"
            "    };",
        ),
        (
            "    mp_int_t buffer_size = parsed_args[0].u_int;\n"
            "    if (buffer_size < 1 || buffer_size > CSI_MAX_BUFFER_SIZE) {\n"
            "        mp_raise_ValueError(MP_ERROR_TEXT(\"buffer_size out of range\"));\n"
            "    }",
            "    mp_int_t max_data_len = parsed_args[ARG_max_data_len].u_int;\n"
            "    if (max_data_len < 1 || max_data_len > CSI_MAX_DATA_LEN) {\n"
            "        mp_raise_ValueError(MP_ERROR_TEXT(\"max_data_len out of range\"));\n"
            "    }\n\n"
            "    size_t record_size = offsetof(csi_frame_t, data) + max_data_len;\n"
            "    size_t max_buffer_size = (UINT16_MAX - 1) / record_size;\n"
            "    mp_int_t buffer_size = parsed_args[ARG_buffer_size].u_int;\n"
            "    if (buffer_size < 1 || (size_t)buffer_size > max_buffer_size) {\n"
            "        mp_raise_ValueError(MP_ERROR_TEXT(\"buffer_size out of range\"));\n"
            "    }",
        ),
        (
            "    state->buffer_size = buffer_size;\n"
            "    esp_exceptions(wifi_csi_enable(state));",
            "    state->buffer_size = buffer_size;\n"
            "    state->max_data_len = max_data_len;\n"
            "    esp_exceptions(wifi_csi_enable(state));",
        ),
        (
            "return MP_OBJ_NEW_SMALL_INT(available / sizeof(csi_frame_t));",
            "return MP_OBJ_NEW_SMALL_INT(available / wifi_csi_record_size(state));",
        ),
    )
    for original, replacement in replacements:
        if original not in source:
            raise RuntimeError(
                f"MicroPython fixed-record CSI anchor is missing: {source_path}"
            )
        source = source.replace(original, replacement, 1)

    if not all(token in source for token in required):
        raise RuntimeError(
            f"MicroPython fixed-record CSI layout is incomplete: {source_path}"
        )
    source_path.write_text(source, encoding="utf-8")


def _configure_project_gc_heap_reserve(micropython_dir: Path) -> None:
    """Prefer PSRAM for split-heap growth without starving native memory."""
    source_path = micropython_dir / "ports" / "esp32" / "gccollect.c"
    source = source_path.read_text(encoding="utf-8")
    psram_aware_split = """size_t gc_get_max_new_split(void) {
    size_t internal_largest = heap_caps_get_largest_free_block(
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t available = internal_largest > ESPECTRE_GC_NATIVE_HEAP_RESERVE
        ? internal_largest - ESPECTRE_GC_NATIVE_HEAP_RESERVE
        : 0;
    #if CONFIG_SPIRAM
    // Large system-malloc requests use PSRAM when ESP-IDF initialized it.
    // Match that allocator policy while retaining the internal-memory reserve.
    size_t external_available = heap_caps_get_largest_free_block(
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (external_available > available) {
        available = external_available;
    }
    #endif
    return available < ESPECTRE_GC_MAX_NEW_SPLIT_SIZE
        ? available
        : ESPECTRE_GC_MAX_NEW_SPLIT_SIZE;
}"""
    if "ESPECTRE_GC_NATIVE_HEAP_RESERVE" in source:
        old_cap = "#define ESPECTRE_GC_MAX_NEW_SPLIT_SIZE (48 * 1024)"
        if old_cap in source:
            source = source.replace(
                old_cap,
                "#define ESPECTRE_GC_MAX_NEW_SPLIT_SIZE (56 * 1024)",
                1,
            )
        previous_reserve = "#define ESPECTRE_GC_NATIVE_HEAP_RESERVE (44 * 1024)"
        if previous_reserve in source:
            source = source.replace(
                previous_reserve,
                "#define ESPECTRE_GC_NATIVE_HEAP_RESERVE (32 * 1024)",
                1,
            )
        if "#ifndef ESPECTRE_GC_MAX_NEW_SPLIT_SIZE" not in source:
            source = source.replace(
                "#define ESPECTRE_GC_MAX_NEW_SPLIT_SIZE (56 * 1024)",
                "#ifndef ESPECTRE_GC_MAX_NEW_SPLIT_SIZE\n"
                "#define ESPECTRE_GC_MAX_NEW_SPLIT_SIZE (56 * 1024)\n"
                "#endif",
                1,
            )
        if "#ifndef ESPECTRE_GC_NATIVE_HEAP_RESERVE" not in source:
            source = source.replace(
                "#define ESPECTRE_GC_NATIVE_HEAP_RESERVE (32 * 1024)",
                "#ifndef ESPECTRE_GC_NATIVE_HEAP_RESERVE\n"
                "#define ESPECTRE_GC_NATIVE_HEAP_RESERVE (32 * 1024)\n"
                "#endif",
                1,
            )
        split_start = source.find("size_t gc_get_max_new_split(void) {")
        split_end = source.find("\n}\n\n#endif", split_start)
        if split_end < 0:
            split_end = source.rfind("\n}")
        if split_start < 0 or split_end < 0:
            raise RuntimeError(
                f"MicroPython split-heap growth function is missing: {source_path}"
            )
        source = (
            source[:split_start]
            + psram_aware_split
            + source[split_end + len("\n}"):]
        )
        source_path.write_text(source, encoding="utf-8")
        if (
            "#define ESPECTRE_GC_MAX_NEW_SPLIT_SIZE (56 * 1024)" not in source
            or "#define ESPECTRE_GC_NATIVE_HEAP_RESERVE (32 * 1024)" not in source
            or "MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT" not in source
            or "MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT" not in source
            or "#if CONFIG_SPIRAM" not in source
        ):
            raise RuntimeError(
                f"MicroPython GC heap-reserve profile is inconsistent: {source_path}"
            )
        return

    original = """// The largest new region that is available to become Python heap is the largest
// free block in the ESP-IDF system heap.
size_t gc_get_max_new_split(void) {
    return heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
}
"""
    replacement = """// Grow the Python heap in bounded regions while retaining one contiguous block
// for Wi-Fi, lwIP, the Direct HTTP server, and its command/SSE sockets.
#ifndef ESPECTRE_GC_MAX_NEW_SPLIT_SIZE
#define ESPECTRE_GC_MAX_NEW_SPLIT_SIZE (56 * 1024)
#endif
#ifndef ESPECTRE_GC_NATIVE_HEAP_RESERVE
#define ESPECTRE_GC_NATIVE_HEAP_RESERVE (32 * 1024)
#endif

""" + psram_aware_split + "\n"
    if original not in source:
        raise RuntimeError(
            f"MicroPython split-heap growth anchor is missing: {source_path}"
        )
    source_path.write_text(source.replace(original, replacement, 1), encoding="utf-8")


def _configure_project_wifi_band_mode(micropython_dir: Path) -> None:
    """Expose the ESP-IDF band selector required by dual-band WLAN targets."""
    source_path = micropython_dir / "ports" / "esp32" / "network_wlan.c"
    source = source_path.read_text(encoding="utf-8")
    setter = """                    case MP_QSTR_band_mode: {
                        wifi_band_mode_t band_mode = mp_obj_get_int(kwargs->table[i].value);
                        esp_exceptions(esp_wifi_set_band_mode(band_mode));
                        if (band_mode == WIFI_BAND_MODE_AUTO) {
                            wifi_protocols_t protocols = {
                                WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
                                WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AC,
                            };
                            wifi_bandwidths_t bandwidths = {WIFI_BW_HT20, WIFI_BW_HT20};
                            esp_exceptions(esp_wifi_set_protocols(self->if_id, &protocols));
                            esp_exceptions(esp_wifi_set_bandwidths(self->if_id, &bandwidths));
                        }
                        break;
                    }
"""
    band_constants = """    { MP_ROM_QSTR(MP_QSTR_BAND_MODE_2G_ONLY), MP_ROM_INT(WIFI_BAND_MODE_2G_ONLY) },
    { MP_ROM_QSTR(MP_QSTR_BAND_MODE_AUTO), MP_ROM_INT(WIFI_BAND_MODE_AUTO) },
"""
    if setter in source and band_constants in source:
        return

    bandwidth_setter = """                    case MP_QSTR_bandwidth: {
                        esp_exceptions(esp_wifi_set_bandwidth(self->if_id, mp_obj_get_int(kwargs->table[i].value)));
                        break;
                    }
"""
    bandwidth_constant = """    { MP_ROM_QSTR(MP_QSTR_BANDWIDTH_20), MP_ROM_INT(WIFI_BW20) },
"""
    if setter not in source and bandwidth_setter not in source:
        raise RuntimeError(
            f"MicroPython WLAN band-mode setter anchor is missing: {source_path}"
        )
    if band_constants not in source and bandwidth_constant not in source:
        raise RuntimeError(
            f"MicroPython WLAN band-mode constant anchor is missing: {source_path}"
        )

    if setter not in source:
        source = source.replace(
            bandwidth_setter,
            setter + bandwidth_setter,
            1,
        )
    if band_constants not in source:
        source = source.replace(
            bandwidth_constant,
            band_constants + bandwidth_constant,
            1,
        )
    source_path.write_text(source, encoding="utf-8")


def _configure_project_s2_usb_serial(micropython_dir: Path) -> None:
    """Keep the ESP32-S2 TinyUSB path stable across ROM bootloader resets."""
    source_path = micropython_dir / "ports" / "esp32" / "usb.c"
    source = source_path.read_text(encoding="utf-8")
    original = """void mp_usbd_port_get_serial_number(char *serial_buf) {
    // use factory default MAC as serial ID
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    MP_STATIC_ASSERT(sizeof(mac) * 2 <= MICROPY_HW_USB_DESC_STR_MAX);
    mp_usbd_hex_str(serial_buf, mac, sizeof(mac));
}
"""
    replacement = """void mp_usbd_port_get_serial_number(char *serial_buf) {
    #if CONFIG_IDF_TARGET_ESP32S2
    // Match the ROM CDC serial so macOS preserves the device path after reset.
    serial_buf[0] = '0';
    serial_buf[1] = '\\0';
    #else
    // use factory default MAC as serial ID
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    MP_STATIC_ASSERT(sizeof(mac) * 2 <= MICROPY_HW_USB_DESC_STR_MAX);
    mp_usbd_hex_str(serial_buf, mac, sizeof(mac));
    #endif
}
"""
    if replacement in source:
        return
    if original not in source:
        raise RuntimeError(
            f"MicroPython USB serial-number anchor is missing: {source_path}"
        )
    source_path.write_text(source.replace(original, replacement, 1), encoding="utf-8")


def _configure_project_wifi_channel_pin(micropython_dir: Path) -> None:
    """Allow a BSSID pin to carry its known channel into ESP-IDF."""
    source_path = micropython_dir / "ports" / "esp32" / "network_wlan.c"
    source = source_path.read_text(encoding="utf-8")
    enum_original = "enum { ARG_ssid, ARG_key, ARG_bssid };"
    enum_patched = "enum { ARG_ssid, ARG_key, ARG_bssid, ARG_channel };"
    bssid_arg = "        { MP_QSTR_bssid, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },\n"
    channel_arg = "        { MP_QSTR_channel, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },\n"
    bssid_block_end = """            memcpy(wifi_sta_config.sta.bssid, p, sizeof(wifi_sta_config.sta.bssid));
        }
"""
    channel_block = """        if (args[ARG_channel].u_int < 0 || args[ARG_channel].u_int > 255) {
            mp_raise_ValueError(MP_ERROR_TEXT("channel out of range"));
        }
        wifi_sta_config.sta.channel = args[ARG_channel].u_int;
"""
    if enum_patched in source and channel_arg in source and channel_block in source:
        return
    if enum_original not in source or bssid_arg not in source or bssid_block_end not in source:
        raise RuntimeError(
            f"MicroPython WLAN channel-pin anchor is missing: {source_path}"
        )
    source = source.replace(enum_original, enum_patched, 1)
    source = source.replace(bssid_arg, bssid_arg + channel_arg, 1)
    source = source.replace(bssid_block_end, bssid_block_end + channel_block, 1)
    source_path.write_text(source, encoding="utf-8")


def _write_manifest(manifest_path: Path) -> None:
    """Freeze only the ESP32 boot and filesystem helpers, never the application."""
    manifest_path.write_text(
        'freeze("$(PORT_DIR)/modules", ("_boot.py", "flashbdev.py", "inisetup.py"))\n',
        encoding="utf-8",
    )


def _stage_firmware_support(source_dir: Path, destination: Path) -> None:
    """Stage the firmware build support used by the project image."""
    support_dir = source_dir / "firmware"
    if not support_dir.is_dir():
        raise RuntimeError(f"Project firmware support directory is missing: {support_dir}")
    if destination.exists():
        shutil.rmtree(destination)
    shutil.copytree(support_dir, destination)


def _cmake_cache_value(cache_path: Path, name: str) -> str | None:
    """Return one value from a CMake cache without interpreting its type."""
    if not cache_path.is_file():
        return None
    prefix = f"{name}:"
    for line in cache_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith(prefix) and "=" in line:
            return line.split("=", 1)[1]
    return None


def _local_build_cache_matches_toolchain(
    build_dir: Path,
    micropython_dir: Path,
    idf_path: Path,
) -> bool:
    """Reject incremental CMake state created with another source or ESP-IDF."""
    expected_project = (micropython_dir / "ports" / "esp32").resolve()
    expected_bootloader = (idf_path / "components" / "bootloader" / "subproject").resolve()
    checks = (
        (build_dir / "CMakeCache.txt", "CMAKE_HOME_DIRECTORY", expected_project),
        (build_dir / "bootloader" / "CMakeCache.txt", "CMAKE_HOME_DIRECTORY", expected_bootloader),
        (build_dir / "bootloader" / "CMakeCache.txt", "IDF_PATH", idf_path.resolve()),
    )
    for cache_path, name, expected in checks:
        cached = _cmake_cache_value(cache_path, name)
        if cached is not None and Path(cached).expanduser().resolve() != expected:
            return False
    return True


def _hash_file_tree(root: Path, extra: bytes = b"") -> str:
    """Return a stable digest of every file under one firmware support tree."""
    digest = hashlib.sha256()
    digest.update(extra)
    if root.is_dir():
        for path in sorted(root.rglob("*")):
            if not path.is_file():
                continue
            digest.update(path.relative_to(root).as_posix().encode("utf-8"))
            digest.update(b"\0")
            digest.update(path.read_bytes())
            digest.update(b"\0")
    return digest.hexdigest()


def _firmware_kconfig_profile(source_dir: Path, chip: str) -> str:
    """Identify the MicroPython board Kconfig inputs used for generated sdkconfig."""
    board_profile = _hash_file_tree(
        source_dir / "firmware" / "boards",
        extra=f"{chip}\n{MICROPYTHON_PATCH_REVISION}\n".encode("utf-8"),
    )
    scheduling_kconfig = (
        source_dir
        / "firmware"
        / "components"
        / "espectre_core"
        / "Kconfig.projbuild"
    )
    digest = hashlib.sha256(board_profile.encode("ascii"))
    if scheduling_kconfig.is_file():
        digest.update(scheduling_kconfig.read_bytes())
    return digest.hexdigest()


def _generated_sdkconfig_is_current(build_dir: Path, profile: str) -> bool:
    """Return whether the CMake sdkconfig already matches the current board defaults."""
    sdkconfig = build_dir / "sdkconfig"
    stamp_path = build_dir / ".espectre-kconfig-profile"
    if not sdkconfig.is_file() or not stamp_path.is_file():
        return False
    try:
        return stamp_path.read_text(encoding="utf-8").strip() == profile
    except OSError:
        return False


def _write_kconfig_profile(build_dir: Path, profile: str) -> None:
    """Record the board Kconfig digest after a successful firmware configure."""
    build_dir.mkdir(parents=True, exist_ok=True)
    (build_dir / ".espectre-kconfig-profile").write_text(profile + "\n", encoding="utf-8")


def build_project_firmware(
    source_dir: Path,
    *,
    chip: str = "esp32",
    clean: bool = False,
    cache_dir: Path = FIRMWARE_CACHE_DIR,
    backend: str = "auto",
    pull_policy: str = "ask",
) -> BuiltProjectFirmware:
    """Build a lean project firmware used with filesystem bytecode."""
    cache_dir.mkdir(parents=True, exist_ok=True)
    lock_path = cache_dir / "micro-esp32.lock"
    with lock_path.open("a+", encoding="utf-8") as lock_file:
        # The checkout, staged support tree, and CMake directory are shared by
        # every MicroPython build. Serialise them so parallel chip benchmarks
        # cannot remove inputs while another compiler is reading them.
        if fcntl is not None:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        elif msvcrt is not None:  # pragma: no cover - exercised on Windows
            lock_file.seek(0, os.SEEK_END)
            if lock_file.tell() == 0:
                lock_file.write("\0")
                lock_file.flush()
            lock_file.seek(0)
            msvcrt.locking(lock_file.fileno(), msvcrt.LK_LOCK, 1)
        try:
            return _build_project_firmware_locked(
                source_dir,
                chip=chip,
                clean=clean,
                cache_dir=cache_dir,
                backend=backend,
                pull_policy=pull_policy,
            )
        finally:
            if fcntl is not None:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
            elif msvcrt is not None:  # pragma: no cover - exercised on Windows
                lock_file.seek(0)
                msvcrt.locking(lock_file.fileno(), msvcrt.LK_UNLCK, 1)


def _build_project_firmware_locked(
    source_dir: Path,
    *,
    chip: str,
    clean: bool,
    cache_dir: Path,
    backend: str,
    pull_policy: str,
) -> BuiltProjectFirmware:
    """Build while holding the shared MicroPython workspace lock."""
    board = PROJECT_FIRMWARE_BOARDS.get(chip)
    firmware_name = PROJECT_FIRMWARE_NAMES.get(chip)
    if board is None or firmware_name is None:
        raise ValueError(f"Unsupported project firmware chip: {chip}")

    resolved_backend = resolve_idf_build_backend(backend, pull_policy)

    workspace = cache_dir / "micro-esp32"
    micropython_dir = workspace / "micropython"
    micropython_lib_dir = workspace / "micropython-lib"
    build_dir_name = resolve_idf_build_dir_name(
        workspace,
        chip,
        container=resolved_backend.mode == "docker",
    )
    assert build_dir_name is not None
    build_dir = workspace / build_dir_name
    support_root = workspace / "firmware-support"
    manifest_path = workspace / "manifest.py"

    _checkout_pinned_repository(
        MICROPYTHON_REPOSITORY,
        MICROPYTHON_COMMIT,
        micropython_dir,
    )
    _checkout_pinned_repository(
        MICROPYTHON_LIB_REPOSITORY,
        MICROPYTHON_LIB_COMMIT,
        micropython_lib_dir,
    )
    patch_stamp_path = _prepare_micropython_patch_revision(micropython_dir)
    _configure_project_identity(micropython_dir)
    _configure_project_csi_capture(micropython_dir, chip)
    _configure_project_csi_rearm(micropython_dir)
    _configure_project_csi_fixed_records(micropython_dir)
    _configure_project_csi_quality(micropython_dir)
    _configure_project_gc_heap_reserve(micropython_dir)
    _configure_project_wifi_channel_pin(micropython_dir)
    _configure_project_s2_usb_serial(micropython_dir)
    if chip == "c5":
        _configure_project_wifi_band_mode(micropython_dir)
    patch_stamp_path.write_text(MICROPYTHON_PATCH_REVISION + "\n", encoding="utf-8")
    _stage_firmware_support(source_dir, support_root)
    _write_manifest(manifest_path)

    idf_path: Path | None = None
    if resolved_backend.mode == "local":
        idf_environment = resolved_backend.idf_environment
        assert idf_environment is not None
        idf_path = idf_environment.install_dir
        if idf_path is None and idf_environment.idf_path_entry:
            idf_path = Path(idf_environment.idf_path_entry).resolve().parents[1]
        if idf_path is None:
            raise RuntimeError("The resolved local ESP-IDF path is unavailable")
        idf_version = _read_idf_version(idf_path)
    else:
        idf_version = IDF_VERSION

    incompatible_cache = False
    if resolved_backend.mode == "local" and build_dir.exists():
        assert idf_path is not None
        incompatible_cache = not _local_build_cache_matches_toolchain(
            build_dir,
            micropython_dir,
            idf_path,
        )
    kconfig_profile = _firmware_kconfig_profile(source_dir, chip)
    if (clean or incompatible_cache) and build_dir.exists():
        shutil.rmtree(build_dir)
    elif not _generated_sdkconfig_is_current(build_dir, kconfig_profile):
        # sdkconfig defaults are copied from the repository support tree. Refresh
        # the generated file only when those defaults changed, so an incremental
        # CMake build can reuse a matching configuration.
        for generated_config in ("sdkconfig", "sdkconfig.old"):
            (build_dir / generated_config).unlink(missing_ok=True)
        (build_dir / ".espectre-kconfig-profile").unlink(missing_ok=True)

    _align_idf_lockfile(micropython_dir, chip, idf_version)
    jobs = str(max(1, min(8, os.cpu_count() or 1)))
    if resolved_backend.mode == "docker":
        build_root = Path("/work") / workspace.resolve().relative_to(REPO_ROOT.resolve())
        core_sdk_root = Path("/work/src/cpp")
    else:
        build_root = workspace.resolve()
        core_sdk_root = (REPO_ROOT / "src" / "cpp").resolve()
    core_component_dir = (
        build_root / "firmware-support" / "components" / "espectre_core"
    )
    traffic_component_dir = (
        build_root / "firmware-support" / "components" / "espectre_runtime_traffic"
    )
    extra_component_dirs = ";".join(
        (str(core_component_dir), str(traffic_component_dir))
    )
    core_build_environment = {
        "ESPECTRE_CORE_SDK_ROOT": str(core_sdk_root),
    }
    commands = [
        ["make", "-C", "micropython/mpy-cross", f"-j{jobs}"],
        [
            "cmake",
            "-S",
            "micropython/ports/esp32",
            "-B",
            build_dir.name,
            "-G",
            "Ninja",
            f"-DMICROPY_BOARD={board}",
            f"-DMICROPY_BOARD_DIR={build_root / 'firmware-support' / 'boards' / board}",
            f"-DMICROPY_FROZEN_MANIFEST={build_root / 'manifest.py'}",
            f"-DMICROPY_LIB_DIR={build_root / 'micropython-lib'}",
            f"-DUSER_C_MODULES={build_root / 'firmware-support' / 'native_components' / 'micropython.cmake'}",
            f"-DESPECTRE_CORE_SDK_ROOT={core_sdk_root}",
            f"-DEXTRA_COMPONENT_DIRS={extra_component_dirs}",
            "-DMICROPY_PY_BTREE=0",
        ],
        [
            "cmake",
            "--build",
            build_dir.name,
            f"-j{jobs}",
        ],
        [
            "python",
            "micropython/ports/esp32/makeimg.py",
            f"{build_dir.name}/sdkconfig",
            f"{build_dir.name}/bootloader/bootloader.bin",
            f"{build_dir.name}/partition_table/partition-table.bin",
            f"{build_dir.name}/{PROJECT_FIRMWARE_PROJECT_NAME}.bin",
            f"{build_dir.name}/firmware.bin",
            f"{build_dir.name}/firmware.uf2",
        ],
    ]
    if resolved_backend.mode == "docker":
        run_toolchain_container(
            frontend="micro",
            workdir=workspace,
            commands=commands,
            repo_root=REPO_ROOT,
            pull_policy=pull_policy,
            docker=resolved_backend.docker,
            environment=core_build_environment,
        )
    else:
        assert resolved_backend.idf_environment is not None
        process_env = dict(
            resolved_backend.idf_environment.process_env or os.environ
        )
        process_env.update(core_build_environment)
        idf_environment = replace(
            resolved_backend.idf_environment,
            process_env=process_env,
        )
        for command in commands:
            run_in_idf_environment(
                command,
                idf_environment,
                cwd=workspace,
            )

    firmware_path = cache_dir / firmware_name
    shutil.copy2(build_dir / "firmware.bin", firmware_path)
    _write_kconfig_profile(build_dir, kconfig_profile)
    return BuiltProjectFirmware(image=firmware_path, build_dir=build_dir)
