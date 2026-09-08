# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
Micro-ESPectre - Device Utils

Minimal allocation-conscious helpers used by the MicroPython runtime.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

try:
    from src.serial_sequence import SerialSequenceTracker
except ImportError:
    from serial_sequence import SerialSequenceTracker

HT20_CSI_LEN = 128
HT20_CSI_LEN_DOUBLE = 256
LLTF20_CSI_LEN_SHORT = 106
HT20_CSI_LEN_SHORT = 114
HT20_CSI_LEN_SHORT_DOUBLE = 228
HT20_CSI_SHORT_LEFT_PAD = 8
HT20_CSI_SHORT_COPY_END = HT20_CSI_SHORT_LEFT_PAD + HT20_CSI_LEN_SHORT
HT20_CSI_SHORT_RIGHT_PAD = HT20_CSI_LEN - HT20_CSI_SHORT_COPY_END
HT20_CSI_SHORT_LEFT_ZEROS = b"\x00" * HT20_CSI_SHORT_LEFT_PAD
HT20_CSI_SHORT_RIGHT_ZEROS = b"\x00" * HT20_CSI_SHORT_RIGHT_PAD
HT20_NUM_SUBCARRIERS = 64
HT20_CSI_HALF_LEN = HT20_CSI_LEN // 2
# Wi-Fi 6 parts deliver HT20 CSI centered on DC (bin = subcarrier + 32), classic
# MACs deliver Espressif's native "0~31, -32~-1" order with DC in bin 0.
# DEFAULT_SUBCARRIERS assumes the centered convention, so classic payloads have to
# be rotated before the band selects the same physical subcarriers on every chip.
# The layouts are told apart by their guard nulls, which the radio reports as
# exactly zero; bins 0 and 32 are null under both conventions and carry no
# information, so only the bins null under exactly one layout are checked.
HT20_CLASSIC_ONLY_NULL_BINS = (29, 30, 31, 33, 34, 35)
HT20_CENTERED_ONLY_NULL_BINS = (1, 2, 3, 61, 62, 63)
HT20_LLTF_DETECTOR_BIN_SOURCES = ((4, 6), (5, 6), (59, 58), (60, 58))
LAYOUT_BINS_UNKNOWN = "unknown"
LAYOUT_BINS_CENTERED = "centered"
LAYOUT_BINS_CLASSIC = "classic"
FORMAT_ID_UNKNOWN = "unknown"
FORMAT_ID_HT20 = "ht20"
LAYOUT_ID_UNKNOWN = "unknown"
LAYOUT_ID_HT20_64 = "ht20_64"
LAYOUT_ID_HT20_57 = "ht20_57"
LAYOUT_ID_LLTF20_53 = "lltf20_53"
LAYOUT_ID_HT20_64_DOUBLE = "ht20_64_double"
LAYOUT_ID_HT20_57_DOUBLE = "ht20_57_double"


def select_csi_capture_profile(chip, wifi_channel):
    """Return the runtime-owned 20 MHz CSI profile for this association."""
    normalized_chip = str(chip or "").upper().replace("ESP32-", "")
    if normalized_chip.startswith("ESP32") and normalized_chip != "ESP32":
        normalized_chip = normalized_chip[5:]
    try:
        wifi_channel = int(wifi_channel or 0)
    except (TypeError, ValueError):
        wifi_channel = 0
    if normalized_chip in ("ESP32", "S2"):
        return "lltf20"
    if normalized_chip == "C5" and wifi_channel > 14:
        return "vht20"
    return "ht20"


PAYLOAD_VIEW_RAW = "raw"
PAYLOAD_VIEW_NORMALIZED = "normalized"
METADATA_SOURCE_WIFI = "wifi_rx_ctrl"
METADATA_SOURCE_EXPLICIT = "explicit"
METADATA_SOURCE_HISTORICAL = "historical_missing_phy"
DISPOSITION_DROP = "drop"
DISPOSITION_SENSE = "sense"
REASON_NONE = "none"
REASON_NULL_OR_EMPTY = "null_or_empty"
REASON_BAD_LENGTH = "bad_length"
REASON_UNSUPPORTED_PHY = "unsupported_phy"
REASON_UNSUPPORTED_WIDTH = "unsupported_width"
REASON_UNEXPECTED_LTF = "unexpected_ltf"
REASON_UNKNOWN_LAYOUT = "unknown_layout"
REASON_MISSING_METADATA = "missing_metadata"
REASON_RX_ERROR = "rx_error"
NORMALIZATION_DOUBLE_HT20 = "double_ht20"
NORMALIZATION_HT57_TO_64 = "ht57_to_64"
NORMALIZATION_LLTF53_TO_64 = "lltf53_to_64"
NORMALIZATION_DOUBLE_HT57_TO_64 = "double_ht57_to_64"
DETECTOR_RESET_DROP_STREAK = 8

_CSI_READ_SUPPORTS_REUSE = None


class CsiPayloadNormalizationState:
    """Reusable metadata state for lower-allocation HT20 normalization."""

    __slots__ = ("assessment", "bin_layout")

    def __init__(self):
        self.assessment = {}
        self.bin_layout = LAYOUT_BINS_UNKNOWN

    def reset(self):
        self.bin_layout = LAYOUT_BINS_UNKNOWN


class CsiFrameTimestampFilter:
    """Reject duplicate or stale CSI frames using the Wi-Fi RX timestamp."""

    def __init__(self):
        self._tracker = SerialSequenceTracker()

    def accept(self, frame):
        if frame is None or len(frame) <= 4:
            return True
        timestamp = frame[4]
        if timestamp is None or timestamp == 0:
            return True
        try:
            return self._tracker.observe(timestamp) >= 0
        except (TypeError, ValueError):
            return True

    def reset(self):
        self._tracker.reset()


def csi_read_frame(wlan, reuse_frame=None):
    """Read one CSI frame, reusing the previous result when supported."""
    global _CSI_READ_SUPPORTS_REUSE

    if _CSI_READ_SUPPORTS_REUSE is not False:
        try:
            frame = wlan.csi_read(reuse_frame)
            _CSI_READ_SUPPORTS_REUSE = True
            return frame
        except TypeError:
            _CSI_READ_SUPPORTS_REUSE = False

    return wlan.csi_read()


def build_csi_format_assessment(
    *,
    format_id=FORMAT_ID_UNKNOWN,
    layout_id=LAYOUT_ID_UNKNOWN,
    metadata_source=METADATA_SOURCE_WIFI,
    payload_view=PAYLOAD_VIEW_RAW,
    disposition=DISPOSITION_DROP,
    reason_code=REASON_BAD_LENGTH,
    normalization_id=None,
    raw_len=0,
    raw_num_subcarriers=0,
    normalized_len=0,
    normalized_num_subcarriers=0,
    out=None,
):
    """Build one MicroPython-friendly CSI format assessment mapping.

    Pass a previously returned mapping as ``out`` to refill it in place; hot
    device loops reuse one assessment dict to avoid per-frame allocations.
    """
    assessment = out if out is not None else {}
    assessment["format_id"] = format_id
    assessment["layout_id"] = layout_id
    assessment["metadata_source"] = metadata_source
    assessment["payload_view"] = payload_view
    assessment["disposition"] = disposition
    assessment["reason_code"] = reason_code
    assessment["normalization_id"] = normalization_id
    assessment["raw_len"] = int(raw_len)
    assessment["raw_num_subcarriers"] = int(raw_num_subcarriers)
    assessment["normalized_len"] = int(normalized_len)
    assessment["normalized_num_subcarriers"] = int(normalized_num_subcarriers)
    assessment["reset_detector_before_consume"] = False
    return assessment


def is_ht20_sensing_phy_fields(sig_mode, cwb):
    """Return True when PHY metadata matches the HT20 sensing contract."""
    return sig_mode == 1 and cwb == 0


def assess_ht20_sensing_phy(sig_mode=None, cwb=None, *, metadata_missing=False,
                            allow_legacy_lltf=False, out=None):
    """Assess whether the observed PHY metadata allows HT20 sensing."""
    if metadata_missing:
        return build_csi_format_assessment(
            format_id=FORMAT_ID_HT20,
            metadata_source=METADATA_SOURCE_HISTORICAL,
            disposition=DISPOSITION_SENSE,
            reason_code=REASON_NONE,
            out=out,
        )
    if sig_mode != 1 and not (allow_legacy_lltf and sig_mode == 0 and cwb == 0):
        return build_csi_format_assessment(
            metadata_source=METADATA_SOURCE_WIFI,
            reason_code=REASON_UNSUPPORTED_PHY,
            out=out,
        )
    if cwb != 0:
        return build_csi_format_assessment(
            metadata_source=METADATA_SOURCE_WIFI,
            reason_code=REASON_UNSUPPORTED_WIDTH,
            out=out,
        )
    return build_csi_format_assessment(
        format_id=FORMAT_ID_HT20,
        metadata_source=METADATA_SOURCE_WIFI,
        disposition=DISPOSITION_SENSE,
        reason_code=REASON_NONE,
        out=out,
    )


def assess_ht20_payload_layout(raw_len, *, expected_len=HT20_CSI_LEN, out=None):
    """Assess one CSI payload layout against the supported HT20 contract."""
    if raw_len <= 0:
        return build_csi_format_assessment(reason_code=REASON_BAD_LENGTH, raw_len=raw_len, out=out)
    if raw_len % 2:
        return build_csi_format_assessment(reason_code=REASON_BAD_LENGTH, raw_len=raw_len, out=out)
    if expected_len != HT20_CSI_LEN:
        if raw_len == expected_len:
            return build_csi_format_assessment(
                format_id=FORMAT_ID_HT20,
                layout_id=LAYOUT_ID_HT20_64,
                disposition=DISPOSITION_SENSE,
                reason_code=REASON_NONE,
                raw_len=raw_len,
                raw_num_subcarriers=raw_len // 2,
                normalized_len=expected_len,
                normalized_num_subcarriers=expected_len // 2,
                out=out,
            )
        return build_csi_format_assessment(
            reason_code=REASON_UNKNOWN_LAYOUT,
            raw_len=raw_len,
            raw_num_subcarriers=raw_len // 2,
            out=out,
        )

    raw_num_subcarriers = raw_len // 2
    if raw_len == HT20_CSI_LEN:
        layout_id = LAYOUT_ID_HT20_64
        payload_view = PAYLOAD_VIEW_RAW
        normalization_id = None
    elif raw_len == LLTF20_CSI_LEN_SHORT:
        layout_id = LAYOUT_ID_LLTF20_53
        payload_view = PAYLOAD_VIEW_NORMALIZED
        normalization_id = NORMALIZATION_LLTF53_TO_64
    elif raw_len == HT20_CSI_LEN_SHORT:
        layout_id = LAYOUT_ID_HT20_57
        payload_view = PAYLOAD_VIEW_NORMALIZED
        normalization_id = NORMALIZATION_HT57_TO_64
    elif raw_len == HT20_CSI_LEN_DOUBLE:
        layout_id = LAYOUT_ID_HT20_64_DOUBLE
        payload_view = PAYLOAD_VIEW_NORMALIZED
        normalization_id = NORMALIZATION_DOUBLE_HT20
    elif raw_len == HT20_CSI_LEN_SHORT_DOUBLE:
        layout_id = LAYOUT_ID_HT20_57_DOUBLE
        payload_view = PAYLOAD_VIEW_NORMALIZED
        normalization_id = NORMALIZATION_DOUBLE_HT57_TO_64
    else:
        return build_csi_format_assessment(
            format_id=FORMAT_ID_HT20,
            reason_code=REASON_UNKNOWN_LAYOUT,
            raw_len=raw_len,
            raw_num_subcarriers=raw_num_subcarriers,
            out=out,
        )
    return build_csi_format_assessment(
        format_id=FORMAT_ID_HT20,
        layout_id=layout_id,
        payload_view=payload_view,
        normalization_id=normalization_id,
        disposition=DISPOSITION_SENSE,
        reason_code=REASON_NONE,
        raw_len=raw_len,
        raw_num_subcarriers=raw_num_subcarriers,
        normalized_len=HT20_CSI_LEN,
        normalized_num_subcarriers=HT20_NUM_SUBCARRIERS,
        out=out,
    )


# Scratch mappings for the per-frame classification path. Device code is
# single-threaded and the intermediates never escape this module, so reusing
# them keeps the hot CSI loop free of per-frame dict allocations.
_FRAME_PHY_SCRATCH = {}
_FRAME_LAYOUT_SCRATCH = {}
_HT20_FULL_WIFI_ASSESSMENT = {
    "format_id": FORMAT_ID_HT20,
    "layout_id": LAYOUT_ID_HT20_64,
    "metadata_source": METADATA_SOURCE_WIFI,
    "payload_view": PAYLOAD_VIEW_RAW,
    "disposition": DISPOSITION_SENSE,
    "reason_code": REASON_NONE,
    "normalization_id": None,
    "raw_len": HT20_CSI_LEN,
    "raw_num_subcarriers": HT20_NUM_SUBCARRIERS,
    "normalized_len": HT20_CSI_LEN,
    "normalized_num_subcarriers": HT20_NUM_SUBCARRIERS,
    "reset_detector_before_consume": False,
}
_HT20_FULL_HISTORICAL_ASSESSMENT = dict(_HT20_FULL_WIFI_ASSESSMENT)
_HT20_FULL_HISTORICAL_ASSESSMENT["metadata_source"] = METADATA_SOURCE_HISTORICAL


def assess_ht20_sensing_frame(frame, csi_data, *, expected_len=HT20_CSI_LEN,
                              metadata_missing=False, allow_legacy_lltf=False,
                              out=None,
                              static_fast_path=False):
    """Classify one MicroPython CSI frame before normalization.

    Pass a reusable mapping as ``out`` in hot loops to avoid per-frame
    allocations. The returned assessment is ``out`` itself unless
    ``static_fast_path`` selects a private, shared, read-only mapping for a
    common full-width frame. Callers must never mutate that shared result.
    """
    if frame is None:
        return build_csi_format_assessment(reason_code=REASON_NULL_OR_EMPTY, out=out)
    try:
        frame_len = len(frame)
    except TypeError:
        frame_len = 0
    try:
        raw_len = len(csi_data)
    except TypeError:
        raw_len = 0

    if frame_len > 21 and frame[21] != 0:
        return build_csi_format_assessment(reason_code=REASON_RX_ERROR, raw_len=raw_len, out=out)

    # The live device path overwhelmingly receives full-width HT20 frames.
    # Reuse a private module mapping instead of rebuilding PHY, layout, and
    # combined mappings field by field for every CSI callback.
    if raw_len == HT20_CSI_LEN and expected_len == HT20_CSI_LEN:
        if frame_len <= 9 or metadata_missing:
            if static_fast_path:
                return _HT20_FULL_HISTORICAL_ASSESSMENT
            assessment = out if out is not None else {}
            assessment.update(_HT20_FULL_HISTORICAL_ASSESSMENT)
            return assessment
        if is_ht20_sensing_phy_fields(frame[7], frame[9]) or (
                allow_legacy_lltf and frame[7] == 0 and frame[9] == 0):
            if static_fast_path:
                return _HT20_FULL_WIFI_ASSESSMENT
            assessment = out if out is not None else {}
            assessment.update(_HT20_FULL_WIFI_ASSESSMENT)
            return assessment

    if frame_len <= 9 or metadata_missing:
        phy_assessment = assess_ht20_sensing_phy(
            metadata_missing=True, out=_FRAME_PHY_SCRATCH
        )
    else:
        phy_assessment = assess_ht20_sensing_phy(
            frame[7], frame[9], metadata_missing=False,
            allow_legacy_lltf=allow_legacy_lltf, out=_FRAME_PHY_SCRATCH
        )
    layout_assessment = assess_ht20_payload_layout(
        raw_len, expected_len=expected_len, out=_FRAME_LAYOUT_SCRATCH
    )
    assessment = build_csi_format_assessment(
        format_id=phy_assessment["format_id"],
        layout_id=layout_assessment["layout_id"],
        metadata_source=phy_assessment["metadata_source"],
        payload_view=layout_assessment["payload_view"],
        disposition=phy_assessment["disposition"],
        reason_code=phy_assessment["reason_code"],
        normalization_id=layout_assessment["normalization_id"],
        raw_len=layout_assessment["raw_len"],
        raw_num_subcarriers=layout_assessment["raw_num_subcarriers"],
        normalized_len=layout_assessment["normalized_len"],
        normalized_num_subcarriers=layout_assessment["normalized_num_subcarriers"],
        out=out,
    )
    if assessment["reason_code"] != REASON_NONE:
        return assessment
    if raw_len == LLTF20_CSI_LEN_SHORT and not (
            allow_legacy_lltf and not metadata_missing and frame_len > 9
            and frame[7] == 0 and frame[9] == 0):
        assessment["disposition"] = DISPOSITION_DROP
        assessment["reason_code"] = REASON_UNEXPECTED_LTF
        return assessment
    if layout_assessment["reason_code"] != REASON_NONE:
        assessment["disposition"] = DISPOSITION_DROP
        assessment["reason_code"] = layout_assessment["reason_code"]
        return assessment
    return assessment


def is_ht20_sensing_frame(frame):
    """Return True when a CSI frame matches the HT20 sensing contract.

    MicroPython ``wlan.csi_read()`` list layout:
    index 7 = ``sig_mode`` (0=legacy, 1=HT, 3=VHT), index 9 = ``cwb``
    (0=20 MHz, 1=40 MHz). Frames without those fields are treated as HT20 for
    older firmware compatibility (same as host NPZ without PHY metadata).
    """
    if frame is None:
        return False
    try:
        frame_len = len(frame)
    except TypeError:
        return False
    if frame_len <= 9:
        return True
    return is_ht20_sensing_phy_fields(frame[7], frame[9])


def _ht20_bins_with_energy(csi_data, bins, first_word_invalid=False):
    populated = 0
    for bin_index in bins:
        if first_word_invalid and bin_index < 2:
            continue
        byte_index = bin_index * 2
        if csi_data[byte_index] != 0 or csi_data[byte_index + 1] != 0:
            populated += 1
    return populated


def detect_ht20_bin_layout(csi_data, expected_len=HT20_CSI_LEN, first_word_invalid=False):
    """Identify which HT20 bin ordering a full-width payload uses.

    Requires positive evidence in both directions: one guard set entirely null
    and the other entirely populated. Absence of energy alone is not enough,
    because a sparse or degenerate payload is null under both conventions.
    """
    try:
        if len(csi_data) != expected_len or expected_len != HT20_CSI_LEN:
            return LAYOUT_BINS_UNKNOWN
    except TypeError:
        return LAYOUT_BINS_UNKNOWN

    classic_energy = _ht20_bins_with_energy(csi_data, HT20_CLASSIC_ONLY_NULL_BINS)
    centered_energy = _ht20_bins_with_energy(csi_data, HT20_CENTERED_ONLY_NULL_BINS, first_word_invalid)
    if classic_energy == 0 and centered_energy == len(HT20_CENTERED_ONLY_NULL_BINS) - int(first_word_invalid):
        return LAYOUT_BINS_CLASSIC
    if centered_energy == 0 and classic_energy == len(HT20_CLASSIC_ONLY_NULL_BINS):
        return LAYOUT_BINS_CENTERED
    return LAYOUT_BINS_UNKNOWN


def rotate_ht20_classic_to_centered(csi_data, remap_buffer=None):
    """Rotate a classic-order HT20 payload into the centered convention.

    Rotating by half the FFT size is its own inverse, so swapping the payload
    halves maps ``0~31, -32~-1`` onto ``-32~+31``.
    """
    if remap_buffer is None or len(remap_buffer) != HT20_CSI_LEN:
        remap_buffer = bytearray(HT20_CSI_LEN)
    view = memoryview(csi_data)
    remap_buffer[:HT20_CSI_HALF_LEN] = view[HT20_CSI_HALF_LEN:]
    remap_buffer[HT20_CSI_HALF_LEN:] = view[:HT20_CSI_HALF_LEN]
    return remap_buffer


def impute_ht20_lltf_detector_bins(csi_data, detector_buffer=None):
    """Copy the nearest live LLTF edge tones into a detector-only view."""
    try:
        if len(csi_data) != HT20_CSI_LEN:
            return None
    except TypeError:
        return None
    if detector_buffer is None or len(detector_buffer) != HT20_CSI_LEN:
        detector_buffer = bytearray(HT20_CSI_LEN)
    if csi_data is not detector_buffer:
        detector_buffer[:] = csi_data
    for target_bin, source_bin in HT20_LLTF_DETECTOR_BIN_SOURCES:
        target = target_bin * 2
        source = source_bin * 2
        detector_buffer[target] = detector_buffer[source]
        detector_buffer[target + 1] = detector_buffer[source + 1]
    return detector_buffer


def _resolve_ht20_bin_layout_once(payload, expected_len, bin_layout, state):
    resolved = state.bin_layout if state is not None else bin_layout
    if resolved is None:
        resolved = LAYOUT_BINS_UNKNOWN
    if resolved == LAYOUT_BINS_UNKNOWN:
        resolved = detect_ht20_bin_layout(payload, expected_len)
        if state is not None and resolved != LAYOUT_BINS_UNKNOWN:
            state.bin_layout = resolved
    return resolved


def normalize_ht20_csi_payload(csi_data, expected_len=128, remap_buffer=None,
                               bin_layout=None, assessment=None, state=None,
                               first_word_invalid=False):
    """Normalize supported CSI payload layouts to one HT20 payload.

    ``bin_layout`` carries the caller's latched ordering. Detection needs a fully
    populated guard set, so it can be inconclusive on an individual packet;
    passing the last confident answer keeps the stream internally consistent.
    Device loops pass their existing ``assessment`` and a reusable ``state`` to
    avoid rebuilding either object for every packet.
    """
    try:
        input_len = len(csi_data)
    except TypeError:
        return None, 0, None
    if assessment is None:
        assessment = assess_ht20_payload_layout(
            input_len,
            expected_len=expected_len,
            out=state.assessment if state is not None else None,
        )
    raw_len = assessment["raw_len"]
    normalization_id = assessment["normalization_id"]
    if assessment["reason_code"] != REASON_NONE:
        return None, raw_len, None

    if first_word_invalid:
        # The first two pairs of compact or classic-order frames include live
        # tones. Only full-width centered frames can be cleaned losslessly.
        if expected_len != HT20_CSI_LEN or raw_len not in (128, 256):
            return None, raw_len, None
        resolved = detect_ht20_bin_layout(memoryview(csi_data)[:128], first_word_invalid=True)
        if resolved != LAYOUT_BINS_CENTERED:
            return None, raw_len, None
        if remap_buffer is None or len(remap_buffer) != expected_len:
            remap_buffer = bytearray(expected_len)
        remap_buffer[:] = memoryview(csi_data)[:expected_len]
        remap_buffer[:4] = b"\x00" * 4
        if state is not None:
            state.bin_layout = resolved
        return remap_buffer, raw_len, normalization_id

    if normalization_id == NORMALIZATION_DOUBLE_HT20:
        collapsed = memoryview(csi_data)[:expected_len]
        if _resolve_ht20_bin_layout_once(
            collapsed, expected_len, bin_layout, state
        ) == LAYOUT_BINS_CLASSIC:
            # Rotate straight from the source so the collapse and the rotation
            # share one pass and never alias the destination buffer.
            return (
                rotate_ht20_classic_to_centered(collapsed, remap_buffer),
                raw_len,
                NORMALIZATION_DOUBLE_HT20,
            )
        if remap_buffer is not None and len(remap_buffer) == expected_len:
            remap_buffer[:] = collapsed
            return remap_buffer, raw_len, NORMALIZATION_DOUBLE_HT20
        return csi_data[:expected_len], raw_len, NORMALIZATION_DOUBLE_HT20

    if raw_len == expected_len:
        # A full-width payload still has to be checked for bin ordering: classic
        # MACs deliver "0~31, -32~-1" while Wi-Fi 6 parts deliver it centered.
        if _resolve_ht20_bin_layout_once(
            csi_data, expected_len, bin_layout, state
        ) == LAYOUT_BINS_CLASSIC:
            return rotate_ht20_classic_to_centered(csi_data, remap_buffer), raw_len, None
        return csi_data, raw_len, None

    if expected_len != HT20_CSI_LEN:
        return None, raw_len, None

    if normalization_id == NORMALIZATION_LLTF53_TO_64:
        if remap_buffer is None or len(remap_buffer) != expected_len:
            remap_buffer = bytearray(expected_len)
        # C5 compact LLTF is centered: -26..+26, with DC at pair 26.
        # Pad six bins on the left and five on the right; DC lands at bin 32.
        remap_buffer[:12] = b"\x00" * 12
        remap_buffer[118:] = b"\x00" * 10
        remap_buffer[12:118] = csi_data
        return remap_buffer, raw_len, NORMALIZATION_LLTF53_TO_64

    short_double_collapsed = False
    working_len = raw_len
    if normalization_id == NORMALIZATION_DOUBLE_HT57_TO_64:
        csi_data = csi_data[:HT20_CSI_LEN_SHORT]
        working_len = HT20_CSI_LEN_SHORT
        short_double_collapsed = True

    if working_len == HT20_CSI_LEN_SHORT:
        if remap_buffer is None or len(remap_buffer) != expected_len:
            remap_buffer = bytearray(expected_len)

        remap_buffer[:HT20_CSI_SHORT_LEFT_PAD] = HT20_CSI_SHORT_LEFT_ZEROS
        remap_buffer[HT20_CSI_SHORT_COPY_END:] = HT20_CSI_SHORT_RIGHT_ZEROS
        remap_buffer[HT20_CSI_SHORT_LEFT_PAD:HT20_CSI_SHORT_COPY_END] = csi_data
        if short_double_collapsed:
            return remap_buffer, raw_len, NORMALIZATION_DOUBLE_HT57_TO_64
        return remap_buffer, raw_len, NORMALIZATION_HT57_TO_64

    return None, raw_len, None


def to_signed_int8(value):
    """Convert an unsigned byte to its signed int8 value."""
    return value if value < 128 else value - 256


def insertion_sort(arr, n):
    """Sort the first ``n`` elements of a small pre-allocated list in place."""
    for i in range(1, n):
        key = arr[i]
        j = i - 1
        while j >= 0 and arr[j] > key:
            arr[j + 1] = arr[j]
            j -= 1
        arr[j + 1] = key


def calculate_variance(values):
    """Calculate population variance with a numerically stable two-pass loop."""
    n = len(values)
    if n == 0:
        return 0.0

    mean = sum(values) / n
    variance_sum = 0.0
    for value in values:
        diff = value - mean
        variance_sum += diff * diff
    return variance_sum / n
