# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - ESPHome Component

ESPHome component for ESPectre WiFi CSI-based motion detection.
Sensors are defined directly in the component (not as separate platforms).

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from pathlib import Path
import ipaddress
import re

from esphome import git
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import logger, sensor, binary_sensor, button, number, select, switch, wifi
from esphome.components.esp32 import (
    add_idf_component,
    add_idf_sdkconfig_option,
    const as esp32_const,
    get_esp32_variant,
)
from esphome.components.wifi import CONF_BAND_MODE
from esphome.const import (
    CONF_COMPONENTS,
    CONF_ESPHOME,
    CONF_EXTERNAL_COMPONENTS,
    CONF_HARDWARE_UART,
    CONF_ID,
    CONF_LOGGER,
    CONF_PROJECT,
    CONF_REF,
    CONF_SOURCE,
    CONF_TYPE,
    CONF_URL,
    CONF_VERSION,
    CONF_WIFI,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_MOTION,
    UNIT_EMPTY,
    UNIT_PERCENT,
    UNIT_DECIBEL_MILLIWATT,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_PULSE,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    TYPE_GIT,
)
from esphome.core import CORE, CoroPriority, coroutine_with_priority

DEPENDENCIES = ["logger", "wifi"]
AUTO_LOAD = ["sensor", "binary_sensor", "button", "number", "select", "switch", "mdns"]

# Configuration parameters
CONF_DIRECT_API = "direct_api"
CONF_SEGMENTATION_WINDOW_SIZE_MS = "segmentation_window_size_ms"
CONF_CSI_TARGET_PPS = "csi_target_pps"
CONF_CSI_TRAFFIC_MODE = "csi_traffic_mode"
CONF_CSI_TRAFFIC_MULTICAST_GROUP = "csi_traffic_multicast_group"
CONF_EVALUATION_INTERVAL_MS = "evaluation_interval_ms"
CONF_MOTION_ON_HITS = "motion_on_hits"
CONF_MOTION_OFF_HITS = "motion_off_hits"

# Low-pass filter
CONF_LOWPASS_ENABLED = "lowpass_enabled"
CONF_LOWPASS_CUTOFF = "lowpass_cutoff"

# Hampel filter
CONF_HAMPEL_ENABLED = "hampel_enabled"
CONF_HAMPEL_WINDOW = "hampel_window"
CONF_HAMPEL_THRESHOLD = "hampel_threshold"


# Traffic generator mode
CONF_TRAFFIC_GENERATOR_MODE = "traffic_generator_mode"

# Detection algorithm
CONF_DETECTION_ALGORITHM = "detection_algorithm"

# Sensors - defined directly in component
CONF_MOVEMENT_SENSOR = "movement_sensor"
CONF_MOTION_SENSOR = "motion_sensor"
CONF_TRAFFIC_RATE_SENSOR = "traffic_rate_sensor"
CONF_CSI_CALLBACK_RATE_SENSOR = "csi_callback_rate_sensor"
CONF_CSI_ACCEPTED_RATE_SENSOR = "csi_accepted_rate_sensor"
CONF_CSI_ADMITTED_RATE_SENSOR = "csi_admitted_rate_sensor"
CONF_CSI_FILTERED_RATE_SENSOR = "csi_filtered_rate_sensor"
CONF_CSI_MISSING_RATE_SENSOR = "csi_missing_rate_sensor"
CONF_CSI_EXCESS_RATE_SENSOR = "csi_excess_rate_sensor"
CONF_CSI_STALE_RATE_SENSOR = "csi_stale_rate_sensor"
CONF_CSI_OUT_OF_ORDER_RATE_SENSOR = "csi_out_of_order_rate_sensor"
CONF_CSI_OCCUPANCY_SENSOR = "csi_occupancy_sensor"
CONF_WIFI_CHANNEL_SENSOR = "wifi_channel_sensor"
CONF_WIFI_RSSI_SENSOR = "wifi_rssi_sensor"
CONF_DIAGNOSTICS_BUTTON = "diagnostics_button"
CONF_CALIBRATION_ACTIVE_SENSOR = "calibration_active_sensor"

# Number controls
CONF_THRESHOLD_NUMBER = "threshold_number"
CONF_MOTION_ON_HITS_NUMBER = "motion_on_hits_number"
CONF_MOTION_OFF_HITS_NUMBER = "motion_off_hits_number"
CONF_DETECTOR_SELECT = "detector_select"
CONF_CSI_TRAFFIC_MODE_SELECT = "csi_traffic_mode_select"
CONF_TRAFFIC_GENERATOR_MODE_SELECT = "traffic_generator_mode_select"

CONF_SENSING_SWITCH = "sensing_switch"
CONF_RECALIBRATE_BUTTON = "recalibrate_button"

espectre_ns = cg.esphome_ns.namespace("espectre_component")
ESpectreComponent = espectre_ns.class_("ESpectreComponent", cg.Component)
ESpectreThresholdNumber = espectre_ns.class_("ESpectreThresholdNumber", number.Number, cg.Component)
ESpectreMotionHitsNumber = espectre_ns.class_("ESpectreMotionHitsNumber", number.Number, cg.Component)
ESpectreDetectorSelect = espectre_ns.class_("ESpectreDetectorSelect", select.Select, cg.Component)
ESpectreTrafficModeSelect = espectre_ns.class_("ESpectreTrafficModeSelect", select.Select, cg.Component)
ESpectreSensingSwitch = espectre_ns.class_("ESpectreSensingSwitch", switch.Switch, cg.Component)
ESpectreRecalibrateButton = espectre_ns.class_("ESpectreRecalibrateButton", button.Button, cg.Component)
ESpectreDiagnosticsButton = espectre_ns.class_("ESpectreDiagnosticsButton", button.Button, cg.Component)

_LIBRARY_ROOT = Path(__file__).resolve().parents[4]
_COMPONENT_ROOT = Path(__file__).resolve().parent
_SCHEMA_HEADER = _LIBRARY_ROOT / "runtime" / "runtime_sensing_schema.h"
_SCHEMA_DEPENDENCY_HEADERS = (
    _LIBRARY_ROOT / "core" / "detector_types.h",
    _LIBRARY_ROOT / "core" / "filter_config.h",
)
_SCHEMA_CONST_PATTERN = re.compile(
    r"constexpr\s+(?:const char \*const|bool|float|size_t|uint8_t|uint16_t|uint32_t)\s+"
    r"([A-Z][A-Z0-9_]+)\s*=\s*([^;]+);"
)

_WIFI_BAND_POLICY_BY_MODE = {
    "AUTO": "auto",
    "2.4GHZ": "2g",
    "5GHZ": "5g",
}


def _parse_schema_literal(raw_value, constants):
    raw_value = raw_value.strip()
    if raw_value in constants:
        return constants[raw_value]
    if raw_value in ("true", "false"):
        return raw_value == "true"
    if raw_value.startswith('"') and raw_value.endswith('"'):
        return raw_value[1:-1]
    if raw_value.endswith(("f", "F", "u", "U")):
        raw_value = raw_value[:-1]
    if "." in raw_value or "e" in raw_value.lower():
        return float(raw_value)
    return int(raw_value)


def _load_runtime_schema(schema_path: Path):
    constants = {"UINT16_MAX": 65535}
    for path in (*_SCHEMA_DEPENDENCY_HEADERS, schema_path):
        for line in path.read_text(encoding="utf-8").splitlines():
            match = _SCHEMA_CONST_PATTERN.search(line)
            if match is None:
                continue
            constants[match.group(1)] = _parse_schema_literal(match.group(2), constants)
    return {name: value for name, value in constants.items() if name.startswith("RUNTIME_")}


_RUNTIME_SCHEMA = _load_runtime_schema(_SCHEMA_HEADER)

THRESHOLD_MIN = _RUNTIME_SCHEMA["RUNTIME_THRESHOLD_MIN"]
THRESHOLD_MAX = _RUNTIME_SCHEMA["RUNTIME_HIGH_ACCURACY_THRESHOLD_MAX"]
SEGMENTATION_WINDOW_SIZE_MS_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_DEFAULT"]
SEGMENTATION_WINDOW_SIZE_MS_MIN = _RUNTIME_SCHEMA["RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_MIN"]
SEGMENTATION_WINDOW_SIZE_MS_MAX = _RUNTIME_SCHEMA["RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_MAX"]
CSI_TARGET_PPS_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_CSI_TARGET_PPS_DEFAULT"]
CSI_TARGET_PPS_MIN = _RUNTIME_SCHEMA["RUNTIME_CSI_TARGET_PPS_MIN"]
CSI_TARGET_PPS_MAX = _RUNTIME_SCHEMA["RUNTIME_CSI_TARGET_PPS_MAX"]
TRAFFIC_GENERATOR_MODE_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_TRAFFIC_GENERATOR_MODE_DEFAULT_NAME"]
CSI_TRAFFIC_MODE_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_CSI_TRAFFIC_MODE_DEFAULT_NAME"]
CSI_TRAFFIC_MULTICAST_GROUP_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_CSI_TRAFFIC_MULTICAST_GROUP_DEFAULT"]
DETECTION_ALGORITHM_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_DETECTION_ALGORITHM_DEFAULT_NAME"]
EVALUATION_INTERVAL_MS_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_EVALUATION_INTERVAL_MS_DEFAULT"]
EVALUATION_INTERVAL_MS_MIN = _RUNTIME_SCHEMA["RUNTIME_EVALUATION_INTERVAL_MS_MIN"]
EVALUATION_INTERVAL_MS_MAX = _RUNTIME_SCHEMA["RUNTIME_EVALUATION_INTERVAL_MS_MAX"]
MOTION_HITS_MIN = _RUNTIME_SCHEMA["RUNTIME_MOTION_HITS_MIN"]
MOTION_HITS_MAX = _RUNTIME_SCHEMA["RUNTIME_MOTION_HITS_MAX"]
MOTION_ON_HITS_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_MOTION_ON_HITS_DEFAULT"]
MOTION_OFF_HITS_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_MOTION_OFF_HITS_DEFAULT"]
LOWPASS_ENABLED_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_LOWPASS_ENABLED_DEFAULT"]
LOWPASS_CUTOFF_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_LOWPASS_CUTOFF_DEFAULT"]
LOWPASS_CUTOFF_MIN = _RUNTIME_SCHEMA["RUNTIME_LOWPASS_CUTOFF_MIN"]
LOWPASS_CUTOFF_MAX = _RUNTIME_SCHEMA["RUNTIME_LOWPASS_CUTOFF_MAX"]
HAMPEL_ENABLED_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_HAMPEL_ENABLED_DEFAULT"]
HAMPEL_WINDOW_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_HAMPEL_WINDOW_DEFAULT"]
HAMPEL_WINDOW_MIN = _RUNTIME_SCHEMA["RUNTIME_HAMPEL_WINDOW_MIN"]
HAMPEL_WINDOW_MAX = _RUNTIME_SCHEMA["RUNTIME_HAMPEL_WINDOW_MAX"]
HAMPEL_THRESHOLD_DEFAULT = _RUNTIME_SCHEMA["RUNTIME_HAMPEL_THRESHOLD_DEFAULT"]
HAMPEL_THRESHOLD_MIN = _RUNTIME_SCHEMA["RUNTIME_HAMPEL_THRESHOLD_MIN"]
HAMPEL_THRESHOLD_MAX = _RUNTIME_SCHEMA["RUNTIME_HAMPEL_THRESHOLD_MAX"]


def validate_csi_traffic_multicast_group(value):
    value = cv.string(value).strip()
    if not value:
        return ""
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as exc:
        raise cv.Invalid("csi_traffic_multicast_group must be an IPv4 multicast address") from exc
    if not address.is_multicast:
        raise cv.Invalid("csi_traffic_multicast_group must be an IPv4 multicast address")
    return str(address)


CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ESpectreComponent),

    # Keep the local Direct HTTP/SSE/raw CSI surface enabled for compatibility.
    cv.Optional(CONF_DIRECT_API, default=True): cv.boolean,
    
    # Motion detection parameters
    cv.Optional(CONF_SEGMENTATION_WINDOW_SIZE_MS, default=SEGMENTATION_WINDOW_SIZE_MS_DEFAULT): cv.int_range(
        min=SEGMENTATION_WINDOW_SIZE_MS_MIN, max=SEGMENTATION_WINDOW_SIZE_MS_MAX
    ),
    # Positive temporal CSI target; traffic ownership is configured separately.
    cv.Optional(CONF_CSI_TARGET_PPS, default=CSI_TARGET_PPS_DEFAULT): cv.int_range(
        min=CSI_TARGET_PPS_MIN, max=CSI_TARGET_PPS_MAX
    ),
    cv.Optional(CONF_CSI_TRAFFIC_MODE, default=CSI_TRAFFIC_MODE_DEFAULT): cv.one_of(
        "internal", "external", lower=True
    ),
    cv.Optional(CONF_CSI_TRAFFIC_MULTICAST_GROUP, default=CSI_TRAFFIC_MULTICAST_GROUP_DEFAULT): validate_csi_traffic_multicast_group,
    
    # Traffic generator mode: ping (default), DNS over UDP, or DNS over TCP.
    cv.Optional(CONF_TRAFFIC_GENERATOR_MODE, default=TRAFFIC_GENERATOR_MODE_DEFAULT): cv.one_of(
        "ping", "dns", "dns_tcp", "wifi_raw", lower=True
    ),
    
    # Detection profile: Lightweight (default) or High Accuracy.
    # Lightweight uses feature fusion and an adaptive startup threshold.
    # High Accuracy uses the trained MLP for stronger detection quality and generalization.
    cv.Optional(CONF_DETECTION_ALGORITHM, default=DETECTION_ALGORITHM_DEFAULT): cv.one_of(
        "lightweight", "high_accuracy", lower=True
    ),
    cv.Optional(CONF_EVALUATION_INTERVAL_MS, default=EVALUATION_INTERVAL_MS_DEFAULT): cv.int_range(
        min=EVALUATION_INTERVAL_MS_MIN, max=EVALUATION_INTERVAL_MS_MAX
    ),
    cv.Optional(CONF_MOTION_ON_HITS, default=MOTION_ON_HITS_DEFAULT): cv.int_range(
        min=MOTION_HITS_MIN, max=MOTION_HITS_MAX
    ),
    cv.Optional(CONF_MOTION_OFF_HITS, default=MOTION_OFF_HITS_DEFAULT): cv.int_range(
        min=MOTION_HITS_MIN, max=MOTION_HITS_MAX
    ),

    # Low-pass filter for noise reduction (disabled by default)
    cv.Optional(CONF_LOWPASS_ENABLED, default=LOWPASS_ENABLED_DEFAULT): cv.boolean,
    cv.Optional(CONF_LOWPASS_CUTOFF, default=LOWPASS_CUTOFF_DEFAULT): cv.float_range(
        min=LOWPASS_CUTOFF_MIN, max=LOWPASS_CUTOFF_MAX
    ),
    
    # Hampel filter for turbulence outlier removal
    cv.Optional(CONF_HAMPEL_ENABLED, default=HAMPEL_ENABLED_DEFAULT): cv.boolean,
    cv.Optional(CONF_HAMPEL_WINDOW, default=HAMPEL_WINDOW_DEFAULT): cv.int_range(
        min=HAMPEL_WINDOW_MIN, max=HAMPEL_WINDOW_MAX
    ),
    cv.Optional(CONF_HAMPEL_THRESHOLD, default=HAMPEL_THRESHOLD_DEFAULT): cv.float_range(
        min=HAMPEL_THRESHOLD_MIN, max=HAMPEL_THRESHOLD_MAX
    ),
    
    # Sensors - optional with defaults, always created
    cv.Optional(CONF_MOVEMENT_SENSOR, default={"name": "Movement Score"}): sensor.sensor_schema(
        unit_of_measurement=UNIT_EMPTY,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_MOTION_SENSOR, default={"name": "Motion Detected"}): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_MOTION,
    ),
    cv.Optional(CONF_CALIBRATION_ACTIVE_SENSOR, default={"name": "Calibration Active"}): binary_sensor.binary_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:tune-vertical",
    ),

    # On-demand diagnostic entities. The component refreshes its internal
    # sample from the existing sensing callback, but publishes these states
    # only when the diagnostics button is pressed.
    cv.Optional(CONF_TRAFFIC_RATE_SENSOR, default={"name": "Traffic TX Rate"}): sensor.sensor_schema(
        unit_of_measurement="pkt/s",
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:upload-network",
    ),
    cv.Optional(CONF_CSI_CALLBACK_RATE_SENSOR, default={"name": "CSI Callback Rate"}): sensor.sensor_schema(
        unit_of_measurement="pkt/s",
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:access-point",
    ),
    cv.Optional(CONF_CSI_ACCEPTED_RATE_SENSOR, default={"name": "CSI Accepted Rate"}): sensor.sensor_schema(
        unit_of_measurement="pkt/s",
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:check-network",
    ),
    cv.Optional(CONF_CSI_ADMITTED_RATE_SENSOR, default={"name": "CSI Admitted Rate"}): sensor.sensor_schema(
        unit_of_measurement="pkt/s",
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:timeline-check-outline",
    ),
    cv.Optional(CONF_CSI_FILTERED_RATE_SENSOR, default={"name": "CSI Filtered Rate"}): sensor.sensor_schema(
        unit_of_measurement="pkt/s",
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:filter-outline",
    ),
    cv.Optional(CONF_CSI_MISSING_RATE_SENSOR, default={"name": "CSI Missing Slot Rate"}): sensor.sensor_schema(
        unit_of_measurement="slot/s",
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:timeline-minus-outline",
    ),
    cv.Optional(CONF_CSI_EXCESS_RATE_SENSOR, default={"name": "CSI Excess Rate"}): sensor.sensor_schema(
        unit_of_measurement="pkt/s",
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:timeline-plus-outline",
    ),
    cv.Optional(CONF_CSI_STALE_RATE_SENSOR, default={"name": "CSI Stale Rate"}): sensor.sensor_schema(
        unit_of_measurement="pkt/s",
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:timer-sand",
    ),
    cv.Optional(CONF_CSI_OUT_OF_ORDER_RATE_SENSOR, default={"name": "CSI Out-of-order Rate"}): sensor.sensor_schema(
        unit_of_measurement="pkt/s",
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:swap-vertical",
    ),
    cv.Optional(CONF_CSI_OCCUPANCY_SENSOR, default={"name": "CSI Temporal Occupancy"}): sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:view-grid-outline",
    ),
    cv.Optional(CONF_WIFI_CHANNEL_SENSOR, default={"name": "WiFi Channel"}): sensor.sensor_schema(
        accuracy_decimals=0,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:wifi-marker",
    ),
    cv.Optional(CONF_WIFI_RSSI_SENSOR, default={"name": "WiFi RSSI"}): sensor.sensor_schema(
        unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    cv.Optional(CONF_DIAGNOSTICS_BUTTON, default={"name": "Refresh Diagnostics"}): button.button_schema(
        ESpectreDiagnosticsButton,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:refresh",
    ),
    
    # Number control for threshold adjustment from HA
    cv.Optional(CONF_THRESHOLD_NUMBER, default={"name": "Threshold"}): number.number_schema(
        ESpectreThresholdNumber,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_PULSE,
    ),
    cv.Optional(CONF_MOTION_ON_HITS_NUMBER, default={"name": "Motion On Hits"}): number.number_schema(
        ESpectreMotionHitsNumber,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:motion-play-outline",
    ),
    cv.Optional(CONF_MOTION_OFF_HITS_NUMBER, default={"name": "Motion Off Hits"}): number.number_schema(
        ESpectreMotionHitsNumber,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:motion-pause-outline",
    ),

    cv.Optional(CONF_DETECTOR_SELECT, default={"name": "Detection Profile"}): select.select_schema(
        ESpectreDetectorSelect,
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
    cv.Optional(CONF_CSI_TRAFFIC_MODE_SELECT, default={"name": "CSI Traffic Ownership"}): select.select_schema(
        ESpectreTrafficModeSelect,
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
    cv.Optional(CONF_TRAFFIC_GENERATOR_MODE_SELECT, default={"name": "CSI Traffic Source"}): select.select_schema(
        ESpectreTrafficModeSelect,
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
    
    cv.Optional(CONF_SENSING_SWITCH, default={"name": "Sensing Enabled"}): switch.switch_schema(
        ESpectreSensingSwitch,
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
    cv.Optional(CONF_RECALIBRATE_BUTTON, default={"name": "Recalibrate"}): button.button_schema(
        ESpectreRecalibrateButton,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:tune-vertical",
    ),
}).extend(cv.COMPONENT_SCHEMA)


def _runtime_wifi_band_policy():
    if get_esp32_variant() != esp32_const.VARIANT_ESP32C5:
        return "2g"

    # ESPHome defaults ESP32-C5 to AUTO when wifi.band_mode is omitted.
    band_mode = str(CORE.config[CONF_WIFI].get(CONF_BAND_MODE, "AUTO"))
    return _WIFI_BAND_POLICY_BY_MODE[band_mode]


def _uses_tinyusb_primary_console():
    """Return whether the selected CDC console lacks reset-capable USB JTAG."""
    logger_config = CORE.config[CONF_LOGGER]
    if logger_config[CONF_HARDWARE_UART] != logger.USB_CDC:
        return False
    try:
        logger.uart_selection(logger.USB_SERIAL_JTAG)
    except cv.Invalid:
        return True
    return False


@coroutine_with_priority(CoroPriority.WORKAROUNDS)
async def _configure_tinyusb_primary_console():
    """Override the ESPHome logger's ROM CDC selection after component setup."""
    add_idf_sdkconfig_option("CONFIG_ESP_CONSOLE_USB_CDC", False)
    add_idf_sdkconfig_option("CONFIG_ESP_CONSOLE_NONE", True)
    add_idf_sdkconfig_option("CONFIG_ESPECTRE_TINYUSB_PRIMARY_CONSOLE", True)
    add_idf_sdkconfig_option("CONFIG_TINYUSB_CDC_ENABLED", True)
    # Match the ESP32-S2 ROM CDC serial so macOS preserves the device path.
    add_idf_sdkconfig_option("CONFIG_TINYUSB_DESC_SERIAL_STRING", "0")


def _frontend_ref_version() -> str | None:
    """Use a numeric ref only from the checkout that supplied this component."""
    for external in CORE.config.get(CONF_EXTERNAL_COMPONENTS, []):
        components = external.get(CONF_COMPONENTS, "all")
        if components != "all" and "espectre" not in components:
            continue
        source = external[CONF_SOURCE]
        if source[CONF_TYPE] != TYPE_GIT:
            continue
        ref = source.get(CONF_REF)
        if not ref or not re.fullmatch(
            r"(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)"
            r"(?:-[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?"
            r"(?:\+[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?",
            ref,
        ):
            continue
        # Reuse ESPHome's cache location without fetching or describing Git tags.
        checkout = git._compute_destination_path(
            f"{source[CONF_URL]}@{ref}", CONF_EXTERNAL_COMPONENTS
        ).resolve()
        if _COMPONENT_ROOT.is_relative_to(checkout):
            return ref
    return None


async def to_code(config):
    project = CORE.config.get(CONF_ESPHOME, {}).get(CONF_PROJECT, {})
    version = project.get(CONF_VERSION) or _frontend_ref_version()
    if version:
        add_idf_sdkconfig_option("CONFIG_APP_PROJECT_VER_FROM_CONFIG", True)
        add_idf_sdkconfig_option("CONFIG_APP_PROJECT_VER", version)
    add_idf_component(name="espectre", path=str(_COMPONENT_ROOT))
    wifi.request_wifi_ip_state_listener()
    wifi.request_wifi_connect_state_listener()

    if _uses_tinyusb_primary_console():
        # ESP-IDF's ROM CDC can enumerate without providing a reliable
        # bidirectional application channel. Keep ESPHome's standard logger
        # and Improv Serial interfaces, but route their CDC operations through
        # the shared TinyUSB primary console.
        CORE.add_job(_configure_tinyusb_primary_console)
        cg.add_define("CONFIG_ESP_CONSOLE_USB_CDC", 1)
        cg.add_define("USE_LOGGER_USB_CDC")
        cg.add_define("USE_LOGGER_UART_SELECTION_USB_CDC")
        logger_var = await cg.get_variable(CORE.config[CONF_LOGGER][CONF_ID])
        cg.add(
            logger_var.set_uart_selection(
                logger.HARDWARE_UART_TO_UART_SELECTION[logger.USB_CDC]
            )
        )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    wifi_var = await cg.get_variable(CORE.config[CONF_WIFI][CONF_ID])
    cg.add(wifi_var.add_ip_state_listener(var))
    cg.add(wifi_var.add_connect_state_listener(var))

    # Set required sdkconfig options for CSI functionality
    # These are automatically applied - user doesn't need to specify them in YAML
    add_idf_sdkconfig_option("CONFIG_ESP_WIFI_CSI_ENABLED", True)
    add_idf_sdkconfig_option("CONFIG_ESPECTRE_SDK_ENABLE_DIRECT", True)
    add_idf_sdkconfig_option("CONFIG_HTTPD_WS_SUPPORT", True)
    add_idf_sdkconfig_option("CONFIG_HTTPD_WS_PRE_HANDSHAKE_CB_SUPPORT", True)
    add_idf_sdkconfig_option("CONFIG_PM_ENABLE", False)
    add_idf_sdkconfig_option("CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE", False)
    
    # Use the shared high-rate Wi-Fi and lwIP baseline required by raw HTTP.
    # RX AMPDU remains disabled because sensing consumes individual CSI frames;
    # TX aggregation and larger queues prevent the managed traffic source from
    # becoming the cadence bottleneck.
    add_idf_sdkconfig_option("CONFIG_ESP_WIFI_AMPDU_TX_ENABLED", True)
    add_idf_sdkconfig_option("CONFIG_ESP_WIFI_AMPDU_RX_ENABLED", False)
    add_idf_sdkconfig_option("CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM", 16)
    add_idf_sdkconfig_option("CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM", 128)
    add_idf_sdkconfig_option("CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM", 128)
    add_idf_sdkconfig_option("CONFIG_LWIP_IRAM_OPTIMIZATION", True)
    add_idf_sdkconfig_option("CONFIG_LWIP_TCPIP_RECVMBOX_SIZE", 64)
    add_idf_sdkconfig_option("CONFIG_LWIP_UDP_RECVMBOX_SIZE", 32)
    # Note: CONFIG_FREERTOS_HZ=1000 is already set by ESPHome
    
    # Threshold is selected automatically at startup and remains adjustable
    # through the runtime number control.
    cg.add(var.set_direct_api(config[CONF_DIRECT_API]))
    cg.add(var.set_segmentation_window_size_ms(config[CONF_SEGMENTATION_WINDOW_SIZE_MS]))
    # ESPHome owns association policy through wifi.band_mode. Mirror that
    # validated choice into the shared runtime so its HT20 radio setup uses the
    # matching fixed-band or per-band ESP-IDF APIs.
    cg.add(var.set_wifi_band_policy(_runtime_wifi_band_policy()))
    cg.add(var.set_csi_target_pps(config[CONF_CSI_TARGET_PPS]))
    cg.add(var.set_csi_traffic_mode(config[CONF_CSI_TRAFFIC_MODE]))
    cg.add(var.set_csi_traffic_multicast_group(config[CONF_CSI_TRAFFIC_MULTICAST_GROUP]))
    cg.add(var.set_traffic_generator_mode(config[CONF_TRAFFIC_GENERATOR_MODE]))
    cg.add(var.set_detection_algorithm(config[CONF_DETECTION_ALGORITHM]))
    cg.add(var.set_evaluation_interval_ms(config[CONF_EVALUATION_INTERVAL_MS]))
    cg.add(var.set_motion_on_hits(config[CONF_MOTION_ON_HITS]))
    cg.add(var.set_motion_off_hits(config[CONF_MOTION_OFF_HITS]))
    
    # Configure Low-pass filter
    cg.add(var.set_lowpass_enabled(config[CONF_LOWPASS_ENABLED]))
    cg.add(var.set_lowpass_cutoff(config[CONF_LOWPASS_CUTOFF]))
    
    # Configure Hampel filter
    cg.add(var.set_hampel_enabled(config[CONF_HAMPEL_ENABLED]))
    cg.add(var.set_hampel_window(config[CONF_HAMPEL_WINDOW]))
    cg.add(var.set_hampel_threshold(config[CONF_HAMPEL_THRESHOLD]))
    
    # Register sensors (required, always present)
    sens = await sensor.new_sensor(config[CONF_MOVEMENT_SENSOR])
    cg.add(var.set_movement_sensor(sens))

    sens = await binary_sensor.new_binary_sensor(config[CONF_MOTION_SENSOR])
    cg.add(var.set_motion_binary_sensor(sens))

    calibration_active = await binary_sensor.new_binary_sensor(config[CONF_CALIBRATION_ACTIVE_SENSOR])
    cg.add(var.set_calibration_active_sensor(calibration_active))

    diagnostic_sensors = (
        (CONF_TRAFFIC_RATE_SENSOR, var.set_traffic_rate_sensor),
        (CONF_CSI_CALLBACK_RATE_SENSOR, var.set_csi_callback_rate_sensor),
        (CONF_CSI_ACCEPTED_RATE_SENSOR, var.set_csi_accepted_rate_sensor),
        (CONF_CSI_ADMITTED_RATE_SENSOR, var.set_csi_admitted_rate_sensor),
        (CONF_CSI_FILTERED_RATE_SENSOR, var.set_csi_filtered_rate_sensor),
        (CONF_CSI_MISSING_RATE_SENSOR, var.set_csi_missing_rate_sensor),
        (CONF_CSI_EXCESS_RATE_SENSOR, var.set_csi_excess_rate_sensor),
        (CONF_CSI_STALE_RATE_SENSOR, var.set_csi_stale_rate_sensor),
        (CONF_CSI_OUT_OF_ORDER_RATE_SENSOR, var.set_csi_out_of_order_rate_sensor),
        (CONF_CSI_OCCUPANCY_SENSOR, var.set_csi_occupancy_sensor),
        (CONF_WIFI_CHANNEL_SENSOR, var.set_wifi_channel_sensor),
        (CONF_WIFI_RSSI_SENSOR, var.set_wifi_rssi_sensor),
    )
    for config_key, setter in diagnostic_sensors:
        sens = await sensor.new_sensor(config[config_key])
        cg.add(setter(sens))

    diagnostics_button = await button.new_button(config[CONF_DIAGNOSTICS_BUTTON])
    cg.add(diagnostics_button.set_parent(var))
    
    # Register threshold number control
    # Note: number.new_number() handles component registration internally
    # Do NOT call register_component separately - it causes double initialization
    # that leads to "Load access fault" crash on boot (null pointer in early setup)
    threshold_step = 0.01
    num = await number.new_number(
        config[CONF_THRESHOLD_NUMBER],
        min_value=THRESHOLD_MIN,
        max_value=THRESHOLD_MAX,
        step=threshold_step,
    )
    cg.add(num.set_parent(var))
    cg.add(var.set_threshold_number(num))

    motion_on_hits = await number.new_number(
        config[CONF_MOTION_ON_HITS_NUMBER],
        min_value=MOTION_HITS_MIN,
        max_value=MOTION_HITS_MAX,
        step=1,
    )
    cg.add(motion_on_hits.set_parent(var))
    cg.add(motion_on_hits.set_motion_on(True))
    cg.add(var.set_motion_on_hits_number(motion_on_hits))

    motion_off_hits = await number.new_number(
        config[CONF_MOTION_OFF_HITS_NUMBER],
        min_value=MOTION_HITS_MIN,
        max_value=MOTION_HITS_MAX,
        step=1,
    )
    cg.add(motion_off_hits.set_parent(var))
    cg.add(motion_off_hits.set_motion_on(False))
    cg.add(var.set_motion_off_hits_number(motion_off_hits))

    detector = await select.new_select(
        config[CONF_DETECTOR_SELECT],
        options=["lightweight", "high_accuracy"],
    )
    cg.add(detector.set_parent(var))
    cg.add(var.set_detector_select(detector))

    csi_traffic_mode = await select.new_select(
        config[CONF_CSI_TRAFFIC_MODE_SELECT],
        options=["internal", "external"],
    )
    cg.add(csi_traffic_mode.set_parent(var))
    cg.add(csi_traffic_mode.set_csi_traffic_mode(True))
    cg.add(var.set_csi_traffic_mode_select(csi_traffic_mode))

    traffic_generator_mode = await select.new_select(
        config[CONF_TRAFFIC_GENERATOR_MODE_SELECT],
        options=["ping", "dns", "dns_tcp", "wifi_raw"],
    )
    cg.add(traffic_generator_mode.set_parent(var))
    cg.add(traffic_generator_mode.set_csi_traffic_mode(False))
    cg.add(var.set_traffic_generator_mode_select(traffic_generator_mode))
    
    sensing = await switch.new_switch(config[CONF_SENSING_SWITCH])
    cg.add(sensing.set_parent(var))
    cg.add(var.set_sensing_switch(sensing))

    recalibrate = await button.new_button(config[CONF_RECALIBRATE_BUTTON])
    cg.add(recalibrate.set_parent(var))
