/*
 * ESPectre - ESPectre Protocol
 *
 * Shared device, command, and OTA protocol types used by frontend
 * transports.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "runtime_snapshot.h"

/**
 * @file espectre_protocol.h
 * @brief Wire types and payload builders for the ESPectre Protocol.
 *
 * The protocol is the contract between a device and whatever consumes it:
 * MQTT topics, Direct HTTP messages, JSON payloads, and the OTA status model.
 * It is specified in `docs/API.md`; this header is the C++ view
 * of that specification.
 *
 * Use it whenever your integration should stay interoperable with the shipped
 * clients — the CLI, Home Assistant discovery, and the web portal all
 * speak it. The builders take a `RuntimeSnapshot` and return a serialized
 * payload, so your transport only moves bytes and never formats them.
 *
 * The parsers never throw: they validate and report failure through an out
 * parameter. They do not all roll back cleanly on rejection, so parse into a
 * copy of your live configuration and commit it only on success, which is what
 * the shipped frontends do.
 */

namespace espectre {

/** Matter Basic Information NodeLabel limit, shared by all Direct frontends. */
inline constexpr size_t ESPECTRE_DEVICE_LABEL_MAX_LENGTH = 32U;
/** Maximum serialized command-request size accepted by every transport. */
inline constexpr size_t ESPECTRE_COMMAND_MAX_PAYLOAD_SIZE = 2048U;

enum class EspectreDirectMethod : uint8_t {
  CAPABILITIES = 0,
  INFO,
  STATUS,
  CONFIG,
  DIAGNOSTICS,
  SET_SENSING,
  SET_DEVICE_LABEL,
  SET_THRESHOLD,
  SET_MOTION_HITS,
  SET_DETECTOR,
  RECALIBRATE,
  START_RAW_STREAM,
  STOP_RAW_STREAM,
  SET_CSI_TRAFFIC_MODE,
  SET_TRAFFIC_GENERATOR_MODE,
  WIFI_ACCESS_POINTS,
  SCAN_WIFI_ACCESS_POINTS,
  SET_WIFI_BSSID,
  CLEAR_WIFI_BSSID,
  CLEAR_WIFI_CONFIG,
  SET_MQTT_CONFIG,
  CLEAR_MQTT_CONFIG,
  OTA_STATUS,
  OTA_CHECK,
  OTA_START,
  DISCOVER_PEERS,
  COUNT,
};

enum class EspectreConfigSection : uint8_t {
  RUNTIME = 0,
  DEVICE,
  WIFI,
  MQTT,
  COUNT,
};

enum class EspectreEvent : uint8_t {
  TELEMETRY = 0,
  STATUS,
  INFO,
  CONFIG,
  OTA_STATUS,
  FAULT,
  COUNT,
};

enum class EspectreApiRouteKind : uint8_t {
  RESOURCE = 0,
  OPERATION,
  STREAM,
};

/** One canonical HTTP/resource mapping used by routing and capability output. */
struct EspectreApiRoute {
  const char *http_method;
  const char *path;
  const char *name;
  const char *command;
  EspectreDirectMethod capability;
  EspectreApiRouteKind kind;
  bool asynchronous;
};

struct EspectreApiEventDescriptor {
  const char *name;
  EspectreEvent event;
  EspectreDirectMethod capability;
};

/** Return the immutable v1 resource registry and its entry count. */
const EspectreApiRoute *espectre_api_routes(size_t *count);
const EspectreApiEventDescriptor *espectre_api_events(size_t *count);

/** Exact Direct command, event, and readable-configuration surface advertised by a frontend. */
struct EspectreCapabilityProfile {
  std::array<bool, static_cast<size_t>(EspectreDirectMethod::COUNT)> methods{};
  std::array<bool, static_cast<size_t>(EspectreConfigSection::COUNT)> config_sections{};
  std::array<bool, static_cast<size_t>(EspectreEvent::COUNT)> events{{true, true, true, true, false, true}};

  bool supports(EspectreDirectMethod method) const {
    return methods[static_cast<size_t>(method)];
  }
  void set(EspectreDirectMethod method, bool enabled = true) {
    methods[static_cast<size_t>(method)] = enabled;
  }
  bool has(EspectreConfigSection section) const {
    return config_sections[static_cast<size_t>(section)];
  }
  void set(EspectreConfigSection section, bool enabled = true) {
    config_sections[static_cast<size_t>(section)] = enabled;
  }
  bool publishes(EspectreEvent event) const {
    return events[static_cast<size_t>(event)];
  }
  void set(EspectreEvent event, bool enabled = true) {
    events[static_cast<size_t>(event)] = enabled;
  }
  void clear_events() {
    events.fill(false);
  }
};

struct RuntimeDiagnosticsSample;

/** Protocol version reported by capabilities and discovery. */
inline constexpr const char *ESPECTRE_PROTOCOL_VERSION = "1.0";
/** DNS-SD TXT record schema advertised as the RFC 6763 `txtvers` value. */
inline constexpr const char *ESPECTRE_DNS_SD_TXT_SCHEMA_VERSION = "1";
/** Maximum canonical command correlation identifier length. */
inline constexpr size_t ESPECTRE_COMMAND_ID_MAX_LENGTH = 64U;
/** Default MQTT topic root. Override per device with `EspectreDeviceConfig::topic_prefix`. */
inline constexpr const char *ESPECTRE_TOPIC_PREFIX = "espectre/v1/devices";
/** Official tagged GitHub Release OTA channel. */
inline constexpr const char *ESPECTRE_OTA_CHANNEL_RELEASE = "release";
/** Rolling `main` OTA channel. Fetches GitHub Releases tag `snapshot`. */
inline constexpr const char *ESPECTRE_OTA_CHANNEL_PREVIEW = "preview";
/** Rolling `develop` OTA channel. Fetches GitHub Releases tag `snapshot-dev`. */
inline constexpr const char *ESPECTRE_OTA_CHANNEL_DEVELOP = "develop";
/** GitHub Releases tag for the `preview` OTA channel. Distinct from branch `main`. */
inline constexpr const char *ESPECTRE_OTA_RELEASE_TAG_PREVIEW = "snapshot";
/** GitHub Releases tag for the `develop` OTA channel. Distinct from branch `develop`. */
inline constexpr const char *ESPECTRE_OTA_RELEASE_TAG_DEVELOP = "snapshot-dev";
/** Sentinel meaning "use the runtime-generated device id". */
inline constexpr uint64_t ESPECTRE_DEFAULT_DEVICE_ID = 0U;
/** Empty label, meaning the device id is used as the display name. */
inline constexpr const char *ESPECTRE_DEFAULT_DEVICE_LABEL = "";

/**
 * Device identity and broker settings.
 *
 * Frontends persist this so a device keeps its identity and connection across
 * reboots and reprovisioning.
 */
struct EspectreDeviceConfig {
  /** Stable device identity. Zero means use the runtime-generated value. */
  uint64_t device_id{ESPECTRE_DEFAULT_DEVICE_ID};
  /** Human-readable name. Empty falls back to the formatted device id. */
  std::string device_label{ESPECTRE_DEFAULT_DEVICE_LABEL};
  /** Broker transport scheme: `mqtt` or `mqtts`. Empty disables MQTT. */
  std::string mqtt_scheme;
  /** Broker DNS hostname, IPv4 address, or IPv6 address, without URI framing. */
  std::string mqtt_host;
  /** Broker port. Zero means MQTT is not configured. */
  uint16_t mqtt_port{0U};
  /** Broker credentials. Leave empty for anonymous brokers. */
  std::string mqtt_username;
  std::string mqtt_password;
  /** Topic root. Change it only if you also change every consumer. */
  std::string topic_prefix{ESPECTRE_TOPIC_PREFIX};
};

/** Link details available to frontends. Canonical MQTT info publishes only the channel. */
struct EspectreNetworkInfo {
  std::string ip_address;
  std::string mac_address;
  /** Wi-Fi channel in use. Zero when unknown. */
  uint8_t channel{0U};
};

/**
 * What the device advertises about itself.
 *
 * The `supports_*` flags are internal inputs used to build the filtered
 * `capabilities` catalog. They are deliberately omitted from `info` so clients
 * have one authoritative feature-discovery surface.
 */
struct EspectreDeviceInfo {
  /** Frontend name, for example `"native"`, `"matter"`, or your own. */
  std::string frontend{"unknown"};
  /** Application version, normally `espectre_firmware_version()`. */
  std::string firmware_version{"unknown"};
  /** Chip target, normally `CONFIG_IDF_TARGET`. */
  std::string chip{"unknown"};
  /** Active detector. Left empty, it is filled from the snapshot. */
  std::string detector;
  /** Automatically selected CSI capture profile. Left empty, it is filled from the snapshot. */
  std::string csi_profile;
  bool supports_info{true};
  bool supports_diagnostics{false};
  /** `update_device` is honored and persists the user-facing label. */
  bool supports_device_config{false};
  bool supports_runtime_threshold{false};
  bool supports_runtime_motion_hits{false};
  bool supports_runtime_detector{false};
  bool supports_manual_recalibration{false};
  bool supports_traffic_control{false};
  bool supports_ota{false};
  /**
   * CSI traffic ownership mode: `"internal"` or `"external"`.
   *
   * Omitted from `info` when empty. Sensing MQTT frontends that own traffic control fill it.
   */
  std::string csi_traffic_mode;
  /**
   * Internal traffic generator mode: `"ping"`, `"dns"`, or `"dns_tcp"`.
   *
   * Omitted from `info` when empty.
   */
  std::string traffic_mode;
  /**
   * Internal traffic generator and temporal-grid target rate, in packets per second.
   *
   * Omitted from `info` when zero.
   */
  uint32_t csi_target_pps{0U};
  /** UDP destination port used by the external CSI traffic generator. */
  uint16_t csi_traffic_udp_port{0U};
  /** IPv4 multicast group used by external CSI traffic, or empty for unicast-only operation. */
  std::string csi_traffic_multicast_group;
  /**
   * Detector evaluation cadence, in milliseconds.
   *
   * Omitted from `info` when zero. Canonical MQTT telemetry follows this interval.
   */
  uint32_t evaluation_interval_ms{0U};
  EspectreNetworkInfo network{};
};

/**
 * A parsed control command.
 *
 * Fields are optional by design: each `has_*` flag says whether the peer
 * actually sent that field, so an unset value is never confused with a zero
 * the caller meant. Apply only the flagged fields.
 */
struct EspectreCommand {
  /** Correlation id echoed in the result payload. May be empty. */
  std::string command_id;
  /** Command verb, for example `"update_sensing"` or `"recalibrate"`. */
  std::string command;
  /** Requested sensing-service state for `update_sensing`. */
  bool sensing_enabled{false};
  bool has_sensing_enabled{false};
  /** User-facing label requested by `update_device`; empty clears it. */
  std::string device_label;
  /** Whether the command carried a valid string-valued `device_label`. */
  bool has_device_label{false};
  float threshold{0.0f};
  bool has_threshold{false};
  uint8_t motion_on_hits{0U};
  uint8_t motion_off_hits{0U};
  bool has_motion_hits{false};
  std::string csi_traffic_mode;
  bool has_csi_traffic_mode{false};
  std::string traffic_generator_mode;
  bool has_traffic_generator_mode{false};
  std::string detector;
  bool has_detector{false};
  std::string wifi_bssid;
  bool has_wifi_bssid{false};
  /** Force reassociation even when `wifi_bssid` is already active. */
  bool wifi_bssid_force{false};
  bool has_wifi_bssid_force{false};
  std::string mqtt_scheme;
  std::string mqtt_host;
  std::string mqtt_username;
  std::string mqtt_password;
  std::string mqtt_topic_prefix;
  uint16_t mqtt_port{0U};
  bool has_mqtt_scheme{false};
  bool has_mqtt_host{false};
  bool has_mqtt_username{false};
  bool has_mqtt_password{false};
  bool has_mqtt_topic_prefix{false};
  bool has_mqtt_port{false};
  /**
   * OTA release channel for `check_ota` and `start_ota`: `"release"`, `"preview"`,
   * or `"develop"`. Empty with `has_ota_channel` false means the firmware default.
   */
  std::string ota_channel;
  bool has_ota_channel{false};
};

/**
 * OTA progress, as reported to clients.
 *
 * A check runs `IDLE` -> `CHECKING` -> `UPDATE_AVAILABLE` or `UP_TO_DATE`.
 * An update continues `DOWNLOADING` -> `APPLYING` -> `REBOOT_SCHEDULED`.
 * `ERROR` is terminal for the attempt and carries the reason in
 * `EspectreOtaStatus::message`.
 */
enum class EspectreOtaState : uint8_t {
  IDLE = 0,
  CHECKING,
  UPDATE_AVAILABLE,
  UP_TO_DATE,
  DOWNLOADING,
  APPLYING,
  REBOOT_SCHEDULED,
  ERROR,
};

/** Full OTA status: state, the versions involved, and the resolved URLs. */
struct EspectreOtaStatus {
  EspectreOtaState state{EspectreOtaState::IDLE};
  std::string current_version{"unknown"};
  std::string target_version;
  std::string manifest_url;
  std::string image_url;
  std::string message;
  /** Build-time OTA channel used when a command omits its channel. */
  std::string default_channel;
  /** Resolved OTA channel for the current attempt. Empty when unused. */
  std::string channel;
  bool busy{false};
  bool update_available{false};
};

/**
 * @name Device identity
 * Resolve, format, and parse the identity a device presents on the wire.
 * @{
 */

/** Format a device id in its canonical wire form. */
std::string format_espectre_device_id(uint64_t device_id);
/**
 * Parse a device id from its wire form.
 *
 * @param value Formatted device id, as produced by `format_espectre_device_id()`.
 * @param device_id Written only when parsing succeeds.
 * @return false on a malformed value, leaving the output untouched.
 */
bool parse_espectre_device_id(const std::string &value, uint64_t *device_id);
/**
 * Pack the first six MAC bytes into the historical numeric representation.
 *
 * @deprecated Runtime firmware uses the cached, domain-separated SHA-256
 * pseudonym from `derive_runtime_device_id()` instead.
 */
[[deprecated("use the runtime-generated device identity")]] uint64_t espectre_device_id_from_mac(
    const uint8_t *mac, size_t mac_len);
/** Conventional device name derived from the immutable device identifier. */
std::string espectre_device_name(uint64_t device_id, const char *chip = nullptr);
/** The id actually in use. Frontend startup replaces the zero sentinel. */
uint64_t espectre_effective_device_id_u64(const EspectreDeviceConfig &config);
/** `espectre_effective_device_id_u64()` in wire form. */
std::string espectre_effective_device_id(const EspectreDeviceConfig &config);
/** The configured label, or the effective device id when no label is set. */
std::string espectre_effective_device_label(const EspectreDeviceConfig &config);
/**
 * Fill in the parts of a device info block the frontend did not set.
 *
 * Takes the detector from `snapshot`, and `default_frontend` / `default_chip`
 * where the caller left the field empty, so each frontend only states what is
 * genuinely its own.
 *
 * @param info What the frontend knows about itself.
 * @param snapshot Source of the detector name. May be `nullptr` when no
 *        snapshot exists yet.
 * @param supports_ota Whether this frontend exposes firmware updates.
 * @param default_frontend Frontend name used when `info.frontend` is empty.
 * @param default_chip Chip name used when `info.chip` is empty.
 * @return A copy of `info` with the gaps filled.
 */
EspectreDeviceInfo normalize_protocol_device_info(const EspectreDeviceInfo &info,
                                                  const RuntimeSnapshot *snapshot,
                                                  bool supports_ota,
                                                  const char *default_frontend,
                                                  const char *default_chip = nullptr);
/** Erase broker settings while preserving identity, for a config reset. */
void clear_espectre_mqtt_config(EspectreDeviceConfig *config);
/**
 * Validate the complete MQTT endpoint in a device configuration.
 *
 * The endpoint requires an exact `mqtt` or `mqtts` scheme, a DNS hostname,
 * IPv4 address, or IPv6 address without URI framing, and a non-zero port.
 *
 * @param config Device configuration carrying the MQTT endpoint.
 * @param error Receives a human-readable reason on failure. May be `nullptr`.
 * @return `true` only when the complete endpoint is valid.
 */
bool validate_espectre_mqtt_config(const EspectreDeviceConfig &config, std::string *error = nullptr);
/** Return whether `config` contains a complete, valid MQTT endpoint. */
bool espectre_mqtt_configured(const EspectreDeviceConfig &config);

/** @} */

/**
 * @name Topics and payloads
 * Build the wire representation from runtime state. Each returns a complete
 * payload ready to hand to a transport.
 * @{
 */

/** Build a full topic from this device's prefix and a trailing segment. */
std::string espectre_topic(const EspectreDeviceConfig &config, const char *suffix);
/** Availability payload. Publish it retained so late subscribers see it. */
std::string espectre_health_payload(const EspectreDeviceConfig &config, bool online, uint32_t timestamp_ms);
/** Stable device identity and build description. Publish retained on connect. */
std::string espectre_device_payload(const EspectreDeviceConfig &config, const EspectreDeviceInfo &info);
/**
 * Filtered command, event, feature, and configuration catalog.
 */
std::string espectre_capabilities_payload(const EspectreDeviceConfig &config,
                                          const EspectreDeviceInfo &info,
                                          const EspectreCapabilityProfile &capabilities);
/**
 * Compatibility overload for existing SDK consumers. New code should pass an
 * `EspectreCapabilityProfile` so readable sections and individual commands are
 * represented independently.
 */
std::string espectre_capabilities_payload(const EspectreDeviceConfig &config,
                                          const EspectreDeviceInfo &info,
                                          bool supports_status = true,
                                          bool supports_config = false,
                                          bool supports_sensing_control = false,
                                          bool supports_wifi_bssid = false,
                                          bool supports_mqtt_config = false,
                                          bool supports_peer_discovery = false,
                                          bool supports_raw_csi = false);
/** Current motion state and score. The payload behind every detector evaluation. */
std::string espectre_motion_payload(const EspectreDeviceConfig &config,
                                    const RuntimeSnapshot &snapshot,
                                    uint32_t timestamp_ms,
                                    uint32_t uptime_s,
                                    const char *frontend);
/**
 * Health counters plus optional rate and link diagnostics.
 *
 * `diagnostics` carries CSI and link rates from `RuntimeDiagnosticsSampler`.
 * Pass `nullptr` only for a frontend that does not expose extended diagnostics.
 */
std::string espectre_diagnostics_payload(const EspectreDeviceConfig &config,
                                         const RuntimeSnapshot &snapshot,
                                         uint32_t timestamp_ms,
                                         uint32_t uptime_s,
                                         float free_memory_kb,
                                         float loop_time_ms,
                                         const RuntimeDiagnosticsSample *diagnostics = nullptr);
/**
 * Acknowledge a command, echoing its `command_id`.
 *
 * Publish one for every command you parse, accepted or not; clients correlate
 * on the id and otherwise cannot tell rejection from packet loss.
 */
std::string espectre_command_result_payload(const EspectreDeviceConfig &config,
                                            const EspectreCommand &command,
                                            bool accepted,
                                            const char *code,
                                            const char *message,
                                            const std::string &data_json = {});
/** Build the canonical flat command request carried by MQTT and Direct HTTP. */
std::string espectre_command_request_payload(const std::string &command_id,
                                             const std::string &command,
                                             const std::string &params_json = "{}");
/** Runtime fault event shared by every transport. */
std::string espectre_fault_payload(const EspectreDeviceConfig &config,
                                   const char *message,
                                   uint32_t timestamp_ms);
/** Executable transport-neutral message samples used by the C++/Python parity gate. */
std::string espectre_message_catalog_payload();
/** OTA progress payload, for each `IOtaService` status callback worth publishing. */
std::string espectre_ota_status_payload(const EspectreDeviceConfig &config,
                                    const EspectreOtaStatus &status,
                                    uint32_t timestamp_ms);

/** @} */

/**
 * @name Command parsing
 * Turn received bytes into validated values.
 * @{
 */

/**
 * Parse a JSON command payload from the MQTT command topic.
 *
 * @param payload Raw message body as received.
 * @param command Populated only on success. Check the `has_*` flags to see
 *        which fields the peer actually sent.
 * @param error Receives a human-readable reason on failure. May be `nullptr`.
 * @return false on malformed input or an unknown command.
 */
bool parse_espectre_command(const std::string &payload, EspectreCommand *command, std::string *error);
/**
 * Parse an already separated command name plus a JSON parameter object.
 *
 * Frontend adapters use this after separating a canonical flat request into
 * internal fields. Validation follows canonical envelope order: correlation
 * identifier, protocol version, command name, and parameters.
 */
bool parse_espectre_command_request(const std::string &command_id,
                                    const std::string &command_name,
                                    const std::string &params_json,
                                    EspectreCommand *command,
                                    std::string *error,
                                    const std::string &protocol_version = ESPECTRE_PROTOCOL_VERSION);
/**
 * Whether `channel` is a published OTA channel name.
 *
 * Accepted values are `release`, `preview`, and `develop`. Empty is not
 * accepted here; omit the field to keep the firmware default.
 */
bool espectre_ota_channel_accepted(const std::string &channel);
/**
 * Built-in GitHub Releases firmware catalog URL for the selected channel.
 *
 * `release` uses `/releases/latest/download/`. `preview` uses tag
 * `ESPECTRE_OTA_RELEASE_TAG_PREVIEW` (`snapshot`). `develop` uses tag
 * `ESPECTRE_OTA_RELEASE_TAG_DEVELOP` (`snapshot-dev`).
 * Each channel publishes `firmware-manifest-<channel>.json`; the OTA service
 * selects the frontend and chip from that catalog.
 *
 * @return Empty when `frontend`, `chip`, or `channel` is not a published value.
 */
std::string espectre_ota_manifest_url(const char *frontend, const char *chip, const std::string &channel);
/**
 * Parse a legacy ASCII `SET_DEVICE_CONFIG:` command.
 *
 * Carries one `key=value` pair, applied in place. A rejected command writes
 * nothing.
 *
 * @param command Full command string, including the `SET_DEVICE_CONFIG:` prefix.
 * @param config Updated in place on success.
 * @param error Receives a human-readable reason on failure. May be `nullptr`.
 */
bool parse_espectre_config_command(const std::string &command, EspectreDeviceConfig *config, std::string *error);
/**
 * Parse a `SET_MQTT_CONFIG:` command, carrying the broker settings.
 *
 * The complete command is parsed and validated before `config` is changed, so
 * rejection leaves the previous value intact. `scheme`, `host`, and `port` are
 * required; the rest keep their previous values.
 */
bool parse_espectre_mqtt_config_command(const std::string &command, EspectreDeviceConfig *config, std::string *error);

/** @} */

}  // namespace espectre
