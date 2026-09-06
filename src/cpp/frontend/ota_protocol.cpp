/*
 * ESPectre - Frontend OTA Protocol
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "ota_protocol.h"
#include "ota_service.h"
#include "runtime/protocol_json.h"

namespace espectre {

namespace {

const char *ota_state_name(EspectreOtaState state) {
  switch (state) {
    case EspectreOtaState::IDLE:
      return "idle";
    case EspectreOtaState::CHECKING:
      return "checking";
    case EspectreOtaState::UPDATE_AVAILABLE:
      return "update_available";
    case EspectreOtaState::UP_TO_DATE:
      return "up_to_date";
    case EspectreOtaState::DOWNLOADING:
      return "downloading";
    case EspectreOtaState::APPLYING:
      return "applying";
    case EspectreOtaState::REBOOT_SCHEDULED:
      return "reboot_scheduled";
    case EspectreOtaState::ERROR:
      return "error";
    default:
      return "unknown";
  }
}

}  // namespace

std::string espectre_ota_status_payload(const EspectreDeviceConfig &config,
                                    const EspectreOtaStatus &status,
                                    uint32_t timestamp_ms) {
  (void) config;
  std::string out;
  out.reserve(192U + status.current_version.size() + status.target_version.size() +
              status.manifest_url.size() + status.image_url.size() + status.message.size() +
              status.default_channel.size() + status.channel.size());
  out = "{";
  append_json_pair(&out, "state", ota_state_name(status.state), true);
  out += ",\"timestamp_ms\":";
  out += std::to_string(static_cast<unsigned>(timestamp_ms));
  out += ",\"busy\":";
  out += status.busy ? "true" : "false";
  out += ",\"update_available\":";
  out += status.update_available ? "true" : "false";
  append_json_pair(&out, "current_version", status.current_version.empty() ? "unknown" : status.current_version.c_str());
  append_json_pair(&out, "target_version", status.target_version.c_str());
  append_json_pair(&out, "manifest_url", status.manifest_url.c_str());
  append_json_pair(&out, "image_url", status.image_url.c_str());
  append_json_pair(&out, "default_channel", status.default_channel.c_str());
  append_json_pair(&out, "channel", status.channel.c_str());
  append_json_pair(&out, "message", status.message.c_str());
  out += "}";
  return out;
}

bool espectre_ota_channel_accepted(const std::string &channel) {
  return channel == ESPECTRE_OTA_CHANNEL_RELEASE || channel == ESPECTRE_OTA_CHANNEL_PREVIEW ||
         channel == ESPECTRE_OTA_CHANNEL_DEVELOP;
}

std::string espectre_ota_manifest_url(const char *frontend, const char *chip, const std::string &channel) {
  if (frontend == nullptr || frontend[0] == '\0' || chip == nullptr || chip[0] == '\0' ||
      !espectre_ota_channel_accepted(channel)) {
    return {};
  }
  std::string url = "https://github.com/francescopace/espectre/releases/";
  if (channel == ESPECTRE_OTA_CHANNEL_RELEASE) {
    url += "latest/download/";
  } else {
    url += "download/";
    url += (channel == ESPECTRE_OTA_CHANNEL_PREVIEW) ? ESPECTRE_OTA_RELEASE_TAG_PREVIEW
                                                     : ESPECTRE_OTA_RELEASE_TAG_DEVELOP;
    url += "/";
  }
  url += "firmware-manifest-";
  url += channel;
  url += ".json";
  return url;
}

namespace {

bool validate_ota_parameters(const std::vector<JsonObjectField> &fields, std::string *error, bool allow_channel) {
  for (const auto &field : fields) {
    if (field.name == "command_id" || field.name == "command") continue;
    if (allow_channel && field.name == "channel") {
      if (field.type == JsonValueType::STRING && espectre_ota_channel_accepted(field.value)) continue;
      if (error != nullptr) *error = "invalid ota channel (accepted: release, preview, and develop)";
      return false;
    }
    if (error != nullptr) {
      *error = (field.name == "manifest_url" || field.name == "image_url" || field.name == "version")
          ? "ota overrides are not supported (manifest_url, image_url, and version are not accepted)"
          : "unknown command parameter";
    }
    return false;
  }
  return true;
}

bool validate_ota_read(const std::vector<JsonObjectField> &fields, EspectreCommand *, std::string *error) {
  return validate_ota_parameters(fields, error, false);
}

bool validate_ota_action(const std::vector<JsonObjectField> &fields, EspectreCommand *command, std::string *error) {
  if (!validate_ota_parameters(fields, error, true)) return false;
  // Preserve decoded parameters without the MQTT envelope or JSON escapes.
  command->extension_parameters = "{";
  if (const auto *channel = find_json_object_field(fields, "channel")) {
    append_json_pair(&command->extension_parameters, "channel", channel->value.c_str(), true);
  }
  command->extension_parameters += '}';
  return true;
}

}  // namespace

const EspectreProtocolExtension &frontend_ota_protocol() {
  static const EspectreProtocolExtension extension{
      {
          {"GET", "/espectre/v1/ota", "ota", "ota", EspectreApiRouteKind::RESOURCE,
           false, false, true, validate_ota_read},
          {"POST", "/espectre/v1/ota/checks", "check_ota", "check_ota", EspectreApiRouteKind::OPERATION,
           true, true, false, validate_ota_action},
          {"POST", "/espectre/v1/ota/updates", "start_ota", "start_ota", EspectreApiRouteKind::OPERATION,
           true, true, false, validate_ota_action},
      },
      {"ota"},
  };
  return extension;
}

FrontendCommandResult execute_frontend_ota_command(
    const EspectreCommand &command, FrontendCommandOrigin origin, IOtaService *service,
    const std::string &current_version, FrontendReadPayloadCallback read_payload) {
  FrontendCommandResult result;
  result.command = command;
  const auto *route = find_extension_route(&frontend_ota_protocol(), command.command);
  if (route == nullptr) return result;
  result.handled = true;
  if (origin == FrontendCommandOrigin::MQTT && !route->mqtt) {
    result.code = "forbidden";
    result.message = "command is not available over MQTT";
    return result;
  }
  if (service == nullptr) {
    result.code = "unavailable";
    result.message = "ota unavailable";
    return result;
  }
  if (command.command == "ota") {
    result.data_json = read_payload ? read_payload(command) : std::string{};
    result.accepted = !result.data_json.empty();
    result.code = result.accepted ? "ok" : "unavailable";
    result.message = result.accepted ? "ota status returned" : "command data is unavailable";
    return result;
  }
  const std::string version = current_version.empty() ? "unknown" : current_version;
  const std::string channel = extract_json_string(command.extension_parameters, "channel");
  result.accepted = command.command == "check_ota"
      ? service->start_check(version, channel) : service->start_update(version, channel);
  result.code = result.accepted ? "ok" : "busy";
  result.message = command.command == "check_ota"
      ? (result.accepted ? "ota check started" : "ota check rejected")
      : (result.accepted ? "ota update started" : "ota update rejected");
  return result;
}

}  // namespace espectre
