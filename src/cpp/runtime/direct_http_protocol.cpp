/*
 * ESPectre - Direct HTTP Protocol
 *
 * HTTP framing for the transport-neutral ESPectre message model.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "direct_http_protocol.h"

#include <vector>

#include "protocol_json.h"

namespace espectre {

bool parse_direct_http_request(const std::string &http_method,
                               const std::string &path,
                               const std::string &payload,
                               DirectRequest *request,
                               std::string *error,
                               const EspectreProtocolExtension *extension) {
  if (request == nullptr) {
    if (error != nullptr) {
      *error = "request output is required";
    }
    return false;
  }
  DirectRequest parsed;
  const auto reject = [&](const char *message) {
    if (error != nullptr) {
      *error = message;
    }
    *request = parsed;
    return false;
  };
  parsed.http_method = http_method;
  parsed.path = path;
  size_t route_count = 0U;
  const EspectreApiRoute *routes = espectre_api_routes(&route_count);
  for (size_t index = 0U; index < route_count; ++index) {
    const EspectreApiRoute &route = routes[index];
    if (route.kind != EspectreApiRouteKind::STREAM &&
        http_method == route.http_method && path == route.path) {
      parsed.command = route.command;
      parsed.asynchronous = route.asynchronous;
      break;
    }
  }
  if (parsed.command.empty() && extension != nullptr && validate_protocol_extension(*extension)) {
    for (const auto &route : extension->routes) {
      if (http_method == route.http_method && path == route.path) {
        parsed.command = route.command;
        parsed.asynchronous = route.asynchronous;
        break;
      }
    }
  }
  if (parsed.command.empty()) return reject("unsupported Direct resource or method");
  if (payload.size() > ESPECTRE_DIRECT_MAX_REQUEST_SIZE) {
    return reject("Direct request exceeds the size limit");
  }

  std::vector<JsonObjectField> fields;
  std::string json_error;
  const std::string normalized_payload = payload.empty() ? "{}" : payload;
  if (!parse_json_object_fields(normalized_payload, &fields, &json_error)) {
    if (error != nullptr) {
      *error = json_error.empty() ? "invalid Direct JSON envelope" : json_error;
    }
    *request = parsed;
    return false;
  }

  // Preserve escaped characters in keys and values for canonical validation.
  parsed.params = normalized_payload;
  *request = std::move(parsed);
  return true;
}

bool direct_http_request_to_command(const DirectRequest &request,
                                    EspectreCommand *command,
                                    std::string *error,
                                    const EspectreProtocolExtension *extension) {
  return parse_espectre_command_request(
      request.command_id, request.command, request.params, command, error, ESPECTRE_PROTOCOL_VERSION, extension);
}

std::string espectre_transport_mapping_payload() {
  std::string out{"{"};
  out += "\"direct\":{\"request\":{\"framing\":\"http_resource\"";
  append_json_pair(&out, "path", ESPECTRE_DIRECT_HTTP_BASE_ENDPOINT);
  append_json_pair(&out, "message", "resource_or_operation");
  out += "},\"result\":{\"framing\":\"http_response_body\",\"message\":\"result\"},"
         "\"events\":{\"framing\":\"sse\"";
  append_json_pair(&out, "path", ESPECTRE_DIRECT_HTTP_EVENTS_ENDPOINT);
  append_json_pair(&out, "name", "event");
  append_json_pair(&out, "data", "event_payload");
  out += "}},\"mqtt\":{\"request\":{\"topic_suffix\":\"commands/request\",\"message\":\"request\"},"
         "\"result\":{\"topic_suffix\":\"commands/result\",\"message\":\"result\"},"
         "\"events\":{\"topic_suffix\":\"{event}\",\"message\":\"event_payload\"}}}";
  return out;
}

std::string espectre_protocol_catalog_payload(const EspectreProtocolExtension *extension) {
  return "{\"message_model\":" + espectre_message_catalog_payload(extension) +
         ",\"transport_mapping\":" + espectre_transport_mapping_payload() + "}";
}

}  // namespace espectre
