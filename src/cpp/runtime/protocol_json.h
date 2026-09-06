/*
 * ESPectre - Protocol JSON
 *
 * JSON helpers for shared ESPectre protocol payloads.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace espectre {

enum class JsonValueType : uint8_t {
  STRING = 0,
  NUMBER,
  BOOLEAN,
  NULL_VALUE,
  OBJECT,
  ARRAY,
};

struct JsonObjectField {
  std::string name;
  JsonValueType type{JsonValueType::NULL_VALUE};
  /** Decoded contents for strings, or the exact JSON token for every other type. */
  std::string value;
};

void append_json_string(std::string *out, const char *value);
void append_json_pair(std::string *out, const char *key, const char *value, bool first = false);
bool has_json_key(const std::string &payload, const char *key);
std::string extract_json_string(const std::string &payload, const char *key);
std::string extract_json_number_token(const std::string &payload, const char *key);
bool decode_urlencoded_component(const std::string &encoded, std::string *decoded, std::string *error = nullptr);
std::string encode_urlencoded_component(const std::string &value);
bool parse_urlencoded_key_value_pairs(const std::string &payload,
                                      std::vector<std::pair<std::string, std::string>> *pairs,
                                      std::string *error = nullptr);
/** Parse and validate one complete JSON object, rejecting duplicate field names. */
bool parse_json_object_fields(const std::string &payload,
                              std::vector<JsonObjectField> *fields,
                              std::string *error = nullptr);
/** Parse a complete array of objects, rejecting invalid or non-object entries. */
bool parse_json_array_objects(const std::string &payload,
                              std::vector<std::vector<JsonObjectField>> *objects,
                              std::string *error = nullptr);
const JsonObjectField *find_json_object_field(const std::vector<JsonObjectField> &fields, const char *name);

}  // namespace espectre
