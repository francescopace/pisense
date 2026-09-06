/*
 * ESPectre - Protocol JSON
 *
 * JSON helpers for shared ESPectre protocol payloads.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "protocol_json.h"

#include <cctype>
#include <cstdint>
#include <cstdlib>

namespace espectre {

namespace {

class JsonReader {
 public:
  explicit JsonReader(const std::string &input) : input_(input) {}

  bool parse_object(std::vector<JsonObjectField> *fields, std::string *error) {
    error_ = error;
    if (fields == nullptr) {
      return fail_("fields output is required");
    }
    fields->clear();
    skip_space_();
    if (!parse_object_(fields, 0U)) {
      return false;
    }
    skip_space_();
    if (position_ != input_.size()) {
      return fail_("unexpected data after JSON object");
    }
    return true;
  }

  bool parse_array_objects(std::vector<std::vector<JsonObjectField>> *objects, std::string *error) {
    error_ = error;
    if (objects == nullptr) {
      return fail_("objects output is required");
    }
    objects->clear();
    skip_space_();
    if (!parse_array_(0U, objects)) {
      return false;
    }
    skip_space_();
    return position_ == input_.size() || fail_("unexpected data after JSON array");
  }

 private:
  static bool hex_value_(char ch, uint32_t *value) {
    if (value == nullptr) {
      return false;
    }
    if (ch >= '0' && ch <= '9') {
      *value = static_cast<uint32_t>(ch - '0');
      return true;
    }
    if (ch >= 'a' && ch <= 'f') {
      *value = static_cast<uint32_t>(10 + ch - 'a');
      return true;
    }
    if (ch >= 'A' && ch <= 'F') {
      *value = static_cast<uint32_t>(10 + ch - 'A');
      return true;
    }
    return false;
  }

  static bool append_utf8_(uint32_t codepoint, std::string *out) {
    if (out == nullptr || codepoint > 0x10FFFFU || (codepoint >= 0xD800U && codepoint <= 0xDFFFU)) {
      return false;
    }
    if (codepoint <= 0x7FU) {
      out->push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7FFU) {
      out->push_back(static_cast<char>(0xC0U | (codepoint >> 6U)));
      out->push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
    } else if (codepoint <= 0xFFFFU) {
      out->push_back(static_cast<char>(0xE0U | (codepoint >> 12U)));
      out->push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
      out->push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
    } else {
      out->push_back(static_cast<char>(0xF0U | (codepoint >> 18U)));
      out->push_back(static_cast<char>(0x80U | ((codepoint >> 12U) & 0x3FU)));
      out->push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
      out->push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
    }
    return true;
  }

  bool fail_(const char *message) {
    if (error_ != nullptr) {
      *error_ = message != nullptr ? message : "invalid JSON";
    }
    return false;
  }

  void skip_space_() {
    while (position_ < input_.size()) {
      const char ch = input_[position_];
      if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') {
        break;
      }
      ++position_;
    }
  }

  bool consume_(char expected) {
    if (position_ >= input_.size() || input_[position_] != expected) {
      return false;
    }
    ++position_;
    return true;
  }

  bool parse_hex_quad_(uint32_t *value) {
    if (value == nullptr || position_ + 4U > input_.size()) {
      return false;
    }
    uint32_t parsed = 0U;
    for (size_t i = 0U; i < 4U; ++i) {
      uint32_t nibble = 0U;
      if (!hex_value_(input_[position_ + i], &nibble)) {
        return false;
      }
      parsed = (parsed << 4U) | nibble;
    }
    position_ += 4U;
    *value = parsed;
    return true;
  }

  bool parse_string_(std::string *value) {
    if (!consume_('"')) {
      return fail_("expected JSON string");
    }
    std::string parsed;
    while (position_ < input_.size()) {
      const unsigned char ch = static_cast<unsigned char>(input_[position_++]);
      if (ch == '"') {
        if (value != nullptr) {
          *value = std::move(parsed);
        }
        return true;
      }
      if (ch < 0x20U) {
        return fail_("unescaped control character in JSON string");
      }
      if (ch != '\\') {
        parsed.push_back(static_cast<char>(ch));
        continue;
      }
      if (position_ >= input_.size()) {
        return fail_("truncated JSON string escape");
      }
      const char escape = input_[position_++];
      switch (escape) {
        case '"':
        case '\\':
        case '/':
          parsed.push_back(escape);
          break;
        case 'b':
          parsed.push_back('\b');
          break;
        case 'f':
          parsed.push_back('\f');
          break;
        case 'n':
          parsed.push_back('\n');
          break;
        case 'r':
          parsed.push_back('\r');
          break;
        case 't':
          parsed.push_back('\t');
          break;
        case 'u': {
          uint32_t codepoint = 0U;
          if (!parse_hex_quad_(&codepoint)) {
            return fail_("invalid JSON unicode escape");
          }
          if (codepoint >= 0xD800U && codepoint <= 0xDBFFU) {
            if (position_ + 6U > input_.size() || input_[position_] != '\\' || input_[position_ + 1U] != 'u') {
              return fail_("missing JSON low surrogate");
            }
            position_ += 2U;
            uint32_t low = 0U;
            if (!parse_hex_quad_(&low) || low < 0xDC00U || low > 0xDFFFU) {
              return fail_("invalid JSON low surrogate");
            }
            codepoint = 0x10000U + ((codepoint - 0xD800U) << 10U) + (low - 0xDC00U);
          } else if (codepoint >= 0xDC00U && codepoint <= 0xDFFFU) {
            return fail_("unexpected JSON low surrogate");
          }
          if (!append_utf8_(codepoint, &parsed)) {
            return fail_("invalid JSON unicode codepoint");
          }
          break;
        }
        default:
          return fail_("invalid JSON string escape");
      }
    }
    return fail_("unterminated JSON string");
  }

  bool parse_number_() {
    const size_t begin = position_;
    if (position_ < input_.size() && input_[position_] == '-') {
      ++position_;
    }
    if (position_ >= input_.size()) {
      return fail_("invalid JSON number");
    }
    if (input_[position_] == '0') {
      ++position_;
      if (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) {
        return fail_("invalid leading zero in JSON number");
      }
    } else if (input_[position_] >= '1' && input_[position_] <= '9') {
      while (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) {
        ++position_;
      }
    } else {
      return fail_("invalid JSON number");
    }
    if (position_ < input_.size() && input_[position_] == '.') {
      ++position_;
      const size_t fraction = position_;
      while (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) {
        ++position_;
      }
      if (position_ == fraction) {
        return fail_("invalid JSON fraction");
      }
    }
    if (position_ < input_.size() && (input_[position_] == 'e' || input_[position_] == 'E')) {
      ++position_;
      if (position_ < input_.size() && (input_[position_] == '+' || input_[position_] == '-')) {
        ++position_;
      }
      const size_t exponent = position_;
      while (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) {
        ++position_;
      }
      if (position_ == exponent) {
        return fail_("invalid JSON exponent");
      }
    }
    return position_ > begin;
  }

  bool parse_literal_(const char *literal) {
    if (literal == nullptr) {
      return false;
    }
    const std::string expected(literal);
    if (input_.compare(position_, expected.size(), expected) != 0) {
      return fail_("invalid JSON literal");
    }
    position_ += expected.size();
    return true;
  }

  bool parse_array_(size_t depth, std::vector<std::vector<JsonObjectField>> *objects = nullptr) {
    if (depth > 16U) {
      return fail_("JSON nesting limit exceeded");
    }
    if (!consume_('[')) {
      return fail_("expected JSON array");
    }
    skip_space_();
    if (consume_(']')) {
      return true;
    }
    while (true) {
      std::vector<JsonObjectField> fields;
      if (objects != nullptr ? !parse_object_(&fields, depth + 1U)
                             : !parse_value_(nullptr, nullptr, depth + 1U)) {
        return false;
      }
      if (objects != nullptr) {
        objects->push_back(std::move(fields));
      }
      skip_space_();
      if (consume_(']')) {
        return true;
      }
      if (!consume_(',')) {
        return fail_("expected comma in JSON array");
      }
      skip_space_();
    }
  }

  bool parse_object_(std::vector<JsonObjectField> *fields, size_t depth) {
    if (depth > 16U) {
      return fail_("JSON nesting limit exceeded");
    }
    if (!consume_('{')) {
      return fail_("expected JSON object");
    }
    skip_space_();
    if (consume_('}')) {
      return true;
    }
    while (true) {
      std::string name;
      if (!parse_string_(&name)) {
        return false;
      }
      if (fields != nullptr) {
        for (const auto &field : *fields) {
          if (field.name == name) {
            return fail_("duplicate JSON object field");
          }
        }
      }
      skip_space_();
      if (!consume_(':')) {
        return fail_("expected colon in JSON object");
      }
      skip_space_();
      JsonObjectField field;
      field.name = std::move(name);
      if (!parse_value_(fields != nullptr ? &field.type : nullptr, fields != nullptr ? &field.value : nullptr,
                        depth + 1U)) {
        return false;
      }
      if (fields != nullptr) {
        fields->push_back(std::move(field));
      }
      skip_space_();
      if (consume_('}')) {
        return true;
      }
      if (!consume_(',')) {
        return fail_("expected comma in JSON object");
      }
      skip_space_();
    }
  }

  bool parse_value_(JsonValueType *type, std::string *value, size_t depth) {
    if (position_ >= input_.size()) {
      return fail_("missing JSON value");
    }
    const size_t begin = position_;
    const char ch = input_[position_];
    if (ch == '"') {
      if (type != nullptr) {
        *type = JsonValueType::STRING;
      }
      return parse_string_(value);
    }
    bool accepted = false;
    JsonValueType parsed_type = JsonValueType::NULL_VALUE;
    if (ch == '{') {
      parsed_type = JsonValueType::OBJECT;
      accepted = parse_object_(nullptr, depth);
    } else if (ch == '[') {
      parsed_type = JsonValueType::ARRAY;
      accepted = parse_array_(depth);
    } else if (ch == 't') {
      parsed_type = JsonValueType::BOOLEAN;
      accepted = parse_literal_("true");
    } else if (ch == 'f') {
      parsed_type = JsonValueType::BOOLEAN;
      accepted = parse_literal_("false");
    } else if (ch == 'n') {
      parsed_type = JsonValueType::NULL_VALUE;
      accepted = parse_literal_("null");
    } else if (ch == '-' || std::isdigit(static_cast<unsigned char>(ch))) {
      parsed_type = JsonValueType::NUMBER;
      accepted = parse_number_();
    } else {
      return fail_("invalid JSON value");
    }
    if (!accepted) {
      return false;
    }
    if (type != nullptr) {
      *type = parsed_type;
    }
    if (value != nullptr) {
      *value = input_.substr(begin, position_ - begin);
    }
    return true;
  }

  const std::string &input_;
  size_t position_{0U};
  std::string *error_{nullptr};
};

}  // namespace

void append_json_string(std::string *out, const char *value) {
  if (out == nullptr) {
    return;
  }
  out->push_back('"');
  if (value != nullptr) {
    for (const char *p = value; *p != '\0'; ++p) {
      switch (*p) {
        case '"':
          out->append("\\\"");
          break;
        case '\\':
          out->append("\\\\");
          break;
        case '\n':
          out->append("\\n");
          break;
        case '\r':
          out->append("\\r");
          break;
        case '\t':
          out->append("\\t");
          break;
        default:
          out->push_back(*p);
          break;
      }
    }
  }
  out->push_back('"');
}

void append_json_pair(std::string *out, const char *key, const char *value, bool first) {
  if (out == nullptr) {
    return;
  }
  if (!first) {
    out->append(",");
  }
  append_json_string(out, key);
  out->append(":");
  append_json_string(out, value);
}

bool has_json_key(const std::string &payload, const char *key) {
  if (key == nullptr || key[0] == '\0') {
    return false;
  }
  const std::string needle = std::string("\"") + key + "\"";
  const size_t key_pos = payload.find(needle);
  return key_pos != std::string::npos && payload.find(':', key_pos + needle.size()) != std::string::npos;
}

std::string extract_json_string(const std::string &payload, const char *key) {
  if (key == nullptr || key[0] == '\0') {
    return {};
  }
  const std::string needle = std::string("\"") + key + "\"";
  const size_t key_pos = payload.find(needle);
  if (key_pos == std::string::npos) {
    return {};
  }
  const size_t colon = payload.find(':', key_pos + needle.size());
  if (colon == std::string::npos) {
    return {};
  }
  const size_t first_quote = payload.find('"', colon + 1);
  if (first_quote == std::string::npos) {
    return {};
  }
  std::string value;
  bool escaped = false;
  for (size_t i = first_quote + 1; i < payload.size(); ++i) {
    const char ch = payload[i];
    if (escaped) {
      value.push_back(ch);
      escaped = false;
      continue;
    }
    if (ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      return value;
    }
    value.push_back(ch);
  }
  return {};
}

std::string extract_json_number_token(const std::string &payload, const char *key) {
  if (key == nullptr || key[0] == '\0') {
    return {};
  }
  const std::string needle = std::string("\"") + key + "\"";
  const size_t key_pos = payload.find(needle);
  if (key_pos == std::string::npos) {
    return {};
  }
  const size_t colon = payload.find(':', key_pos + needle.size());
  if (colon == std::string::npos) {
    return {};
  }
  size_t begin = payload.find_first_not_of(" \t\r\n", colon + 1);
  if (begin == std::string::npos) {
    return {};
  }
  size_t end = begin;
  while (end < payload.size()) {
    const char ch = payload[end];
    if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E') {
      ++end;
      continue;
    }
    break;
  }
  return payload.substr(begin, end - begin);
}

bool decode_urlencoded_component(const std::string &encoded, std::string *decoded, std::string *error) {
  if (decoded == nullptr) {
    if (error != nullptr) {
      *error = "decoded output is required";
    }
    return false;
  }
  decoded->clear();
  decoded->reserve(encoded.size());
  for (size_t i = 0; i < encoded.size(); ++i) {
    const char ch = encoded[i];
    if (ch == '+') {
      decoded->push_back(' ');
      continue;
    }
    if (ch != '%') {
      decoded->push_back(ch);
      continue;
    }
    if (i + 2 >= encoded.size()) {
      if (error != nullptr) {
        *error = "truncated escape sequence";
      }
      return false;
    }
    const char hi = encoded[i + 1];
    const char lo = encoded[i + 2];
    if (!std::isxdigit(static_cast<unsigned char>(hi)) || !std::isxdigit(static_cast<unsigned char>(lo))) {
      if (error != nullptr) {
        *error = "invalid escape sequence";
      }
      return false;
    }
    const auto decode_hex = [](char value) -> uint8_t {
      if (value >= '0' && value <= '9') {
        return static_cast<uint8_t>(value - '0');
      }
      if (value >= 'a' && value <= 'f') {
        return static_cast<uint8_t>(10 + (value - 'a'));
      }
      return static_cast<uint8_t>(10 + (value - 'A'));
    };
    const uint8_t decoded_byte = static_cast<uint8_t>((decode_hex(hi) << 4U) | decode_hex(lo));
    decoded->push_back(static_cast<char>(decoded_byte));
    i += 2;
  }
  return true;
}

std::string encode_urlencoded_component(const std::string &value) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  std::string encoded;
  encoded.reserve(value.size());
  for (const unsigned char ch : value) {
    if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '-' ||
        ch == '_' || ch == '.' || ch == '~') {
      encoded.push_back(static_cast<char>(ch));
      continue;
    }
    encoded.push_back('%');
    encoded.push_back(kHex[(ch >> 4U) & 0x0FU]);
    encoded.push_back(kHex[ch & 0x0FU]);
  }
  return encoded;
}

bool parse_urlencoded_key_value_pairs(const std::string &payload,
                                      std::vector<std::pair<std::string, std::string>> *pairs,
                                      std::string *error) {
  if (pairs == nullptr) {
    if (error != nullptr) {
      *error = "pairs output is required";
    }
    return false;
  }
  pairs->clear();
  if (payload.empty()) {
    if (error != nullptr) {
      *error = "missing payload";
    }
    return false;
  }

  size_t begin = 0;
  while (begin <= payload.size()) {
    const size_t end = payload.find('&', begin);
    const std::string token = payload.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
    if (token.empty()) {
      if (error != nullptr) {
        *error = "empty key-value token";
      }
      return false;
    }
    const size_t eq = token.find('=');
    if (eq == std::string::npos || eq == 0) {
      if (error != nullptr) {
        *error = "invalid key-value token";
      }
      return false;
    }
    std::string key;
    std::string value;
    if (!decode_urlencoded_component(token.substr(0, eq), &key, error) ||
        !decode_urlencoded_component(token.substr(eq + 1), &value, error)) {
      return false;
    }
    if (key.empty()) {
      if (error != nullptr) {
        *error = "empty key";
      }
      return false;
    }
    pairs->emplace_back(std::move(key), std::move(value));
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1;
  }
  return !pairs->empty();
}

bool parse_json_object_fields(const std::string &payload,
                              std::vector<JsonObjectField> *fields,
                              std::string *error) {
  JsonReader reader(payload);
  return reader.parse_object(fields, error);
}

bool parse_json_array_objects(const std::string &payload,
                              std::vector<std::vector<JsonObjectField>> *objects,
                              std::string *error) {
  JsonReader reader(payload);
  return reader.parse_array_objects(objects, error);
}

const JsonObjectField *find_json_object_field(const std::vector<JsonObjectField> &fields, const char *name) {
  if (name == nullptr) {
    return nullptr;
  }
  for (const auto &field : fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

}  // namespace espectre
