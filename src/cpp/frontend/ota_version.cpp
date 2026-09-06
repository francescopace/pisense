/*
 * ESPectre - OTA Version Ordering
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "ota_version.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace espectre {

namespace {

struct ParsedVersion {
  std::array<uint64_t, 3U> core{};
  std::vector<std::string> prerelease;
  uint64_t commit_distance{0U};
  std::string git_hash;
};

bool parse_unsigned(const std::string &value, uint64_t *parsed) {
  if (parsed == nullptr || value.empty()) {
    return false;
  }
  uint64_t result = 0U;
  for (char character : value) {
    if (!std::isdigit(static_cast<unsigned char>(character))) {
      return false;
    }
    const uint64_t digit = static_cast<uint64_t>(character - '0');
    if (result > (std::numeric_limits<uint64_t>::max() - digit) / 10U) {
      return false;
    }
    result = result * 10U + digit;
  }
  *parsed = result;
  return true;
}

std::vector<std::string> split(const std::string &value, char delimiter) {
  std::vector<std::string> parts;
  size_t start = 0U;
  while (start <= value.size()) {
    const size_t end = value.find(delimiter, start);
    parts.push_back(value.substr(start, end == std::string::npos ? std::string::npos : end - start));
    if (end == std::string::npos) {
      break;
    }
    start = end + 1U;
  }
  return parts;
}

bool is_hex_string(const std::string &value) {
  return !value.empty() && std::all_of(value.begin(), value.end(), [](char character) {
           return std::isxdigit(static_cast<unsigned char>(character)) != 0;
         });
}

bool is_valid_prerelease_identifier(const std::string &value) {
  return !value.empty() && std::all_of(value.begin(), value.end(), [](char character) {
           return std::isalnum(static_cast<unsigned char>(character)) != 0 || character == '-';
         });
}

bool parse_version(std::string value, ParsedVersion *parsed) {
  if (parsed == nullptr || value.empty()) {
    return false;
  }
  if (value.size() > 6U && value.compare(value.size() - 6U, 6U, "-dirty") == 0) {
    value.resize(value.size() - 6U);
  }
  const size_t build_metadata = value.find('+');
  if (build_metadata != std::string::npos) {
    value.resize(build_metadata);
  }

  ParsedVersion result;
  const size_t git_marker = value.rfind("-g");
  if (git_marker != std::string::npos) {
    result.git_hash = value.substr(git_marker + 2U);
    const size_t distance_separator = value.rfind('-', git_marker - 1U);
    if (distance_separator == std::string::npos || !is_hex_string(result.git_hash) ||
        !parse_unsigned(value.substr(distance_separator + 1U, git_marker - distance_separator - 1U),
                        &result.commit_distance)) {
      return false;
    }
    value.resize(distance_separator);
  }

  const size_t prerelease_separator = value.find('-');
  const std::string core = value.substr(0U, prerelease_separator);
  const std::vector<std::string> core_parts = split(core, '.');
  if (core_parts.size() != result.core.size()) {
    return false;
  }
  for (size_t index = 0U; index < core_parts.size(); ++index) {
    if (!parse_unsigned(core_parts[index], &result.core[index])) {
      return false;
    }
  }
  if (prerelease_separator != std::string::npos) {
    result.prerelease = split(value.substr(prerelease_separator + 1U), '.');
    if (!std::all_of(result.prerelease.begin(), result.prerelease.end(), is_valid_prerelease_identifier)) {
      return false;
    }
  }
  *parsed = std::move(result);
  return true;
}

int compare_numeric_strings(const std::string &left, const std::string &right) {
  const size_t left_start = left.find_first_not_of('0');
  const size_t right_start = right.find_first_not_of('0');
  const std::string left_normalized = left_start == std::string::npos ? "0" : left.substr(left_start);
  const std::string right_normalized = right_start == std::string::npos ? "0" : right.substr(right_start);
  if (left_normalized.size() != right_normalized.size()) {
    return left_normalized.size() < right_normalized.size() ? -1 : 1;
  }
  if (left_normalized == right_normalized) {
    return 0;
  }
  return left_normalized < right_normalized ? -1 : 1;
}

int compare_prerelease(const std::vector<std::string> &candidate,
                       const std::vector<std::string> &current) {
  if (candidate.empty() != current.empty()) {
    return candidate.empty() ? 1 : -1;
  }
  for (size_t index = 0U; index < std::min(candidate.size(), current.size()); ++index) {
    if (candidate[index] == current[index]) {
      continue;
    }
    const bool candidate_numeric = std::all_of(candidate[index].begin(), candidate[index].end(), [](char value) {
      return std::isdigit(static_cast<unsigned char>(value)) != 0;
    });
    const bool current_numeric = std::all_of(current[index].begin(), current[index].end(), [](char value) {
      return std::isdigit(static_cast<unsigned char>(value)) != 0;
    });
    if (candidate_numeric != current_numeric) {
      return candidate_numeric ? -1 : 1;
    }
    return candidate_numeric ? compare_numeric_strings(candidate[index], current[index])
                             : (candidate[index] < current[index] ? -1 : 1);
  }
  if (candidate.size() == current.size()) {
    return 0;
  }
  return candidate.size() < current.size() ? -1 : 1;
}

int compare_parsed_versions(const ParsedVersion &candidate, const ParsedVersion &current) {
  for (size_t index = 0U; index < candidate.core.size(); ++index) {
    if (candidate.core[index] != current.core[index]) {
      return candidate.core[index] < current.core[index] ? -1 : 1;
    }
  }
  const int prerelease_order = compare_prerelease(candidate.prerelease, current.prerelease);
  if (prerelease_order != 0) {
    return prerelease_order;
  }
  if (candidate.commit_distance == current.commit_distance) {
    return 0;
  }
  return candidate.commit_distance < current.commit_distance ? -1 : 1;
}

}  // namespace

OtaVersionComparison compare_ota_versions(const std::string &candidate,
                                          const std::string &current) {
  ParsedVersion parsed_candidate;
  if (!parse_version(candidate, &parsed_candidate)) {
    return OtaVersionComparison::UNORDERED;
  }
  if (current.empty() || current == "unknown") {
    return OtaVersionComparison::NEWER;
  }
  ParsedVersion parsed_current;
  if (!parse_version(current, &parsed_current)) {
    return OtaVersionComparison::UNORDERED;
  }
  const int order = compare_parsed_versions(parsed_candidate, parsed_current);
  if (order < 0) {
    return OtaVersionComparison::OLDER;
  }
  if (order > 0) {
    return OtaVersionComparison::NEWER;
  }
  if (parsed_candidate.git_hash != parsed_current.git_hash &&
      (!parsed_candidate.git_hash.empty() || !parsed_current.git_hash.empty())) {
    return OtaVersionComparison::UNORDERED;
  }
  return OtaVersionComparison::SAME;
}

}  // namespace espectre
