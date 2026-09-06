/*
 * ESPectre - OTA Version Ordering
 *
 * Orders release and git-describe firmware identities for OTA eligibility.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstdint>
#include <string>

namespace espectre {

enum class OtaVersionComparison : int8_t {
  OLDER = -1,
  SAME = 0,
  NEWER = 1,
  UNORDERED = 2,
};

/**
 * Compare a candidate OTA version with the firmware currently running.
 *
 * Numeric `MAJOR.MINOR.PATCH` tags, SemVer prereleases, and Git identities
 * such as `2.8.0-280-gac7af68` are ordered. A valid candidate is considered
 * newer than the legacy `unknown` current identity. Invalid candidates, or
 * two divergent Git identities at the same commit distance, are unordered.
 */
OtaVersionComparison compare_ota_versions(const std::string &candidate,
                                          const std::string &current);

}  // namespace espectre
