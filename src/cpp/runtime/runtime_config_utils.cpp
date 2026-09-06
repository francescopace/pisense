/*
 * ESPectre - Runtime Config Utils
 *
 * Helpers for normalizing and applying runtime configuration.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "runtime_config_utils.h"

#include <cmath>
#include <cstring>

namespace espectre {

bool validate_runtime_threshold(float threshold) {
  return std::isfinite(threshold) && threshold >= RUNTIME_THRESHOLD_MIN && threshold <= RUNTIME_THRESHOLD_MAX;
}

bool validate_runtime_threshold_for_algorithm(float threshold, DetectionAlgorithm algorithm) {
  return std::isfinite(threshold) && threshold >= RUNTIME_THRESHOLD_MIN &&
         threshold <= runtime_threshold_max(algorithm);
}

bool validate_runtime_float(float value, float min_value, float max_value) {
  return std::isfinite(value) && value >= min_value && value <= max_value;
}

bool validate_runtime_uint32(uint32_t value, uint32_t min_value, uint32_t max_value) {
  return value >= min_value && value <= max_value;
}

bool validate_runtime_uint8(uint8_t value, uint8_t min_value, uint8_t max_value) {
  return value >= min_value && value <= max_value;
}

namespace {

bool wifi_band_policy_valid(WifiBandPolicy policy) {
  return policy == WifiBandPolicy::BAND_2G || policy == WifiBandPolicy::BAND_5G ||
         policy == WifiBandPolicy::AUTO;
}

bool multicast_group_valid(const std::string &group) {
  if (group.empty()) {
    return true;
  }

  unsigned octets[4]{};
  size_t index = 0U;
  size_t offset = 0U;
  while (index < 4U && offset < group.size()) {
    unsigned value = 0U;
    size_t digits = 0U;
    while (offset < group.size() && group[offset] >= '0' && group[offset] <= '9') {
      value = value * 10U + static_cast<unsigned>(group[offset] - '0');
      if (value > 255U) {
        return false;
      }
      ++offset;
      ++digits;
    }
    if (digits == 0U) {
      return false;
    }
    octets[index++] = value;
    if (index < 4U) {
      if (offset >= group.size() || group[offset] != '.') {
        return false;
      }
      ++offset;
    }
  }
  return index == 4U && offset == group.size() && octets[0] >= 224U && octets[0] <= 239U;
}

}  // namespace

RuntimeConfigError validate_runtime_config(const RuntimeConfig &config) {
  if (!runtime_profile_valid(config.runtime_profile)) return RuntimeConfigError::RUNTIME_PROFILE;
  if (!wifi_band_policy_valid(config.wifi_band_policy)) return RuntimeConfigError::WIFI_BAND_POLICY;
  if (!validate_runtime_uint32(config.csi_target_pps, RUNTIME_CSI_TARGET_PPS_MIN,
                               RUNTIME_CSI_TARGET_PPS_MAX)) {
    return RuntimeConfigError::CSI_TARGET_PPS;
  }
  if (!runtime_traffic_mode_valid(config.traffic_generator_mode)) {
    return RuntimeConfigError::TRAFFIC_GENERATOR_MODE;
  }
  if (!runtime_csi_traffic_mode_valid_for_profile(config.runtime_profile,
                                                  config.csi_traffic_mode)) {
    return RuntimeConfigError::CSI_TRAFFIC_MODE;
  }
  if (config.csi_traffic_mode == CsiTrafficMode::EXTERNAL) {
    if (config.csi_traffic_udp_port < RUNTIME_NETWORK_PORT_MIN) {
      return RuntimeConfigError::CSI_TRAFFIC_UDP_PORT;
    }
    if (!multicast_group_valid(config.csi_traffic_multicast_group)) {
      return RuntimeConfigError::CSI_TRAFFIC_MULTICAST_GROUP;
    }
  }

  {
    if (!runtime_detection_algorithm_valid(config.detection_algorithm)) {
      return RuntimeConfigError::DETECTION_ALGORITHM;
    }
    if (!validate_runtime_threshold_for_algorithm(config.segmentation_threshold,
                                                  config.detection_algorithm)) {
      return RuntimeConfigError::SEGMENTATION_THRESHOLD;
    }
    if (!validate_runtime_uint32(config.segmentation_window_size_ms,
                                 RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_MIN,
                                 RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_MAX)) {
      return RuntimeConfigError::SEGMENTATION_WINDOW_SIZE_MS;
    }
    if (!validate_runtime_uint32(config.evaluation_interval_ms,
                                 RUNTIME_EVALUATION_INTERVAL_MS_MIN,
                                 RUNTIME_EVALUATION_INTERVAL_MS_MAX)) {
      return RuntimeConfigError::EVALUATION_INTERVAL_MS;
    }
    if (!validate_runtime_uint8(config.motion_on_hits, RUNTIME_MOTION_HITS_MIN,
                                RUNTIME_MOTION_HITS_MAX) ||
        !validate_runtime_uint8(config.motion_off_hits, RUNTIME_MOTION_HITS_MIN,
                                RUNTIME_MOTION_HITS_MAX)) {
      return RuntimeConfigError::MOTION_HITS;
    }
    if (config.lowpass_enabled &&
        !validate_runtime_float(config.lowpass_cutoff, RUNTIME_LOWPASS_CUTOFF_MIN,
                                RUNTIME_LOWPASS_CUTOFF_MAX)) {
      return RuntimeConfigError::LOWPASS_CUTOFF;
    }
    if (config.hampel_enabled &&
        !validate_runtime_uint8(config.hampel_window, RUNTIME_HAMPEL_WINDOW_MIN,
                                RUNTIME_HAMPEL_WINDOW_MAX)) {
      return RuntimeConfigError::HAMPEL_WINDOW;
    }
    if (config.hampel_enabled &&
        !validate_runtime_float(config.hampel_threshold, RUNTIME_HAMPEL_THRESHOLD_MIN,
                                RUNTIME_HAMPEL_THRESHOLD_MAX)) {
      return RuntimeConfigError::HAMPEL_THRESHOLD;
    }
  }
  return RuntimeConfigError::NONE;
}

const char *runtime_config_error_message(RuntimeConfigError error) {
  switch (error) {
    case RuntimeConfigError::NONE: return "valid configuration";
    case RuntimeConfigError::RUNTIME_PROFILE: return "invalid runtime profile";
    case RuntimeConfigError::WIFI_BAND_POLICY: return "invalid Wi-Fi band policy";
    case RuntimeConfigError::DETECTION_ALGORITHM: return "invalid detection algorithm";
    case RuntimeConfigError::SEGMENTATION_THRESHOLD: return "invalid segmentation threshold";
    case RuntimeConfigError::SEGMENTATION_WINDOW_SIZE_MS: return "invalid segmentation window duration";
    case RuntimeConfigError::CSI_TARGET_PPS: return "invalid CSI target PPS";
    case RuntimeConfigError::TRAFFIC_GENERATOR_MODE: return "invalid traffic generator mode";
    case RuntimeConfigError::CSI_TRAFFIC_MODE: return "invalid CSI traffic mode for runtime profile";
    case RuntimeConfigError::CSI_TRAFFIC_UDP_PORT: return "invalid CSI traffic UDP port";
    case RuntimeConfigError::CSI_TRAFFIC_MULTICAST_GROUP: return "invalid CSI multicast group";
    case RuntimeConfigError::EVALUATION_INTERVAL_MS: return "invalid evaluation interval";
    case RuntimeConfigError::MOTION_HITS: return "invalid motion hit counts";
    case RuntimeConfigError::LOWPASS_CUTOFF: return "invalid low-pass cutoff";
    case RuntimeConfigError::HAMPEL_WINDOW: return "invalid Hampel window";
    case RuntimeConfigError::HAMPEL_THRESHOLD: return "invalid Hampel threshold";
  }
  return "unknown configuration error";
}

const char *runtime_profile_name(RuntimeProfile profile) {
  (void) profile;
  return "sensing";
}

const char *wifi_band_policy_name(WifiBandPolicy policy) {
  switch (policy) {
    case WifiBandPolicy::BAND_5G:
      return "5g";
    case WifiBandPolicy::AUTO:
      return "auto";
    case WifiBandPolicy::BAND_2G:
    default:
      return "2g";
  }
}

const char *traffic_mode_name(RuntimeTrafficMode mode) {
  switch (mode) {
    case RuntimeTrafficMode::PING:
      return RUNTIME_TRAFFIC_GENERATOR_MODE_PING_NAME;
    case RuntimeTrafficMode::DNS:
      return RUNTIME_TRAFFIC_GENERATOR_MODE_DNS_NAME;
    case RuntimeTrafficMode::DNS_TCP:
      return RUNTIME_TRAFFIC_GENERATOR_MODE_DNS_TCP_NAME;
    case RuntimeTrafficMode::WIFI_RAW:
      return RUNTIME_TRAFFIC_GENERATOR_MODE_WIFI_RAW_NAME;
    default:
      return RUNTIME_TRAFFIC_GENERATOR_MODE_PING_NAME;
  }
}

const char *csi_traffic_mode_name(CsiTrafficMode mode) {
  switch (mode) {
    case CsiTrafficMode::EXTERNAL:
      return RUNTIME_CSI_TRAFFIC_MODE_EXTERNAL_NAME;
    case CsiTrafficMode::INTERNAL:
    default:
      return RUNTIME_CSI_TRAFFIC_MODE_INTERNAL_NAME;
  }
}

bool csi_traffic_mode_is_sensing_control(CsiTrafficMode mode) {
  return mode == CsiTrafficMode::INTERNAL || mode == CsiTrafficMode::EXTERNAL;
}

CsiTrafficMode normalize_sensing_csi_traffic_mode(CsiTrafficMode mode) {
  return runtime_csi_traffic_mode_valid(mode) ? mode : CsiTrafficMode::INTERNAL;
}

const char *detection_algorithm_name(DetectionAlgorithm algorithm) {
  switch (algorithm) {
    case DetectionAlgorithm::HIGH_ACCURACY:
      return RUNTIME_DETECTION_ALGORITHM_HIGH_ACCURACY_NAME;
    case DetectionAlgorithm::LIGHTWEIGHT:
    default:
      return RUNTIME_DETECTION_ALGORITHM_LIGHTWEIGHT_NAME;
  }
}

const char *subcarrier_source_name(RuntimeSubcarrierSource source) {
  switch (source) {
    case RuntimeSubcarrierSource::FIXED_DEFAULT:
    default:
      return "fixed";
  }
}

RuntimeTrafficMode parse_traffic_mode(const char *mode) {
  if (mode != nullptr && std::strcmp(mode, RUNTIME_TRAFFIC_GENERATOR_MODE_WIFI_RAW_NAME) == 0) {
    return RuntimeTrafficMode::WIFI_RAW;
  }
  if (mode != nullptr && std::strcmp(mode, RUNTIME_TRAFFIC_GENERATOR_MODE_DNS_NAME) == 0) {
    return RuntimeTrafficMode::DNS;
  }
  if (mode != nullptr && std::strcmp(mode, RUNTIME_TRAFFIC_GENERATOR_MODE_DNS_TCP_NAME) == 0) {
    return RuntimeTrafficMode::DNS_TCP;
  }
  return RuntimeTrafficMode::PING;
}

CsiTrafficMode parse_csi_traffic_mode(const char *mode) {
  if (mode != nullptr && std::strcmp(mode, RUNTIME_CSI_TRAFFIC_MODE_EXTERNAL_NAME) == 0) {
    return CsiTrafficMode::EXTERNAL;
  }
  return CsiTrafficMode::INTERNAL;
}

DetectionAlgorithm parse_detection_algorithm(const char *algorithm) {
  return (algorithm != nullptr && std::strcmp(algorithm, RUNTIME_DETECTION_ALGORITHM_HIGH_ACCURACY_NAME) == 0)
             ? DetectionAlgorithm::HIGH_ACCURACY
             : DetectionAlgorithm::LIGHTWEIGHT;
}

WifiBandPolicy parse_wifi_band_policy(const char *policy) {
  if (policy != nullptr && std::strcmp(policy, "5g") == 0) {
    return WifiBandPolicy::BAND_5G;
  }
  if (policy != nullptr && std::strcmp(policy, "auto") == 0) {
    return WifiBandPolicy::AUTO;
  }
  return WifiBandPolicy::BAND_2G;
}

RuntimeConfig make_runtime_sensing_config() { return RuntimeConfig{}; }

}  // namespace espectre
