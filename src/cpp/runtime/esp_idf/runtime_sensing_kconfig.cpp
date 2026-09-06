/*
 * ESPectre - Runtime Sensing Kconfig
 *
 * Builds the default sensing runtime configuration from Kconfig values.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "runtime_sensing_kconfig.h"

#include <cerrno>
#include <cstdlib>

#include "espectre_log.h"
#include "runtime_config_utils.h"
#include "sdkconfig.h"

#ifndef CONFIG_ESPECTRE_DETECTION_ALGORITHM_LIGHTWEIGHT
#define CONFIG_ESPECTRE_DETECTION_ALGORITHM_LIGHTWEIGHT 1
#endif
#if !defined(CONFIG_ESPECTRE_WIFI_BAND_2G) && \
    !defined(CONFIG_ESPECTRE_WIFI_BAND_5G) && \
    !defined(CONFIG_ESPECTRE_WIFI_BAND_AUTO)
#if defined(CONFIG_SOC_WIFI_SUPPORT_5G) && CONFIG_SOC_WIFI_SUPPORT_5G
#define CONFIG_ESPECTRE_WIFI_BAND_AUTO 1
#else
#define CONFIG_ESPECTRE_WIFI_BAND_2G 1
#endif
#endif
#ifndef CONFIG_ESPECTRE_WIFI_BAND_2G
#define CONFIG_ESPECTRE_WIFI_BAND_2G 0
#endif
#ifndef CONFIG_ESPECTRE_WIFI_BAND_5G
#define CONFIG_ESPECTRE_WIFI_BAND_5G 0
#endif
#ifndef CONFIG_ESPECTRE_WIFI_BAND_AUTO
#define CONFIG_ESPECTRE_WIFI_BAND_AUTO 0
#endif
#ifndef CONFIG_ESPECTRE_DETECTION_ALGORITHM_HIGH_ACCURACY
#define CONFIG_ESPECTRE_DETECTION_ALGORITHM_HIGH_ACCURACY 0
#endif
#ifndef CONFIG_ESPECTRE_SEGMENTATION_WINDOW_SIZE_MS
#define CONFIG_ESPECTRE_SEGMENTATION_WINDOW_SIZE_MS 1000
#endif
#ifndef CONFIG_ESPECTRE_CSI_TARGET_PPS
#define CONFIG_ESPECTRE_CSI_TARGET_PPS 100
#endif
#ifndef CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_PING
#define CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_PING 1
#endif
#ifndef CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_DNS
#define CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_DNS 0
#endif
#ifndef CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_DNS_TCP
#define CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_DNS_TCP 0
#endif
#ifndef CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_WIFI_RAW
#define CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_WIFI_RAW 0
#endif
#ifndef CONFIG_ESPECTRE_EVALUATION_INTERVAL_MS
#define CONFIG_ESPECTRE_EVALUATION_INTERVAL_MS 250
#endif
#ifndef CONFIG_ESPECTRE_MOTION_ON_HITS
#define CONFIG_ESPECTRE_MOTION_ON_HITS 4
#endif
#ifndef CONFIG_ESPECTRE_MOTION_OFF_HITS
#define CONFIG_ESPECTRE_MOTION_OFF_HITS 3
#endif
#ifndef CONFIG_ESPECTRE_LOWPASS_ENABLED
#define CONFIG_ESPECTRE_LOWPASS_ENABLED 0
#endif
#ifndef CONFIG_ESPECTRE_LOWPASS_CUTOFF
#define CONFIG_ESPECTRE_LOWPASS_CUTOFF "11.0"
#endif
#ifndef CONFIG_ESPECTRE_HAMPEL_ENABLED
#define CONFIG_ESPECTRE_HAMPEL_ENABLED 1
#endif
#ifndef CONFIG_ESPECTRE_HAMPEL_WINDOW
#define CONFIG_ESPECTRE_HAMPEL_WINDOW 7
#endif
#ifndef CONFIG_ESPECTRE_HAMPEL_THRESHOLD
#define CONFIG_ESPECTRE_HAMPEL_THRESHOLD "5.0"
#endif
#ifndef CONFIG_ESPECTRE_CSI_TRAFFIC_MULTICAST_GROUP
#define CONFIG_ESPECTRE_CSI_TRAFFIC_MULTICAST_GROUP "239.255.0.1"
#endif

namespace espectre {

namespace {

static const char *const TAG = "espectre.runtime.cfg";

float parse_float_or_default_(const char *value, float default_value, float min_value, float max_value,
                              const char *key) {
  if (value == nullptr || value[0] == '\0') {
    return default_value;
  }
  char *end_ptr = nullptr;
  errno = 0;
  const float parsed = std::strtof(value, &end_ptr);
  const bool parsed_ok = end_ptr != value && end_ptr != nullptr && *end_ptr == '\0' && errno != ERANGE &&
                         validate_runtime_float(parsed, min_value, max_value);
  if (!parsed_ok) {
    ESPECTRE_LOGW(TAG, "Invalid %s=\"%s\", using default %.3f", key, value, static_cast<double>(default_value));
    return default_value;
  }
  return parsed;
}

/**
 * Range-check an integer Kconfig value the same way floats are checked.
 *
 * Downstream clamping partly covers a bad sdkconfig, but silently: the runtime
 * then differs from what was asked for with nothing in the log to say so.
 */
uint32_t clamp_uint32_or_default_(uint32_t value, uint32_t default_value, uint32_t min_value,
                                  uint32_t max_value, const char *key) {
  if (validate_runtime_uint32(value, min_value, max_value)) {
    return value;
  }
  ESPECTRE_LOGW(TAG, "Invalid %s=%u (allowed %u-%u), using default %u", key, static_cast<unsigned>(value),
           static_cast<unsigned>(min_value), static_cast<unsigned>(max_value),
           static_cast<unsigned>(default_value));
  return default_value;
}

uint8_t clamp_uint8_or_default_(uint32_t value, uint8_t default_value, uint8_t min_value,
                                uint8_t max_value, const char *key) {
  if (validate_runtime_uint32(value, min_value, max_value)) {
    return static_cast<uint8_t>(value);
  }
  ESPECTRE_LOGW(TAG, "Invalid %s=%u (allowed %u-%u), using default %u", key, static_cast<unsigned>(value),
           static_cast<unsigned>(min_value), static_cast<unsigned>(max_value),
           static_cast<unsigned>(default_value));
  return default_value;
}

std::string multicast_group_or_default_(const char *value, const char *key) {
  RuntimeConfig probe = make_runtime_sensing_config();
  probe.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
  probe.csi_traffic_multicast_group = value != nullptr ? value : "";
  if (validate_runtime_config(probe) != RuntimeConfigError::CSI_TRAFFIC_MULTICAST_GROUP) {
    return probe.csi_traffic_multicast_group;
  }
  ESPECTRE_LOGW(TAG, "Invalid %s=\"%s\", using default %s", key,
                value != nullptr ? value : "",
                RUNTIME_CSI_TRAFFIC_MULTICAST_GROUP_DEFAULT);
  return RUNTIME_CSI_TRAFFIC_MULTICAST_GROUP_DEFAULT;
}

}  // namespace

RuntimeConfig make_runtime_sensing_config_from_kconfig() {
  RuntimeConfig config = make_runtime_sensing_config();

#if CONFIG_ESPECTRE_WIFI_BAND_5G
  config.wifi_band_policy = WifiBandPolicy::BAND_5G;
#elif CONFIG_ESPECTRE_WIFI_BAND_AUTO
  config.wifi_band_policy = WifiBandPolicy::AUTO;
#else
  config.wifi_band_policy = WifiBandPolicy::BAND_2G;
#endif

#if CONFIG_ESPECTRE_DETECTION_ALGORITHM_HIGH_ACCURACY
  config.detection_algorithm = DetectionAlgorithm::HIGH_ACCURACY;
#else
  config.detection_algorithm = DetectionAlgorithm::LIGHTWEIGHT;
#endif

  config.segmentation_threshold = runtime_default_threshold(config.detection_algorithm);

  config.segmentation_window_size_ms =
      clamp_uint32_or_default_(static_cast<uint32_t>(CONFIG_ESPECTRE_SEGMENTATION_WINDOW_SIZE_MS),
                               RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_DEFAULT,
                               RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_MIN,
                               RUNTIME_SEGMENTATION_WINDOW_SIZE_MS_MAX,
                               "CONFIG_ESPECTRE_SEGMENTATION_WINDOW_SIZE_MS");
  config.csi_target_pps =
      clamp_uint32_or_default_(static_cast<uint32_t>(CONFIG_ESPECTRE_CSI_TARGET_PPS),
                               RUNTIME_CSI_TARGET_PPS_DEFAULT,
                               RUNTIME_CSI_TARGET_PPS_MIN,
                               RUNTIME_CSI_TARGET_PPS_MAX,
                               "CONFIG_ESPECTRE_CSI_TARGET_PPS");
#if CONFIG_ESPECTRE_CSI_TRAFFIC_MODE_EXTERNAL
  config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
#else
  config.csi_traffic_mode = CsiTrafficMode::INTERNAL;
#endif
  config.csi_traffic_multicast_group = multicast_group_or_default_(
      CONFIG_ESPECTRE_CSI_TRAFFIC_MULTICAST_GROUP,
      "CONFIG_ESPECTRE_CSI_TRAFFIC_MULTICAST_GROUP");
#if CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_DNS
  config.traffic_generator_mode = RuntimeTrafficMode::DNS;
#elif CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_WIFI_RAW
  config.traffic_generator_mode = RuntimeTrafficMode::WIFI_RAW;
#elif CONFIG_ESPECTRE_TRAFFIC_GENERATOR_MODE_DNS_TCP
  config.traffic_generator_mode = RuntimeTrafficMode::DNS_TCP;
#else
  config.traffic_generator_mode = RuntimeTrafficMode::PING;
#endif
  config.evaluation_interval_ms =
      clamp_uint32_or_default_(static_cast<uint32_t>(CONFIG_ESPECTRE_EVALUATION_INTERVAL_MS),
                               RUNTIME_EVALUATION_INTERVAL_MS_DEFAULT,
                               RUNTIME_EVALUATION_INTERVAL_MS_MIN,
                               RUNTIME_EVALUATION_INTERVAL_MS_MAX,
                               "CONFIG_ESPECTRE_EVALUATION_INTERVAL_MS");
  config.motion_on_hits =
      clamp_uint8_or_default_(static_cast<uint32_t>(CONFIG_ESPECTRE_MOTION_ON_HITS),
                              RUNTIME_MOTION_ON_HITS_DEFAULT, RUNTIME_MOTION_HITS_MIN,
                              RUNTIME_MOTION_HITS_MAX, "CONFIG_ESPECTRE_MOTION_ON_HITS");
  config.motion_off_hits =
      clamp_uint8_or_default_(static_cast<uint32_t>(CONFIG_ESPECTRE_MOTION_OFF_HITS),
                              RUNTIME_MOTION_OFF_HITS_DEFAULT, RUNTIME_MOTION_HITS_MIN,
                              RUNTIME_MOTION_HITS_MAX, "CONFIG_ESPECTRE_MOTION_OFF_HITS");
  config.lowpass_enabled = CONFIG_ESPECTRE_LOWPASS_ENABLED;
  config.lowpass_cutoff = parse_float_or_default_(CONFIG_ESPECTRE_LOWPASS_CUTOFF,
                                                  RUNTIME_LOWPASS_CUTOFF_DEFAULT,
                                                  RUNTIME_LOWPASS_CUTOFF_MIN,
                                                  RUNTIME_LOWPASS_CUTOFF_MAX,
                                                  "CONFIG_ESPECTRE_LOWPASS_CUTOFF");
  config.hampel_enabled = CONFIG_ESPECTRE_HAMPEL_ENABLED;
  config.hampel_window =
      clamp_uint8_or_default_(static_cast<uint32_t>(CONFIG_ESPECTRE_HAMPEL_WINDOW),
                              RUNTIME_HAMPEL_WINDOW_DEFAULT, RUNTIME_HAMPEL_WINDOW_MIN,
                              RUNTIME_HAMPEL_WINDOW_MAX, "CONFIG_ESPECTRE_HAMPEL_WINDOW");
  config.hampel_threshold = parse_float_or_default_(CONFIG_ESPECTRE_HAMPEL_THRESHOLD,
                                                    RUNTIME_HAMPEL_THRESHOLD_DEFAULT,
                                                    RUNTIME_HAMPEL_THRESHOLD_MIN,
                                                    RUNTIME_HAMPEL_THRESHOLD_MAX,
                                                    "CONFIG_ESPECTRE_HAMPEL_THRESHOLD");

  return config;
}

}  // namespace espectre
