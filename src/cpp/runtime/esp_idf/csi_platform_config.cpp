/*
 * ESPectre - CSI Platform Configuration Helpers
 *
 * Builds ESP-IDF CSI capture settings for the selected sensing profile.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "csi_platform_config.h"

#include "sdkconfig.h"

namespace espectre {

wifi_csi_config_t build_csi_config(CsiCaptureProfile profile) {
  const bool use_lltf = csi_capture_profile_uses_lltf(profile);
#if CONFIG_IDF_TARGET_ESP32C5
  const bool use_vht = profile == CsiCaptureProfile::VHT20;
  return wifi_csi_config_t{
      .enable = 1,
      .acquire_csi_legacy = use_lltf,
      .acquire_csi_force_lltf = 0,
      .acquire_csi_ht20 = !use_vht && !use_lltf,
      .acquire_csi_ht40 = 0,
      .acquire_csi_vht = use_vht,
      .acquire_csi_su = 0,
      .acquire_csi_mu = 0,
      .acquire_csi_dcm = 0,
      .acquire_csi_beamformed = 0,
      .acquire_csi_he_stbc_mode = 0,
      .val_scale_cfg = 0,
      .dump_ack_en = use_lltf,
      .lltf_bit_mode = use_lltf,
      .reserved = 0,
  };
#elif CONFIG_IDF_TARGET_ESP32C6
  return wifi_csi_config_t{
      .enable = 1,
      .acquire_csi_legacy = use_lltf,
      .acquire_csi_ht20 = !use_lltf,
      .acquire_csi_ht40 = 0,
      .acquire_csi_su = 0,
      .acquire_csi_mu = 0,
      .acquire_csi_dcm = 0,
      .acquire_csi_beamformed = 0,
      .acquire_csi_he_stbc = 0,
      .val_scale_cfg = 0,
      .dump_ack_en = use_lltf,
      .reserved = 0,
  };
#else
  return wifi_csi_config_t{
      .lltf_en = use_lltf,
      .htltf_en = !use_lltf,
      .stbc_htltf2_en = false,
      .ltf_merge_en = false,
      .channel_filter_en = false,
      .manu_scale = false,
      .shift = 0,
      .dump_ack_en = use_lltf,
  };
#endif
}

esp_err_t configure_csi(IWiFiCSI *wifi_csi, CsiCaptureProfile profile) {
  if (wifi_csi == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  const wifi_csi_config_t csi_config = build_csi_config(profile);
  return wifi_csi->set_csi_config(&csi_config);
}

}  // namespace espectre
