/*
 * ESPectre - Runtime Helper Unit Tests
 *
 * Covers lightweight runtime helpers that are easy to exercise host-side.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include "csi_capture_service.h"
#include "csi_format_classifier.h"
#include "csi_format.h"
#include "csi_platform_config.h"
#include "runtime_config_utils.h"
#include "mqtt_payload_assembler.h"
#include "runtime_diagnostics.h"
#include "runtime_performance_diagnostics.h"
#include "runtime_time.h"
#include "sta_socket_helpers.h"
#include "wifi_csi_interface.h"

#include <algorithm>
#include <string>
#include <vector>

#include "esp_timer.h"
#include "esp_netif.h"

#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#define private public
#undef private

using namespace espectre;

namespace {

void dummy_csi_callback(void *, wifi_csi_info_t *) {}

struct CapturedCsiPacket {
  std::array<int8_t, HT20_CSI_LEN> payload{};
  uint32_t callback_count{0U};
  int8_t first_value{0};
  int8_t retained_value{0};
  uint16_t info_len{0U};
  size_t normalized_len{0U};
  NormalizedCSIPayloadTag normalization_tag{NormalizedCSIPayloadTag::NONE};
  bool first_word_invalid{true};
  bool missing_lltf_bins_zero{false};
  bool rotated_to_centered{false};
};

struct CapturedChannelChange {
  uint32_t callback_count{0U};
  uint8_t previous_channel{0U};
  uint8_t current_channel{0U};
};

class CaptureWiFiMock final : public IWiFiCSI {
 public:
  esp_err_t set_csi_config(const wifi_csi_config_t *config) override {
    (void)config;
    configure_calls++;
    return ESP_OK;
  }

  esp_err_t set_csi_rx_cb(wifi_csi_cb_t cb, void *ctx) override {
    callback = cb;
    callback_context = ctx;
    callback_registration_calls++;
    return ESP_OK;
  }

  esp_err_t set_csi(bool enable) override {
    enabled = enable;
    if (enable) {
      enable_calls++;
    } else {
      disable_calls++;
    }
    return ESP_OK;
  }

  int configure_calls{0};
  int callback_registration_calls{0};
  int enable_calls{0};
  int disable_calls{0};
  bool enabled{false};
  wifi_csi_cb_t callback{nullptr};
  void *callback_context{nullptr};
};

void capture_csi_packet(void *context, const wifi_csi_info_t *info, const NormalizedCSIPayload &normalized) {
  auto *captured = static_cast<CapturedCsiPacket *>(context);
  captured->callback_count++;
  captured->first_value = normalized.valid() ? normalized.data[0] : 0;
  captured->retained_value = normalized.valid() ? normalized.data[16] : 0;
  captured->info_len = info != nullptr ? info->len : 0U;
  captured->normalized_len = normalized.len;
  captured->normalization_tag = normalized.tag;
  captured->first_word_invalid = info == nullptr || info->first_word_invalid;
  captured->missing_lltf_bins_zero = normalized.valid();
  if (normalized.valid()) {
    std::copy_n(normalized.data, normalized.len, captured->payload.data());
    for (uint8_t bin : HT20_LLTF_MISSING_BINS) {
      const size_t byte_index = static_cast<size_t>(bin) * 2U;
      captured->missing_lltf_bins_zero &=
          normalized.data[byte_index] == 0 && normalized.data[byte_index + 1U] == 0;
    }
  }
  captured->rotated_to_centered = normalized.rotated_to_centered;
}

void capture_channel_change(void *context, uint8_t previous_channel, uint8_t current_channel) {
  auto *captured = static_cast<CapturedChannelChange *>(context);
  captured->callback_count++;
  captured->previous_channel = previous_channel;
  captured->current_channel = current_channel;
}

}  // namespace

void test_wifi_csi_real_forwards_calls_to_mocked_esp_wifi(void) {
    WiFiCSIReal wifi;
    wifi_csi_config_t config{};

    TEST_ASSERT_EQUAL(ESP_OK, wifi.set_csi_config(&config));
    TEST_ASSERT_EQUAL(ESP_OK, wifi.set_csi_rx_cb(dummy_csi_callback, nullptr));
    TEST_ASSERT_EQUAL(ESP_OK, wifi.set_csi(true));
    TEST_ASSERT_EQUAL(ESP_OK, wifi.set_csi(false));
}

void test_lltf_preference_and_vht_capability_resolve_capture_profile(void) {
    TEST_ASSERT_TRUE(resolve_csi_capture_profile(true, true, 36U) ==
                     CsiCaptureProfile::LLTF20);
    TEST_ASSERT_TRUE(resolve_csi_capture_profile(false, true, 6U) ==
                     CsiCaptureProfile::HT20);
    TEST_ASSERT_TRUE(resolve_csi_capture_profile(false, true, 36U) ==
                     CsiCaptureProfile::VHT20);
    TEST_ASSERT_TRUE(resolve_csi_capture_profile(false, false, 36U) ==
                     CsiCaptureProfile::HT20);

    const wifi_csi_config_t config =
        build_csi_config(CsiCaptureProfile::LLTF20);

    TEST_ASSERT_TRUE(config.lltf_en);
    TEST_ASSERT_FALSE(config.htltf_en);
    TEST_ASSERT_FALSE(config.stbc_htltf2_en);
}

void test_lltf20_profile_inherits_supported_ht20_payload_layouts(void) {
    std::array<int8_t, HT20_CSI_LEN_DOUBLE> csi{};
    wifi_csi_info_t info{};
    info.buf = csi.data();
    info.rx_ctrl.sig_mode = 1U;
    info.rx_ctrl.cwb = 0U;

    struct LayoutCase {
      uint16_t raw_len;
      CsiLayoutId layout_id;
      NormalizedCSIPayloadTag normalization_tag;
      bool requires_normalization;
    };
    const LayoutCase cases[] = {
        {HT20_CSI_LEN, CsiLayoutId::HT20_64,
         NormalizedCSIPayloadTag::NONE, false},
        {HT20_CSI_LEN_SHORT, CsiLayoutId::HT20_57,
         NormalizedCSIPayloadTag::HT57_TO_64, true},
        {HT20_CSI_LEN_DOUBLE, CsiLayoutId::HT20_64_DOUBLE,
         NormalizedCSIPayloadTag::DOUBLE_HT20, true},
        {HT20_CSI_LEN_SHORT_DOUBLE, CsiLayoutId::HT20_57_DOUBLE,
         NormalizedCSIPayloadTag::DOUBLE_HT57_TO_64, true},
    };

    for (const LayoutCase &layout : cases) {
      info.len = layout.raw_len;
      const CsiFormatAssessment assessment =
          assess_ht20_sensing_format(&info, CsiCaptureProfile::LLTF20);

      TEST_ASSERT_TRUE(assessment.is_sensing_accepted());
      TEST_ASSERT_TRUE(assessment.layout_id == layout.layout_id);
      TEST_ASSERT_TRUE(assessment.normalization_tag ==
                       layout.normalization_tag);
      TEST_ASSERT_EQUAL(layout.requires_normalization,
                        assessment.requires_normalization());
      TEST_ASSERT_EQUAL(HT20_CSI_LEN, assessment.normalized_len);
      TEST_ASSERT_EQUAL(HT20_NUM_SUBCARRIERS,
                        assessment.normalized_num_subcarriers);
    }
}

void test_compact_lltf_preserves_tones_and_bypasses_latched_classic_order(void) {
    std::array<int8_t, LLTF20_CSI_LEN_SHORT> compact{};
    std::array<int8_t, HT20_CSI_LEN> expected{};
    for (int tone = -26; tone <= 26; ++tone) {
      const size_t source = static_cast<size_t>(tone + 26) * 2U;
      compact[source] = static_cast<int8_t>(tone);
      compact[source + 1U] = static_cast<int8_t>(-tone);
      expected[(tone + 32) * 2] = static_cast<int8_t>(tone);
      expected[(tone + 32) * 2 + 1] = static_cast<int8_t>(-tone);
    }
    std::array<int8_t, HT20_CSI_LEN> scratch{};
    scratch.fill(127);
    const auto normalized = normalize_ht20_csi_payload(
        compact.data(), compact.size(), scratch.data(), scratch.size());
    TEST_ASSERT_TRUE(normalized.valid());
    TEST_ASSERT_EQUAL(HT20_CSI_LEN, normalized.len);
    TEST_ASSERT_TRUE(normalized.tag == NormalizedCSIPayloadTag::LLTF53_TO_64);
    TEST_ASSERT_TRUE(std::equal(expected.begin(), expected.end(), normalized.data));
    TEST_ASSERT_FALSE(normalize_ht20_csi_payload(
        compact.data(), compact.size(), nullptr, scratch.size()).valid());
    TEST_ASSERT_FALSE(normalize_ht20_csi_payload(
        compact.data(), compact.size(), scratch.data(), scratch.size() - 1U).valid());

    CsiCaptureService service;
    CapturedCsiPacket captured;
    service.init();
    service.set_packet_callback(&capture_csi_packet, &captured);
    TEST_ASSERT_EQUAL(ESP_OK, service.enable(CsiCaptureProfile::LLTF20));
    // Establish classic ordering from a full-width packet before compact LLTF.
    scratch.fill(7);
    for (uint8_t bin : HT20_CLASSIC_ONLY_NULL_BINS) {
      scratch[bin * 2U] = scratch[bin * 2U + 1U] = 0;
    }
    wifi_csi_info_t info{};
    info.buf = scratch.data();
    info.len = scratch.size();
    info.rx_ctrl.channel = 6U;
    info.rx_ctrl.timestamp = 100U;
    service.process_packet(&info);
    TEST_ASSERT_TRUE(captured.rotated_to_centered);
    info.buf = compact.data();
    info.len = compact.size();
    info.rx_ctrl.timestamp++;
    service.process_packet(&info);
    TEST_ASSERT_EQUAL(2U, captured.callback_count);
    TEST_ASSERT_EQUAL(LLTF20_CSI_LEN_SHORT, captured.info_len);
    TEST_ASSERT_TRUE(expected == captured.payload);
    TEST_ASSERT_FALSE(captured.rotated_to_centered);
    TEST_ASSERT_TRUE(captured.missing_lltf_bins_zero);
    TEST_ASSERT_EQUAL(ESP_OK, service.disable());

    for (auto profile : {CsiCaptureProfile::LLTF20, CsiCaptureProfile::HT20,
                         CsiCaptureProfile::VHT20}) {
      for (uint8_t sig_mode : {0U, 1U, 3U}) {
        info.rx_ctrl.sig_mode = sig_mode;
        const auto assessment = assess_ht20_sensing_format(&info, profile);
        const bool accepted = profile == CsiCaptureProfile::LLTF20 && sig_mode == 0U;
        TEST_ASSERT_EQUAL(accepted, assessment.is_sensing_accepted());
        if (accepted) {
          TEST_ASSERT_TRUE(assessment.layout_id == CsiLayoutId::LLTF20_53);
          TEST_ASSERT_TRUE(assessment.requires_normalization());
        }
      }
    }
}

void test_lltf20_legacy_frames_reject_unrelated_payload_lengths(void) {
    std::array<int8_t, HT20_CSI_LEN_DOUBLE> csi{};
    wifi_csi_info_t info{};
    info.buf = csi.data();
    info.rx_ctrl.sig_mode = 0U;
    info.rx_ctrl.cwb = 0U;

    info.len = HT20_CSI_LEN;
    TEST_ASSERT_TRUE(assess_ht20_sensing_format(
                         &info, CsiCaptureProfile::LLTF20)
                         .is_sensing_accepted());

    const uint16_t unsupported_lengths[] = {
        HT20_CSI_LEN_SHORT,
        HT20_CSI_LEN_DOUBLE,
        HT20_CSI_LEN_SHORT_DOUBLE,
    };
    for (uint16_t raw_len : unsupported_lengths) {
      info.len = raw_len;
      const CsiFormatAssessment assessment =
          assess_ht20_sensing_format(&info, CsiCaptureProfile::LLTF20);

      TEST_ASSERT_FALSE(assessment.is_sensing_accepted());
      TEST_ASSERT_TRUE(assessment.reason_code ==
                       CsiFormatReasonCode::UNKNOWN_LAYOUT);
      TEST_ASSERT_EQUAL(0U, assessment.normalized_len);
    }
}

void test_csi_capture_service_normalizes_ht_layouts_under_lltf20(void) {
    CsiCaptureService service;
    CapturedCsiPacket captured;
    service.init();
    service.set_packet_callback(&capture_csi_packet, &captured);
    TEST_ASSERT_EQUAL(ESP_OK, service.enable(CsiCaptureProfile::LLTF20));

    std::array<int8_t, HT20_CSI_LEN_DOUBLE> csi{};
    csi.fill(7);
    wifi_csi_info_t info{};
    info.buf = csi.data();
    info.rx_ctrl.sig_mode = 1U;
    info.rx_ctrl.cwb = 0U;
    info.rx_ctrl.channel = 6U;

    struct NormalizationCase {
      uint16_t raw_len;
      NormalizedCSIPayloadTag normalization_tag;
    };
    const NormalizationCase cases[] = {
        {HT20_CSI_LEN, NormalizedCSIPayloadTag::NONE},
        {HT20_CSI_LEN_SHORT, NormalizedCSIPayloadTag::HT57_TO_64},
        {HT20_CSI_LEN_DOUBLE, NormalizedCSIPayloadTag::DOUBLE_HT20},
        {HT20_CSI_LEN_SHORT_DOUBLE,
         NormalizedCSIPayloadTag::DOUBLE_HT57_TO_64},
    };

    uint32_t timestamp = 100U;
    for (const NormalizationCase &layout : cases) {
      captured = {};
      info.len = layout.raw_len;
      info.rx_ctrl.timestamp = timestamp++;
      service.process_packet(&info);

      TEST_ASSERT_EQUAL(1U, captured.callback_count);
      TEST_ASSERT_EQUAL(layout.raw_len, captured.info_len);
      TEST_ASSERT_EQUAL(HT20_CSI_LEN, captured.normalized_len);
      TEST_ASSERT_TRUE(captured.normalization_tag ==
                       layout.normalization_tag);
      TEST_ASSERT_TRUE(captured.missing_lltf_bins_zero);
    }

    TEST_ASSERT_EQUAL(ESP_OK, service.disable());
}

void test_csi_capture_service_filters_duplicate_and_stale_timestamps(void) {
    CsiCaptureService service;
    CapturedCsiPacket captured;
    service.init();
    service.set_packet_callback(&capture_csi_packet, &captured);

    std::array<int8_t, HT20_CSI_LEN> csi{};
    wifi_csi_info_t info{};
    info.buf = csi.data();
    info.len = HT20_CSI_LEN;
    info.rx_ctrl.sig_mode = 1U;
    info.rx_ctrl.cwb = 0U;

    const uint32_t timestamps[] = {100U, 101U, 101U, 50U, 102U};
    for (uint32_t timestamp : timestamps) {
        info.rx_ctrl.timestamp = timestamp;
        service.process_packet(&info);
    }

    TEST_ASSERT_EQUAL(3U, captured.callback_count);
    TEST_ASSERT_EQUAL(3U, service.valid_packets());
    TEST_ASSERT_EQUAL(2U, service.filtered_packets());
    TEST_ASSERT_EQUAL(2U, service.rejected_out_of_order_packets());

    service.reset_session();
    info.rx_ctrl.timestamp = 50U;
    service.process_packet(&info);

    TEST_ASSERT_EQUAL(4U, captured.callback_count);
    TEST_ASSERT_EQUAL(1U, service.valid_packets());
    TEST_ASSERT_EQUAL(0U, service.filtered_packets());
    TEST_ASSERT_EQUAL(0U, service.rejected_out_of_order_packets());
}

void test_csi_capture_service_defers_channel_change_and_resets_session_baseline(void) {
    CaptureWiFiMock wifi;
    CsiCaptureService service;
    CapturedCsiPacket packets;
    CapturedChannelChange channel_change;
    service.init(&wifi);
    service.set_packet_callback(&capture_csi_packet, &packets);
    service.set_channel_change_callback(&capture_channel_change, &channel_change);

    std::array<int8_t, HT20_CSI_LEN> csi{};
    wifi_csi_info_t info{};
    info.buf = csi.data();
    info.len = HT20_CSI_LEN;
    info.rx_ctrl.sig_mode = 1U;
    info.rx_ctrl.cwb = 0U;
    info.rx_ctrl.channel = 8U;
    info.rx_ctrl.timestamp = 100U;

    TEST_ASSERT_EQUAL(ESP_OK, service.enable());
    service.process_packet(&info);
    TEST_ASSERT_EQUAL(1U, packets.callback_count);

    info.rx_ctrl.channel = 10U;
    info.rx_ctrl.timestamp = 101U;
    service.process_packet(&info);
    info.rx_ctrl.timestamp = 102U;
    service.process_packet(&info);

    TEST_ASSERT_EQUAL(1U, packets.callback_count);
    TEST_ASSERT_EQUAL(0U, channel_change.callback_count);
    service.loop();
    TEST_ASSERT_EQUAL(1U, channel_change.callback_count);
    TEST_ASSERT_EQUAL(8U, channel_change.previous_channel);
    TEST_ASSERT_EQUAL(10U, channel_change.current_channel);

    TEST_ASSERT_EQUAL(ESP_OK, service.disable());
    TEST_ASSERT_EQUAL(ESP_OK, service.enable());
    info.rx_ctrl.channel = 11U;
    info.rx_ctrl.timestamp = 1U;
    service.process_packet(&info);
    service.loop();

    TEST_ASSERT_EQUAL(2U, packets.callback_count);
    TEST_ASSERT_EQUAL(1U, channel_change.callback_count);
}

void test_csi_format_classifier_rejects_ht40_before_normalization(void) {
    std::array<int8_t, HT20_CSI_LEN_DOUBLE> csi{};
    wifi_csi_info_t info{};
    info.buf = csi.data();
    info.len = HT20_CSI_LEN_DOUBLE;
    info.rx_ctrl.sig_mode = 1U;
    info.rx_ctrl.cwb = 1U;

    const CsiFormatAssessment assessment =
        assess_ht20_sensing_format(&info, CsiCaptureProfile::HT20);

    TEST_ASSERT_FALSE(assessment.is_sensing_accepted());
    TEST_ASSERT_TRUE(assessment.reason_code == CsiFormatReasonCode::UNSUPPORTED_WIDTH);
    TEST_ASSERT_TRUE(assessment.normalization_tag == NormalizedCSIPayloadTag::NONE);
}

void test_csi_capture_service_zero_fills_lltf_after_layout_detection(void) {
    CsiCaptureService service;
    CapturedCsiPacket captured;
    service.init();
    service.set_packet_callback(&capture_csi_packet, &captured);
    TEST_ASSERT_EQUAL(ESP_OK, service.enable(CsiCaptureProfile::LLTF20));

    std::array<int8_t, HT20_CSI_LEN> csi{};
    csi.fill(7);
    for (uint8_t bin : HT20_CLASSIC_ONLY_NULL_BINS) {
        const size_t byte_index = static_cast<size_t>(bin) * 2U;
        csi[byte_index] = 0;
        csi[byte_index + 1U] = 0;
    }
    wifi_csi_info_t info{};
    info.buf = csi.data();
    info.len = HT20_CSI_LEN;
    info.rx_ctrl.sig_mode = 0U;
    info.rx_ctrl.cwb = 0U;

    service.process_packet(&info);

    TEST_ASSERT_EQUAL(1U, captured.callback_count);
    TEST_ASSERT_TRUE(captured.rotated_to_centered);
    TEST_ASSERT_TRUE(captured.missing_lltf_bins_zero);
    TEST_ASSERT_EQUAL(7, captured.retained_value);
    TEST_ASSERT_EQUAL(7, csi[HT20_LLTF_MISSING_BINS[0] * 2U]);
    TEST_ASSERT_TRUE(service.last_assessment().format_id == CsiFormatId::HT20);
    TEST_ASSERT_EQUAL(ESP_OK, service.disable());
}

void test_csi_capture_service_tracks_format_drop_reasons(void) {
    CsiCaptureService service;
    CapturedCsiPacket captured;
    service.init();
    service.set_packet_callback(&capture_csi_packet, &captured);

    std::array<int8_t, HT20_CSI_LEN> csi{};
    wifi_csi_info_t info{};
    info.buf = csi.data();
    info.len = HT20_CSI_LEN;

    info.rx_ctrl.sig_mode = 3U;
    info.rx_ctrl.cwb = 0U;
    service.process_packet(&info);

    info.rx_ctrl.sig_mode = 1U;
    info.rx_ctrl.cwb = 1U;
    service.process_packet(&info);

    info.len = 64U;
    info.rx_ctrl.cwb = 0U;
    service.process_packet(&info);

    TEST_ASSERT_EQUAL(0U, captured.callback_count);
    TEST_ASSERT_EQUAL(1U, service.unsupported_phy_packets());
    TEST_ASSERT_EQUAL(1U, service.unsupported_width_packets());
    TEST_ASSERT_EQUAL(1U, service.unknown_layout_packets());
    TEST_ASSERT_EQUAL(3U, service.filtered_packets());
}

void test_runtime_config_utils_validate_and_name_values(void) {
    TEST_ASSERT_TRUE(validate_runtime_threshold(0.0f));
    TEST_ASSERT_TRUE(validate_runtime_threshold(1.0f));
    TEST_ASSERT_FALSE(validate_runtime_threshold(-0.1f));
    TEST_ASSERT_FALSE(validate_runtime_threshold(1.1f));
    TEST_ASSERT_EQUAL_STRING("ping", traffic_mode_name(RuntimeTrafficMode::PING));
    TEST_ASSERT_EQUAL_STRING("dns", traffic_mode_name(RuntimeTrafficMode::DNS));
    TEST_ASSERT_EQUAL_STRING("dns_tcp", traffic_mode_name(RuntimeTrafficMode::DNS_TCP));
    TEST_ASSERT_EQUAL_STRING("internal", csi_traffic_mode_name(CsiTrafficMode::INTERNAL));
    TEST_ASSERT_EQUAL_STRING("external", csi_traffic_mode_name(CsiTrafficMode::EXTERNAL));
    TEST_ASSERT_EQUAL_STRING("high_accuracy", detection_algorithm_name(DetectionAlgorithm::HIGH_ACCURACY));
    TEST_ASSERT_EQUAL_STRING("lightweight", detection_algorithm_name(DetectionAlgorithm::LIGHTWEIGHT));
    TEST_ASSERT_EQUAL_STRING("fixed", subcarrier_source_name(RuntimeSubcarrierSource::FIXED_DEFAULT));
    TEST_ASSERT_TRUE(parse_traffic_mode("ping") == RuntimeTrafficMode::PING);
    TEST_ASSERT_TRUE(parse_traffic_mode("dns") == RuntimeTrafficMode::DNS);
    TEST_ASSERT_TRUE(parse_traffic_mode("dns_tcp") == RuntimeTrafficMode::DNS_TCP);
    TEST_ASSERT_TRUE(parse_traffic_mode("unsupported") == RuntimeTrafficMode::PING);
    TEST_ASSERT_TRUE(parse_csi_traffic_mode("internal") == CsiTrafficMode::INTERNAL);
    TEST_ASSERT_TRUE(parse_csi_traffic_mode("external") == CsiTrafficMode::EXTERNAL);
    TEST_ASSERT_TRUE(parse_csi_traffic_mode("pacing") == CsiTrafficMode::INTERNAL);
    TEST_ASSERT_TRUE(parse_csi_traffic_mode("disabled") == CsiTrafficMode::INTERNAL);
    TEST_ASSERT_TRUE(parse_csi_traffic_mode("unsupported") == CsiTrafficMode::INTERNAL);
    TEST_ASSERT_TRUE(csi_traffic_mode_is_sensing_control(CsiTrafficMode::INTERNAL));
    TEST_ASSERT_TRUE(csi_traffic_mode_is_sensing_control(CsiTrafficMode::EXTERNAL));
    TEST_ASSERT_TRUE(csi_traffic_mode_is_sensing_control(CsiTrafficMode::EXTERNAL));
    TEST_ASSERT_TRUE(normalize_sensing_csi_traffic_mode(CsiTrafficMode::INTERNAL) == CsiTrafficMode::INTERNAL);
    TEST_ASSERT_TRUE(parse_detection_algorithm("high_accuracy") == DetectionAlgorithm::HIGH_ACCURACY);
    TEST_ASSERT_TRUE(parse_detection_algorithm("lightweight") == DetectionAlgorithm::LIGHTWEIGHT);
    TEST_ASSERT_EQUAL_STRING("2g", wifi_band_policy_name(WifiBandPolicy::BAND_2G));
    TEST_ASSERT_EQUAL_STRING("5g", wifi_band_policy_name(WifiBandPolicy::BAND_5G));
    TEST_ASSERT_EQUAL_STRING("auto", wifi_band_policy_name(WifiBandPolicy::AUTO));
    TEST_ASSERT_TRUE(parse_wifi_band_policy("2g") == WifiBandPolicy::BAND_2G);
    TEST_ASSERT_TRUE(parse_wifi_band_policy("5g") == WifiBandPolicy::BAND_5G);
    TEST_ASSERT_TRUE(parse_wifi_band_policy("auto") == WifiBandPolicy::AUTO);
    TEST_ASSERT_TRUE(parse_wifi_band_policy("unsupported") == WifiBandPolicy::BAND_2G);
}

void test_runtime_config_validator_covers_the_public_schema(void) {
    RuntimeConfig config;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::NONE);

    config.runtime_profile = static_cast<RuntimeProfile>(0x7f);
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::RUNTIME_PROFILE);
    config = RuntimeConfig{};
    config.wifi_band_policy = static_cast<WifiBandPolicy>(0x7f);
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::WIFI_BAND_POLICY);
    config = RuntimeConfig{};
    config.detection_algorithm = static_cast<DetectionAlgorithm>(0x7f);
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::DETECTION_ALGORITHM);
    config = RuntimeConfig{};
    config.segmentation_threshold = 2.0f;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::SEGMENTATION_THRESHOLD);
    config = RuntimeConfig{};
    config.segmentation_window_size_ms = 0U;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::SEGMENTATION_WINDOW_SIZE_MS);
    config = RuntimeConfig{};
    config.csi_target_pps = 0U;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::CSI_TARGET_PPS);
    config = RuntimeConfig{};
    config.traffic_generator_mode = static_cast<RuntimeTrafficMode>(0x7f);
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::TRAFFIC_GENERATOR_MODE);
    config = RuntimeConfig{};
    config.csi_traffic_mode = static_cast<CsiTrafficMode>(0x7f);
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::CSI_TRAFFIC_MODE);
    config = RuntimeConfig{};
    config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
    config.csi_traffic_udp_port = 0U;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::CSI_TRAFFIC_UDP_PORT);
    config = RuntimeConfig{};
    config.csi_traffic_mode = CsiTrafficMode::EXTERNAL;
    config.csi_traffic_multicast_group = "192.168.1.2";
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::CSI_TRAFFIC_MULTICAST_GROUP);
    config = RuntimeConfig{};
    config.evaluation_interval_ms = 0U;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::EVALUATION_INTERVAL_MS);
    config = RuntimeConfig{};
    config.motion_on_hits = 0U;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::MOTION_HITS);
    config = RuntimeConfig{};
    config.lowpass_enabled = true;
    config.lowpass_cutoff = 1.0f;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::LOWPASS_CUTOFF);
    config = RuntimeConfig{};
    config.hampel_enabled = true;
    config.hampel_window = 2U;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::HAMPEL_WINDOW);
    config = RuntimeConfig{};
    config.hampel_threshold = 0.0f;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::HAMPEL_THRESHOLD);
    config = RuntimeConfig{};
    config.lowpass_enabled = false;
    config.lowpass_cutoff = 1.0f;
    config.hampel_enabled = false;
    config.hampel_window = 0U;
    config.hampel_threshold = 0.0f;
    TEST_ASSERT_TRUE(validate_runtime_config(config) == RuntimeConfigError::NONE);
    TEST_ASSERT_EQUAL_STRING("invalid Hampel window",
                             runtime_config_error_message(RuntimeConfigError::HAMPEL_WINDOW));
}

void test_runtime_diagnostics_emit_expected_key_value_pairs(void) {
    RuntimeConfig config;
    RuntimeSnapshot snapshot;
    config.lowpass_enabled = true;
    snapshot.threshold = 2.5f;
    snapshot.detector_name = "lightweight";
    snapshot.startup_threshold = 0.125f;

    std::vector<std::string> lines;
    visit_runtime_diagnostics(config, snapshot, [&lines](const char *key, const char *value) {
        lines.emplace_back(std::string(key) + "=" + value);
    });

    TEST_ASSERT_TRUE(!lines.empty());
    TEST_ASSERT_TRUE(std::find(lines.begin(), lines.end(), "threshold=2.500000") != lines.end());
    TEST_ASSERT_TRUE(std::find(lines.begin(), lines.end(), "detector=lightweight") != lines.end());
    TEST_ASSERT_TRUE(std::find(lines.begin(), lines.end(), "lowpass=on") != lines.end());
    TEST_ASSERT_TRUE(std::none_of(lines.begin(), lines.end(), [](const std::string &line) {
        return line.rfind("subcarriers=", 0U) == 0U || line.rfind("startup_threshold=", 0U) == 0U;
    }));
}

void test_runtime_diagnostics_sampler_derives_five_second_rates(void) {
    RuntimeDiagnosticsSnapshot baseline;
    baseline.traffic_packets_total = 100U;
    baseline.csi_callbacks_total = 100U;
    baseline.csi_accepted_total = 90U;
    baseline.csi_admitted_total = 80U;
    baseline.csi_filtered_total = 10U;

    RuntimeDiagnosticsSampler sampler;
    sampler.reset(baseline, 1000U);

    RuntimeDiagnosticsSnapshot current = baseline;
    current.traffic_packets_total = 600U;
    current.csi_callbacks_total = 580U;
    current.csi_accepted_total = 540U;
    current.csi_admitted_total = 505U;
    current.csi_filtered_total = 40U;
    current.csi_missing_slots_total = 25U;
    current.csi_excess_total = 15U;
    current.csi_stale_total = 5U;
    current.csi_out_of_order_total = 10U;
    current.csi_occupancy_slots = 82U;
    current.csi_window_slots = 100U;
    current.wifi_channel = 10U;
    current.wifi_rssi_dbm = -55;

    const RuntimeDiagnosticsSample sample = sampler.sample(current, 6000U);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, sample.traffic_tx_pps);
    TEST_ASSERT_EQUAL_FLOAT(96.0f, sample.csi_callback_pps);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, sample.csi_accepted_pps);
    TEST_ASSERT_EQUAL_FLOAT(85.0f, sample.csi_admitted_pps);
    TEST_ASSERT_EQUAL_FLOAT(6.0f, sample.csi_filtered_pps);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, sample.csi_missing_slots_pps);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, sample.csi_excess_pps);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, sample.csi_stale_pps);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, sample.csi_out_of_order_pps);
    TEST_ASSERT_EQUAL_FLOAT(0.82f, sample.csi_occupancy_ratio);
    TEST_ASSERT_EQUAL_UINT8(10U, sample.wifi_channel);
    TEST_ASSERT_EQUAL_INT8(-55, sample.wifi_rssi_dbm);
}

void test_runtime_performance_diagnostics_publish_complete_windows(void) {
    esp_timer_mock::reset(1, 0);
    RuntimePerformanceDiagnostics diagnostics;
    diagnostics.reset();
    diagnostics.update_if_due();
    diagnostics.record_loop_duration(100U);
    diagnostics.record_loop_duration(300U);
    diagnostics.record_detection_timing(900U, 3U, 200U, 400U);

    esp_timer_mock::advance(10000000);
    diagnostics.update_if_due();
    const RuntimePerformanceDiagnosticsSnapshot snapshot = diagnostics.snapshot();

    TEST_ASSERT_TRUE(snapshot.window_ready);
    TEST_ASSERT_EQUAL(10000000U, snapshot.window_duration_us);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.004f, snapshot.runtime_load_percent);
    TEST_ASSERT_EQUAL(2U, snapshot.loop_samples);
    TEST_ASSERT_EQUAL(200U, snapshot.loop_average_us);
    TEST_ASSERT_EQUAL(300U, snapshot.loop_maximum_us);
    TEST_ASSERT_EQUAL(3U, snapshot.detection_samples);
    TEST_ASSERT_EQUAL(900U, snapshot.detection_sum_us);
    TEST_ASSERT_EQUAL(300U, snapshot.detection_average_us);
    TEST_ASSERT_EQUAL(200U, snapshot.detection_minimum_us);
    TEST_ASSERT_EQUAL(400U, snapshot.detection_maximum_us);
}

void test_runtime_performance_diagnostics_json_marks_unready_and_unsupported_values(void) {
    RuntimeDiagnosticsSnapshot diagnostics;
    diagnostics.free_memory_bytes = 4096U;
    diagnostics.minimum_free_memory_bytes = 2048U;
    diagnostics.largest_free_memory_block_bytes = 1024U;
    diagnostics.cpu_frequency_mhz = 160U;

    std::string json{"{\"existing\":1"};
    append_runtime_performance_diagnostics_json(&json, diagnostics);
    json += "}";

    TEST_ASSERT_TRUE(json.find("\"free_memory_kb\":4") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"minimum_free_memory_kb\":2") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"largest_free_memory_kb\":1") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"cpu_frequency_mhz\":160") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"performance_window_ready\":false") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"runtime_load_percent\":null") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"detection_timing_supported\":false") != std::string::npos);
    TEST_ASSERT_TRUE(json.find("\"detection_samples\":null") != std::string::npos);
}

void test_mqtt_payload_assembler_accepts_complete_and_fragmented_payloads(void) {
    MqttPayloadAssembler assembler;

    TEST_ASSERT_TRUE(assembler.append("ping", 4, 4, 0) == MqttPayloadAssembler::Result::COMPLETE);
    TEST_ASSERT_TRUE(assembler.payload() == "ping");
    assembler.reset();

    TEST_ASSERT_TRUE(assembler.append("calib", 5, 9, 0) == MqttPayloadAssembler::Result::INCOMPLETE);
    TEST_ASSERT_TRUE(assembler.append("rate", 4, 9, 5) == MqttPayloadAssembler::Result::COMPLETE);
    TEST_ASSERT_TRUE(assembler.payload() == "calibrate");
}

void test_mqtt_payload_assembler_rejects_invalid_fragments(void) {
    MqttPayloadAssembler assembler;

    TEST_ASSERT_TRUE(assembler.append("abc", 3, 6, 0) == MqttPayloadAssembler::Result::INCOMPLETE);
    TEST_ASSERT_TRUE(assembler.append("def", 3, 6, 2) == MqttPayloadAssembler::Result::INVALID);
    TEST_ASSERT_TRUE(assembler.payload().empty());

    std::string oversized(MqttPayloadAssembler::MAX_PAYLOAD_SIZE + 1U, 'x');
    TEST_ASSERT_TRUE(assembler.append(oversized.data(), oversized.size(), oversized.size(), 0) ==
                     MqttPayloadAssembler::Result::INVALID);
    TEST_ASSERT_TRUE(assembler.payload().empty());
}

void test_sta_socket_binding_rejects_missing_or_invalid_interface(void) {
    esp_netif_mock_reset();
    g_esp_netif_mock.handle_available = false;
    TEST_ASSERT_FALSE(bind_socket_to_sta_interface(-1, "test", "udp"));

    esp_netif_mock_reset();
    g_esp_netif_mock.impl_index = 0;
    TEST_ASSERT_FALSE(bind_socket_to_sta_interface(-1, "test", "udp"));
}

void test_sta_socket_binding_uses_resolved_interface(void) {
    esp_netif_mock_reset();
    unsigned interface_index = if_nametoindex("lo");
    if (interface_index == 0U) {
        interface_index = if_nametoindex("lo0");
    }
    TEST_ASSERT_TRUE(interface_index > 0U);
    g_esp_netif_mock.impl_index = static_cast<int>(interface_index);
    const int sock = socket(AF_INET, SOCK_DGRAM, 0);
    TEST_ASSERT_TRUE(sock >= 0);
    const bool bound = bind_socket_to_sta_interface(sock, "test", "udp");
#if defined(__linux__)
    TEST_ASSERT_TRUE(bound);
#else
    (void)bound;
#endif
    close(sock);
}

int process(void) {
    UNITY_BEGIN();
    RUN_TEST(test_wifi_csi_real_forwards_calls_to_mocked_esp_wifi);
    RUN_TEST(test_lltf_preference_and_vht_capability_resolve_capture_profile);
    RUN_TEST(test_lltf20_profile_inherits_supported_ht20_payload_layouts);
    RUN_TEST(test_compact_lltf_preserves_tones_and_bypasses_latched_classic_order);
    RUN_TEST(test_lltf20_legacy_frames_reject_unrelated_payload_lengths);
    RUN_TEST(test_csi_capture_service_normalizes_ht_layouts_under_lltf20);
    RUN_TEST(test_csi_capture_service_filters_duplicate_and_stale_timestamps);
    RUN_TEST(test_csi_capture_service_defers_channel_change_and_resets_session_baseline);
    RUN_TEST(test_csi_format_classifier_rejects_ht40_before_normalization);
    RUN_TEST(test_csi_capture_service_zero_fills_lltf_after_layout_detection);
    RUN_TEST(test_csi_capture_service_tracks_format_drop_reasons);
    RUN_TEST(test_runtime_config_utils_validate_and_name_values);
    RUN_TEST(test_runtime_config_validator_covers_the_public_schema);
    RUN_TEST(test_runtime_diagnostics_emit_expected_key_value_pairs);
    RUN_TEST(test_runtime_diagnostics_sampler_derives_five_second_rates);
    RUN_TEST(test_runtime_performance_diagnostics_publish_complete_windows);
    RUN_TEST(test_runtime_performance_diagnostics_json_marks_unready_and_unsupported_values);
    RUN_TEST(test_mqtt_payload_assembler_accepts_complete_and_fragmented_payloads);
    RUN_TEST(test_mqtt_payload_assembler_rejects_invalid_fragments);
    RUN_TEST(test_sta_socket_binding_rejects_missing_or_invalid_interface);
    RUN_TEST(test_sta_socket_binding_uses_resolved_interface);
    return UNITY_END();
}

#if defined(ESP_PLATFORM)
extern "C" void app_main(void) { process(); }
#else
int main(int argc, char **argv) { return process(); }
#endif
