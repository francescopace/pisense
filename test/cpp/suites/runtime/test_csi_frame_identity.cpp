/*
 * ESPectre - CSI Frame Identity Unit Tests
 *
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

#include "csi_frame_identity.h"
#include "lwip/inet.h"

using namespace espectre;

namespace {

constexpr uint8_t kLocalMac[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
constexpr uint8_t kOtherMac[6] = {0x66, 0x55, 0x44, 0x33, 0x22, 0x11};
constexpr uint8_t kMulticastMac[6] = {0x01, 0x00, 0x5E, 0x7F, 0x00, 0x01};
constexpr uint32_t kGateway = 0xC0A80101U;
constexpr uint32_t kLocal = 0xC0A80111U;
constexpr uint32_t kOther = 0xC0A80112U;
constexpr uint32_t kMulticast = 0xEFFF0001U;

void write_be16(uint8_t *target, uint16_t value) {
  target[0] = static_cast<uint8_t>(value >> 8U);
  target[1] = static_cast<uint8_t>(value);
}

void write_be32(uint8_t *target, uint32_t value) {
  target[0] = static_cast<uint8_t>(value >> 24U);
  target[1] = static_cast<uint8_t>(value >> 16U);
  target[2] = static_cast<uint8_t>(value >> 8U);
  target[3] = static_cast<uint8_t>(value);
}

std::vector<uint8_t> ipv4_frame(uint8_t protocol,
                                uint32_t source,
                                uint32_t destination,
                                const std::vector<uint8_t> &transport) {
  std::vector<uint8_t> frame(8U + 20U + transport.size(), 0U);
  const uint8_t snap[8] = {0xAAU, 0xAAU, 0x03U, 0U, 0U, 0U, 0x08U, 0U};
  std::memcpy(frame.data(), snap, sizeof(snap));
  uint8_t *ip = frame.data() + 8U;
  ip[0] = 0x45U;
  write_be16(ip + 2U, static_cast<uint16_t>(20U + transport.size()));
  ip[8] = 64U;
  ip[9] = protocol;
  write_be32(ip + 12U, source);
  write_be32(ip + 16U, destination);
  std::memcpy(ip + 20U, transport.data(), transport.size());
  return frame;
}

std::vector<uint8_t> canonical_marker() {
  return std::vector<uint8_t>(RUNTIME_CSI_TRAFFIC_MARKER_BYTES,
                              RUNTIME_CSI_TRAFFIC_MARKER_BYTES +
                                  RUNTIME_CSI_TRAFFIC_MARKER_LENGTH);
}

std::vector<uint8_t> udp_frame(uint32_t destination,
                               uint16_t port,
                               std::vector<uint8_t> marker = canonical_marker()) {
  std::vector<uint8_t> udp(8U + marker.size(), 0U);
  write_be16(udp.data(), 40000U);
  write_be16(udp.data() + 2U, port);
  write_be16(udp.data() + 4U, static_cast<uint16_t>(udp.size()));
  std::copy(marker.begin(), marker.end(), udp.begin() + 8U);
  return ipv4_frame(17U, kOther, destination, udp);
}

std::vector<uint8_t> ping_reply(uint32_t source, uint16_t identifier) {
  std::vector<uint8_t> icmp(8U, 0U);
  write_be16(icmp.data() + 4U, identifier);
  return ipv4_frame(1U, source, kLocal, icmp);
}

std::vector<uint8_t> ping_request(uint32_t source,
                                  uint32_t destination = kLocal,
                                  uint8_t code = 0U) {
  std::vector<uint8_t> icmp(8U, 0U);
  icmp[0] = 8U;
  icmp[1] = code;
  write_be16(icmp.data() + 4U, 0x4321U);
  return ipv4_frame(1U, source, destination, icmp);
}

std::vector<uint8_t> dns_tcp_reply(bool with_payload = true) {
  std::vector<uint8_t> tcp(with_payload ? 34U : 20U, 0U);
  write_be16(tcp.data(), 53U);
  write_be16(tcp.data() + 2U, 40000U);
  tcp[12] = 0x50U;
  tcp[13] = 0x10U;
  if (with_payload) {
    tcp[20] = 0U;
    tcp[21] = 12U;
    tcp[24] = 0x81U;
    tcp[25] = 0x80U;
  }
  return ipv4_frame(6U, kGateway, kLocal, tcp);
}

std::vector<uint8_t> dns_udp_reply() {
  std::vector<uint8_t> udp(20U, 0U);
  write_be16(udp.data(), 53U);
  write_be16(udp.data() + 2U, 40000U);
  write_be16(udp.data() + 4U, static_cast<uint16_t>(udp.size()));
  udp[10U] = 0x81U;
  udp[11U] = 0x80U;
  return ipv4_frame(17U, kGateway, kLocal, udp);
}

wifi_csi_info_t csi_info(const std::vector<uint8_t> &payload, const uint8_t *destination_mac = kLocalMac) {
  wifi_csi_info_t info{};
  info.payload = const_cast<uint8_t *>(payload.data());
  info.payload_len = static_cast<uint16_t>(payload.size());
  std::memcpy(info.dmac, destination_mac, 6U);
  return info;
}

bool matches(const std::vector<uint8_t> &payload,
             const CsiFrameFilterConfig &config,
             const uint8_t *destination_mac = kLocalMac) {
  const wifi_csi_info_t info = csi_info(payload, destination_mac);
  return csi_frame_matches_traffic(&info, config, CsiCaptureProfile::HT20);
}

CsiFrameFilterConfig filter(CsiTrafficMode mode, RuntimeTrafficMode internal = RuntimeTrafficMode::PING) {
  CsiFrameFilterConfig config;
  config.traffic_mode = mode;
  config.internal_mode = internal;
  config.local_ip_addr = inet_addr("192.168.1.17");
  config.gateway_ip_addr = inet_addr("192.168.1.1");
  config.multicast_ip_addr = inet_addr("239.255.0.1");
  config.external_udp_port = 5555U;
  config.internal_icmp_identifier = 0x1234U;
  std::memcpy(config.local_mac_addr, kLocalMac, 6U);
  return config;
}

}  // namespace

void setUp(void) {}
void tearDown(void) {}

void test_local_ack_requires_lltf20_for_every_traffic_mode(void) {
  uint8_t header[10] = {0xD4U, 0U, 0U, 0U};
  std::memcpy(header + 4U, kLocalMac, 6U);
  wifi_csi_info_t info{};
  info.hdr = header;
  info.rx_ctrl.sig_len = sizeof(header) + 4U;

  for (const auto mode : {CsiTrafficMode::INTERNAL, CsiTrafficMode::EXTERNAL}) {
    for (const auto internal : {RuntimeTrafficMode::PING, RuntimeTrafficMode::DNS,
                                RuntimeTrafficMode::DNS_TCP}) {
      const auto config = filter(mode, internal);
      TEST_ASSERT_TRUE(csi_frame_matches_traffic(&info, config, CsiCaptureProfile::LLTF20));
      TEST_ASSERT_FALSE(csi_frame_matches_traffic(&info, config, CsiCaptureProfile::HT20));
      TEST_ASSERT_FALSE(csi_frame_matches_traffic(&info, config, CsiCaptureProfile::VHT20));
    }
  }
}

void test_lltf20_rejects_ack_without_valid_local_receiver_or_header(void) {
  auto config = filter(CsiTrafficMode::EXTERNAL);
  uint8_t header[10] = {0xD4U, 0U, 0U, 0U};
  std::memcpy(header + 4U, kLocalMac, 6U);
  wifi_csi_info_t info{};
  info.hdr = header;
  info.rx_ctrl.sig_len = sizeof(header) + 4U;
  const auto accepted = [&]() {
    return csi_frame_matches_traffic(&info, config, CsiCaptureProfile::LLTF20);
  };
  TEST_ASSERT_TRUE(accepted());
  std::memcpy(header + 4U, kOtherMac, 6U);
  std::memcpy(info.dmac, kLocalMac, 6U);
  TEST_ASSERT_FALSE(accepted());
  std::memcpy(header + 4U, kLocalMac, 6U);
  std::memset(config.local_mac_addr, 0, 6U);
  TEST_ASSERT_FALSE(accepted());
  std::memcpy(config.local_mac_addr, kMulticastMac, 6U);
  std::memcpy(header + 4U, kMulticastMac, 6U);
  TEST_ASSERT_FALSE(accepted());
  std::memcpy(config.local_mac_addr, kLocalMac, 6U);
  std::memcpy(header + 4U, kLocalMac, 6U);
  for (const uint8_t control : {0xC4U, 0x94U, 0x80U, 0x08U, 0xD5U}) {
    header[0] = control;  // CTS, Block ACK, beacon, data, and invalid version.
    TEST_ASSERT_FALSE(accepted());
  }
  header[0] = 0xD4U;
  header[1] = 0x01U;
  TEST_ASSERT_FALSE(accepted());
  header[1] = 0U;
  for (const uint16_t length : {0U, 9U, 13U, 15U}) {
    info.rx_ctrl.sig_len = length;
    TEST_ASSERT_FALSE(accepted());
  }
  info.rx_ctrl.sig_len = sizeof(header) + 4U;
  info.rx_ctrl.rx_state = 1U;
  TEST_ASSERT_FALSE(accepted());
  info.rx_ctrl.rx_state = 0U;
  info.hdr = nullptr;
  TEST_ASSERT_FALSE(accepted());
  TEST_ASSERT_FALSE(csi_frame_matches_traffic(nullptr, config, CsiCaptureProfile::LLTF20));
}

void test_lltf20_preserves_ip_traffic_provenance_filter(void) {
  const auto config = filter(CsiTrafficMode::INTERNAL, RuntimeTrafficMode::DNS_TCP);
  const auto reply = dns_tcp_reply(true);
  const auto tcp_ack = dns_tcp_reply(false);
  const auto unrelated = ping_reply(kGateway, 0x1234U);
  for (const auto *payload : {&reply, &tcp_ack, &unrelated}) {
    const auto info = csi_info(*payload);
    TEST_ASSERT_EQUAL(payload == &reply,
                      csi_frame_matches_traffic(&info, config, CsiCaptureProfile::LLTF20));
  }
}

void test_external_accepts_only_canonical_unicast_and_multicast_marker(void) {
  const CsiFrameFilterConfig config = filter(CsiTrafficMode::EXTERNAL);
  const auto unicast = udp_frame(kLocal, 5555U);
  const auto multicast = udp_frame(kMulticast, 5555U);
  TEST_ASSERT_TRUE(matches(unicast, config));
  TEST_ASSERT_TRUE(matches(multicast, config, kMulticastMac));

  const auto wrong_marker = udp_frame(kLocal, 5555U, {'.'});
  const auto truncated_marker = udp_frame(kLocal, 5555U, {0xF0U, 0x9FU, 0x91U});
  const auto malformed_marker = udp_frame(kLocal, 5555U, {0xF0U, 0x28U, 0x8CU, 0xBCU});
  auto marker_with_trailing_byte = canonical_marker();
  marker_with_trailing_byte.push_back('x');
  const auto oversized_marker = udp_frame(kLocal, 5555U, marker_with_trailing_byte);
  const auto wrong_port = udp_frame(kLocal, 5556U);
  const auto wrong_device = udp_frame(kOther, 5555U);
  TEST_ASSERT_FALSE(matches(wrong_marker, config));
  TEST_ASSERT_FALSE(matches(truncated_marker, config));
  TEST_ASSERT_FALSE(matches(malformed_marker, config));
  TEST_ASSERT_FALSE(matches(oversized_marker, config));
  TEST_ASSERT_FALSE(matches(wrong_port, config));
  TEST_ASSERT_FALSE(matches(wrong_device, config));
}

void test_external_rejects_other_mac_fragments_and_truncation(void) {
  const CsiFrameFilterConfig config = filter(CsiTrafficMode::EXTERNAL);
  const auto valid = udp_frame(kLocal, 5555U);
  auto fragmented = valid;
  fragmented[8U + 6U] = 0x20U;
  auto truncated = valid;
  truncated.resize(20U);
  auto invalid_header = valid;
  invalid_header[8U] = 0x44U;
  auto incomplete_ip = valid;
  write_be16(incomplete_ip.data() + 8U + 2U,
             static_cast<uint16_t>(incomplete_ip.size()));
  TEST_ASSERT_FALSE(matches(valid, config, kOtherMac));
  TEST_ASSERT_FALSE(matches(fragmented, config));
  TEST_ASSERT_FALSE(matches(truncated, config));
  TEST_ASSERT_FALSE(matches(invalid_header, config));
  TEST_ASSERT_FALSE(matches(incomplete_ip, config));
  TEST_ASSERT_FALSE(matches(valid, config, kMulticastMac));
  TEST_ASSERT_FALSE(matches(udp_frame(kMulticast, 5555U), config, kOtherMac));
}

void test_external_accepts_multicast_to_local_mac_with_canonical_identity(void) {
  CsiFrameFilterConfig config = filter(CsiTrafficMode::EXTERNAL);
  const auto multicast = udp_frame(kMulticast, config.external_udp_port);
  TEST_ASSERT_TRUE(matches(multicast, config));
  TEST_ASSERT_FALSE(matches(multicast, config, kOtherMac));
  TEST_ASSERT_FALSE(matches(udp_frame(kMulticast, config.external_udp_port, {'.'}), config));
  TEST_ASSERT_FALSE(matches(udp_frame(kMulticast, config.external_udp_port + 1U), config));

  // Different multicast groups can map to the same Ethernet multicast MAC.
  const auto other_group = udp_frame(kMulticast ^ 0x01000000U, config.external_udp_port);
  TEST_ASSERT_FALSE(matches(other_group, config));
  TEST_ASSERT_FALSE(matches(other_group, config, kMulticastMac));

  config.multicast_ip_addr = 0U;
  TEST_ASSERT_FALSE(matches(multicast, config));
  TEST_ASSERT_FALSE(matches(multicast, config, kMulticastMac));
}

void test_external_rejects_udp_length_mismatch_and_data_after_marker(void) {
  const CsiFrameFilterConfig config = filter(CsiTrafficMode::EXTERNAL);
  auto short_udp = udp_frame(kLocal, 5555U);
  write_be16(short_udp.data() + 8U + 20U + 4U, 8U);
  auto trailing = udp_frame(kLocal, 5555U);
  trailing.push_back('x');
  write_be16(trailing.data() + 8U + 2U,
             static_cast<uint16_t>(trailing.size() - 8U));
  TEST_ASSERT_FALSE(matches(short_udp, config));
  TEST_ASSERT_FALSE(matches(trailing, config));
}

void test_external_accepts_bounded_shifted_llc_frame(void) {
  const CsiFrameFilterConfig config = filter(CsiTrafficMode::EXTERNAL);
  std::vector<uint8_t> shifted(31U, 0x48U);
  const auto valid = udp_frame(kLocal, 5555U);
  shifted.insert(shifted.end(), valid.begin(), valid.end());
  TEST_ASSERT_TRUE(matches(shifted, config));

  std::vector<uint8_t> beyond_scan(64U, 0x48U);
  beyond_scan.insert(beyond_scan.end(), valid.begin(), valid.end());
  TEST_ASSERT_FALSE(matches(beyond_scan, config));
}

void test_external_accepts_unicast_ping_requests(void) {
  const CsiFrameFilterConfig config = filter(CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_TRUE(matches(ping_request(kGateway), config));
  TEST_ASSERT_TRUE(matches(ping_request(kOther), config));
}

void test_external_rejects_other_icmp_traffic(void) {
  const CsiFrameFilterConfig config = filter(CsiTrafficMode::EXTERNAL);
  TEST_ASSERT_FALSE(matches(ping_reply(kOther, 0x4321U), config));
  TEST_ASSERT_FALSE(matches(ping_request(kOther, kLocal, 1U), config));
  TEST_ASSERT_FALSE(matches(ping_request(kOther, kOther), config));
  TEST_ASSERT_FALSE(matches(ping_request(kOther), config, kOtherMac));
  TEST_ASSERT_FALSE(matches(ping_request(kOther, kMulticast), config, kMulticastMac));
}

void test_internal_ping_requires_gateway_echo_reply_and_active_identifier(void) {
  const CsiFrameFilterConfig config = filter(CsiTrafficMode::INTERNAL, RuntimeTrafficMode::PING);
  const auto valid = ping_reply(kGateway, 0x1234U);
  const auto wrong_gateway = ping_reply(kOther, 0x1234U);
  const auto wrong_identifier = ping_reply(kGateway, 0x1235U);
  auto echo_request = valid;
  echo_request[8U + 20U] = 8U;
  TEST_ASSERT_TRUE(matches(valid, config));
  TEST_ASSERT_FALSE(matches(wrong_gateway, config));
  TEST_ASSERT_FALSE(matches(wrong_identifier, config));
  TEST_ASSERT_FALSE(matches(echo_request, config));
}

void test_internal_dns_requires_gateway_tcp_53_payload_and_rejects_ack_only(void) {
  const CsiFrameFilterConfig config = filter(CsiTrafficMode::INTERNAL, RuntimeTrafficMode::DNS_TCP);
  const auto valid = dns_tcp_reply(true);
  const auto ack_only = dns_tcp_reply(false);
  auto length_mismatch = dns_tcp_reply(true);
  length_mismatch[8U + 20U + 21U] = 11U;
  auto query = dns_tcp_reply(true);
  query[8U + 20U + 24U] = 0x01U;
  auto http = dns_tcp_reply(true);
  write_be16(http.data() + 8U + 20U, 80U);
  TEST_ASSERT_TRUE(matches(valid, config));
  TEST_ASSERT_FALSE(matches(ack_only, config));
  TEST_ASSERT_FALSE(matches(length_mismatch, config));
  TEST_ASSERT_FALSE(matches(query, config));
  TEST_ASSERT_FALSE(matches(http, config));
}

void test_internal_dns_udp_requires_gateway_udp_53_response(void) {
  const CsiFrameFilterConfig config = filter(CsiTrafficMode::INTERNAL, RuntimeTrafficMode::DNS);
  const auto valid = dns_udp_reply();
  auto wrong_length = valid;
  write_be16(wrong_length.data() + 8U + 20U + 4U, 19U);
  auto query = valid;
  query[8U + 20U + 10U] = 0x01U;
  auto wrong_port = valid;
  write_be16(wrong_port.data() + 8U + 20U, 54U);
  auto wrong_gateway = valid;
  write_be32(wrong_gateway.data() + 8U + 12U, kOther);
  TEST_ASSERT_TRUE(matches(valid, config));
  TEST_ASSERT_FALSE(matches(wrong_length, config));
  TEST_ASSERT_FALSE(matches(query, config));
  TEST_ASSERT_FALSE(matches(wrong_port, config));
  TEST_ASSERT_FALSE(matches(wrong_gateway, config));
  TEST_ASSERT_FALSE(matches(dns_tcp_reply(true), config));
}

int main() {
  using namespace espectre::test;
  begin_suite();
  RUN_TEST(test_local_ack_requires_lltf20_for_every_traffic_mode);
  RUN_TEST(test_lltf20_rejects_ack_without_valid_local_receiver_or_header);
  RUN_TEST(test_lltf20_preserves_ip_traffic_provenance_filter);
  RUN_TEST(test_external_accepts_only_canonical_unicast_and_multicast_marker);
  RUN_TEST(test_external_rejects_other_mac_fragments_and_truncation);
  RUN_TEST(test_external_accepts_multicast_to_local_mac_with_canonical_identity);
  RUN_TEST(test_external_rejects_udp_length_mismatch_and_data_after_marker);
  RUN_TEST(test_external_accepts_bounded_shifted_llc_frame);
  RUN_TEST(test_external_accepts_unicast_ping_requests);
  RUN_TEST(test_external_rejects_other_icmp_traffic);
  RUN_TEST(test_internal_ping_requires_gateway_echo_reply_and_active_identifier);
  RUN_TEST(test_internal_dns_requires_gateway_tcp_53_payload_and_rejects_ack_only);
  RUN_TEST(test_internal_dns_udp_requires_gateway_udp_53_response);
  return end_suite();
}
