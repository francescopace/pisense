/*
 * ESPectre - CSI Frame Identity
 *
 * Bounded classification of CSI callbacks against the configured ESPectre
 * traffic source.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "csi_frame_identity.h"

#include <algorithm>
#include <cstring>

#include "lwip/inet.h"
#include "mac_address_helpers.h"

namespace espectre {
namespace {

constexpr size_t kLlcSnapHeaderBytes = 8U;
constexpr size_t kIpv4MinimumHeaderBytes = 20U;
constexpr size_t kTransportMinimumHeaderBytes = 8U;
constexpr size_t kLlcScanBytes = 64U;
constexpr uint16_t kEtherTypeIpv4 = 0x0800U;
constexpr uint8_t kIpProtoIcmp = 1U;
constexpr uint8_t kIpProtoTcp = 6U;
constexpr uint8_t kIpProtoUdp = 17U;
constexpr uint8_t kLlcSnapPrefix[6] = {0xAAU, 0xAAU, 0x03U, 0x00U, 0x00U, 0x00U};

bool matches_local_ack(const wifi_csi_info_t *info, const CsiFrameFilterConfig &config) {
  // An ACK has a 10-byte MAC header and a 4-byte FCS, but no transmitter
  // address or IP payload. Read its receiver address directly from the header.
  constexpr size_t kAckFrameBytes = 14U;
  constexpr uint8_t kAckFrameControl = 0xD4U;
  if (info->hdr == nullptr || info->rx_ctrl.sig_len != kAckFrameBytes ||
      info->rx_ctrl.rx_state != 0U || is_zero_mac_address(config.local_mac_addr) ||
      (config.local_mac_addr[0] & 0x01U) != 0U) return false;
  return info->hdr[0] == kAckFrameControl && (info->hdr[1] & 0x03U) == 0U &&
         std::memcmp(info->hdr + 4U, config.local_mac_addr, 6U) == 0;
}

uint16_t read_be16(const uint8_t *data) {
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8U) |
                               static_cast<uint16_t>(data[1]));
}

uint32_t read_be32(const uint8_t *data) {
  return (static_cast<uint32_t>(data[0]) << 24U) |
         (static_cast<uint32_t>(data[1]) << 16U) |
         (static_cast<uint32_t>(data[2]) << 8U) |
         static_cast<uint32_t>(data[3]);
}

struct ParsedIpv4 {
  const uint8_t *transport{nullptr};
  size_t transport_len{0U};
  uint32_t source{0U};
  uint32_t destination{0U};
  uint8_t protocol{0U};
};

bool parse_ipv4(const uint8_t *payload, size_t payload_len, ParsedIpv4 *out) {
  if (payload == nullptr || out == nullptr ||
      payload_len < kLlcSnapHeaderBytes + kIpv4MinimumHeaderBytes) return false;
  if (std::memcmp(payload, kLlcSnapPrefix, sizeof(kLlcSnapPrefix)) != 0 ||
      read_be16(payload + 6U) != kEtherTypeIpv4) return false;
  const uint8_t *ip = payload + kLlcSnapHeaderBytes;
  const size_t available = payload_len - kLlcSnapHeaderBytes;
  if ((ip[0] >> 4U) != 4U) return false;
  const size_t header_len = static_cast<size_t>(ip[0] & 0x0FU) * 4U;
  const size_t total_len = read_be16(ip + 2U);
  if (header_len < kIpv4MinimumHeaderBytes || total_len < header_len || total_len > available) {
    return false;
  }
  if ((read_be16(ip + 6U) & 0x3FFFU) != 0U) return false;
  out->source = read_be32(ip + 12U);
  out->destination = read_be32(ip + 16U);
  out->protocol = ip[9U];
  out->transport = ip + header_len;
  out->transport_len = total_len - header_len;
  return true;
}

bool parse_bounded_payload(const wifi_csi_info_t *info, ParsedIpv4 *out) {
  if (info == nullptr || info->payload == nullptr || info->payload_len == 0U) return false;
  const auto *payload = reinterpret_cast<const uint8_t *>(info->payload);
  if (parse_ipv4(payload, info->payload_len, out)) return true;
  const size_t limit = std::min<size_t>(info->payload_len, kLlcScanBytes);
  for (size_t offset = 1U; offset + kLlcSnapHeaderBytes <= limit; ++offset) {
    if (std::memcmp(payload + offset, kLlcSnapPrefix, sizeof(kLlcSnapPrefix)) == 0 &&
        parse_ipv4(payload + offset, info->payload_len - offset, out)) return true;
  }
  return false;
}

uint32_t host_ip(uint32_t network_order) { return ntohl(network_order); }

bool destination_mac_matches(const ParsedIpv4 &packet,
                             const CsiFrameFilterConfig &config,
                             const uint8_t *frame_mac) {
  if (frame_mac == nullptr || is_zero_mac_address(config.local_mac_addr)) return false;
  if (packet.destination == host_ip(config.local_ip_addr)) {
    return std::memcmp(config.local_mac_addr, frame_mac, 6U) == 0;
  }
  if (config.multicast_ip_addr == 0U ||
      packet.destination != host_ip(config.multicast_ip_addr)) return false;
  // AP multicast-to-unicast delivery preserves the multicast IP destination.
  if (std::memcmp(config.local_mac_addr, frame_mac, 6U) == 0) return true;
  const uint8_t expected[6] = {
      0x01U,
      0x00U,
      0x5EU,
      static_cast<uint8_t>((packet.destination >> 16U) & 0x7FU),
      static_cast<uint8_t>((packet.destination >> 8U) & 0xFFU),
      static_cast<uint8_t>(packet.destination & 0xFFU),
  };
  return std::memcmp(expected, frame_mac, sizeof(expected)) == 0;
}

bool destination_ip_matches(const ParsedIpv4 &packet,
                            const CsiFrameFilterConfig &config,
                            bool allow_multicast) {
  if (config.local_ip_addr == 0U) return false;
  if (packet.destination == host_ip(config.local_ip_addr)) return true;
  return allow_multicast && config.multicast_ip_addr != 0U &&
         packet.destination == host_ip(config.multicast_ip_addr);
}

bool matches_external_udp(const ParsedIpv4 &packet, const CsiFrameFilterConfig &config) {
  if (packet.protocol != kIpProtoUdp || !destination_ip_matches(packet, config, true) ||
      packet.transport_len < kTransportMinimumHeaderBytes + RUNTIME_CSI_TRAFFIC_MARKER_LENGTH) {
    return false;
  }
  const uint16_t udp_len = read_be16(packet.transport + 4U);
  return read_be16(packet.transport + 2U) == config.external_udp_port &&
         udp_len == kTransportMinimumHeaderBytes + RUNTIME_CSI_TRAFFIC_MARKER_LENGTH &&
         udp_len == packet.transport_len &&
         std::memcmp(packet.transport + kTransportMinimumHeaderBytes,
                     RUNTIME_CSI_TRAFFIC_MARKER_BYTES,
                     RUNTIME_CSI_TRAFFIC_MARKER_LENGTH) == 0;
}

bool matches_external_ping(const ParsedIpv4 &packet, const CsiFrameFilterConfig &config) {
  return packet.protocol == kIpProtoIcmp && destination_ip_matches(packet, config, false) &&
         packet.transport_len >= kTransportMinimumHeaderBytes &&
         packet.transport[0] == 8U && packet.transport[1] == 0U;
}

bool matches_internal_ping(const ParsedIpv4 &packet, const CsiFrameFilterConfig &config) {
  return packet.protocol == kIpProtoIcmp && packet.source == host_ip(config.gateway_ip_addr) &&
         destination_ip_matches(packet, config, false) &&
         packet.transport_len >= kTransportMinimumHeaderBytes &&
         packet.transport[0] == 0U && packet.transport[1] == 0U &&
         read_be16(packet.transport + 4U) == config.internal_icmp_identifier;
}

bool matches_internal_dns_tcp(const ParsedIpv4 &packet, const CsiFrameFilterConfig &config) {
  if (packet.protocol != kIpProtoTcp || packet.source != host_ip(config.gateway_ip_addr) ||
      !destination_ip_matches(packet, config, false) || packet.transport_len < 20U ||
      read_be16(packet.transport) != 53U) return false;
  const size_t tcp_header_len = static_cast<size_t>(packet.transport[12U] >> 4U) * 4U;
  if (tcp_header_len < 20U || tcp_header_len + 2U > packet.transport_len) return false;
  const size_t dns_payload_len = packet.transport_len - tcp_header_len - 2U;
  const uint16_t declared_dns_len = read_be16(packet.transport + tcp_header_len);
  const uint8_t *dns = packet.transport + tcp_header_len + 2U;
  return declared_dns_len >= 12U && declared_dns_len == dns_payload_len &&
         (dns[2U] & 0x80U) != 0U;
}

bool matches_internal_dns_udp(const ParsedIpv4 &packet, const CsiFrameFilterConfig &config) {
  if (packet.protocol != kIpProtoUdp || packet.source != host_ip(config.gateway_ip_addr) ||
      !destination_ip_matches(packet, config, false) ||
      packet.transport_len < kTransportMinimumHeaderBytes + 12U ||
      read_be16(packet.transport) != 53U) return false;
  const uint16_t udp_len = read_be16(packet.transport + 4U);
  const uint8_t *dns = packet.transport + kTransportMinimumHeaderBytes;
  return udp_len == packet.transport_len && (dns[2U] & 0x80U) != 0U;
}

}  // namespace

bool csi_frame_matches_traffic(const wifi_csi_info_t *info,
                               const CsiFrameFilterConfig &config,
                               CsiCaptureProfile profile) {
  if (info == nullptr) return false;
  if (csi_capture_profile_uses_lltf(profile) && matches_local_ack(info, config)) return true;
  ParsedIpv4 packet;
  if (!parse_bounded_payload(info, &packet) ||
      !destination_mac_matches(packet, config, info->dmac)) return false;
  if (config.traffic_mode == CsiTrafficMode::EXTERNAL) {
    return matches_external_udp(packet, config) || matches_external_ping(packet, config);
  }
  switch (config.internal_mode) {
    case RuntimeTrafficMode::WIFI_RAW:
      return false;  // Only the LLTF20 ACK path above supplies raw Wi-Fi samples.
    case RuntimeTrafficMode::DNS:
      return matches_internal_dns_udp(packet, config);
    case RuntimeTrafficMode::DNS_TCP:
      return matches_internal_dns_tcp(packet, config);
    case RuntimeTrafficMode::PING:
    default:
      return matches_internal_ping(packet, config);
  }
}

}  // namespace espectre
