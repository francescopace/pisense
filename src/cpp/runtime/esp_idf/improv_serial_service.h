/*
 * ESPectre - Improv Serial Service
 *
 * Standard Improv Wi-Fi provisioning over the ESP-IDF primary console.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <string>

namespace espectre {

constexpr uint8_t kImprovGetMatterOnboardingCommand = 0x80U;

enum class ImprovSerialProvisioningState : uint8_t {
  IDLE = 0,
  PENDING,
  APPLIED,
  FAILED,
};

struct ImprovSerialServiceConfig {
  using BeginProvisioningCallback =
      std::function<bool(const std::string &ssid, const std::string &password, std::string *message)>;
  using ProvisioningStateCallback = std::function<ImprovSerialProvisioningState()>;
  using MatterOnboardingCallback = std::function<bool(std::string *qr, std::string *manual_code)>;

  std::string firmware_name;
  std::string firmware_version;
  std::string hardware_variant;
  std::string device_name;
  std::function<std::string()> device_url;
  BeginProvisioningCallback begin_provisioning;
  ProvisioningStateCallback provisioning_state;
  std::function<bool()> network_connected;
  MatterOnboardingCallback matter_onboarding;
};

class ImprovSerialService {
 public:
  using ReadCallback = std::function<int(uint8_t *data, size_t capacity)>;
  using WriteCallback = std::function<int(const uint8_t *data, size_t length)>;

  explicit ImprovSerialService(ReadCallback read_callback = {},
                               WriteCallback write_callback = {});

  bool setup(ImprovSerialServiceConfig config);
  /** Read and process a bounded number of bytes without blocking. */
  void loop();
  void shutdown();

 private:
  bool process_byte_(uint8_t byte);
  bool handle_command_(uint8_t command, const std::string &ssid, const std::string &password);
  void sync_state_();
  bool send_state_(uint8_t state);
  bool send_error_(uint8_t error);
  bool send_rpc_response_(uint8_t command, const std::initializer_list<std::string> &data);
  bool send_frame_(uint8_t type, const uint8_t *data, size_t length);
  void flush_output_();
  bool connected_() const;
  std::string device_url_() const;

  ImprovSerialServiceConfig config_{};
  ReadCallback read_callback_{};
  WriteCallback write_callback_{};
  std::array<uint8_t, 266U> receive_buffer_{};
  std::array<uint8_t, 1024U> transmit_buffer_{};
  size_t receive_position_{0U};
  size_t transmit_position_{0U};
  size_t transmit_length_{0U};
  uint8_t state_{0U};
  bool setup_complete_{false};
  bool provisioning_request_pending_{false};
};

}  // namespace espectre
