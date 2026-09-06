/*
 * ESPectre - Improv Serial Service
 *
 * Standard Improv Wi-Fi provisioning over the ESP-IDF primary console.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "improv_serial_service.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <improv.h>
#include <esp_system.h>

#if defined(ESP_PLATFORM) && CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include <driver/usb_serial_jtag.h>
#include <driver/usb_serial_jtag_vfs.h>
#endif

#include "espectre_log.h"

namespace espectre {

namespace {

static const char *const TAG = "espectre.improv";
constexpr size_t kMaxReadBytesPerLoop = 64U;

}  // namespace

ImprovSerialService::ImprovSerialService(ReadCallback read_callback,
                                         WriteCallback write_callback)
    : read_callback_(std::move(read_callback)),
      write_callback_(std::move(write_callback)) {}

bool ImprovSerialService::setup(ImprovSerialServiceConfig config) {
  const bool has_any_provisioning_callback =
      config.begin_provisioning || config.provisioning_state || config.network_connected;
  const bool has_all_provisioning_callbacks =
      config.begin_provisioning && config.provisioning_state && config.network_connected;
  if (config.firmware_name.empty() || config.firmware_version.empty() ||
      config.hardware_variant.empty() || config.device_name.empty() ||
      (has_any_provisioning_callback && !has_all_provisioning_callbacks)) {
    ESPECTRE_LOGE(TAG, "Invalid Improv Serial configuration");
    return false;
  }

#if defined(ESP_PLATFORM) && CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  if (!read_callback_ || !write_callback_) {
    if (!usb_serial_jtag_is_driver_installed()) {
      usb_serial_jtag_driver_config_t driver_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
      const esp_err_t result = usb_serial_jtag_driver_install(&driver_config);
      if (result != ESP_OK) {
        ESPECTRE_LOGE(TAG, "Failed to install the primary USB Serial/JTAG driver: %s", esp_err_to_name(result));
        return false;
      }
    }
    // The default polling VFS can lose host-to-device packets while the main
    // loop is busy. The interrupt-driven driver buffers complete Improv RPCs.
    usb_serial_jtag_vfs_use_driver();
    // Improv is a binary protocol. The console VFS defaults to CRLF output and
    // would expand a checksum byte equal to LF into CRLF, corrupting the frame.
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
  }
#endif

  if (!read_callback_) {
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags < 0 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
      ESPECTRE_LOGE(TAG, "Failed to make the primary console non-blocking: errno=%d", errno);
      return false;
    }
    read_callback_ = [](uint8_t *data, size_t capacity) {
      return static_cast<int>(read(STDIN_FILENO, data, capacity));
    };
  }
  if (!write_callback_) {
    const int flags = fcntl(STDOUT_FILENO, F_GETFL, 0);
    if (flags < 0 || fcntl(STDOUT_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
      ESPECTRE_LOGE(TAG, "Failed to make the primary console output non-blocking: errno=%d", errno);
      return false;
    }
    write_callback_ = [](const uint8_t *data, size_t length) {
      return static_cast<int>(write(STDOUT_FILENO, data, length));
    };
  }

  config_ = std::move(config);
  receive_position_ = 0U;
  transmit_position_ = 0U;
  transmit_length_ = 0U;
  provisioning_request_pending_ = false;
  setup_complete_ = true;
  state_ = connected_() ? improv::STATE_PROVISIONED : improv::STATE_AUTHORIZED;
  (void) send_error_(improv::ERROR_NONE);
  (void) send_state_(state_);
  ESPECTRE_LOGI(TAG, "Improv Serial ready on the primary console");
  return true;
}

void ImprovSerialService::loop() {
  if (!setup_complete_) {
    return;
  }

  flush_output_();
  uint8_t bytes[kMaxReadBytesPerLoop]{};
  const int count = read_callback_(bytes, sizeof(bytes));
  if (count > 0) {
    for (int index = 0; index < count; ++index) {
      (void) process_byte_(bytes[index]);
    }
  } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
    ESPECTRE_LOGW(TAG, "Primary console read failed: errno=%d", errno);
  }

  sync_state_();
  flush_output_();
}

void ImprovSerialService::shutdown() {
  setup_complete_ = false;
  provisioning_request_pending_ = false;
  receive_position_ = 0U;
  transmit_position_ = 0U;
  transmit_length_ = 0U;
  state_ = improv::STATE_STOPPED;
}

bool ImprovSerialService::process_byte_(uint8_t byte) {
  if (receive_position_ >= receive_buffer_.size()) {
    receive_position_ = 0U;
  }

  receive_buffer_[receive_position_] = byte;

  // The upstream parser assumes every RPC body contains at least command and
  // length bytes. Reject a shorter body before it can index that payload.
  if (receive_position_ == 9U && receive_buffer_[7] == improv::TYPE_RPC && receive_buffer_[8] < 2U) {
    receive_position_ = 0U;
    (void) send_error_(improv::ERROR_INVALID_RPC);
    return false;
  }

  const bool keep_receiving = improv::parse_improv_serial_byte(
      receive_position_,
      byte,
      receive_buffer_.data(),
      [this](improv::ImprovCommand command) {
        return handle_command_(static_cast<uint8_t>(command.command), command.ssid, command.password);
      },
      [this](improv::Error error) { (void) send_error_(static_cast<uint8_t>(error)); });

  if (keep_receiving) {
    ++receive_position_;
    return true;
  }

  receive_position_ = byte == static_cast<uint8_t>('I') ? 1U : 0U;
  if (receive_position_ == 1U) {
    receive_buffer_[0] = byte;
  }
  return false;
}

bool ImprovSerialService::handle_command_(uint8_t command,
                                          const std::string &ssid,
                                          const std::string &password) {
  (void) send_error_(improv::ERROR_NONE);

  switch (command) {
    case static_cast<uint8_t>(improv::WIFI_SETTINGS): {
      if (!config_.begin_provisioning) {
        (void) send_error_(improv::ERROR_UNKNOWN_RPC);
        return false;
      }
      std::string message;
      if (!config_.begin_provisioning(ssid, password, &message)) {
        ESPECTRE_LOGW(TAG, "Rejected Improv Wi-Fi settings: %s", message.c_str());
        (void) send_error_(improv::ERROR_INVALID_RPC);
        return false;
      }
      provisioning_request_pending_ = true;
      state_ = improv::STATE_PROVISIONING;
      (void) send_state_(state_);
      return false;
    }
    case static_cast<uint8_t>(improv::GET_CURRENT_STATE):
      state_ = connected_() ? improv::STATE_PROVISIONED : improv::STATE_AUTHORIZED;
      (void) send_state_(state_);
      if (state_ == improv::STATE_PROVISIONED) {
        (void) send_rpc_response_(improv::GET_CURRENT_STATE, {device_url_()});
      }
      return false;
    case static_cast<uint8_t>(improv::GET_DEVICE_INFO):
      (void) send_rpc_response_(improv::GET_DEVICE_INFO,
                                {config_.firmware_name,
                                 config_.firmware_version,
                                 config_.hardware_variant,
                                 config_.device_name,
                                 "ESP-IDF",
                                 esp_get_idf_version()});
      return false;
    case static_cast<uint8_t>(improv::GET_NETWORK_STATE): {
      if (!config_.network_connected) {
        (void) send_error_(improv::ERROR_UNKNOWN_RPC);
        return false;
      }
      uint8_t flags = improv::NETWORK_SUPPORTS_WIFI;
      if (connected_()) {
        flags |= improv::NETWORK_IS_ONLINE;
        (void) send_rpc_response_(improv::GET_NETWORK_STATE,
                                  {std::to_string(static_cast<unsigned>(flags)), device_url_()});
      } else {
        (void) send_rpc_response_(improv::GET_NETWORK_STATE,
                                  {std::to_string(static_cast<unsigned>(flags))});
      }
      return false;
    }
    case kImprovGetMatterOnboardingCommand: {
      std::string qr;
      std::string manual_code;
      if (!config_.matter_onboarding || !config_.matter_onboarding(&qr, &manual_code) ||
          qr.empty() || manual_code.empty()) {
        (void) send_error_(improv::ERROR_UNKNOWN);
        return false;
      }
      (void) send_rpc_response_(kImprovGetMatterOnboardingCommand, {qr, manual_code});
      return false;
    }
    case static_cast<uint8_t>(improv::UNKNOWN):
    case static_cast<uint8_t>(improv::BAD_CHECKSUM):
      (void) send_error_(improv::ERROR_INVALID_RPC);
      return false;
    default:
      (void) send_error_(improv::ERROR_UNKNOWN_RPC);
      return false;
  }
}

void ImprovSerialService::sync_state_() {
  if (!config_.provisioning_state) {
    return;
  }
  if (!provisioning_request_pending_) {
    const uint8_t current = connected_() ? improv::STATE_PROVISIONED : improv::STATE_AUTHORIZED;
    if (current != state_) {
      state_ = current;
      (void) send_state_(state_);
    }
    return;
  }

  switch (config_.provisioning_state()) {
    case ImprovSerialProvisioningState::APPLIED:
      if (connected_()) {
        state_ = improv::STATE_PROVISIONED;
        (void) send_state_(state_);
        (void) send_rpc_response_(improv::WIFI_SETTINGS, {device_url_()});
        provisioning_request_pending_ = false;
      }
      break;
    case ImprovSerialProvisioningState::FAILED:
      (void) send_error_(improv::ERROR_UNABLE_TO_CONNECT);
      state_ = connected_() ? improv::STATE_PROVISIONED : improv::STATE_AUTHORIZED;
      (void) send_state_(state_);
      provisioning_request_pending_ = false;
      break;
    default:
      break;
  }
}

bool ImprovSerialService::send_state_(uint8_t state) {
  return send_frame_(improv::TYPE_CURRENT_STATE, &state, 1U);
}

bool ImprovSerialService::send_error_(uint8_t error) {
  return send_frame_(improv::TYPE_ERROR_STATE, &error, 1U);
}

bool ImprovSerialService::send_rpc_response_(uint8_t command,
                                             const std::initializer_list<std::string> &data) {
  const std::vector<std::string> values(data);
  std::vector<uint8_t> response =
      improv::build_rpc_response(static_cast<improv::Command>(command), values, false);
  // sdk-cpp reserves its optional checksum byte even when checksum generation
  // is disabled. Serial framing owns that checksum, so omit the reserved byte.
  if (!response.empty()) {
    response.pop_back();
  }
  return send_frame_(improv::TYPE_RPC_RESPONSE, response.data(), response.size());
}

bool ImprovSerialService::send_frame_(uint8_t type, const uint8_t *data, size_t length) {
  if (length > 255U || (length > 0U && data == nullptr)) {
    return false;
  }

  std::array<uint8_t, 266U> frame{};
  constexpr uint8_t header[] = {'I', 'M', 'P', 'R', 'O', 'V'};
  size_t position = 0U;
  for (uint8_t byte : header) {
    frame[position++] = byte;
  }
  frame[position++] = improv::IMPROV_SERIAL_VERSION;
  frame[position++] = type;
  frame[position++] = static_cast<uint8_t>(length);
  for (size_t index = 0U; index < length; ++index) {
    frame[position++] = data[index];
  }
  uint8_t checksum = 0U;
  for (size_t index = 0U; index < position; ++index) {
    checksum = static_cast<uint8_t>(checksum + frame[index]);
  }
  frame[position++] = checksum;
  // ESP Web Tools uses LF to recover framing after console output or a
  // partial packet that was already buffered when the serial client attached.
  frame[position++] = static_cast<uint8_t>('\n');
  const size_t pending = transmit_length_ - transmit_position_;
  if (pending + position > transmit_buffer_.size()) {
    return false;
  }
  if (pending > 0U && transmit_position_ > 0U) {
    std::memmove(transmit_buffer_.data(), transmit_buffer_.data() + transmit_position_, pending);
  }
  transmit_position_ = 0U;
  transmit_length_ = pending;
  std::memcpy(transmit_buffer_.data() + transmit_length_, frame.data(), position);
  transmit_length_ += position;
  flush_output_();
  return true;
}

void ImprovSerialService::flush_output_() {
  if (!write_callback_ || transmit_position_ >= transmit_length_) {
    transmit_position_ = 0U;
    transmit_length_ = 0U;
    return;
  }
  const size_t remaining = transmit_length_ - transmit_position_;
  const int written = write_callback_(transmit_buffer_.data() + transmit_position_, remaining);
  if (written <= 0) {
    return;
  }
  transmit_position_ += std::min(static_cast<size_t>(written), remaining);
  if (transmit_position_ == transmit_length_) {
    transmit_position_ = 0U;
    transmit_length_ = 0U;
  }
}

bool ImprovSerialService::connected_() const {
  return config_.network_connected && config_.network_connected();
}

std::string ImprovSerialService::device_url_() const {
  return config_.device_url ? config_.device_url() : std::string{};
}

}  // namespace espectre
