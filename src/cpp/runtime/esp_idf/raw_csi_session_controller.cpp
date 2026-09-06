/*
 * ESPectre - Raw CSI Session Controller
 *
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "raw_csi_session_controller.h"

#include <cctype>
#include <utility>

#include <esp_timer.h>
#if defined(ESP_PLATFORM)
#include <esp_random.h>
#endif

namespace espectre {
namespace {

RawCsiChipType raw_chip_type(const std::string &chip) {
  std::string normalized;
  for (char character : chip) {
    if (std::isalnum(static_cast<unsigned char>(character))) {
      normalized.push_back(
          static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
    }
  }
  if (normalized == "esp32c3" || normalized == "c3") return RawCsiChipType::C3;
  if (normalized == "esp32c5" || normalized == "c5") return RawCsiChipType::C5;
  if (normalized == "esp32c6" || normalized == "c6") return RawCsiChipType::C6;
  if (normalized == "esp32s2" || normalized == "s2") return RawCsiChipType::S2;
  if (normalized == "esp32s3" || normalized == "s3") return RawCsiChipType::S3;
  if (normalized == "esp32") return RawCsiChipType::ESP32;
  return RawCsiChipType::UNKNOWN;
}

void fill_session_id(uint8_t *session_id, uint64_t seed) {
#if defined(ESP_PLATFORM)
  esp_fill_random(session_id, ESPECTRE_RAW_CSI_SESSION_ID_BYTES);
#else
  seed ^= static_cast<uint64_t>(esp_timer_get_time());
  for (size_t index = 0U; index < ESPECTRE_RAW_CSI_SESSION_ID_BYTES; ++index) {
    seed ^= seed << 13U;
    seed ^= seed >> 7U;
    seed ^= seed << 17U;
    session_id[index] = static_cast<uint8_t>(seed);
  }
#endif
}

}  // namespace

void RawCsiSessionController::configure(IDirectHttpService *service,
                                        RuntimeFrontendController *runtime,
                                        uint64_t device_id,
                                        std::string chip,
                                        StoppedCallback stopped_callback,
                                        StartedCallback started_callback) {
  service_ = service;
  runtime_ = runtime;
  device_id_ = device_id;
  chip_ = std::move(chip);
  stopped_callback_ = std::move(stopped_callback);
  started_callback_ = std::move(started_callback);
  if (service_ != nullptr) {
    service_->set_raw_session_requested_callback(
        [this](std::string *message) { return this->begin(message); });
  }
}

bool RawCsiSessionController::begin(std::string *message) {
  if (service_ == nullptr || runtime_ == nullptr || device_id_ == 0U ||
      !runtime_->capabilities().supports_raw_csi) {
    if (message != nullptr) *message = "raw CSI collection is unavailable";
    return false;
  }
  if (active_ || runtime_->operation_state() == RuntimeOperationState::RAW_COLLECTION) {
    if (message != nullptr) *message = "a CSI collection is already active";
    return false;
  }
  RawCsiSessionConfig session;
  session.device_id = device_id_;
  session.chip = raw_chip_type(chip_);
  fill_session_id(session.session_id, device_id_);
  if (!service_->start_raw_session(
          session, [this](RawCsiStopReason reason) { handle_stopped_(reason); })) {
    if (message != nullptr) *message = "the CSI collector is busy";
    return false;
  }
  if (!runtime_->start_raw_collection(&offer_packet_, this)) {
    (void) service_->stop_raw_session(RawCsiStopReason::INTERNAL_ERROR);
    if (message != nullptr) *message = "CSI capture could not be started";
    return false;
  }
  active_ = true;
  if (started_callback_) started_callback_();
  if (message != nullptr) *message = "CSI collection started";
  return true;
}

bool RawCsiSessionController::handle_command(const EspectreCommand &command,
                                             const FrontendCommandContext &context,
                                             std::string *code,
                                             std::string *message,
                                             std::string *data_json) {
  if (service_ == nullptr || runtime_ == nullptr || device_id_ == 0U ||
      !runtime_->capabilities().supports_raw_csi) {
    if (code != nullptr) *code = "unsupported";
    if (message != nullptr) *message = "raw CSI collection is unavailable";
    return false;
  }
  (void) command;
  (void) context;
  (void) data_json;
  if (code != nullptr) *code = "unsupported";
  if (message != nullptr) *message = "CSI collection is opened with GET /csi";
  return false;
}

void RawCsiSessionController::ensure_runtime_consistency() {
  if (active_ && runtime_ != nullptr && service_ != nullptr &&
      runtime_->operation_state() != RuntimeOperationState::RAW_COLLECTION) {
    (void) service_->stop_raw_session(RawCsiStopReason::INTERNAL_ERROR);
  }
}

void RawCsiSessionController::shutdown(RawCsiStopReason reason) {
  if (active_ && service_ != nullptr) {
    (void) service_->stop_raw_session(reason);
  }
  active_ = false;
}

bool RawCsiSessionController::offer_packet_(void *context,
                                            const RawCsiPacketView &packet) {
  auto *controller = static_cast<RawCsiSessionController *>(context);
  return controller != nullptr && controller->service_ != nullptr &&
         controller->service_->offer_raw_packet(packet);
}

void RawCsiSessionController::handle_stopped_(RawCsiStopReason reason) {
  active_ = false;
  if (runtime_ != nullptr) (void) runtime_->stop_raw_collection(reason);
  if (stopped_callback_) stopped_callback_(reason);
}

}  // namespace espectre
