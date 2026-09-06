/*
 * ESPectre - Raw CSI Session Controller
 *
 * Shared owner-bound raw collection orchestration for Direct frontends.
 *
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "direct_http_service.h"
#include "frontend_command_engine.h"
#include "runtime_frontend_controller.h"

namespace espectre {

class RawCsiSessionController {
 public:
  using StoppedCallback = std::function<void(RawCsiStopReason reason)>;
  using StartedCallback = std::function<void()>;

  void configure(IDirectHttpService *service,
                 RuntimeFrontendController *runtime,
                 uint64_t device_id,
                 std::string chip,
                 StoppedCallback stopped_callback = {},
                 StartedCallback started_callback = {});
  bool handle_command(const EspectreCommand &command,
                      const FrontendCommandContext &context,
                      std::string *code,
                      std::string *message,
                      std::string *data_json);
  bool begin(std::string *message = nullptr);
  void ensure_runtime_consistency();
  void shutdown(RawCsiStopReason reason = RawCsiStopReason::SHUTDOWN);
  bool active() const { return active_; }

 private:
  static bool offer_packet_(void *context, const RawCsiPacketView &packet);
  void handle_stopped_(RawCsiStopReason reason);

  IDirectHttpService *service_{nullptr};
  RuntimeFrontendController *runtime_{nullptr};
  uint64_t device_id_{0U};
  std::string chip_;
  bool active_{false};
  StoppedCallback stopped_callback_{};
  StartedCallback started_callback_{};
};

}  // namespace espectre
