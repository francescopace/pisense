/*
 * ESPectre - Direct HTTP Service Boundary
 *
 * Transport boundary for local HTTP carriage of canonical ESPectre messages.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "direct_http_protocol.h"
#include "raw_csi.h"

namespace espectre {

struct DirectHttpServiceConfig {
  /** Configuration for ESPectre's production and validation portals. */
  static DirectHttpServiceConfig for_first_party_portals() {
    DirectHttpServiceConfig config;
    config.allowed_origins = {
        "https://espectre.dev",
        "https://www.espectre.dev",
        "https://test.espectre.dev",
    };
    return config;
  }

  std::vector<std::string> allowed_origins;
  uint64_t device_id{0U};
  uint16_t port{ESPECTRE_DIRECT_HTTP_PORT};
  size_t max_event_clients{2U};
  size_t max_pending_requests{4U};
  size_t outbound_queue_depth{8U};
  uint16_t max_requests_per_second{20U};
  uint16_t max_mutations_per_minute{60U};
  bool allow_missing_origin{false};
  bool allow_http_loopback_origins{false};
  /** Optional frontend routes. The immutable catalog must outlive this service. */
  const EspectreProtocolExtension *protocol_extension{nullptr};
};

struct DirectHttpServiceDiagnostics {
  size_t event_client_limit{0U};
  size_t queue_capacity{0U};
  uint32_t accepted_connections{0U};
  uint32_t rejected_connections{0U};
  uint32_t malformed_requests{0U};
  uint32_t oversized_requests{0U};
  uint32_t rate_limited_requests{0U};
  uint32_t dropped_motion_events{0U};
  uint32_t send_failures{0U};
  size_t queued_messages{0U};
};

/** Local HTTP endpoints shared by ESPectre firmware frontends. */
class IDirectHttpService {
 public:
  using RequestHandler = std::function<std::string(const DirectRequest &request)>;
  using ResponseSentCallback = std::function<void(bool sent)>;
  struct DeferredRequestResult {
    bool deferred{false};
    std::string response;
    /** Runs on the frontend task after the response send attempt completes. */
    ResponseSentCallback response_sent_callback{};
  };
  using DeferredRequestHandler =
      std::function<DeferredRequestResult(uint64_t request_token, const DirectRequest &request)>;
  using ClientCountCallback = std::function<void(size_t event_client_count)>;
  using RawSessionRequestedCallback = std::function<bool(std::string *message)>;
  using RawSessionStoppedCallback = std::function<void(RawCsiStopReason reason)>;

  virtual ~IDirectHttpService() = default;

  /** Configure and start the endpoint. Safe to call again after shutdown. */
  virtual bool setup(const DirectHttpServiceConfig &config,
                     RequestHandler request_handler,
                     ClientCountCallback client_count_callback) = 0;
  /**
   * Configure a handler that may complete a request later.
   *
   * The default preserves source compatibility for external transports that
   * implement only synchronous Direct requests. A successful deferred handler
   * must eventually call complete_deferred_response() with the opaque token.
   */
  virtual bool setup_deferred(const DirectHttpServiceConfig &config,
                              DeferredRequestHandler request_handler,
                              ClientCountCallback client_count_callback) {
    (void) config;
    (void) request_handler;
    (void) client_count_callback;
    return false;
  }
  /** Queue a deferred response only if the originating connection is live. */
  virtual bool complete_deferred_response(uint64_t request_token, std::string response) {
    (void) request_token;
    (void) response;
    return false;
  }
  /**
   * Pump deferred receive, dispatch, send work, and application callbacks from
   * the frontend task. Request, client-count, and raw-stop callbacks are never
   * delivered from HTTP server or streaming worker tasks.
   */
  virtual void loop() = 0;
  /** Stop accepting clients, close sockets, and release queued messages. */
  virtual void shutdown() = 0;
  virtual bool running() const = 0;
  virtual size_t event_client_count() const = 0;

  /**
   * Queue a normalized event for every connected client.
   *
   * Telemetry events may replace an older queued event with the same name.
   * State transitions and command responses must never be replaced by
   * telemetry. Returns false when no client can accept the event.
   */
  virtual bool publish_event(const std::string &event_name,
                             const std::string &data_json,
                             bool replaceable_telemetry) = 0;
  virtual DirectHttpServiceDiagnostics diagnostics() const = 0;

  /** Register the frontend-task callback that opens collection for GET /csi. */
  virtual void set_raw_session_requested_callback(RawSessionRequestedCallback callback) {
    (void) callback;
  }

  /** Begin one owner-bound raw session on the service's binary endpoint. */
  virtual bool start_raw_session(const RawCsiSessionConfig &config,
                                 RawSessionStoppedCallback stopped_callback) {
    (void) config;
    (void) stopped_callback;
    return false;
  }
  /**
   * Stop the active raw session and close its binary socket.
   *
   * The stopped callback is delivered by loop(), or synchronously while
   * shutdown() completes on the owning frontend task.
   */
  virtual bool stop_raw_session(RawCsiStopReason reason) {
    (void) reason;
    return false;
  }
  /** Copy one callback-scoped sample into the transport's bounded raw slots. */
  virtual bool offer_raw_packet(const RawCsiPacketView &packet) {
    (void) packet;
    return false;
  }
  virtual RawCsiSessionDiagnostics raw_diagnostics() const { return {}; }
};

}  // namespace espectre
