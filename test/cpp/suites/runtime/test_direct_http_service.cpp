/*
 * ESPectre - ESP-IDF Direct HTTP Service Tests
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>
#include <new>

#include "direct_http_service_esp_idf.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "espectre_protocol.h"

namespace {
bool reject_nothrow_allocation = false;
bool count_allocations = false;
size_t allocation_count = 0U;
}

void* operator new(std::size_t size) {
  if (count_allocations) ++allocation_count;
  if (void *memory = std::malloc(size > 0U ? size : 1U)) return memory;
  throw std::bad_alloc();
}

void operator delete(void *memory) noexcept { std::free(memory); }
void operator delete(void *memory, std::size_t) noexcept { std::free(memory); }

void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  if (reject_nothrow_allocation) return nullptr;
  try {
    return ::operator new(size);
  } catch (...) {
    return nullptr;
  }
}

using namespace espectre;

namespace {

DirectHttpServiceConfig config() {
  DirectHttpServiceConfig value = DirectHttpServiceConfig::for_first_party_portals();
  value.max_event_clients = 2U;
  value.max_pending_requests = 2U;
  value.outbound_queue_depth = 2U;
  value.max_requests_per_second = 20U;
  value.max_mutations_per_minute = 2U;
  value.allow_http_loopback_origins = true;
  return value;
}

httpd_req_t request_for(size_t registered_index, int fd = 7) {
  httpd_req_t request{};
  request.user_ctx = g_httpd_mock.registered_uris[0].user_ctx;
  request.fd = fd;
  request.method = registered_index == 0U ? HTTP_PATCH
                   : registered_index == 3U ? HTTP_OPTIONS
                                            : HTTP_GET;
  request.uri = registered_index == 1U ? ESPECTRE_DIRECT_HTTP_EVENTS_ENDPOINT
                : registered_index == 2U ? ESPECTRE_RAW_CSI_ENDPOINT
                                         : "/espectre/v1/sensing";
  request.content_len = g_httpd_mock.incoming_length;
  return request;
}

httpd_req_t request_for_route(httpd_method_t method, const char *uri, int fd = 7) {
  httpd_req_t request{};
  request.user_ctx = g_httpd_mock.registered_uris[0].user_ctx;
  request.fd = fd;
  request.method = method;
  request.uri = uri;
  request.content_len = g_httpd_mock.incoming_length;
  return request;
}

esp_err_t dispatch_request(httpd_req_t *request) {
  for (int index = 0; index < g_httpd_mock.register_calls; ++index) {
    if (g_httpd_mock.registered_uris[index].method == request->method) {
      return g_httpd_mock.registered_uris[index].handler(request);
    }
  }
  return ESP_ERR_NOT_FOUND;
}

void prepare_json(const char *payload, const char *origin = "https://espectre.dev") {
  httpd_mock_set_incoming(payload);
  httpd_mock_set_header("Origin", origin);
  httpd_mock_set_header("Content-Type", "application/json; charset=utf-8");
}

std::string sent_payload(int index) {
  return std::string(reinterpret_cast<const char *>(g_httpd_mock.sent_payloads[index]),
                     g_httpd_mock.sent_lengths[index]);
}

std::string command_result(const DirectRequest &request, const std::string &data_json = "{}") {
  EspectreDeviceConfig device;
  EspectreCommand command;
  command.command_id = request.command_id;
  command.command = request.command;
  return espectre_command_result_payload(device, command, true, "ok", "completed", data_json);
}

void accept_raw_open(EspIdfDirectHttpService *service,
                     RawCsiSessionConfig session,
                     IDirectHttpService::RawSessionStoppedCallback stopped_callback = {}) {
  service->set_raw_session_requested_callback(
      [service, session, stopped_callback = std::move(stopped_callback)](std::string *) mutable {
        return service->start_raw_session(session, std::move(stopped_callback));
      });
}

void test_idle_loop_does_not_allocate() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const DirectRequest &) { return std::string("{}"); }, {}));
  allocation_count = 0U;
  count_allocations = true;
  for (size_t iteration = 0U; iteration < 100U; ++iteration) service.loop();
  count_allocations = false;
  TEST_ASSERT_EQUAL(0U, allocation_count);
}

void test_raw_storage_failure_and_repeated_sessions_preserve_capacity() {
  httpd_mock_reset();
  // Raw packet storage must not be embedded in the always-present service.
  TEST_ASSERT_TRUE(sizeof(EspIdfDirectHttpService) < 4096U);
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const DirectRequest &) { return std::string("{}"); }, {}));
  RawCsiSessionConfig session;
  session.session_id[0] = 1U;
  reject_nothrow_allocation = true;
  const bool started_without_storage = service.start_raw_session(session, {});
  reject_nothrow_allocation = false;
  TEST_ASSERT_FALSE(started_without_storage);
  TEST_ASSERT_FALSE(service.raw_diagnostics().active);
  int8_t payload[2]{};
  RawCsiPacketView packet;
  packet.csi = payload;
  packet.csi_len = sizeof(payload);
  size_t capacity = 0U;
  for (uint8_t cycle = 0U; cycle < 3U; ++cycle) {
    TEST_ASSERT_TRUE(service.start_raw_session(session, {}));
    size_t accepted = 0U;
    while (accepted < 100U && service.offer_raw_packet(packet)) ++accepted;
    TEST_ASSERT_TRUE(accepted > 0U && accepted < 100U);
    if (cycle == 0U) capacity = accepted;
    TEST_ASSERT_EQUAL(capacity, accepted);
    TEST_ASSERT_TRUE(service.stop_raw_session(RawCsiStopReason::REQUESTED));
    TEST_ASSERT_FALSE(service.offer_raw_packet(packet));
    TEST_ASSERT_EQUAL(capacity + 1U, service.raw_diagnostics().raw_drop_total);
    service.loop();
  }
}

void test_setup_registers_http_post_sse_raw_and_preflight() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  DirectHttpServiceConfig invalid;
  TEST_ASSERT_FALSE(service.setup(invalid, [](const auto &) { return std::string{"{}"}; }, {}));
  DirectHttpServiceConfig zero_request_budget = config();
  zero_request_budget.max_requests_per_second = 0U;
  TEST_ASSERT_FALSE(service.setup(
      zero_request_budget, [](const auto &) { return std::string{"{}"}; }, {}));
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  TEST_ASSERT_TRUE(service.running());
  TEST_ASSERT_EQUAL(6, g_httpd_mock.register_calls);
  TEST_ASSERT_EQUAL_STRING("/espectre/v1/*",
                           g_httpd_mock.registered_uris[0].uri);
  TEST_ASSERT_EQUAL(HTTP_GET, g_httpd_mock.registered_uris[0].method);
  TEST_ASSERT_EQUAL_STRING("/espectre/v1/*",
                           g_httpd_mock.registered_uris[1].uri);
  TEST_ASSERT_EQUAL(HTTP_POST, g_httpd_mock.registered_uris[1].method);
  TEST_ASSERT_EQUAL(HTTP_PUT, g_httpd_mock.registered_uris[2].method);
  TEST_ASSERT_EQUAL(HTTP_DELETE, g_httpd_mock.registered_uris[3].method);
  TEST_ASSERT_EQUAL(HTTP_PATCH, g_httpd_mock.registered_uris[4].method);
  TEST_ASSERT_EQUAL(HTTP_OPTIONS, g_httpd_mock.registered_uris[5].method);
  TEST_ASSERT_EQUAL(7U, g_httpd_mock.last_config.max_open_sockets);
  TEST_ASSERT_EQUAL(8U, g_httpd_mock.last_config.max_uri_handlers);
  TEST_ASSERT_EQUAL(ESPECTRE_DIRECT_HTTP_PORT, g_httpd_mock.last_config.server_port);
  TEST_ASSERT_EQUAL(1U, g_httpd_mock.last_config.task_priority);
  TEST_ASSERT_EQUAL(5U, g_httpd_mock.last_config.recv_wait_timeout);
  TEST_ASSERT_EQUAL(1U, g_httpd_mock.last_config.send_wait_timeout);
  TEST_ASSERT_TRUE(g_httpd_mock.last_config.open_fn != nullptr);
  TEST_ASSERT_EQUAL(ESP_FAIL, g_httpd_mock.last_config.open_fn(nullptr, -1));
  service.shutdown();
  TEST_ASSERT_EQUAL(1, g_httpd_mock.stop_calls);
}

void test_setup_failure_releases_server_and_allows_retry() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  const auto handler = [](const DirectRequest &) { return std::string("{}"); };
  g_httpd_mock.start_result = ESP_FAIL;
  TEST_ASSERT_FALSE(service.setup(config(), handler, {}));
  TEST_ASSERT_FALSE(service.running());
  TEST_ASSERT_EQUAL(0, g_httpd_mock.stop_calls);
  g_httpd_mock.start_result = ESP_OK;
  g_httpd_mock.register_result = ESP_FAIL;
  TEST_ASSERT_FALSE(service.setup(config(), handler, {}));
  TEST_ASSERT_FALSE(service.running());
  TEST_ASSERT_EQUAL(1, g_httpd_mock.stop_calls);
  g_httpd_mock.register_result = ESP_OK;
  TEST_ASSERT_TRUE(service.setup(config(), handler, {}));
  TEST_ASSERT_TRUE(service.running());
  service.shutdown();
  TEST_ASSERT_EQUAL(2, g_httpd_mock.stop_calls);
}

void test_empty_and_oversized_application_responses_return_bounded_command_errors() {
  for (size_t size : {size_t{0}, ESPECTRE_DIRECT_MAX_RESPONSE_SIZE + 1U}) {
    httpd_mock_reset();
    EspIdfDirectHttpService service;
    TEST_ASSERT_TRUE(service.setup(config(), [size](const DirectRequest &) {
      return std::string(size, 'x');
    }, {}));
    prepare_json("{\"enabled\":true}");
    httpd_req_t request = request_for(0U);
    TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&request));
    service.loop();
    const std::string payload = sent_payload(g_httpd_mock.send_calls - 1);
    TEST_ASSERT_TRUE(payload.size() <= ESPECTRE_DIRECT_MAX_RESPONSE_SIZE);
    TEST_ASSERT_TRUE(payload.find("\"accepted\":false") != std::string::npos);
    TEST_ASSERT_TRUE(payload.find("\"code\":\"internal_error\"") != std::string::npos);
    TEST_ASSERT_EQUAL(1, g_httpd_mock.async_complete_calls);
    service.shutdown();
  }
}

void test_incomplete_request_never_reaches_application_handler() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  int requests = 0;
  TEST_ASSERT_TRUE(service.setup(config(), [&](const DirectRequest &) {
    requests++;
    return std::string("{}");
  }, {}));
  prepare_json("{\"enabled\":true}");
  g_httpd_mock.receive_result = ESP_FAIL;
  httpd_req_t request = request_for(0U);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&request));
  TEST_ASSERT_EQUAL_STRING(HTTPD_400_BAD_REQUEST, g_httpd_mock.response_status);
  service.loop();
  TEST_ASSERT_EQUAL(0, requests);
  TEST_ASSERT_EQUAL(0, g_httpd_mock.async_begin_calls);
  service.shutdown();
}

void test_sse_initial_send_failure_releases_the_client_slot() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  auto limited = config();
  limited.max_event_clients = 1U;
  TEST_ASSERT_TRUE(service.setup(limited, [](const DirectRequest &) { return std::string("{}"); }, {}));
  prepare_json("");
  g_httpd_mock.send_result = ESP_FAIL;
  httpd_req_t failed = request_for(1U);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&failed));
  TEST_ASSERT_EQUAL(1U, service.diagnostics().send_failures);
  TEST_ASSERT_EQUAL(1U, service.diagnostics().rejected_connections);
  TEST_ASSERT_EQUAL(1, g_httpd_mock.async_complete_calls);
  g_httpd_mock.send_result = ESP_OK;
  httpd_req_t retry = request_for(1U);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&retry));
  TEST_ASSERT_EQUAL(1U, service.diagnostics().accepted_connections);
  service.shutdown();
}

void test_destructor_does_not_dispatch_application_callbacks() {
  httpd_mock_reset();
  size_t client_count_callbacks = 0U;
  size_t raw_stop_callbacks = 0U;
  {
    EspIdfDirectHttpService service;
    TEST_ASSERT_TRUE(service.setup(
        config(),
        [](const auto &) { return std::string{"{}"}; },
        [&client_count_callbacks](size_t) { ++client_count_callbacks; }));
    RawCsiSessionConfig session{};
    session.session_id[0] = 1U;
    TEST_ASSERT_TRUE(service.start_raw_session(
        session, [&raw_stop_callbacks](RawCsiStopReason) { ++raw_stop_callbacks; }));
  }
  TEST_ASSERT_EQUAL(0U, client_count_callbacks);
  TEST_ASSERT_EQUAL(0U, raw_stop_callbacks);
}

void test_shutdown_releases_the_client_count_callback_before_reuse() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  size_t first_callback_calls = 0U;
  size_t second_callback_calls = 0U;

  TEST_ASSERT_TRUE(service.setup(
      config(),
      [](const auto &) { return std::string{"{}"}; },
      [&first_callback_calls](size_t) { ++first_callback_calls; }));
  service.shutdown();
  TEST_ASSERT_EQUAL(1U, first_callback_calls);

  TEST_ASSERT_TRUE(service.setup(
      config(),
      [](const auto &) { return std::string{"{}"}; },
      [&second_callback_calls](size_t) { ++second_callback_calls; }));
  TEST_ASSERT_EQUAL(1U, first_callback_calls);
  service.shutdown();
  TEST_ASSERT_EQUAL(1U, second_callback_calls);

  service.shutdown();
  TEST_ASSERT_EQUAL(1U, first_callback_calls);
  TEST_ASSERT_EQUAL(1U, second_callback_calls);
}

void test_post_does_not_enqueue_after_shutdown_starts() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));

  prepare_json("{\"protocol_version\":\"1.0\",\"command_id\":\"late\",\"command\":\"status\"}");
  g_httpd_mock.async_begin_callback_context = &service;
  g_httpd_mock.async_begin_callback = [](void *context) {
    static_cast<EspIdfDirectHttpService *>(context)->shutdown();
  };

  httpd_req_t request = request_for(0U, 40);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&request));
  TEST_ASSERT_FALSE(service.running());
  TEST_ASSERT_EQUAL_STRING(HTTPD_503_SERVICE_UNAVAILABLE, g_httpd_mock.response_status);
  TEST_ASSERT_EQUAL(1, g_httpd_mock.async_complete_calls);
  TEST_ASSERT_EQUAL(0U, service.diagnostics().queued_messages);
}

void test_sse_does_not_register_after_shutdown_starts() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));

  httpd_mock_set_header("Origin", "https://espectre.dev");
  g_httpd_mock.async_begin_callback_context = &service;
  g_httpd_mock.async_begin_callback = [](void *context) {
    static_cast<EspIdfDirectHttpService *>(context)->shutdown();
  };

  httpd_req_t request = request_for(1U, 41);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&request));
  TEST_ASSERT_FALSE(service.running());
  TEST_ASSERT_EQUAL(1, g_httpd_mock.async_complete_calls);
  TEST_ASSERT_EQUAL(0U, service.event_client_count());
}

void test_post_validates_origin_content_type_size_and_dispatches_on_loop() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  std::string method;
  TEST_ASSERT_TRUE(service.setup(
      config(),
      [&method](const DirectRequest &request) {
        method = request.command;
        return command_result(request, "{\"methods\":[]}");
      },
      {}));

  prepare_json("{\"enabled\":true}",
               "https://evil.example");
  httpd_req_t rejected = request_for(0U);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&rejected));
  TEST_ASSERT_EQUAL_STRING(HTTPD_403_FORBIDDEN, g_httpd_mock.response_status);

  prepare_json("{\"enabled\":true}");
  httpd_req_t request = request_for(0U);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&request));
  TEST_ASSERT_EQUAL(1, g_httpd_mock.send_calls);
  service.loop();
  TEST_ASSERT_EQUAL_STRING("update_sensing", method.c_str());
  TEST_ASSERT_EQUAL(2, g_httpd_mock.send_calls);
  TEST_ASSERT_TRUE(sent_payload(1).find("\"accepted\":true") != std::string::npos);
  TEST_ASSERT_EQUAL_STRING("application/json; charset=utf-8", g_httpd_mock.response_type);
  TEST_ASSERT_EQUAL_STRING("no-store", g_httpd_mock.cache_control);
  TEST_ASSERT_EQUAL_STRING("https://espectre.dev", g_httpd_mock.allow_origin);
  TEST_ASSERT_EQUAL(1, g_httpd_mock.async_complete_calls);

  prepare_json("{\"enabled\":false}");
  httpd_req_t semantic_rejection = request_for(0U);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&semantic_rejection));
  service.loop();
  TEST_ASSERT_EQUAL_STRING("update_sensing", method.c_str());
  TEST_ASSERT_EQUAL(0U, service.diagnostics().malformed_requests);

  prepare_json("{}");
  httpd_mock_set_header("Content-Type", "text/plain");
  httpd_req_t wrong_type = request_for(0U);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&wrong_type));
  TEST_ASSERT_EQUAL_STRING(HTTPD_415_UNSUPPORTED_MEDIA_TYPE, g_httpd_mock.response_status);

  prepare_json("not-json");
  httpd_req_t malformed = request_for(0U);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&malformed));
  TEST_ASSERT_EQUAL_STRING(HTTPD_400_BAD_REQUEST, g_httpd_mock.response_status);
  TEST_ASSERT_EQUAL(1U, service.diagnostics().malformed_requests);

  prepare_json("{}");
  httpd_req_t oversized = request_for(0U);
  oversized.content_len = ESPECTRE_DIRECT_MAX_REQUEST_SIZE + 1U;
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&oversized));
  TEST_ASSERT_EQUAL_STRING(HTTPD_413_CONTENT_TOO_LARGE, g_httpd_mock.response_status);
  TEST_ASSERT_EQUAL(1U, service.diagnostics().oversized_requests);
}

void test_options_returns_private_network_cors_headers() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  httpd_mock_set_header("Origin", "https://test.espectre.dev");
  httpd_mock_set_header("Access-Control-Request-Private-Network", "true");
  httpd_req_t request = request_for(3U);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&request));
  TEST_ASSERT_EQUAL_STRING("204 No Content", g_httpd_mock.response_status);
  TEST_ASSERT_EQUAL_STRING("https://test.espectre.dev", g_httpd_mock.allow_origin);
  TEST_ASSERT_EQUAL_STRING("true", g_httpd_mock.allow_private_network);
  TEST_ASSERT_EQUAL_STRING("no-store", g_httpd_mock.cache_control);
}

void test_post_distinguishes_queue_saturation_from_mutation_rate_limit() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  DirectHttpServiceConfig limits = config();
  limits.max_pending_requests = 1U;
  limits.max_mutations_per_minute = 1U;
  TEST_ASSERT_TRUE(service.setup(
      limits, [](const DirectRequest &request) {
        return command_result(request);
      }, {}));

  prepare_json("{}");
  httpd_req_t queued = request_for_route(HTTP_GET, "/espectre/v1/health", 20);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&queued));
  prepare_json("{}");
  httpd_req_t full = request_for_route(HTTP_GET, "/espectre/v1/device", 21);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&full));
  TEST_ASSERT_EQUAL_STRING(HTTPD_503_SERVICE_UNAVAILABLE, g_httpd_mock.response_status);
  service.loop();

  prepare_json("{\"protocol_version\":\"1.0\",\"command_id\":\"m1\",\"command\":\"set_threshold\",\"threshold\":0.5}");
  httpd_req_t first_mutation = request_for(0U, 22);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&first_mutation));
  service.loop();
  prepare_json("{\"protocol_version\":\"1.0\",\"command_id\":\"m2\",\"command\":\"recalibrate\"}");
  httpd_req_t limited = request_for(0U, 23);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&limited));
  TEST_ASSERT_EQUAL_STRING(HTTPD_429_TOO_MANY_REQUESTS, g_httpd_mock.response_status);
  TEST_ASSERT_EQUAL(1U, service.diagnostics().rate_limited_requests);
}

void test_frontend_routes_obey_read_mutation_and_raw_collection_policies() {
  httpd_mock_reset();
  esp_timer_mock::reset(100000U, 0U);
  const auto validate = +[](const std::vector<JsonObjectField> &, EspectreCommand *, std::string *) { return true; };
  const EspectreProtocolExtension extension{
      {
          {"GET", "/espectre/v1/vendor", "vendor", "vendor", EspectreApiRouteKind::RESOURCE,
           false, false, true, validate},
          {"POST", "/espectre/v1/vendor/actions", "vendor_action", "vendor_action",
           EspectreApiRouteKind::OPERATION, true, true, false, validate},
      }, {"vendor_done"}};
  DirectHttpServiceConfig limits = config();
  limits.protocol_extension = &extension;
  limits.max_mutations_per_minute = 1U;
  EspIdfDirectHttpService service;
  size_t executed = 0U;
  TEST_ASSERT_TRUE(service.setup(limits, [&executed](const DirectRequest &request) {
    ++executed;
    return command_result(request);
  }, {}));

  RawCsiSessionConfig session{};
  session.session_id[0] = 1U;
  TEST_ASSERT_TRUE(service.start_raw_session(session, {}));
  prepare_json("{}");
  httpd_req_t read = request_for_route(HTTP_GET, "/espectre/v1/vendor");
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&read));
  service.loop();
  TEST_ASSERT_EQUAL(1U, executed);

  prepare_json("{}");
  httpd_req_t action = request_for_route(HTTP_POST, "/espectre/v1/vendor/actions");
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&action));
  TEST_ASSERT_EQUAL_STRING("409 Conflict", g_httpd_mock.response_status);
  TEST_ASSERT_EQUAL(1U, executed);
  TEST_ASSERT_TRUE(service.stop_raw_session(RawCsiStopReason::REQUESTED));

  prepare_json("{}");
  action = request_for_route(HTTP_POST, "/espectre/v1/vendor/actions");
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&action));
  service.loop();
  TEST_ASSERT_EQUAL(2U, executed);
  prepare_json("{}");
  action = request_for_route(HTTP_POST, "/espectre/v1/vendor/actions");
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&action));
  TEST_ASSERT_EQUAL_STRING(HTTPD_429_TOO_MANY_REQUESTS, g_httpd_mock.response_status);
  prepare_json("{}");
  read = request_for_route(HTTP_GET, "/espectre/v1/vendor");
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&read));
  service.loop();
  TEST_ASSERT_EQUAL(3U, executed);
}

void test_post_limits_total_request_rate_before_parsing() {
  httpd_mock_reset();
  esp_timer_mock::reset(100000U, 0U);
  EspIdfDirectHttpService service;
  DirectHttpServiceConfig limits = config();
  limits.max_requests_per_second = 2U;
  TEST_ASSERT_TRUE(service.setup(
      limits, [](const DirectRequest &request) { return command_result(request); }, {}));

  prepare_json("{}");
  httpd_req_t first = request_for_route(HTTP_GET, "/espectre/v1/health", 30);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&first));
  service.loop();
  prepare_json("{}");
  httpd_req_t second = request_for_route(HTTP_GET, "/espectre/v1/diagnostics", 31);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&second));
  service.loop();

  prepare_json("not-json");
  httpd_req_t limited = request_for(0U, 32);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&limited));
  TEST_ASSERT_EQUAL_STRING(HTTPD_429_TOO_MANY_REQUESTS, g_httpd_mock.response_status);
  TEST_ASSERT_EQUAL(1U, service.diagnostics().rate_limited_requests);
  TEST_ASSERT_EQUAL(0U, service.diagnostics().malformed_requests);

  esp_timer_mock::advance(1000000U);
  prepare_json("{}");
  httpd_req_t next_window = request_for_route(HTTP_GET, "/espectre/v1/device", 33);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&next_window));
}

void test_sse_limits_clients_frames_events_coalesces_and_heartbeats() {
  httpd_mock_reset();
  esp_timer_mock::reset(100000U, 0U);
  EspIdfDirectHttpService service;
  size_t reported_clients = 0U;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; },
                                 [&reported_clients](size_t count) { reported_clients = count; }));
  httpd_mock_set_header("Origin", "https://espectre.dev");
  httpd_req_t first = request_for(1U, 11);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&first));
  TEST_ASSERT_EQUAL_STRING("text/event-stream; charset=utf-8", g_httpd_mock.response_type);
  TEST_ASSERT_EQUAL(1U, service.event_client_count());
  TEST_ASSERT_EQUAL(0U, reported_clients);
  service.loop();
  TEST_ASSERT_EQUAL(1U, reported_clients);
  TEST_ASSERT_TRUE(sent_payload(0).find("retry: 3000") != std::string::npos);

  TEST_ASSERT_TRUE(service.publish_event("telemetry", "{\"movement\":0.1}", true));
  TEST_ASSERT_TRUE(service.publish_event("telemetry", "{\"movement\":0.9}", true));
  service.loop();
  TEST_ASSERT_EQUAL(2, g_httpd_mock.send_calls);
  TEST_ASSERT_TRUE(sent_payload(1).find("event: telemetry") != std::string::npos);
  TEST_ASSERT_TRUE(sent_payload(1).find("0.9") != std::string::npos);
  TEST_ASSERT_TRUE(sent_payload(1).find("0.1") == std::string::npos);

  esp_timer_mock::advance(10000000U);
  service.loop();
  TEST_ASSERT_EQUAL_STRING(": ping\n\n", sent_payload(2).c_str());

  httpd_req_t second = request_for(1U, 12);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&second));
  httpd_req_t extra = request_for(1U, 13);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&extra));
  TEST_ASSERT_EQUAL_STRING(HTTPD_503_SERVICE_UNAVAILABLE, g_httpd_mock.response_status);
  TEST_ASSERT_EQUAL(1U, service.diagnostics().rejected_connections);
}

void test_sse_queue_preserves_control_events_when_telemetry_fills_capacity() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string("{}"); }, {}));
  prepare_json("");
  httpd_req_t request = request_for(1U);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&request));
  TEST_ASSERT_FALSE(service.publish_event("", "{}", false));
  TEST_ASSERT_FALSE(service.publish_event("health", "[]", false));
  TEST_ASSERT_TRUE(service.publish_event("telemetry", "{\"movement\":0.1}", true));
  TEST_ASSERT_TRUE(service.publish_event("health", "{\"ready\":true}", false));
  TEST_ASSERT_TRUE(service.publish_event("sensing", "{\"enabled\":false}", false));
  TEST_ASSERT_EQUAL(1U, service.diagnostics().dropped_motion_events);
  TEST_ASSERT_FALSE(service.publish_event("telemetry", "{\"movement\":0.2}", true));
  TEST_ASSERT_EQUAL(2U, service.diagnostics().dropped_motion_events);
  TEST_ASSERT_FALSE(service.publish_event("device", "{}", false));
  service.loop();
  service.loop();
  TEST_ASSERT_EQUAL(3, g_httpd_mock.send_calls);
  TEST_ASSERT_TRUE(sent_payload(1).find("event: health") != std::string::npos);
  TEST_ASSERT_TRUE(sent_payload(2).find("event: sensing") != std::string::npos);
  service.shutdown();
}

void test_raw_open_rejection_releases_request_and_permits_retry() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string("{}"); }, {}));
  for (bool has_callback : {false, true}) {
    if (has_callback) {
      service.set_raw_session_requested_callback([](std::string *message) {
        *message = "runtime busy";
        return false;
      });
    }
    prepare_json("");
    httpd_req_t request = request_for(2U);
    TEST_ASSERT_EQUAL(has_callback ? ESP_OK : ESP_FAIL, dispatch_request(&request));
    service.loop();
    TEST_ASSERT_EQUAL_STRING("409 Conflict", g_httpd_mock.response_status);
    TEST_ASSERT_FALSE(service.raw_diagnostics().active);
  }
  TEST_ASSERT_EQUAL(1, g_httpd_mock.async_complete_calls);
  service.shutdown();
}

void test_sse_peer_close_is_not_a_send_failure() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  httpd_mock_set_header("Origin", "https://espectre.dev");
  httpd_req_t request = request_for(1U, 14);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&request));
  TEST_ASSERT_TRUE(service.publish_event("telemetry", "{\"movement\":0.1}", true));

  g_httpd_mock.send_result = ESP_FAIL;
  errno = ECONNRESET;
  service.loop();

  const DirectHttpServiceDiagnostics diagnostics = service.diagnostics();
  TEST_ASSERT_EQUAL(0U, diagnostics.send_failures);
  TEST_ASSERT_EQUAL(0U, service.event_client_count());
  TEST_ASSERT_EQUAL(1, g_httpd_mock.async_complete_calls);
}

void test_sse_retries_backpressure_before_disconnect() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  httpd_mock_set_header("Origin", "https://espectre.dev");
  httpd_req_t request = request_for(1U, 14);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&request));

  g_httpd_mock.send_result = ESP_FAIL;
  for (size_t attempt = 0; attempt < 3U; ++attempt) {
    TEST_ASSERT_TRUE(service.publish_event("telemetry", "{\"movement\":0.1}", true));
    errno = EAGAIN;
    service.loop();
  }

  const DirectHttpServiceDiagnostics diagnostics = service.diagnostics();
  TEST_ASSERT_EQUAL(3U, diagnostics.send_failures);
  TEST_ASSERT_EQUAL(0U, service.event_client_count());
  TEST_ASSERT_EQUAL(1, g_httpd_mock.async_complete_calls);
}

void test_deferred_post_completes_only_once() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  uint64_t token = 0U;
  std::string request_id;
  TEST_ASSERT_TRUE(service.setup_deferred(
      config(),
      [&token, &request_id](uint64_t current, const DirectRequest &request) {
        token = current;
        request_id = request.command_id;
        return IDirectHttpService::DeferredRequestResult{true, {}, {}};
      },
      {}));
  prepare_json("{}");
  httpd_req_t request = request_for_route(HTTP_GET, "/espectre/v1/devices");
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&request));
  service.loop();
  TEST_ASSERT_TRUE(token != 0U);
  TEST_ASSERT_TRUE(request_id.empty());
  TEST_ASSERT_TRUE(service.complete_deferred_response(
      token, command_result(DirectRequest{request_id, "discover_peers", "{}"}, "{\"devices\":[]}")));
  TEST_ASSERT_FALSE(service.complete_deferred_response(
      token, command_result(DirectRequest{request_id, "discover_peers", "{}"}, "{\"devices\":[]}")));
  TEST_ASSERT_EQUAL(0, g_httpd_mock.send_calls);
  TEST_ASSERT_EQUAL(1U, service.diagnostics().queued_messages);
  service.loop();
  TEST_ASSERT_EQUAL(1, g_httpd_mock.send_calls);
  TEST_ASSERT_EQUAL(0U, service.diagnostics().queued_messages);
  TEST_ASSERT_EQUAL(1, g_httpd_mock.async_complete_calls);
}

void test_response_completion_runs_after_send_and_reports_delivery(void) {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  bool callback_called = false;
  bool response_sent = false;
  int sends_observed_by_callback = 0;
  TEST_ASSERT_TRUE(service.setup_deferred(
      config(),
      [&callback_called, &response_sent, &sends_observed_by_callback](
          uint64_t, const DirectRequest &request) {
        return IDirectHttpService::DeferredRequestResult{
            false,
            command_result(request),
            [&callback_called, &response_sent, &sends_observed_by_callback](bool sent) {
              callback_called = true;
              response_sent = sent;
              sends_observed_by_callback = g_httpd_mock.send_calls;
            },
        };
      },
      {}));

  prepare_json("{\"bssid\":\"AA:BB:CC:DD:EE:FF\"}");
  httpd_req_t request = request_for_route(HTTP_PUT, "/espectre/v1/wifi/bssid");
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&request));
  TEST_ASSERT_FALSE(callback_called);
  service.loop();
  TEST_ASSERT_TRUE(callback_called);
  TEST_ASSERT_TRUE(response_sent);
  TEST_ASSERT_EQUAL(1, sends_observed_by_callback);

  callback_called = false;
  response_sent = true;
  g_httpd_mock.send_result = ESP_FAIL;
  prepare_json("{}");
  request = request_for_route(HTTP_DELETE, "/espectre/v1/wifi/bssid");
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&request));
  service.loop();
  TEST_ASSERT_TRUE(callback_called);
  TEST_ASSERT_FALSE(response_sent);
}

void assert_raw_get_opens_automatic_session_and_emits_v2_frame(const char *origin) {
  httpd_mock_reset();
  esp_timer_mock::reset(100000U, 0U);
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  RawCsiSessionConfig session{};
  session.device_id = 0x112233445566ULL;
  session.chip = RawCsiChipType::C3;
  for (size_t index = 0U; index < sizeof(session.session_id); ++index) {
    session.session_id[index] = static_cast<uint8_t>(index);
  }
  accept_raw_open(&service, session);

  httpd_mock_set_header("Origin", origin);
  httpd_req_t raw_request = request_for(2U, 9);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&raw_request));
  service.loop();
  TEST_ASSERT_TRUE(service.raw_diagnostics().binary_bound);
  TEST_ASSERT_EQUAL_STRING("application/octet-stream", g_httpd_mock.response_type);

  const int8_t csi[] = {1, -2, 3, -4};
  RawCsiPacketView packet{};
  packet.csi = csi;
  packet.csi_len = sizeof(csi);
  packet.captured_at_us = 100000U;
  packet.record_flags = RAW_CSI_FLAG_FRESH;
  packet.channel = 6U;
  packet.rssi_dbm = -45;
  packet.noise_floor_dbm = -96;
  packet.phy_mode = RawCsiPhyMode::HT;
  packet.ltf_type = RawCsiLtfType::HT_LTF;
  packet.channel_width = RawCsiChannelWidth::MHZ_20;
  TEST_ASSERT_TRUE(service.offer_raw_packet(packet));
  service.loop();
  TEST_ASSERT_EQUAL(1, g_httpd_mock.send_calls);
  TEST_ASSERT_EQUAL_STRING(origin, g_httpd_mock.allow_origin);
  TEST_ASSERT_EQUAL(9, g_httpd_mock.sent_fds[0]);
  TEST_ASSERT_EQUAL(sizeof(RawCsiHttpFramePrefix) + sizeof(RawCsiRecordHeaderV8) + sizeof(csi),
                    g_httpd_mock.sent_lengths[0]);
  const auto *prefix = reinterpret_cast<const RawCsiHttpFramePrefix *>(
      g_httpd_mock.sent_payloads[0]);
  TEST_ASSERT_EQUAL(ESPECTRE_RAW_CSI_RESPONSE_MAGIC, prefix->magic);
  TEST_ASSERT_EQUAL(ESPECTRE_RAW_CSI_PROTOCOL_VERSION, prefix->version);
  TEST_ASSERT_EQUAL(RAW_CSI_RECORD_VERSION_V8, prefix->record_version);
  TEST_ASSERT_EQUAL(0U, prefix->flags);
  TEST_ASSERT_EQUAL(1U, prefix->stream_sequence);
  TEST_ASSERT_EQUAL(sizeof(RawCsiRecordHeaderV8) + sizeof(csi), prefix->record_len);
  const auto *header = reinterpret_cast<const RawCsiRecordHeaderV8 *>(
      g_httpd_mock.sent_payloads[0] + sizeof(RawCsiHttpFramePrefix));
  TEST_ASSERT_EQUAL(RAW_CSI_RECORD_VERSION_V8, header->version);
  TEST_ASSERT_EQUAL(100000U, header->device_ticks_us);
  TEST_ASSERT_EQUAL(1U, header->fresh_record_total);
  TEST_ASSERT_TRUE(service.stop_raw_session(RawCsiStopReason::REQUESTED));
  TEST_ASSERT_FALSE(service.raw_diagnostics().active);
}

void test_raw_get_opens_automatic_session_and_emits_v2_frame() {
  assert_raw_get_opens_automatic_session_and_emits_v2_frame("https://test.espectre.dev");
}

void test_raw_loopback_origin_survives_until_first_packet() {
  assert_raw_get_opens_automatic_session_and_emits_v2_frame("http://localhost:5173");
}

void test_raw_loopback_origin_is_sent_when_closed_before_first_packet() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  RawCsiSessionConfig session{};
  session.session_id[0] = 1U;
  accept_raw_open(&service, session);
  httpd_mock_set_header("Origin", "http://localhost:5173");
  httpd_req_t raw_request = request_for(2U, 9);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&raw_request));
  service.loop();
  TEST_ASSERT_EQUAL(0, g_httpd_mock.send_calls);
  TEST_ASSERT_TRUE(service.stop_raw_session(RawCsiStopReason::REQUESTED));
  TEST_ASSERT_EQUAL(1, g_httpd_mock.send_calls);
  TEST_ASSERT_EQUAL_STRING("http://localhost:5173", g_httpd_mock.allow_origin);
  TEST_ASSERT_EQUAL(0U, g_httpd_mock.sent_lengths[0]);
}

void test_raw_batches_up_to_four_records_without_pacing() {
  httpd_mock_reset();
  esp_timer_mock::reset(100000U, 0U);
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  RawCsiSessionConfig session{};
  session.device_id = 0x112233445566ULL;
  session.chip = RawCsiChipType::C3;
  session.session_id[0] = 1U;
  accept_raw_open(&service, session);

  httpd_mock_set_header("Origin", "https://espectre.dev");
  httpd_req_t raw_request = request_for(2U, 9);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&raw_request));
  service.loop();

  const int8_t csi[] = {1, -2, 3, -4};
  RawCsiPacketView packet{};
  packet.csi = csi;
  packet.csi_len = sizeof(csi);
  packet.captured_at_us = 100000U;
  for (uint64_t index = 0U; index < 4U; ++index) {
    packet.captured_at_us = 100000U + index;
    TEST_ASSERT_TRUE(service.offer_raw_packet(packet));
  }
  service.loop();
  TEST_ASSERT_EQUAL(1, g_httpd_mock.send_calls);
  const size_t frame_size = sizeof(RawCsiHttpFramePrefix) + sizeof(RawCsiRecordHeaderV8) + sizeof(csi);
  TEST_ASSERT_EQUAL(4U * frame_size, g_httpd_mock.sent_lengths[0]);
}

void test_second_raw_get_is_rejected_while_the_first_open_is_pending() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  RawCsiSessionConfig session{};
  session.session_id[0] = 7U;
  accept_raw_open(&service, session);

  httpd_mock_set_header("Origin", "https://espectre.dev");
  httpd_req_t first = request_for(2U, 9);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&first));
  httpd_req_t second = request_for(2U, 10);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&second));
  TEST_ASSERT_EQUAL_STRING("409 Conflict", g_httpd_mock.response_status);
  prepare_json("{\"enabled\":false}");
  httpd_req_t mutation = request_for_route(HTTP_PATCH, "/espectre/v1/sensing", 11);
  TEST_ASSERT_EQUAL(ESP_FAIL, dispatch_request(&mutation));
  TEST_ASSERT_EQUAL_STRING("409 Conflict", g_httpd_mock.response_status);
  TEST_ASSERT_FALSE(service.raw_diagnostics().active);
  TEST_ASSERT_FALSE(service.raw_diagnostics().binary_bound);
  service.loop();
  TEST_ASSERT_TRUE(service.raw_diagnostics().active);
  TEST_ASSERT_TRUE(service.raw_diagnostics().binary_bound);
}

void test_raw_ring_drops_new_record_and_accounts_every_offer() {
  httpd_mock_reset();
  esp_timer_mock::reset(100000U, 0U);
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  RawCsiSessionConfig session{};
  session.session_id[0] = 2U;
  accept_raw_open(&service, session);

  httpd_mock_set_header("Origin", "https://espectre.dev");
  httpd_req_t raw_request = request_for(2U, 9);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&raw_request));
  service.loop();

  const int8_t csi[] = {1, -2, 3, -4};
  RawCsiPacketView packet{};
  packet.csi = csi;
  packet.csi_len = sizeof(csi);
  for (size_t index = 0U; index < 16U; ++index) {
    packet.captured_at_us = 100000U + index;
    TEST_ASSERT_TRUE(service.offer_raw_packet(packet));
  }
  TEST_ASSERT_FALSE(service.offer_raw_packet(packet));

  for (size_t batch = 0U; batch < 4U; ++batch) {
    service.loop();
  }
  packet.captured_at_us = 200000U;
  TEST_ASSERT_TRUE(service.offer_raw_packet(packet));
  service.loop();
  const RawCsiSessionDiagnostics diagnostics = service.raw_diagnostics();
  TEST_ASSERT_EQUAL(17U, diagnostics.fresh_record_total);
  TEST_ASSERT_EQUAL(1U, diagnostics.raw_drop_total);
  TEST_ASSERT_EQUAL(diagnostics.stream_sequence,
                    diagnostics.fresh_record_total + diagnostics.raw_drop_total);
  const auto *prefix = reinterpret_cast<const RawCsiHttpFramePrefix *>(
      g_httpd_mock.sent_payloads[4]);
  TEST_ASSERT_EQUAL(18U, prefix->stream_sequence);
}

void test_raw_session_has_no_bind_timeout() {
  httpd_mock_reset();
  esp_timer_mock::reset(100000U, 0U);
  EspIdfDirectHttpService service;
  RawCsiStopReason stopped_reason = RawCsiStopReason::INTERNAL_ERROR;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  RawCsiSessionConfig session{};
  session.session_id[0] = 4U;
  TEST_ASSERT_TRUE(service.start_raw_session(
      session, [&stopped_reason](RawCsiStopReason reason) { stopped_reason = reason; }));

  esp_timer_mock::advance(5000000U);
  service.loop();
  TEST_ASSERT_TRUE(service.raw_diagnostics().active);
  TEST_ASSERT_EQUAL(static_cast<uint8_t>(RawCsiStopReason::INTERNAL_ERROR),
                    static_cast<uint8_t>(stopped_reason));
}

void test_raw_send_failure_accounts_batch_and_stops_slow_client() {
  httpd_mock_reset();
  esp_timer_mock::reset(100000U, 0U);
  EspIdfDirectHttpService service;
  RawCsiStopReason stopped_reason = RawCsiStopReason::INTERNAL_ERROR;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  RawCsiSessionConfig session{};
  session.session_id[0] = 3U;
  accept_raw_open(&service, session,
                  [&stopped_reason](RawCsiStopReason reason) { stopped_reason = reason; });
  httpd_mock_set_header("Origin", "https://espectre.dev");
  httpd_req_t raw_request = request_for(2U, 9);
  TEST_ASSERT_EQUAL(ESP_OK, dispatch_request(&raw_request));
  service.loop();

  const int8_t csi[] = {1, -2, 3, -4};
  RawCsiPacketView packet{};
  packet.csi = csi;
  packet.csi_len = sizeof(csi);
  TEST_ASSERT_TRUE(service.offer_raw_packet(packet));
  g_httpd_mock.send_result = ESP_FAIL;
  service.loop();
  const RawCsiSessionDiagnostics diagnostics = service.raw_diagnostics();
  TEST_ASSERT_FALSE(diagnostics.active);
  TEST_ASSERT_EQUAL(0U, diagnostics.fresh_record_total);
  TEST_ASSERT_EQUAL(1U, diagnostics.raw_drop_total);
  TEST_ASSERT_EQUAL(1U, diagnostics.raw_send_backpressure_total);
  TEST_ASSERT_EQUAL(diagnostics.stream_sequence,
                    diagnostics.fresh_record_total + diagnostics.raw_drop_total);
  TEST_ASSERT_EQUAL(static_cast<uint8_t>(RawCsiStopReason::SLOW_CLIENT),
                    static_cast<uint8_t>(stopped_reason));
}

void test_raw_stop_accounts_records_accepted_but_not_sent() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  RawCsiSessionConfig session{};
  session.session_id[0] = 5U;
  TEST_ASSERT_TRUE(service.start_raw_session(session, {}));
  const int8_t csi[] = {1, -2, 3, -4};
  RawCsiPacketView packet{};
  packet.csi = csi;
  packet.csi_len = sizeof(csi);
  TEST_ASSERT_TRUE(service.offer_raw_packet(packet));
  TEST_ASSERT_TRUE(service.offer_raw_packet(packet));
  TEST_ASSERT_TRUE(service.offer_raw_packet(packet));
  TEST_ASSERT_TRUE(service.stop_raw_session(RawCsiStopReason::REQUESTED));
  const RawCsiSessionDiagnostics diagnostics = service.raw_diagnostics();
  TEST_ASSERT_EQUAL(0U, diagnostics.fresh_record_total);
  TEST_ASSERT_EQUAL(3U, diagnostics.raw_drop_total);
  TEST_ASSERT_EQUAL(3U, diagnostics.stream_sequence);
  TEST_ASSERT_EQUAL(diagnostics.stream_sequence,
                    diagnostics.fresh_record_total + diagnostics.raw_drop_total);
}

void test_raw_assigns_sequence_before_rejecting_an_invalid_offer() {
  httpd_mock_reset();
  EspIdfDirectHttpService service;
  TEST_ASSERT_TRUE(service.setup(config(), [](const auto &) { return std::string{"{}"}; }, {}));
  RawCsiSessionConfig session{};
  session.session_id[0] = 6U;
  TEST_ASSERT_TRUE(service.start_raw_session(session, {}));

  RawCsiPacketView invalid{};
  TEST_ASSERT_FALSE(service.offer_raw_packet(invalid));
  const RawCsiSessionDiagnostics diagnostics = service.raw_diagnostics();
  TEST_ASSERT_EQUAL(1U, diagnostics.stream_sequence);
  TEST_ASSERT_EQUAL(0U, diagnostics.fresh_record_total);
  TEST_ASSERT_EQUAL(1U, diagnostics.raw_drop_total);
  TEST_ASSERT_EQUAL(diagnostics.stream_sequence,
                    diagnostics.fresh_record_total + diagnostics.raw_drop_total);
}

}  // namespace

int main() {
  espectre::test::begin_suite();
  RUN_TEST(test_idle_loop_does_not_allocate);
  RUN_TEST(test_raw_storage_failure_and_repeated_sessions_preserve_capacity);
  RUN_TEST(test_setup_registers_http_post_sse_raw_and_preflight);
  RUN_TEST(test_setup_failure_releases_server_and_allows_retry);
  RUN_TEST(test_empty_and_oversized_application_responses_return_bounded_command_errors);
  RUN_TEST(test_incomplete_request_never_reaches_application_handler);
  RUN_TEST(test_sse_initial_send_failure_releases_the_client_slot);
  RUN_TEST(test_destructor_does_not_dispatch_application_callbacks);
  RUN_TEST(test_shutdown_releases_the_client_count_callback_before_reuse);
  RUN_TEST(test_post_does_not_enqueue_after_shutdown_starts);
  RUN_TEST(test_sse_does_not_register_after_shutdown_starts);
  RUN_TEST(test_post_validates_origin_content_type_size_and_dispatches_on_loop);
  RUN_TEST(test_options_returns_private_network_cors_headers);
  RUN_TEST(test_post_distinguishes_queue_saturation_from_mutation_rate_limit);
  RUN_TEST(test_post_limits_total_request_rate_before_parsing);
  RUN_TEST(test_frontend_routes_obey_read_mutation_and_raw_collection_policies);
  RUN_TEST(test_sse_limits_clients_frames_events_coalesces_and_heartbeats);
  RUN_TEST(test_sse_peer_close_is_not_a_send_failure);
  RUN_TEST(test_sse_queue_preserves_control_events_when_telemetry_fills_capacity);
  RUN_TEST(test_raw_open_rejection_releases_request_and_permits_retry);
  RUN_TEST(test_sse_retries_backpressure_before_disconnect);
  RUN_TEST(test_deferred_post_completes_only_once);
  RUN_TEST(test_response_completion_runs_after_send_and_reports_delivery);
  RUN_TEST(test_raw_get_opens_automatic_session_and_emits_v2_frame);
  RUN_TEST(test_raw_loopback_origin_survives_until_first_packet);
  RUN_TEST(test_raw_loopback_origin_is_sent_when_closed_before_first_packet);
  RUN_TEST(test_raw_batches_up_to_four_records_without_pacing);
  RUN_TEST(test_second_raw_get_is_rejected_while_the_first_open_is_pending);
  RUN_TEST(test_raw_ring_drops_new_record_and_accounts_every_offer);
  RUN_TEST(test_raw_session_has_no_bind_timeout);
  RUN_TEST(test_raw_send_failure_accounts_batch_and_stops_slow_client);
  RUN_TEST(test_raw_stop_accounts_records_accepted_but_not_sent);
  RUN_TEST(test_raw_assigns_sequence_before_rejecting_an_invalid_offer);
  return espectre::test::end_suite();
}
