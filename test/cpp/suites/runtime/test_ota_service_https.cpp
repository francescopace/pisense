/*
 * ESPectre - HTTPS OTA Service Tests
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include "test_harness.h"

#include <atomic>
#include <string>

#define private public
#include "frontend/ota_service_https.h"
#undef private

#include "esp_http_client.h"
#include "esp_https_ota.h"

using namespace espectre;

class LegacyOtaService : public IOtaService {
 public:
  void loop() override {}
  void shutdown() override {}
  bool start_check(const std::string &) override {
    check_calls++;
    return true;
  }
  bool start_update(const std::string &) override {
    update_calls++;
    return true;
  }
  EspectreOtaStatus status() const override { return {}; }
  void set_status_callback(StatusCallback) override {}
  void set_prepare_for_update_callback(PrepareForUpdateCallback) override {}

  int check_calls{0};
  int update_calls{0};
};

void test_legacy_ota_service_rejects_channels_it_cannot_honor(void) {
  LegacyOtaService service;
  IOtaService &api = service;

  TEST_ASSERT_TRUE(api.start_check("3.0.0", ""));
  TEST_ASSERT_TRUE(api.start_update("3.0.0", ""));
  TEST_ASSERT_FALSE(api.start_check("3.0.0", ESPECTRE_OTA_CHANNEL_PREVIEW));
  TEST_ASSERT_FALSE(api.start_update("3.0.0", "invalid"));
  TEST_ASSERT_EQUAL(1, service.check_calls);
  TEST_ASSERT_EQUAL(1, service.update_calls);
}

void setUp(void) {
  esp_http_client_mock_reset();
  g_esp_https_ota_calls = 0;
  g_esp_https_ota_result = ESP_OK;
}
void tearDown(void) {}

namespace {

std::string firmware_catalog(const std::string &artifacts, const char *channel = "develop",
                             const char *version = "3.1.0") {
  return std::string(R"({"schema_version":1,"channel":")") + channel +
      R"(","version":")" + version + R"(","frontends":{
        "esphome":{"artifacts":[{"chip":"esp32s2","build_type":"ota","url":"https://example.invalid/esphome.bin"}]},
        "native":{"artifacts":)" + artifacts + "}}}";
}

const char *const kOtaArtifact =
    R"({"chip":"esp32s2","build_type":"ota","url":"https://example.invalid/fw.bin"})";

}  // namespace

void test_https_ota_manifest_parser_selects_frontend_chip_and_ota_image(void) {
  HttpsOtaService service("native", "esp32s2", OtaReleaseChannel::DEVELOP);
  HttpsOtaService::ManifestInfo manifest;
  std::string error;
  const std::string artifacts = std::string(R"([
      {"chip":"esp32s2","build_type":"factory","url":"https://example.invalid/factory.bin"},
      {"chip":"esp32c3","build_type":"ota","url":"https://example.invalid/c3.bin"},
      {"chip":"esp32s2","build_type":"ota",
       "compliance":[{"url":"https://example.invalid/license.zip"}],
       "url":"https://example.invalid/fw.bin"}])");

  for (const char *channel : {"release", "preview", "develop"}) {
    TEST_ASSERT_TRUE(service.parse_manifest_(firmware_catalog(artifacts, channel), channel, &manifest, &error));
    TEST_ASSERT_EQUAL_STRING("3.1.0", manifest.version.c_str());
    TEST_ASSERT_EQUAL_STRING("https://example.invalid/fw.bin", manifest.image_url.c_str());
  }
}

void test_https_ota_manifest_parser_rejects_missing_or_ambiguous_targets(void) {
  HttpsOtaService service("native", "esp32s2", OtaReleaseChannel::DEVELOP);
  HttpsOtaService::ManifestInfo manifest;
  std::string error;
  for (const std::string &artifacts : {
      std::string("[]"),
      std::string(R"([{"chip":"esp32c3","build_type":"ota","url":"https://example.invalid/c3.bin"}])"),
      std::string(R"([{"chip":"esp32s2","build_type":"factory","url":"https://example.invalid/factory.bin"}])"),
      std::string("[") + kOtaArtifact + "," + kOtaArtifact + "]",
      std::string(R"([{"chip":"esp32s2","build_type":"ota","url":"http://example.invalid/fw.bin"}])"),
      std::string(R"([{"chip":"esp32s2","build_type":"ota"}])"),
      std::string(R"([{"chip":"esp32s2","build_type":"ota","url":42}])"),
      std::string(R"([{"chip":"esp32s2","chip":"esp32s2","build_type":"ota","url":"https://example.invalid/fw.bin"}])"),
      std::string("[null]"), std::string("[{} ,]"), std::string("{}")}) {
    TEST_ASSERT_FALSE(service.parse_manifest_(firmware_catalog(artifacts), "develop", &manifest, &error));
    TEST_ASSERT_FALSE(error.empty());
    TEST_ASSERT_TRUE(manifest.image_url.empty());
  }
  HttpsOtaService absent_frontend("matter", "esp32s2", OtaReleaseChannel::DEVELOP);
  TEST_ASSERT_FALSE(absent_frontend.parse_manifest_(firmware_catalog("[]"), "develop", &manifest, &error));
}

void test_https_ota_manifest_parser_validates_catalog_metadata(void) {
  HttpsOtaService service("native", "esp32s2", OtaReleaseChannel::DEVELOP);
  HttpsOtaService::ManifestInfo manifest;
  std::string error;
  const std::string valid = firmware_catalog(std::string("[") + kOtaArtifact + "]");
  TEST_ASSERT_FALSE(service.parse_manifest_(valid, "preview", &manifest, &error));
  TEST_ASSERT_FALSE(service.parse_manifest_(firmware_catalog("[]", "develop", ""), "develop", &manifest, &error));
  for (const char *schema : {"2", "null", "\"1\""}) {
    std::string invalid = valid;
    invalid.replace(invalid.find("\"schema_version\":1"), 18, std::string("\"schema_version\":") + schema);
    TEST_ASSERT_FALSE(service.parse_manifest_(invalid, "develop", &manifest, &error));
  }
  TEST_ASSERT_FALSE(service.parse_manifest_("{}", "develop", &manifest, &error));
  TEST_ASSERT_FALSE(service.parse_manifest_(valid + "garbage", "develop", &manifest, &error));
  TEST_ASSERT_FALSE(service.parse_manifest_(valid, "develop", nullptr, &error));
}

void test_https_ota_fetch_enforces_status_and_manifest_size(void) {
  HttpsOtaService service("native", "esp32", OtaReleaseChannel::RELEASE);
  std::string body;
  std::string error;

  g_esp_http_client_mock.response_body = "ok";
  TEST_ASSERT_TRUE(service.fetch_https_text_("https://example.invalid/manifest.json", &body, &error));
  TEST_ASSERT_EQUAL_STRING("ok", body.c_str());
  TEST_ASSERT_EQUAL(30000, g_esp_http_client_mock.last_config.timeout_ms);
  TEST_ASSERT_EQUAL(8192, g_esp_http_client_mock.last_config.buffer_size);
  TEST_ASSERT_EQUAL(1024, g_esp_http_client_mock.last_config.buffer_size_tx);

  esp_http_client_mock_reset();
  g_esp_http_client_mock.response_body = firmware_catalog(std::string("[") + kOtaArtifact + "]");
  g_esp_http_client_mock.response_body.insert(1, "\"notes\":\"" + std::string(40000, 'x') + "\",");
  TEST_ASSERT_TRUE(service.fetch_https_text_("https://example.invalid/manifest.json", &body, &error));
  HttpsOtaService target("native", "esp32s2", OtaReleaseChannel::DEVELOP);
  HttpsOtaService::ManifestInfo manifest;
  TEST_ASSERT_TRUE(target.parse_manifest_(body, "develop", &manifest, &error));
  TEST_ASSERT_EQUAL_STRING("https://example.invalid/fw.bin", manifest.image_url.c_str());

  esp_http_client_mock_reset();
  g_esp_http_client_mock.status_code = 503;
  TEST_ASSERT_FALSE(service.fetch_https_text_("https://example.invalid/manifest.json", &body, &error));
  TEST_ASSERT_TRUE(error.find("503") != std::string::npos);

  esp_http_client_mock_reset();
  g_esp_http_client_mock.response_body.assign(64U * 1024U + 1U, 'x');
  TEST_ASSERT_FALSE(service.fetch_https_text_("https://example.invalid/manifest.json", &body, &error));
  TEST_ASSERT_EQUAL_STRING("manifest too large", error.c_str());
  TEST_ASSERT_FALSE(service.fetch_https_text_("", &body, &error));
  TEST_ASSERT_FALSE(service.fetch_https_text_("https://example.invalid", nullptr, &error));
}

void test_https_ota_check_updates_status_and_delivers_callback(void) {
  HttpsOtaService service("native", "esp32", OtaReleaseChannel::PREVIEW);
  g_esp_http_client_mock.response_body =
      firmware_catalog(R"([{"chip":"esp32","build_type":"ota","url":"https://example.invalid/fw.bin"}])",
                       "preview", "99.0.0");
  int callback_count = 0;
  EspectreOtaStatus delivered;
  service.set_status_callback([&](const EspectreOtaStatus& status) {
    callback_count++;
    delivered = status;
  });

  TEST_ASSERT_TRUE(service.start_check("3.0.0"));
  service.loop();

  TEST_ASSERT_EQUAL(1, callback_count);
  TEST_ASSERT_TRUE(delivered.state == EspectreOtaState::UPDATE_AVAILABLE);
  TEST_ASSERT_TRUE(delivered.update_available);
  TEST_ASSERT_EQUAL_STRING("preview", delivered.default_channel.c_str());
  TEST_ASSERT_FALSE(service.start_check("3.0.0", "invalid"));
  service.shutdown();
  TEST_ASSERT_FALSE(service.start_check("3.0.0"));
}

void test_https_ota_update_applies_newer_image_and_delivers_completion_once(void) {
  HttpsOtaService service("native", "esp32s2", OtaReleaseChannel::DEVELOP);
  g_esp_http_client_mock.response_body = firmware_catalog(std::string("[") + kOtaArtifact + "]");
  int preparations = 0;
  int completions = 0;
  service.set_prepare_for_update_callback([&]() { preparations++; });
  service.set_status_callback([&](const EspectreOtaStatus &status) {
    completions++;
    TEST_ASSERT_TRUE(status.state == EspectreOtaState::REBOOT_SCHEDULED);
    TEST_ASSERT_FALSE(status.busy);
  });

  TEST_ASSERT_TRUE(service.start_update("3.0.0"));
  TEST_ASSERT_EQUAL(1, g_esp_https_ota_calls);
  TEST_ASSERT_EQUAL(1, g_esp_http_client_mock.cleanup_calls);
  const auto status = service.status();
  TEST_ASSERT_TRUE(status.state == EspectreOtaState::REBOOT_SCHEDULED);
  TEST_ASSERT_EQUAL_STRING("3.1.0", status.target_version.c_str());
  TEST_ASSERT_EQUAL_STRING("https://example.invalid/fw.bin", status.image_url.c_str());
  service.loop();
  service.loop();
  TEST_ASSERT_EQUAL(1, preparations);
  TEST_ASSERT_EQUAL(1, completions);
  service.shutdown();
  service.loop();
  TEST_ASSERT_FALSE(service.start_update("3.0.0"));
  TEST_ASSERT_TRUE(service.status().state == EspectreOtaState::REBOOT_SCHEDULED);
}

void test_https_ota_update_never_installs_same_older_or_unordered_versions(void) {
  for (const char *current : {"3.1.0", "4.0.0", "3.0.0"}) {
    HttpsOtaService service("native", "esp32s2", OtaReleaseChannel::DEVELOP);
    const bool unordered = std::string(current) == "3.0.0";
    g_esp_http_client_mock.response_body = firmware_catalog(
        std::string("[") + kOtaArtifact + "]", "develop", unordered ? "snapshot" : "3.1.0");
    TEST_ASSERT_TRUE(service.start_update(current, "develop"));
    const auto status = service.status();
    const auto expected = unordered ? EspectreOtaState::ERROR : EspectreOtaState::UP_TO_DATE;
    TEST_ASSERT_TRUE(status.state == expected);
    TEST_ASSERT_FALSE(status.busy);
    TEST_ASSERT_FALSE(status.update_available);
    TEST_ASSERT_EQUAL(0, g_esp_https_ota_calls);
  }
}

void test_https_ota_failure_retains_target_and_allows_retry(void) {
  HttpsOtaService service("native", "esp32s2", OtaReleaseChannel::DEVELOP);
  g_esp_http_client_mock.response_body = firmware_catalog(std::string("[") + kOtaArtifact + "]");
  g_esp_https_ota_result = ESP_FAIL;
  TEST_ASSERT_TRUE(service.start_update("3.0.0"));
  const auto failure = service.status();
  TEST_ASSERT_TRUE(failure.state == EspectreOtaState::ERROR);
  TEST_ASSERT_FALSE(failure.busy);
  TEST_ASSERT_EQUAL_STRING("3.1.0", failure.target_version.c_str());
  TEST_ASSERT_EQUAL_STRING("https://example.invalid/fw.bin", failure.image_url.c_str());
  g_esp_https_ota_result = ESP_OK;
  TEST_ASSERT_TRUE(service.start_update("3.0.0"));
  TEST_ASSERT_TRUE(service.status().state == EspectreOtaState::REBOOT_SCHEDULED);
  TEST_ASSERT_EQUAL(2, g_esp_https_ota_calls);
}

void test_https_ota_check_rejects_unordered_version_and_bad_manifest(void) {
  HttpsOtaService service("native", "esp32s2", OtaReleaseChannel::DEVELOP);
  g_esp_http_client_mock.response_body = firmware_catalog(
      std::string("[") + kOtaArtifact + "]", "develop", "snapshot");
  TEST_ASSERT_TRUE(service.start_check("3.0.0"));
  TEST_ASSERT_TRUE(service.status().state == EspectreOtaState::ERROR);
  g_esp_http_client_mock.response_body = "invalid";
  TEST_ASSERT_TRUE(service.start_check("3.0.0"));
  TEST_ASSERT_TRUE(service.status().state == EspectreOtaState::ERROR);
  TEST_ASSERT_FALSE(service.status().busy);
  TEST_ASSERT_EQUAL(0, g_esp_https_ota_calls);
}

void test_https_ota_fetch_releases_client_after_transport_error(void) {
  HttpsOtaService service("native", "esp32s2", OtaReleaseChannel::DEVELOP);
  std::string body = "stale";
  std::string error;
  g_esp_http_client_mock.init_succeeds = false;
  TEST_ASSERT_FALSE(service.fetch_https_text_("https://example.invalid/manifest.json", &body, &error));
  TEST_ASSERT_TRUE(body.empty());
  TEST_ASSERT_FALSE(error.empty());
  TEST_ASSERT_EQUAL(0, g_esp_http_client_mock.cleanup_calls);
  g_esp_http_client_mock.init_succeeds = true;
  g_esp_http_client_mock.perform_result = ESP_FAIL;
  TEST_ASSERT_FALSE(service.fetch_https_text_("https://example.invalid/manifest.json", &body, &error));
  TEST_ASSERT_EQUAL(1, g_esp_http_client_mock.cleanup_calls);
}

int process(void) {
  UNITY_BEGIN();
  RUN_TEST(test_legacy_ota_service_rejects_channels_it_cannot_honor);
  RUN_TEST(test_https_ota_manifest_parser_selects_frontend_chip_and_ota_image);
  RUN_TEST(test_https_ota_manifest_parser_rejects_missing_or_ambiguous_targets);
  RUN_TEST(test_https_ota_manifest_parser_validates_catalog_metadata);
  RUN_TEST(test_https_ota_fetch_enforces_status_and_manifest_size);
  RUN_TEST(test_https_ota_check_updates_status_and_delivers_callback);
  RUN_TEST(test_https_ota_update_applies_newer_image_and_delivers_completion_once);
  RUN_TEST(test_https_ota_update_never_installs_same_older_or_unordered_versions);
  RUN_TEST(test_https_ota_failure_retains_target_and_allows_retry);
  RUN_TEST(test_https_ota_check_rejects_unordered_version_and_bad_manifest);
  RUN_TEST(test_https_ota_fetch_releases_client_after_transport_error);
  return UNITY_END();
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  return process();
}
