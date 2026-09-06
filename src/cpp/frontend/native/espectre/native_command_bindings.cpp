/*
 * ESPectre - Native Command Bindings
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

#include "native_command_bindings.h"

#include <utility>

#include "frontend_mqtt_helpers.h"
#include "home_assistant_mqtt_frontend.h"
#include "native_direct_frontend.h"
#include "native_frontend.h"
#include "native_mqtt_frontend.h"
#include "protocol_json.h"
#include "runtime_config_utils.h"

namespace espectre {

FrontendCommandResult NativeCommandBindings::execute(const EspectreCommand &command, FrontendCommandOrigin origin,
                                                     bool allow_local_config, uint64_t connection_token) {
  if (owner_.runtime_.operation_state() == RuntimeOperationState::RAW_COLLECTION &&
      !frontend_command_allowed_during_raw_collection(command.command, capability_profile(allow_local_config).extension)) {
    FrontendCommandResult busy;
    busy.handled = true;
    busy.command = command;
    busy.code = "busy_raw_collection";
    busy.message = "mutation is unavailable during raw CSI collection";
    return busy;
  }
  const FrontendCommandCapabilities capabilities = capability_profile(allow_local_config);
  if (find_extension_route(capabilities.extension, command.command) != nullptr) {
    return execute_frontend_ota_command(command, origin, owner_.ota_service_, owner_.device_info_.firmware_version,
        [this](const EspectreCommand &) { return owner_.direct_frontend_->ota_payload(); });
  }
  FrontendCommandResult result = engine_.execute(
      command, FrontendCommandContext{origin, connection_token}, capabilities,
      [this, allow_local_config, capabilities](const EspectreCommand &read) {
        if (read.command == "capabilities") {
          return espectre_capabilities_payload(this->owner_.device_config_, this->owner_.mqtt_protocol_device_info_(),
                                               capabilities);
        }
        if (read.command == "device") {
          return this->owner_.direct_frontend_->device_payload();
        }
        if (read.command == "health") {
          return this->owner_.direct_frontend_->health_payload(!this->owner_.device_info_.network.ip_address.empty());
        }
        if (read.command == "sensing") {
          return this->owner_.direct_frontend_->sensing_payload();
        }
        if (read.command == "wifi" && allow_local_config) {
          return this->owner_.direct_frontend_->wifi_payload();
        }
        if (read.command == "mqtt" && allow_local_config) {
          return this->owner_.direct_frontend_->mqtt_payload();
        }
        if (read.command == "wifi_access_points" && allow_local_config) {
          return this->owner_.direct_frontend_->wifi_access_points_payload();
        }
        if (read.command == "read_diagnostics") {
          return std::string{"{}"};
        }
        return std::string{};
      },
      [this](const std::string &device_label, std::string *message) {
        EspectreDeviceConfig updated_config = this->owner_.device_config_;
        updated_config.device_label = device_label;
        if (this->owner_.device_config_change_callback_ &&
            !this->owner_.device_config_change_callback_(updated_config, false, message)) {
          return false;
        }
        this->owner_.set_device_config(updated_config);
        this->owner_.mqtt_frontend_->home_assistant().schedule_discovery();
        this->owner_.mqtt_frontend_->home_assistant().publish_current_state();
        if (message != nullptr && message->empty()) {
          *message = "device label updated";
        }
        return true;
      },
      [this](float threshold, std::string *message) {
        const bool accepted = this->owner_.handle_threshold_write_(threshold);
        if (message != nullptr && message->empty()) {
          *message = accepted ? "threshold updated" : "threshold rejected";
        }
        return accepted;
      },
      [this](uint8_t motion_on_hits, uint8_t motion_off_hits, std::string *message) {
        const bool accepted = this->owner_.handle_motion_hits_write_(motion_on_hits, motion_off_hits);
        if (message != nullptr && message->empty()) {
          *message = accepted ? "motion hits updated" : "motion hits rejected";
        }
        return accepted;
      },
      [this](CsiTrafficMode mode, std::string *message) {
        const bool accepted = this->owner_.handle_csi_traffic_mode_write_(mode);
        if (message != nullptr && message->empty()) {
          *message = accepted ? "csi traffic mode updated" : "csi traffic mode rejected";
        }
        return accepted;
      },
      [this](RuntimeTrafficMode mode, std::string *message) {
        const bool accepted = this->owner_.handle_traffic_generator_mode_write_(mode);
        if (message != nullptr && message->empty()) {
          *message = accepted ? "traffic generator mode updated" : "traffic generator mode rejected";
        }
        return accepted;
      },
      [this](DetectionAlgorithm algorithm, std::string *message) {
        const bool accepted = this->owner_.handle_detector_write_(algorithm);
        if (message != nullptr) {
          *message = accepted ? "detector updated" : "detector rejected";
        }
        return accepted;
      },
      [this](std::string *message) {
        this->owner_.protocol_recalibration_command_active_ = true;
        const bool accepted = this->owner_.handle_recalibration_write_();
        this->owner_.protocol_recalibration_command_active_ = false;
        if (message != nullptr && message->empty()) {
          *message = accepted ? "recalibration started" : "recalibration rejected";
        }
        return accepted;
      },
      [this](const EspectreCommand &wifi_command, std::string *message) {
        if (wifi_command.command == "scan_wifi") {
          if (!this->owner_.wifi_scan_callback_) {
            if (message != nullptr) *message = "Wi-Fi scanning is unavailable";
            return false;
          }
          return this->owner_.wifi_scan_callback_(message);
        }
        if (wifi_command.command == "clear_wifi_credentials") {
          if (!this->owner_.provisioning_command_callback_) {
            if (message != nullptr) {
              *message = "Wi-Fi configuration removal is unavailable";
            }
            return false;
          }
          return this->owner_.provisioning_command_callback_("CLEAR_WIFI", message);
        }
        if (wifi_command.command == "clear_wifi_bssid") {
          if (!this->owner_.provisioning_command_callback_) {
            if (message != nullptr) {
              *message = "Wi-Fi BSSID removal is unavailable";
            }
            return false;
          }
          return this->owner_.provisioning_command_callback_("SET_WIFI_BSSID:bssid=", message);
        }
        if (!this->owner_.provisioning_command_callback_) {
          if (message != nullptr) {
            *message = "Wi-Fi BSSID selection is unavailable";
          }
          return false;
        }
        const std::string encoded = "SET_WIFI_BSSID:bssid=" + encode_urlencoded_component(wifi_command.wifi_bssid);
        return this->owner_.provisioning_command_callback_(
            encoded + (wifi_command.wifi_bssid_force ? "&force=true" : ""), message);
      },
      [this](const EspectreCommand &mqtt_command, bool clear, std::string *message) {
        EspectreDeviceConfig updated = this->owner_.device_config_;
        if (clear) {
          clear_espectre_mqtt_config(&updated);
        } else {
          updated.mqtt_scheme = mqtt_command.mqtt_scheme;
          updated.mqtt_host = mqtt_command.mqtt_host;
          updated.mqtt_port = mqtt_command.mqtt_port;
          if (mqtt_command.has_mqtt_username) {
            updated.mqtt_username = mqtt_command.mqtt_username;
          }
          if (mqtt_command.has_mqtt_password) {
            updated.mqtt_password = mqtt_command.mqtt_password;
          }
          if (mqtt_command.has_mqtt_topic_prefix) {
            updated.topic_prefix =
                mqtt_command.mqtt_topic_prefix.empty() ? ESPECTRE_TOPIC_PREFIX : mqtt_command.mqtt_topic_prefix;
          }
        }
        if (this->owner_.device_config_change_callback_ &&
            !this->owner_.device_config_change_callback_(updated, false, message)) {
          return false;
        }
        this->owner_.set_device_config(updated);
        this->owner_.mqtt_frontend_->setup();
        if (message != nullptr && message->empty()) {
          *message = clear ? "MQTT configuration cleared" : "MQTT configuration saved";
        }
        return true;
      },
      [this](bool enabled, std::string *message) {
        if (this->owner_.ota_frontend_quiesced_) {
          if (message != nullptr) *message = "sensing is unavailable during OTA";
          return false;
        }
        this->owner_.runtime_.set_services_armed(enabled);
        this->owner_.update_live_telemetry_enabled_();
        if (message != nullptr) {
          *message = enabled ? "sensing started" : "sensing stopped";
        }
        return true;
      },
      [this](const EspectreCommand &raw_command, const FrontendCommandContext &context, std::string *code,
             std::string *message, std::string *data_json) {
        return this->owner_.direct_frontend_->handle_raw_stream_command(raw_command, context, code, message, data_json);
      });

  if (result.accepted && result.command.command == "read_diagnostics") {
    result.data_json = owner_.direct_frontend_->diagnostics_payload();
  }
  if (result.accepted) {
    if ((static_cast<uint8_t>(result.changes) & static_cast<uint8_t>(FrontendCommandChange::HEALTH)) != 0U) {
      owner_.publish_runtime_status_state_();
    }
    if ((static_cast<uint8_t>(result.changes) & static_cast<uint8_t>(FrontendCommandChange::SENSING)) != 0U) {
      const std::string payload = owner_.direct_frontend_->sensing_payload();
      owner_.mqtt_frontend_->publish_message("sensing", payload, true);
      owner_.direct_frontend_->publish_event("sensing", payload);
    }
    if ((static_cast<uint8_t>(result.changes) & static_cast<uint8_t>(FrontendCommandChange::WIFI)) != 0U) {
      const std::string payload = owner_.direct_frontend_->wifi_payload();
      owner_.mqtt_frontend_->publish_message("wifi", owner_.direct_frontend_->wifi_payload(true), true);
      owner_.direct_frontend_->publish_event("wifi", payload);
    }
    if ((static_cast<uint8_t>(result.changes) & static_cast<uint8_t>(FrontendCommandChange::DEVICE)) != 0U) {
      const std::string payload = espectre_device_payload(owner_.device_config_, owner_.mqtt_protocol_device_info_());
      owner_.mqtt_frontend_->publish_message("device", payload, true);
      owner_.direct_frontend_->publish_event("device", payload);
    }
  }
  return result;
}

EspectreCapabilityProfile NativeCommandBindings::capability_profile(bool allow_local_config) const {
  EspectreCapabilityProfile profile;
  profile.extension = owner_.ota_service_ != nullptr ? &frontend_ota_protocol() : nullptr;
  using Method = EspectreDirectMethod;
  profile.set(Method::CAPABILITIES);
  profile.set(Method::INFO);
  profile.set(Method::STATUS);
  profile.set(Method::CONFIG);
  profile.set(Method::DIAGNOSTICS);
  profile.set(Method::SET_SENSING);
  profile.set(Method::SET_DEVICE_LABEL);
  profile.set(Method::SET_THRESHOLD, owner_.runtime_.capabilities().supports_runtime_threshold_updates);
  profile.set(Method::SET_MOTION_HITS, owner_.runtime_.capabilities().supports_runtime_motion_hits_updates);
  profile.set(Method::SET_DETECTOR, owner_.runtime_.capabilities().supports_runtime_detector_selection);
  profile.set(Method::RECALIBRATE, owner_.runtime_.capabilities().supports_manual_recalibration);
  profile.set(Method::SET_CSI_TRAFFIC_MODE, owner_.runtime_.capabilities().supports_traffic_control);
  profile.set(Method::SET_TRAFFIC_GENERATOR_MODE, owner_.runtime_.capabilities().supports_traffic_control);
  profile.set(Method::WIFI_ACCESS_POINTS, allow_local_config);
  profile.set(Method::SCAN_WIFI_ACCESS_POINTS, allow_local_config);
  profile.set(Method::SET_WIFI_BSSID, allow_local_config);
  profile.set(Method::CLEAR_WIFI_BSSID, allow_local_config);
  profile.set(Method::CLEAR_WIFI_CONFIG, allow_local_config);
  profile.set(Method::SET_MQTT_CONFIG, allow_local_config);
  profile.set(Method::CLEAR_MQTT_CONFIG, allow_local_config);
  profile.set(Method::DISCOVER_PEERS, allow_local_config && owner_.direct_frontend_->peer_discovery_available());
  const bool raw_csi = allow_local_config && owner_.direct_frontend_->raw_stream_available() &&
                       owner_.runtime_.capabilities().supports_raw_csi;
  profile.set(Method::START_RAW_STREAM, raw_csi);
  profile.set(Method::STOP_RAW_STREAM, raw_csi);
  profile.set(EspectreConfigSection::RUNTIME);
  profile.set(EspectreConfigSection::DEVICE);
  profile.set(EspectreConfigSection::WIFI, allow_local_config);
  profile.set(EspectreConfigSection::MQTT, allow_local_config);
  return profile;
}

}  // namespace espectre
