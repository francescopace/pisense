/*
 * ESPectre - Command Capabilities Probe
 *
 * Emits the normalized C++ capability and wire-contract catalogs used by the
 * cross-language parity gate.
 *
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
#include <iostream>
#include <string>

#include "runtime/direct_http_protocol.h"
#include "frontend/ota_protocol.h"
#include "runtime/espectre_protocol.h"

int main(int argc, char **argv) {
  espectre::EspectreDeviceConfig config;
  espectre::EspectreDeviceInfo info;
  info.supports_info = true;
  const bool micro_profile = argc > 1 && std::string(argv[1]) == "micro";
  if (micro_profile) {
    espectre::EspectreCapabilityProfile capabilities;
    using Method = espectre::EspectreDirectMethod;
    capabilities.set(Method::CAPABILITIES);
    capabilities.set(Method::INFO);
    capabilities.set(Method::STATUS);
    capabilities.set(Method::CONFIG);
    capabilities.set(Method::DIAGNOSTICS);
    capabilities.set(Method::RECALIBRATE);
    capabilities.set(espectre::EspectreConfigSection::RUNTIME);
    capabilities.set(espectre::EspectreConfigSection::DEVICE);
    capabilities.set(espectre::EspectreConfigSection::WIFI);
    capabilities.clear_events();
    capabilities.set(espectre::EspectreEvent::TELEMETRY);
    std::cout << "{\"capabilities\":"
              << espectre::espectre_capabilities_payload(config, info, capabilities)
              << ",\"message_model\":" << espectre::espectre_message_catalog_payload(&espectre::frontend_ota_protocol()) << "}\n";
    return 0;
  }

  info.supports_diagnostics = true;
  info.supports_runtime_threshold = true;
  info.supports_runtime_motion_hits = true;
  info.supports_runtime_detector = true;
  info.supports_manual_recalibration = true;
  info.supports_traffic_control = true;

  const bool native_profile = argc > 1 && std::string(argv[1]) == "native";
  info.supports_device_config = native_profile;
  std::cout << "{\"capabilities\":"
            << espectre::espectre_capabilities_payload(config,
                                                       info,
                                                       true,
                                                       true,
                                                       native_profile,
                                                       native_profile,
                                                       native_profile,
                                                       native_profile,
                                                       false,
                                                       native_profile ? &espectre::frontend_ota_protocol() : nullptr)
            << ",\"message_model\":" << espectre::espectre_message_catalog_payload(&espectre::frontend_ota_protocol())
            << ",\"transport_mapping\":" << espectre::espectre_transport_mapping_payload()
            << "}\n";
  return 0;
}
