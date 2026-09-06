# ESPectre API

This document owns the public ESPectre application contract carried by Direct HTTP, Server-Sent Events (SSE), and MQTT. Discovery is specified in [DISCOVERY.md](DISCOVERY.md).

## Contract principles

- The application version is `1.0`, and the Direct base path is `/espectre/v1`.
- HTTP and MQTT share resource payloads, operation names, validation, and result codes. Transport framing and delivery policy stay outside resource payloads.
- `protocol_version` appears only in `capabilities` and discovery metadata.
- In application JSON, `device_id` appears only in `device` and multi-device discovery results. The established binary CSI records also carry it.
- This contract replaces the former RPC and explicit CSI-session APIs. There are no legacy routes, commands, topics, or aliases.

## Direct HTTP

Direct HTTP listens on TCP port `62587`. A client starts with `GET /espectre/v1/capabilities` and uses the returned `resources` and `operations` catalogs instead of inferring support from the frontend name.

### Support matrix

| Method | Path | Native | ESPHome | Matter | Micro |
| --- | --- | --- | --- | --- | --- |
| `GET` | `/health`, `/device`, `/capabilities`, `/sensing`, `/wifi`, `/diagnostics`, `/events` | yes | yes | yes | yes |
| `GET` | `/wifi/access-points`, `/devices`, `/csi` | yes | yes | yes | no |
| `GET` | `/mqtt`, `/ota` | yes | no | no | no |
| `PATCH` | `/device`, `/sensing` | yes | yes | yes | no |
| `PATCH` | `/mqtt` | yes | no | no | no |
| `POST` | `/sensing/calibrations` | yes | yes | yes | yes |
| `POST` | `/wifi/scans` | yes | yes | yes | no |
| `POST` | `/ota/checks`, `/ota/updates` | yes | no | no | no |
| `PUT` | `/wifi/bssid` | yes | yes | yes | no |
| `DELETE` | `/wifi/bssid` | yes | yes | yes | no |
| `DELETE` | `/wifi/credentials`, `/mqtt` | yes | no | no | no |

Unsupported resources and method combinations return HTTP `404`. Capability negotiation remains authoritative when a custom build disables an otherwise supported feature.

### Request and response framing

Successful `GET` requests return the resource object directly. A request body, when present, must be a JSON object. An empty body is equivalent to `{}`. C++ frontends accept at most 2,048 request bytes; Micro accepts at most 512 bytes. Send `Content-Type: application/json` whenever the body is non-empty.

Mutations return a result object:

```json
{"accepted":true,"code":"ok","message":"operation accepted","data":{}}
```

`data` is optional. Synchronous mutations use HTTP `200`. Asynchronous and disruptive routes use HTTP `202` after the request has passed application dispatch. Validation, capability, and conflict errors instead use the status codes in [Errors](#errors).

## Resources

### `health`

```json
{"status":"ok","online":true,"uptime_s":42,"timestamp_ms":42000}
```

`status` is `ok` while the device is online and `offline` in an MQTT Last Will. `uptime_s` and `timestamp_ms` use the device's monotonic clock. Calibration and intentional CSI collection do not make health degraded.

### `device`

```json
{
  "device_id": "3cf79180d3a0aca4",
  "name": "Kitchen",
  "label": "Kitchen",
  "frontend": "native",
  "firmware": "3.0.0-rc1",
  "chip": "esp32c6",
  "csi_profile": "ht20"
}
```

`device_id` is the stable 16-character lowercase hexadecimal identity. `label` is the configured label and may be empty. When it is empty, `name` contains the generated stable display name. `csi_profile`, when available, is the runtime-selected read-only profile: `lltf20`, `ht20`, or `vht20`.

### `capabilities`

`capabilities` contains:

| Field | Type | Meaning |
| --- | --- | --- |
| `protocol_version` | string | Application contract version, currently `1.0` |
| `resources` | array of strings | Resource names accepted by `GET`, without the base path |
| `operations` | array of objects | Supported operations as `{name, method, path}` |
| `events` | array of strings | Event names published by this frontend |
| `features.csi` | boolean | Whether `GET /csi` is available |
| `csi` | object, optional | CSI endpoint, framing versions, queue parameters, and external traffic settings |

The CSI feature is named `csi`. Clients must tolerate additive resources, operations, events, feature fields, and CSI parameters.

### `sensing`

| Field | Type | Meaning |
| --- | --- | --- |
| `enabled` | boolean | Desired sensing state |
| `ready` | boolean | Detector output may be consumed |
| `calibrating` | boolean | Calibration is active |
| `mode` | string | `sensing` or `csi_collection` |
| `derived_events_paused` | boolean | Motion and other derived events are suspended |
| `detector` | string | `lightweight` or `high_accuracy` |
| `threshold` | number | Current detector threshold in `[0.0, 1.0]` |
| `motion_on_hits`, `motion_off_hits` | integer | Consecutive evaluations required for each state transition |
| `csi_traffic_mode` | string | `internal` or `external` |
| `traffic_generator_mode` | string | `ping`, `dns`, or `dns_tcp` |
| `csi_target_pps` | integer | Configured CSI traffic target in packets per second |
| `csi_traffic_udp_port` | integer, optional | External CSI traffic UDP port |
| `csi_traffic_multicast_group` | string, optional | External CSI traffic multicast group |

During CSI collection, `mode` is `csi_collection`, `ready` is false, and `derived_events_paused` is true.

### `wifi`

The Direct resource contains `configured`, `connected`, `ssid`, `bssid`, `band`, `channel`, and `rssi_dbm`. `band` is `2g`, `5g`, or an empty string when unknown. Unavailable RSSI is `null`. Native also returns `ip`, `apply_state`, and `apply_message`; these fields are optional for other frontends. Station MAC addresses and stored credentials are never returned.

The retained MQTT `wifi` payload is redacted. It contains `configured`, `connected`, `band`, `channel`, `rssi_dbm`, `apply_state`, and `apply_message`, but omits SSID, BSSID, and IP address.

### `wifi/access-points`

```json
{
  "scanning": false,
  "message": "scan complete",
  "access_points": [
    {"bssid":"aa:bb:cc:dd:ee:ff","rssi_dbm":-51,"channel":10}
  ]
}
```

`access_points` contains the latest completed scan. Starting a later scan sets `scanning` and updates `message`; clients poll this resource for completion.

### `mqtt`

Native returns `configured`, `scheme`, `host`, `port`, `username_configured`, and `topic_prefix`. `configured` is true only when the scheme, host, and port form a valid endpoint. The password is write-only, and the username is represented only by `username_configured`.

### `ota`

OTA is a frontend-owned extension of the canonical message model. Native registers its resource, operations, event, and parameter validation through the same extension catalog for Direct HTTP and MQTT. The sensing SDK does not implement firmware updates.

| Field | Type | Meaning |
| --- | --- | --- |
| `state` | string | `idle`, `checking`, `update_available`, `up_to_date`, `downloading`, `applying`, `reboot_scheduled`, `error`, or `unknown` |
| `timestamp_ms` | integer | Monotonic snapshot timestamp |
| `busy` | boolean | An OTA check or update is running |
| `update_available` | boolean | The latest completed check found a newer image |
| `current_version`, `target_version` | string | Running and candidate firmware versions |
| `manifest_url`, `image_url` | string | Resolved release artifacts; empty before resolution |
| `default_channel`, `channel` | string | Build default and channel used by the current or latest attempt |
| `message` | string | Current status or failure detail |

### `diagnostics`

`GET /diagnostics` returns an on-demand snapshot. It is not published through SSE or a retained MQTT topic. `read_diagnostics` returns the same object inside `commands/result.data` on MQTT.

| Field group | Meaning |
| --- | --- |
| `timestamp_ms`, `uptime` | Monotonic timestamp in milliseconds and uptime in whole seconds |
| `free_memory_kb`, `minimum_free_memory_kb`, `largest_free_memory_kb` | Current heap, cumulative low-water heap, and current largest free block |
| `cpu_frequency_mhz`, `loop_time_ms` | CPU frequency and the most recent frontend loop-body cost |
| `performance_window_ready`, `performance_window_ms`, `runtime_load_percent` | Availability, duration, and runtime-loop load for the latest complete performance window |
| `loop_samples`, `loop_avg_us`, `loop_max_us` | Runtime loop timing for that window |
| `detection_timing_supported`, `detection_samples`, `detection_sum_us`, `detection_avg_us`, `detection_min_us`, `detection_max_us` | Detector timing support and aggregates |
| `traffic_packets_total`, `csi_callbacks_total`, `csi_classified_total`, `csi_provenance_rejected_total`, `csi_accepted_total`, `csi_admitted_total`, `csi_filtered_total` | Cumulative traffic and CSI pipeline counters |
| `csi_pending_frame_drops_total`, `csi_missing_slots_total`, `csi_excess_total`, `csi_stale_total`, `csi_out_of_order_total` | Cumulative queue and temporal-admission drop counters |
| `csi_occupancy_slots`, `csi_window_slots`, `csi_pending_frames`, `csi_pending_frame_capacity` | Detector-window and callback-queue occupancy |
| CSI and traffic fields ending in `_pps`, plus `csi_occupancy` | Cached packet rates and detector-window occupancy ratio |
| `wifi_channel`, `wifi_rssi_dbm` | Current channel and RSSI; unavailable RSSI is `null` |
| `runtime_motion_event_drops_total` | Runtime-to-frontend motion publications overwritten in the bounded mailbox |
| `task_stack_high_water_bytes` | Native frontend task stack headroom; omitted where unavailable |
| `direct_http` | SSE client and queue budgets plus connection, request, delivery, and `dropped_motion_events` counters |
| `raw_csi` | CSI session state, drops, send backpressure, delivered records, and stream sequence |
| `mqtt` | Native MQTT connection, queue, outbox, drop, failure, and reconnect counters |

Memory values use KiB. Timings use microseconds unless the field ends in `_ms`, and rates use packets per second. `csi_occupancy` is the valid fraction of the active detector window. Fields that depend on a complete performance window are `null` until that window is ready. A frontend may omit measurements and transport objects that it cannot provide; clients must not replace a missing value with zero.

## Operations

Operations reject unknown fields. Routes described as taking no parameters accept an empty body or `{}` only.

### Device update

`PATCH /device` accepts:

```json
{"label":"Kitchen"}
```

`label` is required, must be a single-line string, and may contain at most 32 encoded bytes. An empty string clears the label. Success returns HTTP `200` and publishes the updated `device` resource.

### Sensing update and calibration

`PATCH /sensing` accepts any non-empty supported subset of these fields:

| Field | Type and constraint |
| --- | --- |
| `enabled` | boolean |
| `detector` | `lightweight` or `high_accuracy` |
| `threshold` | finite number in `[0.0, 1.0]` |
| `motion_on_hits`, `motion_off_hits` | integers from `1` through `20`; both must be present together |
| `csi_traffic_mode` | `internal` or `external` |
| `traffic_generator_mode` | `ping`, `dns`, or `dns_tcp` |

The request is parsed and all field constraints and capabilities are checked before changes are applied. Success returns HTTP `200` and publishes `sensing`. `POST /sensing/calibrations` takes no parameters, returns HTTP `202` when queued, and returns `409` with code `busy` when calibration is already active.

### Wi-Fi scan and BSSID selection

`POST /wifi/scans` takes no parameters and returns HTTP `202` when the scan starts. Read progress and results from `GET /wifi/access-points`.

`PUT /wifi/bssid` accepts:

```json
{"bssid":"aa:bb:cc:dd:ee:ff","force":false}
```

`bssid` is required and must contain six hexadecimal octets separated by colons. `force` is an optional boolean and defaults to false. If the requested BSSID is already active, the normal request persists the selection without forcing a link transition; `force: true` exercises the disconnect and reconnect lifecycle.

`DELETE /wifi/bssid` clears the pin. `DELETE /wifi/credentials`, available only on Native, removes provisioned Wi-Fi credentials and returns the device to provisioning. Both DELETE routes take no parameters.

The three disruptive Wi-Fi mutations return HTTP `202` before any disconnect. Their result `data` contains the active BSSID observed before apply:

```json
{"accepted":true,"code":"ok","message":"Wi-Fi BSSID update accepted","data":{"current_bssid":"11:22:33:44:55:66"}}
```

### MQTT configuration

`PATCH /mqtt` accepts:

```json
{
  "scheme": "mqtts",
  "host": "broker.example.net",
  "port": 8883,
  "username": "sensor",
  "password": "secret",
  "topic_prefix": "espectre/v1/devices"
}
```

`scheme`, `host`, and `port` are required on every request. `scheme` accepts lowercase `mqtt` or `mqtts`. `host` is a DNS hostname, IPv4 address, or unbracketed IPv6 address without a scheme, user information, port, path, query, fragment, whitespace, or control characters. `port` is an integer from `1` through `65535`.

`username`, `password`, and `topic_prefix` are optional single-line strings with maximum lengths of 128, 256, and 128 bytes. An omitted optional field keeps its stored value. An empty `topic_prefix` restores `espectre/v1/devices`. Success returns HTTP `200`. `DELETE /mqtt` takes no parameters, clears the broker endpoint and credentials, and returns HTTP `200`.

`mqtt` selects explicit plaintext TCP for a trusted local broker. `mqtts` uses the public certificate bundle and verifies the broker hostname. WebSocket transports are not supported.

### OTA actions

`POST /ota/checks` and `POST /ota/updates` accept an optional channel:

```json
{"channel":"release"}
```

`channel` is `release`, `preview`, or `develop`. Omitting it uses the firmware build's default channel. Client-supplied `manifest_url`, `image_url`, and `version` values are rejected, as are all other unknown fields. A started check or update returns HTTP `202`; an already active OTA operation returns `409` with code `busy`. OTA state changes are published as `ota` events and retained on the MQTT `ota` topic.

## Events

`GET /espectre/v1/events` is the only JSON SSE connection. The C++ frontends publish `health`, `device`, `sensing`, `wifi`, `ota`, `motion`, and `fault` according to their capability catalog. Micro publishes only `motion`. Resource events carry complete snapshots. MQTT configuration, diagnostics, discovery results, and CSI never appear in this stream.

A `motion` event is produced for each detector evaluation:

```json
{"timestamp_ms":42000,"state":"idle","score":0.0123}
```

`state` is `idle` or `motion`, and `score` is the detector output. Threshold and detector metadata remain in `sensing`. Under backpressure, replaceable motion events may be coalesced or dropped; C++ diagnostics expose the count as `direct_http.dropped_motion_events`. Future derived events, such as presence and gesture, use this event plane.

A `fault` event reports a runtime error without changing a resource payload:

```json
{"timestamp_ms":42000,"message":"runtime fault"}
```

Each SSE connection receives a `: heartbeat` comment every 10 seconds. There is no replay. C++ frontends support at most two event clients, and Micro supports one. Persistent send failures close the affected stream.

## CSI collection

`GET /espectre/v1/csi` opens the single exclusive binary CSI collection session. No setup request, bearer token, session deletion, or bind timeout exists. Closing the TCP response ends collection.

The C++ runtime requires sensing services to be armed before collection starts. If sensing is disabled or services are suspended for reconfiguration or maintenance, opening collection fails without re-enabling CSI capture or traffic generation. An accepted collection pauses derived sensing while retaining the active capture and traffic services.

Each CSI V8 record retains the established 60-byte little-endian HTTP prefix. The client adopts the 16-byte session identifier from the first frame and rejects a change within the same connection. The producer preserves order; fixed-ring drops remain observable in the transport counters.

While CSI is active, sensing reports `csi_collection`, readiness is false, and motion plus all present or future derived events are paused on every transport. Control and resource events remain available. A second `/csi` request and sensing, Wi-Fi, or OTA mutations return `409`. On close, the runtime restores its prior state, recalibrates when required, and resumes derived events only after readiness returns. When external traffic is configured, the host traffic generator must start before opening `/csi`.

## MQTT

The base topic is `espectre/v1/devices/{device_id}` unless Native is configured with another `topic_prefix`.

| Suffix | Retained | Purpose |
| --- | --- | --- |
| `health` | yes | Health, availability, and Last Will |
| `device` | yes | Device identity and build |
| `capabilities` | yes | Negotiation and supported surface |
| `sensing` | yes | Sensing state and tuning |
| `wifi` | yes | Redacted radio state |
| `ota` | yes | OTA state |
| `motion` | no | Per-evaluation motion event |
| `fault` | no | Runtime fault |
| `commands/result` | no | Correlated command result |

There is no application heartbeat in addition to MQTT keepalive. The retained `health` topic is also the Home Assistant availability topic. The broker publishes the retained offline health Last Will after an ungraceful disconnect.

Publish commands to `commands/request`. Every request has a top-level `command_id` and `command`; parameters are also top-level. Neither `protocol_version` nor `device_id` is present.

```json
{"command_id":"cmd-42","command":"update_sensing","threshold":0.5}
```

`command_id` is 1 to 64 characters and accepts ASCII letters, digits, `_`, `-`, `.`, and `:`. The complete command payload is limited to 2,048 bytes. Results echo `command_id` and `command`, then carry `accepted`, `code`, `message`, and optional `data`.

| Command | Parameters | HTTP-equivalent validation |
| --- | --- | --- |
| `update_device` | `label` | `PATCH /device` |
| `update_sensing` | Any non-empty supported sensing subset | `PATCH /sensing` |
| `recalibrate` | none | `POST /sensing/calibrations` |
| `read_diagnostics` | none | `GET /diagnostics`; snapshot returned in `data` |
| `check_ota` | optional `channel` | `POST /ota/checks` |
| `start_ota` | optional `channel` | `POST /ota/updates` |

There are no MQTT topics for MQTT configuration, diagnostics, discovery, or CSI. Home Assistant Discovery remains a separate adapter profile and uses `health` for availability.

## Errors

Application results use `application/json` and the result object shown above. The C++ Direct dispatcher maps these stable codes as follows:

| HTTP status | Result codes | Meaning |
| --- | --- | --- |
| `400` | `invalid_params` | The JSON object, field set, type, or value is invalid |
| `404` | `unsupported` | The resource or operation is absent from the frontend capability set |
| `409` | `busy`, `conflict`, `busy_raw_collection` | The request conflicts with active calibration, discovery, CSI, or OTA work |
| `200` or `202` | `unavailable`, `internal_error` | Dispatch reached the operation, but its backend could not complete it; the route's synchronous or asynchronous status is retained |

Micro returns an application JSON result with HTTP `503` when a requested snapshot is unavailable. MQTT has no HTTP status; subscribers use `accepted` and `code` in `commands/result`. A command outside the published MQTT command set returns code `forbidden`.

The HTTP service can reject a request before application dispatch. These failures are not canonical result objects. C++ frontends return `text/plain; charset=utf-8`; Micro also uses plain text for some early failures. Clients must inspect the HTTP status and `Content-Type` before parsing JSON.

| HTTP status | Pre-dispatch failure |
| --- | --- |
| `400` | Malformed JSON, invalid route framing, or incomplete body |
| `403` | Origin or transport policy rejected the request |
| `404` | No route exists for the method and path |
| `409` | The HTTP service detected a CSI collection conflict |
| `413` | The body exceeds the frontend request limit |
| `415` | A non-empty body does not declare JSON content |
| `429` | The request or mutation rate limit was reached |
| `503` | The service is stopping, its queue is full, or request handling cannot start |

`rate_limited` is not an application result code. Rate limiting is a transport failure reported with HTTP `429` and counted in diagnostics.

## Security and versioning

Direct HTTP is a trusted-LAN surface. Firmware enforces exact browser Origin allowlists, Private Network Access preflight, bounded bodies, queues, clients, and request rates. It binds to the station interface and does not expose stored Wi-Fi or MQTT passwords. `/mqtt` may gain independent protection only through an additive security extension.

Clients negotiate once through `capabilities.protocol_version`. Compatible additions may add resources, fields, operations, or events within `v1`; consumers must ignore unknown additive fields. An incompatible contract requires a new base-path major version and discovery protocol version.
