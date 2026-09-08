# Tuning Guide

Use this guide with the maintained C++ frontends after a device is installed and connected, even if it is not producing valid motion data yet. Micro-ESPectre users can follow the shared troubleshooting guidance linked from its README; its configuration and control details remain there. This guide explains what to check, what to change, and in what order. Detector formulas and validation evidence remain in [ALGORITHMS.md](ALGORITHMS.md) and the generated [performance report](performance/README.md). Other frontend-specific behavior belongs in the relevant local README.

Inline snippets use ESPHome YAML as a concrete example. CSI means channel state information. Accepted `pps` is the identity-accepted capture supply; admitted `pps` is the detector input after temporal slot admission.

## Tuning Order

1. Verify packet flow, slot occupancy, and sensor placement.
2. If you use `lightweight`, boot with the room quiet.
3. Walk through the monitored area and confirm that the movement score responds to real movement.
4. Choose the detection profile that fits the product's accuracy and resource budget.
5. Tune the threshold.
6. Tune motion-hit filtering if state changes are too slow or unstable.
7. Enable the low-pass filter only if noise remains after the earlier checks.
8. Change `csi_target_pps` or `segmentation_window_size_ms` only as a measured experiment, then rerun the relevant performance validation.

Change one setting at a time and repeat the same quiet-and-motion test after each change.

## Startup And Detection Profile

```yaml
espectre:
  detection_algorithm: lightweight  # or high_accuracy
```

| Profile | Accuracy and cost | Startup behavior |
|---------|-------------------|------------------|
| **Lightweight Detection** (`lightweight`) | Lower detector CPU and working-memory cost, with lower accuracy and generalization than High Accuracy | Uses up to 10 seconds of valid, ready coverage after temporal warmup; a clean `quiet -> motion -> quiet` pattern can finish earlier |
| **High-Accuracy Detection** (`high_accuracy`) | Higher accuracy and generalization, with additional feature state and neural-inference work | Skips threshold calibration; waits for CSI readiness and feature-window warmup |

For Lightweight, stay quiet immediately after boot. After the first quiet phase, one short movement may complete startup early, but it is optional. Repeated movement during the initial quiet phase still reduces calibration quality. Missing or burst-concentrated slots extend the wall-clock duration because they do not count as valid evidence.

Choose `lightweight` when the surrounding firmware needs the smaller active detector state and lower per-packet cost. Choose `high_accuracy` when detection quality matters more than that additional cost. ESPHome, Native, and Matter persist an accepted runtime profile selection through the controls they advertise. A profile change resets the threshold to the selected profile's default, and `high_accuracy -> lightweight` starts calibration. Published Matter firmware starts with `lightweight`.

See [ALGORITHMS.md](ALGORITHMS.md#known-limits) and the [performance report](performance/README.md) for current measurements and known limits. The relevant frontend README owns the exact configuration and control surface.

## Threshold

Lightweight adapts its threshold to the observed room during startup. It may lower that value later after a long quiet stretch if the opening was noisier than the rest of the session. High Accuracy starts from the threshold validated with the exported model.

Both profiles use a `0.0-1.0` probability threshold. Where a frontend advertises writable threshold control, an operator can override it for the current session. A Lightweight override suspends automatic threshold lowering until recalibration or explicit adaptive-threshold application. Operator changes are discarded at boot: Lightweight calibrates again, while High Accuracy restores its trained default.

Inspect the capture-quality counters in logs or [API.md](API.md#diagnostics) when raw packet rate looks healthy but sensing is unstable or unavailable. Hardware-invalid estimates are rejected before both sensing and collection. In particular, a C5/C6 driver can deliver ACK CSI callbacks with a cleared estimate-valid flag: filtering them prevents corrupted input but cannot make that driver produce usable ACK estimates. Compare valid occupancy with another explicitly selected generator; do not lower thresholds to compensate for invalid or missing CSI.

Rules of thumb:

- too many false positives: raise the threshold
- missed movement: lower the threshold

Tune the threshold before changing filters, packet cadence, or detector-window geometry.

## Evaluation Cadence And Hit Filtering

```yaml
espectre:
  evaluation_interval_ms: 250
  motion_on_hits: 4
  motion_off_hits: 3
```

The detector processes every admitted CSI packet into its sliding window, but it evaluates and publishes on the coarser `evaluation_interval_ms` cadence. Packet timestamps drive that cadence; there is no packet-count fallback, so live input and supported replay datasets must provide advancing timestamps.

Each evaluation produces a raw `IDLE` or `MOTION` reading. The runtime requires `motion_on_hits` consecutive opposing readings before publishing `MOTION`, and `motion_off_hits` consecutive readings before returning to `IDLE`. One reading in the current published state clears the pending count. These hits are evaluation ticks, not detector windows.

With regular 250 ms evaluation ticks, the default confirmation latency from a physical transition depends on its alignment with the next tick:

| Transition | Hits | Confirmation latency |
|------------|------|----------------------|
| `IDLE -> MOTION` | `4` | about `0.75-1.0 s` |
| `MOTION -> IDLE` | `3` | about `0.50-0.75 s` |

The lower bound applies when the transition aligns with an evaluation tick; the upper bound applies when it begins just after one. Missing valid coverage can delay the next evaluation further.

Rules of thumb:

- increase `motion_on_hits` to reject brief motion bursts
- increase `motion_off_hits` to keep short idle readings from clearing motion
- reduce the corresponding hit count when confirmation is too slow
- changing `evaluation_interval_ms` scales both confirmation ranges proportionally

ESPHome, Native, and Matter expose persisted runtime hit controls through the surfaces they advertise. Each makes telemetry available on every detector evaluation once `ready_to_publish` is true and a frontend-specific consumer requests it.

## Filters

### Hampel Filter

Default: enabled

```yaml
espectre:
  hampel_enabled: true
  hampel_window: 7
  hampel_threshold: 5.0
```

Hampel filtering suppresses short outlier spikes and feeds both `lightweight` and `high_accuracy`. Keep it enabled unless a controlled comparison shows that it removes useful motion detail in the target environment.

### Low-Pass Filter

Default: disabled

```yaml
espectre:
  lowpass_enabled: true
  lowpass_cutoff: 11.0
```

Use the low-pass filter when a stable installation still produces noise-driven false positives after threshold tuning. A lower cutoff applies more smoothing and may hide fast motion; a higher cutoff preserves more short-term variation.

The current C++ implementations calculate low-pass coefficients against a nominal `100 Hz` sample rate. `lowpass_cutoff` has its nominal frequency meaning when the admitted stream follows that regular cadence. A different target or substantial missing-slot pattern changes the effective time scale, so treat that combination as an experiment and revalidate it.

## Traffic Health And Target Rate

For frontends that expose the shared internal traffic generator:

```yaml
espectre:
  csi_target_pps: 100
  csi_traffic_mode: internal
  traffic_generator_mode: ping
```

`csi_target_pps` defines both the detector's temporal grid and the managed-traffic target. `csi_traffic_mode` independently selects whether the device or an external source supplies traffic. The runtime admits at most one packet per temporal slot, preserves missing slots, and requires at least 70% valid occupancy before detection is ready. Raw packet rate alone is therefore not enough: an access point can deliver packets in bursts that leave both same-slot excess and missing slots.

If occupancy remains below 70%:

1. verify that the selected traffic source is active
2. inspect packet loss, burst delivery, and Wi-Fi placement
3. repair the traffic path when possible
4. if the path cannot sustain the cadence, lower `csi_target_pps` explicitly and rerun performance validation

The runtime never changes the target automatically because doing so would change feature timing.

The C++ sensing frontends support internal `ping`, `dns`, `dns_tcp`, and experimental `wifi_raw`. Ping sends ICMP echo requests, `dns` sends connectionless UDP/53 queries, and `dns_tcp` uses a persistent, non-blocking TCP connection to gateway port `53`. `wifi_raw` sends Null Data directly to the AP and selects LLTF20 for ACK capture. It may improve sampling regularity on some deployments, but usable ACK CSI depends on the device and driver; [SETUP.md](SETUP.md#traffic-generation) owns the validation matrix, runtime transitions, and compatibility limits. The published product configurations use ping. Evaluate valid temporal occupancy, capture-quality counters, and detector readiness when comparing generators; raw packet rate alone is insufficient. Select the mode that remains stable with the deployed device, driver, AP, and gateway resolver; the runtime does not fall back automatically.

Rules of thumb:

- `100 pps`: production default and the cadence used by current training and validation
- lower target: less traffic and lower temporal resolution; requires validation at the chosen cadence
- higher target: more Wi-Fi and CPU cost without guaranteed occupancy or detector improvement; requires validation

The collector, external UDP marker, raw HTTP framing, and persistence behavior belong to [CLI.md](CLI.md#collect), [ML_DATA_COLLECTION.md](ML_DATA_COLLECTION.md), [API.md](API.md#csi-collection), and [SETUP.md](SETUP.md#traffic-generation).

## Detector Window

```yaml
espectre:
  segmentation_window_size_ms: 1000
```

The setting is elapsed time. Together with `csi_target_pps`, it defines the fixed slot count:

```text
window_slots = ceil(csi_target_pps * segmentation_window_size_ms / 1000)
```

The production model, replay gates, training workflow, and published performance evidence use `1000 ms`. Other supported values change feature geometry and response time but are not covered by those published results. Do not use the window as a routine false-positive or latency control; prefer threshold and hit filtering. If a product needs a different window, validate the selected detector profile and the C++/Python parity gates at that setting.

The detailed timing contract and historical-data restrictions live in [ALGORITHMS.md](ALGORITHMS.md#detector-timing) and the [fixed temporal-admission ADR](adr/2026-08-15-use-fixed-temporal-csi-admission.md).

## Sensor Placement

Verify placement before compensating with detector parameters. Start with a stable path through the monitored area, keep the device out of metal enclosures and behind as few heavy obstacles as practical, and confirm that accepted rate and occupancy remain stable while the room is quiet.

A distance of roughly `3-8 m` from the access point is a starting point, not a requirement. Walls, antenna orientation, access-point power, and furniture can matter more than distance. Use the dedicated [sensor placement guide](https://espectre.dev/guides/placement/) for the RSSI ranges, room layouts, and repeatable placement test.

## Troubleshooting

### Too Many False Positives

Try in this order:

1. distinguish real environmental movement, such as fans, curtains, or pets, from radio noise
2. verify occupancy and placement
3. raise the threshold
4. increase `motion_on_hits` if only short bursts become alarms
5. at the default 100 PPS cadence, enable or tune the low-pass filter
6. for Lightweight, recalibrate in a quiet room

### Missing Movements

Try in this order:

1. verify packet flow, occupancy, and placement
2. lower the threshold
3. reduce `motion_on_hits` if the raw score responds but the published state changes too slowly
4. compare High Accuracy when the additional runtime cost fits the product

### Calibration Stalls Or Startup Quality Is Poor

Common causes are movement during the initial quiet phase, insufficient valid slot coverage, a poor radio path, or a chaotic RF environment at boot.

Try:

1. inspect occupancy rather than raw packet rate alone
2. boot again with a quieter room
3. improve placement or traffic delivery
4. let startup finish before judging steady-state quality

### Unstable Detection Or Flickering

Try:

1. verify that packet occupancy is stable
2. raise the threshold if the raw score crosses it repeatedly in a quiet room
3. increase the relevant hit count
4. enable the low-pass filter at the default 100 PPS cadence if the score itself remains noisy

### No CSI Packets

Check Wi-Fi connection status, the traffic source, the CSI-enabled build configuration, and actual packet flow. If logs report protocol or bandwidth as `unavailable`, do not infer a CSI failure from that field alone; use packet counters and calibration progress.

### Mesh Wi-Fi Instability

If the device roams between access points that share an SSID, the radio path, channel, and packet delivery can change underneath the detector. Pin the device to a specific BSSID when roaming causes unstable occupancy or detection. ESPectre exposes this control when it is advertised in its Direct capability catalog. Native persists the pin across reboot. ESPHome persists an ESPectre-only pin and reapplies it after reconnect. Matter persists an ESPectre-owned override for the commissioned SSID, reapplies it after restart only while that SSID matches, and leaves Matter-owned credentials unchanged.

In the browser:

1. Open [Device settings](https://espectre.dev/tools/device-settings/).
2. Connect with the private IP, device name, full 16-character device ID, or the last 6 characters of that ID.
3. Refresh the access-point list, select the BSSID you want, and save.

From the repository CLI:

```bash
./espectre direct post wifi/scans  # start an access-point scan
./espectre direct get wifi/access-points  # list BSSID, channel, and RSSI
./espectre direct put wifi/bssid --data '{"bssid":"AA:BB:CC:DD:EE:FF"}'  # pin one AP
```

The scan is asynchronous, so wait a few seconds after `POST /wifi/scans` before reading `GET /wifi/access-points`. Use `--frontend native`, `--frontend esphome`, or `--frontend matter` to filter discovery, or use `--endpoint` when you already know the Direct base URL. The station reconnects after a pin or clear.

To restore automatic access-point selection without removing the SSID or password, choose automatic selection in Device settings or run:

```bash
./espectre direct delete wifi/bssid
```

Clear a stale pin after replacing or removing an access point. [CLI.md](CLI.md#direct) owns Direct syntax, and [API.md](API.md) owns the methods.

### False Positives After A Wi-Fi Channel Change

Prefer a fixed access-point channel when possible. After a channel change, allow the runtime to reset its detector history and collect fresh valid coverage before evaluating the result.

## Recalibration

Use recalibration after a material placement or radio-environment change when the frontend advertises the control.

- Lightweight starts a fresh threshold calibration; keep the room quiet as you would at boot.
- High Accuracy immediately restores its trained threshold and does not collect a quiet-room window.

## Monitoring

During tuning, watch:

- accepted and admitted packet rates
- slot occupancy and missing-slot or excess-slot diagnostics
- motion state and movement score
- current threshold and calibration state
- heap and runtime-loop stability when comparing firmware variants

Read the packet rates in sequence. Traffic without CSI callbacks points to capture or radio state. Callbacks without accepted packets point to capture validation or identity filtering. Accepted packets without adequate admission or occupancy point to temporal delivery. Stable detector input with unstable output points to threshold, filtering, or detector behavior.

The shared ESP-IDF runtime exposes periodic debug telemetry, but compiler mode, log level, hardware, Wi-Fi setup, and traffic rate must stay fixed for a meaningful firmware comparison. Use the repository [firmware benchmark](../tools/README.md#firmware-benchmark) and generated [performance reports](performance/README.md) for repeatable resource measurements rather than treating an ad hoc tuning session as benchmark evidence.

## Related Docs

- [`README.md`](../README.md)
- [`SETUP.md`](SETUP.md)
- [`ALGORITHMS.md`](ALGORITHMS.md)
- [`ARCHITECTURE.md`](ARCHITECTURE.md)
- [`API.md`](API.md)
- the README of the selected frontend
