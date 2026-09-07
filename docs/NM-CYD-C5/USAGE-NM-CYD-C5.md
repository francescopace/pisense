# ESPectre × NM-CYD-C5 User Guide

> Firmware: `src/cpp/frontend/esphome/examples/espectre-cyd-c5.yaml` (extends `espectre-c5.yaml`)
> Hardware: NM-CYD-C5 (external-antenna version recommended, nm-cyd-c5-ant; ESP32-C5, 2.8" 320×240 ST7789 touch display)

ESPectre is a WiFi CSI (Channel State Information) based human motion detection system. In addition to the core motion detection, this firmware on the NM-CYD-C5 provides complete local interaction: a live motion curve, threshold line display, touch buttons for manual threshold adjustment, and one-tap calibration, plus Home Assistant and web tool support.

---

## 1. Feature Overview

| Feature | Access |
|---|---|
| Motion detection (lightweight / high_accuracy algorithms) | Runs automatically — passive WiFi CSI sensing, nothing to wear |
| Movement curve + Threshold line | On-device display (320×240) |
| Touch threshold adjustment (±0.05, range 0–1) | Screen buttons / HA |
| Auto-calibration (adaptive threshold) | Screen CALIBRATE button / HA Recalibrate button |
| Home Assistant integration | Native ESPHome API (auto-discovery) |
| Web tools (entity control, device settings) | https://espectre.dev/tools/ |

## 2. First-Time Provisioning

The device ships with no saved WiFi credentials and automatically starts a provisioning hotspot on first boot after flashing:

1. On your phone or computer, scan for WiFi networks and connect to the **`ESPectre Fallback`** hotspot;
2. The captive portal opens automatically in the browser (or browse manually to `192.168.4.1`);
3. Select your WiFi network and enter its password — **a 2.4 GHz network is recommended** (ESPectre's CSI detection operates on the 2.4 GHz band);
4. After a successful save, the device reboots and connects to your WiFi automatically;
5. Once connected, **the device IP address is shown directly in the top-left header of the screen** (yellow `NO WIFI` is displayed while disconnected).

> You can also provision over USB with Improv Serial, e.g. `./espectre provision --ssid MyNetwork` or the web flasher at https://espectre.dev/tools/flash/.

## 3. Adding to Home Assistant

Prerequisite: the device and Home Assistant are on the **same subnet**.

1. In Home Assistant, install the **ESPHome** integration (Settings → Devices & Services → Add Integration → ESPHome);
2. Open Home Assistant → **Settings** → find **ESPHome** → click **Add New Device**;
3. When the device and HA are on the same network it is **discovered automatically** (mDNS hostname `espectre`) — click it and confirm to finish adding; if it is not discovered, enter the IP address shown on the screen manually.

Once added, the main entities are:

| Entity | Type | Description |
|---|---|---|
| Movement Score | sensor | Motion score (data source of the curve) |
| Motion Detected | binary_sensor | Motion state (usable as an automation trigger) |
| Threshold | number | Threshold, normalized range 0–1 (linked with the screen buttons) |
| Calibration Active | binary_sensor | ON while calibration is running |
| Recalibrate | button | Triggers re-calibration |
| WiFi Signal | sensor | Signal strength in dBm |

## 4. Screen Layout & Operation

```
┌──────────────────────────────────────────────┐
│ 192.168.1.100   MOTION/IDLE/CAL…  lightweight│ Header: IP / state / algorithm
├──────────────────────────────────────────────┤
│        ╭─╮        Movement curve (cyan)       │
│       ╱   ╲╭─╮    over-threshold segments red │
│  - - - - - - - -  Threshold dashed line (yellow)│
├──────────────────────────────────────────────┤
│  0.32 mv   0.25 thr                 -55dBm   │ Values row
├────────────┬──────────────────┬──────────────┤
│  THR −0.05 │    CALIBRATE     │   THR +0.05  │ Touch buttons
└────────────┴──────────────────┴──────────────┘
```

- **Graph area**: ~68 seconds of scrolling history (~4 points/s), auto-scaled Y axis; curve segments above the yellow threshold line turn red, making the "decision" process intuitive.
- **THR −0.05 / +0.05**: step-adjust the threshold, range 0–1, takes effect immediately.
- **CALIBRATE**: triggers re-calibration (~10 seconds — keep the environment still during it). Calibration recomputes the adaptive threshold; `Calibration Active` is ON while it runs.
- Status text: `MOTION` (red) / `IDLE` (green) / `CALIBRATING...` (blue) / `BOOT` (yellow).

## 5. Web Tools

Open https://espectre.dev/tools/device-settings/ and point it at the device (by IP or `espectre.local`) to view and control the device. The device also serves the ESPectre Direct HTTP API (see `docs/DISCOVERY.md`).

Firmware updates: use the ESPHome dashboard or `esphome upload` (direct over WiFi, no USB needed).

## 6. Daily Usage Tips

- **Placement**: keep the line of sight between the device and the router free of metal obstructions — CSI is sensitive to 2.4 GHz link quality;
- **When to calibrate**: after moving the device, changing the room layout, or when false positives/negatives become noticeable, press CALIBRATE again (keep the room empty and still during calibration); or trigger the Recalibrate button from Home Assistant;
- **Threshold tuning**: more false positives → increase; more missed detections → decrease. For detecting small movements, re-calibrate first before fine-tuning;
- **Self-healing on network loss**: the device reconnects WiFi automatically, and CSI detection resumes by itself — no intervention needed.

## 7. Troubleshooting

| Symptom | Fix |
|---|---|
| Screen shows `NO WIFI` | Not connected: join the `ESPectre Fallback` hotspot and provision again, using a 2.4 GHz network |
| HA cannot find the device | Check the subnet/VLAN; or add it manually with the IP shown on the screen |
| Touch buttons unresponsive or offset | Resistive touch varies per unit — fine-tune the four boundary values in `touchscreen.calibration` in the YAML |
| Calibration fails or is unsatisfactory | Keep the environment absolutely still during calibration; when the signal is too strong (very close to the router) the gain lock skips, which is expected |
| Firmware update | ESPHome dashboard or `esphome upload` (direct over WiFi, no USB) |
