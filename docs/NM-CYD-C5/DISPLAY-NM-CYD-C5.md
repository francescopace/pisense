# NM-CYD-C5 Display Solution (ESPectre × Cheap Yellow Display C5)

> Goal: bring [NM-CYD-C5](https://github.com/RockBase-iot/NM-CYD-C5) (the ESP32-C5 version of the "Cheap Yellow Display", 320×240 ST7789 + XPT2046 resistive touch) into ESPectre, and show a live Movement-score curve with the Movement Threshold line on the local display — with on-screen threshold adjustment and one-tap calibration.

## 1. What the ESPectre component exposes (code survey)

The ESPHome component (`src/cpp/frontend/esphome/components/espectre/`) auto-creates its core entities with defaults; the example only needs to give them IDs. The ones used by the display and touch buttons:

| # | Data / State | Entity | Update cadence |
|---|---|---|---|
| 1 | **Movement score** (detector metric) | `movement_sensor` (sensor) | Every `evaluation_interval_ms` (default 250 ms → 4 Hz) |
| 2 | **Motion state** (IDLE / MOTION, with on/off hit hysteresis) | `motion_sensor` (binary_sensor) | On state change |
| 3 | **Movement Threshold** (normalized, 0.0–1.0, step 0.01; auto-selected at startup, runtime-adjustable) | `threshold_number` (number) | On calibration complete / manual change |
| 4 | **Calibration state** | `calibration_active_sensor` (binary_sensor) | On trigger / completion |
| 5 | **Recalibrate trigger** | `recalibrate_button` (button) | On press |

**Conclusion**: these entities cover everything needed for the requested features — **curve (1) + threshold line (3) + threshold adjustment (3) + calibration (4/5)**. The full loop is possible without touching C++ component code.

## 2. NM-CYD-C5 hardware notes (port mapping)

| Function | Pins | Notes |
|---|---|---|
| SPI bus (shared by display / touch / SD) | SCK=GPIO6, MISO=GPIO2, MOSI=GPIO7 | One `spi:` bus with three devices |
| LCD (ST7789, 240×320, ILI9341 variant possible) | CS=GPIO23, DC=GPIO24, RST = none (-1) | ESPHome `mipi_spi` platform |
| Backlight | GPIO25, active HIGH | `enable_pin` of `mipi_spi` |
| Touch XPT2046 | CS=GPIO1, IRQ not wired | ESPHome `touchscreen: xpt2046`, polled mode; calibration X:185–3700 / Y:250–3800 (from the factory demo) |
| SD card | CS=GPIO10 | Unused here (shares the SPI bus) |
| WS2812 RGB LED | GPIO27 | Optional: motion indicator |
| MCU | ESP32-C5-WROOM-1 (16MB Flash / 8MB PSRAM), 240 MHz | Reuses the base `espectre-c5.yaml` `esp-idf` setup |

The approach mirrors `examples/espectre-s3-touch-lcd.yaml` (Waveshare S3 1.47" display): **add one example YAML, no component code changes**.

## 3. Screen layout (320×240 landscape)

```
┌──────────────────────────────────────────────┐  y=0
│ 192.168.1.100   ●MOTION / ●IDLE / CAL…  algo  │  header 0–26
├──────────────────────────────────────────────┤
│ 0.5┐                              mv:0.32    │
│    │      ╭─╮        ← Movement curve (cyan)  │
│    │     ╱   ╲╭─╮      over-threshold in red  │
│ thr├─ ─ ─ ─ ─ ─ ─ ─ ─  ← Threshold dash (yellow)│
│ 0.0└──────────────────────────  thr:0.25     │
├──────────────────────────────────────────────┤  y=156
│  Mv 0.32   Thr 0.25   CAL/OK   RSSI -55dBm   │  values 156–186
├────────────┬──────────────────┬──────────────┤
│  THR −0.05 │    ⟳ CALIBRATE   │   THR +0.05  │  touch buttons 190–240
└────────────┴──────────────────┴──────────────┘  y=240
```

- **Graph area** (y 28–154): a 272-point ring buffer, one point per published Movement value (~4 points/s at the default 250 ms evaluation interval, one screen ≈ 68 s of history); Y axis auto-scales to `max(2×threshold, history peak, 0.5)`; the threshold is drawn as a yellow dashed line; curve segments above threshold are red, others cyan.
- **Header**: IP address while connected ("NO WIFI" otherwise), MOTION = red / IDLE = green / BOOT = yellow / CALIBRATING = flashing blue; algorithm (lightweight/high_accuracy) on the right.
- **Values row**: current Movement, Threshold (large font), WiFi RSSI.
- **Buttons row**: three touch zones (touchscreen binary_sensor), see §4.

## 4. Interaction

| Touch button | Action |
|---|---|
| `THR −0.05` | `threshold_number` call −0.05, takes effect immediately (clamped 0.0–1.0) |
| `THR +0.05` | same, +0.05 |
| `⟳ CALIBRATE` | `button.press` on `recalibrate_button` → re-calibration (~10 s at the default 1 s × 10 windows; keep the environment still). `calibration_active_sensor` turns on during calibration and off when done |

Refresh strategy: `display` is set to `update_interval: never`; a single 1 s `interval` (main-loop context) handles all redraws. **All** ESPectre entity callbacks may fire from non-main-loop tasks, so **none of them may call `component.update` on the display** — that would race the XPT2046 touchscreen's SPI polling on the same bus and trigger `spi_master: Cannot acquire bus when a polling transaction is in progress` assertion reboots (observed twice in practice, then fixed). The graph lambda de-duplicates curve points, so the fixed cadence loses nothing; touch-button feedback appears on the next redraw.

## 5. Deliverables

| File | Description |
|---|---|
| `src/cpp/frontend/esphome/examples/espectre-cyd-c5.yaml` | Complete NM-CYD-C5 config (extends `espectre-c5.yaml`, with display + touch + buttons) |
| `docs/NM-CYD-C5/DISPLAY-NM-CYD-C5.md` | This design document |
| `docs/NM-CYD-C5/USAGE-NM-CYD-C5.md` / `USAGE-NM-CYD-C5_zh.md` | English / Chinese usage guides |

## 6. Optional future extensions (require component changes)

1. **Calibration progress**: expose a collected/target packet count to draw a real progress bar (currently only "calibrating").
2. **Subcarrier visualization**: calibrated subcarrier indices could be drawn as a mini bar chart (diagnostics page).
3. **RGB LED** (GPIO27): red breathing light on MOTION, as a long-range indicator beyond the screen.
4. **Backlight power saving**: turn the backlight off after N minutes without motion, wake on touch.
