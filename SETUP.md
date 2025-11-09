# 🛜 ESPectre 👻 - Setup Guide

---

## What You Need

---

**Hardware:**
- **ESP32-S3** (recommended): ESP32-S3-DevKitC-1 N16R8 (16MB Flash, 8MB PSRAM)
- **ESP32-C6**: Any ESP32-C6 development board
- **ESP32 (classic)**: ESP32-D0WD or ESP32-WROOM-32 (without PSRAM)
- USB-C or Micro-USB cable (depending on board)
- Wi-Fi router (2.4 GHz)

**Software:**
- ESP-IDF v6.1
- MQTT Broker (Mosquitto or Home Assistant)

---

## Installation

---

### 1. Install ESP-IDF

**Linux:**
```bash
sudo apt-get install git wget flex bison gperf python3 python3-pip \
  python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && git checkout v6.1
./install.sh esp32s3
. ./export.sh
```

**macOS:**
```bash
brew install cmake ninja dfu-util python3

git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && git checkout v6.1
./install.sh esp32s3
. ./export.sh
```

**Windows:**
Download [ESP-IDF Windows Installer](https://dl.espressif.com/dl/esp-idf/)

### 2. Clone and Configure

```bash
git clone https://github.com/francescopace/espectre.git
cd espectre

# IMPORTANT: Copy the correct configuration for your ESP32 chip
# This ensures all critical settings are properly applied

# Clean any previous build and configuration
idf.py fullclean
rm -f sdkconfig

# For ESP32-S3 (recommended, with PSRAM):
cp sdkconfig.defaults.esp32s3 sdkconfig.defaults

# For ESP32-C6 (simplified CSI, no PSRAM):
cp sdkconfig.defaults.esp32c6 sdkconfig.defaults

# For ESP32 classic (no PSRAM):
cp sdkconfig.defaults.esp32 sdkconfig.defaults

# Configure Wi-Fi and MQTT
idf.py menuconfig
```

**Target-specific configurations:**
- **ESP32-S3**: Uses advanced CSI features (LTF configuration), supports PSRAM for larger buffers (8MB)
- **ESP32-C6**: Uses simplified CSI API (basic configuration only), reduced memory buffers, no PSRAM
- **ESP32 (classic)**: Uses advanced CSI features, reduced memory buffers, no PSRAM

**Note:** By copying the target-specific file to `sdkconfig.defaults`, you ensure that all critical configurations (like `CONFIG_ESP_WIFI_CSI_ENABLED` and `CONFIG_SPIRAM`) are properly applied during the build process. The target will be automatically detected from the configuration file.

In menuconfig:
- Go to **ESPectre Configuration**
- Set Wi-Fi SSID and password
- Set MQTT Broker URI (e.g., `mqtt://homeassistant.local:1883`)
- Set MQTT Topic (e.g., `home/espectre/node1`)
- Save with `S`, quit with `Q`

### 3. Build and Flash

```bash
# Build the project
idf.py build

# Flash to device
# Linux
idf.py -p /dev/ttyUSB0 flash

# macOS (find port with: ls /dev/cu.*)
idf.py -p /dev/cu.usbmodem* flash

# Windows
idf.py -p COM3 flash

# Monitor serial output (optional but recommended)
idf.py monitor
```

**If flash fails:** Hold BOOT button, press RESET, release BOOT, then run flash command.

**Exit monitor:** Ctrl+]

---

## Home Assistant

---

### MQTT Sensor

Add to `configuration.yaml`:

```yaml
mqtt:
  sensor:
    - name: "Movement Sensor"
      state_topic: "home/espectre/node1"
      unit_of_measurement: "intensity"
      icon: mdi:motion-sensor
      value_template: "{{ value_json.movement }}"
      json_attributes_topic: "home/espectre/node1"
      json_attributes_template: "{{ value_json | tojson }}"
```

### Automation Example

```yaml
automation:
  - alias: "Movement Alert"
    trigger:
      - platform: numeric_state
        entity_id: sensor.movement_sensor
        above: 0.6
    condition:
      - condition: template
        value_template: "{{ state_attr('sensor.movement_sensor', 'confidence') > 0.7 }}"
    action:
      - service: notify.mobile_app
        data:
          message: "Movement detected!"
```

---

## MQTT Messages

---

**Message format:**
```json
{
  "movement": 0.75,
  "confidence": 0.85,
  "state": "detected",
  "threshold": 0.40,
  "timestamp": 1730066405
}
```

**Fields:**
- `movement`: 0.0-1.0 (intensity)
- `confidence`: 0.0-1.0 (detection confidence)
- `state`: "idle" or "detected"
- `threshold`: current detection threshold
- `timestamp`: Unix timestamp

---

## Calibration & Tuning

---

After installation, follow the **[Calibration & Tuning Guide](CALIBRATION.md)** to:
- Calibrate the sensor for your environment
- Optimize detection parameters
- Troubleshoot common issues
- Configure advanced features

---

### MQTT Commands Reference

---

ESPectre provides an **interactive CLI tool** (`espectre-cli.sh`) for easy configuration and calibration. The CLI automatically handles MQTT communication and provides real-time feedback.

**Quick Start with CLI:**
```bash
# Launch interactive mode
./espectre-cli.sh

# In the interactive session, type commands directly:
espectre> info          # Get current configuration
espectre> stats         # Show statistics
espectre> threshold 0.4 # Set detection threshold
espectre> help          # Show all commands
espectre> exit          # Exit CLI
```

For detailed calibration and tuning instructions, see **[CALIBRATION.md](CALIBRATION.md)**.

#### Direct MQTT Commands (for scripting/automation)

All commands are sent to topic: `<your_topic>/cmd` (e.g., `home/espectre/kitchen/cmd`)  
Responses are published to: `<your_topic>/response`

**Command format:**
```json
{
  "cmd": "command_name",
  "value": value_or_parameter
}
```

**Available commands:**

| Command | Parameter | Description | Example |
|---------|-----------|-------------|---------|
| `threshold` | float (0.0-1.0) | Set detection threshold | `{"cmd": "threshold", "value": 0.40}` |
| `debounce` | int (1-10) | Set consecutive detections needed | `{"cmd": "debounce", "value": 3}` |
| `persistence` | int (1-30) | Timeout in seconds before downgrading state | `{"cmd": "persistence", "value": 3}` |
| `hysteresis` | float (0.1-1.0) | Ratio for threshold hysteresis | `{"cmd": "hysteresis", "value": 0.7}` |
| `variance_scale` | float (100-2000) | Variance normalization scale | `{"cmd": "variance_scale", "value": 400}` |
| `butterworth_filter` | bool | Enable/disable Butterworth low-pass filter (8Hz cutoff) | `{"cmd": "butterworth_filter", "enabled": true}` |
| `wavelet_filter` | bool | Enable/disable Wavelet db4 filter (low-freq noise) | `{"cmd": "wavelet_filter", "enabled": true}` |
| `wavelet_level` | int (1-3) | Wavelet decomposition level (3=max denoising) | `{"cmd": "wavelet_level", "value": 3}` |
| `wavelet_threshold` | float (0.5-2.0) | Wavelet noise threshold (1.0=balanced) | `{"cmd": "wavelet_threshold", "value": 1.0}` |
| `hampel_filter` | bool | Enable/disable Hampel outlier filter | `{"cmd": "hampel_filter", "enabled": true}` |
| `hampel_threshold` | float (1.0-10.0) | Hampel filter sensitivity | `{"cmd": "hampel_threshold", "value": 2.0}` |
| `savgol_filter` | bool | Enable/disable Savitzky-Golay smoothing | `{"cmd": "savgol_filter", "enabled": true}` |
| `info` | none | Get current configuration (includes network, MQTT topics, filters, etc.) | `{"cmd": "info"}` |
| `stats` | none | Get detection statistics | `{"cmd": "stats"}` |
| `analyze` | none | Analyze data and get recommended threshold | `{"cmd": "analyze"}` |
| `features` | none | Get all CSI features with calibration info (selected features show weight) | `{"cmd": "features"}` |
| `logs` | bool | Enable/disable CSI logging | `{"cmd": "logs", "enabled": true}` |
| `calibrate` | action + duration | Automatic calibration (start/stop/status) | `{"cmd": "calibrate", "action": "start", "duration": 60}` |
| `smart_publishing` | bool | Enable/disable smart publishing (reduces MQTT traffic) | `{"cmd": "smart_publishing", "enabled": true}` |
| `adaptive_normalizer` | bool | Enable/disable adaptive normalizer filter | `{"cmd": "adaptive_normalizer", "enabled": true}` |
| `adaptive_normalizer_alpha` | float (0.001-0.1) | Set normalizer learning rate (lower = slower adaptation) | `{"cmd": "adaptive_normalizer_alpha", "value": 0.01}` |
| `adaptive_normalizer_reset_timeout` | int (0-300) | Auto-reset timeout in seconds (0 = disabled) | `{"cmd": "adaptive_normalizer_reset_timeout", "value": 30}` |
| `adaptive_normalizer_stats` | none | Get normalizer statistics (mean, variance, stddev) | `{"cmd": "adaptive_normalizer_stats"}` |
| `traffic_generator_rate` | int (0-50) | Set WiFi traffic rate for continuous CSI (0=disabled, recommended: 15 pps) | `{"cmd": "traffic_generator_rate", "value": 15}` |
| `factory_reset` | none | Restore all settings to factory defaults | `{"cmd": "factory_reset"}` |

**Note:** Feature weights are automatically optimized through the calibration system. Manual weight adjustment has been removed. Use `./espectre-cli.sh calibrate start` to optimize weights for your environment.

#### Info Command Response Structure

The `info` command returns a comprehensive JSON object organized into logical groups:

```json
{
  "network": {
    "ip_address": "192.168.1.100"
  },
  "mqtt": {
    "base_topic": "home/espectre/node1",
    "cmd_topic": "home/espectre/node1/cmd",
    "response_topic": "home/espectre/node1/response"
  },
  "detection": {
    "threshold": 0.4,
    "debounce": 3,
    "persistence_timeout": 5,
    "hysteresis_ratio": 0.8,
    "variance_scale": 500.0
  },
  "filters": {
    "butterworth_enabled": true,
    "wavelet_enabled": false,
    "wavelet_level": 3,
    "wavelet_threshold": 1.0,
    "hampel_enabled": false,
    "hampel_threshold": 3.0,
    "savgol_enabled": false,
    "savgol_window_size": 5,
    "adaptive_normalizer_enabled": true,
    "adaptive_normalizer_alpha": 0.01,
    "adaptive_normalizer_reset_timeout_sec": 60
  },
  "features": {
    "csi_logs_enabled": true,
    "smart_publishing_enabled": true
  }
}
```

**Groups:**
- **`network`**: Network information (IP address for ping/diagnostics)
- **`mqtt`**: MQTT topic configuration
- **`detection`**: Core detection parameters
- **`filters`**: Signal processing filters status
- **`features`**: General capabilities and features

### Factory Reset

Restore all settings to factory defaults and clear all saved data from NVS:

**Via MQTT:**
```bash
mosquitto_pub -h homeassistant.local -t "home/espectre/node1/cmd" \
  -m '{"cmd":"factory_reset"}'
```
**Or use the interactive CLI** (see [CALIBRATION.md](CALIBRATION.md) for details).

**This will:**
- ✅ Clear all saved calibration data from NVS
- ✅ Clear all saved configuration parameters from NVS
- ✅ Restore all parameters to factory defaults
- ✅ Reinitialize the calibration system
- ⚠️ You will need to recalibrate after reset

**When to use:**
- Configuration is corrupted or inconsistent
- Want to start fresh with default settings
- Testing different configurations
- Troubleshooting persistent issues

**Example using mosquitto_pub:**
```bash
# Set threshold
mosquitto_pub -h homeassistant.local -t "home/espectre/kitchen/cmd" \
  -m '{"cmd": "threshold", "value": 0.35}'

# Get statistics
mosquitto_pub -h homeassistant.local -t "home/espectre/kitchen/cmd" \
  -m '{"cmd": "stats"}'

# Listen for response
mosquitto_sub -h homeassistant.local -t "home/espectre/kitchen/response"
```

---

### Traffic Generator

---

**⚠️ IMPORTANT:** ESPectre requires continuous WiFi traffic to receive CSI packets. Without traffic, the ESP32 receives few/no CSI packets, resulting in poor detection and failed calibration.

**What it does:**
- Generates UDP broadcast packets at configurable rate (default: 15 packets/sec)
- Ensures continuous CSI data availability
- Essential for reliable detection and calibration

**Why it's needed:**
- ESP32 only receives CSI when there's WiFi traffic
- Without traffic generator: 10-20 samples during 60s calibration ❌
- With traffic generator: 100+ samples during 60s calibration ✅

**Configuration:**
```bash
# Via CLI
traffic_generator_rate 15  # Enable 15 pps

# Via MQTT
{"cmd":"traffic_generator_rate","value":15}
```

**Recommended rates:**
- **15 pps**: Good balance (default)
- **10 pps**: Minimal overhead
- **20 pps**: Busy environments with interference
- **0 pps**: Disabled (only if you have other continuous WiFi traffic)

**Note:** Rate is constant during both calibration and normal operation to ensure consistency.

**Troubleshooting:**

Verify traffic generator is working:
```bash
# Or use tcpdump for detailed analysis
sudo tcpdump -i en0 -n udp port 12345
```

If no packets appear:
- ✅ Check ESP32 serial monitor for traffic generator errors
- ✅ Verify ESP32 and Mac are on same network
- ✅ Try increasing rate: `traffic_generator_rate 20`

**Enable via MQTT:**
```bash
# Enable wavelet filter
mosquitto_pub -h localhost -t "espectre/cmd" -m '{"cmd":"wavelet_filter","enabled":true}'

# Set decomposition level (1-3, recommended: 3)
mosquitto_pub -h localhost -t "espectre/cmd" -m '{"cmd":"wavelet_level","value":3}'

# Set threshold (0.5-2.0, recommended: 1.0)
mosquitto_pub -h localhost -t "espectre/cmd" -m '{"cmd":"wavelet_threshold","value":1.0}'
```

**Or use the interactive CLI:**
```bash
./espectre-cli.sh
> wv on
> wvl 3
> wvt 1.0
```

**Performance impact:**
- CPU: ~5-8% additional load
- RAM: ~2 KB
- Latency: 320ms warm-up (32 samples)

---

### Smart Publishing

---

**How it works:**
- Publishes immediately when detection state changes (idle ↔ detected)
- Publishes when movement score changes by more than 0.05 (5%)
- Publishes a heartbeat every 5 seconds even if nothing changed
- Skips redundant messages when values are stable

**Benefits:**
- 📉 Reduces MQTT bandwidth by 50-70% during stable periods
- 🔋 Lower network overhead and power consumption
- 📊 Still provides timely updates for state changes
- 💓 Regular heartbeat ensures system is alive

**Default:** Disabled (publishes every detection cycle)

**Enable/disable via MQTT:**
```bash
mosquitto_pub -h homeassistant.local -t "home/espectre/node1/cmd" \
  -m '{"cmd":"smart_publishing","enabled":true}'
```

**Or use the interactive CLI** (see [CALIBRATION.md](CALIBRATION.md) for details).

**When to use:**
- ✅ High-traffic MQTT brokers
- ✅ Battery-powered or low-bandwidth scenarios
- ✅ Multiple ESPectre sensors on same network
- ✅ Home Assistant with many sensors

**When to disable:**
- ❌ Need every single data point for analysis
- ❌ Real-time graphing/monitoring
- ❌ Custom integrations expecting constant updates

---

## Troubleshooting

---

### Wi-Fi Connection Failed

**Check serial monitor:**
```bash
idf.py monitor
```

Look for connection errors and verify:
- ✅ Correct SSID and password in menuconfig
- ✅ Router is broadcasting 2.4GHz network
- ✅ ESP32 is within range of router

### MQTT Not Publishing

**Verify MQTT broker is accessible:**
```bash
mosquitto_sub -h homeassistant.local -t "home/espectre/node1" -v
```

If no messages appear:
- ✅ Check MQTT broker URI in menuconfig
- ✅ Verify MQTT credentials (username/password)
- ✅ Ensure port 1883 is open (or 8883 for TLS)
- ✅ Check serial monitor for MQTT connection errors

#### Flash Failed

**Solution:**
1. Hold BOOT button on ESP32
2. Press RESET button
3. Release BOOT button
4. Run flash command again

---

### Getting Help

---

For detection issues, calibration problems, or advanced troubleshooting:
- 📖 **See**: [Calibration & Tuning Guide](CALIBRATION.md)
- 📝 **GitHub Issues**: [Report problems](https://github.com/francescopace/espectre/issues)
- 📧 **Email**: francesco.pace@gmail.com
