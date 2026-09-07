# ESPectre × NM-CYD-C5 使用说明

> 适用固件：`src/cpp/frontend/esphome/examples/espectre-cyd-c5.yaml`（基于 `espectre-c5.yaml`）
> 适用硬件：NM-CYD-C5（推荐外置天线版本，nm-cyd-c5-ant；ESP32-C5，2.8 寸 320×240 ST7789 触摸屏）

ESPectre 是基于 WiFi CSI（信道状态信息）的人体运动检测系统。本固件在 NM-CYD-C5 上除实现 ESPectre 的人体运动检测功能外，还提供完整的本地交互能力：实时运动曲线、门限线显示、触摸门限手动调节按键、一键校准，同时支持 Home Assistant 和网页工具。

---

## 1. 功能一览

| 功能 | 入口 |
|---|---|
| 运动检测（lightweight / high_accuracy 算法） | 自动运行，WiFi CSI 被动感知，无需佩戴任何设备 |
| Movement 实时曲线 + Threshold 门限线 | 设备屏幕（320×240） |
| 触摸调节门限（±0.05，范围 0–1） | 屏幕按钮 / HA |
| 自动校准（自适应门限） | 屏幕 CALIBRATE 按钮 / HA Recalibrate 按钮 |
| Home Assistant 集成 | 原生 ESPHome API（自动发现） |
| 网页工具（实体查看/控制、设备设置） | https://espectre.dev/tools/ |

## 2. 首次配网

设备烧录固件后首次启动时未保存任何 WiFi 凭据，会自动开启配网热点：

1. 用手机或电脑扫描 WiFi，连接名为 **`ESPectre Fallback`** 的热点；
2. 浏览器会自动弹出配网页面（未弹出则手动访问 `192.168.4.1`）；
3. 选择你当前环境的 WiFi 并输入密码 —— **建议使用 2.4 GHz 网络**（ESPectre 的 CSI 检测工作在 2.4 GHz 频段）；
4. 配置成功后设备自动重启并连接 WiFi；
5. 连接成功后，**屏幕左上角标题栏会直接显示设备 IP 地址**（未连接时显示黄色 `NO WIFI`）。

> 也可以通过 USB 使用 Improv Serial 配网：`./espectre provision --ssid MyNetwork`，或使用 https://espectre.dev/tools/flash/ 网页烧录器。

## 3. 添加到 Home Assistant

前提：设备与 Home Assistant 在**同一网段**。

1. 在 Home Assistant 中安装 **ESPHome** 集成（设置 → 设备与服务 → 添加集成 → ESPHome）；
2. 打开 Home Assistant → **Settings（设置）** → 找到 **ESPHome** → 点击 **Add New Device（添加新设备）**；
3. 设备与 HA 在同一网络时会**自动被发现**（mDNS 主机名 `espectre`），点击确认即可完成添加；未自动发现时手动输入屏幕上的 IP 地址添加。

添加成功后的主要实体：

| 实体 | 类型 | 说明 |
|---|---|---|
| Movement Score | sensor | 运动分数（曲线数据源） |
| Motion Detected | binary_sensor | 运动状态（可用于自动化触发） |
| Threshold | number | 门限，归一化范围 0–1（与屏幕按钮联动） |
| Calibration Active | binary_sensor | 校准进行中为 ON |
| Recalibrate | button | 触发重新校准 |
| WiFi Signal | sensor | 信号强度 dBm |

## 4. 屏幕界面与操作

```
┌──────────────────────────────────────────────┐
│ 192.168.1.100   MOTION/IDLE/CAL…  lightweight│ 标题栏：IP / 状态 / 算法
├──────────────────────────────────────────────┤
│        ╭─╮        Movement 曲线（青）         │
│       ╱   ╲╭─╮    超门限段变红               │
│  - - - - - - - -  Threshold 门限虚线（黄）    │
├──────────────────────────────────────────────┤
│  0.32 mv   0.25 thr                 -55dBm   │ 数值栏
├────────────┬──────────────────┬──────────────┤
│  THR −0.05 │    CALIBRATE     │   THR +0.05  │ 触摸按钮
└────────────┴──────────────────┴──────────────┘
```

- **曲线区**：约 68 秒滚动历史（约 4 点/秒），纵轴自动缩放；曲线超过黄色门限线的部分变红，直观呈现"判决"过程。
- **THR −0.05 / +0.05**：步进调节门限，范围 0–1，实时生效。
- **CALIBRATE**：触发重新校准（约 10 秒，期间请保持环境静止）。校准会重算自适应门限，校准期间 `Calibration Active` 为 ON。
- 状态文字：`MOTION`（红）/ `IDLE`（绿）/ `CALIBRATING...`（蓝）/ `BOOT`（黄）。

## 5. 网页工具

打开 https://espectre.dev/tools/device-settings/ 并指向设备（按 IP 或 `espectre.local`）即可查看和控制设备。设备同时提供 ESPectre Direct HTTP API（见 `docs/DISCOVERY.md`）。

固件更新：使用 ESPHome 仪表盘或 `esphome upload`（WiFi 直连，无需 USB）。

## 6. 日常使用建议

- **安装位置**：设备与路由器之间无金属遮挡，CSI 对 2.4 GHz 链路质量敏感；
- **何时校准**：设备挪动位置、环境布局变化、误报/漏报明显时，按 CALIBRATE 重新校准（校准时保持房间无人走动）；或通过 Home Assistant 触发 Recalibrate 按钮；
- **门限调节原则**：误报多→调大；漏报多→调小。小幅动作检测建议先重新校准再微调；
- **断网自愈**：WiFi 断开后设备自动重连，CSI 与检测自动恢复，无需干预。

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| 屏幕显示 `NO WIFI` | 未连网：连接 `ESPectre Fallback` 热点重新配网，使用 2.4 GHz 网络 |
| HA 搜不到设备 | 检查同网段/VLAN；或手动输入屏幕上的 IP 添加 |
| 触摸按钮无反应/偏移 | 电阻屏个体差异，微调 YAML 中 `touchscreen.calibration` 四个边界值 |
| 校准失败或不理想 | 校准时保持环境绝对静止；信号过强（离路由器极近）时增益锁会跳过，属预期 |
| 固件更新 | ESPHome 仪表盘或 `esphome upload`（WiFi 直连，无需 USB） |
