# JK BMS CAN Support for Battery-Emulator

## Overview

This adds native CAN bus support for **JK BMS** battery management systems, enabling the T-2CAN (or any supported ESP32 CAN board) to read JK BMS data over CAN and translate it to inverter protocols (SMA, Victron, Growatt, etc.).

Three JK BMS battery profiles are now available in the dropdown:

| Battery Type | Protocol | Speed | Interface |
|---|---|---|---|
| **JK BMS RS485 (LFP)** | UART/RS485 (existing) | — | RS485 / Modbus |
| **JK BMS CAN (B/BD Native)** | Young Power native CAN | 250 kbps | CAN A / CAN B |
| **JK BMS PB CAN** | Pylontech LV emulation | 500 kbps | CAN A / CAN B |

## JK BMS PB CAN (Recommended for PB Series)

The PB series JK BMS can be set to "Pylontech" mode in the JK app. In this mode it broadcasts rich telemetry at 500 kbps using the Pylontech LV CAN protocol.

### CAN Frames Parsed

| CAN ID | Content | Update Rate |
|---|---|---|
| `0x351` | Charge Voltage Limit (CVL), Charge Current Limit (CCL), Discharge Current Limit (DCL), Discharge Voltage Limit (DVL) | 1000ms |
| `0x355` | SOC %, SOH % | 1000ms |
| `0x356` | Pack Voltage (0.01V), Current (0.1A), Temperature (0.1°C) | 1000ms |
| `0x359` | Protection flags (OVP, UVP, OCP, OTP) + Warning flags | 1000ms |
| `0x35C` | BMS model, HW/FW version, **Force charge/discharge flags** | 1000ms |
| `0x35E` | Manufacturer name ("PYLON") | 1000ms |

### Key Features
- **BMS-reported charge/discharge limits** — no SOC-based guessing; the BMS sends real CCL/DCL based on cell conditions, temperature, and imbalance
- **Dynamic voltage limits** — CVL/DVL from the BMS override static user settings
- **SOH reporting** — real state-of-health from the BMS
- **Force charge support** — when SOC is critically low, the BMS requests emergency charging and the T-2CAN ensures a minimum 500W charge power is available
- **Protection integration** — OVP/UVP/OCP/OTP flags immediately zero out charge or discharge power

## JK BMS CAN (B/BD Native)

For B-series and BD-series JK BMS units using the proprietary Young Power CAN protocol at 250 kbps.

### CAN Frames Parsed

| CAN ID | Content | Update Rate |
|---|---|---|
| `0x02F4` | Pack Voltage, Current (with -400A offset), SOC, Discharge Time | 20ms |
| `0x04F4` | Max/Min Cell Voltage + Cell Position | 100ms |
| `0x05F4` | Max/Min/Avg Temperature + Probe Position | 100ms |
| `0x07F4` | 15 Alarm Types × 3 Severity Levels (30-bit packed) | Event-triggered |

### Limitations vs PB CAN
- No SOH (hardcoded to 99%)
- No charge/discharge current limits from BMS (uses SOC-based power ramping)
- No charge/discharge voltage limits (uses static user settings)
- No individual cell voltages (only max/min with position)

## Enhanced Logging

A checkbox **"Enhanced JK BMS Logging"** appears in the battery settings when any JK BMS type is selected. When enabled, every incoming CAN frame is decoded and logged with human-readable values:

```
[JKPB] 0x351 CVL=56.0V CCL=50.0A DCL=50.0A DVL=44.0V
[JKPB] 0x355 SOC=85% SOH=99%
[JKPB] 0x356 V=52.14V I=12.3A T=25.0C
[JKPB] 0x359 PROT=0x00 FAULT=0x00 WARN=0x00
```

Logs are visible on the web console and USB serial. Requires reboot after toggling.

## Interface Filtering

The settings page automatically filters the interface dropdown based on battery type:
- JK BMS RS485 → only RS485/Modbus interfaces selectable
- JK BMS CAN variants → only CAN interfaces selectable (CAN A, CAN B, CAN FD)

## Setup Guide (PB Series)

1. In the **JK BMS app**: set CAN protocol to "Pylontech" mode
2. Wire CAN-H and CAN-L from the JK BMS to the T-2CAN (CAN A or CAN B)
3. In the **T-2CAN web UI**: select "JK BMS PB CAN" as battery type
4. Select the appropriate CAN interface (CAN A or CAN B)
5. Set your pack's total capacity in Wh (the BMS doesn't broadcast this)
6. Optionally enable "Enhanced JK BMS Logging" for diagnostics
7. Save and reboot

## Files Changed

### New Files
- `Software/src/battery/JK-BMS-CAN.h` / `.cpp` — B/BD native protocol (250 kbps)
- `Software/src/battery/JK-BMS-PB-CAN.h` / `.cpp` — PB Pylontech emulation (500 kbps)

### Modified Files
- `Software/src/battery/Battery.h` — added `JkBmsCan` and `JkBmsPbCan` enum values
- `Software/src/battery/BATTERIES.h` — added includes
- `Software/src/battery/BATTERIES.cpp` — added name mappings and factory constructors
- `Software/src/devboard/webserver/settings_html.cpp` — added JK logging checkbox, interface filtering JS
- `Software/src/devboard/webserver/webserver.cpp` — added JKLOG bool setting
