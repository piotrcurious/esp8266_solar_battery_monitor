# ESP8266/ESP32 Solar Battery Monitor System Documentation

## Overview
This project is a decentralized monitoring system for solar battery setups. It uses ESP8266 and ESP32 microcontrollers to measure telemetry data (primarily battery voltage) and broadcast it over a local WiFi network using UDP broadcast packets. Multiple receivers can listen to these packets and display the data or trigger actions based on the values received.

## System Architecture

The system consists of three main roles:

1.  **Senders (Sensors):** These nodes measure the battery voltage using their ADC pins, optionally apply filtering, and broadcast a `telemetry_frame` struct via UDP.
2.  **Access Point (AP):** A central node (can be one of the ESPs or a dedicated router) that provides the WiFi network. Using an ESP32 as an AP is recommended for better UDP performance.
3.  **Receivers (Displays/Controllers):** These nodes connect to the AP, listen for UDP broadcast packets, and process the telemetry data. Examples include OLED displays, BlueDisplay-enabled smartphones/tablets, or automated switches.

## Communication Protocol

Data is transmitted using UDP broadcast packets on a specific port (default 5683). The payload is a packed C struct to ensure cross-platform compatibility between ESP8266 and ESP32.

### Telemetry Frame
The standard `telemetry_frame` (found in `telemetry_frame.hpp` or `telemetry_frame.h`) has evolved over time:

- **v1 (Basic):** `voltage_ADC0` (float)
- **v2 (Extended):** Adds `wifi_rssi` (float)
- **v3 (Humidity):** Adds `radio_active_time`, `SH4x_rel_humidity`, and `SH4x_temperature`.

```cpp
#pragma pack(push,1)
struct telemetry_frame {
  float voltage_ADC0;
  float wifi_rssi;
  // Additional fields in newer versions
};
#pragma pack(pop)
```

## Core Components

### Senders
- **Basic `sender/`**: Simple proof-of-concept sending incrementing numbers or voltage strings.
- **`sender_low_power_x/`**: Versions implementing deep sleep and better power management.
- **`SLP4/`**: "Smart Low Power" version. Uses `SensorFilter` (Kalman + Outlier detection) and light sleep between iterations to minimize power consumption while maintaining accuracy.

### Receivers & Displays
- **`reciever_struct_float/`**: Basic receiver for ESP8266 using an OLED display (SSD1306/SH110X).
- **`bt_AP2/`**: Advanced ESP32 receiver that acts as a WiFi AP and an Android BlueDisplay bridge.
    - Features complex graphing using buffered lines and LFSR-based drawing to optimize performance.
    - Supports multiple telemetry fields (humidity, temperature) in `multi_humidity` versions.
- **`gauges_junkbox/`**: Various experiments with analog-style gauges and different display layouts.

### Specialized Modules
- **`7s_battery/`**: Specifically designed for monitoring 7-cell Li-ion packs, using IR or serial for communication.
- **`ammeter_junkbox/`**: Contains code for measuring current and estimating internal resistance (`rint_estimator`).

## Power Management
ESP8266/ESP32 are power-hungry (80mA+ active). This project explores:
- **Modem Sleep:** Disabling WiFi radio when not needed.
- **Light/Deep Sleep:** Reducing CPU power consumption between measurements.
- **Beacon Poking:** Checking for AP presence before attempting a full connection to save power when the AP is offline.

## Setup and Configuration
Most projects require a `wifi_settings.h` file containing:
- `ssid` and `password`
- `broadcast` IP address (e.g., `224.0.1.187` or `255.255.255.255`)
- `port` (e.g., `5683`)

## Known Issues and Limitations
- **ADC Accuracy:** Built-in ESP ADCs are notoriously non-linear. Use external ADCs for high-precision requirements.
- **UDP Reliability:** Broadcast packets are not guaranteed to arrive. The system is designed for periodic updates where occasional loss is acceptable.
- **Power Consumption:** Even with sleep modes, total system power can be significant. High-efficiency DC-DC converters are recommended.
