# Version History and Evolution (Changelog)

This document tracks the evolution of the ESP8266/ESP32 Solar Battery Monitor project.

## [v0.1] - Basic Proof of Concept
- **Sender:** `sender/sender.ino`
- Initial implementation of UDP broadcast.
- Simple string-based or raw integer telemetry.
- No power saving.

## [v0.2] - Struct-based Telemetry
- **Sender:** `sender_low_power/`
- **Receiver:** `reciever_struct_float/`
- Introduced `telemetry_frame` struct for efficient binary data transmission.
- Added basic OLED display support.

## [v0.3] - Low Power & Basic Filtering
- **Sender:** `sender_low_power_2/`, `sender_low_power_3/`
- Improved sleep cycles.
- Introduction of basic smoothing for ADC readings.

## [v0.4] - Advanced Filtering (SLP Series)
- **Sender:** `SLP4/`
- Implementation of `SensorFilter` class.
- Combined outlier detection, rolling average, and Kalman filtering.
- "Smart Low Power" (SLP) logic: checks for AP presence before full wake-up.

## [v0.5] - BlueDisplay Integration
- **Receiver:** `bt_AP2/`
- Shifted to ESP32 for better performance and Bluetooth/WiFi coexistence.
- Integrated BlueDisplay for high-resolution graphing on Android devices.
- Implemented LFSR-based non-linear graph drawing to improve UI responsiveness.

## [v0.6] - Multi-Sensor Support (Humidity & Temperature)
- **Sender/Receiver:** `bt_AP2_multi_humidity/`, `SLP4_humidity/`
- Extended `telemetry_frame` to include SHT4x sensor data.
- Added support for tracking multiple sensor nodes.

## [v1.0] - Refinement and Modularization
- **Current State:** Refactored code in `SLP4` and `bt_AP2_multi_humidity_better_graph2`.
- Optimized power consumption down to ~17-20mA in idle.
- Added support for different display types (SH110X, SSD1306).
- Improved internal resistance estimation in `ammeter_junkbox`.

---

## Notable Experiments (Junkbox)
- **7s_battery:** Specialized code for multi-cell balance monitoring.
- **serial_to_udp:** Bridging wired serial telemetry to the wireless network.
- **gauges_junkbox:** Creative display layouts, including analog needles and multi-layout Progmems.
