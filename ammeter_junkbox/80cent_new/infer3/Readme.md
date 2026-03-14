# infer3 - Display and Advanced Logging

This directory builds upon the inference algorithms developed in `infer2`, adding user interface elements and more structured control classes.

## Key Improvements

- **OLED Support**: `03_oled.ino` integrates an SSD1306 OLED display to provide real-time feedback on panel voltage, inferred current, duty cycles, and temperature.
- **Class-Based Architecture**: The code transitions from flat procedural sketches to object-oriented designs with classes like `SolarMPPTController` and `OLEDDisplay`.
- **Enhanced Filtering**: Uses `MovingAverage` library for smoother sensor readings.
- **Intelligent Load Balancing**: Refined logic for distributing power across multiple DC-DC converters.

## Files

- **infer0a1.ino**: A bridge from the `infer2` series.
- **02.ino**, **03.ino**: Intermediate steps in the refactoring process.
- **03_oled.ino**: The most complete version in this series, featuring a full OLED UI and structured controller logic.
