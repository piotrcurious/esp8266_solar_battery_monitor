# rint2_1_stm8 - STM8 Internal Resistance Estimation

This directory contains a specific implementation of the internal resistance (Rint) estimation algorithm for the STM8 microcontroller.

## Features

- **STM8 Native**: Tailored for the STM8 architecture, focusing on low-level PWM and ADC interactions.
- **Rint Tracking**: Periodically disconnects the load to measure Voc and then uses a calibration phase to estimate the panel's internal resistance.
- **80% Rule**: Implements the standard 80% Voc target for solar power optimization.

## Files

- **rint2_1_stm8.ino**: The primary implementation for the STM8 platform.
