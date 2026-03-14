# rint2_stm8_sduino - STM8 Sduino Implementation

This directory contains ports of the internal resistance (Rint) estimation and MPPT logic specifically for the STM8 platform using the Sduino core.

## Features

- **STM8 Compatibility**: Adjusted pin definitions and PWM handling for STM8 microcontrollers.
- **Simplified Measurement Loop**: Follows the same logic as the AVR versions but optimized for the STM8's resource constraints.
- **Serial Feedback**: Provides debug information via UART for real-time monitoring of Voc, corrected voltage, and internal resistance.

## Files

- **rint2_stm8.ino**: Base port for STM8.
- **rint2_stm8_sduino.ino**: Specifically tailored for the Sduino environment on STM8.
