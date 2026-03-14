# rint_est_2_avr_pwm_mini - AVR Mini-PWM Rint Estimation

This directory contains a specialized implementation of the internal resistance (Rint) estimation algorithm for AVR platforms, utilizing the `AVR_PWM` library for fine-grained control over the switching frequency and duty cycle.

## Features

- **High-Frequency PWM**: Uses `AVR_PWM` to achieve switching frequencies up to 14kHz, reducing ripple and improving measurement stability.
- **Gradient Descent Calibration**: Implements a gradient descent-based calibration loop to estimate `internal_resistance_src` and `internal_resistance_load`.
- **Target Voltage Regulation**: Aims to maintain the panel voltage at 80% of its measured Voc (Open Circuit Voltage).
- **Corrected Voltage Calculation**: Uses the estimated internal resistance and input current to calculate a "corrected" source voltage for more accurate MPPT.

## Subdirectories

- **refactor/**: Contains various refactored versions of the main sketch, testing different structural improvements and optimizations.

## Files

- **rint_est_2_avr_pwm_mini.ino**: The main implementation file.
