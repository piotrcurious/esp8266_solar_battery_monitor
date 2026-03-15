# Tested Hardware Implementations

This directory contains code that has been successfully tested on actual hardware. The primary hardware configuration involves controlling two DC-DC converters via their 'enable' or PWM pins to implement priority-based power distribution.

## Hardware Configuration
- **Converters**: Two DC-DC converters.
- **Priority**: One output is given higher priority (e.g., charging a battery), while the other captures surplus power (e.g., heating).
- **Control**: AI-generated logic (GPT-4o/Claude) manages the duty cycles to maintain the panel near its MPP.

## Subdirectories

- **basic_mppt/** & **basic_mppt2/**: Standard MPPT implementations for Arduino/AVR.
- **basic_mppt2_stm8/**: Port of the MPPT logic to STM8 microcontrollers.
- **3knownC/** & **3knownC_new/**: Advanced Voc and Rint estimation using the "Known Capacitance" RC fitting method.
- **3knownC_v4_hybrid/**: (v4.5.2) Advanced hybrid controller using Gradient Descent + Active Exploration Pulses for fast transient tracking and Stuck-at-OC recovery. Verified via co-simulation.
- **3knownC_emulator_eval/**: Co-simulation environment including Python physics emulator, system analyzer, and automated verification scripts.
- **rint2_1_stm8/** & **rint2_stm8_sduino/**: STM8-specific implementations of internal resistance estimation.
- **rint_est_2_avr_pwm_mini/**: Specialized Rint estimation for AVR platforms using mini-PWM configurations.
- **rint_est_refactor/**: Cleaned up and refactored versions of the resistance estimation algorithms.
