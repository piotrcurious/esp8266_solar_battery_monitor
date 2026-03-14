# basic_mppt2_stm8 - STM8 MPPT with Priority

This directory contains the STM8 port of the MPPT logic, including priority-based multi-converter control.

## Features

- **STM8 Implementation**: Optimized for STM8S series microcontrollers.
- **Priority Control**: Manages multiple converters, ensuring the primary load is satisfied before allocating power to the secondary load.
- **VOC Correction**: Includes logic for correcting the Open Circuit Voltage measurement based on operating conditions.

## Subdirectories

- **prio/**: Contains "hack" versions and VOC correction experiments for the priority logic.

## Files

- **basic_mppt2_stm8.ino**: The main MPPT and priority control implementation for STM8.
