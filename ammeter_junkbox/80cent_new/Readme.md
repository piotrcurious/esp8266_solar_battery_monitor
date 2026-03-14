# 80cent_new - Advanced Solar MPPT and Internal Resistance Estimation

This directory contains experimental code for solar power optimization, specifically focusing on the "80% Voc" rule and advanced internal resistance (Rint) estimation techniques.

## Overview

The project aims to optimize solar energy harvest by maintaining the panel voltage near its Maximum Power Point (MPP), which is typically around 70-80% of its Open Circuit Voltage (Voc). The code explores various ways to estimate Voc and Rint without dedicated current sensors, using instead voltage measurements and PWM load control.

## Core Files in Root

- **basic_mppt2.ino**: A standard Perturb & Observe (P&O) MPPT implementation that periodically measures Voc by temporarily disconnecting the load.
- **basic_priority.ino**: Manages two DC-DC converters with different priorities. It attempts to maximize power output by first saturating the high-priority converter before using the second.
- **inference1.ino**: Implements current inference. Instead of measuring current, it calculates an inferred current based on the voltage drop from the measured Voc, allowing for power-based MPPT without a shunt.
- **inference2.ino**: An evolution of the inference model that includes moving average tracking of Voc trends to improve the accuracy of current estimation under changing light conditions.

## Subdirectories

- **tested/**: Code that has been verified on actual hardware.
- **infer2/** & **infer3/**: Iterative versions of current and Voc inference algorithms.
- **new_voc/**: Experimental methods for measuring and predicting Open Circuit Voltage.
- **current_sense/**: Implementations that utilize or simulate actual current sensing hardware (shunts).

## Hardware Setup

The system typically uses an Arduino-compatible microcontroller (like an Uno or STM8) to control the 'enable' or PWM pins of one or more DC-DC converters. It measures:
- Panel Voltage (V_src)
- Converter Output/Load Voltage (V_load)
- (Optional) Current via shunts in the `current_sense` versions.
