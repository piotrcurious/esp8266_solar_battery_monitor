# 3knownC - MPPT with Known Capacitance RC Fitting

This directory implements an advanced MPPT technique that uses the known input capacitance of the system to estimate the solar panel's internal resistance (Rint) and Open Circuit Voltage (Voc).

## Core Concept

By observing the voltage charging curve of the input capacitor when the load is disconnected, the system can fit an RC model:
`V(t) = Voc - B * exp(-t / tau)`
where `tau = Rint * C_known`. Since `C_known` is a fixed hardware parameter, `Rint` can be directly calculated from the fitted `tau`.

## Features

- **RC Curve Fitting**: Uses Gradient Descent to fit the exponential charging curve.
- **Adaptive Calibration**: Adjusts the frequency of full calibrations based on the divergence detected in partial measurements.
- **R_tau Estimation**: Provides a second estimate of internal resistance based on the time constant of the voltage rise.

## Subdirectories

- **upgreyd/**: Contains major algorithm improvements, including model-based Rint calibration and Gauss-Newton optimization.

## Files

- **3knownC.ino**: The primary implementation of the known capacitance MPPT algorithm.
