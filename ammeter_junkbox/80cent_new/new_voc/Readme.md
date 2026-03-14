# new_voc - Advanced Open Circuit Voltage Estimation

This directory contains experimental code for advanced Open Circuit Voltage (Voc) and internal resistance (Rint) estimation using non-linear curve fitting (RC profiling).

## Key Innovations

- **RC Profile Fitting**: Instead of just waiting for the voltage to settle after disconnecting the load, these sketches sample the voltage rise curve and use mathematical models to predict the final Voc and the panel's internal resistance.
- **Gauss-Newton Optimization**: `3knownC5.ino` implements the Gauss-Newton algorithm for non-linear least squares fitting of the RC charging curve.
- **Adaptive Calibration**: The system detects divergence between short periodic checks and full calibrations to dynamically adjust how often it needs to perform a full measurement.
- **Impedance Modeling**: Includes temperature-compensated models for inductors and MOSFETs to improve the accuracy of the power calculations.

## Files

- **1.ino**: Basic starting point for Voc sampling.
- **2Pluspartial.ino**: Explores partial sampling of the RC curve.
- **3knownC.ino** through **3knownC5.ino**: Iterative improvements on the RC fitting algorithm, with `3knownC5.ino` being the most advanced version featuring Gauss-Newton optimization and robust error handling.

## Mathematical Model

The core model used for fitting is:
`V(t) = Voc * (1 - exp(-t / tau))`
where `tau = Rint * C_in`. By fitting `Voc` and `tau` to the measured voltage samples, the system can estimate both the open circuit voltage and the internal resistance of the solar panel.
