# 3knownC_new - Advanced Verification and Optimization

This directory contains refinements of the "Known Capacitance" (3knownC) method, focusing on verification techniques and handling oscillations.

## Key Features

- **Oscillation-Based Verification**: `oscillating.ino` and `oscillating2.ino` introduce a method to verify the estimated `Voc` by applying small PWM oscillations and observing if the panel's response matches the expected RC behavior. This allows for drift detection without fully disconnecting the load.
- **Improved Fitting Logic**: Various `.cpp` and `.ino` files (e.g., `fit_early_fix.cpp`, `fit_early_fix6.ino`) explore optimized versions of the RC fitting algorithm, likely addressing edge cases or improving convergence speed.
- **Impulse Response**: `impulse.ino` may be exploring the panel's response to single PWM impulses for faster parameter estimation.
- **Loss Correction**: `loss_fix.cpp` focuses on refining the power loss models in the converter to improve MPP tracking accuracy.

## Files

- **3knownC_new.ino**: The baseline implementation for this series.
- **80_only.ino**: A simplified version focusing strictly on the 80% Voc rule.
- **oscillating.ino** / **oscillating2.ino**: Implement oscillation-based Voc verification.
- **fit_early_fix[1-6]**: Various iterations and fixes for the RC curve fitting algorithm.
