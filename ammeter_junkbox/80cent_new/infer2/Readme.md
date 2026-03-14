# infer2 - Iterative Current and Voc Inference Algorithms

This directory contains various iterations of algorithms designed to infer solar panel current and track Open Circuit Voltage (Voc) without dedicated sensors.

## Evolution of Algorithms

- **infer0a.ino**, **basic2.ino**, **infer3.ino**: Early experimental versions of the inference logic.
- **infer4_basic.ino**: Implements a priority-based system for two converters while maintaining a target voltage ratio (80% of Voc).
- **infer5_enh.ino**, **infer5cap.ino**: Enhanced versions, with `infer5cap` likely exploring the effects of input capacitance on voltage stabilization.
- **infer6.ino**, **infer7.ino**, **bare_infer7.ino**: Progressive refinements of the control loop and current estimation.
- **infer8.ino**: Further refinement of the multi-converter priority logic.
- **infer9.ino**: The most advanced in this series, featuring:
    - **Kalman Filtering**: For smoothing noisy panel voltage readings.
    - **Peak Detection with Decay**: A method to track Voc by observing the highest voltage and slowly decaying the estimate.
    - **PID Control**: Adaptive duty cycle adjustments combined with Perturb & Observe (P&O).
    - **Temperature Compensation**: Adjusting targets based on ambient temperature.

## Key Concepts

- **Current Inference**: Estimating the current by observing how much the panel voltage drops under load relative to its estimated Voc.
- **Priority Loading**: Using multiple DC-DC converters where one is treated as the primary load and the second captures any remaining available power.
