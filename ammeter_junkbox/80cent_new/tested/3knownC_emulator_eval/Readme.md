# 3knownC Emulator Evaluation and Optimization

This directory contains a version of the 3knownC (Known Capacitance) method evaluated and improved using a Python-based physical system emulator.

## Methodology

A mock Arduino environment was created to run the C++ controller logic on a host machine. A Python emulator simulates the solar panel (Voc, Rint), input capacitor (C), and PWM-controlled load.

### Emulator Details
- **Model**: Analytical RC solution for a capacitor charged by a voltage source (Voc) through internal resistance (Rint) and discharged by a PWM-switched load.
- **Parameters**:
  - `Voc`: Open Circuit Voltage
  - `Rint`: Internal Resistance of the source
  - `C`: Known Capacitance (default 20000 uF)
  - `Rload`: Load Resistance

## Test Scenarios and Results

### 1. Static Conditions
![Static Test](test_static.png)
- **Observations**: The controller successfully estimates Voc and Rint. Voc converges almost immediately after the first full calibration. Rint (iterative) takes longer but reaches the true value.

### 2. Dynamic Voc
![Dynamic Voc](test_dynamic_voc.png)
- **Observations**: When Voc drops, the partial measurement detection triggers more frequent calibrations, allowing the controller to track the change.

### 3. Dynamic Rint
![Dynamic Rint](test_dynamic_rint.png)
- **Observations**: Increasing Rint is tracked by both the RC fitting (`R_tau`) and the iterative ammeter model (`R_src`).

### 4. High Noise
![High Noise](test_high_noise.png)
- **Observations**: The algorithm remains stable even with significant measurement noise (std=0.1V), thanks to the gradient descent fitting which naturally averages out noise.

## Improvements and Final "Unified" Logic

The final version of the controller (`controller_logic.cpp`) integrates several advanced optimizations:

### 1. Rint Seed Blending
The iterative ammeter-style internal resistance estimate (`internal_resistance_src`) is blended with the estimate derived from the RC time constant (`resistance_tau_est`) from the sampling curve. This allows for nearly instantaneous tracking of resistance changes.

### 2. Adaptive Sampling Intervals
Instead of fixed delays, the controller calculates the sampling interval based on the previous estimate of the time constant $\tau$. It targets a total observation window of approximately 3.5$\tau$, ensuring that the 50 captured samples always contain the most informative part of the exponential charging curve regardless of the source resistance or capacitance.

### 3. Momentum-based Gradient Descent
The RC curve fitting algorithm uses momentum (factor = 0.85) and adaptive learning rates. This prevents the solver from getting stuck in local minima caused by noise and significantly accelerates convergence towards the true $V_{oc}$ and $R_{int}$ values.

### 4. Predictive Oscillation Verification
During the "tracking" phase (between full calibrations), the controller periodically performs a "micro-oscillation" (small PWM change). It compares the measured voltage response against the predicted response from its internal model. If a significant discrepancy is detected (indicating a change in $V_{oc}$ or $R_{int}$), it triggers an immediate full recalibration rather than waiting for the next scheduled interval.

## Test Report (Unified Version)

The unified logic was tested against four distinct scenarios:

| Test Case | Objective | Result |
|-----------|-----------|--------|
| **Static** | Baseline accuracy | High precision convergence in < 2 calibration cycles. |
| **Dynamic Voc** | Tracking source changes | Rapid detection of voltage drops via oscillation verification. |
| **Dynamic Rint** | Tracking panel degradation/shading | Smooth tracking of resistance spikes with zero overshoot. |
| **High Noise** | Robustness | Stable operation with 0.1V standard deviation noise. |

## Files
- `emulator.py`: Physical system model.
- `analyzer.py`: Test runner and data logger.
- `run_tests.py`: Suite of test scenarios.
- `controller_logic.cpp`: The improved C++ controller logic.
- `mock_arduino.*`: Arduino API compatibility layer for host execution.
