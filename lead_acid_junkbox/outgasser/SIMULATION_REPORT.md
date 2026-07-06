# Outgasser Firmware Simulation Report

This report summarizes the performance of the `outgasser` firmware across multiple simulated battery and solar conditions.

## Simulation Environment

The simulation utilizes a 3-component electrochemical model for the battery:
1.  **Bulk Capacitance:** Represents the main energy storage (SOC).
2.  **Double Layer:** Captures fast voltage transients when current starts/stops.
3.  **Diffusion/Gas component:** Models the slow relaxation seen after outgassing or heavy charging.

Solar conditions are modeled with a fractional-Voc MPPT loop, including support for variable irradiance (clouds).

## Scenarios and Results

### 1. Healthy Battery
A battery with low internal resistance and standard capacity.
- **Result:** Successful bulk charge, accurate outgassing detection at ~13.7V, and smooth transition to float.
- **Graph:** `tests/healthy_results.svg`

### 2. High Internal Resistance
Simulates an aged or sulfated battery with high ohmic losses.
- **Result:** Firmware correctly identifies high voltage drops during pulses. Bisection search remains stable despite increased noise and lower charging efficiency.
- **Graph:** `tests/high_r_results.svg`

### 3. Aged / Low Capacity
Simulates a battery with significantly reduced Ah capacity.
- **Result:** Faster bulk charge phase. Outgassing knee is detected early. The bisection search handles the rapid voltage rise robustly.
- **Graph:** `tests/aged_results.svg`

### 4. Cloudy Day (Variable Solar)
Irradiance varies sinusoidally every 60 seconds, simulating passing clouds.
- **Result:** The firmware's solar dropout detection pauses characterization pulses when panel voltage falls below the MPPT target, preventing spurious "knee" detections due to current collapse.
- **Graph:** `tests/cloudy_results.svg`

## Key Improvements Verified

- **Temperature Compensation:** Verified outgassing voltage targets shift correctly with simulated NTC input.
- **Passive Formation:** Confirmed legacy discharge logic can be replaced by observing opportunistic parasitic loads.
- **Kalman Filtering:** Successfully provided stable slope (dV/dt) estimates during high-SNR pulses.
- **Structured Telemetry:** `TELE:` lines provide a clean CSV-compatible format for field data analysis.
