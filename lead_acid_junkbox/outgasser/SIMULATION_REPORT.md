# Outgasser Firmware: Operation & Simulation Report

This document explains the charging algorithm and characterization process implemented in the `outgasser` firmware, verified through extensive simulation across various battery health scenarios.

## Charging Algorithm Flow

The firmware follows a multi-pass state machine to precisely identify the battery's specific electrochemical outgassing point.

### 1. BULK Stage (MODE 5)
- **Goal:** Charge as fast as solar conditions allow using fractional-Voc MPPT.
- **Safety:** "Stall Detection" monitors voltage rise to prevent overcharging failed cells.

### 2. MULTI-PASS CHARACTERIZATION (MODE 2 & 6)
- **Goal:** Statistically robust identification of the outgassing knee.
- **Process:** The firmware performs 3 consecutive passes of the following:
    - **Parasitic measurement:** Holds voltage above idle to establish a current baseline (lower search bound).
    - **Bisection search:** Rapidly narrow down the minimum current that triggers the knee.
    - **Physical verification:** Uses a combination of dV/dt slope analysis (via Kalman filter) and bi-exponential decay fits (after pulses) to confirm the Faradaic reaction.
- **Visualization:** The "Characterization Detail" graphs show the iterative pulses and the calculated "Efficiency Ratio". A drop in efficiency marks the outgassing knee.

### 3. FLOAT Stage (MODE 7)
- **Goal:** Maintain charge at the average outgassing voltage discovered across all passes.

---

## Simulation Scenarios

### Healthy Battery
- **Full Overview:** ![Healthy Full](tests/healthy_full.svg)
- **Characterization Zoom:** ![Healthy Pulses](tests/healthy_pulses.svg)

### High Internal Resistance
- **Full Overview:** ![High-R Full](tests/high_r_full.svg)
- **Characterization Zoom:** ![High-R Pulses](tests/high_r_pulses.svg)

### Aged / Low Capacity
- **Full Overview:** ![Aged Full](tests/aged_full.svg)
- **Characterization Zoom:** ![Aged Pulses](tests/aged_pulses.svg)

### Variable Solar (Cloudy / Stormy)
- **Full Overview:** ![Cloudy Full](tests/cloudy_full.svg)
- **Characterization Detail:** ![Cloudy Pulses](tests/cloudy_pulses.svg)
- **Result:** The system correctly pauses pulsing and characterization timers during deep solar dropouts, ensuring no spurious data corrupts the bisection search.
