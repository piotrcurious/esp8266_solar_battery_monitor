# Outgasser Firmware: Operation & Simulation Report

This document explains the charging algorithm and characterization process implemented in the `outgasser` firmware, verified through extensive simulation across various battery health scenarios.

## Charging Algorithm Flow

The firmware follows a multi-stage state machine to safely charge lead-acid batteries while identifying their specific electrochemical outgassing point.

### 1. BULK Stage (MODE 5)
- **Goal:** Charge the battery as fast as the solar conditions allow.
- **Process:** Utilizes a fractional-Voc MPPT loop to keep the solar panel at its maximum power point. Charging continues until the terminal voltage reaches `BULK_TARGET_V` (default 13.8V).
- **Visualization:** This is the long rising voltage curve at the start of the "Full Cycle" graphs.

### 2. PARASITIC BASELINE Stage (MODE 2)
- **Goal:** Measure the self-discharge and system load.
- **Process:** The charge FET is modulated in a small closed-loop to hold the battery voltage slightly above its idle level. The average current required to maintain this voltage is recorded as the `parasiticCurrent_mA`. This value sets the lower bound for the outgassing search.

### 3. OUTGASSING CHARACTERIZATION (MODE 6)
- **Goal:** Find the minimum current that triggers the "knee" of the water-splitting reaction (outgassing).
- **Process:**
    - **Initial Pulse:** Runs at maximum available solar current to establish a baseline capacitance and see if gassing is reachable.
    - **Bisection Search:** Iteratively tests currents between the parasitic floor and the last known gassing current.
    - **Knee Detection:** Monitors the ratio of the actual slope (dV/dt) to the expected capacitive slope. A significant drop (below 0.7 efficiency) indicates the Faradaic reaction is consuming current.
    - **Decay Verification:** After each pulse, the voltage relaxation is analyzed. A confirmed gassing pulse shows a distinctive bi-exponential decay as gases recombine or diffuse.
- **Visualization:** See the "Characterization Detail" graphs for a zoomed-in view of these diagnostic pulses.

### 4. FLOAT Stage (MODE 7)
- **Goal:** Maintain the battery at its optimal full-charge voltage without excessive gassing.
- **Process:** Holds the battery at the discovered outgassing voltage. The current tapers down naturally as the battery saturates. The cycle completes when the current falls below the termination threshold.

---

## Simulation Scenarios

### Healthy Battery
Standard internal resistance and capacity.
- **Full Overview:** ![Healthy Full](tests/healthy_full.svg)
- **Characterization Zoom:** ![Healthy Pulses](tests/healthy_pulses.svg)

### High Internal Resistance
Simulates an aged/sulfated battery. The characterization zoom shows much higher instantaneous voltage steps ("IR drop") during pulses.
- **Full Overview:** ![High-R Full](tests/high_r_full.svg)
- **Characterization Zoom:** ![High-R Pulses](tests/high_r_pulses.svg)

### Aged / Low Capacity
A battery with very small Ah capacity. The voltage rises much faster during bulk and characterization.
- **Full Overview:** ![Aged Full](tests/aged_full.svg)
- **Characterization Zoom:** ![Aged Pulses](tests/aged_pulses.svg)

### Cloudy Day (Variable Solar)
Irradiance varies sinusoidally. The firmware pauses the bisection pulses during solar dropouts to ensure diagnostic accuracy.
- **Full Overview:** ![Cloudy Full](tests/cloudy_full.svg)
- **Characterization Zoom:** ![Cloudy Pulses](tests/cloudy_pulses.svg)
