# basic_mppt & basic_mppt2 - Standard P&O MPPT

These directories contain standard Perturb & Observe (P&O) MPPT implementations.

## Features

- **P&O Algorithm**: Periodically adjusts the duty cycle and observes the change in power to find the MPP.
- **Voc Measurement**: Periodically disables the load to measure the Open Circuit Voltage (Voc).
- **Frequency Adjustment**: `basic_mppt2` includes `freqadj` variations that explore the impact of different PWM frequencies on the efficiency and stability of the tracking.

## Files

- **basic_mppt/basic_mppt.ino**: Baseline P&O implementation.
- **basic_mppt2/basic_mppt2.ino**: Refined P&O implementation.
- **basic_mppt2/freqadj[1-8].ino**: Experiments with dynamic frequency adjustment.
