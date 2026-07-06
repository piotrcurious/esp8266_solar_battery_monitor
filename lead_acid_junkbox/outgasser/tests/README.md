# Test System for Outgasser

This directory contains a mock Arduino environment and a battery chemistry simulator to test the `outgasser` firmware.

## Files

- `arduino_mock.h/cpp`: Mock implementation of Arduino/ESP32 core, INA219, PID, and Preferences libraries.
- `battery_sim.py`: A Python-based battery and solar panel simulator.
- `test_runner.cpp`: C++ test harness that compiles the `.ino` code (as `.cpp`) and links it with the mock environment and Python simulator.
- `Makefile`: Build script for the test system.

## How to run

1.  Make sure you have `g++` and `python3` installed.
2.  Run `make` to compile the test runner.
3.  Run `./test_runner` to execute the simulation.

The simulation will walk through:
1. BULK charge until 13.8V.
2. PARASITIC baseline characterization.
3. OUTGASSING bisection search pulses.
4. FLOAT charge at the discovered outgassing voltage.

The test passes if it reaches `MODE_CHARGE_DONE`.
