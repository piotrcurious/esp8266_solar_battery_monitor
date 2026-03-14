# current_sense - MPPT with Physical Current Sensors

While much of the `80cent_new` project focuses on *inferring* current, this directory contains code that utilizes actual current sensing hardware (shunts).

## Features

- **Shunt Resistor Integration**: Reads voltage across a shunt resistor to directly measure current.
- **Cross-Validation**: `shunt.ino` includes logic to compare measured current with inferred current from the mathematical models, allowing for self-calibration and diagnostic warnings.
- **Advanced Solar Model**: Includes a more complex equivalent circuit model for the solar panel, considering series and shunt resistance, and temperature coefficients.

## Files

- **shunt.ino**: Demonstrates how to integrate physical current sensing with the existing MPPT and inference frameworks.
