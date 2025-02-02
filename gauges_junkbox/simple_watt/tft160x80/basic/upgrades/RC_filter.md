Key improvements in this version:

1. Added configurable RC filter parameters:
   - Resistance and capacitance settings
   - Automatic time constant and cutoff frequency calculation
   - Theoretical RC response model

2. Enhanced ADC configuration:
   - Configurable reference voltage
   - Resolution setting
   - Input gain/voltage divider support

3. Improved filtering algorithm:
   - Weighted least squares fitting
   - Adaptive process noise based on prediction error
   - Confidence-based estimate blending
   - Dynamic sample timing based on RC time constant

4. Better noise estimation:
   - First-difference based noise estimation
   - Exponential moving average for noise updates
   - Minimum noise floor based on ADC resolution

Example usage:
```cpp
AdvancedKalmanADC filter;

// Configure for typical RC input filter
filter.setRCParameters(10000, 0.1e-6);  // 10k ohm, 0.1uF

// Configure ADC parameters
filter.setADCParameters(3300, 12, 1.0);  // 3.3V ref, 12-bit, unity gain

// Read filtered value
float value = filter.read(34, 10);  // Read GPIO34 with 10 samples
```

Would you like me to:
1. Add temperature compensation for the RC network?
2. Implement automatic RC parameter estimation?
3. Add more diagnostic features?
