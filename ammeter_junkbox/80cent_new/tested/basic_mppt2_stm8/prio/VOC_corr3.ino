#include <math.h>

// Global Variables (Inputs)
float voltage_open_circuit;  // Measured open-circuit voltage (V)
float panel_temperature;      // Panel temperature (°C)

// Global Constants
const float rated_VOC = 40.0;     // Rated open-circuit voltage (V)
const float rated_IOC = 8.0;      // Rated short-circuit current (A)
const float rated_V = 32.0;       // Rated voltage at max power point (V)
const float rated_ISC = 7.5;      // Rated short-circuit current (A)
const int num_cells_series = 72;  // Number of cells in series
const float panel_area = 1.6;     // Panel area in m²
const float Rs = 0.4;             // Estimated series resistance (Ω)
const float Rsh = 300.0;          // Estimated shunt resistance (Ω)
const float n = 1.3;              // Diode ideality factor
const float k = 1.380649e-23;     // Boltzmann Constant (J/K)
const float q = 1.602176634e-19;  // Electron charge (C)
const float STC_TEMPERATURE = 25.0;  
const float STC_IRRADIANCE = 1000.0;

// Output Array (Voltage-Current Curve, 32 Steps)
float output_IV_curve[32][2];

// Newton-Raphson Solver for Current (I)
float solveCurrent(float V, float Iph, float I0, float Vt) {
    float I = Iph;  // Initial guess
    for (int j = 0; j < 5; j++) { // Iterate 5 times for convergence
        float f = Iph - I0 * (exp((V + I * Rs) / Vt) - 1) - (V + I * Rs) / Rsh - I;
        float df = -I0 * exp((V + I * Rs) / Vt) / Vt - 1 - Rs / Rsh;
        I -= f / df;
        if (I < 0) I = 0; // Ensure non-negative current
    }
    return I;
}

// Function to Calculate Solar Irradiance & IV Curve
void calculateSolarIrradiance() {
    float T_K = panel_temperature + 273.15;
    float T_STC_K = STC_TEMPERATURE + 273.15;
    float Vt = (n * k * T_K) / q;

    float delta_T = panel_temperature - STC_TEMPERATURE;
    float TEMP_COEFF_VOC = -0.003;  
    float TEMP_COEFF_ISC = 0.0006;  
    float corrected_VOC = rated_VOC + (TEMP_COEFF_VOC * num_cells_series * delta_T);
    float corrected_ISC = rated_ISC + (TEMP_COEFF_ISC * rated_ISC * delta_T);

    float irradiance = STC_IRRADIANCE * (voltage_open_circuit / corrected_VOC);
    float Iph = corrected_ISC * (irradiance / STC_IRRADIANCE);
    
    float I0_STC = rated_ISC / (exp(rated_VOC / (n * Vt)) - 1);
    float I0 = I0_STC * pow(T_K / T_STC_K, 3) * exp((q * rated_VOC / (n * k)) * (1 / T_STC_K - 1 / T_K));

    for (int i = 0; i < 32; i++) {
        float voltage = (corrected_VOC * i) / 31;
        float current = solveCurrent(voltage, Iph, I0, Vt);
        output_IV_curve[i][0] = voltage;
        output_IV_curve[i][1] = current;
    }
}
