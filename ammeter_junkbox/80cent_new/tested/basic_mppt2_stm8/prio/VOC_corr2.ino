// Global variables for input
float voltage_open_circuit;  // Measured open circuit voltage
float panel_temperature;     // Measured panel temperature in Celsius
float solar_irradiance;     // Measured solar irradiance in W/m²
float wind_speed;           // Wind speed in m/s for thermal model
float ambient_temperature;  // Ambient temperature in Celsius

// Panel specifications at STC (Standard Test Conditions)
const float rated_VOC = 38.4;     // Rated open circuit voltage at STC
const float rated_IOC = 9.71;     // Rated operating current at STC
const float rated_V = 31.2;       // Rated operating voltage at STC
const float rated_ISC = 10.45;    // Rated short circuit current at STC
const float rated_efficiency = 0.201; // Panel efficiency at STC
const float panel_area = 1.6;     // Panel area in m²
const float NOCT = 45.0;          // Nominal Operating Cell Temperature in °C

// Material properties
const float band_gap_ref = 1.121; // Silicon band gap energy at reference temp
const float band_gap = 1.121;     // Actual band gap energy
const float n_diode = 1.3;        // Diode ideality factor
const float k_boltz = 1.380649e-23; // Boltzmann constant
const float q_electron = 1.602176634e-19; // Electron charge

// Temperature coefficients (detailed)
const float TEMP_COEFF_VOC = -0.0037;  // Temperature coefficient for Voc (%/°C)
const float TEMP_COEFF_ISC = 0.0004;   // Temperature coefficient for Isc (%/°C)
const float TEMP_COEFF_PMPP = -0.0041; // Temperature coefficient for maximum power (%/°C)
const float TEMP_COEFF_FF = -0.0016;   // Temperature coefficient for fill factor (%/°C)

// Parasitic resistances
const float rs_ref = 0.221;      // Series resistance at reference conditions (Ω)
const float rsh_ref = 415.405;   // Shunt resistance at reference conditions (Ω)

// Output arrays for V-I curve (32 points)
float output_voltage[32];
float output_current[32];

// Internal model parameters
float a1, a2, a3, a4; // Modified diode equation coefficients

// Calculate cell temperature based on environmental conditions
float calculateCellTemp() {
    // Using Faiman model for cell temperature
    float u0 = 25.0; // Heat transfer coefficient
    float u1 = 6.84; // Wind speed coefficient
    
    float cell_temp = ambient_temperature + 
        (solar_irradiance / (u0 + u1 * wind_speed)) * 
        (1 - rated_efficiency) * (1 - 0.0005 * (ambient_temperature - 25));
        
    // Apply NOCT correction
    cell_temp = cell_temp * (NOCT - 20) / 800;
    
    return cell_temp;
}

// Calculate reverse saturation current
float calculateI0(float temp_k) {
    float vt = k_boltz * temp_k / q_electron;
    float i0_ref = rated_ISC * exp(-rated_VOC / (n_diode * vt));
    
    return i0_ref * pow((temp_k/298.15), 3) * 
           exp((q_electron * (band_gap_ref * 298.15 - band_gap * temp_k)) / 
               (n_diode * k_boltz * temp_k * 298.15));
}

// Calculate series and shunt resistance temperature dependence
void calculateResistances(float temp_k, float* rs, float* rsh) {
    // Temperature dependence of series resistance
    *rs = rs_ref * (1 + 0.0004 * (temp_k - 298.15));
    
    // Temperature dependence of shunt resistance
    *rsh = rsh_ref * (1 - 0.0015 * (temp_k - 298.15));
}

void calculateSolarIrradiance() {
    float cell_temp = calculateCellTemp();
    float temp_k = cell_temp + 273.15;
    float vt = k_boltz * temp_k / q_electron;
    
    // Calculate reverse saturation current
    float i0 = calculateI0(temp_k);
    
    // Calculate temperature-dependent resistances
    float rs, rsh;
    calculateResistances(temp_k, &rs, &rsh);
    
    // Calculate photogenerated current
    float iph = (rated_ISC + TEMP_COEFF_ISC * (cell_temp - 25)) * 
                (solar_irradiance / 1000.0);
    
    // Calculate modified diode equation coefficients
    a1 = rs / rsh;
    a2 = -1 - (rs / rsh);
    a3 = i0 * rs;
    a4 = i0 * rsh;
    
    // Generate V-I curve points using improved single-diode model
    for(int i = 0; i < 32; i++) {
        output_voltage[i] = (voltage_open_circuit * i) / 31.0;
        float v = output_voltage[i];
        
        // Improved single-diode model equation
        float id = i0 * (exp((v + output_current[i] * rs) / (n_diode * vt)) - 1);
        float ish = (v + output_current[i] * rs) / rsh;
        output_current[i] = iph - id - ish;
        
        // Newton-Raphson iteration for better accuracy
        for(int iter = 0; iter < 5; iter++) {
            float f = iph - output_current[i] - 
                     i0 * (exp((v + output_current[i] * rs) / (n_diode * vt)) - 1) - 
                     (v + output_current[i] * rs) / rsh;
            float df = -1 - (i0 * rs * exp((v + output_current[i] * rs) / (n_diode * vt))) / 
                      (n_diode * vt) - rs / rsh;
            output_current[i] = output_current[i] - f / df;
        }
        
        // Ensure current doesn't go negative
        if(output_current[i] < 0) output_current[i] = 0;
    }
}

// Enhanced helper function to get maximum power point with additional parameters
void getMaxPowerPoint(float* max_power, float* mpp_voltage, float* mpp_current, 
                     float* fill_factor, float* efficiency) {
    *max_power = 0;
    *mpp_voltage = 0;
    *mpp_current = 0;
    
    for(int i = 0; i < 32; i++) {
        float power = output_voltage[i] * output_current[i];
        if(power > *max_power) {
            *max_power = power;
            *mpp_voltage = output_voltage[i];
            *mpp_current = output_current[i];
        }
    }
    
    // Calculate fill factor
    *fill_factor = *max_power / (voltage_open_circuit * output_current[0]);
    
    // Calculate actual efficiency
    *efficiency = *max_power / (solar_irradiance * panel_area);
}
