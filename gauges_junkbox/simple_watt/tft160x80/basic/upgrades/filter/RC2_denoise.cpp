#include <Eigen.h>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

class NoiseModel {
private:
    // Noise characteristics
    float thermal_noise;      // Johnson-Nyquist noise
    float quantization_noise; // ADC quantization noise
    float flicker_noise;      // 1/f noise coefficient
    float shot_noise;         // Shot noise coefficient
    float ambient_noise;      // Environmental noise estimate
    
    // Temperature parameters
    float temperature = 298.0;  // Kelvin
    const float k_boltzmann = 1.380649e-23;  // Boltzmann constant
    
    // Frequency analysis
    static const int FFT_SIZE = 64;
    float freq_spectrum[FFT_SIZE/2];
    
    // Calculate thermal noise for given resistance
    float calculateThermalNoise(float resistance) {
        return sqrt(4 * k_boltzmann * temperature * resistance);
    }
    
    // Estimate flicker noise at given frequency
    float calculateFlickerNoise(float frequency, float current) {
        return flicker_noise * current / sqrt(frequency);
    }
    
    // Calculate shot noise for given current
    float calculateShotNoise(float current) {
        const float q_electron = 1.60217663e-19;  // Elementary charge
        return sqrt(2 * q_electron * abs(current));
    }

public:
    NoiseModel() {
        thermal_noise = 0;
        quantization_noise = 0;
        flicker_noise = 1e-6;
        shot_noise = 0;
        ambient_noise = 0;
        memset(freq_spectrum, 0, sizeof(freq_spectrum));
    }
    
    void updateParameters(float temp_celsius, float resistance, float vref, int resolution) {
        temperature = temp_celsius + 273.15;
        thermal_noise = calculateThermalNoise(resistance);
        quantization_noise = vref / (sqrt(12) * pow(2, resolution));
    }
    
    // Estimate noise parameters from sample buffer
    void analyzeNoise(const float* samples, int count, float sampling_rate) {
        // Temporary arrays for FFT
        float real[FFT_SIZE] = {0};
        float imag[FFT_SIZE] = {0};
        
        // Prepare data with Hanning window
        for (int i = 0; i < min(count, FFT_SIZE); i++) {
            float hanning = 0.5 * (1 - cos(2 * PI * i / (FFT_SIZE - 1)));
            real[i] = samples[i] * hanning;
        }
        
        // Perform FFT (simplified implementation)
        for (int k = 0; k < FFT_SIZE/2; k++) {
            for (int n = 0; n < FFT_SIZE; n++) {
                float angle = -2 * PI * k * n / FFT_SIZE;
                freq_spectrum[k] += real[n] * cos(angle);
            }
            freq_spectrum[k] = abs(freq_spectrum[k]) / FFT_SIZE;
        }
        
        // Estimate flicker noise from low frequency components
        float low_freq_power = 0;
        for (int i = 1; i < 5 && i < FFT_SIZE/2; i++) {
            low_freq_power += freq_spectrum[i];
        }
        flicker_noise = low_freq_power / 4;
        
        // Estimate ambient noise from high frequency components
        float high_freq_power = 0;
        for (int i = FFT_SIZE/4; i < FFT_SIZE/2; i++) {
            high_freq_power += freq_spectrum[i];
        }
        ambient_noise = high_freq_power / (FFT_SIZE/4);
    }
    
    // Get total noise estimate for current operating point
    float getTotalNoise(float current, float frequency) {
        float total_squared = 
            pow(thermal_noise, 2) +
            pow(quantization_noise, 2) +
            pow(calculateFlickerNoise(frequency, current), 2) +
            pow(calculateShotNoise(current), 2) +
            pow(ambient_noise, 2);
        
        return sqrt(total_squared);
    }
    
    // Get noise spectrum for analysis
    const float* getNoiseSpectrum() const {
        return freq_spectrum;
    }
};

class AdvancedKalmanADC {
    // ... [Previous AdvancedKalmanADC code remains the same until the private section] ...

private:
    NoiseModel noiseModel;
    
    // Noise analysis parameters
    float sampling_rate;
    static const int NOISE_BUFFER_SIZE = 64;
    float noise_buffer[NOISE_BUFFER_SIZE];
    int noise_buffer_index = 0;
    
    // New method for noise estimation
    void updateNoiseEstimates(const float* samples, int count, float dt) {
        // Update noise buffer
        for (int i = 0; i < count && noise_buffer_index < NOISE_BUFFER_SIZE; i++) {
            noise_buffer[noise_buffer_index++] = samples[i];
        }
        
        if (noise_buffer_index >= NOISE_BUFFER_SIZE) {
            sampling_rate = 1.0f / dt;
            noiseModel.analyzeNoise(noise_buffer, NOISE_BUFFER_SIZE, sampling_rate);
            noise_buffer_index = 0;  // Reset buffer
            
            // Update Kalman filter noise parameters
            float current_estimate = state(1);
            float effective_freq = sampling_rate / 2;  // Nyquist frequency
            
            // Get total noise estimate
            float total_noise = noiseModel.getTotalNoise(current_estimate, effective_freq);
            
            // Update measurement noise
            R = pow(total_noise, 2);
            
            // Update process noise based on noise characteristics
            Q(0,0) = pow(total_noise * 0.1, 2);  // Voltage process noise
            Q(1,1) = pow(total_noise / resistance, 2);  // Current process noise
            Q(2,2) = pow(nominal_tau * 0.01, 2);  // Tau process noise
        }
    }

public:
    // ... [Previous public methods remain the same until setADCParameters] ...

    void setADCParameters(float vref_mv, int resolution, float gain = 1.0) {
        adcVref = vref_mv;
        adcResolution = resolution;
        inputGain = gain;
        
        // Initialize noise model with ADC parameters
        noiseModel.updateParameters(25.0, resistance, vref_mv, resolution);
    }
    
    // New method to set temperature for noise calculations
    void setTemperature(float temp_celsius) {
        noiseModel.updateParameters(temp_celsius, resistance, adcVref, adcResolution);
    }
    
    float read(int pin, int samplesPerRead = 10) {
        samplesPerRead = constrain(samplesPerRead, 2, 64);
        
        // Collect samples with timestamps
        float samples[64];
        float timestamps[64];
        unsigned long startTime = micros();
        
        for (int i = 0; i < samplesPerRead; i++) {
            samples[i] = analogReadMilliVolts(pin) / inputGain;
            timestamps[i] = (micros() - startTime) / 1e6;
            delayMicroseconds(int(state(2) * 1e6 / samplesPerRead));
        }

        float dt = timestamps[samplesPerRead-1] - timestamps[0];
        
        // Update noise estimates
        updateNoiseEstimates(samples, samplesPerRead, dt/samplesPerRead);
        
        // ... [Rest of the read method remains the same] ...
    }
    
    // New method to get noise spectrum for analysis
    const float* getNoiseSpectrum() const {
        return noiseModel.getNoiseSpectrum();
    }
};
