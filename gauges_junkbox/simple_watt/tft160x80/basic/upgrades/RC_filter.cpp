#include <Eigen.h>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

class AdvancedKalmanADC {
private:
    // Kalman filter parameters
    float Q = 0.001;  // Process noise covariance
    float R = 0.1;    // Measurement noise covariance
    float P = 1.0;    // Estimation error covariance
    float K = 0.0;    // Kalman gain
    float x = 0.0;    // State estimate

    // RC filter parameters
    float tau = 0.001;        // RC time constant (seconds)
    float resistance = 10000;  // Input resistance (ohms)
    float capacitance = 0.1e-6; // Input capacitance (farads)
    float cutoffFreq = 0;      // Cutoff frequency (Hz)
    
    // Buffer settings
    static const int HISTORY_SIZE = 32;
    float history[HISTORY_SIZE];
    int historyIndex = 0;
    
    // Chebyshev fitting parameters
    static const int CHEB_ORDER = 4;
    float chebCoeffs[CHEB_ORDER + 1];
    
    // ADC parameters
    float adcVref = 3300.0;  // ADC reference voltage in mV
    int adcResolution = 12;  // ADC resolution in bits
    float inputGain = 1.0;   // Input voltage divider ratio

    // Calculate nth Chebyshev polynomial value
    float chebyshevT(int n, float x) {
        if (n == 0) return 1.0;
        if (n == 1) return x;
        return 2.0 * x * chebyshevT(n - 1, x) - chebyshevT(n - 2, x);
    }

    // Map time domain to [-1, 1]
    float mapTimeToChebDomain(float t, float tMax) {
        return 2.0 * (t / tMax) - 1.0;
    }

    // Theoretical RC response
    float theoreticalRCResponse(float t) {
        return adcVref * (1 - exp(-t / tau));
    }

    // Fit RC response using Chebyshev polynomials
    float fitRCResponseChebyshev(const float* samples, int count, float deltaT) {
        float tMax = deltaT * (count - 1);
        
        // Initialize matrices for least squares fitting
        MatrixXd A(count, CHEB_ORDER + 1);
        VectorXd b(count);
        VectorXd weights(count);
        
        // Build weighted system of equations
        for (int i = 0; i < count; i++) {
            float t = i * deltaT;
            float x = mapTimeToChebDomain(t, tMax);
            
            // Weight samples based on theoretical RC response
            float theoretical = theoreticalRCResponse(t);
            float weight = 1.0 / (1.0 + abs(samples[i] - theoretical));
            weights(i) = weight;
            
            for (int j = 0; j <= CHEB_ORDER; j++) {
                A(i, j) = chebyshevT(j, x) * weight;
            }
            b(i) = samples[i] * weight;
        }
        
        // Solve weighted least squares
        VectorXd coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);
        
        for (int i = 0; i <= CHEB_ORDER; i++) {
            chebCoeffs[i] = coeffs(i);
        }
        
        return coeffs(0);  // Return DC component
    }

    float predictRCResponse(float t, float tMax) {
        float x = mapTimeToChebDomain(t, tMax);
        float value = chebCoeffs[0];
        
        for (int i = 1; i <= CHEB_ORDER; i++) {
            value += chebCoeffs[i] * chebyshevT(i, x);
        }
        
        // Blend with theoretical response
        float theoretical = theoreticalRCResponse(t);
        return 0.7 * value + 0.3 * theoretical;
    }

    float estimateNoise(const float* samples, int count) {
        if (count < 2) return R;
        
        float sum = 0, sum2 = 0;
        for (int i = 1; i < count; i++) {
            float diff = samples[i] - samples[i-1];
            sum += diff;
            sum2 += diff * diff;
        }
        
        float mean = sum / (count - 1);
        float variance = (sum2 / (count - 1)) - (mean * mean);
        return max(sqrt(variance), 0.01f * adcVref);
    }

    void updateHistory(float value) {
        history[historyIndex] = value;
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    }

public:
    AdvancedKalmanADC() {
        reset();
        updateRCParameters();
    }

    // Configure RC filter parameters
    void setRCParameters(float r_ohms, float c_farads) {
        resistance = r_ohms;
        capacitance = c_farads;
        updateRCParameters();
    }

    // Update derived RC parameters
    void updateRCParameters() {
        tau = resistance * capacitance;
        cutoffFreq = 1.0 / (2.0 * PI * tau);
    }

    // Configure ADC parameters
    void setADCParameters(float vref_mv, int resolution, float gain = 1.0) {
        adcVref = vref_mv;
        adcResolution = resolution;
        inputGain = gain;
        R = pow(2, -adcResolution) * adcVref;  // Update measurement noise based on ADC resolution
    }

    // Read and filter ADC value
    float read(int pin, int samplesPerRead = 10) {
        float samples[64];
        samplesPerRead = constrain(samplesPerRead, 2, 64);
        
        unsigned long startTime = micros();
        for (int i = 0; i < samplesPerRead; i++) {
            samples[i] = analogReadMilliVolts(pin) / inputGain;
            delayMicroseconds(int(tau * 1e6 / samplesPerRead));  // Delay based on RC time constant
        }
        float deltaT = (micros() - startTime) / 1e6 / samplesPerRead;

        // Dynamic noise estimation
        float measuredNoise = estimateNoise(samples, samplesPerRead);
        R = 0.9 * R + 0.1 * (measuredNoise * measuredNoise);  // Exponential moving average

        // Predict step with RC model
        float predicted = predictRCResponse(deltaT, deltaT * samplesPerRead);
        float prediction_error = pow(predicted - x, 2);
        Q = 0.1 * prediction_error;  // Adapt process noise based on prediction error
        
        P = P + Q;

        // Fit current samples
        float fittedValue = fitRCResponseChebyshev(samples, samplesPerRead, deltaT);

        // Update step
        K = P / (P + R);
        x = x + K * (fittedValue - x);
        P = (1 - K) * P;

        // Apply historical correction
        if (historyIndex >= samplesPerRead) {
            float historicalAvg = 0;
            for (int i = 0; i < samplesPerRead; i++) {
                historicalAvg += history[(historyIndex - i - 1 + HISTORY_SIZE) % HISTORY_SIZE];
            }
            historicalAvg /= samplesPerRead;
            
            // Blend estimates based on confidence
            float confidence = 1.0 / (1.0 + P);
            x = confidence * x + (1 - confidence) * (0.7 * predicted + 0.3 * historicalAvg);
        }

        updateHistory(x);
        return x;
    }

    // Get current filter parameters
    float getCutoffFrequency() { return cutoffFreq; }
    float getTimeConstant() { return tau; }
    float getResistance() { return resistance; }
    float getCapacitance() { return capacitance; }
    
    // Reset filter state
    void reset() {
        P = 1.0;
        x = 0.0;
        historyIndex = 0;
        for (int i = 0; i < HISTORY_SIZE; i++) history[i] = 0;
        for (int i = 0; i <= CHEB_ORDER; i++) chebCoeffs[i] = 0;
    }
};
