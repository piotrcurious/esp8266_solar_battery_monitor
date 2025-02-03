#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <numeric>

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
    
    // Enhanced Chebyshev and historical analysis parameters
    static const int CHEB_ORDER = 7;
    static const int HISTORY_DEPTH = 64;
    static constexpr float CHEB_SCALE_FACTOR = 1.1;
    float chebCoeffs[CHEB_ORDER + 1];
    float deltaT = 0.0f;
    float prediction_error_sum = 0.0f;
    int currentSamplesPerRead = 10;

    // ADC parameters
    float adcVref = 3300.0;  // ADC reference voltage in mV
    int adcResolution = 12;  // ADC resolution in bits
    float inputGain = 1.0;   // Input voltage divider ratio

    // Historical data structure
    struct SignalSnapshot {
        float value;
        float timeConstant;
        float derivativeCharacteristic;
        unsigned long timestamp;
        float coefficients[CHEB_ORDER + 1];
    };
    
    SignalSnapshot history[HISTORY_DEPTH];
    int historyIndex = 0;

    // Core Chebyshev polynomial calculation
    float chebyshevPolynomial(int n, float x) {
        switch(n) {
            case 0: return 1.0f;
            case 1: return x;
            case 2: return 2.0f * x * x - 1.0f;
            case 3: return x * (4.0f * x * x - 3.0f);
            case 4: return 8.0f * x * x * x * x - 8.0f * x * x + 1.0f;
            default: {
                float a = x, b = 1.0f, c;
                for (int i = 2; i <= n; i++) {
                    c = 2.0f * x * a - b;
                    b = a;
                    a = c;
                }
                return a;
            }
        }
    }

    // Derivative calculation for Chebyshev polynomials
    float chebyshevDerivative(int n, int derivOrder, float x) {
        if (derivOrder == 0) return chebyshevPolynomial(n, x);
        
        switch(derivOrder) {
            case 1: {
                if (n == 0) return 0.0f;
                if (n == 1) return 1.0f;
                float sum = 0.0f;
                for (int k = 1; k <= n; k += 2) {
                    sum += k * chebyshevPolynomial(k-1, x);
                }
                return 2.0f * sum;
            }
            case 2: {
                if (n <= 1) return 0.0f;
                float sum = 0.0f;
                for (int k = 2; k <= n; k += 2) {
                    sum += k * (k-1) * chebyshevPolynomial(k-2, x);
                }
                return 4.0f * sum;
            }
            default: {
                float result = 0.0f;
                float factorial = 1.0f;
                for (int i = 0; i < derivOrder; i++) {
                    factorial *= (i + 1);
                }
                for (int k = derivOrder; k <= n; k++) {
                    float coeff = 1.0f;
                    for (int j = 0; j < derivOrder; j++) {
                        coeff *= (k - j);
                    }
                    result += coeff * chebyshevPolynomial(k - derivOrder, x) / std::pow(2.0f, derivOrder);
                }
                return result * factorial;
            }
        }
    }

    // Advanced domain mapping with non-linear scaling
    float mapToChebDomain(float t, float tMax) {
        float scaledT = std::tanh(t / (0.5f * tMax)) * 2.0f;
        return scaledT * CHEB_SCALE_FACTOR;
    }

    // Enhanced Chebyshev fitting with historical context
    float fitChebyshevWithHistory(const float* samples, int count, float deltaT) {
        float tMax = deltaT * (count - 1);
        
        // Get historical fit coefficients
        std::vector<float> historicalCoeffs(CHEB_ORDER + 1, 0.0f);
        if (historyIndex > 0) {
            for (int i = 0; i <= CHEB_ORDER; i++) {
                float sum = 0.0f;
                int validCount = 0;
                for (int j = 1; j <= std::min(5, historyIndex); j++) {
                    int idx = (historyIndex - j + HISTORY_DEPTH) % HISTORY_DEPTH;
                    sum += history[idx].coefficients[i];
                    validCount++;
                }
                historicalCoeffs[i] = validCount > 0 ? sum / validCount : 0.0f;
            }
        }

        // Compute median for robust scaling
        std::vector<float> sortedSamples(samples, samples + count);
        std::nth_element(sortedSamples.begin(), 
                        sortedSamples.begin() + count/2, 
                        sortedSamples.end());
        float medianSample = sortedSamples[count/2];
        
        // Setup matrices for weighted least squares
        MatrixXd A(count, CHEB_ORDER + 1);
        VectorXd b(count);
        VectorXd weights(count);

        // Enhanced weighting incorporating historical fit
        for (int i = 0; i < count; i++) {
            float t = i * deltaT;
            float x = mapToChebDomain(t, tMax);
            
            // Compute historical prediction
            float historicalPred = 0.0f;
            for (int j = 0; j <= CHEB_ORDER; j++) {
                historicalPred += historicalCoeffs[j] * chebyshevPolynomial(j, x);
            }
            
            // Adaptive weight calculation
            float sampleDev = std::abs(samples[i] - medianSample);
            float historicalDev = std::abs(samples[i] - historicalPred);
            float weight = 1.0f / (1.0f + sampleDev + 0.5f * historicalDev);
            weights(i) = std::pow(weight, 1.5f);
            
            // Build basis matrix
            for (int j = 0; j <= CHEB_ORDER; j++) {
                A(i, j) = chebyshevPolynomial(j, x) * weights(i);
            }
            b(i) = samples[i] * weights(i);
        }

        // Solve with regularization
        MatrixXd regularization = MatrixXd::Identity(CHEB_ORDER + 1, CHEB_ORDER + 1);
        regularization(0, 0) = 0.0f;  // Preserve DC component
        VectorXd coeffs = (A.transpose() * A + 0.01f * regularization).ldlt().solve(A.transpose() * b);

        // Store coefficients
        for (int i = 0; i <= CHEB_ORDER; i++) {
            chebCoeffs[i] = coeffs(i);
        }

        return coeffs(0);  // Return DC component
    }

    // RC response validation using derivatives
    float validateRCResponse(const float* samples, int count) {
        std::vector<float> firstDerivs(count);
        std::vector<float> secondDerivs(count);
        float tMax = deltaT * (count - 1);
        
        for (int i = 0; i < count; i++) {
            float t = i * deltaT;
            float x = mapToChebDomain(t, tMax);
            
            firstDerivs[i] = chebyshevDerivative(CHEB_ORDER, 1, x);
            secondDerivs[i] = chebyshevDerivative(CHEB_ORDER, 2, x);
        }
        
        // Find inflection point
        float estimatedTau = 0.0f;
        for (int i = 1; i < count - 1; i++) {
            if (secondDerivs[i-1] * secondDerivs[i+1] < 0) {
                float inflectionTime = i * deltaT;
                estimatedTau = inflectionTime * (1.0f + 0.5f * std::abs(firstDerivs[i] / 
                    *std::max_element(firstDerivs.begin(), firstDerivs.end())));
                break;
            }
        }
        
        return estimatedTau;
    }

    // Advanced blending factor calculation
    float calculateBlendingFactor() {
        float coeffNorm = 0.0f;
        float coeffVar = 0.0f;
        
        for (int i = 1; i <= CHEB_ORDER; i++) {
            coeffNorm += std::abs(chebCoeffs[i]);
            coeffVar += chebCoeffs[i] * chebCoeffs[i];
        }
        
        coeffVar = std::sqrt(coeffVar / CHEB_ORDER);
        float complexityFactor = coeffNorm / (std::abs(chebCoeffs[0]) + 1e-6f);
        
        // Incorporate historical trend
        float historicalTrend = 0.0f;
        if (historyIndex > 0) {
            int count = std::min(5, historyIndex);
            for (int i = 1; i <= count; i++) {
                int idx = (historyIndex - i + HISTORY_DEPTH) % HISTORY_DEPTH;
                historicalTrend += history[idx].derivativeCharacteristic;
            }
            historicalTrend /= count;
        }
        
        float adaptiveBlend = 0.3f * (1.0f + 
            0.5f * std::tanh(complexityFactor - 1.0f) +
            -0.3f * std::exp(-prediction_error_sum / currentSamplesPerRead) +
            -0.2f * std::tanh(coeffVar) +
            0.2f * std::tanh(historicalTrend)
        );
        
        return std::max(0.1f, std::min(0.7f, adaptiveBlend));
    }

    // Update historical data
    void updateHistory(float value, float timeConstant, float derivChar) {
        historyIndex = (historyIndex + 1) % HISTORY_DEPTH;
        
        history[historyIndex] = {
            value,
            timeConstant,
            derivChar,
            micros(),
            {0}  // Initialize coefficients array
        };
        
        // Copy current coefficients
        std::copy(chebCoeffs, chebCoeffs + CHEB_ORDER + 1, 
                 history[historyIndex].coefficients);
    }

    // Compute derivative characteristics
    float computeDerivativeCharacteristic(const float* samples, int count) {
        std::vector<float> derivatives(count - 1);
        for (int i = 1; i < count; i++) {
            derivatives[i-1] = (samples[i] - samples[i-1]) / deltaT;
        }
        
        float meanDeriv = std::accumulate(derivatives.begin(), derivatives.end(), 0.0f) / 
                         derivatives.size();
        
        float variance = 0.0f;
        for (const float& deriv : derivatives) {
            variance += std::pow(deriv - meanDeriv, 2);
        }
        
        return std::sqrt(variance / derivatives.size());
    }

public:
    AdvancedKalmanADC() {
        reset();
        updateRCParameters();
    }

    void setRCParameters(float r_ohms, float c_farads) {
        resistance = r_ohms;
        capacitance = c_farads;
        updateRCParameters();
    }

    void updateRCParameters() {
        tau = resistance * capacitance;
        cutoffFreq = 1.0f / (2.0f * M_PI * tau);
    }

    void setADCParameters(float vref_mv, int resolution, float gain = 1.0f) {
        adcVref = vref_mv;
        adcResolution = resolution;
        inputGain = gain;
        R = std::pow(2, -adcResolution) * adcVref;
    }

    float read(int pin, int samplesPerRead = 10) {
        currentSamplesPerRead = samplesPerRead;
        samplesPerRead = std::min(64, std::max(2, samplesPerRead));
        std::vector<float> samples(samplesPerRead);
        
        unsigned long startTime = micros();
        for (int i = 0; i < samplesPerRead; i++) {
            unsigned long subsampleTime = micros();
            samples[i] = analogReadMilliVolts(pin) / 1000.0f;
            
            // Subsampling averaging
            constexpr float SUBSAMPLING_AVERAGING = 9.0f;
            int subsamples = 0;
            while (micros() - subsampleTime < static_cast<unsigned long>(tau * 1e6f / samplesPerRead)) {
                samples[i] = (analogReadMilliVolts(pin) / 1000.0f + 
                            (SUBSAMPLING_AVERAGING - 1.0f) * samples[i]) / SUBSAMPLING_AVERAGING;
                subsamples++;
            }
        }
        
        deltaT = (micros() - startTime) / 1e6f / samplesPerRead;

        // Dynamic noise estimation
        float measuredNoise = 0.0f;
        for (int i = 1; i < samplesPerRead; i++) {
            measuredNoise += std::pow(samples[i] - samples[i-1], 2);
        }
        measuredNoise = std::sqrt(measuredNoise / (samplesPerRead - 1));
        R = 0.9f * R + 0.1f * (measuredNoise * measuredNoise);

        // Fit Chebyshev polynomials with historical context
        float fittedValue = fitChebyshevWithHistory(samples.data(), samplesPerRead, deltaT);
        
        // Validate RC response
        float estimatedTau = validateRCResponse(samples.data(), samplesPerRead);

// Continuing from the read() method...
        
        // Compute derivative characteristics for trend analysis
        float derivChar = computeDerivativeCharacteristic(samples.data(), samplesPerRead);
        
        // Calculate prediction error
        prediction_error_sum = 0.0f;
        float tMax = deltaT * samplesPerRead;
        for (int i = 0; i < samplesPerRead; i++) {
            float t = i * deltaT;
            float x = mapToChebDomain(t, tMax);
            float predicted = 0.0f;
            
            for (int j = 0; j <= CHEB_ORDER; j++) {
                predicted += chebCoeffs[j] * chebyshevPolynomial(j, x);
            }
            prediction_error_sum += std::pow(predicted - samples[i], 2);
        }
        
        // Adaptive process noise based on prediction error and RC validation
        float rcDeviation = std::abs(estimatedTau - tau) / tau;
        Q = std::max(0.001f, std::min(0.1f, 
            0.01f * prediction_error_sum / samplesPerRead + 
            0.05f * rcDeviation
        ));

        // Kalman filter update
        P = P + Q;
        K = P / (P + R);
        x = x + K * (fittedValue - x);
        P = (1.0f - K) * P;

        // Apply historical correction using weighted blend
        float blendFactor = calculateBlendingFactor();
        
        if (historyIndex > 0) {
            float historicalEstimate = 0.0f;
            int validCount = 0;
            
            // Compute weighted historical average
            for (int i = 1; i <= std::min(5, historyIndex); i++) {
                int idx = (historyIndex - i + HISTORY_DEPTH) % HISTORY_DEPTH;
                float weight = std::exp(-static_cast<float>(i) / 3.0f);  // Exponential decay
                historicalEstimate += weight * history[idx].value;
                validCount++;
            }
            
            if (validCount > 0) {
                historicalEstimate /= validCount;
                // Blend current estimate with historical data
                x = (1.0f - blendFactor) * x + blendFactor * historicalEstimate;
            }
        }

        // Update historical data
        updateHistory(x, estimatedTau, derivChar);

        return x;
    }

    // Reset filter state
    void reset() {
        P = 1.0f;
        x = 0.0f;
        historyIndex = 0;
        prediction_error_sum = 0.0f;
        
        // Reset coefficients and history
        std::fill(chebCoeffs, chebCoeffs + CHEB_ORDER + 1, 0.0f);
        for (int i = 0; i < HISTORY_DEPTH; i++) {
            history[i] = {0.0f, 0.0f, 0.0f, 0, {0}};
        }
    }

    // Diagnostic functions
    float getCutoffFrequency() { return cutoffFreq; }
    float getTimeConstant() { return tau; }
    float getResistance() { return resistance; }
    float getCapacitance() { return capacitance; }
    float getCurrentBlendFactor() { return calculateBlendingFactor(); }
    float getLastPredictionError() { return prediction_error_sum / currentSamplesPerRead; }
    
    // Get current filter state
    struct FilterState {
        float estimatedValue;
        float processNoise;
        float measurementNoise;
        float kalmanGain;
        float errorCovariance;
        float blendFactor;
        float lastTimeConstant;
    };
    
    FilterState getFilterState() {
        return {
            x,              // Current estimate
            Q,              // Process noise
            R,              // Measurement noise
            K,              // Kalman gain
            P,              // Error covariance
            calculateBlendingFactor(),
            history[historyIndex].timeConstant
        };
    }

    // Get Chebyshev coefficients for analysis
    std::vector<float> getChebyshevCoefficients() {
        return std::vector<float>(chebCoeffs, chebCoeffs + CHEB_ORDER + 1);
    }

    // Theoretical RC response for comparison
    float theoreticalRCResponse(float t, float initialValue, float finalValue) {
        return finalValue - (finalValue - initialValue) * std::exp(-t / tau);
    }
};
