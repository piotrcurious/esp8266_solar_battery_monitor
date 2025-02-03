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
    // Constants
    static constexpr int CHEB_ORDER = 7;
    static constexpr int HISTORY_DEPTH = 64;
    static constexpr float CHEB_SCALE_FACTOR = 1.1f;
    static constexpr float MIN_BLEND_FACTOR = 0.1f;
    static constexpr float MAX_BLEND_FACTOR = 0.7f;
    static constexpr float SUBSAMPLING_AVERAGING = 9.0f;

    // Kalman filter state
    struct KalmanState {
        float Q = 0.001f;  // Process noise covariance
        float R = 0.1f;    // Measurement noise covariance
        float P = 1.0f;    // Estimation error covariance
        float K = 0.0f;    // Kalman gain
        float x = 0.0f;    // State estimate
    } kalman;

    // RC filter configuration
    struct RCConfig {
        float tau = 0.001f;         // RC time constant (seconds)
        float resistance = 10000.0f; // Input resistance (ohms)
        float capacitance = 0.1e-6f; // Input capacitance (farads)
        float cutoffFreq = 0.0f;     // Cutoff frequency (Hz)
    } rc;

    // ADC configuration
    struct ADCConfig {
        float vref = 3300.0f;     // ADC reference voltage in mV
        int resolution = 12;      // ADC resolution in bits
        float inputGain = 1.0f;   // Input voltage divider ratio
    } adc;

    // Signal processing state
    struct ProcessingState {
        float chebCoeffs[CHEB_ORDER + 1] = {0};
        float deltaT = 0.0f;
        float prediction_error_sum = 0.0f;
        int currentSamplesPerRead = 10;
    } proc;

    // Historical data structure with enhanced metadata
    struct SignalSnapshot {
        float value;
        float timeConstant;
        float derivativeCharacteristic;
        unsigned long timestamp;
        float signalToNoiseRatio;
        float coefficients[CHEB_ORDER + 1];
        
        SignalSnapshot() : 
            value(0), timeConstant(0), derivativeCharacteristic(0),
            timestamp(0), signalToNoiseRatio(0) {
            std::fill_n(coefficients, CHEB_ORDER + 1, 0);
        }
    };

    std::vector<SignalSnapshot> history;
    int historyIndex = 0;

    // Enhanced Chebyshev polynomial calculation using lookup table
    static constexpr int CHEB_CACHE_SIZE = 32;
    float chebCache[CHEB_CACHE_SIZE][CHEB_ORDER + 1] = {{0}};
    bool chebCacheInitialized = false;

    void initializeChebCache() {
        if (chebCacheInitialized) return;
        
        for (int i = 0; i < CHEB_CACHE_SIZE; i++) {
            float x = -1.0f + (2.0f * i) / (CHEB_CACHE_SIZE - 1);
            for (int n = 0; n <= CHEB_ORDER; n++) {
                chebCache[i][n] = computeChebyshevPolynomial(n, x);
            }
        }
        chebCacheInitialized = true;
    }

    float computeChebyshevPolynomial(int n, float x) {
        if (n <= 4) {
            switch(n) {
                case 0: return 1.0f;
                case 1: return x;
                case 2: return 2.0f * x * x - 1.0f;
                case 3: return x * (4.0f * x * x - 3.0f);
                case 4: return 8.0f * x * x * x * x - 8.0f * x * x + 1.0f;
            }
        }

        // Use Clenshaw recurrence for higher orders
        float b_km2 = 0, b_km1 = 0, b_k;
        for (int k = n; k >= 0; k--) {
            b_k = (k == n) ? x : 2.0f * x * b_km1 - b_km2;
            b_km2 = b_km1;
            b_km1 = b_k;
        }
        return b_km1 - x * b_km2;
    }

    float interpolateChebyshev(int n, float x) {
        float scaledX = (x + 1.0f) * (CHEB_CACHE_SIZE - 1) / 2.0f;
        int idx = static_cast<int>(scaledX);
        float frac = scaledX - idx;
        
        if (idx >= CHEB_CACHE_SIZE - 1) return chebCache[CHEB_CACHE_SIZE - 1][n];
        if (idx < 0) return chebCache[0][n];
        
        return chebCache[idx][n] * (1.0f - frac) + chebCache[idx + 1][n] * frac;
    }

    // Improved domain mapping with adaptive scaling
    float mapToChebDomain(float t, float tMax) {
        float normalizedT = t / tMax;
        float scaledT = std::tanh(normalizedT * M_PI) * CHEB_SCALE_FACTOR;
        return std::max(-1.0f, std::min(1.0f, scaledT));
    }

    // Enhanced signal quality assessment
    float assessSignalQuality(const std::vector<float>& samples) {
        if (samples.size() < 3) return 0.0f;
        
        float mean = std::accumulate(samples.begin(), samples.end(), 0.0f) / samples.size();
        float variance = 0.0f;
        float shortTermVariance = 0.0f;
        
        for (size_t i = 0; i < samples.size(); i++) {
            variance += std::pow(samples[i] - mean, 2);
            if (i > 0) {
                shortTermVariance += std::pow(samples[i] - samples[i-1], 2);
            }
        }
        
        variance /= samples.size();
        shortTermVariance /= (samples.size() - 1);
        
        float signalToNoise = variance > 0 ? 
            std::sqrt(variance / (shortTermVariance + 1e-6f)) : 0.0f;
            
        return std::min(1.0f, std::max(0.0f, signalToNoise));
    }

public:
    AdvancedKalmanADC() : history(HISTORY_DEPTH) {
        reset();
        updateRCParameters();
        initializeChebCache();
    }

    // Configuration methods
    void setRCParameters(float r_ohms, float c_farads) {
        rc.resistance = std::max(1.0f, r_ohms);
        rc.capacitance = std::max(1e-12f, c_farads);
        updateRCParameters();
    }

    void updateRCParameters() {
        rc.tau = rc.resistance * rc.capacitance;
        rc.cutoffFreq = 1.0f / (2.0f * M_PI * rc.tau);
    }

    void setADCParameters(float vref_mv, int resolution, float gain = 1.0f) {
        adc.vref = std::max(100.0f, vref_mv);
        adc.resolution = std::clamp(resolution, 8, 32);
        adc.inputGain = std::max(0.001f, gain);
        kalman.R = std::pow(2.0f, -adc.resolution) * adc.vref;
    }

    // Main reading method with improved error handling and adaptive sampling
    float read(int pin, int samplesPerRead = 10) {
        if (pin < 0) return 0.0f;
        
        proc.currentSamplesPerRead = std::clamp(samplesPerRead, 2, 64);
        std::vector<float> samples(proc.currentSamplesPerRead);
        
        // Improved sampling with timing verification
        unsigned long startTime = micros();
        unsigned long maxSampleTime = static_cast<unsigned long>(rc.tau * 1e6f);
        
        for (int i = 0; i < proc.currentSamplesPerRead; i++) {
            unsigned long sampleStart = micros();
            samples[i] = analogReadMilliVolts(pin) / 1000.0f;
            
            // Adaptive subsampling
            int subsamples = 0;
            unsigned long elapsed;
            do {
                float newSample = analogReadMilliVolts(pin) / 1000.0f;
                samples[i] = (newSample + SUBSAMPLING_AVERAGING * samples[i]) / 
                            (SUBSAMPLING_AVERAGING + 1.0f);
                subsamples++;
                elapsed = micros() - sampleStart;
            } while (elapsed < maxSampleTime / proc.currentSamplesPerRead && 
                    subsamples < 16);
        }
        
        proc.deltaT = (micros() - startTime) / 1e6f / proc.currentSamplesPerRead;

        // Signal quality assessment
        float signalQuality = assessSignalQuality(samples);
        
        // Update Kalman filter parameters
        updateKalmanParameters(samples, signalQuality);
        
        // Process measurement
        float measurement = processSignal(samples, signalQuality);
        
        // Update filter state
        kalman.x = updateEstimate(measurement);
        
        // Update history
        updateHistory(measurement, signalQuality);
        
        return kalman.x;
    }

    // Reset all state
    void reset() {
        kalman = KalmanState();
        proc.prediction_error_sum = 0.0f;
        historyIndex = 0;
        std::fill_n(proc.chebCoeffs, CHEB_ORDER + 1, 0.0f);
        std::fill(history.begin(), history.end(), SignalSnapshot());
    }

    // Diagnostic methods
    float getCutoffFrequency() const { return rc.cutoffFreq; }
    float getTimeConstant() const { return rc.tau; }
    float getResistance() const { return rc.resistance; }
    float getCapacitance() const { return rc.capacitance; }
    float getCurrentBlendFactor() const { return calculateBlendingFactor(); }
    float getLastPredictionError() const { 
        return proc.prediction_error_sum / proc.currentSamplesPerRead; 
    }

    // Enhanced state reporting
    struct FilterState {
        float estimatedValue;
        float processNoise;
        float measurementNoise;
        float kalmanGain;
        float errorCovariance;
        float blendFactor;
        float signalQuality;
        float lastTimeConstant;
        
        FilterState(const KalmanState& k, const SignalSnapshot& h, float bf) :
            estimatedValue(k.x),
            processNoise(k.Q),
            measurementNoise(k.R),
            kalmanGain(k.K),
            errorCovariance(k.P),
            blendFactor(bf),
            signalQuality(h.signalToNoiseRatio),
            lastTimeConstant(h.timeConstant) {}
    };

    FilterState getFilterState() const {
        return FilterState(
            kalman, 
            history[historyIndex], 
            calculateBlendingFactor()
        );
    }

private:
    float calculateBlendingFactor() const {
        if (historyIndex < 1) return MIN_BLEND_FACTOR;
        
        float signalComplexity = 0.0f;
        for (int i = 1; i <= CHEB_ORDER; i++) {
            signalComplexity += std::abs(proc.chebCoeffs[i]);
        }
        signalComplexity /= (std::abs(proc.chebCoeffs[0]) + 1e-6f);
        
        float historicalQuality = 0.0f;
        int count = std::min(5, historyIndex);
        for (int i = 1; i <= count; i++) {
            int idx = (historyIndex - i + HISTORY_DEPTH) % HISTORY_DEPTH;
            historicalQuality += history[idx].signalToNoiseRatio;
        }
        historicalQuality /= count;
        
        float blend = 0.3f * (
            1.0f + 
            0.5f * std::tanh(signalComplexity - 1.0f) +
            0.3f * historicalQuality +
            -0.2f * std::exp(-proc.prediction_error_sum / proc.currentSamplesPerRead)
        );
        
        return std::clamp(blend, MIN_BLEND_FACTOR, MAX_BLEND_FACTOR);
    }

    void updateKalmanParameters(const std::vector<float>& samples, float signalQuality) {
        // Update measurement noise based on signal quality
        float measuredNoise = 0.0f;
        for (size_t i = 1; i < samples.size(); i++) {
            measuredNoise += std::pow(samples[i] - samples[i-1], 2);
        }
        measuredNoise = std::sqrt(measuredNoise / (samples.size() - 1));
        
        // Exponential smoothing of measurement noise
        kalman.R = 0.9f * kalman.R + 0.1f * (measuredNoise * measuredNoise);
        
        // Adaptive process noise based on signal quality
        kalman.Q = std::max(0.001f, 0.1f * (1.0f - signalQuality));
    }

    float processSignal(const std::vector<float>& samples, float signalQuality) {
        // Fit Chebyshev polynomials
        float tMax = proc.deltaT * (samples.size() - 1);
        MatrixXd A(samples.size(), CHEB_ORDER + 1);
        VectorXd b(samples.size());
        
        for (size_t i = 0; i < samples.size(); i++) {
            float x = mapToChebDomain(i * proc.deltaT, tMax);
            for (int j = 0; j <= CHEB_ORDER; j++) {
                A(i, j) = interpolateChebyshev(j, x);
            }
            b(i) = samples[i];
        }
        
        // Solve with regularization
        MatrixXd reg = MatrixXd::Identity(CHEB_ORDER + 1, CHEB_ORDER + 1);
        reg(0, 0) = 0.0f;  // Preserve DC component
        
    VectorXd coeffs = (A.transpose() * A + 0.01f * reg).ldlt().solve(A.transpose() * b);

        // Store coefficients and compute prediction error
        proc.prediction_error_sum = 0.0f;
        for (int i = 0; i <= CHEB_ORDER; i++) {
            proc.chebCoeffs[i] = coeffs(i);
        }

        // Compute prediction error for each sample
        for (size_t i = 0; i < samples.size(); i++) {
            float predicted = 0.0f;
            float x = mapToChebDomain(i * proc.deltaT, tMax);
            for (int j = 0; j <= CHEB_ORDER; j++) {
                predicted += coeffs(j) * interpolateChebyshev(j, x);
            }
            proc.prediction_error_sum += std::pow(predicted - samples[i], 2);
        }

        return coeffs(0);  // Return DC component
    }

    float updateEstimate(float measurement) {
        // Update Kalman filter state
        kalman.P = kalman.P + kalman.Q;
        kalman.K = kalman.P / (kalman.P + kalman.R);
        float newEstimate = kalman.x + kalman.K * (measurement - kalman.x);
        kalman.P = (1.0f - kalman.K) * kalman.P;

        // Apply historical blending if available
        if (historyIndex > 0) {
            float blendFactor = calculateBlendingFactor();
            float historicalEstimate = 0.0f;
            float totalWeight = 0.0f;

            // Compute weighted historical average with exponential decay
            for (int i = 1; i <= std::min(5, historyIndex); i++) {
                int idx = (historyIndex - i + HISTORY_DEPTH) % HISTORY_DEPTH;
                float weight = std::exp(-static_cast<float>(i) / 3.0f) * 
                             history[idx].signalToNoiseRatio;
                historicalEstimate += weight * history[idx].value;
                totalWeight += weight;
            }

            if (totalWeight > 0.0f) {
                historicalEstimate /= totalWeight;
                newEstimate = (1.0f - blendFactor) * newEstimate + 
                             blendFactor * historicalEstimate;
            }
        }

        return newEstimate;
    }

    void updateHistory(float measurement, float signalQuality) {
        historyIndex = (historyIndex + 1) % HISTORY_DEPTH;
        
        // Compute derivative characteristics
        float derivChar = 0.0f;
        if (historyIndex > 0) {
            int prevIdx = (historyIndex - 1 + HISTORY_DEPTH) % HISTORY_DEPTH;
            derivChar = (measurement - history[prevIdx].value) / proc.deltaT;
        }

        // Update historical record
        history[historyIndex] = SignalSnapshot();
        history[historyIndex].value = measurement;
        history[historyIndex].timeConstant = rc.tau;
        history[historyIndex].derivativeCharacteristic = derivChar;
        history[historyIndex].timestamp = micros();
        history[historyIndex].signalToNoiseRatio = signalQuality;
        
        std::copy(proc.chebCoeffs, proc.chebCoeffs + CHEB_ORDER + 1,
                 history[historyIndex].coefficients);
    }

    // Utility method for theoretical RC response (useful for validation)
    float theoreticalRCResponse(float t, float initialValue, float finalValue) const {
        return finalValue - (finalValue - initialValue) * std::exp(-t / rc.tau);
    }
};
