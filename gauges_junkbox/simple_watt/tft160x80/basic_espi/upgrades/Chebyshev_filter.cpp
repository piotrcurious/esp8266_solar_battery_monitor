#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
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
    static const int CHEB_ORDER = 5;
    float chebCoeffs[CHEB_ORDER + 1];
    float lastDeltaT = 0.001;  // Store last deltaT for predictions
    float lastTMax = 0.001;    // Store last tMax for predictions
    float lastMinSample = 0.0; // Store last min sample for predictions
    float lastMaxSample = 1.0; // Store last max sample for predictions
    
    // ADC parameters
    float adcVref = 3300.0;  // ADC reference voltage in mV
    int adcResolution = 12;  // ADC resolution in bits
    float inputGain = 1.0;   // Input voltage divider ratio

    // Calculate nth Chebyshev polynomial value - iterative implementation to avoid recursion
    float chebyshevT(int n, float x) {
        if (n == 0) return 1.0;
        if (n == 1) return x;
        
        float Tn_2 = 1.0;  // T_0(x)
        float Tn_1 = x;    // T_1(x)
        float Tn = 0.0;    // T_n(x)
        
        for (int i = 2; i <= n; i++) {
            Tn = 2.0 * x * Tn_1 - Tn_2;
            Tn_2 = Tn_1;
            Tn_1 = Tn;
        }
        
        return Tn;
    }

    // Map time domain to [-1, 1]
    float mapTimeToChebDomain(float t, float tMax) {
        return 2.0 * (t / tMax) - 1.0;
    }

    // Theoretical RC response model for a step input
    float theoreticalRCResponse(float t, float minValue, float maxValue) {
        // First-order RC step response: V(t) = Vfinal + (Vinitial - Vfinal) * e^(-t/RC)
        float amplitude = maxValue - minValue;
        float offset = minValue;
        return maxValue - amplitude * exp(-t / tau) + offset;
    }

    // Fit Chebyshev polynomials to the sample data using weighted least squares
    void fitRCResponseChebyshev(const float* samples, int count, float deltaT) {
        float tMax = deltaT * (count - 1);
        lastDeltaT = deltaT;
        lastTMax = tMax;
        
        // Calculate minimum and maximum values of the sample set
        float minSample = *std::min_element(samples, samples + count);
        float maxSample = *std::max_element(samples, samples + count);
        lastMinSample = minSample;
        lastMaxSample = maxSample;
        
        // Initialize matrices for least squares fitting
        MatrixXd A(count, CHEB_ORDER + 1);
        VectorXd b(count);
        VectorXd weights(count);
        
        // Build weighted system of equations
        for (int i = 0; i < count; i++) {
            float t = i * deltaT;
            float x = mapTimeToChebDomain(t, tMax);
            
            // Weight samples based on theoretical RC response
            float theoretical = theoreticalRCResponse(t, minSample, maxSample);
            float weight = 1.0 / (1.0 + std::abs(samples[i] - theoretical));
            weights(i) = weight;
            
            for (int j = 0; j <= CHEB_ORDER; j++) {
                A(i, j) = chebyshevT(j, x) * weight;
            }
            b(i) = samples[i] * weight;
        }
        
        // Solve weighted least squares
        VectorXd coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);
        
        // Store coefficients for future use
        for (int i = 0; i <= CHEB_ORDER; i++) {
            chebCoeffs[i] = coeffs(i);
        }
    }

    // Predict signal value using the full Chebyshev polynomial fit
    float predictRCResponse(float t) {
        float x = mapTimeToChebDomain(t, lastTMax);
        float value = 0.0;
        
        // Calculate the full Chebyshev polynomial sum
        for (int i = 0; i <= CHEB_ORDER; i++) {
            value += chebCoeffs[i] * chebyshevT(i, x);
        }
        
        // Blend with theoretical response for extrapolation robustness
        float theoretical = theoreticalRCResponse(t, lastMinSample, lastMaxSample);
        float blendFactor = 0.7;  // Adjust the blend factor based on your needs
        
        // Use more of the theoretical model when extrapolating beyond the fitted range
        if (t > lastTMax) {
            float extrapolationFactor = constrain((t - lastTMax) / lastTMax, 0.0, 1.0);
            blendFactor = blendFactor * (1.0 - extrapolationFactor);
        }
        
        return blendFactor * value + (1.0 - blendFactor) * theoretical;
    }

    // Get convergence estimate using the sum of Chebyshev coefficients
    float getConvergenceEstimate() {
        float sum = 0.0;
        // Higher order coefficients should decrease in magnitude for a converging series
        for (int i = 1; i <= CHEB_ORDER; i++) {
            sum += abs(chebCoeffs[i]);
        }
        // Calculate ratio of higher-order terms to DC component
        return (abs(chebCoeffs[0]) > 0.0001) ? sum / abs(chebCoeffs[0]) : 100.0;
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

        #define SUBSAMPLES_AMOUNT 16  
        #define SUBSAMPLING_AVERAGING 9.0
        int subsamples = 0;
        
        unsigned long startTime = micros();
        for (int i = 0; i < samplesPerRead; i++) {
            uint32_t subsample_time = micros();
            samples[i] = (analogReadMilliVolts(pin)/1000.0); // Initial sample
            
            // Continue averaging during time based on RC time constant
            while (micros() - subsample_time < (int(tau * 1000000.0 / samplesPerRead))) {
                samples[i] = ((analogReadMilliVolts(pin)/1000.0) + (SUBSAMPLING_AVERAGING-1)*samples[i])/SUBSAMPLING_AVERAGING;
                subsamples++;
            }
        }
        
        float deltaT = (micros() - startTime) / 1000000.0 / samplesPerRead;
        
        // Dynamic noise estimation
        float measuredNoise = estimateNoise(samples, samplesPerRead);
        R = 0.9 * R + 0.1 * (measuredNoise * measuredNoise);  // Exponential moving average

        // Calculate min and max of samples
        float minSample = *std::min_element(samples, samples + samplesPerRead);
        float maxSample = *std::max_element(samples, samples + samplesPerRead);

        // Fit current samples with full Chebyshev model
        fitRCResponseChebyshev(samples, samplesPerRead, deltaT);
        
        // Extrapolate to the next time step
        float fittedValue = predictRCResponse(deltaT * samplesPerRead);
        
        // Get a measure of fit convergence and adjust process noise
        float convergence = getConvergenceEstimate();
        
        // Calculate the prediction error for the entire dataset
        float prediction_error_sum = 0.0f;
        for (int i = 0; i < samplesPerRead; i++) {
            float predicted = predictRCResponse(i * deltaT);
            prediction_error_sum += pow(predicted - samples[i], 2);
        }
        float prediction_error = prediction_error_sum / samplesPerRead;

        // Adapt process noise based on prediction error and convergence
        Q = 0.1 * prediction_error * (1.0 + convergence * 0.1);
        Q = constrain(Q, 0.001, 1.0);  // Ensure Q stays in reasonable bounds
        
        // Kalman filter prediction
        P = P + Q;

        // Kalman filter update
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
            float predicted = predictRCResponse(deltaT * samplesPerRead);
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
    
    // Get Chebyshev coefficient at index
    float getChebyshevCoefficient(int index) {
        if (index >= 0 && index <= CHEB_ORDER) {
            return chebCoeffs[index];
        }
        return 0.0;
    }
    
    // Reset filter state
    void reset() {
        P = 1.0;
        x = 0.0;
        historyIndex = 0;
        for (int i = 0; i < HISTORY_SIZE; i++) history[i] = 0;
        for (int i = 0; i <= CHEB_ORDER; i++) chebCoeffs[i] = 0;
        lastDeltaT = 0.001;
        lastTMax = 0.001;
        lastMinSample = 0.0;
        lastMaxSample = 1.0;
    }
};
