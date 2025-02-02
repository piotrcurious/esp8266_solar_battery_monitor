#include <Eigen.h>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

class AdvancedKalmanADC {
private:
    // Extended state vector: [voltage, current, tau_effective]
    VectorXd state;        // State estimate
    MatrixXd P;           // State covariance
    MatrixXd Q;           // Process noise
    float R;              // Measurement noise
    
    // RC filter parameters
    float nominal_tau;     // Nominal RC time constant
    float resistance;      // Input resistance (ohms)
    float capacitance;     // Input capacitance (farads)
    float cutoffFreq;      // Cutoff frequency (Hz)
    
    // Chebyshev fitting parameters
    static const int CHEB_ORDER = 4;
    float chebCoeffs[CHEB_ORDER + 1];
    
    // Buffer for historical data
    static const int HISTORY_SIZE = 32;
    struct Sample {
        float voltage;
        float current;
        float timestamp;
    };
    Sample history[HISTORY_SIZE];
    int historyIndex = 0;
    
    // ADC parameters
    float adcVref = 3300.0;  // ADC reference voltage in mV
    int adcResolution = 12;  // ADC resolution in bits
    float inputGain = 1.0;   // Input voltage divider ratio

    // Initialize state matrices
    void initializeStateMatrices() {
        state = VectorXd::Zero(3);
        P = MatrixXd::Identity(3, 3);
        Q = MatrixXd::Identity(3, 3) * 1e-6;
        P(2,2) = 0.1;  // Higher initial uncertainty for tau_effective
        Q(2,2) = 1e-7; // Slower variation for tau_effective
    }

    // State transition model
    MatrixXd getStateTransition(float dt) {
        MatrixXd F(3, 3);
        float tau = state(2);  // Current effective tau
        float exp_term = exp(-dt/tau);
        
        F << exp_term, tau*(1-exp_term), 0,
             -exp_term/resistance, exp_term, 0,
             0, 0, 1;
        
        return F;
    }

    // Measurement model
    VectorXd getMeasurement(float voltage) {
        VectorXd z(1);
        z << voltage;
        return z;
    }

    // Measurement matrix
    MatrixXd getMeasurementMatrix() {
        MatrixXd H(1, 3);
        H << 1, 0, 0;  // We measure voltage directly
        return H;
    }

    // Update RC model parameters
    void updateRCParameters(float dt) {
        // Estimate effective tau from recent history
        if (historyIndex > 1) {
            float sum_tau = 0;
            int count = 0;
            
            for (int i = 1; i < min(historyIndex, HISTORY_SIZE); i++) {
                int prev_idx = (historyIndex - i - 1 + HISTORY_SIZE) % HISTORY_SIZE;
                int curr_idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
                
                float dv = history[curr_idx].voltage - history[prev_idx].voltage;
                float i_avg = (history[curr_idx].current + history[prev_idx].current) / 2;
                float dt = history[curr_idx].timestamp - history[prev_idx].timestamp;
                
                if (abs(dv) > 1e-3 && abs(i_avg) > 1e-6) {  // Avoid division by zero
                    float tau_est = -dt / log(1 - dv/(i_avg * resistance));
                    if (tau_est > 0 && tau_est < nominal_tau * 5) {  // Sanity check
                        sum_tau += tau_est;
                        count++;
                    }
                }
            }
            
            if (count > 0) {
                float tau_measured = sum_tau / count;
                state(2) = 0.95 * state(2) + 0.05 * tau_measured;  // Smooth update
            }
        }
    }

    // Chebyshev polynomial evaluation
    float chebyshevT(int n, float x) {
        if (n == 0) return 1.0;
        if (n == 1) return x;
        return 2.0 * x * chebyshevT(n - 1, x) - chebyshevT(n - 2, x);
    }

    // Predict RC response using current state
    float predictVoltage(float dt) {
        float tau = state(2);
        float v = state(0);
        float i = state(1);
        
        return v * exp(-dt/tau) + i * resistance * (1 - exp(-dt/tau));
    }

    // Update history buffer
    void updateHistory(float voltage, float current, float timestamp) {
        history[historyIndex] = {voltage, current, timestamp};
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    }

public:
    AdvancedKalmanADC() : state(3), P(3,3), Q(3,3) {
        initializeStateMatrices();
        nominal_tau = 0.001;  // Default 1ms
        resistance = 10000;   // Default 10k
        capacitance = 0.1e-6; // Default 0.1µF
        updateRCParameters();
    }

    void setRCParameters(float r_ohms, float c_farads) {
        resistance = r_ohms;
        capacitance = c_farads;
        nominal_tau = r_ohms * c_farads;
        state(2) = nominal_tau;  // Initialize effective tau
        updateRCParameters();
    }

    void updateRCParameters() {
        cutoffFreq = 1.0 / (2.0 * PI * nominal_tau);
    }

    void setADCParameters(float vref_mv, int resolution, float gain = 1.0) {
        adcVref = vref_mv;
        adcResolution = resolution;
        inputGain = gain;
        R = pow(2, -adcResolution) * adcVref * 0.1;  // Base measurement noise
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

        // Predict step
        float dt = timestamps[samplesPerRead-1] - timestamps[0];
        MatrixXd F = getStateTransition(dt);
        
        // State prediction
        VectorXd state_pred = F * state;
        MatrixXd P_pred = F * P * F.transpose() + Q;

        // Update RC parameters based on measurements
        updateRCParameters(dt);

        // Calculate innovation
        VectorXd z = getMeasurement(samples[samplesPerRead-1]);
        MatrixXd H = getMeasurementMatrix();
        VectorXd innovation = z - H * state_pred;

        // Update step
        MatrixXd S = H * P_pred * H.transpose();
        S(0,0) += R;
        MatrixXd K = P_pred * H.transpose() * S.inverse();
        
        state = state_pred + K * innovation;
        P = (MatrixXd::Identity(3,3) - K * H) * P_pred;

        // Calculate current
        float current = (samples[samplesPerRead-1] - state(0)) / resistance;
        state(1) = current;  // Update current estimate

        // Update history
        updateHistory(state(0), current, timestamps[samplesPerRead-1]);

        return state(0);  // Return voltage estimate
    }

    // Getters for diagnostics
    float getEffectiveTau() { return state(2); }
    float getCurrent() { return state(1); }
    float getVoltage() { return state(0); }
    float getNominalTau() { return nominal_tau; }
    
    void reset() {
        initializeStateMatrices();
        historyIndex = 0;
        state(2) = nominal_tau;  // Reset effective tau to nominal
    }
};
