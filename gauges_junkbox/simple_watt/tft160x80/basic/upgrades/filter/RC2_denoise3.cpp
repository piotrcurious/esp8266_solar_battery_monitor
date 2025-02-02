class ProcessNoiseAdapter {
private:
    // Signal characteristics tracking
    struct SignalStats {
        float innovation;      // Current innovation (measurement - prediction)
        float innovation_var;  // Innovation variance
        float residual;       // Filter residual
        float residual_var;   // Residual variance
        float dynamic_range;   // Signal dynamic range
    } stats;
    
    // Adaptive parameters
    const float MEMORY_FACTOR = 0.95;     // Exponential forgetting factor
    const float INNOVATION_THRESHOLD = 3.0; // Sigma threshold for innovation
    const int WINDOW_SIZE = 32;           // Statistics window size
    
    // Innovation history
    float innovation_history[32];
    float residual_history[32];
    int history_index = 0;
    
    // Process noise components
    struct NoiseComponents {
        float voltage_noise;   // Voltage state noise
        float current_noise;   // Current state noise
        float tau_noise;       // Time constant state noise
        float correlation;     // State noise correlation
    } noise;
    
    void updateInnovationStats(float new_innovation, float new_residual) {
        // Update history
        innovation_history[history_index] = new_innovation;
        residual_history[history_index] = new_residual;
        history_index = (history_index + 1) % WINDOW_SIZE;
        
        // Update statistics
        float innovation_sum = 0;
        float residual_sum = 0;
        float innovation_sq_sum = 0;
        float residual_sq_sum = 0;
        int valid_samples = 0;
        
        for (int i = 0; i < WINDOW_SIZE; i++) {
            if (abs(innovation_history[i]) > 0 || abs(residual_history[i]) > 0) {
                innovation_sum += innovation_history[i];
                residual_sum += residual_history[i];
                innovation_sq_sum += innovation_history[i] * innovation_history[i];
                residual_sq_sum += residual_history[i] * residual_history[i];
                valid_samples++;
            }
        }
        
        if (valid_samples > 0) {
            stats.innovation = innovation_sum / valid_samples;
            stats.residual = residual_sum / valid_samples;
            stats.innovation_var = innovation_sq_sum / valid_samples - pow(stats.innovation, 2);
            stats.residual_var = residual_sq_sum / valid_samples - pow(stats.residual, 2);
        }
    }
    
    void adaptNoiseComponents(float dt, float signal_bandwidth) {
        // Adapt voltage noise based on innovation statistics
        float innovation_factor = max(1.0f, 
            abs(stats.innovation) / (INNOVATION_THRESHOLD * sqrt(stats.innovation_var)));
        
        // Adapt current noise based on residual statistics
        float residual_factor = max(1.0f,
            abs(stats.residual) / (INNOVATION_THRESHOLD * sqrt(stats.residual_var)));
        
        // Base noise levels scaled by bandwidth
        float base_voltage_noise = stats.dynamic_range * 0.01f * (signal_bandwidth / 1000.0f);
        float base_current_noise = base_voltage_noise / 1000.0f; // Assuming 1kΩ reference
        
        // Update noise components with adaptive factors
        noise.voltage_noise = base_voltage_noise * innovation_factor;
        noise.current_noise = base_current_noise * residual_factor;
        noise.tau_noise = 0.01f * dt * max(innovation_factor, residual_factor);
        
        // Update correlation based on innovation/residual relationship
        float corr = 0;
        for (int i = 0; i < WINDOW_SIZE - 1; i++) {
            corr += innovation_history[i] * residual_history[i+1];
        }
        noise.correlation = constrain(corr / (WINDOW_SIZE * sqrt(stats.innovation_var * stats.residual_var)),
                                   -0.9f, 0.9f);
    }

public:
    ProcessNoiseAdapter() {
        memset(&stats, 0, sizeof(stats));
        memset(&noise, 0, sizeof(noise));
        memset(innovation_history, 0, sizeof(innovation_history));
        memset(residual_history, 0, sizeof(residual_history));
        
        // Initialize with conservative values
        noise.voltage_noise = 0.1f;
        noise.current_noise = 0.01f;
        noise.tau_noise = 0.001f;
        noise.correlation = 0.0f;
    }
    
    void updateStats(float innovation, float residual, float dynamic_range) {
        stats.dynamic_range = dynamic_range;
        updateInnovationStats(innovation, residual);
    }
    
    MatrixXd getProcessNoiseMatrix(float dt, float signal_bandwidth) {
        adaptNoiseComponents(dt, signal_bandwidth);
        
        MatrixXd Q(3, 3);
        
        // Build process noise matrix with correlation
        Q(0,0) = pow(noise.voltage_noise, 2);
        Q(1,1) = pow(noise.current_noise, 2);
        Q(2,2) = pow(noise.tau_noise, 2);
        
        // Add correlation between voltage and current
        Q(0,1) = Q(1,0) = noise.correlation * noise.voltage_noise * noise.current_noise;
        
        // Scale by time step
        Q *= dt;
        
        return Q;
    }
    
    // Diagnostic getters
    float getVoltageNoise() const { return noise.voltage_noise; }
    float getCurrentNoise() const { return noise.current_noise; }
    float getTauNoise() const { return noise.tau_noise; }
    float getCorrelation() const { return noise.correlation; }
    float getInnovationVar() const { return stats.innovation_var; }
    float getResidualVar() const { return stats.residual_var; }
};

class AdvancedKalmanADC {
    // ... [Previous code remains the same until private section] ...
private:
    ProcessNoiseAdapter processNoiseAdapter;
    
    // ... [Other private members remain the same] ...

public:
    float read(int pin, int requested_samples = 10) {
        // Get adaptive sampling parameters
        float sample_rate, sample_interval_us;
        int num_samples;
        sampler.getSamplingParams(sample_rate, num_samples, sample_interval_us);
        
        // [Previous sampling code remains the same until prediction step]
        
        // Predict step with adaptive process noise
        float dt = timestamps[num_samples-1] - timestamps[0];
        MatrixXd F = getStateTransition(dt);
        
        // Calculate innovation
        VectorXd predicted_state = F * state;
        float innovation = samples[num_samples-1] - (predicted_state(0));
        float residual = samples[num_samples-1] - state(0);
        
        // Update process noise adapter
        processNoiseAdapter.updateStats(innovation, residual, sampler.getAmplitude());
        
        // Get adaptive process noise matrix
        MatrixXd Q = processNoiseAdapter.getProcessNoiseMatrix(dt, sampler.getBandwidth());
        
        // State prediction with adaptive noise
        P = F * P * F.transpose() + Q;
        
        // [Rest of Kalman filter update steps remain the same]
        
        return state(0);
    }
    
    // New diagnostic methods
    float getVoltageProcessNoise() const { return processNoiseAdapter.getVoltageNoise(); }
    float getCurrentProcessNoise() const { return processNoiseAdapter.getCurrentNoise(); }
    float getTauProcessNoise() const { return processNoiseAdapter.getTauNoise(); }
    float getProcessNoiseCorrelation() const { return processNoiseAdapter.getCorrelation(); }
};
