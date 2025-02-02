class AdaptiveSampler {
private:
    // Signal characteristics
    float signal_bandwidth;     // Estimated signal bandwidth
    float noise_floor;          // Current noise floor estimate
    float signal_amplitude;     // Current signal amplitude estimate
    float slew_rate;           // Maximum observed slew rate
    
    // Sampling parameters
    const float MIN_SAMPLE_RATE = 100;    // Minimum 100 Hz
    const float MAX_SAMPLE_RATE = 10000;  // Maximum 10 kHz
    const float MIN_SAMPLES = 4;          // Minimum samples per reading
    const float MAX_SAMPLES = 64;         // Maximum samples per reading
    
    // Adaptive thresholds
    const float SNR_THRESHOLD = 6.0;      // Minimum desired SNR in dB
    const float SLEW_THRESHOLD = 0.1;     // Slew rate threshold for rate increase
    
    // Moving statistics
    static const int STAT_BUFFER_SIZE = 32;
    float value_buffer[STAT_BUFFER_SIZE];
    float time_buffer[STAT_BUFFER_SIZE];
    int buffer_index = 0;
    
    // Calculate signal statistics
    void updateSignalStats() {
        if (buffer_index < 2) return;
        
        // Calculate slew rate and amplitude
        float max_slew = 0;
        float min_val = value_buffer[0];
        float max_val = value_buffer[0];
        
        for (int i = 1; i < buffer_index; i++) {
            float dt = time_buffer[i] - time_buffer[i-1];
            float dv = value_buffer[i] - value_buffer[i-1];
            float current_slew = abs(dv / dt);
            max_slew = max(max_slew, current_slew);
            
            min_val = min(min_val, value_buffer[i]);
            max_val = max(max_val, value_buffer[i]);
        }
        
        slew_rate = 0.9 * slew_rate + 0.1 * max_slew;  // Exponential smoothing
        signal_amplitude = 0.9 * signal_amplitude + 0.1 * (max_val - min_val);
        
        // Estimate bandwidth from slew rate and amplitude
        if (signal_amplitude > 0) {
            signal_bandwidth = min(MAX_SAMPLE_RATE/2, slew_rate / (2 * PI * signal_amplitude));
        }
    }
    
public:
    AdaptiveSampler() : 
        signal_bandwidth(100),  // Start with conservative bandwidth estimate
        noise_floor(1.0),      // Initial noise floor estimate (mV)
        signal_amplitude(100),  // Initial amplitude estimate (mV)
        slew_rate(1000)        // Initial slew rate estimate (mV/s)
    {
        memset(value_buffer, 0, sizeof(value_buffer));
        memset(time_buffer, 0, sizeof(time_buffer));
    }
    
    void updateNoiseFloor(float noise_estimate) {
        noise_floor = 0.95 * noise_floor + 0.05 * noise_estimate;
    }
    
    void addSample(float value, float timestamp) {
        if (buffer_index >= STAT_BUFFER_SIZE) {
            // Shift buffer
            for (int i = 1; i < STAT_BUFFER_SIZE; i++) {
                value_buffer[i-1] = value_buffer[i];
                time_buffer[i-1] = time_buffer[i];
            }
            buffer_index = STAT_BUFFER_SIZE - 1;
        }
        
        value_buffer[buffer_index] = value;
        time_buffer[buffer_index] = timestamp;
        buffer_index++;
        
        updateSignalStats();
    }
    
    // Get optimal sampling parameters
    void getSamplingParams(float& sample_rate, int& num_samples, float& sample_interval_us) {
        // Calculate minimum sample rate based on bandwidth
        float min_rate = max(MIN_SAMPLE_RATE, 2.1 * signal_bandwidth);  // Nyquist + margin
        
        // Adjust based on SNR
        float snr = 20 * log10(signal_amplitude / noise_floor);
        float rate_multiplier = max(1.0f, SNR_THRESHOLD / max(1.0f, snr));
        
        // Adjust based on slew rate
        if (slew_rate > SLEW_THRESHOLD * signal_amplitude) {
            rate_multiplier *= 1.5;
        }
        
        // Calculate final sample rate
        sample_rate = constrain(min_rate * rate_multiplier, MIN_SAMPLE_RATE, MAX_SAMPLE_RATE);
        
        // Calculate number of samples based on signal characteristics
        float base_samples = max(MIN_SAMPLES, sample_rate / signal_bandwidth);
        num_samples = constrain((int)base_samples, MIN_SAMPLES, MAX_SAMPLES);
        
        // Calculate sample interval in microseconds
        sample_interval_us = 1e6 / sample_rate;
    }
    
    float getBandwidth() const { return signal_bandwidth; }
    float getNoiseFloor() const { return noise_floor; }
    float getAmplitude() const { return signal_amplitude; }
    float getSlewRate() const { return slew_rate; }
};

class AdvancedKalmanADC {
    // ... [Previous code until private section remains the same] ...
private:
    AdaptiveSampler sampler;
    
    // Timing parameters
    unsigned long last_sample_time = 0;
    float current_sample_rate = 1000;  // Start at 1kHz
    
    float sampleWithTiming(int pin, float interval_us) {
        unsigned long current_time = micros();
        long wait_time = interval_us - (current_time - last_sample_time);
        
        if (wait_time > 0) {
            delayMicroseconds(wait_time);
        }
        
        float value = analogReadMilliVolts(pin) / inputGain;
        last_sample_time = micros();
        return value;
    }

public:
    float read(int pin, int requested_samples = 10) {
        // Get adaptive sampling parameters
        float sample_rate, sample_interval_us;
        int num_samples;
        sampler.getSamplingParams(sample_rate, num_samples, sample_interval_us);
        
        // Override number of samples if specifically requested
        if (requested_samples > 0) {
            num_samples = constrain(requested_samples, MIN_SAMPLES, MAX_SAMPLES);
        }
        
        // Collect samples with adaptive timing
        float samples[MAX_SAMPLES];
        float timestamps[MAX_SAMPLES];
        unsigned long start_time = micros();
        
        for (int i = 0; i < num_samples; i++) {
            samples[i] = sampleWithTiming(pin, sample_interval_us);
            timestamps[i] = (micros() - start_time) / 1e6;
            
            // Update sampler with new data
            sampler.addSample(samples[i], timestamps[i]);
        }
        
        // Update noise estimates
        float total_noise = noiseModel.getTotalNoise(state(1), sample_rate/2);
        sampler.updateNoiseFloor(total_noise);
        
        // Continue with Kalman filter processing...
        // [Rest of the read method remains the same]
        
        return state(0);
    }
    
    // New diagnostic methods
    float getCurrentBandwidth() const { return sampler.getBandwidth(); }
    float getCurrentNoiseFloor() const { return sampler.getNoiseFloor(); }
    float getCurrentSlewRate() const { return sampler.getSlewRate(); }
    float getCurrentAmplitude() const { return sampler.getAmplitude(); }
};
