#include <math.h>
#include <stdio.h>

class RC_Oscillator_Controller {
private:
    float v_target;
    float voc_est;
    float r_est;
    float load_est;
    float pwm_duty;
    bool discharging;

    // Estimation bins
    float sum_v_low, sum_i_low;
    int count_low;
    float sum_v_high, sum_i_high;
    int count_high;

public:
    RC_Oscillator_Controller() {
        voc_est = 20.0f;
        r_est = 5.0f;
        v_target = voc_est * 0.8f;
        load_est = 10.0f;
        pwm_duty = 0.0f;
        discharging = false;

        sum_v_low = sum_i_low = 0;
        count_low = 0;
        sum_v_high = sum_i_high = 0;
        count_high = 0;
    }

    void update(float v, float i_src) {
        // 1. Oscillator Logic with Hysteresis
        // Aim for +/- 5% oscillation around target
        float v_upper = v_target * 1.05f;
        float v_lower = v_target * 0.95f;

        if (discharging) {
            if (v < v_lower) discharging = false;
        } else {
            if (v > v_upper) discharging = true;
        }
        pwm_duty = discharging ? 1.0f : 0.0f;

        // 2. Data Collection for Estimation
        // Use bins at +/- 1% (much tighter to get more data points)
        float b_upper = v_target * 1.01f;
        float b_lower = v_target * 0.99f;

        if (v < b_lower) {
            sum_v_low += v;
            sum_i_low += i_src;
            count_low++;
        } else if (v > b_upper) {
            sum_v_high += v;
            sum_i_high += i_src;
            count_high++;
        }

        // 3. Periodic Parameter Update
        // Update when we have enough samples in both bins (reduced to 20 for faster response)
        if (count_low > 20 && count_high > 20) {
            float v_l = sum_v_low / count_low;
            float i_l = sum_i_low / count_low;
            float v_h = sum_v_high / count_high;
            float i_h = sum_i_high / count_high;

            float di = i_l - i_h; // Should be positive since i_l > i_h (v_l < v_h)
            if (fabs(di) > 0.001f) {
                float r_new = (v_h - v_l) / di;
                // Sanity check for R_int estimate
                if (r_new > 0.1f && r_new < 50.0f) {
                    // Slightly faster alpha for tracking transients
                    r_est = r_est * 0.8f + r_new * 0.2f;
                }

                // Calculate Voc using the fresh R_int and upper bin data
                float voc_new = v_h + i_h * r_est;
                // Clamp and filter Voc estimate
                if (voc_new > 5.0f && voc_new < 50.0f) {
                    voc_est = voc_est * 0.8f + voc_new * 0.2f;
                }

                // Update target for the oscillator
                v_target = voc_est * 0.8f;
            }

            // Reset bins for next batch
            sum_v_low = sum_i_low = 0;
            count_low = 0;
            sum_v_high = sum_i_high = 0;
            count_high = 0;
        }

        // 4. Load Monitoring (for telemetry/analysis)
        float r_load_instant = v / (i_src + 0.001f);
        load_est = load_est * 0.995f + r_load_instant * 0.005f;
    }

    float getPwmDuty() { return pwm_duty; }
    float getVocEst() { return voc_est; }
    float getRintEst() { return r_est; }
    float getLoadEst() { return load_est; }
};
