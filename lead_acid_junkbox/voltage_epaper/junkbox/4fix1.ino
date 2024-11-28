struct MeasurementMonitor {
    float values[BUFFER_SIZE];      // Rolling buffer for measurements
    unsigned long times[BUFFER_SIZE]; // Rolling buffer for timestamps
    float errors[4];               // Errors for each candidate model
    int count;                     // Number of samples in the buffer
    float threshold;               // Allowed deviation threshold
    float shadowValue;             // Shadowed (smoothed) value
    float noiseModel;              // Estimated noise level
    float driftModel;              // Estimated drift rate
    float weights[4];              // Dynamic weights for candidate models
    float confidence[4];           // Confidence for candidate models
    float derivatives[BUFFER_SIZE - 1]; // Stores calculated derivatives
};

// Declare the monitor
MeasurementMonitor monitor;

// Explicitly initialize the monitor in setup
void initializeMonitor(MeasurementMonitor &mon) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        mon.values[i] = 0.0f;
        mon.times[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        mon.errors[i] = 0.0f;
        mon.weights[i] = 0.25f;
        mon.confidence[i] = 1.0f;
    }
    for (int i = 0; i < BUFFER_SIZE - 1; i++) {
        mon.derivatives[i] = 0.0f;
    }
    mon.count = 0;
    mon.threshold = 0.1f;
    mon.shadowValue = 0.0f;
    mon.noiseModel = 0.0f;
    mon.driftModel = 0.0f;
}

void setup() {
    initializeMonitor(monitor);
}

void loop() {
    // Your main code here
}
