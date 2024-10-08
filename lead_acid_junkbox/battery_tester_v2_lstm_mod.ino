// ... [Previous includes and constants remain the same]

#include <vector>
#include <cmath>

// LSTM-inspired plateau detection parameters
const int SEQUENCE_LENGTH = 60;  // Number of time steps to consider
const float LEARNING_RATE = 0.01;
const float FORGET_RATE = 0.1;
const float PLATEAU_THRESHOLD = 0.01;  // Adjusted threshold for plateau detection

// LSTM-inspired cell structure
struct LSTMCell {
    float cellState;
    float hiddenState;
    float forgetGate;
    float inputGate;
    float outputGate;
    float candidateState;
};

// Global variables for LSTM
std::vector<float> voltageSequence;
LSTMCell lstmCell = {0, 0, 0, 0, 0, 0};

// ... [Previous global variables remain the same]

// Function prototypes
float sigmoid(float x);
float tanh_approx(float x);
void updateLSTMCell(float input);
bool isPlateauDetected();

// ... [Previous function implementations remain the same up to findOutgassingVoltage]

bool findOutgassingVoltage(TestResult& result, unsigned long testStartTime) {
    float initialVoltage = batteryVoltage;
    unsigned long startTime = millis();
    bool outgassingDetected = false;
    
    voltageSequence.clear();  // Clear previous sequence
    
    while (!outgassingDetected) {
        if (!setCurrentWithFeedback(currentSetpoint)) return false;
        batteryVoltage = getFilteredVoltage();
        
        voltageSequence.push_back(batteryVoltage);
        if (voltageSequence.size() > SEQUENCE_LENGTH) {
            voltageSequence.erase(voltageSequence.begin());
        }
        
        updateLSTMCell(batteryVoltage);
        
        float elapsedHours = (millis() - startTime) / 3600000.0;
        float voltageRate = (batteryVoltage - initialVoltage) / elapsedHours;
        
        if (voltageRate >= OUTGAS_VOLTAGE_RATE && voltageSequence.size() == SEQUENCE_LENGTH) {
            if (isPlateauDetected()) {
                result.outgassingVoltage = batteryVoltage;
                outgassingDetected = true;
                break;
            }
        }
        
        currentSetpoint += CURRENT_STEP;
        if (isTestLimitExceeded(testStartTime)) return false;
        delay(CURRENT_SETTLE_TIME);
    }
    
    setCurrentWithFeedback(0);  // Stop current
    return outgassingDetected;
}

float sigmoid(float x) {
    return 1.0 / (1.0 + exp(-x));
}

float tanh_approx(float x) {
    if (x < -3) return -1;
    if (x > 3) return 1;
    return x * (27 + x * x) / (27 + 9 * x * x);  // Pade approximation
}

void updateLSTMCell(float input) {
    // Forget gate
    lstmCell.forgetGate = sigmoid(input + lstmCell.hiddenState);
    
    // Input gate
    lstmCell.inputGate = sigmoid(input + lstmCell.hiddenState);
    
    // Candidate state
    lstmCell.candidateState = tanh_approx(input + lstmCell.hiddenState);
    
    // Cell state update
    lstmCell.cellState = lstmCell.forgetGate * lstmCell.cellState + 
                         lstmCell.inputGate * lstmCell.candidateState;
    
    // Output gate
    lstmCell.outputGate = sigmoid(input + lstmCell.hiddenState);
    
    // Hidden state update
    lstmCell.hiddenState = lstmCell.outputGate * tanh_approx(lstmCell.cellState);
}

bool isPlateauDetected() {
    if (voltageSequence.size() < SEQUENCE_LENGTH) return false;
    
    float sumDifferences = 0;
    for (int i = 1; i < SEQUENCE_LENGTH; i++) {
        sumDifferences += abs(voltageSequence[i] - voltageSequence[i-1]);
    }
    
    float averageDifference = sumDifferences / (SEQUENCE_LENGTH - 1);
    
    // Use LSTM cell state to adjust the plateau detection threshold
    float adjustedThreshold = PLATEAU_THRESHOLD * (1 + lstmCell.cellState);
    
    return averageDifference < adjustedThreshold && lstmCell.hiddenState > 0.5;
}

// ... [Rest of the code remains the same]
