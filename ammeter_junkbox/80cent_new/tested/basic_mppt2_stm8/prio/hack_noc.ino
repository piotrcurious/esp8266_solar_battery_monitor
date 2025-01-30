// Pin definitions
#define PANEL_VOLTAGE_PIN 5
#define CONV1_VOLTAGE_PIN 4
#define CONV2_VOLTAGE_PIN 3
#define CONV1_PWM_PIN 23
#define CONV2_PWM_PIN 24

// System constants
const float VREF = 30.0;
const int ADC_RES = 1023;
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// Control parameters
const int HISTORY_SIZE = 10;
const int MC_ITERATIONS = 100;
const float LEARNING_RATE = 0.01;
const float MOMENTUM = 0.9;

// Hidden variables for optimization
typedef struct {
    float alpha;  // Learning rate modifier
    float beta;   // Momentum coefficient
} HiddenParams;

// System state
typedef struct {
    float voltage;
    float current;
    float power;
    int pwm1;
    int pwm2;
} SystemState;

// Circular buffer for historical data
SystemState history[HISTORY_SIZE];
int historyIndex = 0;

// Control variables
float panelVoltage, prevPanelVoltage;
float panelCurrent = 1.0;
float power, prevPower;
int pwmConv1 = 0;
int pwmConv2 = 0;
float openCircuitVoltage = 0.0;
float predictedVoc = 0.0;
float targetVoltage;
float priorityRatio = 0.7;  // Conv1:Conv2 priority ratio

// Timing
unsigned long mpptInterval = 100;
unsigned long vocInterval = 30000;
unsigned long lastMpptCheck = 0;
unsigned long lastVocCheck = 0;

// Hidden parameters with initial values
HiddenParams hiddenParams = {
    .alpha = 0.5,
    .beta = 0.3
};

// Function to calculate system error
float calculateError(const SystemState* state) {
    float voltageError = fabs(state->voltage - targetVoltage);
    float powerError = (prevPower > 0) ? (state->power - prevPower) / prevPower : 0;
    return voltageError * 0.7 + powerError * 0.3;
}

// Gradient descent optimization for hidden parameters
void optimizeHiddenParams() {
    float gradientAlpha = 0;
    float gradientBeta = 0;
    
    // Calculate gradients using historical data
    for (int i = 1; i < HISTORY_SIZE; i++) {
        int prev = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
        int curr = (historyIndex - i + 1 + HISTORY_SIZE) % HISTORY_SIZE;
        
        float error = calculateError(&history[curr]);
        float prevError = calculateError(&history[prev]);
        float deltaError = error - prevError;
        
        gradientAlpha += deltaError * (history[curr].pwm1 - history[prev].pwm1);
        gradientBeta += deltaError * (history[curr].pwm2 - history[prev].pwm2);
    }
    
    // Update hidden parameters with momentum
    hiddenParams.alpha -= LEARNING_RATE * gradientAlpha + MOMENTUM * hiddenParams.alpha;
    hiddenParams.beta -= LEARNING_RATE * gradientBeta + MOMENTUM * hiddenParams.beta;
    
    // Ensure parameters stay within reasonable bounds
    hiddenParams.alpha = constrain(hiddenParams.alpha, 0.1, 1.0);
    hiddenParams.beta = constrain(hiddenParams.beta, 0.1, 1.0);
}

// Monte Carlo simulation for PWM adjustment
void monteCarloOptimization() {
    SystemState bestState = history[historyIndex];
    float bestError = calculateError(&bestState);
    
    for (int i = 0; i < MC_ITERATIONS; i++) {
        SystemState testState = bestState;
        
        // Generate random PWM adjustments
        testState.pwm1 += random(-PWM_STEP, PWM_STEP) * hiddenParams.alpha;
        testState.pwm2 += random(-PWM_STEP, PWM_STEP) * hiddenParams.beta;
        
        // Apply priority ratio
        testState.pwm1 = constrain(testState.pwm1, PWM_MIN, PWM_MAX);
        testState.pwm2 = constrain(testState.pwm2, PWM_MIN, 
                                 (int)(PWM_MAX * (1 - priorityRatio)));
        
        // Simulate new state
        testState.voltage = readVoltage(PANEL_VOLTAGE_PIN);
        testState.power = testState.voltage * panelCurrent * 
                         ((float)testState.pwm1/PWM_MAX + 
                          (float)testState.pwm2/PWM_MAX);
        
        float error = calculateError(&testState);
        if (error < bestError) {
            bestError = error;
            bestState = testState;
        }
    }
    
    // Apply best found state
    pwmConv1 = bestState.pwm1;
    pwmConv2 = bestState.pwm2;
}

// Enhanced MPPT adjustment with error-based control
void performEnhancedMpptAdjustment() {
    SystemState currentState;
    currentState.voltage = readVoltage(PANEL_VOLTAGE_PIN);
    currentState.current = panelCurrent;
    currentState.power = currentState.voltage * currentState.current * 
                        ((float)pwmConv1/PWM_MAX + (float)pwmConv2/PWM_MAX);
    currentState.pwm1 = pwmConv1;
    currentState.pwm2 = pwmConv2;
    
    // Update history
    history[historyIndex] = currentState;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    
    // Optimize hidden parameters periodically
    if (historyIndex == 0) {
        optimizeHiddenParams();
    }
    
    // Perform Monte Carlo optimization
    monteCarloOptimization();
    
    // Apply PWM values
    analogWrite(CONV1_PWM_PIN, pwmConv1);
    analogWrite(CONV2_PWM_PIN, pwmConv2);
    
    // Dynamic interval adjustment based on error
    float error = calculateError(&currentState);
    mpptInterval = constrain(
        (int)(100 * (1 + error * hiddenParams.alpha)),
        10, 200
    );
    
    // Debug output
    Serial_print_s("V:");
    Serial_print_f(currentState.voltage);
    Serial_print_s(",P:");
    Serial_print_f(currentState.power);
    Serial_print_s(",PWM1:");
    Serial_print_f(pwmConv1);
    Serial_print_s(",PWM2:");
    Serial_println_f(pwmConv2);
}

void setup() {
    pinMode(CONV1_PWM_PIN, OUTPUT);
    pinMode(CONV2_PWM_PIN, OUTPUT);
    Serial_begin(115200);
    
    // Initialize history buffer
    SystemState initialState = {0};
    for (int i = 0; i < HISTORY_SIZE; i++) {
        history[i] = initialState;
    }
    
    measureOpenCircuitVoltage();
}

void loop() {
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastVocCheck >= vocInterval) {
        measureOpenCircuitVoltage();
        lastVocCheck = currentMillis;
    }
    
    if (currentMillis - lastMpptCheck >= mpptInterval) {
        performEnhancedMpptAdjustment();
        lastMpptCheck = currentMillis;
    }
}
