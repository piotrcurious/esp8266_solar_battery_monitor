#include <TFT_eSPI.h>
#include <SPI.h>
#include <IRremote.h>

// Display setup
TFT_eSPI tft = TFT_eSPI();

// IR Remote setup
#define IR_RECEIVE_PIN 15
#define IR_BUTTON_1 0xFF30CF  // Example IR codes - adjust for your remote
#define IR_BUTTON_2 0xFF18E7
#define IR_BUTTON_3 0xFF7A85
#define IR_BUTTON_4 0xFF10EF
#define IR_BUTTON_5 0xFF38C7
#define IR_BUTTON_6 0xFF5AA5
#define IR_TIME_1  0xFF42BD
#define IR_TIME_2  0xFF4AB5
#define IR_TIME_3  0xFF52AD

// ADC pins
#define VOLTAGE_IN_PIN 36
#define VOLTAGE_OUT_PIN 39
#define CURRENT_IN_PIN 34

// Graph parameters
#define GRAPH_WIDTH 160
#define GRAPH_HEIGHT 60
#define GRAPH_X 0
#define GRAPH_Y 20

// Buffer size
#define BUFFER_SIZE GRAPH_WIDTH

// Time bases in milliseconds
const unsigned long TIME_BASES[] = {1000, 5000, 10000};  // 1s, 5s, 10s per pixel
int currentTimeBase = 0;

// Graph modes
enum GraphMode {
    INPUT_VOLTAGE,
    OUTPUT_VOLTAGE,
    INPUT_CURRENT,
    INPUT_POWER,
    ENERGY,
    COMBINED
};

GraphMode currentMode = INPUT_VOLTAGE;

// Circular buffers
float voltageInBuffer[BUFFER_SIZE];
float voltageOutBuffer[BUFFER_SIZE];
float currentBuffer[BUFFER_SIZE];
float powerBuffer[BUFFER_SIZE];
float energyBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Scaling factors
const float VOLTAGE_SCALE = 3.3 * 11.0 / 4095.0;  // For voltage divider
const float CURRENT_SCALE = 3.3 / 4095.0 / 0.1;   // For 0.1 ohm shunt

unsigned long lastUpdateTime = 0;
float accumulatedEnergy = 0;

void setup() {
    // Initialize display
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    
    // Initialize IR receiver
    IrReceiver.begin(IR_RECEIVE_PIN);
    
    // Initialize ADC
    analogReadResolution(12);
    
    // Clear buffers
    for(int i = 0; i < BUFFER_SIZE; i++) {
        voltageInBuffer[i] = 0;
        voltageOutBuffer[i] = 0;
        currentBuffer[i] = 0;
        powerBuffer[i] = 0;
        energyBuffer[i] = 0;
    }
}

void loop() {
    // Handle IR remote
    if (IrReceiver.decode()) {
        handleIRCommand();
        IrReceiver.resume();
    }
    
    // Update measurements based on time base
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= TIME_BASES[currentTimeBase]) {
        updateMeasurements();
        updateGraph();
        lastUpdateTime = currentTime;
    }
}

void handleIRCommand() {
    uint32_t command = IrReceiver.decodedIRData.command;
    
    // Graph mode selection
    switch(command) {
        case IR_BUTTON_1: currentMode = INPUT_VOLTAGE; break;
        case IR_BUTTON_2: currentMode = OUTPUT_VOLTAGE; break;
        case IR_BUTTON_3: currentMode = INPUT_CURRENT; break;
        case IR_BUTTON_4: currentMode = INPUT_POWER; break;
        case IR_BUTTON_5: currentMode = ENERGY; break;
        case IR_BUTTON_6: currentMode = COMBINED; break;
        // Time base selection
        case IR_TIME_1: currentTimeBase = 0; break;
        case IR_TIME_2: currentTimeBase = 1; break;
        case IR_TIME_3: currentTimeBase = 2; break;
    }
    
    clearAndRedrawGraph();
}

void updateMeasurements() {
    // Read sensors
    float voltageIn = analogRead(VOLTAGE_IN_PIN) * VOLTAGE_SCALE;
    float voltageOut = analogRead(VOLTAGE_OUT_PIN) * VOLTAGE_SCALE;
    float current = analogRead(CURRENT_IN_PIN) * CURRENT_SCALE;
    float power = voltageIn * current;
    
    // Calculate energy (Wh)
    float energyIncrement = power * (TIME_BASES[currentTimeBase] / 3600000.0);  // Convert ms to hours
    accumulatedEnergy += energyIncrement;
    
    // Update buffers
    voltageInBuffer[bufferIndex] = voltageIn;
    voltageOutBuffer[bufferIndex] = voltageOut;
    currentBuffer[bufferIndex] = current;
    powerBuffer[bufferIndex] = power;
    energyBuffer[bufferIndex] = accumulatedEnergy;
    
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
}

void updateGraph() {
    tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);
    
    // Draw grid
    drawGrid();
    
    // Draw data based on current mode
    switch(currentMode) {
        case INPUT_VOLTAGE:
            drawBuffer(voltageInBuffer, TFT_RED, 20.0);
            drawTitle("Input Voltage (V)");
            break;
        case OUTPUT_VOLTAGE:
            drawBuffer(voltageOutBuffer, TFT_GREEN, 20.0);
            drawTitle("Output Voltage (V)");
            break;
        case INPUT_CURRENT:
            drawBuffer(currentBuffer, TFT_BLUE, 2.0);
            drawTitle("Input Current (A)");
            break;
        case INPUT_POWER:
            drawBuffer(powerBuffer, TFT_YELLOW, 40.0);
            drawTitle("Input Power (W)");
            break;
        case ENERGY:
            drawBuffer(energyBuffer, TFT_MAGENTA, 100.0);
            drawTitle("Energy (Wh)");
            break;
        case COMBINED:
            drawBuffer(voltageInBuffer, TFT_RED, 20.0);
            drawBuffer(currentBuffer, TFT_BLUE, 2.0);
            drawBuffer(powerBuffer, TFT_YELLOW, 40.0);
            drawTitle("Combined");
            break;
    }
}

void drawBuffer(float buffer[], uint16_t color, float scale) {
    int x1, y1, x2, y2;
    
    for(int i = 0; i < BUFFER_SIZE - 1; i++) {
        int index = (bufferIndex + i) % BUFFER_SIZE;
        int nextIndex = (bufferIndex + i + 1) % BUFFER_SIZE;
        
        x1 = GRAPH_X + i;
        x2 = GRAPH_X + i + 1;
        y1 = GRAPH_Y + GRAPH_HEIGHT - (buffer[index] / scale * GRAPH_HEIGHT);
        y2 = GRAPH_Y + GRAPH_HEIGHT - (buffer[nextIndex] / scale * GRAPH_HEIGHT);
        
        tft.drawLine(x1, y1, x2, y2, color);
    }
}

void drawGrid() {
    // Draw horizontal grid lines
    for(int i = 0; i <= 4; i++) {
        int y = GRAPH_Y + (i * GRAPH_HEIGHT / 4);
        tft.drawLine(GRAPH_X, y, GRAPH_X + GRAPH_WIDTH, y, TFT_DARKGREY);
    }
    
    // Draw vertical grid lines
    for(int i = 0; i <= 4; i++) {
        int x = GRAPH_X + (i * GRAPH_WIDTH / 4);
        tft.drawLine(x, GRAPH_Y, x, GRAPH_Y + GRAPH_HEIGHT, TFT_DARKGREY);
    }
}

void drawTitle(const char* title) {
    tft.fillRect(0, 0, GRAPH_WIDTH, 20, TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(5, 5);
    tft.print(title);
    tft.print(" - ");
    tft.print(TIME_BASES[currentTimeBase] / 1000.0);
    tft.print("s/div");
}

void clearAndRedrawGraph() {
    tft.fillScreen(TFT_BLACK);
    updateGraph();
}
