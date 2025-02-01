#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <IRremote.h>

// Display setup
#define TFT_CS 9
#define TFT_RST -1
#define TFT_DC 10
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

#define TFT_BLACK ST77XX_BLACK
#define TFT_WHITE ST77XX_WHITE
#define TFT_RED ST77XX_RED
#define TFT_GREEN ST77XX_GREEN

// IR Remote setup
#define IR_RECEIVE_PIN 19

// ADC pins
#define VOLTAGE_IN_PIN 0

// Scaling factors
const float VOLTAGE_IN_SCALE = 3.0 / 4096;

// Graph parameters
const int BUFFER_SIZE = 256; // Circular buffer size (independent of graph width)
float dataBuffer[BUFFER_SIZE];
unsigned long timeBuffer[BUFFER_SIZE]; // Timestamps for each data point
int bufferIndex = 0;

// Graph configuration
struct GraphConfig {
    int x;
    int y;
    int width;
    int height;
    float yMin;
    float yMax;
    bool autoScale;
};

GraphConfig graphConfig = {0, 16, 160, 64, 0.0, 3.0, true}; // Default config

unsigned long lastUpdateTime = 0;
unsigned long timeWindow = 5000; // Time window in milliseconds

void setup() {
    tft.initR(INITR_MINI160x80_PLUGIN);
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    Serial.begin(115200);

    IrReceiver.begin(IR_RECEIVE_PIN);
    analogReadResolution(12);

    // Initialize buffers
    for (int i = 0; i < BUFFER_SIZE; i++) {
        dataBuffer[i] = 0;
        timeBuffer[i] = 0;
    }
}

void loop() {
    unsigned long currentTime = millis();

    // Update data at regular intervals
    if (currentTime - lastUpdateTime >= 100) {
        updateMeasurements(currentTime);
        drawGraph(graphConfig);
        lastUpdateTime = currentTime;
    }

    // Handle IR commands (e.g., adjust scaling or graph settings)
    if (IrReceiver.decode()) {
        handleIRCommand();
        IrReceiver.resume();
    }
}

void updateMeasurements(unsigned long currentTime) {
    float voltageIn = analogRead(VOLTAGE_IN_PIN) * VOLTAGE_IN_SCALE;

    // Update buffers with new data and timestamp
    dataBuffer[bufferIndex] = voltageIn;
    timeBuffer[bufferIndex] = currentTime;

    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
}

void drawGraph(GraphConfig config) {
    tft.fillRect(config.x, config.y, config.width, config.height, TFT_BLACK);

    // Determine scaling dynamically if auto-scale is enabled
    float yMin = config.autoScale ? findMinInWindow() : config.yMin;
    float yMax = config.autoScale ? findMaxInWindow() : config.yMax;

    // Draw grid and labels
    drawGrid(config, yMin, yMax);

    // Plot data within the time window
    unsigned long now = millis();
    for (int i = 0; i < BUFFER_SIZE - 1; i++) {
        int index1 = (bufferIndex + i) % BUFFER_SIZE;
        int index2 = (bufferIndex + i + 1) % BUFFER_SIZE;

        if (timeBuffer[index1] < now - timeWindow || timeBuffer[index2] < now - timeWindow) continue;

        int x1 = map(timeBuffer[index1], now - timeWindow, now, config.x, config.x + config.width);
        int x2 = map(timeBuffer[index2], now - timeWindow, now, config.x, config.x + config.width);

        int y1 = map(dataBuffer[index1], yMin, yMax, config.y + config.height, config.y);
        int y2 = map(dataBuffer[index2], yMin, yMax, config.y + config.height, config.y);

        tft.drawLine(x1, y1, x2, y2, TFT_RED);
    }
}

void drawGrid(GraphConfig config, float yMin, float yMax) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);

    // Draw horizontal grid lines and labels
    for (int i = 0; i <= 4; i++) {
        int y = config.y + (i * (config.height / 4));
        tft.drawLine(config.x, y, config.x + config.width, y, TFT_WHITE);

        float labelValue = yMax - i * ((yMax - yMin) / 4);
        tft.setCursor(config.x + config.width + 2, y - 3);
        tft.print(labelValue, 1);
    }

    // Draw vertical grid lines
    for (int i = 0; i <= 4; i++) {
        int x = config.x + (i * (config.width / 4));
        tft.drawLine(x, config.y, x, config.y + config.height, TFT_WHITE);
    }
}

float findMinInWindow() {
    unsigned long now = millis();
    float minVal = FLT_MAX;

    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (timeBuffer[i] >= now - timeWindow) {
            minVal = min(minVal, dataBuffer[i]);
        }
    }
    
    return minVal == FLT_MAX ? graphConfig.yMin : minVal;
}

float findMaxInWindow() {
    unsigned long now = millis();
    float maxVal = FLT_MIN;

    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (timeBuffer[i] >= now - timeWindow) {
            maxVal = max(maxVal, dataBuffer[i]);
        }
    }

    return maxVal == FLT_MIN ? graphConfig.yMax : maxVal;
}

void handleIRCommand() {
   // Implement IR commands to adjust scaling or graph settings here.
}
