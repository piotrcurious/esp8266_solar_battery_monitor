//#include <TFT_eSPI.h>
#include <SPI.h>
#include <IRremote.h>

// Display setup
//TFT_eSPI tft = TFT_eSPI();


#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
  #define TFT_CS        9
  #define TFT_RST        -1 // Or set to -1 and connect to Arduino RESET pin
  #define TFT_DC         10

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BLACK ST77XX_BLACK
#define TFT_WHITE ST77XX_WHITE
#define TFT_RED ST77XX_RED
#define TFT_GREEN ST77XX_GREEN
#define TFT_BLUE ST77XX_BLUE
#define TFT_YELLOW ST77XX_YELLOW
#define TFT_CYAN ST77XX_CYAN
#define TFT_MAGENTA ST77XX_MAGENTA
#define TFT_DARKGREY 0x4208
#define TFT_GREY ST77XX_GREY


// IR Remote setup
#define IR_RECEIVE_PIN 19


// IR Remote Key Definitions
namespace RemoteKeys {
  enum KeyCode {
    KEY_0 = 0x11,
    KEY_1 = 0x04,
    KEY_2 = 0x05,
    KEY_3 = 0x06,
    KEY_4 = 0x08,
    KEY_5 = 0x09,
    KEY_6 = 0x0A,
    KEY_7 = 0x0C,
    KEY_8 = 0x0D,
    KEY_9 = 0x0E,
    KEY_UP = 0x60,
    KEY_DOWN = 0x61,
    KEY_LEFT = 0x65,
    KEY_RIGHT = 0x62,
    KEY_OK = 0x68,
    KEY_MENU = 0x79,
    KEY_RED = 0x6c,
    KEY_GREEN = 0x14,
    KEY_YELLOW = 0x15,
    KEY_BLUE = 0x16,
    KEY_VOL_UP = 0x07,
    KEY_VOL_DOWN = 0x0b,
    KEY_CH_UP = 0x12,
    KEY_CH_DOWN = 0x10
  };
}


// ADC pins
#define VOLTAGE_IN_PIN 0
#define VOLTAGE_OUT_PIN 1
#define CURRENT_IN_PIN 2
#define CURRENT_OUT_PIN 3

const float VOLTAGE_IN_SCALE  = 3.0/4096;
const float VOLTAGE_OUT_SCALE = 3.0/4096;
const float CURRENT_IN_SCALE  = 3.3/4096;
const float CURRENT_OUT_SCALE = 3.3/4096;

// Graph parameters
#define GRAPH_WIDTH 160
#define GRAPH_HEIGHT 64
#define GRAPH_X 0
#define GRAPH_Y 16

// Buffer size
#define BUFFER_SIZE GRAPH_WIDTH

// Time bases in milliseconds
const unsigned long TIME_BASES[] = {100, 1000, 5000, 10000};  //0.1s,  1s, 5s, 10s per pixel
int currentTimeBase = 0;

// Graph modes
enum GraphMode {
    INPUT_VOLTAGE,
    OUTPUT_VOLTAGE,
    INPUT_CURRENT,
    OUTPUT_CURRENT,    
    INPUT_POWER,
    ENERGY,
    COMBINED
};

GraphMode currentMode = INPUT_VOLTAGE;

// Circular buffers
float voltageInBuffer[BUFFER_SIZE];
float voltageOutBuffer[BUFFER_SIZE];
float currentInBuffer[BUFFER_SIZE];
float currentOutBuffer[BUFFER_SIZE];
float powerBuffer[BUFFER_SIZE];
float energyBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Scaling factors
//const float VOLTAGE_SCALE = 3.3 / 4095.0;  // For voltage divider
//const float CURRENT_SCALE = 3.3 / 4095.0;   // For 0.1 ohm shunt

unsigned long lastUpdateTime = 0;
float accumulatedEnergy = 0;

void setup() {
    // Initialize display
//    tft.init();
    tft.initR(INITR_MINI160x80_PLUGIN);  // Init adafruit ST7735S mini display
    Serial.begin(115200);
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    
    // Initialize IR receiver
    IrReceiver.begin(IR_RECEIVE_PIN);
    
    // Initialize ADC
    analogReadResolution(12);

    pinMode(VOLTAGE_IN_PIN,INPUT);
    pinMode(VOLTAGE_OUT_PIN,INPUT);
    pinMode(CURRENT_IN_PIN,INPUT);
    pinMode(CURRENT_OUT_PIN,INPUT);
    
    // Clear buffers
    for(int i = 0; i < BUFFER_SIZE; i++) {
        voltageInBuffer[i] = 0;
        voltageOutBuffer[i] = 0;
        currentInBuffer[i] = 0;
        currentOutBuffer[i] = 0;
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
  /*
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
*/
      if (IrReceiver.decodedIRData.protocol == SAMSUNG) {
            if (IrReceiver.decodedIRData.address == 0x7) {
      //debug        
            Serial.print(F("Command 0x"));
            Serial.println(IrReceiver.decodedIRData.command, HEX);
        switch(IrReceiver.decodedIRData.command) {
        case RemoteKeys::KEY_1: currentMode = INPUT_VOLTAGE; break;
        case RemoteKeys::KEY_2: currentMode = OUTPUT_VOLTAGE; break;
        case RemoteKeys::KEY_3: currentMode = INPUT_CURRENT; break;
        case RemoteKeys::KEY_4: currentMode = OUTPUT_CURRENT; break;
        case RemoteKeys::KEY_5: currentMode = INPUT_POWER; break;
        case RemoteKeys::KEY_6: currentMode = ENERGY; break;
        case RemoteKeys::KEY_7: currentMode = COMBINED; break;
        // Time base selection
        case RemoteKeys::KEY_RED: currentTimeBase = 0; break;
        case RemoteKeys::KEY_GREEN: currentTimeBase = 1; break;
        case RemoteKeys::KEY_YELLOW: currentTimeBase = 2; break;
        case RemoteKeys::KEY_BLUE: currentTimeBase = 3; break;

        }
            }
      }
      
    clearAndRedrawGraph();
}

void updateMeasurements() {
    // Read sensors
    float voltageIn   = analogRead(VOLTAGE_IN_PIN) * VOLTAGE_IN_SCALE;
    float voltageOut  = analogRead(VOLTAGE_OUT_PIN) * VOLTAGE_OUT_SCALE;
    float currentIn   = analogRead(CURRENT_IN_PIN) * CURRENT_IN_SCALE;
    float currentOut  = analogRead(CURRENT_OUT_PIN) * CURRENT_OUT_SCALE;

    float power = voltageIn * currentIn;
    
    // Calculate energy (Wh)
    float energyIncrement = power * (TIME_BASES[currentTimeBase] / 3600000.0);  // Convert ms to hours
    accumulatedEnergy += energyIncrement;
    
    // Update buffers
    voltageInBuffer[bufferIndex] = voltageIn;
    voltageOutBuffer[bufferIndex] = voltageOut;
    currentInBuffer[bufferIndex] = currentIn;
    currentOutBuffer[bufferIndex] = currentOut;

    powerBuffer[bufferIndex] = power;
    energyBuffer[bufferIndex] = accumulatedEnergy;
    
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
}

void updateGraph() {
    tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);
    
    // Draw data based on current mode
    switch(currentMode) {
        case INPUT_VOLTAGE:
            drawGrid();
            drawTitle("Vin(V)");
            drawBuffer(voltageInBuffer, TFT_RED, 3.0);
            break;
        case OUTPUT_VOLTAGE:
            drawGrid();
            drawBuffer(voltageOutBuffer, TFT_GREEN, 3.0);
            drawTitle("Vout(V)");
            break;
        case INPUT_CURRENT:
            drawGrid();
            drawBuffer(currentInBuffer, TFT_BLUE, 2.0);
            drawTitle("Iin(A)");
            break;
        case OUTPUT_CURRENT:
            drawGrid();
            drawBuffer(currentOutBuffer, TFT_CYAN, 3.0);
            drawTitle("Iout(A)");
            break;
        case INPUT_POWER:
            drawGrid();
            drawBuffer(powerBuffer, TFT_YELLOW, 5.0);
            drawTitle("Pin(W)");
            break;
        case ENERGY:
            drawGrid();
            drawBuffer(energyBuffer, TFT_MAGENTA, 1.0);
            drawTitle("E(Wh)");
            break;
        case COMBINED:
            drawGrid();
            drawBuffer(voltageInBuffer, TFT_RED, 20.0);
            drawBuffer(currentInBuffer, TFT_BLUE, 2.0);
            drawBuffer(powerBuffer, TFT_YELLOW, 40.0);
            drawTitle("");
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
        y1 = (GRAPH_HEIGHT-1) - ((float)buffer[index]     * ((float)GRAPH_HEIGHT/scale));
        y2 = (GRAPH_HEIGHT-1) - ((float)buffer[nextIndex] * ((float)GRAPH_HEIGHT/scale));    
        if (y1<0) {y1=0;};
        if (y2<0) {y2=0;};
        
//        Serial.println(y1);
        tft.drawLine(x1, y1+GRAPH_Y, x2, y2+GRAPH_Y, color);
    }
}

void drawGrid() {
    // Draw horizontal grid lines
    for(int i = 0; i <= 4; i++) {
        int y = GRAPH_Y + (i * (GRAPH_HEIGHT / 4));
 //       Serial.println(y);
        tft.drawLine(GRAPH_X, y, GRAPH_X + GRAPH_WIDTH, y, TFT_DARKGREY);
    }
    
    // Draw vertical grid lines
    for(int i = 0; i <= 4; i++) {
        int x = GRAPH_X + (i * GRAPH_WIDTH / 4);
        tft.drawLine(x, GRAPH_Y, x, GRAPH_Y + GRAPH_HEIGHT, TFT_DARKGREY);
    }
}

void drawTitle(const char* title) {
    //tft.fillRect(0, 0, GRAPH_WIDTH, 20, TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(5, 5);
    tft.print(title);
    tft.print(" - ");
    tft.print(TIME_BASES[currentTimeBase] / 1000.0,1);
    tft.print("s");
}

void clearAndRedrawGraph() {
    tft.fillScreen(TFT_BLACK);
    updateGraph();
}
