#include <TFT_eSPI.h>
#include <SPI.h>
#include <IRremote.h>

// Display setup
TFT_eSPI tft = TFT_eSPI();

//uint32_t SCREEN_UPDATE_INTERVAL = 100 ;  // screen update interval, in ms. 
//                    // it is possible to synchronize it with display refresh clock by using FRNCTL registers - TODO. 

#define SAMPLE_UPDATE_INTERVAL 100 // interval for sampling and filtering                     

//#include <Adafruit_GFX.h>    // Core graphics library
//#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
//  #define TFT_CS        9
//  #define TFT_RST        -1 // Or set to -1 and connect to Arduino RESET pin
//  #define TFT_DC         10

//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

/*
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
*/

#define TFT_BLACK ST7735_BLACK
#define TFT_WHITE ST7735_WHITE
#define TFT_RED ST7735_RED
#define TFT_GREEN ST7735_GREEN
#define TFT_BLUE ST7735_BLUE
#define TFT_YELLOW ST7735_YELLOW
#define TFT_CYAN ST7735_CYAN
#define TFT_MAGENTA ST7735_MAGENTA
#define TFT_DARKGREY 0x4208
#define TFT_GREY ST7735_GREY



// IR Remote setup
#define IR_RECEIVE_PIN 19

// IR Remote Key Definitions
namespace RemoteKeys {
  enum KeyCode {
    KEY_POWER = 0x02,

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
    KEY_UP          = 0x60,
    KEY_DOWN        = 0x61,
    KEY_LEFT        = 0x65,
    KEY_RIGHT       = 0x62,
    KEY_OK          = 0x68,
    KEY_MENU        = 0x79,
 
    KEY_RED         = 0x6c,
    KEY_GREEN       = 0x14,
    KEY_YELLOW      = 0x15,
    KEY_BLUE        = 0x16,
 
    KEY_VOL_UP      = 0x07,
    KEY_VOL_DOWN    = 0x0b,
    KEY_CH_UP       = 0x12,
    KEY_CH_DOWN     = 0x10,
    
    KEY_REWIND      = 0x45,
    KEY_PLAY        = 0x47,
    KEY_PAUSE       = 0x4A,
    KEY_FORWARD     = 0x48,
    KEY_STOP        = 0x46,

    KEY_SETTINGS    = 0x1A,
    KEY_INFO        = 0x1F,
    KEY_SUBTITLES   = 0x25,
    KEY_MUTE        = 0x0F,
    KEY_NETFLIX     = 0xF3,
    KEY_PRIME_VIDEO = 0xF4

  };
}

// ADC pins
#define VOLTAGE_IN_PIN 0
#define VOLTAGE_OUT_PIN 1
#define CURRENT_IN_PIN 2
#define CURRENT_OUT_PIN 3

// global sensor readings and offsets
const float VOLTAGE_IN_SCALE  = 12;
const float VOLTAGE_OUT_SCALE = 12;

//const float CURRENT_IN_SCALE  = 18.5; //20A sensor
const float CURRENT_IN_SCALE  = 15.70;  //5A sensor

//float currentIn_zero = 1.650;
float currentIn_zero = 1.614;

//const float CURRENT_OUT_SCALE = 18.5;//20A sensor
const float CURRENT_OUT_SCALE = 16.50; //5A sensor

//float currentOut_zero = 1.650;
float currentOut_zero = 1.614;

float voltageIn   = 0;
float voltageOut  = 0;
float currentIn   = 0;
float currentOut  = 0;
float powerIn     = 0; 
float powerOut    = 0;

uint32_t lastMeasurementTime = 0;  // for calculating log deltas
 
// Graph parameters
#define GRAPH_WIDTH 160
#define GRAPH_HEIGHT 72
#define GRAPH_X 0
#define GRAPH_Y 8

// Buffer size
#define BUFFER_SIZE GRAPH_WIDTH

// Time bases in milliseconds
#define MAX_TIMEBASES 7
#define MAX_SCREEN_UPDATE_INTERVAL 4

//const unsigned long TIME_BASES[MAX_TIMEBASES] = {20, 100, 1000, 5000, 10000, 60000, 120000, 540000};  //20ms , 0.1s,  1s, 5s, 10s, 1min(2.5h per screen), 2min(5h per screen),9min(24h per screen) per pixel
//const unsigned long SCREEN_UPDATE_INTERVAL[MAX_SCREEN_UPDATE_INTERVAL] = {100, 200, 500, 1000};  // to reduce flicker, controlled by prime video button

const unsigned long TIME_BASES[MAX_TIMEBASES] = {100, 1000, 5000, 10000, 60000, 120000, 540000};  //20ms , 0.1s,  1s, 5s, 10s, 1min(2.5h per screen), 2min(5h per screen),9min(24h per screen) per pixel
const unsigned long SCREEN_UPDATE_INTERVAL[MAX_SCREEN_UPDATE_INTERVAL] = {100, 200, 500, 1000};  // to reduce flicker, controlled by prime video button

int currentTimeBase = 0;
int currentScreenUpdateInterval = 0;

// Graph modes
enum GraphMode {
    INPUT_VOLTAGE,
    OUTPUT_VOLTAGE,
    INPUT_CURRENT,
    OUTPUT_CURRENT,    
    INPUT_POWER,
    OUTPUT_POWER,
    EFFICIENCY,
    ENERGY,
    COMBINED
};

GraphMode currentMode = INPUT_VOLTAGE;

// Circular buffers

float voltageInBuffer[BUFFER_SIZE];
float voltageInBufferMin = 0;
float voltageInBufferMax = 0;
float voltageOutBuffer[BUFFER_SIZE];
float voltageOutBufferMin = 0;
float voltageOutBufferMax = 0;
float currentInBuffer[BUFFER_SIZE];
float currentInBufferMin = 0;
float currentInBufferMax = 0;
float currentOutBuffer[BUFFER_SIZE];
float currentOutBufferMin = 0;
float currentOutBufferMax = 0;
//float powerInBuffer[BUFFER_SIZE];
//float powerInBufferMin = 0;
//float powerInBufferMax = 0;
//float powerOutBuffer[BUFFER_SIZE];
//float powerOutBufferMin = 0;
//float powerOutBufferMax = 0;
float energyBuffer[BUFFER_SIZE];
float energyBufferMin = 0;
float energyBufferMax = 0;
uint32_t timestampsBuffer[BUFFER_SIZE];

int bufferIndex = 0;

// Scaling factors
//const float VOLTAGE_SCALE = 3.3 / 4095.0;  // For voltage divider
//const float CURRENT_SCALE = 3.3 / 4095.0;   // For 0.1 ohm shunt

unsigned long lastUpdateTime = 0;
unsigned long lastScreenUpdateTime = 0;
unsigned long lastSampleTime = 0;

double accumulatedEnergy = 0;
double lastAccumulatedEnergy = 0;
double lastAccumulatedEnergy1 = 0;
double lastAccumulatedEnergy2 = 0;
double lastAccumulatedEnergy3 = 0;


#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
using Eigen::MatrixXd;
using Eigen::VectorXd;


class AdvancedKalmanADC {
private:
    // Kalman filter parameters
    float Q = 0.00001;   // Process noise covariance
    float R = 0.0001;     // Measurement noise covariance
    float P = 1.0;     // Estimation error covariance
    float K = 0.0;     // Kalman gain
    float x = 0.0;     // State estimate

    // Subsampling parameters (related to the original RC time constant for example)
    float tau = 0.001; // Time constant for subsampling delay (seconds)

    // ADC parameters
    float adcVref = 3300.0; // ADC reference voltage in mV
    int adcResolution = 12; // ADC resolution in bits

public:
    AdvancedKalmanADC() {
        reset();
        // Initialize R based on ADC resolution (optional, can be fixed)
        R = pow(2, -adcResolution) * adcVref / 1000.0f;
    }

    // Configure subsampling time constant
    void setSubsamplingTimeConstant(float timeConstant_seconds) {
        tau = timeConstant_seconds;
    }

    // Configure ADC parameters (optional, if needed)
    void setADCParameters(float vref_mv, int resolution) {
        adcVref = vref_mv;
        adcResolution = resolution;
        R = pow(2, -adcResolution) * adcVref / 1000.0f; // Update R if resolution changes
    }


    // Read and filter ADC value with multiple averaging and subsampling
    float read(int pin, int samplesPerRead = 64) {
        double averageValue = 0.0;
//        samplesPerRead = constrain(samplesPerRead, 1, 64);

        unsigned long startTime = micros();
        for (int i = 0; i < samplesPerRead; i++) {
            float currentValue = analogReadMilliVolts(pin) / 1000.0f; // Convert to volts
            averageValue += currentValue;
        }
        averageValue /= samplesPerRead;
        
        unsigned long sampleTime = micros();
//        Serial.print("Sampled in:");
//        Serial.println(sampleTime-startTime);
        
        // Kalman Filter Update
        P = P + Q;
        K = P / (P + R);
        x = x + K * (averageValue - x);
        P = (1 - K) * P;

        return x;
    }

    // Set Kalman filter process noise covariance
    void setProcessNoise(float q) {
        Q = q;
    }

    // Set Kalman filter measurement noise covariance
    void setMeasurementNoise(float r) {
        R = r;
    }

    // Reset Kalman filter state
    void reset() {
        P = 1.0;
        x = 0.0;
    }

    // Get current state estimate
    float getState() {
        return x;
    }
};

AdvancedKalmanADC filter_currentIn;
AdvancedKalmanADC filter_currentOut;


void setup() {
    // Initialize display
    tft.init();
//    tft.initR(INITR_MINI160x80_PLUGIN);  // Init adafruit ST7735S mini display
//    tft.setSPISpeed(80000000); 
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
        energyBuffer[i] = 0;
        timestampsBuffer[i] = 0;
    }
    //filter_currentIn.setRCParameters(10000,10.0e-6); // 10k , 10uF
    //filter_currentIn.setADCParameters(3300,12); // 3.3V ref, 12bit    

    //filter_currentOut.setRCParameters(10000,10.0e-6); // 10k , 10uF
    //filter_currentOut.setADCParameters(3300,12); // 3.3V ref, 12bit    


//task setup
    // Start sample collection task
    xTaskCreatePinnedToCore(updateSamples_task, "sampling", 4096, NULL, 1, NULL, 0);

}

void updateSamples_task (void* parameter){
  while (1) {
    unsigned long currentTime = millis();
    if (currentTime - lastSampleTime >= SAMPLE_UPDATE_INTERVAL) {
        lastSampleTime = currentTime;
        updateSamples(); // read, filter and store momentary measurements TODO: should go on separate core
    }

    // Update measurements based on time base
    if (currentTime - lastUpdateTime >= TIME_BASES[currentTimeBase]) {
        lastUpdateTime = currentTime;
        updateMeasurements();
    }    
    vTaskDelay(SAMPLE_UPDATE_INTERVAL);// sample update interval delay for other tasks 
  }
}

void loop() {
    // Handle IR remote
    if (IrReceiver.decode()) {
        handleIRCommand();
        IrReceiver.resume();
    }
    unsigned long currentTime = millis();
//    if (currentTime - lastSampleTime >= SAMPLE_UPDATE_INTERVAL) {
//        lastSampleTime = currentTime;
//        updateSamples(); // read, filter and store momentary measurements TODO: should go on separate core
//    }

    // Update measurements based on time base
//    if (currentTime - lastUpdateTime >= TIME_BASES[currentTimeBase]) {
//        lastUpdateTime = currentTime;
//        updateMeasurements();
//        updateGraph();
//    }


    if (currentTime - lastScreenUpdateTime >= SCREEN_UPDATE_INTERVAL[currentScreenUpdateInterval]) {
        lastScreenUpdateTime = currentTime;
        updateGraph();
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
        case RemoteKeys::KEY_RED:        currentMode = INPUT_VOLTAGE; break;
        case RemoteKeys::KEY_GREEN:      currentMode = OUTPUT_VOLTAGE; break;
        case RemoteKeys::KEY_YELLOW:     currentMode = INPUT_CURRENT; break;
        case RemoteKeys::KEY_BLUE:       currentMode = OUTPUT_CURRENT; break;
        
        case RemoteKeys::KEY_SETTINGS:   currentMode = INPUT_POWER; break;
        case RemoteKeys::KEY_INFO:       currentMode = OUTPUT_POWER; break;
        case RemoteKeys::KEY_SUBTITLES:  currentMode = EFFICIENCY; break;
        
        case RemoteKeys::KEY_1:          currentMode = ENERGY; break;
        case RemoteKeys::KEY_2:          currentMode = COMBINED; break;
        // Time base selection
          
        case RemoteKeys::KEY_NETFLIX:    currentTimeBase = (currentTimeBase +1) % MAX_TIMEBASES; break;
        case RemoteKeys::KEY_PRIME_VIDEO:currentScreenUpdateInterval = (currentScreenUpdateInterval +1) % MAX_SCREEN_UPDATE_INTERVAL; break;
 
        case RemoteKeys::KEY_MUTE: {
          if (currentMode == INPUT_CURRENT) {
            currentIn_zero  = readCurrentIn();
          } else {
            currentOut_zero = readCurrentOut();    
          }
          break;
        }


        case RemoteKeys::KEY_POWER:{
           lastAccumulatedEnergy3 = lastAccumulatedEnergy2; // store last accumulated energy
           lastAccumulatedEnergy2 = lastAccumulatedEnergy1;
           lastAccumulatedEnergy1 = lastAccumulatedEnergy;
           lastAccumulatedEnergy = accumulatedEnergy;     
           accumulatedEnergy=0; 
           break;
          }
        
        }
            }
      }
      
    clearAndRedrawGraph();
}

float readCurrentIn(){
//    float current_sample   = (((float)analogReadMilliVolts(CURRENT_IN_PIN) * CURRENT_IN_SCALE)/1000.0);
//    float current_sample   = analogReadMilliVolts(CURRENT_IN_PIN);
      float current_sample     = (filter_currentIn.read(CURRENT_IN_PIN,128));

//    Serial.println(current_sample);

    return current_sample;
}

float readCurrentOut(){
//    float current_sample   = (((float)analogReadMilliVolts(CURRENT_OUT_PIN) * CURRENT_OUT_SCALE)/1000.0);
//    float current_sample   = analogReadMilliVolts(CURRENT_OUT_PIN);
     float current_sample     = (filter_currentOut.read(CURRENT_OUT_PIN,128));

    return current_sample;
}

#define VOLTAGE_AVERAGING 9

void updateSamples() {
    // Read sensors
    
//    voltageIn     = ((float)analogReadMilliVolts(VOLTAGE_IN_PIN) * VOLTAGE_IN_SCALE)/1000.0;
      float voltageIn_new     = ((float)analogReadMilliVolts(VOLTAGE_IN_PIN) * VOLTAGE_IN_SCALE)/1000.0;
      voltageIn = (voltageIn_new + ((VOLTAGE_AVERAGING-1)*voltageIn))/VOLTAGE_AVERAGING;

//      voltageOut    = ((float)analogReadMilliVolts(VOLTAGE_OUT_PIN) * VOLTAGE_OUT_SCALE)/1000.0;
      float voltageOut_new    = ((float)analogReadMilliVolts(VOLTAGE_OUT_PIN) * VOLTAGE_OUT_SCALE)/1000.0;
      voltageOut = (voltageOut_new + ((VOLTAGE_AVERAGING-1)*voltageOut))/VOLTAGE_AVERAGING;

//    currentIn     = ((((float)analogReadMilliVolts(CURRENT_IN_PIN) -currentIn_zero))/1000.0)*CURRENT_IN_SCALE;
      currentIn     = ((filter_currentIn.read(CURRENT_IN_PIN,128))-currentIn_zero)*CURRENT_IN_SCALE;
//    currentIn     = analogReadMilliVolts(CURRENT_IN_PIN);

//    currentOut    = ((((float)analogReadMilliVolts(CURRENT_OUT_PIN) -currentOut_zero))/1000.0)*CURRENT_OUT_SCALE;
      currentOut     = ((filter_currentOut.read(CURRENT_OUT_PIN,128))-currentIn_zero)*CURRENT_OUT_SCALE;

    powerIn       = voltageIn * currentIn;
    powerOut      = voltageOut * currentOut;
}

void updateMeasurements() {

 /* //no need - done in sampling task 
    // Read sensors    
    voltageIn     = ((float)analogReadMilliVolts(VOLTAGE_IN_PIN) * VOLTAGE_IN_SCALE)/1000.0;
    voltageOut    = ((float)analogReadMilliVolts(VOLTAGE_OUT_PIN) * VOLTAGE_OUT_SCALE)/1000.0;
    currentIn     = ((((float)analogReadMilliVolts(CURRENT_IN_PIN) -currentIn_zero))/1000.0)*CURRENT_IN_SCALE;
    currentOut    = ((((float)analogReadMilliVolts(CURRENT_OUT_PIN) -currentOut_zero))/1000.0)*CURRENT_OUT_SCALE;

//    currentOut  = (((float)analogReadMilliVolts(CURRENT_OUT_PIN) * CURRENT_OUT_SCALE)/1000.0)-currentOut_zero;

    double power = voltageIn * currentIn;

*/
    timestampsBuffer[bufferIndex]    = lastMeasurementTime-millis();
    lastMeasurementTime             = millis(); // log timestamp
    
    // Calculate energy (Wh)
    double energyIncrement          = powerIn * (TIME_BASES[currentTimeBase] / 3600000.0);  // Convert ms to hours
    accumulatedEnergy               = accumulatedEnergy + energyIncrement;
    
    // Update buffers
    voltageInBuffer[bufferIndex]    = voltageIn;
    voltageOutBuffer[bufferIndex]   = voltageOut;
    currentInBuffer[bufferIndex]    = currentIn;
    currentOutBuffer[bufferIndex]   = currentOut;
    energyBuffer[bufferIndex]       = accumulatedEnergy;
    
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
}

void updateGraph() {
//    tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);// clear graph area
//    tft.fillRect(0, 0, GRAPH_WIDTH, 8, TFT_BLACK); // clear top label area    
//    tft.fillRect(32, 0, 8*7, 8, TFT_BLACK); // clear top label area (only value)   
//    tft.fillRect(0, GRAPH_Y+GRAPH_HEIGHT-8, GRAPH_WIDTH, 8, TFT_BLACK); // clear bottom label area    

    // Draw data based on current mode
 #define SINGLE_VALUE_X 9*6 

 // ----------------------basic channel values
    switch(currentMode) {
        case INPUT_VOLTAGE:
            tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);// clear graph area
            tft.fillRect(32, 0, 8*8, 8, TFT_BLACK); // clear top label area (only value)   
            drawGrid();
            drawTitle("Vin(V)");
//            drawValue (voltageInBuffer[bufferIndex-1],TFT_RED,SINGLE_VALUE_X);
            drawValue (voltageIn,TFT_RED,SINGLE_VALUE_X,3);

   //         drawBuffer_old(voltageInBuffer, TFT_BLUE, 3.0);
            drawBuffer(voltageInBuffer, TFT_RED, 30.0);
            break;
        case OUTPUT_VOLTAGE:
            tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);// clear graph area
            tft.fillRect(32, 0, 8*8, 8, TFT_BLACK); // clear top label area (only value)   
            drawGrid();
            drawTitle("Vout(V)");
//            drawValue (voltageOutBuffer[bufferIndex-1],TFT_GREEN,SINGLE_VALUE_X);
            drawValue (voltageOut,TFT_GREEN,SINGLE_VALUE_X,3);

//            drawBuffer_old(voltageOutBuffer, TFT_BLACK, 3.0);
            drawBuffer(voltageOutBuffer, TFT_GREEN, 30.0);
            break;
        case INPUT_CURRENT:
            tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);// clear graph area
            tft.fillRect(32, 0, 8*8, 8, TFT_BLACK); // clear top label area (only value)   
            drawGrid();
            drawTitle("Iin(A)");
//            drawValue (currentInBuffer[bufferIndex-1],TFT_YELLOW,SINGLE_VALUE_X);
            drawValue (currentIn,TFT_YELLOW,SINGLE_VALUE_X,3);

            //drawBuffer_old(currentInBuffer, TFT_BLACK, 3.0);
            drawBuffer(currentInBuffer, TFT_YELLOW, 10.0);
            break;
        case OUTPUT_CURRENT:
            tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);// clear graph area
            tft.fillRect(32, 0, 8*8, 8, TFT_BLACK); // clear top label area (only value)   
            drawGrid();
            drawTitle("Iout(A)");
//            drawValue (currentOutBuffer[bufferIndex-1],TFT_BLUE,SINGLE_VALUE_X);
            drawValue (currentOut,TFT_BLUE,SINGLE_VALUE_X,3);
//            drawBuffer_old(currentOutBuffer, TFT_BLACK, 3.0);
            drawBuffer(currentOutBuffer, TFT_BLUE, 10.0);
            break;
    // ------------------------------combined power graphs
        case INPUT_POWER:
            tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);// clear graph area
            tft.fillRect(32, 0, 8*8, 8, TFT_BLACK); // clear top label area (only value)   
            drawGrid();
            drawTitle("Pin(W)");
//            drawMulValue (voltageInBuffer[bufferIndex-1],currentInBuffer[bufferIndex-1],TFT_YELLOW,SINGLE_VALUE_X);
            drawValue (powerIn,TFT_WHITE,SINGLE_VALUE_X,3);
            drawMulBuffer(voltageInBuffer,currentInBuffer, TFT_WHITE, 100.0);
            break;
        case OUTPUT_POWER:
            tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);// clear graph area
            tft.fillRect(32, 0, 8*8, 8, TFT_BLACK); // clear top label area (only value)   
            drawGrid();
            drawTitle("Pout(W)");
//            drawMulValue (voltageOutBuffer[bufferIndex-1],currentOutBuffer[bufferIndex-1],0xeca5,SINGLE_VALUE_X);
            drawValue (powerOut,0xeca5,SINGLE_VALUE_X,3);
            drawMulBuffer(voltageOutBuffer,currentOutBuffer, 0xeca5, 100.0);
            break;
       case EFFICIENCY:
            tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);// clear graph area
            tft.fillRect(32, 0, 8*8, 8, TFT_BLACK); // clear top label area (only value)   
            drawGrid();
            drawTitle("Eff(%)");
            drawEffValue (voltageInBuffer[bufferIndex-1],currentInBuffer[bufferIndex-1],voltageOutBuffer[bufferIndex-1],currentOutBuffer[bufferIndex-1],0xeca5,SINGLE_VALUE_X);
            drawEffBuffer(voltageInBuffer,currentInBuffer,voltageOutBuffer,currentOutBuffer, 0xeca5, 1.0);
            break;
            
        case ENERGY:
            tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);// clear graph area
            tft.fillRect(8*6, 0, 8*5, 8, TFT_BLACK); // clear top label area (only value)   
            tft.fillRect(8*6+8*5, 0, 8*5, 8, TFT_MAGENTA); // clear top label area (only value)   

            drawGrid();
            drawTitle("E(Wh)");
            drawValue (energyBuffer[bufferIndex-1],TFT_MAGENTA,8*6,3);
            drawValue (lastAccumulatedEnergy,TFT_WHITE,8*6+8*5,3);
            drawBuffer(energyBuffer, TFT_MAGENTA, 200.0);
            break;
        case COMBINED:
            tft.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);// clear graph area
            tft.fillRect(0, 0, 159, 8, TFT_BLACK); // clear top label area    

            drawGrid();
            drawValue (voltageIn,TFT_RED,0,2);
            drawValue (voltageOut,TFT_GREEN,5*8,2);
            drawValue (currentIn,TFT_YELLOW,5*8+8+5*8,2);
            drawValue (powerIn,TFT_WHITE,5*8+5*8+8+5*8,2);

            drawBuffer(voltageInBuffer, TFT_RED, 30.0);
            drawBuffer(voltageOutBuffer, TFT_GREEN, 30.0);
            drawBuffer(currentInBuffer, TFT_YELLOW, 10.0);
//            drawMulValue (voltageInBuffer[bufferIndex-1],currentInBuffer[bufferIndex-1],TFT_WHITE,SINGLE_VALUE_X);
            drawMulBuffer(voltageInBuffer,currentInBuffer, TFT_WHITE, 100.0);
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
        if (y1>GRAPH_HEIGHT) {y1=GRAPH_HEIGHT;};
        if (y2>GRAPH_HEIGHT) {y2=GRAPH_HEIGHT;};
//        Serial.println(y1);
        tft.drawLine(x1, y1+GRAPH_Y, x2, y2+GRAPH_Y, color);
    }
}

void drawBuffer_old(float buffer[], uint16_t color, float scale) {
    int x1, y1, x2, y2;
    
    for(int i = 0; i < BUFFER_SIZE - 1; i++) {
        int index = (bufferIndex + i-1) % BUFFER_SIZE;
        int nextIndex = (bufferIndex + i + 1-1) % BUFFER_SIZE;
        
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

void drawMulBuffer(float buffer1[],float buffer2[], uint16_t color, float scale) {
    int x1, y1, x2, y2;
    
    for(int i = 0; i < BUFFER_SIZE - 1; i++) {
        int index = (bufferIndex + i) % BUFFER_SIZE;
        int nextIndex = (bufferIndex + i + 1) % BUFFER_SIZE;
        
        x1 = GRAPH_X + i;
        x2 = GRAPH_X + i + 1;
        y1 = (GRAPH_HEIGHT-1) - ((buffer1[index]*buffer2[index])          * ((float)GRAPH_HEIGHT/scale));
        y2 = (GRAPH_HEIGHT-1) - ((buffer1[nextIndex]*buffer2[nextIndex])  * ((float)GRAPH_HEIGHT/scale));    
        if (y1<0) {y1=0;};
        if (y2<0) {y2=0;};
        
//        Serial.println(y1);
        tft.drawLine(x1, y1+GRAPH_Y, x2, y2+GRAPH_Y, color);
    }
}
void drawEffBuffer(float buffer1[],float buffer2[], float buffer3[], float buffer4[], uint16_t color, float scale) {
    int x1, y1, x2, y2;
    
    for(int i = 0; i < BUFFER_SIZE - 1; i++) {
        int index = (bufferIndex + i) % BUFFER_SIZE;
        int nextIndex = (bufferIndex + i + 1) % BUFFER_SIZE;
        
        x1 = GRAPH_X + i;
        x2 = GRAPH_X + i + 1;
        y1 = (GRAPH_HEIGHT-1) - (((buffer3[index]*buffer4[index])/( (buffer1[index]*buffer2[index])+0.0001) )                  * ((float)GRAPH_HEIGHT/scale));
        y2 = (GRAPH_HEIGHT-1) - (((buffer3[nextIndex]*buffer4[nextIndex])/( (buffer1[nextIndex]*buffer2[nextIndex])+0.0001 ))  * ((float)GRAPH_HEIGHT/scale));    
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
//        tft.drawLine(GRAPH_X, y-1, GRAPH_X + GRAPH_WIDTH, y-1, TFT_DARKGREY);
        tft.drawFastHLine(GRAPH_X, y-1, GRAPH_X + GRAPH_WIDTH, TFT_DARKGREY);

    }
    
    // Draw vertical grid lines
    for(int i = 0; i <= 6; i++) {
        int x = GRAPH_X + (i * ((float)(GRAPH_WIDTH -1)/ 6.0));
//        tft.drawLine(x, GRAPH_Y, x, GRAPH_Y + GRAPH_HEIGHT, TFT_DARKGREY);
        tft.drawFastVLine(x, GRAPH_Y, GRAPH_Y + GRAPH_HEIGHT, TFT_DARKGREY);

    }
}

void drawValue (float value,uint16_t color, uint16_t x, uint8_t decimal_points ) {
//void drawValue (float value,uint16_t color,uint16_t x) {

    tft.setTextColor(color);
    tft.setTextSize(1);
    tft.setCursor(x, 0);
    tft.print(value,decimal_points);  
}
void drawMulValue (float value1, float value2, uint16_t color,uint16_t x) {
    tft.setTextColor(color);
    tft.setTextSize(1);
    tft.setCursor(x, 0);
    tft.print(value1*value2,3);  
}
void drawEffValue (float value1, float value2, float value3, float value4, uint16_t color,uint16_t x) {
    tft.setTextColor(color);
    tft.setTextSize(1);
    tft.setCursor(x, 0);
    tft.print(((value3*value4)/(value1*value2))*100,1);  
}

void drawTitle(const char* title) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(0, 0);
    tft.print(title);
    tft.setCursor(0,GRAPH_Y+GRAPH_HEIGHT-8);
    if (TIME_BASES[currentTimeBase] >= 100) {
    tft.print(TIME_BASES[currentTimeBase] / 1000.0,1);
    tft.print("s");
    }else {
    tft.print(TIME_BASES[currentTimeBase]);
    tft.print("ms");      
    }
    
}

void clearAndRedrawGraph() {
    tft.fillScreen(TFT_BLACK);
    updateGraph();
}
