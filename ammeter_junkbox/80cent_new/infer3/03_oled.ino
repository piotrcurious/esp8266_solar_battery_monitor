#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MovingAverage.h>

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

class OLEDDisplay {
private:
    Adafruit_SSD1306 display;

public:
    OLEDDisplay() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

    void initialize() {
        // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
        if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            for(;;); // Don't proceed, loop forever
        }
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        splashScreen();
    }

    void splashScreen() {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("Solar MPPT System"));
        display.println(F("Initializing..."));
        display.display();
        delay(1000);
    }

    void updateDisplay(float voltage, float current, int duty1, int duty2, float temperature) {
        display.clearDisplay();
        display.setCursor(0,0);

        // First line: Voltage and Current
        display.print(F("V:")); 
        display.print(voltage, 2);
        display.print(F("V "));
        display.print(F("I:")); 
        display.print(current, 2);
        display.println(F("A"));

        // Second line: Duty Cycles
        display.print(F("D1:")); 
        display.print(duty1);
        display.print(F(" D2:")); 
        display.print(duty2);

        // Third line: Temperature
        display.print(F(" T:")); 
        display.print(temperature, 1);
        display.print(F("C"));

        display.display();
    }

    void displayError(const String& errorMsg) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("ERROR:"));
        display.println(errorMsg);
        display.display();
    }
};

// Configuration and Controller classes remain the same as in the previous implementation
// ... [Previous SolarSystemConfig and SolarMPPTController classes] ...

// Modify the SolarMPPTController to include OLED display
class SolarMPPTController {
private:
    // ... [Previous private members] ...
    OLEDDisplay oledDisplay;

public:
    // Constructor and other methods remain the same
    // ...

    void initialize() {
        pinMode(pwmPin1, OUTPUT);
        pinMode(pwmPin2, OUTPUT);
        Serial.begin(115200);
        
        // Initialize I2C and OLED
        Wire.begin();
        oledDisplay.initialize();
    }

    void update() {
        float panelVoltage = readVoltage(panelVoltagePin);
        float filteredVoltage = voltageFilter.add(panelVoltage);
        float temperature = readTemperature();

        updateVocTracking(filteredVoltage);
        updateWeightingFactors();

        float inferredCurrent = estimateAdvancedCurrent(filteredVoltage, dutyCycle1);
        float voltageError = targetPanelVoltage - filteredVoltage;

        performAdvancedPWMControl(voltageError);
        intelligentLoadBalancing(inferredCurrent);

        analogWrite(pwmPin1, dutyCycle1);
        analogWrite(pwmPin2, dutyCycle2);

        // Update OLED Display
        oledDisplay.updateDisplay(
            filteredVoltage, 
            inferredCurrent, 
            dutyCycle1, 
            dutyCycle2, 
            temperature
        );

        // Enhanced logging
        Serial.print("Voltage: "); Serial.print(filteredVoltage);
        Serial.print("V, Current Est: "); Serial.print(inferredCurrent);
        Serial.print("A, Duty1: "); Serial.print(dutyCycle1);
        Serial.print(", Duty2: "); Serial.println(dutyCycle2);
    }
};

// Create global instances
SolarMPPTController solarController(9, 10, A0, A1, A2, A3);

void setup() {
    solarController.initialize();
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastUpdate >= SolarSystemConfig::UPDATE_INTERVAL) {
        lastUpdate = currentTime;
        solarController.update();
    }
}
