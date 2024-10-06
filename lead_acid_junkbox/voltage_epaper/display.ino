#include <GxEPD2_BW.h>       // ePaper library
#include <Fonts/FreeMonoBold9pt7b.h>  // Font library
#include <ESP8266WiFi.h>      // Required for ESP deep sleep functionality

ADC_MODE(ADC_VCC);           // Set ADC to monitor the VCC

// Define ePaper display driver and dimensions
GxEPD2_BW<GxEPD2_213_B73, GxEPD2_213_B73::HEIGHT> display(GxEPD2_213_B73(/*CS=*/15, /*DC=*/4, /*RST=*/5, /*BUSY=*/16));

const int gpioPin = 4;       // GPIO4 to control
const int adcPin = A0;       // Pin to measure voltage
const float voltageDividerRatio = 4.0;  // Adjust depending on your voltage divider
const float highThreshold = 14.0;       // Voltage threshold to stay awake
const float lowThreshold = 13.8;        // Voltage threshold to go back to deep sleep

void setup() {
  pinMode(gpioPin, OUTPUT);
  digitalWrite(gpioPin, LOW); // Start with GPIO4 off
  Serial.begin(115200);
  
  // Initialize the ePaper display
  display.init(115200);       // Initialize display with SPI speed
  display.setRotation(1);     // Rotate the display (if necessary)
  display.setFont(&FreeMonoBold9pt7b);  // Set the font
  display.setTextColor(GxEPD_BLACK);    // Set text color
  display.firstPage();                  // Start first page (required for GxEPD2)
  
  // Display a welcome message
  displayWelcomeMessage();
}

void loop() {
  float batteryVoltage = readBatteryVoltage();

  Serial.print("Battery Voltage: ");
  Serial.println(batteryVoltage);

  // Update the ePaper display with the current voltage
  displayVoltage(batteryVoltage);

  if (batteryVoltage > highThreshold) {
    Serial.println("Voltage is above 14V, staying awake...");
    digitalWrite(gpioPin, HIGH);  // Turn on GPIO4
    
    // Monitor the voltage every 1 second
    while (batteryVoltage > lowThreshold) {
      delay(1000);  // Wait for 1 second
      batteryVoltage = readBatteryVoltage();
      Serial.print("Monitoring Voltage: ");
      Serial.println(batteryVoltage);
      
      // Update the ePaper display every second
      displayVoltage(batteryVoltage);
    }

    // Once voltage falls below 13.8V, turn off GPIO4 and go back to sleep
    Serial.println("Voltage dropped below 13.8V, turning off GPIO4 and going to sleep...");
    digitalWrite(gpioPin, LOW);
    goToDeepSleep();
  } else {
    Serial.println("Voltage is below 14V, going to deep sleep...");
    goToDeepSleep();
  }
}

float readBatteryVoltage() {
  int analogValue = analogRead(adcPin);  // Read from A0 pin
  float voltage = (analogValue / 1024.0) * 3.3 * voltageDividerRatio;  // Convert ADC reading to voltage
  return voltage;
}

void displayVoltage(float voltage) {
  display.firstPage();  // Start ePaper display refresh
  do {
    display.fillScreen(GxEPD_WHITE);  // Clear the screen
    
    // Print voltage information
    display.setCursor(10, 50);        // Set cursor position
    display.print("Voltage: ");
    display.print(voltage);
    display.println(" V");
  } while (display.nextPage());  // Complete page update
}

void displayWelcomeMessage() {
  display.firstPage();  // Start ePaper display refresh
  do {
    display.fillScreen(GxEPD_WHITE);  // Clear the screen
    display.setCursor(10, 50);        // Set cursor position
    display.println("Monitoring Battery...");
  } while (display.nextPage());  // Complete page update
}

void goToDeepSleep() {
  ESP.deepSleep(10e6);  // Sleep for 10 seconds (in microseconds)
}
