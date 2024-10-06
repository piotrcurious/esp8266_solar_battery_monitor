#include <GxEPD2_BW.h>       // Include GxEPD2 library for ePaper
#include <GxEPD2_3C.h>       // Tri-color ePaper support
#include <Adafruit_GFX.h>    // Adafruit GFX library for font support

// Define display and pins for 2.13-inch tri-color ePaper
#define MAX_DISPLAY_BUFFER_SIZE 800 // Adjust if needed based on RAM limitations

GxEPD2_3C<GxEPD2_213c, MAX_DISPLAY_BUFFER_SIZE> display(GxEPD2_213c(/*CS*/ 15, /*DC*/ 4, /*RST*/ 5, /*BUSY*/ 16));

const int gpioPin = 4;  // GPIO4 to control
const int adcPin = A0;  // Pin to measure voltage (use a voltage divider to scale 12V to ESP8266 ADC range)
const float voltageDividerRatio = 4.0;  // Adjust based on your voltage divider (e.g., 4:1 for 12V to 3.3V scaling)
const float highThreshold = 14.0;       // Voltage threshold to stay awake
const float lowThreshold = 13.8;        // Voltage threshold to go back to deep sleep
const float redThreshold = 12.8;        // Display red for voltages below this

void setup() {
  pinMode(gpioPin, OUTPUT);
  digitalWrite(gpioPin, LOW);  // Start with GPIO4 off
  Serial.begin(115200);

  // Initialize the ePaper display
  display.init(115200);
  display.setRotation(1);  // Rotate for landscape view
  display.setTextColor(GxEPD_BLACK); // Default to black text
  display.setFont(&FreeSansBold18pt7b); // Set larger font from Adafruit GFX library
  display.fillScreen(GxEPD_WHITE); // Clear display with white background
  display.display(); // Apply changes
}

void loop() {
  float batteryVoltage = readBatteryVoltage();

  Serial.print("Battery Voltage: ");
  Serial.println(batteryVoltage);

  displayVoltage(batteryVoltage); // Display the current voltage on ePaper

  if (batteryVoltage > highThreshold) {
    Serial.println("Voltage is above 14V, staying awake...");
    digitalWrite(gpioPin, HIGH);  // Turn on GPIO4
    
    // Monitor the voltage every 1 second
    while (batteryVoltage > lowThreshold) {
      delay(1000);  // Wait for 1 second
      batteryVoltage = readBatteryVoltage();
      Serial.print("Monitoring Voltage: ");
      Serial.println(batteryVoltage);
      displayVoltage(batteryVoltage); // Update voltage on ePaper
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

// Function to display battery voltage on ePaper
void displayVoltage(float voltage) {
  display.fillScreen(GxEPD_WHITE);  // Clear the display

  // Set text color based on voltage threshold
  if (voltage < redThreshold) {
    display.setTextColor(GxEPD_RED);  // Display red text for voltages below 12.8V
  } else {
    display.setTextColor(GxEPD_BLACK);  // Display black text for voltages above 12.8V
  }

  // Display the voltage on ePaper
  display.setCursor(30, 90);  // Adjust position if necessary
  display.print("Voltage: ");
  display.print(voltage, 2);  // Print voltage with 2 decimal places
  display.println("V");

  display.display();  // Refresh the ePaper display to show changes
}

// Function to read the battery voltage from ADC
float readBatteryVoltage() {
  int analogValue = analogRead(adcPin);  // Read from A0 pin
  float voltage = (analogValue / 1024.0) * 3.3 * voltageDividerRatio;  // Convert ADC reading to voltage
  return voltage;
}

// Function to go to deep sleep
void goToDeepSleep() {
  display.hibernate();  // Put ePaper display in low-power mode
  ESP.deepSleep(10e6);  // Sleep for 10 seconds (in microseconds)
}
