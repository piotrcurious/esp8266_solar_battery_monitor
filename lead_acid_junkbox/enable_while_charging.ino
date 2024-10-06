//This code uses esp8266 to measure car battery voltage 
//it also enables gpio4 pin if voltage is high enough (battery is charging) 
//this allows charging something else, like power bank or additional battery only when battery is charging 
//perhaps attiny85 could do the same job, but esp8266 is almost same price and allows easier extensions


#include <ESP8266WiFi.h> // Required for ESP deep sleep functionality
ADC_MODE(ADC_VCC);       // Set ADC to monitor the VCC (for ESP8266 internal ADC usage)

const int gpioPin = 4;  // GPIO4 to control
const int adcPin = A0;  // Pin to measure voltage (use a voltage divider to scale 12V to ESP8266 ADC range)
const float voltageDividerRatio = 4.0;  // Adjust depending on the voltage divider (example: 4:1 to scale 12V down to 3V3)
const float highThreshold = 14.0;       // Voltage threshold to stay awake
const float lowThreshold = 13.8;        // Voltage threshold to go back to deep sleep

void setup() {
  pinMode(gpioPin, OUTPUT);
  digitalWrite(gpioPin, LOW); // Start with GPIO4 off
  Serial.begin(115200);
}

void loop() {
  float batteryVoltage = readBatteryVoltage();

  Serial.print("Battery Voltage: ");
  Serial.println(batteryVoltage);

  if (batteryVoltage > highThreshold) {
    Serial.println("Voltage is above 14V, staying awake...");
    digitalWrite(gpioPin, HIGH);  // Turn on GPIO4
    
    // Monitor the voltage every 1 second
    while (batteryVoltage > lowThreshold) {
      delay(1000);  // Wait for 1 second
      batteryVoltage = readBatteryVoltage();
      Serial.print("Monitoring Voltage: ");
      Serial.println(batteryVoltage);
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

void goToDeepSleep() {
  ESP.deepSleep(10e6);  // Sleep for 10 seconds (in microseconds)
}
