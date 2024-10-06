#include <Wire.h>
#include <Adafruit_SHT4x.h>

// Define pin for battery voltage measurement
const int batteryPin = A0; // Analog pin to read battery voltage

// Define voltage divider resistor values (in ohms)
const float R1 = 100000.0; // 100kΩ
const float R2 = 33000.0;  // 33kΩ

// Define lead-acid battery characteristics
const float nominalFloatVoltage = 13.8;  // Float voltage at 25°C
const float nominalOutgassingVoltage = 14.4; // Outgassing threshold at 25°C
const float tempCompensation = -0.003;  // Voltage compensation per cell per °C

// Initialize SH4x sensor
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// Function to read battery voltage
float readBatteryVoltage() {
  int analogValue = analogRead(batteryPin);
  float voltage = analogValue * (5.0 / 1023.0); // Convert to voltage
  return voltage * ((R1 + R2) / R2); // Adjust for voltage divider
}

// Function to compute corrected battery thresholds
void computeThresholds(float temp, float &floatVoltage, float &outgassingVoltage) {
  int cells = 6; // 12V lead-acid battery has 6 cells
  float tempCorrection = tempCompensation * (temp - 25.0) * cells;
  floatVoltage = nominalFloatVoltage + tempCorrection;
  outgassingVoltage = nominalOutgassingVoltage + tempCorrection;
}

void setup() {
  Serial.begin(9600);
  
  // Initialize SH4x sensor
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT4x sensor!");
    while (1);
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  
  Serial.println("Lead-acid Battery Monitor");
}

void loop() {
  // Read battery voltage
  float batteryVoltage = readBatteryVoltage();
  
  // Read temperature and humidity from SH4x sensor
  sensors_event_t temp_event, humidity_event;
  sht4.getEvent(&temp_event, &humidity_event);
  float temperature = temp_event.temperature;
  float humidity = humidity_event.relative_humidity;

  // Compute corrected voltage thresholds
  float floatVoltage, outgassingVoltage;
  computeThresholds(temperature, floatVoltage, outgassingVoltage);
  
  // Output results
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage);
  Serial.println(" V");

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Corrected Float Voltage: ");
  Serial.print(floatVoltage);
  Serial.println(" V");
  
  Serial.print("Corrected Outgassing Threshold Voltage: ");
  Serial.print(outgassingVoltage);
  Serial.println(" V");
  
  delay(2000); // Wait 2 seconds before next reading
}
