// Constants
const float baseChargingVoltage = 14.4; // Base voltage at 25°C
const float tempCompensationCoeff = -0.024; // -24mV/°C per battery (6 cells)
const float referenceTemp = 25.0; // Reference temperature in °C

// Function to calculate the compensated charging voltage
float getCompensatedVoltage(float batteryTemp) {
  // Calculate the voltage offset based on temperature difference from reference
  float voltageOffset = (batteryTemp - referenceTemp) * tempCompensationCoeff;
  
  // Calculate the new charging voltage
  float compensatedVoltage = baseChargingVoltage + voltageOffset;

  // Ensure the voltage stays within safe limits (e.g., between 13.8V and 14.8V)
  compensatedVoltage = constrain(compensatedVoltage, 13.8, 14.8);

  return compensatedVoltage;
}

// Example usage with a temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin where the DS18B20 is connected
const int tempSensorPin = 2;

OneWire oneWire(tempSensorPin);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  sensors.begin();
}

void loop() {
  // Request temperature from sensor
  sensors.requestTemperatures();
  float batteryTemp = sensors.getTempCByIndex(0);

  // Get the compensated charging voltage
  float chargingVoltage = getCompensatedVoltage(batteryTemp);

  // Output to the charger (here we just print it to the serial monitor)
  Serial.print("Battery Temp: ");
  Serial.print(batteryTemp);
  Serial.print("°C, Charging Voltage: ");
  Serial.println(chargingVoltage);

  // Simulate a delay before the next reading
  delay(5000);
}
