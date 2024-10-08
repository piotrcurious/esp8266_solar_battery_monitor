// Constants
const float baseChargingVoltage = 14.4;   // Base voltage at 25°C
const float tempCompensationCoeffPosHigh = -0.022;  // -22mV/°C for positive plates at higher temps
const float tempCompensationCoeffNegHigh = -0.030;  // -30mV/°C for negative plates at higher temps
const float tempCompensationCoeffPosLow = -0.018;   // -18mV/°C for positive plates at lower temps
const float tempCompensationCoeffNegLow = -0.020;   // -20mV/°C for negative plates at lower temps
const float referenceTemp = 25.0;         // Reference temperature in °C
const float nonLinearFactor = 0.0002;     // Small quadratic non-linear factor for high temp compensation

// Function to calculate the compensated charging voltage
float getCompensatedVoltage(float batteryTemp) {
  float voltageOffset = 0.0;

  // Piecewise linear compensation based on temperature region
  if (batteryTemp > referenceTemp) {
    // Above reference temperature, account for non-linear behavior
    float tempDiff = batteryTemp - referenceTemp;
    
    // Use different coefficients for positive and negative plate chemistry
    float posPlateComp = tempCompensationCoeffPosHigh * tempDiff;
    float negPlateComp = tempCompensationCoeffNegHigh * tempDiff;
    
    // Add a small quadratic factor for non-linear behavior at higher temperatures
    voltageOffset = posPlateComp + negPlateComp + (nonLinearFactor * tempDiff * tempDiff);
  } else {
    // Below reference temperature
    float tempDiff = referenceTemp - batteryTemp;
    
    // Different coefficients for positive and negative plate chemistry
    float posPlateComp = tempCompensationCoeffPosLow * tempDiff;
    float negPlateComp = tempCompensationCoeffNegLow * tempDiff;
    
    voltageOffset = posPlateComp + negPlateComp;
  }

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
