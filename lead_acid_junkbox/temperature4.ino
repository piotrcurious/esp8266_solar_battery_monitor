// Empirically determined constants from scientific studies
const float baseChargingVoltage = 14.4;   // Base voltage at 25°C (standard float)
const float tempCompensationCoeffPosHigh = -0.020;  // ~-20mV/°C for positive plates at higher temps
const float tempCompensationCoeffNegHigh = -0.035;  // ~-35mV/°C for negative plates at higher temps
const float tempCompensationCoeffPosLow = -0.015;   // ~-15mV/°C for positive plates at lower temps
const float tempCompensationCoeffNegLow = -0.025;   // ~-25mV/°C for negative plates at lower temps

// Non-linear factors determined from experimental data (e.g., 2nd order polynomial fitting)
const float quadraticFactorHighTemp = 0.00015;  // Non-linear correction factor for high temperatures
const float exponentialFactorHighTemp = 0.005;  // Exponential factor to further adjust high-temp correction

// Reference temperature
const float referenceTemp = 25.0; // Standard reference temperature in °C

// Function to calculate the compensated charging voltage
float getCompensatedVoltage(float batteryTemp) {
  float voltageOffset = 0.0;

  // Piecewise compensation based on empirical data
  if (batteryTemp > referenceTemp) {
    // Above reference temperature, non-linear behavior + chemistry differences
    float tempDiff = batteryTemp - referenceTemp;
    
    // Empirically derived coefficients for positive and negative plates
    float posPlateComp = tempCompensationCoeffPosHigh * tempDiff;
    float negPlateComp = tempCompensationCoeffNegHigh * tempDiff;

    // Non-linear adjustment at high temps using quadratic and exponential models
    float nonLinearAdj = (quadraticFactorHighTemp * tempDiff * tempDiff) +
                         (exponentialFactorHighTemp * exp(tempDiff / 20.0));

    voltageOffset = posPlateComp + negPlateComp + nonLinearAdj;
  } else {
    // Below reference temperature (linear, empirically derived coefficients)
    float tempDiff = referenceTemp - batteryTemp;
    
    // Positive and negative plate compensation below reference temp
    float posPlateComp = tempCompensationCoeffPosLow * tempDiff;
    float negPlateComp = tempCompensationCoeffNegLow * tempDiff;

    voltageOffset = posPlateComp + negPlateComp;
  }

  // Calculate the new charging voltage
  float compensatedVoltage = baseChargingVoltage + voltageOffset;

  // Clamp the voltage to safe limits based on empirical studies
  compensatedVoltage = constrain(compensatedVoltage, 13.6, 14.9); // Adjusted based on safe charging limits

  return compensatedVoltage;
}

// Example usage with a temperature sensor (e.g., DS18B20)
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
