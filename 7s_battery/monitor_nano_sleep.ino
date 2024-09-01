#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// Define SoftwareSerial pins for output
SoftwareSerial mySerial(10, 11);  // RX, TX (connect TX to your receiving device)

// Define the analog input pins
const int analogPins[7] = {A0, A1, A2, A3, A4, A5, A6};

// Maximum voltages for each pin (in volts), adjust these according to your setup
const float maxVoltage[7] = {4.2, 8.4, 12.6, 16.8, 21.0, 25.2, 29.4};

// Number of analog pins
const int numCells = 7;

// Variables to control sleep and wake up
volatile bool wdt_wakeup = false;

// Watchdog Timer interrupt service routine
ISR(WDT_vect) {
  wdt_wakeup = true; // Set flag to indicate WDT wakeup
}

// Function to enable WDT for wakeup
void setupWatchdogTimer() {
  // Clear reset flag
  MCUSR &= ~(1 << WDRF);
  
  // Set Watchdog Timer to interrupt mode and set timeout to approximately 1 second
  WDTCSR = (1 << WDCE) | (1 << WDE);  // Enable configuration changes
  WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1);  // Set timeout to 1 second (approx.)
}

// Function to put the MCU into sleep mode
void enterSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set power-down mode
  sleep_enable();                       // Enable sleep mode
  sleep_mode();                         // Enter sleep mode
  
  // MCU will continue from here after waking up
  sleep_disable();                      // Disable sleep mode
}

// Function to read and convert analog inputs to voltage
float readVoltage(int pin, float maxVolt) {
  int analogValue = analogRead(pin);                   // Read the analog value (0-1023)
  float voltage = (analogValue / 1023.0) * maxVolt;    // Convert to voltage
  return voltage;
}

// Function to calculate individual cell voltages
void calculateCellVoltages(float* stackVoltages, float* cellVoltages) {
  cellVoltages[0] = stackVoltages[0];  // The first cell voltage is the same as the first stack voltage

  // Calculate each cell voltage by subtracting the previous stack voltage
  for (int i = 1; i < numCells; i++) {
    cellVoltages[i] = stackVoltages[i] - stackVoltages[i - 1];
  }
}

void setup() {
  mySerial.begin(1200);        // Start SoftwareSerial at 1200 baud
  setupWatchdogTimer();        // Setup WDT for sleep wakeup
}

void loop() {
  float stackVoltages[numCells];  // Array to store stack voltages
  float cellVoltages[numCells];   // Array to store individual cell voltages

  // Read and convert voltages for each pin
  for (int i = 0; i < numCells; i++) {
    stackVoltages[i] = readVoltage(analogPins[i], maxVoltage[i]);
  }

  // Calculate individual cell voltages
  calculateCellVoltages(stackVoltages, cellVoltages);

  // Print the individual cell voltages
  for (int i = 0; i < numCells; i++) {
    mySerial.print("Cell ");
    mySerial.print(i + 1);
    mySerial.print(": ");
    mySerial.print(cellVoltages[i], 3);  // Print voltage with 3 decimal places
    mySerial.println(" V");
  }

  delay(100);  // Short delay before entering sleep mode for stability

  // Enter sleep mode and wait for WDT wakeup
  wdt_wakeup = false;
  enterSleep();

  // Wait until WDT wakeup occurs
  while (!wdt_wakeup) {
    // MCU will stay in this loop until the ISR sets wdt_wakeup to true
  }
}
