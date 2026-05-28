```cpp
#include <avr/sleep.h>
#include <avr/power.h>
#include <IRremote.h> // Using standard IRremote library for Arduino (AVR)

// --- Configuration ---
const uint8_t DEVICE_ID = 5;      // Unique ID for this specific cell (0 to 31)
const int IR_RECEIVE_PIN = 2;     // Must be D2 (INT0) to allow waking up from PWR_DOWN
const int IR_SEND_PIN = 3;        // Standard PWM pin for IR transmission (OC2B on ATmega328P)

IRsend irsend;
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

// Helper to keep track of waking up state
volatile bool justWokenUp = false;

// Function to read VCC (Internal 1.1V ref)
long readVcc() {
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    // Set reference to Vcc, multiplexer input to internal 1.1V reference bandgap
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  
  delay(10);           // Allow Vref reference voltage to fully settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // Wait for conversion to complete

  uint8_t low  = ADCL; // Lock register
  uint8_t high = ADCH; // Unlock both registers

  long result = (high << 8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV)
  return result; // Vcc in millivolts
}

// ISR (Interrupt Service Routine) for INT0 (Pin 2)
void wakeUp() {
  justWokenUp = true;
}

// Function to put device into deep sleep
void goToSleep() {
  Serial.println(F("Going to Deep Sleep..."));
  Serial.flush();
  
  // Disable IR receiving while going to sleep to prevent race conditions
  irrecv.disableIRIn();
  
  // Configure low-power mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  // Disable Analog-to-Digital Converter to save additional power
  byte old_ADCSRA = ADCSRA;
  ADCSRA = 0;
  
  // Attach interrupt to wake up when IR receiver pulls the line LOW (starts receiving carrier)
  // Low level interrupt is required to wake MCU up from SLEEP_MODE_PWR_DOWN
  attachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN), wakeUp, LOW);
  
  // Put MCU to sleep here
  sleep_cpu();
  
  // ----------------------------------------------------
  // MCU Wakes Up Here
  // ----------------------------------------------------
  
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN));
  
  // Restore Analog-to-Digital Converter
  ADCSRA = old_ADCSRA; 
  
  // Re-enable and reset the IR Receiver to prepare for parsing
  irrecv.enableIRIn();
  
  Serial.println(F("System Woken Up!"));
}

// Sends a 32-bit float over RC5 in 16 sequential chunks
void sendFloatRC5(float value) {
  uint8_t *dataPtr = (uint8_t *)&value;
  
  for (int i = 0; i < 4; i++) { // For each of the 4 bytes in a 32-bit float
    uint8_t byteToSend = dataPtr[i];
    
    for (int chunk = 0; chunk < 4; chunk++) { // Split each byte into 4 chunks (2 bits each)
      uint8_t sequence = (i * 4) + chunk;     // Unique index from 0 to 15
      uint8_t payload = (byteToSend >> (chunk * 2)) & 0x03; // Extract 2 bits
      
      // Construct the 6-bit RC5 Command:
      // Bit 5: 1 (Data Flag / Separator)
      // Bits 4-2: Sequence & 0x07 (3-bit sequence mapping, wraps twice: 0-7, then 0-7)
      // Bits 1-0: 2 bits of raw data payload
      uint8_t rc5Command = 0x20 | ((sequence & 0x07) << 2) | payload;
      
      // Transmit the fragment
      irsend.sendRC5(DEVICE_ID, rc5Command, 1);
      
      // Small gap between chunks to let Master receive and buffer the current packet
      delay(40); 
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(IR_RECEIVE_PIN, INPUT_PULLUP);
  
  // Initialize the IR receiver
  irrecv.enableIRIn();
  Serial.print(F("Cell Node Initialized. ID: "));
  Serial.println(DEVICE_ID);
}

void loop() {
  bool activityDetected = false;
  unsigned long startWait = millis();
  
  // When waking up, give the system 500ms to receive and decode commands before going back to sleep
  while (millis() - startWait < 500) {
    if (irrecv.decode(&results)) {
      activityDetected = true;
      
      // Ensure the received transmission is RC5 and matches our cell's assigned Device ID
      if (results.decode_type == RC5 && results.address == DEVICE_ID) {
        
        // Command 0x01: Master request for battery voltage
        if (results.value == 0x01) {
          Serial.println(F("Query Received! Sending Battery Status..."));
          
          float vcc = readVcc() / 1000.0; // Calculate Vcc and convert to Volts
          
          Serial.print(F("Current Vcc: "));
          Serial.print(vcc, 3);
          Serial.println(F("V"));
          
          sendFloatRC5(vcc); // Send back encoded as 16 chunk frames
        }
      }
      irrecv.resume(); // Receive the next value
    }
  }
  
  // If no IR signals were targeting this device, return to deep sleep
  goToSleep();
}

```
