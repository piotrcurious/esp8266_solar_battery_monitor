#include <avr/sleep.h>
#include <avr/power.h>

#define DECODE_RC5
#include <IRremote.hpp>

// --- Configuration ---
const uint8_t DEVICE_ID = 5;      // Unique ID for this specific cell (0 to 31)
const int IR_RECEIVE_PIN = 2;     // Must be D2 (INT0) to allow waking up from PWR_DOWN
const int IR_SEND_PIN = 3;        // Standard PWM pin for IR transmission (OC2B on ATmega328P)

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
  if (result == 0) return 0;
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
  
  // Disable IR receiving while going to sleep
  IrReceiver.stop();
  
  // Configure low-power mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  // Disable Analog-to-Digital Converter to save additional power
  byte old_ADCSRA = ADCSRA;
  ADCSRA = 0;
  
  // Attach interrupt to wake up when IR receiver pulls the line LOW
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
  
  // Re-enable the IR Receiver
  IrReceiver.start();
  
  Serial.println(F("System Woken Up!"));
}

// Sends a 16-bit voltage (mV) over RC5 in 8 sequential chunks
void sendVoltageRC5(uint16_t mv) {
  for (int i = 0; i < 8; i++) { // 8 chunks of 2 bits = 16 bits
    uint8_t payload = (mv >> (i * 2)) & 0x03; // Extract 2 bits
    
    // Construct the 6-bit RC5 Command:
    // Bit 5: 1 (Data Flag)
    // Bits 4-2: Sequence Index (0 to 7)
    // Bits 1-0: 2 bits of raw data payload
    uint8_t rc5Command = 0x20 | (i << 2) | payload;

    // Transmit the fragment
    IrSender.sendRC5(DEVICE_ID, rc5Command, 0); // 0 repeats for speed

    // Small gap between chunks
    delay(30);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(IR_RECEIVE_PIN, INPUT_PULLUP);
  
  // Initialize the IR receiver and sender
  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
  IrSender.begin(IR_SEND_PIN, DISABLE_LED_FEEDBACK);

  Serial.print(F("Cell Node Initialized. ID: "));
  Serial.println(DEVICE_ID);
}

void loop() {
  unsigned long startWait = millis();
  
  // When waking up, give the system 500ms to receive and decode commands
  while (millis() - startWait < 500) {
    if (IrReceiver.decode()) {
      // Check if it's RC5 and for our address
      if (IrReceiver.decodedIRData.protocol == RC5 && IrReceiver.decodedIRData.address == DEVICE_ID) {
        // Command 0x01: Master request for battery voltage
        if (IrReceiver.decodedIRData.command == 0x01) {
          Serial.println(F("Query Received!"));
          
          uint16_t vcc = (uint16_t)readVcc();
          
          Serial.print(F("Vcc: "));
          Serial.print(vcc);
          Serial.println(F("mV"));
          
          sendVoltageRC5(vcc);
        }
      }
      IrReceiver.resume();
    }
  }
  
  goToSleep();
}
