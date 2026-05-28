#include <avr/sleep.h>
#include <avr/power.h>
#include <IRremote.h> // Ensure you use a version that supports RC5

// Configuration
const uint8_t DEVICE_ID = 5;      // Unique ID for this cell (0-31)
const int IR_RECEIVE_PIN = 2;     // Must be INT0 for wakeup
const int IR_SEND_PIN = 3;        // PWM pin for IR LED
IRsend irsend;
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

// Function to read VCC (Internal 1.1V ref)
long readVcc() {
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(5); 
  ADCSRA |= _BV(ADSC); 
  while (bit_is_set(ADCSRA, ADSC));
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  long result = (high << 8) | low;
  result = 1125300L / result; 
  return result; 
}

// Function to put device into deep sleep
void goToSleep() {
  Serial.println("Entering Deep Sleep...");
  delay(100); // Allow serial to flush
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  // Disable ADC to save power
  byte old_ADCSRA = ADCSRA;
  ADCSRA = 0;
  
  attachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN), wakeUp, LOW);
  
  sleep_cpu();
  
  // --- SLEEPING HERE ---
  
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN));
  ADCSRA = old_ADCSRA; // Re-enable ADC
  Serial.println("Woken up!");
}

void wakeUp() {
  // Handler for external interrupt to wake the MCU
}

void sendFloatRC5(float value) {
  // Convert float to byte array (4 bytes = 32 bits)
  uint8_t *dataPtr = (uint8_t *)&value;
  
  // We need to send 32 bits. Using 2 bits of payload per RC5 packet = 16 packets.
  // Alternatively, we use 4 bits per packet = 8 packets.
  // Command structure: [1][Seq: 3 bits][Data: 2 bits]
  
  for (int i = 0; i < 4; i++) { // For each byte
    uint8_t byteToSend = dataPtr[i];
    
    for (int chunk = 0; chunk < 4; chunk++) { // 4 chunks of 2 bits per byte
      uint8_t sequence = (i * 4) + chunk; // 0 to 15
      uint8_t payload = (byteToSend >> (chunk * 2)) & 0x03;
      
      // Construct RC5 Command: 
      // Bit 5: 1 (Data Flag)
      // Bit 4-1: Sequence (0-15)
      // Bit 0: Placeholder or extra bit? 
      // Let's refine: RC5 has 6 command bits. [1][Seq: 3 bits][Data: 2 bits]
      // This allows 8 chunks (16 bits total). 
      // To send 32 bits, we will send 16 packets.
      
      uint8_t rc5Command = 0x20 | ((sequence & 0x07) << 2) | payload;
      
      // Send: Address (Device ID), Command, Toggle bit
      irsend.sendRC5(DEVICE_ID, rc5Command, 1);
      delay(50); // Gap between fragments
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(IR_RECEIVE_PIN, INPUT);
  irrecv.enableIRIn();
}

void loop() {
  if (irrecv.decode(&results)) {
    // Check if the RC5 address matches our Device ID
    // RC5 Address is bits 6-10 in many IR libraries
    if (results.decode_type == RC5 && results.address == DEVICE_ID) {
      
      // Execute command 0x01: Request Voltage
      if (results.value == 0x01) {
        float vcc = readVcc() / 1000.0; // Convert mV to Float Volts
        sendFloatRC5(vcc);
      }
    }
    irrecv.resume();
  }
  
  // If no signal for a while, go back to sleep
  goToSleep();
}
