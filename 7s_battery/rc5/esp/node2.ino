#include <avr/sleep.h>
#include <avr/power.h>

#define DECODE_RC5
#include <IRremote.hpp>

// --- Configuration ---
const uint8_t DEVICE_ID = 5;
const int IR_RECEIVE_PIN = 2;
const int IR_SEND_PIN = 3;
const long INTERNAL_REF_VOLTAGE = 1125300L; // Adjust if 1.1V ref is slightly different

// Function to read VCC with improved stability
long readVcc() {
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  
  delay(5); // Allow voltage reference to settle
  
  // Dummy read to clear any stale values
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));

  long sum = 0;
  for (int i = 0; i < 4; i++) {
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    sum += ADC;
  }
  
  long average = sum / 4;
  return (average == 0) ? 0 : (INTERNAL_REF_VOLTAGE / average);
}

// Internal Temp Sensor (Note: Needs per-chip calibration)
long readTemp() {
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3); 
  #endif
  
  delay(5); 
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC));
  
  return ADC; 
}

void wakeUp() {
  // ISR: Keep it minimal
  sleep_disable();
}

void goToSleep() {
  IrReceiver.stop();
  
  // Turn off ADC and peripherals
  byte old_ADCSRA = ADCSRA;
  ADCSRA = 0; 
  power_all_disable(); 

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  // Use LOW level interrupt for waking from Power Down
  attachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN), wakeUp, LOW);
  
  sleep_cpu(); 

  // --- WAKING UP HERE ---
  
  detachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN));
  power_all_enable();
  ADCSRA = old_ADCSRA;
  IrReceiver.start();
}

// Chunks 16-bit data into RC5 commands
void sendData16RC5(uint16_t data) {
  uint8_t sum = 0;
  for (int i = 0; i < 8; i++) {
    uint8_t payload = (data >> (i * 2)) & 0x03;
    sum += payload;
    // Command format: [0 1 (3-bit index) (2-bit payload)]
    uint8_t rc5Command = 0x20 | (i << 2) | payload;
    IrSender.sendRC5(DEVICE_ID, rc5Command, 0);
    delay(40); // Slightly longer delay for receiver stability
  }
  // Send 4-bit checksum
  uint8_t checksumCommand = 0x10 | (sum & 0x0F);
  IrSender.sendRC5(DEVICE_ID, checksumCommand, 0);
}

void setup() {
  // Lower baud rate saves a tiny bit of power, but 9600 is fine
  Serial.begin(9600);
  pinMode(IR_RECEIVE_PIN, INPUT_PULLUP);
  
  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
  IrSender.begin(IR_SEND_PIN, DISABLE_LED_FEEDBACK);
  
  Serial.print(F("Node Online. ID: "));
  Serial.println(DEVICE_ID);
}

void loop() {
  unsigned long startWait = millis();
  bool responded = false;

  // Listen for 500ms before going back to sleep
  while (millis() - startWait < 500) {
    if (IrReceiver.decode()) {
      // Logic check: Protocol match and address match
      if (IrReceiver.decodedIRData.protocol == RC5 &&
          IrReceiver.decodedIRData.address == DEVICE_ID) {
        
        uint8_t cmd = IrReceiver.decodedIRData.command;
        
        // Filter for specific command range (0x00 to 0x0F for requests)
        if ((cmd & 0x30) == 0x00) {
          if (cmd == 0x01) {
            sendData16RC5((uint16_t)readVcc());
            responded = true;
          } else if (cmd == 0x02) {
            sendData16RC5((uint16_t)readTemp());
            responded = true;
          }
        }
      }
      IrReceiver.resume();
      if (responded) break; // Exit loop early to save power
    }
  }

  goToSleep();
}
