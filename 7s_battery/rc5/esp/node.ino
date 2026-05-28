#include <avr/sleep.h>
#include <avr/power.h>

#define DECODE_RC5
#include <IRremote.hpp>

// --- Configuration ---
const uint8_t DEVICE_ID = 5;
const int IR_RECEIVE_PIN = 2;
const int IR_SEND_PIN = 3;

volatile bool justWokenUp = false;

long readVcc() {
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  
  // Reduced settling time from 10ms to 2ms
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));

  uint8_t low  = ADCL;
  uint8_t high = ADCH;

  long result = (high << 8) | low;
  if (result == 0) return 0;
  result = 1125300L / result;
  return result;
}

void wakeUp() {
  justWokenUp = true;
}

void goToSleep() {
  IrReceiver.stop();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  byte old_ADCSRA = ADCSRA;
  ADCSRA = 0;
  attachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN), wakeUp, LOW);
  sleep_cpu();
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(IR_RECEIVE_PIN));
  ADCSRA = old_ADCSRA; 
  IrReceiver.start();
}

void sendVoltageRC5(uint16_t mv) {
  uint8_t sum = 0;
  for (int i = 0; i < 8; i++) {
    uint8_t payload = (mv >> (i * 2)) & 0x03;
    sum += payload;
    uint8_t rc5Command = 0x20 | (i << 2) | payload;
    IrSender.sendRC5(DEVICE_ID, rc5Command, 0);
    delay(30);
  }

  uint8_t checksumCommand = 0x10 | (sum & 0x0F);
  IrSender.sendRC5(DEVICE_ID, checksumCommand, 0);
  delay(30);
}

void setup() {
  Serial.begin(9600);
  pinMode(IR_RECEIVE_PIN, INPUT_PULLUP);
  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
  IrSender.begin(IR_SEND_PIN, DISABLE_LED_FEEDBACK);
  Serial.print(F("Cell Node Initialized. ID: "));
  Serial.println(DEVICE_ID);
}

void loop() {
  unsigned long startWait = millis();
  bool responded = false;

  // Power efficiency: exit loop early if we responded
  while (millis() - startWait < 500 && !responded) {
    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.protocol == RC5 &&
          IrReceiver.decodedIRData.address == DEVICE_ID &&
          (IrReceiver.decodedIRData.command & 0x30) == 0x00) {

        if (IrReceiver.decodedIRData.command == 0x01) {
          uint16_t vcc = (uint16_t)readVcc();
          sendVoltageRC5(vcc);
          responded = true; // Flag to go back to sleep immediately
        }
      }
      IrReceiver.resume();
    }
    // Small yield to allow other background tasks (if any)
    if (!responded) delay(1);
  }

  goToSleep();
}
