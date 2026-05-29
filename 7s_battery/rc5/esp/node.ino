#include <avr/sleep.h>
#include <avr/power.h>

#define DECODE_RC5
#include <IRremote.hpp>

// --- Configuration ---
const uint8_t DEVICE_ID = 5;
const int IR_RECEIVE_PIN = 2;
const int IR_SEND_PIN = 3;

volatile bool justWokenUp = false;

// Function to read VCC with 4-sample averaging
long readVcc() {
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(2);
  long sum = 0;
  for (int i = 0; i < 4; i++) {
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    sum += ADC;
    delay(1);
  }
  long average = sum / 4;
  if (average == 0) return 0;
  return 1125300L / average;
}

// Function to read Internal Temp Sensor (approximate)
long readTemp() {
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3); // Internal 1.1V ref, MUX ADC8
  #endif
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  return ADC; // Return raw value for master to interpret
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

void sendData16RC5(uint16_t data) {
  uint8_t sum = 0;
  for (int i = 0; i < 8; i++) {
    uint8_t payload = (data >> (i * 2)) & 0x03;
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

  while (millis() - startWait < 500 && !responded) {
    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.protocol == RC5 &&
          IrReceiver.decodedIRData.address == DEVICE_ID &&
          (IrReceiver.decodedIRData.command & 0x30) == 0x00) {

        uint8_t cmd = IrReceiver.decodedIRData.command;
        if (cmd == 0x01) { // Poll Voltage
          sendData16RC5((uint16_t)readVcc());
          responded = true;
        } else if (cmd == 0x02) { // Poll Aux (Temp)
          sendData16RC5((uint16_t)readTemp());
          responded = true;
        }
      }
      IrReceiver.resume();
    }
    if (!responded) delay(1);
  }

  goToSleep();
}
