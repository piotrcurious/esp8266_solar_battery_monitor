#include <Arduino.h>
#include <avr/pgmspace.h>

// Base64 encoding table
const char PROGMEM b64_alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// LFSR parameters
#define LFSR_BITS 16
#define LFSR_TAPS 0xB400  // Taps for x^16 + x^14 + x^13 + x^11 + 1

// Error correction parameters
#define REDUNDANCY_FACTOR 10
#define SYNC_MARKER_LENGTH 8
#define MESSAGE_LENGTH (16 * 4 * 8 * REDUNDANCY_FACTOR / 6)  // 16 floats * 4 bytes * 8 bits * redundancy / 6 bits per base64 char

// Function prototypes
void initLFSR();
uint16_t stepLFSR();
void encodeFloat(float value, char* output);
void addRedundancy(const char* input, char* output);
void addSyncMarkers(char* message);
void sendEncodedMessage(const char* message);

// Global variables
uint16_t lfsr_state;

void setup() {
  Serial.begin(115200);
  initLFSR();
}

void loop() {
  // Sample array of 16 float variables
  float data[16] = {1.23, 4.56, 7.89, 0.12, 3.45, 6.78, 9.01, 2.34, 5.67, 8.90, 1.23, 4.56, 7.89, 0.12, 3.45, 6.78};
  
  char encoded[MESSAGE_LENGTH + 1];  // +1 for null terminator
  char *ptr = encoded;
  
  // Encode each float
  for (int i = 0; i < 16; i++) {
    encodeFloat(data[i], ptr);
    ptr += 8;  // Move pointer by 8 characters (6 for base64 + 2 for padding)
  }
  *ptr = '\0';  // Null terminate the string
  
  // Add redundancy
  char redundantMessage[MESSAGE_LENGTH * REDUNDANCY_FACTOR + 1];
  addRedundancy(encoded, redundantMessage);
  
  // Add sync markers
  addSyncMarkers(redundantMessage);
  
  // Send the message
  sendEncodedMessage(redundantMessage);
  
  delay(5000);  // Wait 5 seconds before sending the next message
}

void initLFSR() {
  lfsr_state = 0xACE1;  // Non-zero initial state
}

uint16_t stepLFSR() {
  uint16_t bit = ((lfsr_state >> 0) ^ (lfsr_state >> 2) ^ (lfsr_state >> 3) ^ (lfsr_state >> 5)) & 1;
  lfsr_state = (lfsr_state >> 1) | (bit << 15);
  return lfsr_state;
}

void encodeFloat(float value, char* output) {
  uint8_t *bytes = (uint8_t*)&value;
  uint32_t n = ((uint32_t)bytes[3] << 24) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[1] << 8) | bytes[0];
  
  for (int i = 0; i < 6; i++) {
    output[i] = pgm_read_byte(&b64_alphabet[(n >> (6 * (5 - i))) & 0x3F]);
  }
  output[6] = '=';
  output[7] = '=';
}

void addRedundancy(const char* input, char* output) {
  int inputLen = strlen(input);
  int outputIndex = 0;
  
  for (int i = 0; i < inputLen; i++) {
    for (int j = 0; j < REDUNDANCY_FACTOR; j++) {
      output[outputIndex++] = input[i] ^ (stepLFSR() & 0xFF);
    }
  }
  output[outputIndex] = '\0';
}

void addSyncMarkers(char* message) {
  int messageLen = strlen(message);
  int markerInterval = messageLen / (SYNC_MARKER_LENGTH + 1);
  
  for (int i = markerInterval; i < messageLen; i += markerInterval) {
    for (int j = 0; j < SYNC_MARKER_LENGTH; j++) {
      message[i + j] = stepLFSR() & 0xFF;
    }
  }
}

void sendEncodedMessage(const char* message) {
  Serial.println("START_MESSAGE");
  Serial.println(message);
  Serial.println("END_MESSAGE");
}
