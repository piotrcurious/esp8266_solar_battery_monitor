#include <Arduino.h>
#include <avr/pgmspace.h>

// Base64 decoding table
const char PROGMEM b64_index[] = {
    62, -1, -1, -1, 63, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1,
    -1, -1, -1, -1, -1, -1,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    -1, -1, -1, -1, -1, -1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
    36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51
};

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
float decodeFloat(const char* input);
void removeRedundancy(const char* input, char* output);
bool findSyncMarkers(const char* message, int* positions, int* count);
void reconstructMessage(const char* rawMessage, char* reconstructedMessage);

// Global variables
uint16_t lfsr_state;
char receivedMessage[MESSAGE_LENGTH * REDUNDANCY_FACTOR + 1];
int messageIndex = 0;
bool receivingMessage = false;

void setup() {
  Serial.begin(115200);
  initLFSR();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n') {
      if (receivingMessage) {
        receivedMessage[messageIndex] = '\0';
        processMessage();
        messageIndex = 0;
        receivingMessage = false;
      } else if (strcmp(receivedMessage, "START_MESSAGE") == 0) {
        receivingMessage = true;
        messageIndex = 0;
      }
    } else if (receivingMessage && messageIndex < sizeof(receivedMessage) - 1) {
      receivedMessage[messageIndex++] = c;
    } else if (!receivingMessage) {
      receivedMessage[messageIndex++] = c;
      receivedMessage[messageIndex] = '\0';
    }
  }
}

void processMessage() {
  char reconstructedMessage[MESSAGE_LENGTH + 1];
  reconstructMessage(receivedMessage, reconstructedMessage);
  
  float decodedFloats[16];
  for (int i = 0; i < 16; i++) {
    decodedFloats[i] = decodeFloat(&reconstructedMessage[i * 8]);
  }
  
  // Print decoded floats
  Serial.println("Decoded floats:");
  for (int i = 0; i < 16; i++) {
    Serial.print(decodedFloats[i], 6);
    Serial.print(" ");
  }
  Serial.println();
}

void initLFSR() {
  lfsr_state = 0xACE1;  // Non-zero initial state
}

uint16_t stepLFSR() {
  uint16_t bit = ((lfsr_state >> 0) ^ (lfsr_state >> 2) ^ (lfsr_state >> 3) ^ (lfsr_state >> 5)) & 1;
  lfsr_state = (lfsr_state >> 1) | (bit << 15);
  return lfsr_state;
}

float decodeFloat(const char* input) {
  uint32_t n = 0;
  for (int i = 0; i < 6; i++) {
    n = (n << 6) | pgm_read_byte(&b64_index[input[i] - 43]);
  }
  
  uint8_t bytes[4];
  bytes[0] = n & 0xFF;
  bytes[1] = (n >> 8) & 0xFF;
  bytes[2] = (n >> 16) & 0xFF;
  bytes[3] = (n >> 24) & 0xFF;
  
  float value;
  memcpy(&value, bytes, sizeof(float));
  return value;
}

void removeRedundancy(const char* input, char* output) {
  int inputLen = strlen(input);
  int outputIndex = 0;
  
  for (int i = 0; i < inputLen; i += REDUNDANCY_FACTOR) {
    int votes[256] = {0};
    int maxVotes = 0;
    char bestChar = 0;
    
    for (int j = 0; j < REDUNDANCY_FACTOR && (i + j) < inputLen; j++) {
      char c = input[i + j] ^ (stepLFSR() & 0xFF);
      votes[(unsigned char)c]++;
      if (votes[(unsigned char)c] > maxVotes) {
        maxVotes = votes[(unsigned char)c];
        bestChar = c;
      }
    }
    
    output[outputIndex++] = bestChar;
  }
  output[outputIndex] = '\0';
}

bool findSyncMarkers(const char* message, int* positions, int* count) {
  int messageLen = strlen(message);
  int markerInterval = messageLen / (SYNC_MARKER_LENGTH + 1);
  *count = 0;
  
  for (int i = markerInterval; i < messageLen; i += markerInterval) {
    bool isMarker = true;
    for (int j = 0; j < SYNC_MARKER_LENGTH; j++) {
      if (message[i + j] != (stepLFSR() & 0xFF)) {
        isMarker = false;
        break;
      }
    }
    if (isMarker) {
      positions[(*count)++] = i;
    }
    if (*count >= 10) break;  // Limit to 10 markers
  }
  
  return *count > 0;
}

void reconstructMessage(const char* rawMessage, char* reconstructedMessage) {
  int syncPositions[10];
  int syncCount;
  
  if (findSyncMarkers(rawMessage, syncPositions, &syncCount)) {
    int segmentLength = syncPositions[0];
    char segment[segmentLength + 1];
    
    // Process first segment
    strncpy(segment, rawMessage, segmentLength);
    segment[segmentLength] = '\0';
    removeRedundancy(segment, reconstructedMessage);
    
    // Process middle segments
    for (int i = 1; i < syncCount; i++) {
      int start = syncPositions[i - 1] + SYNC_MARKER_LENGTH;
      int length = syncPositions[i] - start;
      strncpy(segment, &rawMessage[start], length);
      segment[length] = '\0';
      removeRedundancy(segment, &reconstructedMessage[strlen(reconstructedMessage)]);
    }
    
    // Process last segment
    int start = syncPositions[syncCount - 1] + SYNC_MARKER_LENGTH;
    strncpy(segment, &rawMessage[start], strlen(rawMessage) - start);
    segment[strlen(rawMessage) - start] = '\0';
    removeRedundancy(segment, &reconstructedMessage[strlen(reconstructedMessage)]);
  } else {
    // If no sync markers found, try to decode the whole message
    removeRedundancy(rawMessage, reconstructedMessage);
  }
}
