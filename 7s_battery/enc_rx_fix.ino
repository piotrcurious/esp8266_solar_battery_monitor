#include <Arduino.h>
#include <avr/pgmspace.h>

// ... (previous code remains the same)

// New function prototypes
void removeRedundancyImproved(const char* input, char* output);
uint16_t predict_lfsr(uint16_t start_state, int steps);

// ... (other functions remain the same)

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

// ... (LFSR functions remain the same)

void removeRedundancyImproved(const char* input, char* output) {
  int inputLen = strlen(input);
  int outputIndex = 0;
  uint16_t local_lfsr_state = lfsr_state;  // Create a local copy of the LFSR state
  
  for (int i = 0; i < inputLen; i += REDUNDANCY_FACTOR) {
    int votes[256] = {0};
    int maxVotes = 0;
    char bestChar = 0;
    
    for (int j = 0; j < REDUNDANCY_FACTOR && (i + j) < inputLen; j++) {
      uint16_t predicted_lfsr = predict_lfsr(local_lfsr_state, j);
      char c = input[i + j] ^ (predicted_lfsr & 0xFF);
      votes[(unsigned char)c]++;
      if (votes[(unsigned char)c] > maxVotes) {
        maxVotes = votes[(unsigned char)c];
        bestChar = c;
      }
    }
    
    output[outputIndex++] = bestChar;
    
    // Update the local LFSR state
    for (int j = 0; j < REDUNDANCY_FACTOR; j++) {
      local_lfsr_state = predict_lfsr(local_lfsr_state, 1);
    }
  }
  output[outputIndex] = '\0';
}

uint16_t predict_lfsr(uint16_t start_state, int steps) {
  uint16_t state = start_state;
  for (int i = 0; i < steps; i++) {
    uint16_t bit = ((state >> 0) ^ (state >> 2) ^ (state >> 3) ^ (state >> 5)) & 1;
    state = (state >> 1) | (bit << 15);
  }
  return state;
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
    removeRedundancyImproved(segment, reconstructedMessage);
    
    // Process middle segments
    for (int i = 1; i < syncCount; i++) {
      int start = syncPositions[i - 1] + SYNC_MARKER_LENGTH;
      int length = syncPositions[i] - start;
      strncpy(segment, &rawMessage[start], length);
      segment[length] = '\0';
      removeRedundancyImproved(segment, &reconstructedMessage[strlen(reconstructedMessage)]);
    }
    
    // Process last segment
    int start = syncPositions[syncCount - 1] + SYNC_MARKER_LENGTH;
    strncpy(segment, &rawMessage[start], strlen(rawMessage) - start);
    segment[strlen(rawMessage) - start] = '\0';
    removeRedundancyImproved(segment, &reconstructedMessage[strlen(reconstructedMessage)]);
  } else {
    // If no sync markers found, try to decode the whole message
    removeRedundancyImproved(rawMessage, reconstructedMessage);
  }
}

// ... (rest of the code remains the same)
