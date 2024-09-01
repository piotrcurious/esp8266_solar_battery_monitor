#include <Arduino.h>
#include <avr/pgmspace.h>

// ... (previous code remains the same)

// New function prototypes
void removeRedundancyWithResync(const char* input, char* output);
uint16_t predict_lfsr(uint16_t start_state, int steps);
uint16_t reverse_lfsr(uint16_t end_state, int steps);
bool is_valid_base64_char(char c);

// ... (other functions remain the same)

void removeRedundancyWithResync(const char* input, char* output) {
  int inputLen = strlen(input);
  int outputIndex = 0;
  uint16_t local_lfsr_state = lfsr_state;  // Create a local copy of the LFSR state
  
  for (int i = 0; i < inputLen; i += REDUNDANCY_FACTOR) {
    int votes[256] = {0};
    int maxVotes = 0;
    char bestChar = 0;
    bool found_reliable_char = false;
    int reliable_char_index = -1;
    
    for (int j = 0; j < REDUNDANCY_FACTOR && (i + j) < inputLen; j++) {
      uint16_t predicted_lfsr = predict_lfsr(local_lfsr_state, j);
      char c = input[i + j] ^ (predicted_lfsr & 0xFF);
      votes[(unsigned char)c]++;
      if (votes[(unsigned char)c] > maxVotes) {
        maxVotes = votes[(unsigned char)c];
        bestChar = c;
      }
      
      // Check if this character is a valid base64 character and has a high vote count
      if (is_valid_base64_char(c) && votes[(unsigned char)c] >= REDUNDANCY_FACTOR / 2) {
        found_reliable_char = true;
        reliable_char_index = j;
        bestChar = c;
        break;  // We found a reliable character, no need to continue voting
      }
    }
    
    if (found_reliable_char) {
      // Resynchronize LFSR state based on the reliable character
      uint16_t expected_lfsr = input[i + reliable_char_index] ^ bestChar;
      local_lfsr_state = reverse_lfsr(expected_lfsr, reliable_char_index);
    }
    
    output[outputIndex++] = bestChar;
    
    // Update the local LFSR state
    local_lfsr_state = predict_lfsr(local_lfsr_state, REDUNDANCY_FACTOR);
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

uint16_t reverse_lfsr(uint16_t end_state, int steps) {
  uint16_t state = end_state;
  for (int i = 0; i < steps; i++) {
    uint16_t bit = (state & 1) ^ ((state >> 14) & 1) ^ ((state >> 13) & 1) ^ ((state >> 11) & 1);
    state = (state << 1) | bit;
  }
  return state;
}

bool is_valid_base64_char(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '+' || c == '/';
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
    removeRedundancyWithResync(segment, reconstructedMessage);
    
    // Process middle segments
    for (int i = 1; i < syncCount; i++) {
      int start = syncPositions[i - 1] + SYNC_MARKER_LENGTH;
      int length = syncPositions[i] - start;
      strncpy(segment, &rawMessage[start], length);
      segment[length] = '\0';
      removeRedundancyWithResync(segment, &reconstructedMessage[strlen(reconstructedMessage)]);
    }
    
    // Process last segment
    int start = syncPositions[syncCount - 1] + SYNC_MARKER_LENGTH;
    strncpy(segment, &rawMessage[start], strlen(rawMessage) - start);
    segment[strlen(rawMessage) - start] = '\0';
    removeRedundancyWithResync(segment, &reconstructedMessage[strlen(reconstructedMessage)]);
  } else {
    // If no sync markers found, try to decode the whole message
    removeRedundancyWithResync(rawMessage, reconstructedMessage);
  }
}

// ... (rest of the code remains the same)
