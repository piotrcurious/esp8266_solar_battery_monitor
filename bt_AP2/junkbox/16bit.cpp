#include <stdint.h>
#include <stdbool.h>

// Maximum number of lines to draw
#define MAX_LINES 1024  // Adjust based on the actual buffer size

// Variables to store the current state of the LFSR and maximum lines
uint16_t lfsr = 1;  // Initial seed value for the LFSR
int maxLines = MAX_LINES;  // Set this to the maximum number of lines in the buffer

// Function to implement a 16-bit Galois LFSR with wraparound tracking
uint16_t galoisLFSR(bool *wrapped) {
    static uint16_t initialSeed = 1;  // Track the initial seed to detect wraparound
    uint16_t lsb = lfsr & 1;  // Get the least significant bit
    lfsr >>= 1;               // Shift the LFSR right by 1
    if (lsb) {
        // Apply the polynomial (0xB400 corresponds to the polynomial x^16 + x^14 + x^13 + x^11 + 1)
        lfsr ^= 0xB400;
    }

    // Check if the LFSR has wrapped around to the initial seed value
    if (lfsr == initialSeed) {
        *wrapped = true;
    }

    return lfsr;
}

// Function to draw a specified number of lines based on the LFSR sequence
void drawLinesUsingLFSR(int numLinesToDraw) {
    int linesDrawn = 0;
    bool wrapped = false;

    while (linesDrawn < numLinesToDraw) {
        uint16_t lineIndex = galoisLFSR(&wrapped);

        // Ensure the generated index is within the range of valid buffered lines
        if (lineIndex < lineBufferIndex) {
            drawBufferedLine(lineIndex);
            linesDrawn++;
        }

        // Break if the LFSR has wrapped around to avoid infinite looping
        if (wrapped) {
            break;
        }
    }
}
