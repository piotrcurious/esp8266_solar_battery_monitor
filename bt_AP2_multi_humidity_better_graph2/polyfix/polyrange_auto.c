#include <stdio.h>
#include <stdint.h>

// Predefined taps for maximal-length LFSR for each bit length (1-16)
static const uint16_t taps[] = {
    0x0,    // 0-bit doesn't exist
    0x1,    // 1-bit LFSR taps
    0x3,    // 2-bit LFSR taps
    0x6,    // 3-bit LFSR taps
    0xC,    // 4-bit LFSR taps
    0x14,   // 5-bit LFSR taps
    0x30,   // 6-bit LFSR taps
    0x60,   // 7-bit LFSR taps
    0xB4,   // 8-bit LFSR taps
    0x110,  // 9-bit LFSR taps
    0x240,  // 10-bit LFSR taps
    0x500,  // 11-bit LFSR taps
    0x829,  // 12-bit LFSR taps
    0x100D, // 13-bit LFSR taps
    0x2015, // 14-bit LFSR taps
    0x4023, // 15-bit LFSR taps
    0x8016  // 16-bit LFSR taps
};

// Assuming a struct for the graph schedule
typedef struct {
    uint16_t dataSize;  // Data size which determines LFSR steps
} GraphSchedule;

GraphSchedule graphSchedule[10]; // Assume graphSchedule has multiple entries

// Function to find the smallest bit length to cover a given dataSize
static uint16_t find_min_bit_length(uint16_t dataSize) {
    uint16_t bit_length = 0;
    
    // Increment bit length until 2^bit_length - 1 >= dataSize
    while ((1 << bit_length) - 1 < dataSize) {
        bit_length++;
    }
    
    return bit_length;
}

// Function to generate maximal length LFSR sequence based on graph schedule dataSize
void generate_lfsr_from_schedule(uint8_t graph_index) {
    uint16_t dataSize = graphSchedule[graph_index].dataSize;

    // Find the smallest bit length such that (2^n - 1) >= dataSize
    uint16_t bit_length = find_min_bit_length(dataSize);

    if (bit_length == 0 || bit_length > 16) {
        printf("Invalid bit length determined from dataSize.\n");
        return;
    }

    uint16_t max_steps = (1 << bit_length) - 1; // Max sequence length for n-bit LFSR
    uint16_t lfsr = 1; // Initial state (must not be 0)
    uint16_t polynomial = taps[bit_length]; // Select tap configuration based on bit length
    uint16_t bit;

    printf("LFSR sequence for graph index %d with %d-bit length (dataSize: %d):\n", graph_index, bit_length, dataSize);
    
    for (uint16_t i = 0; i < max_steps; i++) {
        printf("%d\n", lfsr); // Output the current state
        bit = lfsr & 1;       // Get the last bit (LSB)

        lfsr >>= 1;           // Shift the register
        if (bit) {
            lfsr ^= polynomial; // Apply feedback based on the taps
        }
    }
}

int main() {
    // Example graph schedule data
    graphSchedule[0].dataSize = 255;  // Example for 8-bit LFSR
    graphSchedule[1].dataSize = 1000; // Example requiring more than 10 bits

    // Generate LFSR for graph index 0
    generate_lfsr_from_schedule(0);

    // Generate LFSR for graph index 1
    generate_lfsr_from_schedule(1);

    return 0;
}
