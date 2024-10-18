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

// Function to generate maximal length LFSR sequence
void generate_lfsr(uint16_t bit_length) {
    if (bit_length < 1 || bit_length > 16) {
        printf("Invalid bit length. Please choose a value between 1 and 16.\n");
        return;
    }

    uint16_t max_steps = (1 << bit_length) - 1; // Max sequence length for n-bit LFSR
    uint16_t lfsr = 1; // Initial state (must not be 0)
    uint16_t polynomial = taps[bit_length]; // Select tap configuration based on bit length
    uint16_t bit;

    printf("LFSR sequence for %d-bit:\n", bit_length);
    
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
    uint16_t bit_length = 8; // Example for 8-bit LFSR
    generate_lfsr(bit_length);
    return 0;
}
