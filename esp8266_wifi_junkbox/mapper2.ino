/*
 * ==================================================================
 * ESP8266 D1 Mini Board with Dynamic Pin Layout and Device Mappings
 * ==================================================================
 *
 * Leverages macro-based reflection to define pin mappings, generate 
 * reverse mappings, and produce dynamic ASCII art of the physical 
 * pin layout.
 *
 * NOTE: ASCII art is visible in the compiler output.
 */

/* ==================================================================
 * Central Pin Mapping Definitions
 * ==================================================================
 */

// Structured macro for pin-to-device mappings
#define PIN_MAPPINGS \
    X(PIN_3V3,   -1, "3V3 ")          \
    X(PIN_D0,    16, "FREE")          \
    X(PIN_D1,     5, "I2C_SCL")       \
    X(PIN_D2,     4, "EPAPER_BUSY")   \
    X(PIN_D3,     0, "EPAPER_DC")     \
    X(PIN_D4,     2, "EPAPER_RESET")  \
    X(PIN_D5,    14, "EPAPER_SCLK")   \
    X(PIN_GND,   -1, "GND ")          \
    X(PIN_RST,   -1, "RST ")          \
    X(PIN_A0,    A0, "A0  ")          \
    X(PIN_G,     -1, "G   ")          \
    X(PIN_5V,    -1, "5V  ")          \
    X(PIN_D8,    15, "EPAPER_CS")     \
    X(PIN_D7,    13, "EPAPER_MOSI")   \
    X(PIN_D6,    12, "EPAPER_MISO")   \
    X(PIN_3V3B,  -1, "3V3 ")

/* ==================================================================
 * Generate Pin Definitions
 * ==================================================================
 */

// Generate pin definitions (physical GPIO values)
#define X(NAME, GPIO, LABEL) const int NAME = GPIO;
PIN_MAPPINGS
#undef X

/* ==================================================================
 * Generate Reverse Reflection Table
 * ==================================================================
 */

const char *pin_labels[] = {
#define X(NAME, GPIO, LABEL) [NAME] = LABEL,
    PIN_MAPPINGS
#undef X
};

/* ==================================================================
 * Dynamic ASCII Art Generation
 * ==================================================================
 */

#pragma message("  +----------------------------------+")
#pragma message("  | Left Column         Right Column |")
#pragma message("  +----------------------------------+")

// Physical pin layout
#define ROWS 8
const int left_pins[ROWS] = {PIN_3V3, PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_GND};
const int right_pins[ROWS] = {PIN_RST, PIN_A0, PIN_G, PIN_5V, PIN_D8, PIN_D7, PIN_D6, PIN_3V3B};

// Helper macro to fetch pin labels
#define GET_LABEL(pin) (pin_labels[pin] ? pin_labels[pin] : "N/A")

// Generate rows dynamically
#define PRINT_ROW(left_pin, right_pin)                                     \
    #pragma message("  | " GET_LABEL(left_pin) "     " GET_LABEL(right_pin) " |")

// Print each row
PRINT_ROW(PIN_3V3, PIN_RST)
PRINT_ROW(PIN_D0, PIN_A0)
PRINT_ROW(PIN_D1, PIN_G)
PRINT_ROW(PIN_D2, PIN_5V)
PRINT_ROW(PIN_D3, PIN_D8)
PRINT_ROW(PIN_D4, PIN_D7)
PRINT_ROW(PIN_D5, PIN_D6)
PRINT_ROW(PIN_GND, PIN_3V3B)
#pragma message("  +----------------------------------+")

/* ==================================================================
 * Setup and Loop
 * ==================================================================
 */

void setup() {
  Serial.begin(115200);
  Serial.println("ESP8266 Pin Layout with Dynamic Mappings");
}

void loop() {
  // Placeholder for main logic
}
