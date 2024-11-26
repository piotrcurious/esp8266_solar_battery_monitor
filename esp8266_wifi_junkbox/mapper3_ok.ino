/*
 * ESP8266 D1 Mini Pin Layout with Preprocessor Reflection
 * 
 * This code generates ASCII art dynamically at compile time using
 * preprocessor macros to handle reverse mappings.
 */

/* ==================================================================
 * Central Pin Mapping Definitions
 * ==================================================================
 */

#define PIN_3V3    -1
#define PIN_D0     16
#define PIN_D1      5
#define PIN_D2      4
#define PIN_D3      0
#define PIN_D4      2
#define PIN_D5     14
#define PIN_GND    -1
#define PIN_RST    -1
#define PIN_A0     A0
#define PIN_G      -1
#define PIN_5V     -1
#define PIN_D8     15
#define PIN_D7     13
#define PIN_D6     12
#define PIN_3V3B   -1

// Device mappings
#define EPAPER_CS       PIN_D8
#define EPAPER_DC       PIN_D3
#define EPAPER_RESET    PIN_D4
#define EPAPER_BUSY     PIN_D2
#define EPAPER_SCLK     PIN_D5
#define EPAPER_MOSI     PIN_D7
#define EPAPER_MISO     PIN_D6

// Reverse mappings
#define PIN_D8_MAP      "EPAPER_CS   "
#define PIN_D3_MAP      "EPAPER_DC   "
#define PIN_D4_MAP      "EPAPER_RESET"
#define PIN_D2_MAP      "EPAPER_BUSY "
#define PIN_D5_MAP      "EPAPER_SCLK "
#define PIN_D7_MAP      "EPAPER_MOSI "
#define PIN_D6_MAP      "EPAPER_MISO "
#define PIN_D0_MAP      "FREE        "
#define PIN_D1_MAP      "I2C_SCL     "
#define PIN_A0_MAP      "A0          "
#define PIN_3V3_MAP     "3V3         "
#define PIN_GND_MAP     "GND         "
#define PIN_RST_MAP     "RST         "
#define PIN_G_MAP       "G           "
#define PIN_5V_MAP      "5V          "
#define PIN_3V3B_MAP    "3V3         "

/* ==================================================================
 * ASCII Art Generation
 * ==================================================================
 */

#pragma message("  +----------------------------------+")
#pragma message("  | Left Column         Right Column |")
#pragma message("  +----------------------------------+")
#pragma message("  | " PIN_3V3_MAP "   " PIN_RST_MAP  " |")
#pragma message("  | " PIN_D0_MAP  "   " PIN_A0_MAP   " |")
#pragma message("  | " PIN_D1_MAP  "   " PIN_G_MAP    " |")
#pragma message("  | " PIN_D2_MAP  "   " PIN_5V_MAP   " |")
#pragma message("  | " PIN_D3_MAP  "   " PIN_D8_MAP   " |")
#pragma message("  | " PIN_D4_MAP  "   " PIN_D7_MAP   " |")
#pragma message("  | " PIN_D5_MAP  "   " PIN_D6_MAP   " |")
#pragma message("  | " PIN_GND_MAP "   " PIN_3V3B_MAP " |")
#pragma message("  +----------------------------------+")

/* ==================================================================
 * Setup and Loop
 * ==================================================================
 */

void setup() {
  Serial.begin(115200);
  // Main setup logic
}

void loop() {
  // Main loop logic
}
