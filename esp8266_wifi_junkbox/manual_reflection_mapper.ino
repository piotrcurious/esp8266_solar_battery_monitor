/*
 * ==================================================================
 * ESP8266 D1 Mini Board with Dynamic Pin Layout and Device Mappings
 * ==================================================================
 *
 * Dynamically generates an ASCII art representation of the 
 * D1 Mini board's pin layout using a reverse mapping mechanism 
 * (_MAP defines) for clarity and maintainability.
 *
 * NOTE: The ASCII art is visible in the compiler output.
 */

/* ==================================================================
 * Pin Assignment Definitions (Generic ESP8266 GPIO Pins)
 * ==================================================================
 */

// Physical pin definitions
#define PIN_3V3   -1   // 3.3V Power
#define PIN_D0    16   // GPIO16
#define PIN_D1    5    // GPIO5
#define PIN_D2    4    // GPIO4
#define PIN_D3    0    // GPIO0
#define PIN_D4    2    // GPIO2
#define PIN_D5    14   // GPIO14
#define PIN_GND   -1   // Ground
#define PIN_RST   -1   // Reset
#define PIN_A0    A0   // Analog input
#define PIN_G     -1   // Ground
#define PIN_5V    -1   // 5V Power
#define PIN_D8    15   // GPIO15
#define PIN_D7    13   // GPIO13
#define PIN_D6    12   // GPIO12
#define PIN_3V3B  -1   // 3.3V Power (duplicate)

/* ==================================================================
 * Device Pin Mapping Examples
 * ==================================================================
 */

// Assign devices to pins
#define EPAPER_CS     PIN_D8  // Chip Select
#define EPAPER_DC     PIN_D3  // Data/Command
#define EPAPER_RESET  PIN_D4  // Reset
#define EPAPER_BUSY   PIN_D2  // Busy
#define EPAPER_SCLK   PIN_D5  // SPI Clock
#define EPAPER_MOSI   PIN_D7  // SPI MOSI
#define EPAPER_MISO   PIN_D6  // SPI MISO

#define I2C_SCL       PIN_D1  // I2C Clock
#define I2C_SDA       PIN_D2  // I2C Data

#define ONBOARD_LED   PIN_D4  // Onboard LED

/* ==================================================================
 * Reverse Reflection Map (_MAP Defines)
 * ==================================================================
 */

// Simple reflection for easy maintenance
#define PIN_3V3_MAP   "3V3 "
#define PIN_D0_MAP    "FREE"
#define PIN_D1_MAP    "I2C_SCL"
#define PIN_D2_MAP    "EPAPER_BUSY"
#define PIN_D3_MAP    "EPAPER_DC"
#define PIN_D4_MAP    "EPAPER_RESET/LED"
#define PIN_D5_MAP    "EPAPER_SCLK"
#define PIN_D8_MAP    "EPAPER_CS"
#define PIN_D7_MAP    "EPAPER_MOSI"
#define PIN_D6_MAP    "EPAPER_MISO"
#define PIN_RST_MAP   "RST "
#define PIN_A0_MAP    "A0  "
#define PIN_G_MAP     "G   "
#define PIN_5V_MAP    "5V  "
#define PIN_GND_MAP   "GND "
#define PIN_3V3B_MAP  "3V3 "

/* ==================================================================
 * Dynamic ASCII Art Generation
 * ==================================================================
 */

#pragma message("  +----------------------------------+")
#pragma message("  | Left Column         Right Column |")
#pragma message("  +----------------------------------+")

#define ROWS 8
const int left_pins[ROWS] = {PIN_3V3, PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_GND};
const int right_pins[ROWS] = {PIN_RST, PIN_A0, PIN_G, PIN_5V, PIN_D8, PIN_D7, PIN_D6, PIN_3V3B};

// Helper macros to stringify pin mappings
#define MAP_PIN(pin) (pin == -1 ? "N/A" : PIN_##pin##_MAP)

// Generate rows dynamically
#define PRINT_ROW(left_pin, right_pin)                                    \
    #pragma message("  | " MAP_PIN(left_pin) "     " MAP_PIN(right_pin) " |")

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
