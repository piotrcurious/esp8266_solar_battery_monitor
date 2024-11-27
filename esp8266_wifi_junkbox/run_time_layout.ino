/*
 * ESP8266 D1 Mini Board with Dynamic Pin Layout and Device Mappings
 */

#include <Arduino.h>

/* ==================================================================
 * Central Pin Mapping Definitions
 * ==================================================================
 */

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

#define X(NAME, GPIO, LABEL) const int NAME = GPIO;
PIN_MAPPINGS
#undef X

// Pin labels for runtime reflection
const char *pin_labels[] = {
#define X(NAME, GPIO, LABEL) [NAME] = LABEL,
    PIN_MAPPINGS
#undef X
};

/* ==================================================================
 * ASCII Art Generation at Runtime
 * ==================================================================
 */

const int ROWS = 8;

// Physical layout of pins
const int left_pins[ROWS] = {PIN_3V3, PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_GND};
const int right_pins[ROWS] = {PIN_RST, PIN_A0, PIN_G, PIN_5V, PIN_D8, PIN_D7, PIN_D6, PIN_3V3B};

// Helper function to get label
const char *getLabel(int pin) {
  return (pin >= 0 && pin_labels[pin] != nullptr) ? pin_labels[pin] : "N/A ";
}

// Generate ASCII art at runtime
void printAsciiArt() {
  Serial.println("  +----------------------------------+");
  Serial.println("  | Left Column         Right Column |");
  Serial.println("  +----------------------------------+");

  for (int i = 0; i < ROWS; i++) {
    char left_label[10];
    char right_label[10];

    snprintf(left_label, sizeof(left_label), "%s", getLabel(left_pins[i]));
    snprintf(right_label, sizeof(right_label), "%s", getLabel(right_pins[i]));

    Serial.printf("  | %-16s %-16s |\n", left_label, right_label);
  }

  Serial.println("  +----------------------------------+");
}

/* ==================================================================
 * Setup and Loop
 * ==================================================================
 */

void setup() {
  Serial.begin(115200);
  delay(1000);
  printAsciiArt();
}

void loop() {
  // Main logic here
}
