
// Adafruit OLED display library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Display dimensions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Define the variables to display
// Each variable has a name, a label, a position, a font size, an update rate, and a display mode
// The display modes are: 0 for hex, 1 for dec, 2 for bar, 3 for rolling graph, 4 for analog gauge
// The parameters for each mode are: min, max, color, width, height, radius, etc.
struct Variable {
  String name; // The name of the variable
  String label; // The label to show on the display
  int x; // The x position of the label
  int y; // The y position of the label
  int font; // The font size of the label
  int rate; // The update rate of the variable in milliseconds
  int mode; // The display mode of the variable
  int params[10]; // The parameters for the display mode
};

// Define the list of variables to display
// You can add or remove variables as you wish
Variable variables[] = {
  // Variable 1: temperature in hex
  {
    "temp", // The name of the variable
    "Temp:", // The label to show on the display
    0, // The x position of the label
    0, // The y position of the label
    1, // The font size of the label
    1000, // The update rate of the variable in milliseconds
    0, // The display mode of the variable (0 for hex)
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // The parameters for the display mode (not used for hex)
  },
  // Variable 2: humidity in dec
  {
    "hum", // The name of the variable
    "Hum:", // The label to show on the display
    64, // The x position of the label
    0, // The y position of the label
    1, // The font size of the label
    1000, // The update rate of the variable in milliseconds
    1, // The display mode of the variable (1 for dec)
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // The parameters for the display mode (not used for dec)
  },
  // Variable 3: light in bar
  {
    "light", // The name of the variable
    "Light:", // The label to show on the display
    0, // The x position of the label
    16, // The y position of the label
    1, // The font size of the label
    500, // The update rate of the variable in milliseconds
    2, // The display mode of the variable (2 for bar)
    {
      0, // The minimum value of the variable
      1023, // The maximum value of the variable
      WHITE, // The bar color
      64, // The bar width
      16, // The bar height
      0, 0, 0, 0, 0 // The unused parameters for the display mode
    }
  },
  // Variable 4: sound in rolling graph
  {
    "sound", // The name of the variable
    "Sound:", // The label to show on the display
    64, // The x position of the label
    16, // The y position of the label
    1, // The font size of the label
    100, // The update rate of the variable in milliseconds
    3, // The display mode of the variable (3 for rolling graph)
    {
      0, // The minimum value of the variable
      1023, // The maximum value of the variable
      YELLOW, // The graph color
      64, // The graph width
      32, // The graph height
      0, 0, 0, 0, 0 // The unused parameters for the display mode
    }
  },
  // Variable 5: pressure in analog gauge
  {
    "press", // The name of the variable
    "Press:", // The label to show on the display
    0, // The x position of the label
    32, // The y position of the label
    1, // The font size of the label
    500, // The update rate of the variable in milliseconds
    4, // The display mode of the variable (4 for analog gauge)
    {
      0, // The minimum value of the variable
      100, // The maximum value of the variable
      GREEN, // The gauge color
      32, // The gauge width
      32, // The gauge height
      16, // The gauge radius
      0, 0, 0, 0 // The unused parameters for the display mode
    }
  }
};

// Define the number of variables
#define NUM_VARS (sizeof(variables) / sizeof(variables[0]))

// Define the buffer size for the rolling graph
#define BUFFER_SIZE 64

// Define the buffer for the rolling graph
int buffer[BUFFER_SIZE];

// Define the buffer index for the rolling graph
int bufferIndex = 0;

// Define the last update time for each variable
unsigned long lastUpdate[NUM_VARS];

// Define the current value for each variable
int currentValue[NUM_VARS];

// Initialize the display
void initDisplay() {
  // Initialize with I2C address 0x3C
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;); // Don't proceed, loop forever
  }

  // Clear the display buffer
  display.clearDisplay();

  // Set the display rotation
  display.setRotation(0);

  // Set the display contrast
  display.setContrast(255);

  // Display the buffer on the screen
  display.display();
}

// Update the display
void updateDisplay() {
  // Loop through the variables
  for (int i = 0; i < NUM_VARS; i++) {
    // Get the current variable
    Variable var = variables[i];

    // Check if the variable needs to be updated
    if (millis() - lastUpdate[i] >= var.rate) {
      // Update the last update time
      lastUpdate[i] = millis();

      // Update the current value
      currentValue[i] = readVariable(var.name);

      // Clear the area of the variable
      clearArea(var);
    }

    // Display the label
    display.setTextSize(var.font);
    display.setTextColor(WHITE);
    display.setCursor(var.x, var.y);
    display.print(var.label);

    // Display the value according to the mode
    switch (var.mode) {
      case 0: // Hex
        display.print("0x");
        display.print(currentValue[i], HEX);
        break;
      case 1: // Dec
        display.print(currentValue[i], DEC);
        break;
      case 2: // Bar
        displayBar(var, currentValue[i]);
        break;
      case 3: // Rolling graph
        displayGraph(var, currentValue[i]);
        break;
      case 4: // Analog gauge
        displayGauge(var, currentValue[i]);
        break;
      default: // Invalid mode
        display.print("?");
        break;
    }
  }

  // Display the buffer on the screen
  display.display();
}

// Read a variable from a sensor or a pin
int readVariable(String name) {
  // TODO: Replace this with your own code to read the variable from a sensor or a pin
  // For example, you can use analogRead(), digitalRead(), or a library function
  // For now, we just return a random value
  return random(0, 1024);
}

// Clear the area of a variable
void clearArea(Variable var) {
  // Get the parameters
  int min = var.params[0]; // The minimum value
  int max = var.params[1]; // The maximum value
  int color = var.params[2]; // The color
  int width = var.params[3]; // The width
  int height = var.params[4]; // The height
  int radius = var.params[5]; // The radius

  // Calculate the area to clear
  int x1 = var.x;
  int y1 = var.y;
  int x2 = var.x + width;
  int y2 = var.y + height;

  // Clear the area
  display.fillRect(x1, y1, x2 - x1, y2 - y1, BLACK);
}

// Display a bar
void displayBar(Variable var, int value) {
  // Get the parameters
  int min = var.params[0]; // The minimum value
  int max = var.params[1]; // The maximum value
  int color = var.params[2]; // The bar color
  int width = var.params[3]; // The bar width
  int height = var.params[4]; // The bar height

  // Map the value to the bar range
  int bar = map(value, min, max, 0, width);

  // Draw the bar
  display.fillRect(var.x, var.y, bar, height, color);
}

// Display a rolling graph
void displayGraph(Variable var, int value) {
  // Get the parameters
  int min = var.params[0]; // The minimum value
  int max = var.params[1]; // The maximum value
  int color = var.params[2]; // The graph color
  int width = var.params[3]; // The graph width
  int height = var.params[4]; // The graph height

  // Map the value to the graph range
  int graph = map(value, min, max, 0, height);

  // Shift the buffer to the left
  for (int i = 0; i < BUFFER_SIZE - 1; i++) {
    buffer[i] = buffer[i + 1];
  }

  // Add the new value to the buffer
  buffer[BUFFER_SIZE - 1] = graph;

  // Increment the buffer index
  bufferIndex++;

  // Wrap the buffer index
  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
  }

  // Draw the graph
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int x = var.x + i;
    int y = var.y + height - buffer[i];
    display.drawPixel(x, y, color);
  }
}

// Display an analog gauge
void displayGauge(Variable var, int value) {
  // Get the parameters
  int min = var.params[0]; // The minimum value
  int max = var.params[1]; // The maximum value
  int color = var.params[2]; // The gauge color
  int width = var.params[3]; // The gauge width
  int height = var.params[4]; // The gauge height
  int radius = var.params[5]; // The gauge radius

  // Map the value to the gauge range
  int gauge = map(value, min, max, -90, 90);

  // Calculate the gauge angle in radians
  float angle = gauge * PI / 180.0;

  // Calculate the gauge end point
  int x = var.x + width / 2 + radius * cos(angle);
  int y = var.y + height / 2 + radius * sin(angle);

  // Draw the gauge
  display.drawLine(var.x + width / 2, var.y + height / 2, x, y, color);
}

// Setup function
void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize the display
  initDisplay();

  // Initialize the random seed
  randomSeed(analogRead(0));
}

// Loop function
void loop() {
  // Update the display
  updateDisplay();
}
