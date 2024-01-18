#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define BUFFER_SIZE 128
#define ABS_MIN 0.0 // change this to your absolute minimum value
#define ABS_MAX 10.0 // change this to your absolute maximum value

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float buffer[BUFFER_SIZE]; // circular buffer to store the last 128 values
int index = 0; // index to keep track of the buffer position
float min = ABS_MAX; // minimum value in the buffer
float max = ABS_MIN; // maximum value in the buffer

void setup() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // initialize the display with the I2C address 0x3C
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // don't proceed, loop forever
  }
  display.clearDisplay(); // clear the display
  display.display(); // update the display
}

void loop() {
  float value = readValue(); // read the float value from the sensor or input
  buffer[index] = value; // store the value in the buffer
  index = (index + 1) % BUFFER_SIZE; // increment the index and wrap around if necessary
  updateMinMax(); // update the minimum and maximum values in the buffer
  displayValue(value); // display the value on the OLED screen
  delay(100); // wait for 100 milliseconds
}

float readValue() {
  // this function should return the float value from the sensor or input
  // for testing purposes, we use a random value between ABS_MIN and ABS_MAX
  return random(ABS_MIN * 100, ABS_MAX * 100) / 100.0;
}

void updateMinMax() {
  // this function should update the min and max values based on the buffer contents
  // we use a simple linear search algorithm
  min = ABS_MAX;
  max = ABS_MIN;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (buffer[i] < min) {
      min = buffer[i];
    }
    if (buffer[i] > max) {
      max = buffer[i];
    }
  }
}

void displayValue(float value) {
  // this function should display the value as a vertical arrow pointing at a scale
  // we use the Adafruit GFX library to draw the graphics
  display.clearDisplay(); // clear the display
  display.setTextSize(1); // set the text size to 1
  display.setTextColor(SSD1306_WHITE); // set the text color to white
  display.setCursor(0, 0); // set the cursor position to the top left corner
  display.print(min); // print the minimum value
  display.setCursor(SCREEN_WIDTH - 20, 0); // set the cursor position to the top right corner
  display.print(max); // print the maximum value
  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE); // draw a horizontal line for the scale
  int x = map(value, min, max, 0, SCREEN_WIDTH - 1); // map the value to the x coordinate
  display.drawLine(x, 10, x, SCREEN_HEIGHT - 1, SSD1306_WHITE); // draw a vertical line for the arrow
  display.drawLine(x - 2, 12, x, 10, SSD1306_WHITE); // draw the left part of the arrow head
  display.drawLine(x + 2, 12, x, 10, SSD1306_WHITE); // draw the right part of the arrow head
  display.display(); // update the display
}
