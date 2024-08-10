#include <BlueDisplay.hpp>  // Include the BlueDisplay library

#define GRAPH_WIDTH  128   // Graph width in pixels (adjust as needed)
#define GRAPH_HEIGHT 64    // Graph height in pixels (adjust as needed)

float dataArray[] = {0.1, 0.3, NAN, 0.7, 0.8, 0.9, NAN, 1.2, 1.3, 1.4, 1.5, 1.7, 2.0};  // Example array of float data
const int dataArraySize = sizeof(dataArray) / sizeof(dataArray[0]);

float graph_min = 0.0;  // Minimum Y value for the graph
float graph_max = 2.0;  // Maximum Y value for the graph

BlueDisplay BlueDisplay1;  // Create an instance of BlueDisplay named BlueDisplay1

void setup() {
  BlueDisplay1.begin();  // Initialize BlueDisplay
  BlueDisplay1.clearDisplay();  // Clear the screen
}

void loop() {
  plotGraph(dataArray, dataArraySize, GRAPH_WIDTH, GRAPH_HEIGHT, graph_min, graph_max);
  delay(5000);  // Wait 5 seconds before plotting again (for demonstration purposes)
}

// Function to plot the graph
void plotGraph(float *data, int dataSize, int graphWidth, int graphHeight, float graph_min, float graph_max) {
  int xStart = 0;
  int yStart = 0;

  float xScale = (float)graphWidth / (float)(dataSize - 1);
  float yScale = (float)graphHeight / (graph_max - graph_min);

  int lastX = -1;
  int lastY = -1;
  bool lastValid = false;

  color16_t whiteColor = RGB16_WHITE;
  color16_t grayColor = RGB16_GRAY;

  for (int i = 0; i < dataSize; i++) {
    if (!isnan(data[i])) {  // Check if the current data point is valid
      int x = xStart + (int)(i * xScale);
      int y = yStart + graphHeight - (int)((data[i] - graph_min) * yScale);

      if (lastValid) {
        BlueDisplay1.drawLine(lastX, lastY, x, y, whiteColor);  // Draw a line between the last valid point and the current point using white color
      }

      lastX = x;
      lastY = y;
      lastValid = true;
    } else {
      if (lastValid) {
        // Draw a gray line to mark missing data if the last point was valid
        int j = i + 1;
        while (j < dataSize && isnan(data[j])) {
          j++;
        }

        if (j < dataSize) {
          int nextValidX = xStart + (int)(j * xScale);
          int nextValidY = yStart + graphHeight - (int)((data[j] - graph_min) * yScale);
          BlueDisplay1.drawLine(lastX, lastY, nextValidX, nextValidY, grayColor);  // Use gray color for missing data indicator
        }
      }
      lastValid = false;
    }
  }
}
