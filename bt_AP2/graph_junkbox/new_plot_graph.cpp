#include <Adafruit_GFX.h>        // Include necessary libraries based on your display
#include <Adafruit_ILI9341.h>    // Example for a TFT display, change as per your display

// Assuming display object, modify as per your setup
extern Adafruit_ILI9341 tft;    // External TFT object, replace with your display object

// Define your data array globally or ensure it's accessible
extern float data_array[][/* graph_data size */]; // Define size according to your data structure

// Function to plot a single segment of the graph
void plotGraph(float *data, uint16_t data_startpoint, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, uint8_t graph_number) {
    
    // Determine the number of graphs and the number of data points in each graph
    uint16_t number_of_graphs = sizeof(data_array) / sizeof(data_array[0]);
    uint16_t graph_data_size = sizeof(data_array[0]) / sizeof(data_array[0][0]);

    // Ensure the graph_number is within bounds
    if (graph_number >= number_of_graphs) {
        return;  // Invalid graph number, nothing to plot
    }

    // Ensure the data points to plot are within bounds
    if (data_startpoint >= graph_data_size - 1) {
        return;  // Out of bounds, nothing to plot
    }

    // Get the current and next data points from the correct graph
    float currentValue = data_array[graph_number][data_startpoint];
    float nextValue = data_array[graph_number][data_startpoint + 1];

    // Skip plotting if the current value is NaN
    if (isnan(currentValue)) {
        return;  // Nothing to plot if the current point is NaN
    }

    // Calculate scaling factor for converting data to graph coordinates
    float scale = (graph_max - graph_min) / (float)graphHeight;

    // Calculate x and y positions of the current point
    uint16_t x0 = graphPosX + (data_startpoint * graphWidth) / graph_data_size;
    uint16_t y0 = graphPosY + graphHeight - (uint16_t)((currentValue - graph_min) / scale);

    // Clear the rectangle under the segment if graph_number is 0
    if (graph_number == 0) {
        tft.fillRect(x0, graphPosY, graphWidth / graph_data_size, graphHeight, ILI9341_BLACK);  // Clear only the segment area
    }

    // If the next value is NaN, draw a dot at the current point
    if (isnan(nextValue)) {
        tft.drawPixel(x0, y0, ILI9341_WHITE);  // Draw a dot
    } else {
        // Calculate x and y positions of the next point
        uint16_t x1 = graphPosX + ((data_startpoint + 1) * graphWidth) / graph_data_size;
        uint16_t y1 = graphPosY + graphHeight - (uint16_t)((nextValue - graph_min) / scale);

        // Draw a line between the current point and the next point
        tft.drawLine(x0, y0, x1, y1, ILI9341_WHITE);  // Draw a line
    }
}
