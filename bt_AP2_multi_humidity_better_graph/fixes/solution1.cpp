#include <stdint.h>
#include <stdbool.h>

// Define a new type for the row access function
typedef float* (*BufferRowAccessFunc)(void* buffer, int row);

// Function to access a row in a 2D array
float* access_2d_array_row(void* buffer, int row) {
    return ((float(*)[])buffer)[row];
}

// Function to access a row in a 1D array representing a 2D array
float* access_1d_array_row(void* buffer, int row) {
    return &((float*)buffer)[row * BUFFER_SIZE];
}

void plotGraphMulti(void* minutes_buffer, BufferRowAccessFunc accessRowFunc, 
                    uint16_t bufferNumbers, uint16_t dataSize,
                    uint16_t graphPosX, uint16_t graphPosY,
                    uint16_t graphWidth, uint16_t graphHeight) {
    for (int j = 0; j < dataSize - 1; j++) {
        for (uint8_t i = 0; i < NUMBER_OF_GRAPHS; i++) {
            bool clear_under = (i == 0);
            if (graph_order[i] >= 0 && graph_order[i] < bufferNumbers) {
                float* row = accessRowFunc(minutes_buffer, graph_order[i]);
                plotGraphSection(row, j, dataSize, graphPosX, graphPosY, graphWidth, graphHeight,
                    minutes_buffer_min[graph_order[i]], minutes_buffer_max[graph_order[i]],
                    GRAPH_COLOR[graph_order[i]], clear_under);
            }
        }
    }
    graphComplete = true;
}

// Updated plotGraphSection function signature
void plotGraphSection(float* buffer_row, int j, uint16_t dataSize,
                      uint16_t graphPosX, uint16_t graphPosY,
                      uint16_t graphWidth, uint16_t graphHeight,
                      float min, float max, uint16_t color, bool clear_under) {
    // Implementation here
    // You can access buffer_row[j], buffer_row[j+1], etc.
}

// Usage example
void example_usage() {
    float buffer_2d[10][100];
    float buffer_1d[10 * 100];

    // For 2D array
    plotGraphMulti(buffer_2d, access_2d_array_row, 10, 100, 10, 10, 100, 100);

    // For 1D array representing 2D data
    plotGraphMulti(buffer_1d, access_1d_array_row, 10, 100, 10, 10, 100, 100);
}
