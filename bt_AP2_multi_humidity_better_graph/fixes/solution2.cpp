#include <stdint.h>
#include <stdbool.h>

// Define a new type for the row access function
typedef float* (*BufferRowAccessFunc)(void* buffer, int row);

// Struct to encapsulate buffer information
typedef struct {
    void* data;
    uint16_t rows;
    uint16_t cols;
    BufferRowAccessFunc accessRowFunc;
} Buffer2D;

// Function to access a row in a 2D array
float* access_2d_array_row(void* buffer, int row) {
    return ((float(*)[])buffer)[row];
}

// Function to access a row in a 1D array representing a 2D array
float* access_1d_array_row(void* buffer, int row) {
    return &((float*)buffer)[row * BUFFER_SIZE];
}

void plotGraphMultiStruct(Buffer2D* buffer,
                          uint16_t graphPosX, uint16_t graphPosY,
                          uint16_t graphWidth, uint16_t graphHeight) {
    for (int j = 0; j < buffer->cols - 1; j++) {
        for (uint8_t i = 0; i < NUMBER_OF_GRAPHS; i++) {
            bool clear_under = (i == 0);
            if (graph_order[i] >= 0 && graph_order[i] < buffer->rows) {
                float* row = buffer->accessRowFunc(buffer->data, graph_order[i]);
                plotGraphSection(row, j, buffer->cols, graphPosX, graphPosY, graphWidth, graphHeight,
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
    Buffer2D buf_2d = {
        .data = buffer_2d,
        .rows = 10,
        .cols = 100,
        .accessRowFunc = access_2d_array_row
    };
    plotGraphMultiStruct(&buf_2d, 10, 10, 100, 100);

    // For 1D array representing 2D data
    Buffer2D buf_1d = {
        .data = buffer_1d,
        .rows = 10,
        .cols = 100,
        .accessRowFunc = access_1d_array_row
    };
    plotGraphMultiStruct(&buf_1d, 10, 10, 100, 100);
}
