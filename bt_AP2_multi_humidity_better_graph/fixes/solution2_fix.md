 Certainly! I'll modify Solution 2 to meet the requirement of passing a row pointer to the downstream function. This approach will maintain the benefits of using a struct to encapsulate buffer information while allowing the `plotGraphSection` function to access entire rows of data.



<antArtifact identifier="c-style-plot-graph-multi" type="application/vnd.ant.code" language="c" title="Revised C-style plotGraphMulti solution using struct">
 ```
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
```
</antArtifact>

Here's an explanation of the changes and how this solution addresses your needs:

1. Updated Buffer2D Struct:
   - We've added a `BufferRowAccessFunc accessRowFunc` field to the struct.
   - This function pointer allows us to specify how to access rows for different buffer types.

2. Row Access Functions:
   - We've kept the `access_2d_array_row` and `access_1d_array_row` functions from the previous solution.
   - These functions return pointers to the start of a row in their respective buffer types.

3. Updated `plotGraphMultiStruct`:
   - The function now uses `buffer->accessRowFunc` to get a pointer to the entire row of the buffer.
   - It passes this row pointer to `plotGraphSection`, allowing it to access the entire row.

4. Modified `plotGraphSection`:
   - The function signature now takes a `float* buffer_row` instead of a single float value.
   - This allows `plotGraphSection` to perform operations on the entire row as needed.

5. Usage:
   - When creating a `Buffer2D` struct, you specify the appropriate row access function along with the other buffer properties.

This solution maintains the benefits of using a struct to encapsulate buffer information while allowing `plotGraphSection` to access the entire row of the buffer. It works with both 2D arrays and 1D arrays representing 2D data, and you can easily add support for other data layouts by creating new row access functions.

Key benefits:
1. Encapsulation: All buffer information is contained in a single struct.
2. Flexibility: Works with different buffer layouts by specifying the appropriate access function.
3. Performance: Passes pointers, avoiding unnecessary copying.
4. Compatibility: Maintains a C-style approach, suitable for C codebases or embedded systems.

To integrate this into your existing code:
1. Define the `Buffer2D` struct and `BufferRowAccessFunc` type in your header file.
2. Implement the appropriate row access functions for your buffer types.
3. Update your `plotGraphSection` function to accept a row pointer instead of a single value.
4. Create `Buffer2D` instances for your buffers, specifying the appropriate access function.
5. Call `plotGraphMultiStruct` with your `Buffer2D` instances.

Would you like me to explain any part of this solution in more detail or help with integrating it into your specific codebase?
