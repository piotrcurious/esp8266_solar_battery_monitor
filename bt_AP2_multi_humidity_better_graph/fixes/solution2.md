 Certainly! Let's explore some more C-style solutions that can provide flexibility and ease of use without relying on C++ features. These solutions will focus on using macros, function pointers, and structs to create more elegant C-style approaches.



<antArtifact identifier="c-style-plot-graph-multi" type="application/vnd.ant.code" language="c" title="C-style plotGraphMulti solutions">
#include <stdint.h>
#include <stdbool.h>

// Solution 1: Using function pointers for flexible data access

typedef float (*BufferAccessFunc)(void* buffer, int row, int col);

float access_2d_array(void* buffer, int row, int col) {
    return ((float(*)[])buffer)[row][col];
}

float access_1d_array(void* buffer, int row, int col) {
    return ((float*)buffer)[row * BUFFER_SIZE + col];
}

void plotGraphMulti(void* minutes_buffer, BufferAccessFunc accessFunc, 
                    uint16_t bufferNumbers, uint16_t dataSize,
                    uint16_t graphPosX, uint16_t graphPosY,
                    uint16_t graphWidth, uint16_t graphHeight) {
    for (int j = 0; j < dataSize - 1; j++) {
        for (uint8_t i = 0; i < NUMBER_OF_GRAPHS; i++) {
            bool clear_under = (i == 0);
            if (graph_order[i] >= 0 && graph_order[i] < bufferNumbers) {
                float value = accessFunc(minutes_buffer, graph_order[i], j);
                plotGraphSection(value, j, dataSize, graphPosX, graphPosY, graphWidth, graphHeight,
                    minutes_buffer_min[graph_order[i]], minutes_buffer_max[graph_order[i]],
                    GRAPH_COLOR[graph_order[i]], clear_under);
            }
        }
    }
    graphComplete = true;
}

// Solution 2: Using a struct to encapsulate buffer information

typedef struct {
    void* data;
    uint16_t rows;
    uint16_t cols;
    BufferAccessFunc accessFunc;
} Buffer2D;

void plotGraphMultiStruct(Buffer2D* buffer,
                          uint16_t graphPosX, uint16_t graphPosY,
                          uint16_t graphWidth, uint16_t graphHeight) {
    for (int j = 0; j < buffer->cols - 1; j++) {
        for (uint8_t i = 0; i < NUMBER_OF_GRAPHS; i++) {
            bool clear_under = (i == 0);
            if (graph_order[i] >= 0 && graph_order[i] < buffer->rows) {
                float value = buffer->accessFunc(buffer->data, graph_order[i], j);
                plotGraphSection(value, j, buffer->cols, graphPosX, graphPosY, graphWidth, graphHeight,
                    minutes_buffer_min[graph_order[i]], minutes_buffer_max[graph_order[i]],
                    GRAPH_COLOR[graph_order[i]], clear_under);
            }
        }
    }
    graphComplete = true;
}

// Solution 3: Using macros for inline expansion and type-checking

#define PLOT_GRAPH_MULTI(buffer, bufferNumbers, dataSize, graphPosX, graphPosY, graphWidth, graphHeight) do { \
    _Static_assert(sizeof(buffer[0]) / sizeof(buffer[0][0]) == dataSize, "Buffer size mismatch"); \
    for (int j = 0; j < dataSize - 1; j++) { \
        for (uint8_t i = 0; i < NUMBER_OF_GRAPHS; i++) { \
            bool clear_under = (i == 0); \
            if (graph_order[i] >= 0 && graph_order[i] < bufferNumbers) { \
                plotGraphSection(buffer[graph_order[i]][j], j, dataSize, graphPosX, graphPosY, graphWidth, graphHeight, \
                    minutes_buffer_min[graph_order[i]], minutes_buffer_max[graph_order[i]], \
                    GRAPH_COLOR[graph_order[i]], clear_under); \
            } \
        } \
    } \
    graphComplete = true; \
} while(0)

// Usage examples
void example_usage() {
    float buffer_2d[10][100];
    float buffer_1d[10 * 100];

    // Solution 1 usage
    plotGraphMulti(buffer_2d, access_2d_array, 10, 100, 10, 10, 100, 100);
    plotGraphMulti(buffer_1d, access_1d_array, 10, 100, 10, 10, 100, 100);

    // Solution 2 usage
    Buffer2D buf_struct = {buffer_2d, 10, 100, access_2d_array};
    plotGraphMultiStruct(&buf_struct, 10, 10, 100, 100);

    // Solution 3 usage
    PLOT_GRAPH_MULTI(buffer_2d, 10, 100, 10, 10, 100, 100);
}
</antArtifact>

Let me explain each of these C-style solutions:

1. Using function pointers for flexible data access:
   - This solution uses a function pointer to abstract the way data is accessed from the buffer.
   - It allows you to use the same plotting function for different buffer layouts (2D arrays, 1D arrays representing 2D data, etc.).
   - You provide an appropriate access function depending on your buffer type.

2. Using a struct to encapsulate buffer information:
   - This solution encapsulates all buffer information (data pointer, dimensions, access function) in a struct.
   - It provides a clean interface and allows for easy extension (e.g., adding more buffer properties).
   - The plotting function takes a pointer to this struct, making it easy to pass around.

3. Using macros for inline expansion and type-checking:
   - This solution uses a macro to inline the entire plotting logic.
   - It includes a compile-time assertion to check if the buffer dimensions match the provided sizes.
   - This approach can lead to faster code as it allows for more compiler optimizations.

Each of these solutions has its strengths:
- Solution 1 is flexible and allows for different buffer layouts without changing the plotting function.
- Solution 2 provides a clean, encapsulated interface that's easy to extend.
- Solution 3 offers potential performance benefits and compile-time checks.

The choice between these depends on your specific needs:
- If you need to support multiple buffer layouts, go with Solution 1.
- If you want a clean, extensible interface, choose Solution 2.
- If performance is critical and you want compile-time checks, use Solution 3.

These solutions stick to C-style programming while still providing flexibility and ease of use. They avoid C++ features like templates or std::vector, making them suitable for C codebases or embedded systems where C++ might not be available.

Would you like me to explain any of these solutions in more detail or discuss how to integrate them into your existing code?
