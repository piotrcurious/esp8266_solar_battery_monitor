 I see. Thank you for clarifying that. You're right - the goal is to have a flexible function that can work with arrays of various sizes, and the `dataSize` parameter is meant to inform about the actual size being used. Let's modify the function to achieve this flexibility.



<antArtifact identifier="plot-graph-multi-function" type="application/vnd.ant.code" language="cpp" title="Flexible plotGraphMulti function">
void plotGraphMulti(float *minutes_buffer, uint16_t bufferNumbers, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
                    uint16_t graphWidth, uint16_t graphHeight) {
    for (int j = 0; j < (dataSize-1); j++) {
        for (uint8_t i = 0; i < NUMBER_OF_GRAPHS; i++) {
            bool clear_under = (i == 0);
            if (graph_order[i] >= 0 && graph_order[i] < bufferNumbers) {
                plotGraphSection(minutes_buffer[graph_order[i] * dataSize + j], j, dataSize, graphPosX, graphPosY,
                    graphWidth, graphHeight, minutes_buffer_min[graph_order[i]], 
                    minutes_buffer_max[graph_order[i]], GRAPH_COLOR[graph_order[i]], clear_under);
            }
        }
    }
    graphComplete = true;
}
</antArtifact>

Here are the key changes and explanations:

1. Function parameter: Changed to `float *minutes_buffer`. This allows passing a pointer to the first element of a contiguous block of memory containing all the data.

2. Added a new parameter `uint16_t bufferNumbers` to specify the number of buffers (i.e., the first dimension of the original 2D array).

3. Array indexing: Changed to `minutes_buffer[graph_order[i] * dataSize + j]`. This calculates the correct offset in the flattened 1D array that represents our 2D data.

4. Added a bounds check `graph_order[i] < bufferNumbers` to ensure we don't access out-of-bounds data.

To use this function, you would call it like this:

```cpp
float my_buffer[BUFFER_NUMBERS][BUFFER_SIZE];
// ... fill my_buffer with data ...
plotGraphMulti(&my_buffer[0][0], BUFFER_NUMBERS, BUFFER_SIZE, graphPosX, graphPosY, graphWidth, graphHeight);
```

Or, if you're using dynamically allocated memory:

```cpp
float *my_buffer = new float[buffer_numbers * data_size];
// ... fill my_buffer with data ...
plotGraphMulti(my_buffer, buffer_numbers, data_size, graphPosX, graphPosY, graphWidth, graphHeight);
```

This solution allows you to use arrays of various sizes, as the function now takes the number of buffers and the size of each buffer as separate arguments. The function treats the input as a flat 1D array internally, but calculates offsets to access it as if it were 2D.

A few things to note:

1. This approach assumes that your data is stored in a contiguous block of memory, which is true for standard C arrays and dynamically allocated arrays.

2. The `plotGraphSection` function will need to be adjusted to work with this new data layout if it previously expected a 2D array.

3. Be careful with the `graph_order` array - make sure its values are always less than `bufferNumbers` to avoid out-of-bounds access.

Would you like me to explain any part of this solution further or help with adapting other parts of your code to work with this new approach?
