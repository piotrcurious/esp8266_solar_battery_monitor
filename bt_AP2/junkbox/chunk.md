Thank you for clarifying. Below is the revised version of the code. This version scales the data points to fit within the specified `graphWidth` and handles the drawing of sections based on the x-dimension of the graph, not based on the number of data points.

### Updated Code:
```cpp
// Function to plot the graph based on graph x dimension
void plotGraph(float *data, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, 
               uint16_t plotStartX, uint16_t plotWidth) {
    
    // Ensure plotWidth does not exceed graph width and is valid
    plotWidth = (plotWidth > graphWidth) ? graphWidth : plotWidth;
    plotStartX = (plotStartX > graphWidth) ? graphWidth : plotStartX;

    uint16_t xStart = graphPosX + plotStartX;
    uint16_t yStart = graphPosY;

    // Clear the plot area in the specified section
    BlueDisplay1.fillRect(xStart, yStart, plotWidth, graphHeight, COLOR_BACKGROUND);

    // Calculate the x-scale based on the graph width and the specified plot width
    float xScale = (float)plotWidth / (float)(graphWidth - 1);
    float yScale = (float)graphHeight / (graph_max - graph_min);

    int lastX = -1;
    int lastY = -1;
    bool lastValid = false;

    color16_t whiteColor = COLOR16_WHITE;
    color16_t grayColor = COLOR16_GRAY;

    // Iterate over the graph width (relative to the data range)
    for (int i = 0; i < plotWidth; i++) {
        // Calculate the corresponding data index
        int dataIndex = (i + plotStartX) * dataSize / graphWidth;
        
        if (dataIndex < dataSize && !isnan(data[dataIndex])) {  // Ensure data index is valid and data is not NaN
            int x = xStart + i;
            int y = yStart + graphHeight - (int)((data[dataIndex] - graph_min) * yScale);

            if (lastValid) {
                BlueDisplay1.drawLine(lastX, lastY, x, y, whiteColor);  // Draw line using white color
            }

            lastX = x;
            lastY = y;
            lastValid = true;
        } else {
            if (lastValid) {
                // Draw a gray line to mark missing data if the last point was valid
                int j = i + 1;
                int nextDataIndex = dataIndex + 1;

                while (j < plotWidth && nextDataIndex < dataSize && isnan(data[nextDataIndex])) {
                    j++;
                    nextDataIndex = (j + plotStartX) * dataSize / graphWidth;
                }

                if (nextDataIndex < dataSize) {
                    int nextValidX = xStart + j;
                    int nextValidY = yStart + graphHeight - (int)((data[nextDataIndex] - graph_min) * yScale);

                    BlueDisplay1.drawLine(lastX, lastY, nextValidX, nextValidY, grayColor);  // Use gray color for missing data indicator
                }
            }
            lastValid = false;
        }
    }

    // Draw graph boundaries
    sprintf(sStringBuffer, "%f", graph_min);    
    BlueDisplay1.drawText(xStart + plotWidth - 16 * 8, yStart + graphHeight - 8, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
    BlueDisplay1.drawLine(xStart, yStart + graphHeight - 1, xStart + plotWidth, yStart + graphHeight - 1, COLOR16_RED);  // bottom boundary

    sprintf(sStringBuffer, "%f", graph_max);    
    BlueDisplay1.drawText(xStart + plotWidth - 16 * 8, yStart, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
    BlueDisplay1.drawLine(xStart, yStart, xStart + plotWidth, yStart, COLOR16_RED);  // top boundary

    int finalDataIndex = (plotWidth + plotStartX) * dataSize / graphWidth - 1;
    sprintf(sStringBuffer, "%f", data[finalDataIndex]);  // Get last data point in the plotted section
    int nextValidY = yStart + graphHeight - (int)((data[finalDataIndex] - graph_min) * yScale);
    BlueDisplay1.drawText(xStart + plotWidth - 16 * 8, nextValidY, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
    BlueDisplay1.drawLine(xStart, nextValidY, xStart + plotWidth, nextValidY, COLOR16_LIGHT_GREY);  // horizontal line at last data point
}
```

### Key Changes:

1. **Plotting Relative to Graph X Dimension:**
   - The `plotStartX` parameter specifies where the plotting should begin within the graph's x-dimension.
   - The `plotWidth` parameter defines how much of the graph (in the x-dimension) should be drawn.

2. **Data Index Mapping:**
   - The data points are mapped to the graph width using a calculated data index based on the `plotStartX` and `plotWidth`.
   - This allows for dynamic plotting regardless of the number of data points.

3. **Boundary Handling:**
   - The code correctly clears the specified section of the graph and ensures lines are drawn across boundaries without gaps.

4. **Efficient Sectional Drawing:**
   - The code draws only the specified section of the graph, allowing for discrete chunk plotting and updating.

### Example Usage:
```cpp
// Plot the first half of the graph
plotGraph(data, dataSize, 0, 0, 320, 240, minVal, maxVal, 0, 160);

// Plot the second half of the graph
plotGraph(data, dataSize, 0, 0, 320, 240, minVal, maxVal, 160, 160);
```

This updated version ensures the graph is plotted relative to the x-dimension of the graph area, providing flexibility in visualizing any segment of the data.
