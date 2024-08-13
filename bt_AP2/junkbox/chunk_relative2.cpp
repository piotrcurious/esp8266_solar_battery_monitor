// Function to plot the graph based on graph x dimension, with labels only at the end
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

    // Only draw labels if the plotted section includes the end of the data
    int finalPlottedIndex = (plotStartX + plotWidth) * dataSize / graphWidth - 1;
    if (finalPlottedIndex >= dataSize - 1) {
        // Draw graph boundaries and text labels
        sprintf(sStringBuffer, "%f", graph_min);    
        BlueDisplay1.drawText(xStart + plotWidth - 16 * 8, yStart + graphHeight - 8, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
        BlueDisplay1.drawLine(xStart, yStart + graphHeight - 1, xStart + plotWidth, yStart + graphHeight - 1, COLOR16_RED);  // bottom boundary

        sprintf(sStringBuffer, "%f", graph_max);    
        BlueDisplay1.drawText(xStart + plotWidth - 16 * 8, yStart, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
        BlueDisplay1.drawLine(xStart, yStart, xStart + plotWidth, yStart, COLOR16_RED);  // top boundary

        // Label for the last data point
        int finalDataIndex = dataSize - 1;
        sprintf(sStringBuffer, "%f", data[finalDataIndex]);
        int nextValidY = yStart + graphHeight - (int)((data[finalDataIndex] - graph_min) * yScale);
        BlueDisplay1.drawText(xStart + plotWidth - 16 * 8, nextValidY, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
        BlueDisplay1.drawLine(xStart, nextValidY, xStart + plotWidth, nextValidY, COLOR16_LIGHT_GREY);  // horizontal line at last data point
    }
}
