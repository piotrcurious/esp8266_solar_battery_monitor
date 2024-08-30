void plotGraph_buffered(float *data, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max, color16_t graphColor) {
    // Reset the line buffer index
    lineBufferIndex = 0;
    // Reset the current line index for drawing the buffer
    currentLineIndex = 0;
    graphComplete = false;
    // Reset the graph buffer completeness marker
    lfsr = 1;  // Reset the LFSR to the initial seed value
    linesDrawn = 0;  // Reset the lines drawn counter

    // Store the graph height in the global variable
    globalGraphHeight = graphHeight;
    // Store the graph Y position in the global variable
    globalGraphYPos = graphPosY;

    uint16_t xStart = graphPosX;
    uint16_t yStart = graphPosY;

    int pointsToPlot = graphWidth - (LEGEND_LABEL_FONT_SIZE * LEGEND_LABEL_CHARS);
    int startIndex = dataSize > pointsToPlot ? dataSize - pointsToPlot : 0;

    float xScale = (((float)graphWidth - (LEGEND_LABEL_FONT_SIZE * LEGEND_LABEL_CHARS))) / (float)(pointsToPlot - 1);
    float yScale = (float)graphHeight / (graph_max - graph_min);

    int lastX = -1;
    int lastY = -1;
    bool lastValid = false;

    color16_t grayishColor = toGrayishColor(graphColor, GRAYING_FACTOR);  // Buffer the grayish color for faster plotting

    for (uint16_t i = startIndex; i < dataSize; i++) {
        // Calculate current X position for this data point
        uint16_t x = xStart + (uint16_t)((i - startIndex) * xScale);

        if (!isnan(data[i])) {  // If the data point is valid
            uint16_t y = yStart + graphHeight - (uint16_t)((data[i] - graph_min) * yScale);

            if (lastValid) {
                // Draw a line from the last valid point to the current valid point
                bufferLine(lastX, lastY, x, y, graphColor);
            }

            // Update the last valid point coordinates
            lastX = x;
            lastY = y;
            lastValid = true;
        } else {  // If the data point is missing
            if (lastValid) {
                // Find the next valid point for interpolation
                uint16_t j = i + 1;
                while (j < dataSize && isnan(data[j])) {
                    j++;
                }

                if (j < dataSize) {  // If a next valid point is found
                    // Calculate the coordinates of the next valid point
                    uint16_t nextValidX = xStart + (uint16_t)((j - startIndex) * xScale);
                    uint16_t nextValidY = yStart + graphHeight - (uint16_t)((data[j] - graph_min) * yScale);

                    // Linear interpolation for the current missing point
                    float factor = (float)(i - startIndex + 1) / (float)(j - startIndex + 1);
                    uint16_t interpY = lastY + factor * (nextValidY - lastY);

                    // Draw the interpolated line segment for the missing point
                    bufferLine(lastX, lastY, x, interpY, grayishColor);

                    // Update the last point to the interpolated point
                    lastX = x;
                    lastY = interpY;
                } else {
                    lastValid = false;  // No further valid points found, stop interpolating
                }
            }
        }
    }
}
