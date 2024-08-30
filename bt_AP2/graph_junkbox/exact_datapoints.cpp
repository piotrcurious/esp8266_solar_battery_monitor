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

    color16_t grayishColor = toGrayishColor(graphColor, GRAYING_FACTOR); // Buffer the grayish color for faster plotting

    for (uint16_t i = startIndex; i < dataSize; i++) {
        if (!isnan(data[i])) {  // Check if the current data point is valid
            uint16_t x = xStart + (uint16_t)((i - startIndex) * xScale);
            uint16_t y = yStart + graphHeight - (uint16_t)((data[i] - graph_min) * yScale);

            if (lastValid) {
                bufferLine(lastX, lastY, x, y, graphColor);  // Buffer the line instead of drawing it immediately
            }

            lastX = x;
            lastY = y;
            lastValid = true;
        } else {
            // Handle multiple missing values by finding the next valid point
            uint16_t j = i + 1;
            while (j < dataSize && isnan(data[j])) {
                j++;
            }

            if (lastValid && j < dataSize) {
                uint16_t nextValidX = xStart + (uint16_t)((j - startIndex) * xScale);
                uint16_t nextValidY = yStart + graphHeight - (uint16_t)((data[j] - graph_min) * yScale);

                // Calculate the number of missing points and the increments for interpolation
                uint16_t numMissing = j - i;
                float deltaX = (float)(nextValidX - lastX) / (numMissing + 1);
                float deltaY = (float)(nextValidY - lastY) / (numMissing + 1);

                // Interpolate across all missing points
                for (uint16_t k = 1; k <= numMissing; k++) {
                    uint16_t interpolatedX = lastX + (uint16_t)(deltaX * k);
                    uint16_t interpolatedY = lastY + (uint16_t)(deltaY * k);
                    bufferLine(lastX, lastY, interpolatedX, interpolatedY, grayishColor);  // Buffer the interpolated line
                    lastX = interpolatedX;
                    lastY = interpolatedY;
                }

                // Draw the final line to the next valid point
                bufferLine(lastX, lastY, nextValidX, nextValidY, grayishColor);
                lastX = nextValidX;
                lastY = nextValidY;
                lastValid = true;
            } else {
                // If no valid point is found after missing values, mark as invalid
                lastValid = false;
            }

            // Skip to the last handled index
            i = j - 1;
        }
    }
}
