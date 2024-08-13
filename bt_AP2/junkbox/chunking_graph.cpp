// Function to plot a section of the graph and clean the plot area
void plotGraphSection(float *data, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
                      uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max,
                      uint16_t startIdx, uint16_t endIdx) {
  // Ensure startIdx and endIdx are within bounds
  if (startIdx >= dataSize) startIdx = dataSize - 1;
  if (endIdx >= dataSize) endIdx = dataSize - 1;
  if (startIdx > endIdx) return;  // Invalid range, do nothing

  uint16_t xStart = graphPosX;
  uint16_t yStart = graphPosY;
  uint16_t sectionWidth = (endIdx - startIdx + 1) * (graphWidth / dataSize);

  // Clear the relevant section of the plot area
  BlueDisplay1.fillRect(xStart + (startIdx * (graphWidth / dataSize)), yStart, sectionWidth, graphHeight, COLOR_BACKGROUND);

  float xScale = (float)graphWidth / (float)(dataSize - 1);
  float yScale = (float)graphHeight / (graph_max - graph_min);

  int lastX = -1;
  int lastY = -1;
  bool lastValid = false;

  color16_t whiteColor = COLOR16_WHITE;
  color16_t grayColor = COLOR16_GRAY;

  // Plot the section of the graph
  for (int i = startIdx; i <= endIdx; i++) {
    if (!isnan(data[i])) {  // Check if the current data point is valid
      int x = xStart + (int)(i * xScale);
      int y = yStart + graphHeight - (int)((data[i] - graph_min) * yScale);

      if (lastValid) {
        // If the line crosses the boundary, cap it at the boundary
        int drawStartX = max(lastX, xStart + (startIdx * xScale));
        int drawEndX = min(x, xStart + (endIdx * xScale));

        BlueDisplay1.drawLine(drawStartX, lastY, drawEndX, y, whiteColor);  // Draw line using white color
      }

      lastX = x;
      lastY = y;
      lastValid = true;
    } else {
      if (lastValid) {
        // Draw a gray line to mark missing data if the last point was valid
        int j = i + 1;
        while (j <= endIdx && isnan(data[j])) {
          j++;
        }

        if (j <= endIdx) {
          int nextValidX = xStart + (int)(j * xScale);
          int nextValidY = yStart + graphHeight - (int)((data[j] - graph_min) * yScale);

          // Cap the line within the boundaries
          int drawStartX = max(lastX, xStart + (startIdx * xScale));
          int drawEndX = min(nextValidX, xStart + (endIdx * xScale));

          BlueDisplay1.drawLine(drawStartX, lastY, drawEndX, nextValidY, grayColor);  // Use gray color for missing data indicator
        }
      }
      lastValid = false;
    }
  }

  // Draw graph boundaries
  sprintf(sStringBuffer, "%f", graph_min);    
  BlueDisplay1.drawText(xStart + graphWidth - 16 * 8, yStart + graphHeight - 8, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
  BlueDisplay1.drawLine(xStart, yStart + graphHeight - 1, xStart + graphWidth, yStart + graphHeight - 1, COLOR16_RED);  // bottom boundary

  sprintf(sStringBuffer, "%f", graph_max);    
  BlueDisplay1.drawText(xStart + graphWidth - 16 * 8, yStart, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
  BlueDisplay1.drawLine(xStart, yStart, xStart + graphWidth, yStart, COLOR16_RED);  // top boundary

  sprintf(sStringBuffer, "%f", data[endIdx]);  // Get last data point in the section
  int nextValidY = yStart + graphHeight - (int)((data[endIdx] - graph_min) * yScale);
  BlueDisplay1.drawText(xStart + graphWidth - 16 * 8, nextValidY, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
  BlueDisplay1.drawLine(xStart, nextValidY, xStart + graphWidth, nextValidY, COLOR16_LIGHT_GREY);  // horizontal line at last data point
}
