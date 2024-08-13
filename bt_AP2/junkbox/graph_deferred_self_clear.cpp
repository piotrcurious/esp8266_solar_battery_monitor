#include <stdlib.h>  // For rand()

#define MAX_LINES  1000        // Define maximum buffer size for line storage
#define DRAWN_MAGIC_NUMBER 0xFFFF  // Magic number to mark the line as drawn

// Structure to store line parameters
typedef struct {
    int x1, y1, x2, y2;
    color16_t color;
} LineBuffer;

LineBuffer lineBuffer[MAX_LINES];
int lineBufferIndex = 0;
// Global variable to track the current position in the line buffer
int currentLineIndex = 0;

// Global variable to store the graph height
uint16_t globalGraphHeight = 0;

// Function to add a line to the buffer
void bufferLine(int x1, int y1, int x2, int y2, color16_t color) {
    if (lineBufferIndex < MAX_LINES) {
        lineBuffer[lineBufferIndex].x1 = x1;
        lineBuffer[lineBufferIndex].y1 = y1;
        lineBuffer[lineBufferIndex].x2 = x2;
        lineBuffer[lineBufferIndex].y2 = y2;
        lineBuffer[lineBufferIndex].color = color;
        lineBufferIndex++;
    } else {
        // Handle buffer overflow (e.g., log an error or extend buffer size)
    }
}

// Function to draw a line from the buffer and mark it as drawn
void drawBufferedLine(int index) {
    if (index < lineBufferIndex && lineBuffer[index].color != DRAWN_MAGIC_NUMBER) {
        int x1 = lineBuffer[index].x1;
        int y1 = lineBuffer[index].y1;
        int x2 = lineBuffer[index].x2;
        int y2 = lineBuffer[index].y2;
        
        // Determine the minimum and maximum x values for the rectangle
        int xMin = x1 < x2 ? x1 : x2;
        int xMax = x1 > x2 ? x1 : x2;
        
        // Define the rectangle width and use the global graph height for height
        int rectWidth = xMax - xMin + 1;
        int rectHeight = globalGraphHeight;  // Use the global graph height

        // Clear the rectangle before drawing the line
        BlueDisplay1.fillRect(xMin, 0, rectWidth, rectHeight, COLOR_BACKGROUND);

        // Now draw the line
        BlueDisplay1.drawLine(x1, y1, x2, y2, lineBuffer[index].color);
        
        // Mark the line as drawn
        lineBuffer[index].color = DRAWN_MAGIC_NUMBER;
    }
}

// Modified plotGraph function with buffer reset and storing global graph height
void plotGraph(float *data, uint16_t dataSize, uint16_t graphPosX, uint16_t graphPosY,
               uint16_t graphWidth, uint16_t graphHeight, float graph_min, float graph_max) {
    // Reset the line buffer index
    lineBufferIndex = 0;
    // Reset the current line index for drawing the buffer
    currentLineIndex = 0; 
    
    // Store the graph height in the global variable
    globalGraphHeight = graphHeight;

    uint16_t xStart = graphPosX;
    uint16_t yStart = graphPosY;

    int pointsToPlot = graphWidth;
    int startIndex = dataSize > pointsToPlot ? dataSize - pointsToPlot : 0;

    float xScale = (float)graphWidth / (float)(pointsToPlot - 1);
    float yScale = (float)graphHeight / (graph_max - graph_min);

    int lastX = -1;
    int lastY = -1;
    bool lastValid = false;

    color16_t whiteColor = COLOR16_BLACK;
    color16_t grayColor = COLOR16_BLUE;

    for (int i = startIndex; i < dataSize; i++) {
        if (!isnan(data[i])) {  // Check if the current data point is valid
            int x = xStart + (int)((i - startIndex) * xScale);
            int y = yStart + graphHeight - (int)((data[i] - graph_min) * yScale);

            if (lastValid) {
                bufferLine(lastX, lastY, x, y, whiteColor);  // Buffer the line instead of drawing it immediately
            }

            lastX = x;
            lastY = y;
            lastValid = true;
        } else {
            if (lastValid) {
                int j = i + 1;
                while (j < dataSize && isnan(data[j])) {
                    j++;
                }

                if (j < dataSize) {
                    int nextValidX = xStart + (int)((j - startIndex) * xScale);
                    int nextValidY = yStart + graphHeight - (int)((data[j] - graph_min) * yScale);
                    bufferLine(lastX, lastY, nextValidX, nextValidY, grayColor);  // Buffer the line for missing data
                }
            }
            lastValid = false;
        }
    }

    // Draw the axis boundaries and graph labels
    sprintf(sStringBuffer, "%f", graph_min);    
    BlueDisplay1.drawText(xStart + graphWidth - 16 * 8, yStart + graphHeight - 8, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
    bufferLine(xStart, yStart + graphHeight - 1, xStart + graphWidth, yStart + graphHeight - 1, COLOR16_RED);

    sprintf(sStringBuffer, "%f", graph_max);    
    BlueDisplay1.drawText(xStart + graphWidth - 16 * 8, yStart, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
    bufferLine(xStart, yStart, xStart + graphWidth, yStart, COLOR16_RED);

    sprintf(sStringBuffer, "%f", data[dataSize - 1]);    // Get the last data point
    int lastDataY = yStart + graphHeight - (int)((data[dataSize - 1] - graph_min) * yScale);

    BlueDisplay1.drawText(xStart + graphWidth - 16 * 8, lastDataY, sStringBuffer, 16, COLOR_FOREGROUND, COLOR_BACKGROUND);
    bufferLine(xStart, lastDataY, xStart + graphWidth, lastDataY, COLOR16_LIGHT_GREY);
}

// Function to draw a line from the buffer and mark it as drawn
void drawBufferedLineNoClr(int index) {
    if (index < lineBufferIndex && lineBuffer[index].color != DRAWN_MAGIC_NUMBER) {
        BlueDisplay1.drawLine(lineBuffer[index].x1, lineBuffer[index].y1, 
                              lineBuffer[index].x2, lineBuffer[index].y2, 
                              lineBuffer[index].color);
        lineBuffer[index].color = DRAWN_MAGIC_NUMBER;  // Mark the line as drawn
    }
}

// Function to draw all lines from the buffer
void drawAllBufferedLines() {
    for (int i = 0; i < lineBufferIndex; i++) {
        drawBufferedLine(i);
    }
}

// Function to draw a random subset of lines from the buffer
void drawRandomLines(int numLinesToDraw) {
    if (numLinesToDraw > lineBufferIndex) {
        numLinesToDraw = lineBufferIndex;  // Limit to available lines
    }

    for (int i = 0; i < numLinesToDraw; i++) {
        int randomIndex = rand() % lineBufferIndex;  // Get a random line index
        drawBufferedLine(randomIndex);
    }
}


// Function to draw a specified number of lines sequentially from the buffer
void drawSequentialLines(int numLinesToDraw) {
    // Draw lines sequentially from the current position
    for (int i = 0; i < numLinesToDraw; i++) {
        if (currentLineIndex < lineBufferIndex) {
            drawBufferedLine(currentLineIndex);
            currentLineIndex++;
        } else {
            // If we reach the end of the buffer, stop drawing
            break;
        }
    }
}
