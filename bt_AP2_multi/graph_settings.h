#define MINUTES_GRAPH_BUFFER_MAX 60*24 //1440 *4 = 5760 bytes
#define NUMBER_OF_BUFFERS 2 // two buffers for two sensors, 11520 bytes
float minutes_buffer[NUMBER_OF_BUFFERS][MINUTES_GRAPH_BUFFER_MAX]; // first value is amount of rows
                                                                   // second is size of row
float minutes_buffer_min[NUMBER_OF_BUFFERS]  ; 
float minutes_buffer_max[NUMBER_OF_BUFFERS]  ; 

#define CHUNKS_PER_DRAW 32 // define how many chunks will be drawn per update
uint8_t current_graph = 0 ; // index of buffer used for current graph
// LABELS of graphs
#define LABEL_SIZE 16 // size of labels , in chars
const char graph_labels[NUMBER_OF_BUFFERS][LABEL_SIZE] = {"    voltage    ",
                                                          " signal level  "} ;
//                                                         0123456789012345,   0123456789012345
// SETUP of apperance
#define COLOR_FOREGROUND COLOR16_WHITE
#define GRAPH1_COLOR COLOR16_GREEN
const color16_t GRAPH_COLOR[NUMBER_OF_BUFFERS] ={COLOR16_GREEN,COLOR16_YELLOW} ;
#define COLOR_BACKGROUND COLOR16_BLACK
#define GRAYING_FACTOR 0.9 // factor of making color more gray

// setup intervals
//#define DO_NOT_NEED_BASIC_TOUCH_EVENTS    // Disables basic touch events like down, move and up. Saves 620 bytes program memory and 36 bytes RAM
            // we need basic touch events because we use touch down and touch up for user interface
            
#define MINUTES_INTERVAL 1000*60 //each minute
//#define MINUTES_INTERVAL 1000  // each second
//#define MINUTES_INTERVAL 1000*10  // each 10

#define DEBUG_INTERVAL 5000 //each second
#define LASTVALUE_INTERVAL 1000*10 //each 10 seconds
#define LABELS_INTERVAL 1000*10 //each 10 seconds

uint32_t minutes_millis_last = MINUTES_INTERVAL;
uint32_t debug_millis_last = DEBUG_INTERVAL;
uint32_t labels_millis_last = LABELS_INTERVAL;
uint32_t lastvalue_millis_last = LASTVALUE_INTERVAL;

//#define GRAPH_WIDTH  DISPLAY_WIDTH   // Graph width in pixels (adjust as needed)
//#define GRAPH_HEIGHT DISPLAY_HEIGHT-128    // Graph height in pixels (adjust as needed)

#define LEGEND_LABEL_FONT_SIZE 16 // define font size of labels
#define LEGEND_LABEL_FONT_WIDTH 12 // define font WIDTH of labels

#define LEGEND_LABEL_CHARS 4 // define how many characters per label

#define GRAPH_X  0      // initial position X
#define GRAPH_Y 128-64     // initial position Y
//const int minutes_dataArraySize = sizeof(minutes_buffer) / sizeof(minutes_buffer[0]);
const uint32_t minutes_dataArraySize = sizeof(minutes_buffer[0]) / sizeof(minutes_buffer[0][0]);

#define GRAPH_TEST // uncomment to fill with random values

#define MAX_LINES  MINUTES_GRAPH_BUFFER_MAX+10        // Define maximum buffer size for line storage
#define DRAWN_MAGIC_NUMBER 0xFFFF  // Magic number to mark the line as drawn

// Define the number of iterations for the LFSR
//#define LFSR_MAX_ITERATIONS 2048  // We will work with 2047 iterations
//#define LFSR_POLYNOMIAL 0x0600 ;  // Use the 11bit polynomial
        //  (0x0600 corresponds to the polynomial  x^11 + x^9 + 1)

//#define LFSR_MAX_ITERATIONS 2048  // We will work with 2047 iterations
//#define LFSR_POLYNOMIAL 0x0E04 ;  // Use the 11bit polynomial
        //  (0x0600 corresponds to the polynomial  x^11 + x^10 + x^9 + x^3 + 1)

#define LFSR_MAX_ITERATIONS 4096  // We will work with 4095 iterations
#define LFSR_POLYNOMIAL 0xD008 ;  // Use the 12bit polynomial
        //  (0xD008 corresponds to the polynomial x^12 + x^11 + x^10 + x^4 + 1)

//#define LFSR_MAX_ITERATIONS 65535  // 16bit polynomial with 65535 iterations
//#define LFSR_POLYNOMIAL 0xB400 ;  // 16bit polynomial (0xB400 corresponds to the polynomial x^16 + x^14 + x^13 + x^11 + 1)

// Variables to store the current state of the LFSR and maximum lines
uint16_t lfsr = 1;  // Initial seed value for the LFSR
//int maxLines = MAX_LINES;  // Set this to the maximum number of lines in the buffer
int linesDrawn = 0;  // Track how many lines have been drawn

// Structure to store line parameters
typedef struct {
    uint16_t x1, y1, x2, y2;
    uint16_t color;
} LineBuffer;

LineBuffer lineBuffer[MAX_LINES];
uint16_t lineBufferIndex = 0;
// Global variable to track the current position in the line buffer
uint16_t currentLineIndex = 0;
bool graphComplete = false; // indicates if graph buffer is completely drawn

// Global variable to store the graph height
uint16_t globalGraphHeight = 0;
// Global variable to store the graph Y position
uint16_t globalGraphYPos = 0;

uint16_t displayWidth ; //= BlueDisplay1.getDisplayWidth();
uint16_t displayHeight;  //= BlueDisplay1.getDisplayHeight();