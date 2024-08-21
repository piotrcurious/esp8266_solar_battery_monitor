#include "VariableDefinitions.h"

// Initialize the float variable and its path
float floatVar = 3.14f;
const char* floatVarPath = "/float";

// Initialize the float array and its path
float floatArray[5] = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f};
const char* floatArrayPath = "/array";
const size_t floatArraySize = sizeof(floatArray) / sizeof(floatArray[0]);
