for (int i = 0; i < NUM_POLYNOMIALS; i++) { // Iterate over number of polynomials
    // Normalize data to the range [-1, 1] for better polynomial fitting
    float minVal = data[0];
    float maxVal = data[0];
    for (int j = 1; j < POLYNOMIAL_SIZE; j++) {
      minVal = min(minVal, data[j]);
      maxVal = max(maxVal, data[j]);
    }
    float range = maxVal - minVal;
    float normalizedData[POLYNOMIAL_SIZE];
    for(int j = 0; j < POLYNOMIAL_SIZE; j++){
        normalizedData[j] = (data[j] - minVal) / range * 2.0 - 1.0;
    }
