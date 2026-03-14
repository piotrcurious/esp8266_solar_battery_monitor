#include <iostream>
#include <vector>
#include "../SensorFilter.h"

int main() {
    // Adjusted to match new constructor: size, outlierThreshold, processNoise, measurementNoise
    SensorFilter filter(16, 2.0, 0.1, 1.0);

    float val;
    while (std::cin >> val) {
        float filtered = filter.updateSensorReading(val);
        std::cout << filtered << std::endl;
    }

    return 0;
}
