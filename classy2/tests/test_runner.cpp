#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include "../SensorFilter.h"

int main(int argc, char* argv[]) {
    size_t windowSize = 16;
    float threshold = 2.0f;
    float q = 0.01f;
    float r = 0.1f;

    if (argc >= 2) windowSize = std::stoul(argv[1]);
    if (argc >= 3) threshold = std::stof(argv[2]);
    if (argc >= 4) q = std::stof(argv[3]);
    if (argc >= 5) r = std::stof(argv[4]);

    SensorFilter filter(windowSize, threshold, q, r);

    float val;
    while (std::cin >> val) {
        float filtered = filter.updateSensorReading(val);
        std::cout << filtered << std::endl;
    }

    return 0;
}
