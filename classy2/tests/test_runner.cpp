#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include "../SensorFilter.h"

int main(int argc, char* argv[]) {
    size_t windowSize = 16;
    float threshold = 2.0f;
    float q_pos = 0.01f;
    float q_vel = 0.001f;
    float r = 0.1f;

    if (argc >= 2) windowSize = std::stoul(argv[1]);
    if (argc >= 3) threshold = std::stof(argv[2]);
    if (argc >= 4) q_pos = std::stof(argv[3]);
    if (argc >= 5) q_vel = std::stof(argv[4]);
    if (argc >= 6) r = std::stof(argv[5]);

    SensorFilter filter(windowSize, threshold, q_pos, q_vel, r);

    float val;
    while (std::cin >> val) {
        float filtered = filter.updateSensorReading(val);
        std::cout << filtered << std::endl;
    }

    return 0;
}
