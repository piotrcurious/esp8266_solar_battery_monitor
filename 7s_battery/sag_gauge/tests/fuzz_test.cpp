#include "mock_arduino/Arduino.h"
#include <iostream>
#include <vector>
#include <string>

extern MockSerial Serial;

#define ARDUINO 100
#include "../infer.ino"

void fuzz_serial(const std::string& cmd) {
    Serial._input_buffer = cmd;
    loop();
}

int main() {
    setup();

    printf("--- Fuzzing Serial Parser ---\n");

    std::vector<std::string> payloads = {
        "RESET",
        "SETCAP 100",
        "SETCAP -5",
        "SETCAP 999999",
        "SETCAP abc",
        "UNITS",
        "SETZERO",
        "INVALID_CMD",
        "SETCAP ",
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        "\x01\x02\x03\xFF",
        "SETCAP 20.5",
        "  RESET  "
    };

    for (const auto& p : payloads) {
        printf("Payload: [%s]\n", p.c_str());
        fuzz_serial(p);
    }

    printf("Fuzzing Complete. No crashes observed.\n");
    return 0;
}
