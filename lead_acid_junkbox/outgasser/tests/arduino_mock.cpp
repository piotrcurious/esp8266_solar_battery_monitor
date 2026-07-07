#include "arduino_mock.h"

unsigned long _mock_millis = 0;
SerialMock Serial;
WireMock Wire;
int _mock_duty_ch = 0;
int _mock_ntc_counts = 2000;
std::vector<char> _mock_serial_buf;
unsigned long _nvs_write_count = 0;
