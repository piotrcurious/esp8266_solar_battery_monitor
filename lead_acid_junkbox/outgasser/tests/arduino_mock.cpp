#include "arduino_mock.h"

unsigned long _mock_millis = 0;
SerialMock Serial;
WireMock Wire;
int _mock_duty_ch = 0;
int _mock_duty_dis = 0;
