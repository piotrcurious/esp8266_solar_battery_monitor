#ifndef TELEMETRY_FRAME_HPP
#define TELEMETRY_FRAME_HPP

#include <cstdint>

#pragma pack(push,1)
struct telemetry_frame {
 float voltage_ADC0 = 12.1;
 float radio_active_time = 0 ;
 float wifi_rssi = -84.1;
 float SH4x_rel_humidity = 0;
 float SH4x_temperature = 0 ;
};
#pragma pack(pop)

#endif
