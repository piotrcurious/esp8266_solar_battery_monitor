#pragma pack(push,1)
//typedef struct telemetry_frame {
struct telemetry_frame {
 float voltage_ADC0 = 12.1;
 float radio_active_time = 0 ; 
 float wifi_rssi = -84.1; 
 float SH4x_rel_humidity = 0;
 float SH4x_temperature = 0 ; 
 
};
//} tframe;
#pragma pack(pop)

bool new_packet = false ; // new packet flag
uint32_t total_packets = 0 ; // packets count
