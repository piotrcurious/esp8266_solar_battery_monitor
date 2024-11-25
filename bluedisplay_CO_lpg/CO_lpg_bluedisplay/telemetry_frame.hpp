#pragma pack(push,1)
//typedef struct telemetry_frame {
struct telemetry_frame {

 float CO_sensor = 0.1;
 float LPG_sensor = 0.2;
 
};
//} tframe;
#pragma pack(pop)

bool new_packet = false ; // new packet flag
uint32_t total_packets = 0 ; // packets count
