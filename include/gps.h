#include <Arduino.h>

void gps_time(char * buffer, uint8_t size);
uint8_t gps_sats();
float gps_hdop();
float gps_latitude();
float gps_longitude();
float gps_altitude();
void gps_setup();
void gps_loop();
