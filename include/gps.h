#include <Arduino.h>
#include "axp20x.h"
 AXP20X_Class getAxp2();
void gps_time(char * buffer, uint8_t size); 
//void Time_Remove (char* Tibuffer);
uint8_t gps_sats();
float gps_hdop();
float gps_latitude();
float gps_longitude();
float gps_altitude();
void gps_setup();
void gps_loop();
bool CheckTime(String OpenTime) ; 
 void Turn_Motor(int Di) ;
 
