#include <Arduino.h>

void gps_time(char * buffer, uint8_t size); 
//void Time_Remove (char* Tibuffer);
uint8_t gps_sats();
float gps_hdop();
float gps_latitude();
float gps_longitude();
float gps_altitude();
void gps_setup();
void gps_loop();
void button1check();
void button2check();
void  Buttonsetup() ;
bool CheckTime(String OpenTime) ; 
void Motor_Setup() ; 
void Turn_Motor(int Di) ; 
void Turn_Back_Motor() ; 
void Turn_ON_Motor();