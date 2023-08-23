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
void ReadData () ; 
void   IRAM_ATTR button1Interrupt();
void   IRAM_ATTR button2Interrupt(); 
void IRAM_ATTR button1InterruptRelease (); 
void IRAM_ATTR button2InterruptRelease();
void  Buttonsetup() ;
void WriteDataButton() ; 