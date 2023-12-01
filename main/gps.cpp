/*

  GPS module

  Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include <Arduino.h>
#include <TinyGPS++.h>
#include "configuration.h"
#include "gps.h"
#include <Wire.h>
#include "main.h"
#include <queue>
#include "FS.h"
#include "screen.h"
#include "LittleFS.h"
#include <flash.h>
#include <iostream>
#include <sstream>
#include <iterator>

uint32_t LatitudeBinary;
uint32_t LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
uint8_t sats;

uint32_t Timeb;
uint32_t TimeR;
uint32_t LatitudeBi1;
uint32_t LongitudeBi1;
uint16_t altit1;

uint32_t Timeb1;
uint32_t TimeR1;
uint32_t LatitudeBi2;
uint32_t LongitudeBi2;
uint16_t altit2;

uint32_t Diff1;
uint32_t Diff2;

AXP20X_Class axp2;
std::queue<String> Queue;
std::queue<String> DataButton1;
std::queue<String> DataButton2;
std::queue<String> Queue1;

char t[32]; // used to sprintf for Serial output

TinyGPSPlus _gps;
HardwareSerial _serial_gps(GPS_SERIAL_NUM);
uint8_t BatPercent = axp2.getBattPercentage();
bool Remove = axp2.isVbusRemoveIRQ();
uint8_t Status;


void gps_time(char *buffer, uint8_t size)
{
    snprintf(buffer, size, "%02d:%02d:%02d", _gps.time.hour(), _gps.time.minute(), _gps.time.second());
}

float gps_latitude()
{
    return _gps.location.lat();
}

float gps_longitude()
{
    return _gps.location.lng();
}

float gps_altitude()
{
    return _gps.altitude.meters();
}

float gps_hdop()
{
    return _gps.hdop.hdop();
}

uint8_t gps_sats()
{
    return _gps.satellites.value();
}

void gps_setup()
{
    _serial_gps.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void gps_loop()
{
    while (_serial_gps.available())
    {
        _gps.encode(_serial_gps.read());
    }
}

void Turn_Motor(int Di)
{
     Wire.beginTransmission(ARDUINO_ADDRESS); 
     Wire.write(Di);
     
     Wire.endTransmission();
}
bool CheckTime(String OpenTime)
{
    if (!OpenTime.isEmpty() && OpenTime != "000000")
    {

        const char *Open = OpenTime.c_str();

        std::string timeStr(Open);
        int HH = std::stoi(timeStr.substr(0, 2));
        int MM = std::stoi(timeStr.substr(2, 2));
        int SE = std::stoi(timeStr.substr(4, 2));
        int Distance = std::stoi(timeStr.substr(6, 2));

        if (HH == _gps.time.hour() && MM == _gps.time.minute() && SE == _gps.time.second())
        {
            Turn_Motor(Distance);
            writeFile(LittleFS, Configuration_Time_FILE, "999999");
            return false;
        }
        else
            return true;
    }
}

uint8_t Positons(uint8_t RelativePosition, uint8_t ButtonOrder)
{
    return (14 + RelativePosition + (ButtonOrder * 10));
}

uint8_t size;
String data1;
std::vector<std::string> DATA;
#if defined(PAYLOAD_USE_FULL)

// More data than PAYLOAD_USE_CAYENNE
void buildPacket(uint8_t txBuffer[13])
{
    LatitudeBinary = ((_gps.location.lat() + 90) / 180.0) * 16777215;
    LongitudeBinary = ((_gps.location.lng() + 180) / 360.0) * 16777215;
    hdopGps = _gps.hdop.value() / 10;
    sats = _gps.satellites.value();
    // get time
    char Buffertime[9];
    gps_time(Buffertime, sizeof(Buffertime));
    if (getBaChStatus() == "Charging")
    {
        Status = 1;
    }
    else
    {
        Status = 0;
    }

    sprintf(t, "Lat: %f", _gps.location.lat());
    Serial.println(t);
    sprintf(t, "Lng: %f", _gps.location.lng());
    Serial.println(t);
    sprintf(t, "Hdop: %d", hdopGps);
    Serial.println(t);
    sprintf(t, "Sats: %d", sats);
    Serial.println(t);
    Serial.print("time: ");
    Serial.println(Buffertime);

    uint32_t timebinary = ((_gps.time.hour() * 3600) + (_gps.time.minute() * 60) + _gps.time.second());
    Serial.print("Binary: ");
    Serial.println(timebinary);

    uint8_t BatPercentage = BatPercent;
    Serial.print("Battery pourcentage1: ");
    Serial.println(BatPercent);
    Serial.print("Battery pourcentage: ");
    Serial.println(BatPercentage);
    Serial.print("statu: ");
    Serial.println(Status);

    txBuffer[0] = (LatitudeBinary >> 16) & 0xFF;
    txBuffer[1] = (LatitudeBinary >> 8) & 0xFF;
    txBuffer[2] = LatitudeBinary & 0xFF;
    txBuffer[3] = (LongitudeBinary >> 16) & 0xFF;
    txBuffer[4] = (LongitudeBinary >> 8) & 0xFF;
    txBuffer[5] = LongitudeBinary & 0xFF;
    txBuffer[6] = hdopGps & 0xFF;
    txBuffer[7] = sats & 0xFF;
    txBuffer[8] = (timebinary >> 16) & 0xFF;
    txBuffer[9] = (timebinary >> 8) & 0xFF;
    txBuffer[10] = timebinary & 0xFF;
    txBuffer[11] = BatPercentage & 0xFF;
    txBuffer[12] = Status & 0xFF;
}

#elif defined(PAYLOAD_USE_CAYENNE)

// CAYENNE DF
void buildPacket(uint8_t txBuffer[11])
{
    sprintf(t, "Lat: %f", _gps.location.lat());
    Serial.println(t);
    sprintf(t, "Lng: %f", _gps.location.lng());
    Serial.println(t);
    sprintf(t, "Alt: %f", _gps.altitude.meters());
    Serial.println(t);
    int32_t lat = _gps.location.lat() * 10000;
    int32_t lon = _gps.location.lng() * 10000;
    int32_t alt = _gps.altitude.meters() * 100;

    txBuffer[2] = lat >> 16;
    txBuffer[3] = lat >> 8;
    txBuffer[4] = lat;
    txBuffer[5] = lon >> 16;
    txBuffer[6] = lon >> 8;
    txBuffer[7] = lon;
    txBuffer[8] = alt >> 16;
    txBuffer[9] = alt >> 8;
    txBuffer[10] = alt;
}

#endif
