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
void ReadData()
{
    Queue = readFile(LittleFS, FAILED_DATA_FILE);
}

void Buttonsetup()
{
    pinMode(BUTTON_1_PIN, INPUT_PULLDOWN); // Set the button pin as input with internal pull-up resistor
    pinMode(BUTTON_2_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(BUTTON_2_PIN), button2Interrupt, CHANGE); // Attach interrupt to the button pin
    attachInterrupt(digitalPinToInterrupt(BUTTON_1_PIN), button1Interrupt, CHANGE);
}

void button2Interrupt()
{
    // Interrupt service routine
    // Perform actions when the button state changes
    int buttonState = digitalRead(BUTTON_2_PIN);
    // Read the button state

    if (buttonState == HIGH)
    {
        char Time[9];
        gps_time(Time, sizeof(Time));
        Serial.println("button 2 ");
        Serial.println(Time);

        Timeb = ((_gps.time.hour() * 3600) + (_gps.time.minute() * 60) + _gps.time.second());
        LatitudeBi2 = ((_gps.location.lat() + 90) / 180.0) * 16777215;
        LongitudeBi2 = ((_gps.location.lng() + 180) / 360.0) * 16777215;
    }
    else
    {
        char Time1[9];
        Serial.println("button 2 released ");
        gps_time(Time1, sizeof(Time1));
        TimeR = ((_gps.time.hour() * 3600) + (_gps.time.minute() * 60) + _gps.time.second());
        char TimeS[16];
        char LatS[16];
        char LongS[16];
        itoa(Timeb, TimeS, 10);
        itoa(LatitudeBi2, LatS, 10);
        itoa(LongitudeBi2, LongS, 10);
        std::string ButtonData = "2 " + std::string(TimeS) + " " + std::string(LatS) + " " + std::string(LongS) + '\n';
        const char *data = ButtonData.c_str();
        DataButton1.push(data);
        Serial.println(data);
    }
}
void button1Interrupt()
{
    // Interrupt service routine
    // Perform actions when the button state changes
    int buttonState1 = digitalRead(BUTTON_1_PIN);
    // Read the button state

    if (buttonState1 == HIGH)
    {
        char Timee[9];
        gps_time(Timee, sizeof(Timee));
        Serial.println("button 1 ");

        Timeb1 = ((_gps.time.hour() * 3600) + (_gps.time.minute() * 60) + _gps.time.second());
        LatitudeBi1 = ((_gps.location.lat() + 90) / 180.0) * 16777215;
        LongitudeBi1 = ((_gps.location.lng() + 180) / 360.0) * 16777215;
    }
    else
    {
        char Time2[9];
        Serial.println("button 1 released ");
        gps_time(Time2, sizeof(Time2));
        TimeR1 = ((_gps.time.hour() * 3600) + (_gps.time.minute() * 60) + _gps.time.second());
        char TimeS1[16];
        char LatS1[16];
        char LongS1[16];
       
        itoa(Timeb1, TimeS1, 10);
        itoa(LatitudeBi1, LatS1, 10);
        itoa(LongitudeBi1, LongS1, 10);
        std::string ButtonData1 = "1 " + std::string(TimeS1) + " " + std::string(LatS1) + " " + std::string(LongS1)  + '\n';
        const char *data1 = ButtonData1.c_str();
        DataButton2.push(data1);
        Serial.println(data1);
    }
}
void WriteDataButton()
{
    while (!DataButton1.empty())
    {
        String DataButt = DataButton1.front();
        const char *dat = DataButt.c_str();
        Serial.println("FROM THE QUEUE");
        Serial.println(DataButt);
        appendFile(LittleFS, Button_Data, dat);
        DataButton1.pop();
    }
    while (!DataButton2.empty())
    {
        String DataBut = DataButton2.front();
        const char *da = DataBut.c_str();
        Serial.println("FROM THE QUEUE");
        Serial.println(DataBut);
        appendFile(LittleFS, Button_Data, da);
        DataButton2.pop();
    }
}
uint8_t Positons(uint8_t RelativePosition, uint8_t ButtonOrder)
{
    return (26 + RelativePosition + (ButtonOrder * 10));
}

uint8_t size;
String data1;
std::vector<std::string> DATA;
#if defined(PAYLOAD_USE_FULL)

// More data than PAYLOAD_USE_CAYENNE
void buildPacket(uint8_t txBuffer[57])
{
    WriteDataButton();
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
    if (Queue1.empty())
    {
        Queue1 = readFile(LittleFS, Button_Data);
    }

    if (!Queue.empty())
    {
        String data = Queue.front();

        // std::string data;
        Serial.println("sending data to LoRaWAN");
        Serial.println(data);

        const char *c = data.c_str();
        std::istringstream iss(c);
        std::vector<std ::string> tokens{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};

        double latt = std::stod(tokens[0]);
        char Latdeeg[16];
        dtostrf(latt, 8, 6, Latdeeg);
        Serial.println(Latdeeg);
        double loong = std::stod(tokens[1]);
        uint8_t hdp = std::stoi(tokens[2]);
        uint8_t sta = std::stoi(tokens[3]);
        unsigned int hr = std::stoi(tokens[4]);
        unsigned int Mi = std::stoi(tokens[5]);
        unsigned int Sec = std::stoi(tokens[6]);
        uint8_t BPc = std::stoi(tokens[7]);
        uint8_t Bs = std::stoi(tokens[8]);
        uint32_t LatB = ((latt + 90.0) / 180.0) * 16777215.0;
        uint32_t LongB = ((loong + 180.0) / 360.0) * 16777215.0;
        uint32_t Tb = ((hr * 3600) + (Mi * 60) + Sec);
        uint8_t BPB = (BPc / 100.0) * 255.0;

        if (!Queue1.empty())
        {
            if (Queue1.size() >= 3)
            {
                uint32_t count = 0;
                while (count != 3)
                {
                    size = 3;
                    String dataButton = Queue1.front();
                    const char *cc = dataButton.c_str();
                    DATA.push_back(cc);
                    Serial.println("pushed in vector ");
                    Queue1.pop();
                    count++;
                }
                count = 0;
            }
            else if (Queue1.size() < 3)
            {
                uint32_t count = 0;
                while (count != Queue1.size())
                {
                    size = Queue1.size();
                    String dataButton = Queue1.front();
                    const char *cc = dataButton.c_str();
                    Serial.println("pushed in vector ");
                    DATA.push_back(cc);
                    Queue1.pop();
                    count++;
                }
                count = 0;
            }

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
            txBuffer[13] = (LatB >> 16) & 0xFF;
            txBuffer[14] = (LatB >> 8) & 0xFF;
            txBuffer[15] = LatB & 0xFF;
            txBuffer[16] = (LongB >> 16) & 0xFF;
            txBuffer[17] = (LongB >> 8) & 0xFF;
            txBuffer[18] = LongB & 0xFF;
            txBuffer[19] = hdp & 0xFF;
            txBuffer[20] = sta & 0xFF;
            txBuffer[21] = (Tb >> 16) & 0xFF;
            txBuffer[22] = (Tb >> 8) & 0xFF;
            txBuffer[23] = Tb & 0xFF;
            txBuffer[24] = BPB & 0xFF;
            txBuffer[25] = Bs & 0xFF;
            txBuffer[26] = size & 0xFF;

            for (size_t i = 0; i < size ; i++)
            {
                if (!DATA.empty())
                {
                    std::string DatabuttonsString = DATA.front();
                    const char *DataButtons = DatabuttonsString.c_str();

                    std::istringstream iss(DataButtons);
                    std::vector<std ::string> Items{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
                    uint8_t buttonNum = std::stoi(Items[0]);
                    uint32_t TimeButton = std::stoi(Items[1]);
                    uint32_t LatitudeButton = std::stoi(Items[2]);
                    uint32_t LongitudeButton = std::stoi(Items[3]);


                    txBuffer[Positons(1, i)] = buttonNum & 0xFF;
                    txBuffer[Positons(2, i)] = (TimeButton >> 16) & 0xFF;
                    txBuffer[Positons(3, i)] = (TimeButton >> 8) & 0xFF;
                    txBuffer[Positons(4, i)] = TimeButton & 0xFF;
                    txBuffer[Positons(5, i)] = (LatitudeButton >> 16) & 0xFF;
                    txBuffer[Positons(6, i)] = (LatitudeButton >> 8) & 0xFF;
                    txBuffer[Positons(7, i)] = LatitudeButton & 0xFF;
                    txBuffer[Positons(8, i)] = (LongitudeButton >> 16) & 0xFF;
                    txBuffer[Positons(9, i)] = (LongitudeButton >> 8) ;
                    txBuffer[Positons(10, i)] = LongitudeButton & 0xFF ;
                    DATA.erase(DATA.begin());
                }
            }

            if (LMIC.txrxFlags & TXRX_ACK)
            {
                Queue.pop();
            }
        }
        else
        {
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
            txBuffer[13] = (LatB >> 16) & 0xFF;
            txBuffer[14] = (LatB >> 8) & 0xFF;
            txBuffer[15] = LatB & 0xFF;
            txBuffer[16] = (LongB >> 16) & 0xFF;
            txBuffer[17] = (LongB >> 8) & 0xFF;
            txBuffer[18] = LongB & 0xFF;
            txBuffer[19] = hdp & 0xFF;
            txBuffer[20] = sta & 0xFF;
            txBuffer[21] = (Tb >> 16) & 0xFF;
            txBuffer[22] = (Tb >> 8) & 0xFF;
            txBuffer[23] = Tb & 0xFF;
            txBuffer[24] = BPB & 0xFF;
            txBuffer[25] = Bs & 0xFF;
            for (size_t i = 26; i < 67; i++)
            {
                txBuffer[i] = 0;
            }
            if (LMIC.txrxFlags & TXRX_ACK)
            {
                Queue.pop();
            }
        }
    }
    else
    {
        if (!Queue1.empty())
        {
            if (Queue1.size() >= 3)
            {
                uint32_t count = 0;
                while (count != 3)
                {
                    size = 3;
                    String dataButton = Queue1.front();
                    const char *cc = dataButton.c_str();
                    DATA.push_back(cc);
                    Serial.println("pushed in vector 1 ");
                    Queue1.pop();
                    count++;
                }

                Serial.println("reset count 1 ");
            }
            else if (Queue1.size() < 3)
            {
                uint32_t count = 0;
                size = Queue1.size();
                while (count < size)
                {

                    String dataButton = Queue1.front();
                    const char *cc = dataButton.c_str();
                    Serial.println("pushed in vector 2 ");
                    DATA.push_back(cc);
                    Queue1.pop();
                    count++;
                }

                Serial.println("reset count 2 ");
            }

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
            for (size_t i = 13; i < 26; i++)
            {
                txBuffer[i] = 0;
            }

            txBuffer[26] = size & 0xFF;

            for (size_t i = 0; i < size ; i++)
            {
                if (!DATA.empty())
                {
                    std::string DatabuttonsString = DATA.front();
                    const char *DataButtons = DatabuttonsString.c_str();

                    std::istringstream iss(DataButtons);
                    std::vector<std ::string> Items{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
                    uint8_t buttonNum = std::stoi(Items[0]);
                    uint32_t TimeButton = std::stoi(Items[1]);
                    uint32_t LatitudeButton = std::stoi(Items[2]);
                    uint32_t LongitudeButton = std::stoi(Items[3]);
                

                    txBuffer[Positons(1, i)] = buttonNum & 0xFF;
                    txBuffer[Positons(2, i)] = (TimeButton >> 16) & 0xFF;
                    txBuffer[Positons(3, i)] = (TimeButton >> 8) & 0xFF;
                    txBuffer[Positons(4, i)] = TimeButton & 0xFF;
                    txBuffer[Positons(5, i)] = (LatitudeButton >> 16) & 0xFF;
                    txBuffer[Positons(6, i)] = (LatitudeButton >> 8) & 0xFF;
                    txBuffer[Positons(7, i)] = LatitudeButton & 0xFF;
                    txBuffer[Positons(8, i)] = (LongitudeButton >> 16) & 0xFF;
                    txBuffer[Positons(9, i)] = (LongitudeButton >> 8) & 0xFF;
                    txBuffer[Positons(10, i)] = LongitudeButton & 0xFF;
                    DATA.erase(DATA.begin());
                }
            }
        }
        else
        {
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
            for (size_t i = 13; i < 67; i++)
            {
                txBuffer[i] = 0;
            }
        }
    }
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
