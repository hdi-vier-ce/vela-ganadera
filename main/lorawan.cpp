/*

lorawan module

Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

This code requires the MCCI LoRaWAN LMIC library
by IBM, Matthis Kooijman, Terry Moore, ChaeHee Won, Frank Rose
https://github.com/mcci-catena/arduino-lmic

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
#include <hal/hal.h>
#include <SPI.h>
#include <vector>
#include <Preferences.h>
#include "configuration.h"
#include "credentials.h"
#include "lorawan.h"
#include "screen.h"
#include <queue>
#include "FS.h"
#include "LittleFS.h"
#include <flash.h>
#include <iostream>
#include <sstream>
#include <iterator>

#define FAILED_DATA_FILE "/failedData.txt"
// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// LMIC GPIO configuration
const lmic_pinmap lmic_pins = {
    .nss = NSS_GPIO,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RESET_GPIO,
    .dio = {DIO0_GPIO, DIO1_GPIO, DIO2_GPIO},
};

// Message counter, stored in RTC memory, survives deep sleep.
static RTC_DATA_ATTR uint32_t count = 0;

#ifdef USE_ABP
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}
#endif

#ifdef USE_OTAA
void os_getArtEui(u1_t *buf)
{
    memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui(u1_t *buf) { memcpy(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }
#endif

std::vector<void (*)(uint8_t message)> _lmic_callbacks;

// -----------------------------------------------------------------------------
// Private methods
// -----------------------------------------------------------------------------

void lorawan_sf(unsigned char sf);

void _lorawan_callback(uint8_t message)
{
    for (uint8_t i = 0; i < _lmic_callbacks.size(); i++)
    {
        (_lmic_callbacks[i])(message);
    }
}

void forceTxSingleChannelDr()
{
// Disables all channels, except for the one defined by SINGLE_CHANNEL_GATEWAY
// This only affects uplinks; for downlinks the default
// channels or the configuration from the OTAA Join Accept are used.
#ifdef SINGLE_CHANNEL_GATEWAY
    for (int i = 0; i < 9; i++)
    { // For EU; for US use i<71
        if (i != SINGLE_CHANNEL_GATEWAY)
        {
            LMIC_disableChannel(i);
        }
    }
#endif

    // Set data rate (SF) and transmit power for uplink
    lorawan_sf(LORAWAN_SF);
}

// DevEUI generator using devices's MAC address - from https://github.com/cyberman54/ESP32-Paxcounter/blob/master/src/lorawan.cpp
void gen_lora_deveui(uint8_t *pdeveui)
{
    uint8_t *p = pdeveui, dmac[6];
    int i = 0;
    esp_efuse_mac_get_default(dmac);
    // deveui is LSB, we reverse it so lorawan DEVEUI display
    // will remain the same as MAC address
    // MAC is 6 bytes, devEUI 8, set first 2 ones
    // with an arbitrary value
    *p++ = 0xFF;
    *p++ = 0xFE;
    // Then next 6 bytes are mac address reversed
    for (i = 0; i < 6; i++)
    {
        *p++ = dmac[5 - i];
    }
}

static void printHex2(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

#ifdef USE_OTAA
// generate DevEUI from macaddr if needed
void initDevEUI()
{
    bool needInit = true;
    for (int i = 0; i < sizeof(DEVEUI); i++)
        if (DEVEUI[i])
            needInit = false;

    if (needInit)
        gen_lora_deveui(DEVEUI);

    Serial.print("DevEUI: ");
    for (int i = 0; i < sizeof(DEVEUI); i++)
    {
        if (i != 0)
            Serial.print("-");
        printHex2(DEVEUI[i]);
    }
    Serial.println();
}
#endif
// LMIC library will call this method when an event is fired
bool join;
void onEvent(ev_t event)
{
    switch (event)
    {
    case EV_JOINED:
    {
#ifdef SINGLE_CHANNEL_GATEWAY
        forceTxSingleChannelDr();
#endif

        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
        if (!LORAWAN_ADR)
        {
            LMIC_setLinkCheckMode(0); // Link check problematic if not using ADR. Must be set after join
        }

        Serial.println(F("EV_JOINED"));
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i)
        {
            if (i != 0)
                Serial.print("-");
            printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i)
        {
            if (i != 0)
                Serial.print("-");
            printHex2(nwkKey[i]);
        }
        Serial.println();

        Preferences p;
        if (p.begin("lora", false))
        {
            p.putUInt("netId", netid);
            p.putUInt("devAddr", devaddr);
            p.putBytes("nwkKey", nwkKey, sizeof(nwkKey));
            p.putBytes("artKey", artKey, sizeof(artKey));
            p.end();
        }

        break;
    }
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (inc. RX win. wait)"));
        if (LMIC.txrxFlags & TXRX_ACK)
        { // confirmed Up are acked
            Serial.println(F("Received ack"));
            _lorawan_callback(EV_ACK);
        }
        else
        {
            Serial.println(F("failed to receive ack"));
            _lorawan_callback(EV_FAILED);
        }
        if (LMIC.dataLen)
        {
            Serial.print(F("Data Received: "));
            Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
            Serial.println();
            _lorawan_callback(EV_RESPONSE);
        }
        break;
    default:
        break;
    }

    // Send message callbacks
    _lorawan_callback(event);
}


// -----------------------------------------------------------------------------
// Public methods
// -----------------------------------------------------------------------------

void lorawan_register(void (*callback)(uint8_t message))
{
    _lmic_callbacks.push_back(callback);
}

size_t lorawan_response_len()
{
    return LMIC.dataLen;
}

void lorawan_response(uint8_t *buffer, size_t len)
{
    for (uint8_t i = 0; i < LMIC.dataLen; i++)
    {
        buffer[i] = LMIC.frame[LMIC.dataBeg + i];
    }
}

// If the value for LORA packet counts is unknown, restore from flash
static void initCount()
{
    if (count == 0)
    {
        Preferences p;
        if (p.begin("lora", true))
        {
            count = p.getUInt("count", 0);
            p.end();
        }
    }
}

bool lorawan_setup()
{
    initCount();

    LittleFS.begin();
    // LittleFS.format();
    // Serial.println("LittleFS is init ");

#if defined(USE_OTAA)
    initDevEUI();
#endif

    // SPI interface
    SPI.begin(SCK_GPIO, MISO_GPIO, MOSI_GPIO, NSS_GPIO);

    // LMIC init
    bool InitLMIC = 1 == os_init_ex((const void *)&lmic_pins);
    return InitLMIC;
}
std::queue<String> Queue;
void SendFailedData()
{

    Queue = readFile(LittleFS, FAILED_DATA_FILE);
    while (!Queue.empty())
    {
        String data = Queue.front();
        Queue.pop();
        // std::string data;
        Serial.println("sending data to LoRaWAN");
        Serial.println(data);

        // char* chardata = const_cast<char*>(data.c_str());

        // uint8_t* Failed = reinterpret_cast< uint8_t*>(chardata);
        const char *c = data.c_str();
        std::istringstream iss(c);
        std::vector<std ::string> tokens{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
        double latt = std::stod(tokens[0]);
        double loong = std::stod(tokens[1]);
        uint16_t altt = std::stoi(tokens[2]);
        uint8_t hdp = std::stoi(tokens[3]);
        uint8_t sta = std::stoi(tokens[4]);
        unsigned int hr = std::stoi(tokens[5]);
        unsigned int Mi = std::stoi(tokens[6]);
        unsigned int Sec = std::stoi(tokens[7]);
        uint8_t BPc = std::stoi(tokens[8]);
        uint8_t Bs = std::stoi(tokens[9]);
        uint32_t LatB = ((latt + 90.0) / 180.0) * 16777215.0;
        uint32_t LongB = ((loong + 180.0) / 360.0) * 16777215.0;
        uint32_t Tb = ((hr * 3600) + (Mi * 60) + Sec);
        uint8_t BPB = (BPc / 100) * 255;
        uint8_t Failed[18];

        Failed[0] = (LatB >> 16) & 0xFF;
        Failed[1] = (LatB >> 8) & 0xFF;
        Failed[2] = LatB & 0xFF;
        Failed[3] = (LongB >> 16) & 0xFF;
        Failed[4] = (LongB >> 8) & 0xFF;
        Failed[5] = LongB & 0xFF;
        Failed[6] = (altt >> 8) & 0xFF;
        Failed[7] = altt & 0xFF;
        Failed[8] = hdp & 0xFF;
        Failed[9] = sta & 0xFF;
        Failed[10] = (Tb >> 16) & 0xFF;
        Failed[11] = (Tb >> 8) & 0xFF;
        Failed[12] = Tb & 0xFF;
        Failed[13] = BPB & 0xFF;
        Failed[14] = Bs & 0xFF;

        lorawan_send(Failed, sizeof(Failed), LORAWAN_PORT, true);
        delay(49000);
    }
}

void lorawan_join()
{
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

#ifdef CLOCK_ERROR
    LMIC_setClockError(MAX_CLOCK_ERROR * CLOCK_ERROR / 100);
#endif

#if defined(CFG_eu868)

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

#elif defined(CFG_us915)

    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    // in the US, with TTN, it saves join time if we start on subband 1
    // (channels 8-15). This will get overridden after the join by
    // parameters from the network. If working with other networks or in
    // other regions, this will need to be changed.
    LMIC_selectSubBand(1);

#elif defined(CFG_au915)

    // set sub band for AU915
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/AU-global_conf.json
    LMIC_selectSubBand(1);

#endif

    // defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

#ifdef SINGLE_CHANNEL_GATEWAY
    forceTxSingleChannelDr();
#else
    // Set default rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    lorawan_sf(LORAWAN_SF);
#endif

#if defined(USE_ABP)

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Trigger a false joined
    _lorawan_callback(EV_JOINED);

#elif defined(USE_OTAA)

    // Make LMiC initialize the default channels, choose a channel, and
    // schedule the OTAA join
    LMIC_startJoining();

#ifdef SINGLE_CHANNEL_GATEWAY
    // LMiC will already have decided to send on one of the 3 default
    // channels; ensure it uses the one we want
    LMIC.txChnl = SINGLE_CHANNEL_GATEWAY;
#endif

    Preferences p;
    p.begin("lora", true); // we intentionally ignore failure here
    uint32_t netId = p.getUInt("netId", UINT32_MAX);
    uint32_t devAddr = p.getUInt("devAddr", UINT32_MAX);
    uint8_t nwkKey[16], artKey[16];
    bool keysgood = p.getBytes("nwkKey", nwkKey, sizeof(nwkKey)) == sizeof(nwkKey) &&
                    p.getBytes("artKey", artKey, sizeof(artKey)) == sizeof(artKey);
    p.end(); // close our prefs

    if (!keysgood)
    {
        // We have not yet joined a network, start a full join attempt
        // Make LMiC initialize the default channels, choose a channel, and
        // schedule the OTAA join
        Serial.println("No session saved, joining from scratch");
        LMIC_startJoining();
    }
    else
    {
        Serial.println("Rejoining saved session");
        LMIC_setSession(netId, devAddr, nwkKey, artKey);

        // Trigger a false joined
        _lorawan_callback(EV_JOINED);
    }

#endif
}

void lorawan_sf(unsigned char sf)
{
    LMIC_setDrTxpow(sf, 14);
}

void lorawan_adr(bool enabled)
{
    LMIC_setAdrMode(enabled);
    LMIC_setLinkCheckMode(enabled);
}

uint32_t lorawan_get_count()
{
    return count;
}

static void lorawan_set_cnt()
{
    LMIC_setSeqnoUp(count);

    // We occasionally mirror our count to flash, to ensure that if we lose power we will at least start with a count that is almost correct
    // (otherwise the TNN network will discard packets until count once again reaches the value they've seen).  We limit these writes to a max rate
    // of one write every 5 minutes.  Which should let the FLASH last for 300 years (given the ESP32 NVS algoritm)
    static uint32_t lastWriteMsec = UINT32_MAX; // Ensure we write at least once
    uint32_t now = millis();
    if (now < lastWriteMsec || (now - lastWriteMsec) > 5 * 60 * 1000L)
    { // write if we roll over (50 days) or 5 mins
        lastWriteMsec = now;

        Preferences p;
        if (p.begin("lora", false))
        {
            p.putUInt("count", count);
            p.end();
        }
    }
}

/// Blow away our prefs (i.e. to rejoin from scratch)
void lorawan_erase_prefs()
{
    Preferences p;
    if (p.begin("lora", false))
    {
        p.clear();
        p.end();
    }
}

void lorawan_send(uint8_t *data, uint8_t data_size, uint8_t port, bool confirmed)
{
    lorawan_set_cnt(); // we are about to send using the current packet count

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        _lorawan_callback(EV_PENDING);

        return;
    }
    if (LMIC.txrxFlags & TXRX_NACK) // confirmed UP frame was not acked and the TX-RX combo ---> for cheking the failure
    {
        Serial.println(F("Failure in sending "));
        screen_print("Failed to send \n ");
        _lorawan_callback(EV_FAILED);
        uint32_t lat = (data[0] << 16) | (data[1] << 8) | data[2];
        double Latdeg = ((lat / 16777215.0) * 180.0) - 90;
        uint32_t longg = (data[3] << 16) | (data[4] << 8) | data[5];
        double Longdeg = ((longg / 16777215.0) * 360.0) - 180;
        uint16_t alt = (data[6] << 16) | (data[7] << 8);
        uint8_t Hdop = data[8];
        uint8_t stat = data[9];
        uint32_t time = (data[10] << 16) | (data[11] << 8) | data[12];
        unsigned int Hour = time / 3600;
        unsigned int Min = (time % 3600) / 60;
        unsigned int Sec = time % 60;
        unsigned int Bat = data[13];
        uint8_t BattP = static_cast<int>((Bat / 255.0) * 100);
        uint8_t BattS = data[14];
        char LatdegS[16];
        char LongdegS[16];
        char altS[16];
        char HdoopS[16];
        char statS[16];
        char HourS[16];
        char MinS[16];
        char SecS[16];
        char BattPS[16];
        char BattSS[16];

        dtostrf(Latdeg, 8, 6, LatdegS);
        dtostrf(Longdeg, 8, 6, LongdegS);
        itoa(alt, altS, 10);
        itoa(Hdop, HdoopS, 10);
        itoa(stat, statS, 10);
        itoa(Hour, HourS, 10);
        itoa(Min, MinS, 10);
        itoa(Sec, SecS, 10);
        itoa(BattP, BattPS, 10);
        itoa(BattS, BattSS, 10);

        

        std::string FailData = std::string(LatdegS) + " " + std::string(LongdegS) + " " + std::string(altS) + " " + std::string(HdoopS) + " " + std::string(statS) + " " + std::string(HourS) + " " + std::string(MinS) + " " + std::string(SecS) + " " + std::string(BattPS) + " " + std::string(BattSS) + '\n';
        const char *dst = FailData.c_str();


        appendFile(LittleFS, FAILED_DATA_FILE, dst);

        _lorawan_callback(EV_QUEUED);
        count++;

        return;
    }

    // Prepare upstream data transmission at the next possible time.
    // Parameters are port, data, length, confirmed
    LMIC_setTxData2(port, data, data_size, confirmed ? 1 : 0);
    _lorawan_callback(EV_QUEUED);

    count++;
}


void lorawan_loop()
{
    os_runloop_once();
}
