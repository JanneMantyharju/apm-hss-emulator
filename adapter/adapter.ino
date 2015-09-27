/*
 * Code by Janne Mäntyharju
 * Author note for HSS emulation part: Author Kim Mattsson, base from  Kroeske
 * MavLink code borrowed from MinimOsd project
 *
 * NB! If using Atmega168, change MAVLINK_MAX_PAYLOAD_LEN to 35 in include/mavlink/v1.0/mavlink_types.h
 * Display setup:
 * RPM 1: Distance to home
 * RPM 2: Airspeed
 * TEMP 1: Battery remaining percentage
 * TEMP 2: Throttle position
 * TEMP 3: Variometer
 * TEMP 4: Armed status (1 = Armed)
 */

#include <FastSerial.h>
#include "Wire.h"
#include <GCS_MAVLink.h>
#include <SimpleTimer.h>
#include <DigitalToggle.h>

#define Hitec_i2c  0x08         // Hitec Telemetry module address
#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port
#define SPEKTRUM_MESSAGES 4
#define HITEC_MESSAGES 8
#define HITEC 0
#define SPEKTRUM 1
#define LED_GREEN 8
#define LED_RED 9

FastSerialPort0(Serial);

SimpleTimer  mavTimer;
SimpleTimer  homeTimer;

static uint8_t      got_home = 0;
static uint8_t      fix_type = 0;
static float        lat = 0;                    // latidude
static float        lon = 0;                    // longitude
static float        home_lat = 0;               // home latidude
static float        home_lon = 0;               // home longitude
static float        home_alt = 0;
static uint8_t      alt_cnt = 0;
static float        alt_prev = 0;             // previous altitude
static float        alt = 0;
static float        climb = 0.0;
static float        alpha = 0.8;                // value for exponential moving average

static uint8_t msgId = 0;
static uint8_t telemetry_type = HITEC;

byte data_spektrum[SPEKTRUM_MESSAGES][16] = {
  {0x03, 00, 00, 50, 0x00, 0x00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00},   // 2-3 Current, unsigned 1 unit is 0.1967A
  {0x0a, 00, 00, 50, 0x00, 0x00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00}, // 2-3 Volts, unsigned 1 unit is 0.01v, 6-7 capacity, unsigned 1 unit is 1mAh
  {0x11, 00, 00, 50, 0x01, 0xF9, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00}, // 2-3 airspeed unsigned 1 unit is 1 km/h
  {0x12, 00, 00, 50, 0xFF, 0x00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00} // 2-3 altitude, signed 1 unit is 0.1 m
};

byte data_hss[HITEC_MESSAGES][7] = { 
  {0x11, 0xAF, 0x00, 0x2D, 0x00, 0x00, 0x11}, // 1 Frametype, 4&5 Internal  SPC voltage
  {0x12, 0x00, 0x2C, 0x2C, 0x2E, 0x25, 0x12},     // 1-4 Latitude 5=gps sec,
  {0x13, 0x18, 0x01, 0xD4, 0x22, 0x28, 0x13},     // 1-4 Longitude, 5 = TEMP2
  {0x14, 0x00, 0x00, 0x00, 0x00, 0x28, 0x14},     //1&2=Speed,,3&4 = altitude,5= Temp1
  {0x15, 0x03, 0xFF, 0x01, 0xFE, 0x01, 0x15},     // 1 = Fuelgauge, 2&3 RPM1, 4&5 RPM2
  {0x16, 0x3C, 0x04, 0x1D, 0x00, 0x00, 0x16},     // 1= year,m,date, h, min 
  {0x17, 0x00, 0x99, 0x06, 0x28, 0x28, 0x17},     // 3= GPS Signal 4=Temp3, 5=Temp4
  {0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18}      // 1&2 Voltage
};                              

void Send_Data()
{
  digitalToggle(LED_RED);
  if(telemetry_type == HITEC) {
    Wire.write(data_hss[msgId], 7);
    msgId++;
    msgId %= HITEC_MESSAGES;
  } else {
    Wire.write(data_spektrum[msgId], 16);
    msgId++;
    msgId %= SPEKTRUM_MESSAGES;
  }
}

// Works  , nr is channel, and value is rpm
void setRpm(int nr, unsigned int value)
{
    unsigned char rpml = (value / 10) % 0x100;
    unsigned char rpmh = (value / 10) / 0x100;

    switch (nr) {
        case 1:
            data_hss[4][2] = rpml;
            data_hss[4][3] = rpmh;
            break;

        case 2:
            data_hss[4][4] = rpml;
            data_hss[4][5] = rpmh;
            break;

        default:
            break;
    }
}

/*
 *  Valuerange 0-4
 */
void FuelGauge(unsigned char value)
{
    if (value > 4)
        data_hss[4][1] = 4;
    else
        data_hss[4][1] = value;
}

//works, Range is -40 - +215c
void setTemp(int nr, char value)
{
  value+=40;
    switch (nr) {
        case 1:
            data_hss[3][5] = value;
            break;

        case 2:
            data_hss[2][5] = value;
            break;

        case 3:
            data_hss[6][4] = value;
            break;

        case 4:
            data_hss[6][5] = value;

        default:
            break;
    }
}

//  Works Values 0-500km/h
void setSpeed(unsigned int value)
{
  if(telemetry_type == HITEC) {
    if (value <= 500) {
        data_hss[3][2] = value % 0x100;
        data_hss[3][1] = value / 0x100;
    }
  } else {
    data_spektrum[2][2] = value % 0x100;
    data_spektrum[2][3] = value / 0x100;
  }
}

// Value in meters 
void setAltitude(unsigned int value)
{
  if(telemetry_type == HITEC) {
    if (value < 9999) {
        data_hss[3][4] = value % 0x100;
        data_hss[3][3] = value / 0x100;
    }
  } else {
    value *= 10;
    data_spektrum[3][2] = value % 0x100;
    data_spektrum[3][3] = value / 0x100;
  }
}

// Value is 6-1002, Value is 0.1 volt), minimum measurement is 6 that equals to 0.6v
void setVoltage(unsigned int value)
{
  if(telemetry_type == HITEC) {
    if ((value > 6) & (value < 1001)) {
        data_hss[7][1] = (value - 2) % 0x100;;      // % 0x100;
        data_hss[7][2] = value / 0x100;
    }
  } else {
    value *= 0.1;
    data_spektrum[1][2] = value % 0x100;
    data_spektrum[1][3] = value / 0x100;
  }
}

 // value  1 = 0.1A, presented function is close enough, more accurate with float, but don't want to include the huge library
void setAmpere(unsigned int value)
{
  uint32_t t = 0;
  uint16_t v;

  if(telemetry_type == HITEC) {
    if (value) {
        t = (value - 1);
        t = t * 1435;
        t = t / 1000;
    }
    v = (uint16_t) (t) + 167;
    data_hss[7][3] = v % 0x100;;    // % 0x100;
    data_hss[7][4] = v / 0x100;
  } else {
    value *= 10;
    value /= 0.1967;
    data_spektrum[0][2] = value % 0x100;
    data_spektrum[0][3] = value / 0x100;
  }
}

/*
 * longitude and latitude are in GPS decimal, Aurora shows deg.min.sec
 * format is ddmm ss(,)ss
 */
void convertCoords(long *from, int *dm, int *s)
{
    int x;
    
    *dm = *from / 10000000;    
    *from -= *dm * 10000000;
    *from *= 6;
    x = *from / 1000000;
    *dm *= 100;
    *dm += x;
    
    *from -= x * 1000000;
    *s = *from / 100;
}

void setGPSLon(long longitude)
{
    int dm;
    int s;
    
    convertCoords(&longitude, &dm, &s);
    
    data_hss[2][2] = s & 0xff;
    data_hss[2][1] = s >> 8;

    data_hss[2][4] = dm & 0xff;
    data_hss[2][3] = dm >> 8;
}

void setGPSLat(long latitude)
{
    int dm;
    int s;
    
    convertCoords(&latitude, &dm, &s);
        
    data_hss[1][2] = s & 0xff;
    data_hss[1][1] = s >> 8;

    data_hss[1][4] = dm & 0xff;
    data_hss[1][3] = dm >> 8;
}

//Works, shows the signal strength, one bar /satelite, limit to 5
void setGPSSignal(char c)
{
    data_hss[6][3] = c;
}

void setup()
{
    Serial.begin(TELEMETRY_SPEED);
    // setup mavlink port
    mavlink_comm_0_port = &Serial;
    Wire.begin(Hitec_i2c);
    Wire.onRequest(Send_Data);
    if(telemetry_type == SPEKTRUM)
      TWAMR = 0xff; // respond to any request
    
    homeTimer.Set(&setHomeVars, 120);
    homeTimer.Enable();
    mavTimer.Set(&request_mavlink_rates, 1000);
    mavTimer.Enable();

    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, HIGH);
    delay(1000);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);    
}

void setHomeVars()
{
    float dstlon, dstlat;
 
    if(got_home == 0 && fix_type > 1) {
        if(alt_cnt < 25) {
            if(fabs(alt_prev - alt) > 0.5) {
                alt_cnt = 0;
                alt_prev = alt;
            } else {
                if(++alt_cnt >= 25) {
                    home_alt = alt;
                    home_lat = lat;
                    home_lon = lon;
                    got_home = 1;
                }
            }
        }
    } else if(got_home == 1) {
        // shrinking factor for longitude going to poles direction
        float rads = fabs(home_lat) * 0.0174532925;
        double scaleLongDown = cos(rads);

        //DST to Home
        dstlat = fabs(home_lat - lat) * 111319.5;
        dstlon = fabs(home_lon - lon) * 111319.5 * scaleLongDown;
        setRpm(1, sqrt(sq(dstlat) + sq(dstlon)));
    }
}

void loop()
{
    read_mavlink();
    mavTimer.Run();
    homeTimer.Run();
}

