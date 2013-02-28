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

#define Hitec_i2c  0x08         // Hitec Telemetry module address
#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port
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

char data[8][7] = { {0x11, 0xAF, 0x00, 0x2D, 0x00, 0x00, 0x11}, // 1 Frametype, 4&5 Internal  SPC voltage
{0x12, 0x00, 0x2C, 0x2C, 0x2E, 0x25, 0x12},     // 1-4 Latitude 5=gps sec,
{0x13, 0x18, 0x01, 0xD4, 0x22, 0x28, 0x13},     // 1-4 Longitude, 5 = TEMP2
{0x14, 0x00, 0x00, 0x00, 0x00, 0x28, 0x14},     //1&2=Speed,,3&4 = altitude,5= Temp1
{0x15, 0x03, 0xFF, 0x01, 0xFE, 0x01, 0x15},     // 1 = Fuelgauge, 2&3 RPM1, 4&5 RPM2
{0x16, 0x3C, 0x04, 0x1D, 0x00, 0x00, 0x16},     // 1= year,m,date, h, min 
{0x17, 0x00, 0x99, 0x06, 0x28, 0x28, 0x17},     // 3= GPS Signal 4=Temp3, 5=Temp4
{0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18}
};                              // 1&2 Voltage

void Send_Hitec()
{
    static unsigned char msgId = 0;
    
    if (TWAR != (Hitec_i2c << 1))
        return;                 //check if Hitecs telemetry address, someone else could be sending also through IRQ. 

    // We need to send only one Idrow/request, because of receiver  "odd" handshaking that Wire lib not support (TWEA after every row)
    Wire.write((uint8_t *) data + msgId * 7, 7);
    msgId++;
    msgId %= 8;
}

// Works  , nr is channel, and value is rpm
void setRpm(int nr, unsigned int value)
{
    unsigned char rpml = (value / 10) % 0x100;
    unsigned char rpmh = (value / 10) / 0x100;

    switch (nr) {
        case 1:
            data[4][2] = rpml;
            data[4][3] = rpmh;
            break;

        case 2:
            data[4][4] = rpml;
            data[4][5] = rpmh;
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
        data[4][1] = 4;
    else
        data[4][1] = value;
}

//works, Range is -40 - +215c
void setTemp(int nr, char value)
{
  value+=40;
    switch (nr) {
        case 1:
            data[3][5] = value;
            break;

        case 2:
            data[2][5] = value;
            break;

        case 3:
            data[6][4] = value;
            break;

        case 4:
            data[6][5] = value;

        default:
            break;
    }
}

//  Works Values 0-500km/h
void setSpeed(unsigned int value)
{
    if (value <= 500) {
        data[3][2] = value % 0x100;
        data[3][1] = value / 0x100;
    }
}

// Value in meters 
void setAltitude(unsigned int value)
{
    if (value < 9999) {
        data[3][4] = value % 0x100;
        data[3][3] = value / 0x100;
    }
}

// Value is 6-1002, Value is 0.1 volt), minimum measurement is 6 that equals to 0.6v
void setVoltage(unsigned int value)
{
    if ((value > 6) & (value < 1001)) {
        data[7][1] = (value - 2) % 0x100;;      // % 0x100;
        data[7][2] = value / 0x100;
    }
}

 // value  1 = 0.1A, presented function is close enough, more accurate with float, but don't want to include the huge library
void setAmpere(unsigned int value)
{
    uint32_t t = 0;

    uint16_t v;

    if (value) {
        t = (value - 1);
        t = t * 1435;
        t = t / 1000;
    }
    v = (uint16_t) (t) + 167;
    data[7][3] = v % 0x100;;    // % 0x100;
    data[7][4] = v / 0x100;
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
    
    data[2][2] = s & 0xff;
    data[2][1] = s >> 8;

    data[2][4] = dm & 0xff;
    data[2][3] = dm >> 8;
}

void setGPSLat(long latitude)
{
    int dm;
    int s;
    
    convertCoords(&latitude, &dm, &s);
        
    data[1][2] = s & 0xff;
    data[1][1] = s >> 8;

    data[1][4] = dm & 0xff;
    data[1][3] = dm >> 8;
}

// reguires date in format yymmdd as a long
/*void setGPSDate(long t)
{
    char x[15];
    
    sprintf(x, "%06.6lu", t);
    data[5][1] = (x[0] - 48) * 10 + x[1] - 48;  //yy
    data[5][2] = (x[2] - 48) * 10 + x[3] - 48;  //mm
    data[5][3] = (x[4] - 48) * 10 + x[5] - 48;  //dd
}*/



// Works, Time in t, format hhmmssmss
/*
void setGPSTime(long t)
{

    char x[15];

    sprintf(x, "%09.9lu", t);
    data[5][4] = (x[0] - 48) * 10 + x[1] - 48;
    data[5][5] = (x[2] - 48) * 10 + x[3] - 48;
    data[1][5] = (x[4] - 48) * 10 + x[5] - 48;
}*/



//Works, shows the signal strength, one bar /satelite, limit to 5
void setGPSSignal(char c)
{
    data[6][3] = c;
}

void setup()
{
    Serial.begin(TELEMETRY_SPEED);
    // setup mavlink port
    mavlink_comm_0_port = &Serial;
    Wire.begin(Hitec_i2c);
    Wire.onRequest(Send_Hitec);
    
    homeTimer.Set(&setHomeVars, 120);
    homeTimer.Enable();
    mavTimer.Set(&request_mavlink_rates, 1000);
    mavTimer.Enable();

    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
}

void setHomeVars()
{
    float dstlon, dstlat;
 
    if(got_home == 0 && fix_type > 1) {
        home_lat = lat;
        home_lon = lon;
        got_home = 1;
    } else if(got_home == 1) {
        if(alt_cnt < 25) {
            if(fabs(alt_prev - alt) > 0.5) {
                alt_cnt = 0;
                alt_prev = alt;
            } else {
                if(++alt_cnt >= 25) {
                    home_alt = alt;
                }
            }
        }
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

