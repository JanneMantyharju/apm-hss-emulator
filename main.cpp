/*
 * Author note for HSS emulation part: Author Kim Mattsson, base from  Kroeske
 * 
 * GPS position and time functions are currently commented out. I'll rewrite them later
 * Serial messages are only for debugging and can be removed-
 * 
 * Code by Janne Mäntyharju
 */


#include "WProgram.h"

#include <Wire/Wire.h>

#define Hitec_i2c  0x08         // Hitec Telemetry module address

unsigned char state = 0;
byte last_char = 0;
byte upper = 0;

enum hss_states {
	hss_temp = 0,
	hss_voltage,
	hss_amps,
	hss_current_total,
	hss_battery_remaining,
	hss_alt,
	hss_groundspeed,
	hss_num_satellites,
	hss_airspeed,
	hss_throttle,
	hss_end
};

char data[8][7] = { {0x11, 0xAF, 0x00, 0x2D, 0x00, 0x00, 0x11}, // 1 Frametype, 4&5 Internal  SPC voltage
{0x12, 0x00, 0x2C, 0x2C, 0x2E, 0x25, 0x12},     // 1-4 Latitude 5=gps sec,
{0x13, 0x18, 0x01, 0xD4, 0x22, 0x28, 0x13},     // 1-4 Longitude, 5 = TEMP2
{0x14, 0x00, 0x00, 0x00, 0x00, 0x28, 0x14},     //1&2=Speed,,3&4 = altitude,5= Temp1
{0x15, 0x00, 0xFF, 0x01, 0xFE, 0x01, 0x15},     // 1 = Fuelgauge, 2&3 RPM1, 4&5 RPM2
{0x16, 0x3C, 0x04, 0x1D, 0x00, 0x00, 0x16},     // 1= year,m,date, h, min 
{0x17, 0x00, 0x99, 0x06, 0x28, 0x28, 0x17},     // 3= GPS Signal 4=Temp3, 5=Temp4
{0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18}
};                              // 1&2 Voltage

extern "C" void __cxa_pure_virtual()
 {
     cli();    // disable interrupts
     for(;;);  // do nothing until hard reset
}

void Send_Hitec()
{
    static unsigned char msgId = 0;

    if (TWAR != (Hitec_i2c << 1))
        return;                 //check if Hitecs telemetry address, someone else could be sending also through IRQ. 

    // We need to send only one Idrow/request, because of receiver  "odd" handshaking that Wire lib not support (TWEA after every row)
    Wire.send((uint8_t *) data + msgId * 7, 7);
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

//works, Values are 0-255, value 0 is -40C 255 =215C
void setTemp(int nr, unsigned char value)
{
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

 // value  1 = 0.1mA, presented function is close enough, more accurate with float, but don't want to include the huge library
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
 */
/*void setGPSValue(long longitude, long latitude)
{
    char x[15];
    int d;
    long m;
    int ne;

    // get rid of minus sign, if exist, creates problems in the sprintf & sscanf
    if (longitude < 0) {
        ne = -1;
        longitude = -longitude;
    } else
        ne = 1;

    sprintf(x, "%09.9lu", longitude);
    sscanf(x, "%2d%7ld", &d, &m);
    m = m * 6;
    sprintf(x + 2, "%07.7lu", m);
    sscanf(x, "%4d%5ld", &d, &m);
    m = m / 10;
    data[2][2] = m & 0xff;
    data[2][1] = m >> 8;

    d *= ne;                    // return the sign, so that N/S E/W shows correcly
    data[2][4] = d & 0xff;
    data[2][3] = d >> 8;

    if (latitude < 0) {
        ne = -1;
        latitude = -latitude;
    } else
        ne = 1;
        
    sprintf(x, "%09.9lu", latitude);
    sscanf(x, "%2d%7ld", &d, &m);
    m = m * 6;
    sprintf(x + 2, "%07.7lu", m);
    sscanf(x, "%4d%5ld", &d, &m);
    m = m / 10;
    data[1][2] = m & 0xff;
    data[1][1] = m >> 8;
    d *= ne;
    data[1][4] = d & 0xff;
    data[1][3] = d >> 8;
}*/

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

SIGNAL(SPI_STC_vect)
{
    byte c = SPDR;
    unsigned int value = 0;

    if (c == '*' && last_char == '*') {
        state = 0;
        upper = 0;
        Serial.println("Reset");
    } else {
        if (!upper) {
            upper = 1;
        } else {
            value = last_char * 0xff;
            value += c;
            upper = 0;

            switch (state) {
                case hss_temp:{
                        Serial.print("temp:");
                        Serial.println(value, DEC);
                        setTemp(1, (value / 10) + 40);
                        break;
                    }
                case hss_voltage:{
                        Serial.print("bat:");
                        Serial.println(value, DEC);
                        setVoltage(value);
                        break;
                    }
                case hss_amps:{
                        Serial.print("amp:");
                        Serial.println(value, DEC);
                        setAmpere(value);
                        break;
                    }
                case hss_current_total:{       // total current
                        Serial.print("total:");
                        Serial.println(value, DEC);
                        setRpm(1, value);
                        break;
                    }
                case hss_battery_remaining:{
                        Serial.print("fuel:");
                        Serial.println(value, DEC);
                        FuelGauge(value);
                        break;
                    }
                case hss_alt:{
                        Serial.print("alt:");
                        Serial.println(value, DEC);
                        setAltitude(value);
                        break;
                    }
                case hss_groundspeed:{
                        Serial.print("groundspeed:");
                        Serial.println(value, DEC);
                        setSpeed(value);
                        break;
                    }
                case hss_num_satellites:{
                        Serial.print("gps:");
                        Serial.println(value, DEC);
                        setGPSSignal(value);
                        break;
                    }
                case hss_airspeed:{
                        Serial.print("air:");
                        Serial.println(value, DEC);
                        setRpm(2,value);
                        break;
                    }
                case hss_throttle:{
                        Serial.print("throttle:");
                        Serial.println(value, DEC);  // throttle
                        float t = (value / 12670.0)*100.0;
                        setTemp(2,(unsigned int)t/2+40.0);
                        break;
                    }
                default:
                    Serial.print("unknown:");
                    Serial.println(value, DEC);
                    break;
            }

            state++;
        }
    }

    last_char = c;
}

void setup()
{
    Serial.begin(115200);
    SPCR = _BV(SPE) | _BV(SPIE) | _BV(CPOL) | _BV(CPHA); // Enable SPI & Interrupt, Mode 3
    Wire.begin(Hitec_i2c);
    Wire.onRequest(Send_Hitec);
}

void loop()
{

}

int main(){
       init();
       setup();
       for(;;) loop();
       return 0;
}
