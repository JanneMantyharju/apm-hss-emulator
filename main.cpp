/*
 * Author note for HSS emulation part: Author Kim Mattsson, base from  Kroeske
 * 
 * GPS position and time functions are currently commented out. I'll rewrite them later.
 * Serial messages are only for debugging and can be removed.
 * 
 * Code by Janne Mäntyharju
 */


#include "WProgram.h"

#include <Wire/Wire.h>

#define Hitec_i2c  0x08         // Hitec Telemetry module address
//#define USE_SPI

typedef union {
        long l;
	float f;
        int i;
        unsigned int ui;
        unsigned char uc;
        byte buf[4];
} UNI;

byte state = 0;
UNI u;

void handle_byte(byte);

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
    hss_longitude,
    hss_latitude,
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
    Serial.begin(38400);
#ifdef USE_SPI
    SPCR = _BV(SPE) | _BV(CPOL) | _BV(CPHA); // Enable SPI, Mode 3
#endif
    Wire.begin(Hitec_i2c);
    Wire.onRequest(Send_Hitec);
    DDRB = _BV(DDB0);
}

byte get_data()
{
	byte i;
	byte last_char = 0;
	byte c;
	
	for(i = 0; i < 4; i++) {
#ifdef USE_SPI
        loop_until_bit_is_set(SPSR, SPIF);
        c = SPDR;
#else
		while (!Serial.available());
		c = Serial.read();
#endif
		if (last_char == '*' && c == '*')
			return 0;
		u.buf[i] = c;
		last_char = c;
	}
	
	return 1;
}

void loop()
{	
	if (!get_data()) {
		state = 0;
		Serial.println("**");
		return;
	}

	switch (state) {
		case hss_temp:{
			Serial.print("temp:");
			u.i /= 10;
			Serial.println(u.i, DEC);
			setTemp(1, u.i + 40);
			break;
		}
		case hss_voltage:{
			Serial.print("bat:");
			Serial.println(u.f, DEC);
			u.f *= 10.0;
			setVoltage((unsigned int)u.f);
			break;
		}
		case hss_amps:{
			Serial.print("amp:");
			Serial.println(u.f, DEC);
			u.f *= 100.0;
			setAmpere((unsigned int)u.f);
			break;
		}
		case hss_current_total:{       // total current
			Serial.print("total:");
			Serial.println(u.f, DEC);
			u.f *= 100.0;
			setRpm(1, (unsigned int)u.f);
			break;
		}
		case hss_battery_remaining:{
			Serial.print("fuel:");
			Serial.println(u.uc, DEC);
			FuelGauge(u.uc);
			break;
		}
		case hss_alt:{
			Serial.print("alt:");
			u.ui /= 100;
			Serial.println(u.ui, DEC);
			setAltitude(u.ui);
			break;
		}
		case hss_groundspeed:{
			Serial.print("gpsSpeed:");
			u.l *= 0.036;
	                Serial.println(u.l, 2);
			setSpeed(u.l);
			break;
		}
		case hss_num_satellites:{
			Serial.print("gps:");
			Serial.println(u.uc, DEC);
			setGPSSignal(u.uc);
			if(u.uc > 5)
				PORTB |= _BV(PORTB0);
			else
				PORTB &= ~_BV(PORTB0);
			break;
		}
		case hss_airspeed:{
			Serial.print("air:");
			u.l *= 0.036;
			Serial.println(u.i, DEC);
			setRpm(2,u.i);
			break;
		}
		case hss_throttle:{
			Serial.print("throttle:");
			Serial.println(u.ui, DEC);  // throttle
			setTemp(2,u.ui+40);
			break;
		}
        case hss_longitude:{
            Serial.print("lon:");
            Serial.println(u.l, DEC);
            setGPSLon(u.l);
            break;
        }
        case hss_latitude:{
            Serial.print("lat:");
            Serial.println(u.l, DEC);
            setGPSLat(u.l);
            break;
        }
		default:{
			Serial.print("err:");
			Serial.println(u.ui, DEC);
			break;
		}
	}

	state++;
}

int main(){
       init();
       setup();
       for(;;) loop();
       return 0;
}
