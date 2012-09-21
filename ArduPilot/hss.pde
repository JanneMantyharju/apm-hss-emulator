/*
 * Functions for sending telemetry data via SPI to Hitec sensor station emulator
 *
 * Code: Janne MŠntyharju
 */

#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM1
#define HSS_SS 60      // PF6, chip select
#endif

enum hss_states {
	hss_start = 0,
	hss_temp,
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

typedef union {
        long l;
        float f;
        int i;
        unsigned int ui;
        unsigned char uc;
        byte buf[4];
} UNI;

static byte hss_state = 0;
static UNI u;

void hss_setup()
{
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM1
	pinMode(HSS_SS, OUTPUT);
#endif
}

inline static void hss_send_byte(char c)
{
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
	Serial2.write(c);
#else
	SPI.transfer(c);
#endif
}

void hss_send_data(byte start)
{
	byte i;

#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM1
	digitalWrite(HSS_SS, LOW);
#endif

	if(start) {
		hss_send_byte('*');
		hss_send_byte('*');
	} else {
		for(i = 0; i < 4; i++) {
			hss_send_byte(u.buf[i]);
		}
	}
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM1
	digitalWrite(HSS_SS, HIGH);
#endif

}

void hss_update()
{
	switch (hss_state) {
		case hss_start:
			hss_send_data(true);
			break;
		case hss_temp:
			u.i = barometer.get_temperature();
			break;
		case hss_voltage:
			u.f = battery_voltage1;
			break;
		case hss_amps:
			u.f = current_amps1;
			break;
		case hss_current_total:
			u.f = current_total1;
			break;
		case hss_battery_remaining:
			u.uc = (5.0 * (g.pack_capacity - current_total1) / g.pack_capacity);
			break;
		case hss_alt:
			u.ui = current_loc.alt;
			break;
		case hss_groundspeed:
			u.l = g_gps->ground_speed;
			break;
		case hss_num_satellites:
			u.uc = g_gps->num_sats;
			break;
		case hss_airspeed:
			u.i = airspeed.get_airspeed_cm();
			break;
		case hss_throttle:
			u.ui = g.channel_throttle.servo_out;
			break;
		case hss_longitude:
			u.l = g_gps->longitude;
			break;
		case hss_latitude:
			u.l = g_gps->latitude;
			break;
	}

	if (hss_state != hss_start)
		hss_send_data(false);

	hss_state++;
	if (hss_state == hss_end)
		hss_state = 0;
}
