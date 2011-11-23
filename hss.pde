/*
 * Functions for sending telemetry data via SPI to Hitec sensor station emulator
 *
 * Code: Janne MŠntyharju
 */


#define HSS_SS 60	// PF6, chip select

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
	hss_end
};

static byte hss_state = 0;

void hss_setup()
{
	pinMode(HSS_SS, OUTPUT);
}

void hss_update()
{
	unsigned int v;

	digitalWrite(HSS_SS, LOW);
	delay(1);

	switch (hss_state) {
		case hss_start:
			SPI.transfer('*');
			SPI.transfer('*');
			break;
		case hss_temp:
			SPI.transfer(barometer.Temp >> 8);
			SPI.transfer(barometer.Temp);
			break;
		case hss_voltage:
			v = (battery_voltage * 10.0);
			SPI.transfer(v >> 8);
			SPI.transfer(v);
			break;
		case hss_amps:
			v = (current_amps * 10.0);
			SPI.transfer(v >> 8);
			SPI.transfer(v);
			break;
		case hss_current_total:
			v = current_total;
			SPI.transfer(v >> 8);
			SPI.transfer(v);
			break;
		case hss_battery_remaining:
			v = (5.0 * (g.pack_capacity - current_total) / g.pack_capacity);;
			SPI.transfer(0);
			SPI.transfer(v);
			break;
		case hss_alt:
			v = current_loc.alt / 100.0;
			SPI.transfer(v >> 8);
			SPI.transfer(v);
			break;
		case hss_groundspeed:
			v = g_gps->ground_speed * 0.036;	// convert to km/h
			SPI.transfer(v >> 8);
			SPI.transfer(v);
			break;
		case hss_num_satellites:
			SPI.transfer(0);
			SPI.transfer(g_gps->num_sats);
			break;
		case hss_airspeed:
			v = airspeed * 0.036;	// convert to km/h
			SPI.transfer(v);
			SPI.transfer(v);
			break;
		case hss_throttle:
			v = g.channel_throttle.servo_out;
			SPI.transfer(v);
			SPI.transfer(v);
			break;
	}
	digitalWrite(HSS_SS, HIGH);

	hss_state++;
	if (hss_state == hss_end)
		hss_state = 0;
}
