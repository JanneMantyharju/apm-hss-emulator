#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

static uint8_t      apm_mav_system; 
static uint8_t      apm_mav_component;
static uint8_t      streams_received = 0;

void request_mavlink_rates()
{
    const int  maxStreams = 2;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS,
                                            MAV_DATA_STREAM_EXTRA2};
    const uint16_t MAVRates[maxStreams] = {0x02, 0x02};
    for (int i=0; i < maxStreams; i++) {
        mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
                                             apm_mav_system, apm_mav_component,
                                             MAVStreams[i], MAVRates[i], 1);
    }
}

void disableTx()
{
    mavTimer.Disable();
    UCSR0B &= ~_BV(TXEN0);
}

void read_mavlink() {
    mavlink_message_t msg; 
    mavlink_status_t status;

    //grabing data 
    while(Serial.available() > 0) {
        uint8_t c = Serial.read();
        //trying to grab msg
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            digitalToggle(LED_GREEN);       
            //handle msg
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    streams_received |= _BV(1);
                    apm_mav_system = msg.sysid;
                    apm_mav_component = msg.compid;

                    if(mavlink_msg_heartbeat_get_base_mode(&msg) & (1 << 7)) {
                        setTemp(4,1);
                    } else {
                        setTemp(4,0);
                    }
                    break;
                }
                
                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    streams_received |= _BV(2);
                    setVoltage((mavlink_msg_sys_status_get_voltage_battery(&msg) / 100.0f));
                    setAmpere(mavlink_msg_sys_status_get_current_battery(&msg) / 10.0f);
                    setTemp(1, mavlink_msg_sys_status_get_battery_remaining(&msg));
                    FuelGauge((mavlink_msg_sys_status_get_battery_remaining(&msg) / 100.0f) * 5.0f); //Remaining battery energy: (0%: 0, 100%: 100)
                    break;
                }               
        
                case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    streams_received |= _BV(3);
                    lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
                    lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
                    setGPSLat(mavlink_msg_gps_raw_int_get_lat(&msg));
                    setGPSLon(mavlink_msg_gps_raw_int_get_lon(&msg));
                    setGPSSignal(mavlink_msg_gps_raw_int_get_satellites_visible(&msg));
                    fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                    break;
                }
                
                case MAVLINK_MSG_ID_VFR_HUD:
                {
                    streams_received |= _BV(4);
                    setRpm(2,(mavlink_msg_vfr_hud_get_throttle(&msg) * 10.0f));
                    setSpeed((mavlink_msg_vfr_hud_get_groundspeed(&msg) * 3.6f));
                    setTemp(2, (mavlink_msg_vfr_hud_get_airspeed(&msg) * 3.6f));
                    alt = mavlink_msg_vfr_hud_get_alt(&msg);
                    setAltitude(alt - home_alt);
                    climb = (alpha * mavlink_msg_vfr_hud_get_climb(&msg)) + (1.0 - alpha) * climb;
                    setTemp(3, climb);
                    break;
                }
                                
                default:
                    break;
            }
        }
        if (streams_received == 0x1E)
            disableTx();
    }
}

