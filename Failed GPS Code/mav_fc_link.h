#ifndef MAV_FC_LINK_H
#define MAV_FC_LINK_H

#include <stdint.h>
#include "c_library_v2/common/mavlink.h"

struct mav_fc_state {
	int fd;

	uint8_t sys_id;
	uint8_t comp_id;

	uint8_t target_sys;
	uint8_t target_comp;

	uint8_t base_mode;
	uint32_t custom_mode;
	uint8_t system_status;

	float vbat;
	float current_a;
	float load;

	float baro_press;
	float baro_temp;

	/* gps state */
	double lat_deg;
	double lon_deg;
	float alt_m;              /* relative or AMSL, meters */

	uint8_t gps_fix_type;     /* 0=NO_GPS,1=NO_FIX,2=2D,3=3D,... */
	uint8_t gps_sats;
	float gps_hdop;
	float gps_vdop;
	float gps_ground_speed;   /* m/s */
	float gps_course_deg;     /* degrees, 0..360 */

	/* --- NEW: HOME POSITION STORAGE --- */
    int home_is_set;
    double home_lat_deg;
    double home_lon_deg;
    float home_alt_m;
};

/* open serial device (e.g. /dev/ttyACM0) and init state */
int mav_fc_init(struct mav_fc_state *st, const char *dev_path);

/* close serial fd */
void mav_fc_close(struct mav_fc_state *st);

/*
 * Poll the serial fd and process all available MAVLink messages.
 * timeout_ms < 0  => block forever
 * timeout_ms = 0  => non-blocking
 * timeout_ms > 0  => block up to timeout_ms
 *
 * returns:
 *   >0 : number of MAVLink messages processed
 *    0 : timeout, no data
 *   <0 : error (see errno)
 */
int mav_fc_poll(struct mav_fc_state *st, int timeout_ms);

/* send a heartbeat from BYAI (call periodically from your main loop) */
int mav_fc_send_heartbeat(struct mav_fc_state *st);

/* request basic telemetry from FC (battery + barometer + GPS) */
int mav_fc_request_basic_streams(struct mav_fc_state *st);

/* arm=1 => arm, arm=0 => disarm */
int mav_fc_arm(struct mav_fc_state *st, int arm);

/* set full custom_mode (e.g. 0=STABILIZE, 2=ALT_HOLD, 5=LOITER for ArduCopter) */
int mav_fc_set_mode(struct mav_fc_state *st, uint32_t custom_mode);

/*
 * Send body-frame velocity command (m/s).
 * vx >0 : forward,  vx <0 : backward
 * vy >0 : right,    vy <0 : left
 * vz >0 : down,     vz <0 : up
 */
int mav_fc_set_velocity_body(struct mav_fc_state *st,
			     float vx, float vy, float vz);

/*
 * Command a yaw rate (deg/s, positive = clockwise when viewed from above).
 * Works in GUIDED mode; FC interprets as body yaw-rate command.
 */
int mav_fc_set_yaw_rate(struct mav_fc_state *st, float yaw_rate_deg_s);

/*
 * Send a GPS position target (lat/lon in deg, alt in meters).
 * Uses MAV_FRAME_GLOBAL_RELATIVE_ALT_INT.
 */
int mav_fc_goto_gps(struct mav_fc_state *st,
		    double lat_deg, double lon_deg, float alt_m);

/*
 * Set HOME position to the vehicle's current position.
 * Uses MAV_CMD_DO_SET_HOME with param1=1.
 */
int mav_fc_set_home_here(struct mav_fc_state *st);

/*
 * Override RC throttle channel (chan3) with a raw PWM value (1000–2000).
 * Other channels are left at 0 (no override).
 */
int mav_fc_set_throttle_pwm(struct mav_fc_state *st, uint16_t pwm);

/* Command takeoff to specific altitude (in meters). Requires GUIDED mode and ARMED. */
int mav_fc_takeoff(struct mav_fc_state *st, float alt_m);

#endif /* MAV_FC_LINK_H */

