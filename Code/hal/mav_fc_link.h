/*
 * mav_fc_link.h
 *
 * MAVLink Flight Controller Interface
 *
 * RETURN VALUE CONVENTION:
 * Unless otherwise noted, functions return:
 * 0  : Success (Message packed and written to file descriptor)
 * -1  : Failure (Write failed, often due to disconnected serial cable)
 */

#pragma once

#include <stdint.h>
#include "c_library_v2/common/mavlink.h"

/* * Main state structure for the flight controller link.
 * Holds the serial file descriptor and the latest telemetry data.
 */
struct mav_fc_state {
    int fd;                 // Serial port file descriptor

    uint8_t sys_id;         // Our System ID (Ground Station)
    uint8_t comp_id;        // Our Component ID

    uint8_t target_sys;     // Detected Drone System ID
    uint8_t target_comp;    // Detected Drone Component ID

    /* Telemetry Data */
    uint8_t base_mode;      // Raw base mode bitmask
    uint32_t custom_mode;   // ArduPilot custom flight mode (0=Stab, 2=AltHold, 9=Land)
    uint8_t system_status;  // MAV_STATE (Active, Standby, etc.)

    float vbat;             // Battery Voltage (Volts)
    float current_a;        // Battery Current (Amps)
    float load;             // System Load (%)

    float baro_press;       // Barometric Pressure (Pa)
    float baro_temp;        // Barometric Temperature (degC)

    /* GPS State */
    double lat_deg;         // Latitude (Degrees)
    double lon_deg;         // Longitude (Degrees)
    float alt_m;            // Relative Altitude (Meters) - Barometric or GPS mix

    uint8_t gps_fix_type;   // 0=No GPS, 2=2D Fix, 3=3D Fix
    uint8_t gps_sats;       // Visible Satellites
    float gps_hdop;         // Horizontal Dilution of Precision
    float gps_vdop;         // Vertical Dilution of Precision
    float gps_ground_speed; // Speed over ground (m/s)
    float gps_course_deg;   // Course over ground (Degrees)

    /* Home Position */
    int home_is_set;        // Flag: 1 if Home position has been received
    double home_lat_deg;
    double home_lon_deg;
    float home_alt_m;
};

/* * Opens the serial port (e.g., /dev/ttyACM0) and initializes the state struct.
 * Returns: 0 on success, -1 if file cannot be opened.
 */
int mav_fc_init(struct mav_fc_state *st, const char *dev_path);

/* * Closes the serial port file descriptor.
 */
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

/* * Sends a HEARTBEAT message to the drone.
 * Required periodically (1Hz) to keep the connection alive.
 */
int mav_fc_send_heartbeat(struct mav_fc_state *st);

/* * Requests data streams (Battery, GPS, Attitude) from the FC.
 */
int mav_fc_request_basic_streams(struct mav_fc_state *st);

/* * Arms or Disarms the motors.
 * param arm: 1 = ARM, 0 = DISARM
 */
int mav_fc_arm(struct mav_fc_state *st, int arm);

/* * Sets the flight mode.
 * Common ArduCopter Modes:
 * 0 = STABILIZE
 * 2 = ALT_HOLD
 * 9 = LAND
 */
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
 * Override RC throttle channel 3 (serial motor servo output) with a raw PWM value (1000–2000).
 * Other channels are left at 0 (no override).
 */
int mav_fc_set_throttle_pwm(struct mav_fc_state *st, uint16_t pwm);

/* Command takeoff to specific altitude (in meters). Requires GUIDED mode and ARMED. */
int mav_fc_takeoff(struct mav_fc_state *st, float alt_m);

/* * Sets the raw RC channel overrides (Virtual Joystick).
 * Values: 1000 (Min) to 2000 (Max), 1500 (Center).
 * 0 releases control of that channel to the RC receiver.
 * * r=Roll, p=Pitch, t=Throttle, y=Yaw
 */
int mav_fc_set_rc_override(struct mav_fc_state *st, uint16_t r, uint16_t p, uint16_t t, uint16_t y);

/* Emergency Force Disarm - Kills motors even in flight */
int mav_fc_disarm_emergency(struct mav_fc_state *st);

