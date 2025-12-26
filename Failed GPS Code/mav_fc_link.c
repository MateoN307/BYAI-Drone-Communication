#include "mav_fc_link.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include <math.h>

#define BAUD_RATE   B115200
#define OUR_SYS_ID  255
#define OUR_COMP_ID MAV_COMP_ID_ONBOARD_COMPUTER

static int setup_serial(const char *dev_path)
{
	int fd;
	struct termios tty;

	fd = open(dev_path, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		perror("open");
		return -1;
	}

	if (tcgetattr(fd, &tty) != 0) {
		perror("tcgetattr");
		close(fd);
		return -1;
	}

	cfsetispeed(&tty, BAUD_RATE);
	cfsetospeed(&tty, BAUD_RATE);

	tty.c_cflag = BAUD_RATE | CS8 | CLOCAL | CREAD;
	tty.c_iflag = IGNPAR;
	tty.c_oflag = 0;
	tty.c_lflag = 0;

	tcflush(fd, TCIFLUSH);

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		perror("tcsetattr");
		close(fd);
		return -1;
	}

	return fd;
}

static int send_msg(struct mav_fc_state *st, const mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	ssize_t written;

	len = mavlink_msg_to_send_buffer(buf, msg);

	written = write(st->fd, buf, len);
	if (written < 0) {
		perror("write");
		return -1;
	}

	return 0;
}

int mav_fc_init(struct mav_fc_state *st, const char *dev_path)
{
	memset(st, 0, sizeof(*st));

	st->fd = setup_serial(dev_path);
	if (st->fd < 0)
		return -1;

	st->sys_id = OUR_SYS_ID;
	st->comp_id = OUR_COMP_ID;

	st->target_sys = 0;
	st->target_comp = 0;

	st->vbat = 0.0f;
	st->current_a = 0.0f;
	st->load = 0.0f;
	st->baro_press = 0.0f;
	st->baro_temp = 0.0f;

	st->lat_deg = 0.0;
	st->lon_deg = 0.0;
	st->alt_m = 0.0f;
	st->gps_fix_type = 0;
	st->gps_sats = 0;
	st->gps_hdop = 0.0f;
	st->gps_vdop = 0.0f;
	st->gps_ground_speed = 0.0f;
	st->gps_course_deg = 0.0f;

	// Init home
    st->home_is_set = 0;
    st->home_lat_deg = 0.0;
    st->home_lon_deg = 0.0;
    st->home_alt_m = 0.0f;

	return 0;
}

void mav_fc_close(struct mav_fc_state *st)
{
	if (st->fd >= 0) {
		close(st->fd);
		st->fd = -1;
	}
}

/* internal helper: request one message id at some rate */
static int request_msg_interval(struct mav_fc_state *st,
				uint16_t msg_id, float rate_hz)
{
	mavlink_message_t msg;
	float interval_us;

	if (st->target_sys == 0)
		return -1;

	if (rate_hz > 0.0f)
		interval_us = 1000000.0f / rate_hz;
	else
		interval_us = -1.0f;

	mavlink_msg_command_long_pack(st->sys_id,
				      st->comp_id,
				      &msg,
				      st->target_sys,
				      st->target_comp,
				      MAV_CMD_SET_MESSAGE_INTERVAL,
				      0,
				      (float)msg_id,
				      interval_us,
				      0, 0, 0, 0, 0);

	return send_msg(st, &msg);
}

int mav_fc_request_basic_streams(struct mav_fc_state *st)
{
	if (st->target_sys == 0)
		return -1;

	/* battery, barometer, gps / position */
	request_msg_interval(st, MAVLINK_MSG_ID_SYS_STATUS, 1.0f);
	request_msg_interval(st, MAVLINK_MSG_ID_SCALED_PRESSURE, 2.0f);
	request_msg_interval(st, MAVLINK_MSG_ID_SCALED_PRESSURE2, 2.0f);
	request_msg_interval(st, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2.0f);
	request_msg_interval(st, MAVLINK_MSG_ID_GPS_RAW_INT, 1.0f);

	return 0;
}

int mav_fc_send_heartbeat(struct mav_fc_state *st)
{
	mavlink_message_t msg;

	mavlink_msg_heartbeat_pack(st->sys_id,
				   st->comp_id,
				   &msg,
				   MAV_TYPE_ONBOARD_CONTROLLER,
				   MAV_AUTOPILOT_GENERIC,
				   0,
				   0,
				   MAV_STATE_ACTIVE);

	return send_msg(st, &msg);
}

int mav_fc_arm(struct mav_fc_state *st, int arm)
{
	mavlink_message_t msg;

	if (st->target_sys == 0) {
		fprintf(stderr,
			"arm: target system unknown (no heartbeat yet)\n");
		return -1;
	}

	mavlink_msg_command_long_pack(st->sys_id,
				      st->comp_id,
				      &msg,
				      st->target_sys,
				      st->target_comp,
				      MAV_CMD_COMPONENT_ARM_DISARM,
				      0,
				      arm ? 1.0f : 0.0f,
				      0, 0, 0, 0, 0, 0);

	return send_msg(st, &msg);
}

int mav_fc_set_mode(struct mav_fc_state *st, uint32_t custom_mode)
{
	mavlink_message_t msg;
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	if (st->target_sys == 0) {
		fprintf(stderr,
			"set_mode: target system unknown (no heartbeat yet)\n");
		return -1;
	}

	mavlink_msg_set_mode_pack(st->sys_id,
				  st->comp_id,
				  &msg,
				  st->target_sys,
				  base_mode,
				  custom_mode);

	return send_msg(st, &msg);
}

int mav_fc_set_velocity_body(struct mav_fc_state *st,
			     float vx, float vy, float vz)
{
	mavlink_message_t msg;
	uint16_t type_mask = 0;

	if (st->target_sys == 0) {
		fprintf(stderr,
			"vel: target system unknown (no heartbeat yet)\n");
		return -1;
	}

	/* ignore position (x,y,z) */
	type_mask |= (1 << 0);
	type_mask |= (1 << 1);
	type_mask |= (1 << 2);

	/* use velocity, ignore accel and yaw / yaw rate */
	type_mask |= (1 << 6);
	type_mask |= (1 << 7);
	type_mask |= (1 << 8);
	type_mask |= (1 << 10);
	type_mask |= (1 << 11);

	mavlink_msg_set_position_target_local_ned_pack(
		st->sys_id,
		st->comp_id,
		&msg,
		0,
		st->target_sys,
		st->target_comp,
		MAV_FRAME_BODY_NED,
		type_mask,
		0.0f, 0.0f, 0.0f,
		vx, vy, vz,
		0.0f, 0.0f, 0.0f,
		0.0f, 0.0f);

	return send_msg(st, &msg);
}

int mav_fc_set_yaw_rate(struct mav_fc_state *st, float yaw_rate_deg_s)
{
	mavlink_message_t msg;
	uint16_t type_mask = 0;
	float yaw_rate_rad;

	if (st->target_sys == 0) {
		fprintf(stderr,
			"yawrate: target system unknown (no heartbeat yet)\n");
		return -1;
	}

	/* ignore position and velocity and accel, use yaw_rate only */
	type_mask |= (1 << 0);
	type_mask |= (1 << 1);
	type_mask |= (1 << 2);
	type_mask |= (1 << 3);
	type_mask |= (1 << 4);
	type_mask |= (1 << 5);
	type_mask |= (1 << 6);
	type_mask |= (1 << 7);
	type_mask |= (1 << 8);
	type_mask |= (1 << 10); /* ignore yaw, use yaw_rate */

	yaw_rate_rad = yaw_rate_deg_s * (float)M_PI / 180.0f;

	mavlink_msg_set_position_target_local_ned_pack(
		st->sys_id,
		st->comp_id,
		&msg,
		0,
		st->target_sys,
		st->target_comp,
		MAV_FRAME_BODY_NED,
		type_mask,
		0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, yaw_rate_rad);

	return send_msg(st, &msg);
}

int mav_fc_goto_gps(struct mav_fc_state *st,
		    double lat_deg, double lon_deg, float alt_m)
{
	mavlink_message_t msg;
	int32_t lat_int;
	int32_t lon_int;
	uint16_t type_mask = 0;

	if (st->target_sys == 0) {
		fprintf(stderr,
			"goto: target system unknown (no heartbeat yet)\n");
		return -1;
	}

	lat_int = (int32_t)(lat_deg * 1e7);
	lon_int = (int32_t)(lon_deg * 1e7);

	/* we control position (lat, lon, alt) only */
	type_mask |= (1 << 3);  /* vx */
	type_mask |= (1 << 4);  /* vy */
	type_mask |= (1 << 5);  /* vz */
	type_mask |= (1 << 6);  /* ax */
	type_mask |= (1 << 7);  /* ay */
	type_mask |= (1 << 8);  /* az */
	type_mask |= (1 << 10); /* yaw */
	type_mask |= (1 << 11); /* yaw rate */

	mavlink_msg_set_position_target_global_int_pack(
		st->sys_id,
		st->comp_id,
		&msg,
		0,
		st->target_sys,
		st->target_comp,
		MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
		type_mask,
		lat_int,
		lon_int,
		alt_m,
		0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, 0.0f);

	return send_msg(st, &msg);
}

int mav_fc_set_home_here(struct mav_fc_state *st)
{
	mavlink_message_t msg;

	if (st->target_sys == 0) {
		fprintf(stderr,
			"set_home: target system unknown (no heartbeat yet)\n");
		return -1;
	}

	mavlink_msg_command_long_pack(st->sys_id,
				      st->comp_id,
				      &msg,
				      st->target_sys,
				      st->target_comp,
				      MAV_CMD_DO_SET_HOME,
				      0,
				      1.0f,
				      0, 0, 0,
				      0, 0, 0);

	return send_msg(st, &msg);
}

int mav_fc_set_throttle_pwm(struct mav_fc_state *st, uint16_t pwm)
{
	mavlink_message_t msg;

	if (st->target_sys == 0) {
		fprintf(stderr,
			"throttle: target system unknown (no heartbeat yet)\n");
		return -1;
	}

	if (pwm < 800)
		pwm = 800;
	if (pwm > 2200)
		pwm = 2200;

	/*
	 * RC_CHANNELS_OVERRIDE:
	 * chan1..chan18; 0 means "no change".
	 * We override only channel 3 (throttle).
	 */
	mavlink_msg_rc_channels_override_pack(
		st->sys_id,
		st->comp_id,
		&msg,
		st->target_sys,
		st->target_comp,
		0,          /* chan1 */
		0,          /* chan2 */
		pwm,        /* chan3 - throttle */
		0,          /* chan4 */
		0, 0, 0, 0, /* chan5..8  */
		0, 0, 0, 0, /* chan9..12 */
		0, 0, 0, 0, /* chan13..16 */
		0, 0        /* chan17..18 */
	);

	return send_msg(st, &msg);
}


static const char *mav_result_str(uint8_t r)
{
	switch (r) {
	case MAV_RESULT_ACCEPTED:
		return "ACCEPTED";
	case MAV_RESULT_TEMPORARILY_REJECTED:
		return "TEMPORARILY_REJECTED";
	case MAV_RESULT_DENIED:
		return "DENIED";
	case MAV_RESULT_UNSUPPORTED:
		return "UNSUPPORTED";
	case MAV_RESULT_FAILED:
		return "FAILED";
	case MAV_RESULT_IN_PROGRESS:
		return "IN_PROGRESS";
	default:
		return "UNKNOWN";
	}
}

static void handle_message(struct mav_fc_state *st,
			   const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT: {
		mavlink_heartbeat_t hb;

		mavlink_msg_heartbeat_decode(msg, &hb);

		st->base_mode = hb.base_mode;
		st->custom_mode = hb.custom_mode;
		st->system_status = hb.system_status;

		if (st->target_sys == 0) {
			st->target_sys = msg->sysid;
			st->target_comp = msg->compid;
			printf("[INFO] target system detected: sys=%u comp=%u\n",
			       st->target_sys, st->target_comp);
			mav_fc_request_basic_streams(st);
		}
		break;
	}

	case MAVLINK_MSG_ID_SYS_STATUS: {
		mavlink_sys_status_t sm;

		mavlink_msg_sys_status_decode(msg, &sm);
		st->vbat = sm.voltage_battery / 1000.0f;
		st->current_a = sm.current_battery / 100.0f;
		st->load = sm.load / 10.0f;
		break;
	}

	case MAVLINK_MSG_ID_SCALED_PRESSURE: {
		mavlink_scaled_pressure_t sp;

		mavlink_msg_scaled_pressure_decode(msg, &sp);
		st->baro_press = sp.press_abs;
		st->baro_temp = sp.temperature / 100.0f;
		break;
	}

	case MAVLINK_MSG_ID_SCALED_PRESSURE2: {
		mavlink_scaled_pressure2_t sp2;

		mavlink_msg_scaled_pressure2_decode(msg, &sp2);
		st->baro_press = sp2.press_abs;
		st->baro_temp = sp2.temperature / 100.0f;
		break;
	}

	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
		mavlink_global_position_int_t gp;

		mavlink_msg_global_position_int_decode(msg, &gp);
		st->lat_deg = gp.lat / 1e7;
		st->lon_deg = gp.lon / 1e7;
		st->alt_m = gp.relative_alt / 1000.0f;
		break;
	}

	case MAVLINK_MSG_ID_GPS_RAW_INT: {
		mavlink_gps_raw_int_t gr;

		mavlink_msg_gps_raw_int_decode(msg, &gr);

		st->gps_fix_type = gr.fix_type;
		st->gps_sats = gr.satellites_visible;
		st->gps_hdop = gr.eph / 100.0f;
		st->gps_vdop = gr.epv / 100.0f;

		if (gr.fix_type >= 2 && gr.lat != 0 && gr.lon != 0) {
			st->lat_deg = gr.lat / 1e7;
			st->lon_deg = gr.lon / 1e7;
			st->alt_m = gr.alt / 1000.0f;
			st->gps_ground_speed = gr.vel / 100.0f;
			st->gps_course_deg = gr.cog / 100.0f;
		}
		break;
	}

	case MAVLINK_MSG_ID_HOME_POSITION: {
		mavlink_home_position_t hp;
		mavlink_msg_home_position_decode(msg, &hp);

		st->home_lat_deg = hp.latitude / 1e7;
		st->home_lon_deg = hp.longitude / 1e7;
		st->home_alt_m   = hp.altitude / 1000.0f;
		st->home_is_set  = 1;
        
        printf("[INFO] Home Position Updated: %.7f, %.7f\n", st->home_lat_deg, st->home_lon_deg);
		break;
	}

	case MAVLINK_MSG_ID_COMMAND_ACK: {
		mavlink_command_ack_t ack;

		mavlink_msg_command_ack_decode(msg, &ack);
		printf("[COMMAND_ACK] cmd=%u result=%u (%s)\n",
		       ack.command, ack.result,
		       mav_result_str(ack.result));
		break;
	}

	case MAVLINK_MSG_ID_STATUSTEXT: {
		mavlink_statustext_t stxt;

		mavlink_msg_statustext_decode(msg, &stxt);
		stxt.text[sizeof(stxt.text) - 1] = '\0';

		printf("[STATUSTEXT] severity=%u text=%s\n",
		       stxt.severity, stxt.text);
		break;
	}

	default:
		break;
	}
}

int mav_fc_poll(struct mav_fc_state *st, int timeout_ms)
{
	fd_set readfds;
	struct timeval tv;
	struct timeval *ptv = NULL;
	uint8_t buf[256];
	ssize_t n;
	int ret;
	mavlink_message_t msg;
	mavlink_status_t status;
	int msg_count = 0;
	ssize_t i;

	if (st->fd < 0) {
		errno = EBADF;
		return -1;
	}

	FD_ZERO(&readfds);
	FD_SET(st->fd, &readfds);

	if (timeout_ms >= 0) {
		tv.tv_sec = timeout_ms / 1000;
		tv.tv_usec = (timeout_ms % 1000) * 1000;
		ptv = &tv;
	}

	ret = select(st->fd + 1, &readfds, NULL, NULL, ptv);
	if (ret < 0) {
		if (errno == EINTR)
			return 0;
		perror("select");
		return -1;
	}

	if (ret == 0)
		return 0;

	n = read(st->fd, buf, sizeof(buf));
	if (n < 0) {
		if (errno == EAGAIN || errno == EINTR)
			return 0;
		perror("read");
		return -1;
	}

	for (i = 0; i < n; i++) {
		if (mavlink_parse_char(MAVLINK_COMM_0,
				       buf[i], &msg, &status)) {
			handle_message(st, &msg);
			msg_count++;
		}
	}

	return msg_count;
}

int mav_fc_takeoff(struct mav_fc_state *st, float alt_m)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	/* * MAV_CMD_NAV_TAKEOFF
	 * Param 7: Altitude (meters)
	 */
	mavlink_msg_command_long_pack(st->sys_id, st->comp_id, &msg,
				      st->target_sys, st->target_comp,
				      MAV_CMD_NAV_TAKEOFF, 0,
				      0, 0, 0, 0, 0, 0, alt_m);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	if (write(st->fd, buf, len) != len)
		return -1;

	return 0;
}
