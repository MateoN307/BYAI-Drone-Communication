/* main_drone.c - Runs on BeagleBoard */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <time.h>
#include <stdarg.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "mav_fc_link.h"

#define UDP_TIMEOUT_SEC 4  // Disarm if no data for 4 seconds

#define UDP_PORT 12345

/* Global for the UDP socket and last known client address */
static int udp_sock = -1;
static struct sockaddr_in client_addr;
static socklen_t client_addr_len = sizeof(client_addr);
static int has_client = 0;

/* Helper to send text back to the ground station instead of printf */
static void reply_print(const char *format, ...)
{
    char buffer[1024];
    va_list args;

    if (udp_sock < 0 || !has_client)
        return;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    /* Send the string via UDP to the last known client */
    sendto(udp_sock, buffer, strlen(buffer), 0, 
           (struct sockaddr *)&client_addr, client_addr_len);
}

static void print_help_remote(void)
{
    reply_print("Commands:\n");
    reply_print("  arm                : arm motors\n");
    reply_print("  disarm             : disarm motors\n");
    reply_print("  mode stab          : STABILIZE mode\n");
    reply_print("  mode alt           : ALT_HOLD mode\n");
    reply_print("  mode loiter        : LOITER mode\n");
    reply_print("  mode guided        : GUIDED mode\n");
    reply_print("  vel vx vy vz       : body velocity [m/s]\n");
    reply_print("  yawrate r          : yaw rate [deg/s]\n");
    reply_print("  throttle pwm       : override throttle PWM\n");
    reply_print("  goto lat lon alt   : GPS target\n");
    reply_print("  pos                : print GPS state\n");
    reply_print("  sethome            : set HOME = current pos\n");
    reply_print("  status             : print system state\n");
    reply_print("  help               : show this text\n");
}

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int setup_udp_server(int port)
{
    int s;
    struct sockaddr_in sin;

    s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        perror("socket");
        return -1;
    }

    /* Bind to all interfaces */
    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = htonl(INADDR_ANY);
    sin.sin_port = htons(port);

    if (bind(s, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }

    return s;
}

int main(void)
{
    struct mav_fc_state st;
    double last_hb;
    double last_udp_time;
    int failsafe_triggered = 0;
    fd_set rfds;
    struct timeval tv;
    int ret;
    
    /* Initialize Serial Link to FC */
    if (mav_fc_init(&st, "/dev/ttyACM0") != 0) {
        fprintf(stderr, "failed to init mav_fc_link\n");
        return 1;
    }
    printf("connected to FC via /dev/ttyACM0\n");
    /* Initialize UDP Server */
    udp_sock = setup_udp_server(UDP_PORT);
    if (udp_sock < 0) {
        fprintf(stderr, "failed to setup UDP server\n");
        return 1;
    }
    printf("UDP server listening on port %d\n", UDP_PORT);

    last_hb = now_sec();
    last_udp_time = now_sec();

    while (1) {
        double t = now_sec();
        
        if (t - last_udp_time > UDP_TIMEOUT_SEC) {
            if (!failsafe_triggered) {
                printf("[SAFETY] UDP Timeout! Initiating SAFE LAND...\n");
                reply_print("[SAFETY] LINK LOST - LANDING!\n");
                mav_fc_set_mode(&st, 9);   // LAND mode for ArduCopter
                failsafe_triggered = 1;
                // Wait for landing
                sleep(3);

                // ***** Beep 3 times using ESC twitch ********
                for (int i = 0; i < 3; i++) {
                    mav_fc_set_throttle_pwm(&st, 1100);  // twitch low
                    usleep(150000);                      // 150ms
                    mav_fc_set_throttle_pwm(&st, 1300);  // twitch high
                    usleep(150000);
                }
                mav_fc_set_throttle_pwm(&st, 1000); // fully off
                // ***** End Beep Sequence ********

                // auto disarm
                mav_fc_arm(&st, 0); // disarm permanently
                reply_print("safe-land disarm complete\n");
            }
        } else {
            // Link is healthy
            failsafe_triggered = 0; 
        }

        /* ---------------------------------- */
        if (t - last_hb >= 1.0) {
            mav_fc_send_heartbeat(&st);
            last_hb = t;
        }

        /* Check serial buffer for FC data (timeout 20ms to allow UDP checks) */
        mav_fc_poll(&st, 20);

        /* 2. Networking Loop: Check for UDP commands */
        FD_ZERO(&rfds);
        FD_SET(udp_sock, &rfds);

        tv.tv_sec = 0;
        tv.tv_usec = 0; /* Non-blocking check */

        ret = select(udp_sock + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(udp_sock, &rfds)) {
            char cmd[256];
            ssize_t len;
            struct sockaddr_in sender;
            socklen_t sender_len = sizeof(sender);

            /* Receive packet */
            len = recvfrom(udp_sock, cmd, sizeof(cmd) - 1, 0,
                           (struct sockaddr *)&sender, &sender_len);
            
            if (len > 0) {
                cmd[len] = '\0';
                
                /* Remove newline if present */
                char *nl = strchr(cmd, '\n');
                if (nl) *nl = '\0';

                /* Update client address for replies */
                memcpy(&client_addr, &sender, sizeof(sender));
                has_client = 1;
                last_udp_time = now_sec();
                
                // printf("Received UDP cmd: '%s'\n", cmd);

                /* Process Command using reply_print instead of printf */
                if (strncmp(cmd, "takeoff ", 8) == 0) {
                    float alt;
                    if (sscanf(cmd + 8, "%f", &alt) == 1) {
                        
                        /* --- NEW: GPS SAFETY CHECK --- */
                        if (st.gps_fix_type < 3) {
                            reply_print("ERROR: Takeoff rejected! Need 3D GPS Fix.\n");
                            reply_print("       Current Fix Type: %u (0=No GPS, 3=3D Fix)\n", 
                                        st.gps_fix_type);
                        } 
                        else {
                            /* GPS is good, proceed with Takeoff Sequence */
                            reply_print("AUTO-TAKEOFF SEQUENCE STARTED:\n");
                            
                            // 1. Force GUIDED Mode
                            reply_print(" -> Switching to GUIDED mode...\n");
                            mav_fc_set_mode(&st, 4); 
                            
                            // 2. Force Arming
                            reply_print(" -> Arming motors...\n");
                            mav_fc_arm(&st, 1);
                            
                            // 3. Wait for FC to process arming
                            usleep(500000); 
                            
                            // 4. Send Takeoff Command
                            reply_print(" -> Taking off to %.1fm...\n", alt);
                            mav_fc_takeoff(&st, alt);
                        }
                        /* ----------------------------- */
                        
                    } else {
                        reply_print("usage: takeoff <alt_meters>\n");
                    }
                } else if (strcmp(cmd, "mode guided") == 0) {
                    reply_print("mode GUIDED\n");
                    mav_fc_set_mode(&st, 4); // ArduCopter Mode 4 = GUIDED


                } else if (strcmp(cmd, "arm") == 0) {
                    reply_print("arming...\n");
                    mav_fc_arm(&st, 1);

                } else if (strcmp(cmd, "s") == 0 || strcmp(cmd, "S") == 0) {
                    reply_print("*** !!! EMERGENCY SHUTDOWN !!! ***\n");
                    printf("[EMERGENCY] HARD STOP REQUESTED!\n");
                    mav_fc_arm(&st, 0);   // DISARM immediately (even in air)
                    

                } else if (strcmp(cmd, "disarm") == 0) {
                    reply_print("disarming...\n");
                    mav_fc_arm(&st, 0);

                } else if (strcmp(cmd, "mode stab") == 0) {
                    reply_print("set mode STABILIZE\n");
                    mav_fc_set_mode(&st, 0);

                } else if (strcmp(cmd, "mode alt") == 0) {
                    reply_print("set mode ALT_HOLD\n");
                    mav_fc_set_mode(&st, 2);

                } else if (strcmp(cmd, "mode loiter") == 0) {
                    reply_print("set mode LOITER\n");
                    mav_fc_set_mode(&st, 5);

                } else if (strncmp(cmd, "vel ", 4) == 0) {
                    float vx, vy, vz;
                    if (sscanf(cmd + 4, "%f %f %f", &vx, &vy, &vz) == 3) {
                        reply_print("cmd vel: vx=%.2f vy=%.2f vz=%.2f\n", vx, vy, vz);
                        mav_fc_set_velocity_body(&st, vx, vy, vz);
                    } else {
                        reply_print("usage: vel vx vy vz\n");
                    }

                } else if (strncmp(cmd, "yawrate ", 8) == 0) {
                    float r;
                    if (sscanf(cmd + 8, "%f", &r) == 1) {
                        reply_print("cmd yawrate: %.2f deg/s\n", r);
                        mav_fc_set_yaw_rate(&st, r);
                    } else {
                        reply_print("usage: yawrate r_deg_s\n");
                    }

                } else if (strncmp(cmd, "throttle ", 9) == 0) {
                    int pwm;
                    if (sscanf(cmd + 9, "%d", &pwm) == 1) {
                        reply_print("cmd throttle: %d us\n", pwm);
                        mav_fc_set_throttle_pwm(&st, (uint16_t)pwm);
                    } else {
                        reply_print("usage: throttle pwm(1000-2000)\n");
                    }

                } else if (strncmp(cmd, "goto ", 5) == 0) {
                    double lat, lon;
                    float alt;
                    if (sscanf(cmd + 5, "%lf %lf %f", &lat, &lon, &alt) == 3) {
                        reply_print("goto: lat=%.7f lon=%.7f alt=%.2f\n", lat, lon, alt);
                        mav_fc_goto_gps(&st, lat, lon, alt);
                    } else {
                        reply_print("usage: goto lat lon alt\n");
                    }

                } else if (strcmp(cmd, "pos") == 0) {
                    if (st.gps_fix_type < 2) {
                        reply_print("gps: no fix yet (fix_type=%u sats=%u)\n",
                                    st.gps_fix_type, st.gps_sats);
                    } else {
                        reply_print("gps: lat=%.7f lon=%.7f alt=%.2f m\n"
                                    "     fix_type=%u sats=%u hdop=%.2f vdop=%.2f\n"
                                    "     ground_speed=%.2f m/s course=%.1f deg\n",
                                    st.lat_deg, st.lon_deg, st.alt_m,
                                    st.gps_fix_type, st.gps_sats, st.gps_hdop, st.gps_vdop,
                                    st.gps_ground_speed, st.gps_course_deg);
                    }

                } else if (strcmp(cmd, "sethome") == 0) {
                    if (st.gps_fix_type < 3) {
                        reply_print("sethome: need at least 3D fix (fix_type>=3), current fix=%u\n",
                                    st.gps_fix_type);
                    } else {
                        reply_print("sethome: requesting HOME = current position\n");
                        mav_fc_set_home_here(&st);
                    }

                } else if (strcmp(cmd, "status") == 0) {
                    // Decode Arm status (Bit 7 of base_mode)
                    const char *arm_str = (st.base_mode & 0x80) ? "ARMED" : "DISARMED";

                    reply_print("status: mode=%u (%s) base_mode=0x%02X\n"
                                "        vbat=%.2fV current=%.2fA load=%.1f%%\n"
                                "        baro=%.2fhPa temp=%.2fC\n",
                                st.custom_mode, arm_str, st.base_mode,
                                st.vbat, st.current_a, st.load, 
                                st.baro_press, st.baro_temp);

                    // Decode Home Position
                    if (st.home_is_set) {
                        reply_print("        HOME: lat=%.7f lon=%.7f alt=%.2fm\n", 
                                    st.home_lat_deg, st.home_lon_deg, st.home_alt_m);
                    } else {
                        reply_print("        HOME: [NOT SET]\n");
                    }
                    /* ------------------------------------------- */

                } else if (strcmp(cmd, "help") == 0) {
                    print_help_remote();
                } else if (strcmp(cmd, "beep") == 0) {
                    reply_print("beep test: 3 short beeps\n");
                    printf("[TEST] ESC Beep Test Initiated...\n");
                
                    // 3 short beeps using motor twitch
                    for (int i = 0; i < 3; i++) {
                        mav_fc_set_throttle_pwm(&st, 1100);  // twitch low
                        usleep(150000);                      // 150 ms
                        mav_fc_set_throttle_pwm(&st, 1300);  // twitch high
                        usleep(150000);                      // 150 ms
                    }
                
                    mav_fc_set_throttle_pwm(&st, 1000);  // fully off
                }
                else {
                    //reply_print("unknown command: '%s'\n", cmd);
                }
            }
        }
    }

    mav_fc_close(&st);
    if (udp_sock >= 0) close(udp_sock);
    return 0;
}