/*
 * main_drone.c
 * * Platform: BeagleBoard / BeagleY-AI
 * Role: Onboard Flight Computer
 * * Description:
 * - Connects to Flight Controller via Serial (MAVLink).
 * - Hosts a UDP Server to receive commands from Ground Station.
 * - Parses commands (move, land, status) and translates to MAVLink.
 * - Implements safety failsafes (Connection Lost -> Land).
 * - Performs battery voltage linearization.
 */

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

/* --- CONFIGURATION --- */
#define UDP_TIMEOUT_SEC 4       // Auto-Land if no packets for 4 seconds
#define UDP_PORT 12345          // Listening Port
#define HOVER_SPEED 1500        // PWM for Neutral/Hover
#define TAKEOFF_SPEED 1550      // PWM for Climb

/* --- BATTERY LOOKUP TABLE (4S LiPo) --- */
/* Used for linear interpolation of voltage to percentage */
typedef struct {
    float voltage;
    int percent;
} batt_point_t;

static const batt_point_t BATT_TABLE[] = {
    {16.80f, 100}, // 100% Fully Charged
    {16.60f, 95},
    {16.45f, 90},
    {16.33f, 85},
    {16.09f, 80},
    {15.93f, 75},
    {15.81f, 70},
    {15.66f, 65},
    {15.50f, 60},
    {15.42f, 55},
    {15.34f, 50},
    {15.26f, 45},
    {15.18f, 40},
    {15.14f, 35},
    {15.06f, 30},
    {14.99f, 25},
    {14.91f, 20},
    {14.83f, 15}, // Yellow zone start
    {14.75f, 10}, // Red zone start
    {14.43f, 5},
    {13.09f, 0}   // Empty
};

/* * Interpolates battery percentage from voltage based on BATT_TABLE.
 * Returns: 0-100 (Integer percent)
 */
static int get_battery_pct(float voltage) {
    int count = sizeof(BATT_TABLE) / sizeof(BATT_TABLE[0]);
    
    // Bounds check
    if (voltage >= BATT_TABLE[0].voltage) return 100;
    if (voltage <= BATT_TABLE[count-1].voltage) return 0;

    // Linear Interpolation loop
    for (int i = 0; i < count - 1; i++) {
        if (voltage <= BATT_TABLE[i].voltage && voltage > BATT_TABLE[i+1].voltage) {
            float v_high = BATT_TABLE[i].voltage;
            float v_low = BATT_TABLE[i+1].voltage;
            int p_high = BATT_TABLE[i].percent;
            int p_low = BATT_TABLE[i+1].percent;
            
            float fraction = (voltage - v_low) / (v_high - v_low);
            return p_low + (int)(fraction * (p_high - p_low));
        }
    }
    return 0;
}

/* Global State Variables */
static int udp_sock = -1;
static struct sockaddr_in client_addr;
static socklen_t client_addr_len = sizeof(client_addr);
static int has_client = 0;
static int live_status_active = 0; // Toggle for live dashboard
static double last_status_update = 0;
static float alt_offset = 0.0f; // Altitude Tare Offset

/* Helper: Convert Mode ID to String */
static const char* get_mode_name(uint32_t mode) {
    switch(mode) {
        case 0: return "STABILIZE";
        case 2: return "ALT_HOLD";
        case 3: return "AUTO";
        case 4: return "GUIDED";
        case 5: return "LOITER";
        case 6: return "RTL";
        case 9: return "LAND";
        default: return "UNKNOWN";
    }
}

/* Helper: Send UDP reply to Ground Station */
static void reply_print(const char *format, ...)
{
    char buffer[1024];
    va_list args;
    if (udp_sock < 0 || !has_client) return;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    sendto(udp_sock, buffer, strlen(buffer), 0, (struct sockaddr *)&client_addr, client_addr_len);
}

/* Displays available commands to the user */
static void print_help_remote(void)
{
    reply_print("\n");
    reply_print("================== DRONE COMMAND LIST ==================\n");
    reply_print(" COMMAND            | ACTION                     | PREREQ \n");
    reply_print("--------------------|----------------------------|------\n");
    
    /* Flight Controls */
    reply_print(" arm                | Start Motors               | Thr=1000\n");
    reply_print(" disarm             | Stop Motors                | Landed\n");
    reply_print(" takeoff            | Auto-Launch (AltHold, 2s)  | Disarmed\n");
    reply_print(" land               | Auto-Land (Slow Descent)   | Flying\n");
    reply_print(" s                  | EMERGENCY KILL SWITCH      | NONE\n");
    
    /* Modes */
    reply_print(" mode stab          | Manual Stabilize           | -\n");
    reply_print(" mode alt           | Altitude Hold (Baro)       | -\n");
    
    /* Diagnostics / Testing */
    reply_print(" throttle <pwm>     | Force Throttle (1000-2000) | -\n");
    reply_print(" status             | Show Battery/Mode/Alt      | -\n");
    reply_print(" alt reset          | Tare Altimeter to 0.0m     | Ground\n");
    
    reply_print("========================================================\n");
}

/* Helper: Monotonic Time in Seconds */
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
    if (s < 0) return -1;
    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = htonl(INADDR_ANY);
    sin.sin_port = htons(port);
    if (bind(s, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
        close(s); return -1;
    }
    return s;
}

/* --- MAIN --- */
int main(void)
{
    struct mav_fc_state st;
    double last_hb;
    double last_udp_time;
    int failsafe_triggered = 0;
    fd_set rfds;
    struct timeval tv;
    int ret;
    
    // 1. Init Serial Link
    if (mav_fc_init(&st, "/dev/ttyACM0") != 0) {
        fprintf(stderr, "failed to init mav_fc_link\n");
        return 1;
    }
    printf("connected to FC via /dev/ttyACM0\n");

    // 2. Setup UDP Server
    udp_sock = setup_udp_server(UDP_PORT);
    if (udp_sock < 0) {
        fprintf(stderr, "failed to setup UDP server\n");
        return 1;
    }
    printf("UDP server listening on port %d\n", UDP_PORT);

    last_hb = now_sec();
    last_udp_time = now_sec();

    // 3. Main Loop
    while (1) {
        double t = now_sec();
        
        /* LIVE STATUS DASHBOARD UPDATE (5Hz) */
        if (live_status_active && (t - last_status_update > 0.2)) {
            reply_print("\r[ST] Mod:%-9s | Bat:%3d%% %4.1fV | Cur:%4.1fA | Ld:%4.1f%% | Alt:%5.1fm   ", 
                        get_mode_name(st.custom_mode), 
                        get_battery_pct(st.vbat),
                        st.vbat,
                        st.current_a, 
                        st.load, 
                        st.alt_m - alt_offset);
            last_status_update = t;
        }
        
        /* CONNECTION WATCHDOG (Failsafe) */
        if (t - last_udp_time > UDP_TIMEOUT_SEC) {
            if (!failsafe_triggered) {
                reply_print("[SAFETY] CONNECTION LOST - LANDING!\n");
                mav_fc_set_mode(&st, 9); // LAND
                failsafe_triggered = 1;
            }
        } else {
            failsafe_triggered = 0; 
        }

        /* SEND HEARTBEAT (1Hz) */
        if (t - last_hb >= 1.0) {
            mav_fc_send_heartbeat(&st);
            last_hb = t;
        }

        /* READ FROM MAVLINK */
        mav_fc_poll(&st, 20);

        /* READ FROM UDP */
        FD_ZERO(&rfds);
        FD_SET(udp_sock, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 0; 

        ret = select(udp_sock + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(udp_sock, &rfds)) {
            char cmd[256];
            ssize_t len;
            struct sockaddr_in sender;
            socklen_t sender_len = sizeof(sender);

            len = recvfrom(udp_sock, cmd, sizeof(cmd) - 1, 0, (struct sockaddr *)&sender, &sender_len);
            
            if (len > 0) {
                cmd[len] = '\0';
                char *nl = strchr(cmd, '\n');
                if (nl) *nl = '\0';

                memcpy(&client_addr, &sender, sizeof(sender));
                has_client = 1;
                last_udp_time = now_sec();
                
                /* COMMAND: JOYSTICK MOVE */
                if (strncmp(cmd, "move ", 5) == 0) {
                    int roll, pitch, thr, yaw;
                    if (sscanf(cmd + 5, "%d %d %d %d", &roll, &pitch, &thr, &yaw) == 4) {
                        /* --- DEBUG VISUALIZER --- */
                        printf("[RC INPUT] Roll:%4d Pitch:%4d Thr:%4d Yaw:%4d   \r", roll, pitch, thr, yaw);
                        fflush(stdout);
                        mav_fc_set_rc_override(&st, (uint16_t)roll, (uint16_t)pitch, (uint16_t)thr, (uint16_t)yaw);
                    }
                } 
                else {
                    /* Handle Text Commands */
                    if (strcmp(cmd, "status") != 0) {
                        live_status_active = 0; 
                        printf("\n"); // Clear line for new command output
                    }

                    if (strcmp(cmd, "s") == 0) {
                        reply_print("!!! EMERGENCY KILL TRIGGERED !!!\n");
                        reply_print("FORCING DISARM (MAGIC 21196).\n");
                        mav_fc_disarm_emergency(&st); // Force Stop
                        mav_fc_disarm_emergency(&st); // Redundancy
                    }
                    else if (strcmp(cmd, "status") == 0) {
                        // Enable Live Update
                        live_status_active = 1; 
                        last_status_update = 0; // Trigger update immediately
                    }
                    else if (strncmp(cmd, "takeoff", 7) == 0) {
                        reply_print("MANUAL TAKEOFF...\n");
                        mav_fc_set_mode(&st, 2); 
                        mav_fc_arm(&st, 1);
                        usleep(500000);
                        mav_fc_set_rc_override(&st, 1500, 1500, TAKEOFF_SPEED, 1500);
                        sleep(2);
                        mav_fc_set_rc_override(&st, 1500, 1500, HOVER_SPEED, 1500);
                    }
                    else if (strcmp(cmd, "land") == 0) {
                        reply_print("Initiating LAND mode...\n");
                        mav_fc_set_mode(&st, 9); 
                    }
                    else if (strcmp(cmd, "arm") == 0) {
                        reply_print("arming...\n");
                        mav_fc_arm(&st, 1);
                    }
                    else if (strcmp(cmd, "disarm") == 0) {
                        reply_print("disarming...\n");
                        mav_fc_arm(&st, 0);
                    } 
                    else if (strcmp(cmd, "mode stab") == 0) {
                        reply_print("mode STABILIZE\n");
                        mav_fc_set_mode(&st, 0);
                    } 
                    else if (strcmp(cmd, "mode alt") == 0) {
                        reply_print("mode ALT_HOLD\n");
                        mav_fc_set_mode(&st, 2);
                    }
                    else if (strcmp(cmd, "alt reset") == 0) {
                        alt_offset = st.alt_m; // Store current bad value as the offset
                        reply_print("[INFO] Altimeter Tared. Offset: %.2fm\n", alt_offset);
                        // Force a status update immediately to show the new 0.0
                        if (live_status_active) last_status_update = 0;
                    }
                    else if (strncmp(cmd, "throttle ", 9) == 0) {
                        int pwm;
                        sscanf(cmd + 9, "%d", &pwm);
                        reply_print("cmd throttle: %d\n", pwm);
                        mav_fc_set_throttle_pwm(&st, (uint16_t)pwm);
                    }
                    else if (strcmp(cmd, "help") == 0) {
                        print_help_remote();
                    }
                    else {
                        reply_print("unknown command: %s\n", cmd);
                        reply_print("type 'help' for command list\n");
                    }
                }
            }
        }
    }
    mav_fc_close(&st);
    if (udp_sock >= 0) close(udp_sock);
    return 0;
}