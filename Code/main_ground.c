/*
 * main_ground.c
 * * Platform: Linux Laptop / VM
 * Role: Ground Station & RC Transmitter
 * * Description:
 * - Reads PS4 Controller via Linux Joystick API (/dev/input/js1).
 * - Maps Raw Inputs (-32767 to +32767) to PWM (1000 to 2000).
 * - Sends UDP packets to the Drone at 20Hz.
 * - Implements "Emergency Stop" (Double Enter).
 * - Implements "Throttle Lock" to allow manual commands to override joystick.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <errno.h>
#include <time.h>

/* --- CONFIGURATION --- */
#define TARGET_IP "10.237.57.69"  
#define TARGET_PORT 12345
#define JOY_DEV "/dev/input/js1"  

// Send RC updates every 50ms (20Hz)
#define RC_UPDATE_INTERVAL_MS 50 

/* Deadzone settings (Raw Joystick Values) */
#define DEADZONE 4500

/* Standard Mode 2 Mapping */
#define AXIS_YAW      0 
#define AXIS_THROTTLE 1 
#define AXIS_ROLL     3 
#define AXIS_PITCH    4 

struct {
    int roll;
    int pitch;
    int throttle;
    int yaw;
} rc_state = {1500, 1500, 1500, 1500}; 

/* Flag: If 1, joystick throttle input is ignored until stick moves significantly */
int throttle_override_active = 0;

/* * Maps raw joystick input (-32767 to 32767) to PWM (1000 to 2000) 
 * Handles Deadzone and Inversion.
 */
int map_axis(int value, int invert) {
    if (abs(value) < DEADZONE) return 1500; 
    if (invert) value = -value;
    int adjusted_val = (value > 0) ? (value - DEADZONE) : (value + DEADZONE);
    int range = 32767 - DEADZONE;
    int delta = (adjusted_val * 500) / range;
    int pwm = 1500 + delta;
    if (pwm < 1000) pwm = 1000;
    if (pwm > 2000) pwm = 2000;
    return pwm;
}

long long current_time_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
}

/* --- MAIN --- */
int main(void)
{
    int sock, joy_fd;
    struct sockaddr_in target_addr;
    char line[256];
    long long last_rc_sent = 0;
    
    long long last_enter_time = 0; 

    /* 1. Setup UDP Socket */
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket");
        return 1;
    }

    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(TARGET_PORT);
    if (inet_pton(AF_INET, TARGET_IP, &target_addr.sin_addr) <= 0) {
        perror("inet_pton");
        return 1;
    }

    /* 2. Open Joystick */
    printf("Opening Joystick: %s ...\n", JOY_DEV);
    joy_fd = open(JOY_DEV, O_RDONLY | O_NONBLOCK);
    if (joy_fd < 0) {
        printf("[WARNING] Could not open joystick %s.\n", JOY_DEV);
    } else {
        printf("Joystick connected successfully.\n");
    }

    printf("Ground Station Started. Target: %s\n\n", TARGET_IP);
    printf("--- IMPORTANT CONTROLS ---\n\n");
    printf(" [ENTER] x2  : EMERGENCY STOP (Kill Motors)\n");
    printf(" 'help'      : Show All Other Drone Commands\n\n");
    printf("--- JOYSTICK MAPPING (Mode 2) ---\n\n");
    printf(" L-Stick Y   : Throttle (Up/Down)\n");
    printf(" L-Stick X   : Yaw (Rotation)\n");
    printf(" R-Stick Y   : Pitch (Forward/Back)\n");
    printf(" R-Stick X   : Roll (Left/Right)\n\n");
    printf("----------------\n");

    rc_state.throttle = 1000; // Default to idle

    while (1) {
        /* Emergency Timer Logic */
        if (last_enter_time > 0) {
            long long current_now = current_time_ms();
            long long diff = current_now - last_enter_time;
            
            if (diff > 500) {
                // Timeout Expired (0.5s passed)
                printf("\n[SYSTEM] Emergency Abort Sequence Deactivated. Resuming Standard Flight Protocol.\n");
                last_enter_time = 0; // Reset timer
            } else {
                // Live Countdown (Overwrites the current line)
                printf("\r[WARNING] EMERGENCY PRIME ACTIVE: %.1f sec remaining...  ", (500 - diff) / 1000.0);
                fflush(stdout);
            }
        }
        fd_set rfds;
        struct timeval tv;
        int max_fd = sock;

        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);
        FD_SET(STDIN_FILENO, &rfds);
        if (joy_fd >= 0) {
            FD_SET(joy_fd, &rfds);
            if (joy_fd > max_fd) max_fd = joy_fd;
        }

        tv.tv_sec = 0;
        tv.tv_usec = 20000; 

        int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);

        /* A. Handle Joystick Input */
        if (joy_fd >= 0 && FD_ISSET(joy_fd, &rfds)) {
            struct js_event je;
            while (read(joy_fd, &je, sizeof(je)) > 0) {
                if (je.type & JS_EVENT_AXIS) {
                    switch (je.number) {
                        case AXIS_ROLL: rc_state.roll = map_axis(je.value, 0); break;
                        case AXIS_PITCH: rc_state.pitch = map_axis(je.value, 0); break;
                        case AXIS_THROTTLE:
                            {
                                int new_thr = map_axis(je.value, 1);
                                /* Check for Override Lock */
                                if (throttle_override_active) {
                                    if (new_thr != 1500) { 
                                        throttle_override_active = 0; // Cancel override
                                        rc_state.throttle = new_thr;
                                        printf("[INFO] Joystick Regained Control (Throttle)\n");
                                    }
                                } else {
                                    rc_state.throttle = new_thr;
                                }
                            }
                            break;
                        case AXIS_YAW: rc_state.yaw = map_axis(je.value, 0); break;
                    }
                }
            }
        }

        /* B. Send RC Packet */
        long long now = current_time_ms();
        if (now - last_rc_sent > RC_UPDATE_INTERVAL_MS) {
            char rc_cmd[128];
            snprintf(rc_cmd, sizeof(rc_cmd), "move %d %d %d %d", 
                     rc_state.roll, rc_state.pitch, rc_state.throttle, rc_state.yaw);
            sendto(sock, rc_cmd, strlen(rc_cmd), 0, (struct sockaddr *)&target_addr, sizeof(target_addr));
            last_rc_sent = now;
        }

        /* C. Handle UDP Replies */
        if (FD_ISSET(sock, &rfds)) {
            char buffer[1024];
            ssize_t len;
            struct sockaddr_in sender;
            socklen_t sender_len = sizeof(sender);
            len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&sender, &sender_len);
            if (len > 0) {
                buffer[len] = '\0';
                printf("%s", buffer); 
                fflush(stdout);
            }
        }

        /* D. Handle User Keyboard Commands */
        if (FD_ISSET(STDIN_FILENO, &rfds)) {
            if (fgets(line, sizeof(line), stdin)) {
                char *nl = strchr(line, '\n');
                if (nl) *nl = '\0';

                // --- EMERGENCY STOP LOGIC ---
                if (strlen(line) == 0) {
                    long long now_ms = current_time_ms();
                    
                    // Check if pressed twice within 500ms
                    if (now_ms - last_enter_time < 500) {
                         printf("\n!!! EMERGENCY STOP TRIGGERED !!!\n");
                         
                         // REDUNDANCY: Send 5 kill packets
                         for(int i=0; i<5; i++) {
                             sendto(sock, "s", 1, 0, (struct sockaddr *)&target_addr, sizeof(target_addr));
                             usleep(2000); // 2ms delay between packets
                         }
                         last_enter_time = 0; // Reset timer
                    } else {
                        printf("[INFO] Press ENTER again to Emergency Stop\n");
                        last_enter_time = now_ms;
                    }
                    
                    continue; // Don't send empty packet
                }

                // --- MANUAL OVERRIDE LOGIC ---
                if (strncmp(line, "throttle ", 9) == 0) {
                    int pwm;
                    if (sscanf(line + 9, "%d", &pwm) == 1) {
                        rc_state.throttle = pwm;
                        throttle_override_active = 1; // Enable Lock
                        printf("[INFO] Manual Throttle Override applied: %d\n", pwm);
                        printf("(Move throttle stick to regain control)\n");
                    }
                }
                // Send normal text commands
                sendto(sock, line, strlen(line), 0,
                       (struct sockaddr *)&target_addr, sizeof(target_addr));

                if (strcmp(line, "quit") == 0) break;
            }
        }
    }

    if (joy_fd >= 0) close(joy_fd);
    close(sock);
    return 0;
}