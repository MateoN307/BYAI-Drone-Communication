/* main_ground.c - Runs on Ground Computer */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <sys/socket.h>

/* Configure Target IP here (BeagleBoard IP) */
#define TARGET_IP "10.237.57.69" 
#define TARGET_PORT 12345

#define PING_INTERVAL_SEC 2

int main(void)
{
    int sock;
    struct sockaddr_in target_addr;
    char line[256];
    char last_cmd[256];

    /* Setup UDP Socket */
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

    printf("Ground Station Started.\n");
    printf("Sending to %s:%d\n", TARGET_IP, TARGET_PORT);
    printf("Type 'help' for commands, 'quit' to exit.\n");

    last_cmd[0] = '\0';

    while (1) {
        fd_set rfds;
        struct timeval tv;
        int retval;

        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);           // Watch for incoming UDP
        FD_SET(STDIN_FILENO, &rfds);

        /* TIMEOUT SETUP: Wake up every 1.0 second to send heartbeat */
        tv.tv_sec = PING_INTERVAL_SEC; 
        tv.tv_usec = 0;

        retval = select(sock + 1, &rfds, NULL, NULL, &tv);

        if (retval == 0) {
            /* Timeout occurred (User is idle), send Ping */
            char *ping_cmd = "ping";
            sendto(sock, ping_cmd, strlen(ping_cmd), 0,
                  (struct sockaddr *)&target_addr, sizeof(target_addr));
            continue; 
        }
            if (FD_ISSET(sock, &rfds)) {
                char buffer[1024];
                ssize_t len;
                struct sockaddr_in sender;
                socklen_t sender_len = sizeof(sender);

                len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                               (struct sockaddr *)&sender, &sender_len);
                if (len > 0) {
                    buffer[len] = '\0';
                    printf("%s", buffer); /* Buffer usually contains newlines */
                    fflush(stdout);
                }
            }

            /* 2. Handle user commands */
            if (FD_ISSET(STDIN_FILENO, &rfds)) {
                if (!fgets(line, sizeof(line), stdin))
                    break;

                /* Remove newline */
                char *nl = strchr(line, '\n');
                if (nl) *nl = '\0';

                char cmd[256];

                /* Handle Empty Input (Repeat Last Command) */
                if (line[0] == '\0') {
                    if (last_cmd[0] == '\0')
                        continue;
                    strncpy(cmd, last_cmd, sizeof(cmd));
                    printf("(repeat) %s\n", cmd);
                } else {
                    strncpy(cmd, line, sizeof(cmd));
                    if (strcmp(cmd, "quit") == 0) break;
                }
                cmd[sizeof(cmd) - 1] = '\0';

                /* Save for history */
                strncpy(last_cmd, cmd, sizeof(last_cmd));

                /* Send to Drone */
                sendto(sock, cmd, strlen(cmd), 0,
                       (struct sockaddr *)&target_addr, sizeof(target_addr));
            }
        }
        close(sock);
        return 0;
    }
