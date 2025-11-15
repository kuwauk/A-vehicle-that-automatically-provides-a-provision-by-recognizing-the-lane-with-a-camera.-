#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

#define SERIAL_PORT "/dev/ttyUSB1"
#define BAUDRATE B460800

int setup_serial(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) < 0) return -1;

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    return tcsetattr(fd, TCSANOW, &tty);
}

void hex_dump(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
    printf("\n");
}

void send_cmd(int fd, uint8_t cmd) {
    uint8_t req[2] = {0xA5, cmd};
    write(fd, req, 2);
}

int read_response(int fd, uint8_t* buf, int len) {
    int total = 0, n;
    while (total < len) {
        n = read(fd, buf + total, len - total);
        if (n <= 0) break;
        total += n;
    }
    return total;
}

void get_health(int fd) {
    printf("\n[GET_HEALTH]\n");
    send_cmd(fd, 0x52);  // GET_HEALTH
    uint8_t resp[10]; // 7-byte descriptor + 3-byte payload
    if (read_response(fd, resp, 10) == 10) {
        hex_dump(resp, 10);
        const char* status[] = {"Good", "Warning", "Error"};
        printf("Status: %s (%d)\n", status[resp[7]], resp[7]);
    } else {
        printf("Health read failed\n");
    }
}

void stop_scan(int fd) {
    printf("\n[STOP]\n");
    send_cmd(fd, 0x25);  // STOP
    usleep(100000); // wait a bit
}

void start_scan(int fd) {
    printf("\n[START_SCAN]\n");
    send_cmd(fd, 0x20); // SCAN

    uint8_t desc[7];
    if (read_response(fd, desc, 7) != 7) {
        printf("Failed to read scan descriptor\n");
        return;
    }

    printf("Scan descriptor: ");
    hex_dump(desc, 7);

    printf("Receiving scan data (sample 10 nodes)...\n");

    for (int i = 0; i < 36000; i++) {
        uint8_t node[5];
        if (read_response(fd, node, 5) == 5) {
            // ??? ?? ??: [angle_q6, dist_q2, quality]
            uint16_t angle_raw = ((node[1] << 8) | node[0]) >> 1;
            float angle_deg = angle_raw / 64.0f;

            uint16_t dist_raw = (node[3] << 8) | node[2];
            float dist_mm = dist_raw / 4.0f;

            int quality = node[4] >> 2;

            printf("Angle: %6.2f�, Dist: %6.1f mm, Quality: %d\n", angle_deg, dist_mm, quality);
        } else {
            printf("Node read failed\n");
            break;
        }
    }
}

int main() {
    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open");
        return EXIT_FAILURE;
    }
    if (setup_serial(fd) < 0) {
        perror("serial setup");
        close(fd);
        return EXIT_FAILURE;
    }

    get_health(fd);      // ?? ??
    start_scan(fd);      // ?? ?? + 10? ?? ??
    stop_scan(fd);       // ?? ??

    close(fd);
    return EXIT_SUCCESS;
}
