#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

#define SERIAL_PORT "/dev/ttyUSB1"
#define BAUDRATE    B460800

// --- ??? ?? ?? ---
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
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 5;

    return tcsetattr(fd, TCSANOW, &tty);
}

// ?? ?? ??
void hex_dump(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
    printf("\n");
}

// 2??? ?? ?? (0xA5, cmd)
void send_cmd(int fd, uint8_t cmd) {
    uint8_t req[2] = {0xA5, cmd};
    write(fd, req, 2);
}

// ??? ???? ?? ??
int read_response(int fd, uint8_t* buf, int len) {
    int total = 0, n;
    while (total < len) {
        n = read(fd, buf + total, len - total);
        if (n <= 0) return total;
        total += n;
    }
    return total;
}

// ?? ?? (?? ? 1?? ??)
void get_health(int fd) {
    printf("\n[GET_HEALTH]\n");
    send_cmd(fd, 0x52);
    uint8_t resp[10];
    int n = read_response(fd, resp, 10);
    if (n == 10) {
        hex_dump(resp, 10);
        uint8_t code = resp[7];
        const char* status[] = {"Good", "Warning", "Error", "Unknown"};
        size_t idx = (code <= 2) ? code : 3;
        printf("Status: %s (%u)\n\n", status[idx], (unsigned)code);
    } else {
        fprintf(stderr, "Health read failed (got %d bytes)\n\n", n);
    }
}

// ?? ??? ??
void start_scan(int fd) {
    printf("[START_SCAN]\n");
    send_cmd(fd, 0x20);
    // 7??? descriptor ??
    uint8_t desc[7];
    if (read_response(fd, desc, 7) != 7) {
        fprintf(stderr, "Failed to read scan descriptor\n");
        exit(EXIT_FAILURE);
    }
    printf("Scan descriptor: ");
    hex_dump(desc, 7);
    printf("Streaming scan data...\n\n");
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

    // 1) ?? ?? (? ?)
    get_health(fd);
    // 2) ???? ?? ??
    start_scan(fd);

    // 3) ?? ??: 5??? ?? ?? ?? ?? & ??
    while (1) {
        uint8_t node[5];
        int n = read_response(fd, node, 5);
        if (n == 5) {
            // angle_q6: ?? 15?? >> 1
            uint16_t angle_raw = ((node[1] << 8) | node[0]) >> 1;
            float angle_deg = angle_raw / 64.0f;
            // dist_q2: ???
            uint16_t dist_raw = (node[3] << 8) | node[2];
            float dist_mm = dist_raw / 4.0f;
            int quality = node[4] >> 2;
            // ??
            printf("Angle: %6.2f�, Dist: %6.1f mm, Quality: %d\n",
                   angle_deg, dist_mm, quality);
        } else if (n == 0) {
            // ???? ?? EOF
            usleep(1000);
        } else {
            fprintf(stderr, "Partial read (%d bytes), skipping\n", n);
        }
    }

    // (?? ??)
    close(fd);
    return EXIT_SUCCESS;
}
