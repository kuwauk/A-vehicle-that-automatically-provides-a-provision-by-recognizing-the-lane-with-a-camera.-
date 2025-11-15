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
    if (tcgetattr(fd, &tty) < 0) {
        perror("tcgetattr");
        return -1;
    }
    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8 bits
    tty.c_cflag &= ~PARENB;                       // no parity
    tty.c_cflag &= ~CSTOPB;                       // 1 stop bit
    tty.c_cflag |= CLOCAL | CREAD;                // ignore modem controls, enable reading

    tty.c_lflag = 0;                              // no canonical processing
    tty.c_oflag = 0;                              // no remapping, no delays
    tty.c_iflag = 0;                              // no special handling

    tty.c_cc[VMIN]  = 1;   // read blocks until at least 1 byte arrives
    tty.c_cc[VTIME] = 5;   // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }
    return 0;
}

int main() {
    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "Unable to open %s: %s\n", SERIAL_PORT, strerror(errno));
        return EXIT_FAILURE;
    }
    if (setup_serial(fd) < 0) {
        close(fd);
        return EXIT_FAILURE;
    }

    // GET_INFO ??: 0xA5 0x50
    uint8_t req[] = {0xA5, 0x50};
    if (write(fd, req, sizeof(req)) != sizeof(req)) {
        perror("write");
        close(fd);
        return EXIT_FAILURE;
    }

    // ?? ?????(7???) + ??(20???) ??
    uint8_t buf[27];
    int n = read(fd, buf, sizeof(buf));
    if (n < 0) {
        perror("read");
    } else {
        printf("Read %d bytes:\n", n);
        for (int i = 0; i < n; i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");
        // (?? ?? ?? ?? ??)
    }

    close(fd);
    return EXIT_SUCCESS;
}
