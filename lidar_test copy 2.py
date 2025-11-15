import os
import termios
import time

SERIAL_PORT = "/dev/ttyUSB1"
BAUDRATE = termios.B460800


def setup_serial(fd):
    """라즈베리파이 호환 termios 설정"""
    attrs = termios.tcgetattr(fd)

    # 입력 / 출력 속도 설정 (직접 대입)
    attrs[4] = BAUDRATE  # ISPEED
    attrs[5] = BAUDRATE  # OSPEED

    # c_cflag 설정: 8N1, 로컬, 읽기 허용
    attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    attrs[2] |= termios.CLOCAL | termios.CREAD

    # 원시 모드 (canonical 모드 비활성화)
    attrs[3] = 0  # c_lflag
    attrs[1] = 0  # c_oflag
    attrs[0] = 0  # c_iflag

    # 블로킹 읽기 설정
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 5  # 0.5초

    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def hex_dump(data: bytes):
    print(" ".join(f"{b:02X}" for b in data))


def main():
    try:
        fd = os.open(SERIAL_PORT, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
        print(f"Opened {SERIAL_PORT}")
    except OSError as e:
        print(f"Unable to open {SERIAL_PORT}: {e}")
        return

    try:
        setup_serial(fd)

        # GET_INFO 명령 전송 (0xA5 0x50)
        req = bytes([0xA5, 0x50])
        os.write(fd, req)
        print("Sent: A5 50 (GET_INFO)")

        # 잠시 대기 후 읽기
        time.sleep(0.1)
        data = os.read(fd, 27)

        print(f"\nRead {len(data)} bytes:")
        hex_dump(data)

        if len(data) == 27:
            desc = data[:7]
            payload = data[7:]

            print("\nDescriptor (7 bytes):")
            hex_dump(desc)
            print("Payload (20 bytes):")
            hex_dump(payload)

            # Payload 파싱
            model = payload[0]
            firmware_minor = payload[1]
            firmware_major = payload[2]
            hardware = payload[3]
            serialnum = payload[4:]

            print(f"\nModel: {model}")
            print(f"Firmware: v{firmware_major}.{firmware_minor}")
            print(f"Hardware: {hardware}")
            print("Serial Number:", " ".join(f"{b:02X}" for b in serialnum))
        else:
            print("Unexpected response length")

    finally:
        os.close(fd)
        print("\nSerial closed.")


if __name__ == "__main__":
    main()
