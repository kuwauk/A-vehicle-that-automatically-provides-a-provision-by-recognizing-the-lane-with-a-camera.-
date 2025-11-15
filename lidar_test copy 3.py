import serial
import struct
import time

SERIAL_PORT = "/dev/ttyUSB1"
BAUDRATE = 460800


def hex_dump(data: bytes):
    print(" ".join(f"{b:02X}" for b in data))


def send_cmd(ser: serial.Serial, cmd: int):
    req = bytes([0xA5, cmd])
    ser.write(req)


def read_response(ser: serial.Serial, length: int) -> bytes:
    data = b''
    while len(data) < length:
        chunk = ser.read(length - len(data))
        if not chunk:
            break
        data += chunk
    return data


def get_health(ser: serial.Serial):
    print("\n[GET_HEALTH]")
    send_cmd(ser, 0x52)  # GET_HEALTH
    resp = read_response(ser, 10)  # 7-byte descriptor + 3-byte payload

    if len(resp) == 10:
        hex_dump(resp)
        status_code = resp[7]
        status_text = {0: "Good", 1: "Warning", 2: "Error"}.get(status_code, "Unknown")
        print(f"Status: {status_text} ({status_code})")
    else:
        print("Health read failed")


def stop_scan(ser: serial.Serial):
    print("\n[STOP]")
    send_cmd(ser, 0x25)  # STOP
    time.sleep(0.1)


def start_scan(ser: serial.Serial):
    print("\n[START_SCAN]")
    send_cmd(ser, 0x20)  # SCAN

    desc = read_response(ser, 7)
    if len(desc) != 7:
        print("Failed to read scan descriptor")
        return

    print("Scan descriptor: ", end="")
    hex_dump(desc)
    print("Receiving scan data (360 samples)...")

    for i in range(360):
        node = read_response(ser, 5)
        if len(node) == 5:
            # Lidar data: [angle_q6, dist_q2, quality]
            angle_raw = ((node[1] << 8) | node[0]) >> 1
            angle_deg = angle_raw / 64.0

            dist_raw = (node[3] << 8) | node[2]
            dist_mm = dist_raw / 4.0

            quality = node[4] >> 2

            print(f"Angle: {angle_deg:6.2f}°, Dist: {dist_mm:6.1f} mm, Quality: {quality}")
        else:
            print("Node read failed")
            break


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.5)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        get_health(ser)
        start_scan(ser)
        stop_scan(ser)
    finally:
        ser.close()
        print("\nSerial closed.")


if __name__ == "__main__":
    main()
