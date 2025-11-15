from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'
# C1 ?? baudrate? 115200 ?? 256000?? ?? 115200?? ??

lidar = RPLidar(PORT_NAME, baudrate=460800)

try:
    lidar.start_motor()
    info = lidar.get_info()
    print("Device Info:", info)

    health = lidar.get_health()
    print("Health:", health)

    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            if distance > 0:
                print(f"Angle: {angle:.1f}�, Distance: {distance:.1f} mm")
                # ??? ?? ?? ?? ??

except Exception as e:
    print("Error:", e)

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
