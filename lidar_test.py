import serial
import struct
import time
import threading
from collections import deque

SERIAL_PORT = "/dev/ttyUSB1"
BAUDRATE = 460800
TIMEOUT = 0.5  # seconds
QUALITY_THRESHOLD = 3

class RPLidarC1:
    def __init__(self, port: str = SERIAL_PORT, baudrate: int = BAUDRATE, timeout: float = TIMEOUT):
        self.ser = serial.Serial(
            port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self._running = False
        self._buffer = deque()
        self._lock = threading.Lock()
        self._thread = None

    def _read_loop(self):
        while self._running:
            data = self.ser.read(5)
            if len(data) < 5:
                continue
            raw_ang = ((data[1] << 8) | data[0]) >> 1
            angle = (raw_ang / 64.0) % 360.0
            raw_dist = (data[3] << 8) | data[2]
            dist = raw_dist / 4.0
            quality = data[4] >> 2
            if quality >= QUALITY_THRESHOLD and dist > 50:
                with self._lock:
                    self._buffer.append((angle, dist))

    def start(self):
        # send start scan command
        self.ser.write(bytes([0xA5, 0x20]))
        # discard descriptor
        self.ser.read(7)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self.ser.close()

    def get_count(self) -> int:
        """
        Return the number of buffered measurements and clear buffer.
        """
        with self._lock:
            count = len(self._buffer)
            self._buffer.clear()
        return count
    
    def get_distances(self) -> list[float]:
        with self._lock:
            distances = [d for (_, d) in self._buffer]
            self._buffer.clear()
        return distances

# Usage example (to be removed when imported as module):
# lidar = RPLidarC1()
# lidar.start()
# time.sleep(1)
# print(lidar.get_count())
# lidar.stop()
