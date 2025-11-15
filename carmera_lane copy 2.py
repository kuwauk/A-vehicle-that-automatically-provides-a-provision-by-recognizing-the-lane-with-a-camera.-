import cv2
import numpy as np
import threading
import queue
import serial
import time
from collections import deque
from xbox_con import get_controller_state
from picamera2 import Picamera2
from libcamera import Transform

QUALITY_THRESHOLD = 6

# ===================== RPLidarC1 =====================
class RPLidarC1:
    def __init__(self, port="/dev/ttyUSB1", baudrate=460800, timeout=0.5):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self._buffer = deque()
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def _read_loop(self):
        while self._running:
            data = self.ser.read(5)
            if len(data) < 5:
                continue

            # === 각도 계산 (Q6 format) ===
            raw_ang = ((data[1] & 0x7F) << 8) | data[0]
            angle = (raw_ang >> 1) / 64.0  # Degree
            angle = angle % 360  # 🔹 각도 보정

            # === 거리 계산 (mm) ===
            raw_dist = (data[3] << 8) | data[2]
            distance = raw_dist / 4.0

            if distance <= 0:
                continue

            # === 품질값 ===
            quality = data[4] >> 2

            # 🔥 Raw → Clean (필터링)
            if quality >= QUALITY_THRESHOLD and 148 < distance < 8000:
                with self._lock:
                    self._buffer.append((angle, distance, quality))


                    
                    
    def get_measurements(self):
        with self._lock:
            data = list(self._buffer)
            self._buffer.clear()
        return data



    def start(self):
        self.ser.write(bytes([0xA5, 0x20]))  # start scan
        self.ser.read(7)  # discard descriptor
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self.ser.close()

    def get_distances(self):
        with self._lock:
            distances = list(self._buffer)
            self._buffer.clear()
        return distances

# ===================== CameraLaneProcessor =====================



class CameraLaneProcessor:
    def __init__(self, width=640, height=480, roi_top_ratio=0.6):
        self.picam2 = Picamera2()
        cam_config = self.picam2.create_preview_configuration(
            main={'size': (width, height), 'format': "RGB888"},
            transform=Transform(hflip=True, vflip=True),
            controls={"FrameDurationLimits": (8333, 8333)}  # 120fps 제한
        )
        self.picam2.configure(cam_config)
        self.picam2.start()

        self.roi_top_ratio = roi_top_ratio
        self.frame_queue = queue.Queue(maxsize=2)
        self.running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()

        self.last_angle = 0.0
        self.angle_buffer = []

    def _capture_loop(self):
        while self.running:
            frame = self.picam2.capture_array()
            if not self.frame_queue.full():
                self.frame_queue.put(frame)

    def process_frame(self):
        try:
            frame = self.frame_queue.get(timeout=1)
        except queue.Empty:
            return None

        height, width = frame.shape[:2]

        # === ROI 설정 ===
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        roi_polygon = np.array([[
            (0, height),
            (0, int(height * self.roi_top_ratio)),
            (width, int(height * self.roi_top_ratio)),
            (width, height)
        ]], dtype=np.int32)
        cv2.fillPoly(roi_mask, roi_polygon, 255)

        roi = cv2.bitwise_and(frame, frame, mask=roi_mask)

        # === 1. Gray + Blur ===
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 0)

        # === 2. Sobel Edge ===
        sobel = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
        abs_sobel = np.absolute(sobel)
        max_val = np.max(abs_sobel)
        scaled_sobel = np.uint8(255 * abs_sobel / max_val) if max_val != 0 else abs_sobel
        edge_binary = np.zeros_like(scaled_sobel)
        edge_binary[(scaled_sobel >= 50) & (scaled_sobel <= 200)] = 255

        # === 3. HSV 색상 필터 ===
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))
        yellow_mask = cv2.inRange(hsv, (15, 100, 100), (40, 255, 255))
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # === 4. 엣지 + 색상 결합 ===
        combined_mask = cv2.bitwise_or(edge_binary, color_mask)
        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

        # === 5. ROI 마스크 적용 ===
        masked = cv2.bitwise_and(combined_mask, roi_mask)

        # === 6. 차선 컨투어 검출 ===
        contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers_left, centers_right = [], []
        h_roi, w_roi = masked.shape

        for contour in contours:
            if cv2.contourArea(contour) > 150:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    if cx < w_roi // 2:
                        centers_left.append(cx)
                    else:
                        centers_right.append(cx)

        overlay = frame.copy()
        angle_deg = 0.0
        left = np.mean(centers_left) if centers_left else None
        right = np.mean(centers_right) if centers_right else None

        # === 7. 중심 및 각도 계산 ===
        if left is not None and right is not None:
            lane_center_x = int((left + right) / 2)
            frame_center_x = w_roi // 2
            offset = lane_center_x - frame_center_x
            angle_rad = np.arctan2(offset, h_roi)
            angle_deg = np.degrees(angle_rad)

            cv2.line(overlay, (frame_center_x, height), (lane_center_x, height), (0, 255, 0), 2)
            cv2.circle(overlay, (lane_center_x, height), 5, (0, 255, 255), -1)
            cv2.putText(overlay, f"Angle: {angle_deg:.2f}", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        else:
            cv2.putText(overlay, "Lane Lost", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            angle_deg = self.last_angle

        # === 8. 각도 평활화 (moving average) ===
        self.angle_buffer.append(angle_deg)
        if len(self.angle_buffer) > 5:
            self.angle_buffer.pop(0)
        angle_deg = sum(self.angle_buffer) / len(self.angle_buffer)
        self.last_angle = angle_deg

        return overlay, angle_deg

    def release(self):
        self.running = False
        self.picam2.stop()
        cv2.destroyAllWindows()

# ===================== Main Loop =====================
if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=15)
        time.sleep(2)
    except serial.SerialException as e:
        print(f"[ERROR] Serial connection failed: {e}")
        exit()

    lidar = RPLidarC1()
    lidar.start()
    time.sleep(0.5)

    processor = CameraLaneProcessor(width=320, height=240, roi_top_ratio=0.1)
    camera_mode = True
    prev_y = False
    pulse_hist = []

    try:
        while True:
            state = get_controller_state()
            if state is None:
                continue

            fx, bx = state.rt, state.lt
            if state.y and not prev_y:
                camera_mode = not camera_mode
                print(f"*** Mode toggled: {'CAMERA' if camera_mode else 'MANUAL'} ***")
            prev_y = state.y

            if camera_mode:
                res = processor.process_frame()
                if res is None:
                    continue
                overlay, angle_deg = res
                

                pulse = int(1500 + (angle_deg / 15.0) * 500)
                pulse = max(800, min(2200, pulse))
                pulse_hist.append(pulse)
                if len(pulse_hist) > 3:
                    pulse_hist.pop(0)
                pulse = int(sum(pulse_hist) / len(pulse_hist))

                # ----------------- 라이다 거리 확인 -----------------
                # ----------------- 라이다 거리 확인 (정면 좌우 15도) -----------------
                measures = lidar.get_measurements()
                front_data = [d for (a, d, q) in measures if -10 <= a <= 10 or a >= 350]
                min_dist = min(front_data) if front_data else float('inf')

                cv2.putText(overlay, f"Front Dist: {min_dist:.1f} mm",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)



                if min_dist <= 200:
                    pulse_2 = 1500
                    time.sleep(1)
                else:
                    raw = (fx - bx) / 655.34 / 1.3
                    pulse_2 = int(1500 + raw)
                    pulse_2 = max(800, min(2200, pulse_2))

                cv2.imshow("Overlay", overlay)
            else:
                pulse = int(1500 + (state.ls_x / 100))
                pulse = max(800, min(2200, pulse))
                pulse_2 = int(1500 + ((fx - bx) / 450) / 1.3)
                pulse_2 = max(800, min(2200, pulse_2))
                debug_frame = 255 * np.ones((100, 400, 3), dtype=np.uint8)
                cv2.putText(debug_frame, f"Manual Pulse: {pulse}, {pulse_2}",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                cv2.imshow("Overlay", debug_frame)

            try:
                ser.write(f"{pulse},{pulse_2}\n".encode('ascii'))
            except serial.SerialException as e:
                print(f"[ERROR] Serial write failed: {e}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        processor.release()
        lidar.stop()
        ser.close()
