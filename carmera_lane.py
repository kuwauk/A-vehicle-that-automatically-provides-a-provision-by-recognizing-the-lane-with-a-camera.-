import cv2
import numpy as np
import threading
import queue
from xbox_con import get_controller_state
import serial, time
from picamera2 import Picamera2
from lidar_test import RPLidarC1
from libcamera import Transform


class CameraLaneProcessor:
    def __init__(self, width=320, height=240, roi_top_ratio=0.1):
        self.picam2 = Picamera2()
        cam_config = self.picam2.create_preview_configuration(
            main={'size': (width, height), 'format': "RGB888"},
            transform=Transform(hflip=True, vflip=True),
            controls={"FrameDurationLimits": (8333, 8333)}
        )
        self.picam2.configure(cam_config)
        self.picam2.start()

        self.roi_top_ratio = roi_top_ratio
        self.frame_queue = queue.Queue(maxsize=2)
        self.running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def _capture_loop(self):
        while self.running:
            frame = self.picam2.capture_array()
            h, w = frame.shape[:2]
            y0 = int(h * self.roi_top_ratio)
            roi = frame[y0:h, :]
            if not self.frame_queue.full():
                self.frame_queue.put((frame, roi, y0))

# (??)

    def process_frame(self):
        try:
            frame, roi, y0 = self.frame_queue.get(timeout=1)
        except queue.Empty:
            return None

        # --- [1] 전처리 ---
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # --- [2] 엣지 검출 ---
        edges = cv2.Canny(blur, 30, 150)

        # --- [3] 색상 기반 차선 검출 ---
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        white_mask = cv2.inRange(hsv, (0, 0, 160), (180, 80, 255))
        yellow_mask = cv2.inRange(hsv, (10, 60, 100), (45, 255, 255))
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # --- [4] 병합 및 잡음 제거 ---
        combined = cv2.bitwise_or(edges, color_mask)
        kernel = np.ones((5, 5), np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)

        # --- [5] 윤곽선 분석 ---
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h_roi, w_roi = combined.shape

        centers_left, centers_right = [], []
        for cnt in contours:
            if cv2.contourArea(cnt) > 150:
                M = cv2.moments(cnt)
                if M['m00']:
                    cx = int(M['m10'] / M['m00'])
                    if cx < w_roi // 2:
                        centers_left.append(cx)
                    else:
                        centers_right.append(cx)

        overlay = frame.copy()
        angle = 0.0

        left = np.mean(centers_left) if centers_left else None
        right = np.mean(centers_right) if centers_right else None

        if left is not None and right is not None:
            center_x = int((left + right) / 2)
            offset = center_x - (w_roi // 2)
            angle = np.degrees(np.arctan2(offset, h_roi))

            cv2.line(overlay, (0, y0 + h_roi), (center_x, y0 + h_roi), (0, 255, 0), 2)
            cv2.putText(overlay, f"Angle:{angle:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            angle = getattr(self, "last_angle", 0.0)
            cv2.putText(overlay, "Lane Lost", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # --- [6] angle 안정화 필터 ---
        if not hasattr(self, "angle_buffer"):
            self.angle_buffer = []
        self.angle_buffer.append(angle)
        if len(self.angle_buffer) > 5:
            self.angle_buffer.pop(0)
        angle = sum(self.angle_buffer) / len(self.angle_buffer)
        self.last_angle = angle

        # --- [7] 시각화 출력 ---
       
        cv2.imshow("Combined", combined)
        cv2.imshow("Overlay", overlay)

        return overlay, angle



    def release(self):
        self.running = False
        self.picam2.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=15)
        time.sleep(2)
    except serial.SerialException as e:
        print(f"[ERROR] Serial connection failed: {e}")
        exit()
    lidar = RPLidarC1(port="/dev/ttyUSB1", baudrate=460800, timeout=0.5)
    lidar.start()
    time.sleep(0.5)  # ?? ??? ?? ??

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
                overlay, angle = res

                pulse = int(1500 + (angle / 15.0) * 500)
                pulse = max(800, min(2200, pulse))
                pulse_hist.append(pulse)
                if len(pulse_hist) > 3:
                    pulse_hist.pop(0)
                pulse = int(sum(pulse_hist) / len(pulse_hist))

                dists = lidar.get_distances()              # List[float]
                min_dist = min(dists, default=float('inf'))

            # Throttle pulse: min_dist ? 130 ?? ??(1500)
                if min_dist <= 200:
                    pulse_2 = 1500
                    time.sleep(0.5)
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
        ser.close()