import cv2
import numpy as np

import cv2
import numpy as np

class CameraLaneDetector:
    def __init__(self, cam_index=0):
        # USB ?? ??
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise RuntimeError("Camera open failed.")

import cv2
import numpy as np

class CameraLaneDetector:
    def __init__(self, cam_index=0):
        # USB ?? ??
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise RuntimeError("Camera open failed.")

    def process_frame(self):
        # --- 1. ??? ?? ---
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Frame read failed.")
        # (?? ? ??)
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        h, w = frame.shape[:2]

        # --- 2. Sobel X edge ?? ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 0)
        sobel = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
        abs_sobel = np.absolute(sobel)
        maxv = np.max(abs_sobel)
        scaled = np.uint8(255 * abs_sobel / maxv) if maxv != 0 else np.zeros_like(abs_sobel, dtype=np.uint8)
        edge_binary = np.zeros_like(scaled)
        edge_binary[(scaled >= 50) & (scaled <= 200)] = 255

        # --- 3. HSV color mask ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask  = cv2.inRange(hsv, (0, 0, 200),  (180, 30, 255))
        yellow_mask = cv2.inRange(hsv, (15,100,100),(40,255,255))
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # --- 4. ROI mask (?? ?? 40%) ---
        roi_mask = np.zeros_like(edge_binary)
        pts = np.array([[
            (0,   h),
            (0,   int(h*0.4)),
            (w,   int(h*0.4)),
            (w,   h)
        ]], dtype=np.int32)
        cv2.fillPoly(roi_mask, pts, 255)

        inv_roi = cv2.bitwise_not(roi_mask)
        blurred_frame = cv2.GaussianBlur(frame, (15,15), 0)
        roi_area     = cv2.bitwise_and(frame, frame, mask=roi_mask)
        non_roi_blur = cv2.bitwise_and(blurred_frame, blurred_frame, mask=inv_roi)
        blended = cv2.add(roi_area, non_roi_blur)

        # --- 5. Combined mask & final masked ROI ---
        combined_mask = cv2.bitwise_or(edge_binary, color_mask)
        masked        = cv2.bitwise_and(combined_mask, roi_mask)

        # --- 6. Contour-based left/right lane detection ---
        contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        left_lane = right_lane = None
        angle_deg = 0.0

        for cnt in contours:
            if cv2.contourArea(cnt) > 100:
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    if cx < w//2:
                        left_lane = (cx, cy)
                    else:
                        right_lane = (cx, cy)

        # --- 7. Compute angle & draw overlay ---
        overlay = frame.copy()
        if left_lane and right_lane:
            lx, ly = left_lane
            rx, ry = right_lane
            lane_center_x   = (lx + rx) // 2
            frame_center_x  = w // 2
            offset          = lane_center_x - frame_center_x
            angle_rad       = np.arctan2(offset, h)
            angle_deg       = float(np.degrees(angle_rad))

            cv2.line(overlay,
                     (frame_center_x, h),
                     (lane_center_x,   h),
                     (0,255,0), 2)
            cv2.circle(
                overlay,
                (lane_center_x, h),
                5, (0,255,255), -1
            )
            cv2.putText(
                overlay,
                f"Angle: {angle_deg:.2f}",
                (10,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0,255,0), 2
            )

        # --- 8. Red highlight overlay ---
        red_hi = np.zeros_like(frame)
        red_hi[:] = [0,0,255]
        red_mask = cv2.bitwise_and(red_hi, red_hi, mask=masked)
        overlay = cv2.addWeighted(overlay, 1.0, red_mask, 1.0, 0)

        return frame, masked, blended, overlay, angle_deg

    def release(self):
        self.cap.release()
