from picamera2 import Picamera2
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# === ?? ?? ? ?? ===
servo_pin = 12  # ?? ??? ??? GPIO ? ??
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# PWM ?? (50Hz)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(7.5)  # ?? ?? (0?)

# === set_angle ?? ===
def set_angle(angle):
    # set_angle ?? ??? ?? ?? ??? ??
    # ?? ??? ??? ???? ??? ??? ?????.
    # ?? ??, angle? ???? ?? ??? ??? ?? ?????.
    print(f"Setting steering angle to {angle} degrees")
    # ? ??? ??? ?? ??? ???? ??? ?????.
    # ???? ??? ???? ?????, ?? ???? ??? ?? ???? ???.
    pass

# === ??? ?? ===
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

while True:
    image = picam2.capture_array()
    image = cv2.rotate(image, cv2.ROTATE_180)
    height, width = image.shape[:2]

    # === 1. ?????? & ?? ===
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 0)

    # === 2. Sobel X ?? ?? ===
    sobel = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
    abs_sobel = np.absolute(sobel)
    max_val = np.max(abs_sobel)
    scaled_sobel = np.uint8(255 * abs_sobel / max_val) if max_val != 0 else abs_sobel
    edge_binary = np.zeros_like(scaled_sobel)
    edge_binary[(scaled_sobel >= 50) & (scaled_sobel <= 200)] = 255

    # === 3. HSV ?? ?? (??, ???) ===
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))
    yellow_mask = cv2.inRange(hsv, (15, 100, 100), (40, 255, 255))
    color_mask = cv2.bitwise_or(white_mask, yellow_mask)

    # === 4. ROI ?? ===
    roi_mask = np.zeros_like(edge_binary)
    roi_polygon = np.array([[
        (0, height),
        (0, int(height * 0.4)),
        (width, int(height * 0.4)),
        (width, height)
    ]], dtype=np.int32)
    cv2.fillPoly(roi_mask, roi_polygon, 255)

    inv_roi_mask = cv2.bitwise_not(roi_mask)

    blurred_image = cv2.GaussianBlur(image, (15, 15), 0)
    roi_area = cv2.bitwise_and(image, image, mask=roi_mask)
    non_roi_blur = cv2.bitwise_and(blurred_image, blurred_image, mask=inv_roi_mask)
    blended = cv2.add(roi_area, non_roi_blur)

    # === 5. ?? & ?? ??? ?? ===
    combined_mask = cv2.bitwise_or(edge_binary, color_mask)
    masked = cv2.bitwise_and(combined_mask, roi_mask)

    # === 6. ?? ?? ===
    contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    left_lane_center_x = None
    right_lane_center_x = None
    angle_deg = 0

    if contours:
        # ?? ??? ?? ??
        left_lane = None
        right_lane = None

        for contour in contours:
            if cv2.contourArea(contour) > 100:  # ?? ?? ??? ???? ??
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])  # ?? x ??
                    cy = int(M['m01'] / M['m00'])  # ?? y ??

                    # ??? ??/?? ??
                    if cx < width // 2:  # ?? ??
                        left_lane = (cx, cy)
                    else:  # ??? ??
                        right_lane = (cx, cy)

        # ??? ?? ??? ?? ????? ?
        if left_lane and right_lane:
            left_cx, left_cy = left_lane
            right_cx, right_cy = right_lane

            lane_center_x = (left_cx + right_cx) // 2
            frame_center_x = width // 2
            offset = lane_center_x - frame_center_x

            # ?? ??? ?? ??? ??? ???? ?? ??
            angle_rad = np.arctan2(offset, height)
            angle_deg = np.degrees(angle_rad)

            # ??? ????
            overlay = image.copy()
            cv2.line(overlay, (frame_center_x, height), (lane_center_x, height), (0, 255, 0), 2)
            cv2.circle(overlay, (lane_center_x, height), 5, (0, 255, 255), -1)
            cv2.putText(overlay, f"Angle: {angle_deg:.2f}", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        else:
            overlay = image.copy()
    else:
        overlay = image.copy()

    # === 7. ?? ?? ===
    red_highlight = np.zeros_like(image)
    red_highlight[:, :] = [255, 0, 0]  # ??? ??
    red_highlight_masked = cv2.bitwise_and(red_highlight, red_highlight, mask=masked)
    overlay = cv2.addWeighted(overlay, 1.0, red_highlight_masked, 1.0, 0)

    # === 8. ?? ?? ?? ===
    cv2.imshow("Original", image)
    cv2.imshow("ROI Applied", masked)
    cv2.imshow("Blended ROI View", blended)
    cv2.imshow("Overlay with Red Highlight", overlay)

    # ?? ?? ? set_angle ??
    set_angle(angle_deg)

    # ?? ??
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ?? ? ???
cv2.destroyAllWindows()
GPIO.cleanup()
