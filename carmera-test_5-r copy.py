import cv2
import numpy as np

def process_frame(image):
    height, width = image.shape[:2]

    # === 1. Grayscale & Gaussian Blur ===
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 0)

    # === 2. Sobel Edge Detection ===
    sobel = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
    abs_sobel = np.absolute(sobel)
    max_val = np.max(abs_sobel)
    scaled_sobel = np.uint8(255 * abs_sobel / max_val) if max_val != 0 else abs_sobel
    edge_binary = np.zeros_like(scaled_sobel)
    edge_binary[(scaled_sobel >= 50) & (scaled_sobel <= 200)] = 255

    # === 3. HSV Color Mask (White + Yellow) ===
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))
    yellow_mask = cv2.inRange(hsv, (15, 100, 100), (40, 255, 255))
    color_mask = cv2.bitwise_or(white_mask, yellow_mask)

    # === 4. ROI Mask ===
    roi_mask = np.zeros_like(edge_binary)
    roi_polygon = np.array([[ 
        (0, height),
        (0, int(height * 0.6)),
        (width, int(height * 0.6)),
        (width, height)
    ]], dtype=np.int32)
    cv2.fillPoly(roi_mask, roi_polygon, 255)

    # === 5. max - min ?? + ??? ===
    # === min-max ?? ?? ===

# min_mask? edge_binary? color_mask?? ???? ??
    min_mask = np.minimum(edge_binary, color_mask)

# threshold ???? ???
    _, min_mask = cv2.threshold(min_mask, 30, 255, cv2.THRESH_BINARY)

# ROI ???? ???? ?? ??? ??
    masked = cv2.bitwise_and(min_mask, roi_mask)


    # === 6. Contour ?? ?? ===
    
    contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    left_lane, right_lane = None, None
    angle_deg = 0

    for contour in contours:
        if cv2.contourArea(contour) > 100:
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                if cx < width // 2:
                    left_lane = (cx, cy)
                else:
                    right_lane = (cx, cy)

    overlay = image.copy()
    if left_lane and right_lane:
        left_cx, left_cy = left_lane
        right_cx, right_cy = right_lane
        lane_center_x = (left_cx + right_cx) // 2
        frame_center_x = width // 2
        offset = lane_center_x - frame_center_x
        angle_rad = np.arctan2(offset, height)
        angle_deg = np.degrees(angle_rad)
        cv2.line(overlay, (frame_center_x, height), (lane_center_x, height), (0, 255, 0), 2)
        cv2.circle(overlay, (lane_center_x, height), 5, (0, 255, 255), -1)
        cv2.putText(overlay, f"Angle: {angle_deg:.2f}", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

    # === 7. Red Highlight Overlay ===
    red_highlight = np.zeros_like(image)
    red_highlight[:, :] = [0, 0, 255]
    red_highlight_masked = cv2.bitwise_and(red_highlight, red_highlight, mask=masked)
    overlay = cv2.addWeighted(overlay, 1.0, red_highlight_masked, 1.0, 0)

    return image, masked, overlay

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera open failed.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read failed.")
            break

        original, masked, overlay = process_frame(frame)

        cv2.imshow("Original", original)
        cv2.imshow("ROI Applied", masked)
        cv2.imshow("Overlay", overlay)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()