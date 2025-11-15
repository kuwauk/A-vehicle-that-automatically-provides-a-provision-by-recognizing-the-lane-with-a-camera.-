from picamera2 import Picamera2
import cv2
import numpy as np

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

    # === 2. Sobel X ?? ===
    sobel = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
    abs_sobel = np.absolute(sobel)
    max_val = np.max(abs_sobel)
    scaled_sobel = np.uint8(255 * abs_sobel / max_val) if max_val != 0 else abs_sobel
    edge_binary = np.zeros_like(scaled_sobel)
    edge_binary[(scaled_sobel >= 50) & (scaled_sobel <= 200)] = 255

    # === 3. HSV ?? ?? ===
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))
    yellow_mask = cv2.inRange(hsv, (15, 100, 100), (40, 255, 255))
    color_mask = cv2.bitwise_or(white_mask, yellow_mask)

    # === 4. ROI ?? ===
    # 1. ROI ??? ??
    roi_mask = np.zeros_like(edge_binary)
    roi_polygon = np.array([[ 
        (0, height),
        (0, int(height * 0.4)),
        (width, int(height * 0.4)),
        (width, height)
    ]], dtype=np.int32)
    cv2.fillPoly(roi_mask, roi_polygon, 255)

    # 2. ROI ?? ?? ??
    inv_roi_mask = cv2.bitwise_not(roi_mask)

    # 3. ?? ????? ?? ??? ?? ???
    blurred_image = cv2.GaussianBlur(image, (15, 15), 0)

    # 4. ROI ?? ?? ??, ??? ?? ??
    roi_area = cv2.bitwise_and(image, image, mask=roi_mask)
    non_roi_blur = cv2.bitwise_and(blurred_image, blurred_image, mask=inv_roi_mask)
    blended = cv2.add(roi_area, non_roi_blur)


    # === 5. ?? ?? ===
    combined_mask = cv2.bitwise_or(edge_binary, color_mask)
    masked = cv2.bitwise_and(combined_mask, roi_mask)

    # === 6. ?? ????? ???? ===
    overlay = image.copy()
    red_highlight = np.zeros_like(image)
    red_highlight[:, :] = [255, 0, 0]  # RGB ???
    red_highlight_masked = cv2.bitwise_and(red_highlight, red_highlight, mask=masked)
    overlay = cv2.addWeighted(overlay, 1.0, red_highlight_masked, 1.0, 0)  # ?? ?? ?

    # === 7. ??? ===
    cv2.imshow("Original", image)
    cv2.imshow("ROI Applied", masked)
    cv2.imshow("Blended ROI View", blended)
    cv2.imshow("Overlay with Red Highlight", overlay)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
