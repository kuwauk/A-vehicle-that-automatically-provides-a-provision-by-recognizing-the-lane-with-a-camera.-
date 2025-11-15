import cv2
import numpy as np

# === 1. ??? ???? ===
image = cv2.imread("road_image.jpg")
if image is None:
    raise FileNotFoundError("???? ?? ? ????.")

height, width = image.shape[:2]

# === 2. ?????? + ?? ===
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (9, 9), 0)  # ?? ?? ???? ? ????

# === 3. Sobel X ?? ?? ===
sobel = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
abs_sobel = np.absolute(sobel)
scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))

# === 4. Threshold ?? (?? ??) ===
binary = np.zeros_like(scaled_sobel)
# Threshold ? ?? ??
binary[(scaled_sobel >= 50) & (scaled_sobel <= 200)] = 255

# === 5. ?? ?? ?? (HSV) ===
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# ?? ??
white_lower = np.array([0, 0, 200])
white_upper = np.array([180, 30, 255])
white_mask = cv2.inRange(hsv, white_lower, white_upper)

# ??? ??
yellow_lower = np.array([15, 100, 100])  # ?? ?? ??
yellow_upper = np.array([40, 255, 255])
yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

# ?? ?? ??? (?? + ???)
color_mask = cv2.bitwise_or(white_mask, yellow_mask)

# === 6. ROI ?? (?? ?? 50%) ===
roi_mask = np.zeros_like(binary)
polygon = np.array([[
    (0, height),
    (0, int(height * 0.5)),  # ?? 50%? ??
    (width, int(height * 0.5)),  # ?? 50%? ??
    (width, height)
]], dtype=np.int32)
cv2.fillPoly(roi_mask, polygon, 255)

# === 7. ?? ?? ?? (???) ===
# Sobel ??? ?? ??? ?? (??? ?? ??? ??)
combined = cv2.bitwise_or(binary, color_mask)
masked = cv2.bitwise_and(combined, roi_mask)

# === 8. ??? ???? ??? ===
overlay = image.copy()
red_mask = np.zeros_like(image)
red_mask[:, :] = [0, 0, 255]  # ???

# ROI ?? ? ???? ??? ??? ???
highlight = cv2.bitwise_and(red_mask, red_mask, mask=masked)
overlay = cv2.addWeighted(overlay, 1.0, highlight, 0.6, 0)

# === 9. ??? ===
cv2.imshow("Original", image)
cv2.imshow("Sobel + Color Mask", combined)
cv2.imshow("ROI Applied (Binary)", masked)  # ???? ROI ?? ??? ??
cv2.imshow("Final Overlay (Red Highlight)", overlay)

cv2.waitKey(0)
cv2.destroyAllWindows()
