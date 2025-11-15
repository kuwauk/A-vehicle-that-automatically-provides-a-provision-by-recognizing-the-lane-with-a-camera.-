import cv2
import numpy as np

# === 1. ??? ???? ===
image = cv2.imread("road_image.jpg")  # ???? ?? ??? ??
if image is None:
    raise FileNotFoundError("???? ?? ? ????. ??? ?????.")

height, width = image.shape[:2]

# === 2. ?????? ?? ===
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# === 3. ???? ?? ===
blur = cv2.GaussianBlur(gray, (5, 5), 0)

# === 4. Sobel X ?? ?? ===
sobel = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
abs_sobel = np.absolute(sobel)
scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))

# === 5. Thresholding (?? ??) ===
binary = np.zeros_like(scaled_sobel)
binary[(scaled_sobel >= 20) & (scaled_sobel <= 100)] = 255

# === 6. ???? (ROI) ?? ===
roi = np.zeros_like(binary)
polygon = np.array([[
    (0, height),
    (0, int(height * 0.6)),
    (width, int(height * 0.6)),
    (width, height)
]], dtype=np.int32)
cv2.fillPoly(roi, polygon, 255)
masked = cv2.bitwise_and(binary, roi)

# === 7. ?? ??? ===
cv2.imshow("Original", image)
cv2.imshow("Sobel Binary", binary)
cv2.imshow("ROI Applied", masked)

cv2.waitKey(0)
cv2.destroyAllWindows()
