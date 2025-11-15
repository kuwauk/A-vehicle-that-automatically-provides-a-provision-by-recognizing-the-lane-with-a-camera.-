from picamera2 import Picamera2
from libcamera import Transform
import cv2
import numpy as np
import time

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (1280, 720)},
    transform=Transform(hflip=1, vflip=1)
)
picam2.configure(config)
picam2.start()
time.sleep(2)

while True:
    frame = picam2.capture_array()

    # ? ?? ??
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # ? ?? + ?? (? ????)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 30, 100)  # ???? low threshold ??

    # ? ???? ???? ?? ?? ??
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    edges = cv2.erode(edges, kernel, iterations=1)

    # ? ?? ??(ROI) ?? ??
    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (int(0.0 * width), height),
        (int(1.0 * width), height),
        (int(0.6 * width), int(height * 0.55)),
        (int(0.4 * width), int(height * 0.55))
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)

    # ? ?? ??
    lines = cv2.HoughLinesP(cropped_edges, 1, np.pi / 180, threshold=40, minLineLength=40, maxLineGap=15)
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

    # ? ?? ??
    combined = cv2.addWeighted(frame, 0.8, line_image, 1, 0)
    cv2.imshow("Optimized Road Line Detection", combined)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
