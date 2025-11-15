from picamera2 import Picamera2, Preview
import time
import cv2

picam2 = Picamera2()

# ??? ???
width, height = 640, 480

# ?? ???? ? 80%? ?? (FOV 100? ? 80? ?? ??)
fov_ratio = 0.8
new_width = int(width * fov_ratio)
new_height = int(height * fov_ratio)
x_offset = int((width - new_width) / 2)
y_offset = int((height - new_height) / 2)

zoom_rect = (x_offset, y_offset, new_width, new_height)

config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (width, height)})
picam2.configure(config)
picam2.set_controls({"ScalerCrop": zoom_rect})
picam2.start()

time.sleep(2)
frame = picam2.capture_array()

cv2.imshow("FOV 80 approx", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
