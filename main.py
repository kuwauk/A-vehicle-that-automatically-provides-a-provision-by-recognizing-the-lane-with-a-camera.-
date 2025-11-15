import cv2
import numpy as np
from carmera_lane import CameraLaneProcessor

from servo_control import angle_to_pwm  # ? ?? ?? ?? ????

class MainApp:
    def __init__(self):
        self.detector = CameraLaneProcessor(width=640, height=480)

    def run(self):
        try:
            while True:
                image, masked, blended, overlay, angle = self.detector.process_frame()

                # ? ?? ?? ?? ??
                angle_to_pwm(angle)

                # ?? ??
                cv2.imshow("Original", image)
                cv2.imshow("ROI Applied", masked)
                cv2.imshow("Blended ROI View", blended)
                cv2.imshow("Overlay with Red Highlight", overlay)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.detector.cap.release()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    MainApp().run()
