import cv2
import numpy as np

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(img, mask)

def detect_lane_angle(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    height, width = edges.shape
    roi_vertices = np.array([[
        (0, height),
        (width // 2, height // 2),
        (width, height)
    ]], dtype=np.int32)
    cropped_edges = region_of_interest(edges, roi_vertices)

    lines = cv2.HoughLinesP(cropped_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=150)
    if lines is None:
        return None, frame

    slopes = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 == x1:
            continue  # ???? ??
        slope = (y2 - y1) / (x2 - x1)
        slopes.append(slope)
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if not slopes:
        return None, frame

    avg_slope = np.mean(slopes)
    angle_rad = np.arctan(avg_slope)
    angle_deg = np.degrees(angle_rad)
    return angle_deg, frame

def main():
    cap = cv2.VideoCapture(0)  # USB ?? ??
    if not cap.isOpened():
        print("???? ? ? ????.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        angle, annotated_frame = detect_lane_angle(frame)
        if angle is not None:
            cv2.putText(annotated_frame, f"Angle: {angle:.2f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('Lane Detection', annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
