"""Offline check of the 36h11 detector settings used in imu_video.py, and a printable tag sheet."""
import cv2
import numpy as np
from pathlib import Path

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())

def tag(i, px):  # tag plus white quiet zone
    return cv2.copyMakeBorder(cv2.aruco.generateImageMarker(d, i, px), px // 5, px // 5, px // 5, px // 5,
                              cv2.BORDER_CONSTANT, value=255)

frame = np.full((480, 640), 170, np.uint8)
frame[40:40 + 168, 40:40 + 168] = tag(1, 120)
frame[250:250 + 112, 300:300 + 112] = tag(7, 80)            # smaller
t16 = tag(16, 120)
M = cv2.getPerspectiveTransform(np.float32([[0, 0], [168, 0], [168, 168], [0, 168]]),
                                np.float32([[440, 60], [600, 90], [590, 230], [450, 210]]))  # tilted
warped = cv2.warpPerspective(t16, M, (640, 480), borderValue=0)
mask = cv2.warpPerspective(np.full_like(t16, 255), M, (640, 480)) > 0
frame[mask] = warped[mask]
corners, ids, _ = det.detectMarkers(frame)
print("expected [1, 7, 16], detected", sorted(ids.flatten().tolist()) if ids is not None else [])

sheet = np.full((1100, 850), 255, np.uint8)                 # US letter at 100 dpi
for n, i in enumerate((1, 2, 3, 4)):
    r, c = divmod(n, 2)
    sheet[60 + r * 520:60 + r * 520 + 360, 45 + c * 400:45 + c * 400 + 360] = cv2.aruco.generateImageMarker(d, i, 360)
    cv2.putText(sheet, f"36h11 id {i}", (45 + c * 400, 60 + r * 520 + 400), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2)
cv2.imwrite(str(Path(__file__).with_name("apriltags_36h11_ids_1-4.png")), sheet)
print("wrote apriltags_36h11_ids_1-4.png")
