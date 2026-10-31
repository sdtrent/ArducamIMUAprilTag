"""Live video from the Arducam B0678 with its IMU readings and the IDs of visible AprilTags (36h11, the FRC
family) drawn on the picture. The IMU is read in a background thread (it only returns data while the video
streams), so the video never waits for it.

Usage: python imu_video.py [seconds] [--snapshot FILE]
  keys: q or Esc quit, s save a snapshot (PNG) and the current IMU values next to this script"""
import math
import sys
import threading
import time
from pathlib import Path

import cv2
from arducam_uvc_stereo_sdk import ImuConversionOptions, convert_imu, open_device, scan_devices

args = sys.argv[1:]
SNAPSHOT = Path(args[args.index("--snapshot") + 1]) if "--snapshot" in args else None
numbers = [a for a in args if a.replace(".", "", 1).isdigit()]
SECONDS = float(numbers[0]) if numbers else None
WIDTH, HEIGHT = 640, 480
IMU_HZ = 50  # plenty for a display; reading flat out (~140/s) starved the video loop
HERE = Path(__file__).resolve().parent
WINDOW = "Arducam B0678 - IMU and AprilTags"
DETECTOR = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                   cv2.aruco.DetectorParameters())

devices = scan_devices()
if not devices:
    sys.exit("no Arducam camera found (is it plugged in, and not open in another app?)")
info = devices[0]
index = 1
for item in getattr(info, "opencv", None) or []:
    if "CAP_DSHOW" in item:
        index = int(item["CAP_DSHOW"])

cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
if not cap.isOpened():
    sys.exit("could not open the video stream (close the Camera app or any other program using the camera)")

cam = open_device(info)
cam.open_imu()
model = getattr(getattr(cam.capability_info.capability_report, "imu", None), "model", "") or "IMU"
latest = {"v": None, "rate": 0.0}
stop = threading.Event()


def read_imu():
    opts, reads, since = ImuConversionOptions(), 0, time.monotonic()
    while not stop.is_set():
        time.sleep(1 / IMU_HZ)
        try:
            latest["v"] = convert_imu(cam.read_imu(), opts)
        except Exception:
            time.sleep(0.01)
            continue
        reads += 1
        now = time.monotonic()
        if now - since >= 1.0:
            latest["rate"], reads, since = reads / (now - since), 0, now


def text(img, s, y, color=(255, 255, 255)):
    cv2.putText(img, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


def gravity_arrow(img, v):
    """Arrow showing the gravity direction in the image plane (camera x right, y down is assumed; check by tilting)."""
    cx, cy, r = img.shape[1] - 70, 70, 55
    cv2.circle(img, (cx, cy), r, (80, 80, 80), 1, cv2.LINE_AA)
    cv2.arrowedLine(img, (cx, cy), (int(cx + r * max(-1, min(1, v.accel_y_g))), int(cy + r * max(-1, min(1, v.accel_x_g)))),
                    (0, 200, 255), 2, cv2.LINE_AA, tipLength=0.25)


def find_tags(img):
    """Outline each visible 36h11 tag with its ID; returns the sorted IDs."""
    corners, ids, _ = DETECTOR.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return []
    for quad, tag in zip(corners, ids.flatten()):
        pts = quad.reshape(4, 2).astype(int)
        cv2.polylines(img, [pts], True, (0, 255, 0), 2, cv2.LINE_AA)
        cx, cy = pts.mean(axis=0).astype(int)
        label = str(tag)
        (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
        cv2.putText(img, label, (cx - w // 2, cy + h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(img, label, (cx - w // 2, cy + h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
    return sorted(int(t) for t in ids.flatten())


def overlay(img, fps, tags):
    v = latest["v"]
    cv2.rectangle(img, (0, 0), (430, 165), (0, 0, 0), -1)
    text(img, f"{model}   video {fps:4.1f} fps   IMU {latest['rate']:4.0f} reads/s", 20, (180, 220, 255))
    text(img, "tags (36h11): " + (", ".join(map(str, tags)) if tags else "none"), 150, (0, 255, 0) if tags else (160, 160, 160))
    if v is None or v.accel_x_g == v.accel_y_g == v.accel_z_g == 0:
        text(img, "waiting for IMU data...", 50, (0, 200, 255))
        return
    mag = math.sqrt(v.accel_x_g ** 2 + v.accel_y_g ** 2 + v.accel_z_g ** 2)
    text(img, f"accel g   x {v.accel_x_g:+6.3f}  y {v.accel_y_g:+6.3f}  z {v.accel_z_g:+6.3f}", 50)
    text(img, f"|accel|   {mag:6.3f} g", 75)
    text(img, f"gyro dps  x {v.gyro_x_dps:+7.2f} y {v.gyro_y_dps:+7.2f} z {v.gyro_z_dps:+7.2f}", 100)
    text(img, f"temp      {v.temperature_c:5.1f} C", 125)
    gravity_arrow(img, v)


def save(img):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = SNAPSHOT or HERE / f"snapshot_{stamp}.png"
    cv2.imwrite(str(path), img)
    v = latest["v"]
    if v is not None:
        path.with_suffix(".txt").write_text(
            f"accel_g {v.accel_x_g} {v.accel_y_g} {v.accel_z_g}\ngyro_dps {v.gyro_x_dps} {v.gyro_y_dps} {v.gyro_z_dps}\n"
            f"temp_c {v.temperature_c}\n")
    print(f"saved {path}")


threading.Thread(target=read_imu, daemon=True).start()
t0 = time.monotonic()
fps, frames, since = 0.0, 0, t0
frame = None
last_tags = []
try:
    while SECONDS is None or time.monotonic() - t0 < SECONDS:
        ok, frame = cap.read()
        if not ok:
            continue
        frames += 1
        now = time.monotonic()
        if now - since >= 1.0:
            fps, frames, since = frames / (now - since), 0, now
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        tags = find_tags(frame)
        if tags != last_tags:
            print("tags:", ", ".join(map(str, tags)) if tags else "none", flush=True)
            last_tags = tags
        overlay(frame, fps, tags)
        cv2.imshow(WINDOW, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key == ord("s"):
            save(frame)
    if SNAPSHOT and frame is not None:
        save(frame)
finally:
    stop.set()
    time.sleep(0.05)
    cam.close_imu()
    cap.release()
    cv2.destroyAllWindows()
