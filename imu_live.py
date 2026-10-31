"""Live IMU readout for the Arducam B0678 in the terminal. The B0678 only returns IMU data while video
is streaming, so a background thread keeps a 640x480 stream running (no window is shown).

Usage: python imu_live.py [seconds]   (no argument: run until Ctrl+C)"""
import math
import sys
import threading
import time

import cv2
from arducam_uvc_stereo_sdk import ImuConversionOptions, convert_imu, open_device, scan_devices

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else None
PRINT_EVERY = 0.1  # s

devices = scan_devices()
if not devices:
    sys.exit("no Arducam camera found (is it plugged in, and not open in another app?)")
info = devices[0]
backend, index = cv2.CAP_DSHOW, 1
for item in getattr(info, "opencv", None) or []:
    if "CAP_DSHOW" in item:
        index = int(item["CAP_DSHOW"])

cap = cv2.VideoCapture(index, backend)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    sys.exit("could not open the video stream (close the Camera app or any other program using the camera)")
stop = threading.Event()


def pump():
    while not stop.is_set():
        cap.read()


threading.Thread(target=pump, daemon=True).start()

cam = open_device(info)
cam.open_imu()
opts = ImuConversionOptions()
model = getattr(getattr(cam.capability_info.capability_report, "imu", None), "model", "") or "IMU"
print(f"{model} on Arducam 0x{info.vid:04x}:0x{info.pid:04x}, video index {index}. Ctrl+C to stop.\n")
print("   ax g     ay g     az g   |a| g  |   gx dps   gy dps   gz dps  |  temp C  reads/s")

t0 = last_print = time.monotonic()
reads = 0
try:
    while SECONDS is None or time.monotonic() - t0 < SECONDS:
        v = convert_imu(cam.read_imu(), opts)
        reads += 1
        now = time.monotonic()
        if now - last_print < PRINT_EVERY:
            continue
        if v.accel_x_g == v.accel_y_g == v.accel_z_g == 0:
            line = "waiting for data (the video stream is starting)..."
        else:
            mag = math.sqrt(v.accel_x_g ** 2 + v.accel_y_g ** 2 + v.accel_z_g ** 2)
            line = (f"{v.accel_x_g:+8.3f} {v.accel_y_g:+8.3f} {v.accel_z_g:+8.3f} {mag:6.3f}  | "
                    f"{v.gyro_x_dps:+8.2f} {v.gyro_y_dps:+8.2f} {v.gyro_z_dps:+8.2f}  | "
                    f"{v.temperature_c:6.1f}  {reads / (now - last_print):6.0f}")
        print("\r" + line.ljust(88), end="", flush=True)
        last_print, reads = now, 0
except KeyboardInterrupt:
    pass
finally:
    print()
    stop.set()
    cam.close_imu()
    cap.release()
