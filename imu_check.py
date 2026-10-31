"""MVP check of the Arducam B0678 IMU: capability report, IMU read with and without a video stream,
then ~10 s of samples while streaming, with sanity checks and a CSV of every sample."""
import csv
import math
import statistics as st
import sys
import threading
import time
from pathlib import Path

import cv2
from arducam_uvc_stereo_sdk import (ImuConversionOptions, ReportedDeviceCapability, convert_imu, open_device,
                                    scan_devices)

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
OUT = Path(__file__).with_name("imu_samples.csv")

devices = scan_devices()
print(f"devices found: {len(devices)}")
for i, d in enumerate(devices):
    print(f"  [{i}] vid=0x{d.vid:04x} pid=0x{d.pid:04x} node={getattr(d, 'video_node', '')} "
          f"bus={d.bus_number} address={d.device_address} opencv={getattr(d, 'opencv', None)}")
if not devices:
    sys.exit("no Arducam device found by the SDK")
info = devices[0]
cam = open_device(info)

# --- 1. capability report
print("\n== capability report")
ci = cam.capability_info
rep = ci.capability_report
print(f"  source={ci.source.name}")
if rep is None:
    print("  (device does not report capabilities)")
else:
    names = [n for n in ("VIDEO", "FLASH", "IMU", "MICROPHONE", "EXT_TRIGGER")
             if rep.capability_bits & getattr(ReportedDeviceCapability, n)]
    print(f"  protocol {rep.major_version}.{rep.minor_version}  bits=0x{int(rep.capability_bits):016X}  features: {', '.join(names)}")
    if rep.flash:
        print(f"  flash user space: start=0x{rep.flash.user_space_start:08X} length={rep.flash.user_space_length} bytes")
    if rep.imu:
        c = rep.imu.conversion
        print(f"  IMU model: {rep.imu.model or '(not reported)'}")
        print(f"  IMU XU: unit={rep.imu.xu_unit} selector={rep.imu.xu_selector} length={rep.imu.xu_length} "
              f"decimal_exponent={rep.imu.decimal_exponent}")
        print(f"  conversion: temp offset {c.temperature_offset_c} C, {c.temperature_lsb_per_c} LSB/C, "
              f"{c.accel_lsb_per_g} LSB/g, {c.gyro_lsb_per_dps} LSB/(deg/s)")

cam.open_imu()
opts = ImuConversionOptions()

# --- 2. IMU read with no stream
print("\n== IMU read without a video stream")
try:
    raw = cam.read_imu()
    v = convert_imu(raw, opts)
    print(f"  ok: accel=({v.accel_x_g:.3f},{v.accel_y_g:.3f},{v.accel_z_g:.3f}) g  raw={raw}")
except Exception as exc:
    print(f"  failed (expected for B0678): {type(exc).__name__}: {exc}")

# --- 3. start the video stream in a thread
index = None
backend = cv2.CAP_DSHOW
opencv = getattr(info, "opencv", None)
for item in ([opencv] if isinstance(opencv, dict) else opencv or []):
    for name, src in item.items():
        if name in ("CAP_DSHOW", "CAP_MSMF") and index is None:
            backend, index = getattr(cv2, name), int(src)
index = 1 if index is None else index
cap = cv2.VideoCapture(index, backend)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
frames = {"n": 0, "shape": None}
stop = threading.Event()


def pump():
    while not stop.is_set():
        ok, f = cap.read()
        if ok:
            frames["n"] += 1
            frames["shape"] = f.shape


t = threading.Thread(target=pump, daemon=True)
t.start()
deadline = time.monotonic() + 5
while frames["n"] < 5 and time.monotonic() < deadline:
    time.sleep(0.05)
print(f"\n== stream: index {index} backend {backend}, frames so far {frames['n']}, shape {frames['shape']}")

# --- 4. read the IMU while streaming
rows, errors, first_error = [], 0, None
t0 = time.monotonic()
while time.monotonic() - t0 < SECONDS:
    ts = time.monotonic() - t0
    try:
        raw = cam.read_imu()
        v = convert_imu(raw, opts)
    except Exception as exc:
        errors += 1
        first_error = first_error or f"{type(exc).__name__}: {exc}"
        time.sleep(0.01)
        continue
    rows.append({"t_s": round(ts, 5), "temp_c": v.temperature_c,
                 "ax_g": v.accel_x_g, "ay_g": v.accel_y_g, "az_g": v.accel_z_g,
                 "gx_dps": v.gyro_x_dps, "gy_dps": v.gyro_y_dps, "gz_dps": v.gyro_z_dps,
                 "raw_ax": raw.accel_x_raw, "raw_ay": raw.accel_y_raw, "raw_az": raw.accel_z_raw,
                 "raw_gx": raw.gyro_x_raw, "raw_gy": raw.gyro_y_raw, "raw_gz": raw.gyro_z_raw,
                 "raw_temp": raw.temperature_raw})
elapsed = time.monotonic() - t0
stop.set()
t.join(timeout=2)
cap.release()
cam.close_imu()

print(f"\n== IMU while streaming ({elapsed:.1f} s; video frames {frames['n']})")
print(f"  reads ok: {len(rows)}  errors: {errors}{'  first: ' + first_error if first_error else ''}")
if not rows:
    sys.exit("no IMU samples")
with OUT.open("w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

keys = ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")
distinct = sum(1 for a, b in zip(rows, rows[1:]) if any(a[k] != b[k] for k in keys)) + 1
dts = [b["t_s"] - a["t_s"] for a, b in zip(rows, rows[1:])]
print(f"  read rate: {len(rows) / elapsed:.0f}/s, read interval median {st.median(dts) * 1000:.2f} ms" if dts else "")
print(f"  distinct samples: {distinct} (~{distinct / elapsed:.0f}/s, the sensor's update rate as seen over USB)")
for k in keys + ("temp_c",):
    vals = [r[k] for r in rows]
    print(f"  {k:7s} mean {st.mean(vals):9.4f}  std {st.pstdev(vals):8.4f}  min {min(vals):9.4f}  max {max(vals):9.4f}")
mag = [math.sqrt(r["ax_g"] ** 2 + r["ay_g"] ** 2 + r["az_g"] ** 2) for r in rows]
gyro = [math.sqrt(r["gx_dps"] ** 2 + r["gy_dps"] ** 2 + r["gz_dps"] ** 2) for r in rows]
m = st.mean(mag)
axis = max(("x", "y", "z"), key=lambda a: abs(st.mean(r[f"a{a}_g"] for r in rows)))
print(f"\n== sanity (camera at rest)")
print(f"  |accel| mean {m:.4f} g  -> {'PASS' if 0.9 < m < 1.1 else 'CHECK'} (expect ~1.000 g); gravity mostly on {axis}")
print(f"  |gyro| mean {st.mean(gyro):.3f} deg/s -> {'PASS' if st.mean(gyro) < 3 else 'CHECK (moving or large bias)'}")
tc = st.mean(r["temp_c"] for r in rows)
print(f"  temperature {tc:.1f} C -> {'PASS' if 10 < tc < 70 else 'CHECK'}")
print(f"  data changing between reads -> {'PASS' if distinct > 1 else 'FAIL (frozen values)'}")
print(f"\nsamples saved to {OUT}")
