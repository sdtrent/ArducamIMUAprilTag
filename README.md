# Arducam IMU + AprilTag viewer

Live video from an **Arducam B0678** USB camera (2 MP global shutter, USB ID `0C45:0678`) with its
built-in IMU readings and the IDs of visible **AprilTags** (36h11, the FRC family) drawn on the picture.

## Setup (Windows)

```
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python imu_video.py
```

Close the Windows Camera app (or anything else using the camera) first: only one program can stream it.

## Scripts

| Script | What it does |
|---|---|
| `imu_video.py` | Video window with accelerometer (g), gyroscope (deg/s), temperature, a gravity arrow and the IDs of visible tags (green outline + ID on each tag; the terminal prints the list when it changes). Keys: `q`/Esc quit, `s` snapshot PNG + IMU values. `imu_video.py 10` stops after 10 s. |
| `imu_live.py` | IMU only: one updating line in the terminal (Ctrl+C to stop). |
| `imu_check.py` | Verification: capability report, IMU read with and without video, ~10 s of samples with sanity checks (about 1 g at rest), every sample saved to `imu_samples.csv`. |
| `tag_check.py` | Offline check of the tag detector on synthetic tags, and writes the printable sheet `apriltags_36h11_ids_1-4.png` (tags 1-4, letter page). |

## The camera's IMU

Verified 2026-10-31 on Windows 11 with `arducam-uvc-stereo-sdk` 0.3.1:

- **IMU:** TDK ICM-42602, 6-axis. Scale 2048 LSB/g (±16 g) and 16.4 LSB/(deg/s) (±2000 deg/s); temperature 132.48 LSB/°C, offset 25 °C.
- **How it is read:** not a separate USB device. The data comes through a UVC extension unit (unit 4, selector 10, 17 bytes) inside the video interface, using the SDK's `scan_devices`, `open_device`, `open_imu`, `read_imu` and `convert_imu`.
- **Only while streaming:** with no video stream, reads succeed but return zeros. Every script here keeps a stream running.
- **Rates:** about 140 reads/s from Python on Windows (about 95 new samples/s). `imu_video.py` reads at about 30/s so the video keeps its 30 fps at 640x480.
- **At rest:** |accel| 0.997 g, accel noise about 0.0004 g; gyro bias about 0.1-0.5 deg/s per axis, noise about 0.02 deg/s.
- **Also reported by the camera:** 4 KB of flash user space (for calibration JSON) and an external trigger input.

Arducam's examples: <https://github.com/ArduCAM/Arducam-RoAD.B-USB-Demo>

## Known limits

- The gravity arrow assumes the IMU's x/y axes match the image; check by tilting the camera.
- 1600x1200 ran at only a few fps through OpenCV's DirectShow backend in a first test; 640x480 runs at 30 fps.
- Tag IDs only: distance and angle to a tag need a camera calibration and the printed tag size.
