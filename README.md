# trigger_cam

ROS Noetic node for the **e-con See3CAM_24CUG** (2.3 MP global shutter USB camera) with hardware trigger mode support.

## Dependencies

- ROS Noetic
- `roscpp`, `sensor_msgs`, `cv_bridge`
- OpenCV 4.x
- Linux kernel HID/V4L2 (no extra drivers — camera is UVC compliant)

## Build

```bash
cd /opt/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Trigger mode (default)
```bash
roslaunch trigger_cam trigger_cam.launch trigger_mode:=true exposure:=33333
```

### Master mode (free-running)
```bash
roslaunch trigger_cam trigger_cam.launch trigger_mode:=false
```

### Override device or resolution
```bash
roslaunch trigger_cam trigger_cam.launch device:=/dev/video1 width:=1920 height:=1080
```

## Launch parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device` | `/dev/video0` | V4L2 device path |
| `width` | `1920` | Frame width (pixels) |
| `height` | `1080` | Frame height (pixels) |
| `fps` | `30` | Requested frame rate (master mode) |
| `format` | `MJPG` | Capture format (`MJPG` or `YUYV`) |
| `trigger_mode` | `true` | `true` = external hardware trigger, `false` = free-running |
| `auto_lock` | `true` | Lock AE/AWB when entering trigger mode |
| `exposure` | `33333` | Exposure time in microseconds (50–100,000,000) |
| `publish_raw` | `true` | Publish `sensor_msgs/Image` on `~/image_raw` |
| `publish_compressed` | `true` | Publish `sensor_msgs/CompressedImage` on `~/image_raw/compressed` |
| `jpeg_quality` | `20` | JPEG quality for compressed output (0–100) |

## Topics published

| Topic | Type | Description |
|-------|------|-------------|
| `~/image_raw` | `sensor_msgs/Image` | Raw BGR8 frames |
| `~/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG compressed frames |

## Trigger mode notes

- The hardware trigger input is **GPIO pin 4** (TRIG) on the camera's 6-pin connector
- Signal: active high, 1.17–5V, minimum pulse width 10 µs
- At 10 Hz trigger rate the maximum safe exposure is ~90,000 µs
- Exposure is set and locked before entering trigger mode; `auto_lock` freezes AE/AWB at the same time
- The first frame after startup is discarded to flush any pre-configuration buffer

## HID initialization sequence

The camera requires a specific HID command order on every start:

1. OS identification (`0x70`)
2. Reset to factory defaults (`0xFF`)
3. Set exposure
4. Set stream mode (trigger/master) with auto-lock
5. Disable flip — **must be last**, mode switch disturbs flip state in firmware

Flip disable is verified by readback and retried up to 5 times.
