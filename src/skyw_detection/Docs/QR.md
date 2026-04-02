# QR Detection Guide

## Overview

The QR detector in this package is currently script-based and decodes QR data from the UAV camera stream.
It subscribes to a camera topic, decodes with `pyzbar`, and publishes decoded text to a ROS 2 topic.

- **Input image topic:** `/depth_cam/rgb/image_raw`
- **Output topic:** `/qr_decoded` (`std_msgs/String`)
- **Implementation file:** `skyw_detection/scripts/qrcode_detecter.py`

## Prerequisites

Before running QR detection:

1. PX4 SITL + Gazebo world is running.
2. Micro-XRCE-DDS Agent is running (for PX4 <-> ROS 2 bridge).
3. Camera image bridge is publishing ROS images.

Quick topic check:

```bash
ros2 topic list | rg "image_raw|qr_decoded"
```

## Run QR Detector

From the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/sky_warrior_ws/install/setup.bash
python3 ~/sky_warrior_ws/src/skyw_detection/skyw_detection/scripts/qrcode_detecter.py
```

## Verify Output

Read decoded payloads:

```bash
ros2 topic echo /qr_decoded
```

## How It Works

- Receives `sensor_msgs/Image` frames from the camera topic.
- Converts ROS image to OpenCV BGR image via `cv_bridge`.
- Runs `pyzbar.decode(...)` on each frame.
- Draws detection polygons and decoded text on a local OpenCV window.
- Publishes only newly changed decoded strings to avoid repeated spam.

## QR Creation Helper

You can generate QR images for simulation using:

- `skyw_detection/scripts/qrcode_creater.py`

It reads mission config files and creates:
- `qr-simple.jpg` (YAML-formatted data)
- `qr-competition.jpg` (JSON-formatted data)

Example:

```bash
python3 ~/sky_warrior_ws/src/skyw_detection/skyw_detection/scripts/qrcode_creater.py
```

## Notes and Current Limitations

- The detector is not yet exposed as a `ros2 run skyw_detection ...` console entry point.
- Camera topic is hardcoded in the current script (`/depth_cam/rgb/image_raw`).
- Visualization uses `cv2.imshow`, so it requires a GUI session.

## Troubleshooting

- **No messages on `/qr_decoded`:**
  - Confirm image topic is active:
    `ros2 topic hz /depth_cam/rgb/image_raw`
  - Ensure QR image is visible and large enough in camera FOV.
- **Import errors (`pyzbar`, `cv_bridge`, `cv2`):**
  - Install missing dependencies in your environment and source ROS setup again.
- **Window does not appear:**
  - Check if you are running with desktop/GUI access (X11/Wayland forwarding).
