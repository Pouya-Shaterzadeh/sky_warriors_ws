# SkyW Detection Package

This package provides onboard perception modules used by the Sky Warrior mission:

- QR detection and decoding from camera images
- Landing pad color detection (red and blue)

These modules are intended to feed higher-level mission logic (task allocation, formation update, and landing decisions).

## What This Package Includes

### 1) QR Detection

- Node: `qrcode_detector`
- Launch file: `skyw_detection/launch/qr_detection.launch.py`
- Input: camera image topic (`sensor_msgs/Image`)
- Output: decoded payload topic (default `/qr_decoded`, `std_msgs/String`)

Typical mission use:
- Decode formation target, destination, or landing instructions from a QR code.

### 2) Landing Pad Color Detection

- Node: `color_detector_node`
- Launch file: `skyw_detection/launch/detection.launch.py`
- Input: camera image (+ drone pose when map transform is enabled)
- Outputs:
	- `/skyw_detection/pads` (`geometry_msgs/PoseArray`)
	- `/skyw_detection/pad_markers` (`visualization_msgs/MarkerArray`)
	- Optional debug mask topics

Typical mission use:
- Identify red/blue pad location and support final landing guidance.

## Quick Start

After workspace build:

```bash
cd ~/sky_warrior_ws
colcon build --packages-select skyw_detection
source install/setup.bash
```

Run QR node:

```bash
ros2 launch skyw_detection qr_detection.launch.py
```

Run color detector:

```bash
ros2 launch skyw_detection detection.launch.py
```

If needed, remap input topics:

```bash
ros2 launch skyw_detection qr_detection.launch.py camera_topic:=/camera/image_raw
ros2 launch skyw_detection detection.launch.py camera_topic:=/camera/image_raw pose_topic:=/drone1/pose
```

## Gazebo/PX4 Integration

When running in simulation, you usually need bridges for camera and drone pose topics.

- Launch helper: `skyw_detection/launch/requirements.launch.py`
- Purpose:
	- Bridge Gazebo camera image to ROS 2 image topic
	- Start pose bridge used by detection stack

## Parameters You Will Commonly Tune

- QR:
	- `camera_topic`
	- `decoded_topic`
	- `binary_threshold`
	- `enable_visualization`
- Color:
	- `camera_topic`
	- `pose_topic`
	- `marker_frame_id`
	- `thresholds_file` (HSV config)
	- Camera intrinsics and camera-to-drone transform

## Documentation

Detailed guides are available in:

- `Docs/QR.md`
- `Docs/Color-Detector.md`

## Current Limitation

Planned feature not implemented yet:

- Save GPS/world coordinates automatically once a landing pad is detected.
