# QR Detection Guide

## Overview

The QR node decodes QR codes from a `sensor_msgs/Image` stream using **pyzbar** on a **grayscale + fixed binary threshold** image (same idea as the tutorial [decode_qr.py](https://github.com/Pouya-Shaterzadeh/pkg_cv_ros_tutorial_by_dhanuzch/blob/main/scripts/decode_qr.py)).

- **Node implementation:** `skyw_detection/python/qr_code_detector_node.py`
- **Standard run:** `ros2 run skyw_detection qrcode_detector`
- **Launch:** `ros2 launch skyw_detection qr_detection.launch.py`
- **Default camera topic:** `/camera/image_raw`
- **Default output:** `/qr_decoded` (`std_msgs/String`)

## Prerequisites

1. PX4 SITL + Gazebo (or hardware) with a camera publishing ROS images.
2. Micro-XRCE-DDS Agent when using PX4.
3. A **ros_gz_bridge** `parameter_bridge` (or equivalent) so Gazebo camera images reach ROS 2 — same role as the bridge in [1_world_and_script.launch.py](https://github.com/Pouya-Shaterzadeh/pkg_cv_ros_tutorial_by_dhanuzch/blob/main/launch/1_world_and_script.launch.py).

This package provides `requirements.launch.py` to bridge the Gazebo camera topic to `/camera/image_raw` (tune `world_name` / `model_name` to match your sim).

Quick topic check:

```bash
ros2 topic list | rg "image_raw|qr_decoded"
```

## Run (recommended)

After building and sourcing the workspace:

```bash
source /opt/ros/<distro>/setup.bash
source ~/sky_warrior_ws/install/setup.bash
ros2 run skyw_detection qrcode_detector
```

With camera remapping via parameters:

```bash
ros2 run skyw_detection qrcode_detector --ros-args -p camera_topic:=/camera/image_raw
```

## Launch file

```bash
ros2 launch skyw_detection qr_detection.launch.py camera_topic:=/camera/image_raw
```

Arguments: `camera_topic`, `decoded_topic`, `enable_visualization`, `binary_threshold`.

## Verify output

```bash
ros2 topic echo /qr_decoded
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_topic` | `/camera/image_raw` | Input image |
| `decoded_topic` | `/qr_decoded` | Decoded string publisher |
| `enable_visualization` | `true` | OpenCV window |
| `binary_threshold` | `45` | `cv2.threshold` value for pyzbar input |
| `publish_only_on_change` | `true` | Avoid republishing identical payloads |

## Troubleshooting

- **No `/qr_decoded`:** `ros2 topic hz <your_camera_topic>` — fix the bridge or `camera_topic` first.
- **No GUI window:** set `enable_visualization:=false` headless, or use X11 forwarding.
- **Poor decode rate:** adjust `binary_threshold` or lighting; ensure QR fills enough of the image.
