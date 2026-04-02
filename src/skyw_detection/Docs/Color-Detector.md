# Color Detector Guide

## Overview

The color detector finds red and blue landing pads from the UAV camera feed using HSV masks and contour filtering.
It can estimate 3D positions of detected pads and publish RViz markers.

- **Main node:** `color_detector_node`
- **Launch file:** `launch/detection.launch.py`
- **Threshold config:** `config/color_thresholds.yaml`

## Inputs and Outputs

### Inputs
- `camera_topic` (default: `/camera/image_raw`) as `sensor_msgs/Image`
- `pose_topic` (default: `/drone1/pose`) as `geometry_msgs/PoseStamped` when map transform is enabled

### Outputs
- `/skyw_detection/pads` (`geometry_msgs/PoseArray`)
- `/skyw_detection/pad_markers` (`visualization_msgs/MarkerArray`)
- Optional debug masks:
  - `/skyw_detection/debug/red_mask`
  - `/skyw_detection/debug/blue_mask`

## Run the Node

### Option A: Direct run

```bash
source /opt/ros/jazzy/setup.bash
source ~/sky_warrior_ws/install/setup.bash
ros2 run skyw_detection color_detector_node
```

### Option B: Launch with parameters

```bash
source /opt/ros/jazzy/setup.bash
source ~/sky_warrior_ws/install/setup.bash
ros2 launch skyw_detection detection.launch.py
```

Example with custom camera and pose topics:

```bash
ros2 launch skyw_detection detection.launch.py \
  camera_topic:=/camera/image_raw \
  pose_topic:=/drone1/pose \
  marker_frame_id:=map
```

## Key Parameters

From `detection.launch.py` and node defaults:

- `camera_topic`: camera image stream for detection
- `pose_topic`: drone pose used for camera->drone->map transformation
- `use_drone_pose_transform`: place detections in map frame when `true`
- `marker_frame_id`: frame for output markers/poses (default `map`)
- `thresholds_file`: HSV thresholds YAML path (package-relative or absolute)
- `camera_fx`, `camera_fy`, `camera_cx`, `camera_cy`: intrinsics for depth and projection
- `camera_to_drone_translation`, `camera_to_drone_rpy`: camera extrinsics in drone frame

## Detection Pipeline

1. Convert image to BGR (`cv_bridge`).
2. Build HSV masks for red and blue.
3. Filter contours by area/shape thresholds.
4. Estimate depth from apparent pad size (pixel diameter).
5. Back-project pixel center to 3D point in camera frame.
6. Optionally transform to map frame using drone pose.
7. Publish `PoseArray` and `MarkerArray`.

## Tuning Tips

- Edit `config/color_thresholds.yaml` for HSV bounds and contour limits.
- Enable debug masks in thresholds (`general.debug`) to inspect segmentation quality.
- Start with static lighting conditions, then widen HSV ranges gradually.
- Verify camera intrinsics/extrinsics if 3D positions are shifted or unstable.

## Quick Verification

```bash
ros2 topic echo /skyw_detection/pads
ros2 topic echo /skyw_detection/pad_markers
```

Debug image preview:

```bash
ros2 topic hz /skyw_detection/debug/red_mask
ros2 topic hz /skyw_detection/debug/blue_mask
```

## Troubleshooting

- **No detections:**
  - Check `camera_topic` is correct and image stream exists.
  - Validate HSV ranges in `color_thresholds.yaml`.
- **Markers in wrong place:**
  - Re-check `camera_fx/fy/cx/cy`.
  - Re-check `camera_to_drone_translation` and `camera_to_drone_rpy`.
  - Confirm `pose_topic` publishes valid pose.
- **Intermittent detections:**
  - Reduce contour filters aggressiveness.
  - Improve scene lighting or pad contrast.
