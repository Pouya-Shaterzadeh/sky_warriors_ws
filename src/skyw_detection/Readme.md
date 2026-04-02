# Sky Warrior Detection Package

This package provides the following:
QR code detection and decoding capabilities for UAVs using onboard cameras.
Landing Point Blue/Red color detections with opencv.

### Prerequisites

1. Ensure PX4 is connected via Micro-XRCE-DDS Agent
2. Launch simulation with a camera-equipped drone model

### Terminal 1 — PX4 Simulation (Gazebo + Autopilot)

Launches the Gazebo Harmonic simulator with the `warehouse1` world and spawns a drone with a mono camera.

```bash
source ~/sky_warriors_ws/src/skyw_simulation/gz_env.sh
cd ~/PX4-Autopilot
PX4_GZ_WORLD=warehouse1 make px4_sitl gz_x500_mono_cam
```
> **Note:** Wait until you see `INFO [commander] Ready for takeoff!` before proceeding to the next terminals.
### Terminal 2 — Micro-XRCE-DDS Agent (PX4 ↔ ROS 2 Bridge)

Bridges PX4 autopilot telemetry (GPS, attitude, battery, etc.) into ROS 2 topics.

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 3 — Camera Bridge (Gazebo ↔ ROS 2)

Bridges the drone's camera feed from Gazebo transport into a ROS 2 image topic for the detection node.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera:=/depth_cam/rgb/image_raw
```
ros2 run ros_gz_bridge parameter_bridge \
  /world/hexagon_world/model/x500_mono_cam_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image \
  --ros-args -r /world/hexagon_world/model/x500_mono_cam_0/link/camera_link/sensor/imager/image:=/camera/image_raw
---

### Terminal 4 — QR Code Detection Node

Runs the Python-based QR code detector that processes the camera feed in real-time.

```bash
source /opt/ros/jazzy/setup.bash
source ~/sky_warriors_ws/install/setup.bash
source ~/sky_warriors_ws/.venv/bin/activate
python3 ~/sky_warriors_ws/src/skyw_detection/skyw_detection/qrcode_detecter.py
```

### Terminal 5 — Color Detection Node



### Launch Detection

```bash
ros2 run skyw_detection qrcode_detector
```

## Simulation Setup

### Method 1: Integrated Build

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=warehouse1 make px4_sitl gz_x500_mono_cam
```

### Method 2: Standalone PX4 Launch

```bash
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
  PX4_SYS_AUTOSTART=4009 \
  PX4_SIM_MODEL=gz_x500_mono_cam \
  PX4_GZ_MODEL=x500_mono_cam \
  PX4_GZ_WORLD=warehouse1 \
  ./build/px4_sitl_default/bin/px4
```





## TROUBLESHOOT
## Configuration

- Camera topics and parameters can be configured in the package launch files
- Detection sensitivity and QR code processing parameters are adjustable in the node configuration
# Stop PX4 SITL
pkill -f "px4_sitl_default/bin/px4" || true
pkill -f "make px4_sitl" || true

# Stop Gazebo (gz sim + gui if any)
pkill -f "gz sim" || true
pkill -f "gz gui" || true

# Stop Micro XRCE DDS Agent
pkill -f "MicroXRCEAgent" || true
