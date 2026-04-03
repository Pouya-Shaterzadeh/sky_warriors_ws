# Sky Warrior Packages

This directory contains ROS 2 packages for the Sky Warrior multi-drone system. Each folder is a dedicated ROS 2 package for a specific task.

## Package Overview

- **skyw_simulation** - Gazebo simulation environment for spawning drones
- **skyw_control** - Multi-drone control algorithms and formation control
- **skyw_swarm** - Swarm behavior coordination
- **skyw_detection** - QR code detection and decoding
- **skyw_comm** - Communication protocols
- **skyw_bringup** - Launch configurations
- **skyw_interfaces** - Custom ROS 2 messages and services
- **skyw_utils** - Utility functions and tools

## Quick Start

Make sure all prerequisites (ROS 2 Jazzy, Gazebo Harmonic, PX4-Autopilot, MicroXRCE-DDS Agent) are installed before proceeding.

### Method 1: Multi-Drone Simulation (Quick Launch)

1. From `src/`, run `./launch_gz_px4_sitl.sh 3 x500` to start Gazebo Harmonic + PX4 SITL and spawn 3 drones
2. Run the Micro-XRCE-DDS Agent to connect ROS 2 and QGroundControl to the UAVs
3. Launch the desired control or detection package

### Method 2: Single Drone with QR Detection (4 Terminal Setup)

You need **4 separate terminals** to run the full simulation + QR detection pipeline.

#### Terminal 1 — PX4 Simulation (Gazebo + Autopilot)

Launches the Gazebo Harmonic simulator with the `warehouse1` world and spawns a drone with a mono camera.

```bash
source ~/sky_warriors_ws/src/skyw_simulation/gz_env.sh
cd ~/PX4-Autopilot
PX4_GZ_WORLD=warehouse1 make px4_sitl gz_x500_mono_cam
```

> **Note:** Wait until you see `INFO [commander] Ready for takeoff!` before proceeding to the next terminals.

#### Terminal 2 — Micro-XRCE-DDS Agent (PX4 ↔ ROS 2 Bridge)

Bridges PX4 autopilot telemetry (GPS, attitude, battery, etc.) into ROS 2 topics.

```bash
MicroXRCEAgent udp4 -p 8888
```

#### Terminal 3 — Camera Bridge (Gazebo ↔ ROS 2)

Bridges the drone's camera feed from Gazebo transport into a ROS 2 image topic for the detection node.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera:=/depth_cam/rgb/image_raw
```

#### Terminal 4 — QR Code Detection Node

Runs the Python-based QR code detector that processes the camera feed in real-time.

```bash
source /opt/ros/jazzy/setup.bash
source ~/sky_warriors_ws/install/setup.bash
source ~/sky_warriors_ws/.venv/bin/activate
python3 ~/sky_warriors_ws/src/skyw_detection/skyw_detection/qrcode_detecter.py
```

### Architecture Overview

```
┌──────────┐   Terminal 2    ┌────────┐   Terminal 3    ┌──────────────┐
│   PX4    │── DDS Agent ──▶ │ ROS 2  │◀── gz_bridge ──│   Gazebo     │
│Autopilot │  (telemetry)    │  World │  (camera feed)  │  Simulator   │
└──────────┘                 └────┬───┘                 └──────────────┘
                                  │
                                  ▼
                          Terminal 4: Python
                          QR Code Detector
```

> **Tips:**
> - All 4 terminals run **continuously** — none of them "finish." They stay alive until you press `Ctrl+C`.
> - Open **QGroundControl** to arm the drone, take off, and fly towards QR codes inside the warehouse.
> - If `MicroXRCEAgent` fails with `bind error port 8888`, it means a previous instance is still running. Kill it with `pkill MicroXRCEAgent` first.

## Development Status

**Note:** Several packages (`skyw_control`, `skyw_swarm`, `skyw_detection`) are under active development. Each team should:

1. Fork the repository
2. Focus on their assigned package
3. Create pull requests with implemented features
4. Document any issues or blockers

## Contributing

When working on a specific package, ensure you test your changes in the simulation environment before submitting pull requests. 
