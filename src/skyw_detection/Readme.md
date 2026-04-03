# Sky Warrior Detection Package

This package provides QR code detection and decoding capabilities for UAVs using onboard cameras.

## Features

- Real-time QR code detection from drone camera feed
- QR code position estimation
- ROS 2 integration with PX4 autopilot

## Running the Detection Node

### Prerequisites

1. Ensure PX4 is connected via Micro-XRCE-DDS Agent
2. Launch simulation with a camera-equipped drone model

### Launch Detection

```bash
ros2 run skyw_detection qrcode_detector
```

## Simulation Setup

### Method 1: Integrated Build

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=skyw_multi_lz_world make px4_sitl gz_x500_mono_cam
```

### Method 2: Standalone PX4 Launch

```bash
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
  PX4_SYS_AUTOSTART=4009 \
  PX4_SIM_MODEL=gz_x500_mono_cam \
  PX4_GZ_MODEL=x500_mono_cam \
  PX4_GZ_WORLD=skyw_multi_lz_world \
  ./build/px4_sitl_default/bin/px4
```

## Configuration

- Camera topics and parameters can be configured in the package launch files
- Detection sensitivity and QR code processing parameters are adjustable in the node configuration
