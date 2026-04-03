# Sky Warrior Packages

This directory contains ROS 2 packages for the Sky Warrior multi-drone system. Each folder is a dedicated ROS 2 package for a specific task.

## Package Overview

- **skyw_simulation** - Gazebo simulation environment for spawning drones
- **skyw_control** - Multi-drone control algorithms and formation control
- **skyw_swarm** - Swarm behavior coordination
- **skyw_detection** - QR code detection and decoding
- **skyw_bringup** - Launch configurations
- **skyw_interfaces** - Custom ROS 2 messages and services
- **skyw_utils** - Utility functions and tools

## Quick Start
git clone

git clone https://github.com/Roboticistprogrammer/sky_warriors_ws.git && cd sky_warriors_ws
vcs import src/thirdparty < dependencies.repos
colcon build px4_msgs px4_ros_com


To get started with the simulation:

1. Use `skyw_bringup` package to spawn 3 drones as follows:
cd /skyw_warriors_ws/src/skyw_bringup
./startup.sh "/home/roboticistprogrammer/sky_warrior_ws/src/skyw_bringup/world/world.sdf"


2. Go to sky_warrior_ws/src/Sim-Scenario.md to see how to test each step of competition

## Development Status

**Note:** Several packages (`skyw_control`, `skyw_swarm`, `skyw_detection`) are under active development. Each team should:

1. Fork the repository
2. Focus on their assigned package
3. Create pull requests with implemented features
4. Document any issues or blockers

## Contributing

When working on a specific package, ensure you test your changes in the simulation environment before submitting pull requests. 
