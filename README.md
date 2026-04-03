# Sky Warrior Workspace - Docker Development Environment

## Workspace Structure

```
sky_warrior_ws/
├── .devcontainer/          # VS Code DevContainer configuration
│   ├── devcontainer.json   # VS Code config
│   ├── Dockerfile          # Base image definition
│   └── postCreateCommand.sh # Setup script
├── docker-compose.yml      # Docker Compose configuration
├── quick-start.sh          # Interactive helper script
└── src/                    # ROS 2 packages
── skyw_control/       # Swarm control algorithms
    ├── skyw_simulation/    # Simulation packages
    ├── skyw_bringup/       # Launch configurations
    ├── skyw_comm/          # Communication
    ├── skyw_interfaces/    # Custom messages
    ├── skyw_detection/     # QR Detect & Decode
    ├── skyw_swarm/         # Swarm behaviors
    └── skyw_utils/         # Utilities
```

## Prerequisites

- **PX4-Autopilot** - Flight control stack for drones
- **ROS 2 Humble** - Robot Operating System 2
- **Micro-XRCE-DDS Agent** - Communication bridge between PX4 and ROS 2
- **Gazebo Garden** - Simulation environment

## Building the Workspace

### Build Sky Warrior Packages

```bash
# Navigate to workspace
cd ~/sky_warrior_ws
source /opt/ros/humble/setup.bash

# Build custom messages first
colcon build --packages-select px4_msgs skyw_swarm
source install/setup.bash

# Build all packages
colcon build --symlink-install
source install/setup.bash
```

### Running Packages

```bash
source ~/sky_warrior_ws/install/setup.bash
ros2 launch <package_name> <launch_file>
```

## Configuration Tips

**Set up persistent ROS environment:**
```bash
# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/sky_warrior_ws/install/setup.bash" >> ~/.bashrc
```
