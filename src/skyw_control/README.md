# Sky Warrior Control Package

This package provides scalable multi-drone control algorithms for testing and deploying control laws on UAV fleets. The default controller implements a leader-follower formation control approach based on the paper:

**Reference:** [Distributed leader-follower formation control for multiple quadrotors with weighted topology](https://hal.science/hal-01180491/document) by Zhicheng Hou and Isabelle Fantoni.

## Running the Controller

```bash
source ~/sky_warrior_ws/install/setup.bash
ros2 launch skyw_control <launch_file>
```
## Configuration

The following configuration files control the swarm behavior:

- **`config/swarm_config.json`** - Swarm setup and drone parameters
- **`config/control_config.json`** - Controller parameters and modes
- **`config/gains.yaml`** - PID controller gains
- **`config/Trajectories/`** - Predefined waypoint trajectories

### Modifying Configuration

Edit these configuration files on your host machine and rebuild the package:

```bash
colcon build --packages-select skyw_control
source install/setup.bash
```
### Mission implemented now

1. Take off all 3 drones to a safe hold altitude (`takeoff_z`, NED frame).
2. Keep drones 2 and 3 hovering near reference.
3. Send drone 1 (`x500_mono`) to `wall_1` target:
   - wall pose reference: `(x=5, y=0, z=-1, yaw=1.57)` in PX4 local frame
4. Hold and scan QR until decode or timeout.

### Run

```bash
colcon build --packages-select skyw_swarm
source install/setup.bash
ros2 launch skyw_swarm swarm_mission_scenario.launch.py use_sim_time:=true
```

### Useful launch overrides

```bash
ros2 launch skyw_swarm swarm_mission_scenario.launch.py \
  use_sim_time:=true \
  takeoff_z:=-2.5 \
  wall_x:=5.0 wall_y:=0.0 wall_z:=-1.0 wall_yaw:=1.57
```

### Important note about frames

PX4 local setpoints are NED, so **up is negative Z**.  
If your world reference is ENU `(5, 0, +1)`, the equivalent NED setpoint is typically `(5, 0, -1)` for altitude.
