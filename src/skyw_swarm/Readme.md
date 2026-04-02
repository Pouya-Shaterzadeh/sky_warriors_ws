# Setup

colcon build --packages-select skyw_swarm
source install/setup.bash

# Run (3 terminals required)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp


## Terminal 1: PX4 Pose Bridge
ros2 run skyw_swarm px4_pose_bridge.py

## Terminal 2: Formation Server
ros2 run skyw_swarm formation_server.py --ros-args -p drone_count:=2

## Terminal 3: Formation Client
ros2 run skyw_swarm formation_client.py

# Verification
ros2 interface show skyw_swarm/action/SetFormation
ros2 node list
ros2 action list
ros2 topic list | grep pose
ros2 topic echo /drone1/pose

# Using px4 conversion node
cd ~/sky_warrior_ws
colcon build --packages-select skyw_swarm
source install/setup.bash

Terminal1
source install/setup.bash
ros2 run skyw_swarm px4_pose_bridge.py

Terminal2
source install/setup.bash
ros2 run skyw_swarm formation_server.py --ros-args -p drone_count:=3

Terminal3
source install/setup.bash
ros2 run skyw_swarm formation_client.py

# Examples

ros2 action send_goal /set_formation skyw_swarm/action/SetFormation "{formation_type: 'line', spacing: 2.0, altitude: 2.0, rotation: 0.0, drone_count: 3}"
ros2 action send_goal /set_formation skyw_swarm/action/SetFormation "{formation_type: 'v', spacing: 2.0, altitude: 2.0, rotation: 0.0, drone_count: 3}"
ros2 action send_goal /set_formation skyw_swarm/action/SetFormation "{formation_type: 'arrow_head', spacing: 2.0, altitude: 2.0, rotation: 0.0, drone_count: 3}"

# YAML config usage
ros2 run skyw_swarm formation_server.py --ros-args --params-file $(ros2 pkg prefix skyw_swarm)/share/skyw_swarm/config/formation.yaml
ros2 run skyw_swarm formation_client.py --ros-args --params-file $(ros2 pkg prefix skyw_swarm)/share/skyw_swarm/config/formation.yaml

# Launch-file workflow (recommended)
ros2 launch skyw_swarm swarm_launch.py

# Behavior-only launch (Aerostack2-style)
ros2 launch skyw_swarm swarm_behavior.launch.py

# Offboard bridge launch (PX4 setpoints)
ros2 launch skyw_swarm swarm_offboard.launch.py

# Full mission scenario (recommended for SITL demo)
# - Starts pose bridge + formation server + offboard bridge
# - Sends takeoff-like goal to 5m, then transitions to line formation
ros2 launch skyw_swarm swarm_mission_scenario.launch.py

# Scenario customization
ros2 launch skyw_swarm swarm_mission_scenario.launch.py \
    drone_count:=3 \
    takeoff_altitude:=5.0 \
    mission_altitude:=5.0 \
    mission_spacing:=2.0 \
    initial_goal_delay_s:=3.0 \
    mission_delay_s:=8.0

# Launch with YAML + client auto-start
ros2 launch skyw_swarm swarm_launch.py \
	params_file:=$(ros2 pkg prefix skyw_swarm)/share/skyw_swarm/config/formation.yaml \
	drone_count:=3 \
	start_client:=true \
	client_delay:=2.0

# Optional launch args
ros2 launch skyw_swarm swarm_launch.py \
	use_sim_time:=true \
	log_level:=debug \
	namespace:=swarm

# Test a specific formation goal while launch is running
ros2 action send_goal /set_formation skyw_swarm/action/SetFormation \
"{formation_type: 'line', spacing: 2.0, altitude: 3.0, rotation: 0.0, drone_count: 3}"

# Try different formations without changing YAML
ros2 action send_goal /set_formation skyw_swarm/action/SetFormation \
"{formation_type: 'v', spacing: 2.0, altitude: 3.0, rotation: 0.0, drone_count: 3}"