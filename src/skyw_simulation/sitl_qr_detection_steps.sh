#!/bin/bash
# Step-by-step: PX4 + world.sdf + ROS bridges + arm drone 1 for QR / camera testing.
# Run each step in a separate terminal (or use the "check" step from one terminal).

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORLD="${WS}/src/skyw_bringup/world/world.sdf"
PX4_DIR="${HOME}/PX4-Autopilot"

usage() {
	echo "Usage: $0 <step>"
	echo "  1   Print command for Terminal 1: tmux SITL (3x PX4 + MicroXRCE)"
	echo "  2   Print commands for Terminal 2–4: ROS 2 (source + bridges + offboard + setpoint)"
	echo "  check   After step 1 is up: verify Gazebo world + clock (needs gz on PATH)"
	echo "  help"
}

step1() {
	echo "=== Terminal 1 — PX4 SITL (kills old tmux session name px4-sitl if you re-run startup) ==="
	echo "tmux kill-session -t px4-sitl 2>/dev/null || true"
	echo "bash \"${WS}/src/skyw_simulation/startup.sh\" \"${WORLD}\""
	echo ""
	echo "Wait until PX4-1 shows 'Gazebo world is ready' and models appear in gz gui."
}

step2() {
	echo "=== Terminal 2 — workspace + requirements (camera bridge + pose bridge) ==="
	echo "cd \"${WS}\" && source install/setup.bash"
	echo "ros2 launch skyw_detection requirements.launch.py world_name:=skyw_hexagon model_name:=x500_mono_cam_0"
	echo ""
	echo "=== Terminal 3 — offboard bridge (arms + offboard when setpoints flow) ==="
	echo "cd \"${WS}\" && source install/setup.bash"
	echo "ros2 launch skyw_swarm swarm_offboard.launch.py drone_count:=3 use_sim_time:=false"
	echo ""
	echo "=== Terminal 4 — hold setpoint for drone 1 (NED: z negative = up); tune x,y,z as needed ==="
	echo "cd \"${WS}\" && source install/setup.bash"
	echo "ros2 topic pub -r 20 /drone1/setpoint_position geometry_msgs/msg/PoseStamped \\"
	echo "  \"{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: -5.0}, orientation: {w: 1.0}}}\""
	echo ""
	echo "=== Optional — Terminal 5 — color detection ==="
	echo "ros2 launch skyw_detection detection.launch.py camera_topic:=/camera/image_raw pose_topic:=/drone1/pose"
}

check_gz() {
	if ! command -v gz >/dev/null 2>&1; then
		echo "[check] gz not on PATH; skip or install gz-harmonic."
		exit 1
	fi
	echo "[check] Topics matching /world/skyw_hexagon/ (expect clock + model topics once spawned):"
	gz topic -l 2>/dev/null | grep -E '^/world/skyw_hexagon/' | head -20 || true
}

case "${1:-help}" in
1) step1 ;;
2) step2 ;;
check) check_gz ;;
help | *) usage ;;
esac
