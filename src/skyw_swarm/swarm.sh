#!/bin/bash

set -e

session="swarm"
WS_DIR="${HOME}/sky_warrior_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"
DRONE_COUNT="${DRONE_COUNT:-3}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTDDS_XML="${SCRIPT_DIR}/config/fastdds.xml"

echo "Workspace directory: ${WS_DIR}"
echo "Session name: ${session}"
echo "Fast DDS profile: ${FASTDDS_XML}"

if [ ! -f "${ROS_SETUP}" ]; then
	echo "[ERROR] ROS setup file not found: ${ROS_SETUP}" >&2
	exit 1
fi

if [ ! -f "${FASTDDS_XML}" ]; then
	echo "[ERROR] Fast DDS profile not found: ${FASTDDS_XML}" >&2
	exit 1
fi

# If the session already exists, just attach to it
if tmux has-session -t "${session}" 2>/dev/null; then
	echo "tmux session '${session}' already exists. Attaching..."
	exec tmux attach -t "${session}"
fi

echo "Starting new tmux session '${session}'..."

tmux new-session -d -s "${session}" -n "Bridge" \
	"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_USE_QOS_FROM_XML=1 FASTDDS_DEFAULT_PROFILES_FILE=\"${FASTDDS_XML}\" && source \"${ROS_SETUP}\" && if [ -f \"${WS_SETUP}\" ]; then source \"${WS_SETUP}\"; fi && ros2 run skyw_swarm px4_pose_bridge.py --ros-args -p drone_count:=${DRONE_COUNT}"

tmux new-window -t "${session}:" -n "Server" \
	"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_USE_QOS_FROM_XML=1 FASTDDS_DEFAULT_PROFILES_FILE=\"${FASTDDS_XML}\" && source \"${ROS_SETUP}\" && if [ -f \"${WS_SETUP}\" ]; then source \"${WS_SETUP}\"; fi && ros2 run skyw_swarm formation_server.py --ros-args -p drone_count:=${DRONE_COUNT}"

tmux new-window -t "${session}:" -n "Client" \
	"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_USE_QOS_FROM_XML=1 FASTDDS_DEFAULT_PROFILES_FILE=\"${FASTDDS_XML}\" && source \"${ROS_SETUP}\" && if [ -f \"${WS_SETUP}\" ]; then source \"${WS_SETUP}\"; fi && sleep 2 && ros2 run skyw_swarm formation_client.py"

echo "tmux session '${session}' is ready."
exec tmux attach -t "${session}"

