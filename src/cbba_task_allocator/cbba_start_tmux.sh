#!/bin/bash

set -euo pipefail

session="my-project"

# Resolve important paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_ACTIVATE="${PROJECT_ROOT}/.venv/bin/activate"
PX4_DIR="${HOME}/PX4-Autopilot"
QGC_APP="${HOME}/Apps/QGround-5.0.8.AppImage"
NEW_METHOD_DIR="${SCRIPT_DIR}/new-method"
START_GAZEBO_SCRIPT="${NEW_METHOD_DIR}/1_start_gazebo.sh"
SPAWN_PX4_SCRIPT="${NEW_METHOD_DIR}/2_spawn_px4_sitl.sh"

echo "Using project root: ${PROJECT_ROOT}"
echo "Session name: ${session}"

# Basic path validation for this project
if [ ! -d "${PROJECT_ROOT}" ]; then
	echo "[ERROR] Project root not found: ${PROJECT_ROOT}" >&2
	exit 1
fi

if [ ! -f "${START_GAZEBO_SCRIPT}" ]; then
	echo "[ERROR] Gazebo start script not found: ${START_GAZEBO_SCRIPT}" >&2
	exit 1
fi

if [ ! -f "${SPAWN_PX4_SCRIPT}" ]; then
	echo "[ERROR] PX4 spawn script not found: ${SPAWN_PX4_SCRIPT}" >&2
	exit 1
fi

if [ ! -f "${VENV_ACTIVATE}" ]; then
	echo "[WARNING] Python venv not found at ${VENV_ACTIVATE}. Mission will run without venv." >&2
fi

if [ ! -d "${PX4_DIR}" ]; then
	echo "[WARNING] PX4 directory not found at ${PX4_DIR}. First window command may fail." >&2
fi

if [ ! -f "${QGC_APP}" ]; then
	echo "[WARNING] QGroundControl not found at ${QGC_APP}. Second window command may fail." >&2
fi

# If the session already exists, just attach to it
if tmux has-session -t "${session}" 2>/dev/null; then
	echo "tmux session '${session}' already exists. Attaching..."
	exec tmux attach -t "${session}"
fi

echo "Starting new tmux session '${session}'..."

# Window 1: Gazebo (standalone)
tmux new-session -d -s "${session}" -n "Gazebo" \
	"cd \"${NEW_METHOD_DIR}\" && ./1_start_gazebo.sh"

# Window 2: QGroundControl
tmux new-window -t "${session}:" -n "QGroundControl" \
	"\"${QGC_APP}\""

# Window 3: PX4 SITL spawner (3 UAVs)
tmux new-window -t "${session}:" -n "PX4-Spawn" \
	"cd \"${NEW_METHOD_DIR}\" && ./2_spawn_px4_sitl.sh 3 x500"

# Window 4: Build workspace (interfaces + CBBA + swarm + detection)
tmux new-window -t "${session}:" -n "Build" \
	"cd \"${PROJECT_ROOT}\" && colcon build --packages-select skyw_interfaces cbba_task_allocator skyw_swarm skyw_detection"

# Window 5: Formation server (action)
tmux new-window -t "${session}:" -n "Formation" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 rzssssss<aun skyw_swarm formation_server --ros-args -p drone_count:=3"

# Window 6: QR detection (leader camera)
tmux new-window -t "${session}:" -n "QR-Detect" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run skyw_detection qrcode_detector"

# Window 7: CBBA leader
tmux new-window -t "${session}:" -n "CBBA-Leader" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator cbba_leader_node --ros-args -p agent_id:=1"

# Window 8: CBBA leader state
tmux new-window -t "${session}:" -n "CBBA-L1-State" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator agent_state_node --ros-args -p agent_id:=1 -p px4_ns:='' -p capability_mask:=1"

# Window 9: CBBA leader agent
tmux new-window -t "${session}:" -n "CBBA-L1-Agent" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator cbba_agent_node --ros-args -p agent_id:=1 -p capability_mask:=1"

# Window 10: CBBA leader executor
tmux new-window -t "${session}:" -n "CBBA-L1-Exec" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator task_executor_node --ros-args -p agent_id:=1 -p drone_count:=3"

# Window 11: QR task adapter
tmux new-window -t "${session}:" -n "CBBA-QR" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator task_adapter_qr --ros-args -p input_topic:=/qr_decoded"

# Window 12: CBBA follower 2 state
tmux new-window -t "${session}:" -n "CBBA-F2-State" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator agent_state_node --ros-args -p agent_id:=2 -p px4_ns:='/px4_1' -p capability_mask:=0"

# Window 13: CBBA follower 2 agent
tmux new-window -t "${session}:" -n "CBBA-F2-Agent" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator cbba_agent_node --ros-args -p agent_id:=2 -p capability_mask:=0"

# Window 14: CBBA follower 2 executor
tmux new-window -t "${session}:" -n "CBBA-F2-Exec" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator task_executor_node --ros-args -p agent_id:=2 -p drone_count:=3"

# Window 15: CBBA follower 3 state
tmux new-window -t "${session}:" -n "CBBA-F3-State" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator agent_state_node --ros-args -p agent_id:=3 -p px4_ns:='/px4_2' -p capability_mask:=0"

# Window 16: CBBA follower 3 agent
tmux new-window -t "${session}:" -n "CBBA-F3-Agent" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator cbba_agent_node --ros-args -p agent_id:=3 -p capability_mask:=0"

# Window 17: CBBA follower 3 executor
tmux new-window -t "${session}:" -n "CBBA-F3-Exec" \
	"source \"${PROJECT_ROOT}/install/setup.bash\" && ros2 run cbba_task_allocator task_executor_node --ros-args -p agent_id:=3 -p drone_count:=3"

echo "tmux session '${session}' is ready. Attaching..."
exec tmux attach -t "${session}"

