#!/bin/bash

set -e

session="px4-sitl"
PX4_DIR="${HOME}/PX4-Autopilot"
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
PX4_GZ_MODELS="${PX4_DIR}/Tools/simulation/gz/models"
PX4_GZ_WORLDS="${PX4_DIR}/Tools/simulation/gz/worlds"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTDDS_XML="${SCRIPT_DIR}/config/fastdds.xml"

echo "PX4 directory: ${PX4_DIR}"
echo "Session name: ${session}"
echo "Fast DDS profile: ${FASTDDS_XML}"

# Use only PX4 Gazebo resources to avoid stale paths from previously sourced shells.
if [ -d "${PX4_GZ_MODELS}" ] && [ -d "${PX4_GZ_WORLDS}" ]; then
	export GZ_SIM_RESOURCE_PATH="${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}"
elif [ -d "${PX4_GZ_MODELS}" ]; then
	export GZ_SIM_RESOURCE_PATH="${PX4_GZ_MODELS}"
fi

if [ ! -x "${PX4_BIN}" ]; then
	echo "[ERROR] PX4 binary not found or not executable: ${PX4_BIN}" >&2
	exit 1
fi

if [ ! -f "${FASTDDS_XML}" ]; then
	echo "[ERROR] Fast DDS profile not found: ${FASTDDS_XML}" >&2
	exit 1
fi

if tmux has-session -t "${session}" 2>/dev/null; then
	echo "tmux session '${session}' already exists. Attaching..."
	exec tmux attach -t "${session}"
fi

echo "Starting new tmux session '${session}'..."

# Window 1: PX4 SITL instance 1 (x500_mono_cam)
tmux new-session -d -s "${session}" -n "PX4-1" \
	"cd \"${PX4_DIR}\" && UXRCE_DDS_PRT=8888 PX4_SYS_AUTOSTART=4010 PX4_SIM_MODEL=gz_x500_mono_cam \"${PX4_BIN}\" -i 1"

# Window 2: PX4 SITL instance 2 (x500)
tmux new-window -t "${session}:" -n "PX4-2" \
	"cd \"${PX4_DIR}\" && UXRCE_DDS_PRT=8888 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"0,1\" PX4_SIM_MODEL=gz_x500 \"${PX4_BIN}\" -i 2"

# Window 3: PX4 SITL instance 3 (x500)
tmux new-window -t "${session}:" -n "PX4-3" \
	"cd \"${PX4_DIR}\" && UXRCE_DDS_PRT=8888 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"0,2\" PX4_SIM_MODEL=gz_x500 \"${PX4_BIN}\" -i 3"

# Window 4: Micro XRCE Agent
tmux new-window -t "${session}:" -n "MicroXRCE" \
	"export RMW_FASTRTPS_USE_QOS_FROM_XML=1 FASTDDS_DEFAULT_PROFILES_FILE=\"${FASTDDS_XML}\" && MicroXRCEAgent udp4 -p 8888"

echo "tmux session '${session}' is ready."
exec tmux attach -t "${session}"
