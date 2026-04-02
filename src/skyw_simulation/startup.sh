#!/bin/bash

set -e

session="px4-sitl"
PX4_DIR="${HOME}/PX4-Autopilot"
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
PX4_GZ_MODELS="${PX4_DIR}/Tools/simulation/gz/models"
PX4_GZ_WORLDS="${PX4_DIR}/Tools/simulation/gz/worlds"
PX4_GZ_PLUGINS="${PX4_DIR}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
PX4_GZ_SERVER_CONFIG="${PX4_DIR}/src/modules/simulation/gz_bridge/server.config"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CUSTOM_WORLD_DIR_DEFAULT="${SCRIPT_DIR}/world"
CUSTOM_WORLD_FILE="${1:-}"
RMW_FASTRTPS_USE_QOS_FROM_XML=1
FASTDDS_DEFAULT_PROFILES_FILE=/home/roboticistprogrammer/sky_warrior_ws/src/skyw_swarm/config/fastdds.xml

echo "PX4 directory: ${PX4_DIR}"
echo "Session name: ${session}"
echo "Fast DDS profile: ${FASTDDS_DEFAULT_PROFILES_FILE}"

# Compose Gazebo resources while preserving any custom paths from the current shell.
GZ_RESOURCE_PATHS=""
append_resource_path() {
	local candidate="$1"
	if [ -n "${candidate}" ] && [ -d "${candidate}" ]; then
		if [ -z "${GZ_RESOURCE_PATHS}" ]; then
			GZ_RESOURCE_PATHS="${candidate}"
		else
			GZ_RESOURCE_PATHS="${GZ_RESOURCE_PATHS}:${candidate}"
		fi
	fi
}

if [ -n "${GZ_SIM_RESOURCE_PATH}" ]; then
	IFS=':' read -r -a EXISTING_GZ_PATHS <<<"${GZ_SIM_RESOURCE_PATH}"
	for p in "${EXISTING_GZ_PATHS[@]}"; do
		append_resource_path "${p}"
	done
fi
append_resource_path "${PX4_GZ_MODELS}"
append_resource_path "${PX4_GZ_WORLDS}"
append_resource_path "${CUSTOM_WORLD_DIR_DEFAULT}"
export GZ_SIM_RESOURCE_PATH="${GZ_RESOURCE_PATHS}"

if [ -n "${CUSTOM_WORLD_FILE}" ]; then
	if [ ! -f "${CUSTOM_WORLD_FILE}" ]; then
		echo "[ERROR] Provided world file does not exist: ${CUSTOM_WORLD_FILE}" >&2
		exit 1
	fi
	CUSTOM_WORLD_DIR="$(dirname "${CUSTOM_WORLD_FILE}")"
	CUSTOM_WORLD_NAME="$(basename "${CUSTOM_WORLD_FILE}")"
	CUSTOM_WORLD_NAME="${CUSTOM_WORLD_NAME%.sdf}"
	# Put custom world directory first to avoid filename collisions
	# with PX4 built-in worlds (e.g. another world.sdf).
	GZ_RESOURCE_PATHS=""
	append_resource_path "${CUSTOM_WORLD_DIR}"
	append_resource_path "${PX4_GZ_MODELS}"
	append_resource_path "${PX4_GZ_WORLDS}"
	append_resource_path "${CUSTOM_WORLD_DIR_DEFAULT}"
	export GZ_SIM_RESOURCE_PATH="${GZ_RESOURCE_PATHS}"
	export PX4_GZ_WORLD="${CUSTOM_WORLD_NAME}"
	echo "Using custom world: ${CUSTOM_WORLD_FILE} (PX4_GZ_WORLD=${PX4_GZ_WORLD})"
fi

# Ensure Gazebo can load PX4 simulation systems/services used for model spawning.
if [ -d "${PX4_GZ_PLUGINS}" ]; then
	if [ -n "${GZ_SIM_SYSTEM_PLUGIN_PATH}" ]; then
		export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:${PX4_GZ_PLUGINS}"
	else
		export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_GZ_PLUGINS}"
	fi
fi

if [ -f "${PX4_GZ_SERVER_CONFIG}" ]; then
	export GZ_SIM_SERVER_CONFIG_PATH="${PX4_GZ_SERVER_CONFIG}"
fi

if [ ! -x "${PX4_BIN}" ]; then
	echo "[ERROR] PX4 binary not found or not executable: ${PX4_BIN}" >&2
	exit 1
fi

if [ ! -f "${FASTDDS_DEFAULT_PROFILES_FILE}" ]; then
	echo "[ERROR] Fast DDS profile not found: ${FASTDDS_DEFAULT_PROFILES_FILE}" >&2
	exit 1
fi

if tmux has-session -t "${session}" 2>/dev/null; then
	echo "tmux session '${session}' already exists. Attaching..."
	exec tmux attach -t "${session}"
fi

echo "Starting new tmux session '${session}'..."

# Window 1: PX4 SITL instance 1 (x500_mono_cam)
tmux new-session -d -s "${session}" -n "PX4-1" \
	"cd \"${PX4_DIR}\" && PX4_SYS_AUTOSTART=4010 PX4_SIM_MODEL=gz_x500_mono_cam \"${PX4_BIN}\" -i 1"

# Window 2: PX4 SITL instance 2 (x500)
tmux new-window -t "${session}:" -n "PX4-2" \
	"cd \"${PX4_DIR}\" && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"0,1\" PX4_SIM_MODEL=gz_x500 \"${PX4_BIN}\" -i 2"

# Window 3: PX4 SITL instance 3 (x500)
tmux new-window -t "${session}:" -n "PX4-3" \
	"cd \"${PX4_DIR}\" && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"0,2\" PX4_SIM_MODEL=gz_x500 \"${PX4_BIN}\" -i 3"

# Window 4: Micro XRCE Agent
tmux new-window -t "${session}:" -n "MicroXRCE" \
	"export RMW_FASTRTPS_USE_QOS_FROM_XML=1 FASTDDS_DEFAULT_PROFILES_FILE=\"${FASTDDS_DEFAULT_PROFILES_FILE}\" && MicroXRCEAgent udp4 -p 8888"

echo "tmux session '${session}' is ready."
exec tmux attach -t "${session}"
