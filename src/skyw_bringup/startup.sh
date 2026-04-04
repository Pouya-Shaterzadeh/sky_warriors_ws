#!/bin/bash

set -e

session="px4-sitl"
PX4_DIR="${HOME}/PX4-Autopilot"
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
PX4_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/gz_env.sh"
CUSTOM_WORLD="${1:-}"
# Fixed name under PX4 worlds/ (px4-rc.gzsim sources gz_env.sh and resets PX4_GZ_WORLDS to here).
SKYW_WORLD_NAME="skyw_hexagon"
RMW_FASTRTPS_USE_QOS_FROM_XML=1
# Dynamically determine workspace root (go up two levels from SCRIPT_DIR)
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
FASTDDS_DEFAULT_PROFILES_FILE="${WS_DIR}/src/skyw_swarm/config/fastdds.xml"

echo "PX4 directory: ${PX4_DIR}"
echo "Session name: ${session}"
echo "Fast DDS profile: ${FASTDDS_DEFAULT_PROFILES_FILE}"

PX4_ENV_PREFIX=""
if [ -n "${CUSTOM_WORLD}" ]; then
	if [ ! -f "${CUSTOM_WORLD}" ]; then
		echo "[ERROR] World file not found: ${CUSTOM_WORLD}" >&2
		exit 1
	fi
	REAL_WORLD="$(realpath "${CUSTOM_WORLD}")"
	BRINGUP_DIR="$(dirname "${REAL_WORLD}")"
	mkdir -p "${PX4_WORLDS_DIR}"
	ln -sf "${REAL_WORLD}" "${PX4_WORLDS_DIR}/${SKYW_WORLD_NAME}.sdf"
	# gz_env.sh appends to GZ_SIM_RESOURCE_PATH; prepend bringup so model://qr_wall resolves.
	PX4_ENV_PREFIX="export GZ_SIM_RESOURCE_PATH=\"${BRINGUP_DIR}:\$GZ_SIM_RESOURCE_PATH\"; export PX4_GZ_WORLD=${SKYW_WORLD_NAME}; "
	echo "Linked ${REAL_WORLD} -> ${PX4_WORLDS_DIR}/${SKYW_WORLD_NAME}.sdf (PX4_GZ_WORLD=${SKYW_WORLD_NAME})"
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

tmux new-session -d -s "${session}" -n "PX4-1" \
	"${PX4_ENV_PREFIX}cd \"${PX4_DIR}\" && unset PX4_GZ_STANDALONE; PX4_SYS_AUTOSTART=4010 PX4_GZ_MODEL_POSE=\"-7,5\" PX4_SIM_MODEL=gz_x500_mono_cam_down \"${PX4_BIN}\" -i 1"

sleep 12

tmux new-window -t "${session}:" -n "PX4-2" \
	"${PX4_ENV_PREFIX}cd \"${PX4_DIR}\" && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"-7,4\" PX4_SIM_MODEL=gz_x500 \"${PX4_BIN}\" -i 2"

tmux new-window -t "${session}:" -n "PX4-3" \
	"${PX4_ENV_PREFIX}cd \"${PX4_DIR}\" && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"-7,6\" PX4_SIM_MODEL=gz_x500 \"${PX4_BIN}\" -i 3"

tmux new-window -t "${session}:" -n "MicroXRCE" \
	"export RMW_FASTRTPS_USE_QOS_FROM_XML=1 FASTDDS_DEFAULT_PROFILES_FILE=\"${FASTDDS_DEFAULT_PROFILES_FILE}\" && MicroXRCEAgent udp4 -p 8888"

echo "tmux session '${session}' is ready."
exec tmux attach -t "${session}"
