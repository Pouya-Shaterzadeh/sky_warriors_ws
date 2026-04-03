#!/bin/bash
# File: launch_classic_3_iris_lz.sh
# Launches Gazebo Classic with 3 iris drones and uploads their landing zone missions

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Change to the PX4-Autopilot directory (required for the scripts below)
cd ~/PX4-Autopilot || exit

# Start Micro-XRCE-DDS Agent in the background
echo "[1/3] Starting Micro-XRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 >/dev/null 2>&1 &
MICRO_AGENT_PID=$!

echo "[2/3] Starting Gazebo 11 Classic with 3 iris drones at landing zones..."
echo "Landing Zone mapping:"
echo "  - Drone 0 (port 14541) at LZ (0, 0)"
echo "  - Drone 1 (port 14542) at LZ (4, 3)"
echo "  - Drone 2 (port 14543) at LZ (8, 0)"
echo ""

# Prevent Gazebo Harmonic's `gz` tool from intercepting the Gazebo 11 Classic command
function gz() {
    /usr/bin/gz11 "$@"
}
export -f gz

# Unset ROS version to prevent Gazebo 11 from trying to load missing/unnecessary ROS 2 plugins
export ROS_VERSION=""

# Copy custom world file to PX4 worlds directory
CUSTOM_WORLD="${SCRIPT_DIR}/skyw_simulation/world/multi_lz_classic.world"
PX4_WORLD_DIR="${HOME}/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"

if [ -f "${CUSTOM_WORLD}" ]; then
    cp "${CUSTOM_WORLD}" "${PX4_WORLD_DIR}/"
    echo "Custom world with landing zones copied to PX4."
fi

# Launch Gazebo Classic with 3 iris drones at their respective landing zone positions
./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -w multi_lz_classic -s "iris:1:0:0, iris:1:4:3, iris:1:8:0" &
GAZEBO_PID=$!

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo "Gazebo Classic is running with 3 iris drones"
echo "QGroundControl ports: 14541, 14542, 14543"
echo ""
echo "Manual PX4 failsafe commands:"
echo "  cd ${SCRIPT_DIR}/failsafes/scripts"
echo "  python3 mavsdkrunner.py"
echo ""
echo "Automated MAVSDK failsafe tests:"
echo "  cd ${SCRIPT_DIR}/failsafes/scripts"
echo "  python3 mavsdkrunner_automated.py"
echo ""
echo "Press Ctrl+C to stop all"
echo "=========================================="
echo ""

# Wait for Gazebo process
wait $GAZEBO_PID

# Cleanup Micro-XRCE-DDS agent when Gazebo exits
kill $MICRO_AGENT_PID 2>/dev/null || true
