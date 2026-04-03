#!/bin/bash
#
# Demo script: Arms 3 drones, takes off to 5m, then performs V and Line formations.
#
# Prerequisites:
#   - PX4 SITL running (./startup.sh)
#   - Workspace sourced: source install/setup.bash
#
# This script starts the required bridges and formation server, then sends goals.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/../.."

# Source workspace if not already sourced
if ! ros2 pkg list 2>/dev/null | grep -q skyw_swarm; then
    echo "Sourcing workspace..."
    source "${WS_DIR}/install/setup.bash"
fi

cleanup() {
    echo ""
    echo "Shutting down..."
    # Kill background processes
    [[ -n "${POSE_BRIDGE_PID}" ]] && kill "${POSE_BRIDGE_PID}" 2>/dev/null || true
    [[ -n "${OFFBOARD_BRIDGE_PID}" ]] && kill "${OFFBOARD_BRIDGE_PID}" 2>/dev/null || true
    [[ -n "${FORMATION_SERVER_PID}" ]] && kill "${FORMATION_SERVER_PID}" 2>/dev/null || true
    wait 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT

echo "=========================================="
echo "  Sky Warriors Formation Demo"
echo "=========================================="
echo ""

# Start pose bridge (subscribes to PX4 local position, publishes /droneX/pose)
echo "[1/3] Starting PX4 Pose Bridge..."
ros2 run skyw_swarm px4_pose_bridge.py &
POSE_BRIDGE_PID=$!
sleep 2

# Start offboard bridge (subscribes to /droneX/setpoint_position, sends to PX4)
# This also handles auto-arm and auto-offboard when setpoints are received
echo "[2/3] Starting PX4 Offboard Bridge..."
ros2 run skyw_swarm px4_offboard_bridge.py &
OFFBOARD_BRIDGE_PID=$!
sleep 2

# Start formation server (action server for /set_formation)
echo "[3/3] Starting Formation Server..."
ros2 run skyw_swarm formation_server.py &
FORMATION_SERVER_PID=$!
sleep 3

echo ""
echo "All nodes started. Waiting for drones to initialize..."
sleep 5

# Check if formation action server is available
echo ""
echo "Checking action server..."
if ! ros2 action list | grep -q /set_formation; then
    echo "[ERROR] Formation action server not found!"
    exit 1
fi
echo "Action server ready: /set_formation"

echo ""
echo "=========================================="
echo "  Phase 1: Takeoff to 5m (V-formation)"
echo "=========================================="
echo ""
echo "Sending V-formation goal at 5m altitude..."
echo "This will arm the drones and take them to formation positions."
echo ""

ros2 action send_goal /set_formation skyw_swarm/action/SetFormation \
    "{formation_type: 'v', spacing: 2.0, altitude: -5.0, rotation: 0.0, drone_count: 3}" \
    --feedback

echo ""
echo "V-formation complete! Holding for 10 seconds..."
sleep 10

echo ""
echo "=========================================="
echo "  Phase 2: Transition to Line formation"
echo "=========================================="
echo ""
echo "Sending Line-formation goal..."
echo ""

ros2 action send_goal /set_formation skyw_swarm/action/SetFormation \
    "{formation_type: 'line', spacing: 2.0, altitude: -5.0, rotation: 0.0, drone_count: 3}" \
    --feedback

echo ""
echo "Line-formation complete! Holding for 10 seconds..."
sleep 10

echo ""
echo "=========================================="
echo "  Demo Complete!"
echo "=========================================="
echo ""
echo "Press Ctrl+C to land and exit, or wait 30 seconds..."
sleep 30

echo "Demo finished."
