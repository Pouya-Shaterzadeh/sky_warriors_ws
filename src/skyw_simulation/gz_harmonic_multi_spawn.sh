#!/bin/bash
# Multi-vehicle spawn script for Gazebo Harmonic + PX4
# Based on: https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation

set -e

# Configuration
NUM_VEHICLES=${1:-3}
VEHICLE_MODEL=${2:-x500}
WORLD_FILE="$(dirname "$0")/world/warehouse1.sdf"
PX4_DIR="${HOME}/PX4-Autopilot"

echo "=========================================="
echo "Gazebo Harmonic Multi-Vehicle Diagnostic"
echo "=========================================="
echo "Number of vehicles: $NUM_VEHICLES"
echo "Vehicle model: $VEHICLE_MODEL"
echo "World file: $WORLD_FILE"
echo "PX4 Directory: $PX4_DIR"
echo ""

# Source gz environment
if [ -f "$(dirname "$0")/gz_env.sh" ]; then
    echo "[INFO] Sourcing gz_env.sh..."
    source "$(dirname "$0")/gz_env.sh"
else
    echo "[ERROR] gz_env.sh not found!"
    exit 1
fi

# Check if PX4 directory exists
if [ ! -d "$PX4_DIR" ]; then
    echo "[ERROR] PX4 directory not found: $PX4_DIR"
    exit 1
fi

# Check if PX4 is built
if [ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    echo "[ERROR] PX4 not built. Please run: cd $PX4_DIR && make px4_sitl gz_${VEHICLE_MODEL}"
    exit 1
fi

# Check if world file exists
if [ ! -f "$WORLD_FILE" ]; then
    echo "[ERROR] World file not found: $WORLD_FILE"
    exit 1
fi

echo "[INFO] Environment check passed!"
echo ""

# Display environment variables
echo "[DEBUG] Environment variables:"
echo "  GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "  GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "  GZ_SIM_SERVER_CONFIG_PATH=$GZ_SIM_SERVER_CONFIG_PATH"
echo "  PX4_GZ_MODELS=$PX4_GZ_MODELS"
echo ""

# Check if gz command is available
if ! command -v gz &> /dev/null; then
    echo "[ERROR] 'gz' command not found. Is Gazebo Harmonic installed?"
    exit 1
fi

echo "[INFO] Gazebo version:"
gz sim --version
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "[INFO] Cleaning up..."
    pkill -9 -f "px4.*-i" || true
    pkill -9 -f "gz sim" || true
    sleep 2
    echo "[INFO] Cleanup done"
}

trap cleanup EXIT SIGINT SIGTERM

# Kill any existing instances
echo "[INFO] Killing any existing PX4 and Gazebo instances..."
cleanup
sleep 2

cd "$PX4_DIR"

echo ""
echo "=========================================="
echo "Starting Multi-Vehicle Simulation"
echo "=========================================="
echo ""

# Method 1: Try using PX4_GZ_MODEL_POSE for positioning
echo "[INFO] Starting vehicles using PX4_GZ_MODEL_POSE method..."
echo ""

for i in $(seq 0 $((NUM_VEHICLES - 1))); do
    instance=$i
    x_pos=$((i * 3))
    y_pos=0
    
    echo "[INFO] Starting vehicle $instance at position ($x_pos, $y_pos)..."
    
    if [ $instance -eq 0 ]; then
        # First instance: Start gz sim with world (NO standalone mode)
        echo "  Command: PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"$x_pos,$y_pos\" PX4_GZ_MODEL=${VEHICLE_MODEL} ./build/px4_sitl_default/bin/px4 -i $instance"
        
        PX4_SYS_AUTOSTART=4010 \
        PX4_GZ_MODEL_POSE="$x_pos,$y_pos" \
        PX4_GZ_MODEL=${VEHICLE_MODEL} \
        PX4_SIM_MODEL="gz_${VEHICLE_MODEL}" \
        PX4_GZ_WORLD=$(basename ${WORLD_FILE} .sdf) \
        ./build/px4_sitl_default/bin/px4 -i $instance &
        
        PX4_PID_0=$!
        echo "  PID: $PX4_PID_0"
        echo "  Waiting 10 seconds for gz server to start..."
        sleep 10
        
        # Check if gz sim is running
        if pgrep -f "gz sim" > /dev/null; then
            echo "  [SUCCESS] gz sim server is running"
        else
            echo "  [ERROR] gz sim server not running!"
            echo "  Check if world file is correct and model exists"
            exit 1
        fi
        
    else
        # Subsequent instances: Connect to existing gz server
        echo "  Command: PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"$x_pos,$y_pos\" PX4_GZ_MODEL=${VEHICLE_MODEL} ./build/px4_sitl_default/bin/px4 -i $instance"
        
        PX4_SYS_AUTOSTART=4010 \
        PX4_GZ_MODEL_POSE="$x_pos,$y_pos" \
        PX4_GZ_MODEL=${VEHICLE_MODEL} \
        PX4_SIM_MODEL="gz_${VEHICLE_MODEL}" \
        ./build/px4_sitl_default/bin/px4 -i $instance &
        
        echo "  PID: $!"
        sleep 3
    fi
    
    echo ""
done

echo "=========================================="
echo "All vehicles started!"
echo "=========================================="
echo ""
echo "Debugging Information:"
echo "----------------------"

# Check running processes
echo "[DEBUG] Running PX4 instances:"
pgrep -a -f "px4.*-i" || echo "  No PX4 instances found!"
echo ""

echo "[DEBUG] Running Gazebo instances:"
pgrep -a -f "gz sim" || echo "  No Gazebo instances found!"
echo ""

# Try to list models in simulation
echo "[DEBUG] Models in simulation (attempting to list):"
sleep 2
gz model --list 2>&1 || echo "  Could not list models"
echo ""

# Check gz topics
echo "[DEBUG] Active Gazebo topics:"
gz topic --list 2>&1 | head -20 || echo "  Could not list topics"
echo ""

# Check MAVLink ports
echo "[DEBUG] Checking MAVLink ports:"
for i in $(seq 0 $((NUM_VEHICLES - 1))); do
    udp_port=$((14540 + i))
    tcp_port=$((4560 + i))
    echo "  Vehicle $i: UDP=$udp_port, TCP=$tcp_port"
    netstat -tuln 2>/dev/null | grep -E "($udp_port|$tcp_port)" || echo "    Ports not active yet"
done
echo ""

echo "=========================================="
echo "To monitor:"
echo "  - gz sim GUI: Should show warehouse1 world with vehicles"
echo "  - QGC: Should connect to vehicles on UDP ports 14540, 14541, 14542..."
echo "  - Logs: Check $PX4_DIR/build/px4_sitl_default/rootfs/*/out.log"
echo "=========================================="
echo ""
echo "Press Ctrl+C to stop all instances"

# Keep script running
wait
