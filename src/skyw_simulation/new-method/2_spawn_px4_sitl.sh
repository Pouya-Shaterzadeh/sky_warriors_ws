#!/bin/bash
# Spawn PX4 SITL instances that connect to running Gazebo
# Run this AFTER Gazebo is already running (1_start_gazebo.sh)

set -e

NUM_VEHICLES=${1:-1}
VEHICLE_MODEL=${2:-x500}
SPEED_FACTOR=${3:-1}  # Default 1, use 0.5 for low RAM systems
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PX4_DIR="${HOME}/PX4-Autopilot"

echo "=========================================="
echo "Spawning PX4 SITL Instances"
echo "=========================================="
echo "Vehicles: $NUM_VEHICLES | Model: $VEHICLE_MODEL"
echo "World: warehouse1"
echo "Speed Factor: $SPEED_FACTOR (use 0.5 for 8GB RAM systems)"
echo ""

# Source environment variables (critical for PX4 to find Gazebo)
source "${WORKSPACE_DIR}/gz_env.sh"

# Cleanup function
cleanup() {
    echo ""
    echo "Cleaning up PX4 instances..."
    pkill -9 -f "px4.*-i" || true
    sleep 1
}

trap cleanup EXIT SIGINT SIGTERM

# Clean any existing PX4 instances
cleanup

# Check if Gazebo is running
if ! pgrep -f "gz sim" > /dev/null; then
    echo "ERROR: Gazebo is not running!"
    echo "Please run 1_start_gazebo.sh first in another terminal"
    exit 1
fi

echo "✓ Gazebo is running"
echo ""

# Check RAM and recommend longer waits for low memory systems
TOTAL_RAM=$(free -g | awk '/^Mem:/{print $2}')
if [ "$TOTAL_RAM" -le 8 ]; then
    echo "⚠ Low RAM detected (${TOTAL_RAM}GB) - Using extended wait times"
    WAIT_TIME=10
else
    WAIT_TIME=5
fi
echo ""

cd "$PX4_DIR"

echo "Spawning vehicles..."
echo ""

# Function to check if model is spawned in Gazebo
check_model_spawned() {
    local instance=$1
    local model_name="${VEHICLE_MODEL}_${instance}"
    timeout 2 gz model --list 2>/dev/null | grep -q "$model_name" && return 0 || return 1
}

for i in $(seq 0 $((NUM_VEHICLES - 1))); do
    x_pos=$((i * 4))
    y_pos=0
    
    echo "[$i] Spawning vehicle at ($x_pos, $y_pos)..."
    
    PX4_GZ_STANDALONE=1 \
    PX4_SIM_SPEED_FACTOR=$SPEED_FACTOR \
    PX4_GZ_WORLD=warehouse1 \
    PX4_SYS_AUTOSTART=4010 \
    PX4_SIM_MODEL="gz_${VEHICLE_MODEL}" \
    PX4_GZ_MODEL_POSE="$x_pos,$y_pos" \
    PX4_GZ_MODEL="${VEHICLE_MODEL}" \
    GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH" \
    GZ_SIM_SYSTEM_PLUGIN_PATH="$GZ_SIM_SYSTEM_PLUGIN_PATH" \
    ./build/px4_sitl_default/bin/px4 -i $i &
    
    echo "  Waiting ${WAIT_TIME}s for model to spawn..."
    sleep $WAIT_TIME
    
    # Verify model is in Gazebo (retry up to 3 times)
    for retry in {1..3}; do
        if check_model_spawned $i; then
            echo "  ✓ Model ${VEHICLE_MODEL}_${i} confirmed in Gazebo"
            break
        else
            if [ $retry -lt 3 ]; then
                echo "  ⚠ Model not detected yet, waiting 3s more..."
                sleep 3
            else
                echo "  ⚠ Warning: Could not confirm model in Gazebo (may still be loading)"
            fi
        fi
    done
done

echo ""
echo "=========================================="
echo "All vehicles spawned!"
echo "=========================================="
echo ""

# Status check
echo "Status Check:"
echo "-------------"
echo ""

echo "PX4 Instances:"
pgrep -a -f "px4.*-i" | nl

echo ""
echo "MAVLink Ports:"
for i in $(seq 0 $((NUM_VEHICLES - 1))); do
    port=$((14540 + i))
    if ss -tuln 2>/dev/null | grep -q ":$port " || netstat -tuln 2>/dev/null | grep -q ":$port "; then
        echo "  ✓ Vehicle $i: UDP $port"
    else
        echo "  ✗ Vehicle $i: UDP $port not active"
    fi
done

echo ""
echo "=========================================="
echo "Connect QGC to ports: 14540-$((14540+NUM_VEHICLES-1))"
echo "Press Ctrl+C to stop all PX4 instances"
echo "=========================================="

# Keep running
wait
