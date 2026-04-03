#!/bin/bash
# File: launch_3_x500_walls.sh

# Change to the PX4-Autopilot directory (required for the scripts below)
cd ~/PX4-Autopilot || exit

# Start Micro-XRCE-DDS Agent in the background
MicroXRCEAgent udp4 -p 8888 &
MICRO_AGENT_PID=$!

echo "Starting Gazebo 11 Classic with 3 px4-SITL standard models..."
echo "Note: Using 'iris' which is the Gazebo Classic equivalent for 'x500'."

# Prevent Gazebo Harmonic's `gz` tool from intercepting the Gazebo 11 Classic command
function gz() {
    /usr/bin/gz11 "$@"
}
export -f gz

# Unset ROS version to prevent Gazebo 11 from trying to load missing/unnecessary ROS 2 plugins
export ROS_VERSION=""

# In Gazebo 11 Classic, we use sitl_multiple_run.sh to orchestrate gzserver, model injection, and px4 instances.
# Since a native 'walls' world might not be available for Gazebo Classic, we fallback to 'empty' 
# (You can change "empty" back to "walls" if you have a custom walls.world file).
./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -w empty -s "iris:1:0:0, iris:1:3:0, iris:1:6:0"

# Cleanup Micro-XRCE-DDS agent when script terminates (e.g., when gzclient is closed)
kill $MICRO_AGENT_PID
