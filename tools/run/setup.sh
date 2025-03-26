#!/bin/bash
set -e  # Exit on error

# Get ROS2 workspace directory
ROS2_WS="$FLYCHAMS_ROS2_PATH/ros2_ws"

# Source ROS2 workspace
if [ -f "$ROS2_WS/install/setup.bash" ]; then
  source "$ROS2_WS/install/setup.bash"
else
  echo "Error: ROS workspace not found at $ROS2_WS" >&2
  exit 1
fi

# Launch FlyChams with AirSim
echo "Setting up FlyingChameleons simulation..."
ros2 launch flychams_bringup setup.launch.py &
FLYCHAMS_PID=$!

# Print helpful information
echo ""
echo "FlyingChameleons setup is running in the background (PID: $FLYCHAMS_PID)"
echo "You can now enter ROS2 commands in this terminal"
echo "Press Ctrl+C to exit and terminate the setup process"
echo ""

# Keep the script running but allow user input
while true; do
  # Show a prompt
  echo -n "flychams_setup$ "
  read -r cmd
  
  # Execute the command
  eval "$cmd"
done