#!/bin/bash
set -e  # Exit on error

# Arguments   
RECORD=${1:-"false"}

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
echo "Launching FlyingChameleons with AirSim"
ros2 launch flychams_bringup flychams_airsim.launch.py &
FLYCHAMS_PID=$!

# Launch bag record conditionally
if [ "$RECORD" = "true" ]; then
  ros2 launch flychams_bringup bag_record.launch.py &
  BAG_RECORD_PID=$!
else
  BAG_RECORD_PID=0
fi

# Cleanup on exit
trap "echo 'Shutting down processes...'; kill $FLYCHAMS_PID $BAG_RECORD_PID 2>/dev/null" EXIT

# Print helpful information
echo ""
echo "FlyChams with AirSim is running in the background (PID: $FLYCHAMS_PID)"
if [ "$RECORD" = "true" ]; then
  echo "Bag recording is running in the background (PID: $BAG_RECORD_PID)"
fi
echo "You can now enter ROS2 commands in this terminal."
echo "Press Ctrl+C to exit and terminate all processes."
echo ""

# Keep the script running but allow user input
while true; do
  # Show a prompt
  echo -n "flychams_cmd$ "
  read -r cmd
  
  # Execute the command
  eval "$cmd"
done