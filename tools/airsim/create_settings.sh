#!/bin/bash

# Get ROS2 workspace directory
ROS_WS="$FLYCHAMS_ROS2_PATH/ros2_ws"

# Source ROS2 workspace
if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash"
else
  echo "Error: ROS workspace not found at $ROS_WS" >&2
  exit 1
fi

# Launch Airsim settings parser with parameters
ros2 launch flychams_bringup airsim_settings.launch.py