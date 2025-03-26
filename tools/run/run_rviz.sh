#!/bin/bash

RVIZ_CONFIG=${1:-"default.rviz"}

# Get ROS2 workspace directory
ROS2_WS="$FLYCHAMS_ROS2_PATH/ros2_ws"

# Source ROS2 workspace
if [ -f "$ROS2_WS/install/setup.bash" ]; then
  source "$ROS2_WS/install/setup.bash"
else
  echo "Error: ROS workspace not found at $ROS2_WS" >&2
  exit 1
fi

# Launch RViz
ros2 launch flychams_bringup rviz.launch.py config:="$RVIZ_CONFIG"