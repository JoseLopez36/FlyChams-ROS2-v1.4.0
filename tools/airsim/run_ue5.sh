#!/bin/bash
# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Source the environment file relative to the script location
source "$SCRIPT_DIR/../../docker/config.env"

# Launch UE5
$FLYCHAMS_SIM_UE5_PATH/FlyChamsSim.sh -settings="$FLYCHAMS_ROS2_PATH/config/settings.json"