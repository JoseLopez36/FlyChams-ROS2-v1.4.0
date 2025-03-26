#!/bin/bash
# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Source the environment file relative to the script location
source "$SCRIPT_DIR/config.env"

# Launch PX4 SITL
$PX4_AUTOPILOT_PATH/Tools/docker_run.sh 'export PX4_SIM_HOSTNAME=172.17.0.1 && make px4_sitl_default none_iris'