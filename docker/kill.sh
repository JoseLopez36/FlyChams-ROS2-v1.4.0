#!/bin/bash
# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Source the environment file relative to the script location
source "$SCRIPT_DIR/config.env"

# Get the running container
RUNNING_CONTAINER=$(docker ps -q -f "name=$CONTAINER_NAME")

# Check if the container is running and stop it
if [ -n "$RUNNING_CONTAINER" ]; then
    echo "Stopping container: $CONTAINER_NAME"
    docker stop $RUNNING_CONTAINER
    echo "Removing container: $CONTAINER_NAME"
    docker rm $RUNNING_CONTAINER
else
    echo "No containers running with the name: $CONTAINER_NAME"
fi
