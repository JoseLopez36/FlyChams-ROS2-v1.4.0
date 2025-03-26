#!/bin/bash
# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Source the environment file relative to the script location
source "$SCRIPT_DIR/config.env"
# Allow X11 forwarding
xhost +local:docker

# Check if container is already running
if docker ps --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container is already running, opening a new terminal"
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/iron/setup.bash; exec bash"
else
    echo "Container is not running, starting it"
	docker run --rm -it \
		--name $CONTAINER_NAME \
		--privileged \
	    -v ${FLYCHAMS_ROS2_PATH}:/home/testuser/FlyChams-ROS2 \
        -v ${FLYCHAMS_AIRSIM_PATH}:/home/testuser/FlyChams-Cosys-AirSim \
        -v ${FLYCHAMS_PX4_PATH}:/home/testuser/PX4-Autopilot \
		--env DISPLAY=$DISPLAY \
		--volume $X11_SOCKET:$X11_SOCKET \
		--network $NETWORK \
		$(for port in $PORTS; do echo -p $port; done) \
		$IMAGE_NAME \
        bash
fi