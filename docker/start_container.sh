#!/bin/bash
CONTAINER_NAME=flychams-ros2-container

# Check if container is already running
if sudo docker ps --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container is already running, opening a new terminal"
    sudo docker exec -it ${CONTAINER_NAME} bash -c "source /opt/ros/iron/setup.bash; exec bash"
else
    echo "Container is not running, starting it"
    sudo xhost +local:docker
    sudo docker run --rm -it \
    --net=host \
    --env DISPLAY=$DISPLAY \
    -v ${FLYCHAMS_ROS2_PATH}:/home/testuser/FlyChams-ROS2 \
    --name ${CONTAINER_NAME} \
    flychams-ros2:latest \
    bash
fi