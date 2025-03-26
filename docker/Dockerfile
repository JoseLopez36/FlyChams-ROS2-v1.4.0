FROM ros:iron-ros-base

# Install Cosys-AirSim ROS2 dependencies 
RUN     apt-get update && \
        apt-get install -y \
        apt-utils \
        ros-$ROS_DISTRO-tf2-sensor-msgs ros-$ROS_DISTRO-tf2-geometry-msgs ros-$ROS_DISTRO-mavros* \
        ros-$ROS_DISTRO-vision-opencv ros-$ROS_DISTRO-image-transport \
        libyaml-cpp-dev \
        ros-iron-pcl-ros \
        ros-iron-pcl-conversions \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        && echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc && \
        rm -rf /var/lib/apt/lists/* && \
        apt-get clean

# Install GeographicLib datasets for mavros
RUN     wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh && \
        chmod +x install_geographiclib_datasets.sh && \
        ./install_geographiclib_datasets.sh && \
        rm install_geographiclib_datasets.sh

# Additional installation
RUN     apt-get update && \
        apt-get install -y \
        ros-iron-rviz2 \
        ros-iron-plotjuggler-ros \
        && rm -rf /var/lib/apt/lists/* && \
        apt-get clean

# Create a non-root user with sudo access
ARG USER_NAME=testuser
ENV USER_NAME=${USER_NAME}
RUN     adduser --disabled-password --gecos '' $USER_NAME \
        && adduser $USER_NAME sudo \
        && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Ensure the user owns their home directory
RUN     chown -R $USER_NAME:$USER_NAME /home/$USER_NAME

# Switch to the non-root user
USER    $USER_NAME
WORKDIR /home/${USER_NAME}       
RUN     echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN     echo "export FLYCHAMS_ROS2_PATH=/home/${USER_NAME}/FlyChams-ROS2" >> ~/.bashrc
RUN     echo "export FLYCHAMS_COSYS_AIRSIM_PATH=/home/${USER_NAME}/FlyChams-Cosys-AirSim" >> ~/.bashrc
RUN     echo "export FLYCHAMS_PX4_PATH=/home/${USER_NAME}/PX4-Autopilot" >> ~/.bashrc