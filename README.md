# Flying Chameleons: Multi-UAV System for Autonomous Target Tracking

A ROS2-based system for coordinated multi-UAV target tracking using various advanced simulation frameworks (e.g. AirSim, Unreal Engine 5, etc.).

## Overview

The Flying Chameleons (FlyChams) project implements a complete system for controlling and coordinating multiple UAVs equipped with modifiable tracking systems. The primary goal is to optimize target tracking through collaborative agent positioning and camera control.

The project leverages:
- **Unreal Engine 5** for photorealistic simulation
- **AirSim** for high-fidelity physics simulation
- **PX4** for commercial flight control
- **ROS2** for the distributed robotics framework

## Key Features

- Multi-agent coordination for optimal target coverage
- Independent control of multiple cameras per agent
- Clustering algorithms for grouping and tracking targets
- Real-time visualization and monitoring tools
- Configurable missions via Excel configuration files
- Realistic simulation in photorealistic Unreal Engine environments

## System Architecture

| Package                 | Description                                     |
| ----------------------- | ----------------------------------------------- |
| `flychams_core`         | Core domain models, utilities, and interfaces   |
| `flychams_bringup`      | Launch files and configuration for the system   |
| `flychams_control`      | Control algorithms for aerial agents            |
| `flychams_perception`   | Perception algorithms for clustering targets    |
| `flychams_coordination` | Coordination algorithms for multi-agent systems |
| `flychams_targets`      | Target management and trajectory control        |
| `flychams_dashboard`    | Visualization and monitoring tools              |
| `flychams_interfaces`   | Custom message and service for FlyChams         |
| `airsim_wrapper`        | ROS2 interface to the AirSim simulator          |
| `airsim_interfaces`     | Custom message and service for AirSim           |

## Prerequisites

### Software Requirements

- **Ubuntu 20.04, 22.04, or 24.04** (or compatible Linux distribution)
- **Docker** (for running the system in a container)
- **Unreal Engine 5.2.1** (optional, for developing new environments)

### Hardware Requirements

- Medium to high-end CPU (e.g. Intel i7-12700K or AMD Ryzen 7 5800X)
- Medium to high-end GPU with latest drivers (e.g. NVIDIA RTX 3070 or AMD RX 6800 XT)
- 16 GB of RAM
- 32 GB of RAM

### Test setup

- **Ubuntu 24.04**
- **AMD RX 7800 XT**
- **Intel i7-13700KF**
- **32 GB of RAM**

## Installation

### 1. Clone the FlyChams environment repositories

```bash
git clone https://github.com/JoseLopez36/FlyChams-ROS2.git
git clone https://github.com/JoseLopez36/FlyChams-Cosys-AirSim.git
```

### 2. Setup the PX4-Autopilot repository

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot/
# We recommend using the 1.12.0 stable release
git checkout v1.12.0
# Build and run SITL in docker container
Tools/docker_run.sh 'export PX4_SIM_HOSTNAME=172.17.0.1 && make px4_sitl_default none_iris'
```
*Note: 172.17.0.1 is the IP address of the host machine from the docker container. Check this corresponds to the IP address of the host machine. If you are running PX4 directly on the host machine, you can use 127.0.0.1.*

### 3. Setup environment variables

Add the following to your `/etc/bash.bashrc` file to setup permanent environment variables:

```bash
sudo nano /etc/bash.bashrc
# Add the following to your /etc/bash.bashrc file
export FLYCHAMS_ROS2_PATH="/path/to/FlyChams-ROS2"
export FLYCHAMS_COSYS_AIRSIM_PATH="/path/to/FlyChams-Cosys-AirSim"
export PX4_AUTOPILOT_PATH="/path/to/PX4-Autopilot"
```

### 4. Build the docker image

```bash
cd $FLYCHAMS_ROS2_PATH/docker
./build_image.sh
```
*Note: This will build the docker image and tag it as flychams-docker. It may take a while to build the image.*

### 5. Run the docker container

```bash
./start_container.sh
```
*Note: This will run the docker container. It will mount the current directory to the /home/flychams/FlyChams-ROS2 and FlyChams-Cosys-AirSim directories in the container. The current directory should contain the FlyChams-ROS2 and FlyChams-Cosys-AirSim repositories.*

### 6. Setup and build FlyChams-Cosys-AirSim

```bash
$FLYCHAMS_ROS2_PATH/tools/build_airsim_dep.sh
```
*Note: This will clean, setup and build the dependencies for FlyChams-Cosys-AirSim.*

### 7. Build the ROS2 workspace

```bash
PARALLEL=3 $FLYCHAMS_ROS2_PATH/tools/build_ros2_ws.sh
```
*Note: This will build the ROS2 workspace in the ros2_ws directory. It will use 3 threads for parallel building. We don´t recommend using many more threads as the build may fail.*

## Usage

### 1. Generate AirSim Settings

You only need to generate the AirSim settings once. After that, you can skip this step unless you make changes to the spreadsheet configuration file.

For generating the AirSim settings, run the following command inside the docker container:
```bash
$FLYCHAMS_ROS2_PATH/tools/create_airsim_settings.sh
```

### 2. (Optional) Run PX4 SITL

If you want to run the PX4 SITL (must be configured in the `Configuration.xlsx` file), you can use the following command:

```bash
$PX4_AUTOPILOT_PATH/Tools/docker_run.sh 'export PX4_SIM_HOSTNAME=172.17.0.1 && make px4_sitl_default none_iris'
```

### 3. Launch the Unreal Engine Simulation

For this step, you need to have an UE project with the FlyChams-Cosys-AirSim plugin installed. You can find exported projects in the `FlyChams-Sim-UE5` repository releases.

```bash
/path/to/FlyChamsSim.sh -settings="$FLYCHAMS_ROS2_PATH/config/settings.json"
```

### 4. Launch ROS2 System

The ROS2 system is launched in two phases:

#### Setup Phase

```bash
ros2 launch flychams_bringup setup.launch.py
```

This will:
1. Start the AirSim interface
2. Register all elements (agents, targets, clusters)
3. Initialize the system configurations

#### Runtime Phase

```bash
ros2 launch flychams_bringup run.launch.py
```

This will start all the control, perception, coordination, target, and dashboard nodes. You can customize which nodes are launched by using the command as follows:

```bash
# Launch only tracking node with warning level logging
ros2 launch flychams_bringup run.launch.py track:=True log_track:=warn
```

### 5. (Optional) Visualization

To view the system in RViz:

```bash
$FLYCHAMS_ROS2_PATH/tools/run_rviz.sh
```

To plot simulation data on runtime, we recommend using `PlotJuggler` (already installed in the docker container). To run it, use the following command:

```bash
$FLYCHAMS_ROS2_PATH/tools/run_plotjuggler.sh
```

You can also plot previous rosbag data by importing them into the PlotJuggler window. More info [here](https://plotjuggler.io/). To record rosbags you must configure it in the `Configuration.xlsx` file and use the following command:

```bash
ros2 launch flychams_bringup rosbag.launch.py
```

## Configuration

The system is mainly configured using an Excel spreadsheet (`Configuration.xlsx`). This file includes:

1. **Missions** - General mission characteristics
2. **Simulations** - Simulation environment details
3. **Maps** - Map definitions
4. **Targets** - Target definitions and trajectories
5. **Agents** - Agent configurations
6. **AgentHeads** - Head (Gimbal/Camera) specifications
7. **DroneModels** - UAV model specifications
8. **GimbalModels** - Gimbal model specifications
9. **GimbalLinks** - Gimbal link specifications
10. **CameraModels** - Camera model specifications

## Directory Structure

```
FlyChams-ROS2/
├── config/                         # Configuration files
│   └── Configuration.xlsx          # Main configuration spreadsheet
│   └── Trajectories/               # Trajectory files for targets
├── ros2_ws/                        # ROS2 workspace
│   └── src/                        # Source packages
│       ├── flychams_core/          # Core domain models, utilities and interfaces
│       ├── flychams_bringup/       # Launch and configuration
│       ├── flychams_control/       # Agent control
│       ├── flychams_perception/    # Target perception
│       ├── flychams_coordination/  # Multi-agent coordination
│       ├── flychams_targets/       # Target management
│       ├── flychams_dashboard/     # Visualization and monitoring
│       ├── flychams_interfaces/    # Custom message and service for FlyChams
│       ├── airsim_interfaces/      # Custom message and service for AirSim wrapper
│       └── airsim_wrapper/         # ROS2 AirSim wrapper
├── experiments/                    # Experiment data and settings
├── docker/                         # Docker files and scripts
└── tools/                          # Utility scripts and tools
```

## Known Limitations

1. **Performance Limitations**
   - The system has been tested with a single UAV agent, with up to 4 cameras.
   - Performance may degrade significantly with more elements.

2. **AirSim Integration**
   - Currently requires a specific fork of AirSim with custom modifications.
   - Limited support for AirSim's newer features.

3. **Real Hardware Integration**
   - Current implementation focuses on simulation; real hardware integration will be available in future releases.

## Contact

For more information, please contact [josloprui6@alum.us.es](mailto:josloprui6@alum.us.es).
