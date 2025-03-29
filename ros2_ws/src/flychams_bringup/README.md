# Flychams Bringup: Entry Point for the Flying Chameleons Project

Launch files and configuration for bringing up the entire Flying Chameleons (FlyChams) system. This package contains everything necessary to configure and start the system, including launch files, parameter configurations, and element registration nodes.

## Overview

The `flychams_bringup` package serves as the entry point for running the FlyChams system. It provides:

1. **Launch files** for setting up and running the entire system
2. **Registration nodes** for registering agents, targets, clusters, and GUI elements
3. **Configuration files** for system parameters, topic names, and frame IDs
4. **Utility nodes** for setup and runtime needs

## Components

### Launch Files

- `setup.launch.py` - Sets up the environment, including:
  - AirSim wrapper initialization (i.e. AirSim clients, AirSim settings, transformation broadcasts, etc.)
  - Element registration nodes (for agents, targets, clusters), so that they can be discovered by other nodes
  - Loading and creating dynamic system configurations (i.e. airsim settings, etc.)

- `run.launch.py` - Launches the runtime system, including:
  - Control nodes
  - Perception nodes
  - Coordination nodes
  - Target nodes
  - Dashboard nodes

- `rviz.launch.py` - Convenience launch for launching RViz2 with appropriate configuration

- `rosbag.launch.py` - Convenience launch for recording system data to a rosbag file

- `airsim_settings.launch.py` - Convenience launch for creating AirSim settings JSON file

### Setup Nodes

Setup nodes are responsible for initializing the system and preparing it for runtime operation:

1. **Registrator** - The main registration node that manages other registration modules:
   - Registers agents, targets, and clusters based on configuration
   - Initializes external tools (e.g. AirSim)

2. **AirSim Settings** - Utility for parsing mission configurations:
   - Generates AirSim settings JSON file based on the parsed configuration

### Runtime Nodes

Runtime nodes are launched by the `run.launch.py` file and perform the core functionality of the system:

1. **Control Nodes** (from flychams_control package):
   - `drone_control:` Controls the movement of aerial agents
   - `head_control:` Controls the heads of the agents

2. **Perception Nodes** (from flychams_perception package):
   - `target_clustering:` Groups targets into meaningful clusters
   - `cluster_analysis:` Analyzes target cluster assignments

3. **Coordination Nodes** (from flychams_coordination package):
   - `agent_assignment:` Assigns agents to clusters
   - `agent_analysis:` Analyzes agent cluster assignments
   - `agent_positioning:` Determines optimal agent positions
   - `agent_tracking:` Manages tracking of clusters by agents

4. **Target Nodes** (from flychams_targets package):
   - `target_state:` Manages target state (for debugging)
   - `target_control:` Manages target movement and behavior

5. **Dashboard Nodes** (from flychams_dashboard package):
   - `gui_manager:` Provides user interface for system monitoring and control
   - `visualization:` Creates RViz2 visualizations and system metrics for plotting

## Usage

### Setup Phase

The setup phase initializes the environment and prepares it for operation:

```bash
ros2 launch flychams_bringup setup.launch.py
```

This will:
1. Start the AirSim interface
2. Register all elements (agents, targets, clusters)
3. Initialize the system configurations

### Runtime Phase

The runtime phase starts all the operational nodes:

```bash
ros2 launch flychams_bringup run.launch.py
```

This will start all the control, perception, coordination, target and dashboard nodes.

### Customization

You can customize which nodes are launched using launch arguments:

```bash
# Launch drone control node with warning level logging
ros2 launch flychams_bringup run.launch.py log_drone_control:=warn
```

```bash
# Don´t launch agent tracking node
ros2 launch flychams_bringup run.launch.py agent_tracking:=False
```

## Configuration

Configuration files are stored in the `config` directory and organized by category:

- `core/` - Core system configuration
  - `system.yaml` - General simulation parameters (e.g. simulation speed, paths to configuration files, etc.)
  - `topics.yaml` - Topic naming configuration
  - `frames.yaml` - Frame ID configuration

- `packages/` - Package-specific configuration
  - `control/` - Control nodes configuration
  - `perception/` - Perception nodes configuration
  - `coordination/` - Coordination nodes configuration
  - `targets/` - Target nodes configuration
  - `dashboard/` - Dashboard nodes configuration

You can customize these files to change system behavior without modifying code.