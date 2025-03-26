# Flychams Core: Foundation and Utilities for the Flying Chameleons Project

The core package for the Flying Chameleons (FlyChams) project, providing domain models, utilities, and interfaces used across the entire system architecture.

## Overview

The `flychams_core` package serves as the foundation for the FlyChams system. It contains:

1. **Core domain types and models** for representing agents, targets, and clusters (commonly named "elements" across the project)
2. **Configuration parsing** for mission settings and simulation parameters
3. **Utility functions** for mathematical operations, camera parameters, and more
4. **Tools** for interaction with utilities and external frameworks (e.g. AirSim)
5. **Base classes** for ROS node implementations

This package follows a domain-driven design approach, separating the core domain models from implementation details.

## Components

### Base Module

- `base_module.hpp` - Base class for all modules (i.e. sub-nodes) in the system, providing common utilities
- `base_discoverer_node.hpp` - Base class for node implementations that discover elements
- `base_registrator_node.hpp` - Base class for node implementations that register elements

### Types

- `core_types.hpp` - Core domain types including:
  - Enumerations for various domain concepts (Framework, TrackingMode, HeadRole, etc.)
  - Data structures for geometry (Pose, Twist, Odometry)
  - Camera and tracking related structures (CameraParameters, WindowParameters, etc.)
  - Metrics structures for performance monitoring

- `ros_types.hpp` - ROS-specific type definitions, including custom message types

### Config

- `config_parser.hpp` - Parses configuration files in Excel format ("Configuration.xlsx" found in project root, under config folder)
- `config_types.hpp` - Data structures for configuration (MissionConfig, AgentConfig, etc.)

### Tools

- `config_tools.hpp` - Tools for working with configuration objects
- `external_tools.hpp` - Base class for tools that interface with external frameworks (e.g. AirSim)
- `airsim_tools.hpp` - AirSim-specific utilities for interfacing with the AirSim simulator
- `topic_tools.hpp` - Utilities for topic naming and management (e.g. topic names, subscriber/publisher creation, etc.)
- `transform_tools.hpp` - Transformation utilities (e.g. frame names, coordinate transformations, etc.)

### Utils

- `ros_utils.hpp` - ROS-specific utility functions
- `msg_conversions.hpp` - Conversions between domain types and ROS messages
- `camera_utils.hpp` - Camera-related utility functions (e.g. projection)
- `math_utils.hpp` - Mathematical utility functions (e.g. vector operations)

## External Dependencies

- [OpenXLSX](https://github.com/troldal/OpenXLSX) - For Excel file parsing
- [nlohmann/json](https://github.com/nlohmann/json) - For JSON file handling
- Eigen - For mathematical operations
- ROS2 - For minimal ROS-specific typedefs 