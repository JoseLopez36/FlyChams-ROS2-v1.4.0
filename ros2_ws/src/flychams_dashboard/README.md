# Flychams Dashboard: Visualization and monitoring package for the Flying Chameleons Project

Visualization and monitoring package for the Flying Chameleons (FlyChams) project that provides user interfaces and system visualization.

## Overview

The `flychams_dashboard` package provides user interface and visualization tools for the FlyChams system. It handles:

1. **GUI interface** - Provides a graphical user interface for monitoring the system
2. **Data visualization** - Creates visual representations of system state

## Nodes

### GUI

- Graphical user interface for system monitoring:
  - Manages windowing and display of system information

### Visualization

- Handles visual representation of system state:
  - Creates and publishes RViz2 markers
  - Provides metrics for agents, targets, clusters, etc. for plotting purposes

## Configuration

Configuration parameters for the dashboard nodes can be found in `flychams_bringup` package.