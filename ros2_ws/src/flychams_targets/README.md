# Flychams Targets: Target management package for the Flying Chameleons Project

Target management package for the Flying Chameleons (FlyChams) project that handles target behavior and trajectories.

## Overview

The `flychams_targets` package manages the movement and behavior of targets in the FlyChams system. It handles:

1. **Target control** - Controls target movement in simulation
2. **Trajectory reading** - Reads target trajectories from .csv files

## Nodes

### Target Control

- Main node that handles target movement and behavior:
  - Reads target trajectories from .csv files
  - Publishes target state information
  - Commands target movement in simulation

## Configuration

Configuration parameters for the target nodes can be found in `flychams_bringup` package.