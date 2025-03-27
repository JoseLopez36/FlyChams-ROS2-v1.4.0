# Flychams Targets: Target management package for the Flying Chameleons Project

Target management package for the Flying Chameleons (FlyChams) project that handles target behavior and trajectories.

## Overview

The `flychams_targets` package manages the movement and behavior of targets in the FlyChams system. It handles:

1. **Target state** - Publishes target ground truth state for debugging and simulation purposes

2. **Target control** - Controls target movement in simulation

3. **Trajectory reading** - Reads target trajectories from .csv files

## Nodes

### Target State

- Node that handles target state publishing:
  - Reads target trajectory from .csv file
  - Publishes target ground truth state

### Target Control

- Node that handles target movement and behavior:
  - Reads target trajectories from .csv files
  - Commands target movement in simulation

## Configuration

Configuration parameters for the target nodes can be found in `flychams_bringup` package.