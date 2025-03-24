# Flychams Control: Control nodes for the Flying Chameleons Project

Control package for the Flying Chameleons (FlyChams) project that manages the movement and behavior of aerial agents.

## Overview

The `flychams_control` package provides control algorithms for aerial agents in the FlyChams system. It handles:

1. **Agent movement control** - Manages the flight control of aerial platforms
2. **Trajectory tracking** - Implements algorithms for following specified trajectories
3. **Position control** - Maintains desired positions based on commands from coordination nodes

## Nodes

### Agent Control

- Main node that handles agent movement:
  - Manages agent state and transitions between flight modes (e.g. arming, disarming, takingoff, moving, landing, etc.)
  - Receives position commands from coordination nodes
  - Implements speed scheduling and other control algorithms

## Configuration

Configuration parameters for the control nodes can be found in `flychams_bringup` package.