# Flychams Coordination: Coordination package for the Flying Chameleons Project

Coordination package for the Flying Chameleons (FlyChams) project that manages multi-agent cooperation for target tracking.

## Overview

The `flychams_coordination` package implements coordination algorithms for multi-agent systems in the FlyChams project. It handles:

1. **Agent positioning** - Determines optimal positions for agents
2. **Agent assignment** - Assigns agents to clusters or regions
3. **Target tracking** - Coordinates agents to maintain optimal tracking of targets

## Nodes

### Positioning

- Determines optimal positions for agents:
  - Considers tracking quality (e.g. distance, verticality, etc.)
  - Generates position commands for agents

### Assignment

- Assigns agents to clusters:
  - Implements assignment algorithms
  - Optimizes overall system performance

### Tracking

- Manages target tracking:
  - Implements novel tracking strategies
  - Maintains consistent tracking of moving targets
  - Manages the content of tracking windows for the dashboard

## Configuration

Configuration parameters for the coordination nodes can be found in `flychams_bringup` package.