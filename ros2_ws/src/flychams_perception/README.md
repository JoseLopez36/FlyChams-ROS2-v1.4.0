# Flychams Perception: Perception package for the Flying Chameleons Project

Perception package for the Flying Chameleons (FlyChams) project that processes target data to cluster them into meaningful groups.

## Overview

The `flychams_perception` package provides perception algorithms for the FlyChams system. It handles:

1. **Clustering** - Groups detected targets into meaningful clusters

## Nodes

### Clustering

- Main node that handles target clustering:
  - Receives processed target data (e.g. 3D positions, etc.)
  - Implements K-means clustering with specific modifications
  - Publishes cluster information for coordination nodes

## Configuration

Configuration parameters for the perception nodes can be found in `flychams_bringup` package.