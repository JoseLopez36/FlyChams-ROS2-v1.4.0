# Flychams Perception: Perception package for the Flying Chameleons Project

Perception package for the Flying Chameleons (FlyChams) project that processes target data to cluster them into meaningful groups and computes their geometric properties.

## Overview

The `flychams_perception` package provides perception algorithms for the FlyChams system. It handles:

1. **Target clustering** - Groups detected targets into meaningful clusters

2. **Cluster analysis** - Analyzes cluster properties and publishes them

## Nodes

### Target Clustering

- Node that handles target clustering:
  - Receives processed target positions
  - Implements K-means clustering with specific modifications
  - Publishes target assignments for further processing

### Cluster Analysis

- Node that handles cluster analysis:
  - Receives target assignments
  - Computes enclosing circle for each cluster
  - Publishes cluster properties

## Configuration

Configuration parameters for the perception nodes can be found in `flychams_bringup` package.