#!/bin/bash

cd $FLYCHAMS_COSYS_AIRSIM_PATH || { echo "Directory $FLYCHAMS_COSYS_AIRSIM_PATH was not found."; exit 1; }

echo "Building AirSim dependencies..."

./build.sh

echo "AirSim dependencies built successfully"