#!/bin/bash

cd $FLYCHAMS_COSYS_AIRSIM_PATH || { echo "Directory $FLYCHAMS_COSYS_AIRSIM_PATH was not found."; exit 1; }

echo "Setting up AirSim dependencies..."

./setup.sh

echo "AirSim dependencies setup successfully"