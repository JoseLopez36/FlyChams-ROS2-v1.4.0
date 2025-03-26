#!/bin/bash

cd $FLYCHAMS_COSYS_AIRSIM_PATH || { echo "Directory $FLYCHAMS_COSYS_AIRSIM_PATH was not found."; exit 1; }

echo "Cleaning AirSim dependencies..."

./clean.sh

echo "AirSim dependencies cleaned successfully"