#!/bin/bash
# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Source the environment file relative to the script location
source "$SCRIPT_DIR/config.env"

# Build the Docker image
echo "Building the Docker image '$IMAGE_NAME'..."
if docker build --rm -t $IMAGE_NAME --file $SCRIPT_DIR/Dockerfile .; then
    echo "Image '$IMAGE_NAME' built successfully."
else
    echo "Failed to build the Docker image."
    exit 1
fi