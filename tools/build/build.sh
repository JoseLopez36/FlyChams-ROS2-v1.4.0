#!/bin/bash

# Check for -j flag to specify thread count
N_THREADS=$(nproc)
while getopts "j:" opt; do
  case $opt in
    j)
      N_THREADS=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done
shift $((OPTIND-1))

# Verify directory
WORKSPACE_DIR=$FLYCHAMS_ROS2_PATH/ros2_ws
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "❌ Directory $WORKSPACE_DIR was not found."
    exit 1
fi

cd "$WORKSPACE_DIR" || { echo "❌ Failed to navigate to $WORKSPACE_DIR"; exit 1; }

# Verify arguments
if [ $# -eq 0 ]; then
    echo ""
    echo ""
    echo ""
    echo "============================================================================================"
    echo "============================================================================================"
    echo "🚀 STARTING FULL WORKSPACE BUILD (using $N_THREADS threads)"
    echo "============================================================================================"
    echo "============================================================================================"
    echo ""
    echo ""
    echo ""
    colcon build --parallel-workers $N_THREADS --cmake-args -DCMAKE_BUILD_TYPE=Release || {
        echo ""
        echo "============================================================================================"
        echo "❌ Build error."
        echo "============================================================================================"
        echo ""
        exit 1
    }
else
    echo ""
    echo ""
    echo ""
    echo "============================================================================================"
    echo "============================================================================================"
    echo "🚀 BUILDING SPECIFIED PACKAGES: $@ (using $N_THREADS threads)"
    echo "============================================================================================"
    echo "============================================================================================"
    echo ""
    echo ""
    echo ""
    colcon build --parallel-workers $N_THREADS --packages-select "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release || {
        echo ""
        echo "============================================================================================"
        echo "❌ Build error for packages: $@"
        echo "============================================================================================"
        echo ""
        exit 1
    }
fi

source install/setup.bash

echo ""
echo ""
echo ""
echo "============================================================================================"
echo "============================================================================================"
echo "✅ BUILD PROCESS COMPLETED SUCCESSFULLY"
echo "============================================================================================"
echo "============================================================================================"
echo ""
echo ""
echo ""
