#!/bin/bash

# Exit on first error
set -e

# Ensure right workspace
cd ~/ws_ros2_odrive

# Build
echo "Building..."
colcon build

echo "Sourcing..."
source ./install/setup.bash

echo "Done!"