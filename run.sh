#!/bin/bash

# 1. Define the Workspace and Source paths
WS_DIR="$HOME/catkin_ws"
SRC_DIR="$WS_DIR/src"

# 2. Ensure the src directory exists
mkdir -p "$SRC_DIR"

# 3. Identify where we are currently (inside M-IBVS)
# We assume you git cloned M-IBVS into ~/catkin_ws/src/M-IBVS
CURRENT_DIR="$(pwd)"

echo ">> Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y ros-noetic-geographic-msgs ros-noetic-twist-mux python3-vcstool

echo ">> Importing source dependencies to $SRC_DIR..."
# 'vcs import ..' tells the tool to put the new repos in the parent folder
# so they sit side-by-side with M-IBVS
if [ -f "ros_deps.repo" ]; then
    vcs import .. < ros_deps.repo
else
    echo "!! Error: ros_deps.repo not found in $(pwd)"
    exit 1
fi

echo ">> Finalizing dependencies with rosdep..."
# This is better than manual apt-get installs. It reads the package.xml 
# of all folders in src and installs what's missing.
cd "$SRC_DIR"

echo "-------------------------------------------------------"
echo ">> Done! Everything is now located in: $SRC_DIR"
echo ">> Run: cd $WS_DIR && catkin build"
echo "-------------------------------------------------------"
