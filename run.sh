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


# 1. Define paths
WS_DIR="$HOME/catkin_ws"
TARGET_SRC="$WS_DIR/src"
REPO_DIR="$WS_DIR/M-IBVS"

# 2. Ensure the main src directory exists
mkdir -p "$TARGET_SRC"

echo ">> Moving packages from $REPO_DIR/src to $TARGET_SRC..."

# 3. Move the contents. 
# We use 'shopt -s dotglob' to make sure hidden files (like .git or .gitignore) move too.
shopt -s dotglob
mv "$REPO_DIR/src/"* "$TARGET_SRC/" 2>/dev/null

# 4. Clean up the now-empty src folder inside M-IBVS
rmdir "$REPO_DIR/src" 2>/dev/null

echo ">> Done! Everything is now in $TARGET_SRC"
echo ">> Your M-IBVS configuration files (like ros_deps.repo) are still in $REPO_DIR"

# 5. Final check
ls "$TARGET_SRC"

echo "-------------------------------------------------------"
echo ">> Done! Everything is now located in: $SRC_DIR"
echo ">> Run: cd $WS_DIR && catkin build"
echo "-------------------------------------------------------"
