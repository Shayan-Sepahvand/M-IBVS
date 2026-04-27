#!/bin/bash

WS_DIR="$(pwd)"
mkdir -p "$WS_DIR/src"

echo ">> Importing source dependencies..."
vcs import src < ros_deps.repo

echo ">> Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y ros-noetic-geographic-msgs ros-noetic-twist-mux

mv ar_track_alvar bebop_simulator hector_gazebo hector_localization hector_quadrotor "$WS_DIR/src/" 2>/dev/null || echo "Some folders already moved or missing."

echo ">> Cleaning up M-IBVS structure..."
mv "$WS_DIR/src/M-IBVS/"* "$WS_DIR/src/" 2>/dev/null
rmdir "$WS_DIR/src/M-IBVS" 2>/dev/null

echo ">> Workspace ready to build! Run: catkin build"
