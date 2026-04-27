
WS_DIR="$(pwd)"
mkdir -p "$WS_DIR/src"

echo ">> Importing source dependencies..."
# vcs handles cloning and switching branches automatically
vcs import < ros_deps.repo

echo ">> Installing system dependencies..."
sudo apt-get update
sudo apt-get install ros-noetic-geographic-msgs
sudo apt-get install ros-noetic-twist-mux

mv ar_track_alvar  bebop_simulator  hector_gazebo  hector_localization  hector_quadrotor .. 
echo ">> Workspace ready to build...(use catkin build)"
