## ROS Neotic Package for Moment-Based IBVS

How to install the package, assuming the ros neotics workspace has already been set-up:


```bash
cd ~/catkin_ws
./run.sh
cd ~/catkin_ws
mv ~/catkin_ws/M-IBVS/* ~/catkin_ws/src/mibvs/
rm -rf M-IBVS
catkin build
source ./devel/setup.bash
```

## Launching the bebop drone in Gazebo Environment:

```bash
roslaunch mibvs spawn_gazebo.launch
```
