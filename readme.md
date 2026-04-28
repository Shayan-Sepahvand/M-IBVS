# 🚀AI-Enabled Image-based Visual Servoing Using ROS+Gazebo


## 📖 Overview

This project implements a computer vision pipeline to detect and track industrial waste containers using YOLOv10. By combining 2D object detection with camera intrinsics and the bin's known physical dimensions, the system accurately estimates the full 3D spatial trajectory of the container.


## ⚙️ Requirements

| Component       | Version / Details           |
|-----------------|-----------------------------|
| OS              | Ubuntu 20.04                |
| Python  | 3.8                   |
| ROS   | Noetic                       |


## ⚙️ Installation Guide

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

## How to Run

```bash
roslaunch mibvs spawn_gazebo.launch
roslaunch mibvs startup.launch
```


