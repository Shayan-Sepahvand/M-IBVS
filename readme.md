<div align="center">

# SONN-IBVS: Robust Image-based Visual Servoing of a Quadrotor Using Self-Organizing Neural Networks


## 📖 Overview

The problem of robust image-based visual servoing of an aerial robot for tracking a moving vehicle is addressed in this brief. First, a camera perspective projection model is used with a coordinate frame attached to the quadrotor with a fixed relative pose with respect to it. Extensive ROS Gazebo simulations and real-world experiments are conducted to assess the effectiveness of this method. This is a repo presenting the following work. Please cite this article in case it is useful for you:

```bash
@article{sepahvand2024robust,
  title={Robust image-based visual servoing of an aerial robot using self-organizing neural networks},
  author={Sepahvand, Shayan and Janabi-Sharifi, Farrokh and Masnavi, Houman and Aghili, Farhad and Amiri, Niloufar},
  journal={International Journal of Control, Automation and Systems},
  volume={22},
  number={12},
  pages={3762--3776},
  year={2024},
  publisher={Springer}
}
```

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

This only runs the point-based IBVS without SONN.

```bash
roslaunch mibvs spawn_gazebo.launch
roslaunch mibvs startup.launch
```


