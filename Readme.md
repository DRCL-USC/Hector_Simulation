# Hector Open-source Simulation Software in ROS

## HECTOR: Humanoid for Enhanced ConTrol and Open-source Research

This branch contains the ROS+Gazebo simulation for the Hector humanoid robot. 

<img src="https://github.com/DRCL-USC/Hector_Simulation/blob/Matlab_Simulation/STL%20files/Hector_picture.jpg" width=50% height=50%>

Introduction Video: https://youtu.be/NcW-NFwjMh0

## Dependencies:
1. 


## System Requirements:
If you want to simulate with [Gazebo](http://gazebosim.org/), we recommend **x86 platform**. **ARM platform** is not suggested for simulation. So, if you run this code on ARM platform, please remove *Simulation related* folder first.

The current system environment is: 

* Ubuntu 20.04 + ROS Noetic* (recommended, tested stable) 


## Installation:
+ 


## Configuration:
Use command to open .bashrc file:
* `gedit ~/.bashrc`

Make sure the following exist in your `~/.bashrc` file or export them in terminal. `noetic`, `gazebo-11` and `~/catkin_ws` should be replaced in your own case.
```
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo-11/setup.sh
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/catkin_ws:${ROS_PACKAGE_PATH}
export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=~/catkin_ws/devel/lib:${LD_LIBRARY_PATH}
```

## Build:
* `cd ~/catkin_ws`
* `catkin_make`

NOTE: If it is the first time to compile, Please compile the laikago_msgs first by following command:
  
complie the package 
  * `catkin_make -DCMAKE_BUILD_TYPE=Release`


### launch and run gazebo simulation:
* `roslaunch unitree_gazebo biped.launch`
The robot should be standing on the ground 

* In a new terminal: `rosrun hector_control hector_ctrl`

* Click the start button at the bottom of the simulator, the robot should stand up/move away


## Cite Us:
Thank you for choosing our software for your research and development, we highly appreciate your citing our work:

1. Force-and-moment-based model predictive control for achieving highly dynamic locomotion on bipedal robots: https://arxiv.org/abs/2104.00065
``` 
  @inproceedings{li2021force,
  title={Force-and-moment-based model predictive control for achieving highly dynamic locomotion on bipedal robots},
  author={Li, Junheng and Nguyen, Quan},
  booktitle={2021 60th IEEE Conference on Decision and Control (CDC)},
  pages={1024--1030},
  year={2021},
  organization={IEEE}
}
```

## Contact Information:
Yiyu Chen -- yiyuc@usc.edu
Junheng Li -- junhengl@usc.edu

## License 
Please read the License.md for details. 

