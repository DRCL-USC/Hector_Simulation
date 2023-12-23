# Hector Open-source Simulation Software in ROS

## HECTOR: Humanoid for Enhanced ConTrol and Open-source Research

This branch contains the ROS+Gazebo simulation for the Hector humanoid robot. 

For humanoid ROS model with arms use the ROS_Humanoid_Simulation branch.

<img src="https://github.com/DRCL-USC/Hector_Simulation/blob/Matlab_Simulation/STL%20files/Hector_picture.jpg" width=50% height=50%>

Introduction Video: https://youtu.be/NcW-NFwjMh0

## Dependencies:
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)
* [ROS](http://wiki.ros.org/) Neotic
* [Gazebo](https://gazebosim.org/home) 11
* [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) (>3.3)
* unitree_legged_sdk 
* [qpOASES](https://github.com/coin-or/qpOASES)
* ROS_Packages
```
sudo apt-get install ros-noetic-controller-manager ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-velocity-controllers ros-noetic-position-controllers ros-noetic-robot-controllers ros-noetic-robot-state-publisher ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

## System Requirements:
If you want to simulate with [Gazebo](http://gazebosim.org/), we recommend **x86 platform**. **ARM platform** is not suggested for simulation. So, if you run this code on ARM platform, please remove *Simulation related* folder first.

The current system environment is: 

* Ubuntu 20.04 + ROS Noetic* (recommended, tested stable) 


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

* In a new terminal, enter and source your workspace, then run: `rosrun hector_control hector_ctrl`

* Click the start button at the bottom of the simulator, the robot should stand up/move away
* In some occasions the controller does not kick in after starting, please terminate the controller with ctrl + \\. Then go back to the simulator, pause, and reset (ctrl + R). Rerun controller. 

## Keyboard Control: 
* Inside the terminal window, use W or S to control x direction speed
* Use A or D to control robot turning (TODO: seems to have a small bug when turning past 90 degrees)
* Use J or L to control y direction speed

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

2. Dynamic Loco-manipulation on HECTOR: Humanoid for Enhanced ConTrol and Open-source Research
https://arxiv.org/pdf/2312.11868.pdf
```
@article{li2023dynamic,
  title={Dynamic Loco-manipulation on HECTOR: Humanoid for Enhanced ConTrol and Open-source Research},
  author={Li, Junheng and Ma, Junchao and Kolt, Omar and Shah, Manas and Nguyen, Quan},
  journal={arXiv preprint arXiv:2312.11868},
  year={2023}
}
```

## Contact Information:
Yiyu Chen -- yiyuc@usc.edu
Junheng Li -- junhengl@usc.edu

## License 
Please read the License.md for details. 

## Acknowledgement：
The authors would like to express special thanks to MIT Biomimetic Lab for providing the [cheetah MPC framework](https://github.com/dbdxnuliba/mit-biomimetics_Cheetah) and Unitree Robotics for providing the Unitree gazebo simulation framework.

