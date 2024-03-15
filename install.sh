#!/bin/bash

cp -rf ./Hector_ROS_Simulation/* ../catkin_ws/src/
cd ../catkin_ws/
catkin_make
