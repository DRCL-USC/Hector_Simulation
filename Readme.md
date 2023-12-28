# Hector Open-source Simulation Software in MATLAB/Simulink

## HECTOR: Humanoid for Enhanced ConTrol and Open-source Research

This branch contains the MATLAB/Simulink simulation for the Hector humanoid robot. 

<img src="https://github.com/DRCL-USC/Hector_Simulation/blob/Matlab_Simulation/STL%20files/Hector_picture.jpg" width=50% height=50%>

Introduction Video: https://youtu.be/NcW-NFwjMh0

## Dependencies:
1. [MATLAB & Simulink 2023a + ](https://www.mathworks.com/?s_tid=gn_logo)
2. [Simulink Simscape + Simscape Multibody Library](https://www.mathworks.com/products/simscape-multibody.html)
3. [CasADi for MATLAB](https://web.casadi.org/)
4. [qpOASES](https://github.com/coin-or/qpOASES) (included in CasADi)

## System Requirements:
+ It is suggested to run this simulation on a Windows PC with Windows 10 and later;
+ Linux PC may run into problems with path definitions in Simulink (but easily fixable on the user end);

## Installation:
+ Download and unzip Hector_Simulation to your desired directory; 
+ Download and unzip [CasADi MATLAB](https://web.casadi.org/get/) to the same directory;
+ Download and install MATLAB 2023a (or later version);

## How to Run:
+ Launch MATLAB 2023a (or later version);
+ Add to path: entire simulation folder and subfolders, entire CasADi Library folder and subfolders;
+ Open and run Initiate_Simulation.m to load model and simulation parameters;
+ Open Hector_Simulation.slx and modify simulation base on your need;
+ Press Ctrl + D to build model;
+ Run model/simulation in Mechanics Explorer (note that simulation does not run in real-time);
+ User may comment/uncomment arm assembly and control in Simulink model;

## Switch controller for Loco-manipulation:
+ Default controller is LocomotionMPC
+ If using LocoManipulationMPC, please modify MPC block and add a clock to the input for the contact schedule feature.
+ You may create an object for the robot to handle via variable mass blocks in Simscape.

## Cite Us:
Thank you for choosing our software for your research and development, we highly appreciate your citing our work:

1. HECTOR Project: "Dynamic Loco-manipulation on HECTOR: Humanoid for Enhanced ConTrol and Open-source Research"
https://arxiv.org/pdf/2312.11868.pdf
```
@article{li2023dynamic,
  title={Dynamic Loco-manipulation on HECTOR: Humanoid for Enhanced ConTrol and Open-source Research},
  author={Li, Junheng and Ma, Junchao and Kolt, Omar and Shah, Manas and Nguyen, Quan},
  journal={arXiv preprint arXiv:2312.11868},
  year={2023}
}
```

2. Force-and-moment-based Locomotion MPC: "Force-and-moment-based model predictive control for achieving highly dynamic locomotion on bipedal robots"  https://arxiv.org/abs/2104.00065
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
Junheng Li -- junhengl@usc.edu

## License：
Both Hector simulation and controller code follow the 3-Clause BSD License. Please read the License.md for details. 

## Acknowledgment:
The authors would like to thank Zhanhao Le and Han Gong for contributing to the Matlab/Simulink open-source software.

