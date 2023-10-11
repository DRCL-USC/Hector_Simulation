#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    GAZEBO_A1,
    REAL_A1
};

enum class UserCommand{
    // EXIT,
    NONE,
    START,      // walking
    L2_A,       // fixedStand
    L2_B,       // passive
    L2_X,       // pushing
    L2_Y,       // probe
    L1_X,       // QPStand 
    L1_A,      
    L1_Y       
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    PDSTAND,
    QPSTAND,
    WALKING,
    PUSHING,
    PROBE,
    SLAM,       // slam
};


#endif  // ENUMCLASS_H