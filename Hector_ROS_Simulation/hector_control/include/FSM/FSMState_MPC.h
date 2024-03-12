#ifndef MPC_H
#define MPC_H

#include "FSMState.h"
#include "../../ConvexMPC/ConvexMPCLocomotion.h"

class FSMState_MPC: public FSMState
{
    public:
        FSMState_MPC(ControlFSMData *data);
        ~FSMState_MPC(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();
        int gaitNum; // Specifiy 1 for standing or 2 for walking
    
    private:
        ConvexMPCLocomotion Cmpc;
        int counter;
        Vec3<double> v_des_body;
        double turn_rate = 0;
        double pitch, roll;
};

#endif