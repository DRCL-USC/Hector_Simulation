#include "../../include/common/DesiredCommand.h"

void DesiredStateData::zero(){
    pre_stateDes = Vec12<double>::Zero();
    stateDes = Vec12<double>::Zero();
}

void DesiredStateCommand::setStateCommands(double r, double p, Vec3<double> v_des, double yaw_rate){
    if(firstRun)
    {
      data.pre_stateDes(5) = stateEstimate->rpy(2);
      firstRun = false;
    }
    
    data.stateDes(6) = v_des[0];
    data.stateDes(7) = v_des[1];
    data.stateDes(8) = 0;

    data.stateDes(9) = 0;
    data.stateDes(10) = 0;
    data.stateDes(11) = yaw_rate;

    data.stateDes(0) = stateEstimate->position(0) + dt * data.stateDes(6);
    data.stateDes(1) = stateEstimate->position(1) + dt * data.stateDes(7);
    data.stateDes(5) = data.pre_stateDes(5) + dt * data.stateDes(11);

    if(data.stateDes(5) > 3.1 && stateEstimate->rpy(2) < 0){
      data.stateDes(5) = stateEstimate->rpy(2);
    }


     if(data.stateDes(5)  < -3.1 && stateEstimate->rpy(2) > 0){
      data.stateDes(5) = stateEstimate->rpy(2);
    }

    data.pre_stateDes(5) = data.stateDes(5);

     // Roll
    data.stateDes(3) = r;

    // Pitch
    data.stateDes(4) = p;
}

double DesiredStateCommand::deadband(double command, double minVal, double maxVal){
    return (command + 1)*(maxVal-minVal)/2.0 + minVal;
}