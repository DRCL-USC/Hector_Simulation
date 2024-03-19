#include "../../include/common/SwingLegController.h"

/******************************************************************************************************/
/******************************************************************************************************/

swingLegController::swingLegController(ControlFSMData *data, Gait* gait, double dtSwing){
    this->data = data;
    this->gait = gait;
    _dtSwing = dtSwing;
    seResult = data->_stateEstimator->getResult();
    updateFootPosition();
    
    for(int i = 0; i < nLegs; i++){
      footSwingTrajectory[i].setHeight(0.0);
      footSwingTrajectory[i].setInitialPosition(pFoot_w[i]);
      footSwingTrajectory[i].setFinalPosition(pFoot_w[i]);
    }
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateFootPosition(){

    for(int i = 0; i < nLegs; i++){    
    pFoot_w[i] =  seResult.position + seResult.rBody.transpose() 
                * ( data->_biped->getHipYawLocation(i) + data->_legController->data[i].p); 
    }

    pFoot_w[0][2] = 0.0;
    pFoot_w[1][2] = 0.0;
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingStates(){
    swingStates = gait->getSwingSubPhase();
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingTimes(){
    for(int leg = 0; leg < nLegs; leg++){
        if(firstSwing[leg]){
            swingTimes[leg] = _dtSwing * gait->_swing;
        }else{
            swingTimes[leg] -= _dt;
            if(swingTimes[leg] <= 0){
                firstSwing[leg] = true;
            }
        }
    }
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeFootPlacement(){

    auto &stateCommand = data->_desiredStateCommand;

    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    Vec3<double> v_des_world;

    v_des_world = seResult.rBody.transpose() * v_des_robot;    
    for(int foot = 0; foot < nLegs; foot++){
        footSwingTrajectory[foot].setHeight(0.15);
        Vec3<double> Pf = seResult.position +
                seResult.rBody.transpose() * (data->_biped->getHipYawLocation(foot))
                + seResult.vWorld * swingTimes[foot];

        double p_rel_max =  0.3;
        double pfx_rel   =  1.75 * seResult.vWorld[0] * 0.5 * gait->_stance * _dtSwing +
                            0.1  * (seResult.vWorld[0] - v_des_world[0]);

        double pfy_rel   =  1.75 * seResult.vWorld[1] * 0.5 * gait->_stance * _dtSwing +
                            0.1  * (seResult.vWorld[1] - v_des_world[1]);
        pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

        Pf[0] += pfx_rel;
        Pf[1] += pfy_rel; 
        Pf[2] = 0.0;

        footSwingTrajectory[foot].setFinalPosition(Pf);        

    }
}
