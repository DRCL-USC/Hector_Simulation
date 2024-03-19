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