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


/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeFootDesiredPosition(){
    for(int foot = 0; foot < nLegs; foot++){
        if(swingStates[foot] > 0){
            if (firstSwing[foot]){
              std::cout << "firstSwing[" << foot << "] = "<< firstSwing[foot] << "\n";  
              firstSwing[foot] = false;
              footSwingTrajectory[foot].setInitialPosition(pFoot_w[foot]);
               }
            //Compute and get the desired foot position and velocity
            footSwingTrajectory[foot].computeSwingTrajectoryBezier(swingStates[foot], 0.2); //FIX: second argument not used in function
            Vec3<double> pDesFootWorld = footSwingTrajectory[foot].getPosition().cast<double>();
            Vec3<double> vDesFootWorld = footSwingTrajectory[foot].getVelocity().cast<double>();

            double side = (foot == 1) ? 1.0 : -1.0; //Left foot (0) side = -1.0, Right foot (1) side = 1.0
            Eigen::Vector3d hipWidthOffSet = {-0.015, side*-0.055, 0.0}; // TODO: sync with Biped.h
            
            pFoot_b[foot] = seResult.rBody * (pDesFootWorld - seResult.position) + hipWidthOffSet ;
            vFoot_b[foot] = seResult.rBody * (vDesFootWorld*0 - seResult.vWorld);             
        }
    }    
}