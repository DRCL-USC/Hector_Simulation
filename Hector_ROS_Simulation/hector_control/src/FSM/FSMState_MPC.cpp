#include "../../include/FSM/FSMState_MPC.h"

FSMState_MPC::FSMState_MPC(ControlFSMData *data)
                 :FSMState(data, FSMStateName::MPC, "mpc"),
                  Cmpc(0.001, 40),
                  gaitNum(2) {}

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}

void FSMState_MPC::enter()
{
    v_des_body << 0, 0, 0;
    pitch = 0;
    roll = 0;
     _data->_interface->zeroCmdPanel();
    counter = 0;
    _data->_desiredStateCommand->firstRun = true;
    _data->_stateEstimator->run(); 
    _data->_legController->zeroCommand();
    Cmpc.firstRun = true;
}

void FSMState_MPC::run()
{
    _data->_legController->updateData(_data->_lowState);
    _data->_stateEstimator->run(); 
    _userValue = _data->_lowState->userValue;

    if(gaitNum == 1 /*Standing*/){
            v_des_body[0] = 0.0;
            v_des_body[1] = 0.0;
            turn_rate = 0.0;
            _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);

            Cmpc.setGaitNum(gaitNum); 
            Cmpc.run(*_data);
    }
    else if(gaitNum == 2 /*Stepping*/){
            v_des_body[0] = (double)invNormalize(_userValue.ly, -0.75, 0.75);
            v_des_body[1] = (double)invNormalize(_userValue.rx, -0.25, 0.25);
            turn_rate = (double)invNormalize(_userValue.lx, -0.5, 0.5);
            _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);

            Cmpc.setGaitNum(gaitNum); // 2 for walking
            Cmpc.run(*_data);
    }else{
        std::cout << "Invalid gait number" << std::endl;
        gaitNum = 1; // Default to standing
    }

    _data->_legController->updateCommand(_data->_lowCmd);  
    counter++;
}

void FSMState_MPC::exit()
{      
    counter = 0; 
    _data->_interface->zeroCmdPanel();
}

FSMStateName FSMState_MPC::checkTransition()
{
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::MPC;
    }
}