#include "../../include/FSM/FSMState_Passive.h"

FSMState_Passive::FSMState_Passive(ControlFSMData *data):
                  FSMState(data, FSMStateName::PASSIVE, "passive"){}

void FSMState_Passive::enter()
{
    _data->_legController->zeroCommand();
    for(int i = 0; i < 2; i++)
    {
        _data->_legController->commands[i].kdJoint.diagonal()<< 5, 5, 5, 5, 5;
    }

}

void FSMState_Passive::run()
{
    _data->_legController->updateData(_data->_lowState);
    _data->_stateEstimator->run();
    _data->_legController->updateCommand(_data->_lowCmd);
}

void FSMState_Passive::exit()
{
    for(int i = 0; i < 2; i++)
    {
        _data->_legController->commands[i].kdJoint.setZero();
    }
}

FSMStateName FSMState_Passive::checkTransition()
{
    if(_lowState->userCmd == UserCommand::L1_X){
        FSMStateName::WALKING;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}