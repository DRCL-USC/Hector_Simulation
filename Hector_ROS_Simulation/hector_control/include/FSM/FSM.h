#ifndef FSM_H
#define FSM_H

#include "FSMState.h"
#include "FSMState_Passive.h"
#include "FSMState_Walking.h"
#include "FSMState_TO.h"
#include "../common/enumClass.h"

struct FSMStateList{
    FSMState *invalid;
    FSMState_Passive *passive;
    FSMState_Walking *walking;
    FSMState_TO *TO;

   
    void deletePtr(){
        delete invalid;
        delete passive;
        delete walking;
        delete TO;
    }  
};

class FSM{
    public:
        FSM(ControlFSMData *data);
        ~FSM();
        void initialize();
        void run();
    private:
        FSMState* getNextState(FSMStateName stateName);
        bool checkSafty();
        ControlFSMData *_data;
        FSMState *_currentState;
        FSMState *_nextState;
        FSMStateName _nextStateName;
        FSMStateList _stateList;
        FSMMode _mode;
        long long _startTime;
        int count;
};

#endif