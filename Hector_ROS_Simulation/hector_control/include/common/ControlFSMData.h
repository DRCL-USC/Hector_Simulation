#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "DesiredCommand.h"
#include "LowLevelController.h"
#include "Biped.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"
#include "../interface/IOInterface.h"
#include "StateEstimatorContainer.h"

struct ControlFSMData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Biped *_biped;
  StateEstimatorContainer *_stateEstimator;
  LowLevelController *_legController;
  DesiredStateCommand *_desiredStateCommand;
  IOInterface *_interface;
  LowlevelCmd *_lowCmd;
  LowlevelState *_lowState;

  void sendRecv(){
    _interface->sendRecv(_lowCmd, _lowState);
  }
};


#endif  // CONTROLFSM_H