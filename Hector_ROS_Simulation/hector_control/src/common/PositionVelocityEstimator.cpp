#include "../../include/common/PositionVelocityEstimator.h"

void CheaterPositionVelocityEstimator::run() {
 // std::cout << "run StateEstimator" << std::endl;
  for(int i = 0; i < 3; i++){
    this->_stateEstimatorData.result->position[i] = this->_stateEstimatorData.lowState->position[i];
    this->_stateEstimatorData.result->vWorld[i] = this->_stateEstimatorData.lowState->vWorld[i];
  }

  this->_stateEstimatorData.result->vBody=
  this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;
}
