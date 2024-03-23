#include "../../include/common/PositionVelocityEstimator.h"

void CheaterPositionVelocityEstimator::run() {
 // std::cout << "run StateEstimator" << std::endl;
  for(int i = 0; i < 3; i++){
    this->_stateEstimatorData.result->position[i] = this->_stateEstimatorData.lowState->position[i];
    this->_stateEstimatorData.result->vWorld[i] = this->_stateEstimatorData.lowState->vWorld[i];
  }

  this->_stateEstimatorData.result->vBody=
  this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;

  // std::cout << "vx: " << this->_stateEstimatorData.result->vWorld[0] << std::endl;
  // std::cout << "vy: " << this->_stateEstimatorData.result->vWorld[1] << std::endl;
  // std::cout << "vz: " << this->_stateEstimatorData.result->vWorld[2] << std::endl;
  // std::cout << "vx body: " << this->_stateEstimatorData.result->vBody[0] << std::endl;
  // std::cout << "vy body: " << this->_stateEstimatorData.result->vBody[1] << std::endl;
  // std::cout << "vz body: " << this->_stateEstimatorData.result->vBody[2] << std::endl;

}
