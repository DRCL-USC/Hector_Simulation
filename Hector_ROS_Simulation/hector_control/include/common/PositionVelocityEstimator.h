/*!
 * @file PositionVelocityEstimator.h
 * @brief compute body position/velocity in world/body frames
 */ 



#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H
#include "StateEstimatorContainer.h"

class CheaterPositionVelocityEstimator : public GenericEstimator{
  public:
    virtual void run();
    virtual void setup() {};
};
#endif