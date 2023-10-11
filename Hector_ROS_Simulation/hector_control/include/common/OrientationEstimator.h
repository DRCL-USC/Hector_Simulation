/*!
 * @file
 * @brief Orientation Estimation Algorithms
 * 
 * orientation: quaternion
 * rBody: transformation matrix( vBody = Rbody * vWorld)
 * omegaBody: angular vel in body frame
 * omegaWorld: ... in world frame
 * rpy: roll pitch yaw
 */

#ifndef PROJECT_ORIENTATIONESTIMATOR_H
#define PROJECT_ORIENTATIONESTIMATOR_H

#include "StateEstimatorContainer.h"
#include "Math/orientation_tools.h"

/*!
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 */

class CheaterOrientationEstimator : public GenericEstimator {
 public:
  virtual void run();
  virtual void setup() {}
};

#endif