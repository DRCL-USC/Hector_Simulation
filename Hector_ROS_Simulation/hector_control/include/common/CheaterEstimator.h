#ifndef CHEATER_ESTIMATOR
#define CHEATER_ESTIMATOR

#include "StateEstimatorContainer.h"
#include "Math/orientation_tools.h"

class CheaterRobotStateEstimator: public GenericEstimator{
    public:
        virtual void run()
        {
            this->_stateEstimatorData.result->orientation[0] = this->_stateEstimatorData.lowState->imu.quaternion[0];
            this->_stateEstimatorData.result->orientation[1] = this->_stateEstimatorData.lowState->imu.quaternion[1];
            this->_stateEstimatorData.result->orientation[2] = this->_stateEstimatorData.lowState->imu.quaternion[2];
            this->_stateEstimatorData.result->orientation[3] = this->_stateEstimatorData.lowState->imu.quaternion[3];

            this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
                this->_stateEstimatorData.result->orientation);
            // when wrting to this variable, we write the world frame omega into it instead of IMU reading
            this->_stateEstimatorData.result->omegaWorld[0] = this->_stateEstimatorData.lowState->imu.gyroscope[0];
            this->_stateEstimatorData.result->omegaWorld[1] = this->_stateEstimatorData.lowState->imu.gyroscope[1];
            this->_stateEstimatorData.result->omegaWorld[2] = this->_stateEstimatorData.lowState->imu.gyroscope[2];

            this->_stateEstimatorData.result->omegaBody = 
                this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->omegaWorld;
            this->_stateEstimatorData.result->rpy =
                ori::quatToRPY(this->_stateEstimatorData.result->orientation);
            
            this->_stateEstimatorData.result->vWorld[0] = this->_stateEstimatorData.lowState->vWorld[0];
            this->_stateEstimatorData.result->vWorld[1] = this->_stateEstimatorData.lowState->vWorld[1];
            this->_stateEstimatorData.result->vWorld[2] = this->_stateEstimatorData.lowState->vWorld[2];

            this->_stateEstimatorData.result->position[0] = this->_stateEstimatorData.lowState->position[0];
            this->_stateEstimatorData.result->position[1] = this->_stateEstimatorData.lowState->position[1];
            this->_stateEstimatorData.result->position[2] = this->_stateEstimatorData.lowState->position[2];
            // don't need acceleration at all
            this->_stateEstimatorData.result->aBody[0] = 0;
            this->_stateEstimatorData.result->aBody[1] = 0;
            this->_stateEstimatorData.result->aBody[2] = 0;

            this->_stateEstimatorData.result->aWorld =
            this->_stateEstimatorData.result->rBody.transpose() *
            this->_stateEstimatorData.result->aBody;
        }
        virtual void setup() {}
};

#endif 