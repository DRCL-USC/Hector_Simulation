#ifndef OFFLINE_TRAJ_H
#define OFFLINE_TRAJ_H
#include <ros/ros.h>
#include <ros/package.h> // to get the path to the package

#include "FSMState.h"
#include <fstream>

class FSMState_TO: public FSMState
{
    public:
      
        /**
         * @brief Construct a new FSMState_TO object
         * @param data The data to be used by the state
         * In constructor, the trajectory data is read from a file
         * modify the path to the file as needed
        */
        FSMState_TO(ControlFSMData *data);
     
        ~FSMState_TO(){}
        void enter();
        /**
         * @brief Function to run the state
         * The function is used to update the desired precomputed trajectory 
         * and send the desired state commands to the leg controller
        */
        void run();
        void exit();

        /**
         * @brief Function to check if the state should transition to another state
         * @return FSMStateName The name of the next state
         * @note If the trajectory data is not loaded, the state transitions to passive
        */
        FSMStateName checkTransition();

        bool traj_data_loaded;
    
    private:
        /**
         * @brief Function to read data from a file and return an Eigen::MatrixXd
         * @param filePath The path to the file
         * @return Eigen::MatrixXd The data read from the file
         * The data is expected be in csv format
        */
        Eigen::MatrixXd readDataFromFile(const std::string& filePath);

        /**
         * @brief Function to extract the trajectory at a given time
         * @param fullTrajectory The full trajectory data
         * @param currentTrajectory The extracted trajectory        
         * @param time The time at which to extract the trajectory
         * @param dataFrequency The frequency at which the data was recorded
         * @note If the time is greater than the time of the last data point, the last data point is returned for safety
        */
        void extractTrajectory(const Eigen::MatrixXd& fullTrajectory, Eigen::VectorXd& currentTrajectory, double time, double dataFrequency);
        
        int counter;
        Eigen::MatrixXd traj_ref;
        Eigen::VectorXd currentRef;

};

#endif