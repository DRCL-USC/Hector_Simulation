#ifndef HIGHLEVELIO_H
#define HIGHLEVELIO_H

#include "ros/ros.h"
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include<boost/array.hpp>
#include "IOInterface.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighCmd.h"
#include "unitree_legged_msgs/HighState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include "planning_msgs/waypoint.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <string>

class HIGHLEVELIO : public IOInterface
{
    public:
        HIGHLEVELIO(std::string robot_name);
        ~HIGHLEVELIO();
        void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

    private:
        void sendCmd(const LowlevelCmd *cmd, LowlevelState *state);
        void recvState(LowlevelState *state);
        ros::NodeHandle _nm;
        ros::Subscriber _servo_sub[12], _imu_sub, _foot_force_sub[4], _high_cmd_sub, _high_state_sub;
        ros::Publisher _servo_pub[12], _odom_pub;
        unitree_legged_msgs::LowCmd _lowCmd;
        unitree_legged_msgs::LowState _lowState;
        std::string _robot_name;

        // tf::TransformBroadcaster _odomBroadcaster;
        ros::Time _currentTime;
        // geometry_msgs::TransformStamped _odomTF;
        nav_msgs::Odometry _odomMsg;

        void initRecv(); // initialize subscribers
        void initSend(); // initialize publishers
    
        void imuCallback(const sensor_msgs::Imu & msg);
        void HighCmdCallback(const planning_msgs::waypoint& msg);
        void HighStateCallback(const planning_msgs::waypoint& msg);
        // add cheater callback from gazebo later

        void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
        void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
        void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);

        void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
        void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
        void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);

        void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
        void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
        void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);

        void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
        void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
        void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);

        void FRfootCallback(const geometry_msgs::WrenchStamped& msg);
        void FLfootCallback(const geometry_msgs::WrenchStamped& msg);
        void RRfootCallback(const geometry_msgs::WrenchStamped& msg);
        void RLfootCallback(const geometry_msgs::WrenchStamped& msg);

        boost::array<double, 36> _odom_pose_covariance = {1e-9, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 1e-9, 0, 0, 0, 
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0, 
                                        0, 0, 0, 0, 1e6, 0, 
                                        0, 0, 0, 0, 0, 1e-9};
        boost::array<double, 36> _odom_twist_covariance = {1e-9, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 1e-9, 0, 0, 0, 
                                        0, 0, 1e6, 0, 0, 0, 
                                        0, 0, 0, 1e6, 0, 0, 
                                        0, 0, 0, 0, 1e6, 0, 
                                        0, 0, 0, 0, 0, 1e-9};
};

#endif