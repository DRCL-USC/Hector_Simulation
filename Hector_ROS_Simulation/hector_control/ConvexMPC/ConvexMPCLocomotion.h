#ifndef CONVEXMPCLOCOMOTION_H
#define CONVEXMPCLOCOMOTION_H

#include <eigen3/Eigen/Dense>
#include "../include/common/FootSwingTrajectory.h"
#include "../include/common/ControlFSMData.h"
#include "../include/common/cppTypes.h"
#include "GaitGenerator.h"
#include <fstream>

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Array4d;
using Eigen::Array2d;
using Eigen::Array2i;
using Eigen::Array2f;

using namespace std;

struct CMPC_Result {
  LegControllerCommand commands[2];
  Vec2<float> contactPhase;
};

class ConvexMPCLocomotion {
public:
  ConvexMPCLocomotion(double _dt, int _iterations_between_mpc);
  void initialize();

  void run(ControlFSMData& data);
  void setGaitNum(int gaitNum){gaitNumber = gaitNum % 7; if(gaitNum%7 ==0) gaitNumber = 7; return;}
  Vec3<double> pBody_des;
  Vec3<double> vBody_des;
  Vec3<double> aBody_des;

  Vec3<double> pBody_RPY_des;
  Vec3<double> vBody_Ori_des;

  Vec3<double> pFoot_des[2];
  Vec3<double> vFoot_des[2];
  Vec3<double> aFoot_des[2];

  Vec3<double> Fr_des[2];

  Vec2<double> contact_state;

  bool climb = 0;
  ofstream foot_position;
  bool firstRun = true;

private:
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData& data, bool omniMode);
  int iterationsBetweenMPC;
  int horizonLength;
  double dt;
  double dtMPC;
  int iterationCounter = 0;
  Vec6<double> f_ff[2];
  Vec12<double> Forces_Sol;
  Vec2<double> swingTimes;
  FootSwingTrajectory<double> footSwingTrajectories[2];
  Gait trotting, bounding, pacing, walking, galloping, pronking, standing;
  Mat3<double> Kp, Kd, Kp_stance, Kd_stance;

  bool firstSwing[2] = {true, true};
  double swingTimeRemaining[2];
  double stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<double> world_position_desired;
  Vec3<double> rpy_int;
  Vec3<double> rpy_comp;
  Vec3<double> pFoot[2];
  CMPC_Result result;
  double trajAll[12*36];

  Mat43<double> W; // W = [1, px, py]
  Vec3<double> a; // a = [a_0, a_1, a_2]; z(x,y) = a_0 + a_1x + a_2y
  Vec4<double> pz;
  double ground_pitch;
};


#endif //CONVEXMPCLOCOMOTION_H
