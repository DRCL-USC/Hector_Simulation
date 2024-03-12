#include <iostream>
#include "../include/common/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"

using namespace ori;
using Eigen::Dynamic;


/* =========================== Controller ============================= */
ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc) : 
 iterationsBetweenMPC(_iterations_between_mpc),
 horizonLength(10),
 dt(_dt),
 walking(horizonLength, Vec2<int>(0, 5), Vec2<int>(5, 5), "Walking"),
 standing(horizonLength, Vec2<int>(0, 0), Vec2<int>(10, 10), "Standing")
{
  gaitNumber = 1;
  dtMPC = dt * iterationsBetweenMPC;
  rpy_int[2] = 0;
  for (int i = 0; i < 2; i++)
    firstSwing[i] = true;

  foot_position.open("foot_pos.txt");
}

/******************************************************************************************************/
/******************************************************************************************************/

void ConvexMPCLocomotion::run(ControlFSMData &data)
{
  bool omniMode = false;

  auto &seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;

  // pick gait
  Gait *gait = &standing;
  if (gaitNumber == 1)
    gait = &standing;
  else if (gaitNumber == 2)
    gait = &walking;

  current_gait = gaitNumber;
  // integrate position setpoint
  Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
  Vec3<double> v_des_world;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = 0.55; //.5;;;

  // get then foot location in world frame
  for (int i = 0; i < 2; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() 
    * (data._biped->getHip2Location(i) + data._legController->data[i].p);
  }

  // some first time initialization
  if (firstRun)
  {
    std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0, 0, 0); // connect to desired state command later
    Vec3<double> v_des_world(0, 0, 0); // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    //
    if (gaitNumber == 1)
    {
      pBody_des[0] = seResult.position[0];
      pBody_des[1] = seResult.position[1];
      pBody_des[2] = 0.55;

      vBody_des[0] = 0;
      vBody_des[0] = 0;
    }

    for (int i = 0; i < 2; i++)
    {
      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }

  contact_state = gait->getContactSubPhase();

  // foot placement
  swingTimes[0] = dtMPC * gait->_swing;
  swingTimes[1] = dtMPC * gait->_swing;

  double side_sign[2] = {1, -1};
  double interleave_y[2] = {-0.1, 0.1};
  double interleave_gain = -0.2;
  double v_abs = std::fabs(seResult.vBody[0]);
  for (int i = 0; i < 2; i++)
  {
    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }

    // if (firstSwing[i])
    // {

      footSwingTrajectories[i].setHeight(0.1);
      Vec3<double> offset(0, side_sign[i] * 0.0, -0.0);
      // simple heuristic function
      Vec3<double> pRobotFrame = (data._biped->getHip2Location(i) + offset);
 
      Vec3<double> des_vel;

      des_vel[0] = stateCommand->data.stateDes(6);
      des_vel[1] = stateCommand->data.stateDes(7);
      des_vel[2] = stateCommand->data.stateDes(8);

      Vec3<double> Pf = seResult.position +
                        seResult.rBody.transpose() * pRobotFrame 
                        + seResult.vWorld * swingTimeRemaining[i];

      
      double p_rel_max = 0.4;
      double pfx_rel = -0.015 + seResult.vWorld[0] * 0.5 * gait->_stance * dtMPC +
                       0.02 * (seResult.vWorld[0] - v_des_world[0]);

      double pfy_rel = seResult.vWorld[1] * 0.5 * gait->_stance * dtMPC +
                       0.02 * (seResult.vWorld[1] - v_des_world[1]);
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      // std:: cout << "pfy_rel =" << pfy_rel << "\n";
      Pf[0] += pfx_rel;
      Pf[1] += pfy_rel; //+ interleave_y[i] * v_abs * interleave_gain;
      Pf[2] = -0.0;

      footSwingTrajectories[i].setFinalPosition(Pf);
    // }
  }

  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter); 

  // load LCM leg swing gains
  Kp << 300, 0, 0,
      0, 300, 0,
      0, 0, 300;
  Kp_stance =  0* Kp;

  Kd << 10, 0, 0,
      0, 10, 0,
      0, 0, 10;
  Kd_stance = 0*Kd;
  // gait
  Vec2<double> contactStates = gait->getContactSubPhase();
  Vec2<double> swingStates = gait->getSwingSubPhase();

  int *mpcTable = gait->mpc_gait();
  

  updateMPCIfNeeded(mpcTable, data, omniMode);

  iterationCounter++;

  Vec2<double> se_contactState(0, 0);

  for (int foot = 0; foot < 2; foot++)
  {

    double contactState = contactStates(foot);
    double swingState = swingStates(foot); 
    std::cout << "swing " << foot << ": " << swingState << std::endl;
    std::cout << "Contact " << foot << ": " << contactState << std::endl;
    Vec3<double> pFootWorld;

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      double side = -1.0 ;
      if (foot == 1){
        side = 1.0;
      }
      Vec3<double> hipOffset = {0, side*-0.02, -0.136};
      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - hipOffset;
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      if (vDesLeg.hasNaN())
      {
        vDesLeg << 0, 0, 0;
      }

      data._legController->commands[foot].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp;
      data._legController->commands[foot].kdCartesian = Kd;
      data._legController->commands[foot].kptoe = 5; // 0
      data._legController->commands[foot].kdtoe = 0.1;
      se_contactState[foot] = contactState;
    }

    else if (contactState > 0) // foot is in stance
    { 
      firstSwing[foot] = true;
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._biped->getHip2Location(foot);
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      if (vDesLeg.hasNaN())
        {
         vDesLeg << 0, 0, 0;
         }

      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp_stance; // 0
      data._legController->commands[foot].kdCartesian = Kd_stance;

      data._legController->commands[foot].kptoe = 0; // 0
      data._legController->commands[foot].kdtoe = 0;

      data._legController->commands[foot].feedforwardForce = f_ff[foot];

      se_contactState[foot] = contactState;

    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData &data, bool omniMode)
{

  if ((iterationCounter % 5) == 0)
  {

    auto seResult = data._stateEstimator->getResult();
    auto &stateCommand = data._desiredStateCommand;

    double *p = seResult.position.data();
    double *v = seResult.vWorld.data();
    double *w = seResult.omegaWorld.data();
    double *quat = seResult.orientation.data();

    //Joint angles to compute foot rotation
    Eigen::Matrix<double, 10, 1> q;
      for (int i = 0; i < 2; i++)
    {
      for (int k = 0; k < 5; k++)
      {
        q(i * 5 + k) = data._legController->data[i].q(k);
      }
    }

    double *joint_angles = q.data();

    double PI = 3.14159265359;
    //Joint angles offset correction
    q(2) +=  0.3*PI;
    q(3) -=  0.6*PI;
    q(4) +=  0.3*PI;

    q(7) +=  0.3*PI;
    q(8) -=  0.6*PI;
    q(9) +=  0.3*PI;

    double PI2 = 2*PI;
    for(int i = 0; i < 10; i++){
      q(i) = fmod(q(i) , PI2);
    }

    double r[6];
    for (int i = 0; i < 6; i++)
    {
      r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
    }
    //MPC Weights
    double Q[12] = {100, 100, 150,  200, 200, 300,  1, 1, 1,  1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    double Alpha[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

    double *weights = Q;
    double *Alpha_K = Alpha;

    double yaw = seResult.rpy[2];

    std::cout << "current position: " << p[0] << "  "<< p[1] << "  "<< p[2] << std::endl;


    v_des_robot << stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0;

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    const double max_pos_error = .05;
    double xStart = world_position_desired[0];
    double yStart = world_position_desired[1];

    if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
    if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

    if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
    if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;

    // Vec3<double> ori_des_world;
    ori_des_world << stateCommand->data.stateDes[3], stateCommand->data.stateDes[4], stateCommand->data.stateDes[5];    

    double trajInitial[12] = {/*rpy_comp[0] + */stateCommand->data.stateDes[3],  // 0
                              /*rpy_comp[1] + */stateCommand->data.stateDes[4],    // 1
                              seResult.rpy[2]*0,    // 2
                              xStart,                                   // 3
                              yStart,                                   // 4
                              0.55 ,   // 5
                              0,                                        // 6
                              0,                                        // 7
                              stateCommand->data.stateDes[11],  // 8
                              v_des_world[0],                           // 9
                              v_des_world[1],                           // 10
                              0};                                       // 11

    for (int i = 0; i < horizonLength; i++)
    {
      for (int j = 0; j < 12; j++)
        trajAll[12 * i + j] = trajInitial[j];

      if(i == 0) // start at current position  TODO consider not doing this
      {
        trajAll[0] = seResult.rpy[0];
        trajAll[1] = seResult.rpy[1];
        trajAll[2] = seResult.rpy[2];
        trajAll[3] = seResult.position[0];
        trajAll[4] = seResult.position[1];
        trajAll[5] = seResult.position[2];
      }
      else
      {
        if (v_des_world[0] == 0) {
        trajAll[12*i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
        }
        else{
         trajAll[12*i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0]; 
        }
        if (v_des_world[1] == 0) {
        trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
        }
        else{
         trajAll[12*i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1]; 
        }
        if (stateCommand->data.stateDes[11] == 0){
        trajAll[12*i + 4] = trajInitial[4];
         }
        else{
        trajAll[12*i + 2] = yaw + i * dtMPC * stateCommand->data.stateDes[11];
        //std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
        }
      }
      std::cout << "traj " << i << std::endl;
      for (int j = 0; j < 12; j++) {
        std::cout << trajAll[12 * i + j] << "  ";
      }
          std::cout<< " " <<std::endl;

    }

    //MPC Solver Setup
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC, horizonLength, 0.25, 500);
    
    //Solve MPC
    Timer t_mpc_solve;
    t_mpc_solve.start();
    update_problem_data(p, v, quat, w, r, joint_angles ,yaw, weights, trajAll, Alpha_K, mpcTable);
    printf("MPC Solve time %f ms\n", t_mpc_solve.getMs());

    //Get solution and update foot forces    
    for (int leg = 0; leg < 2; leg++)
    {
      Vec3<double> GRF;
      Vec3<double> GRF_R;
      Vec3<double> GRM;
      Vec3<double> GRM_R;
      Vec6<double> f;
      for (int axis = 0; axis < 3; axis++)
      {
        GRF[axis] = get_solution(leg * 3 + axis);
        GRM[axis] = get_solution(leg * 3 + axis + 6);
      }
      GRF_R = - seResult.rBody * GRF;
      GRM_R = - seResult.rBody * GRM;
      std::cout << "RBody: " << seResult.rBody << std::endl;

      for (int i = 0; i < 3; i++){
        f(i) = GRF_R(i);
        f(i+3) = GRM_R(i);
      }
      f_ff[leg] = f;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/

  // void ConvexMPCLocomotion::GenerateTrajectory(int* mpcTable, ControlFSMData& data, bool omniMode, StateEstimate& _seResult){
    
  //   Vec3<double> v_des_world = _seResult.rBody.transpose() * v_des_robot;
  //   const double max_pos_error = .15;
  //   double xStart = world_position_desired[0];
  //   double yStart = world_position_desired[1];

  //   if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
  //   if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

  //   if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
  //   if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

  //   world_position_desired[0] = xStart;
  //   world_position_desired[1] = yStart;

  //   double trajInitial[12] = {ori_des_world[0],  // 0
  //                             ori_des_world[1],    // 1
  //                             0,    // 2
  //                             0,                                   // 3
  //                             0,                                   // 4
  //                             0.5 ,   // 5
  //                             0,                                        // 6
  //                             0,                                        // 7
  //                             stateCommand->data.stateDes[11],  // 8
  //                             v_des_world[0],                           // 9
  //                             v_des_world[1],                           // 10
  //                             0};                                       // 11

  //   for (int i = 0; i < horizonLength; i++)
  //   {
  //     for (int j = 0; j < 12; j++)
  //       trajAll[12 * i + j] = trajInitial[j];

  //     if(i == 0) // start at current position  TODO consider not doing this
  //     {
  //       trajAll[0] = seResult.rpy[0];
  //       trajAll[1] = seResult.rpy[1];
  //       trajAll[2] = seResult.rpy[2];
  //       trajAll[3] = seResult.position[0];
  //       trajAll[4] = seResult.position[1];
  //       trajAll[5] = seResult.position[2];
  //     }
  //     else
  //     {
  //       if (v_des_world[0] == 0) {
  //       trajAll[12*i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
  //       }
  //       else{
  //        trajAll[12*i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0]; 
  //       }
  //       if (v_des_world[1] == 0) {
  //       trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
  //       }
  //       else{
  //        trajAll[12*i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1]; 
  //       }
  //       if (stateCommand->data.stateDes[11] == 0){
  //       trajAll[12*i + 4] = trajInitial[4];
  //        }
  //       else{
  //       trajAll[12*i + 2] = yaw + i * dtMPC * stateCommand->data.stateDes[11];
  //       //std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
  //       }
  //     }
  //     std::cout << "traj " << i << std::endl;
  //     for (int j = 0; j < 12; j++) {
  //       std::cout << trajAll[12 * i + j] << "  ";
  //     }
  //         std::cout<< " " <<std::endl;

  //   }

  // }
