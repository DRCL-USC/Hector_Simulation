#pragma once

#include <iostream>
#include "../include/common/cppTypes.h"

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Array4d;
using Eigen::Array2d;
using Eigen::Array2i;
using Eigen::Array2f;
using namespace std;


/**
 * @file gaitgenerator.h
 * @brief Gait Generation for Bipedal Locomotion
 *
 * This file provides the definition for the Gait class, which is responsible for
 * generating and managing various gait patterns for a bipedal robot. 
 *
 */


class Gait
{
public:
  Gait(int nMPC_segments, Vec2<int> offsets, Vec2<int>  durations, const std::string& name="");
  ~Gait();
  Vec2<double> getContactSubPhase();
  Vec2<double> getSwingSubPhase();
  int* mpc_gait();
  void setIterations(int iterationsPerMPC, int currentIteration);
  int _stance;
  int _swing;


private:
  int _nMPC_segments;
  int* _mpc_table;
  Array2i _offsets;           // offset in mpc segments
  Array2i _durations;         // duration of step in mpc segments
  Array2d _offsetsPhase;      // offsets in phase (0 to 1)
  Array2d _durationsPhase;    // durations in phase (0 to 1)
  int _iteration;
  int _nIterations;
  int currentIteration;
  double _phase;

};