#include "GaitGenerator.h"

// ====================== GAIT Class Implementation ======================= //

// Constructor: Initializes gait parameters using provided values.
Gait::Gait(int nMPC_segments, Vec2<int> offsets, Vec2<int> durations, const std::string &name) : _offsets(offsets.array()), 
                                                                                                 _durations(durations.array()),
                                                                                                 _nIterations(nMPC_segments)
{
  _mpc_table = new int[nMPC_segments * 2];
  _offsetsPhase = offsets.cast<double>() / (double)nMPC_segments;     
  _durationsPhase = durations.cast<double>() / (double)nMPC_segments; 
  _stance = durations[0];                
  _swing = nMPC_segments - durations[0]; 
}

/******************************************************************************************************/
/******************************************************************************************************/

Gait::~Gait()
{
  delete[] _mpc_table;
}

/******************************************************************************************************/
/******************************************************************************************************/

// Compute and return the current subphase of contact.
Vec2<double> Gait::getContactSubPhase()
{
  Array2d progress = _phase - _offsetsPhase; 

  for (int i = 0; i < 2; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.;
    if (progress[i] > _durationsPhase[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsPhase[i];
    }
  }

  return progress.matrix();
}

/******************************************************************************************************/
/******************************************************************************************************/

// Compute and return the current subphase of swing.
Vec2<double> Gait::getSwingSubPhase()
{
  Array2d swing_offset = _offsetsPhase + _durationsPhase; 
  for (int i = 0; i < 2; i++)
    if (swing_offset[i] > 1)
      swing_offset[i] -= 1.;
  Array2d swing_duration = 1. - _durationsPhase; 

  Array2d progress = _phase - swing_offset;

  for (int i = 0; i < 2; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.;
    if (progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  return progress.matrix();
}

/******************************************************************************************************/
/******************************************************************************************************/

// Generate and return the MPC gait table.
int *Gait::mpc_gait()
{
  for (int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration) % _nIterations;
    Array2i progress = iter - _offsets; // 0 5
    for (int j = 0; j < 2; j++)
    {
      if (progress[j] < 0)
        progress[j] += _nIterations;
      if (progress[j] < _durations[j])
        _mpc_table[i * 2 + j] = 1;
      else
        _mpc_table[i * 2 + j] = 0;
    }
  }

  return _mpc_table;
}

/******************************************************************************************************/
/******************************************************************************************************/

// Update iteration and phase based on the given values.
void Gait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (double)(currentIteration % (iterationsPerMPC * _nIterations)) / (double)(iterationsPerMPC * _nIterations);
}