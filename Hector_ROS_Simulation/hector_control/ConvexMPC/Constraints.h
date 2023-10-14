#ifndef CONSTRAINT_H
#define CONSTRAINT_H
#include <eigen3/Eigen/Dense>
#include "common_types.h"
#include "RobotState.h"

/**
 * @file Constraints.h
 * @brief Constraints calculations for force-and-moment based convex MPC
 *
 * This file provides the definition for the Constraints class, which is responsible for
 * calculating the constraints needed for the convex MPC including friction cone constraints,
 * force limits, and moment limits.
 *
 */ 

class Constraints {
public:
    Constraints(RobotState& rs, const Eigen::MatrixXf& q, int horizon, int num_constraints, float motorTorqueLimit, float big_number, float f_max, const Eigen::MatrixXf& gait);

    Eigen::VectorXf GetUpperBound() const;
    Eigen::VectorXf GetLowerBound() const;
    Eigen::MatrixXf GetConstraintMatrix() const;
    
private:
    RobotState& rs_;
    int Num_Variables;
    int Num_Constraints;
    int horizon_;
    float motorTorqueLimit_;
    float BIG_NUMBER_;
    float f_max_;
    Eigen::MatrixXf gait_;
    Eigen::MatrixXf q;
    Matrix<fpt, 3, 3> R_foot_L;
    Matrix<fpt, 3, 3> R_foot_R;

    Eigen::VectorXf U_b; // Upper bound
    Eigen::VectorXf L_b; // Lower bound

    Eigen::MatrixXf A_c; // Constraint matrix

    void CalculateUpperBound();
    void CalculateLowerBound();   
    void CalculateConstarintMatrix(); 
    void CalculateFootRotationMatrix();
};

#endif //CONSTRAINT_H
