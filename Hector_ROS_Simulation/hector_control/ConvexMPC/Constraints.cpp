#include "Constraints.h"

// ====================== MPC Solver Constraints Class Implementation ======================= //

Constraints::Constraints(RobotState& rs, const Eigen::MatrixXf& jointAngles, int horizon, int num_constraints, float motorTorqueLimit, float big_number, float f_max, const Eigen::MatrixXf& gait)
: rs_(rs), q(jointAngles), horizon_(horizon), Num_Constraints(num_constraints) , motorTorqueLimit_(motorTorqueLimit), BIG_NUMBER_(big_number), f_max_(f_max), gait_(gait) {

    Num_Variables = 12;
    U_b = Eigen::VectorXf::Zero(horizon * Num_Constraints);
    L_b = Eigen::VectorXf::Zero(horizon * Num_Constraints);
    A_c = Eigen::MatrixXf::Zero(Num_Constraints, Num_Variables);

    CalculateUpperBound();
    // CalculateLowerBound();
    CalculateFootRotationMatrix();
    CalculateConstarintMatrix();
    
}

/******************************************************************************************************/
/******************************************************************************************************/
//Calculates upper bound for the QP
void Constraints::CalculateUpperBound() {

  for (s16 leg = 0; leg < 2; leg ++){

    for(s16 i = 0; i < horizon_; i++){
      for (s16 j = 0; j < 4; j++){  // bound
        U_b(8*leg + j + 16*i) = BIG_NUMBER_;
        L_b(8*leg + j + 16*i) = 0.0f;
      }
        U_b(8*leg + 4 + 16*i) = 0.01f;
        U_b(8*leg + 5 + 16*i) = 0.0f;
        U_b(8*leg + 6 + 16*i) = 0.0f;
        U_b(8*leg + 7 + 16*i) = f_max_ * gait_(2*i + 0);
        L_b(8*leg + 4 + 16*i) = 0.0f;
        L_b(8*leg + 5 + 16*i) = -BIG_NUMBER_;
        L_b(8*leg + 6 + 16*i) = -BIG_NUMBER_;
        L_b(8*leg + 7 + 16*i) = 0.0f ;
    }
  }  
//   for(s16 i = 0; i < horizon_; i++){
//       U_b(0 + Num_Constraints*i) = BIG_NUMBER_;
//       U_b(1 + Num_Constraints*i) = BIG_NUMBER_;
//       U_b(2 + Num_Constraints*i) = BIG_NUMBER_;
//       U_b(3 + Num_Constraints*i) = BIG_NUMBER_;

//       U_b(4 + Num_Constraints*i) = 0.01;

//     //   U_b(5 + Num_Constraints*i) = motorTorqueLimit_;
//     //   U_b(6 + Num_Constraints*i) = motorTorqueLimit_;
//     //   U_b(7 + Num_Constraints*i) = motorTorqueLimit_;
//     //   U_b(8 + Num_Constraints*i) = motorTorqueLimit_;      
//     //   U_b(9 + Num_Constraints*i) = motorTorqueLimit_;

//       U_b(5 + Num_Constraints*i) = BIG_NUMBER_;      
//       U_b(6 + Num_Constraints*i) = BIG_NUMBER_;

//       U_b(7 + Num_Constraints*i) = f_max_ * gait_(2*i + 0) ;
//       // 
//       U_b(8 + Num_Constraints*i) = BIG_NUMBER_;
//       U_b(9+ Num_Constraints*i) = BIG_NUMBER_;
//       U_b(10 + Num_Constraints*i) = BIG_NUMBER_;
//       U_b(11 + Num_Constraints*i) = BIG_NUMBER_;
      
//       U_b(12 + Num_Constraints*i) = 0.01;

//     //   U_b(18 + Num_Constraints*i) = motorTorqueLimit_;
//     //   U_b(19 + Num_Constraints*i) = motorTorqueLimit_;
//     //   U_b(20 + Num_Constraints*i) = motorTorqueLimit_;
//     //   U_b(21 + Num_Constraints*i) = motorTorqueLimit_;      
//     //   U_b(22 + Num_Constraints*i) = motorTorqueLimit_;

//       U_b(13 + Num_Constraints*i) = BIG_NUMBER_;      
//       U_b(14 + Num_Constraints*i) = BIG_NUMBER_;

//       U_b(15 + Num_Constraints*i) = f_max_ * gait_(2*i + 1);
//   }

}

/******************************************************************************************************/
/******************************************************************************************************/
//Calculates lower bound for the QP
void Constraints::CalculateLowerBound() {
  // QP Lower Bound
  for(s16 i = 0; i < horizon_; i++){
      L_b(0 + Num_Constraints*i) = 0.0f;
      L_b(1 + Num_Constraints*i) = 0.0f;
      L_b(2 + Num_Constraints*i) = 0.0f;
      L_b(3 + Num_Constraints*i) = 0.0f;

      L_b(4 + Num_Constraints*i) = 0.0f;
      L_b(5 + Num_Constraints*i) = 0.0f;      
      L_b(6 + Num_Constraints*i) = 0.0f;

    //   L_b(7 + Num_Constraints*i) = -motorTorqueLimit_;
    //   L_b(8 + Num_Constraints*i) = -motorTorqueLimit_;
    //   L_b(9 + Num_Constraints*i) = -motorTorqueLimit_;
    //   L_b(10 + Num_Constraints*i) = -motorTorqueLimit_;      
    //   L_b(11 + Num_Constraints*i) = -motorTorqueLimit_;

      L_b(7 + Num_Constraints*i) = 0.0f ;
      // 
      L_b(8 + Num_Constraints*i) = 0.0f;
      L_b(9 + Num_Constraints*i) = 0.0f;
      L_b(10 + Num_Constraints*i) = 0.0f;
      L_b(11 + Num_Constraints*i) = 0.0f;
      
      L_b(12 + Num_Constraints*i) = 0.0f;
      L_b(13 + Num_Constraints*i) = 0.0f;      
      L_b(14 + Num_Constraints*i) = 0.0f;

    //   L_b(20 + Num_Constraints*i) = -motorTorqueLimit_;
    //   L_b(21 + Num_Constraints*i) = -motorTorqueLimit_;
    //   L_b(22 + Num_Constraints*i) = -motorTorqueLimit_;
    //   L_b(23 + Num_Constraints*i) = -motorTorqueLimit_;      
    //   L_b(24 + Num_Constraints*i) = -motorTorqueLimit_;

      L_b(15 + Num_Constraints*i) = 0.0f;
  }    
}

/******************************************************************************************************/
/******************************************************************************************************/
//Calculates constraint matrix for the QP
void Constraints::CalculateConstarintMatrix() {
        fpt mu = 5.0;
        fpt lt = 0.09;
        fpt lh = 0.06;
        Matrix<fpt,1,3> Moment_selection(1.f, 0, 0);
        Matrix<fpt,1,3> lt_vec;
        Matrix<fpt,1,3> lt_3D;
        lt_vec << 0, 0, lt;

        Matrix<fpt,1,3> lh_vec;
        Matrix<fpt,1,3> lh_3D;
        lh_vec << 0, 0, lh;

        Matrix<fpt,1,3> M_vec;
        M_vec << 0, 1.0, 0;
        Matrix<fpt,1,3> M_3D;

//leg 1
  A_c.block<1, 12>(0, 0) //Friction leg 1
      << -mu, 0, 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  A_c.block<1, 12>(1, 0)
      << mu, 0, 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  A_c.block<1, 12>(2, 0)
      << 0, -mu, 1.f,  0, 0, 0, 0, 0, 0, 0, 0, 0;
  A_c.block<1, 12>(3, 0)
      << 0,  mu, 1.f,  0, 0, 0, 0, 0, 0, 0, 0, 0;        
  A_c.block<1, 12>(4, 0) //Mx Leg 1
      << 0, 0, 0, 0, 0, 0, Moment_selection * R_foot_L.transpose()* rs_.R.transpose(),  0, 0, 0;
  // F_control.block<5, 3>(5, 0) 
  //     << Jv_L.transpose() * rs.R.transpose();
  // F_control.block<5, 3>(5, 6) 
  //     << Jw_L.transpose() * rs.R.transpose();    
  A_c.block<1, 12>(5, 0) //Line Leg 1
      << -lt_vec * R_foot_L.transpose()* rs_.R.transpose(),   0, 0, 0,  M_vec * R_foot_L.transpose()* rs_.R.transpose(),  0, 0, 0;
  A_c.block<1, 12>(6, 0)
      <<  -lh_vec * R_foot_L.transpose()* rs_.R.transpose(),   0, 0, 0, -M_vec * R_foot_L.transpose()* rs_.R.transpose(),  0, 0, 0;
  A_c.block<1, 12>(7, 0) //Fz Leg 1
      << 0, 0, 2.f, 0, 0, 0,   0, 0, 0, 0, 0, 0;
  

//leg 2
  A_c.block<1, 12>(8, 0) //Friction leg 2
      <<  0, 0, 0,   -mu, 0, 1.f, 0, 0, 0, 0, 0, 0;
  A_c.block<1, 12>(9, 0)
      <<  0, 0, 0,    mu, 0, 1.f, 0, 0, 0, 0, 0, 0;
  A_c.block<1, 12>(10, 0)
      <<   0, 0, 0, 0, -mu, 1.f, 0, 0, 0, 0, 0, 0;
  A_c.block<1, 12>(11, 0)
      <<   0, 0, 0, 0, mu, 1.f, 0, 0, 0, 0, 0, 0;        
  A_c.block<1, 12>(12, 0) //Mx Leg 1
      << 0, 0, 0, 0, 0, 0, 0, 0, 0, Moment_selection * R_foot_R.transpose()* rs_.R.transpose();
  // F_control.block<5, 3>(5, 3) 
  //     << Jv_R.transpose() * rs.R.transpose();
  // F_control.block<5, 3>(5, 9) 
  //     << Jw_R.transpose() * rs.R.transpose();  
  A_c.block<1, 12>(13, 0) //Line Leg 2
      << 0, 0, 0,     -lt_vec * R_foot_R.transpose()* rs_.R.transpose(), 0, 0, 0,    M_vec * R_foot_R.transpose()* rs_.R.transpose();
  A_c.block<1, 12>(14, 0)
      << 0, 0, 0,    -lh_vec * R_foot_R.transpose()* rs_.R.transpose(),  0, 0, 0,   M_vec * R_foot_R.transpose()* rs_.R.transpose();  
  A_c.block<1, 12>(15, 0)  //Fz Leg 2
      << 0, 0, 0, 0, 0, 2.f, 0, 0, 0, 0, 0, 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
//Calculates foot rotation matrix needed for constraint matrix
void Constraints::CalculateFootRotationMatrix() {
    R_foot_L << - 1.0*sin(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))) - cos(q(4))*(1.0*sin(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))), -1.0*cos(q(1))*sin(q(0)), cos(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))) - sin(q(4))*(1.0*sin(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))),
              cos(q(4))*(cos(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) - 1.0*sin(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))) - 1.0*sin(q(4))*(sin(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) + cos(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))),      cos(q(0))*cos(q(1)), cos(q(4))*(sin(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) + cos(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))) + sin(q(4))*(cos(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) - 1.0*sin(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))),
                                                                                                                                                                                                                            -1.0*sin(q(2) + q(3) + q(4))*cos(q(1)),              sin(q(1)),                                                                                                                                                                                                                             cos(q(2) + q(3) + q(4))*cos(q(1));
    R_foot_R << - 1.0*sin(q(9))*(cos(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + sin(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))) - cos(q(9))*(1.0*sin(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) - cos(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))), -1.0*cos(q(6))*sin(q(5)), cos(q(9))*(cos(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + sin(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))) - sin(q(9))*(1.0*sin(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) - cos(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))),
              cos(q(9))*(cos(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) - 1.0*sin(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))) - 1.0*sin(q(9))*(sin(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + cos(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))),      cos(q(5))*cos(q(6)), cos(q(9))*(sin(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + cos(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))) + sin(q(9))*(cos(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) - 1.0*sin(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))),
                                                                                                                                                                                                                            -1.0*sin(q(7) + q(8) + q(9))*cos(q(6)),              sin(q(6)),                                                                                                                                                                                                                             cos(q(7) + q(8) + q(9))*cos(q(6));
  

}

/******************************************************************************************************/
/******************************************************************************************************/

Eigen::VectorXf Constraints::GetUpperBound() const {
    return U_b;
}

/******************************************************************************************************/
/******************************************************************************************************/

Eigen::VectorXf Constraints::GetLowerBound() const {
    return L_b;
}

/******************************************************************************************************/
/******************************************************************************************************/

Eigen::MatrixXf Constraints::GetConstraintMatrix() const {
    return A_c;
}