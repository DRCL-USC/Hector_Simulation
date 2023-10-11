
#include "SolverMPC.h"
#include "common_types.h"
#include "convexMPC_interface.h"
#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "../third_party/qpOASES/include/qpOASES.hpp"
#include <stdio.h>
#include <sys/time.h>
#include "../include/common/Utilities/Timer.h"
#include <fstream>

// #define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10
// big enough to act like infinity, small enough to avoid numerical weirdness.

RobotState rs;
using Eigen::Dynamic;
using std::cout;
using std::endl;
using std::ofstream;

// qpOASES::real_t a;

Matrix<fpt, Dynamic, 13> A_qp;
Matrix<fpt, Dynamic, Dynamic> B_qp;
Matrix<fpt, 13, 12> Bdt;
Matrix<fpt, 13, 13> Adt;
Matrix<fpt, 25, 25> ABc, expmm;
Matrix<fpt, Dynamic, Dynamic> S;
Matrix<fpt, Dynamic, 1> X_d;
Matrix<fpt, Dynamic, 1> U_b;
Matrix<fpt, Dynamic, 1> L_b;
Matrix<fpt, Dynamic, Dynamic> fmat;
Matrix<fpt, Dynamic, Dynamic> qH;
Matrix<fpt, Dynamic, 1> qg;
Matrix<fpt, 26, 12> F_control;
Matrix<fpt, 1, 3> tx_F;
Matrix<fpt, 1, 3> ty_F;
Matrix<fpt, 1, 3> DNFG;

// Matrix<fpt,Dynamic,Dynamic> eye_12h;
Matrix<fpt, Dynamic, Dynamic> Alpha_diag;
Matrix<fpt, Dynamic, Dynamic> Alpha_rep;

qpOASES::real_t *H_qpoases;
qpOASES::real_t *g_qpoases;
qpOASES::real_t *A_qpoases;
qpOASES::real_t *lb_qpoases;
qpOASES::real_t *ub_qpoases;
qpOASES::real_t *q_soln;

qpOASES::real_t *H_red;
qpOASES::real_t *g_red;
qpOASES::real_t *A_red;
qpOASES::real_t *lb_red;
qpOASES::real_t *ub_red;
qpOASES::real_t *q_red;
u8 real_allocated = 0;

char var_elim[2000];
char con_elim[2000];

Matrix<fpt, 3, 3> euler_to_rotation(fpt roll, fpt pitch, fpt yaw) {
    
    fpt r = roll;
    fpt p = pitch;
    fpt y = yaw;

    // Calculate rotation matrix
    Matrix<fpt, 3, 3> Rx, Ry, Rz, Rb;
    Rx << 1, 0, 0,
          0, cos(r), -sin(r),
          0, sin(r), cos(r);
    Ry << cos(p), 0, sin(p),
          0, 1, 0,
          -sin(p), 0, cos(p);
    Rz << cos(y), -sin(y), 0,
          sin(y), cos(y), 0,
          0, 0, 1;
    Matrix<fpt, 3, 3> R;
    R << 1, sin(r)*tan(p), cos(r)*tan(p),
          0, cos(r), -sin(r),
          0, sin(r)/cos(p), cos(r)/cos(p);  
    // Rb << cos(y)*cos(p), -sin(y), 0,
    //       sin(y)*cos(p), cos(y), 0,
    //       -sin(p), 0, 1; 
    // Rb << cos(y)/sin(p), sin(y)/sin(p), 0,
    //       -sin(y), cos(y), 0,
    //       -cos(p)*cos(y)/sin(p), -cos(p)*sin(y)/sin(p), 1;
    // Matrix<fpt, 3, 3> R = Rb.inverse();
    // Matrix<fpt, 3, 3> R = Rz * Ry * Rx;   
    return R;
}

Matrix<fpt, 3, 5> leg_Jv(Matrix<fpt, 3, 3> R, fpt q0,  fpt q1, fpt q2, fpt q3, fpt q4, int leg) {
    // double q0 = q(0);
    // double q1 = q(1);
    // double q2 = q(2);
    // double q3 = q(3);
    // double q4 = q(4);

    double side; // 1 for Left legs; -1 for right legs
    if (leg == 1){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = -1.0;
        }
    if (leg == 0){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = 1.0;
    }
    Matrix<fpt, 3, 5> Jv;    
    Jv(0, 0) =  sin(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135) + cos(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    Jv(1, 0) =  sin(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2))) - 1.0*cos(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135);
    Jv(2, 0) =  0.0;

    Jv(0, 1) =  -1.0*sin(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    Jv(1, 1) =  cos(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    Jv(2, 1) =  sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q1)*(0.018*side + 0.0025);

    Jv(0, 2) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2));
    Jv(1, 2) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    Jv(2, 2) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));

    Jv(0, 3) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3));
    Jv(1, 3) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    Jv(2, 3) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));

    Jv(0, 4) =  0.04*sin(q2 + q3 + q4)*sin(q0)*sin(q1) - 0.04*cos(q2 + q3 + q4)*cos(q0);
    Jv(1, 4) =  - 0.04*cos(q2 + q3 + q4)*sin(q0) - 0.04*sin(q2 + q3 + q4)*cos(q0)*sin(q1);
    Jv(2, 4) =  0.04*sin(q2 + q3 + q4)*cos(q1);
    return Jv;
   }

Matrix<fpt, 3, 5> leg_Jw(Matrix<fpt, 3, 3> R, fpt q0,  fpt q1, fpt q2, fpt q3, fpt q4, int leg) {
    // double q0 = q(0);
    // double q1 = q(1);
    // double q2 = q(2);
    // double q3 = q(3);
    // double q4 = q(4);

    double side; // 1 for Left legs; -1 for right legs
    if (leg == 1){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = -1.0;
        }
    if (leg == 0){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = 1.0;
    }
    Matrix<fpt, 3, 5> Jw;    
    Jw(1, 0) = 0.0;
    Jw(2, 0) = 0.0;
    Jw(3, 0) = 1.0;
    
    Jw(1, 1) = cos(q0);
    Jw(2, 1) = sin(q0);
    Jw(3, 1) = 0.0;

    Jw(1, 2) = -cos(q1)*sin(q0);
    Jw(2, 2) = cos(q0)*cos(q1);
    Jw(3, 2) = sin(q1);

    Jw(1, 3) = -cos(q1)*sin(q0);
    Jw(2, 3) = cos(q0)*cos(q1);
    Jw(3, 3) = sin(q1);

    Jw(1, 4) = -cos(q1)*sin(q0);
    Jw(2, 4) = cos(q0)*cos(q1);
    Jw(3, 4) = sin(q1);
    return Jw;
   }

// Returns QP solution
mfp *get_q_soln()
{
  return q_soln;
}

s8 near_zero(fpt a)
{
  return (a < 0.0001 && a > -.0001);
}

s8 near_one(fpt a)
{
  return near_zero(a - 2);
}

s8 is_even(int a)
{
  return is_even(fmod(a,2) == 0);
}

s8 is_odd(int a)
{
  return is_even(fmod(a,2) != 0);
}

// Sets parameter matrices to qpOASES type:
void matrix_to_real(qpOASES::real_t *dst, Matrix<fpt, Dynamic, Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for (s16 r = 0; r < rows; r++)
  {
    for (s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r, c);
      a++;
    }
  }
}

void c2qp(Matrix<fpt, 13, 13> Ac, Matrix<fpt, 13, 12> Bc, fpt dt, s16 horizon)
{
#ifdef K_PRINT_EVERYTHING
  cout << "Adt: \n"
       << Adt << "\nBdt:\n"
       << Bdt << endl;
#endif
  if (horizon > 19)
  {
    throw std::runtime_error("horizon is too long!");
  }

  Matrix<fpt, 13, 13> Acd = Matrix<fpt, 13, 13>::Identity() + dt * Ac;
  Matrix<fpt, 13, 12> Bcd = dt * Bc;

  for (int i = 0; i < 10; i++)
  {
    Eigen::Matrix<fpt, 13, 13> Acdm;
    Acdm = Matrix<fpt, 13, 13>::Identity();
    for (int j = 0; j < i + 1; j++)
    {
      Acdm *= Acd;
    }

    A_qp.block<13, 13>(i * 13, 0) << Acdm;
  }

  Eigen::Matrix<fpt, 13, 13> Acdp;
  for (int i = 0; i < 10; i++)
  {
    for (int j = 0; j < i + 1; j++)
    {
      Acdp = Matrix<fpt, 13, 13>::Identity();
      for (int k = 0; k < i - j; k++)
      {
        Acdp *= Acd;
      }

      if (i - j == 0)
      {
        Acdp = Matrix<fpt, 13, 13>::Identity();
      }

      B_qp.block<13, 12>(i * 13, j * 12) << Acdp * Bcd;
    }
  }

  for (int i = 0; i < 10; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      B_qp.block<13, 12>(i * 13, j * 12) << Matrix<fpt, 13, 12>::Zero();
    }
  }

 #ifdef K_PRINT_EVERYTHING
  cout << "AQP:\n"
       << A_qp << "\nBQP:\n"
       << B_qp << endl;
 #endif
}

// Resizing & initaliztation:
void resize_qp_mats(s16 horizon)
{
  int mcount = 0;
  int h2 = horizon * horizon;

  A_qp.resize(13 * horizon, Eigen::NoChange);
  mcount += 13 * horizon * 1;

  B_qp.resize(13 * horizon, 12 * horizon);
  mcount += 13 * h2 * 12;

  S.resize(13 * horizon, 13 * horizon);
  mcount += 13 * 13 * h2;

  X_d.resize(13 * horizon, Eigen::NoChange);
  mcount += 13 * horizon;

  U_b.resize(26*horizon, Eigen::NoChange);
  mcount += 26*horizon;

  L_b.resize(26*horizon, Eigen::NoChange);
  mcount += 26*horizon;

  fmat.resize(26*horizon, 12*horizon);
  mcount += 26*12*h2;

  qH.resize(12 * horizon, 12 * horizon);
  mcount += 12 * 12 * h2;

  qg.resize(12 * horizon, Eigen::NoChange);
  mcount += 12 * horizon;

  // eye_12h.resize(12*horizon, 12*horizon);
  // mcount += 12*12*horizon;

  Alpha_rep.resize(12 * horizon, 12 * horizon);
  mcount += 12 * 12 * horizon;

  // printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  L_b.setZero();
  fmat.setZero();
  qH.setZero();
  // eye_12h.setIdentity();
  Alpha_rep.setZero();

  // TODO: use realloc instead of free/malloc on size changes

  if (real_allocated)
  {

    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  H_qpoases = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_qpoases = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_qpoases = (qpOASES::real_t*)malloc(12*26*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*26*h2;
  lb_qpoases = (qpOASES::real_t*)malloc(26*1*horizon*sizeof(qpOASES::real_t));
  mcount += 26*horizon;
  ub_qpoases = (qpOASES::real_t*)malloc(26*1*horizon*sizeof(qpOASES::real_t));
  mcount += 26*horizon;
  q_soln = (qpOASES::real_t *)malloc(12 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;

  H_red = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_red = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_red = (qpOASES::real_t*)malloc(12*26*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*26*h2;
  lb_red = (qpOASES::real_t*)malloc(26*1*horizon*sizeof(qpOASES::real_t));
  mcount += 26*horizon;
  ub_red = (qpOASES::real_t*)malloc(26*1*horizon*sizeof(qpOASES::real_t));
  mcount += 26*horizon;
  q_red = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  real_allocated = 1;

  // printf("malloc'd %d floating point numbers.\n",mcount);

#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n", horizon);
#endif
}

// Matrix Operations:
inline Matrix<fpt, 3, 3> cross_mat(Matrix<fpt, 3, 3> I_inv, Matrix<fpt, 3, 1> r)
{
  Matrix<fpt, 3, 3> cm;
  cm << 0.f, -r(2), r(1),
      r(2), 0.f, -r(0),
      -r(1), r(0), 0.f;
  return I_inv * cm;
}

// continuous time state space matrices.
void ct_ss_mats(Matrix<fpt, 3, 3> I_world, fpt m, Matrix<fpt, 3, 2> r_feet, Matrix<fpt, 3, 3> R_yaw, Matrix<fpt, 13, 13> &A, Matrix<fpt, 13, 12> &B)
{
  A.setZero();
  A.block<3, 3>(0, 6) << R_yaw;
  A.block<3, 3>(3, 9) << Matrix<fpt, 3, 3>::Identity();
  A.block<3, 1>(9, 12) << 0, 0, -1.f;
  // std::cout << "///////////// A:" << A << std::endl;

  B.setZero();
  Matrix<fpt, 3, 3> I_inv = I_world.inverse();
  Matrix<fpt, 3, 3> Im;
  Im << 0, 0, 0,  0, 1.f, 0,  0, 0, 1.f;
  for (s16 b = 0; b < 2; b++)
  {
    B.block<3, 3>(6, b * 3) << cross_mat(I_inv, r_feet.col(b));
    // std::cout << "r_feet :" << r_feet.col(b) << std::endl;
  }
  B.block<3, 3>(6, 6) << I_inv ;
  B.block<3, 3>(6, 9) << I_inv ;  //switch
  B.block<3, 3>(9, 0) << Matrix<fpt, 3, 3>::Identity() / (m/1) ; //switch 
  B.block<3, 3>(9, 3) << Matrix<fpt, 3, 3>::Identity() / (m/1) ;
  // std::cout << "B:" << B << std::endl;
}

void quat_to_rpy(Quaternionf q, Matrix<fpt, 3, 1> &rpy)
{
  // from my MATLAB implementation

  // edge case!
  fpt as = t_min(2. * (q.w() * q.y() - q.x() * q.z()), .99999);
  rpy(0) = atan2(2.f * (q.w() * q.x() + q.y() * q.z()), 1. - 2.f * (sq(q.x()) + sq(q.y())));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f * (q.w() * q.z() + q.x() * q.y()), 1. - 2.f * (sq(q.y()) + sq(q.z())));
  // std::cout << "MPC solver rpy: " << rpy(0) << " " << rpy(1) << " " << rpy(2) << std::endl;
}
void print_problem_setup(problem_setup *setup)
{
  printf("DT: %.3f\n", setup->dt);
  printf("Mu: %.3f\n", setup->mu);
  printf("F_Max: %.3f\n", setup->f_max);
  printf("Horizon: %d\n", setup->horizon);
}

void print_update_data(update_data_t *update, s16 horizon)
{
  print_named_array("p", update->p, 1, 3);
  print_named_array("v", update->v, 1, 3);
  print_named_array("q", update->q, 1, 4);
  print_named_array("w", update->w, 1, 3);
  print_named_array("r", update->r, 3, 2);
  pnv("Yaw", update->yaw);
  print_named_array("weights", update->weights, 1, 12);
  print_named_array("trajectory", update->traj, horizon, 12);
  print_named_array("Alpha", update->Alpha_K, 1, 12);
  print_named_array("gait", update->gait, horizon, 2);
}

Matrix<fpt, 13, 1> x_0;
Matrix<fpt, 3, 3> I_world;
Matrix<fpt, 13, 13> A_ct;
Matrix<fpt, 13, 12> B_ct_r;

// Main function:
void solve_mpc(update_data_t *update, problem_setup *setup)
{
  //Joint angles to compute foot rotation
  Matrix<fpt, 10, 1> q;
  for(int i = 0; i < 10; i++)
  {
    q(i) = update->joint_angles[i];
  }

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
        
  // cout << "solver q:\n"
      //  << q << endl;
  //
  rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw);
#ifdef K_PRINT_EVERYTHING

  printf("-----------------\n");
  printf("   PROBLEM DATA  \n");
  printf("-----------------\n");
  print_problem_setup(setup);

  printf("-----------------\n");
  printf("    ROBOT DATA   \n");
  printf("-----------------\n");
  rs.print();
  print_update_data(update, setup->horizon);
#endif

  // roll pitch yaw
  Matrix<fpt, 3, 1> rpy;
  quat_to_rpy(rs.q, rpy);
  // Eigen::Matrix3d rot_mat;
  Matrix<fpt, 3, 3> Rb;
  std::cout << "rpy: " << rpy(0) << ", " << rpy(1) << ", " << rpy(2) << std::endl;
  //  std::cout << "R: " << rs.R << std::endl;
  Rb = euler_to_rotation(rpy(0), rpy(1), rpy(2));
  // std::cout << "rpy: \n" << rpy << std::endl;
  // std::cout << "ROTM: \n" << rot_mat << std::endl;
  // initial state (13 state representation)
  x_0 << rpy(0), rpy(1), rpy(2), rs.p, rs.w, rs.v, 9.81f;
  I_world = rs.R * rs.I_body * rs.R.transpose(); // original
  // std::cout << "I_world: " << I_world << std::endl;
  // std::cout << "I_body: "  << rs.I_body << std::endl;
  // I_world = rs.R_yaw.transpose() * rs.I_body * rs.R_yaw;
  // cout<<rs.R_yaw<<endl;
  // ct_ss_mats(I_world, rs.m, rs.r_feet, rs.R_yaw, A_ct, B_ct_r);
  ct_ss_mats(I_world, 12.0, rs.r_feet, Rb, A_ct, B_ct_r);
  // std::cout << "r_feet: "  << rs.r_feet << std::endl;

  // Rotation of Foot:
  Matrix<fpt, 3, 3> R_foot_L;
  Matrix<fpt, 3, 3> R_foot_R;
  R_foot_L << - 1.0*sin(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))) - cos(q(4))*(1.0*sin(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))), -1.0*cos(q(1))*sin(q(0)), cos(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))) - sin(q(4))*(1.0*sin(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))),
              cos(q(4))*(cos(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) - 1.0*sin(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))) - 1.0*sin(q(4))*(sin(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) + cos(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))),      cos(q(0))*cos(q(1)), cos(q(4))*(sin(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) + cos(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))) + sin(q(4))*(cos(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) - 1.0*sin(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))),
                                                                                                                                                                                                                            -1.0*sin(q(2) + q(3) + q(4))*cos(q(1)),              sin(q(1)),                                                                                                                                                                                                                             cos(q(2) + q(3) + q(4))*cos(q(1));
  R_foot_R << - 1.0*sin(q(9))*(cos(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + sin(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))) - cos(q(9))*(1.0*sin(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) - cos(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))), -1.0*cos(q(6))*sin(q(5)), cos(q(9))*(cos(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + sin(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))) - sin(q(9))*(1.0*sin(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) - cos(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))),
              cos(q(9))*(cos(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) - 1.0*sin(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))) - 1.0*sin(q(9))*(sin(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + cos(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))),      cos(q(5))*cos(q(6)), cos(q(9))*(sin(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + cos(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))) + sin(q(9))*(cos(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) - 1.0*sin(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))),
                                                                                                                                                                                                                            -1.0*sin(q(7) + q(8) + q(9))*cos(q(6)),              sin(q(6)),                                                                                                                                                                                                                             cos(q(7) + q(8) + q(9))*cos(q(6));
  
  // Jacobian mapping:
  Matrix<fpt, 3, 5> Jv_L;
  Matrix<fpt, 3, 5> Jv_R;
  Matrix<fpt, 3, 5> Jw_L;
  Matrix<fpt, 3, 5> Jw_R;

  Jv_L = leg_Jv (rs.R, q(0), q(1), q(2), q(3), q(4), 0);
  Jv_R = leg_Jv (rs.R, q(5), q(6), q(7), q(8), q(9), 1);
  Jw_L = leg_Jw (rs.R, q(0), q(1), q(2), q(3), q(4), 0);
  Jw_R = leg_Jw (rs.R, q(5), q(6), q(7), q(8), q(9), 1);

  // std::cout << "Jv_L: " << Jv_L << std::endl;
  // std::cout << "rs.R: " << rs.R << std::endl;
  // std::cout << "R_foot_L: " << R_foot_L << std::endl;
  // std::cout << "R_foot_R: " << R_foot_R << std::endl;

  // cout << "q1: " << q1 << endl;

#ifdef K_PRINT_EVERYTHING
  cout << "Initial state: \n"
       << x_0 << endl;
  cout << "World Inertia: \n"
       << I_world << endl;
  cout << "A CT: \n"
       << A_ct << endl;
  cout << "B CT (simplified): \n"
       << B_ct_r << endl;
#endif
  // QP matrices
  c2qp(A_ct, B_ct_r, setup->dt, setup->horizon);

  // weights
  Matrix<fpt, 13, 1> full_weight;
  for (u8 i = 0; i < 12; i++)
    full_weight(i) = update->weights[i];
  full_weight(12) = 0.f;
  S.diagonal() = full_weight.replicate(setup->horizon, 1);

  // trajectory
  for (s16 i = 0; i < setup->horizon; i++)
  {
    for (s16 j = 0; j < 12; j++)
      X_d(13 * i + j, 0) = update->traj[12 * i + j];
  }
  // cout<<"XD:\n"<<X_d<<endl;

  double motorTorqueLimit = 33.5;
  // QP Upper Bound
  for(s16 i = 0; i < setup->horizon; i++){
      U_b(0 + 26*i) = BIG_NUMBER;
      U_b(1 + 26*i) = BIG_NUMBER;
      U_b(2 + 26*i) = BIG_NUMBER;
      U_b(3 + 26*i) = BIG_NUMBER;

      U_b(4 + 26*i) = 0.01;
      U_b(5 + 26*i) = BIG_NUMBER;      
      U_b(6 + 26*i) = BIG_NUMBER;

      U_b(7 + 26*i) = motorTorqueLimit;
      U_b(8 + 26*i) = motorTorqueLimit;
      U_b(9 + 26*i) = motorTorqueLimit;
      U_b(10 + 26*i) = motorTorqueLimit;      
      U_b(11 + 26*i) = motorTorqueLimit;

      U_b(12 + 26*i) = setup->f_max * update->gait[2*i + 0] ;
      // 
      U_b(13 + 26*i) = BIG_NUMBER;
      U_b(14+ 26*i) = BIG_NUMBER;
      U_b(15 + 26*i) = BIG_NUMBER;
      U_b(16 + 26*i) = BIG_NUMBER;
      
      U_b(17 + 26*i) = 0.01;
      U_b(18 + 26*i) = BIG_NUMBER;      
      U_b(19 + 26*i) = BIG_NUMBER;

      U_b(20 + 26*i) = motorTorqueLimit;
      U_b(21 + 26*i) = motorTorqueLimit;
      U_b(22 + 26*i) = motorTorqueLimit;
      U_b(23 + 26*i) = motorTorqueLimit;      
      U_b(24 + 26*i) = motorTorqueLimit;

      U_b(25 + 26*i) = setup->f_max * update->gait[2*i + 1];
  }

  // QP Lower Bound
  for(s16 i = 0; i < setup->horizon; i++){
      L_b(0 + 26*i) = 0.0f;
      L_b(1 + 26*i) = 0.0f;
      L_b(2 + 26*i) = 0.0f;
      L_b(3 + 26*i) = 0.0f;

      L_b(4 + 26*i) = 0.0f;
      L_b(5 + 26*i) = 0.0f;      
      L_b(6 + 26*i) = 0.0f;

      L_b(7 + 26*i) = -motorTorqueLimit;
      L_b(8 + 26*i) = -motorTorqueLimit;
      L_b(9 + 26*i) = -motorTorqueLimit;
      L_b(10 + 26*i) = -motorTorqueLimit;      
      L_b(11 + 26*i) = -motorTorqueLimit;

      L_b(12 + 26*i) = 0.0f ;
      // 
      L_b(13 + 26*i) = 0.0f;
      L_b(14 + 26*i) = 0.0f;
      L_b(15 + 26*i) = 0.0f;
      L_b(16 + 26*i) = 0.0f;
      
      L_b(17 + 26*i) = 0.0f;
      L_b(18 + 26*i) = 0.0f;      
      L_b(19 + 26*i) = 0.0f;

      L_b(20 + 26*i) = -motorTorqueLimit;
      L_b(21 + 26*i) = -motorTorqueLimit;
      L_b(22 + 26*i) = -motorTorqueLimit;
      L_b(23 + 26*i) = -motorTorqueLimit;      
      L_b(24 + 26*i) = -motorTorqueLimit;

      L_b(25 + 26*i) = 0.0f;
  }

  // std::cout << "U_b =" << U_b << "\n";

  // QP Lower Bound


  // Initalization of Line Contact Constraint Parameters
  // fpt mu = 1.f/setup->mu;
  fpt mu = 5.0;
  fpt lt = 0.09;
  fpt lh = 0.06;

  // Matrix<fpt,5,3> f_block;
  Matrix<fpt,10,12> f_blockz;
  Matrix<fpt,26,12> F_control;

  Matrix<fpt,1,3> lt_vec;
  Matrix<fpt,1,3> lt_3D;
  lt_vec << 0, 0, lt;

  Matrix<fpt,1,3> lh_vec;
  Matrix<fpt,1,3> lh_3D;
  lh_vec << 0, 0, lh;

  Matrix<fpt,1,3> M_vec;
  M_vec << 1.0, 0, 0;
  Matrix<fpt,1,3> M_3D;

  Matrix<fpt,1,3> Moment_selection(1.f, 0, 0);
  Matrix<fpt,1,3> Moment_selection_3D;

  F_control.setZero();
//leg 1
  F_control.block<1, 12>(0, 0) //Friction leg 1
      << -mu, 0, 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(1, 0)
      << mu, 0, 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(2, 0)
      << 0, -mu, 1.f,  0, 0, 0, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(3, 0)
      << 0,  mu, 1.f,  0, 0, 0, 0, 0, 0, 0, 0, 0;        
  F_control.block<1, 12>(4, 0) //Mx Leg 1
      << 0, 0, 0, 0, 0, 0, Moment_selection * R_foot_L.transpose()* rs.R.transpose(),  0, 0, 0;
  F_control.block<5, 3>(5, 0) 
      << Jv_L.transpose() * rs.R.transpose();
  F_control.block<5, 3>(5, 6) 
      << Jw_L.transpose() * rs.R.transpose();    
  F_control.block<1, 12>(10, 0) //Line Leg 1
      << lt_vec * R_foot_L.transpose()* rs.R.transpose(),   0, 0, 0,  M_vec * R_foot_L.transpose()* rs.R.transpose(),  0, 0, 0;
  F_control.block<1, 12>(11, 0)
      <<  lh_vec * R_foot_L.transpose()* rs.R.transpose(),   0, 0, 0, -M_vec * R_foot_L.transpose()* rs.R.transpose(),  0, 0, 0;
  F_control.block<1, 12>(12, 0) //Fz Leg 1
      << 0, 0, 2.f, 0, 0, 0,   0, 0, 0, 0, 0, 0;

  

//leg 2
  F_control.block<1, 12>(13, 0) //Friction leg 2
      <<  0, 0, 0,   -mu, 0, 1.f, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(14, 0)
      <<  0, 0, 0,    mu, 0, 1.f, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(15, 0)
      <<   0, 0, 0, 0, -mu, 1.f, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(16, 0)
      <<   0, 0, 0, 0, mu, 1.f, 0, 0, 0, 0, 0, 0;        
  F_control.block<1, 12>(17, 0) //Mx Leg 1
      << 0, 0, 0, 0, 0, 0, 0, 0, 0, Moment_selection * R_foot_R.transpose()* rs.R.transpose();
  F_control.block<5, 3>(5, 3) 
      << Jv_R.transpose() * rs.R.transpose();
  F_control.block<5, 3>(5, 9) 
      << Jw_R.transpose() * rs.R.transpose();  
  F_control.block<1, 12>(23, 0) //Line Leg 2
      << 0, 0, 0,     M_vec * R_foot_R.transpose()* rs.R.transpose(), 0, 0, 0,    lt_vec * R_foot_R.transpose()* rs.R.transpose();
  F_control.block<1, 12>(24, 0)
      << 0, 0, 0,    -M_vec * R_foot_R.transpose()* rs.R.transpose(),  0, 0, 0,   lh_vec * R_foot_R.transpose()* rs.R.transpose();  
  F_control.block<1, 12>(25, 0)  //Fz Leg 2
      << 0, 0, 0, 0, 0, 2.f, 0, 0, 0, 0, 0, 0;


  //  std:: cout << "F_control =" << F_control << "\n";  


  // Print for debug

  // std::cout << "F_control =\n"
  //           << F_control << "\n";

  // Set to fmat QP
  for(s16 i = 0; i < setup->horizon; i++)
  {
    fmat.block(i*26,i*12,26,12) = F_control;
  }
  // Construct K:
  Alpha_diag.resize(12, 12);
  Alpha_diag.setZero();

  for (s16 i = 0; i < 12; i++)
  {
    Alpha_diag.block(i, i, 1, 1) << update->Alpha_K[i];
  }
  for (s16 i = 0; i < setup->horizon; i++)
  {
    Alpha_rep.block(i * 12, i * 12, 12, 12) << Alpha_diag;
  }
  // Equivalent to Matlab Formulaion
  qH = 2 * (B_qp.transpose() * S * B_qp + Alpha_rep);
  qg = 2 * B_qp.transpose() * S * (A_qp * x_0 - X_d);
  // std::cout << "error:" << qg << std::endl;

  // Calls function that sets parameters matrices in qpOASES types
  matrix_to_real(H_qpoases,qH,setup->horizon*12, setup->horizon*12);
  matrix_to_real(g_qpoases,qg,setup->horizon*12, 1);
  matrix_to_real(A_qpoases,fmat,setup->horizon*26, setup->horizon*12);
  matrix_to_real(ub_qpoases,U_b,setup->horizon*26, 1);
  matrix_to_real(lb_qpoases,L_b,setup->horizon*26, 1);
  // std::cout << "fmat =\n"
            // << fmat << "\n";
    // for(s16 i = 0; i < 26*setup->horizon; i++)
    //   lb_qpoases[i] = 0.0f;

  s16 num_constraints = 26*setup->horizon;
  s16 num_variables = 12*setup->horizon;

  // Max # of working set recalculations
  qpOASES::int_t nWSR = 500;

  int new_vars = num_variables;
  int new_cons = num_constraints;

  for (int i = 0; i < num_constraints; i++)
    con_elim[i] = 0;

  for (int i = 0; i < num_variables; i++)
    var_elim[i] = 0;

  for (int i = 0; i < num_constraints; i++)
  {
    if (!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i]))) continue;
      // printf("check 1\n");
      // std::cout<< i << std::endl;
      
    double *c_row = &A_qpoases[i * num_variables];
    // std::cout << "c_row: " << std::endl;
    for (int j = 0; j < num_variables; j++)
    {
      
      // std::cout << c_row[j] << std::endl;
      if (near_one(c_row[j]))
      {
          new_vars -= 6;
          new_cons -= 13;
          // // int cs = ((j)*13)/6 + 6;//j+2*i;//(j*7)/6 -6;
          // int cs = ceil((j+4)/6)*13-1;
          int cs;
          if (j%2 == 0){
            cs = (j+4)/6*13-1;
          }
          else{
            cs = (j+1)/6*13+12;
          }

          std::cout << "j = " << j << "; " << "cs: " << cs <<std::endl;
          // var_elim[j-8] = 1;
          // var_elim[j-7] = 1;
          // var_elim[j-6] = 1;
          var_elim[j+6] = 1;
          var_elim[j+5] = 1;
          var_elim[j+4] = 1;
          var_elim[j-2] = 1;
          var_elim[j-1] = 1;
          var_elim[j  ] = 1;
          
          con_elim[cs-0] = 1;
          con_elim[cs-1] = 1;
          con_elim[cs-2] = 1;
          con_elim[cs-3] = 1;
          con_elim[cs-4] = 1;         
          con_elim[cs-5] = 1;
          con_elim[cs-6] = 1;
          con_elim[cs-7] = 1;
          con_elim[cs-8] = 1;
          con_elim[cs-9] = 1;         
          con_elim[cs-10] = 1;
          con_elim[cs-11] = 1;
          con_elim[cs-12] = 1;
      }
    }
  }
  
  std::cout << "newvars" << new_vars << std::endl;
  std::cout << "newcons" << new_cons << std::endl;
  // std::cout << "var_elim" << var_elim << std::endl;
  // std::cout << "con_elim" << con_elim << std::endl;

  // if(new_vars != num_variables)
  if (1 == 1)
  {
    int var_ind[new_vars];
    int con_ind[new_cons];
    int vc = 0;
    for (int i = 0; i < num_variables; i++)
    {
      if (!var_elim[i])
      {
        if (!(vc < new_vars))
        {
          printf("BAD ERROR 1\n");
        }
        var_ind[vc] = i;
        vc++;
      }
    }
    vc = 0;
    for (int i = 0; i < num_constraints; i++)
    {
      if (!con_elim[i])
      {
        if (!(vc < new_cons))
        {
          printf("BAD ERROR 2\n");
        }
        con_ind[vc] = i;
        vc++;
      }
    }
    for (int i = 0; i < new_vars; i++)
    {
      int olda = var_ind[i];
      g_red[i] = g_qpoases[olda];
      for (int j = 0; j < new_vars; j++)
      {
        int oldb = var_ind[j];
        H_red[i * new_vars + j] = H_qpoases[olda * num_variables + oldb];
      }
    }

    for (int con = 0; con < new_cons; con++)
    {
      for (int st = 0; st < new_vars; st++)
      {
        float cval = A_qpoases[(num_variables * con_ind[con]) + var_ind[st]];
        A_red[con * new_vars + st] = cval;
      }
    }

    for (int i = 0; i < new_cons; i++)
    {
      int old = con_ind[i];
      ub_red[i] = ub_qpoases[old];
      lb_red[i] = lb_qpoases[old];
    }

    Timer solve_timer;

    // qpOASES problem
    qpOASES::QProblem problem_red(new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    // op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);
    // int_t nWSR = 50000;

    // QP initialized

    int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
    // int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
    (void)rval;
    // printf("A_red: %.3f", A_red);
    // cout << "A_qpoases:" << &A_qpoases << std::endl;
    // std::cout << "lb_red:" << *lb_red << std::endl;
    // std::cout << "ub_red:" << *ub_red << std::endl;
    // // Stores Solution into q_red
    int rval2 = problem_red.getPrimalSolution(q_red);

    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
      printf("failed to solve!\n");

    printf("solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);

    // Reformats solution and stores into q_red
    vc = 0;
    for (int i = 0; i < num_variables; i++)
    {
      if (var_elim[i])
      {
        q_soln[i] = 0.0f;
      }
      else
      {
        q_soln[i] = q_red[vc];
        vc++;
      }
    }
  }

#ifdef K_PRINT_EVERYTHING
  cout<<"fmat:\n"<<fmat<<endl;
#endif
}