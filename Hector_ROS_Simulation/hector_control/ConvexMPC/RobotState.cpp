#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt yaw_)
{
    for(u8 i = 0; i < 3; i++)
    {
        this->p(i) = p_[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = yaw;

    //for(u8 i = 0; i < 12; i++)
    //    this->r_feet(i) = r[i];
    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 2; c++){
        // std::cout<< "r_ : " << rs << ", "<< c << ": " << r_[rs*2 + c] <<std::endl;
        this->r_feet(rs,c) = r_[rs*2 + c];
        }   
    R = this->q.toRotationMatrix();
    fpt yc = cos(yaw_);
    fpt ys = sin(yaw_);

    R_yaw <<  yc,  -ys,   0,
             ys,  yc,   0,
               0,   0,   1;

    
    // R_yaw = R;     
    // R = R_yaw;     
    Matrix<fpt,3,1> Id;

    // Id << 0.064f, 0.057f, 0.016f; //trunk
    // Id << 0.2351, 0.2230, 0.0323; // adding hips/
    Id << 0.5413, 0.5200, 0.0691; //adding thighs
    
    I_body.diagonal() = Id;
    // I_body << 0.5435, 0.0, -0.0321,
            //   0.0,  0.5200, 0.00,
            //   -0.0321, 0.0, 0.0713;

    //TODO: Consider normalizing quaternion??
}

void RobotState::print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<p.transpose()
       <<"\nVelocity\n"<<v.transpose()<<"\nAngular Veloctiy\n"
       <<w.transpose()<<"\nRotation\n"<<R<<"\nYaw Rotation\n"
       <<R_yaw<<"\nFoot Locations\n"<<r_feet<<"\nInertia\n"<<I_body<<endl;
}



