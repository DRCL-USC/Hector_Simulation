/*
MIT License

Copyright (c) 2019 MIT Biomimetics Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include "cppTypes.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"
#include "Biped.h"


/*!
 * Data sent from control algorithm to legs
 */ 
    struct LegControllerCommand{
        LegControllerCommand() {zero();}

        void zero();

        Vec5<double> qDes, qdDes, tau;
        Vec3<double> pDes, vDes;
        Mat5<double> kpJoint, kdJoint;
        Vec6<double> feedforwardForce;
        Vec3<double> hiptoeforce;
        Mat3<double> kpCartesian;
        Mat3<double> kdCartesian;
        double kptoe;
        double kdtoe;
    };

/*!
 * Data returned from legs to control code
 */ 
    struct LegControllerData{
        LegControllerData() {zero();}
        void setBiped(Biped& biped) { hector = &biped; }

        void zero();
        Vec5<double> q, qd;
        Vec3<double> p, v;
        Mat65<double> J_force_moment;
        Mat35<double> J_force;
        Vec5<double> tau;
        Biped* hector;
    };

/*!
 * Controller for 2 legs of hector
 */ 
    class LegController {
      public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        LegController(Biped& biped) : _biped(biped) {
            for (auto& dat : data) dat.setBiped(_biped);
            for(int i = 0; i < 2; i++){
                commands[i].zero();
                data[i].zero();
            }
        };
        
        void zeroCommand();
        void edampCommand(double gain);
        void updateData(const LowlevelState* state);
        void updateCommand(LowlevelCmd* cmd);
        void setEnabled(bool enabled) {_legsEnabled = enabled;};

        LegControllerCommand commands[2];
        LegControllerData data[2];
        bool _legsEnabled = false;
        std::string limbName[5] = {"Hip 1", "Hip 2", "Thigh", "Knee ", "Toe  "};
        std::string Side[2] = {"Left ", "Right"};        
        Biped& _biped;
    };

    void computeLegJacobianAndPosition(Biped& _biped, Vec5<double>& q, Mat65<double>* J_f_m, Mat35<double>* J_f, 
                                       Vec3<double>* p, int leg);

#endif