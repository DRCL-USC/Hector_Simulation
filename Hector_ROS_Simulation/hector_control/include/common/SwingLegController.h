#ifndef SWINGLEGCONTROLLER_H
#define SWINGLEGCONTROLLER_H
#include "../../ConvexMPC/GaitGenerator.h"
#include "../../include/common/ControlFSMData.h"
#include "../../include/common/FootSwingTrajectory.h"
#include "../../include/common/cppTypes.h"
#include "../../include/common/LegController.h"

class swingLegController {
    public:
        static constexpr int nLegs = 2;

        swingLegController() = default;
        swingLegController(ControlFSMData *data, Gait* gait, double dtSwing);
        ~swingLegController();

    private:
        Gait* gait;
        const ControlFSMData* data;
        StateEstimate seResult;

        // constants can be adjusted if needed
        const double _dt = 0.001;
        const double footHeight = 0.1;        

}; // class swingLegController

#endif // SWINGLEGCONTROLLER_H    