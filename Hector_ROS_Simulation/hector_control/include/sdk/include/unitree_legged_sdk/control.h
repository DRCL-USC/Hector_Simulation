/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _UNITREE_LEGGED_CONTROL_H_
#define _UNITREE_LEGGED_CONTROL_H_

#include "comm.h"
#include "quadruped.h"

namespace UNITREE_LEGGED_SDK 
{

class Control{
public:
	Control(LeggedType type, uint8_t level);
	~Control();
	void InitCmdData(HighCmd& cmd);
	void InitCmdData(LowCmd& cmd);
	void SetBandWidth(int hz);
	void PositionLimit(LowCmd&);     	            // only effect under Low Level control in Position mode
	void PowerProtect(LowCmd&, LowState&, int);   /* only effect under Low Level control, input factor: 1~10, 
												means 10%~100% power limit. If you are new, then use 1; if you are familiar, 
												then can try bigger number or even comment this function. */
	void PositionProtect(LowCmd&, LowState&, double limit = 0.087);  // default limit is 5 degree
private:
	int BandWidth;        // 1kHz, for transmit
	int WattLimit, Wcount;     // Watt. When limit to 100, you can triger it with 4 hands shaking.
	double Hip_max, Hip_min, Thigh_max, Thigh_min, Calf_max, Calf_min;
};


}

#endif
