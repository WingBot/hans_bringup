/*
 * HansHardware.h
 *
 *  Created on: Jul 18, 2016
 *      Author: Paul&szr@giimagv
 */

#ifndef INCLUDE_HANS_BRINGUP_HANSHARDWARE_H_
#define INCLUDE_HANS_BRINGUP_HANSHARDWARE_H_
#include "hans_bringup/data_definition.h"


namespace AGVHardware {

	//double WheelSpdGetRadPS(const double &RPM);
	//double WheelSpdGetRPM(const double &RadPS);

	void ComputeOdom(const AGVSENSORS *AgvData, const char LastTime, OdomPackage &OdomMsg);
    void DriveCommand(const int LeftWheelSpeed, const int RightWheelSpeed, ControlPackge &CmdPackage);
    ControlPackge StopCommand(void);

};

#endif /* INCLUDE_HANS_BRINGUP_HANSHARDWARE_H_ */
