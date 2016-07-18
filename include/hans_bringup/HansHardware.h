/*
 * HansHardware.h
 *
 *  Created on: Jul 18, 2016
 *      Author: agv
 */

#ifndef INCLUDE_HANS_BRINGUP_HANSHARDWARE_H_
#define INCLUDE_HANS_BRINGUP_HANSHARDWARE_H_


class AGVHardware {
public:
	AGVHardware();
	~AGVHardware();
	double WheelSpdGetRadPS(const double *RPM);
	double WheelSpdGetRPM(const double *RadPS);
	void ComputeOdom(const AgvVelocity *WheelSpd, float LastTime, AgvState *PreLocat);

private:

};




#endif /* INCLUDE_HANS_BRINGUP_HANSHARDWARE_H_ */
