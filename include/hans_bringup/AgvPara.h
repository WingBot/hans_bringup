/*
 * AgvPara.h
 *
 *  Created on: Jul 18, 2016
 *      Author: agv
 */

#ifndef INCLUDE_HANS_BRINGUP_AGVPARA_H_
#define INCLUDE_HANS_BRINGUP_AGVPARA_H_

#include "hans_bringup/MathPara.h"

const float WheelR = 0.0675;		//	WheelRadius unit:mm
const float WheelRR = 0.0675; 		//	Right WheelRadius for model the odometry unit:mm
const float WheelRL = 0.0675; 		//	Left WheelRadius for model the odometry unit:mm
const float WheelBase = 0.589;		//	WheelBase unit: m
const int ReducRatio = 300;  		//	Reduction radio of reduction gear
const int PlusePR = 300;   			//	encoder para : plus/roll


const double Cm = (PI * WheelR) / (ReducRatio * PlusePR);			//脉冲数到线距离的转换因子

#endif /* INCLUDE_HANS_BRINGUP_AGVPARA_H_ */
