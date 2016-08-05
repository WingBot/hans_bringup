/*
 * AGVCmdFunction.h
 *
 *  Created on: Aug 3, 2016
 *      Author: agv
 */

#ifndef INCLUDE_HANS_BRINGUP_AGVCMDFUNCTION_H_
#define INCLUDE_HANS_BRINGUP_AGVCMDFUNCTION_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "DataType.h"
#include <math.h>
#include <iostream>
#include <iomanip>


#ifndef PI
#define PI      3.1415926535898 // 圆周率
#endif
#define PI2     6.2831853071796 // 2倍圆周率
#define PI_2    1.5707963267949 // 1/2圆周率
#define DEG2RAD  0.0174532925199 // 角度转换为弧度参数
#define RAD2DEG 57.2957795130823 // 弧度转换为角度参数

const float WheelR = 0.0675;		//	WheelRadius unit:m
const float WheelRR = 0.0675; 		//	Right WheelRadius for model the odometry unit:m
const float WheelRL = 0.0675; 		//	Left WheelRadius for model the odometry unit:m
const float WheelBase = 0.589;		//	WheelBase unit: m
const int ReducRatio = 30;  		//	Reduction radio of reduction gear
const int PlusePR = 30;   			//	encoder para : plus/roll
static const float DEG_TO_RAD = 0.0174532925;

const double Cm = (2* PI * WheelR) / (ReducRatio * PlusePR);			//脉冲数到线距离m的转换因子


bool FillCommand(const int LeftWheelPWM, const int RightWheelPWM, ControlPackge & cmdPackage);
bool FillStaticFeedback(HANsFeedbackData &agv);
bool GetDeltaTimeSec(U_uint16_t &time_temp, U_uint16_t &time_new,float_t &dt_sec);
bool GetDeltaPluse(U_int32_t &PWM_new,U_int32_t &PWM,int32_t &dpluse,int &timeout);
bool TransPoseThetaRange(ODOM &odom_msg,float_t dPassHeading);
void FillCmd2Buff(char* write_buff,ControlPackge &Buffer);
void AppendCmd2Buff(VBytes &send_buf,ControlPackge &cmdpack);
void ComputeOdom(int32_t dpluse_l ,int32_t dpluse_r, float_t dt_sec, ODOM &odom_msg,nav_msgs::Odometry &Odometry_msg_);
bool OdomInitial(ODOM &OdomMsg,nav_msgs::Odometry &Odometry_msg_);
void PWM2RAD(int32_t &PWM,float_t &RADs);
void RAD2PWM(float_t &RADs,int32_t &PWM);


#endif /* INCLUDE_HANS_BRINGUP_AGVCMDFUNCTION_H_ */
