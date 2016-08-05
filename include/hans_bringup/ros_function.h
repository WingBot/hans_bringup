/*
 * ros_function.h
 *
 *  Created on: Jul 23, 2016
 *      Author: agv
 */

#ifndef INCLUDE_ROS_FUNCTION_H_
#define INCLUDE_ROS_FUNCTION_H_
#include <ros/ros.h>
#include "AGVCmdFunction.h"

bool Vel2MotorPWM(const geometry_msgs::TwistConstPtr &Vel,int &PWM_ml,int &PWM_mr)
{
	 //W左轮角速度　＝V车/ R轮半径－（W车＊Ｂ车轮距）／（２＊Ｒ轮半径）　　rad/s　　(Vel->linear.x / WheelR - (WheelBase * Vel->angular.z)/(2*WheelR))
	 //W右轮角速度　＝V车/ R轮半径＋（W车＊Ｂ车轮距）／（２＊Ｒ轮半径）　　rad/s
	//W电机角速度　＝　W轮角速度*30  rad/s
	// N = W * 30 / PI  rad/s  ==> 转/min
	PWM_ml = (int)(    (  Vel->linear.x / WheelR - (WheelBase * Vel->angular.z)/(2*WheelR)   )   * 30 * 30 / PI );
	PWM_mr = (int)(   (   Vel->linear.x / WheelR + (WheelBase * Vel->angular.z)/(2*WheelR)  )  * 30 * 30 / PI );
     std::cout<<"PWM_ML"<<PWM_ml<<std::endl;
}




#endif /* INCLUDE_ROS_FUNCTION_H_ */
