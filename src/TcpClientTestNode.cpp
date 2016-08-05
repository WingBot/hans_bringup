/*
 * TcpClientTestNode.cpp
 *
 *  Created on: Jul 18, 2016
 *      Author: szr@giimagv
 */


#include "ros/ros.h"
#include "cstring"
#include "geometry_msgs/Twist.h"

#include "hans_bringup/data_definition.h"
#include "hans_bringup/HansHardware.h"
#include "hans_bringup/TCPClient.h"
#include "hans_bringup/AgvPara.h"
#include "hans_bringup/GeneralFunction.h"


TcpClient AgvClient;


void CmdVelCall(const geometry_msgs::TwistConstPtr &Vel)
{
    ControlPackge DriveCmd;
	int Omega_L = (int)(Vel->linear.x / WheelR - (WheelBase * Vel->angular.z)/(2*WheelR)) * 60;	//roll/s -> roll/min.
	int Omega_R = (int)(Vel->linear.x / WheelR + (WheelBase * Vel->angular.z)/(2*WheelR)) * 60;

	AGVHardware::DriveCommand(Omega_L, Omega_R, DriveCmd);           //RPM;
	AgvClient.ClientSend(DriveCmd);

}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "AgvTcpTestNode_sr");
	ros::NodeHandle n;
	AgvClient.ClientControl("192.168.1.33", 2000);

	ros::Subscriber VelSub = n.subscribe("/cmd_vel", 100, &CmdVelCall);
	ros::topic::waitForMessage<geometry_msgs::Twist>("/cmd_vel");

	AGVDATA Buffer;

	AgvClient.ClientReceive(Buffer);

	ros::Rate loop_rate(8);
	while(n.ok())
	{
		ros::spinOnce();              	 // check for incoming messages
		loop_rate.sleep();
	}
	AgvClient.ClientCloseSockfd();
	exit(0);

}





