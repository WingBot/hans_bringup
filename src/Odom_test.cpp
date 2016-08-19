/*
 * Odom_test.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: szr@giimagv
 */
#include "ros/ros.h"
#include "cstring"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "hans_bringup/BoostTCPClient.h"
#include "hans_bringup/AGVCmdFunction.h"
#include "hans_bringup/ros_function.h"
#include "boost/thread.hpp"
#include "hans_bringup/EncoderVel.h"

const int READ_TIMEOUT = 50;//unit：ms HZ
const int WRITE_TIMEOUT = 50;//unit：ms

boost::asio::io_service io_;
tcpclient client(io_);
extern ros::Publisher odom_pub;

void CmdVelCallback(const geometry_msgs::TwistConstPtr &Vel)
{
	ControlPackge cmd_test;
	int PWMMotor_left,PWMMotor_right;
	Vel2MotorPWM(Vel,PWMMotor_left,PWMMotor_right);
	FillCommand(PWMMotor_left,PWMMotor_right,cmd_test);
	client.write_handle(cmd_test,WRITE_TIMEOUT);
}


int main(int argc, char **argv) {
	try {
		ros::init(argc, argv, "Hans_base");
		ros::NodeHandle n;
		tcp::endpoint endpoint_(boost::asio::ip::address::from_string("192.168.1.32"),2000);
		client.connect(endpoint_);

		ros::Subscriber VelSub = n.subscribe("/cmd_vel", 100, CmdVelCallback);

		ODOM OdomMsg;
		nav_msgs::Odometry Odometry_msg_;
		OdomInitial(OdomMsg,Odometry_msg_);
		boost::thread thread1(boost::bind(&tcpclient::read_handle,&client,READ_TIMEOUT,OdomMsg,Odometry_msg_));
		odom_pub = n.advertise<nav_msgs::Odometry>("/hans_odom", 1000);
		ros::Rate loopRate(8);
		while(n.ok())
			{
				ros::spinOnce();              	 // check for incoming messages
				loopRate.sleep();
			}
		} catch (std::exception &e) {
				std::cerr<<e.what()<<std::endl;
		}
	return 0;
}



