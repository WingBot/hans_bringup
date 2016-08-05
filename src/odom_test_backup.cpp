/*
/*
 * Odom_test.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: agv
 */

#include <sys/time.h>
#include <sys/select.h>
#include <time.h>
#include <stdio.h>


#include "ros/ros.h"
#include "cstring"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "hans_bringup/BoostTCPClient.h"
#include "hans_bringup/AGVCmdFunction.h"
#include "hans_bringup/ros_function.h"
#include "boost/thread.hpp"


const int TEST_LEFT_PWM = 400;
const int TEST_RIGHT_PWM = -400;
const int READ_TIMEOUT = 25;//单位：ms 40HZ
const int TIMEOUT = 50;//单位：ms
//const int PUB_TIMEOUT = 50;
boost::asio::io_service io_;
tcpclient client(io_);

extern ros::Publisher odom_pub;
//extern boost::mutex mutex;

void CmdVelCallback(const geometry_msgs::TwistConstPtr &Vel)
{

	ControlPackge cmd_test;
	int PWMMotor_left,PWMMotor_right;
	//将ros话题线速度转化为电机的PWM,存于PWMMotor_left,PWMMotor_right;
	Vel2MotorPWM(Vel,PWMMotor_left,PWMMotor_right);
	//ROS_INFO(" send LeftWheelRPM  = %d", PWMMotor_left);
    //ROS_INFO(" send RightWheelRPM  = %d", PWMMotor_right);
    //FillCommand(TEST_LEFT_PWM,TEST_RIGHT_PWM,cmd_test);
    //将转换后的电机转速指令填充到指令包
	FillCommand(PWMMotor_left,PWMMotor_right,cmd_test);
	//std::cout<<"write"<<std::endl;
	client.write_handle(cmd_test,TIMEOUT);
}
/*
seconds: the seconds; mseconds: the micro seconds
void setTimer(int seconds, int mseconds)
{
        struct timeval temp;

        temp.tv_sec = seconds;
        temp.tv_usec = mseconds;

        select(0, NULL, NULL, NULL, &temp);
        printf("timeout\n");

        return ;
}
*/


/*
void updateodom(ODOM &odom_msg,nav_msgs::Odometry &Odometry_msg_)
{	for (;;) {

	 //boost::lock_guard<boost::mutex>  lock(mutex);
		mutex.lock();
	Odometry_msg_.header.stamp = ros::Time::now();
	Odometry_msg_.header.frame_id = "odom";
	Odometry_msg_.child_frame_id = "base_link";

	Odometry_msg_.pose.pose.position.x = odom_msg.Pose_X;
	Odometry_msg_.pose.pose.position.y = odom_msg.Pose_Y;

	Odometry_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_msg.Pose_Theta);

      Odometry_msg_.pose.covariance[0]  =
	  Odometry_msg_.pose.covariance[7]  =
	  Odometry_msg_.pose.covariance[14] =
	  Odometry_msg_.pose.covariance[21] =
	  Odometry_msg_.pose.covariance[28] =
	  Odometry_msg_.pose.covariance[35] = 0.031;

	  Odometry_msg_.twist.twist.linear.x  =  odom_msg.Vx;
	  Odometry_msg_.twist.twist.linear.y  =  0.0;
	  Odometry_msg_.twist.twist.linear.z  =  0.0;
	  Odometry_msg_.twist.twist.angular.x =  0.0;
	  Odometry_msg_.twist.twist.angular.y =  0.0;
	  Odometry_msg_.twist.twist.angular.z =  odom_msg.Vyaw;

	  Odometry_msg_.twist.covariance[0]  =
	  Odometry_msg_.twist.covariance[7]  =
	  Odometry_msg_.twist.covariance[14] =
	  Odometry_msg_.twist.covariance[21] =
	  Odometry_msg_.twist.covariance[28] =
	  Odometry_msg_.twist.covariance[35] = 999;

		geometry_msgs::TransformStamped Transform_msg_;
		Transform_msg_.header.stamp    = ros::Time::now();
		  Transform_msg_.header.frame_id = "odom";
		  Transform_msg_.child_frame_id  = "base_link";

		  Transform_msg_.transform.translation.x = odom_msg.Pose_X;
		  Transform_msg_.transform.translation.y =odom_msg.Pose_Y;
		  Transform_msg_.transform.translation.z = 0.0;
		  Transform_msg_.transform.rotation      = tf::createQuaternionMsgFromYaw(odom_msg.Pose_Theta);
		  mutex.unlock();
		  tf::TransformBroadcaster odom_broadcaster;
		    odom_broadcaster.sendTransform(Transform_msg_);
		    odom_pub.publish(Odometry_msg_);
		    setTimer(0, 50000);


}
}
*/


int main(int argc, char **argv) {
/*	try {*/
		ros::init(argc, argv, "BoostTcpTestNode");
		ros::NodeHandle n;
		tcp::endpoint endpoint_(boost::asio::ip::address::from_string("192.168.1.33"),2000);
		client.connect(endpoint_);

		ros::Subscriber VelSub = n.subscribe("/turtlebot_teleop/cmd_vel", 100, CmdVelCallback);
		//ros::topic::waitForMessage<geometry_msgs::Twist>("/turtlebot_teleop/cmd_vel");
		/*std::cout<<"waiting..."<<std::endl;
		boost::asio::io_service io_;
		tcp::endpoint endpoint_(boost::asio::ip::address::from_string("192.168.1.33"),2000);
		//tcp::endpoint endpoint_(boost::asio::ip::address::from_string("127.0.0.1"),2000);
		tcpclient client(io_);
		client.connect(endpoint_);
		//bool ReadData = false;
		std::cout<<"connecting..."<<std::endl;
		//std::cout<<"read_handle"<<std::endl;
		//bool OdometryInitialized = false;

		//client.read_handle(TIMEOUT);
		//std::cout<<"fill"<<std::endl;
		//ControlPackge cmd_test;
		//FillCommand(TEST_LEFT_PWM,TEST_RIGHT_PWM,cmd_test);
		//std::cout<<"write"<<std::endl;
		//client.write_handle(cmd_test,TIMEOUT);

		//boost::thread thread2(boost::bind(&tcpclient::write_handle,&client,cmd_test,TIMEOUT));


	   // thread2.join();
	   */

/*	} catch (std::exception &e) {
		std::cerr<<e.what()<<std::endl;
	}*/
		ODOM OdomMsg;
		nav_msgs::Odometry Odometry_msg_;
		//OdomInitial(OdomMsg,Odometry_msg_);

		boost::thread thread1(boost::bind(&tcpclient::read_handle,&client,READ_TIMEOUT,OdomMsg,Odometry_msg_));
		//boost::thread thread2(boost::bind(&updateodom,OdomMsg,Odometry_msg_));
		odom_pub = n.advertise<nav_msgs::Odometry>("/hans_odom", 2);
	  // m_OdometryPublisher = m_NodeHandle.advertise<nav_msgs::Odometry>(odometryTopic, 1);
		// thread1.join();
	ros::Rate loopRate(8);
		while(n.ok())
		{
			ros::spinOnce();              	 // check for incoming messages
			loopRate.sleep();
		}
	return 0;
}



 * odom_test_backup.cpp
 *
 *  Created on: Aug 4, 2016
 *      Author: agv
 */




