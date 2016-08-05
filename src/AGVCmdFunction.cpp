/*
 * AGVCmdFunction.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: agv
 */

#include "hans_bringup/AGVCmdFunction.h"
#include "hans_bringup/BoostTCPClient.h"
#include <boost/thread.hpp>
boost::mutex mutex;
ros::Publisher odom_pub;
bool FillCommand(const int LeftWheelPWM, const int RightWheelPWM, ControlPackge & cmdPackage)
      {
           U_int32_t LeftWheel, RightWheel;

           LeftWheel.value = LeftWheelPWM;
           RightWheel.value = RightWheelPWM;

           cmdPackage.cmd.WheelLeft.bit[0] = LeftWheel.bit[0];
           cmdPackage.cmd.WheelLeft.bit[1] = LeftWheel.bit[1];
           cmdPackage.cmd.WheelLeft.bit[2] = LeftWheel.bit[2];
           cmdPackage.cmd.WheelLeft.bit[3] = LeftWheel.bit[3];
           cmdPackage.cmd.WheelRight.bit[0] = RightWheel.bit[0];
           cmdPackage.cmd.WheelRight.bit[1] = RightWheel.bit[1];
           cmdPackage.cmd.WheelRight.bit[2] = RightWheel.bit[2];
           cmdPackage.cmd.WheelRight.bit[3] = RightWheel.bit[3];
           //std::cout<<"cmd_package_Pluse:"<<cmdPackage.cmd.WheelLeft.value<<std::endl;
          return true;
      }

bool FillStaticFeedback(HANsFeedbackData &agv)
{
	agv.Data.FrameDataHeader.FrameHeader.bit[0] = 0xAA;
	agv.Data.FrameDataHeader.FrameHeader.bit[1] = 0x55;
	agv.Data.FrameDataHeader.DatasLength = 0x1E;
	agv.Data.SensorsData.MsgHeader = 0x01;
	agv.Data.SensorsData.MsgLength = 0x13;
	agv.Data.SensorsData.Bumper = 0x00;
	agv.Data.SensorsData.WheelMissed = 0x00;
	agv.Data.SensorsData.TouchGround = 0x00;
	agv.Data.SensorsData.PWMLeft = 0x00;
	agv.Data.SensorsData.PWMRight = 0x00;
	agv.Data.SensorsData.HANBattery = 0x00;
	agv.Data.SensorsData.HANButton = 0x00;
	agv.Data.SensorsData.HANCharger = 0x00;
	agv.Data.SensorsData.CurrentBeyond = 0x00;
	agv.Data.Gyromsg.MsgHeader = 0x04;
	agv.Data.Gyromsg.MsgLength = 0x07;
	agv.Data.Gyromsg.NULL1 = 0x00;
	agv.Data.Gyromsg.NULL2 = 0x00;
	agv.Data.Gyromsg.NULL3 = 0x00;
	agv.Data.checksum = 0x00;
	return true;
	}



bool GetDeltaTimeSec(U_uint16_t &time_temp, U_uint16_t &time_new,float_t &dt_sec)
{
	uint16_t dt_raw;
	if(time_new.value > time_temp.value )
		dt_raw = time_new.value - time_temp.value;
	else
		dt_raw = time_new.value + (TIME_RANGE - time_temp.value);
	dt_sec = dt_raw * 2 / 1000.0;//time counter to second
	//std::cout<<"2*dt_raw:"<<std::fixed<<std::setprecision(9)<<(2*dt_raw)\
						<<"\tdt_sec:"<<std::fixed<<std::setprecision(9)<<dt_sec<<std::endl;
	return true;
}

bool GetDeltaPluse(U_int32_t &PWM_new,U_int32_t &PWM,int32_t &dpluse,int &timeout)
{
	int32_t dpluse_max = MOTOR_MAX_PWM * timeout;
	if ( (PWM_new.value - PWM.value)> dpluse_max) {
		dpluse =  (PWM_new.value - PWM.value) - PLUSE_RANGE;
		}
	else if ((PWM_new.value - PWM.value) < (-dpluse_max)) {
		dpluse = (PWM_new.value - PWM.value) + PLUSE_RANGE;
	}
	else {
		dpluse = (PWM_new.value - PWM.value);
	}
	return true;
}

bool TransPoseThetaRange(ODOM &odom_msg,float_t dPassHeading)
{
	//float_t PreHeading = odom_msg.Pose_Theta;
	float_t Pose_Theta_temp = odom_msg.Pose_Theta + dPassHeading;
	if (Pose_Theta_temp > PI) {
		odom_msg.Pose_Theta = Pose_Theta_temp - ((Pose_Theta_temp + PI) / (2*PI)) *2*PI;
	}
	else if (Pose_Theta_temp < -PI) {
		odom_msg.Pose_Theta = Pose_Theta_temp + ((Pose_Theta_temp - PI) / (-2*PI)) *2*PI;
	}
	else {
		odom_msg.Pose_Theta = Pose_Theta_temp;
	}
	return true;
}


void FillCmd2Buff(char* write_buff,ControlPackge &Buffer)
	{
		write_buff[0] = Buffer.FrameDataHeader.FrameHeader.bit[0];
		write_buff[1] = Buffer.FrameDataHeader.FrameHeader.bit[1];
		write_buff[2] = Buffer.FrameDataHeader.DatasLength;
		write_buff[3] = Buffer.cmd.DataHeader;
		write_buff[4] = Buffer.cmd.DataLenght;
		write_buff[5] = Buffer.cmd.WheelLeft.bit[0];
		write_buff[6] = Buffer.cmd.WheelLeft.bit[1];
		write_buff[7] = Buffer.cmd.WheelLeft.bit[2];
		write_buff[8] = Buffer.cmd.WheelLeft.bit[3];
		write_buff[9] = Buffer.cmd.WheelRight.bit[0];
		write_buff[10] = Buffer.cmd.WheelRight.bit[1];
		write_buff[11] = Buffer.cmd.WheelRight.bit[2];
		write_buff[12] = Buffer.cmd.WheelRight.bit[3];
		write_buff[13] = Buffer.checksum;
	}

void AppendCmd2Buff(VBytes &send_buf,ControlPackge &cmdpack)
{
	send_buf.resize(sizeof(ControlPackge));
	send_buf.push_back(cmdpack.FrameDataHeader.FrameHeader.bit[0]);
	send_buf.push_back(cmdpack.FrameDataHeader.FrameHeader.bit[1]);
	send_buf.push_back(cmdpack.FrameDataHeader.DatasLength);
	send_buf.push_back(cmdpack.cmd.DataHeader);
	send_buf.push_back(cmdpack.cmd.DataLenght);
	send_buf.push_back(cmdpack.cmd.WheelLeft.bit[0]);
	send_buf.push_back(cmdpack.cmd.WheelLeft.bit[1]);
	send_buf.push_back(cmdpack.cmd.WheelLeft.bit[2]);
	send_buf.push_back(cmdpack.cmd.WheelLeft.bit[3]);
	send_buf.push_back(cmdpack.cmd.WheelRight.bit[0]);
	send_buf.push_back(cmdpack.cmd.WheelRight.bit[1]);
	send_buf.push_back(cmdpack.cmd.WheelRight.bit[2]);
	send_buf.push_back(cmdpack.cmd.WheelRight.bit[3]);
	send_buf.push_back(cmdpack.checksum);
}

void ComputeOdom(int32_t dpluse_l ,int32_t dpluse_r, float_t dt_sec, ODOM &odom_msg,nav_msgs::Odometry &Odometry_msg_)
{
	/*  if (OdometryInitialized)
	  {*/
	float_t dPassPath_L = Cm * dpluse_l;
	//float_t dPassPath_one = Cm * 900.0;
	//std::cout<<"Cm:"<<std::showpoint<<dPassPath_one;
	//std::cout<<"\tdpath_l:"<<std::showpoint<<dPassPath_L;
	float_t dPassPath_R = Cm * dpluse_r;
	//std::cout<<"\tdpath_r:"<<std::showpoint<<dPassPath_R;
	float_t dPassPath = (dPassPath_R + dPassPath_L) / 2.0;
	//std::cout<<"dPassPath:"<<std::showpoint<<dPassPath;
	//float_t dwPassPath = Cm*(dpluse_r - dpluse_l);
	//PassPath = PadPassPath;
	//std::cout<<"\tdwPassPath:"<<std::showpoint<<dwPassPath;

	//对直线进行修正
	float_t dPassHeading = 0.0;

	//dPassHeading = dPassheadingTemp;

	float_t dPassheadingTemp = Cm * (dpluse_r - dpluse_l) / WheelBase;
	if ((dPassheadingTemp >= -0.00081 && dPassheadingTemp <= 0.00081)) {
		dPassHeading = 0.0;
	}
	else {
		dPassHeading = dPassheadingTemp;
	}
	//float_t dPassHeading = Cm * (dpluse_r - dpluse_l) / WheelBase;
	//std::cout<<"\tdPassHeading:"<<std::showpoint<<dPassHeading;
	//float_t PreHeading = odom_msg.Pose_Theta;
	float_t AverageHeading = odom_msg.Pose_Theta +dPassHeading / 2.0;
	//std::cout<<"\tAverageHeading:"<<std::showpoint<<AverageHeading;

	//{
		//boost::lock_guard<boost::mutex> lock(mutex);
		//mutex.lock();
	odom_msg.Pose_X += dPassPath * cos(AverageHeading);

	//std::cout<<"\tcos:"<<std::fixed<<std::setprecision(9)<<cos(AverageHeading);

	//std::cout<<"\tPose_X:"<<std::showpoint<<odom_msg.Pose_X<<std::endl;
	odom_msg.Pose_Y += dPassPath *sin(AverageHeading);


	//odom_msg.Pose_Theta += dPassHeading;
	//将角度转为ROS的YAW取值区间[-PI,PI]
	TransPoseThetaRange(odom_msg,dPassHeading);
	odom_msg.Vx = dPassPath / dt_sec;
	odom_msg.Vyaw = dPassHeading / dt_sec;
//	mutex.unlock();
	//}
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
		  tf::TransformBroadcaster odom_broadcaster;
		    odom_broadcaster.sendTransform(Transform_msg_);
		    odom_pub.publish(Odometry_msg_);
	 //tf::createQuaternionMsgFromYaw(previousHeading + deltaHeading);
	/* }
	  else {
		  odom_msg.Vx = 0.0;
		  odom_msg.Vyaw = 0.0;
		  odom_msg.Pose_X = 0.0;
		  odom_msg.Pose_Y = 0.0;
		  odom_msg.Pose_Theta = 0.0;
		  OdometryInitialized = true;
	}*/
}
bool OdomInitial(ODOM &OdomMsg,nav_msgs::Odometry &Odometry_msg_){
			OdomMsg.Vx = 0.0;
			OdomMsg.Vyaw = 0.0;
			OdomMsg.Pose_X = 0.0;
			OdomMsg.Pose_Y = 0.0;
			OdomMsg.Pose_Theta = 0.0;
			Odometry_msg_.header.stamp = ros::Time::now();
			Odometry_msg_.header.frame_id = "odom";
			Odometry_msg_.child_frame_id = "base_link";

			Odometry_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

			Odometry_msg_.pose.covariance[0]  =
			  Odometry_msg_.pose.covariance[7]  =
			  Odometry_msg_.pose.covariance[14] =
			  Odometry_msg_.pose.covariance[21] =
			  Odometry_msg_.pose.covariance[28] =
			  Odometry_msg_.pose.covariance[35] = 0.031;

			  Odometry_msg_.twist.twist.linear.x  =  0.0;
			  Odometry_msg_.twist.twist.linear.y  =  0.0;
			  Odometry_msg_.twist.twist.linear.z  =  0.0;
			  Odometry_msg_.twist.twist.angular.x =  0.0;
			  Odometry_msg_.twist.twist.angular.y =  0.0;
			  Odometry_msg_.twist.twist.angular.z =  0.0;

			  Odometry_msg_.twist.covariance[0]  =
			  Odometry_msg_.twist.covariance[7]  =
			  Odometry_msg_.twist.covariance[14] =
			  Odometry_msg_.twist.covariance[21] =
			  Odometry_msg_.twist.covariance[28] =
			  Odometry_msg_.twist.covariance[35] = 999;
			return true;
}
void PWM2RAD(int32_t &PWM,float_t &RADs)
{
		RADs = PWM * 2 * PI / 60 ;
}
void RAD2PWM(float_t &RADs,int32_t &PWM)
{
	PWM = (int)(RADs * 60 / ( 2 * PI ));
}


