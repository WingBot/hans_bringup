/*
 * BoostTCPClient.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: agv
 */



#include "hans_bringup/BoostTCPClient.h"

	tcpclient::tcpclient(boost::asio::io_service &io_service)
											:soc_(io_service),
											 timer_(io_service)
											 				{}
	tcpclient::~tcpclient(){}

	void tcpclient::connect(tcp::endpoint &iendpoint)
	{
		std::cout<<"connecting"<<std::endl;
		boost::system::error_code error;
		soc_.connect(iendpoint,error);
		if (error) {
			std::cout<<"reconnect."<<std::endl;
			soc_.connect(iendpoint);
		}

	}
	bool tcpclient::read_handle(int timeout,ODOM &OdomMsg,nav_msgs::Odometry &Odometry_msg_)
	{
		bool ReadData = false;
		for(;;){
	    U_uint16_t time_temp,time_new;
	    U_int32_t PWMLeft, PWMRight,PWMLeft_new, PWMRight_new;
	    U_int16_t Gyro_angle,Gyro_angleRate,Gyro_angle_new,Gyro_angleRate_new;
		VBytes read_buff_;
		read_buff_.resize(MAX_RCV_SIZE);
		boost::system::error_code error;
		size_t slen_ = soc_.read_some(boost::asio::buffer(read_buff_),error);
		if (error == boost::asio::error::eof)
			break;
		else if (error)
			throw boost::system::error_code(error);

		if(slen_ > 0 && ( slen_ % 34 == 0))
		{
			int Nfull = slen_ / 34;
			uint8_t str_begin = read_buff_[Nfull*34 -34];

			if(str_begin == 170)  //0xaa
			{
				if (!ReadData) {
				time_temp.bit[0] = read_buff_[Nfull*34 - 29];
				time_temp.bit[1] = read_buff_[Nfull*34 - 28];

				PWMLeft.bit[0] = read_buff_[Nfull*34 - 24];
				PWMLeft.bit[1] = read_buff_[Nfull*34 - 23];
				PWMLeft.bit[2] = read_buff_[Nfull*34 - 22];
				PWMLeft.bit[3] = read_buff_[Nfull*34 - 21];

				PWMRight.bit[0] = read_buff_[Nfull*34 - 20];
				PWMRight.bit[1] = read_buff_[Nfull*34 - 19];
				PWMRight.bit[2] = read_buff_[Nfull*34 - 18];
				PWMRight.bit[3] = read_buff_[Nfull*34 - 17];
				}
				time_new.bit[0] = read_buff_[Nfull*34 - 29];
				time_new.bit[1] = read_buff_[Nfull*34 - 28];
				PWMLeft_new.bit[0] = read_buff_[Nfull*34 - 24];
				PWMLeft_new.bit[1] = read_buff_[Nfull*34 - 23];
				PWMLeft_new.bit[2] = read_buff_[Nfull*34 - 22];
				PWMLeft_new.bit[3] = read_buff_[Nfull*34 - 21];

				PWMRight_new.bit[0] = read_buff_[Nfull*34 - 20];
				PWMRight_new.bit[1] = read_buff_[Nfull*34 - 19];
				PWMRight_new.bit[2] = read_buff_[Nfull*34 - 18];
				PWMRight_new.bit[3] = read_buff_[Nfull*34 - 17];
			}
			str_begin = read_buff_[Nfull*34 - 10];//陀螺仪的帧头
			if (str_begin == 4)//0x04
			{
				if (!ReadData) {
				Gyro_angle.bit[0] = read_buff_[Nfull*34 - 8];
				Gyro_angle.bit[1] = read_buff_[Nfull*34 - 7];
				Gyro_angleRate.bit[0] = read_buff_[Nfull*34 - 6];
				Gyro_angleRate.bit[1] = read_buff_[Nfull*34 - 5];
				ReadData = true;
				}
				Gyro_angle_new.bit[0] = read_buff_[Nfull*34 - 8];
				Gyro_angle_new.bit[1] = read_buff_[Nfull*34 - 7];
				Gyro_angleRate_new.bit[0] = read_buff_[Nfull*34 - 6];
				Gyro_angleRate_new.bit[1] = read_buff_[Nfull*34 - 5];
			}
			//uint16_t dt_raw;
			//时间容器溢出处理
			/*if(time_new.value > time_temp.value )
				dt_raw = time_new.value - time_temp.value;
			else
				dt_raw = time_new.value + (TIME_RANGE - time_temp.value);*/
			//float_t dt_sec = dt_raw * 2 / 1000.0;//time counter to second
			float_t dt_sec = 0.0;
			//std::cout<<"time_temp:"<<std::fixed<<std::setprecision(9)<<time_temp.value<<std::endl;
			GetDeltaTimeSec(time_temp,time_new,dt_sec);//时间容器溢出处理并转化为秒
			int32_t dpluse_l ,dpluse_r;
			//int32_t dpluse_max = MOTOR_MAX_PWM * timeout;
			//编码器容器溢出处理
			/*if ( (PWMLeft_new.value - PWMLeft.value)> dpluse_max) {
				dpluse_l =  (PWMLeft_new.value - PWMLeft.value) - PLUSE_RANGE;
				}
			else if ((PWMLeft_new.value - PWMLeft.value) < (-dpluse_max)) {
				dpluse_l = (PWMLeft_new.value - PWMLeft.value) + PLUSE_RANGE;
			}
			else {
				dpluse_l = (PWMLeft_new.value - PWMLeft.value);
			}*/
			GetDeltaPluse(PWMLeft_new,PWMLeft,dpluse_l,timeout);//编码器容器溢出处理
			GetDeltaPluse(PWMRight_new,PWMRight,dpluse_r,timeout);//编码器容器溢出处理
			/*if ( (PWMRight_new.value - PWMRight.value)> dpluse_max) {
				 dpluse_r =  (PWMRight_new.value - PWMRight.value) - PLUSE_RANGE;
				}
			else if ((PWMRight_new.value - PWMRight.value) < (-dpluse_max)) {
				 dpluse_r = (PWMRight_new.value - PWMRight.value) + PLUSE_RANGE;
			}
			else {
				 dpluse_r = (PWMRight_new.value - PWMRight.value);
			}*/

			std::cout<<"========================"<<std::endl;
			std::cout<<"recv bytes:"<<slen_\
								<<"\ttime:"<<time_temp.value\
								<<"\tx:"<<std::fixed<<std::setprecision(5)<<OdomMsg.Pose_X\
								<<"\ty:"<<std::fixed<<std::setprecision(5)<<OdomMsg.Pose_Y\
								<<"\theading:"<<std::fixed<<std::setprecision(5)<<OdomMsg.Pose_Theta\
								<<"\t\tVx:"<<std::fixed<<std::setprecision(5)<<OdomMsg.Vx\
								<<"\tVyaw:"<<std::fixed<<std::setprecision(5)<<OdomMsg.Vyaw<<std::endl;
			ComputeOdom(dpluse_l,dpluse_r,dt_sec,OdomMsg,Odometry_msg_);
			time_temp.bit[0] = time_new.bit[0];
			time_temp.bit[1] = time_new.bit[1];

			PWMLeft.bit[0] = PWMLeft_new.bit[0];
			PWMLeft.bit[1] = PWMLeft_new.bit[1];
			PWMLeft.bit[2] = PWMLeft_new.bit[2];
			PWMLeft.bit[3] = PWMLeft_new.bit[3];

			PWMRight.bit[0] = PWMRight_new.bit[0];
			PWMRight.bit[1] = PWMRight_new.bit[1];
			PWMRight.bit[2] = PWMRight_new.bit[2];
			PWMRight.bit[3] = PWMRight_new.bit[3];

			Gyro_angle.bit[0] = Gyro_angle_new.bit[0];
			Gyro_angle.bit[1] = Gyro_angle_new.bit[1];
			Gyro_angleRate.bit[0] = Gyro_angleRate_new.bit[0];
			Gyro_angleRate.bit[1] = Gyro_angleRate_new.bit[1];



							   /*std::cout <<"odom_theta:"<<OdomMsg.Pose_Theta\
									  << "\tangle:"<<Gyro_angle.value\
							   <<"\tangleRate:"<<Gyro_angleRate.value<<std::endl;*/

/*			std::cout<<"bytes: "<<slen_\
								<<"\tT: "<<time_temp.value\
								<<"\tdTs: "<<std::showpoint<< dt_sec\
								<<"\tdP_m: "<<dpluse_max \
								<<"\tL: "<<PWMLeft.value\
								<<"\tR: "<<PWMRight.value\
								<<"\tdt_l: "<<dpluse_l\
								<<"\tdt_r: "<<dpluse_r<<std::endl;*/

//								<<"\tA:"<<Gyro_angle.value\;
//								<<"\tdA:"<<Gyro_angleRate.value;
//			<<"\tdT:"<<dt_raw\;
		}
		else if (slen_ == 0) {
					std::cout<<"receive fail."<<std::endl;
		}

		timer_.expires_from_now(boost::posix_time::milliseconds(timeout));
		timer_.wait();

	    }
		return true;
	}

	bool tcpclient::write_handle(ControlPackge &cmd,int timeout)
	{
		//for(;;){
		//ControlPackge Buffer;
		char write_buff[sizeof(ControlPackge)];
		memset(write_buff,'0',sizeof(write_buff));
		FillCmd2Buff(write_buff,cmd);
		boost::system::error_code error;
		//size_t slen_ = soc_.write_some(boost::asio::buffer(write_buff),error);
		soc_.write_some(boost::asio::buffer(write_buff),error);
		if (error == boost::asio::error::eof)
			return -1;
		else if (error)
			throw boost::system::error_code(error);
		//std::cout<<"cmd:\t"<<cmd.cmd.WheelLeft.value<<std::endl;
		//std::cout<<"send bytes:"<<slen_<<std::endl;
	//	timer_.expires_from_now(boost::posix_time::milliseconds(timeout));
	//	timer_.wait();
	//	}
		return 0 ;
	}

