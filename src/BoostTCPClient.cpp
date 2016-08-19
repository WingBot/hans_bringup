/*
 * BoostTCPClient.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: szr@giimagv
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

		if(slen_ > 0 && ((slen_ % FEEDBACK_LENTH)==0))
		{
			int Nfull = slen_ / FEEDBACK_LENTH;
			uint8_t str_begin = read_buff_[Nfull*FEEDBACK_LENTH -FEEDBACK_LENTH];

			if(str_begin == 170)  //0xaa
			{
				if (!ReadData) {
					for (int var = 0; var < 2; ++var) {
						time_temp.bit[var] = read_buff_[Nfull*FEEDBACK_LENTH -29+var];
					}
					for (int var = 0; var < 4; ++var) {
						PWMLeft.bit[var] = read_buff_[Nfull*FEEDBACK_LENTH - 24+var];
						PWMRight.bit[var] = read_buff_[Nfull*FEEDBACK_LENTH - 20+var];
					}
				}
				for (int var = 0; var < 2; ++var) {
					time_new.bit[var] = read_buff_[Nfull*FEEDBACK_LENTH -29+var];
				}
				for (int var = 0; var < 4; ++var) {
					PWMLeft_new.bit[var] = read_buff_[Nfull*FEEDBACK_LENTH - 24+var];
					PWMRight_new.bit[var] = read_buff_[Nfull*FEEDBACK_LENTH - 20+var];
				}
			}
			str_begin = read_buff_[Nfull*FEEDBACK_LENTH - 10];//陀螺仪的帧头
			if (str_begin == 4)//0x04
			{
				if (!ReadData) {
					for (int var = 0; var < 2; ++var) {
						Gyro_angle.bit[var] = read_buff_[Nfull*FEEDBACK_LENTH - 8+var];
						Gyro_angleRate.bit[var] = read_buff_[Nfull*FEEDBACK_LENTH - 6+var];
					}
				ReadData = true;
				}
				for (int var = 0; var < 2; ++var) {
					Gyro_angle_new.bit[var] = read_buff_[Nfull*FEEDBACK_LENTH - 8+var];
					Gyro_angleRate_new.bit[var] = read_buff_[Nfull*FEEDBACK_LENTH - 6+var];
				}
			}
			float_t dt_sec = 0.0;
			GetDeltaTimeSec(time_temp,time_new,dt_sec);
			int32_t dpluse_l ,dpluse_r;
			GetDeltaPluse(PWMLeft_new,PWMLeft,dpluse_l,timeout);
			GetDeltaPluse(PWMRight_new,PWMRight,dpluse_r,timeout);
			std::cout<<"========================"<<std::endl;
			std::cout<<"bytes:\t\t"<<slen_<<std::endl;
			std::cout<<"time:\t\t"<<time_temp.value<<std::endl;

			std::cout<<"read_rate:\t"<<timeout<<std::endl;
			std::cout<<"x:\t\t"<<std::fixed<<std::setprecision(5)<<OdomMsg.Pose_X<<std::endl;
			std::cout<<"y:\t\t"<<std::fixed<<std::setprecision(5)<<OdomMsg.Pose_Y<<std::endl;
			std::cout<<"theta:\t\t"<<std::fixed<<std::setprecision(5)<<OdomMsg.Pose_Theta<<std::endl;
			std::cout<<"Vx:\t\t"<<std::fixed<<std::setprecision(5)<<OdomMsg.Vx<<std::endl;
			std::cout<<"Vyaw:\t\t"<<std::fixed<<std::setprecision(5)<<OdomMsg.Vyaw<<std::endl;
			std::cout<<"Gyro_angle:\t"<<Gyro_angle.value<<std::endl;
			/*std::cout<<"bytes:"<<slen_\
								<<"\ttime:"<<time_temp.value\
								<<"\tx:"<<std::fixed<<std::setprecision(5)<<OdomMsg.Pose_X\
								<<"\ty:"<<std::fixed<<std::setprecision(5)<<OdomMsg.Pose_Y\
								<<"\theading:"<<std::fixed<<std::setprecision(5)<<OdomMsg.Pose_Theta\
								<<"\t\tVx:"<<std::fixed<<std::setprecision(5)<<OdomMsg.Vx\
								<<"\tVyaw:"<<std::fixed<<std::setprecision(5)<<OdomMsg.Vyaw<<std::endl;*/
			ComputeOdom(dpluse_l,dpluse_r,dt_sec,OdomMsg,Odometry_msg_);
			for (int var = 0; var < 2; ++var) {
				time_temp.bit[var] = time_new.bit[var];
				Gyro_angle.bit[var] = Gyro_angle_new.bit[var];
				Gyro_angleRate.bit[var] = Gyro_angleRate_new.bit[var];
			}
			for (int var = 0; var < 4; ++var) {
				PWMLeft.bit[var] = PWMLeft_new.bit[var];
				PWMRight.bit[var] = PWMRight_new.bit[var];
			}
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
		char write_buff[sizeof(ControlPackge)];
		memset(write_buff,'0',sizeof(write_buff));
		FillCmd2Buff(write_buff,cmd);
		boost::system::error_code error;
		soc_.write_some(boost::asio::buffer(write_buff),error);
		if (error == boost::asio::error::eof)
			return -1;
		else if (error)
			throw boost::system::error_code(error);
		return 0 ;
	}

