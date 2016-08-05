/*
 * BoostTCPClient.h
 *
 *  Created on: Aug 3, 2016
 *      Author: agv
 */

#ifndef INCLUDE_HANS_BRINGUP_BOOSTTCPCLIENT_H_
#define INCLUDE_HANS_BRINGUP_BOOSTTCPCLIENT_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/asio/deadline_timer.hpp>

#include "AGVCmdFunction.h"
#include "DataType.h"

const size_t MAX_RCV_SIZE = 1024*2;
const size_t MAX_SEND_SIZE = sizeof(ControlPackge);
const uint16_t  TIME_RANGE = 65535;//uint16_t 最大取值
const int32_t PLUSE_RANGE = 4294967295;// int32_t 最大值
const float TIME_TO_SEC_SCALE = 2/1000;//下位机一个时间计数刻度为2ms
using boost::asio::ip::tcp;
class tcpclient{
public:
	tcpclient(boost::asio::io_service &io_service);
	~tcpclient();
	void connect(tcp::endpoint &iendpoint);
	bool read_handle(int timeout,ODOM &OdomMsg,nav_msgs::Odometry &Odometry_msg_);
	bool write_handle(ControlPackge &cmd,int timeout);
private:
	tcp::socket soc_;
	boost::asio::deadline_timer timer_;
};




#endif /* INCLUDE_HANS_BRINGUP_BOOSTTCPCLIENT_H_ */
