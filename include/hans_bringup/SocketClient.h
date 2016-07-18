/*
 * SocketClient.h
 *
 *   Created on: 2016年7月5日
 *       Author: birl
 *  Modified on: 2016.07.15
 *   	 Author: szr@giimagv
 */

#ifndef INCLUDE_HANS_BRINGUP_SOCKETCLIENT_H_
#define INCLUDE_HANS_BRINGUP_SOCKETCLIENT_H_
#include <sys/socket.h>
#include <iostream>
#include <stdlib.h>
#include <cstring>
#include <errno.h>
#include <sys/types.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "hans_bringup/data_definition.h"
class SocketClient
{
	public:
		SocketClient();
		~SocketClient();

		bool CreatSocket();
		bool ConnetToServer ( const std::string &host, const int port );

		//Data Transmission Function
		//bool SendMsg (char* cmd) const;
		//bool SendMsg (std::string ) const;
		bool SendMsg (const ControlPackge &cmd) const;
		//int RecvMsg ( std::string& ) const;
		int RecvMsg ( AGVDATA &buffer ) const;
		std::string SendCommandWithFeedback(std::string &) const;
		void CloseSocket();
	private:
		bool Socket_valid() const { return sockfd != -1; }		//agv_sock＝-1 if create Socket failed, the function will return false.

	//封装socket参数．
	private:

		struct sockaddr_in serv_addr;
		int sockfd;
};




#endif /* INCLUDE_HANS_BRINGUP_SOCKETCLIENT_H_ */
