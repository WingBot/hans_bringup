/*
 * TCPClient.h
 *
 *  Created on: Jul 18, 2016
 *      Author: szr@giimagv
 */

#ifndef INCLUDE_HANS_BRINGUP_TCPCLIENT_H_
#define INCLUDE_HANS_BRINGUP_TCPCLIENT_H_

#include<iostream>
#include<stdlib.h>
#include<cstring>
#include<errno.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include <arpa/inet.h>

#include "hans_bringup/data_definition.h"
//const int MAX_RECVBYTES = 30;				//接收数据的最大字节数．

class TcpClient
{
    public:
             TcpClient();
            ~TcpClient();
    public:
             void ClientCloseSockfd();
             int  GetClientFd()const;
             void ClientControl(const char *cpHostName,const unsigned short int &iPort);
             int  ClientSend(unsigned const char *buffer,const int &iSize);
             int  ClientSend(const ControlPackge &cmd);
             int  ClientReceive(char *pBuffer,const int &iBufferSize,const int &iWantByte);
             int  ClientReceive(AGVDATA &buffer);
    private:
             void ClientGetHostByName(const char *cpHostName);
             void ClientSocket();
             void ClientSocketAddresInit(const unsigned short int &iPort);
             void ClientConnect();
    private:
           // unsigned  int	m_iMaxQueConnNum;       //表示最大连接数
            int	m_iSockfd;
            int m_iSendbytes;
            int	m_iRecvbytes;
    private:
            typedef  struct
            {
               struct hostent	*host;
               struct sockaddr_in	serv_addr;
            }MyClientStruct;
    MyClientStruct  *m_stpMyClientControl;
};



#endif /* INCLUDE_HANS_BRINGUP_TCPCLIENT_H_ */
