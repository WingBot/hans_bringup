/*
 * SocketClient.cpp
 *
 *   Created on: 2016年7月5日
 *       Author: birl
 *  Modified on: 2016.07.15
 *   	 Author: szr@giimagv
 */
#include <stdio.h>

#include "hans_bringup/SocketClient.h"

SocketClient::SocketClient():sockfd(-1)
{
	bzero(&serv_addr,sizeof(serv_addr));  //初始化服务器端serv_addr结构为0
	//memset(&serv_addr, 0, sizeof(serv_addr));  //初始化服务器端serv_addr结构
}
SocketClient::~SocketClient()
{
    if ( Socket_valid() )			//socket描述符还存在时关闭socket.
	close( sockfd );
}
bool SocketClient::CreatSocket()
{
	sockfd = socket(AF_INET, SOCK_STREAM, 0);   //服务器端开始建立socket描述符
	if(!Socket_valid())
	{
		printf("Socket create failed.");
		return false;
	}
	return true;
}

bool SocketClient::ConnetToServer ( const std::string &host, const int port)
{
	if (!Socket_valid()) return false;		//判断socket描述符是否有效．
	/* 服务器端填充 serv_addr结构  */
	serv_addr.sin_family = AF_INET;	//IPv4.AF_INET决定了要用ipv4地址（32位的）与端口号（16位的）的组合
	serv_addr.sin_port = htons(port); //服务器端端口号
//	m_servaddr.sin_addr.s_addr = inet_addr("192.168.1.1");		//IP地址与端口号都是在主程序中指定的，这里只是为了示意补全m_servaddr．
	if( inet_pton(AF_INET, host.c_str(), &serv_addr.sin_addr) <= 0)	//将字符串转换成网络字节,地址由字符串转换为二进制数
	{
		perror("Fail to convert.");
		//exit(1);
		//printf("inet_pton error for %s\n",host.c_str());
		return false;
	}


//	int inet_pton(int af, const char *src, void *dst);
//	转换字符串到网络地址，第一个参数af是地址族，转换后存在dst中。
//	　　if(inet_pton(AF_INET,"192.168.11.6",&addr_n)<0)/*地址由字符串转换为二级制数*/
//	　　{
//	　　perror("fail to convert");
//	　　exit(1);
//	　　}
    if(connect(sockfd,(struct sockaddr *)(&serv_addr),sizeof(serv_addr)) == -1)
    {
    	fprintf(stderr,"Connect AGVServer Error:%s\a\n",strerror(errno));
        return false;
        //exit(1);
    }
//	int conn_statust = connect(m_sock, (sockaddr*)&m_servaddr, sizeof(m_servaddr));
//	if(connect(m_sock, (struct sockaddr*)&m_servaddr, sizeof(m_servaddr)) < 0)	//连接服务器．
//	{
//		printf("Connect URServer error: %s(errno: %d)\n",strerror(errno),errno);
//		return false;
//	}
	return true;
}
////发送不需要请求UR返回信息的命令．
//bool SocketClient::SendMsg (char* cmd) const
//{
//	if( send(sockfd, cmd, 10, 0) < 0)				//send函数被Modbus TCP以20HZ的频率占用，会影响正常的TCP/IP通信，所以换一组通信函数．
//	{
//		printf("Send msg error: %s(errno: %d)\n", strerror(errno), errno);
//		return false;
//	}
//	return true;
//}

bool SocketClient::SendMsg (const ControlPackge &cmd) const
{
        if( send(sockfd, &cmd, sizeof(cmd), 0) < 0)
        {
                printf("Send msgs error: %s(errno: %d)\n", strerror(errno), errno);
                return false;
        }
        return true;
}

//发送不需要请求UR返回信息的命令．
//bool SocketClient::SendMsg (std::string s) const
//{
//	//s = s + "\n";
//	if( send(m_sock, s.c_str(), (unsigned int)s.size(), 0) < 0)				//send函数被Modbus TCP以20HZ的频率占用，会影响正常的TCP/IP通信，所以换一组通信函数．
//	{
//		printf("Send msg error: %s(errno: %d)\n", strerror(errno), errno);
//		return false;
//	}
//	if(write(m_sock, s.c_str(), (unsigned int)s.size()) < 0)
//	{
//		printf("Send msg error: %s(errno: %d)\n", strerror(errno), errno);
//		return false;
//	}
//	return true;
//}

//int SocketClient::RecvMsg ( std::string& s) const
//{
//	char buf[MAX_RECVBYTES + 1];				//接收缓冲区，最后一个用于存放'\0'.
//	s = "";
//	memset(buf, 0, MAX_RECVBYTES + 1);		//初始化缓存区．
//	int status = recv( m_sock,buf, MAX_RECVBYTES ,0 );						//与send函数对应的接收函数．
//	if(-1 == status)
//	{
//		printf("Receive msg error:%s(errno: %d)\n", strerror(errno), errno);
//		return -1;			//接收错误．
//	}
//	return status;
///*
//	int status = read( m_sock, buf, MAX_RECVBYTES);
//	if(-1 == status)
//	{
//		printf("Receive msg error:%s(errno: %d)\n", strerror(errno), errno);
//		return -1;			//接收错误．
//	}
//	else if(status == 0)
//		return 0;			//接收到了文件结尾处．
//	else
//	{
//		s = buf;				//string 类对'='有操作符重载，所以可以直接赋char数组名．
//		return status;		//返回接收到的字节数．
//	}
//*/
//}

int SocketClient::RecvMsg (AGVDATA &buffer ) const
{
//	int status = recv( m_sock,&buffer, MAX_RECVBYTES ,0 );						//与send函数对应的接收函数．
//        int n = sizeof(buffer);
    int status = recv( sockfd, &buffer, sizeof(buffer), 0);
	if(-1 == status)
	{
		printf("Receive msgs error:%s(errno: %d)\n", strerror(errno), errno);
		return -1;			//接收错误．
	}
        else if(status == 0)
            return 0;                       //接收到了文件结尾处．
        return status;          //返回接收到的字节数．
 /*
  * Ssize_t read(int fd,void *buf,size_t nbyte)
  * Ssize_t write(int fd,const void *buf,size_t nbytes);
    Read函数是负责从fd中读取内容，当读取成功时，read返回实际读取到的字节数，如果返回值是0，表示已经读取到文件的结束了，小于0表示是读取错误。
    Int recv(int fd,void *buf,int len,int flags)
    Int send(int fd,void *buf,int len,int flags)

	int status = read( m_sock, &buffer, MAX_RECVBYTES+1);
	if(-1 == status)
	{
		printf("Receive msg error:%s(errno: %d)\n", strerror(errno), errno);
		return -1;			//接收错误．
	}
	else if(status == 0)
		return 0;			//接收到了文件结尾处．
	return status;		//返回接收到的字节数．
*/
}

//发送需要请求UR返回数据的命令，及时接收，避免造成阻塞．
//std::string SocketClient::SendCommandWithFeedback(std::string & cmd) const
//{
////	cmd += "\n";		//UR命令格式要求在字符串末尾加＇\n＇．
//	std::string s_feedback= "";
//	if(!sendMsg(cmd))
//	{
//		printf("Connect UR5 server failed.\n");
//		return "SENDFALSE";	//返回发送错误标志字符串．
//	}
//	//命令发送成功，必须接收UR的返回信息，避免造成阻塞．
//	int bytesRec = recvMsg(s_feedback);
//	if(bytesRec <= 0)
//	{
//		printf("Receive feedback data false!");
//		return "RECVFALSE";	//返回接收错误标志字符串．
//	}
//	return s_feedback;
//}
void SocketClient::CloseSocket()
{
	close(sockfd);
}


