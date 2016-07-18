/*
 * TCPClient.cpp
 *
 *  Created on: Jul 18, 2016
 *      Author: szr@giimagv
 */

#include"hans_bringup/TCPClient.h"
#include <stdio.h>
#include <unistd.h>
#include <netdb.h>
/*****************TCP Client*************************/
TcpClient::TcpClient()
{
    //this->PORT=2000;                //另一端的server port 端口号
    this->m_iSockfd=0;
    this->m_iSendbytes=0;
    this->m_iRecvbytes=0;
    this->m_stpMyClientControl =new MyClientStruct;
    memset(m_stpMyClientControl,0,sizeof(MyClientStruct));
    std::cout<<"Create MyTcpClient Class finished !"<<std::endl;
}

TcpClient::~TcpClient()
{
    if(!m_stpMyClientControl)
    {
      delete m_stpMyClientControl;
      m_stpMyClientControl=NULL;
      std::cout<<"Delete m_stpMyClientControl !!!"<<std::endl;

    }

    }

void TcpClient::ClientGetHostByName(const char *cpHostName)
{

    /*地址解析函数*/
    if ((m_stpMyClientControl->host = gethostbyname(cpHostName)) == NULL)
    {
        perror("gethostbyname");
        exit(1);
    }
    std::cout<<"The Client Host Name:"<<cpHostName<<std::endl;
}

void TcpClient::ClientSocketAddresInit(const unsigned short int &iPort)
{

    /*设置sockaddr_in 结构体中相关参数*/
    m_stpMyClientControl->serv_addr.sin_family = AF_INET;
    m_stpMyClientControl->serv_addr.sin_port = htons(iPort);
    m_stpMyClientControl->serv_addr.sin_addr = *((struct in_addr *)m_stpMyClientControl->host->h_addr);
    bzero(&(m_stpMyClientControl->serv_addr.sin_zero), 8);
    std::cout<<"The client_sockaddr_init OK! The PORT:"<<iPort<<std::endl;
    }

void TcpClient::ClientSocket()
{
    /*创建socket*/
    if ((m_iSockfd = socket(AF_INET,SOCK_STREAM,0)) == -1)
    {
        perror("socket");
        exit(1);
    }
    std::cout<<"The Client Create The Socket !!"<<std::endl;

}

void TcpClient::ClientConnect()
{
       /*调用connect函数主动发起对服务器端的连接*/
       int  my_true=1;
       while(my_true)
       {
            if(connect(m_iSockfd,(struct sockaddr *)&m_stpMyClientControl->serv_addr, sizeof(struct sockaddr))== -1)
            {
                std::cout<<"Can not connect,I  will try"<<std::endl;
            }
            else
                my_true=0;
       }
       std::cout<<"The Client Connect OK !!"<<std::endl;
}


int TcpClient::ClientSend(unsigned const char *sbuffer,const int &iSize)//这里必须要用size，因为传过来的buffer只是一个地址。
{
    int m_iSendbytes_ok=0;
    while(m_iSendbytes_ok<iSize)
    {
        if ((m_iSendbytes = send(m_iSockfd, sbuffer+m_iSendbytes_ok,iSize-m_iSendbytes_ok, 0)) == -1)
        {
            perror("send");
            exit(1);
        }
        m_iSendbytes_ok+=m_iSendbytes;
    }
    std::cout<<"Send OK!!"<<std::endl;
    return  m_iSendbytes;

}

int TcpClient::ClientSend(const ControlPackge &Cmd)
{
        int m_iSendbytes_ok=0;
        int iSize=sizeof(Cmd);
        while(m_iSendbytes_ok<iSize)
        {
            if ((m_iSendbytes = send(m_iSockfd, &Cmd, iSize-m_iSendbytes_ok, 0)) == -1)
            {
                perror("send");
                exit(1);
            }
            m_iSendbytes_ok+=m_iSendbytes;
        }
        std::cout<<"Send OK!!"<<std::endl;
        return  m_iSendbytes;
}



int TcpClient::ClientReceive(char*rBuffer,const int &iBufferSize,const int &iWantByte)
{
    memset(rBuffer , 0, iBufferSize);
    int iGet = recv(m_iSockfd,rBuffer,iWantByte,0);
    if(iGet < iWantByte)
    {
        std::cout<<"Socket Get Not Equal to WantByte"<<std::endl;
    }
    if(iGet < iBufferSize)
    {
        rBuffer[iGet] = '\0';
    }
    return iGet;
}

int TcpClient::ClientReceive(AGVDATA &rBuffer)
{
    memset(&rBuffer, 0, sizeof(rBuffer));
    int iGet = recv(m_iSockfd,&rBuffer,sizeof(rBuffer),0);
    if(iGet < sizeof(rBuffer))
    {
        std::cout<<"Socket Get Not Equal to WantByte"<<std::endl;
    }
    if(iGet < sizeof(rBuffer))
    {
        rBuffer[iGet] = '\0';
    }
    return iGet;
}


void TcpClient::ClientCloseSockfd()
{
    close(m_iSockfd);
}

void TcpClient::ClientControl(const char *cpHostName,const unsigned short int &iPort)
{

        ClientGetHostByName(cpHostName);
        ClientSocket();
        ClientSocketAddresInit(iPort);
        // 接收缓冲区
        int nRecvBufLen = 32 * 1024; //设置为32K?????
        (m_iSockfd,SOL_SOCKET, SO_RCVBUF, (const char*)&nRecvBufLen, sizeof(int));
     	ClientConnect();
    }

int TcpClient::GetClientFd()const
{
    return m_iSockfd;
}


