/*
 * data_definition.h
 *
 *  Created on: 2016年7月7日
 *      Author: birl
 */

#ifndef SOURCE_DIRECTORY__INCLUDE_HAN_AGV_DATA_DEFINITION_H_
#define SOURCE_DIRECTORY__INCLUDE_HAN_AGV_DATA_DEFINITION_H_

/************
 * 接收AGV返回的数据
 *
 * **************/

typedef unsigned char CheckSUM;		//校验类型．
typedef unsigned char byte;

typedef struct HANsAGVMsgHeader		//父数据头
{
	unsigned char FrameHeader[2];
	unsigned char DatasLength;
}HANSMSGHEADER;

typedef struct HANsAGVSensorsMsg		//base sensors message.
{
	byte MsgHeader;
	byte MsgLength;
	byte TimeStamp[2];
	byte Bumper;			//保险杠
	byte WheelMissed;
	byte TouchGround;
	char WheelLeft_Encoder[4];
	char WheelRight_Encoder[4];
	byte PWMLeft;
	byte PWMRight;
	byte HANButton;
	byte HANCharger;
	byte HANBattery;
	byte CurrentBeyond;	//过电流．
}AGVSENSORS;

typedef struct HANsAGVGyroMsg		//陀螺仪传感器数据．
{
    byte MsgHeader;
    byte MsgLength;

	char Angle[2];
	char AngleRate[2];
	byte NULL1;
	byte NULL2;
	byte NULL3;
}GyroMSG;

typedef struct HANsAGVFeedbackData
{
	HANSMSGHEADER FrameDataHeader;
	AGVSENSORS SensorsData;
	GyroMSG Gyromsg;
	CheckSUM checksum;
}AGVDATA;

//=============================================================
/*
 * 发送控制指令
 */

//车轮转速的高/低字节．
union CommandBit
{
    int WheelSpeed;     //单个轮子的轮速(RPM),注意单位的统一。
	byte NumBit[4];     //轮速对应的四个字节。
};
//CommandBit wheel_L, wheel_R;

struct ControlCommand
{
        byte DataHeader;        //控制指令的数据类型编号。
        byte DataLenght;      //默认数据长度为8个字节。
        byte WheelLeft[4];
        byte WheelRight[4];
};

struct ControlPackge
{
        HANSMSGHEADER FrameDataHeader;
        ControlCommand cmd;
        CheckSUM checksum;
        ControlPackge()
        {
              FrameDataHeader.FrameHeader[0] = 0xAA;
              FrameDataHeader.FrameHeader[1] = 0x55;
              FrameDataHeader.DatasLength = 0x0B;
              cmd.DataHeader = 0x01;
              cmd.DataLenght = 0x08;
              checksum = 0x00;
        }
};
//=============================================================
/*
 * 里程计数据结构
 */
//=============================================================
struct AgvPose   // 2d pose
{
	float X;
	float Y;
	float Theta;
};

struct AgvVelocity
{
	float AgvLinear;
	float AgvAngular;
};

struct AgvState
{
AgvPose Locat;
AgvVelocity VehVel;
float PassPath; //基座中心点走过路径
float PassTheta;
};






#endif /* SOURCE_DIRECTORY__INCLUDE_HAN_AGV_DATA_DEFINITION_H_ */
