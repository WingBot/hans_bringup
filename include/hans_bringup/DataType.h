/*
 * DataType.h
 *
 *  Created on: Aug 3, 2016
 *      Author: agv
 */

#ifndef INCLUDE_HANS_BRINGUP_DATATYPE_H_
#define INCLUDE_HANS_BRINGUP_DATATYPE_H_


/************
 * 接收AGV返回的数据
 *
 * **************/
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <vector>



const uint16_t MOTOR_MAX_PWM = 3000;
const uint16_t MOTOR_MIN_PWM = 100;
typedef unsigned char CheckSUM;		//校验类型．
typedef unsigned char byte;
typedef std::vector<int8_t> VBytes;
union  U_uint16_t	//
{
	uint16_t value;
	uint8_t bit[2];
};

union  U_uint32_t	//
{
	uint32_t value;
	uint8_t bit[4];
};

union  U_uint64_t	//
{
	uint32_t value;
	uint8_t bit[8];
};

union  U_int16_t	//
{
	int16_t value;
	int8_t bit[2];
};

union  U_int32_t	//
{
	int32_t value;
	int8_t bit[4];
};

union  U_int64_t
{
	int32_t value;
	int8_t bit[8];
};


typedef struct HANsAGVMsgHeader		//父数据头
{
	U_uint16_t FrameHeader;
	uint8_t DatasLength;
}HANSMSGHEADER;

typedef struct HANsAGVSensorsMsg		//base sensors message.
{
	byte MsgHeader;
	byte MsgLength;
	U_uint16_t TimeStamp;
	byte Bumper;			//保险杠
	byte WheelMissed;
	byte TouchGround;
	U_int32_t WheelLeft_Encoder;
	U_int32_t WheelRight_Encoder;
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

	U_int16_t Angle;
	U_int16_t AngleRate;
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


union HANsFeedbackData
{
	byte bit[34];
	AGVDATA Data;
};
static const size_t AGVDATA_SIZE = sizeof(AGVDATA);
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
        U_int32_t WheelLeft;
        U_int32_t WheelRight;
};

struct ControlPackge
{
        HANSMSGHEADER FrameDataHeader;
        ControlCommand cmd;
        CheckSUM checksum;
        ControlPackge()
        {
        	FrameDataHeader.FrameHeader.bit[0] = 0xAA;
            FrameDataHeader.FrameHeader.bit[1] = 0x55;
              FrameDataHeader.DatasLength = 0x0B;
              cmd.DataHeader = 0x01;
              cmd.DataLenght = 0x08;
              checksum = 0x00;
        }
};

typedef struct OdomPackage
{
	double_t Vx;
	double_t Vyaw;
	double_t Pose_X;
	double_t Pose_Y;
	float_t Pose_Theta;
}ODOM;





#endif /* INCLUDE_HANS_BRINGUP_DATATYPE_H_ */
