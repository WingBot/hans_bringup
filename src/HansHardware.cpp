/*
 * HansHardware.cpp
 *
 *  Created on: Jul 18, 2016
 *      Author: Paul&szr@giimagv
 */
#include "hans_bringup/data_definition.h"
#include "hans_bringup/HansHardware.h"
#include "hans_bringup/AgvPara.h"

#include "math.h"

namespace AGVHardware
{
	void ComputeOdom(const AGVSENSORS *AgvData, const ushort_int2 LastTime, OdomPackage &OdomMsg)
	{
		ushort_int2 CurrentTime;
		unsigned short int dt;
		for (int i = 0; i < 2; ++i) {
			CurrentTime.Bit[i]= AgvData->TimeStamp[i];
		}
		dt = CurrentTime.Value - LastTime.Value;
		wchar_t4 LeftWhlPluse,RightWhlPluse;
		for (int i = 0; i < 4; ++i) {
			LeftWhlPluse.Bit[i] = AgvData->WheelLeft_Encoder[i];
			RightWhlPluse.Bit[i] = AgvData->WheelRight_Encoder[i];
		}
		PassPath dPassPath;
		dPassPath.Left = Cm * (double)LeftWhlPluse.Value;
		dPassPath.Right = Cm * (double)RightWhlPluse.Value;
		OdomMsg.PassPath = (dPassPath.Right + dPassPath.Left)/2;
		OdomMsg.PassTheta = (dPassPath.Right - dPassPath.Right)/WheelBase;
		OdomMsg.Locat.Theta += OdomMsg.PassTheta;
		OdomMsg.Locat.X += OdomMsg.PassPath*cos(OdomMsg.PassTheta/2);
		OdomMsg.Locat.Y += OdomMsg.PassPath*sin(OdomMsg.PassTheta/2);
		OdomMsg.Velo.Linear = OdomMsg.PassPath/dt;
		OdomMsg.Velo.Angular = OdomMsg.PassTheta/dt;
		for (int i = 0; i < 2; ++i) {
			OdomMsg.TimeStamp.Bit[i]=LastTime.Bit[i];
		}
	}

    void DriveCommand(const int LeftWheelSpeed, const int RightWheelSpeed, ControlPackge &CmdPackage)
      {
           CommandBit LeftWheel, RightWheel;

           LeftWheel.WheelSpeed = LeftWheelSpeed;
           RightWheel.WheelSpeed = RightWheelSpeed;

           CmdPackage.cmd.WheelLeft[0] = LeftWheel.NumBit[0];
           CmdPackage.cmd.WheelLeft[1] = LeftWheel.NumBit[1];
           CmdPackage.cmd.WheelLeft[2] = LeftWheel.NumBit[2];
           CmdPackage.cmd.WheelLeft[3] = LeftWheel.NumBit[3];

           CmdPackage.cmd.WheelRight[0] = RightWheel.NumBit[0];
           CmdPackage.cmd.WheelRight[1] = RightWheel.NumBit[1];
           CmdPackage.cmd.WheelRight[2] = RightWheel.NumBit[2];
           CmdPackage.cmd.WheelRight[3] = RightWheel.NumBit[3];

       }

      // Stop the mobile base.
    ControlPackge StopCommand()
      {
          ControlPackge stop_cmd;
          DriveCommand(0, 0, stop_cmd);
          return stop_cmd;
      }

};



