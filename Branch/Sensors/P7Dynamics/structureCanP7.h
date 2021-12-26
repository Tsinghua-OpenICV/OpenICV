//  created:    2017/12/15
//  author:     Kun JIANG
//              Copyright Tsinghua University
//
//  version:    $Id: $
//
//  purpose:

#ifndef __STRUCTURECANP7_H__
#define __STRUCTURECANP7_H__

#include <cstddef>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iostream>
#include<msgpack.hpp>
#include "CanFrame.h"
using namespace std;
/// Vehicle speed CAN frame
/// frequency 10 Hz
/// 0.6 km/h minimum threshold


typedef float float32;
typedef double float64;

typedef char int8;
typedef unsigned char uint8;

typedef unsigned short uint16;
typedef short int16;

typedef unsigned int uint32;
typedef int int32;
//=====================================================

//update message

typedef struct
{
	uint8 SteeringAngleVD;
	float32 SteeringAngleSpd;
	float32 SteeringAngle;
	float32 ResponseTorque;
	uint8 ResponseTorqueVD;
	uint8 SteeringAngleSpdVD;
	uint8 MsgCounter;
	uint8 Checksum;
	MSGPACK_DEFINE(SteeringAngleVD,SteeringAngleSpd,SteeringAngle,ResponseTorque,ResponseTorqueVD,SteeringAngleSpdVD,MsgCounter,Checksum);
} CanFrame_SCU_IPC_1_0x174; //0x174;

typedef struct
{
	float32 FLWheelSpd;
	float32 FRWheelSpd;
	float32 RLWheelSpd;
	float32 RRWheelSpd;
	uint8 FLWheelSpdVD;
	uint8 FRWheelSpdVD;
	uint8 RLWheelSpdVD;
	uint8 RRWheelSpdVD;
	MSGPACK_DEFINE(FLWheelSpd,FRWheelSpd,RLWheelSpd,RRWheelSpd,FLWheelSpdVD,FRWheelSpdVD,RLWheelSpdVD,RRWheelSpdVD);
} CanFrame_SCU_IPC_2_0x175; //0x175

typedef struct
{
	uint8 DBWSt;
	uint8 VehSpdVD;
	uint8 BrkPedalSt;
	uint8 BrkPedalStVD;
	float32 VehSpd;
	uint8 BrkLightOn;
	uint8 MsgCounter;
	uint8 Checksum;
	MSGPACK_DEFINE(DBWSt,VehSpdVD,BrkPedalSt,BrkPedalStVD,VehSpd,BrkLightOn,MsgCounter,Checksum);
} CanFrame_SCU_IPC_3_0x176; //0x176

typedef struct
{
	uint8 ActVehLongAccelVD;
	uint8 ActVehLateralAccelVD;
	uint8 YAWVD;
	float32 YAW;
	float32 ActVehLongAccel;
	float32 ActVehLateralAccel;
	uint8 EPBSysSt;
	uint8 MsgCounter;
	uint8 Checksum;
	MSGPACK_DEFINE(ActVehLongAccelVD,ActVehLateralAccelVD,YAWVD,YAW,ActVehLongAccel,ActVehLateralAccel,EPBSysSt,MsgCounter,Checksum);
} CanFrame_SCU_IPC_4_0x177; //0x177

typedef struct
{
	uint8 DriverDoorLockSt;
	uint8 DriverDoorAjarSt;
	uint8 PsngrDoorAjarSt;
	uint8 RLDoorAjarSt;
	uint8 RRDoorAjarSt;
	uint8 LTurnLampOutputSt;
	uint8 RTurnLampOutputSt;
	uint8 HazardLampOutputSt;
	uint8 LowBeamOutputSt;
	uint8 HighBeamOutputSt;
	uint8 Horndriverst;
	uint8 FrontWiperOutputSt;
	uint8 PowerMode;
	MSGPACK_DEFINE(DriverDoorLockSt,DriverDoorAjarSt,PsngrDoorAjarSt,RLDoorAjarSt,RRDoorAjarSt,LTurnLampOutputSt,RTurnLampOutputSt, HazardLampOutputSt, LowBeamOutputSt, HighBeamOutputSt,Horndriverst,FrontWiperOutputSt,PowerMode);
} CanFrame_SCU_IPC_5_0x178; //0x178

typedef struct
{
	float32 dstBat_Dsp;
	float32 AccPedalSig;
	uint8 BrkPedalSt;
	uint8 BrkPedalStVD;
	uint8 CurrentGearLevVD;
	uint8 CurrentGearLev;
	uint8 MsgCounter;
	uint8 Checksum;
	MSGPACK_DEFINE(dstBat_Dsp,AccPedalSig,BrkPedalSt,BrkPedalStVD,CurrentGearLevVD,CurrentGearLev,MsgCounter,Checksum);
} CanFrame_SCU_IPC_6_0x179; //0x179

typedef struct
{
	uint8 PSeatBeltWarning;
	uint8 DriverSeatBeltWarning;
	uint8 _1ndLPSeatBeltWar;
	uint8 _1ndMPSeatBeltWar;
	uint8 _1ndRPSeatBeltWar;
	uint8 TotalOdometerVD;
	float32 TotalOdometer;
	uint8 ACSt;
	uint8 ErrSt;
	uint8 OverrideRes;
	MSGPACK_DEFINE(PSeatBeltWarning,DriverSeatBeltWarning,_1ndLPSeatBeltWar,_1ndMPSeatBeltWar,_1ndRPSeatBeltWar,TotalOdometerVD,TotalOdometer,ACSt,ErrSt,OverrideRes);
}CanFrame_SCU_IPC_7_0x17A; //0x17A

typedef struct _pid{
    float SetSpeed;            //定义设定值
    float ActualSpeed;         //定义实际值
    float err;                 //定义偏差值
    float err_last;            //定义上一个偏差值
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
    float accleration;         //定义加速度值（控制执行器的变量）
    float integral;            //定义积分值
	float umax;
    float umin;
}PID;


#endif
