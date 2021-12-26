//  created:    2017/12/15
//
//  author:     Kun JIANG
//              Copyright Tsinghua University
// 
//  version:    $Id: $
//
//  purpose:    

#ifndef  __STRUCTURECANXP_H__
#define __STRUCTURECANXP_H__

#include <msgpack.hpp>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <sstream>
#include "CanFrame.h"
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

//update message State signal
typedef struct
{
 uint8 SteeringAngleVD;//
 uint16 SteeringAngleSpd; //
 uint32 SteeringAngle; // -780~779.9
 uint32 ResponseTorque; //-10.24~10.23
 uint8 ResponseTorqueVD; //
 uint8 SteeringAngleSpdVD; //
 uint8 MsgCounter; //
 uint8 Checksum; // 
   MSGPACK_DEFINE(SteeringAngleVD, SteeringAngleSpd, SteeringAngle, ResponseTorque, ResponseTorqueVD,
   SteeringAngleSpdVD,  MsgCounter, Checksum);
}CanFrame_SCU_IPC_1_0x20A;  //0x20A;

typedef struct
{
 uint32 FLWheelSpd; // 0~240
 uint32 FRWheelSpd; // 0~240
 uint32 RLWheelSpd; // 0~240
 uint32 RRWheelSpd; // 0~240
 uint8 FLWheelSpdVD; //
 uint8 FRWheelSpdVD; //
 uint8 RLWheelSpdVD; //
 uint8 RRWheelSpdVD; //
   MSGPACK_DEFINE(FLWheelSpd, FRWheelSpd, RLWheelSpd, RRWheelSpd, FLWheelSpdVD,
   FRWheelSpdVD,  RLWheelSpdVD, RRWheelSpdVD);
}CanFrame_SCU_IPC_2_0x205;  //0x205;

typedef struct
{
 uint8 DBWSt; // 
 uint8 VehSpdVD; //
 uint8 BrkPedalSt; // 
 uint8 BrkPedalStVD; // 
 uint32 VehSpd; // 0~240
 uint8 BrkLightOn; //
 uint8 MsgCounter; //
 uint8 Checksum; //
   MSGPACK_DEFINE(DBWSt, VehSpdVD, BrkPedalSt, BrkPedalStVD, VehSpd,
   BrkLightOn,  MsgCounter, Checksum);
}CanFrame_SCU_IPC_3_0x206;  //0x206;

typedef struct
{
 uint8 ActVehLongAccelVD; // 
 uint8 ActVehLateralAccelVD; //
 uint8 YAWVD; // 
 uint32 YAW; // -93~93
 uint32 ActVehLongAccel; // -1.8~1.8
 uint32 ActVehLateralAccel; // -1.8~1.8
 uint8 EPBSysSt; //
 uint8 MsgCounter; //
 uint8 Checksum; //
   MSGPACK_DEFINE(ActVehLongAccelVD, ActVehLateralAccelVD, YAWVD, YAW, ActVehLongAccel,
   ActVehLateralAccel,  EPBSysSt, MsgCounter, Checksum);
}CanFrame_SCU_IPC_4_0x207;  //0x207;

typedef struct
{
 uint8 DriverDoorLockSt; // 
 uint8 DriverDoorAjarSt; //
 uint8 PsngrDoorAjarSt; // 
 uint8 RLDoorAjarSt; // 
 uint8 RRDoorAjarSt; //
 uint8 LTurnLampOutputSt; // 
 uint8 RTurnLampOutputSt; // 
 uint8 HazardLampOutputSt; //
 uint8 LowBeamOutputSt; // 
 uint8 HighBeamOutputSt; // 
 uint8 Horndriverst; //
 uint8 FrontWiperOutputSt; // 
 uint8 PowerMode; // 

   MSGPACK_DEFINE(DriverDoorLockSt, DriverDoorAjarSt, PsngrDoorAjarSt, RLDoorAjarSt, RRDoorAjarSt,
    LTurnLampOutputSt, RTurnLampOutputSt , HazardLampOutputSt, LowBeamOutputSt, 
    HighBeamOutputSt, Horndriverst, FrontWiperOutputSt, PowerMode);
}CanFrame_SCU_IPC_5_0x208;  //0x208;

typedef struct
{
 uint16 dstBat_Dsp; // 
 uint8 AccPedalSig; //
 uint8 BrkPedalSt; // 
 uint8 BrkPedalStVD; //
 uint8 CurrentGearLevVD; // 
 uint8 CurrentGearLev; //
 uint8 MsgCounter; // 
 uint8 Checksum; // 
   MSGPACK_DEFINE(dstBat_Dsp, AccPedalSig, BrkPedalSt, BrkPedalStVD, CurrentGearLevVD,
   CurrentGearLev, MsgCounter, Checksum);
}CanFrame_SCU_IPC_6_0x209;  //0x209;

typedef struct
{
 uint8 PSeatBeltWarning; // 
 uint8 DriverSeatBeltWarning; //
 uint8 _1ndLPSeatBeltWar; // 
 uint8 _1ndMPSeatBeltWar; //
 uint8 _1ndRPSeatBeltWar; // 
 uint8 TotalOdometerVD; //
 uint32 TotalOdometer; // 
 uint8 ACSt; // 
 uint8 ErrSt; // 
 uint8 OverrideRes; // 

   MSGPACK_DEFINE(PSeatBeltWarning, DriverSeatBeltWarning, _1ndLPSeatBeltWar, _1ndMPSeatBeltWar, _1ndRPSeatBeltWar,
   TotalOdometerVD, TotalOdometer, ACSt, ErrSt, OverrideRes);
}CanFrame_SCU_IPC_7_0x301;  //0x301;

///////////timerange/////////////////////////
//timestamptimerange
///////////////////////////////////////////////////
//State signal
typedef struct
{
   road_time_t time;
   road_timerange_t timerange;
   CanFrame_SCU_IPC_1_0x20A data;
     MSGPACK_DEFINE(time,timerange,data);
} TimestampedCanFrame_SCU_IPC_1_0x20A;    //0x20A

typedef struct
{
   road_time_t time;
   road_timerange_t timerange;
   CanFrame_SCU_IPC_2_0x205 data;
     MSGPACK_DEFINE(time,timerange,data);
} TimestampedCanFrame_SCU_IPC_2_0x205;      //0x205

typedef struct
{
   road_time_t time;
   road_timerange_t timerange;
   CanFrame_SCU_IPC_3_0x206 data;
     MSGPACK_DEFINE(time,timerange,data);
} TimestampedCanFrame_SCU_IPC_3_0x206;      //0x206

typedef struct
{
   road_time_t time;
   road_timerange_t timerange;
   CanFrame_SCU_IPC_4_0x207 data;
     MSGPACK_DEFINE(time,timerange,data);
} TimestampedCanFrame_SCU_IPC_4_0x207;      //0x207

typedef struct
{
   road_time_t time;
   road_timerange_t timerange;
   CanFrame_SCU_IPC_5_0x208 data;
     MSGPACK_DEFINE(time,timerange,data);
} TimestampedCanFrame_SCU_IPC_5_0x208;      //0x208

typedef struct
{
   road_time_t time;
   road_timerange_t timerange;
   CanFrame_SCU_IPC_6_0x209 data;
     MSGPACK_DEFINE(time,timerange,data);
} TimestampedCanFrame_SCU_IPC_6_0x209;      //0x209

typedef struct
{
   road_time_t time;
   road_timerange_t timerange;
   CanFrame_SCU_IPC_7_0x301 data;
     MSGPACK_DEFINE(time,timerange,data);
} TimestampedCanFrame_SCU_IPC_7_0x301;      //0x301

#endif 
