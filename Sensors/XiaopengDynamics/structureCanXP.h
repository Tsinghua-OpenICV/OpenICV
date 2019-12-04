//  created:    2017/12/15
//
//  author:     Kun JIANG
//              Copyright Tsinghua University
// 
//  version:    $Id: $
//
//  purpose:    

#ifndef __STRUCTURECANXP_H__
#define __STRUCTURECANXP_H__

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

//update message

//wheel speed  factor 0.01 range (-300 300)
typedef  struct
{

 int16 LF_WhlSpd; //
 int16 LR_WhlSpd; //
 int16 RF_WhlSpd; //
 int16 RR_WhlSpd; //
 MSGPACK_DEFINE(LF_WhlSpd,LR_WhlSpd,RF_WhlSpd,RR_WhlSpd);
} CanFrameSPD_510;//0x510;


//EPS angle speed and torque  factor 0.02 range (-600 600)
typedef  struct
{
 int16 EPS_StrngWhlTorq; //
 int16 EPS_angle_spd_ccp;//
 int16 EPS_angle_ccp;//
  MSGPACK_DEFINE(EPS_StrngWhlTorq,EPS_angle_spd_ccp,EPS_angle_ccp);

} CanFrameEPS_511;//0x511;

//state of braking and light  brake (0  255)  light (0 3)  drivemode(0 3)
typedef struct
{
 uint8 BT_TranCycle;//BT cycle;
 uint8 State_Braking_CCP; //
 uint8 State_TurningLight_CCP; //
 uint8 CurDriveMode_CCP; //

   MSGPACK_DEFINE(BT_TranCycle,State_Braking_CCP,State_TurningLight_CCP,CurDriveMode_CCP);

}CanFrameBT_509;//0x509;

//state of accpedal brkpadal eps and gear
typedef struct 
{ 
 uint8 Motor_TranCycle;//Motor cycle;
 uint8 AccPedal_CCP; //
 uint8 BrkPedal_CCP; //
 uint8 EpbState_CCP;//
 uint8 GearState_CCP; //
MSGPACK_DEFINE(Motor_TranCycle,AccPedal_CCP,BrkPedal_CCP,EpbState_CCP,GearState_CCP);

}CanFrameMotor_50B;//0x50B;

//Communtication order
typedef struct 
{
 int16 SCU_TarSpeed_Req;//Expected wheel speed
 int16 SCU_EPSAngle_Req;//Expected eps angle
 uint8 Com_AutoMode;//auto mode switch
 uint8 Com_TurnLight;//turn light
 uint8 Com_VoiceAlarm;//voice
 MSGPACK_DEFINE(SCU_TarSpeed_Req,SCU_EPSAngle_Req,Com_AutoMode,Com_TurnLight,Com_VoiceAlarm);

}CanFrameCOM_5F0;//0x5F0;


///////////timerange/////////////////////////
//timestamptimerange
///////////////////////////////////////////////////

typedef struct
{
   road_time_t time;
   road_timerange_t timerange;
   CanFrameSPD_510 data;
     MSGPACK_DEFINE(time,timerange,data);

} TimestampedCanFrameSPD_510;//0x510

typedef  struct
{
    road_time_t time;
    road_timerange_t timerange;       ///< low frequency vehicle speed (in km/h)
	CanFrameEPS_511 data;
         MSGPACK_DEFINE(time,timerange,data);

} TimestampedCanFrameEPS_511;//0x511;

typedef  struct
{
    road_time_t time;
    road_timerange_t timerange;
	CanFrameBT_509 data;
     MSGPACK_DEFINE(time,timerange,data);

} TimestampedCanFrameBT_509;//0x509;

typedef  struct
{
    road_time_t time;
    road_timerange_t timerange;
	CanFrameMotor_50B data;
         MSGPACK_DEFINE(time,timerange,data);

} TimestampedCanFrameMotor_50B;//0x50B;

typedef  struct
{
    road_time_t time;
    road_timerange_t timerange;  
	CanFrameCOM_5F0 data;
         MSGPACK_DEFINE(time,timerange,data);

} TimestampedCanFrameCOM_5F0;//0x5F0;



#endif 
