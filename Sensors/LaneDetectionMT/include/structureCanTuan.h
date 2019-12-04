//  created:    2017/12/15
//
//  author:     Kun JIANG
//              Copyright Tsinghua University
// 
//  version:    $Id: $
//
//  purpose:    

#ifndef __STRUCTURECANTUAN_H__
#define __STRUCTURECANTUAN_H__

#include <cstddef>
#include <cstdlib>
#include <string>
#include <sstream>
#include "CanFrame.h"
/// Vehicle speed CAN frame
/// frequency 10 Hz
/// 0.6 km/h minimum threshold



typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;
//=====================================================
typedef struct
{
float Speed;
float ax;
int data;


}veh_info;
typedef struct
{
 uint8 YawState; //12|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 axState; //13|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 ayState; //14|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float32 ay; //16|8@1+ (0.01,-1.27) [-1.27|1.27] "Unit_ForceOfGravi"  Frontradar,MQB_MFK_2
 float32 ax; //24|10@1+ (0.03125,-16) [-16|15.90625] "Unit_MeterPerSeconSquar"  Frontradar,MQB_MFK_2
 float32 YawRate; //40|14@1+ (0.01,0) [0|163.82] "Unit_DegreOfArcPerSecon"  Frontradar,MQB_MFK_2
 uint8 YawToRight; //54|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
} CanFrameTuanESP_02;//0x101
//=====================================================
typedef struct CanFrameTouranLH_EPS_03
{ 
 float32 EPSRxHCA_Status; //32|4@1+ (1,0) [0|15] ""  MQB_MFK_2
 float32 EPS_StrWhlTorque; //40|10@1+ (0.01,0) [0|8] "Unit_NewtoMeter"  Frontradar,MQB_MFK_2
 uint8 EPS_StrWhlTorqueDrt; //Steering wheel torque direction//54|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 EPS_StrWhlTorqueSt; //St=State//55|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
}CanFrameTouranLH_EPS_03;
//=====================================================
typedef  struct
{
           ///< low frequency vehicle speed (in km/h)

 uint8 BrakePressureState; //12|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float32 BrakePressure; //
	  ///< accurate odometer for car equiped with electrical brake (in cm)
} CanFrameTuanESP_05;//0x106;
//=====================================================
typedef  struct
{
 float32 LF_WhlSpdDrt;//Left Front_Wheel Speed direction; //56|2@1+ (1,0) [0|3] ""  Frontradar,MQB_MFK_2
 float32 LR_WhlSpdDrt; //Left Rear //58|2@1+ (1,0) [0|3] ""  Frontradar,MQB_MFK_2
 float32 RF_WhlSpdDrt; //60|2@1+ (1,0) [0|3] ""  Frontradar,MQB_MFK_2
 float32 RR_WhlSpdDrt; //62|2@1+ (1,0) [0|3] ""  Frontradar,MQB_MFK_2

} CanFrameTuanESP_10;//0x116;
//=====================================================

typedef struct CanFrameTouranACC_02
{ 
 float32 ACC_02CRC; //check//0|8@1+ (1,0) [0|255] ""  GatewayMQB,MQBMFK2
 float32 ACC_02BZ; //counter//8|4@1+ (1,0) [0|15] ""  GatewayMQB,MQBMFK2
 float32 ExpectedSpeed; //12|10@1+ (0.32,0) [0|327.04] "UnitKiloMeterPerHour"  GatewayMQB,MQBMFK2
 float32 SpacingFactor; //24|10@1+ (1,0) [1|1021] ""  GatewayMQB,MQBMFK2
 float32 NoiseAlarm; //34|2@1+ (1,0) [0|3] ""  GatewayMQB
 float32 SetSpacing; //37|3@1+ (1,0) [0|7] ""  GatewayMQB
 uint8 ImageAlarm; //40|1@1+ (1,0) [0|1] ""  GatewayMQB
 float32 ACC02Status; //61|3@1+ (1,0) [0|7] ""  GatewayMQB
}CanFrameTouranACC_02;
//=====================================================
typedef struct CanFrameTouranACC_06
{
 float32 ACC_06CRC; //0|8@1+ (1,0) [0|255] ""  GatewayMQB,MQBMFK2
 float32 ACC_06BZ; //8|4@1+ (1,0) [0|15] ""  GatewayMQB,MQBMFK2
 float32 aLowerDeviation; //16|6@1+ (0.024,0) [0|1.512] "UnitMeterPerSeconSquar"  GatewayMQB
 float32 aExpectedValue; //24|11@1+ (0.005,-7.22) [-7.22|3.005] "UnitMeterPerSeconSquar"  GatewayMQB
 float32 aUpperDeviation; //35|5@1+ (0.0625,0) [0|1.9375] "UnitMeterPerSeconSquar"  GatewayMQB
 float32 DclrtGradRmdValue; //Deceleration gradient Recommended value//40|8@1+ (0.05,0) [0|12.75] "UnitMeterPerCubicSecon"  GatewayMQB
 float32 AclrtGradRmdValue; //acceleration gradient Recommended value//48|8@1+ (0.05,0) [0|12.75] "UnitMeterPerCubicSecon"  GatewayMQB
 uint8 StartRequest06; //56|1@1+ (1,0) [0|1] ""  GatewayMQB
 uint8 ParkingRequest06; //57|1@1+ (1,0) [0|1] ""  GatewayMQB
 float32 ACC06Status; //60|3@1+ (1,0) [0|7] ""  GatewayMQB,MQBMFK2
}CanFrameTouranACC_06; 
//=====================================================
typedef struct CanFrameTouranACC_07
{
 float32 ACC_07CRC; //0|8@1+ (1,0) [0|255] ""  GatewayMQB
 float32 ACC_07BZ; //8|4@1+ (1,0) [0|15] ""  GatewayMQB
 float32 ParkingDistance; //12|11@1+ (0.01,0) [0|20.45] "UnitMeter"  GatewayMQB
 uint8 ParkingRequest07; //23|1@1+ (1,0) [0|1] ""  GatewayMQB
 uint8 StartRequest07; //24|1@1+ (1,0) [0|1] ""  GatewayMQB
 uint8 aExpValue; //acceleration deceleration expected value
}CanFrameTouranACC_07; 
//=====================================================
typedef struct CanFrameTouranPLA_01
{
 float32 PLA_CRC;//check//0|8@1+ (1,0) [0|255] "" Vector__XXX
 float32 PLA_BZ; //Counter//8|4@1+ (1,0) [0|15] "" Vector__XXX
 uint8 PLABrkRqtSt;//PLA braking request state //12|4@1+ (1,0) [0|15] "" Vector__XXX
 float32 PLAExpStrWhlAngle; //Expected Steering wheel angle//16|13@1+ (0.1,0) [0|819.1] "Unit_DegreOfArc" Vector__XXX
 uint8 PLAExpStrWhlAngleDrt; //31|1@1+ (1,0) [0|1] "" Vector__XXX
 float32 PLARequestStatus; //32|4@1+ (1,0) [0|15] "" Vector__XXX
 float32 PLABrkTorque; //36|13@1+ (4,0) [0|32760] "Unit_NewtoMeter" Vector__XXX
 float32 PLABrkDeceleration; //36|7@1+ (0.1,0) [0|12] "Unit_MeterPerSeconSquar" Vector__XXX
 uint8 PLABrkEnable; //43|1@1+ (1,0) [0|1] "" Vector__XXX
 uint8 BrkTrqAndDeceSwt;//brake torque and brake deceleration Switching //50|1@1+ (1,0) [0|1] "" Vector__XXX
 uint8 PLAParking; //51|1@1+ (1,0) [0|1] "" Vector__XXX
 float32 PLAPrkDistance; //52|11@1+ (0.01,0) [0.01|20.45] "Unit_Meter" Vector__XXX
 uint8 PLASignalTxCyclic; //63|1@1+ (1,0) [0|1] "" Vector__XXX
}CanFrameTouranPLA_01;
//=====================================================
typedef struct CanFrameTouranHCA_01
{
 float32 ExpStrWhlTorque;//Expected Steering wheel torque //16|9@1+ (0.01,0) [0|5.11] "Unit_NewtoMeter"  Gateway_MQB
 uint8 HCATranCycle;//Transmission cycle; //30|1@1+ (1,0) [0|1] ""  Gateway_MQB
 uint8 ExpStrWhlTorqueDrt;//direction //31|1@1+ (1,0) [0|1] ""  Gateway_MQB
 float32 HCA_Status; //32|4@1+ (1,0) [0|15] ""  Gateway_MQB
}CanFrameTouranHCA_01;
//=====================================================
typedef  struct
{

 float32 LF_WhlSpd; //0|16@1+ (0.0075,0) [0|491.49] "Unit_KiloMeterPerHour"  Frontradar,MQB_MFK_2
 float32 LR_WhlSpd; //16|16@1+ (0.0075,0) [0|491.49] "Unit_KiloMeterPerHour"  Frontradar,MQB_MFK_2
 float32 RF_WhlSpd; //32|16@1+ (0.0075,0) [0|491.49] "Unit_KiloMeterPerHour"  Frontradar,MQB_MFK_2
 float32 RR_WhlSpd; //48|16@1+ (0.0075,0) [0|491.49] "Unit_KiloMeterPerHour"  Frontradar,MQB_MFK_2
} CanFrameTuanESP_19;//0x0B2;
//=====================================================
typedef struct CanFrameTuanESP_33
{ 
 uint8 ACCSignalContinuity; //38|1@1+ (1,0) [0|1] ""  Frontradar
}CanFrameTuanESP_33;


//=====================================================

typedef  struct
{
 float32 Speed; //32|16@1+ (0.01,0) [0|655.32] "Unit_KiloMeterPerHour"  Frontradar,MQB_MFK_2
 uint8 ESP_SystemStatus; //50|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 SpeedState; //55|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
} CanFrameTuanESP_21;//0x0FD;
//=====================================================
typedef struct CanFrameTouranTSK_06
{
 float32 TSK_status;
}CanFrameTouranTSK_06;
//=====================================================
typedef struct CanFrameTouranMotor_20
{ 
 float32 aPedalPercent; //Percentage of accelerator pedal//12|8@1+ (0.4,0) [0|101.6] "Unit_PerCent"  Frontradar,MQB_MFK_2
 uint8 aPedalPercentSt; //20|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float32 ThrottleGradientSize; //21|8@1+ (25,0) [0|6350] "Unit_PerCentPerSecon" Vector__XXX
 uint8 ThrottleGradientPN;//positive and negative //29|1@1+ (1,0) [0|1] ""  Frontradar
 uint8 EngNeutralTorque; //engine neutral torque//37|1@1+ (1,0) [0|1] "" Vector__XXX
}CanFrameTouranMotor_20;
//=====================================================
typedef struct
{
 bool PicWarn; 
 bool AudioWarn; 
 bool SteeringToRight;
 float SteeringAngle;
 bool PLA_actif; 
 bool TorqueToRight; 
 float Torque; 
 bool HCA_aktif; 
 float Acceleration; 
 bool ACC_aktif; 
 
}GL1000Message;//ox6FF;
//=====================================================

typedef struct
{
 uint8 LWI_Sensorstatus; //12|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 LWI_StrWhlAngleSt; //steering wheel angle State//15|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float32 LWI_StrWhlAngleSize; //16|13@1+ (0.1,0) [0|800] "Unit_DegreOfArc"  Frontradar,MQB_MFK_2
 uint8 LWI_StrWhlAngleDrt; //29|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 LWI_StrWhlSpeedDrt; //30|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float32 LWI_StrWhlSpeedSize; //31|9@1+ (5,0) [0|2500] "Unit_DegreOfArcPerSecon"  Frontradar,MQB_MFK_2
 }CanFrameTuanLWI_01;//0x086;
//=====================================================

typedef struct CanFrameGear_11
{ 
 float32 Stalls; //42|4@1+ (1,0) [0|15] ""  Frontradar
 float32 TargetStalls; //60|4@1+ (1,0) [0|15] "" Vector__XXX
}CanFrameGear_11;
//=====================================================
typedef struct CanFrameGateway
{ 
 uint8 EngineSpeedState; //8|1@1+ (1,0) [0|1] ""  Frontradar
 float32 EngineSpeed; //16|16@1+ (0.25,0) [0|16383] "Unit_MinutInver"  Frontradar
}CanFrameGateway;
//=====================================================
typedef struct CanFrameTouranEPB_01
{
float32 EPBFaultStatus;
float32 EPBSwitch;
uint8 EPBSwitchState;
float32 EPB_PressingForce;
float32 EPB_Status;
}CanFrameTouranEPB_01;
//=====================================================
typedef struct CanFrameTouranMotor_14
{ 
 uint8 BrakeSwitch; //30|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
}CanFrameTouranMotor_14;
//=====================================================
typedef struct CanFrameTouranMotor_Code_01
{ 
 float32 EngTorqueCoefficient; //12|2@1+ (1,0) [1|3] "Unit_NewtoMeter" Vector__XXX
}CanFrameTouranMotor_Code_01;
//=====================================================
typedef struct CanFrameTouranOBD_01
{
 float32 ThtottlePosition; //16|8@1+ (0.392156862745098,0) [0|100] "Unit_PerCent" Vector__XXX
 float32 aPedalPosition; //40|8@1+ (0.392156862745098,0) [0|100] "Unit_PerCent" Vector__XXX
}CanFrameTouranOBD_01;
////////////////////////////////////////////////
//timestamp
///////////////////////////////////////////////////

typedef struct
{
   road_time_t time;
   road_timerange_t timerange;
   CanFrameTuanESP_02 data;
} TimestampedCanFrameTuanESP_02;//0x101

typedef  struct
{
    road_time_t time;
    road_timerange_t timerange;       ///< low frequency vehicle speed (in km/h)
	CanFrameTuanESP_05 data;
	  ///< accurate odometer for car equiped with electrical brake (in cm)
} TimestampedCanFrameTuanESP_05;//0x106;
typedef  struct
{
    road_time_t time;
    road_timerange_t timerange;
	CanFrameTuanESP_10 data;


} TimestampedCanFrameTuanESP_10;//0x116;
typedef  struct
{

    road_time_t time;
    road_timerange_t timerange;
	CanFrameTuanESP_19 data;

} TimestampedCanFrameTuanESP_19;//0x0B2;
typedef  struct
{
    road_time_t time;
    road_timerange_t timerange;    ///< low frequency vehicle speed (in km/h)

	CanFrameTuanESP_21 data;
} TimestampedCanFrameTuanESP_21;//0x0FD;

typedef struct
{
    road_time_t time;
    road_timerange_t timerange;
	GL1000Message data;
 
}TimestampedGL1000Message;//ox6FF;

typedef struct
{
    road_time_t time;
    road_timerange_t timerange;
	CanFrameTuanLWI_01 data;
 }TimestampedCanFrameTuanLWI_01;//0x086;





#endif 
