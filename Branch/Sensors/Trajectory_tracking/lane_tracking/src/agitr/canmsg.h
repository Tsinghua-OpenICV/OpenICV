//  @ Project : CAN
//  @ File Name : canmsg.h
//  @ Date : 2018/2/1
//  @ Author :
//
//


#ifndef _CANMESSAGES_H
#define _CANMESSAGES_H

#include <cstdlib>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "agitr/can.h"
#include <visualization_msgs/MarkerArray.h>
#include "boost_udp.h"
#include "std_msgs/Float32.h"
#define PI 3.1415926


typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32; 



typedef struct ControlMsg  //recive messages and control 
{
  float32 acclerationIn;
  float32 steeringAngleIn;
}ControlMsg;

typedef struct _CANMsg
{
  uint8 DLC      :4;//Data length,0<=DLC<=8
  uint8 reserved :2;
  uint8 RTR      :1;//1 for remote request
  uint8 FF       :1;//Frame form. 1 = Extended, 0 = Standard
  uint32 ID;//ID of message
  uint8 data[8];//Data of message
}stCANMsg;
 

typedef struct TouranLWI_01
{ 
 uint8 LWI_Sensorstatus; //12|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 LWI_StrWhlAngleSt; //steering wheel angle State//15|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float32 LWI_StrWhlAngleSize; //16|13@1+ (0.1,0) [0|800] "Unit_DegreOfArc"  Frontradar,MQB_MFK_2
 uint8 LWI_StrWhlAngleDrt; //29|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 LWI_StrWhlSpeedDrt; //30|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float32 LWI_StrWhlSpeedSize; //31|9@1+ (5,0) [0|2500] "Unit_DegreOfArcPerSecon"  Frontradar,MQB_MFK_2
}TouranLWI_01;
//=====================================================
typedef struct TouranGetriebe_11
{ 
 float32 Stalls; //42|4@1+ (1,0) [0|15] ""  Frontradar
 float32 TargetStalls; //60|4@1+ (1,0) [0|15] "" Vector__XXX
}TouranGetriebe_11;
//=====================================================
typedef struct TouranGateway_73
{ 
 uint8 EngineSpeedState; //8|1@1+ (1,0) [0|1] ""  Frontradar
 float32 EngineSpeed; //16|16@1+ (0.25,0) [0|16383] "Unit_MinutInver"  Frontradar
}TouranGateway_73;
//=====================================================
typedef struct TouranESP_33
{ 
 uint8 ACCSignalContinuity; //38|1@1+ (1,0) [0|1] ""  Frontradar
}TouranESP_33;
//=====================================================
typedef struct TouranESP_21
{ 
 float32 Speed; //32|16@1+ (0.01,0) [0|655.32] "Unit_KiloMeterPerHour"  Frontradar,MQB_MFK_2
 uint8 ESP_SystemStatus; //50|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 SpeedState; //55|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
}TouranESP_21;
//=====================================================
typedef struct TouranESP_19
{ 
 float32 LF_WhlSpd; //0|16@1+ (0.0075,0) [0|491.49] "Unit_KiloMeterPerHour"  Frontradar,MQB_MFK_2
 float32 LR_WhlSpd; //16|16@1+ (0.0075,0) [0|491.49] "Unit_KiloMeterPerHour"  Frontradar,MQB_MFK_2
 float32 RF_WhlSpd; //32|16@1+ (0.0075,0) [0|491.49] "Unit_KiloMeterPerHour"  Frontradar,MQB_MFK_2
 float32 RR_WhlSpd; //48|16@1+ (0.0075,0) [0|491.49] "Unit_KiloMeterPerHour"  Frontradar,MQB_MFK_2
}TouranESP_19;
//=====================================================
typedef struct TouranMotor_20
{ 
 float32 aPedalPercent; //Percentage of accelerator pedal//12|8@1+ (0.4,0) [0|101.6] "Unit_PerCent"  Frontradar,MQB_MFK_2
 uint8 aPedalPercentSt; //20|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float32 ThrottleGradientSize; //21|8@1+ (25,0) [0|6350] "Unit_PerCentPerSecon" Vector__XXX
 uint8 ThrottleGradientPN;//positive and negative //29|1@1+ (1,0) [0|1] ""  Frontradar
 uint8 EngNeutralTorque; //engine neutral torque//37|1@1+ (1,0) [0|1] "" Vector__XXX
}TouranMotor_20;
//=====================================================
typedef struct TouranMotor_14
{ 
 uint8 BrakeSwitch; //30|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
}TouranMotor_14;
//=====================================================
typedef struct TouranLH_EPS_03
{ 
 float32 EPSRxHCA_Status; //32|4@1+ (1,0) [0|15] ""  MQB_MFK_2
 float32 EPS_StrWhlTorque; //40|10@1+ (0.01,0) [0|8] "Unit_NewtoMeter"  Frontradar,MQB_MFK_2
 uint8 EPS_StrWhlTorqueDrt; //Steering wheel torque direction//54|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 EPS_StrWhlTorqueSt; //St=State//55|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
}TouranLH_EPS_03;
//=====================================================
typedef struct TouranHCA_01
{
 float32 ExpStrWhlTorque;//Expected Steering wheel torque //16|9@1+ (0.01,0) [0|5.11] "Unit_NewtoMeter"  Gateway_MQB
 uint8 HCATranCycle;//Transmission cycle; //30|1@1+ (1,0) [0|1] ""  Gateway_MQB
 uint8 ExpStrWhlTorqueDrt;//direction //31|1@1+ (1,0) [0|1] ""  Gateway_MQB
 float32 HCA_Status; //32|4@1+ (1,0) [0|15] ""  Gateway_MQB
}TouranHCA_01;
//=====================================================
typedef struct TouranMotor_Code_01
{ 
 float32 EngTorqueCoefficient; //12|2@1+ (1,0) [1|3] "Unit_NewtoMeter" Vector__XXX
}TouranMotor_Code_01;
//=====================================================
typedef struct TouranOBD_01
{
 float32 ThtottlePosition; //16|8@1+ (0.392156862745098,0) [0|100] "Unit_PerCent" Vector__XXX
 float32 aPedalPosition; //40|8@1+ (0.392156862745098,0) [0|100] "Unit_PerCent" Vector__XXX
}TouranOBD_01;
//=====================================================
typedef struct TouranPLA_01
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
}TouranPLA_01;
//=====================================================

typedef struct TouranESP_02
{ 
 uint8 YawState; //12|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 axState; //13|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 uint8 ayState; //14|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float32 ay; //16|8@1+ (0.01,-1.27) [-1.27|1.27] "Unit_ForceOfGravi"  Frontradar,MQB_MFK_2
 float32 ax; //24|10@1+ (0.03125,-16) [-16|15.90625] "Unit_MeterPerSeconSquar"  Frontradar,MQB_MFK_2
 float32 YawRate; //40|14@1+ (0.01,0) [0|163.82] "Unit_DegreOfArcPerSecon"  Frontradar,MQB_MFK_2
 uint8 YawToRight; //54|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
}TouranESP_02;
//==========================================
typedef struct TouranESP_05
{ 
 uint8 BrakePressureState; //12|1@1+ (1,0) [0|1] ""  Frontradar,MQB_MFK_2
 float32 BrakePressure; //16|10@1+ (0.3,-30) [-30|276.6] "Unit_Bar"  Frontradar,MQB_MFK_2
}TouranESP_05;
//==========================================
typedef struct TouranESP_10
{
 float32 LF_WhlSpdDrt;//Left Front_Wheel Speed direction; //56|2@1+ (1,0) [0|3] ""  Frontradar,MQB_MFK_2
 float32 LR_WhlSpdDrt; //Left Rear //58|2@1+ (1,0) [0|3] ""  Frontradar,MQB_MFK_2
 float32 RF_WhlSpdDrt; //60|2@1+ (1,0) [0|3] ""  Frontradar,MQB_MFK_2
 float32 RR_WhlSpdDrt; //62|2@1+ (1,0) [0|3] ""  Frontradar,MQB_MFK_2
}TouranESP_10; 

//==========================================
typedef struct TouranACC_02
{ 
 float32 ACC_02CRC; //check//0|8@1+ (1,0) [0|255] ""  GatewayMQB,MQBMFK2
 float32 ACC_02BZ; //counter//8|4@1+ (1,0) [0|15] ""  GatewayMQB,MQBMFK2
 float32 ExpectedSpeed; //12|10@1+ (0.32,0) [0|327.04] "UnitKiloMeterPerHour"  GatewayMQB,MQBMFK2
 float32 SpacingFactor; //24|10@1+ (1,0) [1|1021] ""  GatewayMQB,MQBMFK2
 float32 NoiseAlarm; //34|2@1+ (1,0) [0|3] ""  GatewayMQB
 float32 SetSpacing; //37|3@1+ (1,0) [0|7] ""  GatewayMQB
 uint8 ImageAlarm; //40|1@1+ (1,0) [0|1] ""  GatewayMQB
 float32 ACC02Status; //61|3@1+ (1,0) [0|7] ""  GatewayMQB
}TouranACC_02;

//==========================================
typedef struct TouranACC_06
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
}TouranACC_06; 
//=========================================
typedef struct TouranACC_07
{
 float32 ACC_07CRC; //0|8@1+ (1,0) [0|255] ""  GatewayMQB
 float32 ACC_07BZ; //8|4@1+ (1,0) [0|15] ""  GatewayMQB
 float32 ParkingDistance; //12|11@1+ (0.01,0) [0|20.45] "UnitMeter"  GatewayMQB
 uint8 ParkingRequest07; //23|1@1+ (1,0) [0|1] ""  GatewayMQB
 uint8 StartRequest07; //24|1@1+ (1,0) [0|1] ""  GatewayMQB
 uint8 aExpValue; //acceleration deceleration expected value
}TouranACC_07; 
//=========================================
typedef struct TouranTSK_06
{
 float32 TSK_status;
}TouranTSK_06;
//=========================================
typedef struct TouranEPB_01
{
float32 EPBFaultStatus;
float32 EPBSwitch;
uint8 EPBSwitchState;
float32 EPB_PressingForce;
float32 EPB_Status;
}TouranEPB_01;
//=========================================
class canmessages
{
public:
  canmessages(ros::NodeHandle nh);
  ~canmessages();
  void run(float acclerationIn, float steeringAngleIn);  // the entry


private:
  void initUdp();

  
  void runCanMain();
  bool recCanDate(uint16);  
  void pubLrrRadarData();
  void sendCanConfig();
  //=================================
  void CANbusLWI_01(stCANMsg *frame);
  void CANbusGetriebe_11(stCANMsg *frame);
  void CANbusGateway_73(stCANMsg *frame);
  void CANbusESP_33(stCANMsg *frame);
  void CANbusESP_21(stCANMsg *frame);
  void CANbusESP_19(stCANMsg *frame);
  void CANbusMotor_20(stCANMsg *frame);
  void CANbusMotor_14(stCANMsg *frame);
  void CANbusLH_EPS_03(stCANMsg *frame);
  void CANbusHCA_01(stCANMsg *frame);
  void CANbusMotor_Code_01(stCANMsg *frame);
  void CANbusOBD_01(stCANMsg *frame);
  void CANbusPLA_01(stCANMsg *frame);

  //==================================
  void CANbusESP_02(stCANMsg *frame);
  void CANbusESP_05(stCANMsg *frame);
  void CANbusESP_10(stCANMsg *frame);
  void CANbusACC_02(stCANMsg *frame);
  void CANbusACC_06(stCANMsg *frame);
  void CANbusACC_07(stCANMsg *frame);
  void CANbusTSK_06(stCANMsg *frame);
  void CANbusEPB_01(stCANMsg *frame);
  //==================================
  
  float accControl;
  float steeringAngleControl;
  int accActive;
  int steerActive;
  int steerOri;
  bool flagaa;

private:
  static const int SendPeriod = 20; //ms
  static const int LrrMaxTarNum = 256;

  string ipHost;
  int portHost;
  string ipTarget;
  int portTarget;
 

  static const int MaxUdpBufferSize = 1024;
  uint8 udpBuffer[MaxUdpBufferSize]; //received udp buffer

  //==========================================
  TouranLWI_01 LWI_01;
  TouranGetriebe_11 Getriebe_11 ;
  TouranGateway_73 Gateway_73;
  TouranESP_33 ESP_33;
  TouranESP_21 ESP_21;
  TouranESP_19 ESP_19;
  TouranMotor_20 Motor_20;
  TouranMotor_14 Motor_14;
  TouranLH_EPS_03 LH_EPS_03;
  TouranHCA_01 HCA_01;
  TouranMotor_Code_01 Motor_Code_01;
  TouranOBD_01 OBD_01;
  TouranPLA_01 PLA_01;
  //============================================
  TouranESP_02 ESP_02;
  TouranESP_05 ESP_05;
  TouranESP_10 ESP_10;
  TouranACC_02 ACC_02;
  TouranACC_06 ACC_06;
  TouranACC_07 ACC_07;
  TouranTSK_06 TSK_06;
  TouranEPB_01 EPB_01;
  //============================================
  Boost_UDP *boostUdp;

  ros::Publisher pub_can;//pub_markerArray;

   
};



#endif  //_canmessages_H
