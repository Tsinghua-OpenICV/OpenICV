/**
 * @file auto_control_node.cpp
 * @brief xiaopeng G3 auto control message:
 * 	IPC_SCU_1_0x106.msg
 * 	IPC_SCU_2_0x102.msg
 * 	IPC_SCU_3_0x103.msg 
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-07-10
 */

#include <iostream>
#include <string>
#include <numeric>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "param.h"
#include "canbus_msgs/IPC_SCU_1_0x106.h"
#include "canbus_msgs/IPC_SCU_2_0x102.h"
#include "canbus_msgs/IPC_SCU_3_0x103.h"
#include "XUdp.h"
#include "CanFrame.h"

using namespace ros;
using namespace std;

string ip_addr;
unsigned int ip_port;


uint8_t CalculateCheckSum(const uint8_t *input, const uint32_t length) {
  return static_cast<uint8_t>(std::accumulate(input, input + length, 0) ^ 0xFF);
}

/**
 * @brief IPC_SCU_1.
 *
 * @param msg : canbus_msgs::IPC_SCU_1_0x106
 */
void
SendCanFrame_IPC_SCU_1 (const canbus_msgs::IPC_SCU_1_0x106 & msg)
{
  uint8_t DBWReq;
  if (msg.IPC_SCU_DBWReq >= 0 && msg.IPC_SCU_DBWReq <= 15)
    DBWReq = msg.IPC_SCU_DBWReq;

  uint8_t ReadyReq;
  if (msg.IPC_SCU_ReadyReq == 0x00 || msg.IPC_SCU_ReadyReq == 0x01)
    ReadyReq = msg.IPC_SCU_ReadyReq;

  uint8_t BrakeLight;
  if (msg.IPC_SCU_BrakeLight >= 0 && msg.IPC_SCU_BrakeLight <= 3)
    BrakeLight = msg.IPC_SCU_BrakeLight;

  uint16_t SteerAngleReq = round((msg.IPC_SCU_SteerAngleReq+780) * 10);    //-780～779.9
  if(SteerAngleReq  > 15599) 
    SteerAngleReq = 15599;

  uint8_t SteerAngleReqVD;
  if (msg.IPC_SCU_SteerAngleReqVD == 0x00 || msg.IPC_SCU_SteerAngleReqVD == 0x01)
    SteerAngleReqVD = msg.IPC_SCU_SteerAngleReqVD;

  uint16_t TorsionBarTqReq = round((msg.IPC_SCU_TorsionBarTqReq+10.24) * 100);    //-10.24～10.23
  if(TorsionBarTqReq  > 2047) 
    TorsionBarTqReq = 2047;

  uint8_t TorsionBarTqReqVD;
  if (msg.IPC_SCU_TorsionBarTqReqVD == 0x00 || msg.IPC_SCU_TorsionBarTqReqVD == 0x01)
    TorsionBarTqReqVD = msg.IPC_SCU_TorsionBarTqReqVD;

  uint8_t GearReq;
  if (msg.IPC_SCU_GearReq >= 0 && msg.IPC_SCU_GearReq <= 4)
    GearReq = msg.IPC_SCU_GearReq;

  static int MsgCounter = 0;
  if(MsgCounter > 0x0f){
    MsgCounter = 0;
  }

  uint8_t Buf106[13];
  XUdp xudp;

  //Msg_ID:0x106
  //=======================0f0============================
  memset (Buf106, 0, sizeof (Buf106));
  Buf106[0] = 0x08;		//0x08
  Buf106[1] = 0x00;
  Buf106[2] = 0x00;
  Buf106[3] = 0x01;
  Buf106[4] = 0x06;   //0x0106

  //////////////data segment/////////////
  Buf106[5] = DBWReq << 4;      //Motorola MSB
  Buf106[7] = (ReadyReq << 6) | (BrakeLight << 4) | ((SteerAngleReq >> 12) & 0x000f );
  Buf106[8] = (SteerAngleReq >> 4) & 0x00ff;
  Buf106[9] = ((SteerAngleReq & 0x000f) << 4) | (SteerAngleReqVD << 3) | ((TorsionBarTqReq >> 8) & 0x0007);
  Buf106[10] = TorsionBarTqReq & 0x00ff;
  Buf106[11] = (TorsionBarTqReqVD << 7) | (GearReq << 4) | MsgCounter;
  MsgCounter++;
  uint8_t Checksum = CalculateCheckSum(&Buf106[5], 7);
  // uint8_t Checksum = 0xff ^ (Buf106[5] + Buf106[6] + Buf106[7] 
  //                             + Buf106[8] + Buf106[9] + Buf106[10] + Buf106[11]);
  Buf106[12] = Checksum;  

  int len = xudp.Send (ip_addr.c_str (), ip_port, Buf106, 13);	// Send to XBR.
  if(len < 0)
    ROS_WARN("canbus - socket send fault data.");

  // ROS_INFO("extl acceleration demand H: %u", (uint16_t)Buf106[5]);
  // ROS_INFO("extl acceleration demand L: %u", (uint16_t)Buf106[6]);
  // ROS_INFO("-------Send IPC_SCU_1_0x106 message ---------");
}


/**
 * @brief IPC_SCU_2.
 *
 * @param msg : canbus_msgs::IPC_SCU_2_0x102
 */
void
SendCanFrame_IPC_SCU_2 (const canbus_msgs::IPC_SCU_2_0x102 & msg)
{
  uint16_t MotorTorqReq = round((msg.IPC_SCU_MotorTorqReq+200) * 2);    //-200～311.5
  if(MotorTorqReq  > 1023) 
    MotorTorqReq = 1023;

  uint8_t MotorTorqReqVD;
  if (msg.IPC_SCU_MotorTorqReqVD == 0x00 || msg.IPC_SCU_MotorTorqReqVD == 0x01)
    MotorTorqReqVD = msg.IPC_SCU_MotorTorqReqVD;

  uint8_t ParkingReqToEPBVD;
  if (msg.IPC_SCU_ParkingReqToEPBVD == 0x00 || msg.IPC_SCU_ParkingReqToEPBVD == 0x01)
    ParkingReqToEPBVD = msg.IPC_SCU_ParkingReqToEPBVD;

  uint8_t ParkingReqToEPB;
  if (msg.IPC_SCU_ParkingReqToEPB >= 0 && msg.IPC_SCU_ParkingReqToEPB <= 3)
    ParkingReqToEPB = msg.IPC_SCU_ParkingReqToEPB;

  uint16_t AccDecelReq = round((msg.IPC_SCU_AccDecelReq+15) * 50);    //-15～5
  if(AccDecelReq  > 1000) 
    AccDecelReq = 1000;

  uint8_t AccDecelReqVD;
  if (msg.IPC_SCU_AccDecelReqVD == 0x00 || msg.IPC_SCU_AccDecelReqVD == 0x01)
    AccDecelReqVD = msg.IPC_SCU_AccDecelReqVD;

  uint16_t VehSpd = round((msg.IPC_SCU_VehSpd+50) * 10);    //-50～120
  if(VehSpd  > 1700) 
    VehSpd = 1700;

  uint8_t VehSpdVD;
  if (msg.IPC_SCU_VehSpdVD == 0x00 || msg.IPC_SCU_VehSpdVD == 0x01)
    VehSpdVD = msg.IPC_SCU_VehSpdVD;

  static int MsgCounter = 0;
  if(MsgCounter > 0x0f){
    MsgCounter = 0;
  }

  uint8_t Buf102[13];
  XUdp xudp;

  //Msg_ID:0x102
  //=======================0f0============================
  memset (Buf102, 0, sizeof (Buf102));
  Buf102[0] = 0x08;		//0x08
  Buf102[1] = 0x00;
  Buf102[2] = 0x00;
  Buf102[3] = 0x01;
  Buf102[4] = 0x02;   //0x0102

  //////////////data segment/////////////
  Buf102[5] = (MotorTorqReq >> 2) & 0x00ff;      //Motorola MSB
  Buf102[6] = ((MotorTorqReq & 0x0003) << 6) | (MotorTorqReqVD << 5) | (ParkingReqToEPBVD << 4) |
       ((ParkingReqToEPB & 0x03) << 2) | ((AccDecelReq >> 8) & 0x0003);
  Buf102[7] = AccDecelReq & 0x00ff;
  Buf102[8] = (AccDecelReqVD << 7)  | ((VehSpd  >> 6)& 0x007f);
  Buf102[9] = (VehSpd & 0x003f << 2) | (VehSpdVD << 1);
  Buf102[11] = MsgCounter;
  MsgCounter++;
  uint8_t Checksum = CalculateCheckSum(&Buf102[5], 7);
  // uint8_t Checksum = 0xff ^ (Buf102[5] + Buf102[6] + Buf102[7] 
  //                             + Buf102[8] + Buf102[9] + Buf102[10] + Buf102[11]);  
  Buf102[12] = Checksum;

  int len = xudp.Send (ip_addr.c_str (), ip_port, Buf102, 13);	// Send to XBR.
  if(len < 0)
    ROS_WARN("canbus - socket send fault data.");

  // ROS_INFO("extl acceleration demand H: %u", (uint16_t)Buf102[5]);
  // ROS_INFO("extl acceleration demand L: %u", (uint16_t)Buf102[6]);
  // ROS_INFO("-------Send IPC_SCU_2_0x102 message ---------");
}

/**
 * @brief IPC_SCU_3.
 *
 * @param msg : canbus_msgs::IPC_SCU_3_0x103
 */
void
SendCanFrame_IPC_SCU_3 (const canbus_msgs::IPC_SCU_3_0x103 & msg)
{
  uint8_t LowBeamSWSt;
  if (msg.IPC_SCU_LowBeamSWSt == 0x00 || msg.IPC_SCU_LowBeamSWSt == 0x01)
    LowBeamSWSt = msg.IPC_SCU_LowBeamSWSt;

  uint8_t LowBeamSWStVD;
  if (msg.IPC_SCU_LowBeamSWStVD == 0x00 || msg.IPC_SCU_LowBeamSWStVD == 0x01)
    LowBeamSWStVD = msg.IPC_SCU_LowBeamSWStVD;

  uint8_t HighBeamSWSt;
  if (msg.IPC_SCU_HighBeamSWSt == 0x00 || msg.IPC_SCU_HighBeamSWSt == 0x01)
    HighBeamSWSt = msg.IPC_SCU_HighBeamSWSt;

  uint8_t HighBeamSWStVD;
  if (msg.IPC_SCU_HighBeamSWStVD == 0x00 || msg.IPC_SCU_HighBeamSWStVD == 0x01)
    HighBeamSWStVD = msg.IPC_SCU_HighBeamSWStVD;

  uint8_t HazardLampSWSt;
  if (msg.IPC_SCU_HazardLampSWSt >= 0 && msg.IPC_SCU_HazardLampSWSt <= 3)
    HazardLampSWSt = msg.IPC_SCU_HazardLampSWSt;

  uint8_t LTurnLampSWSt;
  if (msg.IPC_SCU_LTurnLampSWSt >= 0 && msg.IPC_SCU_LTurnLampSWSt <= 3)
    LTurnLampSWSt = msg.IPC_SCU_LTurnLampSWSt;

  uint8_t RTurnLampSWSt;
  if (msg.IPC_SCU_RTurnLampSWSt >= 0 && msg.IPC_SCU_RTurnLampSWSt <= 3)
    RTurnLampSWSt = msg.IPC_SCU_RTurnLampSWSt;

  uint8_t HorndriverSWst;
  if (msg.IPC_SCU_HorndriverSWst >= 0 && msg.IPC_SCU_HorndriverSWst <= 3)
    HorndriverSWst = msg.IPC_SCU_HorndriverSWst;

  uint8_t Buf103[13];
  XUdp xudp;

  //Msg_ID:0x103
  //=======================0f0============================
  memset (Buf103, 0, sizeof (Buf103));
  Buf103[0] = 0x08;		//0x08
  Buf103[1] = 0x00;
  Buf103[2] = 0x00;
  Buf103[3] = 0x01;
  Buf103[4] = 0x03;   //0x0103

  //////////////data segment/////////////
  Buf103[5] =  (HazardLampSWSt << 6) | (LTurnLampSWSt << 4) | (HighBeamSWStVD << 3) |
       (HighBeamSWSt << 2) | (LowBeamSWStVD << 1) | LowBeamSWSt ;      //Motorola MSB
  Buf103[6] = (RTurnLampSWSt << 6) | (HorndriverSWst << 4);

  int len = xudp.Send (ip_addr.c_str (), ip_port, Buf103, 13);	// Send to XBR.
  if(len < 0)
    ROS_WARN("canbus - socket send fault data.");

  // ROS_INFO("extl acceleration demand H: %u", (uint16_t)Buf103[5]);
  // ROS_INFO("extl acceleration demand L: %u", (uint16_t)Buf103[6]);
  // ROS_INFO("-------Send IPC_SCU_3_0x103 message ---------");
}

//all callback function
void
callback_IPC_SCU_1(const canbus_msgs::IPC_SCU_1_0x106 & msg)
{
  // ROS_INFO("received messages IPC_SCU_1_0x106:");
  // ROS_INFO("Parameter provided to the brake system from external sources: %f", msg.ExtlAccelerationDemand);

  SendCanFrame_IPC_SCU_1(msg);
}

void
callback_IPC_SCU_2(const canbus_msgs::IPC_SCU_2_0x102 & msg)
{
  // ROS_INFO("received messages IPC_SCU_2_0x102:");

  SendCanFrame_IPC_SCU_2(msg);
}

void
callback_IPC_SCU_3(const canbus_msgs::IPC_SCU_3_0x103 & msg)
{
  // ROS_INFO("received messages IPC_SCU_3_0x103:");

  SendCanFrame_IPC_SCU_3(msg);
}

int
main (int argc, char **argv)
{
  init (argc, argv, "auto_control_node");
  NodeHandle n;

  ip_addr = getParam < string > ("auto_control/addr", "192.168.110.101");
  ip_port = getParam < int >("auto_control/port", 4001);

  Subscriber sub_xbr = n.subscribe ("/canbus/IPC_SCU_1", 1000, callback_IPC_SCU_1);
  Subscriber sub_auto_control = n.subscribe ("/canbus/IPC_SCU_2", 1000, callback_IPC_SCU_2);
  Subscriber sub_auto_control1 = n.subscribe ("/canbus/IPC_SCU_3", 1000, callback_IPC_SCU_3);
  ros::MultiThreadedSpinner s(3);   //5 ros threads for all subscribers.
  ros::spin(s);
  return 0;
}
