/**
 * @file auto_state_node.cpp
 * @brief xiaopeng G3 feedback message::
 * 	SCU_IPC_1_0x20A.msg
 * 	SCU_IPC_2_0x205.msg
 * 	SCU_IPC_3_0x206.msg 
 * 	SCU_IPC_4_0x207.msg
 * 	SCU_IPC_5_0x208.msg
 * 	SCU_IPC_6_0x209.msg 
 * 	SCU_IPC_7_0x301.msg 
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2019-7-13
 */

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <numeric>
#include "canbus_msgs/SCU_IPC_1_0x20A.h"
#include "canbus_msgs/SCU_IPC_2_0x205.h"
#include "canbus_msgs/SCU_IPC_3_0x206.h"
#include "canbus_msgs/SCU_IPC_4_0x207.h"
#include "canbus_msgs/SCU_IPC_5_0x208.h"
#include "canbus_msgs/SCU_IPC_6_0x209.h"
#include "canbus_msgs/SCU_IPC_7_0x301.h"
#include "XUdp.h"
#include "param.h"

// CanNet
#include "structureCanXP.h"
#include "CanFrame.h"
#include "receive/Can_SCU_IPC_1_0x20A.hpp"
#include "receive/Can_SCU_IPC_2_0x205.hpp"
#include "receive/Can_SCU_IPC_3_0x206.hpp"
#include "receive/Can_SCU_IPC_4_0x207.hpp"
#include "receive/Can_SCU_IPC_5_0x208.hpp"
#include "receive/Can_SCU_IPC_6_0x209.hpp"
#include "receive/Can_SCU_IPC_7_0x301.hpp"
 
using namespace ros;
using namespace std;

string ipAddr;
unsigned int ipPort;

uint8_t CalculateCheckSum(const uint8_t *input, const uint32_t length) {
  return static_cast<uint8_t>(std::accumulate(input, input + length, 0) ^ 0xFF);
}

int
main (int argc, char **argv)
{
  init (argc, argv, "auto_state_node");
  NodeHandle n;

  XUdp xudp;
  uint8 buf[512] = { 0 };
  char ip[100] = { 0 };
  int len;

  // Received from cannet
  Can_SCU_IPC_1_0x20A SCU_IPC_1_0x20A;
  Can_SCU_IPC_2_0x205 SCU_IPC_2_0x205;
  Can_SCU_IPC_3_0x206 SCU_IPC_3_0x206;
  Can_SCU_IPC_4_0x207 SCU_IPC_4_0x207;
  Can_SCU_IPC_5_0x208 SCU_IPC_5_0x208;
  Can_SCU_IPC_6_0x209 SCU_IPC_6_0x209;
  Can_SCU_IPC_7_0x301 SCU_IPC_7_0x301;

  ipAddr = getParam < string > ("auto_control/addr", "192.168.110.101");
  ipPort = getParam < int >("auto_control/port", 4001);

  Publisher pub_SCU_IPC_1 = n.advertise < canbus_msgs::SCU_IPC_1_0x20A > ("/canbus/SCU_IPC_1", 1000);			//1000:message number to buffer up 
  Publisher pub_SCU_IPC_2 = n.advertise < canbus_msgs::SCU_IPC_2_0x205 > ("/canbus/SCU_IPC_2", 1000);
  Publisher pub_SCU_IPC_3 = n.advertise < canbus_msgs::SCU_IPC_3_0x206 > ("/canbus/SCU_IPC_3", 1000);
  Publisher pub_SCU_IPC_4 = n.advertise < canbus_msgs::SCU_IPC_4_0x207 > ("/canbus/SCU_IPC_4", 1000);
  Publisher pub_SCU_IPC_5 = n.advertise < canbus_msgs::SCU_IPC_5_0x208 > ("/canbus/SCU_IPC_5", 1000);
  Publisher pub_SCU_IPC_6 = n.advertise < canbus_msgs::SCU_IPC_6_0x209 > ("/canbus/SCU_IPC_6", 1000);
  Publisher pub_SCU_IPC_7 = n.advertise < canbus_msgs::SCU_IPC_7_0x301 > ("/canbus/SCU_IPC_7", 1000);

  if (xudp.Bind (ipPort) < 0){
    ROS_ERROR("canbus - bind socket fail");
    return -1;
	}

  while (ok ())
    {
      uint8 *p;
      memset (buf, 0, sizeof (buf));
      memset (ip, 0, sizeof (ip));

      len = xudp.Receive (buf, 512, ip);
	  if(len < 0)
	  	ROS_WARN("canbus - socket receives fault data.");
//      cout << buf << endl;
//      cout << " received length: " << len << endl;

      int n = len / 13;
      p = buf;

      if (len % 13 == 0)	// 13 times
	{
	  while (n)
	    {
	      uint16 i, j;

	      int flag = 0;
	      TimestampedCanFrame Tframe;

	      Tframe.frame.id = ((uint16) (*(p+3) << 8) + *(p+4));
	      for (j = 0; j < 8; j++) {
		Tframe.frame.data[j] = *(p+j+5);
	      }

	      switch (Tframe.frame.id)
		{
		case 0x20A:
		{
  			canbus_msgs::SCU_IPC_1_0x20A scu_ipc_1_0x20A;
			SCU_IPC_1_0x20A.SetData(Tframe);
			SCU_IPC_1_0x20A.decode();
			CanFrame_SCU_IPC_1_0x20A data20A = *(SCU_IPC_1_0x20A.data());

			scu_ipc_1_0x20A.SCU_IPC_SteeringAngleVD = data20A.SteeringAngleVD;
			
			uint16 steering_angle_speed  = data20A.SteeringAngleSpd;
			if(steering_angle_speed > 1016) steering_angle_speed = 1016;
			scu_ipc_1_0x20A.SCU_IPC_SteeringAngleSpd = steering_angle_speed;	

			float steering_angle = data20A.SteeringAngle * 0.1 - 780;		//
			if(steering_angle > 779.9) steering_angle = 779.9;
			scu_ipc_1_0x20A.SCU_IPC_SteeringAngle = steering_angle;

			float response_torque = data20A.ResponseTorque * 0.01 - 10.24;
			if(response_torque > 10.23) response_torque = 10.23;
			scu_ipc_1_0x20A.SCU_IPC_ResponseTorque = response_torque;

			scu_ipc_1_0x20A.SCU_IPC_ResponseTorqueVD = data20A.ResponseTorqueVD;

			scu_ipc_1_0x20A.SCU_IPC_SteeringAngleSpdVD = data20A.SteeringAngleSpdVD;

			uint8 msg_counter = data20A.MsgCounter;
			if(msg_counter > 15) msg_counter = 15;
			scu_ipc_1_0x20A.SCU_IPC_1_MsgCounter = msg_counter;	

  			uint8_t check_sum = CalculateCheckSum(Tframe.frame.data, 7);
			scu_ipc_1_0x20A.SCU_IPC_1_Checksum = check_sum;

			// if(check_sum == data20A.Checksum){
				pub_SCU_IPC_1.publish(scu_ipc_1_0x20A);
			// }
			// else{
				// ROS_WARN_STREAM("canbus - SCU_IPC_1 checksum error:checksum " << (int)check_sum 
					// << ", and CheckSum is " << (int)data20A.Checksum);
			// }
			break;
		}

		case 0x205:
		{
  			canbus_msgs::SCU_IPC_2_0x205 scu_ipc_2_0x205;
			SCU_IPC_2_0x205.SetData(Tframe);
			SCU_IPC_2_0x205.decode();
			CanFrame_SCU_IPC_2_0x205 data205 = *(SCU_IPC_2_0x205.data());

			float fl_wheel_speed = data205.FLWheelSpd * 0.05625;		//0~240 kmph
			if(fl_wheel_speed > 240) fl_wheel_speed = 240;
			scu_ipc_2_0x205.SCU_IPC_FLWheelSpd = fl_wheel_speed;
			
			float fr_wheel_speed = data205.FRWheelSpd * 0.05625;		//0~240 kmph
			if(fr_wheel_speed > 240) fr_wheel_speed = 240;
			scu_ipc_2_0x205.SCU_IPC_FRWheelSpd = fr_wheel_speed;

			float rl_wheel_speed = data205.RLWheelSpd * 0.05625;		//0~240 kmph
			if(rl_wheel_speed > 240) rl_wheel_speed = 240;
			scu_ipc_2_0x205.SCU_IPC_RLWheelSpd = rl_wheel_speed;

			float rr_wheel_speed = data205.RRWheelSpd * 0.05625;		//0~240 kmph
			if(rr_wheel_speed > 240) rr_wheel_speed = 240;
			scu_ipc_2_0x205.SCU_IPC_RRWheelSpd = rr_wheel_speed;

			scu_ipc_2_0x205.SCU_IPC_FLWheelSpdVD = data205.FLWheelSpdVD;

			scu_ipc_2_0x205.SCU_IPC_FRWheelSpdVD = data205.FRWheelSpdVD;

			scu_ipc_2_0x205.SCU_IPC_RLWheelSpdVD = data205.RLWheelSpdVD;

			scu_ipc_2_0x205.SCU_IPC_RRWheelSpdVD = data205.RRWheelSpdVD;

			pub_SCU_IPC_2.publish(scu_ipc_2_0x205);
			break;
		}

		case 0x206:
		{
  			canbus_msgs::SCU_IPC_3_0x206 scu_ipc_3_0x206;
			SCU_IPC_3_0x206.SetData(Tframe);
			SCU_IPC_3_0x206.decode();
			CanFrame_SCU_IPC_3_0x206 data206 = *(SCU_IPC_3_0x206.data());

			uint8 dbw_st = data206.DBWSt;
			if(dbw_st > 15) dbw_st = 15;
			scu_ipc_3_0x206.SCU_IPC_DBWSt = dbw_st;

			scu_ipc_3_0x206.SCU_IPC_VehSpdVD = data206.VehSpdVD;

			scu_ipc_3_0x206.SCU_IPC_BrkPedalSt = data206.BrkPedalSt;

			scu_ipc_3_0x206.SCU_IPC_BrkPedalStVD = data206.BrkPedalStVD;

			float vehicle_speed = data206.VehSpd * 0.05625;		//0~240 kmph
			if(vehicle_speed > 240) vehicle_speed = 240;
			scu_ipc_3_0x206.SCU_IPC_VehSpd = vehicle_speed;

			scu_ipc_3_0x206.SCU_IPC_BrkLightOn = data206.BrkLightOn;	//not used now

			uint8 msg_counter = data206.MsgCounter;
			if(msg_counter > 15) msg_counter = 15;
			scu_ipc_3_0x206.SCU_IPC_3_MsgCounter = msg_counter;	

  			uint8_t check_sum = CalculateCheckSum(Tframe.frame.data, 7);
			scu_ipc_3_0x206.SCU_IPC_3_Checksum = check_sum;

			if(check_sum == data206.Checksum){
				pub_SCU_IPC_3.publish(scu_ipc_3_0x206);
			}
			else{
				ROS_WARN("canbus - SCU_IPC_3 checksum error. ");
			}			
			break;
		}

		case 0x207:			
		{
  			canbus_msgs::SCU_IPC_4_0x207 scu_ipc_4_0x207;
			SCU_IPC_4_0x207.SetData(Tframe);
			SCU_IPC_4_0x207.decode();
			CanFrame_SCU_IPC_4_0x207 data207 = *(SCU_IPC_4_0x207.data());

			scu_ipc_4_0x207.SCU_IPC_ActVehLongAccelVD = data207.ActVehLongAccelVD;

			scu_ipc_4_0x207.SCU_IPC_ActVehLateralAccelVD = data207.ActVehLateralAccelVD;

			scu_ipc_4_0x207.SCU_IPC_YAWVD = data207.YAWVD;

			float yaw = data207.YAW * 0.0625 - 93;		//-93~93
			if(yaw > 93) yaw = 93;
			scu_ipc_4_0x207.SCU_IPC_YAW = yaw;

			float act_veh_long_accel = data207.ActVehLongAccel * 0.002768 - 1.8;		//-1.8~1.8
			if(act_veh_long_accel > 1.8) act_veh_long_accel = 1.8;
			scu_ipc_4_0x207.SCU_IPC_ActVehLongAccel = act_veh_long_accel;

			float act_veh_lateral_accel = data207.ActVehLateralAccel * 0.002768 - 1.8;		//-1.8~1.8
			if(act_veh_lateral_accel > 1.8) act_veh_lateral_accel = 1.8;
			scu_ipc_4_0x207.SCU_IPC_ActVehLateralAccel = act_veh_lateral_accel;

			uint8 epb_sys_st = data207.EPBSysSt;
			if(epb_sys_st > 7) epb_sys_st = 7;
			scu_ipc_4_0x207.SCU_IPC_EPBSysSt = epb_sys_st;	

			uint8 msg_counter = data207.MsgCounter;
			if(msg_counter > 15) msg_counter = 15;
			scu_ipc_4_0x207.SCU_IPC_4_MsgCounter = msg_counter;	

  			uint8_t check_sum = CalculateCheckSum(Tframe.frame.data, 7);
			scu_ipc_4_0x207.SCU_IPC_4_Checksum = check_sum;	

			if(check_sum == data207.Checksum){
				pub_SCU_IPC_4.publish(scu_ipc_4_0x207);
			}
			else{
				ROS_WARN("canbus - SCU_IPC_4 checksum error. ");
			}						
			break;
		}

		case 0x208:
		{
  			canbus_msgs::SCU_IPC_5_0x208 scu_ipc_5_0x208;
			SCU_IPC_5_0x208.SetData(Tframe);
			SCU_IPC_5_0x208.decode();
			CanFrame_SCU_IPC_5_0x208 data208 = *(SCU_IPC_5_0x208.data());

			scu_ipc_5_0x208.SCU_IPC_DriverDoorLockSt = data208.DriverDoorLockSt;

			scu_ipc_5_0x208.SCU_IPC_DriverDoorAjarSt = data208.DriverDoorAjarSt;

			scu_ipc_5_0x208.SCU_IPC_PsngrDoorAjarSt = data208.PsngrDoorAjarSt;

			scu_ipc_5_0x208.SCU_IPC_RLDoorAjarSt = data208.RLDoorAjarSt;

			scu_ipc_5_0x208.SCU_IPC_RRDoorAjarSt = data208.RRDoorAjarSt;

			scu_ipc_5_0x208.SCU_IPC_LTurnLampOutputSt = data208.LTurnLampOutputSt;

			scu_ipc_5_0x208.SCU_IPC_RTurnLampOutputSt = data208.RTurnLampOutputSt;

			scu_ipc_5_0x208.SCU_IPC_HazardLampOutputSt = data208.HazardLampOutputSt;

			scu_ipc_5_0x208.SCU_IPC_LowBeamOutputSt = data208.LowBeamOutputSt;

			scu_ipc_5_0x208.SCU_IPC_HighBeamOutputSt = data208.HighBeamOutputSt;

			scu_ipc_5_0x208.SCU_IPC_Horndriverst = data208.Horndriverst;

			uint8 front_wiper_output_st = data208.FrontWiperOutputSt;
			if(front_wiper_output_st > 3) front_wiper_output_st = 3;
			scu_ipc_5_0x208.SCU_IPC_FrontWiperOutputSt = front_wiper_output_st;	

			uint8 power_mode = data208.PowerMode;
			if(power_mode > 3) power_mode = 3;
			scu_ipc_5_0x208.SCU_IPC_PowerMode = power_mode;	

			pub_SCU_IPC_5.publish(scu_ipc_5_0x208);
			break;
		}

		case 0x209:	
		{
  			canbus_msgs::SCU_IPC_6_0x209 scu_ipc_6_0x209;
			SCU_IPC_6_0x209.SetData(Tframe);
			SCU_IPC_6_0x209.decode();
			CanFrame_SCU_IPC_6_0x209 data209 = *(SCU_IPC_6_0x209.data());

			uint16 dst_bat_dsp  = data209.dstBat_Dsp;
			if(dst_bat_dsp > 1022) dst_bat_dsp = 1022;
			scu_ipc_6_0x209.SCU_IPC_dstBat_Dsp = dst_bat_dsp;	

			uint8 acc_pedal_sig  = data209.AccPedalSig;
			if(acc_pedal_sig > 100) acc_pedal_sig = 100;
			scu_ipc_6_0x209.SCU_IPC_AccPedalSig = acc_pedal_sig;	

			scu_ipc_6_0x209.SCU_IPC_BrkPedalSt = data209.BrkPedalSt;

			scu_ipc_6_0x209.SCU_IPC_BrkPedalStVD = data209.BrkPedalStVD;

			scu_ipc_6_0x209.SCU_IPC_CurrentGearLevVD = data209.CurrentGearLevVD;

			uint8 current_gear_lev  = data209.CurrentGearLev;
			if(current_gear_lev > 7) current_gear_lev = 7;
			scu_ipc_6_0x209.SCU_IPC_CurrentGearLev = current_gear_lev;	

			uint8 msg_counter = data209.MsgCounter;
			if(msg_counter > 15) msg_counter = 15;
			scu_ipc_6_0x209.SCU_IPC_6_MsgCounter = msg_counter;	

  			uint8_t check_sum = CalculateCheckSum(Tframe.frame.data, 7);
			scu_ipc_6_0x209.SCU_IPC_6_Checksum = check_sum;	

			if(check_sum == data209.Checksum){
				pub_SCU_IPC_6.publish(scu_ipc_6_0x209);
			}
			else{
				ROS_WARN("canbus - SCU_IPC_6 checksum error. ");
			}			
			break;
		}

		case 0x301:
		{
  			canbus_msgs::SCU_IPC_7_0x301 scu_ipc_7_0x301;
			SCU_IPC_7_0x301.SetData(Tframe);
			SCU_IPC_7_0x301.decode();
			CanFrame_SCU_IPC_7_0x301 data301 = *(SCU_IPC_7_0x301.data());

			scu_ipc_7_0x301.SCU_IPC_PSeatBeltWarning = data301.PSeatBeltWarning;	//not used now

			scu_ipc_7_0x301.SCU_IPC_DriverSeatBeltWarning = data301.DriverSeatBeltWarning;

			scu_ipc_7_0x301.SCU_IPC_1ndLPSeatBeltWar = data301._1ndLPSeatBeltWar;	//not used now

			scu_ipc_7_0x301.SCU_IPC_1ndMPSeatBeltWar = data301._1ndMPSeatBeltWar;	//not used now

			scu_ipc_7_0x301.SCU_IPC_1ndRPSeatBeltWar = data301._1ndRPSeatBeltWar;	//not used now

			scu_ipc_7_0x301.SCU_IPC_TotalOdometerVD = data301.TotalOdometerVD;

			float total_odometer = data301.TotalOdometer * 0.5;		// 0~524287.5
			if(total_odometer > 524287.5) total_odometer = 524287.5;
			scu_ipc_7_0x301.SCU_IPC_TotalOdometer = total_odometer;

			scu_ipc_7_0x301.SCU_IPC_ACSt = data301.ACSt;

			uint8 err_st = data301.ErrSt;
			if(err_st > 15) err_st = 15;
			scu_ipc_7_0x301.SCU_IPC_ErrSt = err_st;	

			uint8 override_res = data301.OverrideRes;
			if(override_res > 15) override_res = 15;
			scu_ipc_7_0x301.SCU_IPC_OverrideRes = override_res;	

			pub_SCU_IPC_7.publish(scu_ipc_7_0x301);
			break;
		}

		default:
			//ROS_WARN("canbus - frame ID: %x is out of range.", Tframe.frame.id);
			break;
		}
	      n--;
	      p = p + 13;
	    }		//end of while (n)
	}		//end of if (len % 13 == 0)
	else
		ROS_WARN("canbus - length of received data is not times of 13.");
    }	//end of while (ok ())
  return 0;
}

