//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//

#ifndef _P7Dynamics_H
#define _P7Dynamics_H

#include <cstdlib>
#include <string>
#include <sstream>

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeManager.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"

#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Net/icvUdpReceiverSource.h"

#include "OpenICV/structure/structureVehinfo.h"
#include "OpenICV/structure/structure_gps.h"
#include "OpenICV/structure/structureIMU.h"
#include "OpenICV/Basis/icvStructureData.hxx"

#include "OpenICV/structure/data_demo_type1.h"

#include "Can_SCU_IPC_1_0x174.h"
#include "Can_SCU_IPC_2_0x175.h"
#include "Can_SCU_IPC_3_0x176.h"
#include "Can_SCU_IPC_4_0x177.h"
#include "Can_SCU_IPC_5_0x178.h"
#include "Can_SCU_IPC_6_0x179.h"
#include "Can_SCU_IPC_7_0x17A.h"

#include "CanFrame.h"
#include "inputmessage.h"

#include <iomanip>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <bitset>
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

using namespace std;
using namespace icv;
using namespace icv::function;
typedef data::icvStructureData<Imu> icvImu;
typedef data::icvStructureData<donnees_gps> icvGPS;
typedef data::icvStructureData<NavSatFix> icvNavSatFix;

typedef icv::data::icvStructureData<data_demo_type2>    icvdata_demo_type2;

typedef data::icvStructureData<CanFrame_SCU_IPC_1_0x174> SCU_IPC_0x174;
typedef data::icvStructureData<CanFrame_SCU_IPC_2_0x175> SCU_IPC_0x175;
typedef data::icvStructureData<CanFrame_SCU_IPC_3_0x176> SCU_IPC_0x176;
typedef data::icvStructureData<CanFrame_SCU_IPC_4_0x177> SCU_IPC_0x177;
typedef data::icvStructureData<CanFrame_SCU_IPC_5_0x178> SCU_IPC_0x178;
typedef data::icvStructureData<CanFrame_SCU_IPC_6_0x179> SCU_IPC_0x179;
typedef data::icvStructureData<CanFrame_SCU_IPC_7_0x17A> SCU_IPC_0x17A;


class P7Dynamics : public icvUdpReceiverSource
{
public:
	//typedef data::icvStructureData<CanFrameMotor_50B> icv_CanFrameMotor_50B;

	P7Dynamics(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info)
	{

		if (_information.Contains("ax"))
			ax_send = _information.GetDecimal("ax");
		if (_information.Contains("steer"))
			steerangle_send = _information.GetDecimal("steer");
		if (_information.Contains("receive"))
			recei_can_ = _information.GetBoolean("receive");
		if (_information.Contains("send"))
			send_can_ = _information.GetBoolean("send");
		// Register_Sub("msg_fix");
		// Register_Sub("msg_imu");
		// Register_Sub("msg_vel");
		Register_Pub("data_174");
		Register_Pub("data_175");
		Register_Pub("data_176");
		Register_Pub("data_177");
		Register_Pub("data_178");
		Register_Pub("data_179");
		Register_Pub("data_17A");

		Register_Sub("acc_str");


		ICV_LOG_INFO << "XIAOPENG send message once";
		//icv_testMotor_50B=new icv_CanFrameMotor_50B();
		//ICV_LOG_INFO<<"XIAOPENG send message once EpbState_CCP "<<int (data4.EpbState_CCP);

		icv_thread loop(icv_bind(&P7Dynamics::processsendcan, this));
		icv_thread_guard g(loop);
	};

	void processsendcan()
	{
		temp_ag = 0;
		//PID_init();
		while (send_can_)
		{
			// icvSubscribe("msg_fix", &GPS_DATA);
			// icvSubscribe("msg_imu", &IMU_DATA);
			// icvSubscribe("msg_vel", &VEL_DATA);
			/*
			if (GPS_DATA.is_not_empty() && IMU_DATA.is_not_empty())
			{
				gps_data = GPS_DATA.getvalue();
				imu_data = IMU_DATA.getvalue();
				vel_data = VEL_DATA.getvalue();
			}
			*/

			//sendcan_171(DBWReq, ReadyReq, BrakeLight, SteerAngleReq, SteerAngleReqVD, TorsionBarTqReq,
			//            TorsionBarTqReqVD, GearReq);
			//sendcan_172(MotorTorqReq，MotorTorqReqVD，ParkingReqToEPBVD，ParkingReqToEPB，AccDecelReq，
			//            AccDecelReqVD，VehSpd，VehSpdVD);
			//sendcan_173(LowBeamSWSt, LowBeamSWStVD, HighBeamSWSt; HighBeamSWStVD,	HazardLampSWSt,
			//            LTurnLampSWSt, RTurnLampSWSt,	HorndriverSWst)
			if (temp_ag == 0)
			{
				sendcan_171(0x1, 0, 0, 0, 0, 0, 0, 0);
				usleep(1000000);
				ICV_LOG_INFO << "Init!\n";
				sendcan_171(0x3, 0, 0, 0, 0, 0, 0, 0);
				usleep(1000000);
				ICV_LOG_INFO << "Into!\n";
				sendcan_171(0x3, 0, 0, 0, 0, 0, 0, 0x2);
				usleep(1000000);
				ICV_LOG_INFO << "Gear D request!\n";
				sendcan_172(0, 0, 1, 0x1, 0, 0, 0, 0);
				ICV_LOG_INFO << "EPB release request!\n";
				usleep(1000000);
				temp_ag = 1;
			}
            
			CanFrame_SCU_IPC_4_0x177 decode_data_177 = *(test_177.data());

			if(decode_data_177.EPBSysSt == 1)
			{
			//float speed_traj = 10.0;
			//CanFrame_SCU_IPC_3_0x176 decode_data_176 = *(test_176.data());
			//float speed_navi_new = decode_data_176.VehSpd * 0.05625;

 			//float accleration_pid = PID_realize(speed_traj, speed_navi_new); // speed_traj 规划轨迹速度输出，speed_navi惯导速度输出
			
			 icvSubscribe("acc_str", &ACC_STR);
			 acc_steer = ACC_STR.getvalue();
			ICV_LOG_DEBUG << "acc_steer.acc :" << acc_steer.acc;

			float accleration = acc_steer.acc;
			float steeringangle =acc_steer.steer;

			printf("Vehicle acc: %.6f\n", accleration);
			sendcan_171(0x3, 0, 0, steeringangle, 1, 0, 0, 0);
			sendcan_172(0, 0, 0, 0, accleration, 1, 0, 1);
			sendcan_173(0, 0, 0, 0, 0, 0, 0, 0);

			usleep(50000);
			//ICV_LOG_INFO << "A cycle\n";
			}

		}
	}

	// void PID_init()
	// {
	// 	pid.SetSpeed = 0.0;
	// 	pid.ActualSpeed = 0.0;
	// 	pid.err = 0.0;
	// 	pid.err_last = 0.0;
	// 	pid.accleration = 0.0;
	// 	pid.integral = 0.0;
	// 	pid.Kp = 5e-2; // TODO: PID parameter
	// 	pid.Ki = 2e-4;
	// 	pid.Kd = 0;
	// 	pid.umax = 40; // max speed 11m/s  40km/h
	// 	pid.umin = 0;  // min speed 0m/s
	// }

	// float PID_realize(float speed_traj, float speed_navi)
	// {
	// 	pid.SetSpeed = speed_traj;
	// 	pid.err = pid.SetSpeed - pid.ActualSpeed;
	// 	printf("pid.err: %.6f\n", pid.err);
	// 	if (pid.ActualSpeed > pid.umax) //抗积分饱和的实现
	// 	{
	// 		if (abs(pid.err) > 1) //积分分离过程， 速度误差大于 3.6km/h
	// 		{
	// 			index = 0;
	// 		}
	// 		else
	// 		{
	// 			index = 1;
	// 			if (pid.err < 0)
	// 			{
	// 				pid.integral += pid.err;
	// 			}
	// 		}
	// 	}
	// 	else if (pid.ActualSpeed < pid.umin)
	// 	{
	// 		if (abs(pid.err) > 1) //积分分离过程
	// 		{
	// 			index = 0;
	// 		}
	// 		else
	// 		{
	// 			index = 1;
	// 			if (pid.err > 0)
	// 			{
	// 				pid.integral += pid.err;
	// 			}
	// 		}
	// 	}
	// 	else
	// 	{
	// 		if (abs(pid.err) > 1) //积分分离过程
	// 		{
	// 			index = 0;
	// 		}
	// 		else
	// 		{
	// 			index = 1;
	// 			pid.integral += pid.err;
	// 		}
	// 	}
	// 	pid.accleration = pid.Kp * pid.err + index * pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_last);

	// 	if (pid.accleration >= 3)
	// 		pid.accleration = 3;
	// 	else if (pid.accleration <= -5)
	// 		pid.accleration = -5;

	// 	pid.err_last = pid.err;
	// 	pid.ActualSpeed = speed_navi;
	// 	printf("pid_realize: pid.acc = %.6f\n", pid.accleration);
	// 	return pid.accleration;
	// }

	uint8_t CalculateCheckSum(const uint8_t *input, const uint32_t length)
	{
		return static_cast<uint8_t>(std::accumulate(input, input + length, 0) ^ 0xFF);
	}

	void sendcan_171(uint8_t DBWReq, uint8_t ReadyReq, uint8_t BrakeLight,
					 uint16_t SteerAngleReq, uint8_t SteerAngleReqVD, uint16_t TorsionBarTqReq,
					 uint8_t TorsionBarTqReqVD, uint8_t GearReq)
	{
		//------------------------------------------0x171 message-------------------------------------------
		SteerAngleReq = round((SteerAngleReq + 780) * 10);		  //-780～779.9
		TorsionBarTqReq = round((TorsionBarTqReq + 10.24) * 100); //-10.24～10.23

		if (DBWReq >= 0 && DBWReq <= 15)
			DBWReq = DBWReq;

		if (ReadyReq == 0x00 || ReadyReq == 0x01)
			ReadyReq = ReadyReq;

		if (BrakeLight >= 0 && BrakeLight <= 3)
			BrakeLight = BrakeLight;

		if (SteerAngleReq > 15599)
			SteerAngleReq = 15599;

		if (SteerAngleReqVD == 0x00 || SteerAngleReqVD == 0x01)
			SteerAngleReqVD = SteerAngleReqVD;

		if (TorsionBarTqReq > 2047)
			TorsionBarTqReq = 2047;

		if (TorsionBarTqReqVD == 0x00 || TorsionBarTqReqVD == 0x01)
			TorsionBarTqReqVD = TorsionBarTqReqVD;

		if (GearReq >= 0 && GearReq <= 4)
			GearReq = GearReq;

		// static int MsgCounter = 0;
		if (MsgCounter_171 > 0x0f)
		{
			MsgCounter_171 = 0;
		}

		uint8_t data_171[69] = {0};
		//canfdnet send type ----------------------
		//total bytes is 69, the fist byte is frame info ,
		//frame info data is  0 0 1 0 1 1 1 1 = 0x2F
		data_171[0] = 0x3F;
		data_171[1] = 0x00;
		data_171[2] = 0x00;
		data_171[3] = 0x01;
		data_171[4] = 0x71;

		data_171[5] = DBWReq << 4;
		data_171[7] = (ReadyReq << 6) | (BrakeLight << 4) | ((SteerAngleReq >> 12) & 0x000f);
		data_171[8] = (SteerAngleReq >> 4) & 0x00ff;
		data_171[9] = ((SteerAngleReq & 0x000f) << 4) | (SteerAngleReqVD << 3) | ((TorsionBarTqReq >> 8) & 0x0007);
		data_171[10] = TorsionBarTqReq & 0x00ff;
		data_171[11] = (TorsionBarTqReqVD << 7) | (GearReq << 4) | MsgCounter_171;
		MsgCounter_171++;
		uint8_t Checksum_171 = CalculateCheckSum(&data_171[5], 7);
		data_171[12] = Checksum_171;
		send(data_171, 69);
	}

	void sendcan_172(uint16_t MotorTorqReq, uint8_t MotorTorqReqVD, uint8_t ParkingReqToEPBVD,
					 uint8_t ParkingReqToEPB, float32 AccDecelReq, uint8_t AccDecelReqVD,
					 uint16_t VehSpd, uint8_t VehSpdVD)
	{
		//------------------------------------------0x172 message-------------------------------------------
		MotorTorqReq = round((MotorTorqReq + 200) * 2); //-200～311.5
		uint16_t AccDecelReq_int = (uint16_t)((AccDecelReq + 15) * 50) - 3;	//-15～5
		// uint16_t AccDecelReq_int = 748; 
		VehSpd = round((VehSpd + 50) * 10);				//-50～120

		if (MotorTorqReq > 1023)
			MotorTorqReq = 1023;

		if (MotorTorqReqVD == 0x00 || MotorTorqReqVD == 0x01)
			MotorTorqReqVD = MotorTorqReqVD;

		if (ParkingReqToEPBVD == 0x00 || ParkingReqToEPBVD == 0x01)
			ParkingReqToEPBVD = ParkingReqToEPBVD;

		if (ParkingReqToEPB >= 0 && ParkingReqToEPB <= 3)
			ParkingReqToEPB = ParkingReqToEPB;

		if (AccDecelReq > 1000)
			AccDecelReq = 1000;

		if (AccDecelReqVD == 0x00 || AccDecelReqVD == 0x01)
			AccDecelReqVD = AccDecelReqVD;

		if (VehSpd > 1700)
			VehSpd = 1700;

		if (VehSpdVD == 0x00 || VehSpdVD == 0x01)
			VehSpdVD = VehSpdVD;

		if (MsgCounter_172 > 0x0f)
		{
			MsgCounter_172 = 0;
		}

		uint8_t data_172[69] = {0};
		//canfdnet send type ----------------------
		//total bytes is 69, the fist byte is frame info ,
		//frame info data is  0 0 1 0 1 1 1 1 = 0x2F
		data_172[0] = 0x3F;
		data_172[1] = 0x00;
		data_172[2] = 0x00;
		data_172[3] = 0x01;
		data_172[4] = 0x72;

		data_172[5] = (MotorTorqReq >> 2) & 0x00ff; //Motorola MSB
		data_172[6] = ((MotorTorqReq & 0x0003) << 6) | (MotorTorqReqVD << 5) | (ParkingReqToEPBVD << 4) |
					  ((ParkingReqToEPB & 0x03) << 2) | ((AccDecelReq_int >> 8) & 0x0003);
		data_172[7] = AccDecelReq_int & 0x00ff;
		data_172[8] = (AccDecelReqVD << 7) | ((VehSpd >> 6) & 0x007f);
		data_172[9] = (VehSpd & 0x003f << 2) | (VehSpdVD << 1);
		data_172[11] = MsgCounter_172;
		MsgCounter_172++;
		uint8_t Checksum_172 = CalculateCheckSum(&data_172[5], 7);
		data_172[12] = Checksum_172;
		send(data_172, 69);
	}

	void sendcan_173(uint8_t LowBeamSWSt, uint8_t LowBeamSWStVD, uint8_t HighBeamSWSt,
					 uint8_t HighBeamSWStVD, uint8_t HazardLampSWSt, uint8_t LTurnLampSWSt,
					 uint8_t RTurnLampSWSt, uint8_t HorndriverSWst)
	{
		//------------------------------------------0x173 message-------------------------------------------
		if (LowBeamSWSt == 0x00 || LowBeamSWSt == 0x01)
			LowBeamSWSt = LowBeamSWSt;

		if (LowBeamSWStVD == 0x00 || LowBeamSWStVD == 0x01)
			LowBeamSWStVD = LowBeamSWStVD;

		if (HighBeamSWSt == 0x00 || HighBeamSWSt == 0x01)
			HighBeamSWSt = HighBeamSWSt;

		if (HighBeamSWStVD == 0x00 || HighBeamSWStVD == 0x01)
			HighBeamSWStVD = HighBeamSWStVD;

		if (HazardLampSWSt >= 0 && HazardLampSWSt <= 3)
			HazardLampSWSt = HazardLampSWSt;

		if (LTurnLampSWSt >= 0 && LTurnLampSWSt <= 3)
			LTurnLampSWSt = LTurnLampSWSt;

		if (RTurnLampSWSt >= 0 && RTurnLampSWSt <= 3)
			RTurnLampSWSt = RTurnLampSWSt;

		if (HorndriverSWst >= 0 && HorndriverSWst <= 3)
			HorndriverSWst = HorndriverSWst;

		uint8_t data_173[69] = {0};
		//canfdnet send type ----------------------
		//total bytes is 69, the fist byte is frame info ,
		//frame info data is  0 0 1 0 1 1 1 1 = 0x2F
		data_173[0] = 0x3F;
		data_173[1] = 0x00;
		data_173[2] = 0x00;
		data_173[3] = 0x01;
		data_173[4] = 0x73;

		data_173[5] = (HazardLampSWSt << 6) | (LTurnLampSWSt << 4) | (HighBeamSWStVD << 3) |
					  (HighBeamSWSt << 2) | (LowBeamSWStVD << 1) | LowBeamSWSt; //Motorola MSB
		data_173[6] = (RTurnLampSWSt << 6) | (HorndriverSWst << 4);
		send(data_173, 69);
	}

	virtual void Process(std::istream &stream) override
	{

		count_++;
		// ICV_LOG_DEBUG<<"    ";
		// ICV_LOG_DEBUG << "length  can receive:"<< _buffer.size() ;
		if (_buffer.size() % 69 == 0 && _buffer.size() > 0) //69
		{
			while (!stream.eof() && _buffer.size() > 0)
			{
				char udpBuffer[69];
				stream.read(udpBuffer, 69);

				uint16 i, j;
				//lrr
				int flag = 0;
				//frames.clear();
				TimestampedCanFrame Tframe;
				uint8 temp1, temp2;
				temp1 = (uint8)(udpBuffer[3]);
				temp2 = (uint8)(udpBuffer[4]);
				Tframe.frame.id = ((uint16)(temp1 << 8) + temp2);

				ICV_LOG_DEBUG<<"readingID"<<Tframe.frame.id;
				// fill up udpBufferGroup
				for (j = 0; j < 8; j++)
				{
					Tframe.frame.data[j] = udpBuffer[j + 5];
				}

				switch (Tframe.frame.id)
				{
				case 0x174:
				{
					test_174.decode_174(Tframe.frame.data);
					CanFrame_SCU_IPC_1_0x174 decode_data_174 = *(test_174.data());
					//ICV_LOG_DEBUG << "0x174 - SCU_IPC_SteeringAngle :" << decode_data_174.SteeringAngle;
					//ICV_LOG_DEBUG << "0x174 - SCU_IPC_SteeringAnglespd :" << decode_data_174.SteeringAngleSpd;
					decode_data_174_t.setvalue(decode_data_174);
					icvPublish("data_174",&decode_data_174_t);
					break;
				}
				case 0x175:
				{
					test_175.decode_175(Tframe.frame.data);
					CanFrame_SCU_IPC_2_0x175 decode_data_175 = *(test_175.data());
					decode_data_175_t.setvalue(decode_data_175);
					icvPublish("data_175",&decode_data_175_t);
					// ICV_LOG_DEBUG << "0x175 - SCU_IPC_FLWheelSpd :" << decode_data_175.FLWheelSpd;
					// ICV_LOG_DEBUG << "0x175 - SCU_IPC_FRWheelSpd :" << decode_data_175.FRWheelSpd;
					// ICV_LOG_DEBUG << "0x175 - SCU_IPC_RLWheelSpd :" << decode_data_175.RLWheelSpd;
					// ICV_LOG_DEBUG << "0x175 - SCU_IPC_RRWheelSpd :" << decode_data_175.RRWheelSpd;
					// ICV_LOG_DEBUG << "0x175 - SCU_IPC_FLWheelSpdVD :" << decode_data_175.FLWheelSpdVD;
					// ICV_LOG_DEBUG << "0x175 - SCU_IPC_FRWheelSpdVD :" << decode_data_175.FRWheelSpdVD;
					// ICV_LOG_DEBUG << "0x175 - SCU_IPC_RLWheelSpdVD :" << decode_data_175.RLWheelSpdVD;
					// ICV_LOG_DEBUG << "0x175 - SCU_IPC_RRWheelSpdVD :" << decode_data_175.RRWheelSpdVD;
					break;
				}
				case 0x176:
				{
					test_176.decode_176(Tframe.frame.data);
					CanFrame_SCU_IPC_3_0x176 decode_data_176 = *(test_176.data());
					decode_data_176_t.setvalue(decode_data_176);
					icvPublish("data_176",&decode_data_176_t);
					// ICV_LOG_DEBUG << "0x176 - SCU_IPC_DBWSt :" << (uint16_t)decode_data_176.DBWSt;
					// ICV_LOG_DEBUG << "0x176 - SCU_IPC_VehSpd:" << decode_data_176.VehSpd;
					// ICV_LOG_DEBUG << "0x176 - SCU_IPC_VehSpdVD:" << (int16)decode_data_176.VehSpdVD;
					break;
				}
				case 0x177:
				{
					test_177.decode_177(Tframe.frame.data);
					CanFrame_SCU_IPC_4_0x177 decode_data_177 = *(test_177.data());
					decode_data_177_t.setvalue(decode_data_177);
					icvPublish("data_177",&decode_data_177_t);
					//ICV_LOG_DEBUG << "0x177 - SCU_IPC_YAW :" << (int16)decode_data_177.YAW;
					break;
				}
				case 0x178:
				{
					test_178.decode_178(Tframe.frame.data);
					CanFrame_SCU_IPC_5_0x178 decode_data_178 = *(test_178.data());
					decode_data_178_t.setvalue(decode_data_178);
					icvPublish("data_178",&decode_data_178_t);
					//ICV_LOG_DEBUG << "0x178 - SCU_IPC_DriverDoorLockSt :" << (int16)decode_data_178.DriverDoorLockSt;
					//ICV_LOG_DEBUG << "0x178 - SCU_IPC_DriverDoorAjarSt :" << (int16)decode_data_178.DriverDoorAjarSt;
					//ICV_LOG_DEBUG << "0x178 - SCU_IPC_PsngrDoorAjarSt :" << (int16)decode_data_178.PsngrDoorAjarSt;
					//ICV_LOG_DEBUG << "0x178 - SCU_IPC_RLDoorAjarSt :" << (int16)decode_data_178.RLDoorAjarSt;	
					//ICV_LOG_DEBUG << "0x178 - SCU_IPC_RRDoorAjarSt :" << (int16)decode_data_178.RRDoorAjarSt;				

					break;
				}
				case 0x179:
				{
					test_179.decode_179(Tframe.frame.data);
					CanFrame_SCU_IPC_6_0x179 decode_data_179 = *(test_179.data());
					decode_data_179_t.setvalue(decode_data_179);
					icvPublish("data_179",&decode_data_179_t);
					//ICV_LOG_DEBUG << "0x179 - SCU_IPC_CurrentGearLev :" << (uint16_t)decode_data_179.CurrentGearLev;
					break;
				}
				case 0x17A:
				{
					test_17A.decode_17A(Tframe.frame.data);
					CanFrame_SCU_IPC_7_0x17A decode_data_17A = *(test_17A.data());
					//ICV_LOG_DEBUG << "0x17A - SCU_IPC_ErrSt :" << (uint16_t)decode_data_17A.ErrSt;
					break;
				}
				default:
					break;
				} //end switch
			}	  //end while
		}		  //end if
		else
		{
			//std::cout << "Error: have not received can data, or byte number is wrong!" << std::endl;
		}

	} //end process

private:
	float steeringdata;
	//
	static const int MaxUdpBufferSize = 1024;
	// uint8 udpBuffer[MaxUdpBufferSize]; //received udp buffer
	// uint8 sendBuffer[104]; //received udp buffer
	CanFrame CanFrame_sent;
	//vector<TimestampedCanFrame> frames;
	bool send_can_ = false, recei_can_ = true;

	Can_SCU_IPC_1_0x174 test_174;
	Can_SCU_IPC_2_0x175 test_175;
	Can_SCU_IPC_3_0x176 test_176;
	Can_SCU_IPC_4_0x177 test_177;
	Can_SCU_IPC_5_0x178 test_178;
	Can_SCU_IPC_6_0x179 test_179;
	Can_SCU_IPC_7_0x17A test_17A;

    SCU_IPC_0x174  decode_data_174_t;
	SCU_IPC_0x175  decode_data_175_t;
	SCU_IPC_0x176  decode_data_176_t;
	SCU_IPC_0x177  decode_data_177_t;
	SCU_IPC_0x178  decode_data_178_t;
	SCU_IPC_0x179  decode_data_179_t;
	SCU_IPC_0x17A  decode_data_17A_t;
	
	PID pid;

	//icv_CanFrameMotor_50B* icv_testMotor_50B;
	//inputmessage message_T_exam;
	std::bitset<64> can_sent;
	std::bitset<8> temp_8;

	int startbit, length;
	int16 temp_ag = 0;
	int index;

	uint8 MsgCounter_171 = 0;
	uint8 MsgCounter_172 = 0;
	uint8 MsgCounter_173 = 0;

	std::bitset<8> can_sent_bit[8];

	int count_ = 0;
	int count_2 = 0;
	float ax_send = 0, steerangle_send = 0;
	uint8 EPS_ACUAbortfeedback_l = 1;
	uint8 EPS_ACUEpasFAILED_l = 1;
	uint8 EPS_EPSFailed_l = 1;
	uint8 ESP_QDCACC_l = 1;
	uint8 controlfeedback_l = 0;

	 icvdata_demo_type2 ACC_STR;
	 data_demo_type2 acc_steer;

	icv::data::icvStructureData<NavSatFix> GPS_DATA;
	NavSatFix gps_data;
	icv::data::icvStructureData<TwistWithCovarianceStamped> VEL_DATA;
	TwistWithCovarianceStamped vel_data;
	Imu imu_data;
	icv::data::icvStructureData<Imu> IMU_DATA;
};
ICV_REGISTER_FUNCTION(P7Dynamics)

#endif //
