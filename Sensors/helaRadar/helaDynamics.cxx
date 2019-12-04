//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//

#ifndef _helaRadar_H
#define _helaRadar_H

#include <cstdlib>
#include <string>
#include <sstream>

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Net/icvUdpReceiverSource.h"

#include "OpenICV/structure/structureVehinfo.h"
#include "OpenICV/Basis/icvStructureData.hxx"

#include "CanEPS_511.hpp"
#include "CanSPD_510.hpp"
#include "CanBT_509.hpp"
#include "CanMotor_50B.hpp"
#include "CanCOM_5F0.hpp"
#include "structureCanXP.h"

#include "CanFrame.h"

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

class helaRadar : public icvUdpReceiverSource
{
  public:
	helaRadar(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info)
	{

		if (_information.Contains("ax"))
			ax_send = _information.GetDecimal("ax");
		if (_information.Contains("steer"))
			steerangle_send = _information.GetDecimal("steer");
		if (_information.Contains("receive"))
			recei_can_ = _information.GetBoolean("receive");
		if (_information.Contains("send"))
			send_can_ = _information.GetBoolean("send");
	};


	void sendcan(float ESC_FLWheelSpdIn,float ESC_FRWheelSpdIn,float steeringAngleIn, int GearStateIn)
	{
		float steeringangle_value=0.0;
		float speed_value=0.0;
		float yawrate_value=0.0;
        float VehicleStandstill_thr = 0.5;
		int VehicleStandstill ;
		float L=2.0 ;
		int Gear=0;
		int MovingDirection=0;
		int SteeringAngleSign=1;
		int YawRateSign=0;
		int SteeringAngle=0;
		int YawRate=0;
		int Speed=0;
		uint8 sendBuf5f1[13];
		uint8 sendBuf5f2[13];
		//======================get value==========================
		steeringangle_value = steeringAngleIn * 0.02;
		speed_value = (ESC_FLWheelSpdIn + ESC_FRWheelSpdIn)*0.01/2.0;
		yawrate_value = speed_value*steeringangle_value / L;

        if(GearStateIn==1)//D
			Gear = 2;
        else if(GearStateIn==2)//N
            Gear = 4;
        else if(GearStateIn==3)//R
			Gear = 3;
        else if(GearStateIn==4)//P
			Gear = 1;

		if(fabs(speed_value)>VehicleStandstill_thr)
		{
			VehicleStandstill = 0;
			if(speed_value>0.0)
				MovingDirection = 1;
			else
				MovingDirection = 2;
		}
		if(fabs(speed_value)<=VehicleStandstill_thr)
		{
			VehicleStandstill = 1;
			MovingDirection = 3;
		}					
		if(yawrate_value<0.0)
			YawRateSign =1;
		if(steeringangle_value<0.0)
			SteeringAngleSign = 1;
		SteeringAngle = int(fabs(steeringangle_value)/0.1);
		YawRate = int(fabs(yawrate_value)/0.01);
		Speed = int(fabs(speed_value)/0.01);
           	//======================encode==========================

		//=======================0f0============================
		memset(sendBuf5f1, 0, sizeof(sendBuf5f1));
		memset(sendBuf5f2, 0, sizeof(sendBuf5f2));
         	//////////////send 100/////////////
		sendBuf5f1[0] = 0x08; //0x08
		sendBuf5f1[1] = 0x00;
		sendBuf5f1[2] = 0x00;
		sendBuf5f1[3] = 0x01;
		sendBuf5f1[4] = 0x00;
		//////////////data segment/////////////
		sendBuf5f1[5] = Speed & 0x00ff;//((voicealarm1 << 4) | (automode1))& 0xff;//dns 调试
		sendBuf5f1[6] = (Speed & 0xff00)>> 8;//tarspeed_req1 & 0x00ff;
		if(YawRate<pow(2,14))
		{
			if(YawRateSign ==1)
				YawRate = YawRate + pow(2,14);
			if(VehicleStandstill == 1)
				YawRate = YawRate + pow(2,15);
		}
		else
		{
			YawRate = pow(2,14)-1;
			if(YawRateSign ==1)
				YawRate = YawRate + pow(2,14);
			if(VehicleStandstill == 1)
				YawRate = YawRate + pow(2,15);
		}
		sendBuf5f1[7] = YawRate & 0x00ff;//(tarspeed_req1 & 0xff00) >> 8;
		sendBuf5f1[8] = (YawRate & 0xff00)>> 8;
		
		sendBuf5f1[9] = (SteeringAngle & 0x1fe0)>>8;
		sendBuf5f1[10] = SteeringAngle & (0x001f)<<3;
		sendBuf5f1[11] = 0x00;
		sendBuf5f1[12] = (SteeringAngleSign & 0x01)<<7 + (MovingDirection & 0x03)<<5;
                /////////////////////send 101////////////////////
        sendBuf5f2[0] = 0x08; //0x08
		sendBuf5f2[1] = 0x00;
		sendBuf5f2[2] = 0x00;
		sendBuf5f2[3] = 0x01;
		sendBuf5f2[4] = 0x01;
		//////////////data segment/////////////
		sendBuf5f2[5] = 0x00;
		sendBuf5f2[6] = 0x00;
		sendBuf5f2[7] = 0x00;
		sendBuf5f2[8] = 0x00;
		sendBuf5f2[9] = (Gear & 0xff) << 5;
		sendBuf5f2[10] = 0x00;
		sendBuf5f2[11] = 0x00;
		sendBuf5f2[12] = 0x00;
		send(sendBuf5f1, 13);
		send(sendBuf5f2, 13);

		//std::cout << "voice and mode " << (int16)sendBuf5f1[5] << std::endl;
		//std::cout << "speed L " << (int16)sendBuf5f1[6] << std::endl;
		//std::cout << "speed H " << (int16)sendBuf5f1[7] << std::endl;
		//std::cout<<"eps angle request "<<epsangele_req1<<std::endl;
		//std::cout<<"angle L "<<(int16)sendBuf5f1[10]<<std::endl;
		//std::cout<<"angle H "<<(int16)sendBuf5f1[11]<<std::endl;
		//std::cout<<"-------Send control message ---------"<<std::endl;
	}

	

	virtual void Process(std::istream &stream) override
	{
		count_++;
		//ICV_LOG_INFO<<"count_"<<count_;
		if (send_can_)
		{

			if( ((int16)data1.LF_WhlSpd !=0) || ((int16)data2.EPS_angle_ccp !=0) || ((int16)data4.GearState_CCP !=0))
				sendcan(float(data1.LF_WhlSpd),float(data1.RF_WhlSpd),float(data2.EPS_angle_ccp),data4.GearState_CCP);
			else			
				sendcan(0, 0, 0, 0);
		}

		if (recei_can_)
		{
			if (_buffer.size() % 13 == 0) //13
			{
				while (!stream.eof())
				{
					char udpBuffer[13];
					stream.read(udpBuffer, 13);

					uint16 i, j;

					//lrr
					int flag = 0;
					TimestampedCanFrame Tframe;
					Tframe.frame.id = ((uint16)(udpBuffer[3] << 8) + udpBuffer[4]);

					//ICV_LOG_DEBUG<<"reading";
					// fill up udpBufferGroup
					for (j = 0; j < 8; j++)
					{
						Tframe.frame.data[j] = udpBuffer[j + 5];
					}
					switch (Tframe.frame.id)
					{
					//add  new
					case 0x510:
					{
						testSPD_510.SetData(Tframe);
						testSPD_510.decode();
						data1 = *(testSPD_510.data());
						ICV_LOG_DEBUG << "left wheel speed :" << data1.LF_WhlSpd;
						ICV_LOG_DEBUG << "right wheel speed :" << data1.RF_WhlSpd;
						ICV_LOG_DEBUG << "right wheel speed :" << data1.LR_WhlSpd;
						ICV_LOG_DEBUG << "right wheel speed :" << data1.RR_WhlSpd;
						break;
					}

					case 0x511:
					{
						testEPS_511.SetData(Tframe);
						testEPS_511.decode();
						data2 = *(testEPS_511.data());
						ICV_LOG_DEBUG<<"EPS angle :"<<data2.EPS_angle_ccp;
						//ICV_LOG_DEBUG<<"EPS whl torq :"<<data2.EPS_StrngWhlTorq;

						break;
					}

					case 0x509:
					{
						testBT_509.SetData(Tframe);
						testBT_509.decode();
						data3 = *(testBT_509.data());
						//ICV_LOG_DEBUG<<"light state :"<<(int)(data3.State_TurningLight_CCP);
						//ICV_LOG_DEBUG<<"drive mode :"<<(int)(data3.CurDriveMode_CCP);

						break;
					}

					case 0x50B:
					{
						testMotor_50B.SetData(Tframe);
						testMotor_50B.decode();
						data4 = *(testMotor_50B.data());
						ICV_LOG_DEBUG<<"gear state :"<<(int)(data4.GearState_CCP);
						//ICV_LOG_DEBUG<<"brk pedal :"<<(int)(data4.BrkPedal_CCP);
						// epb 0松开 1 拉紧 2 松开过程 4拉紧过程
						//ICV_LOG_DEBUG<<"EpbState_CCP:"<<(int)(data4.EpbState_CCP);
						break;
					}

					} //end switch
				}	 //end while
			}		  //end if
			else
			{
				std::cout << "Error: have not received can data, or byte number is wrong!" << std::endl;
			}
		}

	} //end process

  private:
	//dns begin
	CanFrameSPD_510 data1;
	CanFrameEPS_511 data2;
	CanFrameBT_509 data3;
	CanFrameMotor_50B data4;

	static const int MaxUdpBufferSize = 1024;
	// uint8 udpBuffer[MaxUdpBufferSize]; //received udp buffer
	// uint8 sendBuffer[104]; //received udp buffer
	CanFrame CanFrame_sent;
	//vector<TimestampedCanFrame> frames;
	bool send_can_ = true, recei_can_ = true;
	CanSPD_510 testSPD_510;
	CanBT_509 testBT_509;
	CanEPS_511 testEPS_511;
	CanMotor_50B testMotor_50B;
	CanCOM_5F0 testCOM_5F0;
	std::bitset<64> can_sent;
	std::bitset<8> temp_8;

	int startbit, length;

	std::bitset<8> can_sent_bit[8];

	int count_ = 0;
	int count_2 = 0;
	float ax_send = 0, steerangle_send = 0;

	////

	////
};
ICV_REGISTER_FUNCTION(helaRadar)

#endif //
