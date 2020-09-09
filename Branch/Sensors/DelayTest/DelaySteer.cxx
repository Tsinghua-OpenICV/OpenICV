/*
*  code for test steering command delay
*  1. send a steering commande at certain velocity
*  2. record the command send time and the time when the steering angle reaches at certain values
*
*  Author: Shengjie Kou
*  Contact me at ksj18@mails.tsinghua.edu.cn for any questions  
*/

#ifndef _DelaySteer_H
#define _DelaySteer_H



#include <cstdlib>
#include <string>
#include <sstream>

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Basis/icvSemaphore.h"
#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/structure/structureVehinfo.h"
#include "OpenICV/Basis/icvStructureData.hxx"

#include  "CanEPS_511.hpp" 
#include  "CanSPD_510.hpp" 
#include  "CanBT_509.hpp" 
#include  "CanMotor_50B.hpp" 
#include  "CanCOM_5F0.hpp"  

#include "CanFrame.h"

#include <iomanip>
#include <iostream>
#include <boost/thread/thread.hpp>
#include<cmath>
#include<algorithm>
#include<iostream>  
#include<bitset> 
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

class DelaySteer: public icvUdpReceiverSource
{
public:

DelaySteer(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info){

	// 从json文件读取信息
	if(_information.Contains("vx"))
		vx_send_=_information.GetDecimal("vx");
	if(_information.Contains("steer"))
		steerangle_send_=_information.GetDecimal("steer");
	if(_information.Contains("receive"))
		recei_can_=_information.GetBoolean("receive");
	if(_information.Contains("send"))
		send_can_=_information.GetBoolean("send");
	
	timesync_=new icvSemaphore(1);
	icv_thread loop(bind(&DelaySteer::InnerLoop, this));
	ICV_LOG_INFO<<"Required Speed: "<<(vx_send_*0.00390625*3.6)<<"km/h, Required Steering Angle: "<<steerangle_send_ ;
};


void sendcan( int16 steeringAngleIn, int16 wheelspeedIn, uint8 control_mode, uint8 alarm_level, uint8 light_level)
{
	uint8 sendBuf5f1[13]; 
	uint8 automode1;
	uint8 voicealarm1;
	uint8 turnlight1;
	uint16 epsangele_req1;
	int16  tarspeed_req1;
	float  mid_epsangle_req1;

	mid_epsangle_req1 = steeringAngleIn + 30000;

	epsangele_req1=(uint16) mid_epsangle_req1;

	tarspeed_req1=wheelspeedIn; 

	if(control_mode==0x00 || control_mode==0x01) automode1=control_mode; 
	//   else automode1=automode1;

	if(alarm_level>=0 && alarm_level<=3) voicealarm1=alarm_level;
	//ICV_LOG_INFO<<"XIAOPENG alarm1";
	//   else voicealarm1=voicealarm1;

	if(light_level>=0 && light_level<=4) turnlight1=light_level; 
	//   else turnlight1=turnlight1;
	if(epsangele_req1>=60000) epsangele_req1=60000;
		else if (epsangele_req1<=0) epsangele_req1=0;   

	if(tarspeed_req1>=2048) tarspeed_req1=2048; 
		else if(tarspeed_req1<=-2048) tarspeed_req1=-2048;
	//=======================0f0============================
	memset(sendBuf5f1, 0, sizeof(sendBuf5f1));
	sendBuf5f1[0] = 0x08;  //0x08
	sendBuf5f1[1] = 0x00;
	sendBuf5f1[2] = 0x00; 
	sendBuf5f1[3] = 0x05;
	sendBuf5f1[4] = 0xf0;
	//////////////data segment/////////////
	sendBuf5f1[5] = (voicealarm1<<4) | (automode1); 
	sendBuf5f1[6] = tarspeed_req1;
	sendBuf5f1[7] = tarspeed_req1>>8;  
	sendBuf5f1[8] = 0x00;
	sendBuf5f1[9] = 0x00;
	sendBuf5f1[10]= epsangele_req1;
	sendBuf5f1[11]= epsangele_req1>>8;
	sendBuf5f1[12]= turnlight1; 

	send(sendBuf5f1,13);
}

void InnerLoop()
{
	while(true)
	{
		timesync_->Release();
		usleep(500);
	}
}

virtual void Process(std::istream& stream) override
{
	timesync_->Lock();   
	count_++;
	if (send_can_)
	{
		int begincount=1000; 		
		if ((int16)data4.EpbState_CCP !=0)
		{
			sendcan(0, vx_send_, 1, 0, 3);
			sleep(0.2);
			begincount=count_; 
		}
		else if (count_ < begincount + 20)
		{
			sendcan(0, 0, 1, 0, 1);
		}
		else
		{
			sendcan(steerangle_send_, vx_send_, 1, 0, 3);
		}	
	}

	if(recei_can_)
	{
	if (_buffer.size() % 13 == 0)
	{
		while (!stream.eof())
		{
			char udpBuffer[13];
			stream.read(udpBuffer, 13);

			uint16 i, j;
			int flag = 0;
			TimestampedCanFrame Tframe;
			Tframe.frame.id = ((uint16)(udpBuffer[3] << 8) + udpBuffer[4]);
			for (j = 0; j < 8; j++)
			{
				Tframe.frame.data[j] = udpBuffer[j + 5];
			}
			switch (Tframe.frame.id)
			{
				case 0x510: 
				{
					testSPD_510.SetData(Tframe);
					testSPD_510.decode();
					data1 = *(testSPD_510.data());
					vel_ = (data1.LR_WhlSpd + data1.RR_WhlSpd)*0.01 ;
					float send_speed_kmph = vx_send_*0.00390625*3.6 ;
					if (vel_ < 10) // 10 为设定车速,在该车速下测转向响应
					{
						vx_send_ = 10/3.6/0.00390625 ;
						steerangle_send_ = 0 ;
					}
					else if (vel_ >= 10) // 车速到达设定车速之后, 开始发送转向指令
					{
						vx_send_ = 10/3.6/0.00390625;
						if (first_send_)
						{
							send_time_ = SyncClock::time_us();
							ICV_LOG_INFO<<"Command send time: "<<send_time_ ;
							first_send_ = false ;
							send_steer_flag_ = true;
						}
					}
					// ICV_LOG_DEBUG<<"left wheel speed :"<<data1.LF_WhlSpd;
					// ICV_LOG_DEBUG<<"right wheel speed :"<<data1.RF_WhlSpd;
					break;
				}

				case 0x511: 
				{
					testEPS_511.SetData(Tframe);
					testEPS_511.decode();
					data2 = *(testEPS_511.data());
					eps_angle_ = data2.EPS_angle_ccp;
					// 如果达到设定车速,send_steer_flag为true,开始发送转向指令;每隔90°发送一次指令
					if (send_steer_flag_)
					{
						if (eps_angle_<4500)
						{
							steerangle_send_ = 4500;
						}
						else if ((eps_angle_>=4500)&&(eps_angle_<9000))
						{
							steerangle_send_ = 9000;
							if (first_90_)
							{
								time_90_ = SyncClock::time_us();
								ICV_LOG_INFO<<"steer delay 0-90: "<<(time_90_-send_time_)/1000<<" ms";
								first_90_=false;
							}
						}

						else if ((eps_angle_>=9000)&&(eps_angle_<3*4500))
						{
							steerangle_send_ = 3*4500;
							if (first_180_)
							{
								time_180_ = SyncClock::time_us();
								ICV_LOG_INFO<<"steer delay 90-180: "<<(time_180_-time_90_)/1000<<" ms";
								first_180_=false;
							}
						}
						else if ((eps_angle_>=3*4500)&&(eps_angle_<4*4500))
						{
							steerangle_send_ = 4*4500;
							if (first_270_)
							{
								time_270_ = SyncClock::time_us();
								ICV_LOG_INFO<<"steer delay 180-270: "<<(time_270_-time_180_)/1000<<" ms";
								first_270_=false;
							}
						}
						else if ((eps_angle_>=4*4500)&&(eps_angle_<5*4500))
						{
							steerangle_send_ = 5*4500;
							if (first_360_)
							{
								time_360_ = SyncClock::time_us();
								ICV_LOG_INFO<<"steer delay 270-360: "<<(time_360_-time_270_)/1000<<" ms";
								first_360_=false;
							}
						}
						else if ((eps_angle_>=5*4500)&&(eps_angle_<6*4500))
						{
							steerangle_send_ = 6*4500;
							if (first_450_)
							{
								time_450_ = SyncClock::time_us();
								ICV_LOG_INFO<<"steer delay 360-450: "<<(time_450_-time_360_)/1000<<" ms";
								first_450_=false;
							}
						}
						else if (eps_angle_>=6*4500)
						{
							steerangle_send_ = 6*4500;
							if (first_540_)
							{
								time_540_ = SyncClock::time_us();
								ICV_LOG_INFO<<"steer delay 450-540: "<<(time_540_-time_450_)/1000<<" ms";
								first_540_=false;
							}
						}
					}
					//ICV_LOG_DEBUG<<"EPS angle :"<<data2.EPS_angle_ccp;
					// ICV_LOG_DEBUG<<"EPS whl torq :"<<data2.EPS_StrngWhlTorq;
					break;
				}

				case 0x509: 
				{
					testBT_509.SetData(Tframe);
					testBT_509.decode();
					data3 = *(testBT_509.data());
					// ICV_LOG_DEBUG<<"light state :"<<(int)(data3.State_TurningLight_CCP);
					// ICV_LOG_DEBUG<<"drive mode :"<<(int)(data3.CurDriveMode_CCP);
					break;
				}

				case 0x50B: 
				{
					testMotor_50B.SetData(Tframe);
					testMotor_50B.decode();
					data4 = *(testMotor_50B.data());
					// ICV_LOG_DEBUG<<"acc pedal :"<<(int)(data4.AccPedal_CCP);
					// ICV_LOG_DEBUG<<"brk pedal :"<<(int)(data4.BrkPedal_CCP);
					break;
				}

			}//end switch
		}//end while
	}//end if 
	else
	{
		std::cout<<"Error: have not received can data, or byte number is wrong!"<<std::endl;
	}
}
}//end process	
	
private:
	float vel_, eps_angle_ ;
	float vx_send_, steerangle_send_ ;
	bool send_can_=false, recei_can_=true;
	bool first_send_ = true, send_steer_flag_ = false ;
	time_t send_time_, time_90_, time_180_, time_270_, time_360_, time_450_, time_540_;
	bool first_90_=true, first_180_=true, first_270_=true, first_360_=true, first_450_=true, first_540_=true;
	CanSPD_510   testSPD_510;
	CanBT_509    testBT_509;
	CanEPS_511   testEPS_511;
	CanMotor_50B testMotor_50B;
	CanCOM_5F0   testCOM_5F0;
	CanFrameSPD_510 data1;
	CanFrameEPS_511 data2;
	CanFrameBT_509 data3;
	CanFrameMotor_50B data4;
	int count_=0; 
	icvSemaphore* timesync_;
};
ICV_REGISTER_FUNCTION(DelaySteer)

#endif  //
