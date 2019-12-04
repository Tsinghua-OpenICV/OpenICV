//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _TuanDynamics_H
#define _TuanDynamics_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/structure/structureCanTuan.h"
#include "OpenICV/structure/structureVehinfo.h"
#include "OpenICV/Net/icvUdpReceiverSource.h"


#include <cstdlib>
#include <string>
#include <sstream>
#include  "CanTuan641.hpp" 
#include  "CanTuan130.hpp" 
#include  "CanTuan126.hpp" 
#include  "CanTuan122.hpp" 
#include  "CanTuan121.hpp" 
#include  "CanTuan120.hpp" 
#include  "CanTuan116.hpp" 
#include  "CanTuan106.hpp" 
#include  "CanTuan101.hpp" 
#include  "CanTuan086.hpp" 
#include  "CanTuan30C.hpp" 
#include  "CanTuan13X.hpp" 
#include  "CanTuan12E.hpp" 
#include  "CanTuan6FF.hpp" 
#include  "CanTuan09F.hpp" 
#include  "CanTuan3DC.hpp" 
#include  "CanTuan3BE.hpp" 
#include  "CanTuan1AB.hpp" 
#include  "CanTuan0FD.hpp" 
#include  "CanTuan0B2.hpp"
#include  "CanTuan0AD.hpp" 
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



using namespace icv;
using namespace std;
using namespace core;
using namespace icv::function;
typedef data::icvStructureData<veh_info>    icvveh_info;

class TuanDynamics: public icvUdpReceiverSource
{
public:
  TuanDynamics(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info){

	if(_information.Contains("ax"))ax_send=_information.GetDecimal("ax");
	if(_information.Contains("steer"))steerangle_send=_information.GetDecimal("steer");


	    };
//   virtual void ConfigurateOutput(std::vector<icvNodeOutput*>& outputPorts) override {
// 	 // CheckDataType<icvDoubleData>(outputPorts[0]);
// ini_Output(outputPorts,1);

// 			icvDataObject *datai=new icvveh_info();
// 	   outputPorts[0]->SetDataObject( datai);

//   }

//   virtual void ConfigurateInput(std::vector<icvNodeInput*>& inputPorts) override {
// ini_Input(inputPorts,2);
// 	       icvNodeOutput* output_con1= inputPorts[0]->GetConnections()[0];
// 	  bool correct;
// 	   if(output_con1->RequireDataObject(false))
//              correct= dynamic_cast<data::icvDoubleData*>(output_con1->RequireDataObject(false));
//         else
//         {
//            // output_con1->SetDataObject((icvDataObject*)(new data::icvDoubleData));
//            // correct= true;
//         }
// 		if (!correct)ICV_LOG_TRACE<<"Data type not matched";
// 		else ICV_LOG_TRACE<<"Data type  matched";
// 		       icvNodeOutput* output_con2= inputPorts[1]->GetConnections()[0];
// 	   if(output_con1->RequireDataObject(false))
//              correct= dynamic_cast<data::icvDoubleData*>(output_con2->RequireDataObject(false));
//         else
//         {
//            //output_con2->SetDataObject((icvDataObject*)(new data::icvDoubleData));
//             //correct= true;
//         }
// 		if (!correct)ICV_LOG_TRACE<<"Data type not matched";
// 		else ICV_LOG_TRACE<<"Data type  matched";



//   }

void sendcan(float acclerationIn, float steeringAngleIn, float steeringtorqueIn, int steer_control_flag)
  {

			float accControl;
			float steeringAngleControl;
			int accActive;
			int steerActive;
			int steerOri;
			bool flagaa;

			float steeringtorqueControl;
			int steer_torqueActive;
			int steer_torqueOri;


			if (acclerationIn < -7.0){
				acclerationIn = -7.0;
				accActive = 1;
			}
			else if(acclerationIn > 3.0){
				acclerationIn = 3.0;
				accActive = 1;
			}
			else{
				accActive = 1;
			}
//steeringAngle control   
			if(steer_control_flag==false){
				if (steeringAngleIn<0.0){
				steeringAngleIn = -steeringAngleIn;
				steerOri = 1;
				steerActive = 1;
				}
				else{
				steerOri = 0;
				steerActive = 1;
				}
				if (steeringAngleIn>487.0)
				steeringAngleIn = 487.0;
			}
			//steeringtorque control
			if(steer_control_flag==true){
				if (steeringtorqueIn<0.0){
				steeringtorqueIn = -steeringtorqueIn;
				steer_torqueOri = 1;
				steer_torqueActive = 1;
				}
				else{
				steer_torqueOri = 0;
				steer_torqueActive = 1;
				}

				if (steeringtorqueIn>5.1)
				steeringtorqueIn = 5;
			}

			accControl = acclerationIn;
			steeringAngleControl = steeringAngleIn;
			steeringtorqueControl = steeringtorqueIn;

			uint8 sendBuf6ff[13]; 
  // ros::Rate loop_rate(20);
        int accCan = (int)((accControl+7.22)/0.005);
        int steeringAngleCan  = (int)((steeringAngleControl/0.1));
        int steeringtorqueCan = (int)(steeringtorqueControl/0.01);
        bool breakstatus = brake_status1;

        int ret1;
//=======================6ff============================
        memset(sendBuf6ff, 0, sizeof(sendBuf6ff));
        sendBuf6ff[0] = 0x08;
        sendBuf6ff[1] = 0x00;
        sendBuf6ff[2] = 0x00; 
        sendBuf6ff[3] = 0x06;
        sendBuf6ff[4] = 0xff;
//////////////data segment/////////////
      sendBuf6ff[5] = (steerOri<<5)|(steer_torqueOri<<4)|((steerActive && (!breakstatus))<<2)|((steer_torqueActive && (!breakstatus))<<1)|(accActive&& (!breakstatus));    //|按位或，<<左移符號。
        sendBuf6ff[6] = accCan&255;
        sendBuf6ff[7] = (accCan&1792)>>8; 
        sendBuf6ff[8] = steeringtorqueCan&255;
        sendBuf6ff[9] = (steeringtorqueCan&256)>>8;
        sendBuf6ff[10]= steeringAngleCan&255;
        sendBuf6ff[11]= (steeringAngleCan&7936)>>8;
        sendBuf6ff[12]= 0x00;
 
    
       send(sendBuf6ff, 13);

  };


  void TranstoCanFrame(GL1000Message &message_T,uint8 brake_status)
{
		uint8 sendBuf6ff[13]; 
  // ros::Rate loop_rate(20);
 unsigned short SteeringAngle,Torque,Acceleration;
	bool PicWarn=message_T.PicWarn; //23	1
	bool AudioWarn=message_T.AudioWarn; //22	1
	bool SteeringToRight=message_T.SteeringToRight;//5	1
	if(message_T.SteeringAngle<819.1)
	 SteeringAngle=message_T.SteeringAngle*10;else SteeringAngle=0;//40	13
	bool PLA_actif=message_T.PLA_actif; //2	1
	bool TorqueToRight=message_T.TorqueToRight; //4	1
	if(message_T.Torque<5.11)
	 Torque=message_T.Torque*100;else Torque=0; //24	9
	bool HCA_aktif=message_T.HCA_aktif; //1	1
	if(message_T.Acceleration<3.005&&message_T.Acceleration>-7.22)Acceleration=(message_T.Acceleration+7.22)*200; else Acceleration=-1;//8	11
	bool ACC_aktif=message_T.ACC_aktif; //0	1
	
   

//=======================6ff============================
        memset(sendBuf6ff, 0, sizeof(sendBuf6ff));
        sendBuf6ff[0] = 0x08;
        sendBuf6ff[1] = 0x00;
        sendBuf6ff[2] = 0x00; 
        sendBuf6ff[3] = 0x06;
        sendBuf6ff[4] = 0xff;
//////////////data segment/////////////
        sendBuf6ff[5] = (SteeringToRight<<5)|(TorqueToRight<<4)|((PLA_actif && (!brake_status))<<2)|((HCA_aktif && (!brake_status))<<1)|(ACC_aktif&& (!brake_status));    //|按位或，<<左移符號。
        sendBuf6ff[6] = Acceleration&255;
        sendBuf6ff[7] = (Acceleration&1792)>>8; 
        sendBuf6ff[8] = Torque&255;
        sendBuf6ff[9] = (Torque&256)>>8;
        sendBuf6ff[10]= SteeringAngle&255;
        sendBuf6ff[11]= (SteeringAngle&7936)>>8;
        sendBuf6ff[12]= 0x00;
	send(sendBuf6ff,13);
 std::cout<<"steering angle"<<message_T.SteeringAngle<<std::endl;
}

  virtual void Process(std::istream& stream) override
  {

	ax_send=read_Input<data::icvDoubleData>(0);
    steerangle_send=read_Input<data::icvDoubleData>(1);
	  count_++;
	
	if(false)
	 {
		 sendcan(ax_send,steerangle_send,0,0);
	 }

	  if (_buffer.size() % 13 == 0)
	  {
		  while (!stream.eof())
		  {
			  char udpBuffer[13];
			  stream.read(udpBuffer, 13);

			  uint16 i, j;

			  //lrr
			  int flag = 0;
			  //frames.clear();
			  TimestampedCanFrame Tframe;

			  Tframe.frame.id = ((uint16)(udpBuffer[3] << 8) + udpBuffer[4]);


			  // fill up udpBufferGroup
			  for (j = 0; j < 8; j++)
			  {
				  Tframe.frame.data[j] = udpBuffer[j + 5];
			  }
			  switch (Tframe.frame.id)
			  {
			  case 0x086:
				  {
					  testcan086.SetData(Tframe);
				  testcan086.decode();
				  CanFrameTuanLWI_01 data1 = *(CanFrameTuanLWI_01*)(testcan086.data());
				//  std::cout<<"steering angle"<<data1.SteeringAngle<<std::endl;
				dataout.steerAngle=data1.LWI_StrWhlAngleSize;
				dataout.steerDir=data1.LWI_StrWhlAngleDrt;
				dataout.steerSpeed=data1.LWI_StrWhlSpeedSize;
				vehinfoup1=true;
				  ICV_LOG_DEBUG << data1.LWI_StrWhlAngleSize;
				 }

			  case 0x101:
				 { testcan101.SetData(Tframe);
				  testcan101.decode();
				  CanFrameTuanESP_02 data2 = *(CanFrameTuanESP_02*)(testcan101.data());
				  dataout.ax=data2.ax;
				  dataout.YawRate=data2.YawRate;
				  dataout.YawDir=data2.YawToRight;
				  vehinfoup2=true;
			//	std::cout<<"ax"<<data2.ax<<std::endl;
			break;

				  ICV_LOG_DEBUG <<"ax" <<data2.ax;
				  }
			case 0x0AD:
			{
			testcan0AD.SetData(Tframe);
			testcan0AD.decode();
			CanFrameGear_11 data8 = *(CanFrameGear_11*)(testcan0AD.data());


				break;
			}
			  case 0x106:
				{ testcan106.SetData(Tframe);
				  testcan106.decode();

				  CanFrameTuanESP_05 data3 = *(CanFrameTuanESP_05*)(testcan106.data());
			//	  std::cout<<"brake pressure"<<data3.brakepressure<<std::endl;
				  ICV_LOG_DEBUG <<"brake"<< data3.BrakePressure;
				  break;
			  }

			  	case 0x116: 
				  {
					  
			testcan116.SetData(Tframe);
			testcan116.decode();
			CanFrameTuanESP_10 data4 = *(CanFrameTuanESP_10*)(testcan116.data());
			// ICV_LOG_DEBUG <<"brake"<< data3.brakepressure;
			break;

			}
			case 0x0B2: 
			{
				testcan0B2.SetData(Tframe);
				testcan0B2.decode();
			CanFrameTuanESP_19 data5 = *(CanFrameTuanESP_19*)(testcan0B2.data());
			break;
			}
				case 0x0FD: 
			{
				testcan0FD.SetData(Tframe);
			testcan0FD.decode();
			CanFrameTuanESP_21 data6 = *(CanFrameTuanESP_21*)(testcan0FD.data());
			break;
			}
				case 0x6FF: 
				{
			testcan6FF.SetData(Tframe);
			testcan6FF.decode();
			GL1000Message data7 = *(GL1000Message*)(testcan6FF.data());
			break;
			}
			case 0x3BE: 
				{
			testcan3BE.SetData(Tframe);
			testcan3BE.decode();
			CanFrameTouranMotor_14 data9 = *(CanFrameTouranMotor_14*)(testcan3BE.data());
			brake_status1=data9.BrakeSwitch;
			break;
			}
			  //frames.push_back(Tframe);
		  }


		  // outData[0]->As<icv::opencv::icvCvMatData>() = mFrame;
		//  frames.clear();
	  }
	  }
	  else
	  {
		  printf("Error: have not received can data, or byte number is wrong!\n");
	  }

	  if(vehinfoup1&&vehinfoup2) 
	  {
		  send_Output<icvveh_info>(1) = dataout;
	  
	  }
  
  }

  
	
	
private:

  
  static const int MaxUdpBufferSize = 1024;
 // uint8 udpBuffer[MaxUdpBufferSize]; //received udp buffer
 // uint8 sendBuffer[104]; //received udp buffer
 bool vehinfoup1=false,vehinfoup2=false;
CanFrame CanFrame_sent;
  //vector<TimestampedCanFrame> frames;
  CanTuan641 testcan641;
  CanTuan130 testcan130;
  CanTuan126 testcan126;
  CanTuan122 testcan122;
  CanTuan121 testcan121;
  CanTuan120 testcan120;
  CanTuan116 testcan116;
  CanTuan106 testcan106;
  CanTuan101 testcan101;
  CanTuan086 testcan086;
  CanTuan30C testcan30C;
  CanTuan13X testcan13X;
  CanTuan12E testcan12E;  
  CanTuan09F testcan09F;
  CanTuan6FF testcan6FF;
  CanTuan3DC testcan3DC;
  CanTuan3BE testcan3BE;
  CanTuan1AB testcan1AB;
  CanTuan0FD testcan0FD;
  CanTuan0B2 testcan0B2;
  CanTuan0AD testcan0AD;
uint8 brake_status1=0;
veh_info dataout;

  GL1000Message message_T_exam;
	std::bitset<64> can_sent; 
	std::bitset<8> temp_8; int startbit,length;
		std::bitset<8> can_sent_bit[8];
		int count_=0;
		double ax_send=0,steerangle_send=0;

  //CanFrame Can_tosent;

   
};
ICV_REGISTER_FUNCTION(TuanDynamics)

#endif  //
