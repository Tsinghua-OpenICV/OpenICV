//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _XIAOPENGDYNAMICS_H
#define _XIAOPENGDYNAMICS_H


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

#include  "CanEPS_511.hpp" 
#include  "CanSPD_510.hpp" 
#include  "CanBT_509.hpp" 
#include  "CanMotor_50B.hpp" 
#include  "CanCOM_5F0.hpp"  

#include "CanFrame.h"
#include "inputmessage.h"

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


class XiaoPengDynamics: public icvUdpReceiverSource
{
public:
typedef data::icvStructureData<CanFrameMotor_50B> icv_CanFrameMotor_50B;

  XiaoPengDynamics(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info){

	if(_information.Contains("ax"))ax_send=_information.GetDecimal("ax");
	if(_information.Contains("steer"))steerangle_send=_information.GetDecimal("steer");
	if(_information.Contains("receive"))recei_can_=_information.GetBoolean("receive");
	if(_information.Contains("send"))send_can_=_information.GetBoolean("send");

    //ICV_LOG_INFO<<"XIAOPENG send message once";
	icv_testMotor_50B=new icv_CanFrameMotor_50B();

	    };


//  virtual void ConfigurateInput(std::vector<icvNodeInput*>& inputPorts) override {			
//     		ini_Input(inputPorts,0);
// }

//   virtual void ConfigurateOutput(std::vector<icvNodeOutput*>& outputPorts) override {
    
//          			ini_Output(outputPorts,0);
// // icvDataObject* outdataini[4];

// // outdataini[0]=new icvImu;outdataini[0]->Reserve();
// // outdataini[1]=new icvNavSatFix;outdataini[1]->Reserve();
// // outdataini[2]=new icvTwistWithCovarianceStamped;outdataini[2]->Reserve();
// // outdataini[3]=new icvOdometry;outdataini[3]->Reserve();

// //   outputPorts[0]->SetDataObject(outdataini[0]);
// //   outputPorts[1]->SetDataObject(outdataini[1]);
// //   outputPorts[2]->SetDataObject(outdataini[2]);
// //   outputPorts[3]->SetDataObject(outdataini[3]);

//   }

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

	//if(epsangele_req1>=60000) epsangele_req1=60000;
	//   else if (epsangele_req1<=0) epsangele_req1=0;

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
        //std::cout<<"eps angle request "<<epsangele_req1<<std::endl;
		//std::cout<<"angle L "<<(int16)sendBuf5f1[10]<<std::endl;
		//std::cout<<"angle H "<<(int16)sendBuf5f1[11]<<std::endl;
		//std::cout<<"-------Send control message ---------"<<std::endl;

  }

    void TranstoCanFrame(inputmessage &message_T)
{
	uint8 sendBuf5f0[13]; 
    uint8 automode=message_T.automode;
    uint8 voicealarm=message_T.voicealarm;
   // ICV_LOG_INFO<<"XIAOPENG alarm2";

    uint8 turnlight=message_T.turnlight;
    uint16 epsangele_req=message_T.epsangele_req;
    int16  tarspeed_req=message_T.tarspeed_req;

	if(message_T.automode==0x00 || message_T.automode==0x01) automode=message_T.automode; 
//	   else automode=automode;//

	if(message_T.voicealarm>=0 || message_T.voicealarm<=3) voicealarm=message_T.voicealarm; 
//	   else voicealarm=voicealarm;//

	if(message_T.turnlight>=0 || message_T.turnlight<=4) turnlight=message_T.turnlight; 
//	   else turnlight=turnlight;//

	if(epsangele_req>=30000) epsangele_req=30000;
	   else if (epsangele_req<=-30000) epsangele_req=-30000;

	if(tarspeed_req>=2048) tarspeed_req=2048; 
	   else if(tarspeed_req<=-2048) tarspeed_req=-2048;
	   
//=======================0f0============================
        memset(sendBuf5f0, 0, sizeof(sendBuf5f0));
        sendBuf5f0[0] = 0x08;
        sendBuf5f0[1] = 0x00;
        sendBuf5f0[2] = 0x00; 
        sendBuf5f0[3] = 0x05;
        sendBuf5f0[4] = 0xf0;
//////////////data segment/////////////
        sendBuf5f0[5] = (voicealarm<<4) | (automode); 
        sendBuf5f0[6] = tarspeed_req && 0xff;
        sendBuf5f0[7] = tarspeed_req>>8;  
        sendBuf5f0[8] = 0x00;
        sendBuf5f0[9] = 0x00;
        sendBuf5f0[10]= epsangele_req && 0xff;
        sendBuf5f0[11]= epsangele_req>>8;
        sendBuf5f0[12]= turnlight; 

	    send(sendBuf5f0,13);
//        std::cout<<"eps angle request"<<message_T.epsangele_req<<std::endl;
}



  virtual void Process(std::istream& stream) override
  {
	  	     count_++;

		if (send_can_)
		{
		//steeringdata = read_Input<data::icvFloat32Data>(0);
		// ICV_LOG_INFO<<"CAN:steering"<<steeringdata;
//(steeringAngleIn,wheelspeedIn,control_mode,alarm_level,light_level)
      // if (count_<1) sendcan( steeringdata, 0, 1, 0, 0);
	    
	   sendcan(0 , 0, 1, 0, 3);
//		 usleep(1000000);
//		 ICV_LOG_INFO<<"send can";
		}

		if(recei_can_)

		{
				if (_buffer.size() % 13 == 0)   //13 
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
							CanFrameSPD_510 data1 = *(testSPD_510.data());
							ICV_LOG_DEBUG<<"left wheel speed :"<<data1.LF_WhlSpd;
							ICV_LOG_DEBUG<<"right wheel speed :"<<data1.RF_WhlSpd;
							break;
							}

							case 0x511: 
							{
							testEPS_511.SetData(Tframe);
							testEPS_511.decode();
							CanFrameEPS_511 data2 = *(testEPS_511.data());
							ICV_LOG_DEBUG<<"EPS angle :"<<data2.EPS_angle_ccp;
							ICV_LOG_DEBUG<<"EPS whl torq :"<<data2.EPS_StrngWhlTorq;

							break;
							}

							case 0x509: 
							{
							testBT_509.SetData(Tframe);
							testBT_509.decode();
							CanFrameBT_509 data3 = *(testBT_509.data());
							ICV_LOG_DEBUG<<"light state :"<<(int)(data3.State_TurningLight_CCP);
							ICV_LOG_DEBUG<<"drive mode :"<<(int)(data3.CurDriveMode_CCP);

							break;
							}

							case 0x50B: 
							{
							testMotor_50B.SetData(Tframe);
							testMotor_50B.decode();
							CanFrameMotor_50B data4 = *(testMotor_50B.data());
							icv_testMotor_50B->setoutvalue(data4);
							Send_Out(icv_testMotor_50B,0);
							ICV_LOG_DEBUG<<"acc pedal :"<<(int)(data4.AccPedal_CCP);
							ICV_LOG_DEBUG<<"brk pedal :"<<(int)(data4.BrkPedal_CCP);

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
  
 float steeringdata;
  //邓楠山结束
  static const int MaxUdpBufferSize = 1024;
 // uint8 udpBuffer[MaxUdpBufferSize]; //received udp buffer
 // uint8 sendBuffer[104]; //received udp buffer
  CanFrame CanFrame_sent;
  //vector<TimestampedCanFrame> frames;
bool send_can_=false, recei_can_=true;
  CanSPD_510   testSPD_510;
  CanBT_509    testBT_509;
  CanEPS_511   testEPS_511;
  CanMotor_50B testMotor_50B;
  CanCOM_5F0   testCOM_5F0;
icv_CanFrameMotor_50B *icv_testMotor_50B;
    inputmessage message_T_exam;
	std::bitset<64> can_sent; 
	std::bitset<8> temp_8; 
	
	int startbit,length;
	
	std::bitset<8> can_sent_bit[8];
	
	int count_=0;
	int count_2=0;
	float ax_send=0,steerangle_send=0;

   ////

   ////
	

};
ICV_REGISTER_FUNCTION(XiaoPengDynamics)

#endif  //
