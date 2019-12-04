#ifndef _RadarContiLong_H
#define _RadarContiLong_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/structure/RadarFrameLong.h"
#include "OpenICV/Basis/icvStructureData.hxx"

#include "CanFrame.h"
#include <boost/thread/thread.hpp>

using namespace icv;
using namespace core;
using namespace icv::function;

class RadarContiLong: public icvUdpReceiverSource
{
public:
  	typedef data::icvStructureData<RadarLongOut>    icvLRRradardata;

  	RadarContiLong(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info)
	{
		temp_resu=new icvLRRradardata();
	};

	void sendRadarConfig() 
	{
        int ret1, ret2,ret3;
        uint8 sendBuf200[13],sendBuf202[13],sendBuf301[13];
        uint16 velocity =0;// (uint16)(vehicleInfo.velocity / 0.0625); // km/h-->m/s, default value: 0
        int16 yawrate = 0;//(int16)(vehicleInfo.yawrate / 0.0625); // rad/s-->deg/s, default value: 0

        memset(sendBuf200, 0, sizeof(sendBuf200));
        sendBuf200[0] = 0x08;
        sendBuf200[1] = 0x00;
        sendBuf200[2] = 0x00;
        sendBuf200[3] = 0x02;
        sendBuf200[4] = 0x00;


        sendBuf200[5] = 0xf9;//  (11111001);sensor id;power not to change
        sendBuf200[6] = 0x19;//  (00011001);64+32+4=100;x2=200m
        sendBuf200[7] = 0x00; // (00000000)
        sendBuf200[8] =0x00;// empty;
        sendBuf200[9] =0x08;// sensor iD:0,output 1:objects,power standard;
        sendBuf200[10]=0x9c;// nvm,sorted by range;extenstion and quality yes;CTRL DElay sendornot false
        sendBuf200[11]=0x01;//rcs standard
        sendBuf200[12]=0x00;//empty
        send(sendBuf200, 13);
		usleep(10000);
		memset(sendBuf202, 0, sizeof(sendBuf202));
                                                                                                                                    
        sendBuf202[0] = 0x08;
        sendBuf202[1] = 0x00;
        sendBuf202[2] = 0x00;                                                                                               
        sendBuf202[3] = 0x02;
        sendBuf202[4] = 0x02;

        sendBuf202[5] = 0x8e; //oject filter,filtering radial distance
        sendBuf202[6] = 0x00;  //00
        sendBuf202[7] = 0x05; //min:5x0.1=0.5m
        sendBuf202[8] = 0x07;//7x256+13x8=1896 x0.1 max 189.6m
        sendBuf202[9] = 0xd0;                                                                                             
        send(sendBuf202, 13);
		usleep(10000);
	}


void procLrrRadarData() 
{
	uint validIdx = 0;
	for (int i = 0; i <=numOfTgt; i++)//LrrMaxTarNum
	{
    if ((lrrRadarRawData[i].rangex > 0)&&(lrrRadarRawData[i].rangex < 200/0.2)&&(lrrRadarRawData[i].isUpdated==1))
  	{
		lrrRadarProcData[validIdx].id = lrrRadarRawData[i].objID;
		lrrRadarProcData[validIdx].flag = 1;
		lrrRadarProcData[validIdx].x = (float)(lrrRadarRawData[i].rangex*0.2); //scale: 0.2
		// ICV_LOG_INFO<<"long rangex: transfer  "<<validIdx<<": "<< (lrrRadarRawData[i].rangex*0.2);

		lrrRadarProcData[validIdx].y = (float)(lrrRadarRawData[i].rangey*0.2); //scale: 0.2
		lrrRadarProcData[validIdx].relspeedx = (float)(lrrRadarRawData[i].speedx*0.25); //scale: 0.25
		lrrRadarProcData[validIdx].relspeedy = (float)(lrrRadarRawData[i].speedy*0.25); //scale: 0.25
		lrrRadarProcData[validIdx].obj_amp = (float)(lrrRadarRawData[i].obj_amp*0.5); //scale: 0.25
		lrrRadarProcData[validIdx].objDynProp = lrrRadarRawData[i].objDynProp ;
		
		
		lrrRadarProcData[validIdx].rangex_rms =(float)(lrrRadarRawData[i].rangex_rms*0.33);
		lrrRadarProcData[validIdx].rangey_rms =(float)(lrrRadarRawData[i].rangey_rms*0.33);
		lrrRadarProcData[validIdx].speedx_rms =(float)(lrrRadarRawData[i].speedx_rms*0.33);
		lrrRadarProcData[validIdx].speedy_rms =(float)(lrrRadarRawData[i].speedy_rms*0.33);
		lrrRadarProcData[validIdx].accx_rms =(float)(lrrRadarRawData[i].accx_rms*0.33);
		lrrRadarProcData[validIdx].accy_rms =(float)(lrrRadarRawData[i].accy_rms*0.33);

		lrrRadarProcData[validIdx].orient_rms =(float)(lrrRadarRawData[i].orient_rms*6.0);

		lrrRadarProcData[validIdx].objProbExist=lrrRadarRawData[i].objProbExist;
		lrrRadarProcData[validIdx].objMeasState=lrrRadarRawData[i].objMeasState;

		lrrRadarProcData[validIdx].accx =(float)(lrrRadarRawData[i].accx*0.01);
		lrrRadarProcData[validIdx].accy =(float)(lrrRadarRawData[i].accy*0.01);

		lrrRadarProcData[validIdx].objClass =  lrrRadarRawData[i].objClass;
		lrrRadarProcData[validIdx].ObjectOrientAngel =(float)(lrrRadarRawData[i].ObjectOrientAngel*0.4);

		lrrRadarProcData[validIdx].ObjectWidth =(float)(lrrRadarRawData[i].ObjectWidth*0.2);
		lrrRadarProcData[validIdx].ObjectLength =(float)(lrrRadarRawData[i].ObjectLength*0.2);
      	validIdx++;
    }
  }
  //"printf(#validIdx: %d\n", validIdx);
}
void radarStatusOutput(TimestampedCanFrame Tframe)
{
	uint8 *pData = Tframe.frame.data;
	uint16 temp,temp1;

	if( Tframe.frame.id==0x201)
	{
		temp =0;
		temp=(short)(pData[0]&0x40);
		temp=temp>>6;
		Configdata201.nvmReadStatus=temp;

		temp =0;
		temp=(short)(pData[0]&0x80);
		temp=temp>>7;
		Configdata201.nvmWriteStatus=temp;

		temp=0;
		temp1=0;
		temp=(short)(pData[1]&0xff);
		temp=temp<<2;
		temp1=(short)(pData[2]&0xc0);
		temp1=temp1>>6;
		temp=temp+temp1;

		Configdata201.maxDistanceCfg=temp*2;

		temp=0;
		temp=(short)(pData[2]&0x20);
		temp=temp>>5;
		Configdata201.persistentError=temp;


		temp=0;
		temp=(short)(pData[2]&0x10);
		temp=temp>>4;
		Configdata201.interface=temp;
		temp=0;
		temp=(short)(pData[2]&0x08);
		temp=temp>>3;
		Configdata201.temperatureError=temp;
		temp=0;
		temp=(short)(pData[2]&0x04);
		temp=temp>>2;
		Configdata201.temporaryError=temp;
		temp=0;
		temp=(short)(pData[2]&0x02);
		temp=temp>>1;
		Configdata201.voltageError=temp;
		temp=0;
		temp=(short)(pData[5]&0x02);
		temp=temp>>1;
		Configdata201.ctrlRelayError=temp;
		temp=0;
		temp=(short)(pData[5]&0x10);
		temp=temp>>4;
		Configdata201.sendQualityCfg=temp;
		temp=0;
		temp=(short)(pData[5]&0x20);
		temp=temp>>5;
		Configdata201.sendExtInfoCfg=temp;
		temp=0;
		temp=(short)(pData[5]&0xc0);
		temp=temp>>6;
		Configdata201.motionRxstate=temp;
		temp=0;
		temp1=0;
		temp=(short)(pData[3]&0x03);
		temp=temp<<1;
		temp1=(short)(pData[4]&0x80);
		temp1=temp1>>7;
		temp=temp+temp1;
		Configdata201.powerCfg=temp;
		temp=0;
		temp1=0;
		temp=(short)(pData[7]&0x1c);
		temp=temp>>2;
		Configdata201.rcs_thred=temp;
		Configdata201.sensorID=(short)(pData[4]&0x07);

		temp=0;
		temp1=0;
		temp=(short)(pData[4]&0x70);
		temp=temp>>4;
		Configdata201.sortIndex=temp;


		temp=0;
		temp1=0;
		temp=(short)(pData[5]&0x0c);
		temp=temp>>2;
		Configdata201.outputCfg=temp;
	}
 
}
void procCanFrame(TimestampedCanFrame Tframe) 
{
	uint16 i;
	uint8 *pData = Tframe.frame.data;
	int16 temp,temp1,temp2,temp3;
	if( Tframe.frame.id==0x60a)
	{
		DataFlag=1;
		currentNumObj=(short)(pData[0]&0xff);
		numOfTgt=currentNumObj;
		numB=0;
		numC=0;
		numD=0;
		memset(lrrRadarRawData, 0, sizeof(sLrrRawData) * LrrMaxTarNum);
		memset(lrrRadarProcData, 0, sizeof(sLrrProcData) * LrrMaxTarNum);
		// ICV_LOG_INFO<<"long NUM "<<numOfTgt;
	}


	if( Tframe.frame.id==0x60b)
	{
		//ICV_LOG_INFO<<"OX60B";

		i=(numB);
		lrrRadarRawData[i].isUpdated = 1;
		temp = 0;
		temp = (short)(pData[0]&0xff);
		lrrRadarRawData[i].objID = (temp);
		temp = 0;
		temp1 = 0;
		temp = (short)(pData[1]&0xff);

		temp = temp<<5;
		temp1 = (pData[2]&0xf8);
		temp1 = temp1>>3;
		temp=temp+temp1;
		lrrRadarRawData[i].rangex = (temp-2500);   // 还需要*0.2
		// ICV_LOG_INFO<<"long rangex:"<< (lrrRadarRawData[i].rangex*0.2);

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[2]&0x07);
		temp = temp<<8;
		temp1 = (short)(pData[3]&0xff);
		temp=temp+temp1;
		lrrRadarRawData[i].rangey =(temp-1023);   // 还需要*0.2


		temp = 0;
		temp1 = 0;
		temp = (short)(pData[4]&0xff);
		temp = temp<<2;
		temp1 = (short)(pData[5]&0xc0);
		temp1 = temp1>>6;
		temp+=temp1;
		lrrRadarRawData[i].speedx = (temp-512);   // 还需要*0.25

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[5]&0x3f);
		temp = temp<<3;
		temp1 = (short)(pData[6]&0xe0);
		temp1 = temp1>>5;
		temp+=temp1;
		lrrRadarRawData[i].speedy = (temp-256);    // 还需要*0.25

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[6]&0x07);
		lrrRadarRawData[i].objDynProp = (temp);    // probability

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[7]&0xff);
		lrrRadarRawData[i].obj_amp = (temp-128);    // probability chengyi 0.5

		(numB)++;
	}

	/////add///////

	if(Tframe.frame.id==0x60c)
	{
	
		i=0;
		i=(numC);

		
		temp = 0;
		temp = (short)(pData[0]&0xff);
		lrrRadarRawData[i].objID = (temp);

		//ROS_INFO("objID_C:%d",lrrRadarRawData[i].objID);

		temp = 0;   
		temp1 = 0;
		temp = (short)(pData[1]&0xf8);
		temp=temp>>3;
		lrrRadarRawData[i].rangex_rms = (temp);

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[1]&0x07);
		temp=temp<<2;
		temp1 = (short)(pData[2]&0xc0);
		temp1=temp1>>6;
		temp=temp+temp1;
		lrrRadarRawData[i].rangey_rms = (temp);

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[2]&0x3e);
		temp=temp>>1;
		lrrRadarRawData[i].speedx_rms=(temp);

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[2]&0x01);
		temp=temp<<4;
		temp1 = (short)(pData[3]&0xf0);
		temp1=temp1>>4;
		temp=temp+temp1;
		lrrRadarRawData[i].speedy_rms=(temp);

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[3]&0x0f);
		temp=temp<<1;
		temp1 = (short)(pData[4]&0x80);
		temp1=temp1>>7;
		temp=temp+temp1;
		lrrRadarRawData[i].accx_rms=(temp);


		temp = 0;
		temp1 = 0;
		temp = (short)(pData[4]&0x7c);

		temp=temp>>2;
	
		lrrRadarRawData[i].accy_rms=(temp);

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[4]&0x03);
		temp=temp<<3;
		temp1 = (short)(pData[5]&0xe0);
		temp1=temp1>>5;
		temp=temp+temp1;
		lrrRadarRawData[i].orient_rms=(temp);


		temp = 0;
		temp = (short)(pData[6]&0xe0);
		temp=temp>>5;

		lrrRadarRawData[i].objProbExist=(temp);

		temp = 0;
		temp = (short)(pData[6]&0x1c);
		temp=temp>>2;

		lrrRadarRawData[i].objMeasState=(temp);

		numC++;


	}
	///--------------------------------------------------///
	if(Tframe.frame.id==0x60d)
	{
	
		i=0;
		i=(numD);

		
		temp = 0;
		temp = (short)(pData[0]&0xff);

		lrrRadarRawData[i].objID = (temp);

		//ROS_INFO("objID_D:%d",lrrRadarRawData[i].objID);

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[1]&0xff);
		temp=temp<<3;
		temp1 = (short)(pData[2]&0xe0);
		temp=temp>>5;
		temp=temp+temp1;
		lrrRadarRawData[i].accx = (temp-1000);

	
		temp = 0;
		temp1 = 0;
		temp = (short)(pData[2]&0x1f);
		temp=temp<<4;
		temp1 = (short)(pData[3]&0xf0);
		temp=temp>>4;
		temp=temp+temp1;
		lrrRadarRawData[i].accy = (temp-250);

		temp = 0;
		temp1 = 0;
		temp = (short)(pData[3]&0x07);
		
		lrrRadarRawData[i].objClass = (temp);


		temp = 0;
		temp1 = 0;
		temp = (short)(pData[4]&0xff);
		temp=temp<<2;
		temp1 = (short)(pData[5]&0xc0);
		temp=temp>>6;
		temp=temp+temp1;
		lrrRadarRawData[i].ObjectOrientAngel = (temp-450);

		temp = 0;

		temp = (short)(pData[6]&0xff);
		
		lrrRadarRawData[i].ObjectLength = (temp);

		temp = 0;

		temp = (short)(pData[7]&0xff);
		
		lrrRadarRawData[i].ObjectWidth = (temp);
	

		numD++;


	}


}


virtual void Process(std::istream& stream) override
{
	// ICV_LOG_INFO<<"debug 1";
	// ICV_LOG_INFO<<"radar buffer size:"<<_buffer.size() ;
	if(!configured_)
	{
		sendRadarConfig();
		configured_=true;

	}
    // ICV_LOG_INFO<<"debug 2";
	if (_buffer.size() % 13 == 0)
	{
	int i_count=_buffer.size() /13;
	// ICV_LOG_INFO<<"frame num:"<<i_count;
	for(int i=0;i<i_count;i++)
	{
		char udpBuffer[13];
		stream.read(udpBuffer, 13);
		Tframe.frame.id = ((uint16)(udpBuffer[3] << 8) + udpBuffer[4]);
		//	ICV_LOG_INFO<<"Frame id"<<Tframe.frame.id ;
	  	for (int j = 0; j < 8; j++)
		{
			Tframe.frame.data[j] = udpBuffer[j + 5];
		}
		procCanFrame(Tframe);
		//ICV_LOG_INFO<<"radar NUMB: "<<numB ;
		// ICV_LOG_INFO<<"debug 3--";
		// ICV_LOG_INFO<<"data flag:"<<DataFlag;

		if(DataFlag==1)
		{
			// ICV_LOG_INFO<<"NUMB: "<<numB<<" NUMC:"<<numC<<" NUMD"<<numD ;
			if(numC==numB &&numD==numC&&(numB>0))
			{
			

			DataFlag=0;
			procLrrRadarData();

			temp_out.targetnum_=numB;
			ICV_LOG_INFO<<"long radar source target num:"<<numB ;
			temp_out.header_.stamp = SyncClock::now_us().time_since_epoch().count();
			// temp_out.header_.frame_id = "lrrdata";
			// strcpy(temp_out.header_.frame_id , "lrrdata");
			// ICV_LOG_INFO<<"debug 4";
			// ICV_LOG_INFO<<"num: "<<numB ;

			for (int i=0;i<numB;i++)
			{
				
				temp_out.alldata_[i]=lrrRadarProcData[i];
				// ICV_LOG_INFO<<"range long"<<lrrRadarProcData[i].x ;
			}
			// ICV_LOG_INFO<<"debug 5";

				// clear, fill up and publish msgObj
			temp_resu->setoutvalue(temp_out);
			// ICV_LOG_INFO<<"debug 5.5";
			Send_Out(temp_resu,0);
			currentNumObj=0;
			// ICV_LOG_INFO<<"debug 6";

			}	
		}
	}

	}
	else
	{
		printf("Error: have not received can data, or byte number is wrong!\n");
	}
	//outData[0]->As<icv::opencv::icvCvMatData>() = mFrame; 
	// ICV_LOG_INFO<<"debug 7";
}

	
private:
	static const int LrrMaxTarNum = 255;
	sLrrRawData lrrRadarRawData[LrrMaxTarNum];
	sLrrProcData lrrRadarProcData[LrrMaxTarNum];
	RadarLongOut temp_out;
	int numB,numC,numD,currentNumObj;
	static const int MaxUdpBufferSize = 1024;
	sLrrConfigdata201 Configdata201;
	int DataFlag=0,numOfTgt;
	icvLRRradardata* temp_resu;
	TimestampedCanFrame Tframe;
	bool configured_=false;
   
};
ICV_REGISTER_FUNCTION(RadarContiLong)

#endif  //
