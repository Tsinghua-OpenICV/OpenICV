

#ifndef _RadarContiSide_H
#define _RadarContiSide_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
//#include "OpenICV/Data/icvPrimitiveData.hxx"
#include "OpenICV/Net/icvUdpReceiverSource.h"
#include "OpenICV/structure/RadarFrameSide.h"
#include "OpenICV/Basis/icvStructureData.hxx"

#include "CanFrame.h"

#include <boost/thread/thread.hpp>



using namespace icv;
using namespace core;
using namespace icv::function;
class RadarContiSide: public icvUdpReceiverSource
{
public:
  typedef data::icvStructureData<RadarSideOut>    icvSRRradardata;

  RadarContiSide(icv_shared_ptr<const icvMetaData> info) : icvUdpReceiverSource(info){

    temp_recu=new icvSRRradardata();

	    };



void sendRadarConfig() 
{
   

  
        int ret1, ret2,ret3;
        uint8 sendBuf200[13],sendBuf202[13],sendBuf301[13];
        //vehicleInfo.velocity = 0; // TODO, need to be modified
        uint16 velocity =0;// (uint16)(vehicleInfo.velocity / 0.0625); // km/h-->m/s, default value: 0
        int16 yawrate = 0;//(int16)(vehicleInfo.yawrate / 0.0625); // rad/s-->deg/s, default value: 0

        memset(sendBuf200, 0, sizeof(sendBuf200));
        sendBuf200[0] = 0x08;
        sendBuf200[1] = 0x00;
        sendBuf200[2] = 0x00;
        sendBuf200[3] = 0x02;
        sendBuf200[4] = 0x00;

///////////data segment///////////

        sendBuf200[5] = 0xf9;//  (11111001);sensor id;power not to change
        sendBuf200[6] = 0x19;//  (00011001);64+32+4=100;x2=200m
        sendBuf200[7] = 0x00; // (00000000)
        sendBuf200[8] =0x00;// empty;
        sendBuf200[9] =0x08;// sensor iD:0,output 1:objects,power standard;
        sendBuf200[10]=0x9c;// nvm,sorted by range;extenstion and quality yes;CTRL DElay sendornot false
        sendBuf200[11]=0x01;//rcs standard
        sendBuf200[12]=0x00;//empty
        send(sendBuf200, 13);

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





}


void procSrrRadarData() 
{
  uint validIdx = 0;
  for (int i = 0; i <=numOfTgt; i++)//LrrMaxTarNum
  {

    if ((srrRadarRawData[i].rangex > 0)&&(srrRadarRawData[i].rangex < 200/0.2)&&(srrRadarRawData[i].isUpdated==1))
  {


    
      srrRadarProcData[validIdx].flag = 1;

      srrRadarProcData[validIdx].id = srrRadarRawData[i].objID;

      srrRadarProcData[validIdx].x = (float)(srrRadarRawData[i].rangex*0.1); //scale: 0.1

   //   ICV_LOG_INFO<<"range x"<<srrRadarProcData[validIdx].x ;

      srrRadarProcData[validIdx].y = (float)(srrRadarRawData[i].rangey*0.1); //scale: 0.1
      srrRadarProcData[validIdx].relspeedx = (float)(srrRadarRawData[i].speedx*0.02); //scale: 0.02
      srrRadarProcData[validIdx].relspeedy = (float)(srrRadarRawData[i].speedy*0.25); //scale: 0.25
      srrRadarProcData[validIdx].obj_amp = (float)(srrRadarRawData[i].obj_amp*0.5); //scale: 0.25
      srrRadarProcData[validIdx].trackLifeTime = (float)(srrRadarRawData[i].trackLifeTime*0.1);//scale:0.1
      srrRadarProcData[validIdx].trackIndex2 =srrRadarRawData[i].trackIndex2;//scale:1
      srrRadarProcData[validIdx].trackIndex =srrRadarRawData[i].trackIndex;

      validIdx++;
    
  }
  }

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
		if( Tframe.frame.id==0x60b)
		{
				DataFlag=1;
			
	     currentNumObj=(short)(Tframe.frame.data[0]&0xff);
   		  numOfTgt=currentNumObj;
    //  ICV_LOG_INFO<<"side NUMB: "<<currentNumObj;

     numC=0;
	 numD=0;
     memset(srrRadarRawData, 0, sizeof(sSrrRawData) * SrrMaxTarNum);
     memset(srrRadarProcData, 0, sizeof(sSrrProcData) * SrrMaxTarNum);
		}


		if( Tframe.frame.id==0x60c)
		{
			i=(numC);


			srrRadarRawData[i].isUpdated = 1;

			temp = 0;
			temp1=0;
			temp = (short)(pData[0]&0xff);
			temp=temp<<8;
			temp1=(short)(pData[1]&0xff);
			temp=temp+temp1;
			srrRadarRawData[i].objID = (temp);


			temp = 0;
			temp1=0;
			temp = (short)(pData[3]&0x1f); 
			srrRadarRawData[i].trackIndex=(temp);


			temp = 0;
			temp1=0;
			temp = (short)(pData[2]&0x3f);
			temp=temp<<3;
			temp1=(short)(pData[3]&0xe0);
			temp1=temp1>>5;
			temp=temp+temp1;    
			srrRadarRawData[i].rangex=(temp);
			temp = 0;
			temp1=0;
			temp = (short)(pData[4]&0xff);
			temp=temp<<2;
			temp1=(short)(pData[5]&0xc0);
			temp1=temp1>>6;
			temp=temp+temp1;    
			srrRadarRawData[i].rangey=(temp-511);

			temp = 0;
			temp1=0;
			temp = (short)(pData[5]&0x3f);
			temp=temp<<6;
			temp1=(short)(pData[6]&0xfc);
			temp1=temp1>>2;
			temp=temp+temp1;    
			srrRadarRawData[i].speedx=(temp-1750);

			temp = 0;
			temp1=0;
			temp = (short)(pData[7]&0xff);
			srrRadarRawData[i].speedy=(temp-128);
			(numC)++;
		}

		/////add///////

		if(Tframe.frame.id==0x60d)
		{
		
			i=0;
			i=(numD);

			temp=0;
			temp1=0;
			temp=(short)(pData[0]&0xff);
			srrRadarRawData[i].obj_amp=(temp-100);


			temp=0;
			temp1=0;	  
			temp=(short)(pData[1]&0xff);
			temp<<8;
			temp1=(short)(pData[2]&0xff);
			temp=temp+temp1;
				srrRadarRawData[i].trackLifeTime=temp;

			temp=0;
			temp1=0;

			temp=(short)(pData[3]&0x1f);
			srrRadarRawData[i].trackIndex2=temp; 
				(numD)++;


		}
		///--------------------------------------------------///
	


}


  virtual void Process(std::istream& stream) override
  {
	  if (_buffer.size() % 13 == 0)
	  {
		  while (!stream.eof())
		  {
			  char udpBuffer[13];
			  stream.read(udpBuffer, 13);

			  uint16 i, j;

			  //lrr
			  //frames.clear();
			  TimestampedCanFrame Tframe;

			  Tframe.frame.id = ((uint16)(udpBuffer[3] << 8) + udpBuffer[4]);

//ICV_LOG_INFO<<"SIDE RADAR ID"<<Tframe.frame.id;
			  // fill up udpBufferGroup
			  for (j = 0; j < 8; j++)
			  {
				  Tframe.frame.data[j] = udpBuffer[j + 5];
			  }

			  //int flag = 0;
		
				procCanFrame(Tframe);

					//procSrrRadarData();


				if(DataFlag==1)
				{
					if((numC==numD)&&(numC>0))
					{
					DataFlag=0;
				
					procSrrRadarData();
          temp_out.targetnum_=numC;
          ICV_LOG_INFO<<"side radar source target num:"<<numC ;
          temp_out.header_.stamp = SyncClock::now_us().time_since_epoch().count();
          // temp_out.header_.frame_id = "srrdata";
          // strcpy(temp_out.header_.frame_id , "srrdata");
         // ICV_LOG_INFO<<"RANGE SIDE target num X:"<<numOfTgt<<"numbc:"<<numC;

          for(int i=0;i<numC;i++)
          {
            temp_out.alldata_[i]=srrRadarProcData[i];

         // ICV_LOG_INFO<<"RANGE SIDE RADAR X:"<<i<<" ,"<<srrRadarProcData[i].x;
          }
					temp_recu->setoutvalue(temp_out);

          Send_Out(temp_recu,0);
						// clear, fill up and publish msgObj
					currentNumObj=0;

					}	
				}
		






			  //frames.push_back(Tframe);
		  }


		  // outData[0]->As<icv::opencv::icvCvMatData>() = mFrame;
		//  frames.clear();
	  }
	  else
	  {
		  printf("Error: have not received can data, or byte number is wrong!\n");
	  }
	  //outData[0]->As<icv::opencv::icvCvMatData>() = mFrame;
  
  }


	
	
private:
	static const int SrrMaxTarNum = 255;
	sSrrRawData srrRadarRawData[SrrMaxTarNum];
	sSrrProcData srrRadarProcData[SrrMaxTarNum];
	int numB,numC,numD,currentNumObj;
	static const int MaxUdpBufferSize = 1024;
	sSrrConfigdata201 Configdata201;
	int DataFlag=0,numOfTgt;
  RadarSideOut temp_out;
   icvSRRradardata* temp_recu;
};
ICV_REGISTER_FUNCTION(RadarContiSide)

#endif  //
