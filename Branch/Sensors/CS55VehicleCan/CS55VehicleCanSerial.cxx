//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _CS55VehicleCanSerial_H
#define _CS55VehicleCanSerial_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/structure/structureFrame.h"
#include "OpenICV/structure/structure_gps.h"
#include "OpenICV/structure/structure_CS55.h"


#include <cstdlib>
#include <string>
#include <sstream>
#include <boost/thread/thread.hpp>
#include "NMEA0183/NMEA0183.H"

#define PI 3.1415926
#define UNKNOWN_NMEA_FRAME -1


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
class CS55CanSerial: public icvFunction
{
public:

  CS55CanSerial(icv_shared_ptr<const icvMetaData> info) : icvFunction(info)
  {


    Register_Sub("SerialPort");
    Register_Pub("CS55CanInfo");

    Register_Sub("CS55control");
    Register_Pub("SerialPortSender",false);

  tempCS55_READ= new icvCS55_READ_MCU();
  tempCS55_CONT= new icvCS55_CONTR_MCU();

  };


CS55CanSerial() : CS55CanSerial(nullptr) {};






int frameType(string frame)
{
    if (uint8_t(frame[4])==0x82) return 1;//"MCU2PC";
    else if (uint8_t(frame[4])==0x81) return 2;//"PC2MCU";
    else return -1;
};




bool analyzeFrame(icv::buffFrame * currentFrame)
{
  // Process the remaining bytes in the current frame
    int nextByteToProcess_ = 0;
   bool startOfFrame_=false;
    bool endOfFrame_ = false;
  while(nextByteToProcess_ < currentFrame->data.length() ) 
  {
    uint8_t currentChar = currentFrame->data[nextByteToProcess_++];
    // first looking for start-of-frame
    if (!startOfFrame_ && (nextByteToProcess_+2<currentFrame->data.length())&&(currentChar == 0xFF)) 
    {
        uint32_t headercode=(uint32_t)(currentChar << 16 +currentFrame->data[nextByteToProcess_]<<8+ currentFrame->data[nextByteToProcess_+1]);
        if(headercode==0xFFA55A)
          {
            startOfFrame_ = true;
          endOfFrame_ = false;

          frameToDecode_.time_stamp_ = currentFrame->time_stamp_;
          frameToDecode_.data="";
          }
    } else if (startOfFrame_ && !endOfFrame_ && (currentChar == 0x96||currentChar == 0x93)) {
      // Looking for end-of-frame
      startOfFrame_ = false;
      endOfFrame_ = true;
      frameToDecode_.length_=frameToDecode_.data.length();
      return true;  // There is a new frame to decode
    }
    if ((startOfFrame_) && (!endOfFrame_)) {
      frameToDecode_.data.push_back(currentChar);
    }
  }
  return false; // No new frame to decode, wait for more data
};

buffFrame encodeFrame(icvCS55_CONTR_MCU* frametmp)
{
  buffFrame tmp;
  string data_tmp;
  tmp.time_stamp_ = frametmp->GetSourceTime();
  tmp.length_=22;
	uint8 sendBufcs55[22]; 
  //=======================0f0============================
        memset(sendBufcs55, 0, sizeof(sendBufcs55));
        sendBufcs55[0] = 0xFF;  //0x08
        sendBufcs55[1] = 0xA5;
        sendBufcs55[2] = 0x5A; 
        sendBufcs55[3] = 0x12;
        sendBufcs55[4] = 0x81;
//////////////data segment/////////////
        sendBufcs55[5] = (frametmp->getvalue().TargetStrAngle)>>8;  
        sendBufcs55[6] = (frametmp->getvalue().TargetStrAngle)&& 0xFF;
        sendBufcs55[9] = (frametmp->getvalue().TargetAccelReq)>>8;  
        sendBufcs55[10] =(frametmp->getvalue().TargetAccelReq)&& 0xFF;
        sendBufcs55[11] = (frametmp->getvalue().TargetDecelReq)>>8;  
        sendBufcs55[12]= (frametmp->getvalue().TargetDecelReq)&& 0xFF;
        sendBufcs55[13]= frametmp->getvalue().TargetDrivingStatus;
        sendBufcs55[14]= frametmp->getvalue().TargetModeSelect ; 
        sendBufcs55[15]= frametmp->getvalue().TargetShiftPosition ; 
        sendBufcs55[16]= frametmp->getvalue().BodyEleControl; 
        sendBufcs55[17]= frametmp->getvalue().SpecificControl; 


  tmp.data.insert(0,(char*)sendBufcs55,22);
  return tmp;
        





};


int decodeFrame(int type)
{
  double lat_rad = 0, lon_rad = 0;
  int indexGSV = 0;
  int indexGSA = 0;




  switch(type)
  {
  case -1:
    ICV_LOG_INFO<<"Unknown frame received !";
    break;
  case 1:
  //MCU 2 PC
  tempCS55_READ->getvaluePtr()->RealStrAngle=(double)(frameToDecode_.data[5]<<8+frameToDecode_.data[6])*0.1;
  tempCS55_READ->getvaluePtr()->RealShiftPosition=frameToDecode_.data[12];
  tempCS55_READ->getvaluePtr()->CurrentDrvMode=frameToDecode_.data[13];
  tempCS55_READ->getvaluePtr()->SysSwStatus=frameToDecode_.data[14];
  tempCS55_READ->getvaluePtr()->VehicleSysFault=frameToDecode_.data[15];
  tempCS55_READ->getvaluePtr()->VehicleSpeed=(double)(frameToDecode_.data[20]<<8+frameToDecode_.data[21])*0.1;
  icvPublish("CS55CanInfo",tempCS55_READ);


    break;

  case 2:
  tempCS55_CONT->getvaluePtr()->TargetStrAngle =(short)(frameToDecode_.data[5]<<8+frameToDecode_.data[6]);
  tempCS55_CONT->getvaluePtr()->TargetAccelReq=(short)(frameToDecode_.data[9]<<8+frameToDecode_.data[10]);
  tempCS55_CONT->getvaluePtr()->TargetDecelReq=(short)(frameToDecode_.data[11]<<8+frameToDecode_.data[12]);
  tempCS55_CONT->getvaluePtr()->TargetDrivingStatus=frameToDecode_.data[13];
  tempCS55_CONT->getvaluePtr()->TargetModeSelect=frameToDecode_.data[14];
  tempCS55_CONT->getvaluePtr()->TargetShiftPosition=frameToDecode_.data[15];
  tempCS55_CONT->getvaluePtr()->BodyEleControl=frameToDecode_.data[16];
  tempCS55_CONT->getvaluePtr()->SpecificControl=frameToDecode_.data[17];

    break;
   

  default:
      return -1;
  }

  return 1;
};
  virtual void Execute() override
  {
    icv::icvbuffFrame temp;
   // icv:icv
    icvSubscribe("SerialPort",&temp);
    if (analyzeFrame(temp.getvaluePtr())) 
    {
      // a new complete NMEA frame has arrived, decode it.
      type = frameType(frameToDecode_.data);
      if (type != -1) 
      {
        if (decodeFrame(type) == -1) 
        {
            ICV_LOG_WARN<<"Failed to decode the dataframe\n";
        }

      }
	  }


    icvSubscribe("CS55control",tempCS55_CONT);
   buffFrame tmpFrame= (encodeFrame(tempCS55_CONT));
   icvbuffFrame *contro_cm= new icvbuffFrame(tmpFrame);
   icvPublish("SerialPortSender",contro_cm,false);


  
     


	};

private:

  
  static const int MaxUdpBufferSize = 1024;

	std::string portName_;
  int type = -1;

  buffFrame frameToDecode_;
  icvCS55_READ_MCU* tempCS55_READ;
  icvCS55_CONTR_MCU* tempCS55_CONT;


};


ICV_REGISTER_FUNCTION(CS55CanSerial)

#endif 
