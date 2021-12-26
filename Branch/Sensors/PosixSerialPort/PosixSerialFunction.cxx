//  @ Project : RADAR
//  @ File Name : sensorlrr.h
//  @ Date : 2016/12/22
//  @ Author :
//
//


#ifndef _PosixSerial_H
#define _PosixSerial_H

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/structure/structureFrame.h"



#include <cstdlib>
#include <string>
#include <sstream>
#include <boost/thread/thread.hpp>
#include "PosixSerialPort.h"

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
class PosixSerialFunction: public icvFunction
{

public:

  PosixSerialFunction(icv_shared_ptr<const icvMetaData> info) : icvFunction(info)
  {
    ICV_LOG_INFO<<"Starting posix serial function"<<endl;
    setPortCOM(_information.GetString("port").c_str());
    serialPort = new PosixSerialPort();
    if (!serialPort->openPort(portName_.c_str()))
    {
      ICV_LOG_INFO<<"Failed to open the port " << portName_;
    }
    else
    {ICV_LOG_INFO<<"Posix Serial port open";

    }

    
    baudrate=115200;
    bytesize=8 ;
    parity=0;
    stopbits=1 ;
    if(_information.Contains("baudrate"))baudrate=_information.GetInteger("baudrate");
    if(_information.Contains("bytesize"))bytesize=_information.GetInteger("bytesize");
    if(_information.Contains("parity"))  parity  =_information.GetInteger("parity");
    if(_information.Contains("stopbits"))stopbits=_information.GetInteger("stopbits");
   
   
    if(_information.Contains("sendernode"))
    {
      send_bool=true;
      sendernode=_information.GetString("sendernode");
    }

    serialPort->configurePort(baudrate,bytesize,parity,stopbits-1);
  //icv_thread loop(icv_bind(&PosixSerialPort::run, this));
  //icv_thread_guard g(loop);
  tempdata=new (data::icvStructureData<buffFrame>) (); 
  tempdata_send=new (data::icvStructureData<buffFrame>) (); 

    Register_Pub("SerialPort");
    //Register_Sub("SerialPortSender",sendernode);

  };


PosixSerialFunction() : PosixSerialFunction(nullptr) {}

void setPortCOM(const char * port)
{
  portName_ = port;
}

 
   virtual void Execute() override
  {
    //  ICV_LOG_INFO<<"Starting posix serial execute"<<endl;
    if(send_bool)
    {
    icvSubscribe("SerialPortSender",tempdata_send,sendernode);
    ;
    serialPort->writeBuffer(tempdata_send->getvalue().data,tempdata_send->getvalue().length_);


    }

  currentFrame_ = serialPort->getNextFrame();

      if  (PortOpened&&(currentFrame_!= NULL)) 
	{
    std::cout<<"timestamp: "<<currentFrame_->time_stamp_<<" length: "<<currentFrame_->length_<<" content: "<<currentFrame_->data<<endl;
    tempdata->setvalue(*currentFrame_);
	  icvPublish("SerialPort",tempdata);


  }

    // Check if we got a PPS frame

	};

private:
  int baudrate,bytesize,parity,stopbits;
  buffFrame* currentFrame_;
	PosixSerialPort * serialPort;
	std::string portName_;
  bool PortOpened=false;
  icvbuffFrame *tempdata, *tempdata_send; 
  string sendernode;
  bool send_bool=false;


  };



ICV_REGISTER_FUNCTION(PosixSerialFunction)



#endif 
