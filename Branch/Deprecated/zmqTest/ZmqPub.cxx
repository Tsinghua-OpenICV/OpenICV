#ifndef __ZmqPub_H__
#define __ZmqPub_H__

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Basis/icvZmqNet.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/structure/structureIMU.h"
#include "icvMsgEncode.hpp"

using namespace icv;
using namespace core;
using namespace icv::function;
typedef data::icvStructureData<Imu> icvImu;
class ZmqPub: public ZmqNetSource
{
public:

  ZmqPub(icv_shared_ptr<const icvMetaData> info) : ZmqNetSource(info)
  {
    ICV_LOG_INFO<<"init zmq pub";
    count=0;
    Register_Sub("random number");
  }


  virtual void Process() override
  {
    // ICV_LOG_INFO<<"Publishing number "<<count;
    // char update [20];
    // sprintf (update, "hello zmq");
    icv::data::icvInt64Data a;
    
        //icvSubscribe<icv::data::icvInt64Data>("Random number");
    icvSubscribe("Random number",&a);
    //imu_data=read_Input<icvImu>(0);

    int Len ;
    std::string dataType = "1" ; // 1 for fusion; 2 for ublox fix; 3 for ublox vel 
    time_t source_time = 0 ;
    char *udpMsg = udpMsgEncode(imu_data,Len,source_time,dataType); 
    //ICV_LOG_INFO<<"send imu acceleration X = "<<imu_data.linear_acceleration.x;
    ICV_LOG_INFO<<"send imu acceleration X = "<<a.getvalue();
    send(udpMsg, Len);       
    udpMsgDelete(udpMsg);
    count++;
  }

private:
  int count;
  Imu imu_data;
};
ICV_REGISTER_FUNCTION(ZmqPub)

#endif 
