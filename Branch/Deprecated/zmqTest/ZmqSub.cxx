#ifndef __ZmqSub_H__
#define __ZmqSub_H__

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"

#include "OpenICV/Basis/icvZmqNet.h"
#include "OpenICV/Basis/icvStructureData.hxx"
#include "OpenICV/structure/structureIMU.h"
#include "icvMsgEncode.hpp"

using namespace icv;
using namespace core;
using namespace icv::function;
class ZmqSub: public ZmqNetSource
{
public:

  ZmqSub(icv_shared_ptr<const icvMetaData> info) : ZmqNetSource(info)
  {
    ICV_LOG_INFO<<"init zmq test";
  }


  virtual void Process() override
  {
    // ICV_LOG_INFO<<"Sub !!!!!!!!!!!!!!!!";
    int size = recv();
    // ICV_LOG_INFO<<"recv size = "<<size ;
    if (size != -1)
    {
        char* receive = (char*)_buffer;
        Imu msg;
        time_t sourceT;
        udpMsgDecode(msg, receive, sourceT);
        std::cout<<"Imu acceleration X = "<<msg.linear_acceleration.x<<std::endl;
    }
  }

private:

};
ICV_REGISTER_FUNCTION(ZmqSub)

#endif 
