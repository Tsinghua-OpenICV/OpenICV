#ifndef __ICV_ZMQ_NET__
#define __ICV_ZMQ_NET__

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include "OpenICV/Basis/icvSemaphore.h"

#include "OpenICV/Core/icvTime.h"
#include "OpenICV/Core/icvConfig.h"
#include "OpenICV/Core/icvMacros.h"
#include <zmq.hpp>



namespace icv{namespace function{


using namespace icv::core;
using namespace std;
class ZmqNetSource : public icvFunction
{
public:
    ZmqNetSource(icv_shared_ptr<const icvMetaData> info);
    ~ZmqNetSource();

    virtual void Execute() ICV_OVERRIDE;
    void send(void* buffer_send,int length);
    int recv();

protected:
    virtual void Process() = 0;

protected:
    char _buffer[2048];
    void* context;
    void* subscriber;
    void* publisher; 
};
}}

#endif
