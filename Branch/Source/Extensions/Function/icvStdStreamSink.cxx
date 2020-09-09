#include "OpenICV/Extensions/UsefulFunctions/icvStdStreamSink.h"
#include "OpenICV/Core/icvDataObject.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvFunctionFactory.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"

#include <iostream>
namespace icv { namespace function
{    using namespace icv::data;


    ICV_REGISTER_FUNCTION(icvStdStreamSink)

    ICV_CONSTEXPR char KEY_OUTPUT_STREAM[] = "stream";

    icvStdStreamSink::icvStdStreamSink() : icvStdStreamSink(ICV_NULLPTR) {}
    icvStdStreamSink::icvStdStreamSink(icv_shared_ptr<icvMetaData> params)
        : icvFunction( params)
    {


    //Register_Sub("Random number");
    Register_Sub_Remote("tcp://127.0.0.1:2000");
    //Register_Sub_Remote("ipc://127.0.0.1:5555");
   
    }


    void icvStdStreamSink::Execute()
    {

        icv::data::icvInt64Data a;
        //icvSubscribe<icv::data::icvInt64Data>("Random number");
        //icvSubscribe("Random number",&a);
        icvSubscribe_Remote("tcp://127.0.0.1:2000",&a);
        int b=a.getvalue();
         ICV_LOG_INFO<<"before : "<<b<<"  after : "<<3*b;

    }
}}
