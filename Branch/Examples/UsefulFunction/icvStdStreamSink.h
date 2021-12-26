#ifndef icvStdStreamSink_h
#define icvStdStreamSink_h

#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Basis/icvPrimitiveData.hxx"
#include <sstream>
namespace icv { namespace function
{
    using namespace icv::core;
    
    class icvStdStreamSink : public core::icvFunction
    {
    public:
        icvStdStreamSink(icv_shared_ptr<core::icvMetaData> params);
        icvStdStreamSink();

     

        virtual void Execute() ICV_OVERRIDE;

    private:
        std::ostream* _output;
        icvSubscriber* sub1;
        icvPublisher* pub1;
        icv::data::icvIntData* rec1;
        bool read_input_flag=true;
         stringstream ss;
    };
}}

#endif // icvStdStreamSink_h
