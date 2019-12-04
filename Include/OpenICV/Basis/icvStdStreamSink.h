#ifndef icvStdStreamSink_h
#define icvStdStreamSink_h

#include "OpenICV/Core/icvFunction.h"

namespace icv { namespace function
{
    using namespace icv::core;
    
    class icvStdStreamSink : public core::icvFunction
    {
    public:
        icvStdStreamSink(icv_shared_ptr<core::icvMetaData> params);
        icvStdStreamSink();

     

        virtual void Execute(std::vector<icvNodeInput*>& inputPorts, std::vector<icvNodeOutput*>& outputPorts) ICV_OVERRIDE;

    private:
        std::ostream* _output;
    };
}}

#endif // icvStdStreamSink_h
