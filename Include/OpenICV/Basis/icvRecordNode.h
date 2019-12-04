#ifndef icvRecordNode_h
#define icvRecordNode_h

#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Basis/icvBaseRecorder.h"

namespace icv { namespace node
{
    using namespace icv::core;

    class icvRecordNode : public icvNode
    {
    public:
        icvRecordNode(icvFunction* function) : icvNode(function) {}
        icvRecordNode(icvFunction* function,
            icv_shared_ptr<const icvMetaData> params,
            icv_shared_ptr<const icvMetaData>* input_params,
            icv_shared_ptr<const icvMetaData>* output_params,int num_inport,int num_outport);

        virtual void Start(icvMonitor* monitor) ICV_OVERRIDE;
        virtual void Progress() ICV_OVERRIDE;
        virtual void Abort() ICV_OVERRIDE;
        virtual void Trigger(icvNodeOutput* caller) ICV_OVERRIDE;

    private:
        icv::_impl::icvBaseRecorder* _recorder;

    private:
        void InnerLoop(Uint32 port);
    };
}}

#endif // icvRecordNode_h
