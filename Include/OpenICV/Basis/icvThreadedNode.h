#ifndef icvThreadedNode_h
#define icvThreadedNode_h

#define ICV_CONFIG_THREAD
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_THREAD

#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvNode.h"

namespace icv { namespace node
{
    using namespace icv::core;

    class icvThreadedNode : public icvNode
    {
    public:
        icvThreadedNode(icvFunction* function) : icvNode(function) {}
        icvThreadedNode(icvFunction* function,
            icv_shared_ptr<const icvMetaData> params,
            icv_shared_ptr<const icvMetaData>* input_params,
            icv_shared_ptr<const icvMetaData>* output_params,int num_inport,int num_outport)
            : icvNode(function, params, input_params, output_params, num_inport, num_outport) {}

        virtual void Start(icvMonitor* monitor) ICV_OVERRIDE;
        virtual void Progress() ICV_OVERRIDE;
        virtual void Abort() ICV_OVERRIDE;
        virtual void Trigger(icvNodeOutput* caller) ICV_OVERRIDE;

    private:
        icv_mutex _updatorMutex;
        icv_condition_variable _updator;

    private:
        void InnerLoop();
    };
}}

#endif // icvThreadedNode_h
