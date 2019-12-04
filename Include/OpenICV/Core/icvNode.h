#ifndef icvNode_h
#define icvNode_h

#define ICV_CONFIG_STDINT
#define ICV_CONFIG_POINTERS
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_STDINT
#undef ICV_CONFIG_POINTERS

#include <vector>
#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMacros.h"
#include "OpenICV/Core/icvMetaData.h"

namespace icv { namespace core
{
    class icvNodeInput;
    class icvNodeOutput;
    class icvFunction;
    class icvMonitor;

    class icvNode: public icvObject
    {
    public:
        // Every node will take charge of finalization of its inner function 
        icvNode(icvFunction* function);
        icvNode(icvFunction* function,
                icv_shared_ptr<const icvMetaData> params,
                icv_shared_ptr<const icvMetaData>* input_params,
                icv_shared_ptr<const icvMetaData>* output_params,int num_inport,int num_outport);
        virtual ~icvNode();

        typedef icv_shared_ptr<icvNode> Ptr;

        icvNodeInput* GetInputPort(int port = 0);
        icvNodeOutput* GetOutputPort(int port = 0);

        // Information
        ICV_PROPERTY_GETSET_PTR(ExtraInformation, _information, icvMetaData&)

        // Monitoring
        virtual void Start(icvMonitor* monitor) = 0;
        virtual void Progress() = 0;
        virtual void Abort() = 0;
        virtual void Trigger(icvNodeOutput* caller) = 0;

    protected:
        virtual void Execute();

        icvMetaData _information;

        // TODO: change to icvNodeInput* icvNodeOutput*
        std::vector<icvNodeInput*> _inputPorts;
        std::vector<icvNodeOutput*> _outputPorts;
        int _num_in=10,_num_out=5;

        icvFunction* _function;

    public:
        friend class icvNodeInput;
        friend class icvNodeOutput;
    };

    void Connect(icvNode::Ptr producer, int producerPort, icvNode::Ptr consumer, int consumerPort);
    void Disconnect(icvNode::Ptr producer, int producerPort, icvNode::Ptr consumer, int consumerPort);
    void Connect(icvNodeInput* input, icvNodeOutput* output);
    void Disconnect(icvNodeInput* input, icvNodeOutput* output);
}}

#endif // icvNode_h
