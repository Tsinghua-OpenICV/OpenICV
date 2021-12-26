#ifndef icvNode_h
#define icvNode_h

#define ICV_CONFIG_THREAD
#define ICV_CONFIG_ATOMIC
#define ICV_CONFIG_STDINT
#define ICV_CONFIG_POINTERS
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_STDINT
#undef ICV_CONFIG_POINTERS
#undef ICV_CONFIG_THREAD
#undef ICV_CONFIG_ATOMIC

#include <vector>
#include "OpenICV/Core/icvObject.h"
#include "OpenICV/Core/icvMacros.h"
#include "OpenICV/Core/icvMetaData.h"
#include "OpenICV/Basis/icvSemaphore.h"
#include "OpenICV/Core/icvNodeManager.h"
#include "OpenICV/Core/icvZmq.h"
namespace icv { namespace core
{
    class icvSubscriberInterface;
    class icvPublisherInterface;
    class icvFunction; 
    class icvNodeManager;

    class icvNode: public icvObject
    {
    public:
        // Every node will take charge of finalization of its inner function 
        icvNode();
        icvNode(icvFunction* function);
        icvNode(icvFunction* function,
                icv_shared_ptr<const icvMetaData> params,
                icv_shared_ptr<const icvMetaData>* input_params,
                icv_shared_ptr<const icvMetaData>* output_params);
        virtual ~icvNode();

        typedef icv_shared_ptr<icvNode> Ptr;

        //icvSubscriber* GetInputPort(int port = 0);
       // icvPublisher* GetOutputPort(int port = 0);

        // Information
        ICV_PROPERTY_GETSET_PTR(ExtraInformation, _information, icvMetaData&)

        //get loop period in useconds
        int get_period(){return micro_Sec_loop;}
        // Monitoring
        void Start(icvNodeManager* monitor) ;
        void InnerLoop();
        bool is_active(){return active_;};
        void set_active(bool a){ active_=a;};

        virtual void Progress() = 0;
        virtual void Abort() = 0;
        void Trigger(icvPublisherInterface* caller) ;
        void syncLoop();
        std::string get_nodename(){return nodename_;};
        std::vector<icvSubscriberInterface*> GetInputPorts(){return _inputPorts;}
        std::vector<icvPublisherInterface*> GetOutputPorts(){return _outputPorts;}
        icvNodeManager* GetMonitor(){return monitor_;}
    protected:
        virtual void Run();

        icvMetaData _information;

        // TODO: change to icvSubscriber* icvPublisher*
        std::vector<icvSubscriberInterface*> _inputPorts;
        std::vector<icvPublisherInterface*> _outputPorts;
        int _num_in=10,_num_out=5;
        int micro_Sec_loop;
        icvSemaphore* timesync_;
        spin_lock _lock;
        bool fix_freq=true;
        icvFunction* _function;
        std::string nodename_;
        icvNodeManager* monitor_;

    public:
        // friend class icvSubscriber;
        // friend class icvPublisher; //for temp test

    private:
        bool active_=true;
    };

    void Connect(icvNode::Ptr producer, int producerPort, icvNode::Ptr consumer, int consumerPort);
    void Disconnect(icvNode::Ptr producer, int producerPort, icvNode::Ptr consumer, int consumerPort);
    void Connect(icvSubscriberInterface* input, icvPublisherInterface* output);
    void Disconnect(icvSubscriberInterface* input, icvPublisherInterface* output);



}}

#endif // icvNode_h
