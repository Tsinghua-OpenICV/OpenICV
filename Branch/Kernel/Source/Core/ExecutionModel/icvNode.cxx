#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvDataObject.h"
#include "OpenICV/Engine/icvConfigDefinitions.h"
#include <pthread.h>
using namespace std;
using namespace icv::engine;

namespace icv { namespace core
{
    icvNode::icvNode(icvFunction* function) : icvNode(function, ICV_NULLPTR, ICV_NULLPTR, ICV_NULLPTR) {}

    icvNode::icvNode(icvFunction* function,
        icv_shared_ptr<const icvMetaData> info,
        icv_shared_ptr<const icvMetaData>* input_info,
        icv_shared_ptr<const icvMetaData>* output_info) :
        _function(function)
    {
        nodename_=info->GetString(_keys::nodename);
        ICV_LOG_INFO<< "Initialize Node @" << nodename_;
        //ICV_LOG_INFO<< "The number of publisher is " << _function->GetPublisherPtrvector().size();
        //_inputPorts.insert(_inputPorts.begin(),_function->GetSubscriberPtrvector().begin(),_function->GetSubscriberPtrvector().end());
        //_outputPorts.insert(_outputPorts.end(),_function->GetPublisherPtrvector().begin(),_function->GetPublisherPtrvector().end());
        _outputPorts=_function->GetPublisherPtrvector();
        _function->set_nodeowner(this,nodename_);
        _function->Register_INPROC();
        if (info) _information = *info;
        micro_Sec_loop=_function->get_loop_time_micro();
        timesync_=new icvSemaphore(1);

        icv_thread loop(icv_bind(&icvNode::syncLoop, this));
        icv_thread_guard g(loop);

        //ICV_LOG_INFO<<"hardware supported thread number"<< loop.hardware_concurrency();
       
  

    }
    icvNode::icvNode(){  
        ICV_LOG_INFO<< "Initialize Node with default constructor ";
        }


    icvNode::~icvNode()
    {
        // FIXME: Ensures the connection are closed?
        for (int i = 0; i < _inputPorts.size(); i++)
            delete _inputPorts[i];
        for (int i = 0; i < _outputPorts.size(); i++)
            delete _outputPorts[i];
        delete _function;

        active_=false;
    }


     void icvNode::Start(icvNodeManager* monitor)
     {
    //    // _function->RegisterSuspend(icv_this_thread::sleep_for<Duration64::rep, Duration64::period>);
        icv_thread loop(icv_bind(&icvNode::InnerLoop, this));
        //loop.detach();
         icv_thread_guard g(loop);
         monitor_=monitor;
         monitor_->rec_thread_id(this->get_nodename(),&loop);

     }







          void icvNode::InnerLoop()
     {
             while (active_)
            {
                
      

                Run();
                monitor_->response_monitor(this->get_nodename());
            }
     }
    

    void icvNode::syncLoop()
    {
        ICV_LOG_INFO<<"frequency of node "<<this->get_nodename()<<" : "<<1000000/micro_Sec_loop<<" "<<active_;
        while(active_)
        {   

        timesync_->Release();
        usleep(micro_Sec_loop);
        }
    }



   void icvNode::Trigger(icvPublisherInterface* caller)
    {
        _lock.unlock();
    }
    void icvNode::Run()
    {


       if(_function->has_Triggerinput())
       { _lock.lock(); }  
       else timesync_->Lock() ;


        _function->Execute();

  

       // ICV_LOG_INFO << "test updates @ Thread " << icv_this_thread::get_id();
    }

    // void Connect(icvNode::Ptr producer, int producerPort, icvNode::Ptr consumer, int consumerPort)
    // {
    //     Connect(consumer->GetInputPort(consumerPort), producer->GetOutputPort(producerPort));
    // }

    // void Disconnect(icvNode::Ptr producer, int producerPort, icvNode::Ptr consumer, int consumerPort)
    // {
    //     Disconnect(consumer->GetInputPort(consumerPort), producer->GetOutputPort(producerPort));
    // }

    void Connect(icvSubscriberInterface* input, icvPublisherInterface* output)
    {
        input->GetConnections().push_back(output);
        output->GetConnections().push_back(input);
    }
    void Disconnect(icvSubscriberInterface* input, icvPublisherInterface* output)
    {
        // Remove output port from input connections
        auto inconnection = find(input->GetConnections().begin(), input->GetConnections().end(), output);
        if (inconnection != input->GetConnections().end()) input->GetConnections().erase(inconnection);

        // Remove input port from output connections
        auto outconnection = find(output->GetConnections().begin(), output->GetConnections().end(), input);
        if (outconnection != output->GetConnections().end()) output->GetConnections().erase(outconnection);
    }
}}
