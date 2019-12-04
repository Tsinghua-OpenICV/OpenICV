#include "OpenICV/Basis/icvThreadedNode.h"
#include "OpenICV/Core/icvNodeFactory.h"
#include "OpenICV/Core/icvNodeInput.h"
#include "OpenICV/Core/icvNodeOutput.h"
#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvDataObject.h"

#define ICV_FUNCTION
#include "OpenICV/Core/icvConfig.h"
#undef ICV_FUNCTION

namespace icv { namespace node
{
    ICV_REGISTER_NODE(icvThreadedNode)

    void icvThreadedNode::Start(icvMonitor* monitor)
    {
        _function->ConfigurateInput(_inputPorts);
        _function->ConfigurateOutput(_outputPorts);
        _function->RegisterSuspend(icv_this_thread::sleep_for<Duration64::rep, Duration64::period>);
        icv_thread loop(icv_bind(&icvThreadedNode::InnerLoop, this));
    }

    void icvThreadedNode::Progress()
    {
        return; // TODO: implement
    }

    void icvThreadedNode::Abort()
    {
        return; // TODO: implement
    }
    
    void icvThreadedNode::Trigger(icvNodeOutput* caller)
    {
        //_updator.notify_one();
    }

    void icvThreadedNode::InnerLoop()
    {
        ICV_LOG_TRACE << "Started icvThreadedNode InnerLoop @ Thread " << icv_this_thread::get_id();
        if (_inputPorts.size() > 0)
        {

            //icv_unique_lock<icv_mutex> lock(_updatorMutex);
            while (true)
            {
                //_updator.wait(lock);
                //ICV_LOG_TRACE << "Update signal @ Thread " << icv_this_thread::get_id();
      

                Execute();
            }
        }
        else
        {

            while (true)
            {
        //ICV_LOG_TRACE << "Update signal ac @ Thread " << icv_this_thread::get_id();

                Execute();
            }
        }
    }
}}
