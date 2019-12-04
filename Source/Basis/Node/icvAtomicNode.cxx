#include "OpenICV/Basis/icvAtomicNode.h"
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
    ICV_REGISTER_NODE(icvAtomicNode)

    void icvAtomicNode::Start(icvMonitor* monitor)
    {
        _function->ConfigurateInput(_inputPorts);
        _function->ConfigurateOutput(_outputPorts);
        _function->RegisterSuspend(icv_this_thread::sleep_for<Duration64::rep, Duration64::period>);
        icv_thread loop(icv_bind(&icvAtomicNode::InnerLoop, this));
    }

    void icvAtomicNode::Progress()
    {
        return; // TODO: implement
    }

    void icvAtomicNode::Abort()
    {
        return; // TODO: implement
    }
    
    void icvAtomicNode::Trigger(icvNodeOutput* caller)
    {
        _lock.unlock();
    }

    void icvAtomicNode::InnerLoop()
    {
        ICV_LOG_TRACE << "Started icvAtomicNode InnerLoop @ Thread " << icv_this_thread::get_id();
        if (_inputPorts.size() > 0)
        {
            while (true)
            {
                _lock.lock();
                ICV_LOG_TRACE << "Update signal @ Thread " << icv_this_thread::get_id();

                Execute();
            }
        }
        else
        {
            while (true)
            {
                Execute();
            }
        }
    }
}}
