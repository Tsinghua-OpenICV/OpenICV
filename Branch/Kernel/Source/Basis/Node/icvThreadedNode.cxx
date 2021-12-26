#include "OpenICV/Basis/icvThreadedNode.h"
#include "OpenICV/Core/icvNodeFactory.h"
#include "OpenICV/Core/icvSubscriber.h"
#include "OpenICV/Core/icvPublisher.h"
#include "OpenICV/Core/icvFunction.h"
#include "OpenICV/Core/icvDataObject.h"

#define ICV_FUNCTION
#include "OpenICV/Core/icvConfig.h"
#undef ICV_FUNCTION

namespace icv { namespace node
{
    ICV_REGISTER_NODE(icvThreadedNode)

    // void icvThreadedNode::Start(icvNodeManager* monitor)
    // {
    //    // _function->RegisterSuspend(icv_this_thread::sleep_for<Duration64::rep, Duration64::period>);
    //     icv_thread loop(icv_bind(&icvThreadedNode::InnerLoop, this));
    //    // icv_thread_guard g(loop);
    //     monitor_=monitor;

    // }

    void icvThreadedNode::Progress()
    {
        return; // TODO: implement
    }

    void icvThreadedNode::Abort()

    {
        return; // TODO: implement
    }
    
 


}}
