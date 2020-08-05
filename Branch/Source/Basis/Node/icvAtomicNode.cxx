#include "OpenICV/Basis/icvAtomicNode.h"
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
    ICV_REGISTER_NODE(icvAtomicNode)

    // void icvAtomicNode::Start(icvNodeManager* monitor)
    // {

    //    // _function->RegisterSuspend(icv_this_thread::sleep_for<Duration64::rep, Duration64::period>);
    //     icv_thread loop(icv_bind(&icvAtomicNode::InnerLoop, this));
    //    // icv_thread_guard g(loop);
    // }

    void icvAtomicNode::Progress()
    {
        return; // TODO: implement
    }

    void icvAtomicNode::Abort()
    {
        return; // TODO: implement
    }
    
 


}}
