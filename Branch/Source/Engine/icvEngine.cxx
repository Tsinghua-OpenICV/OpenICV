#include "OpenICV/Engine/icvEngine.h"
#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Core/icvMacros.h"

using namespace std;
//#define MONITOR_TIME  100000;

namespace icv { namespace engine
{
    void icvEngine::Start()
    {
                for (icvNode::Ptr node : _nodes)
        {
            ICV_LOG_INFO << " Start node @" << node->get_nodename();
            node->Start(node_manager); // TODO: Add a main monitor
        }
        icv_thread monitorall(icv_bind(&icvEngine::innerloop, this));
        monitorall.detach();
    }
    void icvEngine::innerloop()
    {
        while(true)
        {
        usleep(100000);
        node_manager->check_delayed_node();
        }
    }
    void icvEngine::Hold()
    {
        while (true) // Block immediately
            sleep(10);
        // TODO: Support events by using Event Loop here
    }

    void icvEngine::AddNode(icv_shared_ptr<icvNode> node)
    {
        _nodes.emplace(node);
    }

    void icvEngine::RemoveNode(icv_shared_ptr<icvNode> node)
    {
        _nodes.erase(node);
    }
}}
