#include "OpenICV/Engine/icvEngine.h"
#include "OpenICV/Core/icvNode.h"
#include "OpenICV/Core/icvMacros.h"

using namespace std;

namespace icv { namespace engine
{
    void icvEngine::Start()
    {
        // XXX: Topological sort is needed maybe
        for (icvNode::Ptr node : _nodes)
        {
            ICV_LOG_TRACE << "Start node @" << node;
            node->Start(ICV_NULLPTR); // TODO: Add a main monitor
        }
    }

    void icvEngine::Hold()
    {
		while (true) // Block immediately
			boost::this_thread::sleep(boost::posix_time::milliseconds(50)); 
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
