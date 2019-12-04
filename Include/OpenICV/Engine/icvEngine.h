#ifndef icvEngine_h
#define icvEngine_h

#define ICV_CONFIG_CONTAINERS
#define ICV_CONFIG_POINTERS
#include "OpenICV/Core/icvConfig.h"
#undef ICV_CONFIG_CONTAINERS
#undef ICV_CONFIG_POINTERS

#include "OpenICV/Core/icvNode.h"

namespace icv { namespace engine
{
    using namespace icv::core;

    class icvEngine : public icvObject
    {
    public:
        virtual void Start(); // Open nodes and prevent the program from exiting
        virtual void Hold(); // Prevent the process from exit

        virtual void AddNode(icv_shared_ptr<icvNode> node);
        virtual void RemoveNode(icv_shared_ptr<icvNode> node);

    protected:
        icv_set<icv_shared_ptr<icvNode>> _nodes;
    };
}}

#endif // icvEngine_h