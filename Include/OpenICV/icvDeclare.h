// TODO: Make this file generated from ICV_XXX_REGISTER or from cmake
#ifndef icvDeclare_h
#define icvDeclare_h

// Functions
#include "OpenICV/Basis/icvRandomSource.h"
#include "OpenICV/Basis/icvStdStreamSink.h"

// Nodes
#include "OpenICV/Basis/icvThreadedNode.h"

namespace icv
{
    namespace _declare
    {
        // Declare this method to make all registerer take effect
        //     if the classes are not directly used in program (will be generated
        //     from configuration files).
        // Only needed if classes are build statically, acts like `--whole-archive` flag.
        void doDeclare()
        {
            // Functions
            icv::function::icvRandomSource stub_icvRandomSource;
            icv::function::icvStdStreamSink stub_icvStdStreamSink;

            // Nodes
            icv::node::icvThreadedNode stub_icvThreadedNode(ICV_NULLPTR);
        }
    }
}

#endif icvDeclare_h