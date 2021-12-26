// TODO: Make this file generated from ICV_XXX_REGISTER or from cmake
#ifndef icvDeclare_h
#define icvDeclare_h

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

            // Nodes
            icv::node::icvThreadedNode stub_icvThreadedNode(ICV_NULLPTR);
        }
    }
}

#endif 
