#ifndef icvTypedFunction_hxx
#define icvTypedFunction_hxx

#include "OpenICV/Core/icvFunction.h"

namespace icv { namespace function
{
    // The type list can be single type or a list generated from boost::mpl::vector
    template <typename InputTypes, typename OutputTypes>
    class icvTypedFunction : icvFunction
    {
        
    };

    template <typename Input1, typename Input2, ...>
    class icvTypedSink : icvFunction
    {
        
    };

    template <typename Output1, typename Output2, ...>
    class icvTypedSource : icvFunction
    {
        
    };
}}

#endif // icvTypedFunction_hxx
