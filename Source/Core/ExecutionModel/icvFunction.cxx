#include "OpenICV/Core/icvFunction.h"

namespace icv { namespace core
{
    void icvFunction::RegisterSuspend(const icv_function<void(Duration64)>& func)
    {
        _sleepFunc = func;
    }
}}
